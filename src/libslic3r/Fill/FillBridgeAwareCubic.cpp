#include "FillBridgeAwareCubic.hpp"

#include "BridgeAwareCubicPlan.hpp"
#include "../BridgeAwareCubicMaterial.hpp"
#include "../ClipperUtils.hpp"
#include "../EdgeGrid.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"
#include "../ShortestPath.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>

namespace Slic3r {

namespace {

Point along(const Line &line, double t)
{
    const Vec2d a = line.a.cast<double>(), d = (line.b - line.a).cast<double>();
    return Point(coord_t(std::llround(a.x() + t * d.x())), coord_t(std::llround(a.y() + t * d.y())));
}

// A planned line's end that stops within `reach` of the fill boundary is carried on to the boundary, so that the stock connector
// anchors it to the perimeter as it anchors Cubic's lines (user decision, 9 November 2026: every line the pattern adds is anchored
// like Cubic's). Returns the end moved, or the end itself.
Point carried_to_boundary(const Point &from, const Point &end, const BridgeAwareCubic::AreaIndex &index, double reach)
{
    const Vec2d d = (end - from).cast<double>();
    const double length = d.norm();
    if (length < 1.)
        return end;
    const Vec2d  unit = d / length;
    const Point  far(coord_t(std::llround(end.x() + unit.x() * reach)), coord_t(std::llround(end.y() + unit.y() * reach)));
    const Line   probe(from, far);
    const double probe_length = length + reach;
    const double t_end = length / probe_length;
    for (const auto &interval : index.segment_inside(probe.a, probe.b))
        if (interval.first <= t_end + 1e-9 && interval.second >= t_end - 1e-9) {
            if (interval.second >= 1. - 1e-9)
                return end;    // the boundary is farther than the reach: the end stays in the void
            return along(probe, interval.second);
        }
    return end;
}

// Anchors the interior ends of the paths where they meet another path of the layer (a needed strand ends on the line of its
// cell's face: a bulk line or a parent strand): a nub along the met path's centre line from the junction, `anchor_length`
// long, clipped to the met segment, appended to the path. Ends within `reach` of the fill boundary are the connector's.
void anchor_junctions(Polylines &paths, const ExPolygons &boundary, double reach_scaled, double anchor_length_scaled)
{
    if (paths.size() < 2 || anchor_length_scaled < scale_(0.01))
        return;
    struct Segment { size_t path; Point a, b; };
    std::vector<Segment> segments;
    for (size_t i = 0; i < paths.size(); ++i)
        for (size_t k = 1; k < paths[i].points.size(); ++k)
            if (paths[i].points[k - 1] != paths[i].points[k])
                segments.push_back({ i, paths[i].points[k - 1], paths[i].points[k] });
    // A uniform grid of the segments by the cells their boxes cover.
    const coord_t cell = coord_t(scale_(2.));
    const auto cell_of = [cell](coord_t v) { return int(std::floor(double(v) / double(cell))); };
    std::map<std::pair<int, int>, std::vector<size_t>> grid;
    for (size_t s = 0; s < segments.size(); ++s) {
        const int x0 = cell_of(std::min(segments[s].a.x(), segments[s].b.x())), x1 = cell_of(std::max(segments[s].a.x(), segments[s].b.x()));
        const int y0 = cell_of(std::min(segments[s].a.y(), segments[s].b.y())), y1 = cell_of(std::max(segments[s].a.y(), segments[s].b.y()));
        for (int x = x0; x <= x1; ++x)
            for (int y = y0; y <= y1; ++y)
                grid[{ x, y }].push_back(s);
    }
    EdgeGrid::Grid boundary_grid;
    boundary_grid.set_bbox(get_extents(boundary).inflated(coord_t(scale_(1.))));
    boundary_grid.create(boundary, coord_t(scale_(2.)));
    const auto on_boundary = [&boundary_grid, reach_scaled](const Point &p) { return boundary_grid.closest_point_signed_distance(p, coord_t(reach_scaled)).valid(); };
    // Two ends that meet each other (within a bead of one another, both off the boundary) are joined into one path, as the
    // connector joins two line ends on the boundary; a nub on each would print both pieces twice and fold the path back on itself.
    {
        struct End { size_t path; bool front; Point p; };
        std::vector<End> ends;
        for (size_t i = 0; i < paths.size(); ++i)
            if (paths[i].points.size() >= 2)
                for (const bool front : { true, false }) {
                    const Point p = front ? paths[i].points.front() : paths[i].points.back();
                    if (!on_boundary(p))
                        ends.push_back({ i, front, p });
                }
        std::map<std::pair<int, int>, std::vector<size_t>> end_grid;
        for (size_t e = 0; e < ends.size(); ++e)
            end_grid[{ cell_of(ends[e].p.x()), cell_of(ends[e].p.y()) }].push_back(e);
        std::vector<bool> consumed(ends.size(), false);
        std::vector<size_t> merged_into(paths.size());
        for (size_t i = 0; i < paths.size(); ++i) merged_into[i] = i;
        const auto root = [&merged_into](size_t i) { while (merged_into[i] != i) i = merged_into[i]; return i; };
        for (size_t e = 0; e < ends.size(); ++e) {
            if (consumed[e])
                continue;
            std::optional<size_t> mate;
            double best = 2. * reach_scaled;
            const int cx = cell_of(ends[e].p.x()), cy = cell_of(ends[e].p.y());
            for (int x = cx - 1; x <= cx + 1; ++x)
                for (int y = cy - 1; y <= cy + 1; ++y) {
                    auto it = end_grid.find({ x, y });
                    if (it == end_grid.end())
                        continue;
                    for (const size_t f : it->second) {
                        if (f == e || consumed[f] || root(ends[f].path) == root(ends[e].path))
                            continue;
                        const double d = (ends[f].p - ends[e].p).cast<double>().norm();
                        if (d <= best) { best = d; mate = f; }
                    }
                }
            if (!mate)
                continue;
            // Join: path of e, oriented to end at e's end, then the path of the mate from its meeting end.
            Polyline &a = paths[root(ends[e].path)], &b = paths[root(ends[*mate].path)];
            Points joined;
            const bool a_ends_at_e = (ends[e].front ? a.points.front() : a.points.back()) == ends[e].p;
            const bool b_starts_at_mate = (ends[*mate].front ? b.points.front() : b.points.back()) == ends[*mate].p;
            if (!a_ends_at_e || !b_starts_at_mate)
                continue;    // an end already moved by an earlier join
            joined = a.points;
            if (ends[e].front)
                std::reverse(joined.begin(), joined.end());
            Points tail = b.points;
            if (!ends[*mate].front)
                std::reverse(tail.begin(), tail.end());
            if (tail.front() == joined.back())
                tail.erase(tail.begin());
            append(joined, tail);
            a.points = std::move(joined);
            b.points.clear();
            merged_into[root(ends[*mate].path)] = root(ends[e].path);
            consumed[e] = consumed[*mate] = true;
        }
        paths.erase(std::remove_if(paths.begin(), paths.end(), [](const Polyline &pl) { return pl.points.size() < 2; }), paths.end());
        // The segment index is rebuilt below from the joined paths.
        segments.clear();
        grid.clear();
        for (size_t i = 0; i < paths.size(); ++i)
            for (size_t k = 1; k < paths[i].points.size(); ++k)
                if (paths[i].points[k - 1] != paths[i].points[k])
                    segments.push_back({ i, paths[i].points[k - 1], paths[i].points[k] });
        for (size_t s = 0; s < segments.size(); ++s) {
            const int x0 = cell_of(std::min(segments[s].a.x(), segments[s].b.x())), x1 = cell_of(std::max(segments[s].a.x(), segments[s].b.x()));
            const int y0 = cell_of(std::min(segments[s].a.y(), segments[s].b.y())), y1 = cell_of(std::max(segments[s].a.y(), segments[s].b.y()));
            for (int x = x0; x <= x1; ++x)
                for (int y = y0; y <= y1; ++y)
                    grid[{ x, y }].push_back(s);
        }
    }
    struct Nub { size_t path; bool front; Point q, e; };
    std::vector<Nub> nubs;
    for (size_t i = 0; i < paths.size(); ++i) {
        if (paths[i].points.size() < 2)
            continue;
        for (const bool front : { true, false }) {
            const Point p = front ? paths[i].points.front() : paths[i].points.back();
            if (on_boundary(p))
                continue;
            std::optional<size_t> best;
            double best_distance = reach_scaled;
            const int cx = cell_of(p.x()), cy = cell_of(p.y());
            for (int x = cx - 1; x <= cx + 1; ++x)
                for (int y = cy - 1; y <= cy + 1; ++y) {
                    auto it = grid.find({ x, y });
                    if (it == grid.end())
                        continue;
                    for (const size_t s : it->second) {
                        // Another line, or another part of the same chain (the connector may have joined the met line into this
                        // path along the boundary); never the end's own segment.
                        if (segments[s].path == i && (segments[s].a == p || segments[s].b == p))
                            continue;
                        const double distance = Line(segments[s].a, segments[s].b).distance_to(p);
                        if (distance <= best_distance) {
                            best_distance = distance;
                            best = s;
                        }
                    }
                }
            if (!best)
                continue;
            const Segment &met = segments[*best];
            const Vec2d a = met.a.cast<double>(), d = (met.b - met.a).cast<double>();
            const double length = d.norm();
            if (length < 1.)
                continue;
            // A junction, not a crossing: the path stops at the met line. Its own last segment, extended, meets the line within
            // the reach ahead of the end, or the end lies on the line; a path that runs on through the line has crossed it.
            {
                const Point &before = front ? paths[i].points[1] : paths[i].points[paths[i].points.size() - 2];
                const Vec2d own = (p - before).cast<double>();
                const double own_length = own.norm();
                if (own_length >= 1.) {
                    const Vec2d n(-d.y() / length, d.x() / length);                 // the met line's normal
                    const double side_end = n.dot(p.cast<double>() - a), side_before = n.dot(before.cast<double>() - a);
                    const bool crossed = side_end * side_before < 0. && std::abs(side_end) > scale_(0.01);
                    if (crossed)
                        continue;
                }
            }
            const double t = std::min(1., std::max(0., (p.cast<double>() - a).dot(d) / (length * length)));
            const Vec2d q = a + t * d;
            const double room_forward = (1. - t) * length, room_backward = t * length;
            const double sign = room_forward >= room_backward ? 1. : -1.;
            const double nub = std::min(anchor_length_scaled, sign > 0. ? room_forward : room_backward);
            if (nub < scale_(0.01))
                continue;
            const Vec2d e = q + sign * nub / length * d;
            nubs.push_back({ i, front, Point(coord_t(std::llround(q.x())), coord_t(std::llround(q.y()))), Point(coord_t(std::llround(e.x())), coord_t(std::llround(e.y()))) });
        }
    }
    for (const Nub &nub : nubs) {
        Points &points = paths[nub.path].points;
        if (nub.front) {
            Points head;
            head.push_back(nub.e);
            if (nub.q != points.front())
                head.push_back(nub.q);
            points.insert(points.begin(), head.begin(), head.end());
        } else {
            if (nub.q != points.back())
                points.push_back(nub.q);
            points.push_back(nub.e);
        }
    }
}

} // namespace

void FillBridgeAwareCubic::bind(const BridgeAwareCubic::ObjectPlan &plan, const Layer &layer)
{
    m_plan        = &plan;
    m_layer       = &layer;
    m_layer_index = layer.id() - layer.object()->get_layer(0)->id();
}

Polylines FillBridgeAwareCubic::fill_surface(const Surface *surface, const FillParams &params)
{
    using namespace BridgeAwareCubic;
    // 1. Without a plan, or where the plan has nothing for the island's group, the group prints as Cubic.
    const GroupPlan *group = m_plan != nullptr ? m_plan->group_of_region(this->lock_region_id) : nullptr;
    if (group == nullptr || !group->has_content() || m_layer_index >= group->layer_count() || m_layer == nullptr ||
        this->lock_region_id >= m_layer->regions().size())
        return FillCubic::fill_surface(surface, params);

    const LatticeParams &lattice = group->lattice();
    const ExPolygon &island = surface->expolygon;
    const BoundingBox bbox = get_extents(island);

    // The courses of the regions whose sparse fill this island holds: the group's regions.
    Polylines courses;
    for (const size_t region_id : group->regions())
        append(courses, m_plan->courses_at(m_layer_index, region_id));
    const PrintRegionConfig &config = m_layer->regions()[this->lock_region_id]->region().config();
    const ExPolygons boundary = fill_boundary(island, courses, lattice.spacing, lattice.width, config.infill_wall_overlap.get_abs_value(100.));

    Polylines out;
    // 4. Courses of the layer, clipped to the island, printed whole (each clipped alone: Clipper splits open paths clipped together).
    Polylines course_pieces;
    const Polygons island_polygons = to_polygons(island);
    for (const Polyline &course : courses)
        append(course_pieces, intersection_pl(Polylines{ course }, island_polygons));
    if (boundary.empty()) {
        append(out, std::move(course_pieces));
        return out;
    }
    const AreaIndex index(boundary);

    // 2. Bulk lines: the three families, every sub-line, the extremes included, each clipped to the boundary along its trace.
    Polylines bulk;
    for (const Line &line : bulk_lines(lattice, this->z, bbox))
        for (const auto &interval : index.segment_inside(line.a, line.b)) {
            const Point a = along(line, interval.first), b = along(line, interval.second);
            if (a != b)
                bulk.emplace_back(Points{ a, b });
        }

    // 3. Strands of the layer whose trace meets the island, clipped along their trace; pieces shorter than a bead dropped. An end
    //    that stops within two beads of the fill boundary along its trace is carried on to it (as the plan carries the needed
    //    strands), so that the connector anchors it to the perimeter.
    Polylines strands;
    const double gap_b = std::max(0., m_plan->gap_of_region(this->lock_region_id) - 0.1);    // G_b of the region (μ = 0.1 mm)
    const double shortest = scale_(lattice.width - 1e-4), reach = scale_(0.5 * lattice.width), carry = scale_(std::min(2. * lattice.width, gap_b));
    for (const Line &line : group->strands_at(m_layer_index, bbox))
        for (const auto &interval : index.segment_inside(line.a, line.b)) {
            Point a = along(line, interval.first), b = along(line, interval.second);
            if ((b - a).cast<double>().norm() < shortest)
                continue;
            const Point a0 = a, b0 = b;
            a = carried_to_boundary(b0, a0, index, carry);
            b = carried_to_boundary(a0, b0, index, carry);
            strands.emplace_back(Points{ a, b });
        }

    // 5. Every line goes through the connector as Cubic's do (user decision, 9 November 2026: the pattern's lines are anchored
    //    like Cubic's, with the region's sparse infill anchor settings; the connector's behaviour at the boundary is accepted).
    //    Ends away from the boundary are not the connector's; those that meet another line of the layer get a nub along it.
    Polylines lines = std::move(bulk);
    append(lines, std::move(strands));
    append(lines, std::move(course_pieces));
    Polylines connected;
    if (params.dont_connect() || lines.size() <= 1) {
        if (lines.size() > 1)
            lines = chain_polylines(std::move(lines));
        connected = std::move(lines);
    } else
        connect_infill(std::move(lines), to_polygons(boundary), get_extents(island.contour), connected, this->spacing, params);
    if (!params.dont_connect() && params.anchor_length > 0.f)
        anchor_junctions(connected, boundary, reach, scale_(double(params.anchor_length)));
    append(out, std::move(connected));
    return out;
}

namespace BridgeAwareCubic {

void print_uncovered(Layer & /* layer */)
{
    // Section 3.8's pass, built in a later stage.
}

} // namespace BridgeAwareCubic

} // namespace Slic3r
