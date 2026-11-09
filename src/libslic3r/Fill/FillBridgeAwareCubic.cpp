#include "FillBridgeAwareCubic.hpp"

#include "BridgeAwareCubicPlan.hpp"
#include "../BridgeAwareCubicMaterial.hpp"
#include "../ClipperUtils.hpp"
#include "../ExtrusionEntityCollection.hpp"
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
// cell's face: a bulk line or a parent strand): two ends that meet each other are joined into one path; an end that meets the
// interior of another line gets a nub along that line's centre line from the junction, `anchor_length` long, clipped to the met
// segment, appended to the path. Ends within `reach` of the fill boundary are the connector's. `join` is the longest link between
// two meeting ends (a bead, or the planning gap when that is smaller).
void anchor_junctions(Polylines &paths, const ExPolygons &boundary, double reach_scaled, double anchor_length_scaled, double join_scaled)
{
    if (paths.empty() || anchor_length_scaled < scale_(0.01))
        return;
    EdgeGrid::Grid boundary_grid;
    boundary_grid.set_bbox(get_extents(boundary).inflated(coord_t(scale_(1.))));
    boundary_grid.create(boundary, coord_t(scale_(2.)));
    const auto on_boundary = [&boundary_grid, reach_scaled](const Point &p) { return boundary_grid.closest_point_signed_distance(p, coord_t(reach_scaled)).valid(); };
    // The interior ends: nothing to do without them (most islands away from the roofs).
    struct End { size_t path; bool front; Point p; };
    std::vector<End> ends;
    for (size_t i = 0; i < paths.size(); ++i)
        if (paths[i].points.size() >= 2)
            for (const bool front : { true, false }) {
                const Point p = front ? paths[i].points.front() : paths[i].points.back();
                if (!on_boundary(p))
                    ends.push_back({ i, front, p });
            }
    if (ends.empty())
        return;
    // The direction of a path's end segment, pointing into the path (the path has at least two points).
    const auto end_direction = [](const Polyline &path, bool front) {
        const Points &pts = path.points;
        const Point &a = front ? pts.front() : pts.back(), &b = front ? pts[1] : pts[pts.size() - 2];
        Vec2d d = (b - a).cast<double>();
        const double n = d.norm();
        return n > 0. ? Vec2d(d / n) : Vec2d(1., 0.);
    };
    // 1. Two ends that meet each other (within `join_scaled`, both off the boundary, their lines not parallel: parallel neighbours
    //    are the sub-lines of a band or two strands side by side, which do not meet) are joined into one path, as the connector
    //    joins two line ends on the boundary; a nub on each would print both pieces twice and fold the path back on itself.
    {
        const coord_t cell = coord_t(scale_(2.));
        const auto cell_of = [cell](coord_t v) { return int(std::floor(double(v) / double(cell))); };
        std::map<std::pair<int, int>, std::vector<size_t>> end_grid;
        for (size_t e = 0; e < ends.size(); ++e)
            end_grid[{ cell_of(ends[e].p.x()), cell_of(ends[e].p.y()) }].push_back(e);
        std::vector<bool> consumed(ends.size(), false);
        std::vector<size_t> merged_into(paths.size());
        for (size_t i = 0; i < paths.size(); ++i) merged_into[i] = i;
        const auto root = [&merged_into](size_t i) { while (merged_into[i] != i) i = merged_into[i]; return i; };
        // An end's path after the joins so far: the root of its merge set, and whether the end is still that path's front or
        // back (an end an earlier join absorbed into the interior takes no further part). The original path of a joined end is
        // empty, so nothing reads it.
        const auto live_end = [&](size_t e, size_t &path_index, bool &front) {
            path_index = root(ends[e].path);
            const Points &pts = paths[path_index].points;
            if (pts.size() < 2)
                return false;
            front = pts.front() == ends[e].p;
            return front || pts.back() == ends[e].p;
        };
        for (size_t e = 0; e < ends.size(); ++e) {
            if (consumed[e])
                continue;
            size_t path_e = 0;
            bool front_e = false;
            if (!live_end(e, path_e, front_e))
                continue;
            const Vec2d de = end_direction(paths[path_e], front_e);
            std::optional<size_t> mate;
            double best = join_scaled;
            const int cx = cell_of(ends[e].p.x()), cy = cell_of(ends[e].p.y());
            for (int x = cx - 1; x <= cx + 1; ++x)
                for (int y = cy - 1; y <= cy + 1; ++y) {
                    auto it = end_grid.find({ x, y });
                    if (it == end_grid.end())
                        continue;
                    for (const size_t f : it->second) {
                        if (f == e || consumed[f])
                            continue;
                        size_t path_f = 0;
                        bool front_f = false;
                        if (!live_end(f, path_f, front_f) || path_f == path_e)
                            continue;
                        const double d = (ends[f].p - ends[e].p).cast<double>().norm();
                        if (d > best)
                            continue;
                        // Parallel lines meet only head on (the gap between them within half a bead).
                        const Vec2d df = end_direction(paths[path_f], front_f);
                        if (std::abs(de.x() * df.y() - de.y() * df.x()) < std::sin(5. * M_PI / 180.)) {
                            const Vec2d gap = (ends[f].p - ends[e].p).cast<double>();
                            if (std::abs(gap.x() * de.y() - gap.y() * de.x()) > 0.5 * reach_scaled)
                                continue;
                        }
                        best = d;
                        mate = f;
                    }
                }
            if (!mate)
                continue;
            Polyline &a = paths[root(ends[e].path)], &b = paths[root(ends[*mate].path)];
            // Orientation from the paths' actual ends (an earlier join may have reversed them).
            const bool e_front = a.points.front() == ends[e].p, e_back = a.points.back() == ends[e].p;
            const bool m_front = b.points.front() == ends[*mate].p, m_back = b.points.back() == ends[*mate].p;
            if ((!e_front && !e_back) || (!m_front && !m_back))
                continue;    // an end already moved by an earlier join
            Points joined = a.points;
            if (e_front && !e_back)
                std::reverse(joined.begin(), joined.end());
            Points tail = b.points;
            if (m_back && !m_front)
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
    }
    // 2. The remaining interior ends: a nub along the line they meet. The paths are indexed by the cells their segments cross.
    ends.clear();
    for (size_t i = 0; i < paths.size(); ++i)
        if (paths[i].points.size() >= 2)
            for (const bool front : { true, false }) {
                const Point p = front ? paths[i].points.front() : paths[i].points.back();
                if (!on_boundary(p))
                    ends.push_back({ i, front, p });
            }
    if (ends.empty())
        return;
    std::vector<Points> as_points;
    as_points.reserve(paths.size());
    for (const Polyline &pl : paths)
        as_points.push_back(pl.points);
    EdgeGrid::Grid grid;
    BoundingBox all;
    for (const Polyline &pl : paths)
        all.merge(get_extents(pl));
    grid.set_bbox(all.inflated(coord_t(scale_(1.))));
    grid.create(as_points, coord_t(scale_(2.)), true);
    struct Nub { size_t path; bool front; Point q, e; };
    std::vector<Nub> nubs;
    for (const End &end : ends) {
        const size_t i = end.path;
        const Point &p = end.p;
        const Point &before = end.front ? paths[i].points[1] : paths[i].points[paths[i].points.size() - 2];
        // The candidate segments within reach: of another path, or of this chain beyond the end's own segment.
        std::optional<Line> best;
        double best_distance = reach_scaled;
        const auto visitor = [&](coord_t row, coord_t col) {
            const auto range = grid.cell_data_range(row, col);
            for (auto it = range.first; it != range.second; ++it) {
                const size_t path_index = it->first;
                const EdgeGrid::Contour &contour = grid.contours()[path_index];
                const Point &a = contour.segment_start(it->second), &b = contour.segment_end(it->second);
                if (path_index == i && (a == p || b == p))
                    continue;    // the end's own segment
                const double distance = Line(a, b).distance_to(p);
                if (distance <= best_distance) {
                    best_distance = distance;
                    best = Line(a, b);
                }
            }
            return true;
        };
        BoundingBox around;    // (a box from one point twice is undefined in this fork: merge, then offset)
        around.merge(p);
        around.offset(coord_t(reach_scaled) + 1);
        grid.visit_cells_intersecting_box(around, visitor);
        if (!best)
            continue;
        const Vec2d a = best->a.cast<double>(), d = (best->b - best->a).cast<double>();
        const double length = d.norm();
        if (length < 1.)
            continue;
        // An end within half a bead of the met line is a junction whether it stops short of the line or overshoots it by up to
        // that much (the beads overlap either way, and the sweep's rule (iv) counts both as fused).
        (void)before;
        const double t = std::min(1., std::max(0., (p.cast<double>() - a).dot(d) / (length * length)));
        const Vec2d q = a + t * d;
        const double room_forward = (1. - t) * length, room_backward = t * length;
        const double sign = room_forward >= room_backward ? 1. : -1.;
        const double nub = std::min(anchor_length_scaled, sign > 0. ? room_forward : room_backward);
        if (nub < scale_(0.01))
            continue;
        const Vec2d e = q + sign * nub / length * d;
        nubs.push_back({ i, end.front, Point(coord_t(std::llround(q.x())), coord_t(std::llround(q.y()))), Point(coord_t(std::llround(e.x())), coord_t(std::llround(e.y()))) });
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

// Trims from each path's ends the segments that run off the lattice directions inside a zone the connector could not see as
// material (the band beside a course of the layer or of the layer below), unless they run along the
// island's own boundary (both ends within half a bead of it: a hook along the wall, Cubic's) or along a course of the layer (the
// course itself, or a nub along it): the connector's boundary excludes those zones, so no line end lies on their edges, but a hook
// from an end on the wall can turn the corner onto such an edge and hang over the void. Nothing on a lattice line is touched; a
// path left with one point is dropped.
void trim_virtual_hooks(Polylines &paths, const ExPolygons &zones, const ExPolygons &island_boundary, const Polylines &courses,
                        const BridgeAwareCubic::LatticeParams &lattice, double reach_scaled)
{
    if (zones.empty() || paths.empty() || island_boundary.empty())
        return;
    const BridgeAwareCubic::AreaIndex zone_index(zones);
    EdgeGrid::Grid wall_grid;
    wall_grid.set_bbox(get_extents(island_boundary).inflated(coord_t(scale_(1.))));
    wall_grid.create(island_boundary, coord_t(scale_(2.)));
    EdgeGrid::Grid course_grid;
    std::vector<Points> course_points;
    for (const Polyline &course : courses)
        if (course.points.size() >= 2)
            course_points.push_back(course.points);
    if (!course_points.empty()) {
        BoundingBox all;
        for (const Points &pts : course_points)
            for (const Point &pt : pts)
                all.merge(pt);
        course_grid.set_bbox(all.inflated(coord_t(scale_(1.))));
        course_grid.create(course_points, coord_t(scale_(2.)), true);
    }
    const auto near_wall   = [&](const Point &p) { return wall_grid.closest_point_signed_distance(p, coord_t(reach_scaled)).valid(); };
    const auto near_course = [&](const Point &p) { return !course_points.empty() && course_grid.closest_point_signed_distance(p, coord_t(reach_scaled)).valid(); };
    const double cos_tolerance = std::cos(2. * M_PI / 180.);
    const auto virtual_segment = [&](const Point &a, const Point &b) {
        const Vec2d d = (b - a).cast<double>();
        const double n = d.norm();
        if (n <= 0.)
            return true;
        for (int k = 0; k < 3; ++k)
            if (std::abs(d.dot(lattice.direction(k))) >= n * cos_tolerance)
                return false;    // on a lattice direction: a line or a nub along one
        const Point mid((a.x() + b.x()) / 2, (a.y() + b.y()) / 2);
        if (!zone_index.contains(mid))
            return false;
        if (near_course(a) && near_course(b))
            return false;    // a course, or a nub along it
        return !(near_wall(a) && near_wall(b));    // along the wall: Cubic's hook, kept; a turn off the wall is not
    };
    for (Polyline &pl : paths) {
        Points &pts = pl.points;
        while (pts.size() >= 2 && virtual_segment(pts[0], pts[1]))
            pts.erase(pts.begin());
        while (pts.size() >= 2 && virtual_segment(pts[pts.size() - 2], pts.back()))
            pts.pop_back();
    }
    paths.erase(std::remove_if(paths.begin(), paths.end(), [](const Polyline &pl) { return pl.points.size() < 2; }), paths.end());
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

    // The courses of the regions whose sparse fill this island holds: the group's regions. Seam walls and rails cut the fill
    // boundary; a solid column's courses lie inside its faces and do not.
    Polylines courses, boundary_courses, courses_below;
    for (const size_t region_id : group->regions()) {
        append(courses, m_plan->courses_at(m_layer_index, region_id));
        append(boundary_courses, m_plan->boundary_courses_at(m_layer_index, region_id));
        if (m_layer_index > 0)
            append(courses_below, m_plan->boundary_courses_at(m_layer_index - 1, region_id));
    }
    // A course of the layer below that this layer repeats (a seam wall, laid on every layer of its seam) is this layer's own
    // business; only the courses this layer does not have (rails) make a zone from below.
    if (!courses_below.empty() && !boundary_courses.empty())
        courses_below = diff_pl(courses_below, offset(boundary_courses, float(scale_(lattice.width)), ClipperLib::jtRound, float(scale_(0.001))));
    const PrintRegionConfig &config = m_layer->regions()[this->lock_region_id]->region().config();
    const double overlap_percent = config.infill_wall_overlap.get_abs_value(100.);
    const ExPolygons boundary = fill_boundary(island, boundary_courses, lattice.spacing, lattice.width, overlap_percent);
    // The connector's boundary: the island's inset without the courses' cut, less a zone around each boundary course of the layer
    // and of the layer below, wide enough that the line ends there lie well inside it. At a course of the layer those ends are
    // junctions with the course (a nub along it, like any line met), not the connector's (a hook along a band beside the course,
    // hanging where the course is on one layer only). Above a rail the bridge it anchors ends a bridge bead past it, and the
    // island's edge there is the bridge's front: a hook along the front would run beside the bridge beads' ends over the void
    // inside the rail (the rib's K2: a 5 mm hook along the front), so the lines end on the front instead and run into it (below).
    const double zone = 1.5 * lattice.width + 0.5 * lattice.spacing;
    Polylines zone_courses = boundary_courses;
    append(zone_courses, courses_below);
    const ExPolygons plain_boundary = zone_courses.empty() ? boundary : fill_boundary(island, Polylines(), lattice.spacing, lattice.width, overlap_percent);
    ExPolygons virtual_zones;    // what the connector's boundary leaves out: no hook may run along their edges (below)
    if (!zone_courses.empty())
        virtual_zones = union_ex(offset(zone_courses, float(scale_(zone)), ClipperLib::jtRound, float(scale_(0.001))));
    const ExPolygons connector_boundary = virtual_zones.empty() ? boundary : diff_ex(plain_boundary, virtual_zones);
    const AreaIndex below_zone(courses_below.empty() ? ExPolygons()
        : union_ex(offset(courses_below, float(scale_(zone)), ClipperLib::jtRound, float(scale_(0.001)))));
    // The walls' beads and their overlap band: a bulk end whose run-on would land there stays at the fill boundary (Cubic's depth).
    const AreaIndex wall_band(courses_below.empty() ? ExPolygons()
        : offset_ex(m_plan->material(m_layer_index).walls, float(scale_(lattice.spacing)), ClipperLib::jtRound, float(scale_(0.001))));

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
    const AreaIndex surface_index(ExPolygons{ island });
    const double spacing_scaled = scale_(lattice.spacing);
    // Where the fill boundary cut a line (a surface laid after the plan took the sparse area there: the generator's ensuring strip
    // one spacing wide along a bridge's edge, a bridge's front above a rail, a thick base), the line runs on to the edge of the
    // sparse surface, at most a spacing further, so that it meets what that surface prints instead of stopping half a spacing
    // short of it. `on_surface` are the line's intervals inside the island, `interval` the piece inside the fill boundary; the
    // ends to carry are chosen by the caller (a strand's both; a bulk line's inside the zone of a course of the layer below).
    const auto run_on_to_surface = [spacing_scaled](const Line &line, double length, const std::vector<std::pair<double, double>> &on_surface,
                                                   const std::pair<double, double> &interval, Point &a, Point &b, bool at_a, bool at_b) {
        for (const auto &s : on_surface) {
            if (at_a && interval.first > 1e-6 && s.first < interval.first - 1e-9 && s.second >= interval.first - 1e-9 && (interval.first - s.first) * length <= spacing_scaled)
                a = along(line, s.first);
            if (at_b && interval.second < 1. - 1e-6 && s.second > interval.second + 1e-9 && s.first <= interval.second + 1e-9 && (s.second - interval.second) * length <= spacing_scaled)
                b = along(line, s.second);
        }
    };

    // 2. Bulk lines: the three families, every sub-line, the extremes included, each clipped to the boundary along its trace. An
    //    end inside the zone of a course of the layer below (a rail: the island's edge there is the front of the bridge the rail
    //    anchors) runs on to the surface's edge, into the bridge beads' ends, since the connector lays no hook there.
    Polylines bulk;
    for (const Line &line : bulk_lines(lattice, this->z, bbox)) {
        std::optional<std::vector<std::pair<double, double>>> on_surface;
        const double length = (line.b - line.a).cast<double>().norm();
        for (const auto &interval : index.segment_inside(line.a, line.b)) {
            Point a = along(line, interval.first), b = along(line, interval.second);
            if (a == b)
                continue;
            if (!below_zone.empty()) {
                const bool at_a = below_zone.contains(a), at_b = below_zone.contains(b);
                if (at_a || at_b) {
                    if (!on_surface)
                        on_surface = surface_index.segment_inside(line.a, line.b);
                    run_on_to_surface(line, length, *on_surface, interval, a, b, at_a, at_b);
                    if (at_a && !wall_band.empty() && wall_band.contains(a))
                        a = along(line, interval.first);
                    if (at_b && !wall_band.empty() && wall_band.contains(b))
                        b = along(line, interval.second);
                }
            }
            bulk.emplace_back(Points{ a, b });
        }
    }

    // 3. Strands of the layer whose trace meets the island, clipped along their trace; pieces shorter than a bead dropped. An end
    //    that stops within two beads of the fill boundary along its trace is carried on to it (as the plan carries the needed
    //    strands), so that the connector anchors it to the perimeter.
    Polylines strands;
    const double gap_b = std::max(0., m_plan->gap_of_region(this->lock_region_id) - 0.1);    // G_b of the region (μ = 0.1 mm)
    // The carry of an end to the boundary exists so that the connector anchors it there like a bulk line's end; without anchors it
    // would only lengthen the line into the void (the plan held the end where it was, not at the boundary).
    const bool anchoring = !params.dont_connect() && params.anchor_length > 0.f;
    const double shortest = scale_(lattice.width - 1e-4), reach = scale_(0.5 * lattice.width), carry = anchoring ? scale_(std::min(2. * lattice.width, gap_b)) : 0.;
    for (const Line &line : group->strands_at(m_layer_index, bbox)) {
        const double length = (line.b - line.a).cast<double>().norm();
        const std::vector<std::pair<double, double>> on_surface = surface_index.segment_inside(line.a, line.b);
        for (const auto &interval : index.segment_inside(line.a, line.b)) {
            Point a = along(line, interval.first), b = along(line, interval.second);
            if (a == b)
                continue;
            const Point a0 = a, b0 = b;
            if (carry > 0.) {
                a = carried_to_boundary(b0, a0, index, carry);
                b = carried_to_boundary(a0, b0, index, carry);
            }
            // Where the fill boundary cut a planned line, it runs on to the sparse surface's edge (above). A line the plan itself
            // ended at the boundary is left as it is.
            run_on_to_surface(line, length, on_surface, interval, a, b, true, true);
            // A bead long after the carries (a piece exactly a bead long that a later surface cut must run on, not vanish).
            if ((b - a).cast<double>().norm() < shortest)
                continue;
            strands.emplace_back(Points{ a, b });
        }
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
        connect_infill(std::move(lines), to_polygons(connector_boundary), get_extents(island.contour), connected, this->spacing, params);
    if (!params.dont_connect() && params.anchor_length > 0.f)
        anchor_junctions(connected, connector_boundary, reach, scale_(double(params.anchor_length)), scale_(std::min(lattice.width, gap_b)));
    if (!virtual_zones.empty() && !params.dont_connect())
        trim_virtual_hooks(connected, virtual_zones, plain_boundary, courses, lattice, reach);
    append(out, std::move(connected));
    return out;
}

namespace BridgeAwareCubic {

// The pass of section 3.8, once per layer after every filler: the plan's lines of the layer that the pattern's fillers did not
// print (strands and courses outside the islands handed to them: the generator and the thick bases take sparse area after the
// plan), kept where no extrusion of the layer prints (perimeters and fills by their nominal width: where what took the area prints
// material, that material holds; where it prints nothing, the plan's line must still print, since the preview showed it and a
// bridge may be anchored on it, and a column above may rest on it), pieces shorter than a bead dropped, printed as sparse infill
// of the region at the sparse flow, one collection per island of the outline. The pass changes no path already made.
void print_uncovered(Layer &layer)
{
    const PrintObject *object = layer.object();
    const ObjectPlan  *plan   = object != nullptr ? object->bridge_aware_cubic_plan() : nullptr;
    if (plan == nullptr || object->layers().empty())
        return;
    const size_t layer_index = layer.id() - object->layers().front()->id();
    if (layer_index >= plan->layer_count())
        return;
    // Anything planned on this layer?
    bool anything = false;
    for (const GroupPlan &group : plan->groups())
        if (layer_index < group.layer_count() && !group.strands(layer_index).empty())
            anything = true;
    for (size_t r = 0; r < layer.regions().size() && !anything; ++r)
        if (!plan->courses_at(layer_index, r).empty())
            anything = true;
    if (!anything)
        return;
    // The islands the fillers were handed (the final surfaces). The footprint of everything the layer prints is built only when a
    // line turns out uncovered, which is rare.
    const std::vector<SparseIsland> islands = sparse_islands(layer);
    const BoundingBox bbox = get_extents(layer.lslices);
    std::optional<ExPolygons> printed_ex;
    const auto printed = [&]() -> const ExPolygons& {
        if (!printed_ex) {
            Polygons footprint;
            for (const LayerRegion *region : layer.regions()) {
                region->perimeters.polygons_covered_by_width(footprint, 0.f);
                region->thin_fills.polygons_covered_by_width(footprint, 0.f);
                region->fills.polygons_covered_by_width(footprint, 0.f);
            }
            printed_ex = union_ex(footprint);
        }
        return *printed_ex;
    };
    for (size_t g = 0; g < plan->groups().size(); ++g) {
        const GroupPlan &group = plan->groups()[g];
        if (layer_index >= group.layer_count() || group.regions().empty())
            continue;
        const double w = group.lattice().width;
        // The group's islands on this layer.
        Polygons handed;
        for (const SparseIsland &island : islands)
            for (const size_t region_id : island.region_ids)
                if (plan->group_of_region(region_id) == &group) {
                    append(handed, to_polygons(island.island));
                    break;
                }
        // The plan's strands of the layer, clipped to the plan's own fill boundary.
        Polylines strands;
        const LayerBoundary &boundary = group.boundary(layer_index);
        for (const Line &line : group.strands_at(layer_index, bbox))
            for (const auto &interval : boundary.index.segment_inside(line.a, line.b)) {
                const Vec2d a = line.a.cast<double>(), d = (line.b - line.a).cast<double>();
                const Point pa(coord_t(std::llround(a.x() + interval.first * d.x())), coord_t(std::llround(a.y() + interval.first * d.y())));
                const Point pb(coord_t(std::llround(a.x() + interval.second * d.x())), coord_t(std::llround(a.y() + interval.second * d.y())));
                if (pa != pb)
                    strands.emplace_back(Points{ pa, pb });
            }
        // Uncovered: outside the islands the fillers were handed. Each region keeps its own courses.
        const Polylines uncovered_strands = strands.empty() ? Polylines() : (handed.empty() ? strands : diff_pl(strands, handed));
        std::vector<Polylines> uncovered_courses(group.regions().size());
        bool any_course = false;
        for (size_t k = 0; k < group.regions().size(); ++k) {
            const Polylines &courses = plan->courses_at(layer_index, group.regions()[k]);
            if (courses.empty())
                continue;
            uncovered_courses[k] = handed.empty() ? courses : diff_pl(courses, handed);
            any_course = any_course || !uncovered_courses[k].empty();
        }
        if (uncovered_strands.empty() && !any_course)
            continue;
        // Each region prints its own courses and the strands over its slices, as its sparse infill at the sparse flow, one
        // collection per island of the outline (the group key ignores the extruder and the speeds, which are the region's).
        for (size_t k = 0; k < group.regions().size(); ++k) {
            const size_t region_id = group.regions()[k];
            if (region_id >= layer.regions().size())
                continue;
            LayerRegion &layerm = *layer.regions()[region_id];
            Polylines lines = uncovered_courses[k];
            if (!uncovered_strands.empty() && !layerm.slices.empty())
                append(lines, intersection_pl(uncovered_strands, to_polygons(layerm.slices.surfaces)));
            if (lines.empty())
                continue;
            if (!printed().empty())
                lines = diff_pl(lines, printed());
            Polylines kept;
            for (Polyline &piece : lines)
                if (piece.length() >= scale_(w - 1e-4))
                    kept.push_back(std::move(piece));
            if (kept.empty())
                continue;
            const Flow flow = layerm.flow(frInfill);
            for (const ExPolygon &outline : layer.lslices) {
                Polylines inside = intersection_pl(kept, outline);
                inside.erase(std::remove_if(inside.begin(), inside.end(), [w](const Polyline &p) { return p.length() < scale_(w - 1e-4); }), inside.end());
                if (inside.empty())
                    continue;
                ExtrusionEntityCollection *collection = new ExtrusionEntityCollection();
                collection->no_sort = false;
                extrusion_entities_append_paths(collection->entities, std::move(inside), erInternalInfill, flow.mm3_per_mm(), float(flow.width()), float(flow.height()));
                layerm.fills.entities.push_back(collection);
            }
        }
    }
}

} // namespace BridgeAwareCubic

} // namespace Slic3r
