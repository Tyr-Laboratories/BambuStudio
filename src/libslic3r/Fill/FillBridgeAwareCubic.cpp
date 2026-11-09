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
#include <tuple>
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

// The parts of a course inside an island, each run on along the course by up to `run_on` (scaled) where the island's edge cut it
// and the course goes on: a surface laid after the plan (the generator's one-spacing ensuring strip along a bridge's edge) cuts a
// course short of the material the plan ended it on (the wall's overlap), and the end it leaves hangs. The run-on stops at the
// course's own end; pieces whose run-ons meet are joined.
Polylines course_inside(const Polyline &course, const BridgeAwareCubic::AreaIndex &island, double run_on)
{
    Polylines out;
    if (course.points.size() < 2)
        return out;
    // Arc positions of the course's vertices, and the arc intervals inside the island.
    std::vector<double> arc(course.points.size(), 0.);
    for (size_t i = 1; i < course.points.size(); ++i)
        arc[i] = arc[i - 1] + (course.points[i] - course.points[i - 1]).cast<double>().norm();
    const double total = arc.back();
    if (total <= 0.)
        return out;
    std::vector<std::pair<double, double>> inside;
    for (size_t i = 1; i < course.points.size(); ++i) {
        const double length = arc[i] - arc[i - 1];
        if (length <= 0.)
            continue;
        for (const auto &interval : island.segment_inside(course.points[i - 1], course.points[i])) {
            const double a = arc[i - 1] + interval.first * length, b = arc[i - 1] + interval.second * length;
            if (!inside.empty() && a <= inside.back().second + 1.)
                inside.back().second = std::max(inside.back().second, b);
            else
                inside.emplace_back(a, b);
        }
    }
    // Run on where the island cut the course (not at the course's own ends), then merge what meets.
    for (auto &interval : inside) {
        if (interval.first > 1.)
            interval.first = std::max(0., interval.first - run_on);
        if (interval.second < total - 1.)
            interval.second = std::min(total, interval.second + run_on);
    }
    std::vector<std::pair<double, double>> merged;
    for (const auto &interval : inside) {
        if (!merged.empty() && interval.first <= merged.back().second + 1.)
            merged.back().second = std::max(merged.back().second, interval.second);
        else
            merged.push_back(interval);
    }
    // The sub-polylines of the course over those arc intervals.
    const auto point_at = [&](double at) {
        const size_t i = size_t(std::upper_bound(arc.begin(), arc.end(), at) - arc.begin());
        if (i == 0)
            return course.points.front();
        if (i >= arc.size())
            return course.points.back();
        const double length = arc[i] - arc[i - 1];
        const double t = length > 0. ? (at - arc[i - 1]) / length : 0.;
        const Vec2d p = course.points[i - 1].cast<double>() + t * (course.points[i] - course.points[i - 1]).cast<double>();
        return Point(coord_t(std::llround(p.x())), coord_t(std::llround(p.y())));
    };
    for (const auto &[a, b] : merged) {
        if (b - a < 1.)
            continue;
        Polyline piece;
        piece.points.push_back(point_at(a));
        for (size_t i = 0; i < course.points.size(); ++i)
            if (arc[i] > a && arc[i] < b && course.points[i] != piece.points.back())
                piece.points.push_back(course.points[i]);
        const Point end = point_at(b);
        if (end != piece.points.back())
            piece.points.push_back(end);
        if (piece.points.size() >= 2)
            out.push_back(std::move(piece));
    }
    return out;
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

// Cuts out of the paths every run of segments that follows the edge of a zone the connector could not see as material (the band
// beside a course of the layer or of the layer below, a combined island's single-layer zone): the connector's boundary leaves
// those zones out, so its hooks and joins can run along their edges, and there they hang over the void. A segment follows an edge
// when its ends and its middle lie within `margin` of the zones' contour (the connector copies the contour's vertices); one that
// lies the same way on the island's own boundary (a hook along the wall, Cubic's) or on a course of the layer (the course itself,
// or a nub along it) is kept. A path is split where a run is cut out of its middle; pieces left with one point are dropped.
void trim_virtual_hooks(Polylines &paths, const ExPolygons &zones, const ExPolygons &island_boundary, const Polylines &courses,
                        double margin_scaled)
{
    if (zones.empty() || paths.empty())
        return;
    EdgeGrid::Grid zone_grid;
    zone_grid.set_bbox(get_extents(zones).inflated(coord_t(scale_(1.))));
    zone_grid.create(zones, coord_t(scale_(2.)));
    EdgeGrid::Grid wall_grid;
    if (!island_boundary.empty()) {
        wall_grid.set_bbox(get_extents(island_boundary).inflated(coord_t(scale_(1.))));
        wall_grid.create(island_boundary, coord_t(scale_(2.)));
    }
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
    const coord_t margin = std::max<coord_t>(1, coord_t(margin_scaled));
    const auto on_edge   = [&](const Point &p) { return zone_grid.closest_point_signed_distance(p, margin).valid(); };
    const auto on_wall   = [&](const Point &p) { return !island_boundary.empty() && wall_grid.closest_point_signed_distance(p, margin).valid(); };
    const auto on_course = [&](const Point &p) { return !course_points.empty() && course_grid.closest_point_signed_distance(p, margin).valid(); };
    const auto virtual_segment = [&](const Point &a, const Point &b) {
        const Point mid((a.x() + b.x()) / 2, (a.y() + b.y()) / 2);
        if (!(on_edge(a) && on_edge(b) && on_edge(mid)))
            return false;
        if (on_course(a) && on_course(b) && on_course(mid))
            return false;    // a course, or a nub along it
        return !(on_wall(a) && on_wall(b) && on_wall(mid));    // along the wall: Cubic's hook, kept
    };
    Polylines out;
    out.reserve(paths.size());
    for (Polyline &pl : paths) {
        const Points &pts = pl.points;
        Polyline piece;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            if (virtual_segment(pts[i], pts[i + 1])) {
                if (piece.points.size() >= 2)
                    out.push_back(std::move(piece));
                piece.points.clear();
                continue;
            }
            if (piece.points.empty())
                piece.points.push_back(pts[i]);
            piece.points.push_back(pts[i + 1]);
        }
        if (piece.points.size() >= 2)
            out.push_back(std::move(piece));
    }
    paths = std::move(out);
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

    // 6. A combined island (3.9a): the plan's lattice at the combined thickness outside the single-layer zone of its group of
    //    layers, and nothing inside it (the pass prints the plan's lines there at layer height on every layer of the group). The
    //    lattice outside is the plan's, so that a zone line on the group's lowest layer rests on the thick line of the group below
    //    as on its own line one layer below (rule i). The zone's edge is not material: no hook runs along it (a virtual zone).
    //    The island's beads stand on the layer below the group's lowest layer, so the zone from below is that layer's. The fill
    //    group carries the combined thickness in millimetres; the layer count comes from the region's final surfaces.
    ExPolygons combined;
    size_t     lowest = m_layer_index;    // the lowest layer this island's beads fill
    if (surface->thickness > m_layer->height + EPSILON || surface->thickness_layers > 1) {
        const auto [top, count] = BridgeAwareCubic::combined_group_of(*m_layer, m_layer_index, this->lock_region_id);
        if (count > 1) {
            combined = combined_zone(*m_plan, top, count);
            lowest   = top + 1 >= count ? top + 1 - count : 0;
        }
    }
    const bool is_combined = lowest != m_layer_index || !combined.empty();

    // The courses of the regions whose sparse fill this island holds: the group's regions. Seam walls and rails cut the fill
    // boundary; a solid column's courses lie inside its faces and do not.
    Polylines courses, boundary_courses, courses_below;
    for (const size_t region_id : group->regions()) {
        append(courses, m_plan->courses_at(m_layer_index, region_id));
        append(boundary_courses, m_plan->boundary_courses_at(m_layer_index, region_id));
        if (lowest > 0)
            append(courses_below, m_plan->boundary_courses_at(lowest - 1, region_id));
    }
    // A course of the layer below that this layer repeats (a seam wall, laid on every layer of its seam) is this layer's own
    // business; only the courses this layer does not have (rails) make a zone from below.
    if (!courses_below.empty() && !boundary_courses.empty())
        courses_below = diff_pl(courses_below, offset(boundary_courses, float(scale_(lattice.width)), ClipperLib::jtRound, float(scale_(0.001))));
    const PrintRegionConfig &config = m_layer->regions()[this->lock_region_id]->region().config();
    const double overlap_percent = config.infill_wall_overlap.get_abs_value(100.);
    ExPolygons boundary = fill_boundary(island, boundary_courses, lattice.spacing, lattice.width, overlap_percent);
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
    if (!combined.empty()) {
        boundary = diff_ex(boundary, combined);
        append(virtual_zones, combined);
        virtual_zones = union_ex(virtual_zones);
    }
    const ExPolygons connector_boundary = virtual_zones.empty() ? boundary : diff_ex(plain_boundary, virtual_zones);
    const AreaIndex below_zone(courses_below.empty() ? ExPolygons()
        : union_ex(offset(courses_below, float(scale_(zone)), ClipperLib::jtRound, float(scale_(0.001)))));
    // The walls' beads and their overlap band: a bulk end whose run-on would land there stays at the fill boundary (Cubic's depth).
    const AreaIndex wall_band(courses_below.empty() ? ExPolygons()
        : offset_ex(m_plan->material(m_layer_index).walls, float(scale_(lattice.spacing)), ClipperLib::jtRound, float(scale_(0.001))));

    Polylines out;
    // 4. Courses of the layer, clipped to the island, printed whole; where the island's edge cut one (a surface laid after the plan),
    //    it runs on along the course across a one-spacing strip and a bead beyond it, so that its end stays on the material the plan
    //    ended it on.
    Polylines course_pieces;
    if (!is_combined) {
        const AreaIndex island_index(ExPolygons{ island });
        const double run_on = scale_(lattice.spacing + lattice.width);
        for (const Polyline &course : courses)
            append(course_pieces, course_inside(course, island_index, run_on));
    }
    if (boundary.empty()) {
        append(out, std::move(course_pieces));
        return out;
    }
    const AreaIndex index(boundary);
    const AreaIndex surface_index(ExPolygons{ island });
    const double spacing_scaled = scale_(lattice.spacing);
    // Where the fill boundary cut a line (a surface laid after the plan took the sparse area there: the generator's ensuring strip
    // one spacing wide along a bridge's edge, a bridge's front above a rail, a thick base), the line runs on past the edge of the
    // sparse surface into the outer bead of what the other surface prints there, half a spacing beyond the edge (a spacing from
    // the cut), so that its end lies on that bead and is fused to it (3.8, rule iv) instead of stopping beside it. `on_surface` are
    // the line's intervals inside the island, `interval` the piece inside the fill boundary; the ends to carry are chosen by the
    // caller (a strand's both; a bulk line's inside the zone of a course of the layer below).
    const auto run_on_to_surface = [spacing_scaled](const Line &line, double length, const std::vector<std::pair<double, double>> &on_surface,
                                                   const std::pair<double, double> &interval, Point &a, Point &b, bool at_a, bool at_b) {
        const double into = 0.5 * spacing_scaled / length;
        for (const auto &s : on_surface) {
            if (at_a && interval.first > 1e-6 && s.first < interval.first - 1e-9 && s.second >= interval.first - 1e-9 && (interval.first - s.first) * length <= spacing_scaled)
                a = along(line, std::max(0., s.first - into));
            if (at_b && interval.second < 1. - 1e-6 && s.second > interval.second + 1e-9 && s.first <= interval.second + 1e-9 && (s.second - interval.second) * length <= spacing_scaled)
                b = along(line, std::min(1., s.second + into));
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
    // The ends carried on to the boundary, with where the plan ended them: a carry the connector did not anchor is undone after it
    // (the carried stretch would hang over the void; the plan held the end where it was).
    std::map<std::pair<coord_t, coord_t>, Point> carried_from;
    for (const Line &line : is_combined ? Lines() : group->strands_at(m_layer_index, bbox)) {
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
                if (a != a0)
                    carried_from.emplace(std::make_pair(a.x(), a.y()), a0);
                if (b != b0)
                    carried_from.emplace(std::make_pair(b.x(), b.y()), b0);
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
        trim_virtual_hooks(connected, virtual_zones, plain_boundary, courses, scale_(m_plan->path_margin()));
    if (!carried_from.empty())
        for (Polyline &path : connected) {
            if (path.points.size() < 2)
                continue;
            for (Point *end : { &path.points.front(), &path.points.back() }) {
                const auto found = carried_from.find(std::make_pair(end->x(), end->y()));
                if (found != carried_from.end())
                    *end = found->second;
            }
        }
    append(out, std::move(connected));
    return out;
}

namespace BridgeAwareCubic {

// The single-layer zone of a combined group of layers ending at `top` and `count` layers tall (3.9a): the union of Z_l over them.
ExPolygons combined_zone(const ObjectPlan &plan, size_t top, size_t count)
{
    ExPolygons zone;
    for (size_t k = 0; k < count && k <= top; ++k)
        append(zone, plan.single_layer_zone(top - k));
    return union_ex(zone);
}

// The combined group a layer belongs to, from the final surfaces of a region: the top layer carries a sparse surface
// `thickness_layers` tall, the layers below it carry void surfaces. Returns (top, count), or count 0 when the layer is not
// combined.
std::pair<size_t, size_t> combined_group_of(const Layer &layer, size_t layer_index, size_t region_id)
{
    const PrintObject &object = *layer.object();
    if (region_id >= layer.regions().size())
        return { 0, 0 };
    const auto surfaces_of = [&](size_t l) -> const Surfaces& { return object.layers()[l]->regions()[region_id]->fill_surfaces.surfaces; };
    bool has_void = false;
    for (const Surface &surface : surfaces_of(layer_index)) {
        if (surface.surface_type == stInternal && surface.thickness_layers > 1)
            return { layer_index, size_t(surface.thickness_layers) };
        if (surface.surface_type == stInternalVoid)
            has_void = true;
    }
    if (!has_void)
        return { 0, 0 };
    // Upward to the group's top: every layer of a group below its top carries a void surface.
    for (size_t b = layer_index + 1; b < object.layers().size(); ++b) {
        if (region_id >= object.layers()[b]->regions().size())
            break;
        bool void_here = false;
        for (const Surface &surface : surfaces_of(b)) {
            if (surface.surface_type == stInternal && surface.thickness_layers > 1 && b + 1 >= layer_index + size_t(surface.thickness_layers))
                return { b, size_t(surface.thickness_layers) };
            if (surface.surface_type == stInternalVoid)
                void_here = true;
        }
        if (!void_here)
            break;
    }
    return { 0, 0 };
}

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
    // Under infill combination (3.9a), the single-layer zone of a region's group of layers (per region: regions combine apart).
    std::map<std::pair<size_t, size_t>, ExPolygons> zones;
    const auto zone_of_region = [&](size_t region_id) -> const ExPolygons* {
        if (region_id >= layer.regions().size() || plan->group_of_region(region_id) == nullptr)
            return nullptr;
        const std::pair<size_t, size_t> key = combined_group_of(layer, layer_index, region_id);
        if (key.second < 2)
            return nullptr;
        auto it = zones.find(key);
        if (it == zones.end())
            it = zones.emplace(key, combined_zone(*plan, key.first, key.second)).first;
        return it->second.empty() ? nullptr : &it->second;
    };
    // Anything planned on this layer? A zone counts: it holds the plan's lines of every layer of its group, including the layers
    // that plan nothing of their own.
    bool anything = false;
    for (const GroupPlan &group : plan->groups())
        if (layer_index < group.layer_count() && !group.strands(layer_index).empty())
            anything = true;
    for (size_t r = 0; r < layer.regions().size() && !anything; ++r)
        if (!plan->courses_at(layer_index, r).empty() || zone_of_region(r) != nullptr)
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
        // Uncovered: outside the islands the fillers were handed (each region below leaves out its own combination zone, whose
        // strands print with the zone's lines). Each region keeps its own courses.
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
        // Under infill combination (3.9a) the fillers skipped the single-layer zone of each region's group of layers: the plan's
        // lines of this layer inside it (the bulk lines, and the strands and courses the islands covered) print at layer height,
        // clipped to the zone. Inside the zone each bulk line is held along its length by the same lattice line one layer below
        // (rule i): the zone's lines on the group's other layers, or, under its lowest layer, the thick line of the group below,
        // which is the plan's lattice too. No hook runs along the zone's edge, which is not material; where a line ends on another
        // line of the zone, it gets a nub along it (or the two ends join), with the region's sparse infill anchor settings.
        // A lattice line's trace drifts from layer to layer, so the same zone would cut it at different places on the group's
        // layers: where the zone's edge runs at a slant to the line, a zone line of a higher layer would end beyond the one below
        // it, and where a plane's line lies outside the zone on the group's top layer (the filler prints it thick there, filling
        // every layer of the group) it could lie inside on a lower one and print twice. Each family's lines are therefore cut by
        // the zone as the top layer sees it, carried down with the family's drift: on every layer of the group a plane's line is
        // cut at the same place along it, so that each zone line rests on the one below and the thick line takes over exactly
        // where the zone lines end.
        const auto zone_lines_of = [&](const ExPolygons &zone, size_t region_id, size_t top) {
            Polylines lines;
            const LatticeParams &lattice = group.lattice();
            const double z = plan->print_z(layer_index);
            ExPolygons seen_by_any;    // the zone as every family sees it: the connector's boundary and the trim's virtual zone
            for (int k = 0; k < 3; ++k) {
                ExPolygons seen = zone;
                if (top != layer_index && top < plan->layer_count()) {
                    const Point here = lattice.scaled(trace(lattice, PlaneKey{ k, 0, 0 }, z).origin);
                    const Point there = lattice.scaled(trace(lattice, PlaneKey{ k, 0, 0 }, plan->print_z(top)).origin);
                    for (ExPolygon &part : seen)
                        part.translate(here - there);
                }
                append(seen_by_any, seen);
                const ExPolygons zone_boundary = intersection_ex(seen, boundary.boundary);
                if (zone_boundary.empty())
                    continue;
                const AreaIndex zone_index(zone_boundary);
                const Vec2d direction = lattice.direction(k);
                for (const Line &line : bulk_lines(lattice, z, bbox)) {
                    const Vec2d d = (line.b - line.a).cast<double>();
                    if (std::abs(d.normalized().dot(direction)) < 0.999)
                        continue;    // another family's line
                    for (const auto &interval : zone_index.segment_inside(line.a, line.b)) {
                        const Vec2d a = line.a.cast<double>();
                        const Point pa(coord_t(std::llround(a.x() + interval.first * d.x())), coord_t(std::llround(a.y() + interval.first * d.y())));
                        const Point pb(coord_t(std::llround(a.x() + interval.second * d.x())), coord_t(std::llround(a.y() + interval.second * d.y())));
                        if (pa != pb)
                            lines.emplace_back(Points{ pa, pb });
                    }
                }
            }
            seen_by_any = union_ex(seen_by_any);
            // The connector's boundary as the filler's (3.8): no hook along a course of the layer or a rail of the layer below,
            // whose ends there are junctions; those zones join the zone's edge as edges no hook may run along.
            Polylines zone_courses = plan->boundary_courses_at(layer_index, region_id);
            if (layer_index > 0) {
                Polylines below = plan->boundary_courses_at(layer_index - 1, region_id);
                if (!below.empty() && !zone_courses.empty())
                    below = diff_pl(below, offset(zone_courses, float(scale_(w)), ClipperLib::jtRound, float(scale_(0.001))));
                append(zone_courses, std::move(below));
            }
            ExPolygons virtual_zones = seen_by_any;
            ExPolygons zone_boundary = intersection_ex(seen_by_any, boundary.boundary);
            if (!zone_courses.empty()) {
                const ExPolygons course_zones = union_ex(offset(zone_courses, float(scale_(1.5 * w + 0.5 * lattice.spacing)), ClipperLib::jtRound, float(scale_(0.001))));
                zone_boundary = diff_ex(zone_boundary, course_zones);
                // Kept apart from the zone, not united with it: the connector's boundary has the course zones' edges inside the
                // zone too, and the trim reads every contour of the list.
                append(virtual_zones, course_zones);
            }
            // The strands of the layer inside the zone, on every layer of the group (on its void layers no island was handed), and the
            // courses an island covered there (the filler skipped the zone).
            append(lines, intersection_pl(strands, zone));
            if (!handed.empty())
                append(lines, intersection_pl(intersection_pl(plan->courses_at(layer_index, region_id), handed), zone));
            if (lines.empty() || zone_boundary.empty())
                return lines;
            // Connected as the filler connects its lines (3.8 step 5), with the region's anchor settings: the connector's hooks
            // and joins along the walls bound the zone's corners as they bound the island's; those along the zone's own edge,
            // which is not material, are cut out; an end that meets another line gets a nub.
            const PrintRegionConfig &config = layer.regions()[region_id]->region().config();
            const double spacing = group.lattice().spacing;
            FillParams params;
            params.density           = float(0.01 * config.sparse_infill_density.value);
            params.anchor_length     = float(config.sparse_infill_anchor.percent ? 0.01 * config.sparse_infill_anchor.value * spacing : config.sparse_infill_anchor.value);
            params.anchor_length_max = float(config.sparse_infill_anchor_max.percent ? 0.01 * config.sparse_infill_anchor_max.value * spacing : config.sparse_infill_anchor_max.value);
            params.anchor_length     = std::min(params.anchor_length, params.anchor_length_max);
            if (params.dont_connect() || lines.size() <= 1)
                return lines;
            Polylines connected;
            Fill::connect_infill(std::move(lines), to_polygons(zone_boundary), get_extents(zone_boundary), connected, spacing, params);
            const double gap_b = std::max(0., plan->gap_of_region(region_id) - 0.1);
            if (params.anchor_length > 0.f)
                anchor_junctions(connected, zone_boundary, scale_(0.5 * w), scale_(double(params.anchor_length)), scale_(std::min(w, gap_b)));
            trim_virtual_hooks(connected, virtual_zones, boundary.boundary, plan->courses_at(layer_index, region_id), scale_(plan->path_margin()));
            return connected;
        };
        std::vector<Polylines> zone_lines(group.regions().size());
        bool any_zone = false;
        for (size_t k = 0; k < group.regions().size(); ++k)
            if (const ExPolygons *zone = zone_of_region(group.regions()[k])) {
                zone_lines[k] = zone_lines_of(*zone, group.regions()[k], combined_group_of(layer, layer_index, group.regions()[k]).first);
                any_zone = any_zone || !zone_lines[k].empty();
            }
        if (uncovered_strands.empty() && !any_course && !any_zone)
            continue;
        // Each region prints its own courses and the strands over its slices, as its sparse infill at the sparse flow, one
        // collection per island of the outline (the group key ignores the extruder and the speeds, which are the region's).
        for (size_t k = 0; k < group.regions().size(); ++k) {
            const size_t region_id = group.regions()[k];
            if (region_id >= layer.regions().size())
                continue;
            LayerRegion &layerm = *layer.regions()[region_id];
            Polylines lines = uncovered_courses[k];
            if (!uncovered_strands.empty() && !layerm.slices.empty()) {
                Polylines over_slices = intersection_pl(uncovered_strands, to_polygons(layerm.slices.surfaces));
                if (const ExPolygons *zone = zone_of_region(region_id))
                    over_slices = diff_pl(over_slices, *zone);
                append(lines, std::move(over_slices));
            }
            if (!zone_lines[k].empty() && !layerm.slices.empty())
                append(lines, intersection_pl(zone_lines[k], to_polygons(layerm.slices.surfaces)));
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
