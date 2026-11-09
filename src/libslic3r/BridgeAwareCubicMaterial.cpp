#include "BridgeAwareCubicMaterial.hpp"

#include "ClipperUtils.hpp"
#include "ExtrusionEntityCollection.hpp"
#include "Fill/Fill.hpp"
#include "Flow.hpp"
#include "Layer.hpp"
#include "Print.hpp"
#include "Surface.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace Slic3r {
namespace BridgeAwareCubic {

namespace {

constexpr double SOLID_RULE    = 0.55;    // printing(A) = opening(A, 0.55 · s_role): a fill lays a line where ≥ 1.1 spacings wide
constexpr float  ARC_TOLERANCE = float(scale_(0.001));
constexpr double COURSE_OVERLAP_MM = 0.05;   // a line ends this far inside a course's bead (3.5a's overlap into the neighbour)

ExPolygons round_offset(const ExPolygons &polygons, double delta_mm)
{
    return offset_ex(polygons, float(scale_(delta_mm)), ClipperLib::jtRound, ARC_TOLERANCE);
}

// The spacing of the flow that prints a non-sparse surface of the region.
double role_spacing(const LayerRegion &region, const Surface &surface)
{
    const PrintObjectConfig &object_config = region.layer()->object()->config();
    if (surface.surface_type == stBottomBridge)
        return region.bridging_flow(frSolidInfill, object_config.thick_bridges).spacing();
    if (surface.surface_type == stInternalBridge)
        return region.bridging_flow(frSolidInfill, !surface.bridge_followup).spacing();
    if (surface.is_top())
        return region.flow(frTopSolidInfill).spacing();
    if (surface.surface_type == stInternal)       // a region at 100 %: solid lines of the sparse flow
        return region.flow(frInfill).spacing();
    return region.flow(frSolidInfill).spacing();
}

bool is_sparse_region(const LayerRegion &region)
{
    const double density = region.region().config().sparse_infill_density.value;
    return density > 0. && density < 100.;
}

// The void components that meet the sparse surfaces of each group's regions.
std::vector<ExPolygons> void_by_group(const ExPolygons &void_area, const Layer &layer, const std::vector<std::vector<size_t>> &groups_regions)
{
    std::vector<ExPolygons> out(groups_regions.size());
    for (size_t g = 0; g < groups_regions.size(); ++g) {
        Polygons sparse;
        for (const size_t region_id : groups_regions[g]) {
            if (region_id >= layer.regions().size())
                continue;
            const LayerRegion &region = *layer.regions()[region_id];
            if (!is_sparse_region(region))
                continue;
            for (const Surface &surface : region.fill_surfaces.surfaces)
                if (surface.surface_type == stInternal)
                    append(sparse, to_polygons(surface.expolygon));
        }
        if (sparse.empty())
            continue;
        const BoundingBox sparse_bbox = get_extents(sparse);
        for (const ExPolygon &component : void_area)
            if (component.contour.bounding_box().overlap(sparse_bbox) && !intersection_ex(ExPolygons{ component }, sparse).empty())
                out[g].push_back(component);
    }
    return out;
}

// Liang–Barsky: the parameter range of a + t (b − a), t in [0, 1], inside a box. Returns false when nothing is inside.
bool clip_to_box(const Point &a, const Point &b, const BoundingBox &box, double &t0, double &t1)
{
    t0 = 0.; t1 = 1.;
    const double dx = double(b.x() - a.x()), dy = double(b.y() - a.y());
    const double p[4] = { -dx, dx, -dy, dy };
    const double q[4] = { double(a.x() - box.min.x()), double(box.max.x() - a.x()), double(a.y() - box.min.y()), double(box.max.y() - a.y()) };
    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0.) {
            if (q[i] < 0.) return false;
        } else {
            const double t = q[i] / p[i];
            if (p[i] < 0.) { if (t > t1) return false; if (t > t0) t0 = t; }
            else           { if (t < t0) return false; if (t < t1) t1 = t; }
        }
    }
    return t0 <= t1;
}

} // namespace

LayerMaterial printed_material(const Layer &layer, const std::vector<std::vector<size_t>> &groups_regions)
{
    LayerMaterial material;
    Polygons wall_beads;
    Polygons may_print, surely;
    for (const LayerRegion *region : layer.regions()) {
        // The walls are final when the plan is built: their beads by nominal width (the gap fill between them included) are the
        // walls on both sides. The surfaces' wall band (lslices less the no-overlap fill area) is wider by the gap the perimeter
        // generator leaves between the last wall and the infill, which prints nothing: counting it as support held strand ends
        // 0.2 to 0.5 mm from any bead (found by the K2 checker on the small cube, 9 November 2026).
        region->perimeters.polygons_covered_by_width(wall_beads, 0.f);
        region->thin_fills.polygons_covered_by_width(wall_beads, 0.f);
        const bool sparse_region = is_sparse_region(*region);
        for (const Surface &surface : region->fill_surfaces.surfaces) {
            if (surface.surface_type == stInternalVoid)
                continue;
            if (surface.surface_type == stInternal && sparse_region)
                continue;
            // A non-sparse surface: solid, top, bottom, a bridge, or the sparse surface of a region at 100 % or 0 %.
            if (surface.surface_type == stInternal && region->region().config().sparse_infill_density.value <= 0.)
                continue;    // a 0 % region prints nothing here: void to its neighbours
            const ExPolygons area{ surface.expolygon };
            append(may_print, to_polygons(area));
            const double s_role = role_spacing(*region, surface);
            append(surely, to_polygons(opening_ex(area, float(scale_(SOLID_RULE * s_role)), ClipperLib::jtRound, ARC_TOLERANCE)));
        }
    }
    material.walls         = intersection_ex(union_ex(wall_beads), layer.lslices);
    append(may_print, to_polygons(material.walls));
    append(surely, to_polygons(material.walls));
    material.may_print     = union_ex(may_print);
    material.surely_prints = union_ex(surely);
    material.void_area     = diff_ex(layer.lslices, material.surely_prints);
    material.void_of_group = void_by_group(material.void_area, layer, groups_regions);
    return material;
}

void add_courses(LayerMaterial &material, const Polylines &courses, double width_mm, const Layer &layer,
                 const std::vector<std::vector<size_t>> &groups_regions)
{
    if (courses.empty())
        return;
    // Round ends: an extruded bead ends in a half disc, and a course that ends inside a wall's bead must seal the void there.
    const Polygons beads = offset(courses, float(scale_(0.5 * width_mm)), ClipperLib::jtRound, ARC_TOLERANCE, ClipperLib::etOpenRound);
    Polygons may = to_polygons(material.may_print), surely = to_polygons(material.surely_prints);
    append(may, beads);
    append(surely, beads);
    material.may_print     = union_ex(may);
    material.surely_prints = union_ex(surely);
    material.void_area     = diff_ex(layer.lslices, material.surely_prints);
    material.void_of_group = void_by_group(material.void_area, layer, groups_regions);
}

std::vector<SparseIsland> sparse_islands(const Layer &layer)
{
    std::vector<SparseIsland> out;
    for (SparseIslandOfPattern &island : sparse_islands_of_pattern(layer, ipBridgeAwareCubic))
        out.push_back({ std::move(island.region_ids), std::move(island.island) });
    return out;
}

ExPolygons fill_boundary(const ExPolygon &island, const Polylines &courses, double spacing_mm, double width_mm)
{
    // FillCubic: ExPolygonWithOffset(expolygon, 0, float(scale_(overlap − 0.5 · spacing))) with overlap 0, miter joins, limit 3.
    const coord_t inset = coord_t(float(scale_(-0.5 * spacing_mm)));
    ExPolygons boundary = offset_ex(island, float(inset), ClipperLib::jtMiter, 3.);
    if (!courses.empty()) {
        // A line ends inside a course's bead, w/2 − 0.05 from its centre line (the user's anchoring rule, 9 November 2026: a line
        // that meets another extrusion is fused to it by a nub along it), not half a spacing short of it as at a perimeter: the
        // course is one bead, often on one layer only (a rail), and a hook along a band half a spacing from it would hang over the
        // void.
        const double keep = std::max(0.01, 0.5 * width_mm - COURSE_OVERLAP_MM);
        boundary = diff_ex(boundary, offset(courses, float(scale_(keep)), ClipperLib::jtRound, ARC_TOLERANCE));
    }
    return boundary;
}

AreaIndex::AreaIndex(ExPolygons polygons, coord_t resolution) : m_polygons(std::move(polygons))
{
    if (m_polygons.empty())
        return;
    m_bbox = get_extents(m_polygons);
    BoundingBox grid_bbox = m_bbox;
    grid_bbox.offset(2 * resolution);
    m_grid = std::make_unique<EdgeGrid::Grid>(grid_bbox);
    m_grid->create(m_polygons, resolution);
    // Whether each cell's centre lies inside, by the ray cast of contains() (+x, an edge counted where exactly one of its ends lies
    // above the ray), one row at a time: the x where the edges cross the row's centre line, and the count of those right of each
    // centre. A query then starts from the centre of its own cell.
    const BoundingBox &g = m_grid->bbox();
    const coord_t res = m_grid->resolution();
    const size_t rows = m_grid->rows(), cols = m_grid->cols();
    std::vector<std::vector<double>> row_crossings(rows);
    for (const EdgeGrid::Contour &contour : m_grid->contours())
        for (size_t i = 0; i < contour.num_segments(); ++i) {
            const Point &s = contour.segment_start(i), &e = contour.segment_end(i);
            if (s.y() == e.y())
                continue;
            const coord_t lo = std::min(s.y(), e.y()), hi = std::max(s.y(), e.y());
            // The rows whose centre y_c satisfies lo <= y_c < hi.
            const int64_t first = std::max<int64_t>(0, (int64_t(lo) - g.min.y() - res / 2 + res - 1) / res);
            for (int64_t r = first; r < int64_t(rows); ++r) {
                const coord_t y_c = g.min.y() + coord_t(r) * res + res / 2;
                if (y_c < lo)
                    continue;
                if (y_c >= hi)
                    break;
                row_crossings[size_t(r)].push_back(double(s.x()) + double(y_c - s.y()) * double(e.x() - s.x()) / double(e.y() - s.y()));
            }
        }
    m_centre_inside.assign(rows * cols, 0);
    for (size_t r = 0; r < rows; ++r) {
        std::vector<double> &xs = row_crossings[r];
        std::sort(xs.begin(), xs.end());
        for (size_t c = 0; c < cols; ++c) {
            const double x_c = double(g.min.x() + coord_t(c) * res + res / 2);
            const size_t right = size_t(xs.end() - std::upper_bound(xs.begin(), xs.end(), x_c));
            m_centre_inside[r * cols + c] = uint8_t(right & 1);
        }
    }
}

std::vector<double> AreaIndex::crossings(const Point &a, const Point &b) const
{
    std::vector<double> out;
    crossings(a, b, out);
    return out;
}

void AreaIndex::crossings(const Point &a, const Point &b, std::vector<double> &out) const
{
    const size_t first = out.size();
    if (!m_grid || a == b)
        return;
    // The edges met so far: an edge spans several cells and counts once. A flat buffer reused by the thread (a query meets a few
    // edges), not a node per edge.
    thread_local std::vector<std::pair<size_t, size_t>> seen;
    seen.clear();
    const Vec2crd d = b - a;
    const auto visitor_fn = [&](coord_t row, coord_t col) {
        const auto range = m_grid->cell_data_range(row, col);
        for (auto it = range.first; it != range.second; ++it) {
            if (std::find(seen.begin(), seen.end(), *it) != seen.end())
                continue;
            seen.push_back(*it);
            const EdgeGrid::Contour &contour = m_grid->contours()[it->first];
            const Point &s = contour.segment_start(it->second), &e = contour.segment_end(it->second);
            // Segment a→b against edge s→e: a + t d = s + u (e − s).
            const Vec2crd f = e - s;
            const int64_t denominator = int64_t(d.x()) * int64_t(f.y()) - int64_t(d.y()) * int64_t(f.x());
            if (denominator == 0)
                continue;
            const Vec2crd as = s - a;
            const int64_t t_num = int64_t(as.x()) * int64_t(f.y()) - int64_t(as.y()) * int64_t(f.x());
            const int64_t u_num = int64_t(as.x()) * int64_t(d.y()) - int64_t(as.y()) * int64_t(d.x());
            const double t = double(t_num) / double(denominator), u = double(u_num) / double(denominator);
            if (t >= 0. && t <= 1. && u >= 0. && u <= 1.)
                out.push_back(t);
        }
        return true;
    };
    struct Visitor { const decltype(visitor_fn) &fn; bool operator()(coord_t row, coord_t col) { return fn(row, col); } } visitor{ visitor_fn };
    m_grid->visit_cells_intersecting_line(a, b, visitor);
    std::sort(out.begin() + first, out.end());
}

bool AreaIndex::contains(const Point &point) const
{
    if (!m_grid || !m_bbox.contains(point))
        return false;
    // From the centre of the point's own cell: the state there, flipped by every edge that the path from the point along y = p.y to
    // the centre's x and then along that x to the centre crosses. Every such edge passes through the cell, so the cell's own edges
    // decide; each leg counts an edge on the ray cast's half-open rule.
    const BoundingBox &g = m_grid->bbox();
    const coord_t res = m_grid->resolution();
    const size_t col = size_t((point.x() - g.min.x()) / res), row = size_t((point.y() - g.min.y()) / res);
    const Point centre(g.min.x() + coord_t(col) * res + res / 2, g.min.y() + coord_t(row) * res + res / 2);
    bool inside = m_centre_inside[row * m_grid->cols() + col] != 0;
    const double x_lo = double(std::min(point.x(), centre.x())), x_hi = double(std::max(point.x(), centre.x()));
    const coord_t y_lo = std::min(point.y(), centre.y()), y_hi = std::max(point.y(), centre.y());
    const auto range = m_grid->cell_data_range(coord_t(row), coord_t(col));
    for (auto it = range.first; it != range.second; ++it) {
        const EdgeGrid::Contour &contour = m_grid->contours()[it->first];
        const Point &s = contour.segment_start(it->second), &e = contour.segment_end(it->second);
        // The leg along y = p.y: the edges whose crossing of that line lies in (x_lo, x_hi], where the ray casts from the point and
        // from (centre.x, p.y) differ.
        if ((s.y() > point.y()) != (e.y() > point.y())) {
            const double x = double(s.x()) + double(point.y() - s.y()) * double(e.x() - s.x()) / double(e.y() - s.y());
            if (x > x_lo && x <= x_hi)
                inside = !inside;
        }
        // The leg along x = centre.x: the edges crossing that line between its ends, where the state along it changes. On the ray
        // cast's rule a point at the crossing's own height reads the side above an edge rising to the right and the side below
        // one falling to the right or flat (a flat edge acts through its ends), so a crossing at the leg's lower end counts only
        // for a rising edge and one at its upper end only for the others.
        if ((s.x() > centre.x()) != (e.x() > centre.x())) {
            const double y = double(s.y()) + double(centre.x() - s.x()) * double(e.y() - s.y()) / double(e.x() - s.x());
            const bool rising = (int64_t(e.y()) - s.y()) * (int64_t(e.x()) - s.x()) > 0;
            if (rising ? (y >= double(y_lo) && y < double(y_hi)) : (y > double(y_lo) && y <= double(y_hi)))
                inside = !inside;
        }
    }
    return inside;
}

std::vector<std::pair<double, double>> AreaIndex::segment_inside(const Point &a, const Point &b) const
{
    std::vector<std::pair<double, double>> out;
    if (!m_grid)
        return out;
    if (a == b) {
        if (contains(a)) out.emplace_back(0., 0.);
        return out;
    }
    double t0, t1;
    if (!clip_to_box(a, b, m_bbox, t0, t1))
        return out;
    const Vec2d da = a.cast<double>(), dd = (b - a).cast<double>();
    const auto at = [&](double t) { return Point(coord_t(std::llround(da.x() + t * dd.x())), coord_t(std::llround(da.y() + t * dd.y()))); };
    // The clipped segment's ends and its crossings between them, in a buffer the thread reuses.
    thread_local std::vector<double> cuts;
    cuts.clear();
    cuts.push_back(t0);
    crossings(at(t0), at(t1), cuts);
    // The crossings came back as parameters of the clipped segment; map them to a→b.
    for (size_t i = 1; i < cuts.size(); ++i)
        cuts[i] = t0 + cuts[i] * (t1 - t0);
    cuts.push_back(t1);
    for (size_t i = 1; i < cuts.size(); ++i) {
        if (cuts[i] - cuts[i - 1] <= 0.)
            continue;
        if (contains(at(0.5 * (cuts[i - 1] + cuts[i])))) {
            if (!out.empty() && std::abs(out.back().second - cuts[i - 1]) < 1e-12)
                out.back().second = cuts[i];
            else
                out.emplace_back(cuts[i - 1], cuts[i]);
        }
    }
    return out;
}

bool AreaIndex::segment_inside_whole(const Point &a, const Point &b) const
{
    if (!m_grid)
        return false;
    if (a == b)
        return false;    // segment_inside gives the degenerate interval [0, 0]
    double t0, t1;
    if (!clip_to_box(a, b, m_bbox, t0, t1))
        return false;
    const Vec2d da = a.cast<double>(), dd = (b - a).cast<double>();
    const auto at = [&](double t) { return Point(coord_t(std::llround(da.x() + t * dd.x())), coord_t(std::llround(da.y() + t * dd.y()))); };
    thread_local std::vector<double> cuts;
    cuts.clear();
    cuts.push_back(t0);
    crossings(at(t0), at(t1), cuts);
    for (size_t i = 1; i < cuts.size(); ++i)
        cuts[i] = t0 + cuts[i] * (t1 - t0);
    cuts.push_back(t1);
    // segment_inside's intervals, merged as it merges them, counted rather than stored.
    size_t count = 0;
    double first = 0., last = 0.;
    for (size_t i = 1; i < cuts.size(); ++i) {
        if (cuts[i] - cuts[i - 1] <= 0.)
            continue;
        if (contains(at(0.5 * (cuts[i - 1] + cuts[i])))) {
            if (count > 0 && std::abs(last - cuts[i - 1]) < 1e-12)
                last = cuts[i];
            else {
                if (++count > 1)
                    return false;
                first = cuts[i - 1];
                last  = cuts[i];
            }
        }
    }
    return count == 1 && first <= 1e-9 && last >= 1. - 1e-9;
}

bool AreaIndex::segment_crosses_edge(const Point &a, const Point &b) const
{
    if (!m_grid)
        return false;
    double t0, t1;
    if (!clip_to_box(a, b, m_bbox, t0, t1))
        return false;
    const Vec2d da = a.cast<double>(), dd = (b - a).cast<double>();
    const auto at = [&](double t) { return Point(coord_t(std::llround(da.x() + t * dd.x())), coord_t(std::llround(da.y() + t * dd.y()))); };
    return !crossings(at(t0), at(t1)).empty();
}

} // namespace BridgeAwareCubic
} // namespace Slic3r
