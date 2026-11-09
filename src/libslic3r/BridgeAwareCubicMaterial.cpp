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

ExPolygons fill_boundary(const ExPolygon &island, const Polylines &courses, double spacing_mm, double width_mm,
                         double infill_wall_overlap_percent)
{
    // FillCubic: ExPolygonWithOffset(expolygon, 0, float(scale_(overlap − 0.5 · spacing))) with overlap 0, miter joins, limit 3.
    const coord_t inset = coord_t(float(scale_(-0.5 * spacing_mm)));
    ExPolygons boundary = offset_ex(island, float(inset), ClipperLib::jtMiter, 3.);
    if (!courses.empty()) {
        // A line ends inside a course's bead, w/2 − 0.05 from its centre line (the user's anchoring rule, 9 November 2026: a line
        // that meets another extrusion is fused to it by a nub along it), not half a spacing short of it as at a perimeter: the
        // course is one bead, often on one layer only (a rail), and a hook along a band half a spacing from it would hang over the
        // void. `infill_wall_overlap_percent` no longer enters; the signature stays for the callers.
        (void)infill_wall_overlap_percent;
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
}

std::vector<double> AreaIndex::crossings(const Point &a, const Point &b) const
{
    std::vector<double> out;
    if (!m_grid || a == b)
        return out;
    std::set<std::pair<size_t, size_t>> seen;
    const Vec2crd d = b - a;
    const auto visitor_fn = [&](coord_t row, coord_t col) {
        const auto range = m_grid->cell_data_range(row, col);
        for (auto it = range.first; it != range.second; ++it) {
            if (!seen.insert(*it).second)
                continue;
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
    std::sort(out.begin(), out.end());
    return out;
}

bool AreaIndex::contains(const Point &point) const
{
    if (!m_grid || !m_bbox.contains(point))
        return false;
    // Ray casting along +x to just past the polygons' bounding box, counting proper crossings of the edges.
    const Point far(m_bbox.max.x() + m_grid->resolution(), point.y());
    std::set<std::pair<size_t, size_t>> seen;
    size_t crossings_count = 0;
    const auto visitor_fn = [&](coord_t row, coord_t col) {
        const auto range = m_grid->cell_data_range(row, col);
        for (auto it = range.first; it != range.second; ++it) {
            if (!seen.insert(*it).second)
                continue;
            const EdgeGrid::Contour &contour = m_grid->contours()[it->first];
            const Point &s = contour.segment_start(it->second), &e = contour.segment_end(it->second);
            if ((s.y() > point.y()) == (e.y() > point.y()))
                continue;
            // x of the edge at the ray's height, compared to the point's x: (x − s.x) (e.y − s.y) = (p.y − s.y) (e.x − s.x)
            const double x = double(s.x()) + double(point.y() - s.y()) * double(e.x() - s.x()) / double(e.y() - s.y());
            if (x > double(point.x()))
                ++crossings_count;
        }
        return true;
    };
    struct Visitor { const decltype(visitor_fn) &fn; bool operator()(coord_t row, coord_t col) { return fn(row, col); } } visitor{ visitor_fn };
    m_grid->visit_cells_intersecting_line(point, far, visitor);
    return (crossings_count & 1) == 1;
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
    std::vector<double> cuts = crossings(at(t0), at(t1));
    // The crossings came back as parameters of the clipped segment; map them to a→b.
    for (double &t : cuts)
        t = t0 + t * (t1 - t0);
    cuts.insert(cuts.begin(), t0);
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
