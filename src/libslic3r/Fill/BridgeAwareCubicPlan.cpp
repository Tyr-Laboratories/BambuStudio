#include "BridgeAwareCubicPlan.hpp"

#include "../ClipperUtils.hpp"
#include "../Flow.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"
#include "../PrintConfig.hpp"
#include "../Surface.hpp"

#include "../Geometry/ConvexHull.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <tuple>

namespace Slic3r {
namespace BridgeAwareCubic {

namespace {

constexpr float  ARC_TOLERANCE = float(scale_(0.001));
constexpr double SLIVER_MM     = 0.002;   // numerical slivers of a demand footprint are opened away by this

bool prints_pattern(const PrintRegionConfig &config)
{
    return config.sparse_infill_pattern.value == ipBridgeAwareCubic && config.sparse_infill_density.value > 0. &&
           config.sparse_infill_density.value < 100.;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------------
// GroupPlan

bool GroupPlan::has_content() const
{
    if (m_has_courses)
        return true;
    for (const std::vector<Strand> &layer : m_strands)
        if (!layer.empty())
            return true;
    return false;
}

Line GroupPlan::segment(const Strand &strand, size_t /* layer */, double z, double subline_offset) const
{
    const Trace t = trace(m_lattice, strand.plane, z);
    const Vec2d shift = subline_offset * m_lattice.normal(strand.plane.family);
    return Line(m_lattice.scaled(t.point(strand.t0) + shift), m_lattice.scaled(t.point(strand.t1) + shift));
}

Lines GroupPlan::strands_at(size_t layer, const BoundingBox &bbox) const
{
    Lines out;
    if (layer >= m_strands.size())
        return out;
    // Same-plane strands that overlap or touch print as one line.
    std::vector<Strand> strands = m_strands[layer];
    std::sort(strands.begin(), strands.end(), [](const Strand &a, const Strand &b) { return a.plane != b.plane ? a.plane < b.plane : a.t0 < b.t0; });
    std::vector<Strand> merged_strands;
    for (const Strand &s : strands) {
        if (!merged_strands.empty() && merged_strands.back().plane == s.plane && s.t0 <= merged_strands.back().t1 + 1e-3)    // touching within a micrometre
            merged_strands.back().t1 = std::max(merged_strands.back().t1, s.t1);
        else
            merged_strands.push_back(s);
    }
    const std::vector<double> offsets = m_lattice.subline_offsets();
    for (const Strand &strand : merged_strands)
        for (const double offset : offsets) {
            // The caller gives the z of the layer through the plan; a strand is defined on its layer only.
            Line line = segment(strand, layer, m_layer_z[layer], offset);
            // (A BoundingBox built from one point twice is undefined, and a merge then replaces it: build it by merging both ends.)
            BoundingBox line_bbox;
            line_bbox.merge(line.a);
            line_bbox.merge(line.b);
            if (line_bbox.overlap(bbox))
                out.push_back(line);
        }
    return out;
}

// ---------------------------------------------------------------------------------------------------------------------------
// ObjectPlan

std::unique_ptr<ObjectPlan> ObjectPlan::build_for_verification(const PrintObject &object, const std::function<void()> &cancel)
{
    auto plan = std::make_unique<ObjectPlan>();
    plan->collect_regions(object, true);
    if (plan->m_groups.empty())
        return plan;
    cancel();
    plan->collect_material(object, cancel);
    plan->collect_planes(object, cancel);
    return plan;
}

std::unique_ptr<ObjectPlan> ObjectPlan::build(const PrintObject &object, const std::function<void()> &cancel)
{
    auto plan = std::make_unique<ObjectPlan>();
    plan->collect_regions(object);
    if (plan->m_groups.empty())
        return plan;
    cancel();
    plan->collect_material(object, cancel);
    plan->collect_seam_walls(object, cancel);
    plan->collect_planes(object, cancel);
    // Rails (3.5b) widen the footprints here; built in a later stage.
    plan->collect_boundaries(object, cancel);
    for (GroupPlan &group : plan->m_groups) {
        refine(group, *plan, cancel);
        sweep(group, *plan, cancel);
    }
    return plan;
}

void ObjectPlan::collect_regions(const PrintObject &object, bool any_sparse_pattern)
{
    const Print &print = *object.print();
    const size_t region_count = object.num_printing_regions();
    m_group_of_region.assign(region_count, -1);
    m_gap_of_region.assign(region_count, 0.);
    m_bead_of_region.assign(region_count, 0.);
    const BoundingBox object_bbox = object.bounding_box();
    // Group key: (pitch, θ_0, s_f, N).
    std::map<std::tuple<coord_t, double, double, int>, size_t> groups_by_key;
    for (size_t region_id = 0; region_id < region_count; ++region_id) {
        const PrintRegion       &region = object.printing_region(region_id);
        const PrintRegionConfig &config = region.config();
        const double nozzle_perimeter = print.config().nozzle_diameter.get_at(region.extruder(frPerimeter) - 1);
        const Flow   perimeter        = region.flow(object, frPerimeter, object.config().layer_height, false);
        const Flow   external         = region.flow(object, frExternalPerimeter, object.config().layer_height, false);
        const Flow   solid            = region.flow(object, frSolidInfill, object.config().layer_height, false);
        const double bridge_bead      = nozzle_perimeter * std::sqrt(std::max(0.01, config.bridge_flow.value));
        m_bead_of_region[region_id]   = std::min({ double(perimeter.width()), double(external.width()), double(solid.width()), bridge_bead });
        const bool is_sparse = config.sparse_infill_density.value > 0. && config.sparse_infill_density.value < 100.;
        if (!(any_sparse_pattern ? is_sparse : prints_pattern(config)))
            continue;
        const double nozzle_sparse = print.config().nozzle_diameter.get_at(region.extruder(frInfill) - 1);
        const ConfigOptionDef *gap_def = print_config_def.get("bridge_aware_cubic_max_gap");
        const double gap_max = gap_def != nullptr ? gap_def->max : 1000.;
        m_gap_of_region[region_id] = std::min(std::max(config.bridge_aware_cubic_max_gap.value, nozzle_sparse), gap_max);
        const Flow   sparse  = region.flow(object, frInfill, object.config().layer_height, false);
        const float  density = float(0.01 * float(config.sparse_infill_density.value));
        LatticeParams lattice = LatticeParams::make(config.infill_direction.value, density, sparse.spacing(), sparse.width(),
                                                    int(config.fill_multiline.value), object_bbox);
        const auto key = std::make_tuple(lattice.pitch, lattice.theta[0], lattice.spacing, lattice.multiline);
        auto it = groups_by_key.find(key);
        if (it == groups_by_key.end()) {
            it = groups_by_key.emplace(key, m_groups.size()).first;
            m_groups.emplace_back();
            m_groups.back().m_lattice = lattice;
            m_groups_regions.emplace_back();
        }
        GroupPlan &group = m_groups[it->second];
        group.m_regions.push_back(region_id);
        group.m_bridge_layers = std::max(group.m_bridge_layers, std::max(1, std::min(3, int(config.internal_bridge_layers.value))));
        m_groups_regions[it->second].push_back(region_id);
        m_group_of_region[region_id] = int(it->second);
    }
    // The output pipeline moves a vertex by at most the G-code's rounding of every coordinate to 1 µm (up to 1.42 µm between two
    // beads' centre lines; the checker's round offsets deviate another 1 µm inward), and a vertex of a path with more than two
    // points also by the simplification of LayerRegion::simplify_entity: the print's `resolution`, or SPARSE_INFILL_RESOLUTION
    // when arcs are fitted to sparse infill. The plan's lines are two-point paths; the stock pipeline's paths are not.
    m_vertex_margin = 0.0025;
    const double simplification = print.config().enable_arc_fitting.value ? std::max(print.config().resolution.value, double(SPARSE_INFILL_RESOLUTION))
                                                                            : print.config().resolution.value;
    m_path_margin = simplification + m_vertex_margin;
    const size_t layers = object.layers().size();
    m_layer_heights.resize(layers);
    m_overhang.resize(layers);
    for (size_t i = 0; i < layers; ++i) {
        const Layer &layer = *object.layers()[i];
        m_layer_heights[i] = { layer.print_z, layer.height };
        // The perimeter generator's overhang distance: half the nozzle diameter of the extruder that prints the walls.
        double nozzle = 0.4;
        if (!layer.regions().empty())
            nozzle = print.config().nozzle_diameter.get_at(layer.regions().front()->region().extruder(frPerimeter) - 1);
        m_overhang[i] = 0.5 * nozzle;
    }
    m_courses.assign(layers, std::vector<Polylines>(region_count));
    m_boundary_courses.assign(layers, std::vector<Polylines>(region_count));
    // Whether the connector anchors ends and the filler makes nubs: Fill.cpp's reading of the region's anchor settings.
    for (GroupPlan &group : m_groups) {
        const PrintRegionConfig &config = object.printing_region(group.m_regions.front()).config();
        const double spacing = group.m_lattice.spacing;
        double anchor = config.sparse_infill_anchor.value, anchor_max = config.sparse_infill_anchor_max.value;
        if (config.sparse_infill_anchor.percent)
            anchor *= 0.01 * spacing;
        if (config.sparse_infill_anchor_max.percent)
            anchor_max *= 0.01 * spacing;
        anchor = std::min(anchor, anchor_max);
        group.m_anchors = anchor_max >= 0.05 && anchor > 0.;
    }
    for (GroupPlan &group : m_groups) {
        group.m_boundaries.resize(layers);
        group.m_strands.resize(layers);
        group.m_layer_z.resize(layers);
        for (size_t i = 0; i < layers; ++i)
            group.m_layer_z[i] = m_layer_heights[i].first;
    }
}

void ObjectPlan::collect_material(const PrintObject &object, const std::function<void()> &cancel)
{
    const size_t layers = object.layers().size();
    m_materials.resize(layers);
    for (size_t i = 0; i < layers; ++i) {
        cancel();
        m_materials[i] = printed_material(*object.layers()[i], m_groups_regions);
    }
}

// Section 3.5a: seam walls. Where the sparse fill of a region of the pattern borders sparse fill of another lattice (another
// group or another pattern), a 0 % region or the air outside a part without walls, no lattice line belongs to both sides and the
// two voids would join into one opening that no refinement can bound. One bead of the region's sparse infill then runs just
// inside its boundary: it separates the voids, the lattice ends on it, and it is material of its layer on both sides from the
// moment it is planned. Between two of the pattern's groups the region with the lower index lays the wall; against another
// pattern, a 0 % region or air the pattern's region always does. A boundary is open by what lies next to it, never by what is
// missing: material of any shape opens nothing.
void ObjectPlan::collect_seam_walls(const PrintObject &object, const std::function<void()> &cancel)
{
    constexpr double DELTA_S = 0.01;    // δ_s: a boundary is open where it lies within this of the neighbour, mm
    const size_t layers = object.layers().size();
    const size_t region_count = m_group_of_region.size();
    for (size_t n = 0; n < layers; ++n) {
        cancel();
        const Layer &layer = *object.layers()[n];
        // The sparse surfaces of every region of the layer, and what each is to its neighbours: beads (sparse fill of any
        // lattice), a void (a 0 % region) or material (100 %).
        std::vector<ExPolygons> sparse(region_count);
        std::vector<char>       kind(region_count, 0);
        std::set<int>           groups_present;
        bool other_lattice = false, void_region = false, all_walled = true;
        for (size_t r = 0; r < region_count && r < layer.regions().size(); ++r) {
            const LayerRegion &lr = *layer.regions()[r];
            for (const Surface &surface : lr.fill_surfaces.surfaces)
                if (surface.surface_type == stInternal)
                    sparse[r].push_back(surface.expolygon);
            if (sparse[r].empty())
                continue;
            sparse[r] = union_ex(sparse[r]);
            const PrintRegionConfig &config = lr.region().config();
            const double density = config.sparse_infill_density.value;
            kind[r] = density <= 0. ? 'v' : density < 100. ? 'b' : 'm';
            if (kind[r] == 'v')
                void_region = true;
            if (kind[r] == 'b') {
                if (m_group_of_region[r] >= 0)
                    groups_present.insert(m_group_of_region[r]);
                else
                    other_lattice = true;
            }
            if (config.wall_loops.value <= 0)
                all_walled = false;
        }
        if (groups_present.empty())
            continue;
        // A layer whose sparse fill is all of one group, each region with walls and none at 0 %, has no seam.
        if (groups_present.size() == 1 && !other_lattice && !void_region && all_walled)
            continue;
        for (size_t r = 0; r < region_count; ++r) {
            if (kind[r] != 'b' || m_group_of_region[r] < 0)
                continue;
            const size_t g = size_t(m_group_of_region[r]);
            GroupPlan   &group = m_groups[g];
            const double w = group.m_lattice.width;
            Polygons bead_neighbours, void_neighbours;
            for (size_t q = 0; q < region_count; ++q) {
                if (q == r || kind[q] == 0 || kind[q] == 'm')
                    continue;
                if (kind[q] == 'v') {
                    append(void_neighbours, to_polygons(sparse[q]));
                    continue;
                }
                const int gq = m_group_of_region[q];
                if (gq < 0 || (size_t(gq) != g && r < q))
                    append(bead_neighbours, to_polygons(sparse[q]));
            }
            const Polylines boundary = to_polylines(sparse[r]);
            Polylines open_beads, open_void;
            if (!bead_neighbours.empty())
                open_beads = intersection_pl(boundary, offset(bead_neighbours, float(scale_(DELTA_S))));
            if (!void_neighbours.empty())
                open_void = intersection_pl(boundary, offset(void_neighbours, float(scale_(DELTA_S))));
            // The air outside a part without walls: the boundary within δ_s of the outside of the outline.
            if (!all_walled)
                append(open_void, diff_pl(boundary, offset_ex(layer.lslices, -float(scale_(DELTA_S)))));
            Polylines walls;
            const auto lay = [&](const Polylines &open, double inset) {
                if (open.empty())
                    return;
                const ExPolygons inset_area = offset_ex(sparse[r], -float(scale_(inset)), ClipperLib::jtRound, ARC_TOLERANCE);
                if (inset_area.empty())
                    return;
                // The inset contour within inset + δ_s of the open boundary, carried round the corner for one bead width at each end
                // of an open stretch so that it still meets the material after path simplification (the reach from the stretch's end
                // point to a contour point one bead past the inset corner). A closed contour comes back split at its start point:
                // pieces that meet end to end are joined before the length test. Pieces shorter than a bead are dropped (one
                // threshold for "a bead long", shared with the strands).
                const double reach = std::sqrt((inset + w) * (inset + w) + inset * inset) + DELTA_S;
                const Polygons near_open = offset(open, float(scale_(reach)), ClipperLib::jtRound, ARC_TOLERANCE, ClipperLib::etOpenRound);
                Polylines pieces = intersection_pl(to_polylines(inset_area), near_open);
                for (bool joined = true; joined;) {
                    joined = false;
                    for (size_t i = 0; i < pieces.size() && !joined; ++i)
                        for (size_t j = 0; j < pieces.size() && !joined; ++j) {
                            if (i == j || pieces[i].points.size() < 2 || pieces[j].points.size() < 2)
                                continue;
                            Points &a = pieces[i].points, &b = pieces[j].points;
                            if (a.back() == b.front()) {
                                a.insert(a.end(), b.begin() + 1, b.end());
                            } else if (a.back() == b.back()) {
                                a.insert(a.end(), b.rbegin() + 1, b.rend());
                            } else if (a.front() == b.back()) {
                                a.insert(a.begin(), b.begin(), b.end() - 1);
                            } else if (a.front() == b.front()) {
                                a.insert(a.begin(), b.rbegin(), b.rend() - 1);
                            } else
                                continue;
                            pieces.erase(pieces.begin() + j);
                            joined = true;
                        }
                }
                for (Polyline &piece : pieces)
                    if (piece.length() >= scale_(w - 1e-4))
                        walls.push_back(std::move(piece));
            };
            lay(open_beads, 0.5 * w - 0.05);    // the 0.05 mm overlap closes the channel the neighbour's butt ends would leave
            lay(open_void, 0.5 * w);
            if (walls.empty())
                continue;
            // A lowest course over another pattern's void, a 0 % region or air, beyond h_n of the material below, cannot be bounded
            // by the refinement: it prints as planned and a development build warns (5.2). Over the group's own void the plane
            // beneath it holds it (3.4).
            if (n > 0) {
                const LayerMaterial &below = m_materials[n - 1];
                const Polygons supported = offset(below.surely_prints, float(scale_(m_overhang[n])), ClipperLib::jtRound, ARC_TOLERANCE);
                for (const Polyline &hanging : diff_pl(walls, supported)) {
                    if (hanging.points.size() < 2)
                        continue;
                    if (g < below.void_of_group.size() && !intersection_pl(Polylines{ hanging }, below.void_of_group[g]).empty())
                        continue;
                    const Point mid = hanging.points[hanging.points.size() / 2];
                    m_statistics.courses_over_other_void.push_back(Place{ n, Vec2d(unscale<double>(mid.x()), unscale<double>(mid.y())), "seam wall" });
                }
            }
            add_courses(n, r, walls, true);
            group.mark_courses();
            BridgeAwareCubic::add_courses(m_materials[n], walls, w, layer, m_groups_regions);
        }
    }
}

void ObjectPlan::collect_planes(const PrintObject &object, const std::function<void()> &cancel)
{
    const size_t layers = object.layers().size();
    for (size_t n = 1; n < layers; ++n) {
        cancel();
        const LayerMaterial &upper = m_materials[n], &lower = m_materials[n - 1];
        if (upper.may_print.empty())
            continue;
        // The material that may print on n beyond the support that surely prints on n − 1.
        const ExPolygons unsupported = diff_ex(upper.may_print,
            offset_ex(lower.surely_prints, float(scale_(m_overhang[n])), ClipperLib::jtRound, ARC_TOLERANCE));
        if (unsupported.empty())
            continue;
        const Layer &below = *object.layers()[n - 1];
        for (size_t g = 0; g < m_groups.size(); ++g) {
            GroupPlan &group = m_groups[g];
            if (g >= lower.void_of_group.size() || lower.void_of_group[g].empty())
                continue;
            ExPolygons demand = intersection_ex(unsupported, lower.void_of_group[g]);
            demand = opening_ex(demand, float(scale_(SLIVER_MM)), ClipperLib::jtRound, ARC_TOLERANCE);
            if (demand.empty())
                continue;
            // The gap of a piece is the gap of the region whose sparse surface at n − 1 lies nearest; where the group's regions
            // differ in gap, the demand is split by their surfaces grown by s_f.
            std::vector<std::pair<double, ExPolygons>> pieces;
            bool one_gap = true;
            for (const size_t region_id : group.m_regions)
                if (m_gap_of_region[region_id] != m_gap_of_region[group.m_regions.front()])
                    one_gap = false;
            if (one_gap) {
                pieces.emplace_back(m_gap_of_region[group.m_regions.front()], std::move(demand));
            } else {
                for (const size_t region_id : group.m_regions) {
                    if (region_id >= below.regions().size())
                        continue;
                    Polygons sparse;
                    for (const Surface &surface : below.regions()[region_id]->fill_surfaces.surfaces)
                        if (surface.surface_type == stInternal)
                            append(sparse, to_polygons(surface.expolygon));
                    if (sparse.empty())
                        continue;
                    ExPolygons part = intersection_ex(demand, offset_ex(union_ex(sparse), float(scale_(group.m_lattice.spacing)), ClipperLib::jtRound, ARC_TOLERANCE));
                    if (!part.empty())
                        pieces.emplace_back(m_gap_of_region[region_id], std::move(part));
                }
            }
            // The roof's narrowest bead that will cross the void: the narrowest wall or solid bead of the roof's regions, or the
            // internal bridge strand. The stock candidate test decides which in a later stage; the narrowest is conservative.
            double bead = std::numeric_limits<double>::max();
            for (size_t r = 0; r < object.layers()[n]->regions().size() && r < m_bead_of_region.size(); ++r)
                bead = std::min(bead, m_bead_of_region[r]);
            if (bead == std::numeric_limits<double>::max())
                bead = 2. * m_overhang[n];
            for (auto &[gap, footprint] : pieces)
                for (int j = 0; j <= group.m_bridge_layers; ++j) {
                    if (int(n) - 1 - j < 0)
                        break;
                    Plane plane;
                    plane.roof_layer  = n;
                    plane.plane_layer = n - 1 - j;
                    plane.gap         = gap;
                    plane.bead        = bead;
                    plane.footprint   = footprint;
                    plane.bbox        = get_extents(footprint);
                    group.m_planes.push_back(std::move(plane));
                }
        }
    }
    for (GroupPlan &group : m_groups)
        std::sort(group.m_planes.begin(), group.m_planes.end(), [](const Plane &a, const Plane &b) {
            return a.plane_layer != b.plane_layer ? a.plane_layer < b.plane_layer : a.roof_layer < b.roof_layer;
        });
}

void ObjectPlan::collect_boundaries(const PrintObject &object, const std::function<void()> &cancel)
{
    const size_t layers = object.layers().size();
    for (size_t i = 0; i < layers; ++i) {
        cancel();
        const Layer &layer = *object.layers()[i];
        std::vector<SparseIsland> islands = sparse_islands(layer);
        for (size_t g = 0; g < m_groups.size(); ++g) {
            GroupPlan &group = m_groups[g];
            const PrintRegionConfig &config = object.printing_region(group.m_regions.front()).config();
            Polylines courses;
            for (const size_t region_id : group.m_regions)
                append(courses, m_boundary_courses[i][region_id]);
            ExPolygons boundary;
            for (const SparseIsland &island : islands) {
                bool of_group = false;
                for (const size_t region_id : island.region_ids)
                    if (region_id < m_group_of_region.size() && m_group_of_region[region_id] == int(g))
                        of_group = true;
                if (!of_group)
                    continue;
                append(boundary, fill_boundary(island.island, courses, group.m_lattice.spacing, group.m_lattice.width,
                                               config.infill_wall_overlap.get_abs_value(100.)));
            }
            group.m_boundaries[i].boundary = union_ex(boundary);
            group.m_boundaries[i].index    = AreaIndex(group.m_boundaries[i].boundary);
        }
    }
}

const GroupPlan* ObjectPlan::group_of_region(size_t region_id) const
{
    if (region_id >= m_group_of_region.size() || m_group_of_region[region_id] < 0)
        return nullptr;
    return &m_groups[m_group_of_region[region_id]];
}

const Polylines& ObjectPlan::courses_at(size_t layer, size_t region_id) const
{
    if (layer >= m_courses.size() || region_id >= m_courses[layer].size())
        return m_no_courses;
    return m_courses[layer][region_id];
}

const Polylines& ObjectPlan::boundary_courses_at(size_t layer, size_t region_id) const
{
    if (layer >= m_boundary_courses.size() || region_id >= m_boundary_courses[layer].size())
        return m_no_courses;
    return m_boundary_courses[layer][region_id];
}

std::vector<PlaneSummary> ObjectPlan::planes() const
{
    std::vector<PlaneSummary> out;
    for (const GroupPlan &group : m_groups)
        for (const Plane &plane : group.m_planes)
            out.push_back({ plane.roof_layer, plane.plane_layer, plane.gap, area(plane.footprint) * SCALING_FACTOR * SCALING_FACTOR });
    return out;
}

ExPolygons ObjectPlan::single_layer_zone(size_t /* layer */) const
{
    // Section 3.9a, built in a later stage.
    return ExPolygons();
}

double ObjectPlan::gap_of_region(size_t region_id) const
{
    return region_id < m_gap_of_region.size() ? m_gap_of_region[region_id] : 0.;
}

// ---------------------------------------------------------------------------------------------------------------------------
// Later stages

// Section 3.6c, built in the next stage: a cell that fails but whose children would reach 100 % density.
namespace { struct RefinementPlane; }
static void solid_column(GroupPlan &group, ObjectPlan &plan, const Cell &cell, const std::vector<const RefinementPlane*> &failing);

// ---------------------------------------------------------------------------------------------------------------------------
// Section 3.6: refinement. Open cells; needed strands where a plane fails; solid columns where the children would saturate.

namespace {

constexpr double MU          = 0.1;        // the planning margin, mm
constexpr size_t VISIT_WARN  = 1000000;    // development information only
constexpr size_t CANCEL_EACH = 1024;

// The largest distance between two vertices of a polygon.
double diameter_mm(const Points &points)
{
    double best = 0.;
    for (size_t i = 0; i < points.size(); ++i)
        for (size_t j = i + 1; j < points.size(); ++j)
            best = std::max(best, (points[i] - points[j]).cast<double>().norm());
    return unscale<double>(best);
}

// A strip's length: L(X) = per(X)/2 − 4 · area(X) / per(X) (appendix A).
double strip_length_mm(const ExPolygon &shape)
{
    double perimeter = shape.contour.length();
    for (const Polygon &hole : shape.holes)
        perimeter += hole.length();
    if (perimeter <= 0.)
        return 0.;
    const double a = std::abs(shape.area());
    return unscale<double>(0.5 * perimeter - 4. * a / perimeter);
}

// The largest extent of a component under a roof: the hull's diameter or the strip length, whichever is longer.
double span_mm(const ExPolygon &component)
{
    return std::max(diameter_mm(Geometry::convex_hull(component.contour.points).points), strip_length_mm(component));
}

// Whether the convex polygon lies inside the area: every vertex inside and no edge crossing the area's edges.
bool inside_area(const Polygon &polygon, const AreaIndex &area)
{
    if (area.empty())
        return false;
    for (const Point &p : polygon.points)
        if (!area.contains(p))
            return false;
    for (size_t i = 0; i < polygon.points.size(); ++i)
        if (area.segment_crosses_edge(polygon.points[i], polygon.points[(i + 1) % polygon.points.size()]))
            return false;
    return true;
}

// The length (mm) of the longest part of the segment that lies inside the area.
double longest_inside_mm(const Point &a, const Point &b, const AreaIndex &area)
{
    double best = 0.;
    const double length = unscale<double>((b - a).cast<double>().norm());
    for (const auto &interval : area.segment_inside(a, b))
        best = std::max(best, (interval.second - interval.first) * length);
    return best;
}

// The convex polygon shrunk by `distance` (scaled) along the edges flagged in `shrink`: the intersection of its edges' half
// planes, each moved inward where flagged (Sutherland–Hodgman against every moved half plane).
Polygon shrink_faces(const Polygon &polygon, const std::vector<bool> &shrink, double distance)
{
    const size_t n = polygon.points.size();
    const bool ccw = polygon.is_counter_clockwise();
    std::vector<Vec2d> subject;
    for (const Point &p : polygon.points)
        subject.push_back(p.cast<double>());
    for (size_t i = 0; i < n && !subject.empty(); ++i) {
        if (!shrink[i])
            continue;
        const Vec2d a = polygon.points[i].cast<double>(), b = polygon.points[(i + 1) % n].cast<double>();
        Vec2d edge = b - a;
        if (edge.norm() < 1.)
            continue;
        // The inward normal of the edge.
        Vec2d inward = ccw ? Vec2d(-edge.y(), edge.x()) : Vec2d(edge.y(), -edge.x());
        inward.normalize();
        const double limit = inward.dot(a) + distance;     // keep inward · p ≥ limit
        std::vector<Vec2d> clipped;
        for (size_t j = 0; j < subject.size(); ++j) {
            const Vec2d &p = subject[j], &q = subject[(j + 1) % subject.size()];
            const double dp = inward.dot(p) - limit, dq = inward.dot(q) - limit;
            if (dp >= 0.)
                clipped.push_back(p);
            if ((dp >= 0.) != (dq >= 0.))
                clipped.push_back(p + (dp / (dp - dq)) * (q - p));
        }
        subject = std::move(clipped);
    }
    Polygon out;
    for (const Vec2d &p : subject)
        out.points.emplace_back(coord_t(std::llround(p.x())), coord_t(std::llround(p.y())));
    return out;
}

struct RefinementPlane
{
    const Plane *plane;
    size_t       group_index;
    ExPolygons   grown;        // D⁺ grown by r/2
    AreaIndex    footprint;    // of D⁺
};

struct RootResult
{
    std::vector<std::pair<size_t, Strand>> strands;   // (layer, strand)
    std::vector<std::pair<Cell, std::vector<const RefinementPlane*>>> solid_columns;
    size_t visits = 0;
};

class Descent
{
public:
    Descent(const GroupPlan &group, const ObjectPlan &plan, const std::vector<RefinementPlane> &planes,
            std::atomic<size_t> &visits, const std::function<void()> &cancel)
        : m_group(group), m_plan(plan), m_planes(planes), m_visits(visits), m_cancel(cancel) {}

    void visit(const Cell &cell, const std::vector<const RefinementPlane*> &candidates, RootResult &out)
    {
        ++out.visits;
        if ((++m_visits % CANCEL_EACH) == 0)
            m_cancel();
        std::vector<const RefinementPlane*> failing;
        const LatticeParams &lattice = m_group.lattice();
        const double band = lattice.band_width();
        for (const RefinementPlane *rp : candidates) {
            const Plane &plane = *rp->plane;
            const double z = m_plan.print_z(plane.plane_layer);
            const auto range = height_range(lattice, cell);
            if (!(z > range.first && z < range.second))
                continue;
            const Polygon section_polygon = section(lattice, cell, z);
            if (section_polygon.points.size() < 3)
                continue;
            if (!section_polygon.bounding_box().overlap(plane.bbox))
                continue;
            const LayerBoundary &boundary = m_group.boundary(plane.plane_layer);
            const double gap_bound = plane.gap - MU;
            // The quick bound: every face prints, and the section's diameter less a band is within the gap.
            if (inside_area(section_polygon, boundary.index) && diameter_mm(section_polygon.points) - band <= gap_bound)
                continue;
            // The cell must meet the footprint.
            if (intersection_ex(ExPolygons{ ExPolygon(section_polygon) }, plane.footprint).empty())
                continue;
            // The void a roof bead can occupy: the section shrunk by half a band along its printed faces, within the group's void,
            // opened by half the roof's bead.
            const size_t n = section_polygon.points.size();
            std::vector<bool> printed(n, false);
            for (size_t i = 0; i < n; ++i)
                printed[i] = longest_inside_mm(section_polygon.points[i], section_polygon.points[(i + 1) % n], boundary.index) >= lattice.width;
            const Polygon shrunk = shrink_faces(section_polygon, printed, scale_(0.5 * band));
            if (shrunk.points.size() < 3)
                continue;
            const LayerMaterial &material = m_plan.material(plane.plane_layer);
            if (rp->group_index >= material.void_of_group.size())
                continue;
            ExPolygons q = intersection_ex(ExPolygons{ ExPolygon(shrunk) }, material.void_of_group[rp->group_index]);
            q = opening_ex(q, float(scale_(0.5 * plane.bead)), ClipperLib::jtRound, ARC_TOLERANCE);
            const ExPolygons f = intersection_ex(q, rp->grown);
            for (const ExPolygon &component : f)
                if (span_mm(component) > gap_bound) {
                    failing.push_back(rp);
                    break;
                }
        }
        if (failing.empty())
            return;
        if (double(lattice.density) * double(int64_t(1) << (cell.depth + 1)) >= 1.) {
            out.solid_columns.emplace_back(cell, failing);
            return;
        }
        needed_strands(cell, failing, out);
        for (const Cell &child : children(cell))
            visit(child, failing, out);
    }

private:
    // The strands of the three new planes under each failing footprint: the plane's chord of the cell's section, within the
    // footprint and the fill boundary, each piece at least a bead long.
    void needed_strands(const Cell &cell, const std::vector<const RefinementPlane*> &failing, RootResult &out)
    {
        const LatticeParams &lattice = m_group.lattice();
        const double w = lattice.width;
        for (const RefinementPlane *rp : failing) {
            const Plane &plane = *rp->plane;
            const size_t m = plane.plane_layer;
            const double z = m_plan.print_z(m);
            const Polygon section_polygon = section(lattice, cell, z);
            if (section_polygon.points.size() < 3)
                continue;
            const LayerBoundary &boundary = m_group.boundary(m);
            if (boundary.index.empty())
                continue;
            for (const PlaneKey &key : split_planes(cell)) {
                const Trace t = trace(lattice, key, z);
                // The chord of the section: the trace's parameters at the section's vertices bound it; clip the long segment.
                double t_lo = std::numeric_limits<double>::max(), t_hi = std::numeric_limits<double>::lowest();
                for (const Point &p : section_polygon.points) {
                    const double tp = t.parameter(lattice.relative(p));
                    t_lo = std::min(t_lo, tp);
                    t_hi = std::max(t_hi, tp);
                }
                // The probe reaches a margin plus the carry past the section, so that a carried end meets a true boundary crossing.
                const double carry = std::min(2. * w, plane.gap - MU);
                const double pad = 1. + carry;
                const Point a = lattice.scaled(t.point(t_lo - pad)), b = lattice.scaled(t.point(t_hi + pad));
                const double length = t_hi - t_lo + 2. * pad;
                const auto to_t = [&](double fraction) { return t_lo - pad + fraction * length; };
                // Inside the section, the footprint and the boundary, as parameter intervals.
                const AreaIndex section_index(ExPolygons{ ExPolygon(section_polygon) });
                std::vector<std::pair<double, double>> pieces;
                for (const auto &in_section : section_index.segment_inside(a, b))
                    for (const auto &in_footprint : rp->footprint.segment_inside(a, b)) {
                        const double lo = std::max(in_section.first, in_footprint.first), hi = std::min(in_section.second, in_footprint.second);
                        if (hi > lo)
                            pieces.emplace_back(to_t(lo), to_t(hi));
                    }
                if (pieces.empty())
                    continue;
                for (const auto &in_boundary : boundary.index.segment_inside(a, b)) {
                    const double bl = to_t(in_boundary.first), bh = to_t(in_boundary.second);
                    if (bh - bl < w - 1e-4)
                        continue;    // a boundary interval shorter than a bead: the face is a gap
                    // The interval's ends are boundary crossings unless they are the probe's own ends.
                    const bool bl_crossing = in_boundary.first > 1e-9, bh_crossing = in_boundary.second < 1. - 1e-9;
                    for (const auto &piece : pieces) {
                        double lo = std::max(piece.first, bl), hi = std::min(piece.second, bh);
                        if (hi <= lo)
                            continue;
                        // An end that stops within two beads of the fill boundary along its trace is carried on to it (the filler
                        // does the same before the connector anchors it to the perimeter): the footprint's edge lies half a nozzle
                        // from the wall, on the edge of the material's reach, where rounding would decide whether the end is held;
                        // two beads along the trace reach the boundary from half a bead away at angles down to 15°. The carried
                        // part bridges to the wall's reach, so it is never longer than G_b.
                        if (bl_crossing && lo - bl <= carry)
                            lo = bl;
                        if (bh_crossing && bh - hi <= carry)
                            hi = bh;
                        if (hi - lo < w - 1e-4) {
                            // Lengthen to a bead, centred, then shifted inside the boundary interval.
                            const double mid = 0.5 * (lo + hi);
                            lo = mid - 0.5 * w;
                            hi = mid + 0.5 * w;
                            if (lo < bl) { hi += bl - lo; lo = bl; }
                            if (hi > bh) { lo -= hi - bh; hi = bh; }
                        }
                        out.strands.emplace_back(m, Strand{ key, lo, hi, StrandKind::Needed, plane.gap });
                    }
                }
            }
        }
    }

    const GroupPlan                     &m_group;
    const ObjectPlan                    &m_plan;
    const std::vector<RefinementPlane>  &m_planes;
    std::atomic<size_t>                 &m_visits;
    const std::function<void()>         &m_cancel;
};

// Merges the needed strands of one plane on one layer where they overlap or touch.
void merge_needed(std::vector<Strand> &strands)
{
    std::vector<Strand> needed, other;
    for (const Strand &s : strands)
        (s.kind == StrandKind::Needed ? needed : other).push_back(s);
    std::sort(needed.begin(), needed.end(), [](const Strand &a, const Strand &b) {
        return a.plane != b.plane ? a.plane < b.plane : a.t0 < b.t0;
    });
    std::vector<Strand> merged;
    for (const Strand &s : needed) {
        if (!merged.empty() && merged.back().plane == s.plane && s.t0 <= merged.back().t1 + 1e-9)
            merged.back().t1 = std::max(merged.back().t1, s.t1);
        else
            merged.push_back(s);
    }
    append(merged, other);
    strands = std::move(merged);
}

} // namespace

void refine(GroupPlan &group, ObjectPlan &plan, const std::function<void()> &cancel)
{
    if (group.m_planes.empty())
        return;
    const LatticeParams &lattice = group.m_lattice;
    const size_t group_index = size_t(std::find_if(plan.m_groups.begin(), plan.m_groups.end(), [&group](const GroupPlan &g) { return &g == &group; }) - plan.m_groups.begin());
    // No root when even the largest section of a coarse cell is within the smallest gap of the group.
    double smallest_gap = std::numeric_limits<double>::max();
    for (const Plane &plane : group.m_planes)
        smallest_gap = std::min(smallest_gap, plane.gap);
    if (std::sqrt(2.) * lattice.a0 - lattice.band_width() <= smallest_gap - MU)
        return;
    std::vector<RefinementPlane> planes;
    planes.reserve(group.m_planes.size());
    for (const Plane &plane : group.m_planes) {
        RefinementPlane rp{ &plane, group_index, offset_ex(plane.footprint, float(scale_(0.5 * plane.bead)), ClipperLib::jtRound, ARC_TOLERANCE), AreaIndex(plane.footprint) };
        planes.push_back(std::move(rp));
    }
    // The roots: the coarse cells that meet some footprint at its plane's height, in a fixed order.
    std::vector<Cell> root_cells;
    for (const Plane &plane : group.m_planes) {
        cancel();
        for (const Cell &cell : roots(lattice, plane.bbox, plan.print_z(plane.plane_layer)))
            if (std::none_of(root_cells.begin(), root_cells.end(), [&cell](const Cell &c) {
                    return c.index[0] == cell.index[0] && c.index[1] == cell.index[1] && c.index[2] == cell.index[2]; }))
                root_cells.push_back(cell);
    }
    std::vector<const RefinementPlane*> all_planes;
    for (const RefinementPlane &rp : planes)
        all_planes.push_back(&rp);
    std::atomic<size_t> visits{ 0 };
    std::vector<RootResult> results(root_cells.size());
    Descent descent(group, plan, planes, visits, cancel);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, root_cells.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++i)
            descent.visit(root_cells[i], all_planes, results[i]);
    });
    // Joined in root order, so that the result equals a serial descent.
    size_t total = 0;
    for (RootResult &result : results) {
        total += result.visits;
        for (auto &[layer, strand] : result.strands)
            group.m_strands[layer].push_back(strand);
        for (auto &[cell, failing] : result.solid_columns)
            solid_column(group, plan, cell, failing);
    }
    for (std::vector<Strand> &layer : group.m_strands)
        merge_needed(layer);
    plan.m_statistics.visits += total;
    if (total > VISIT_WARN)
        plan.m_statistics.visit_warning = true;
}

// Section 3.6c: a cell that fails but whose children would reach 100 % density becomes a solid column. Its refinement faces
// print on every layer of the window as strands along the section's edges; between two faces of one family, alternating between
// two families from layer to layer, parallel courses divide the face spacing into ⌈spacing / s_f⌉ equal parts, none on a face,
// each clipped to the section shrunk by W_N/2 − w/4. The window opens on the layer at or just below the height where the section
// first grows past G_b and closes at the highest plane that failed in the cell.
static void solid_column(GroupPlan &group, ObjectPlan &plan, const Cell &cell, const std::vector<const RefinementPlane*> &failing)
{
    const LatticeParams &lattice = group.lattice();
    const double w = lattice.width, band = lattice.band_width();
    double smallest_gap = std::numeric_limits<double>::max();
    size_t close = 0;
    for (const RefinementPlane *rp : failing) {
        smallest_gap = std::min(smallest_gap, rp->plane->gap);
        close = std::max(close, rp->plane->plane_layer);
    }
    const double gap_bound = smallest_gap - MU;
    const auto range = height_range(lattice, cell);
    // The window: from the layer at or just below the height where the section first grows past G_b.
    size_t open = size_t(-1);
    for (size_t l = 0; l <= close && l < plan.layer_count(); ++l) {
        const double z = plan.print_z(l);
        if (z <= range.first)
            continue;
        if (z >= range.second)
            break;
        const Polygon p = section(lattice, cell, z);
        if (p.points.size() >= 3 && diameter_mm(p.points) > gap_bound) {
            open = l > 0 ? l - 1 : 0;
            break;
        }
    }
    if (open == size_t(-1) || open > close)
        return;    // a section too small for a course is covered by its faces' beads
    const double clip = 0.5 * band - 0.25 * w;
    for (size_t l = open; l <= close; ++l) {
        const double z = plan.print_z(l);
        if (!(z > range.first && z < range.second))
            continue;
        const Polygon p = section(lattice, cell, z);
        if (p.points.size() < 3)
            continue;
        // The refinement faces: strands along the section's edges that lie on them.
        for (int k = 0; k < 3; ++k)
            for (const bool upper : { false, true }) {
                const PlaneKey f = face(cell, k, upper);
                if (f.depth == 0)
                    continue;    // a bulk plane prints anyway (face() returns canonical keys)
                const Trace t = trace(lattice, f, z);
                double lo = std::numeric_limits<double>::max(), hi = std::numeric_limits<double>::lowest();
                for (const Point &v : p.points) {
                    const Vec2d rel = lattice.relative(v);
                    if (std::abs(lattice.normal(k).dot(rel - t.origin)) < 1e-6) {
                        const double tv = t.parameter(rel);
                        lo = std::min(lo, tv);
                        hi = std::max(hi, tv);
                    }
                }
                if (hi - lo >= w - 1e-4)
                    group.add_strand(l, Strand{ f, lo, hi, StrandKind::SolidFace, smallest_gap });
            }
        // The courses: one family per layer, alternating.
        const int k = int(l % 2);
        const Trace lower_face = trace(lattice, face(cell, k, false), z), upper_face = trace(lattice, face(cell, k, true), z);
        const double face_spacing = std::abs(lattice.normal(k).dot(upper_face.origin - lower_face.origin));
        const int parts = std::max(1, int(std::ceil(face_spacing / lattice.spacing)));
        if (parts < 2)
            continue;
        const ExPolygons shrunk = offset_ex(ExPolygon(p), float(-scale_(clip)), ClipperLib::jtMiter, 3.);
        if (shrunk.empty())
            continue;
        const AreaIndex shrunk_index(shrunk);
        size_t region_id = group.regions().front();
        Polylines courses;
        const double c0 = lattice.normal(k).dot(lower_face.origin);
        const Vec2d n = lattice.normal(k), d = lattice.direction(k);
        // The chord parameters are bounded by the section's vertices along d.
        double t_lo = std::numeric_limits<double>::max(), t_hi = std::numeric_limits<double>::lowest();
        for (const Point &v : p.points) {
            const double tv = d.dot(lattice.relative(v));
            t_lo = std::min(t_lo, tv);
            t_hi = std::max(t_hi, tv);
        }
        for (int i = 1; i < parts; ++i) {
            const double c = c0 + (face_spacing * i) / parts * (lattice.normal(k).dot(upper_face.origin - lower_face.origin) > 0 ? 1. : -1.);
            const Vec2d origin = c * n;
            const Point a = lattice.scaled(origin + (t_lo - 1.) * d), b = lattice.scaled(origin + (t_hi + 1.) * d);
            for (const auto &interval : shrunk_index.segment_inside(a, b)) {
                const Vec2d pa = origin + (t_lo - 1. + interval.first * (t_hi - t_lo + 2.)) * d;
                const Vec2d pb = origin + (t_lo - 1. + interval.second * (t_hi - t_lo + 2.)) * d;
                if ((pb - pa).norm() >= w - 1e-4)
                    courses.emplace_back(Points{ lattice.scaled(pa), lattice.scaled(pb) });
            }
        }
        if (!courses.empty()) {
            plan.add_courses(l, region_id, courses, false);
            group.mark_courses();
        }
    }
}

// sweep(): Fill/BridgeAwareCubicSweep.cpp.

} // namespace BridgeAwareCubic
} // namespace Slic3r
