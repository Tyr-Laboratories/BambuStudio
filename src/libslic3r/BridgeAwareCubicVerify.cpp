#include "BridgeAwareCubicVerify.hpp"

#include "BridgeAwareCubicMaterial.hpp"
#include "ClipperUtils.hpp"
#include "ExtrusionEntity.hpp"
#include "ExtrusionEntityCollection.hpp"
#include "Fill/BridgeAwareCubicPlan.hpp"
#include "Geometry/ConvexHull.hpp"
#include "Layer.hpp"
#include "Print.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace Slic3r {
namespace BridgeAwareCubic {

namespace {

constexpr double EPSILON_MM    = 0.01;    // touching
constexpr float  ARC_TOLERANCE = float(scale_(0.001));

ExPolygons round_offset(const ExPolygons &area, double delta_mm)
{
    return offset_ex(area, float(scale_(delta_mm)), ClipperLib::jtRound, ARC_TOLERANCE);
}

double diameter_mm(const Points &points)
{
    double best = 0.;
    for (size_t i = 0; i < points.size(); ++i)
        for (size_t j = i + 1; j < points.size(); ++j)
            best = std::max(best, (points[i] - points[j]).cast<double>().norm());
    return unscale<double>(best);
}

double strip_length_mm(const ExPolygon &shape)
{
    double perimeter = shape.contour.length();
    for (const Polygon &hole : shape.holes)
        perimeter += hole.length();
    if (perimeter <= 0.)
        return 0.;
    return unscale<double>(0.5 * perimeter - 4. * std::abs(shape.area()) / perimeter);
}

bool inside(const ExPolygons &area, const Point &p)
{
    return std::any_of(area.begin(), area.end(), [&p](const ExPolygon &part) { return part.contains(p); });
}

// A path of the roof's layer: its centre line, width, role and whether it is open.
struct RoofPath
{
    Polyline      line;
    double        width = 0.;
    ExtrusionRole role = erNone;
    bool          open = true;
};

void collect_paths(const ExtrusionEntity &entity, std::vector<RoofPath> &out)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(&entity)) {
        for (const ExtrusionEntity *child : collection->entities)
            collect_paths(*child, out);
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(&entity)) {
        if (loop->paths.empty())
            return;
        Polyline line;
        for (const ExtrusionPath &path : loop->paths)
            append(line.points, path.polyline.points);
        if (!line.points.empty() && line.points.front() != line.points.back())
            line.points.push_back(line.points.front());
        out.push_back({ std::move(line), loop->paths.front().width, loop->paths.front().role(), false });
    } else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath*>(&entity)) {
        if (multi->paths.empty())
            return;
        Polyline line;
        for (const ExtrusionPath &path : multi->paths)
            append(line.points, path.polyline.points);
        out.push_back({ std::move(line), multi->paths.front().width, multi->paths.front().role(), true });
    } else if (const auto *path = dynamic_cast<const ExtrusionPath*>(&entity)) {
        out.push_back({ path->polyline, path->width, path->role(), true });
    }
}

Polygons footprint_of_layer(const Layer &layer)
{
    Polygons out;
    for (const LayerRegion *region : layer.regions()) {
        region->perimeters.polygons_covered_by_width(out, 0.f);
        region->thin_fills.polygons_covered_by_width(out, 0.f);
        region->fills.polygons_covered_by_width(out, 0.f);
    }
    return out;
}

std::string place(const Vec2d &p)
{
    std::ostringstream text;
    text.precision(2);
    text << std::fixed << "(" << p.x() << ", " << p.y() << ")";
    return text.str();
}

} // namespace

bool VerificationReport::passed() const
{
    return std::all_of(planes.begin(), planes.end(), [](const PlaneReport &p) { return p.passed; });
}

VerificationReport verify(const PrintObject &object, const std::function<void()> &cancel)
{
    const ObjectPlan *plan = object.bridge_aware_cubic_plan();
    if (plan == nullptr)
        return VerificationReport();
    return verify(object, *plan, cancel);
}

VerificationReport verify(const PrintObject &object, const ObjectPlan &plan, const std::function<void()> &cancel)
{
    VerificationReport report;
    const auto &layers = object.layers();
    for (size_t g = 0; g < plan.groups().size(); ++g) {
        const GroupPlan &group = plan.groups()[g];
        const double w = group.lattice().width;
        for (const Plane &plane : group.planes()) {
            cancel();
            if (plane.plane_layer >= layers.size() || plane.roof_layer >= layers.size())
                continue;
            const Layer &below = *layers[plane.plane_layer], &roof_layer = *layers[plane.roof_layer];
            const LayerMaterial &material = plan.material(plane.plane_layer);
            if (g >= material.void_of_group.size())
                continue;
            PlaneReport r;
            r.roof_layer  = plane.roof_layer;
            r.plane_layer = plane.plane_layer;
            r.z           = below.print_z;
            r.gap         = plane.gap;
            r.bead        = plane.bead;
            const auto fail = [&r](const std::string &what, const Vec2d &at) {
                if (r.passed) {
                    r.passed  = false;
                    r.failure = what;
                    r.where   = at;
                }
            };
            // The footprints of what the plane's layer prints, by nominal width.
            const ExPolygons printed = union_ex(footprint_of_layer(below));
            const ExPolygons &void_of_group = material.void_of_group[g];
            // K1(a): the openings of the void under the roof.
            const ExPolygons free_area = diff_ex(below.lslices, printed);
            const ExPolygons opened = opening_ex(free_area, float(scale_(0.5 * plane.bead)), ClipperLib::jtRound, ARC_TOLERANCE);
            for (const ExPolygon &component : intersection_ex(opened, round_offset(plane.footprint, 0.5 * plane.bead))) {
                if (intersection_ex(ExPolygons{ component }, void_of_group).empty())
                    continue;
                ++r.openings;
                const double span = std::max(diameter_mm(Geometry::convex_hull(component.contour.points).points), strip_length_mm(component));
                if (span > r.max_opening)
                    r.max_opening = span;
                if (span > plane.gap)
                    fail("Sparse infill opening of " + std::to_string(span) + " mm under the roof exceeds the gap", unscale(component.contour.centroid()));
            }
            // K1(b): the runs and the free ends of the roof's paths over the void.
            std::vector<RoofPath> paths;
            Polylines course_lines;
            for (const LayerRegion *region : roof_layer.regions()) {
                collect_paths(region->perimeters, paths);
                collect_paths(region->fills, paths);
            }
            for (const size_t region_id : group.regions())
                append(course_lines, plan.courses_at(plane.roof_layer, region_id));
            const Polygons course_beads = offset(course_lines, float(scale_(0.5 * w)), ClipperLib::jtRound, ARC_TOLERANCE);
            const double h_n = plan.overhang_distance(plane.roof_layer);
            const ExPolygons support = round_offset(printed, h_n + EPSILON_MM);
            const ExPolygons near_footprint = round_offset(plane.footprint, 0.5 * plane.bead);
            Polygons walls_above;
            for (const LayerRegion *region : roof_layer.regions())
                region->perimeters.polygons_covered_by_width(walls_above, 0.f);
            for (const RoofPath &path : paths) {
                const bool sparse = path.role == erInternalInfill;
                if (sparse) {
                    // Only the pattern's courses among the sparse paths count as roof material.
                    if (course_beads.empty() || path.line.points.empty())
                        continue;
                    const Point mid = path.line.points[path.line.points.size() / 2];
                    if (!inside(union_ex(course_beads), mid))
                        continue;
                }
                if (intersection_pl(Polylines{ path.line }, near_footprint).empty() && path.role != erBridgeInfill)
                    continue;
                for (const Polyline &run : diff_pl(Polylines{ path.line }, support)) {
                    if (intersection_pl(Polylines{ run }, void_of_group).empty())
                        continue;
                    const double length = unscale<double>(run.length());
                    const bool under_footprint = !intersection_pl(Polylines{ run }, near_footprint).empty();
                    if (path.role == erBridgeInfill && !under_footprint) {
                        ++r.growth_runs;
                        r.max_growth_run = std::max(r.max_growth_run, length);
                        continue;
                    }
                    ++r.runs;
                    r.max_run = std::max(r.max_run, length);
                    if (length > plane.gap)
                        fail(std::string(ExtrusionEntity::role_to_string(path.role)) + " run of " + std::to_string(length) + " mm over the sparse infill exceeds the gap",
                             unscale(run.points[run.points.size() / 2]));
                }
                if (!path.open || path.line.points.size() < 2)
                    continue;
                const ExPolygons reach = round_offset(printed, 0.5 * path.width + EPSILON_MM);
                const ExPolygons wall_overlap = offset_ex(walls_above, float(scale_(0.5 * path.width + EPSILON_MM)), ClipperLib::jtRound, ARC_TOLERANCE);
                for (const Point &end : { path.line.points.front(), path.line.points.back() })
                    if (inside(void_of_group, end) && !inside(reach, end) && !inside(wall_overlap, end)) {
                        if (path.role == erBridgeInfill) {
                            ++r.bridge_free_ends;
                            fail("Bridge end hangs over the sparse infill", unscale(end));
                        } else
                            ++r.other_free_ends;
                    }
            }
            report.planes.push_back(std::move(r));
        }
    }
    return report;
}

std::vector<std::string> plan_warnings(const PrintObject &object)
{
    std::vector<std::string> out;
    const ObjectPlan *plan = object.bridge_aware_cubic_plan();
    if (plan == nullptr)
        return out;
    const PlanStatistics &stats = plan->statistics();
    const std::string name = "Bridge aware cubic, object \"" + object.model_object()->name + "\"";
    const auto region_text = [&](size_t region_id) {
        const PrintRegionConfig &config = object.printing_region(region_id).config();
        std::ostringstream text;
        text << "region at " << config.sparse_infill_density.value << " % with a gap of " << plan->gap_of_region(region_id) << " mm";
        return text.str();
    };
    const auto height = [&](size_t layer) {
        std::ostringstream text;
        text.precision(2);
        text << std::fixed << plan->print_z(layer) << " mm";
        return text.str();
    };
    for (const Place &p : stats.unheld)
        out.push_back(name + ": a sparse infill piece of the plan on the layer at " + height(p.layer) + " is not held from below at " + place(p.where) + " (" + p.reason + ")");
    for (const Place &p : stats.courses_over_other_void)
        out.push_back(name + ": the lowest course of the plan on the layer at " + height(p.layer) + " lies over void of another region or air at " + place(p.where) + " (" + p.reason + ")");
    if (stats.visit_warning)
        out.push_back(name + ": the refinement visited more than 1,000,000 cells (" + std::to_string(stats.visits) + ")");
    for (const size_t layer : stats.thick_layers)
        out.push_back(name + ": the layer at " + height(layer) + " is too thick for the lattice to rest on itself (the drift of a plane's trace exceeds half a bead)");
    for (const GroupPlan &group : plan->groups())
        if (!group.regions().empty())
            (void)region_text(group.regions().front());
    return out;
}

} // namespace BridgeAwareCubic
} // namespace Slic3r
