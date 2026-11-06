#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSerialization.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSupport.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

using namespace Slic3r;

namespace {

DynamicPrintConfig modifier_config()
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "5%"}, {"sparse_infill_pattern", "bridgeawarecubic"},
        {"bridge_aware_cubic_max_gap", "8"}, {"infill_direction", "45"},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
        {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"}
    });
    return config;
}

Model fixture(double height = 18.)
{
    Model model;
    auto *object = model.add_object();
    object->name = "Bridge aware cubic modifier fixture";
    object->add_volume(TriangleMesh(its_make_cube(48., 28., height)));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

ModelVolume *right_modifier(Model &model, double height = 18.)
{
    TriangleMesh mesh(its_make_cube(24., 28., height));
    mesh.translate(24., 0., 0.);
    auto *modifier = model.objects.front()->add_volume(mesh);
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    return modifier;
}

void visit_paths(const ExtrusionEntity &entity, const std::function<void(const ExtrusionPath &)> &visit)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) visit(*path);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const auto &path : multi->paths) visit(path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const auto &path : loop->paths) visit(path);
    else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const auto *child : collection->entities) visit_paths(*child, visit);
}

void slice(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();
}

struct SelectedRoofCoverage {
    size_t strands = 0;
    double length_mm = 0.;
};

SelectedRoofCoverage selected_roof_coverage(const Polyline &path, const ExPolygons &scope, double minimum_span_mm)
{
    SelectedRoofCoverage coverage;
    for (const Line &line : path.lines()) {
        if (line.a == line.b) continue;
        double selected_length = 0.;
        for (const Polyline &piece : intersection_pl(Polylines{Polyline(line.a, line.b)}, scope))
            selected_length += unscale<double>(piece.length());
        // Count substantial straight roof spans, not short joining strokes or
        // the number of collections into which a serpentine fill was grouped.
        if (selected_length >= minimum_span_mm) {
            ++coverage.strands;
            coverage.length_mm += selected_length;
        }
    }
    return coverage;
}

struct GapReport {
    std::map<double, size_t> targets_by_gap;
    std::map<double, SelectedRoofCoverage> coverage_by_gap;
    std::set<size_t> target_layers;
};

// Recertify final sliced paths independently of the planner, sharing the
// finite-bead contact and output-coordinate models. Nominal lower widths omit
// the upper section at overlapping bridge heights and can overstate air runs.
// Whole paths remain uncut across modifier seams; each region's own mask only
// selects the complete runs/cells governed by its unchanged gap limit.
GapReport check_emitted_gaps(const Print &print)
{
    namespace BAC = BridgeAwareCubic;
    namespace Contact = BridgeAwareCubicContact;
    namespace Validation = BridgeAwareCubicValidation;
    GapReport report;
    const PrintObject &object = *print.get_object(size_t(0));
    const auto &layers = object.layers();
    REQUIRE_FALSE(object.instances().empty());
    size_t phase_index = 0;
    for (const PrintInstance &instance : object.instances()) {
        BAC::SerializedXY phase;
        phase.shift = unscale(Vec2crd(instance.shift));
        if (!print.config().extruder_offset.values.empty()) phase.nozzle_offset = print.config().extruder_offset.values.front();
        phase.plate_origin = print.get_plate_origin().head<2>();
        const Point origin = phase(Point(0, 0));
        const auto emitted = [&](const ExtrusionPath &source) {
            ExtrusionPath path = source;
            path.polyline = phase.emitted_polyline(source.polyline);
            path.polyline.translate(-origin);
            return path;
        };
        for (size_t index = 1; index < layers.size(); ++index) {
            const Layer &lower = *layers[index - 1], &upper = *layers[index];
            struct Policy { size_t region; double gap; ExPolygons sparse, bridges; };
            std::vector<Policy> policies;
            ExPolygons all_sparse;
            for (size_t region_id = 0; region_id < lower.regions().size(); ++region_id) {
                const LayerRegion &region = *lower.get_region(int(region_id));
                const auto &config = region.region().config();
                if (config.sparse_infill_pattern.value != ipBridgeAwareCubic ||
                    config.sparse_infill_density.value <= 0. || config.sparse_infill_density.value >= 100.) continue;
                Policy policy{region_id, config.bridge_aware_cubic_max_gap.value, {}, {}};
                for (const Surface &surface : region.fill_surfaces.surfaces) {
                    if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
                        policy.sparse.push_back(surface.expolygon);
                    if (surface.surface_type == stInternalBridge) policy.bridges.push_back(surface.expolygon);
                }
                policy.sparse = union_ex(policy.sparse);
                policy.bridges = union_ex(policy.bridges);
                if (!policy.sparse.empty() || !policy.bridges.empty()) {
                    append(all_sparse, policy.sparse);
                    policies.push_back(std::move(policy));
                }
            }
            if (policies.empty()) continue;
            ExtrusionPaths prior_paths;
            for (const LayerRegion *region : lower.regions()) {
                const auto collect = [&](const ExtrusionPath &path) {
                    if (!path.is_force_no_extrusion() && path.mm3_per_mm > 0. && path.polyline.size() >= 2 && path.role() != erIroning)
                        prior_paths.push_back(emitted(path));
                };
                visit_paths(region->perimeters, collect);
                visit_paths(region->fills, collect);
            }
            std::vector<Contact::Bead> prior;
            for (const ExtrusionPath &path : prior_paths)
                prior.push_back({&path, GCodeWriter::serialized_z(lower.print_z, true)});
            Polylines sources;
            for (const LayerRegion *region : upper.regions()) {
                const auto collect = [&](const ExtrusionPath &path) {
                    if (!path.is_force_no_extrusion() && path.mm3_per_mm > 0. && path.polyline.size() >= 2)
                        sources.push_back(path.polyline);
                };
                visit_paths(region->perimeters, collect);
                visit_paths(region->fills, collect);
            }
            const ExPolygons all_direct = BAC::serialized_demand(union_ex(all_sparse), sources, phase);
            std::map<double, Validation::PreparedGeometryPtr> prepared;
            for (size_t emitter = 0; emitter < upper.regions().size(); ++emitter) {
                const auto inspect_path = [&](const ExtrusionPath &source, bool count_fill_target) {
                    if (source.is_force_no_extrusion() || source.mm3_per_mm <= 0. || source.polyline.size() < 2 ||
                        source.role() == erInternalInfill || source.role() == erIroning) return;
                    const ExtrusionPath path = emitted(source);
                    const ExPolygons stroke = union_ex(offset(Polylines{source.polyline}, scale_(.5 * source.width)));
                    std::vector<std::pair<ExPolygons, ExPolygons>> scopes;
                    bool selected = false;
                    for (const Policy &policy : policies) {
                        // Preserve the original direct fill-target counters;
                        // perimeter and follow-up checks add coverage only.
                        if (count_fill_target && !intersection_pl(Polylines{source.polyline}, policy.sparse).empty()) {
                            ++report.targets_by_gap[policy.gap];
                            report.target_layers.insert(index);
                            const auto coverage = selected_roof_coverage(path.polyline,
                                BAC::serialized_demand(policy.sparse, {source.polyline}, phase), policy.gap);
                            report.coverage_by_gap[policy.gap].strands += coverage.strands;
                            report.coverage_by_gap[policy.gap].length_mm += coverage.length_mm;
                        }
                        scopes.emplace_back(intersection_ex(stroke, policy.sparse), intersection_ex(stroke, policy.bridges));
                        selected |= !scopes.back().first.empty() || !scopes.back().second.empty();
                    }
                    if (!selected) return;
                    const auto contact = Contact::footprint(prior, path, GCodeWriter::serialized_z(upper.print_z, true), 0.,
                        Contact::FootprintMode::ContactsOnly);
                    CAPTURE(index, phase_index, emitter, upper.print_z, lower.print_z, path.width, path.height,
                            contact.contact_z, contact.failure_reason);
                    REQUIRE(contact.input_valid);
                    REQUIRE(contact.completed);
                    Polylines air;
                    for (const auto &segment : contact.segments) {
                        const size_t point = segment.segment_index;
                        append(air, diff_pl(Polylines{Polyline(path.polyline.points[point], path.polyline.points[point + 1])},
                                           segment.centerline_support));
                    }
                    const ExPolygons swept = union_ex(offset(air, scale_(.5 * path.width)));
                    for (size_t policy_index = 0; policy_index < policies.size(); ++policy_index) {
                        const Policy &policy = policies[policy_index];
                        const auto inspect = [&](const ExPolygons &scope, bool cells) {
                            if (scope.empty()) return;
                            const ExPolygons demand = intersection_ex(swept, BAC::serialized_demand(scope, {source.polyline}, phase));
                            if (demand.empty()) return;
                            Validation::Result result;
                            if (cells) {
                                auto cached = prepared.find(contact.contact_z);
                                if (cached == prepared.end()) {
                                    const auto physical = Contact::footprint(prior, path,
                                        GCodeWriter::serialized_z(upper.print_z, true), 0.);
                                    REQUIRE(physical.input_valid);
                                    REQUIRE(physical.completed);
                                    REQUIRE(physical.contact_z == contact.contact_z);
                                    const ExPolygons domain = BAC::support_topology_domain(physical.physical_footprint, all_direct);
                                    cached = prepared.emplace(contact.contact_z,
                                        Validation::prepare_geometry(domain, physical.physical_footprint)).first;
                                }
                                result = Validation::validate_prepared_contacts(*cached->second, {path.polyline}, demand,
                                    policy.gap, 0., {contact});
                            } else {
                                // Slots between bridge follow-up strands are
                                // not cubic cells; their full runs need anchors.
                                result = Validation::validate_contacts({}, {}, {path.polyline}, demand, policy.gap, 0., {contact});
                            }
                            CAPTURE(policy.region, policy.gap, cells, result.failure_reason,
                                    result.max_unsupported_mm, result.max_void_diameter_mm);
                            REQUIRE(result.input_valid);
                            REQUIRE(result.completed);
                            CHECK(result.passed());
                        };
                        inspect(scopes[policy_index].first, true);
                        inspect(scopes[policy_index].second, false);
                    }
                };
                const LayerRegion &region = *upper.get_region(int(emitter));
                visit_paths(region.perimeters, [&](const ExtrusionPath &path) { inspect_path(path, false); });
                visit_paths(region.fills, [&](const ExtrusionPath &path) { inspect_path(path, true); });
            }
        }
        ++phase_index;
    }
    return report;
}

// Compare deposited geometry independently of path order and direction. The
// cross-section and role are retained so combined and ordinary paths differ.
using Segment = std::tuple<size_t, size_t, int, double, double, double, coord_t, coord_t, coord_t, coord_t>;
std::vector<Segment> deposited_segments(const Print &print)
{
    std::vector<Segment> result;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (size_t region_id = 0; region_id < layer->regions().size(); ++region_id) {
            const LayerRegion *region = layer->get_region(int(region_id));
            const auto record = [&](const ExtrusionPath &path) {
                if (path.is_force_no_extrusion()) return;
                for (const Line &line : path.polyline.lines()) {
                    Point a = line.a, b = line.b;
                    if (a == b) continue;
                    if (std::make_pair(a.x(), a.y()) > std::make_pair(b.x(), b.y())) std::swap(a, b);
                    result.emplace_back(layer->id(), region_id, int(path.role()), path.width, path.height,
                                        path.mm3_per_mm, a.x(), a.y(), b.x(), b.y());
                }
            };
            visit_paths(region->perimeters, record);
            visit_paths(region->fills, record);
        }
    std::sort(result.begin(), result.end());
    return result;
}

using RegionPath = std::tuple<size_t, int, double, double, double, bool, Points>;
std::vector<RegionPath> region_paths(const PrintObject &object, size_t region_id)
{
    std::vector<RegionPath> result;
    for (const Layer *layer : object.layers()) {
        const auto record = [&](const ExtrusionPath &path) {
            result.emplace_back(layer->id(), int(path.role()), path.width, path.height,
                path.mm3_per_mm, path.is_force_no_extrusion(), path.polyline.points);
        };
        const LayerRegion &region = *layer->get_region(int(region_id));
        visit_paths(region.perimeters, record);
        visit_paths(region.fills, record);
    }
    std::sort(result.begin(), result.end());
    return result;
}

struct BridgeSurfaceSnapshot {
    size_t layer, region;
    double z, angle;
    bool followup;
    ExPolygon area;
};

std::vector<BridgeSurfaceSnapshot> bridge_surfaces(const Print &print)
{
    std::vector<BridgeSurfaceSnapshot> result;
    const auto &layers = print.get_object(size_t(0))->layers();
    for (size_t layer_index = 0; layer_index < layers.size(); ++layer_index)
        for (size_t region_id = 0; region_id < layers[layer_index]->regions().size(); ++region_id)
            for (const Surface &surface : layers[layer_index]->get_region(int(region_id))->fill_surfaces.surfaces)
                if (surface.surface_type == stInternalBridge) {
                    REQUIRE(std::isfinite(surface.bridge_angle));
                    REQUIRE(surface.bridge_angle >= 0.);
                    result.push_back({layer_index, region_id, layers[layer_index]->print_z,
                                      surface.bridge_angle, surface.bridge_followup, surface.expolygon});
                }
    return result;
}

bool same_bridge_domain(const BridgeSurfaceSnapshot &first, const BridgeSurfaceSnapshot &second)
{
    return first.layer == second.layer && first.region == second.region && first.z == second.z &&
           first.followup == second.followup && diff_ex(first.area, second.area).empty() &&
           diff_ex(second.area, first.area).empty();
}

void check_same_bridge_surfaces(const std::vector<BridgeSurfaceSnapshot> &expected,
                                const std::vector<BridgeSurfaceSnapshot> &actual, bool compare_angles = true)
{
    REQUIRE_FALSE(expected.empty());
    REQUIRE(actual.size() == expected.size());
    std::vector<bool> matched(actual.size(), false);
    for (const auto &surface : expected) {
        size_t match = actual.size();
        for (size_t i = 0; i < actual.size(); ++i)
            if (!matched[i] && same_bridge_domain(surface, actual[i])) {
                match = i;
                break;
            }
        CAPTURE(surface.layer, surface.region, surface.followup);
        REQUIRE(match < actual.size());
        matched[match] = true;
        if (compare_angles)
            CHECK(std::abs(std::sin(surface.angle - actual[match].angle)) < 1.e-9);
    }
}

// Establish a real orientation proposal across a region boundary. This checks
// complete deposited upper paths; clipping is used only after the verifier has
// identified an excessive two-ended air run, never to invent a seam anchor.
bool has_cross_region_orientation_seed(const Print &print, size_t supplier, size_t neighbor,
                                      const std::vector<BridgeSurfaceSnapshot> &surfaces, double gap,
                                      std::ostream &diagnostic)
{
    namespace Contact = BridgeAwareCubicContact;
    namespace Validation = BridgeAwareCubicValidation;
    const auto &layers = print.get_object(size_t(0))->layers();
    std::set<size_t> candidates;
    for (const auto &surface : surfaces) {
        diagnostic << "bridge Z=" << surface.z << " region=" << surface.region
                   << " angle=" << surface.angle * 180. / PI << " contour=";
        for (size_t point = 0; point < std::min(size_t(4), surface.area.contour.points.size()); ++point)
            diagnostic << ' ' << unscale<double>(surface.area.contour.points[point].x()) << ','
                       << unscale<double>(surface.area.contour.points[point].y());
        diagnostic << '\n';
        if (surface.region == neighbor && surface.layer + 1 < layers.size()) candidates.insert(surface.layer);
    }
    bool found = false;
    for (size_t lower_index : candidates) {
        ExPolygons supplier_area, neighbor_area;
        for (const auto &surface : surfaces) {
            if (surface.layer != lower_index) continue;
            if (surface.region == supplier) supplier_area.push_back(surface.area);
            if (surface.region == neighbor) neighbor_area.push_back(surface.area);
        }
        if (neighbor_area.empty()) continue;
        std::vector<Contact::Bead> prior;
        for (const LayerRegion *region : layers[lower_index]->regions()) {
            const auto record = [&](const ExtrusionPath &path) {
                if (!path.is_force_no_extrusion() && path.mm3_per_mm > 0. && path.polyline.length() > 0.)
                    prior.push_back({&path, layers[lower_index]->print_z});
            };
            visit_paths(region->perimeters, record);
            visit_paths(region->fills, record);
        }
        const Layer &upper = *layers[lower_index + 1];
        size_t checked_paths = 0, excessive_runs = 0, cross_runs = 0;
        double maximum_two_ended = 0., longest_neighbor_piece = 0.;
        const auto check_path = [&](const ExtrusionPath &path) {
                if (path.role() == erInternalInfill || path.is_force_no_extrusion() ||
                    path.mm3_per_mm <= 0. || path.polyline.length() == 0.) return;
                ++checked_paths;
                const auto contact = Contact::footprint(prior, path, upper.print_z, 0., Contact::FootprintMode::ContactsOnly);
                REQUIRE(contact.input_valid);
                REQUIRE(contact.completed);
                // The protected BAC region has a permissive policy. It must
                // not supply a separate over-gap reason to rotate itself.
                // Check every relevant path and plane even after the first
                // cross-region witness has already been found.
                const auto own = Validation::validate_contacts({}, {}, {path.polyline}, neighbor_area,
                                                                100., .001, {contact});
                REQUIRE(own.input_valid);
                REQUIRE(own.completed);
                REQUIRE(std::none_of(own.unsupported_runs.begin(), own.unsupported_runs.end(),
                    [](const auto &run) { return run.exceeds_gap; }));
                if (found || supplier_area.empty()) return;
                const auto result = Validation::validate_contacts({}, {}, {path.polyline}, supplier_area,
                                                                   gap, .001, {contact});
                REQUIRE(result.input_valid);
                REQUIRE(result.completed);
                for (const auto &run : result.unsupported_runs) {
                    if (run.kind == Validation::UnsupportedRunKind::TwoEndedBridge)
                        maximum_two_ended = std::max(maximum_two_ended, run.length_mm);
                    if (!run.exceeds_gap || run.kind != Validation::UnsupportedRunKind::TwoEndedBridge) continue;
                    ++excessive_runs;
                    Polyline interval = path.polyline;
                    interval.clip_end(scale_(std::max(0., unscale<double>(interval.length()) - run.end_mm)));
                    if (interval.size() >= 2) interval.clip_start(scale_(run.begin_mm));
                    if (interval.size() < 2 || intersection_pl(Polylines{interval}, supplier_area).empty()) continue;
                    for (const auto &surface : surfaces) {
                        if (surface.layer != lower_index || surface.region != neighbor) continue;
                        const auto pieces = intersection_pl(Polylines{interval}, ExPolygons{surface.area});
                        if (!pieces.empty()) ++cross_runs;
                        for (const Polyline &piece : pieces)
                            for (const Line &line : piece.lines()) {
                                longest_neighbor_piece = std::max(longest_neighbor_piece, unscale<double>(line.length()));
                                if (unscale<double>(line.length()) < std::max(.5, 2. * double(path.width))) continue;
                                const Vec2d direction = (line.b - line.a).cast<double>();
                                const double angle = std::atan2(direction.y(), direction.x());
                                if (std::abs(std::cos(surface.angle - angle)) > 1.e-3) found = true;
                            }
                    }
                }
            };
        for (const LayerRegion *region : upper.regions()) {
            visit_paths(region->perimeters, check_path);
            visit_paths(region->fills, check_path);
        }
        diagnostic << "upper Z=" << upper.print_z << " paths=" << checked_paths
                   << " max_two_ended=" << maximum_two_ended << " excessive=" << excessive_runs
                   << " crossing=" << cross_runs << " longest_neighbor_piece=" << longest_neighbor_piece << '\n';
    }
    return found;
}

} // namespace

TEST_CASE("Bridge aware roof coverage counts selected strands independently of path joining",
          "[BridgeAwareCubic][CoverageCounter]")
{
    const Polyline joined(Points{Point::new_scale(0., 0.), Point::new_scale(10., 0.),
        Point::new_scale(10., .5), Point::new_scale(0., .5),
        Point::new_scale(0., 1.), Point::new_scale(10., 1.)});
    const ExPolygons scope{ExPolygon(Polygon(Points{Point::new_scale(2., -1.), Point::new_scale(8., -1.),
        Point::new_scale(8., 2.), Point::new_scale(2., 2.)}))};
    const auto whole = selected_roof_coverage(joined, scope, 4.);
    SelectedRoofCoverage separate;
    for (const double y : {0., .5, 1.}) {
        const auto strand = selected_roof_coverage(Polyline(Point::new_scale(0., y), Point::new_scale(10., y)), scope, 4.);
        separate.strands += strand.strands;
        separate.length_mm += strand.length_mm;
    }
    CHECK(whole.strands == 3);
    CHECK(whole.length_mm == Approx(18.));
    CHECK(whole.strands == separate.strands);
    CHECK(whole.length_mm == Approx(separate.length_mm));
    CHECK(selected_roof_coverage(joined, scope, 7.).strands == 0);
}

TEST_CASE("Bridge aware modifier oracle distinguishes overlapping bridge contact from nominal strokes",
          "[BridgeAwareCubic][ContactModel][OverlapOracle]")
{
    namespace BAC = BridgeAwareCubic;
    namespace Contact = BridgeAwareCubicContact;
    namespace Validation = BridgeAwareCubicValidation;
    constexpr double gap = 3.;
    ExtrusionPath lower(erInternalInfill, .08, .4f, .2f);
    lower.polyline = Polyline(Point::new_scale(0., 0.), Point::new_scale(10., 0.));
    ExtrusionPath upper(erBridgeInfill, .12937261164188385, .405859977f, .4f);
    upper.polyline = Polyline(Point::new_scale(1., .3), Point::new_scale(9., .3));
    BAC::SerializedXY phase;
    phase.shift = Vec2d(103.99999, 93.99999);
    const Point origin = phase(Point(0, 0));
    for (ExtrusionPath *path : {&lower, &upper}) {
        path->polyline = phase.emitted_polyline(path->polyline);
        path->polyline.translate(-origin);
    }
    Polygons nominal_support;
    lower.polygons_covered_by_width(nominal_support, 0.f);
    const ExPolygons demand = union_ex(offset(Polylines{upper.polyline}, scale_(.5 * upper.width)));
    const auto nominal = Validation::validate({}, union_ex(nominal_support), {upper.polyline}, demand, gap, 0.);
    REQUIRE(nominal.input_valid);
    REQUIRE(nominal.completed);
    REQUIRE_FALSE(nominal.passed());
    CHECK(nominal.max_unsupported_mm == Approx(8.));

    const auto check_contact = [&](const ExtrusionPath &path, double print_z) {
        const auto contact = Contact::footprint({{&lower, .2}}, path, print_z, 0.);
        REQUIRE(contact.input_valid);
        REQUIRE(contact.completed);
        const ExPolygons own_demand = union_ex(offset(Polylines{path.polyline}, scale_(.5 * path.width)));
        const auto result = Validation::validate_contacts({}, contact.physical_footprint,
            {path.polyline}, own_demand, gap, 0., {contact});
        REQUIRE(result.input_valid);
        REQUIRE(result.completed);
        return std::make_pair(contact, result);
    };
    const auto overlapping = check_contact(upper, .4);
    CHECK(overlapping.first.contact_z == Approx(.1).margin(1.e-7));
    // The actual overlap plane has .4 mm lower and about .35227 mm
    // upper sections: their combined half-width exceeds the .3 mm offset.
    CHECK(Contact::section_width_mm(lower.width, lower.height, .2, overlapping.first.contact_z) == Approx(.4).margin(1.e-6));
    CHECK(Contact::section_width_mm(upper.width, upper.height, .4, overlapping.first.contact_z) == Approx(.35227014).margin(1.e-6));
    CHECK(overlapping.second.passed());
    CHECK(overlapping.second.unsupported_runs.empty());

    // Raising the same bridge to a tangent plane removes that broad upper
    // section. Moving it beyond both sections also removes real contact.
    const auto tangent = check_contact(upper, .6);
    CHECK(tangent.first.contact_z == Approx(.2).margin(1.e-7));
    CHECK_FALSE(tangent.second.passed());
    CHECK(tangent.second.max_unsupported_mm == Approx(8.));
    ExtrusionPath remote = upper;
    remote.polyline.translate(Point(coord_t(0), coord_t(scale_(.15))));
    const auto separated = check_contact(remote, .4);
    CHECK_FALSE(separated.second.passed());
    CHECK(separated.second.max_unsupported_mm == Approx(8.));
}

TEST_CASE("Bridge aware cubic modifier plans retain local density angle and gap ownership",
          "[BridgeAwareCubic][Integration][Modifiers]")
{
    auto model = fixture();
    auto *modifier = right_modifier(model);
    modifier->config.set("bridge_aware_cubic_max_gap", 3.);
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(12.));
    modifier->config.set("infill_direction", 17.);
    auto config = modifier_config();
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    const auto *object = print.get_object(size_t(0));
    std::set<const BridgeAwareCubic::Plan *> plans;
    std::set<double> gaps;
    for (size_t i = 0; i < object->num_printing_regions(); ++i) {
        const auto &region_config = object->printing_region(i).config();
        const auto *plan = object->bridge_aware_cubic_plan(i);
        REQUIRE(plan != nullptr);
        plans.insert(plan);
        gaps.insert(region_config.bridge_aware_cubic_max_gap.value);
        CHECK(plan->parameters().density == Approx(region_config.sparse_infill_density.value * 0.01));
        CHECK(plan->parameters().angle == Approx(region_config.infill_direction.value * PI / 180.));
        REQUIRE_FALSE(plan->demands().empty());
    }
    REQUIRE(plans.size() == 2);
    REQUIRE((gaps == std::set<double>{3., 8.}));
    const auto report = check_emitted_gaps(print);
    REQUIRE(report.targets_by_gap.count(3.) == 1);
    REQUIRE(report.targets_by_gap.count(8.) == 1);
    CHECK(report.targets_by_gap.at(3.) > 0);
    CHECK(report.targets_by_gap.at(8.) > 0);
}

TEST_CASE("Bridge aware cubic modifier edits rebuild to the same paths as a fresh slice",
          "[BridgeAwareCubic][Integration][Modifiers][Reslice]")
{
    auto model = fixture();
    auto *modifier = right_modifier(model);
    modifier->config.set("bridge_aware_cubic_max_gap", 6.);
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(10.));
    modifier->config.set("infill_direction", 45.);
    auto config = modifier_config();
    Print reused;
    slice(reused, model, config);
    const auto baseline = deposited_segments(reused);
    const auto baseline_bridges = bridge_surfaces(reused);
    REQUIRE_FALSE(baseline.empty());
    REQUIRE_FALSE(baseline_bridges.empty());

    SECTION("Gap change invalidates support planning") { modifier->config.set("bridge_aware_cubic_max_gap", 3.); }
    SECTION("Direction change invalidates support planning") { modifier->config.set("infill_direction", 17.); }
    SECTION("Density change invalidates support planning") {
        modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(15.));
    }
    reused.apply(model, config);
    reused.process();
    const auto changed = deposited_segments(reused);
    CHECK(changed != baseline);
    Print fresh;
    slice(fresh, model, config);
    CHECK(changed == deposited_segments(fresh));
    check_same_bridge_surfaces(bridge_surfaces(fresh), bridge_surfaces(reused));
    REQUIRE_FALSE(check_emitted_gaps(reused).target_layers.empty());

    modifier->config.set("bridge_aware_cubic_max_gap", 6.);
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(10.));
    modifier->config.set("infill_direction", 45.);
    reused.apply(model, config);
    reused.process();
    CHECK(deposited_segments(reused) == baseline);
    check_same_bridge_surfaces(baseline_bridges, bridge_surfaces(reused));
}

TEST_CASE("Bridge aware cubic retains explicit bridge-angle control during support preparation",
          "[BridgeAwareCubic][Integration][Modifiers][BridgeOrientation][ExplicitAngle]")
{
    auto model = fixture();
    auto *modifier = right_modifier(model);
    modifier->config.set("bridge_aware_cubic_max_gap", 100.);
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(12.));
    modifier->config.set("infill_direction", 17.);
    auto config = modifier_config();
    config.set_deserialize_strict({{"bridge_angle", "37"}, {"bridge_aware_cubic_max_gap", "100"}});
    Print permissive;
    REQUIRE_NOTHROW(slice(permissive, model, config));
    const auto baseline = bridge_surfaces(permissive);
    REQUIRE_FALSE(baseline.empty());

    modifier->config.set("bridge_aware_cubic_max_gap", 3.);
    config.set_deserialize_strict({{"bridge_aware_cubic_max_gap", "8"}});
    Print strict;
    try {
        slice(strict, model, config);
    } catch (const SlicingError &error) {
        // An explicit direction may make the requested span infeasible. It
        // must retain that choice and report the actual follow-up obligation.
        const std::string message = error.what();
        INFO(message);
        REQUIRE(message.find("A follow-up bridge layer lacks adequately anchored preceding-layer support.") != std::string::npos);
    }
    REQUIRE(strict.objects().size() == 1);
    const PrintObject &object = *strict.get_object(size_t(0));
    for (size_t region = 0; region < object.num_printing_regions(); ++region)
        REQUIRE(object.printing_region(region).config().bridge_angle.value == 37.);
    check_same_bridge_surfaces(baseline, bridge_surfaces(strict));
}

TEST_CASE("Bridge aware automatic orientation preserves relative angles throughout a bridge stack",
          "[BridgeAwareCubic][Integration][BridgeOrientation][Stack]")
{
    const int count = GENERATE(2, 3);
    Model model;
    ModelObject *object = model.add_object();
    object->name = "Bridge aware cubic stack angle preservation";
    object->add_volume(TriangleMesh(its_make_cube(26. + .7 * count, 22. + .3 * count, 13.6 + .4 * count)));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    auto config = modifier_config();
    config.set_deserialize_strict({{"sparse_infill_density", "12%"}, {"bridge_aware_cubic_max_gap", "100"},
        {"internal_bridge_layers", std::to_string(count)}, {"thick_bridges", "1"},
        {"infill_direction", std::to_string(13. + 23. * count)}});
    Print permissive;
    REQUIRE_NOTHROW(slice(permissive, model, config));
    const auto baseline = bridge_surfaces(permissive);
    REQUIRE_FALSE(baseline.empty());
    config.set_deserialize_strict({{"bridge_aware_cubic_max_gap", count == 2 ? "5" : "6"}});
    Print strict;
    REQUIRE_NOTHROW(slice(strict, model, config));
    const auto actual = bridge_surfaces(strict);
    check_same_bridge_surfaces(baseline, actual, false);

    const auto baseline_angle = [&](const BridgeSurfaceSnapshot &surface) {
        const auto match = std::find_if(baseline.begin(), baseline.end(),
            [&](const BridgeSurfaceSnapshot &candidate) { return same_bridge_domain(surface, candidate); });
        REQUIRE(match != baseline.end());
        return match->angle;
    };
    std::set<size_t> planes, followup_planes, connected_planes, orthogonal_planes, emitted_planes;
    size_t connections = 0;
    for (const auto &upper : actual) {
        planes.insert(upper.layer);
        if (upper.followup) followup_planes.insert(upper.layer);
        const LayerRegion &region = *strict.get_object(size_t(0))->get_layer(upper.layer)->get_region(int(upper.region));
        visit_paths(region.fills, [&](const ExtrusionPath &path) {
            if (is_bridge(path.role()) && !path.is_force_no_extrusion() && path.mm3_per_mm > 0. &&
                path.polyline.length() > 0. && !intersection_pl(Polylines{path.polyline}, ExPolygons{upper.area}).empty())
                emitted_planes.insert(upper.layer);
        });
        if (!upper.followup) continue;
        for (const auto &lower : actual) {
            if (lower.layer + 1 != upper.layer || lower.region != upper.region ||
                intersection_ex(lower.area, upper.area).empty()) continue;
            ++connections;
            connected_planes.insert(upper.layer);
            const double lower_before = baseline_angle(lower), upper_before = baseline_angle(upper);
            // Reconstructed anchor fringe can touch another base component.
            // Identify the original orthogonal lineage from its baseline,
            // while preserving every overlapping pair's measured offset.
            if (std::abs(std::cos(upper_before - lower_before)) < 1.e-9) {
                orthogonal_planes.insert(upper.layer);
                CHECK(std::abs(std::cos(upper.angle - lower.angle)) < 1.e-9);
            }
            // Uniform rotation is the contract; a stack with no beneficial
            // trial may legitimately retain its original absolute direction.
            CHECK(std::abs(std::sin((upper.angle - upper_before) - (lower.angle - lower_before))) < 1.e-9);
        }
    }
    REQUIRE(planes.size() == size_t(count));
    REQUIRE(followup_planes.size() == size_t(count - 1));
    CHECK(emitted_planes == planes);
    CHECK(connected_planes == followup_planes);
    CHECK(orthogonal_planes == followup_planes);
    CHECK(connections >= size_t(count - 1));
}

TEST_CASE("Bridge aware orientation stage preserves a neighboring region's material policy",
          "[BridgeAwareCubic][Integration][Modifiers][BridgeOrientation][NeighborPolicy][OrientationStage]")
{
    const bool incompatible_bac = GENERATE(false, true);
    CAPTURE(incompatible_bac);
    auto model = fixture();
    // Include the first bridge plane but finish below its continuous solid
    // successor, allowing a complete upper air run to cross the region seam.
    ModelVolume *modifier = right_modifier(model, 17.6);
    modifier->config.set("bridge_aware_cubic_max_gap", 100.);
    if (incompatible_bac)
        modifier->config.set_key_value("sparse_infill_line_width", new ConfigOptionFloat(.6));
    else
        modifier->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipCubic));
    // Preserve the lattice contrast of the rotating modifier fixture while
    // leaving its continuous upper successor at 17 degrees. Applying 17
    // degrees to both lower halves instead produces a well-supported bridge
    // direction around 169 degrees and cannot exercise cross-policy rotation.
    ModelVolume *supplier_modifier = model.objects.front()->add_volume(TriangleMesh(its_make_cube(24., 28., 17.6)));
    supplier_modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    supplier_modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(5.));
    supplier_modifier->config.set("infill_direction", 45.);
    auto config = modifier_config();
    config.set_deserialize_strict({{"bridge_angle", "0"}, {"bridge_aware_cubic_max_gap", "3"},
        {"sparse_infill_pattern", "cubic"}, {"sparse_infill_density", "12%"}, {"infill_direction", "17"},
        {"infill_combination", "0"}});
    REQUIRE_FALSE(PrintObject::infill_only_where_needed);
    // Exercise the production orientation stage on real ordinary bridge
    // surfaces and generated paths. The full BAC construction solve has its
    // own integration fixtures and is not a precondition for this scope test.
    Print generated;
    REQUIRE_NOTHROW(slice(generated, model, config));
    PrintObject &object = *generated.get_object(size_t(0));
    // Touching modifier boxes allocate a combined configuration even though
    // their polygon intersection has no material. Select physical regions:
    // the dormant combination inherits the neighbor's G100 and left angle.
    std::set<size_t> material_regions;
    for (const Layer *layer : object.layers())
        for (size_t index = 0; index < object.num_printing_regions(); ++index)
            if (!union_ex(to_expolygons(layer->get_region(int(index))->slices.surfaces)).empty())
                material_regions.insert(index);
    REQUIRE(material_regions.size() == 3);
    size_t neighbor = size_t(-1), supplier = size_t(-1), parent = size_t(-1);
    for (size_t index = 0; index < object.num_printing_regions(); ++index) {
        const auto &region = object.printing_region(index).config();
        REQUIRE(region.bridge_angle.value == 0.);
        REQUIRE(region.sparse_infill_density.value > 0.);
        REQUIRE(region.sparse_infill_pattern.value == ipCubic);
        REQUIRE(object.bridge_aware_cubic_plan(index) == nullptr);
        if (material_regions.count(index) == 0) {
            for (const Layer *layer : object.layers())
                REQUIRE(layer->get_region(int(index))->slices.empty());
            REQUIRE(region_paths(object, index).empty());
            continue;
        }
        if (region.bridge_aware_cubic_max_gap.value == 100.) neighbor = index;
        else if (region.infill_direction.value == 45.) supplier = index;
        else parent = index;
    }
    REQUIRE(neighbor != size_t(-1));
    REQUIRE(supplier != size_t(-1));
    REQUIRE(parent != size_t(-1));
    REQUIRE(neighbor != supplier);
    REQUIRE(parent != neighbor);
    REQUIRE(parent != supplier);
    const auto &neighbor_config = object.printing_region(neighbor).config();
    REQUIRE(neighbor_config.sparse_infill_density.value == 12.);
    REQUIRE(neighbor_config.infill_direction.value == 17.);
    REQUIRE(neighbor_config.sparse_infill_line_width.value == Approx(incompatible_bac ? .6 : .4));
    REQUIRE(object.printing_region(supplier).config().sparse_infill_density.value == 5.);
    REQUIRE(object.printing_region(supplier).config().bridge_aware_cubic_max_gap.value == 3.);
    REQUIRE(object.printing_region(parent).config().sparse_infill_density.value == 12.);
    REQUIRE(object.printing_region(parent).config().infill_direction.value == 17.);
    REQUIRE(object.printing_region(parent).config().bridge_aware_cubic_max_gap.value == 3.);
    const ExPolygons left_half{ExPolygon(Polygon({Point::new_scale(-24., -14.), Point::new_scale(0., -14.),
        Point::new_scale(0., 14.), Point::new_scale(-24., 14.)}))};
    const ExPolygons right_half{ExPolygon(Polygon({Point::new_scale(0., -14.), Point::new_scale(24., -14.),
        Point::new_scale(24., 14.), Point::new_scale(0., 14.)}))};
    size_t support_planes = 0, parent_planes = 0;
    for (const Layer *layer : object.layers()) {
        if (layer->print_z < 17.6 - EPSILON) continue;
        Polygons region_polygons;
        for (const LayerRegion *region : layer->regions())
            polygons_append(region_polygons, to_polygons(region->slices.surfaces));
        const ExPolygons material = union_ex(region_polygons);
        const ExPolygons expected_material = union_ex(left_half, right_half);
        REQUIRE(diff_ex(material, expected_material).empty());
        REQUIRE(diff_ex(expected_material, material).empty());
        // Layer::make_slices expands a multi-region union by the exact
        // ClipperSafetyOffset. That overlap clearance is not region material.
        const ExPolygons expected_islands = union_safety_offset_ex(region_polygons);
        REQUIRE(diff_ex(layer->lslices, expected_islands).empty());
        REQUIRE(diff_ex(expected_islands, layer->lslices).empty());
        const auto region_material = [&](size_t index) {
            return union_ex(to_expolygons(layer->get_region(int(index))->slices.surfaces));
        };
        if (std::abs(layer->print_z - 17.6) < EPSILON) {
            ++support_planes;
            REQUIRE(region_material(parent).empty());
            for (const auto &expected : {std::make_pair(supplier, intersection_ex(material, left_half)),
                                         std::make_pair(neighbor, intersection_ex(material, right_half))}) {
                REQUIRE_FALSE(expected.second.empty());
                const ExPolygons actual = region_material(expected.first);
                REQUIRE(diff_ex(actual, expected.second).empty());
                REQUIRE(diff_ex(expected.second, actual).empty());
            }
        } else {
            ++parent_planes;
            REQUIRE(region_material(supplier).empty());
            REQUIRE(region_material(neighbor).empty());
            REQUIRE_FALSE(region_material(parent).empty());
            REQUIRE(diff_ex(region_material(parent), material).empty());
            REQUIRE(diff_ex(material, region_material(parent)).empty());
        }
    }
    REQUIRE(support_planes == 1);
    REQUIRE(parent_planes == 2);
    for (size_t index : material_regions) {
        PrintRegion &region = const_cast<PrintRegion &>(object.printing_region(index));
        PrintRegionConfig modified = region.config();
        modified.sparse_infill_pattern.value = index == neighbor && !incompatible_bac ? ipCubic : ipBridgeAwareCubic;
        region.set_config(std::move(modified));
    }
    const auto regenerate = [&] {
        for (Layer *layer : object.layers()) layer->make_fills(nullptr, nullptr, nullptr);
    };
    // BAC keeps region fill groups distinct. Establish the baseline after
    // that regrouping; null plans use the actual ordinary Cubic generator.
    REQUIRE_NOTHROW(regenerate());
    const auto all_baseline = bridge_surfaces(generated);
    const auto protected_paths = region_paths(object, neighbor);
    REQUIRE_FALSE(protected_paths.empty());
    std::vector<BridgeSurfaceSnapshot> protected_baseline;
    for (const auto &surface : all_baseline)
        if (surface.region == neighbor) protected_baseline.push_back(surface);
    REQUIRE_FALSE(protected_baseline.empty());
    size_t neighbor_bridge_paths = 0;
    for (const auto &surface : protected_baseline)
        visit_paths(object.get_layer(surface.layer)->get_region(int(neighbor))->fills, [&](const ExtrusionPath &path) {
            if (is_bridge(path.role()) && !path.is_force_no_extrusion() && path.mm3_per_mm > 0. &&
                !intersection_pl(Polylines{path.polyline}, ExPolygons{surface.area}).empty()) ++neighbor_bridge_paths;
        });
    REQUIRE(neighbor_bridge_paths > 0);
    std::ostringstream seed_diagnostic;
    const bool has_seed = has_cross_region_orientation_seed(generated, supplier, neighbor, all_baseline, 3., seed_diagnostic);
    INFO(seed_diagnostic.str());
    REQUIRE(has_seed);

    REQUIRE_NOTHROW(BridgeAwareCubic::orient_automatic_bridges(object, regenerate));
    std::vector<BridgeSurfaceSnapshot> protected_actual;
    for (const auto &surface : bridge_surfaces(generated))
        if (surface.region == neighbor) protected_actual.push_back(surface);
    check_same_bridge_surfaces(protected_baseline, protected_actual);
    const auto actual_paths = region_paths(object, neighbor);
    std::ostringstream path_delta;
    if (actual_paths != protected_paths) {
        std::vector<RegionPath> removed, added;
        std::set_difference(protected_paths.begin(), protected_paths.end(), actual_paths.begin(), actual_paths.end(),
            std::back_inserter(removed));
        std::set_difference(actual_paths.begin(), actual_paths.end(), protected_paths.begin(), protected_paths.end(),
            std::back_inserter(added));
        path_delta.precision(17);
        path_delta << "protected path delta: before=" << protected_paths.size() << " after=" << actual_paths.size()
            << " removed=" << removed.size() << " added=" << added.size() << '\n';
        const auto describe = [&](const char *label, const RegionPath &path) {
            const Points &points = std::get<6>(path);
            path_delta << label << " layer=" << std::get<0>(path) << " role=" << std::get<1>(path)
                << " width=" << std::get<2>(path) << " height=" << std::get<3>(path)
                << " volume=" << std::get<4>(path) << " no_extrusion=" << std::get<5>(path)
                << " points=" << points.size();
            for (size_t i = 0; i < points.size(); ++i)
                if (i < 8 || i + 2 >= points.size()) path_delta << " [" << i << "]=" << points[i].x() << ',' << points[i].y();
            path_delta << '\n';
        };
        for (size_t i = 0; i < std::min(size_t(4), std::max(removed.size(), added.size())); ++i) {
            if (i < removed.size()) describe("before", removed[i]);
            if (i < added.size()) describe("after", added[i]);
            if (i < removed.size() && i < added.size()) {
                const Points &before = std::get<6>(removed[i]), &after = std::get<6>(added[i]);
                for (size_t point = 0; point < std::min(before.size(), after.size()); ++point)
                    if (before[point] != after[point]) {
                        path_delta << "first differing point=" << point << " before=" << before[point].x() << ',' << before[point].y()
                            << " after=" << after[point].x() << ',' << after[point].y() << '\n';
                        break;
                    }
            }
        }
    }
    INFO(path_delta.str());
    CHECK(actual_paths == protected_paths);
    for (size_t index = 0; index < object.num_printing_regions(); ++index)
        CHECK(object.bridge_aware_cubic_plan(index) == nullptr);
}

TEST_CASE("Bridge aware cubic does not fill an empty neighboring region to certify a roof",
          "[BridgeAwareCubic][Integration][Modifiers][Limits]")
{
    auto model = fixture();
    // Stop this modifier at the plane below the roof so the upper extrusion
    // remains continuous across the supporting-region seam.
    right_modifier(model, 17.4)->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(0.));
    auto config = modifier_config();
    Print print;
    std::string failure;
    try { slice(print, model, config); }
    catch (const SlicingError &error) {
        failure = error.what();
        // An infeasible boundary must be diagnosed, not mistaken for a missing
        // enum, unrelated exception, or a successful certified slice.
        CHECK(failure.find("Bridge aware cubic") != std::string::npos);
        CHECK(failure.find("layer") != std::string::npos);
    }
    CAPTURE(failure);
    REQUIRE(print.objects().size() == 1);
    REQUIRE(print.get_object(size_t(0))->layer_count() > 50);
    size_t empty_regions = 0, participating_paths = 0;
    bool zero_at_support_plane = false, parent_roof_above = false;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions()) {
            const bool empty = region->region().config().sparse_infill_density.value == 0.;
            if (empty && !region->slices.empty()) ++empty_regions;
            if (empty && std::abs(layer->print_z - 17.4) < EPSILON && !region->slices.empty())
                zero_at_support_plane = true;
            if (!empty && std::abs(layer->print_z - 17.6) < EPSILON)
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    parent_roof_above |= surface.is_solid();
            visit_paths(region->fills, [&](const ExtrusionPath &path) {
                if (path.role() != erInternalInfill || path.is_force_no_extrusion()) return;
                CHECK_FALSE(empty);
                ++participating_paths;
            });
        }
    REQUIRE(empty_regions > 50);
    REQUIRE(zero_at_support_plane);
    REQUIRE(parent_roof_above);
    if (failure.empty()) {
        // A real active-side partition can close the cells at the seam. In
        // that valid outcome, verify complete unsupported runs without ever
        // treating the modifier edge itself as deposited material.
        REQUIRE(participating_paths > 0);
        REQUIRE_FALSE(check_emitted_gaps(print).target_layers.empty());
    }
}

TEST_CASE("Bridge aware cubic respects a through cavity and multiple roof elevations",
          "[BridgeAwareCubic][Integration][Geometry]")
{
    Model model;
    auto *object = model.add_object();
    object->name = "Bridge aware cubic stepped cavity fixture";
    TriangleMesh stepped(its_make_cube(48., 28., 12.));
    TriangleMesh tower(its_make_cube(18., 16., 8.));
    tower.translate(2., 6., 12.);
    stepped.merge(tower);
    object->add_volume(stepped);
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    TriangleMesh bore(its_make_cube(6., 6., 22.));
    bore.translate(7., 11., -1.);
    auto *negative = object->add_volume(bore);
    negative->set_type(ModelVolumeType::NEGATIVE_VOLUME);
    auto config = modifier_config();
    config.set_deserialize_strict({{"bridge_aware_cubic_max_gap", "5"}, {"sparse_infill_density", "15%"}});
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    size_t hole_layers = 0;
    bool tower_material_seen = false, tower_infill_seen = false;
    for (const Layer *layer : print.get_object(size_t(0))->layers()) {
        // Classified top surfaces can surround continuing internal material.
        // Derive physical voids from the complete layer, including any nested
        // printable islands, rather than treating those classification holes
        // as forbidden space.
        const ExPolygons material = union_ex(layer->lslices);
        Polygons outer_contours;
        for (const ExPolygon &section : material) outer_contours.push_back(section.contour);
        const ExPolygons holes = diff_ex(union_ex(outer_contours), material);
        if (holes.empty()) continue;
        ++hole_layers;
        const auto cavity_core = offset_ex(holes, -scale_(0.25));
        REQUIRE(holes.size() == 1);
        REQUIRE(cavity_core.size() == 1);
        const BoundingBox bore_bounds = get_extents(holes), core_bounds = get_extents(cavity_core);
        constexpr double coordinate_margin = .001;
        CHECK(unscale<double>(bore_bounds.min.x()) == Approx(-17.).margin(coordinate_margin));
        CHECK(unscale<double>(bore_bounds.min.y()) == Approx(-3.).margin(coordinate_margin));
        CHECK(unscale<double>(bore_bounds.size().x()) == Approx(6.).margin(coordinate_margin));
        CHECK(unscale<double>(bore_bounds.size().y()) == Approx(6.).margin(coordinate_margin));
        CHECK(unscale<double>(core_bounds.min.x()) == Approx(-16.75).margin(coordinate_margin));
        CHECK(unscale<double>(core_bounds.min.y()) == Approx(-2.75).margin(coordinate_margin));
        CHECK(unscale<double>(core_bounds.size().x()) == Approx(5.5).margin(coordinate_margin));
        CHECK(unscale<double>(core_bounds.size().y()) == Approx(5.5).margin(coordinate_margin));
        ExPolygons continuing_tower;
        if (std::abs(layer->print_z - 12.) < EPSILON) {
            Polygons classification_holes;
            for (const LayerRegion *region : layer->regions())
                for (const Surface &surface : region->slices.surfaces)
                    for (Polygon hole : surface.expolygon.holes) {
                        hole.reverse();
                        classification_holes.push_back(std::move(hole));
                    }
            continuing_tower = intersection_ex(union_ex(classification_holes), material);
            REQUIRE(std::any_of(continuing_tower.begin(), continuing_tower.end(), [](const ExPolygon &area) {
                return area.contains(Point::new_scale(-20., 0.), true);
            }));
            CHECK(intersection_ex(continuing_tower, holes).empty());
            tower_material_seen = true;
        }
        for (size_t region_index = 0; region_index < layer->regions().size(); ++region_index) {
            const LayerRegion *region = layer->get_region(int(region_index));
            Polygons deposited, sparse_deposited;
            visit_paths(region->fills, [&](const ExtrusionPath &path) {
                if (!path.is_force_no_extrusion()) {
                    path.polygons_covered_by_width(deposited, 0.f);
                    if (path.role() == erInternalInfill) path.polygons_covered_by_width(sparse_deposited, 0.f);
                }
            });
            if (!continuing_tower.empty() && !intersection_ex(sparse_deposited, continuing_tower).empty())
                tower_infill_seen = true;
            std::ostringstream diagnostic;
            if (!intersection_ex(deposited, cavity_core).empty()) {
                diagnostic << "cavity intrusion layer=" << layer->id() << " Z=" << layer->print_z
                           << " region=" << region_index << '\n';
                size_t path_index = 0, reported = 0;
                visit_paths(region->fills, [&](const ExtrusionPath &path) {
                    const size_t index = path_index++;
                    if (path.is_force_no_extrusion() || reported >= 8) return;
                    Polygons bead;
                    path.polygons_covered_by_width(bead, 0.f);
                    const ExPolygons intrusion = intersection_ex(bead, cavity_core);
                    if (intrusion.empty()) return;
                    ++reported;
                    diagnostic << "path=" << index << " role=" << int(path.role()) << " width=" << path.width
                               << " height=" << path.height << " volume=" << path.mm3_per_mm
                               << " vertices=" << path.polyline.size() << " points=";
                    for (size_t point = 0; point < std::min(size_t(256), path.polyline.size()); ++point)
                        diagnostic << ' ' << unscale<double>(path.polyline.points[point].x()) << ','
                                   << unscale<double>(path.polyline.points[point].y());
                    diagnostic << '\n';
                    for (size_t polygon = 0; polygon < std::min(size_t(4), intrusion.size()); ++polygon) {
                        diagnostic << "intrusion=";
                        for (size_t point = 0; point < std::min(size_t(32), intrusion[polygon].contour.points.size()); ++point)
                            diagnostic << ' ' << unscale<double>(intrusion[polygon].contour.points[point].x()) << ','
                                       << unscale<double>(intrusion[polygon].contour.points[point].y());
                        diagnostic << '\n';
                    }
                    for (size_t owner = 0; owner < layer->regions().size(); ++owner)
                        if (const auto *plan = print.get_object(size_t(0))->bridge_aware_cubic_plan(owner))
                            for (const auto &attachment : plan->attachments(layer->print_z))
                                if (attachment.replacement.points == path.polyline.points) {
                                    diagnostic << "saved attachment owner=" << owner << " emitter=" << attachment.emitting_region
                                               << " solid_cut=" << attachment.solid_cut << " original=";
                                    for (size_t point = 0; point < std::min(size_t(256), attachment.original.size()); ++point)
                                        diagnostic << ' ' << unscale<double>(attachment.original.points[point].x()) << ','
                                                   << unscale<double>(attachment.original.points[point].y());
                                    diagnostic << '\n';
                                }
                });
            }
            INFO(diagnostic.str());
            CHECK(intersection_ex(deposited, cavity_core).empty());
        }
    }
    REQUIRE(hole_layers > 50);
    REQUIRE(tower_material_seen);
    REQUIRE(tower_infill_seen);
    const auto report = check_emitted_gaps(print);
    REQUIRE(report.target_layers.size() >= 2);
    CHECK(*report.target_layers.rbegin() - *report.target_layers.begin() > 25);
}

TEST_CASE("Bridge aware cubic combines coarse bulk while retaining roof support deposition",
          "[BridgeAwareCubic][Integration][Combination]")
{
    auto model = fixture(60.);
    auto config = modifier_config();
    config.set_deserialize_strict({{"bridge_aware_cubic_max_gap", "4"}, {"sparse_infill_density", "15%"},
                                   {"infill_combination", "1"}});
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    size_t combined_bulk_paths = 0, refined_paths = 0;
    const auto *object = print.get_object(size_t(0));
    for (const Layer *layer : object->layers())
        for (size_t i = 0; i < layer->regions().size(); ++i) {
            const LayerRegion *region = layer->regions()[i];
            const auto *plan = object->bridge_aware_cubic_plan(i);
            REQUIRE(plan != nullptr);
            const auto refinement = plan->refinement_area(layer->print_z);
            visit_paths(region->fills, [&](const ExtrusionPath &path) {
                if (path.role() != erInternalInfill || path.is_force_no_extrusion()) return;
                if (path.height > layer->height + EPSILON && layer->print_z < 25.) ++combined_bulk_paths;
                if (!intersection_pl(Polylines{path.polyline}, refinement).empty()) {
                    ++refined_paths;
                    CHECK(path.height <= layer->height + EPSILON);
                }
            });
        }
    REQUIRE(combined_bulk_paths > 0);
    REQUIRE(refined_paths > 0);
    const auto report = check_emitted_gaps(print);
    REQUIRE(report.targets_by_gap.count(4.) == 1);
    REQUIRE(report.coverage_by_gap.count(4.) == 1);
    const auto &coverage = report.coverage_by_gap.at(4.);
    CAPTURE(report.targets_by_gap.at(4.), coverage.strands, coverage.length_mm);
    // A joined roof can contain many strands in just one ExtrusionPath.
    // Require more than twenty selected spans of at least the own-region gap.
    CHECK(coverage.strands > 20);
    CHECK(coverage.length_mm > 20. * 4.);
}
