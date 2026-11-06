#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSerialization.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSupport.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <set>

using namespace Slic3r;
namespace BACValidation = Slic3r::BridgeAwareCubicValidation;

namespace {

ExPolygon rectangle(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon{Points{Point::new_scale(x0, y0), Point::new_scale(x1, y0),
                                   Point::new_scale(x1, y1), Point::new_scale(x0, y1)}});
}

void visit_paths(const ExtrusionEntity &entity, const std::function<void(const ExtrusionPath &)> &visit)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) {
        if (!path->is_force_no_extrusion() && path->mm3_per_mm > 0. && path->polyline.size() >= 2)
            visit(*path);
    } else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
        for (const auto &path : multi->paths) visit_paths(path, visit);
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
        for (const auto &path : loop->paths) visit_paths(path, visit);
    } else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        for (const auto *child : collection->entities) visit_paths(*child, visit);
    }
}

void visit_layer(const Layer &layer, const std::function<void(const ExtrusionPath &)> &visit)
{
    for (const LayerRegion *region : layer.regions()) {
        visit_paths(region->perimeters, visit);
        visit_paths(region->fills, visit);
    }
}

DynamicPrintConfig acceptance_config(double gap = 5.)
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "12%"}, {"sparse_infill_pattern", "bridgeawarecubic"},
        {"bridge_aware_cubic_max_gap", std::to_string(gap)}, {"infill_direction", "17"},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
        {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"}
    });
    return config;
}

Model cube(double width, double depth, double height)
{
    Model model;
    auto *object = model.add_object();
    object->name = "Bridge aware cubic independent acceptance fixture";
    object->add_volume(TriangleMesh(its_make_cube(width, depth, height)));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

void slice(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();
}

ExPolygons deposited_support(const Layer &layer)
{
    Polygons beads;
    visit_layer(layer, [&](const ExtrusionPath &path) { path.polygons_covered_by_width(beads, 0.f); });
    // Require positive overlap, independently of the planner's own tolerance.
    return offset_ex(union_ex(beads), -scale_(.001));
}

void dump_oracle_layer(size_t index, const Layer &lower, const Layer &upper, const char *reason,
                       const ExPolygons &support, const Polylines &required = {}, const ExPolygons &demand = {},
                       const BACValidation::Result *result = nullptr, double gap = 5.)
{
    const char *diagnostic = std::getenv("BAC_DEBUG_FAILURE");
    if (diagnostic == nullptr) return;
    std::ofstream out(std::string(diagnostic) + ".oracle", std::ios::app);
    out << std::setprecision(14) << "oracle " << reason << " index " << index
        << " upper_z " << upper.print_z << " lower_z " << lower.print_z << '\n';
    const auto points = [&](const Points &vertices) {
        for (const Point &point : vertices) out << ' ' << unscale<double>(point.x()) << ',' << unscale<double>(point.y());
        out << '\n';
    };
    const auto polygon = [&](const char *label, const ExPolygon &shape) {
        out << label << " contour";
        points(shape.contour.points);
        for (const Polygon &hole : shape.holes) {
            out << label << " hole";
            points(hole.points);
        }
    };
    for (const ExPolygon &shape : support) polygon("oracle_support", shape);
    for (const ExPolygon &shape : demand) polygon("oracle_demand", shape);
    for (size_t path = 0; path < required.size(); ++path) {
        out << "required " << path << " points";
        points(required[path].points);
    }
    if (result != nullptr)
        for (const auto &run : result->unsupported_runs)
            out << "run " << run.path_index << ' ' << run.begin_mm << ' ' << run.end_mm << ' '
                << run.length_mm << " kind " << int(run.kind) << " exceeds " << run.exceeds_gap << '\n';
    const auto layer = [&](const char *label, const Layer &source) {
        for (size_t region_id = 0; region_id < source.regions().size(); ++region_id) {
            const LayerRegion &region = *source.get_region(int(region_id));
            const auto &config = region.region().config();
            out << label << " region " << region_id << " pattern " << int(config.sparse_infill_pattern.value)
                << " density " << config.sparse_infill_density.value << " gap " << config.bridge_aware_cubic_max_gap.value << '\n';
            const auto path = [&](const ExtrusionPath &path) {
                out << label << " region " << region_id << " width " << path.width << " height " << path.height
                    << " role " << int(path.role()) << " points";
                points(path.polyline.points);
            };
            visit_paths(region.perimeters, path);
            visit_paths(region.fills, path);
            for (const Surface &surface : region.fill_surfaces.surfaces) {
                out << label << " surface region " << region_id << " type " << int(surface.surface_type)
                    << " followup " << surface.bridge_followup << " angle " << surface.bridge_angle << '\n';
                polygon(label, surface.expolygon);
            }
        }
    };
    layer("lower", lower);
    layer("upper", upper);
    // Diagnostic companion only: the original oracle assertions below remain
    // unchanged. This uses the shared physical contact model, not independent
    // nominal-width measurements, and reads no planner state or cell layout.
    ExPolygons eligible;
    const bool followup = std::string(reason) == "followup_failure";
    for (const LayerRegion *region : lower.regions())
        for (const Surface &surface : region->fill_surfaces.surfaces)
            if (followup ? surface.surface_type == stInternalBridge :
                (surface.surface_type == stInternal || surface.surface_type == stInternalVoid))
                eligible.push_back(surface.expolygon);
    eligible = union_ex(eligible);
    const PrintObject &object = *upper.object();
    const Print &print = *object.print();
    size_t phase_index = 0;
    for (const PrintInstance &instance : object.instances()) {
        BridgeAwareCubic::SerializedXY phase;
        phase.shift = unscale(Vec2crd(instance.shift));
        if (!print.config().extruder_offset.values.empty())
            phase.nozzle_offset = print.config().extruder_offset.values.front();
        phase.plate_origin = print.get_plate_origin().head<2>();
        const Point origin = phase(Point(0, 0));
        const auto emitted = [&](const ExtrusionPath &source) {
            ExtrusionPath copy = source;
            copy.polyline = phase.emitted_polyline(source.polyline);
            copy.polyline.translate(-origin);
            return copy;
        };
        ExtrusionPaths lower_paths;
        visit_layer(lower, [&](const ExtrusionPath &path) { lower_paths.push_back(emitted(path)); });
        std::vector<BridgeAwareCubicContact::Bead> lower_beads;
        for (const ExtrusionPath &path : lower_paths)
            lower_beads.push_back({&path, GCodeWriter::serialized_z(lower.print_z, true)});
        for (size_t path_index = 0; path_index < required.size(); ++path_index) {
            const ExtrusionPath *source = nullptr;
            visit_layer(upper, [&](const ExtrusionPath &path) {
                if (source == nullptr && path.polyline.points == required[path_index].points) source = &path;
            });
            if (source == nullptr) {
                out << "shared_contact missing_source " << path_index << '\n';
                continue;
            }
            const ExtrusionPath actual = emitted(*source);
            const auto contact = BridgeAwareCubicContact::footprint(lower_beads, actual,
                GCodeWriter::serialized_z(upper.print_z, true), 0.);
            Polylines air;
            for (const auto &segment : contact.segments) {
                const size_t point = segment.segment_index;
                append(air, diff_pl(Polylines{Polyline(Points{actual.polyline.points[point],
                    actual.polyline.points[point + 1]})}, segment.centerline_support));
            }
            if (!contact.input_valid || !contact.completed || contact.segments.empty()) air = {actual.polyline};
            const ExPolygons source_stroke = union_ex(offset(Polylines{source->polyline}, scale_(.5 * source->width)));
            const ExPolygons own_scope = intersection_ex(source_stroke, eligible);
            const ExPolygons preserved = BridgeAwareCubic::serialized_demand(own_scope, {source->polyline}, phase);
            const ExPolygons selected = intersection_ex(preserved, union_ex(offset(air, scale_(.5 * actual.width))));
            const ExPolygons actual_stroke = union_ex(offset(Polylines{actual.polyline}, scale_(.5 * actual.width)));
            const auto full_path = BACValidation::validate_contacts({}, contact.physical_footprint,
                {actual.polyline}, actual_stroke, gap, 0., {contact});
            const ExPolygons domain = followup || selected.empty() ? ExPolygons{} :
                BridgeAwareCubic::support_topology_domain(contact.physical_footprint, preserved);
            const auto scoped = BACValidation::validate_contacts(domain, contact.physical_footprint,
                {actual.polyline}, selected, gap, 0., {contact});
            out << "shared_contact phase " << phase_index << " path " << path_index
                << " width " << actual.width << " height " << actual.height << " contact_z " << contact.contact_z
                << " input_valid " << contact.input_valid << " completed " << contact.completed
                << " selected_polygons " << selected.size() << " full_path_passed " << full_path.passed()
                << " full_path_max " << full_path.max_unsupported_mm << " scoped_passed " << scoped.passed()
                << " scoped_max " << scoped.max_unsupported_mm << " scoped_cell_max " << scoped.max_void_diameter_mm
                << " reason " << scoped.failure_reason << '\n';
            for (const auto &run : full_path.unsupported_runs)
                out << "shared_contact_run " << path_index << ' ' << run.begin_mm << ' ' << run.end_mm
                    << ' ' << run.length_mm << " kind " << int(run.kind) << " exceeds " << run.exceeds_gap << '\n';
            for (const ExPolygon &shape : selected) polygon("shared_contact_selected", shape);
        }
        ++phase_index;
    }
    out << "end_oracle\n";
}

struct Measurements {
    size_t selected_paths = 0;
    size_t wall_paths = 0;
    size_t bridge_followup_paths = 0;
    size_t height_transition_paths = 0;
    size_t void_cells = 0;
    size_t anchored_runs = 0;
    double max_gap = 0.;
    double max_void = 0.;
    std::set<size_t> target_layers;
    std::set<double> wall_target_z;
};

// This recertification is independent of the planner, sharing its physical
// bead-contact and output-coordinate models. It starts from deposited paths
// and sliced material, never Plan demands, depths, leaves or expected cells.
// Nominal-width geometry remains covered by the separate synthetic controls
// and exported-path measurements; it cannot substitute for finite bead contact
// at variable-height overlap planes. Each full path retains its own scope.
Measurements measure_emitted_geometry(const Print &print, double gap)
{
    Measurements measured;
    const PrintObject &object = *print.get_object(size_t(0));
    const auto &layers = object.layers();
    for (const PrintInstance &instance : object.instances()) {
        BridgeAwareCubic::SerializedXY phase;
        phase.shift = unscale(Vec2crd(instance.shift));
        if (!print.config().extruder_offset.values.empty()) phase.nozzle_offset = print.config().extruder_offset.values.front();
        phase.plate_origin = print.get_plate_origin().head<2>();
        const Point origin = phase(Point(0, 0));
        const auto emitted = [&](const ExtrusionPath &source) {
            ExtrusionPath copy = source;
            copy.polyline = phase.emitted_polyline(source.polyline);
            copy.polyline.translate(-origin);
            return copy;
        };
        for (size_t index = 1; index < layers.size(); ++index) {
            const Layer &lower = *layers[index - 1];
            const Layer &upper = *layers[index];
            ExPolygons sparse, bridge_planes;
            for (const LayerRegion *region : lower.regions())
                for (const Surface &surface : region->fill_surfaces.surfaces) {
                    if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
                        sparse.push_back(surface.expolygon);
                    if (surface.surface_type == stInternalBridge)
                        bridge_planes.push_back(surface.expolygon);
                }
            sparse = union_ex(sparse);
            bridge_planes = union_ex(bridge_planes);
            if (sparse.empty() && bridge_planes.empty()) continue;
            ExtrusionPaths prior_paths;
            visit_layer(lower, [&](const ExtrusionPath &path) { prior_paths.push_back(emitted(path)); });
            std::vector<BridgeAwareCubicContact::Bead> prior;
            for (const ExtrusionPath &path : prior_paths)
                prior.push_back({&path, GCodeWriter::serialized_z(lower.print_z, true)});
            Polylines source_paths;
            visit_layer(upper, [&](const ExtrusionPath &path) { source_paths.push_back(path.polyline); });
            const ExPolygons all_direct = BridgeAwareCubic::serialized_demand(sparse, source_paths, phase);
            std::map<double, BACValidation::PreparedGeometryPtr> prepared;
            visit_layer(upper, [&](const ExtrusionPath &source) {
                if (source.role() == erInternalInfill || source.role() == erIroning) return;
                const ExPolygons source_stroke = union_ex(offset(Polylines{source.polyline}, scale_(.5 * source.width)));
                const ExPolygons direct_scope = intersection_ex(source_stroke, sparse);
                const ExPolygons followup_scope = intersection_ex(source_stroke, bridge_planes);
                if (direct_scope.empty() && followup_scope.empty()) return;
                const ExtrusionPath path = emitted(source);
                const auto contact = BridgeAwareCubicContact::footprint(prior, path,
                    GCodeWriter::serialized_z(upper.print_z, true), 0., BridgeAwareCubicContact::FootprintMode::ContactsOnly);
                CAPTURE(index, upper.print_z, lower.print_z, path.height, contact.failure_reason);
                REQUIRE(contact.input_valid);
                REQUIRE(contact.completed);
                Polylines air;
                for (const auto &segment : contact.segments) {
                    const size_t i = segment.segment_index;
                    append(air, diff_pl(Polylines{Polyline(Points{path.polyline.points[i], path.polyline.points[i + 1]})},
                                       segment.centerline_support));
                }
                const ExPolygons swept = union_ex(offset(air, scale_(.5 * path.width)));
                const ExPolygons direct = intersection_ex(swept,
                    BridgeAwareCubic::serialized_demand(direct_scope, {source.polyline}, phase));
                const ExPolygons followup = intersection_ex(swept,
                    BridgeAwareCubic::serialized_demand(followup_scope, {source.polyline}, phase));
                if (direct.empty() && followup.empty()) return;
                ++measured.selected_paths;
                if (path.role() == erPerimeter || path.role() == erExternalPerimeter || path.role() == erOverhangPerimeter) {
                    ++measured.wall_paths;
                    measured.wall_target_z.insert(upper.print_z);
                }
                if (!followup.empty()) ++measured.bridge_followup_paths;
                if (std::abs(upper.height - lower.height) > .001) ++measured.height_transition_paths;
                measured.target_layers.insert(index);
                const auto inspect = [&](const ExPolygons &demand, bool cells) {
                    if (demand.empty()) return;
                    BACValidation::Result result;
                    if (cells) {
                        auto cached = prepared.find(contact.contact_z);
                        if (cached == prepared.end()) {
                            const auto physical = BridgeAwareCubicContact::footprint(prior, path,
                                GCodeWriter::serialized_z(upper.print_z, true), 0.);
                            REQUIRE(physical.input_valid);
                            REQUIRE(physical.completed);
                            REQUIRE(physical.contact_z == contact.contact_z);
                            const ExPolygons domain = BridgeAwareCubic::support_topology_domain(physical.physical_footprint, all_direct);
                            cached = prepared.emplace(contact.contact_z,
                                BACValidation::prepare_geometry(domain, physical.physical_footprint)).first;
                        }
                        result = BACValidation::validate_prepared_contacts(*cached->second, {path.polyline}, demand, gap, 0., {contact});
                    } else {
                        // Follow-up strands use actual predecessor contact; the
                        // long slots between bridge strands are not cubic cells.
                        result = BACValidation::validate_contacts({}, {}, {path.polyline}, demand, gap, 0., {contact});
                    }
                    if (!result.passed())
                        dump_oracle_layer(index, lower, upper, cells ? "direct_failure" : "followup_failure",
                                          deposited_support(lower), {source.polyline}, cells ? direct_scope : followup_scope, &result, gap);
                    CAPTURE(index, upper.print_z, gap, result.failure_reason,
                            result.max_unsupported_mm, result.max_void_diameter_mm);
                    REQUIRE(result.input_valid);
                    REQUIRE(result.completed);
                    CHECK(result.passed());
                    measured.void_cells += result.void_cells.size();
                    measured.anchored_runs += result.unsupported_runs.size();
                    measured.max_gap = std::max(measured.max_gap, result.max_unsupported_mm);
                    measured.max_void = std::max(measured.max_void, result.max_void_diameter_mm);
                };
                inspect(direct, true);
                inspect(followup, false);
            });
        }
    }
    return measured;
}

void require_inward_construction_landing(const Print &print)
{
    namespace Contact = BridgeAwareCubicContact;
    const PrintObject &object = *print.get_object(size_t(0));
    const auto &layers = object.layers();
    const auto found = std::find_if(layers.begin(), layers.end(), [](const Layer *layer) {
        return std::abs(layer->print_z - 9.8) < EPSILON;
    });
    REQUIRE(found != layers.end());
    REQUIRE(found != layers.begin());
    const Layer &upper = **found, &lower = **std::prev(found);
    REQUIRE(lower.print_z == Approx(9.6));

    // The captured failure was a 1.059 mm floating construction strand, not
    // an overlong bridge. This narrow interior corridor selects its direction
    // without relying on a planner cell ID or admitting crossing bulk strands.
    const Polyline interior(Point::new_scale(8.72, 8.12), Point::new_scale(8.96, 7.33));
    const ExPolygons corridor = union_ex(offset(Polylines{interior}, scale_(.025)));
    const ExPolygons neighborhood = union_ex(offset(Polylines{interior}, scale_(1.)));
    std::vector<const ExtrusionPath *> targets;
    visit_layer(upper, [&](const ExtrusionPath &path) {
        if (path.role() != erInternalInfill) return;
        for (const Polyline &piece : intersection_pl(Polylines{path.polyline}, corridor))
            if (unscale<double>(piece.length()) > .5) {
                targets.push_back(&path);
                break;
            }
    });
    // Final coarsening may remove this provisional strand altogether. If it
    // survives it still needs real anchors; its survival is not the demand.
    size_t nearby_thick_bridges = 0;
    visit_layer(lower, [&](const ExtrusionPath &path) {
        if (path.role() != erBridgeInfill || path.height <= lower.height + EPSILON) return;
        Polygons bead;
        path.polygons_covered_by_width(bead, 0.f);
        if (!intersection_ex(bead, neighborhood).empty()) ++nearby_thick_bridges;
    });
    REQUIRE(nearby_thick_bridges > 0);
    REQUIRE_FALSE(object.instances().empty());
    const auto check_complete_paths = [&](const Layer &before, const Layer &after,
                                          const std::vector<const ExtrusionPath *> &required) {
        for (const PrintInstance &instance : object.instances()) {
            BridgeAwareCubic::SerializedXY phase;
            phase.shift = unscale(Vec2crd(instance.shift));
            if (!print.config().extruder_offset.values.empty()) phase.nozzle_offset = print.config().extruder_offset.values.front();
            phase.plate_origin = print.get_plate_origin().head<2>();
            const Point origin = phase(Point(0, 0));
            const auto emitted = [&](const ExtrusionPath &source) {
                ExtrusionPath actual = source;
                actual.polyline = phase.emitted_polyline(source.polyline);
                actual.polyline.translate(-origin);
                return actual;
            };
            ExtrusionPaths previous;
            visit_layer(before, [&](const ExtrusionPath &path) { previous.push_back(emitted(path)); });
            std::vector<Contact::Bead> beads;
            for (const ExtrusionPath &path : previous)
                beads.push_back({&path, GCodeWriter::serialized_z(before.print_z, true)});
            for (const ExtrusionPath *source : required) {
                const ExtrusionPath actual = emitted(*source);
                const auto contact = Contact::footprint(beads, actual, GCodeWriter::serialized_z(after.print_z, true),
                    0., Contact::FootprintMode::ContactsOnly);
                CAPTURE(after.print_z, before.print_z, actual.width, actual.height, contact.failure_reason);
                REQUIRE(contact.input_valid);
                REQUIRE(contact.completed);
                const ExPolygons demand = union_ex(offset(Polylines{actual.polyline}, scale_(.5 * actual.width)));
                const auto result = BACValidation::validate_contacts({}, {}, {actual.polyline}, demand, 5., 0., {contact});
                CAPTURE(result.failure_reason, result.max_unsupported_mm);
                REQUIRE(result.input_valid);
                REQUIRE(result.completed);
                // Keep complete emitted paths, including their actual starts
                // and ends. Short floating and one-sided runs must also fail.
                CHECK(result.passed());
            }
        }
    };
    check_complete_paths(lower, upper, targets);

    // The continuing obligation is the right rib's first real model walls.
    // Derive that island from sliced material and select positive wall length
    // over sparse fill; neither a cached construction ID nor its old direction
    // is required to survive a valid final solve.
    const auto first_wall = std::find_if(layers.begin(), layers.end(), [](const Layer *layer) {
        return std::abs(layer->print_z - 10.2) < EPSILON;
    });
    REQUIRE(first_wall != layers.end());
    REQUIRE(first_wall != layers.begin());
    const Layer &wall_upper = **first_wall, &wall_lower = **std::prev(first_wall);
    REQUIRE(wall_lower.print_z == Approx(10.));
    REQUIRE(wall_upper.lslices.size() == 2);
    const auto right_rib = std::min_element(wall_upper.lslices.begin(), wall_upper.lslices.end(),
        [](const ExPolygon &a, const ExPolygon &b) { return a.area() < b.area(); });
    REQUIRE(right_rib->area() * SCALING_FACTOR * SCALING_FACTOR == Approx(4. * 18.).margin(.01));
    REQUIRE(right_rib->contains(Point::new_scale(8., 0.)));
    ExPolygons sparse;
    for (const LayerRegion *region : wall_lower.regions())
        for (const Surface &surface : region->fill_surfaces.surfaces)
            if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
                sparse.push_back(surface.expolygon);
    const ExPolygons selected = intersection_ex(union_ex(sparse), ExPolygons{*right_rib});
    REQUIRE_FALSE(selected.empty());
    std::vector<const ExtrusionPath *> walls;
    double selected_wall_length = 0.;
    for (const LayerRegion *region : wall_upper.regions())
        visit_paths(region->perimeters, [&](const ExtrusionPath &path) {
            if (!is_perimeter(path.role())) return;
            double length = 0.;
            for (const Polyline &piece : intersection_pl(Polylines{path.polyline}, selected))
                length += unscale<double>(piece.length());
            if (length > 0.) {
                walls.push_back(&path);
                selected_wall_length += length;
            }
        });
    REQUIRE_FALSE(walls.empty());
    REQUIRE(selected_wall_length > 1.);
    check_complete_paths(wall_lower, wall_upper, walls);
}

} // namespace

TEST_CASE("Bridge aware emitted bead contact can support a centerline outside the lower nominal stroke",
          "[BridgeAwareCubic][Acceptance][ContactModel]")
{
    // At a tangent variable-height plane each .45 x .12 bead has a .33 mm
    // flat section. Their .25 mm lateral separation therefore has real area
    // contact even though the upper axis is outside the lower .225 mm radius.
    ExtrusionPath lower(erInternalInfill, .04, .45f, .12f);
    lower.polyline = Polyline{Point::new_scale(0., 0.), Point::new_scale(10., 0.)};
    ExtrusionPath upper(erBridgeInfill, .04, .45f, .12f);
    upper.polyline = Polyline{Point::new_scale(1., .25), Point::new_scale(9., .25)};
    BridgeAwareCubic::SerializedXY phase;
    phase.shift = Vec2d(.00024, .00032);
    lower.polyline = phase.emitted_polyline(lower.polyline);
    upper.polyline = phase.emitted_polyline(upper.polyline);
    const ExPolygons demand = union_ex(offset(Polylines{upper.polyline}, scale_(.5 * upper.width)));
    Polygons nominal;
    lower.polygons_covered_by_width(nominal, 0.f);
    const auto centerline_only = BACValidation::validate({}, union_ex(nominal), {upper.polyline}, demand, 5., .00001);
    REQUIRE_FALSE(centerline_only.passed());
    REQUIRE(centerline_only.max_unsupported_mm > 5.);
    const auto contact = BridgeAwareCubicContact::footprint({{&lower, .12}}, upper, .24, 0.);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    const auto physical = BACValidation::validate_contacts({}, contact.physical_footprint,
        {upper.polyline}, demand, 5., 0., {contact});
    REQUIRE(physical.passed());
    CHECK(physical.unsupported_runs.empty());
}

TEST_CASE("Bridge aware cubic emitted voids survive varied demand and lattice phases",
          "[BridgeAwareCubic][Acceptance][Phases]")
{
    struct Phase { double x, y, z, angle, gap; };
    const std::array<Phase, 6> phases{{
        {-9.1, -7.3, 12.1, 0., 3.}, {2.7, 8.2, 13.8, 17., 5.},
        {7.8, -2.6, 18.4, 43., 4.}, {-3.4, 5.9, 21.7, 71., 6.},
        {11.2, 1.3, 24.3, 113., 3.5}, {-6.7, -10.1, 27.6, 151., 5.5}
    }};
    const ExPolygon domain = rectangle(-40., -35., 40., 35.);
    const ExPolygons frame = diff_ex(ExPolygons{domain}, ExPolygons{rectangle(-39.5, -34.5, 39.5, 34.5)});
    size_t rejected_bulk_controls = 0;
    for (const Phase &phase : phases) {
        CAPTURE(phase.x, phase.y, phase.z, phase.angle, phase.gap);
        BridgeAwareCubic::Parameters parameters;
        parameters.bounds = BoundingBoxf3(Vec3d(-40., -35., 0.), Vec3d(40., 35., 35.));
        parameters.flow_spacing = .45;
        parameters.density = .10;
        parameters.angle = phase.angle * PI / 180.;
        const ExPolygons demand{rectangle(phase.x - 7., phase.y - .225, phase.x + 7., phase.y + .225)};
        const Polylines roof{Polyline{Point::new_scale(-39.75, phase.y), Point::new_scale(39.75, phase.y)}};
        const auto inspect = [&](const BridgeAwareCubic::Plan &plan) {
            ExPolygons support = union_ex(offset(plan.lines(phase.z, domain), scale_(.225)));
            append(support, frame);
            return BACValidation::validate({domain}, union_ex(support), roof, demand, phase.gap, .00001);
        };
        const BridgeAwareCubic::Plan refined(parameters, {{demand, phase.z, phase.gap}});
        const auto result = inspect(refined);
        CAPTURE(result.failure_reason, result.max_unsupported_mm, result.max_void_diameter_mm);
        REQUIRE(result.passed());
        REQUIRE_FALSE(result.void_cells.empty());
        CHECK(result.max_void_diameter_mm <= phase.gap);
        CHECK(result.max_unsupported_mm <= phase.gap);
        // The oracle must detect the original missing-refinement behavior.
        const BridgeAwareCubic::Plan bulk(parameters, {});
        if (!inspect(bulk).passed()) ++rejected_bulk_controls;
    }
    REQUIRE(rejected_bulk_controls == phases.size());
}

TEST_CASE("Bridge aware cubic qualifies actual cells and anchors for every bridge stack count",
          "[BridgeAwareCubic][Acceptance][Stack]")
{
    const int count = GENERATE(1, 2, 3);
    const double gap = count == 1 ? 3.5 : count == 2 ? 5. : 6.;
    auto model = cube(26. + .7 * count, 22. + .3 * count, 13.6 + .4 * count);
    auto config = acceptance_config(gap);
    config.set_deserialize_strict({{"internal_bridge_layers", std::to_string(count)},
                                   {"thick_bridges", "1"},
                                   {"infill_direction", std::to_string(13. + 23. * count)}});
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    std::set<size_t> followup_layers;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions())
            for (const Surface &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == stInternalBridge && surface.bridge_followup)
                    followup_layers.insert(layer->id());
    REQUIRE(followup_layers.size() == size_t(count - 1));
    const auto report = measure_emitted_geometry(print, gap);
    REQUIRE(report.selected_paths > 0);
    REQUIRE(report.void_cells > 0);
    REQUIRE(report.anchored_runs > 0);
    CHECK(report.max_gap <= gap);
    CHECK(report.max_void <= gap);
    if (count > 1) REQUIRE(report.bridge_followup_paths > 0);
}

TEST_CASE("Bridge aware cubic supports ribs whose walls begin inside a sparse body",
          "[BridgeAwareCubic][Acceptance][InwardWalls]")
{
    Model model;
    auto *object = model.add_object();
    object->name = "Bridge aware cubic inward wall acceptance fixture";
    TriangleMesh first(its_make_cube(5., 22., 5.));
    first.translate(6., 2., 10.);
    TriangleMesh second(its_make_cube(4., 18., 3.4));
    second.translate(22., 4., 10.);
    // Assemble in one common coordinate frame before add_volume recenters the
    // mesh. Merging raw ribs into an already centered volume moves them above
    // and beside the body when the volume transform is applied.
    TriangleMesh body(its_make_cube(32., 26., 10.));
    body.merge(first);
    body.merge(second);
    object->add_volume(std::move(body));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    auto config = acceptance_config(5.);
    // Ordinary shell projection would make the inward wall bands solid before
    // the ribs begin. Exercise sparse infill's own wall-support obligation.
    config.set_deserialize_strict({{"ensure_vertical_shell_thickness", "disabled"}});
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    const auto &rib_layers = print.get_object(size_t(0))->layers();
    const auto first_rib_layer = std::find_if(rib_layers.begin(), rib_layers.end(), [](const Layer *layer) {
        return std::abs(layer->print_z - 10.2) < EPSILON;
    });
    REQUIRE(first_rib_layer != rib_layers.end());
    REQUIRE(first_rib_layer != rib_layers.begin());
    REQUIRE((*first_rib_layer)->lslices.size() == 2);
    double rib_area = 0., body_area = 0.;
    for (const ExPolygon &shape : (*first_rib_layer)->lslices) rib_area += shape.area() * SCALING_FACTOR * SCALING_FACTOR;
    for (const ExPolygon &shape : (*std::prev(first_rib_layer))->lslices) body_area += shape.area() * SCALING_FACTOR * SCALING_FACTOR;
    CHECK(rib_area == Approx(5. * 22. + 4. * 18.).margin(.01));
    CHECK(body_area == Approx(32. * 26.).margin(.01));
    require_inward_construction_landing(print);
    const auto report = measure_emitted_geometry(print, 5.);
    if (report.wall_paths == 0) {
        const auto &layers = print.get_object(size_t(0))->layers();
        for (size_t index = 1; index < layers.size(); ++index)
            if (std::abs(layers[index]->print_z - 10.2) < EPSILON)
                dump_oracle_layer(index, *layers[index - 1], *layers[index], "rib_zero_selected_walls",
                                  deposited_support(*layers[index - 1]));
    }
    REQUIRE(report.selected_paths > 0);
    REQUIRE(report.wall_paths > 0);
    REQUIRE(std::any_of(report.wall_target_z.begin(), report.wall_target_z.end(),
                        [](double z) { return std::abs(z - 10.2) < EPSILON; }));
    REQUIRE(report.void_cells > 0);
    REQUIRE(report.target_layers.size() >= 2);
}

TEST_CASE("Bridge aware cubic qualifies roof support with variable layer heights",
          "[BridgeAwareCubic][Acceptance][VariableHeight]")
{
    auto model = cube(27.3, 23.7, 14.);
    model.objects.front()->layer_height_profile.set(
        std::vector<coordf_t>{0., .2, 3., .2, 5., .12, 12., .12, 14., .24});
    auto config = acceptance_config(4.5);
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    std::set<coord_t> layer_heights;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        layer_heights.insert(scale_(layer->height));
    REQUIRE(layer_heights.size() >= 3);
    const auto report = measure_emitted_geometry(print, 4.5);
    REQUIRE(report.void_cells > 0);
    REQUIRE(report.anchored_runs > 0);
    REQUIRE(report.height_transition_paths > 0);
    CHECK(report.max_gap <= 4.5);
}

TEST_CASE("Bridge aware cubic leaves a blind cavity empty while supporting its surrounding roof",
          "[BridgeAwareCubic][Acceptance][HoleClosure]")
{
    auto model = cube(28., 24., 14.);
    TriangleMesh bore(its_make_cube(3., 3., 9.));
    bore.translate(12.5, 10.5, -.5);
    auto *negative = model.objects.front()->add_volume(bore);
    negative->set_type(ModelVolumeType::NEGATIVE_VOLUME);
    auto config = acceptance_config(5.);
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    const auto &layers = print.get_object(size_t(0))->layers();
    size_t cavity_layers = 0, closures = 0;
    for (size_t index = 1; index < layers.size(); ++index) {
        Polygons holes;
        for (const ExPolygon &slice : layers[index - 1]->lslices)
            for (Polygon hole : slice.holes) {
                hole.make_counter_clockwise();
                holes.push_back(std::move(hole));
            }
        if (holes.empty()) continue;
        ++cavity_layers;
        const ExPolygons core = offset_ex(holes, -scale_(.35));
        Polygons material;
        visit_layer(*layers[index - 1], [&](const ExtrusionPath &path) { path.polygons_covered_by_width(material, 0.f); });
        CHECK(intersection_ex(material, core).empty());
        // The intentionally empty cavity is outside sparse demand. Check its
        // short closing bridge directly without inventing a model-cell wall.
        Polylines roof;
        visit_layer(*layers[index], [&](const ExtrusionPath &path) {
            if (path.role() != erInternalInfill && !intersection_pl(Polylines{path.polyline}, core).empty())
                roof.push_back(path.polyline);
        });
        if (roof.empty()) continue;
        ++closures;
        const auto result = BACValidation::validate({}, deposited_support(*layers[index - 1]), roof, core, 5., .00001);
        CAPTURE(index, result.failure_reason, result.max_unsupported_mm);
        CHECK(result.passed());
    }
    REQUIRE(cavity_layers > 20);
    REQUIRE(closures > 0);
    const auto report = measure_emitted_geometry(print, 5.);
    REQUIRE(report.void_cells > 0);
}
