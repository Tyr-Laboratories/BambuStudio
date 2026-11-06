#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/Fill/BridgeAwareCubicAttachment.hpp"
#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicClipMask.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSerialization.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSupport.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Attachment = Slic3r::BridgeAwareCubicAttachment;
namespace Validation = Slic3r::BridgeAwareCubicValidation;
namespace Contact = Slic3r::BridgeAwareCubicContact;

namespace {

ExPolygon box(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon({Point::new_scale(x0, y0), Point::new_scale(x1, y0),
                              Point::new_scale(x1, y1), Point::new_scale(x0, y1)}));
}

Polyline line(double x0, double y0, double x1, double y1)
{
    return Polyline(Point::new_scale(x0, y0), Point::new_scale(x1, y1));
}

void visit_paths(const ExtrusionEntity &entity, const std::function<void(const ExtrusionPath &)> &visit)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity))
        visit(*path);
    else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const ExtrusionEntity *child : collection->entities) visit_paths(*child, visit);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const ExtrusionPath &path : multi->paths) visit(path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const ExtrusionPath &path : loop->paths) visit(path);
}

std::vector<const ExtrusionPath *> paths(const LayerRegion &region)
{
    std::vector<const ExtrusionPath *> result;
    visit_paths(region.fills, [&](const ExtrusionPath &path) { result.push_back(&path); });
    return result;
}

double interface_path_length(const Polylines &polylines)
{
    double result = 0.;
    for (const Polyline &polyline : polylines) result += unscale<double>(polyline.length());
    return result;
}

Point formatter_xy(const BAC::SerializedXY &transform, const Point &point)
{
    // Use the production formatter as an independent decimal-output oracle.
    const Vec2d by_gcode = unscale(point) + transform.shift - transform.nozzle_offset;
    const Vec2d by_writer(by_gcode.x() - transform.plate_origin.x(),
                          by_gcode.y() - transform.plate_origin.y());
    GCodeG1Formatter formatter;
    formatter.emit_xy(by_writer);
    std::istringstream emitted(formatter.string());
    std::string token;
    Point result(0, 0);
    bool found_x = false, found_y = false;
    while (emitted >> token) {
        if (token.front() == 'X') {
            result.x() = coord_t(std::llround(std::stod(token.substr(1)) / SCALING_FACTOR));
            found_x = true;
        } else if (token.front() == 'Y') {
            result.y() = coord_t(std::llround(std::stod(token.substr(1)) / SCALING_FACTOR));
            found_y = true;
        }
    }
    REQUIRE(found_x);
    REQUIRE(found_y);
    return result;
}

// Preserve complete path point sequences, including orientation and endpoints.
// Sorting removes only collection scheduling order from the exact comparison.
using ReslicePath = std::tuple<size_t, size_t, double, int, double, double, double, Points>;

std::vector<ReslicePath> reslice_paths(const Print &print, bool perimeters)
{
    std::vector<ReslicePath> result;
    const auto &layers = print.get_object(size_t(0))->layers();
    for (size_t layer_index = 0; layer_index < layers.size(); ++layer_index) {
        const Layer &layer = *layers[layer_index];
        for (size_t region_index = 0; region_index < layer.region_count(); ++region_index) {
            const LayerRegion &region = *layer.get_region(int(region_index));
            const auto record = [&](const ExtrusionPath &path) {
                result.emplace_back(layer_index, region_index, layer.print_z, int(path.role()),
                    path.width, path.height, path.mm3_per_mm, path.polyline.points);
            };
            visit_paths(perimeters ? region.perimeters : region.fills, record);
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

void require_rib_perimeters_above_sparse(const Print &print, double rib_base_z)
{
    const PrintObject &object = *print.get_object(size_t(0));
    const auto &layers = object.layers();
    size_t target_layers = 0, target_paths = 0;
    size_t closed_seam_witnesses = 0;
    double new_wall_length = 0., sparse_contact_length = 0.;
    for (size_t index = 1; index < layers.size(); ++index) {
        const Layer &lower = *layers[index - 1], &upper = *layers[index];
        if (std::abs(lower.print_z - rib_base_z) > EPSILON ||
            std::abs(upper.print_z - (rib_base_z + .2)) > EPSILON) continue;
        ExPolygons sparse;
        Polygons previous_non_sparse, previous_sparse;
        for (const LayerRegion *region : lower.regions()) {
            for (const Surface &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
                    sparse.push_back(surface.expolygon);
            const auto deposited = [&](const ExtrusionPath &path) {
                if (path.mm3_per_mm <= 0. || path.role() == erIroning) return;
                path.polygons_covered_by_width(
                    path.role() == erInternalInfill ? previous_sparse : previous_non_sparse, 0.f);
            };
            visit_paths(region->perimeters, deposited);
            visit_paths(region->fills, deposited);
        }
        REQUIRE_FALSE(sparse.empty());
        sparse = union_ex(sparse);
        previous_non_sparse = union_(previous_non_sparse);
        previous_sparse = union_(previous_sparse);
        REQUIRE_FALSE(previous_sparse.empty());
        std::vector<std::pair<const ExtrusionPath *, double>> closed_targets;
        for (const LayerRegion *region : upper.regions())
            visit_paths(region->perimeters, [&](const ExtrusionPath &path) {
                if (!is_perimeter(path.role()) || path.mm3_per_mm <= 0. || path.is_force_no_extrusion()) return;
                // Shell expansion and bridge anchors may extend into nominal
                // sparse surfaces. Exclude their actual deposited footprint.
                const Polylines new_walls = diff_pl(intersection_pl(Polylines{path.polyline}, sparse), previous_non_sparse);
                const double length = interface_path_length(new_walls);
                if (length > 0.) {
                    ++target_paths;
                    new_wall_length += length;
                    sparse_contact_length += interface_path_length(intersection_pl(new_walls, previous_sparse));
                }
                if (path.polyline.size() >= 2 && path.polyline.points.front() == path.polyline.points.back() &&
                    !intersection_pl(Polylines{path.polyline}, sparse).empty())
                    closed_targets.emplace_back(&path, region->region().config().bridge_aware_cubic_max_gap.value);
            });
        REQUIRE_FALSE(object.instances().empty());
        for (const PrintInstance &instance : object.instances()) {
            BAC::SerializedXY phase;
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
            for (const LayerRegion *region : lower.regions()) {
                const auto collect = [&](const ExtrusionPath &path) {
                    if (!path.is_force_no_extrusion() && path.mm3_per_mm > 0. && path.polyline.size() >= 2 && path.role() != erIroning)
                        previous.push_back(emitted(path));
                };
                visit_paths(region->perimeters, collect);
                visit_paths(region->fills, collect);
            }
            std::vector<Contact::Bead> beads;
            for (const ExtrusionPath &path : previous)
                beads.push_back({&path, GCodeWriter::serialized_z(lower.print_z, true)});
            for (const auto &[source, gap] : closed_targets) {
                const ExtrusionPath actual = emitted(*source);
                const auto contact = Contact::footprint(beads, actual, GCodeWriter::serialized_z(upper.print_z, true),
                    0., Contact::FootprintMode::ContactsOnly);
                CAPTURE(upper.print_z, gap, actual.width, actual.height, contact.failure_reason);
                REQUIRE(contact.input_valid);
                REQUIRE(contact.completed);
                REQUIRE_FALSE(contact.segments.empty());
                const ExPolygons demand = union_ex(offset(Polylines{actual.polyline}, scale_(.5 * actual.width)));
                // Preserve the loop's actual seam and orientation. Clipping or
                // rotating it could hide an unsupported start/end at that seam.
                const auto result = Validation::validate_contacts({}, {}, {actual.polyline}, demand, gap, 0., {contact});
                CAPTURE(result.failure_reason, result.max_unsupported_mm);
                REQUIRE(result.input_valid);
                REQUIRE(result.completed);
                CHECK(result.passed());
                const Point seam = actual.polyline.points.front();
                const auto seam_supported = [&](const Contact::SegmentContact &segment, bool beginning) {
                    const size_t point = segment.segment_index;
                    const Polyline axis(actual.polyline.points[point], actual.polyline.points[point + 1]);
                    for (const Polyline &piece : intersection_pl(Polylines{axis}, segment.centerline_support))
                        if (piece.length() > 0. && (piece.points.front() == seam || piece.points.back() == seam)) return true;
                    for (const Contact::ExactInterval &interval : segment.line_contacts)
                        if (beginning ? interval.begin_numerator == 0 : interval.end_numerator == interval.end_denominator)
                            return true;
                    return false;
                };
                // An arbitrary nominal bead boundary is not a landing. Each
                // occurrence of the fixed seam needs finite physical contact.
                CHECK(seam_supported(contact.segments.front(), true));
                CHECK(seam_supported(contact.segments.back(), false));
                ++closed_seam_witnesses;
            }
        }
        ++target_layers;
    }
    REQUIRE(target_layers == 1);
    REQUIRE(target_paths > 0);
    REQUIRE(new_wall_length > 1.);
    REQUIRE(sparse_contact_length > 0.);
    REQUIRE(closed_seam_witnesses > 0);
}

struct InterfaceFixture {
    Model model;
    Print print;
    PrintObject *object = nullptr;
    Layer *layer = nullptr;
    ExPolygons allowed;
    ExPolygon cavity = box(-.5, -.5, .5, .5);

    explicit InterfaceFixture(bool two_output_phases = false)
    {
        ModelObject *model_object = model.add_object();
        model_object->name = "Bridge aware cubic emission fixture";
        model_object->add_volume(TriangleMesh(its_make_cube(20., 20., 1.2)));
        TriangleMesh modifier_mesh(its_make_cube(10., 20., 1.2));
        modifier_mesh.translate(10., 0., 0.);
        ModelVolume *modifier = model_object->add_volume(modifier_mesh);
        modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
        modifier->config.set("bridge_aware_cubic_max_gap", 7.);
        model_object->add_instance()->set_offset(Vec3d(60., 60., 0.));
        if (two_output_phases)
            model_object->add_instance()->set_offset(Vec3d(60.0005, 60.0005, 0.));

        DynamicPrintConfig config;
        config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
        config.set_deserialize_strict({
            {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
            {"sparse_infill_pattern", "bridgeawarecubic"}, {"sparse_infill_density", "5%"},
            {"bridge_aware_cubic_max_gap", "8"}, {"fill_multiline", "1"},
            {"top_shell_layers", "0"}, {"top_shell_thickness", "0"},
            {"bottom_shell_layers", "0"}, {"bottom_shell_thickness", "0"},
            {"wall_loops", "2"}, {"wall_generator", "classic"},
            {"minimum_sparse_infill_area", "0"}, {"detect_narrow_internal_solid_infill", "0"},
            {"brim_type", "no_brim"}, {"enable_arc_fitting", "0"}, {"seam_gap", "0"}
        });
        print.set_status_silent();
        print.auto_assign_extruders(model_object);
        print.apply(model, config);
        print.process();
        object = print.get_object(size_t(0));
        REQUIRE(object->num_printing_regions() == 2);
        REQUIRE(object->layer_count() > 2);
        layer = object->get_layer(2);
        REQUIRE(layer->region_count() == 2);

        // Keep real region flows and plan ownership, but make the emitted-path
        // fixture independent of roof selection or the hierarchy's split depth.
        allowed = diff_ex(box(-5., -5., 5., 5.), ExPolygons{cavity});
        layer->lslices = allowed;
        for (size_t region_id = 0; region_id < 2; ++region_id) {
            LayerRegion *region = layer->get_region(int(region_id));
            const ExPolygon half = region_id == 0 ? box(-5., -5., 0., 5.) : box(0., -5., 5., 5.);
            const ExPolygons domain = intersection_ex(half, allowed);
            region->slices.clear();
            region->slices.append(domain, stInternal);
            region->fill_surfaces.clear();
            region->fill_surfaces.append(domain, stInternal);
            region->perimeters.clear();
            region->fills.clear();
            install(region_id, {}, {});
        }
    }

    void install(size_t storage_region, std::vector<BAC::LayerClosure> closures,
                 std::vector<BAC::LayerAttachment> attachments)
    {
        // PrintObject owns a mutable Plan; its public getter intentionally
        // returns const for slicer consumers. Replace that value only in this
        // controlled fixture, without exposing production cache mutation APIs.
        BAC::Plan *plan = const_cast<BAC::Plan *>(object->bridge_aware_cubic_plan(storage_region));
        REQUIRE(plan != nullptr);
        const BAC::Parameters parameters = plan->parameters();
        *plan = BAC::Plan(parameters, {}, std::move(closures), std::move(attachments));
    }

    void clear_fills()
    {
        for (LayerRegion *region : layer->regions()) region->fills.clear();
    }

    void append(size_t region_id, const Polyline &polyline, float width, float height)
    {
        auto *path = new ExtrusionPath(erSolidInfill, double(width) * height, width, height);
        path->polyline = polyline;
        layer->get_region(int(region_id))->fills.entities.push_back(path);
    }
};

} // namespace

TEST_CASE("Bridge aware partition provenance tolerates rounded short segments without accepting crossings",
          "[BridgeAwareCubic][Interfaces][PartitionProvenance]")
{
    // Both actual short cube segments lie within two coordinate units of
    // their ideal 105-degree partition, but their angles are rounded. Clipping
    // them through a thin tube can create residual finite-width wedges.
    const bool latest_cube_segment = GENERATE(false, true);
    Polyline candidate = latest_cube_segment ?
        Polyline(Point(516040, 1328481), Point(515152, 1331795)) :
        Polyline(Point(-856506, -1331795), Point(-857108, -1329549));
    const Vec2d midpoint = unscale(latest_cube_segment ? Point(515596, 1330138) : Point(-856807, -1330672));
    const double angle = 105. * PI / 180.;
    const Vec2d direction(std::cos(angle), std::sin(angle));
    const Vec2d normal(-direction.y(), direction.x());
    const auto segment = [](const Vec2d &center, const Vec2d &axis, double begin, double end) {
        return Polyline(Point::new_scale(Vec2d(center + begin * axis)),
                        Point::new_scale(Vec2d(center + end * axis)));
    };
    Polyline partition = segment(midpoint, direction, -10., 10.);
    bool genuine_owner = true;
    SECTION("Its long ideal partition owns the complete short segment") {}
    SECTION("Reversing both paths preserves ownership") {
        candidate.reverse();
        partition.reverse();
    }
    SECTION("A perpendicular crossing does not own the segment") {
        partition = segment(midpoint, normal, -10., 10.);
        genuine_owner = false;
    }
    SECTION("An oblique crossing does not own the segment") {
        partition = segment(midpoint, Vec2d(1., 0.), -10., 10.);
        genuine_owner = false;
    }
    SECTION("A parallel partition one millimetre away does not own the segment") {
        partition = segment(Vec2d(midpoint + normal), direction, -10., 10.);
        genuine_owner = false;
    }
    SECTION("A disjoint collinear partition does not own the segment") {
        partition = segment(midpoint, direction, 20., 30.);
        genuine_owner = false;
    }
    const Polylines matching = BAC::matching_partition_segments({candidate}, {partition});
    if (!genuine_owner) {
        CHECK(matching.empty());
    } else {
        REQUIRE_FALSE(matching.empty());
        REQUIRE(matching.size() == 1);
        CHECK(matching.front().points == candidate.points);
        constexpr float coordinate_tolerance = 2.f;
        const Polylines candidates{candidate};
        CHECK(diff_pl(candidates, offset(matching, coordinate_tolerance,
            ClipperLib::jtRound, 0., ClipperLib::etOpenRound)).empty());
        CHECK(diff_pl(matching, offset(candidates, coordinate_tolerance,
            ClipperLib::jtRound, 0., ClipperLib::etOpenRound)).empty());
        CHECK(interface_path_length(matching) ==
              Approx(unscale<double>(candidate.length())).margin(4. * SCALING_FACTOR));
    }
}

TEST_CASE("Bridge aware partition provenance still clips a partially owned source segment",
          "[BridgeAwareCubic][Interfaces][PartitionProvenance]")
{
    const Polyline candidate(Point(0, 0), Point(400000, 0));
    const Polyline partition(Point(100000, 0), Point(300000, 0));
    const Polylines matching = BAC::matching_partition_segments({candidate}, {partition});
    REQUIRE(matching.size() == 1);
    CHECK(matching.front().points != candidate.points);
    CHECK(interface_path_length(matching) == Approx(2.));
    const Point first = matching.front().first_point(), last = matching.front().last_point();
    CHECK(((first == Point(100000, 0) && last == Point(300000, 0)) ||
           (first == Point(300000, 0) && last == Point(100000, 0))));
}

TEST_CASE("Bridge aware construction completeness requires positive matching geometry for a tiny source",
          "[BridgeAwareCubic][Interfaces][ConstructionMatch]")
{
    constexpr double tolerance = .00216;
    Polyline source(Point(10, 859559), Point(17, 859552));
    if (GENERATE(false, true)) source.reverse();
    const Points original = source.points;
    REQUIRE(unscale<double>(source.length()) < tolerance);

    const Polylines unmatched = intersection_pl(Polylines{source}, ExPolygons{box(1., 8., 2., 9.)});
    REQUIRE(unmatched.empty());
    // A zero-match residual is shorter than the old absolute tolerance, but
    // cannot turn an ordinary bulk sliver into added construction material.
    REQUIRE(source.length() - scale_(interface_path_length(unmatched)) <= scale_(tolerance));
    CHECK_FALSE(BAC::complete_construction_match(source, unmatched, tolerance));

    const Polylines clipped = intersection_pl(Polylines{source}, ExPolygons{box(-.01, 8.58, .01, 8.61)});
    REQUIRE_FALSE(clipped.empty());
    CHECK(interface_path_length(clipped) == Approx(unscale<double>(source.length())));
    CHECK(BAC::complete_construction_match(source, clipped, tolerance));
    const Polylines owned = BAC::matching_partition_segments(Polylines{source}, Polylines{source});
    REQUIRE(owned.size() == 1);
    CHECK(owned.front().points == original);
    CHECK(BAC::complete_construction_match(source, owned, tolerance));
    CHECK(source.points == original);
}

TEST_CASE("Bridge aware construction completeness rejects real partial and unmatched long sources",
          "[BridgeAwareCubic][Interfaces][ConstructionMatch]")
{
    const Polyline source = line(0., 0., 10., 0.);
    const Points original = source.points;
    constexpr double tolerance = .00216;
    const Polylines partial = intersection_pl(Polylines{source}, ExPolygons{box(-1., -1., 4., 1.)});
    REQUIRE_FALSE(partial.empty());
    REQUIRE(interface_path_length(partial) == Approx(4.));
    CHECK_FALSE(BAC::complete_construction_match(source, partial, tolerance));
    const Polylines unmatched = intersection_pl(Polylines{source}, ExPolygons{box(-1., 2., 11., 3.)});
    REQUIRE(unmatched.empty());
    CHECK_FALSE(BAC::complete_construction_match(source, unmatched, tolerance));
    CHECK(source.points == original);
}

TEST_CASE("Bridge aware emission removes only wholly stationary owned construction paths",
          "[BridgeAwareCubic][Interfaces][ConstructionMatch][Serialization][StationaryConstruction]")
{
    InterfaceFixture fixture(true);
    constexpr size_t region_id = 1;
    LayerRegion &region = *fixture.layer->get_region(int(region_id));
    BAC::Plan *plan = const_cast<BAC::Plan *>(fixture.object->bridge_aware_cubic_plan(region_id));
    REQUIRE(plan != nullptr);
    const double z = fixture.layer->print_z;
    const ExPolygon interior = box(1., 1., 4., 4.);
    *plan = BAC::Plan(plan->parameters(), {BAC::Demand{{interior}, z, 2.}});
    REQUIRE_FALSE(plan->refinements().empty());

    Polylines partitions;
    for (const Surface &surface : region.slices.surfaces)
        for (const auto &owned : plan->partition_lines(z, surface.expolygon))
            append(partitions, owned.second);
    REQUIRE_FALSE(partitions.empty());
    BAC::Plan ordinary_plan(plan->parameters(), {});
    const BoundingBox bounds = fixture.object->bounding_box();
    const ExPolygon broad_domain(Polygon({bounds.min, Point(bounds.max.x(), bounds.min.y()),
        bounds.max, Point(bounds.min.x(), bounds.max.y())}));
    const Polylines ordinary_bulk = ordinary_plan.lines(z, broad_domain);
    REQUIRE_FALSE(ordinary_bulk.empty());
    Polyline moving;
    for (const auto &owned : plan->partition_lines(z, interior))
        for (const Polyline &partition : owned.second)
            for (const Line &segment : partition.lines())
                if (segment.length() > moving.length()) moving = Polyline(segment.a, segment.b);
    REQUIRE(unscale<double>(moving.length()) > .5);
    REQUIRE(BAC::complete_construction_match(moving,
        BAC::matching_partition_segments(Polylines{moving}, partitions), 2. * SCALING_FACTOR));

    REQUIRE(fixture.object->instances().size() == 2);
    std::vector<BAC::SerializedXY> phases;
    for (const PrintInstance &instance : fixture.object->instances()) {
        BAC::SerializedXY phase;
        phase.shift = unscale(Vec2crd(instance.shift));
        phase.nozzle_offset = fixture.print.config().extruder_offset.values.front();
        phase.plate_origin = fixture.print.get_plate_origin().head<2>();
        phases.push_back(phase);
    }
    const auto stationary = [](const Polyline &path, const BAC::SerializedXY &phase) {
        const Polyline emitted = phase.emitted_polyline(path);
        return !emitted.empty() && std::all_of(emitted.points.begin(), emitted.points.end(),
            [&](const Point &point) { return point == emitted.first_point(); });
    };
    const Vec2d start = unscale(moving.first_point());
    const Vec2d delta = unscale(moving.last_point()) - start;
    const Vec2d direction = delta.normalized();
    Polyline owned_stationary;
    for (size_t sample = 0; sample < 1000 && owned_stationary.empty(); ++sample) {
        const Vec2d begin = start + (.1 + .8 * double(sample) / 999.) * delta;
        const Polyline candidate(Point::new_scale(begin), Point::new_scale(Vec2d(begin + .0002 * direction)));
        if (candidate.first_point() != candidate.last_point() && stationary(candidate, phases[0]) &&
            !stationary(candidate, phases[1]) && BAC::complete_construction_match(candidate,
                BAC::matching_partition_segments(Polylines{candidate}, partitions), 2. * SCALING_FACTOR) &&
            BAC::matching_partition_segments(Polylines{candidate}, ordinary_bulk).empty())
            owned_stationary = candidate;
    }
    REQUIRE_FALSE(owned_stationary.empty());
    REQUIRE(stationary(owned_stationary, phases[0]));
    REQUIRE_FALSE(stationary(owned_stationary, phases[1]));
    for (const auto &phase : phases) REQUIRE_FALSE(stationary(moving, phase));

    // Translate by whole output-grid steps to preserve the stationary phase,
    // while requiring an actual zero match against every active partition.
    Polyline ordinary_stationary;
    for (int offset = 1; offset <= 20 && ordinary_stationary.empty(); ++offset) {
        Polyline candidate = owned_stationary;
        const Point shift(scale_(.01 * offset), scale_(.013 * offset));
        for (Point &point : candidate.points) point += shift;
        if (diff_pl(Polylines{candidate}, region.slices.surfaces.front().expolygon).empty() &&
            stationary(candidate, phases[0]) &&
            BAC::matching_partition_segments(Polylines{candidate}, partitions).empty())
            ordinary_stationary = std::move(candidate);
    }
    REQUIRE_FALSE(ordinary_stationary.empty());
    REQUIRE(stationary(ordinary_stationary, phases[0]));
    REQUIRE(intersection_pl(Polylines{ordinary_stationary}, plan->solid_area(z)).empty());
    REQUIRE(plan->closures(z).empty());
    REQUIRE(plan->attachments(z).empty());

    // A microscopic original bulk strand at a true crossing can lie wholly
    // inside a partition's two-coordinate-unit provenance envelope despite
    // having a different direction. Positive bulk provenance must protect it.
    Polyline ordinary_crossing;
    const ExPolygons interior_material = offset_ex(to_expolygons(region.slices.surfaces), -scale_(.25));
    const auto cross = [](const Vec2d &a, const Vec2d &b) { return a.x() * b.y() - a.y() * b.x(); };
    const auto rounded_point = [](const Vec2d &point) {
        return Point(coord_t(std::llround(point.x())), coord_t(std::llround(point.y())));
    };
    for (const Polyline &bulk : ordinary_bulk) for (const Line &source : bulk.lines())
        for (const Polyline &partition : partitions) for (const Line &owner : partition.lines()) {
            if (!ordinary_crossing.empty()) continue;
            const Vec2d p = source.a.cast<double>(), r = (source.b - source.a).cast<double>();
            const Vec2d q = owner.a.cast<double>(), s = (owner.b - owner.a).cast<double>();
            const double denominator = cross(r, s);
            if (r.norm() == 0. || s.norm() == 0. || std::abs(denominator) < .1 * r.norm() * s.norm()) continue;
            const double t = cross(q - p, s) / denominator, u = cross(q - p, r) / denominator;
            if (t < 0. || t > 1. || u < 0. || u > 1.) continue;
            const Vec2d intersection = p + t * r, axis = r.normalized();
            const Polyline candidate(rounded_point(Vec2d(intersection - 1.25 * axis)),
                                     rounded_point(Vec2d(intersection + 1.25 * axis)));
            if (candidate.first_point() == candidate.last_point() ||
                !diff_pl(Polylines{candidate}, interior_material).empty() ||
                !intersection_pl(Polylines{candidate}, plan->solid_area(z)).empty()) continue;
            if (!std::any_of(phases.begin(), phases.end(), [&](const BAC::SerializedXY &phase) {
                    return stationary(candidate, phase);
                })) continue;
            if (BAC::complete_construction_match(candidate,
                    BAC::matching_partition_segments(Polylines{candidate}, Polylines{bulk}), 2. * SCALING_FACTOR) &&
                BAC::complete_construction_match(candidate,
                    BAC::matching_partition_segments(Polylines{candidate}, Polylines{partition}), 2. * SCALING_FACTOR))
                ordinary_crossing = candidate;
        }
    REQUIRE_FALSE(ordinary_crossing.empty());
    REQUIRE(unscale<double>(ordinary_crossing.length()) < .0001);
    REQUIRE(BAC::complete_construction_match(ordinary_crossing,
        BAC::matching_partition_segments(Polylines{ordinary_crossing}, ordinary_bulk), 2. * SCALING_FACTOR));
    REQUIRE(BAC::complete_construction_match(ordinary_crossing,
        BAC::matching_partition_segments(Polylines{ordinary_crossing}, partitions), 2. * SCALING_FACTOR));
    const Flow flow = region.flow(frInfill);
    const float width = float(flow.width()), height = float(flow.height());
    const auto seed = [&] {
        fixture.clear_fills();
        for (const Polyline &polyline : {owned_stationary, ordinary_stationary, ordinary_crossing, moving}) {
            auto *path = new ExtrusionPath(erInternalInfill, flow.mm3_per_mm(), width, height);
            path->polyline = polyline;
            region.fills.entities.push_back(path);
        }
    };
    const auto check = [&] {
        const auto actual = paths(region);
        REQUIRE(actual.size() == 3);
        for (const Polyline &expected : {ordinary_stationary, ordinary_crossing, moving}) {
            const auto found = std::find_if(actual.begin(), actual.end(), [&](const ExtrusionPath *path) {
                return path->polyline.points == expected.points;
            });
            REQUIRE(found != actual.end());
            CHECK((*found)->role() == erInternalInfill);
            CHECK((*found)->width == width);
            CHECK((*found)->height == height);
            CHECK((*found)->mm3_per_mm == flow.mm3_per_mm());
        }
    };
    seed();
    REQUIRE(paths(region).size() == 4);
    BAC::apply_layer_paths(*fixture.layer);
    check();
    BAC::apply_layer_paths(*fixture.layer);
    check();
    seed();
    BAC::apply_layer_paths(*fixture.layer);
    check();
}

TEST_CASE("Bridge aware repair selects late intervals on the verified serialized path",
          "[BridgeAwareCubic][Interfaces][SerializedRepair]")
{
    // Each tooth is long enough to emit, but decimal XY serialization flattens
    // it. The accumulated source arclength exceeds emitted arclength by over
    // 1.7 mm, moving a late repair far from the verified defect if reused.
    Points points;
    points.reserve(2002);
    for (coord_t i = 0; i <= 2000; ++i)
        points.emplace_back(i * 10, i % 2 == 0 ? -49 : 49);
    points.emplace_back(500000, 0);
    const Polyline original(std::move(points));
    const BAC::SerializedXY transform;
    const Polyline verified = transform.emitted_polyline(original);
    const double original_length = unscale<double>(original.length());
    const double verified_length = unscale<double>(verified.length());
    REQUIRE(verified_length == Approx(5.));
    REQUIRE(original_length - verified_length > 1.7);
    constexpr double width = .4, tolerance = .00002;
    const ExPolygons original_demand = union_ex(offset(Polylines{original}, float(scale_(width / 2.)),
        ClipperLib::jtRound, 0., ClipperLib::etOpenRound));
    REQUIRE_FALSE(original_demand.empty());
    const double begin = verified_length - .03, end = verified_length - .01;
    const ExPolygons selected = BAC::unsupported_interval_demand(
        verified, begin, end, width, tolerance, original_demand);
    REQUIRE_FALSE(selected.empty());
    CHECK(diff_ex(selected, original_demand).empty());
    CHECK(unscale<double>(get_extents(selected).min.x()) > 4.7);
    CHECK(unscale<double>(get_extents(selected).max.x()) < 5.21);
    CHECK_FALSE(intersection_ex(selected, ExPolygons{box(4.97, -.01, 4.99, .01)}).empty());

    // The same finite demand permits the erroneous location too, so clipping
    // cannot hide the difference between original and verified arclength.
    const ExPolygons misplaced = BAC::unsupported_interval_demand(
        original, begin, end, width, tolerance, original_demand);
    REQUIRE_FALSE(misplaced.empty());
    CHECK(unscale<double>(get_extents(misplaced).max.x()) < 3.5);
    CHECK(intersection_ex(selected, misplaced).empty());
}

TEST_CASE("Bridge aware repair confines serialized buffer fringes to their original source bead",
          "[BridgeAwareCubic][Interfaces][SerializedRepair][NeighborDemand]")
{
    const Polyline source(Point(-100000, 49), Point(100000, 49));
    const Polyline neighbor(Point(-100000, -39951), Point(100000, -39951));
    const BAC::SerializedXY transform;
    const Polyline verified = transform.emitted_polyline(source);
    REQUIRE(verified.first_point().y() == 0);
    REQUIRE(unscale<double>(source.first_point().y()) == Approx(.00049));
    constexpr double width = .4, tolerance = .00002;
    const ExPolygons source_bead = union_ex(offset(Polylines{source}, float(scale_(width / 2.))));
    const ExPolygons neighbor_bead = union_ex(offset(Polylines{neighbor}, float(scale_(width / 2.))));
    ExPolygons group = source_bead;
    group.insert(group.end(), neighbor_bead.begin(), neighbor_bead.end());
    group = union_ex(group);
    const ExPolygons neighbor_only = diff_ex(neighbor_bead, source_bead);
    REQUIRE_FALSE(source_bead.empty());
    REQUIRE_FALSE(neighbor_only.empty());
    REQUIRE_FALSE(group.empty());

    // Rounding moves this path toward its neighbor. A radial repair buffer
    // clipped only to their grouped demand steals part of the adjacent bead.
    const ExPolygons unowned = BAC::unsupported_interval_demand(
        verified, .5, 1.5, width, tolerance, group);
    REQUIRE_FALSE(unowned.empty());
    REQUIRE_FALSE(intersection_ex(unowned, neighbor_only).empty());
    const ExPolygons owned = BAC::unsupported_interval_demand(
        verified, .5, 1.5, width, tolerance, group, &source);
    REQUIRE_FALSE(owned.empty());
    CHECK(diff_ex(owned, source_bead).empty());
    CHECK(intersection_ex(owned, neighbor_only).empty());
    CHECK_FALSE(intersection_ex(owned, ExPolygons{box(-.4, -.1, .4, .1)}).empty());
}

TEST_CASE("Bridge aware XY serialization preserves the exporter operation order at rounding ties",
          "[BridgeAwareCubic][Serialization]")
{
    BAC::SerializedXY transform;
    transform.shift = unscale(Point(1000203, 1000056));
    transform.nozzle_offset = Vec2d(.00003, 1.23456);
    transform.plate_origin = Vec2d(2., 2.);
    const Point source(50, 250);
    const Point serialized = transform(source);
    CHECK(serialized == Point(800300, 676800));
    CHECK(serialized == formatter_xy(transform, source));

    // Both tempting algebraic rewrites change a tie in binary arithmetic.
    const double integer_first = (unscale<double>(source.x() + 1000203) - .00003) - 2.;
    const double combined_offset = unscale<double>(source.y()) + ((transform.shift.y() - 1.23456) - 2.);
    CHECK(std::round(integer_first * 1000.) == 8002.);
    CHECK(std::round(combined_offset * 1000.) == 6769.);
}

TEST_CASE("Bridge aware XY serialization matches signed ties and effective nozzle offsets",
          "[BridgeAwareCubic][Serialization]")
{
    const BAC::SerializedXY zero;
    CHECK(zero(Point(49, -49)) == Point(0, 0));
    CHECK(zero(Point(50, -50)) == Point(100, -100));
    CHECK(zero(Point(51, -51)) == Point(100, -100));
    CHECK(zero(Point(150, -150)) == formatter_xy(zero, Point(150, -150)));

    BAC::SerializedXY other_tool;
    other_tool.nozzle_offset = Vec2d(.0002, -.0002);
    const Point source(50, -50);
    CHECK(other_tool(source) == Point(0, 0));
    CHECK(other_tool(source) == formatter_xy(other_tool, source));
    CHECK(other_tool(source) != zero(source));
    // Raw command coordinates from different tool offsets are not a common
    // physical frame; this helper deliberately applies no lossy inverse map.
}

TEST_CASE("Bridge aware XY serialization retains common joints while instance phase changes rounding",
          "[BridgeAwareCubic][Serialization]")
{
    const Point joint(40, -40);
    const Polylines paths{Polyline(Point(-100000, 0), joint), Polyline(joint, Point(0, 100000))};
    const BAC::SerializedXY zero;
    BAC::SerializedXY shifted;
    shifted.shift = unscale(Point(20, -20));
    const Polylines origin_paths = zero(paths), shifted_paths = shifted(paths);
    CHECK(origin_paths[0].last_point() == origin_paths[1].first_point());
    CHECK(shifted_paths[0].last_point() == shifted_paths[1].first_point());
    CHECK(origin_paths[0].last_point() == Point(0, 0));
    CHECK(shifted_paths[0].last_point() == Point(100, -100));
    CHECK(shifted(joint) == formatter_xy(shifted, joint));
    CHECK(shifted(Point(0, 0)) == Point(0, 0));
}

TEST_CASE("Bridge aware XY serialization transforms material contours and holes in the path frame",
          "[BridgeAwareCubic][Serialization]")
{
    ExPolygon domain = box(-2., -2., 2., 2.);
    domain.holes.push_back(box(-.5, -.5, .5, .5).contour);
    domain.holes.front().make_clockwise();
    BAC::SerializedXY transform;
    transform.shift = unscale(Point(1000020, -5000020));
    transform.nozzle_offset = Vec2d(.0012, -.0013);
    transform.plate_origin = Vec2d(3.12345, -2.34567);
    const ExPolygons transformed = transform(ExPolygons{domain});
    REQUIRE(transformed.size() == 1);
    REQUIRE(transformed.front().holes.size() == 1);
    const auto check_ring = [&](const Polygon &original, const Polygon &result) {
        REQUIRE(result.size() == original.size());
        for (size_t i = 0; i < original.size(); ++i)
            CHECK(result.points[i] == formatter_xy(transform, original.points[i]));
    };
    check_ring(domain.contour, transformed.front().contour);
    check_ring(domain.holes.front(), transformed.front().holes.front());
    const Polyline boundary(domain.contour.points[0], domain.contour.points[1]);
    const Polyline transformed_boundary = transform(boundary);
    CHECK(transformed_boundary.first_point() == transformed.front().contour.points[0]);
    CHECK(transformed_boundary.last_point() == transformed.front().contour.points[1]);
}

TEST_CASE("Bridge aware XY emission retains nonzero source segments including rounded zero moves",
          "[BridgeAwareCubic][Serialization]")
{
    const BAC::SerializedXY transform;
    const Polyline source(Points{Point(0, 0), Point(5, 0), Point(100, 0), Point(105, 0)});
    const auto original_points = source.points;
    const Polyline emitted = transform.emitted_polyline(source);
    const Points expected{Point(0, 0), Point(0, 0), Point(100, 0), Point(100, 0)};
    CHECK(emitted.points == expected);
    CHECK(source.points == original_points);
    CHECK(transform(source).size() == 4);

    const Polyline rounded_zero(Point(20, 0), Point(40, 0));
    const Polyline retained = transform.emitted_polyline(rounded_zero);
    REQUIRE(retained.size() == 2);
    CHECK(retained.first_point() == retained.last_point());
    const Polyline short_across_rounding_boundary(Point(45, 0), Point(50, 0));
    CHECK(transform(short_across_rounding_boundary).last_point() == Point(100, 0));
    CHECK(transform.emitted_polyline(short_across_rounding_boundary).points ==
          transform(short_across_rounding_boundary).points);
    const Polyline exact_duplicates(Points{Point(0, 0), Point(0, 0), Point(5, 0), Point(5, 0),
                                          Point(100, 0), Point(105, 0), Point(105, 0)});
    CHECK(transform.emitted_polyline(exact_duplicates).points == expected);
    CHECK(transform.emitted_polyline(Polyline{}).empty());
}

TEST_CASE("Bridge aware XY serialization retains tiny edges across phase-dependent output boundaries",
          "[BridgeAwareCubic][Serialization][TinyEdges]")
{
    const coord_t sign = GENERATE(coord_t(-1), coord_t(1));
    const Polyline original(Point(sign * 40, 0), Point(sign * 45, 0));
    REQUIRE(unscale<double>(original.length()) < EPSILON);
    const BAC::SerializedXY zero;
    REQUIRE(zero(original.first_point()) == zero(original.last_point()));
    CHECK_NOTHROW(zero.validate_path(original));
    CHECK(zero.emitted_polyline(original).points == zero(original).points);
    REQUIRE(zero.emitted_polyline(original).size() == 2);

    BAC::SerializedXY phased;
    phased.shift = unscale(Point(sign * 7, 0));
    REQUIRE(phased(original.first_point()) != phased(original.last_point()));
    CHECK_NOTHROW(phased.validate_path(original));
    CHECK(phased.emitted_polyline(original).points == phased(original).points);
    REQUIRE(phased.emitted_polyline(original).size() == 2);
    Polyline reversed = original;
    reversed.reverse();
    CHECK_NOTHROW(phased.validate_path(reversed));
    CHECK(phased.emitted_polyline(reversed).points == phased(reversed).points);
    for (const BAC::SerializedXY &phase : {zero, phased}) {
        Polyline reverse_output = phase.emitted_polyline(reversed);
        reverse_output.reverse();
        CHECK(reverse_output.points == phase.emitted_polyline(original).points);
    }
}

TEST_CASE("Bridge aware XY serialization retains equal-output tiny edges under reversal",
          "[BridgeAwareCubic][Serialization][TinyEdges]")
{
    BAC::SerializedXY transform;
    transform.shift = unscale(Point(12, -5));
    const Polyline original(Points{Point(0, 0), Point(5, 0), Point(100, 0), Point(105, 0)});
    Polyline reversed = original;
    reversed.reverse();
    CHECK_NOTHROW(transform.validate_path(original));
    CHECK_NOTHROW(transform.validate_path(reversed));
    const Polyline forward_output = transform.emitted_polyline(original);
    Polyline reverse_output = transform.emitted_polyline(reversed);
    REQUIRE(forward_output.size() == 4);
    REQUIRE(reverse_output.size() == 4);
    reverse_output.reverse();
    CHECK(reverse_output.points == forward_output.points);
    CHECK(forward_output.first_point() == transform(original.first_point()));
    CHECK(forward_output.last_point() == transform(original.last_point()));
}

TEST_CASE("Bridge aware XY serialization exposes a finite material join lost by decimal rounding",
          "[BridgeAwareCubic][Serialization][Contact]")
{
    ExtrusionPath first(erInternalInfill, .08, .4498f, .2f);
    first.polyline = Polyline(Point(-100000, 0), Point(45, 0));
    ExtrusionPath second(erInternalInfill, .08, .4498f, .2f);
    second.polyline = Polyline(Point(35, 24970), Point(100000, 24970));
    ExtrusionPath upper(erSolidInfill, .08, .45f, .2f);
    upper.polyline = line(-1., -1., 1., 1.);
    const auto physical_section = [&](const ExtrusionPath &lower) {
        const auto contact = Contact::footprint({{&lower, .2}}, upper, .4, .0001);
        REQUIRE(contact.input_valid);
        REQUIRE(contact.completed);
        return union_ex(contact.physical_footprint);
    };
    const ExPolygons first_before = physical_section(first), second_before = physical_section(second);
    REQUIRE_FALSE(intersection_ex(first_before, second_before).empty());
    const BAC::SerializedXY transform;
    first.polyline = transform.emitted_polyline(first.polyline);
    second.polyline = transform.emitted_polyline(second.polyline);
    CHECK(first.polyline.last_point() == Point(0, 0));
    CHECK(second.polyline.first_point() == Point(0, 25000));
    const ExPolygons first_after = physical_section(first), second_after = physical_section(second);
    CHECK(intersection_ex(first_after, second_after).empty());
}

TEST_CASE("Bridge aware XY serialization rejects invalid coordinates instead of clamping",
          "[BridgeAwareCubic][Serialization]")
{
    BAC::SerializedXY transform;
    transform.shift.x() = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS(transform(Point(0, 0)), std::out_of_range);
    transform.shift.x() = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(transform(Point(0, 0)), std::out_of_range);
    transform.shift.x() = std::numeric_limits<double>::max();
    CHECK_THROWS_AS(transform(Point(0, 0)), std::out_of_range);
    transform.shift.x() = 1e14;
    CHECK_THROWS_AS(transform(Point(0, 0)), std::out_of_range);
}

TEST_CASE("Bridge aware cubic perimeter demands reslice identically to a fresh rib print",
          "[BridgeAwareCubic][Integration][Reslice][PerimeterDemand]")
{
    constexpr double rib_base_z = 6.;
    Model model;
    ModelObject *object = model.add_object();
    object->name = "Bridge aware cubic perimeter demand reslice";
    TriangleMesh body(its_make_cube(24., 20., rib_base_z));
    TriangleMesh rib(its_make_cube(4., 16., 2.));
    rib.translate(8., 2., rib_base_z);
    body.merge(rib);
    object->add_volume(std::move(body));
    object->add_instance()->set_offset(Vec3d(60., 60., 0.));

    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_pattern", "bridgeawarecubic"}, {"sparse_infill_density", "10%"},
        {"bridge_aware_cubic_max_gap", "6"}, {"infill_direction", "17"},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"ensure_vertical_shell_thickness", "disabled"},
        {"minimum_sparse_infill_area", "0"}, {"detect_narrow_internal_solid_infill", "0"},
        {"brim_type", "no_brim"}, {"enable_arc_fitting", "0"},
        {"thick_bridges", "0"}, {"bridge_flow", "1"}, {"internal_bridge_layers", "1"},
        {"fill_multiline", "1"}, {"seam_gap", "0"}
    });
    const auto slice = [&](Print &print) {
        print.set_status_silent();
        print.auto_assign_extruders(object);
        print.apply(model, config);
        print.process();
    };
    Print reused;
    REQUIRE_NOTHROW(slice(reused));
    require_rib_perimeters_above_sparse(reused, rib_base_z);
    REQUIRE_FALSE(reslice_paths(reused, true).empty());
    REQUIRE_FALSE(reslice_paths(reused, false).empty());

    SECTION("Changing the gap rebuilds perimeter support") {
        config.set_deserialize_strict({{"bridge_aware_cubic_max_gap", "8"}});
    }
    SECTION("Changing sparse density rebuilds perimeter support") {
        config.set_deserialize_strict({{"sparse_infill_density", "15%"}});
    }
    REQUIRE_NOTHROW(reused.apply(model, config));
    REQUIRE_NOTHROW(reused.process());
    Print fresh;
    REQUIRE_NOTHROW(slice(fresh));
    require_rib_perimeters_above_sparse(reused, rib_base_z);
    require_rib_perimeters_above_sparse(fresh, rib_base_z);
    CHECK(reslice_paths(reused, true) == reslice_paths(fresh, true));
    CHECK(reslice_paths(reused, false) == reslice_paths(fresh, false));
}

TEST_CASE("Bridge aware solid cuts select the new endpoint without inheriting a remote bulk gap",
          "[BridgeAwareCubic][Interfaces][SolidCut]")
{
    const Polyline actual = line(-12., 0., 0., 0.);
    const ExPolygons solid_area{box(0., -2., 2., 2.)};
    constexpr double width = .4, tolerance = .001, gap = 5.;
    const ExPolygons selected = BAC::solid_cut_endpoint_demand(actual, solid_area, width, tolerance);
    REQUIRE_FALSE(selected.empty());
    CHECK(intersection_ex(selected, ExPolygons{box(-11., -.5, -5., .5)}).empty());

    ExPolygons support{box(-12.5, -.5, -11., .5), box(-5., -.5, -.2, .5)};
    const auto cut = Validation::validate({}, support, {actual}, selected, gap, tolerance);
    REQUIRE(cut.input_valid);
    REQUIRE(cut.completed);
    REQUIRE(cut.unsupported_runs.size() == 1);
    CHECK(cut.unsupported_runs.front().kind == Validation::UnsupportedRunKind::OneSidedOverhang);
    CHECK(cut.unsupported_runs.front().length_mm == Approx(.2));
    CHECK_FALSE(cut.unsupported_runs.front().exceeds_gap);
    CHECK_FALSE(cut.passed());

    // A real finite anchor repairs the new terminal tail. Keep the complete
    // emitted path: the six-millimetre bulk gap still exists farther away.
    support.push_back(box(-.05, -.5, .5, .5));
    const auto anchored = Validation::validate({}, support, {actual}, selected, gap, tolerance);
    REQUIRE(anchored.input_valid);
    REQUIRE(anchored.completed);
    REQUIRE(anchored.unsupported_runs.size() == 1);
    CHECK(anchored.unsupported_runs.front().kind == Validation::UnsupportedRunKind::TwoEndedBridge);
    CHECK(anchored.unsupported_runs.front().length_mm == Approx(.15));
    CHECK(anchored.passed());

    const auto whole_path = Validation::validate({}, support, {actual},
        {box(-13., -.5, 1., .5)}, gap, tolerance);
    REQUIRE(whole_path.input_valid);
    REQUIRE(whole_path.completed);
    CHECK(whole_path.max_unsupported_mm == Approx(6.));
    CHECK_FALSE(whole_path.passed());
    CHECK(std::any_of(whole_path.unsupported_runs.begin(), whole_path.unsupported_runs.end(),
        [](const Validation::UnsupportedRun &run) {
            return run.kind == Validation::UnsupportedRunKind::TwoEndedBridge && run.exceeds_gap;
        }));
}

TEST_CASE("Bridge aware solid cut demand requires an endpoint at the solid boundary",
          "[BridgeAwareCubic][Interfaces][SolidCut]")
{
    const ExPolygons solid_area{box(0., -2., 2., 2.)};
    constexpr double width = .4, tolerance = .001;
    CHECK(BAC::solid_cut_endpoint_demand(line(-12., 0., -3., 0.),
        solid_area, width, tolerance).empty());
    // Crossing the patch is insufficient if neither emitted endpoint is cut.
    CHECK(BAC::solid_cut_endpoint_demand(line(-3., 0., 3., 0.),
        solid_area, width, tolerance).empty());
    CHECK_FALSE(BAC::solid_cut_endpoint_demand(line(-12., 0., -.5 * tolerance, 0.),
        solid_area, width, tolerance).empty());
    CHECK(BAC::solid_cut_endpoint_demand(line(-12., 0., -2. * tolerance, 0.),
        solid_area, width, tolerance).empty());
}

TEST_CASE("Bridge aware cubic interface closures have one physical owner and matching preview",
          "[BridgeAwareCubic][Interfaces][Emission]")
{
    InterfaceFixture fixture;
    BAC::LayerClosure closure;
    closure.z = fixture.layer->print_z;
    closure.paths = {line(0., -4., 0., -1.), line(0., 1., 0., 4.)};
    closure.reservation = union_ex(offset(closure.paths, scale_(.5)));
    closure.emitting_region = 1;
    fixture.install(0, {closure}, {});

    const Polylines preview = BAC::layer_closure_lines(*fixture.layer);
    REQUIRE(interface_path_length(preview) == Approx(6.));
    BAC::apply_layer_paths(*fixture.layer);
    REQUIRE(paths(*fixture.layer->get_region(0)).empty());
    const auto emitted = paths(*fixture.layer->get_region(1));
    REQUIRE(emitted.size() == 2);
    Polylines output;
    Polygons swept;
    for (const ExtrusionPath *path : emitted) {
        CHECK(path->role() == erInternalInfill);
        CHECK(path->width == Approx(fixture.layer->get_region(1)->flow(frInfill).width()));
        CHECK(path->height == Approx(fixture.layer->height));
        output.push_back(path->polyline);
        path->polygons_covered_by_width(swept, 0.f);
    }
    CHECK(interface_path_length(output) == Approx(6.));
    REQUIRE(output.size() == preview.size());
    for (size_t i = 0; i < output.size(); ++i)
        CHECK(output[i].points == preview[i].points);
    // Clipper coordinates are 0.00001 mm here. A 0.000001 mm buffer rounds to
    // zero; use two coordinate units and round caps for the coverage check.
    // Exact point equality above independently rules out any hidden movement.
    const float coordinate_tolerance = float(scale_(2. * SCALING_FACTOR));
    CHECK(diff_pl(output, offset(preview, coordinate_tolerance,
        ClipperLib::jtRound, 0., ClipperLib::etOpenRound)).empty());
    CHECK(diff_pl(preview, offset(output, coordinate_tolerance,
        ClipperLib::jtRound, 0., ClipperLib::etOpenRound)).empty());
    CHECK(diff_ex(swept, fixture.allowed).empty());
    CHECK(intersection_ex(swept, ExPolygons{fixture.cavity}).empty());

    // Reapplying the hook must not double-extrude an already emitted closure.
    BAC::apply_layer_paths(*fixture.layer);
    CHECK(paths(*fixture.layer->get_region(1)).size() == 2);
    CHECK(interface_path_length(BAC::layer_closure_lines(*fixture.layer)) == Approx(6.));
    fixture.clear_fills();
    BAC::apply_layer_paths(*fixture.layer);
    CHECK(paths(*fixture.layer->get_region(1)).size() == 2);
}

TEST_CASE("Bridge aware cubic closure payload cannot deposit through an intentional cavity",
          "[BridgeAwareCubic][Interfaces][Cavity]")
{
    InterfaceFixture fixture;
    BAC::LayerClosure closure;
    closure.z = fixture.layer->print_z;
    closure.paths = {line(0., -2., 0., 2.)};
    closure.reservation = union_ex(offset(closure.paths, scale_(.5)));
    closure.emitting_region = 1;
    fixture.install(0, {closure}, {});
    REQUIRE_THROWS(BAC::apply_layer_paths(*fixture.layer));
}

TEST_CASE("Bridge aware closure beads exclude nonparticipating and incompatible neighboring material",
          "[BridgeAwareCubic][Interfaces][Emission][ClosureEligibility]")
{
    InterfaceFixture fixture;
    BAC::LayerClosure closure;
    closure.z = fixture.layer->print_z;
    closure.emitting_region = 0;
    closure.paths = {line(0., 1., 0., 4.)};
    closure.reservation = union_ex(offset(closure.paths, scale_(.5)));
    fixture.install(0, {closure}, {});
    PrintRegion &neighbor = const_cast<PrintRegion &>(fixture.layer->get_region(1)->region());
    const PrintRegionConfig original = neighbor.config();
    const std::vector<std::pair<std::string, std::string>> incompatible{
        {"sparse_infill_pattern", "cubic"}, {"sparse_infill_density", "0%"},
        {"sparse_infill_density", "100%"}, {"sparse_infill_line_width", "0.8"},
        {"sparse_infill_filament", "2"}};
    const Flow emitter_flow = fixture.layer->get_region(0)->flow(frInfill);
    const ExPolygons bead = union_ex(offset(closure.paths, scale_(.5 * emitter_flow.width())));
    REQUIRE(diff_ex(bead, fixture.allowed).empty());
    REQUIRE_FALSE(intersection_ex(bead, ExPolygons{box(.01, 1., 1., 4.)}).empty());
    for (const auto &change : incompatible) {
        INFO(change.first << '=' << change.second);
        PrintRegionConfig modified = original;
        modified.set_deserialize_strict(change.first, change.second);
        neighbor.set_config(std::move(modified));
        fixture.clear_fills();
        // The centerline belongs to the emitter's shared boundary. Its real
        // bead still cannot consume forbidden material on the other side.
        CHECK_THROWS_AS(BAC::apply_layer_paths(*fixture.layer), SlicingError);
        CHECK(paths(*fixture.layer->get_region(0)).empty());
        CHECK(paths(*fixture.layer->get_region(1)).empty());
    }
}

TEST_CASE("Bridge aware closure payload requires actual material owned by its saved emitter",
          "[BridgeAwareCubic][Interfaces][Emission][ClosureEligibility][OwnerPresence]")
{
    const bool ordinary_base = GENERATE(false, true);
    CAPTURE(ordinary_base);
    InterfaceFixture fixture;
    LayerRegion &absent = *fixture.layer->get_region(0);
    LayerRegion &present = *fixture.layer->get_region(1);
    absent.slices.clear();
    absent.fill_surfaces.clear();
    present.slices.clear();
    present.slices.append(fixture.allowed, stInternal);
    present.fill_surfaces.clear();
    present.fill_surfaces.append(fixture.allowed, stInternal);
    if (ordinary_base) {
        PrintRegion &region = const_cast<PrintRegion &>(present.region());
        PrintRegionConfig modified = region.config();
        modified.set_deserialize_strict("sparse_infill_pattern", "cubic");
        region.set_config(std::move(modified));
    }
    BAC::LayerClosure closure;
    closure.z = fixture.layer->print_z;
    closure.emitting_region = 0;
    closure.paths = {line(2., 1., 2., 4.)};
    closure.reservation = union_ex(offset(closure.paths, scale_(.5)));
    fixture.install(0, {closure}, {});
    const ExPolygons bead = union_ex(offset(closure.paths, scale_(.5 * absent.flow(frInfill).width())));
    REQUIRE(diff_ex(bead, fixture.allowed).empty());
    REQUIRE(absent.slices.empty());
    REQUIRE_FALSE(present.slices.empty());
    // A descending course cannot retain an emitter that ended above this
    // layer, even when another compatible BAC region owns the whole section.
    REQUIRE_THROWS_AS(BAC::apply_layer_paths(*fixture.layer), SlicingError);
    CHECK(paths(absent).empty());
    CHECK(paths(present).empty());
}

TEST_CASE("Bridge aware closures retain compatible policy seams and same-owner solid boundaries",
          "[BridgeAwareCubic][Interfaces][Emission][ClosureEligibility]")
{
    InterfaceFixture fixture;
    BAC::LayerClosure closure;
    closure.z = fixture.layer->print_z;
    closure.emitting_region = 0;
    SECTION("Different participating density angle and gap retain one shared-boundary owner") {
        PrintRegion &neighbor = const_cast<PrintRegion &>(fixture.layer->get_region(1)->region());
        PrintRegionConfig modified = neighbor.config();
        modified.set_deserialize_strict("sparse_infill_density", "17%");
        modified.set_deserialize_strict("infill_direction", "23");
        modified.set_deserialize_strict("bridge_aware_cubic_max_gap", "3");
        neighbor.set_config(std::move(modified));
        const Flow owner_flow = fixture.layer->get_region(0)->flow(frInfill);
        const Flow neighbor_flow = fixture.layer->get_region(1)->flow(frInfill);
        REQUIRE(owner_flow.width() == neighbor_flow.width());
        REQUIRE(owner_flow.height() == neighbor_flow.height());
        REQUIRE(owner_flow.mm3_per_mm() == neighbor_flow.mm3_per_mm());
        closure.paths = {line(0., 1., 0., 4.)};
    }
    SECTION("One participating emitter may close its sparse to solid boundary") {
        LayerRegion &emitter = *fixture.layer->get_region(0);
        emitter.fill_surfaces.clear();
        emitter.fill_surfaces.append(intersection_ex(fixture.allowed, ExPolygons{box(-5., -5., 0., 2.)}), stInternal);
        emitter.fill_surfaces.append(intersection_ex(fixture.allowed, ExPolygons{box(-5., 2., 0., 5.)}), stInternalSolid);
        closure.paths = {line(-2., 1., -2., 4.)};
    }
    closure.reservation = union_ex(offset(closure.paths, scale_(.5)));
    // Storage in two region plans still has one explicit physical emitter.
    fixture.install(0, {closure}, {});
    fixture.install(1, {closure}, {});
    REQUIRE_NOTHROW(BAC::apply_layer_paths(*fixture.layer));
    REQUIRE_NOTHROW(BAC::apply_layer_paths(*fixture.layer));
    const auto emitted = paths(*fixture.layer->get_region(0));
    REQUIRE(emitted.size() == 1);
    CHECK(paths(*fixture.layer->get_region(1)).empty());
    CHECK(emitted.front()->polyline.points == closure.paths.front().points);
    CHECK(emitted.front()->role() == erInternalInfill);
    Polygons footprint;
    emitted.front()->polygons_covered_by_width(footprint, 0.f);
    CHECK(diff_ex(footprint, fixture.allowed).empty());
}

TEST_CASE("Bridge aware cubic attachment replay matches region geometry width and height",
          "[BridgeAwareCubic][Interfaces][AttachmentReplay]")
{
    InterfaceFixture fixture;
    const Polyline original = line(1., 2., 3., 2.);
    const Polyline replacement = line(.7, 2., 3.3, 2.);
    const Polyline different = line(1., 2.3, 3., 2.3);
    Polyline reversed = original;
    reversed.reverse();
    constexpr float width = .42f;
    constexpr float height = .2f;
    BAC::LayerAttachment attachment;
    attachment.z = fixture.layer->print_z;
    attachment.emitting_region = 1;
    attachment.original = original;
    attachment.replacement = replacement;
    attachment.width = width;
    attachment.height = height;
    fixture.install(0, {}, {attachment});

    const auto seed = [&] {
        fixture.clear_fills();
        fixture.append(1, original, width, height);
        fixture.append(1, original, .56f, height);
        fixture.append(1, original, width, .3f);
        fixture.append(1, different, width, height);
        fixture.append(1, reversed, width, height);
        fixture.append(0, original, width, height);
    };
    const auto check = [&] {
        const auto actual = paths(*fixture.layer->get_region(1));
        REQUIRE(actual.size() == 5);
        CHECK(actual[0]->polyline.points == replacement.points);
        CHECK(actual[0]->width == width);
        CHECK(actual[0]->height == height);
        CHECK(actual[1]->polyline.points == original.points);
        CHECK(actual[2]->polyline.points == original.points);
        CHECK(actual[3]->polyline.points == different.points);
        CHECK(actual[4]->polyline.points == reversed.points);
        const auto other_region = paths(*fixture.layer->get_region(0));
        REQUIRE(other_region.size() == 1);
        CHECK(other_region.front()->polyline.points == original.points);
    };
    seed();
    BAC::apply_layer_paths(*fixture.layer);
    check();
    BAC::apply_layer_paths(*fixture.layer);
    check();
    seed();
    BAC::apply_layer_paths(*fixture.layer);
    check();
}

TEST_CASE("Bridge aware cubic removes a single-ended construction attachment with its exact owner",
          "[BridgeAwareCubic][Interfaces][AttachmentReplay][OwnerRemoval]")
{
    InterfaceFixture fixture;
    constexpr size_t region_id = 1;
    BAC::Plan *plan = const_cast<BAC::Plan *>(fixture.object->bridge_aware_cubic_plan(region_id));
    REQUIRE(plan != nullptr);
    const double z = fixture.layer->print_z;
    const ExPolygon interior = box(1., 1., 4., 4.);
    const BAC::Demand demand{{interior}, z, 2.};
    BAC::Plan demanded(plan->parameters(), {demand});
    REQUIRE_FALSE(demanded.active_demands().empty());
    REQUIRE_FALSE(demanded.refinements().empty());

    // Select an actual partition emitted by an active refinement at this
    // plane. The attachment's owner is therefore its real construction
    // producer, not merely a surviving cell elsewhere in the hierarchy.
    BAC::CellId owner;
    Polyline original;
    for (const BAC::Refinement &refinement : demanded.refinements()) {
        if (z < refinement.bottom_z - EPSILON || z > refinement.top_z + EPSILON) continue;
        for (const Polyline &partition : demanded.partition_lines(refinement.id, z, interior))
            if (partition.size() >= 2 && partition.length() > original.length()) {
                owner = refinement.id;
                original = partition;
            }
    }
    REQUIRE(original.size() >= 2);
    REQUIRE(unscale<double>(original.length()) > .5);
    const Vec2d direction = (original.last_point() - original.points[original.size() - 2]).cast<double>().normalized();
    Polyline replacement = original;
    replacement.points.push_back(Point::new_scale(Vec2d(unscale(original.last_point()) + .2 * direction)));
    REQUIRE(replacement.first_point() == original.first_point());
    REQUIRE(replacement.last_point() != original.last_point());

    const Flow flow = fixture.layer->get_region(int(region_id))->flow(frInfill);
    const float width = float(flow.width()), height = float(flow.height());
    const ExPolygons replacement_bead = union_ex(offset(Polylines{replacement}, float(scale_(width / 2.)),
        ClipperLib::jtRound, 0., ClipperLib::etOpenRound));
    REQUIRE(diff_ex(replacement_bead, fixture.allowed).empty());
    REQUIRE(diff_ex(replacement_bead, ExPolygons{box(0., -5., 5., 5.)}).empty());
    BAC::LayerAttachment attachment;
    attachment.z = z;
    attachment.emitting_region = region_id;
    attachment.original = original;
    attachment.replacement = replacement;
    attachment.width = width;
    attachment.height = height;
    attachment.owners = {owner};
    *plan = demanded.with_payloads({}, {attachment});
    REQUIRE(plan->attachments(z).size() == 1);
    REQUIRE(std::any_of(plan->refinements().begin(), plan->refinements().end(),
        [&](const BAC::Refinement &refinement) { return refinement.id == owner; }));

    const auto seed_original = [&] {
        fixture.clear_fills();
        auto *path = new ExtrusionPath(erInternalInfill, flow.mm3_per_mm(), width, height);
        path->polyline = original;
        fixture.layer->get_region(int(region_id))->fills.entities.push_back(path);
    };
    seed_original();
    BAC::apply_layer_paths(*fixture.layer);
    {
        const auto actual = paths(*fixture.layer->get_region(int(region_id)));
        REQUIRE(actual.size() == 1);
        CHECK(actual.front()->polyline.points == replacement.points);
        CHECK(actual.front()->role() == erInternalInfill);
        CHECK(actual.front()->width == width);
        CHECK(actual.front()->height == height);
    }

    *plan = plan->collapsed(owner);
    REQUIRE(std::none_of(plan->refinements().begin(), plan->refinements().end(),
        [&](const BAC::Refinement &refinement) { return refinement.id == owner; }));
    REQUIRE(plan->attachments(z).empty());
    seed_original();
    BAC::apply_layer_paths(*fixture.layer);
    const auto actual = paths(*fixture.layer->get_region(int(region_id)));
    REQUIRE(actual.size() == 1);
    CHECK(actual.front()->polyline.points == original.points);
    CHECK(actual.front()->role() == erInternalInfill);
    CHECK(actual.front()->width == width);
    CHECK(actual.front()->height == height);
}

TEST_CASE("Bridge aware partition provenance includes integer butt-cap clipping roundoff",
          "[BridgeAwareCubic][Interfaces][PartitionProvenance][BooleanRoundoff]")
{
    const Polylines owned{Polyline(Point(516040, 1328481), Point(515152, 1331795))};
    const Points original_points = owned.front().points;
    const ExPolygons failed{ExPolygon(Polygon(Points{
        Point(517168, 1328783), Point(528409, 1331795), Point(515179, 1331795),
        Point(514892, 1331725), Point(495834, 1326619), Point(496591, 1323795)}))};
    const double width = double(float(.4));
    constexpr double tolerance = .00216;
    const ExPolygons material = union_ex(offset(owned, scale_(.5 * width)));
    const ExPolygons radius_only = union_ex(offset(owned, scale_(.5 * width + tolerance)));
    REQUIRE_FALSE(material.empty());
    REQUIRE_FALSE(diff_ex(failed, radius_only).empty());

    const ExPolygons provenance = BAC::partition_provenance_footprint(owned, width, tolerance);
    REQUIRE_FALSE(provenance.empty());
    CHECK(diff_ex(failed, provenance).empty());
    // Ownership tolerates Boolean roundoff, but this larger envelope must
    // never replace the actual deposited bead or move its source path.
    CHECK_FALSE(diff_ex(provenance, material).empty());
    CHECK(owned.front().points == original_points);
    const ExPolygons unchanged_material = union_ex(offset(owned, scale_(.5 * width)));
    CHECK(diff_ex(material, unchanged_material).empty());
    CHECK(diff_ex(unchanged_material, material).empty());
}

TEST_CASE("Bridge aware serialized demand uses every source vertex and a local integer displacement bound",
          "[BridgeAwareCubic][Interfaces][Serialization][DemandSelection]")
{
    const ExPolygons source{ExPolygon(Polygon(Points{
        Point(400020, 25000), Point(400040, 25000),
        Point(400040, 175000), Point(400020, 175000)}))};
    const Points original_contour = source.front().contour.points;
    const Polyline path(Points{Point(400030, -25000), Point(400049, 12349), Point(400030, 225000)});
    BAC::SerializedXY phase;
    phase.shift = unscale(Point(1200000, -500000));
    const Point origin = phase(Point(0, 0));
    REQUIRE(phase(path.points[1]) - origin - path.points[1] == Point(-49, -49));
    REQUIRE(phase(path.first_point()) - origin - path.first_point() == Point(-30, 0));

    const ExPolygons selected = BAC::serialized_demand(source, {path}, phase);
    REQUIRE_FALSE(selected.empty());
    CHECK(diff_ex(source, selected).empty());
    CHECK(source.front().contour.points == original_contour);
    BoundingBox bounds;
    for (const ExPolygon &polygon : selected) bounds.merge(polygon.contour.points);
    // The interior vertex requires 49+49 grid units; Boolean operations add
    // two more. Endpoint-only or Euclidean bounds produce a smaller result.
    CHECK(bounds.min == Point(399920, 24900));
    CHECK(bounds.max == Point(400140, 175100));
}

TEST_CASE("Bridge aware serialized thin ownership still selects a large physical void",
          "[BridgeAwareCubic][Interfaces][Serialization][DemandSelection][TopologyDomain]")
{
    const ExPolygons support = union_ex(ExPolygons{
        box(-.5, -.5, 18.5, 0.), box(-.5, 2., 18.5, 2.5),
        box(-.5, 0., 0., 2.), box(18., 0., 18.5, 2.)});
    const ExPolygons source{ExPolygon(Polygon(Points{
        Point(400020, 25000), Point(400040, 25000),
        Point(400040, 175000), Point(400020, 175000)}))};
    const Polyline original(Point(400030, -25000), Point(400030, 225000));
    const BAC::SerializedXY phase;
    const Polyline verified = phase.emitted_polyline(original);
    REQUIRE(verified.first_point().x() == 400000);

    const ExPolygons old_selection = union_ex(phase(source));
    REQUIRE(old_selection.empty());
    const ExPolygons selected = BAC::serialized_demand(source, {original}, phase);
    REQUIRE_FALSE(selected.empty());
    REQUIRE_FALSE(intersection_pl(Polylines{verified}, to_polygons(selected)).empty());
    const ExPolygons domain = BAC::support_topology_domain(support, selected);
    REQUIRE(diff_ex(support, domain).empty());
    REQUIRE(diff_ex(selected, domain).empty());

    constexpr double gap = 5., tolerance = 2. * SCALING_FACTOR;
    const auto lost = Validation::validate(domain, support, {verified}, old_selection, gap, tolerance);
    REQUIRE(lost.input_valid);
    REQUIRE(lost.completed);
    CHECK(lost.passed());
    const auto path_only = Validation::validate({}, support, {verified}, selected, gap, tolerance);
    REQUIRE(path_only.input_valid);
    REQUIRE(path_only.completed);
    REQUIRE(path_only.unsupported_runs.size() == 1);
    CHECK(path_only.unsupported_runs.front().kind == Validation::UnsupportedRunKind::TwoEndedBridge);
    CHECK(path_only.max_unsupported_mm == Approx(2.).margin(2. * SCALING_FACTOR));
    CHECK(path_only.passed());

    const auto physical_cell = Validation::validate(domain, support, {verified}, selected, gap, tolerance);
    REQUIRE(physical_cell.input_valid);
    REQUIRE(physical_cell.completed);
    REQUIRE(physical_cell.void_cells.size() == 1);
    CHECK(physical_cell.void_cells.front().closed);
    CHECK(physical_cell.void_cells.front().diameter_mm > 18.);
    CHECK(physical_cell.void_cells.front().exceeds_gap);
    CHECK_FALSE(physical_cell.passed());
}

TEST_CASE("Bridge aware topology window leaves the exterior void open",
          "[BridgeAwareCubic][Interfaces][TopologyDomain]")
{
    // Two real end anchors suffice for this short path, but neither side of
    // the surrounding air cell has deposited material to close it.
    const ExPolygons support{box(-.5, -.5, 18.5, 0.), box(-.5, 2., 18.5, 2.5)};
    const Polyline path = line(4., -.25, 4., 2.25);
    const ExPolygons selected{box(3.9, .25, 4.1, 1.75)};
    const ExPolygons domain = BAC::support_topology_domain(support, selected);
    REQUIRE(domain.size() == 1);
    BoundingBox bounds(domain.front().contour.points);
    CHECK(unscale<double>(bounds.min.x()) < -1.49);
    CHECK(unscale<double>(bounds.max.x()) > 19.49);
    const auto result = Validation::validate(domain, support, {path}, selected, 50., 2. * SCALING_FACTOR);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::TwoEndedBridge);
    CHECK_FALSE(result.unsupported_runs.front().exceeds_gap);
    REQUIRE(result.void_cells.size() == 1);
    CHECK_FALSE(result.void_cells.front().closed);
    CHECK_FALSE(result.void_cells.front().exceeds_gap);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware convergent attachments preserve every applicable whole-path obligation",
          "[BridgeAwareCubic][Interfaces][AttachmentReplay][DemandSelection]")
{
    const Polyline final_path(Points{Point::new_scale(0., 0.), Point::new_scale(1., 0.),
                                    Point::new_scale(9., 0.), Point::new_scale(10., 0.)});
    const ExPolygons support{box(-1., -.5, 2., .5), box(7., -.5, 11., .5)};
    const ExPolygons full_selection = union_ex(offset(Polylines{final_path}, scale_(.2)));
    BAC::LayerAttachment whole;
    whole.original = Polyline(Points{final_path.points[0], final_path.points[1], final_path.points[2]});
    whole.replacement = final_path;
    whole.width = .4;
    whole.height = .2;
    BAC::LayerAttachment cut = whole;
    cut.original = Polyline(Points{final_path.points[1], final_path.points[2], final_path.points[3]});
    cut.solid_cut = true;
    cut.demand_selection = {box(8.5, -.2, 10.1, .2)};

    const auto validate_selection = [&](const std::optional<ExPolygons> &selection) {
        return Validation::validate({}, support, {final_path}, selection ? *selection : full_selection,
                                    2., 2. * SCALING_FACTOR);
    };
    const auto require_whole_gap = [&](const std::optional<ExPolygons> &selection) {
        REQUIRE_FALSE(selection.has_value());
        const auto result = validate_selection(selection);
        REQUIRE(result.input_valid);
        REQUIRE(result.completed);
        REQUIRE(result.unsupported_runs.size() == 1);
        CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::TwoEndedBridge);
        CHECK(result.max_unsupported_mm == Approx(5.).margin(2. * SCALING_FACTOR));
        CHECK(result.unsupported_runs.front().exceeds_gap);
        CHECK_FALSE(result.passed());
    };

    SECTION("Either order of convergent records preserves the whole obligation") {
        require_whole_gap(BAC::merged_attachment_selection(false, {whole, cut}, {0, 1}));
        require_whole_gap(BAC::merged_attachment_selection(false, {cut, whole}, {0, 1}));
    }
    SECTION("Ordinary full coverage overrides a cut-only producer") {
        require_whole_gap(BAC::merged_attachment_selection(true, {cut}, {0}));
    }
    SECTION("Incomplete ordinary coverage retains just the applicable cut mask") {
        const auto selected = BAC::merged_attachment_selection(false, {cut}, {0});
        REQUIRE(selected.has_value());
        CHECK(diff_ex(*selected, cut.demand_selection).empty());
        CHECK(diff_ex(cut.demand_selection, *selected).empty());
        const auto result = validate_selection(selected);
        REQUIRE(result.input_valid);
        REQUIRE(result.completed);
        CHECK(result.unsupported_runs.empty());
        CHECK(result.passed());
    }
    SECTION("An unrelated whole-path producer does not broaden the selected lineage") {
        for (bool reverse : {false, true}) {
            const std::vector<BAC::LayerAttachment> records = reverse ?
                std::vector<BAC::LayerAttachment>{cut, whole} : std::vector<BAC::LayerAttachment>{whole, cut};
            const auto selected = BAC::merged_attachment_selection(false, records, {reverse ? 0u : 1u});
            REQUIRE(selected.has_value());
            CHECK(diff_ex(*selected, cut.demand_selection).empty());
            CHECK(diff_ex(cut.demand_selection, *selected).empty());
            const auto result = validate_selection(selected);
            REQUIRE(result.input_valid);
            REQUIRE(result.completed);
            CHECK(result.passed());
        }
    }
    SECTION("Two restricted producers union both terminal masks without selecting the central gap") {
        BAC::LayerAttachment left_cut = cut;
        left_cut.original = whole.original;
        left_cut.demand_selection = {box(-.1, -.2, 1.5, .2)};
        const auto selected = BAC::merged_attachment_selection(false, {left_cut, cut}, {0, 1});
        REQUIRE(selected.has_value());
        ExPolygons expected = left_cut.demand_selection;
        append(expected, cut.demand_selection);
        expected = union_ex(expected);
        CHECK(diff_ex(*selected, expected).empty());
        CHECK(diff_ex(expected, *selected).empty());
        CHECK(intersection_ex(*selected, ExPolygons{box(4., -.2, 5., .2)}).empty());
        const auto result = validate_selection(selected);
        REQUIRE(result.input_valid);
        REQUIRE(result.completed);
        CHECK(result.unsupported_runs.empty());
        CHECK(result.passed());
    }
    SECTION("An empty cut mask is the explicit whole-path sentinel") {
        cut.demand_selection.clear();
        require_whole_gap(BAC::merged_attachment_selection(false, {cut}, {0}));
    }
    SECTION("An ordinary producer remains whole even if it stores a mask") {
        whole.demand_selection = cut.demand_selection;
        require_whole_gap(BAC::merged_attachment_selection(false, {whole}, {0}));
    }
    SECTION("No applicable attachment leaves the ordinary whole-path obligation") {
        require_whole_gap(BAC::merged_attachment_selection(false, {cut}, {}));
    }
}

TEST_CASE("Bridge aware compatible modifier material permits one owned endpoint attachment",
          "[BridgeAwareCubic][Interfaces][AttachmentDomain][AttachmentReplay][Contact]")
{
    InterfaceFixture fixture;
    constexpr size_t emitter = 0;
    const ExPolygons domain = BAC::endpoint_attachment_domain(*fixture.layer, emitter);
    REQUIRE(diff_ex(fixture.allowed, domain).empty());
    REQUIRE(diff_ex(domain, fixture.allowed).empty());
    const Flow flow = fixture.layer->get_region(int(emitter))->flow(frSolidInfill);
    ExtrusionPath original(erSolidInfill, flow.mm3_per_mm(), float(flow.width()), float(flow.height()));
    original.polyline = Polyline(Points{Point::new_scale(-2., 2.), Point::new_scale(-1., 2.),
                                       Point::new_scale(-.025, 2.)});
    const Points original_points = original.polyline.points;
    ExtrusionPath left = original, right = original;
    left.polyline = line(-3., 2., -.15, 2.);
    right.polyline = line(.15, 2., 3., 2.);
    const double upper_z = fixture.layer->print_z, lower_z = upper_z - fixture.layer->height;
    const std::vector<Contact::Bead> prior{{&left, lower_z}, {&right, lower_z}};
    const ExPolygons original_demand = union_ex(offset(Polylines{original.polyline}, scale_(.5 * original.width)));
    const auto initial_contact = Contact::footprint(prior, original, upper_z, 0.);
    REQUIRE(initial_contact.input_valid);
    REQUIRE(initial_contact.completed);
    const auto initial = Validation::validate_contacts({}, initial_contact.physical_footprint,
        {original.polyline}, original_demand, 1., 0., {initial_contact});
    REQUIRE(initial.input_valid);
    REQUIRE(initial.completed);
    REQUIRE(initial.unsupported_runs.size() == 1);
    REQUIRE(initial.unsupported_runs.front().kind == Validation::UnsupportedRunKind::OneSidedOverhang);
    REQUIRE_FALSE(initial.passed());

    // Query real prior material along the terminal tangent beyond the original
    // endpoint. The probe supplies search contacts, never deposited support.
    ExtrusionPath probe = original;
    probe.polyline = line(-2., 2., 1., 2.);
    const auto search_contact = Contact::footprint(prior, probe, upper_z, 0.);
    REQUIRE(search_contact.input_valid);
    REQUIRE(search_contact.completed);
    ExPolygons support;
    for (const auto &segment : search_contact.segments) append(support, segment.centerline_support);
    support = union_ex(support);
    const auto attached = Attachment::attach_endpoints(original.polyline, support, domain,
        original.width, 1., 0., .01);
    REQUIRE(attached.has_value());
    REQUIRE(attached->size() == original.polyline.size() + 1);
    CHECK(std::equal(original_points.begin(), original_points.end(), attached->points.begin()));
    CHECK(attached->first_point() == original.polyline.first_point());
    CHECK(attached->last_point().x() > scale_(.15));
    CHECK(attached->last_point().y() == original.polyline.last_point().y());
    CHECK(original.polyline.points == original_points);
    const ExPolygons added_bead = diff_ex(union_ex(offset(Polylines{*attached}, scale_(.5 * original.width))),
                                        original_demand);
    REQUIRE_FALSE(added_bead.empty());
    CHECK(diff_ex(added_bead, domain).empty());
    CHECK_FALSE(intersection_ex(added_bead, ExPolygons{box(0., 1., 5., 3.)}).empty());
    CHECK_FALSE(Attachment::attach_endpoints(original.polyline, support,
        intersection_ex(domain, ExPolygons{box(-5., -5., 0., 5.)}), original.width, 1., 0., .01));

    BAC::LayerAttachment attachment;
    attachment.z = upper_z;
    attachment.emitting_region = emitter;
    attachment.original = original.polyline;
    attachment.replacement = *attached;
    attachment.width = original.width;
    attachment.height = original.height;
    // Store in the adjacent plan to prove payload storage does not transfer
    // emission ownership across the compatible modifier seam.
    fixture.install(1, {}, {attachment});
    fixture.layer->get_region(int(emitter))->fills.entities.push_back(new ExtrusionPath(original));
    BAC::apply_layer_paths(*fixture.layer);
    BAC::apply_layer_paths(*fixture.layer);
    const auto emitted = paths(*fixture.layer->get_region(int(emitter)));
    REQUIRE(emitted.size() == 1);
    REQUIRE(paths(*fixture.layer->get_region(1)).empty());
    CHECK(emitted.front()->polyline.points == attached->points);
    CHECK(emitted.front()->width == original.width);
    CHECK(emitted.front()->height == original.height);
    CHECK(emitted.front()->role() == original.role());
    CHECK(emitted.front()->mm3_per_mm == original.mm3_per_mm);

    for (double displacement : {-.0004, 0., .0004}) {
        BAC::SerializedXY phase;
        phase.shift = Vec2d(80. + displacement, 80. - displacement);
        ExtrusionPath emitted_left = left, emitted_right = right, emitted_upper = *emitted.front();
        emitted_left.polyline = phase.emitted_polyline(left.polyline);
        emitted_right.polyline = phase.emitted_polyline(right.polyline);
        emitted_upper.polyline = phase.emitted_polyline(emitted_upper.polyline);
        const auto contact = Contact::footprint({{&emitted_left, lower_z}, {&emitted_right, lower_z}},
                                                emitted_upper, upper_z, 0.);
        REQUIRE(contact.input_valid);
        REQUIRE(contact.completed);
        const ExPolygons demand = union_ex(offset(Polylines{emitted_upper.polyline}, scale_(.5 * emitted_upper.width)));
        const auto result = Validation::validate_contacts({}, contact.physical_footprint,
            {emitted_upper.polyline}, demand, 1., 0., {contact});
        REQUIRE(result.input_valid);
        REQUIRE(result.completed);
        REQUIRE(result.unsupported_runs.size() == 1);
        CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::TwoEndedBridge);
        CHECK(result.passed());
    }
}

TEST_CASE("Bridge aware endpoint domains exclude incompatible modifiers and actual cavities",
          "[BridgeAwareCubic][Interfaces][AttachmentDomain]")
{
    InterfaceFixture fixture;
    PrintRegion &neighbor = const_cast<PrintRegion &>(fixture.layer->get_region(1)->region());
    const PrintRegionConfig original_config = neighbor.config();
    const ExPolygons emitter_only = intersection_ex(fixture.allowed, ExPolygons{box(-5., -5., 0., 5.)});
    const std::vector<std::pair<std::string, std::string>> incompatible{
        {"sparse_infill_density", "0%"}, {"sparse_infill_density", "100%"},
        {"sparse_infill_pattern", "cubic"},
        {"wall_filament", "2"}, {"solid_infill_filament", "2"}, {"sparse_infill_filament", "2"},
        {"outer_wall_line_width", "0.8"}, {"inner_wall_line_width", "0.8"},
        {"sparse_infill_line_width", "0.8"}, {"skin_infill_line_width", "0.8"},
        {"skeleton_infill_line_width", "0.8"}, {"internal_solid_infill_line_width", "0.8"},
        {"top_surface_line_width", "0.8"}, {"bridge_flow", "0.8"}};
    for (const auto &change : incompatible) {
        INFO(change.first << "=" << change.second);
        PrintRegionConfig modified = original_config;
        modified.set_deserialize_strict(change.first, change.second);
        neighbor.set_config(std::move(modified));
        const ExPolygons domain = BAC::endpoint_attachment_domain(*fixture.layer, 0);
        CHECK(diff_ex(emitter_only, domain).empty());
        CHECK(diff_ex(domain, emitter_only).empty());
        // The emitter's own material remains usable even when its neighbor
        // fails the participation or material/flow compatibility checks.
        CHECK_FALSE(intersection_ex(domain, ExPolygons{box(-4., 1., -1., 3.)}).empty());
        CHECK(intersection_ex(domain, ExPolygons{box(1., 1., 4., 3.)}).empty());
    }
    neighbor.set_config(original_config);
    REQUIRE(diff_ex(fixture.allowed, BAC::endpoint_attachment_domain(*fixture.layer, 0)).empty());

    // Region slices deliberately overclaim the cavity. The whole model's
    // actual section must still cut it from the combined attachment domain.
    for (size_t id = 0; id < 2; ++id) {
        LayerRegion *region = fixture.layer->get_region(int(id));
        region->slices.clear();
        region->slices.append(ExPolygons{id == 0 ? box(-5., -5., 0., 5.) : box(0., -5., 5., 5.)}, stInternal);
    }
    const ExPolygons domain = BAC::endpoint_attachment_domain(*fixture.layer, 0);
    CHECK(diff_ex(domain, fixture.allowed).empty());
    CHECK(intersection_ex(domain, ExPolygons{fixture.cavity}).empty());
    const Polyline original = line(-2., 0., -.7, 0.);
    const ExPolygons support{box(-3., -.5, -1., .5), box(.7, -.5, 3., .5)};
    REQUIRE(Attachment::attach_endpoints(original, support, ExPolygons{box(-5., -5., 5., 5.)}, .4, 2., 0.));
    CHECK_FALSE(Attachment::attach_endpoints(original, support, domain, .4, 2., 0.));
}

TEST_CASE("Bridge aware clipped partition provenance retains the actual source axis",
          "[BridgeAwareCubic][Interfaces][PartitionProvenance][SourceAxis]")
{
    const Point start(-59488, -1131795), first_cut(-58929, -1129372),
                second_cut(-21640, -967856), finish(57011, -627181);
    Polylines source{Polyline(start, finish)};
    Polylines owned{Polyline(start, first_cut), Polyline(first_cut, second_cut), Polyline(second_cut, finish)};
    const ExPolygons failed{ExPolygon(Polygon(Points{
        Point(-39221, -1132914), Point(-44063, -1131795),
        Point(-59488, -1131795), Point(-40001, -1136293)}))};
    const double width = double(float(.4));
    constexpr double tolerance = .00216;
    // Rounding the 0.025 mm fragment changes its angle enough to move its
    // finite-width butt corner by over three grid units. A fixed two-unit
    // envelope around that rotated fragment cannot preserve the source bead.
    const ExPolygons rotated = offset_ex(union_ex(offset(Polylines{owned.front()},
        scale_(.5 * width + tolerance))), 2.);
    const Point source_corner(-40001, -1136293);
    // The missed sliver is thinner than one integer coordinate. Testing its
    // exact source vertex avoids a second Boolean operation rounding it away;
    // envelope construction still follows the old per-owner operation order.
    REQUIRE(std::none_of(rotated.begin(), rotated.end(),
        [&](const ExPolygon &polygon) { return polygon.contains(source_corner); }));
    const ExPolygons first_source_axis = BAC::partition_provenance_footprint(
        Polylines{owned.front()}, width, tolerance, &source);
    CHECK(std::any_of(first_source_axis.begin(), first_source_axis.end(),
        [&](const ExPolygon &polygon) { return polygon.contains(source_corner); }));
    const ExPolygons provenance = BAC::partition_provenance_footprint(owned, width, tolerance, &source);
    REQUIRE_FALSE(provenance.empty());
    CHECK(diff_ex(failed, provenance).empty());

    for (Polyline &path : source) path.reverse();
    for (Polyline &path : owned) path.reverse();
    const ExPolygons reversed = BAC::partition_provenance_footprint(owned, width, tolerance, &source);
    CHECK(diff_ex(failed, reversed).empty());

    // The original axis fixes direction only. It must not give a clipped
    // producer ownership of an unrelated interval near the far endpoint.
    const Polylines first_fragment{owned.front()};
    const ExPolygons partial = BAC::partition_provenance_footprint(first_fragment, width, tolerance, &source);
    REQUIRE_FALSE(partial.empty());
    const ExPolygons distant{ExPolygon(Polygon(Points{
        Point(finish.x() - 10, finish.y() - 10), Point(finish.x() + 10, finish.y() - 10),
        Point(finish.x() + 10, finish.y() + 10), Point(finish.x() - 10, finish.y() + 10)}))};
    CHECK(intersection_ex(partial, distant).empty());
    CHECK(source.front().first_point() == finish);
    CHECK(source.front().last_point() == start);
}

TEST_CASE("Bridge aware variable-height solid provenance covers source Boolean roundoff",
          "[BridgeAwareCubic][Interfaces][PartitionProvenance][BooleanRoundoff]")
{
    const Polylines source{Polyline(Point(-912378, -1048085), Point(-911632, -1044853))};
    const ExPolygons failed{ExPolygon(Polygon(Points{
        Point(-892234, -1049741), Point(-913664, -1044384), Point(-931120, -1040355),
        Point(-931789, -1043253), Point(-913446, -1047838), Point(-892890, -1052583)}))};
    const double width = double(float(.4));
    constexpr double tolerance = .00432;
    const ExPolygons radius_only = union_ex(offset(source, scale_(.5 * width + tolerance)));
    REQUIRE_FALSE(diff_ex(failed, radius_only).empty());
    const ExPolygons provenance = BAC::partition_provenance_footprint(source, width, tolerance);
    REQUIRE_FALSE(provenance.empty());
    CHECK(diff_ex(failed, provenance).empty());
    CHECK(source.front().first_point() == Point(-912378, -1048085));
    CHECK(source.front().last_point() == Point(-911632, -1044853));
}

TEST_CASE("Bridge aware two-ended construction replays two independently owned terminal records",
          "[BridgeAwareCubic][Interfaces][AttachmentReplay][OwnerRemoval][AttachmentChain]")
{
    InterfaceFixture fixture;
    constexpr size_t emitter = 1;
    BAC::Plan *plan = const_cast<BAC::Plan *>(fixture.object->bridge_aware_cubic_plan(emitter));
    REQUIRE(plan != nullptr);
    const double z = fixture.layer->print_z;
    const ExPolygon interior = box(1., 1., 4., 4.);
    BAC::Plan demanded(plan->parameters(), {BAC::Demand{{interior}, z, 2.}});
    std::vector<BAC::CellId> candidates;
    for (const BAC::Refinement &refinement : demanded.refinements())
        if (!refinement.solid && z >= refinement.bottom_z && z <= refinement.top_z &&
            !demanded.partition_lines(refinement.id, z, interior).empty())
            candidates.push_back(refinement.id);
    std::optional<std::pair<BAC::CellId, BAC::CellId>> owners;
    for (size_t i = 0; i < candidates.size() && !owners; ++i)
        for (size_t j = i + 1; j < candidates.size(); ++j)
            if (candidates[i].depth == candidates[j].depth && !(candidates[i] == candidates[j])) {
                owners = std::make_pair(candidates[i], candidates[j]);
                break;
            }
    REQUIRE(owners.has_value());
    const BAC::CellId first_owner = owners->first, second_owner = owners->second;
    // Real directly demanded siblings make the collapse trials independent.
    // Controlled horizontal paths isolate replay identity and payload lifetime.
    const Polyline a(Points{Point::new_scale(1.4, 2.), Point::new_scale(2., 2.), Point::new_scale(3.6, 2.)});
    Polyline b = a;
    b.points.insert(b.points.begin(), Point::new_scale(1.2, 2.));
    Polyline c = b;
    c.points.push_back(Point::new_scale(3.8, 2.));
    const Flow flow = fixture.layer->get_region(int(emitter))->flow(frInfill);
    const float width = float(flow.width()), height = float(flow.height());
    REQUIRE(diff_ex(union_ex(offset(Polylines{c}, scale_(.5 * width))), fixture.allowed).empty());
    BAC::LayerAttachment first;
    first.z = z;
    first.emitting_region = emitter;
    first.original = a;
    first.replacement = b;
    first.width = width;
    first.height = height;
    first.owners = {first_owner};
    first.solid_cut = true;
    first.demand_selection = {box(1.1, 1.8, 1.5, 2.2)};
    BAC::LayerAttachment second = first;
    second.original = b;
    second.replacement = c;
    second.owners = {second_owner};
    second.demand_selection = {box(3.5, 1.8, 3.9, 2.2)};
    BAC::Plan baseline = demanded.with_payloads({}, {second, first});
    const auto has_owner = [](const BAC::Plan &value, const BAC::CellId &id) {
        return std::any_of(value.refinements().begin(), value.refinements().end(),
            [&](const BAC::Refinement &refinement) { return refinement.id == id; });
    };
    const auto replay = [&](const Polyline &expected, const std::vector<size_t> &expected_lineage,
                            const std::optional<ExPolygons> &expected_selection) {
        const auto records = plan->attachments(z);
        for (const auto &record : records) {
            REQUIRE(record.owners.size() == 1);
            if (record.original.points == a.points) CHECK(record.owners.front() == first_owner);
            else {
                REQUIRE(record.original.points == b.points);
                CHECK(record.owners.front() == second_owner);
            }
        }
        const auto resolved = Attachment::resolve_chain(a, width, height, emitter, records);
        CHECK(resolved.polyline.points == expected.points);
        CHECK(resolved.applied_indices == expected_lineage);
        const std::set<size_t> lineage(resolved.applied_indices.begin(), resolved.applied_indices.end());
        const auto selected = BAC::merged_attachment_selection(false, records, lineage);
        REQUIRE(selected.has_value() == expected_selection.has_value());
        if (selected) {
            CHECK(diff_ex(*selected, *expected_selection).empty());
            CHECK(diff_ex(*expected_selection, *selected).empty());
        }
        // Each trial starts with A, so a missing first record cannot inherit B
        // or C from a prior successful application.
        fixture.clear_fills();
        auto *path = new ExtrusionPath(erInternalInfill, flow.mm3_per_mm(), width, height);
        path->polyline = a;
        fixture.layer->get_region(int(emitter))->fills.entities.push_back(path);
        BAC::apply_layer_paths(*fixture.layer);
        BAC::apply_layer_paths(*fixture.layer);
        const auto emitted = paths(*fixture.layer->get_region(int(emitter)));
        REQUIRE(emitted.size() == 1);
        CHECK(emitted.front()->polyline.points == expected.points);
        CHECK(emitted.front()->width == width);
        CHECK(emitted.front()->height == height);
        CHECK(emitted.front()->role() == erInternalInfill);
        CHECK(emitted.front()->mm3_per_mm == flow.mm3_per_mm());
        CHECK(paths(*fixture.layer->get_region(0)).empty());
    };

    *plan = baseline.clone();
    REQUIRE(plan->attachments(z).size() == 2);
    REQUIRE(has_owner(*plan, first_owner));
    REQUIRE(has_owner(*plan, second_owner));
    ExPolygons both_masks = first.demand_selection;
    append(both_masks, second.demand_selection);
    replay(c, {1, 0}, union_ex(both_masks));

    *plan = baseline.collapsed(first_owner);
    REQUIRE_FALSE(has_owner(*plan, first_owner));
    REQUIRE(has_owner(*plan, second_owner));
    REQUIRE(plan->attachments(z).size() == 1);
    REQUIRE(plan->attachments(z).front().original.points == b.points);
    replay(a, {}, std::nullopt);

    *plan = baseline.collapsed(second_owner);
    REQUIRE(has_owner(*plan, first_owner));
    REQUIRE_FALSE(has_owner(*plan, second_owner));
    REQUIRE(plan->attachments(z).size() == 1);
    REQUIRE(plan->attachments(z).front().original.points == a.points);
    replay(b, {0}, first.demand_selection);
}

TEST_CASE("Bridge aware solid-cut provenance retains the full selected source bead corner",
          "[BridgeAwareCubic][Interfaces][PartitionProvenance][SolidCut]")
{
    const Polyline source(Point(-1031795, -338416), Point(-551502, -467110));
    const double width = double(float(.4));
    const coord_t radius = scale_(.5 * width + .00216);
    REQUIRE(radius == 20216);
    const Point terminal = source.last_point();
    const ExPolygons selection{ExPolygon(Polygon(Points{
        terminal + Point(-radius, -radius), terminal + Point(radius, -radius),
        terminal + Point(radius, radius), terminal + Point(-radius, radius)}))};
    const ExPolygons failed{ExPolygon(Polygon(Points{
        Point(-547949, -453850), Point(-546326, -447792), Point(-549676, -446894),
        Point(-571718, -446894), Point(-571718, -482398), Point(-556680, -486428)}))};
    const ExPolygons old_produced{ExPolygon(Polygon(Points{
        Point(-384355, -330786), Point(-391537, -303978), Point(-609449, -245590),
        Point(-629073, -265214), Point(-570685, -483122), Point(-543879, -490310)}))};
    REQUIRE_FALSE(diff_ex(failed, old_produced).empty());
    const ExPolygons produced = BAC::solid_cut_provenance_footprint(source, selection, width);
    REQUIRE_FALSE(produced.empty());
    CHECK(diff_ex(failed, produced).empty());
    const ExPolygons material = union_ex(offset(Polylines{source}, scale_(.5 * width)));
    CHECK(diff_ex(produced, offset_ex(material, 2.)).empty());
    CHECK(diff_ex(produced, offset_ex(selection, 2.)).empty());
    const ExPolygons remote_prefix{box(-10.4, -3.6, -9.9, -3.2)};
    REQUIRE_FALSE(intersection_ex(material, remote_prefix).empty());
    CHECK(intersection_ex(produced, remote_prefix).empty());
    CHECK(BAC::solid_cut_provenance_footprint(source, {}, width).empty());

    // A retained cut selection still belongs to the old terminal after an
    // attachment extends the current endpoint. Neither current endpoint is
    // in the selection; recomputing endpoint membership would lose this work.
    Polyline extended = source;
    extended.points.push_back(terminal + Point(48296, -12941));
    for (const Point &endpoint : {extended.first_point(), extended.last_point()})
        REQUIRE(std::none_of(selection.begin(), selection.end(),
            [&](const ExPolygon &polygon) { return polygon.contains(endpoint); }));
    const ExPolygons retained = BAC::solid_cut_provenance_footprint(extended, selection, width);
    REQUIRE_FALSE(retained.empty());
    CHECK(diff_ex(failed, retained).empty());
    CHECK(diff_ex(retained, offset_ex(selection, 2.)).empty());
    CHECK(intersection_ex(retained, remote_prefix).empty());
    CHECK(source.first_point() == Point(-1031795, -338416));
    CHECK(source.last_point() == terminal);

    for (double invalid_width : {0., -.4, std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::quiet_NaN()})
        CHECK_THROWS_AS(BAC::solid_cut_provenance_footprint(source, selection, invalid_width), std::invalid_argument);
}

namespace {
using ClipPathCoordinates = std::vector<std::pair<coord_t, coord_t>>;

std::vector<ClipPathCoordinates> canonical_clip_paths(const Polylines &paths)
{
    std::vector<ClipPathCoordinates> result;
    for (const Polyline &path : paths) {
        ClipPathCoordinates points;
        for (const Point &point : path.points) points.emplace_back(point.x(), point.y());
        ClipPathCoordinates reversed(points.rbegin(), points.rend());
        result.push_back(std::min(points, reversed));
    }
    std::sort(result.begin(), result.end());
    return result;
}
}

TEST_CASE("Bridge aware cubic clip-mask filtering retains containing rings and touching bounds",
          "[BridgeAwareCubic][Interfaces][ClipFilter]")
{
    Polygon hole = box(3., 3., 7., 7.).contour;
    hole.reverse();
    const Polygons mask{box(0., 0., 10., 10.).contour, hole,
                        box(8., 2., 12., 8.).contour,
                        box(20., 20., 30., 30.).contour, Polygon{}};
    const BAC::BoundedClipMask prepared(mask);

    // The containing contour must survive even though none of its vertices
    // is in the path bounds. A containing hole must survive for the same reason.
    const Polygons inside = prepared.candidates(line(1., 1., 2., 1.));
    REQUIRE(inside.size() == 1);
    CHECK(inside.front().points == mask.front().points);
    const Polygons in_hole = prepared.candidates(line(4., 5., 6., 5.));
    REQUIRE(in_hole.size() == 2);
    CHECK(in_hole[1].points == hole.points);
    CHECK(intersection_pl({line(4., 5., 6., 5.)}, in_hole).empty());

    // Zero-height and zero-width subject bounds are real paths. Inclusive
    // overlap also retains a ring touching only at an endpoint or boundary.
    CHECK(prepared.candidates(line(0., 0., 10., 0.)).size() == 1);
    CHECK(prepared.candidates(line(0., 0., 0., 10.)).size() == 1);
    CHECK(prepared.candidates(line(-1., 0., 0., 0.)).size() == 1);
    CHECK(prepared.candidates(line(-2., -2., -1., -1.)).empty());
    CHECK(prepared.candidates(Polyline{}).empty());
    CHECK(prepared.candidates(Polyline(Points{Point::new_scale(-2., -2.),
                                              Point::new_scale(5., 5.),
                                              Point::new_scale(12., -2.)})).size() == 3);

    // Cache lifetime must not depend on the caller retaining the original mask.
    const BAC::BoundedClipMask temporary_mask(Polygons{box(0., 0., 1., 1.).contour});
    CHECK(temporary_mask.candidates(line(.2, .2, .8, .2)).size() == 1);
}

TEST_CASE("Bridge aware cubic filtered clipping matches full masks across holes and crossings",
          "[BridgeAwareCubic][Interfaces][ClipFilter]")
{
    Polylines lattice;
    for (int i = 0; i <= 8; ++i) {
        lattice.push_back(line(double(i), 0., double(i), 8.));
        lattice.push_back(line(0., double(i), 8., double(i)));
    }
    Polygons mask = offset(lattice, scale_(.02));
    // Preserve nonzero winding when an appended patch overlaps the normalized
    // lattice. Remote rings should not enter any local clipping operation.
    mask.push_back(box(3.4, 3.4, 4.6, 4.6).contour);
    for (int i = 0; i < 64; ++i)
        mask.push_back(box(30. + 2. * i, 30., 31. + 2. * i, 31.).contour);
    const BAC::BoundedClipMask prepared(mask);
    REQUIRE(prepared.candidates(line(-1., 4., 9., 4.)).size() < mask.size());

    Polylines subjects{line(-1., -.02, 9., -.02), line(-1., .02, 9., .02),
                      line(-1., -1., 9., 9.), line(2.2, 2.2, 2.8, 2.2),
                      line(3.5, 3.5, 4.5, 3.5), line(20., 20., 21., 21.),
                      Polyline(Points{Point::new_scale(-1., -1.), Point::new_scale(4., 4.),
                                      Point::new_scale(9., -1.)})};
    for (int i = 0; i <= 16; ++i) {
        subjects.push_back(line(-1., .5 * i, 9., .5 * i));
        subjects.push_back(line(.5 * i, -1., .5 * i, 9.));
    }
    for (Polyline subject : subjects) {
        for (bool reversed : {false, true}) {
            if (reversed) subject.reverse();
            const Polylines full = intersection_pl({subject}, mask);
            const Polylines filtered = intersection_pl({subject}, prepared.candidates(subject));
            CHECK(canonical_clip_paths(filtered) == canonical_clip_paths(full));
            CHECK(interface_path_length(filtered) == Approx(interface_path_length(full)));
        }
    }
}
