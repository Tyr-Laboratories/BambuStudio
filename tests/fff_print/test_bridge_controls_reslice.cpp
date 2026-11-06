#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

using namespace Slic3r;

namespace {

DynamicPrintConfig reslice_bridge_config(const std::string &generator)
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "15%"}, {"sparse_infill_pattern", "grid"},
        {"top_shell_layers", "4"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", "0"}, {"wall_generator", generator},
        {"thick_bridges", "1"}, {"bridge_flow", "1"},
        {"bridge_layer_density", "100%"}, {"bridge_line_travel_extend", "15%"},
        {"infill_wall_overlap", "15%"},
        {"internal_bridge_layers", "1"}
    });
    return config;
}

Model reslice_bridge_model(bool internal)
{
    Model model;
    auto *object = model.add_object();
    TriangleMesh mesh(its_make_cube(internal ? 30. : 6., 20., 8.));
    if (!internal) {
        TriangleMesh right(its_make_cube(6., 20., 8.));
        right.translate(24., 0., 0.);
        mesh.merge(right);
        TriangleMesh roof(its_make_cube(30., 20., 1.2));
        roof.translate(0., 0., 8.);
        mesh.merge(roof);
    }
    object->add_volume(mesh);
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

void slice_bridge_model(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    REQUIRE(print.get_object(size_t(0))->config().wall_generator.serialize() ==
            config.option("wall_generator")->serialize());
    print.process();
}

void visit_paths(const ExtrusionEntity &entity, const std::function<void(const ExtrusionPath &)> &visit)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity))
        visit(*path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const auto &path : loop->paths) visit(path);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const auto &path : multi->paths) visit(path);
    else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const auto *child : collection->entities) visit_paths(*child, visit);
    else
        FAIL("Unexpected extrusion entity in bridge re-slicing fixture");
}

// Direction and collection ordering do not change the material deposited.
// Retain each segment's layer, role, cross-section, extrusion flow and whether
// it is a connector that explicitly suppresses extrusion.
using Segment = std::tuple<size_t, int, double, double, double, double, bool, coord_t, coord_t, coord_t, coord_t>;
struct BridgePathSnapshot {
    std::vector<Segment> perimeters, ordinary_fill, bridges;
    double bridge_length = 0.;
};

BridgePathSnapshot snapshot_bridge_paths(const Print &print)
{
    BridgePathSnapshot result;
    for (const auto *layer : print.get_object(size_t(0))->layers()) {
        auto record = [&](const ExtrusionPath &path, bool perimeter) {
            auto &segments = perimeter ? result.perimeters :
                path.role() == erBridgeInfill ? result.bridges : result.ordinary_fill;
            for (const auto &line : path.polyline.lines()) {
                Point a = line.a, b = line.b;
                if (a == b) continue;
                if (std::make_pair(a.x(), a.y()) > std::make_pair(b.x(), b.y()))
                    std::swap(a, b);
                segments.emplace_back(layer->id(), int(path.role()), path.width, path.height,
                    path.mm3_per_mm, path.overhang_degree, path.is_force_no_extrusion(),
                    a.x(), a.y(), b.x(), b.y());
                if (!perimeter && path.role() == erBridgeInfill)
                    result.bridge_length += unscale<double>(line.length());
            }
        };
        for (const auto *region : layer->regions()) {
            visit_paths(region->perimeters, [&](const ExtrusionPath &path) { record(path, true); });
            visit_paths(region->fills, [&](const ExtrusionPath &path) { record(path, false); });
        }
    }
    for (auto *segments : {&result.perimeters, &result.ordinary_fill, &result.bridges})
        std::sort(segments->begin(), segments->end());
    return result;
}

void check_same_segments(const std::vector<Segment> &actual, const std::vector<Segment> &expected,
                         const char *comparison = "deposited paths")
{
    CAPTURE(comparison, actual.size(), expected.size());
    std::ostringstream diagnostic;
    diagnostic << std::setprecision(17);
    if (actual != expected) {
        std::vector<Segment> added, removed;
        std::set_difference(actual.begin(), actual.end(), expected.begin(), expected.end(), std::back_inserter(added));
        std::set_difference(expected.begin(), expected.end(), actual.begin(), actual.end(), std::back_inserter(removed));
        diagnostic << "unequal tuples: actual-only=" << added.size() << ", expected-only=" << removed.size();
        auto coordinates = [](const Segment &segment) {
            return std::array<double, 4>{double(std::get<7>(segment)), double(std::get<8>(segment)),
                                         double(std::get<9>(segment)), double(std::get<10>(segment))};
        };
        auto tuple_text = [](const Segment &segment) {
            std::ostringstream out;
            out << std::setprecision(17) << '(';
            std::apply([&](const auto &...value) { ((out << value << ','), ...); }, segment);
            out << ')';
            return out.str();
        };
        double max_coordinate_delta = 0., max_width_delta = 0., max_height_delta = 0.;
        double max_flow_delta = 0., max_overhang_delta = 0.;
        size_t unmatched_layer_role = 0, connector_flag_differences = 0;
        for (size_t index = 0; index < added.size(); ++index) {
            const auto &source = added[index];
            const auto source_xy = coordinates(source);
            const Segment *nearest = nullptr;
            double nearest_distance = std::numeric_limits<double>::infinity();
            bool nearest_reversed = false;
            for (const auto &candidate : expected) {
                if (std::get<0>(candidate) != std::get<0>(source) ||
                    std::get<1>(candidate) != std::get<1>(source))
                    continue;
                const auto candidate_xy = coordinates(candidate);
                for (bool reversed : {false, true}) {
                    double distance = 0.;
                    for (size_t axis = 0; axis < source_xy.size(); ++axis) {
                        const double delta = source_xy[axis] - candidate_xy[(axis + (reversed ? 2 : 0)) % 4];
                        distance += delta * delta;
                    }
                    if (distance < nearest_distance) {
                        nearest_distance = distance;
                        nearest = &candidate;
                        nearest_reversed = reversed;
                    }
                }
            }
            if (nearest == nullptr) {
                ++unmatched_layer_role;
                continue;
            }
            const auto nearest_xy = coordinates(*nearest);
            double coordinate_delta = 0.;
            for (size_t axis = 0; axis < source_xy.size(); ++axis)
                coordinate_delta = std::max(coordinate_delta,
                    std::abs(source_xy[axis] - nearest_xy[(axis + (nearest_reversed ? 2 : 0)) % 4]));
            max_coordinate_delta = std::max(max_coordinate_delta, coordinate_delta);
            max_width_delta = std::max(max_width_delta, std::abs(std::get<2>(source) - std::get<2>(*nearest)));
            max_height_delta = std::max(max_height_delta, std::abs(std::get<3>(source) - std::get<3>(*nearest)));
            max_flow_delta = std::max(max_flow_delta, std::abs(std::get<4>(source) - std::get<4>(*nearest)));
            max_overhang_delta = std::max(max_overhang_delta, std::abs(std::get<5>(source) - std::get<5>(*nearest)));
            connector_flag_differences += std::get<6>(source) != std::get<6>(*nearest);
            if (index < 4)
                diagnostic << "\nactual " << tuple_text(source) << "\nnearest expected " << tuple_text(*nearest)
                           << ", endpoint order reversed=" << nearest_reversed
                           << ", max XY delta in scaled units=" << coordinate_delta;
        }
        diagnostic << "\nnearest same-layer/role maxima: XY scaled=" << max_coordinate_delta
                   << ", XY mm=" << max_coordinate_delta * SCALING_FACTOR
                   << ", width=" << max_width_delta << ", height=" << max_height_delta
                   << ", mm3_per_mm=" << max_flow_delta << ", overhang_degree=" << max_overhang_delta
                   << ", connector flag differences=" << connector_flag_differences
                   << ", missing layer/role=" << unmatched_layer_role;
    }
    INFO(diagnostic.str());
    CHECK(bool(actual == expected));
}

} // namespace

TEST_CASE("Existing monotonic travel extension does not alter bridge extrusion", "[BridgeControls][LegacyTravel]")
{
    const bool internal = GENERATE(false, true);
    const std::string top_pattern = GENERATE("monotonic", "monotonicline");
    CAPTURE(internal, top_pattern);
    auto config = reslice_bridge_config("classic");
    config.set_deserialize_strict({{"top_surface_pattern", top_pattern}});
    // Equal bridge and general overlap retain the stock 15% overlap geometry;
    // density and internal bridge layer count retain their defaults.

    std::vector<Segment> baseline;
    for (double extension : {0., 45., 100.}) {
        CAPTURE(extension);
        config.set_key_value("monotonic_travel_into_wall", new ConfigOptionPercent(extension));
        // Fresh slices ensure equality cannot be an artifact of missed
        // invalidation when changing the existing travel setting.
        auto model = reslice_bridge_model(internal);
        Print print;
        slice_bridge_model(print, model, config);
        const auto snapshot = snapshot_bridge_paths(print);
        REQUIRE_FALSE(snapshot.bridges.empty());
        if (extension == 0.)
            baseline = snapshot.bridges;
        else
            check_same_segments(snapshot.bridges, baseline);
    }
}

TEST_CASE("Bridge pitch and overlap edits rebuild their geometry without changing ordinary extrusion", "[BridgeControls][Reslice]")
{
    const std::string generator = GENERATE("classic", "arachne");
    const bool internal = GENERATE(false, true);
    const std::string key = GENERATE("bridge_layer_density", "bridge_line_travel_extend");
    CAPTURE(generator, internal, key);
    auto config = reslice_bridge_config(generator);
    auto model = reslice_bridge_model(internal);
    Print print;
    slice_bridge_model(print, model, config);
    const auto baseline = snapshot_bridge_paths(print);
    REQUIRE_FALSE(baseline.perimeters.empty());
    REQUIRE_FALSE(baseline.ordinary_fill.empty());
    REQUIRE_FALSE(baseline.bridges.empty());

    const bool density_edit = key == "bridge_layer_density";
    const double default_value = density_edit ? 100. : 15.;
    const double changed_value = key == "bridge_layer_density" ? 50. : 100.;
    config.set_key_value(key, new ConfigOptionPercent(changed_value));
    print.apply(model, config);
    const auto *printed = print.get_object(size_t(0));
    CHECK(printed->is_step_done(posSlice));
    CHECK(printed->is_step_done(posPerimeters) == density_edit);
    CHECK(printed->is_step_done(posPrepareInfill) == density_edit);
    CHECK(printed->is_step_done(posSupportMaterial));
    CHECK_FALSE(printed->is_step_done(posInfill));
    print.process();
    const auto changed = snapshot_bridge_paths(print);
    check_same_segments(changed.perimeters, baseline.perimeters);
    check_same_segments(changed.ordinary_fill, baseline.ordinary_fill);
    CHECK(bool(changed.bridges != baseline.bridges));
    if (key == "bridge_layer_density")
        CHECK(changed.bridge_length < baseline.bridge_length * 0.8);
    else
        CHECK(changed.bridge_length > baseline.bridge_length);

    // A changed already-sliced project must produce the same paths as a fresh
    // slice, rather than retaining old pitch or overlap boundaries in its caches.
    Print fresh;
    slice_bridge_model(fresh, model, config);
    const auto expected_changed = snapshot_bridge_paths(fresh);
    check_same_segments(changed.perimeters, expected_changed.perimeters);
    check_same_segments(changed.ordinary_fill, expected_changed.ordinary_fill);
    check_same_segments(changed.bridges, expected_changed.bridges);

    config.set_key_value(key, new ConfigOptionPercent(default_value));
    print.apply(model, config);
    CHECK(print.get_object(size_t(0))->is_step_done(posPerimeters) == density_edit);
    CHECK(print.get_object(size_t(0))->is_step_done(posPrepareInfill) == density_edit);
    CHECK_FALSE(print.get_object(size_t(0))->is_step_done(posInfill));
    print.process();
    const auto restored = snapshot_bridge_paths(print);
    check_same_segments(restored.perimeters, baseline.perimeters);
    check_same_segments(restored.ordinary_fill, baseline.ordinary_fill);
    check_same_segments(restored.bridges, baseline.bridges);
}

TEST_CASE("Bridge overlap uses the native wall footprint without changing classification or ordinary fill", "[BridgeControls][Overlap]")
{
    const std::string generator = GENERATE("classic", "arachne");
    const bool internal = GENERATE(false, true);
    const double requested = GENERATE(1., 100.);
    CAPTURE(generator, internal, requested);
    auto config = reslice_bridge_config(generator);
    auto model = reslice_bridge_model(internal);
    Print baseline;
    slice_bridge_model(baseline, model, config);
    config.set_key_value("bridge_line_travel_extend", new ConfigOptionPercent(requested));
    Print changed;
    slice_bridge_model(changed, model, config);

    // The ordinary overlap setting exercises the pre-existing perimeter
    // generator's native fill boundary. Only compare this boundary: changing
    // general overlap also legitimately changes interior shell classification.
    config.set_key_value("infill_wall_overlap", new ConfigOptionPercent(requested));
    Print native_boundary;
    slice_bridge_model(native_boundary, model, config);
    const auto before_paths = snapshot_bridge_paths(baseline);
    const auto after_paths = snapshot_bridge_paths(changed);
    REQUIRE_FALSE(before_paths.bridges.empty());
    REQUIRE_FALSE(before_paths.ordinary_fill.empty());
    CHECK(bool(after_paths.bridges != before_paths.bridges));
    check_same_segments(after_paths.ordinary_fill, before_paths.ordinary_fill, "bridge-only overlap preserves ordinary extrusion");
    check_same_segments(after_paths.perimeters, before_paths.perimeters, "bridge-only overlap preserves walls");

    auto polygon_area_mm2 = [](const ExPolygons &areas) {
        double result = 0.;
        for (const auto &area : areas) result += area.area();
        return result * SCALING_FACTOR * SCALING_FACTOR;
    };
    struct BridgeCoverage {
        Polylines centerlines;
        ExPolygons deposited;
        ExPolygons spacing_coverage;
        coord_t maximum_width = 0;
        BoundingBox bounds;
    };
    auto coverage = [](const LayerRegion &region) {
        BridgeCoverage result;
        Polygons deposited, spacing_coverage;
        visit_paths(region.fills, [&](const ExtrusionPath &path) {
            if (path.role() != erBridgeInfill || path.is_force_no_extrusion()) return;
            result.centerlines.push_back(path.polyline);
            path.polygons_covered_by_width(deposited, 0.f);
            // Stock thick bridges intentionally leave BRIDGE_EXTRA_SPACING
            // between beads. Cover each strand's actual pitch, so grid phase
            // changes are allowed but a missing strand or region is detected.
            polygons_append(spacing_coverage, offset(path.polyline,
                0.5f * float(Flow::bridging_flow(path.width, 0.f).scaled_spacing()) + float(SCALED_EPSILON)));
            result.maximum_width = std::max(result.maximum_width, coord_t(std::ceil(scale_(path.width))));
            for (const auto &point : path.polyline.points) result.bounds.merge(point);
        });
        result.deposited = union_ex(deposited);
        result.spacing_coverage = union_ex(spacing_coverage);
        return result;
    };
    auto *before = baseline.get_object(size_t(0));
    auto *after = changed.get_object(size_t(0));
    auto *native = native_boundary.get_object(size_t(0));
    REQUIRE(after->layer_count() == before->layer_count());
    REQUIRE(native->layer_count() == before->layer_count());
    double changed_wall_band_coverage = 0., lost_common_core_coverage = 0.;
    bool endpoints_changed = false;
    size_t bridge_regions = 0;
    for (size_t index = 0; index < before->layer_count(); ++index) {
        const auto *before_layer = before->get_layer(index);
        const auto *after_layer = after->get_layer(index);
        const auto *native_layer = native->get_layer(index);
        REQUIRE(after_layer->regions().size() == before_layer->regions().size());
        REQUIRE(native_layer->regions().size() == before_layer->regions().size());
        for (size_t region_index = 0; region_index < before_layer->regions().size(); ++region_index) {
            CAPTURE(index, region_index);
            const auto *before_region = before_layer->get_region(int(region_index));
            const auto *after_region = after_layer->get_region(int(region_index));
            const auto *native_region = native_layer->get_region(int(region_index));
            // Dedicated bridge boundaries must be exactly the native overlap
            // footprint, while all classified surfaces retain their geometry.
            CHECK(diff_ex(after_region->bridge_fill_expolygons, native_region->fill_expolygons).empty());
            CHECK(diff_ex(native_region->fill_expolygons, after_region->bridge_fill_expolygons).empty());
            std::map<SurfaceType, ExPolygons> before_surfaces, after_surfaces;
            for (const auto &surface : before_region->fill_surfaces.surfaces)
                before_surfaces[surface.surface_type].push_back(surface.expolygon);
            for (const auto &surface : after_region->fill_surfaces.surfaces)
                after_surfaces[surface.surface_type].push_back(surface.expolygon);
            REQUIRE(after_surfaces.size() == before_surfaces.size());
            for (const auto &[type, areas] : before_surfaces) {
                REQUIRE(after_surfaces.count(type) == 1);
                CHECK(diff_ex(areas, after_surfaces.at(type)).empty());
                CHECK(diff_ex(after_surfaces.at(type), areas).empty());
            }
            const auto baseline_coverage = coverage(*before_region);
            if (baseline_coverage.centerlines.empty()) continue;
            ++bridge_regions;
            const auto changed_coverage = coverage(*after_region);
            REQUIRE_FALSE(changed_coverage.centerlines.empty());
            CHECK(diff_pl(changed_coverage.centerlines, native_region->fill_expolygons).empty());
            endpoints_changed |= baseline_coverage.bounds.min != changed_coverage.bounds.min ||
                                 baseline_coverage.bounds.max != changed_coverage.bounds.max;

            // Separate wall overlap must retain the classified bridge's
            // interior. Check actual strand-spacing coverage there as well:
            // the existing native spacing adjustment may shift bead gaps.
            ExPolygons bridge_domain, adjusted_domain;
            for (const auto &surface : before_region->fill_surfaces.surfaces)
                if (surface.is_bridge()) {
                    bridge_domain.push_back(surface.expolygon);
                    append(adjusted_domain, after_region->bridge_fill_boundary(surface.expolygon));
                }
            const auto common_interior = offset_ex(intersection_ex(bridge_domain,
                intersection_ex(before_region->fill_expolygons, native_region->fill_expolygons)),
                -double(std::max(baseline_coverage.maximum_width, changed_coverage.maximum_width)));
            CHECK(diff_ex(common_interior, adjusted_domain).empty());
            const auto uncovered = diff_ex(common_interior, changed_coverage.spacing_coverage);
            lost_common_core_coverage += polygon_area_mm2(uncovered);
            // Integer polygon offsets can leave long sub-micron slivers.
            // Bound their radius by the slicer's native geometric epsilon;
            // a missing strand or an unfilled patch survives this erosion.
            CHECK(offset_ex(uncovered, -SCALED_EPSILON).empty());
            if (requested > 15.) {
                const auto added_wall_band = diff_ex(native_region->fill_expolygons, before_region->fill_expolygons);
                changed_wall_band_coverage += polygon_area_mm2(intersection_ex(changed_coverage.deposited, added_wall_band));
            } else {
                const auto removed_wall_band = diff_ex(before_region->fill_expolygons, native_region->fill_expolygons);
                changed_wall_band_coverage += polygon_area_mm2(intersection_ex(baseline_coverage.deposited, removed_wall_band));
            }
        }
    }
    CAPTURE(bridge_regions, changed_wall_band_coverage, lost_common_core_coverage);
    REQUIRE(bridge_regions > 0);
    CHECK(endpoints_changed);
    CHECK(changed_wall_band_coverage > 0.);
}

TEST_CASE("Internal bridge layer overrides stay inside their modifier region", "[BridgeControls][Layers][Modifiers]")
{
    const std::string generator = GENERATE("classic", "arachne");
    CAPTURE(generator);
    auto config = reslice_bridge_config(generator);
    auto model = reslice_bridge_model(true);
    auto *object = model.objects.front();
    TriangleMesh modifier_mesh(its_make_cube(20., 30., 10.));
    modifier_mesh.translate(-5., -5., -1.);
    auto *modifier = object->add_volume(modifier_mesh);
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    modifier->config.set("internal_bridge_layers", 3);
    Print print;
    slice_bridge_model(print, model, config);

    std::map<int, std::set<size_t>> bridge_planes;
    std::map<int, ExPolygons> bridge_areas;
    for (const auto *layer : print.get_object(size_t(0))->layers()) {
        for (const auto *region : layer->regions()) {
            const int requested = region->region().config().internal_bridge_layers.value;
            const ExPolygons region_material = to_expolygons(region->slices.surfaces);
            for (const auto &surface : region->fill_surfaces.surfaces) {
                if (surface.surface_type != stInternalBridge) continue;
                CAPTURE(layer->id(), requested);
                bridge_planes[requested].insert(layer->id());
                bridge_areas[requested].push_back(surface.expolygon);
                CHECK(diff_ex(ExPolygons{surface.expolygon}, region_material).empty());
            }
        }
    }
    REQUIRE(bridge_planes.count(1) == 1);
    REQUIRE(bridge_planes.count(3) == 1);
    CHECK(bridge_planes.at(1).size() == 1);
    CHECK(bridge_planes.at(3).size() == 3);
    CHECK(*bridge_planes.at(1).rbegin() == *bridge_planes.at(3).rbegin());
    CHECK(intersection_ex(bridge_areas.at(1), bridge_areas.at(3)).empty());
}
