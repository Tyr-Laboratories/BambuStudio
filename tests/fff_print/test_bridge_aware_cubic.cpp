#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/Preset.hpp"

#include <algorithm>
#include <functional>
#include <limits>

using namespace Slic3r;

namespace {
DynamicPrintConfig bridge_aware_config(double gap = 5.)
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig&>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "5%"}, {"sparse_infill_pattern", "bridgeawarecubic"},
        {"bridge_aware_cubic_max_gap", std::to_string(gap)},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
        {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"}
    });
    return config;
}

Model bridge_aware_cube()
{
    Model model;
    auto *object = model.add_object();
    object->name = "Bridge aware cubic fixture";
    object->add_volume(TriangleMesh(its_make_cube(32., 28., 18.)));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

void paths(const ExtrusionEntity &entity, const std::function<void(const ExtrusionPath&)> &visit)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath*>(&entity)) visit(*path);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath*>(&entity))
        for (const auto &path : multi->paths) visit(path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(&entity))
        for (const auto &path : loop->paths) visit(path);
    else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(&entity))
        for (const auto *child : collection->entities) paths(*child, visit);
}

void slice(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();
}

double sparse_length(const Print &print)
{
    double result = 0.;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions())
            paths(region->fills, [&](const ExtrusionPath &p) {
                if (p.role() == erInternalInfill) result += unscale<double>(p.length());
            });
    return result;
}

// Independent interval measurement on emitted toolpaths. No planner depth or
// expected-cell helper is used to derive the maximum roof span.
std::pair<size_t, double> roof_gaps(const Print &print)
{
    size_t checked = 0;
    double longest = 0.;
    const auto &layers = print.get_object(size_t(0))->layers();
    for (size_t i = 1; i < layers.size(); ++i) {
        ExPolygons sparse;
        Polygons support;
        for (const LayerRegion *region : layers[i - 1]->regions()) {
            for (const Surface &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == stInternal) sparse.push_back(surface.expolygon);
            region->perimeters.polygons_covered_by_width(support, 0.f);
            region->fills.polygons_covered_by_width(support, 0.f);
        }
        if (sparse.empty()) continue;
        for (const LayerRegion *region : layers[i]->regions())
            paths(region->fills, [&](const ExtrusionPath &p) {
                if (p.role() == erInternalInfill || intersection_pl(Polylines{p.polyline}, sparse).empty()) return;
                ++checked;
                for (const Polyline &gap : diff_pl(Polylines{p.polyline}, support)) {
                    if (!intersection_pl(Polylines{gap}, sparse).empty())
                        longest = std::max(longest, unscale<double>(gap.length()));
                }
            });
    }
    return {checked, longest};
}
}

TEST_CASE("Bridge aware cubic bounds emitted roof spans over low density infill",
          "[BridgeAwareCubic][Integration]")
{
    auto model = bridge_aware_cube();
    auto config = bridge_aware_config();
    // Establish that this fixture reproduces the long-span problem with the
    // ordinary lattice, using the same independently measured emitted paths.
    auto control_config = config;
    control_config.set_deserialize_strict({{"sparse_infill_pattern", "cubic"}});
    Print control;
    slice(control, model, control_config);
    REQUIRE(roof_gaps(control).second > 5.);
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    const auto [count, gap] = roof_gaps(print);
    REQUIRE(count > 0);
    CHECK(gap <= 5.);
    CHECK(sparse_length(print) > 0.);
}

TEST_CASE("Bridge aware cubic gap changes regenerate the hierarchy",
          "[BridgeAwareCubic][Integration][Reslice]")
{
    auto model = bridge_aware_cube();
    auto config = bridge_aware_config(8.);
    Print print;
    slice(print, model, config);
    const double coarse = sparse_length(print);
    config.set_deserialize_strict({{"bridge_aware_cubic_max_gap", "3"}});
    print.apply(model, config);
    print.process();
    const auto [count, gap] = roof_gaps(print);
    REQUIRE(count > 0);
    CHECK(gap <= 3.);
    CHECK(sparse_length(print) > coarse);
    const double fine = sparse_length(print);
    print.apply(model, config);
    print.process();
    CHECK(sparse_length(print) == Approx(fine));
}

TEST_CASE("Bridge aware cubic retains support under relocated bridge stacks and combined infill",
          "[BridgeAwareCubic][Integration][Stack]")
{
    auto model = bridge_aware_cube();
    auto config = bridge_aware_config();
    config.set_deserialize_strict({{"internal_bridge_layers", "3"}, {"infill_combination", "1"}});
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    const auto [count, gap] = roof_gaps(print);
    REQUIRE(count > 0);
    CHECK(gap <= 5.);
}

TEST_CASE("Bridge aware cubic rejects infeasible gaps instead of certifying an oversized opening",
          "[BridgeAwareCubic][Integration][Limits]")
{
    auto model = bridge_aware_cube();
    auto config = bridge_aware_config(0.01);
    Print print;
    REQUIRE_THROWS(slice(print, model, config));
}

TEST_CASE("Bridge aware cubic zero and full densities retain explicit fill semantics",
          "[BridgeAwareCubic][Integration][Density]")
{
    auto model = bridge_aware_cube();
    auto config = bridge_aware_config();
    const std::string density = GENERATE("0%", "100%");
    config.set_deserialize_strict({{"sparse_infill_density", density}});
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    if (density == "0%") CHECK(sparse_length(print) == 0.);
    else CHECK(sparse_length(print) > 0.);
}

TEST_CASE("Bridge aware cubic rejects export transformations outside its geometry checks",
          "[BridgeAwareCubic][Integration][ExportGuards]")
{
    auto model = bridge_aware_cube();
    auto config = bridge_aware_config();
    std::string expected_error;
    SECTION("seam clipping") {
        config.set_deserialize_strict({{"seam_gap", "10%"}});
        expected_error = "Seam gap";
    }
    SECTION("spiral vase with an active sparse object override") {
        config.set_deserialize_strict({{"spiral_mode", "1"}});
        // The global spiral preset normalizes sparse density to zero. A model
        // override is applied afterwards and can reactivate incompatible BAC.
        model.objects.front()->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(5.));
        expected_error = "Spiral vase";
    }
    SECTION("scarf override") {
        config.set_deserialize_strict({{"override_filament_scarf_seam_setting", "1"}, {"seam_slope_type", "all"}});
        expected_error = "Scarf";
    }
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    REQUIRE(print.get_object(size_t(0))->has_bridge_aware_cubic());
    REQUIRE_THAT(print.validate().string, Catch::Contains(expected_error));
    REQUIRE_THROWS_AS(print.process(), SlicingError);
}

TEST_CASE("Spiral vase normalizes bridge aware cubic to inactive sparse infill",
          "[BridgeAwareCubic][Integration][ExportGuards]")
{
    auto model = bridge_aware_cube();
    auto config = bridge_aware_config();
    config.set_deserialize_strict({{"spiral_mode", "1"}});
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    const PrintObject *object = print.get_object(size_t(0));
    REQUIRE(print.config().spiral_mode.value);
    REQUIRE_FALSE(object->has_bridge_aware_cubic());
    for (const PrintRegion &region : object->all_regions())
        REQUIRE(region.config().sparse_infill_density.value == 0.);
    REQUIRE(sparse_length(print) == 0.);
}

TEST_CASE("Active bridge aware cubic requires one finite shared nozzle offset",
          "[BridgeAwareCubic][Integration][ExportGuards][NozzleOffsets]")
{
    const std::string variant = GENERATE("equal", "different", "nan", "infinite");
    CAPTURE(variant);
    Vec2d first(.125, -.25), second = first;
    if (variant == "different") second.x() += .5;
    else if (variant == "nan") first.x() = second.x() = std::numeric_limits<double>::quiet_NaN();
    else if (variant == "infinite") first.y() = second.y() = std::numeric_limits<double>::infinity();

    auto model = bridge_aware_cube();
    auto config = bridge_aware_config();
    config.set_key_value("extruder_offset", new ConfigOptionPoints{first, second});
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    REQUIRE(print.config().extruder_offset.values.size() == 2);
    REQUIRE(print.get_object(size_t(0))->has_bridge_aware_cubic());

    if (variant == "equal") {
        REQUIRE(print.validate().string.empty());
        REQUIRE_NOTHROW(print.process());
        REQUIRE(sparse_length(print) > 0.);
    } else {
        REQUIRE_THAT(print.validate().string, Catch::Contains("identical extruder offsets"));
        REQUIRE_THROWS_WITH(print.process(), Catch::Contains("identical nozzle XY offsets"));
    }
}

TEST_CASE("Nozzle offset restriction applies only to active bridge aware cubic",
          "[BridgeAwareCubic][Integration][ExportGuards][NozzleOffsets]")
{
    const std::string mode = GENERATE("ordinary cubic", "zero density", "solid density");
    CAPTURE(mode);
    auto model = bridge_aware_cube();
    auto config = bridge_aware_config();
    if (mode == "ordinary cubic") config.set_deserialize_strict({{"sparse_infill_pattern", "cubic"}});
    else config.set_deserialize_strict({{"sparse_infill_density", mode == "zero density" ? "0%" : "100%"}});
    config.set_key_value("extruder_offset", new ConfigOptionPoints{Vec2d(.125, -.25), Vec2d(.625, -.25)});
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    REQUIRE(print.config().extruder_offset.values.size() == 2);
    REQUIRE_FALSE(print.get_object(size_t(0))->has_bridge_aware_cubic());
    REQUIRE(print.validate().string.find("Bridge aware cubic") == std::string::npos);
    REQUIRE_NOTHROW(print.process());
}

TEST_CASE("Bridge aware cubic refuses extrusion caches without a support plan",
          "[BridgeAwareCubic][Integration][Cache]")
{
    auto model = bridge_aware_cube();
    auto config = bridge_aware_config();
    Print print;
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    CHECK(print.load_cached_data("/nonexistent-bridge-aware-cubic-cache") == CLI_IMPORT_CACHE_DATA_CAN_NOT_USE);
    config.set_deserialize_strict({{"sparse_infill_pattern", "cubic"}});
    print.apply(model, config);
    CHECK(print.load_cached_data("/nonexistent-bridge-aware-cubic-cache") == CLI_IMPORT_CACHE_NOT_FOUND);
}

TEST_CASE("Bridge aware cubic checks sparse bridge follow-up layers",
          "[BridgeAwareCubic][Integration][BridgeDensity]")
{
    auto model = bridge_aware_cube();
    auto config = bridge_aware_config();
    config.set_deserialize_strict({{"internal_bridge_layers", "3"}, {"bridge_layer_density", "5%"}});
    Print print;
    REQUIRE_THROWS_WITH(slice(print, model, config),
        Catch::Matchers::Contains("Bridge Layer Density") &&
        Catch::Matchers::Contains("follow-up bridge layer"));
}

namespace {

DynamicPrintConfig mixed_sublayer_guard_config()
{
    auto config = DynamicPrintConfig::full_print_config();
    config.apply(bridge_aware_config());
    config.set_num_filaments(3);
    const auto &defaults = FullPrintConfig::defaults();
    // Match the complete filament fixtures used by test_printgcode.cpp:
    // set_num_filaments alone does not cover every per-filament profile array.
    for (const std::string &key : Preset::filament_options()) {
        if (key.find("compatible_") == 0) continue;
        ConfigOption *option = config.option(key);
        if (option != nullptr && option->is_vector())
            static_cast<ConfigOptionVectorBase *>(option)->resize(3, defaults.option(key));
    }
    config.set_key_value("filament_self_index", new ConfigOptionInts{1, 2, 3});
    config.set_key_value("filament_map", new ConfigOptionInts{1, 1, 1});
    config.set_key_value("filament_volume_map", new ConfigOptionInts{int(nvtStandard), int(nvtStandard), int(nvtStandard)});
    config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats(std::vector<double>(9, 0.)));
    config.set_key_value("filament_is_mixed", new ConfigOptionBools{false, false, true});
    config.set_key_value("filament_mixed_components", new ConfigOptionStrings{"", "", "1,2"});
    config.set_key_value("filament_mixed_sublayer_ratios", new ConfigOptionStrings{"", "", "0.5,0.5"});
    config.set_deserialize_strict({{"enable_mixed_color_sublayer", "1"},
        {"override_filament_scarf_seam_setting", "1"}, {"seam_slope_type", "none"},
        {"apply_scarf_seam_on_circles", "0"}});
    return config;
}

} // namespace

TEST_CASE("Bridge aware cubic rejects mixed sublayers assigned to enabled model roles",
          "[BridgeAwareCubic][Integration][ExportGuards][MixedSublayers]")
{
    const std::string assignment = GENERATE("object", "wall_filament", "sparse_infill_filament", "solid_infill_filament", "gradient");
    CAPTURE(assignment);
    auto model = bridge_aware_cube();
    auto config = mixed_sublayer_guard_config();
    auto *model_object = model.objects.front();
    model_object->config.set("extruder", assignment == "object" || assignment == "gradient" ? 3 : 1);
    if (assignment == "wall_filament" || assignment == "sparse_infill_filament" || assignment == "solid_infill_filament")
        model_object->config.set(assignment, 3);
    if (assignment == "gradient") {
        config.set_key_value("filament_mixed_gradient", new ConfigOptionBools{false, false, true});
        config.set_key_value("filament_mixed_gradient_range", new ConfigOptionStrings{"", "", "0.1,0.9"});
    }
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model_object);
    print.apply(model, config);
    const PrintObject *object = print.get_object(size_t(0));
    REQUIRE(object->has_bridge_aware_cubic());
    const auto error = print.validate();
    REQUIRE_THAT(error.string, Catch::Contains("Mixed color sublayer"));
    CHECK(error.opt_key == "enable_mixed_color_sublayer");
    CHECK(error.object == object);
    CHECK(object->layer_count() == 0);
}

TEST_CASE("Bridge aware cubic mixed sublayer guard follows enabled object roles",
          "[BridgeAwareCubic][Integration][ExportGuards][MixedSublayers]")
{
    const std::string mode = GENERATE("disabled sublayers", "unused mixed slot", "ordinary cubic",
        "zero density", "solid density", "ordinary neighboring object", "disabled wall role");
    CAPTURE(mode);
    auto model = bridge_aware_cube();
    auto config = mixed_sublayer_guard_config();
    auto *model_object = model.objects.front();
    model_object->config.set("extruder", 3);
    if (mode == "disabled sublayers")
        config.set_deserialize_strict({{"enable_mixed_color_sublayer", "0"}});
    else if (mode == "ordinary cubic")
        config.set_deserialize_strict({{"sparse_infill_pattern", "cubic"}});
    else if (mode == "zero density" || mode == "solid density")
        config.set_deserialize_strict({{"sparse_infill_density", mode == "zero density" ? "0%" : "100%"}});
    else {
        model_object->config.set("extruder", 1);
        if (mode == "ordinary neighboring object") {
            auto *ordinary = model.add_object();
            ordinary->name = "Ordinary object with mixed sublayers";
            ordinary->config.set("extruder", 3);
            ordinary->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipCubic));
            ordinary->add_volume(TriangleMesh(its_make_cube(16., 16., 18.)));
            ordinary->add_instance()->set_offset(Vec3d(140., 80., 0.));
        } else if (mode == "disabled wall role") {
            model_object->config.set("wall_filament", 3);
            model_object->config.set("wall_loops", 0);
        }
    }
    Print print;
    print.set_status_silent();
    for (ModelObject *object : model.objects) print.auto_assign_extruders(object);
    print.apply(model, config);
    const auto error = print.validate();
    CAPTURE(error.string);
    REQUIRE(error.string.empty());
}
