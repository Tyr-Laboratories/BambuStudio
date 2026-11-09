// The generator of the release benchmark's native projects (docs/bridge-aware-cubic-benchmark.md): benchmark tooling, not a test of
// the pattern. Built as its own executable, outside the pattern's test suite; its fixtures are the suite's.
#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"
#include "bridge_aware_cubic_observer.hpp"

#include "libslic3r/Format/bbs_3mf.hpp"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <iostream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace BACFixtures;
using namespace BACObserver;

// Writes the native projects of the official benchmark into BAC_BENCHMARK_FIXTURE_DIR, refusing to overwrite; carried over from
// the earlier branch, its fixtures unchanged.
TEST_CASE("Bridge aware cubic writes native CLI benchmark fixtures", "[.][BACBenchmarkFixtures]")
{
    namespace fs = boost::filesystem;
    const char *requested_directory = std::getenv("BAC_BENCHMARK_FIXTURE_DIR");
    REQUIRE(requested_directory != nullptr);
    REQUIRE(std::string(requested_directory).size() > 0);
    const fs::path directory(requested_directory);
    REQUIRE(directory.is_absolute());
    fs::create_directories(directory);
    REQUIRE(fs::is_directory(directory));
    // Preflight every destination before writing any artifact. A repeated run
    // must use a fresh directory rather than overwrite a previous measurement.
    REQUIRE_FALSE(fs::exists(directory / "manifest.json"));
    for (const char *size : {"small", "large", "rib", "modifier"})
        for (const char *pattern : {"cubic", "bridgeawarecubic"})
            for (const char *suffix : {".3mf", ".3mf.tmp", ".config.ini", ".config.json"})
                REQUIRE_FALSE(fs::exists(directory / (std::string(size) + "-" + pattern + suffix)));

    Workspace workspace;
    DynamicPrintConfig defaults;
    // DynamicPrintConfig::full_print_config() currently supplies region
    // defaults only. Explicitly complete object, region and printer defaults.
    defaults.apply(FullPrintConfig::defaults());
    defaults.apply(observed(fixture_config(5., "bridgeawarecubic")));
    defaults.set_deserialize_strict({{"inherits", ""}, {"inherits_group", ""}});
    nlohmann::json manifest = {
        {"schema_version", 1},
        {"generator_version", SLIC3R_VERSION},
        {"configuration_basis", "FullPrintConfig defaults + named fixture config, normalized by Print::apply"},
        {"slicing_performed", false},
        {"allowed_pair_config_differences", {"sparse_infill_pattern", "bridge_aware_cubic_max_gap"}},
        {"fixtures", nlohmann::json::array()}
    };
    using SerializedSettings = std::map<std::string, std::string>;
    std::map<std::string, SerializedSettings> common_settings_by_family;
    for (const std::string size : {"small", "large", "rib", "modifier"}) {
        const bool rib = size == "rib", modifier = size == "modifier";
        const std::string family = rib ? "rib" : modifier ? "modifier" : "cube";
        const double width = modifier ? 48. : size == "large" ? 64. : 32.;
        const double depth = rib ? 26. : size == "large" ? 56. : 28.;
        const double height = rib ? 15. : 18.;
        const double density = rib ? 12. : 5., gap = modifier ? 8. : 5.;
        const size_t model_triangles = rib ? 36 : 12;
        DynamicPrintConfig fixture_defaults = defaults;
        if (rib || modifier) {
            // Keep these physical cases equal to acceptance_config() and
            // modifier_config(), including defaults absent from the observed fixture config.
            fixture_defaults.clear();
            fixture_defaults.apply(FullPrintConfig::defaults());
            fixture_defaults.set_deserialize_strict({
                {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
                {"sparse_infill_density", rib ? "12%" : "5%"},
                {"sparse_infill_pattern", "bridgeawarecubic"},
                {"bridge_aware_cubic_max_gap", rib ? "5" : "8"},
                {"infill_direction", rib ? "17" : "45"},
                {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
                {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
                {"wall_loops", "2"}, {"wall_generator", "classic"},
                {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
                {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
                {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"},
                {"inherits", ""}, {"inherits_group", ""}
            });
            if (rib) fixture_defaults.set_deserialize_strict({{"ensure_vertical_shell_thickness", "disabled"}});
        }
        // Keep every fixture on the cubes' existing empty machine scripts.
        // Native defaults can move Z before the slicer declares G90/G21.
        fixture_defaults.set_deserialize_strict({{"machine_start_gcode", ""},
                                                 {"machine_end_gcode", ""}});
        Model model;
        ModelObject *object = model.add_object();
        object->name = "BAC benchmark " + size;
        ModelVolume *volume = nullptr, *right = nullptr;
        if (rib) {
            // Same common-frame assembly as the InwardWalls acceptance case.
            TriangleMesh body(its_make_cube(32., 26., 10.));
            TriangleMesh first(its_make_cube(5., 22., 5.));
            first.translate(6., 2., 10.);
            TriangleMesh second(its_make_cube(4., 18., 3.4));
            second.translate(22., 4., 10.);
            body.merge(first);
            body.merge(second);
            volume = object->add_volume(std::move(body));
            volume->name = "Benchmark inward ribs";
        } else {
            volume = object->add_volume(TriangleMesh(its_make_cube(width, depth, height)));
            volume->name = "Benchmark cube";
        }
        if (modifier) {
            TriangleMesh mesh(its_make_cube(24., 28., 18.));
            mesh.translate(24., 0., 0.);
            right = object->add_volume(std::move(mesh));
            right->set_type(ModelVolumeType::PARAMETER_MODIFIER);
            right->name = "Benchmark right half modifier";
            right->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(12.));
            right->config.set("infill_direction", 17.);
        }
        object->add_instance()->set_offset(Vec3d(80., 80., 0.));
        REQUIRE(object->facets_count() == model_triangles);
        REQUIRE(object->volumes.size() == size_t(modifier ? 2 : 1));
        for (const ModelVolume *part : object->volumes) REQUIRE(part->material_id().empty());
        const auto bounds = object->bounding_box();
        REQUIRE(bounds.min.x() == Approx(80.));
        REQUIRE(bounds.min.y() == Approx(80.));
        REQUIRE(bounds.min.z() == Approx(0.));
        REQUIRE(bounds.max.x() == Approx(80. + width));
        REQUIRE(bounds.max.y() == Approx(80. + depth));
        REQUIRE(bounds.max.z() == Approx(height));
        std::set<std::pair<int, int>> assignments{{0, 0}};
        PlateData plate(0, assignments, false);
        plate.plate_name = "Benchmark plate";
        std::vector<SerializedSettings> common_volume_settings;
        for (const bool bac : {false, true}) {
            const std::string pattern = bac ? "bridgeawarecubic" : "cubic";
            const std::string stem = size + "-" + pattern;
            DynamicPrintConfig config = fixture_defaults;
            config.set_deserialize_strict({{"sparse_infill_pattern", pattern}});
            if (right) {
                if (bac) right->config.set("bridge_aware_cubic_max_gap", 3.);
                else right->config.erase("bridge_aware_cubic_max_gap");
            }
            Print configured;
            configured.set_status_silent();
            configured.auto_assign_extruders(object);
            configured.apply(model, config);
            REQUIRE(configured.objects().size() == 1);
            REQUIRE(configured.get_object(size_t(0))->layers().empty());
            REQUIRE_FALSE(configured.is_step_done(posSlice));
            config = configured.full_print_config();
            for (const char *key : {"machine_start_gcode", "machine_end_gcode"}) {
                CAPTURE(stem, key);
                REQUIRE(config.opt_string(key).empty());
            }
            // The native CLI indexes process, filament and printer inheritance
            // slots directly. Empty entries preserve no-inheritance semantics;
            // an empty vector is not a valid CLI project shape.
            const auto *filament_settings = config.option<ConfigOptionStrings>("filament_settings_id");
            REQUIRE(filament_settings != nullptr);
            const size_t filament_count = filament_settings->values.size();
            REQUIRE(filament_count == 1);
            config.set_key_value("inherits_group", new ConfigOptionStrings(filament_count + 2, ""));
            // Packaging also indexes one filament ID per slot. Empty IDs keep
            // the fixture independent of any named material/profile identity.
            config.set_key_value("filament_ids", new ConfigOptionStrings(filament_count, ""));
            // Stock Cubic must not depend on ignoring an unknown BAC option.
            // Remove it after applying defaults, which may reintroduce it.
            if (!bac) config.erase("bridge_aware_cubic_max_gap");
            REQUIRE(object->config.empty());
            REQUIRE(volume->config.empty());
            REQUIRE(plate.config.empty());
            REQUIRE(config.opt_serialize("inherits").empty());
            const auto &inheritance = config.option<ConfigOptionStrings>("inherits_group")->values;
            REQUIRE(inheritance.size() == filament_count + 2);
            REQUIRE(std::all_of(inheritance.begin(), inheritance.end(), [](const std::string &entry) { return entry.empty(); }));
            const auto &filament_ids = config.option<ConfigOptionStrings>("filament_ids")->values;
            REQUIRE(filament_ids.size() == filament_count);
            REQUIRE(std::all_of(filament_ids.begin(), filament_ids.end(), [](const std::string &entry) { return entry.empty(); }));
            std::map<std::string, std::string> comparable;
            std::map<std::string, std::string> serialized_settings;
            std::vector<std::string> nil_settings;
            for (const std::string &key : config.keys()) {
                serialized_settings.emplace(key, config.opt_serialize(key));
                if (config.option(key)->is_nil()) nil_settings.push_back(key);
                if (key != "sparse_infill_pattern" && key != "bridge_aware_cubic_max_gap")
                    comparable.emplace(key, config.opt_serialize(key));
            }
            auto &common_settings = common_settings_by_family[family];
            if (common_settings.empty()) common_settings = comparable;
            else REQUIRE(comparable == common_settings);

            nlohmann::json volumes = nlohmann::json::array();
            std::vector<SerializedSettings> comparable_volumes;
            size_t total_triangles = 0;
            for (size_t index = 0; index < object->volumes.size(); ++index) {
                const ModelVolume &part = *object->volumes[index];
                SerializedSettings local, local_comparable;
                std::vector<std::string> local_nil;
                for (const std::string &key : part.config.keys()) {
                    local.emplace(key, part.config.opt_serialize(key));
                    if (part.config.option(key)->is_nil()) local_nil.push_back(key);
                    if (key != "bridge_aware_cubic_max_gap") local_comparable.emplace(key, part.config.opt_serialize(key));
                }
                comparable_volumes.push_back(std::move(local_comparable));
                if (&part == right) {
                    REQUIRE(local.size() == size_t(bac ? 3 : 2));
                    REQUIRE(local.at("sparse_infill_density") == "12%");
                    REQUIRE(local.at("infill_direction") == "17");
                    if (bac) REQUIRE(local.at("bridge_aware_cubic_max_gap") == "3");
                } else REQUIRE(part.config.empty());
                const auto part_bounds = part.mesh().transformed_bounding_box(part.get_matrix());
                total_triangles += part.mesh().facets_count();
                volumes.push_back({
                    {"index", index}, {"name", part.name}, {"type", ModelVolume::type_to_string(part.type())},
                    {"triangle_count", part.mesh().facets_count()},
                    {"bounds_min_mm", {part_bounds.min.x(), part_bounds.min.y(), part_bounds.min.z()}},
                    {"bounds_max_mm", {part_bounds.max.x(), part_bounds.max.y(), part_bounds.max.z()}},
                    {"serialized_config", local}, {"nil_config_keys", local_nil}
                });
            }
            if (common_volume_settings.empty()) common_volume_settings = comparable_volumes;
            else REQUIRE(comparable_volumes == common_volume_settings);
            REQUIRE(total_triangles == model_triangles + size_t(modifier ? 12 : 0));

            const std::string archive = (directory / (stem + ".3mf")).string();
            StoreParams params;
            params.path = archive.c_str();
            params.model = &model;
            params.config = &config;
            params.plate_data_list = {&plate};
            REQUIRE(store_bbs_3mf(params));
            REQUIRE(fs::file_size(archive) > 0);
            const std::string snapshot = (directory / (stem + ".config.ini")).string();
            config.save(snapshot);
            REQUIRE(fs::file_size(snapshot) > 0);
            // Match the embedded project's nozzle-stat representation while
            // retaining the complete applied values separately in the INI.
            DynamicPrintConfig project_config = config;
            split_nozzle_stats_for_export(project_config);
            const std::string project_snapshot = (directory / (stem + ".config.json")).string();
            project_config.save_to_json(project_snapshot, "project_settings", "project", SLIC3R_VERSION);
            REQUIRE(fs::file_size(project_snapshot) > 0);
            manifest["fixtures"].push_back({
                {"name", stem}, {"size", size}, {"family", family}, {"pattern", pattern},
                {"source_fixture", rib ? "InwardWalls" : modifier ? "Modifiers local density angle and gap ownership" : "BACQualification"},
                {"archive", stem + ".3mf"}, {"config_ini", stem + ".config.ini"},
                {"project_config_json", stem + ".config.json"},
                {"dimensions_mm", {width, depth, height}}, {"instance_offset_mm", {80., 80., 0.}},
                {"bounds_min_mm", {bounds.min.x(), bounds.min.y(), bounds.min.z()}},
                {"bounds_max_mm", {bounds.max.x(), bounds.max.y(), bounds.max.z()}},
                {"expected_layer_count", rib ? 75 : 90}, {"triangle_count", total_triangles},
                {"model_triangle_count", model_triangles}, {"volume_count", object->volumes.size()}, {"volumes", volumes},
                {"object_count", 1}, {"instance_count", 1}, {"plate_count", 1},
                {"filament_count", filament_count},
                {"density_percent", density}, {"bac_gap_mm", bac ? nlohmann::json(gap) : nlohmann::json(nullptr)},
                {"config_key_count", config.keys().size()},
                {"serialized_config", serialized_settings}, {"nil_config_keys", nil_settings}
            });
        }
    }
    const std::string manifest_path = (directory / "manifest.json").string();
    std::ofstream output(manifest_path);
    REQUIRE(output.is_open());
    output << std::setw(2) << manifest << '\n';
    output.close();
    REQUIRE(output.good());
    std::cout << "BAC_BENCHMARK_FIXTURES manifest=" << manifest_path << " archives=8 slicing_performed=0" << std::endl;
}
