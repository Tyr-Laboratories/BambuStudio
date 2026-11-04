#include <catch2/catch.hpp>
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <set>

using namespace Slic3r;

namespace {
struct Workspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
                                  boost::filesystem::unique_path("bridge-acceleration-%%%%-%%%%");
    std::string old_data = data_dir(), old_temp = temporary_dir();
    Workspace() {
        boost::filesystem::create_directories(path);
        set_data_dir(path.string());
        set_temporary_dir(path.string());
    }
    ~Workspace() {
        set_data_dir(old_data);
        set_temporary_dir(old_temp);
        boost::system::error_code ec;
        boost::filesystem::remove_all(path, ec);
    }
};

DynamicPrintConfig bridge_config()
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "15%"}, {"sparse_infill_pattern", "grid"},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"default_acceleration", "2000"}, {"initial_layer_acceleration", "300"},
        {"outer_wall_acceleration", "700"}, {"inner_wall_acceleration", "900"},
        {"top_surface_acceleration", "600"}, {"sparse_infill_acceleration", "1500"},
        {"machine_max_acceleration_extruding", "10000"},
        {"enable_arc_fitting", "0"}, {"use_relative_e_distances", "1"},
        {"slow_down_for_layer_cooling", "0"}, {"brim_type", "no_brim"},
        {"wall_generator", "classic"}
    });
    return config;
}

Model bridge_model(bool internal)
{
    Model model;
    auto *object = model.add_object();
    object->name = internal ? "Roof over sparse infill" : "Bridge between pillars";
    TriangleMesh mesh(its_make_cube(internal ? 30. : 6., 20., 5.));
    if (!internal) {
        TriangleMesh right(its_make_cube(6., 20., 5.));
        right.translate(24., 0., 0.);
        mesh.merge(right);
        TriangleMesh roof(its_make_cube(30., 20., 1.));
        roof.translate(0., 0., 5.);
        mesh.merge(roof);
    }
    object->add_volume(mesh);
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

struct Accelerations {
    std::set<int> bridge, walls, overhang_walls, top, first_layer;
};

Accelerations read_accelerations(const std::string &file, const DynamicPrintConfig &config)
{
    Accelerations out;
    GCodeReader reader;
    reader.apply_config(config);
    std::string role;
    int acceleration = 0, layer = 0;
    REQUIRE(reader.parse_file(file, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.comment() == " CHANGE_LAYER") ++layer;
        const auto tag = line.comment().find(" FEATURE: ");
        if (tag != std::string_view::npos) role = std::string(line.comment().substr(tag + 10));
        if (line.cmd_is("M204")) {
            float value;
            if (line.has_value('S', value) || line.has_value('P', value)) acceleration = int(value);
        }
        if (!line.extruding(self) || line.dist_XY(self) <= 0.) return;
        if (layer == 1) out.first_layer.insert(acceleration);
        else if (role == "Bridge") out.bridge.insert(acceleration);
        else if (role == "Outer wall") out.walls.insert(acceleration);
        else if (role == "Overhang wall") out.overhang_walls.insert(acceleration);
        else if (role == "Top surface") out.top.insert(acceleration);
    }));
    return out;
}
}

TEST_CASE("Bridge acceleration persists in process profiles and Bambu projects", "[BridgeAcceleration][Config]")
{
    Workspace workspace;
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintConfig &>(FullPrintConfig::defaults()));
    REQUIRE(config.has("bridge_acceleration"));
    REQUIRE(config.opt_float_nullable("bridge_acceleration", 0) == 0.);
    config.set_deserialize_strict({{"bridge_acceleration", "800,1200"}});
    REQUIRE(std::count(Preset::print_options().begin(), Preset::print_options().end(), "bridge_acceleration") == 1);
    // Per-nozzle settings survive both text configuration and project loading.
    DynamicPrintConfig loaded;
    loaded.set_deserialize_strict({{"bridge_acceleration", config.option("bridge_acceleration")->serialize()}});
    REQUIRE(loaded.opt_float_nullable("bridge_acceleration", 0) == 800.);
    REQUIRE(loaded.opt_float_nullable("bridge_acceleration", 1) == 1200.);
    Preset preset(Preset::TYPE_PRINT, "Bridge acceleration test");
    preset.file = (workspace.path / "process.json").string();
    preset.config = loaded;
    REQUIRE(preset.save(nullptr));
    Preset reopened_preset(Preset::TYPE_PRINT, preset.name);
    reopened_preset.file = preset.file;
    reopened_preset.reload(Preset(Preset::TYPE_PRINT, "Default"));
    REQUIRE(reopened_preset.config.opt_float_nullable("bridge_acceleration", 0) == 800.);
    REQUIRE(reopened_preset.config.opt_float_nullable("bridge_acceleration", 1) == 1200.);
    Model model = bridge_model(true);
    const auto filename = (workspace.path / "bridge.3mf").string();
    StoreParams params;
    params.path = filename.c_str(); params.model = &model; params.config = &loaded;
    REQUIRE(store_bbs_3mf(params));
    Model reopened;
    DynamicPrintConfig reopened_config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    bool is_bambu = false; Semver version;
    PlateDataPtrs plates; std::vector<Preset *> presets;
    REQUIRE(load_bbs_3mf(filename.c_str(), &reopened_config, &substitutions, &reopened, &plates, &presets,
                        &is_bambu, &version, nullptr, LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    for (auto *plate : plates) delete plate;
    REQUIRE(reopened_config.option("bridge_acceleration")->serialize() == "800,1200");
}

TEST_CASE("H2S project restores bridge acceleration against its system profile", "[BridgeAcceleration][Config][ProfileRestore]")
{
    Workspace workspace;
    PresetBundle bundle;
    const auto profiles = boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() / "resources" / "profiles";
    bundle.load_vendor_configs_from_json(profiles.string(), "BBL", PresetBundle::LoadSystem,
                                         ForwardCompatibilitySubstitutionRule::Disable);
    const std::string base_name = "0.20mm Standard @BBL H2S";
    const auto *base = bundle.prints.find_preset(base_name, false);
    REQUIRE(base != nullptr);
    REQUIRE(base->config.option<ConfigOptionStrings>("print_extruder_variant")->size() == 3);
    auto saved = base->config;
    saved.set_deserialize_strict({{"inherits", base_name}, {"bridge_acceleration", "2500"}});
    std::string expected = "2500,2500,2500";
    SECTION("single value from an existing Tyr project") {}
    SECTION("explicit variant values retain their nozzle association") {
        saved.set_deserialize_strict({
            {"print_extruder_variant", "Direct Drive E3D High Flow;Direct Drive Standard;Direct Drive High Flow"},
            {"bridge_acceleration", "700,2500,1200"}});
        expected = "2500,1200,700";
    }
    SECTION("unset setting in an older project uses normal acceleration") {
        saved.erase("bridge_acceleration");
        expected = "0,0,0";
    }
    auto result = bundle.prints.load_external_preset("test.3mf", "Saved project.3mf", "Custom H2S process", saved,
        {"bridge_acceleration"}, PresetCollection::LoadAndSelect::Always, Semver());
    REQUIRE(result.first != nullptr);
    REQUIRE(bundle.prints.get_edited_preset().config.option("bridge_acceleration")->serialize() == expected);
    REQUIRE(bundle.prints.find_preset(base_name, false)->config.option("bridge_acceleration")->serialize() == "0,0,0");
}

TEST_CASE("Regular and internal bridge extrusion uses its dedicated acceleration", "[BridgeAcceleration][GCode]")
{
    const bool internal = GENERATE(false, true);
    const int requested = GENERATE(800, 0);
    CAPTURE(internal, requested);
    Workspace workspace;
    auto config = bridge_config();
    config.set_deserialize_strict({{"bridge_acceleration", std::to_string(requested)}});
    auto model = bridge_model(internal);
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();
    // Ensure the fixtures exercise the two distinct surface classifications.
    size_t surfaces = 0;
    for (const auto *layer : print.get_object(size_t(0))->layers())
        for (const auto *region : layer->regions())
            for (const auto &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == (internal ? stInternalBridge : stBottomBridge)) ++surfaces;
    REQUIRE(surfaces > 0);
    const auto filename = (workspace.path / "bridge.gcode").string();
    GCodeProcessorResult result;
    print.export_gcode(filename, &result);
    const auto values = read_accelerations(filename, config);
    REQUIRE(values.bridge == std::set<int>{requested == 0 ? 2000 : requested});
    REQUIRE(values.walls == std::set<int>{700});
    if (!internal) REQUIRE(values.overhang_walls == std::set<int>{700});
    REQUIRE(values.top == std::set<int>{600});
    REQUIRE(values.first_layer == std::set<int>{300});

    // A settings change on an already sliced job must refresh exported motion.
    config.set_deserialize_strict({{"bridge_acceleration", "1100"}});
    print.apply(model, config);
    REQUIRE_FALSE(print.is_step_done(psGCodeExport));
    print.process();
    print.export_gcode(filename, &result);
    REQUIRE(read_accelerations(filename, config).bridge == std::set<int>{1100});
}

TEST_CASE("Bridge acceleration respects motion limits", "[BridgeAcceleration][GCode]")
{
    const bool height_limit = GENERATE(false, true);
    Workspace workspace;
    auto config = bridge_config();
    config.set_deserialize_strict({{"bridge_acceleration", "1200"}, {"wall_generator", "arachne"}});
    if (height_limit) {
        config.set_deserialize_strict({{"enable_height_slowdown", "1"},
            {"slowdown_start_height", "1"}, {"slowdown_end_height", "2"},
            {"slowdown_start_acc", "450"}, {"slowdown_end_acc", "450"}});
    } else {
        config.set_deserialize_strict({{"machine_max_acceleration_extruding", "625"}});
    }
    auto model = bridge_model(false);
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();
    const auto filename = (workspace.path / "limited.gcode").string();
    GCodeProcessorResult result;
    print.export_gcode(filename, &result);
    REQUIRE(read_accelerations(filename, config).bridge == std::set<int>{height_limit ? 450 : 625});
}
