#include <catch2/catch.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"

#include <algorithm>
#include <array>
#include <boost/filesystem.hpp>

using namespace Slic3r;

namespace {

struct BridgeConfigWorkspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("bridge-controls-config-%%%%-%%%%");
    std::string old_data = data_dir(), old_temp = temporary_dir();

    BridgeConfigWorkspace()
    {
        boost::filesystem::create_directories(path);
        set_data_dir(path.string());
        set_temporary_dir(path.string());
    }

    ~BridgeConfigWorkspace()
    {
        set_data_dir(old_data);
        set_temporary_dir(old_temp);
        boost::system::error_code ec;
        boost::filesystem::remove_all(path, ec);
    }
};

constexpr std::array<const char *, 3> bridge_control_keys = {
    "bridge_layer_density", "bridge_line_travel_extend", "internal_bridge_layers"
};

void check_bridge_controls(const ConfigBase &config, double density, double extension, int layers)
{
    for (const auto *key : bridge_control_keys)
        REQUIRE(config.option(key) != nullptr);
    REQUIRE(config.option("bridge_layer_density")->type() == coPercent);
    REQUIRE(config.option("bridge_line_travel_extend")->type() == coPercent);
    REQUIRE(config.option("internal_bridge_layers")->type() == coInt);
    CHECK(config.option<ConfigOptionPercent>("bridge_layer_density")->value == density);
    CHECK(config.option<ConfigOptionPercent>("bridge_line_travel_extend")->value == extension);
    CHECK(config.option<ConfigOptionInt>("internal_bridge_layers")->value == layers);
}

void set_bridge_controls(ModelConfig &config, double density, double extension, int layers)
{
    config.set_key_value("bridge_layer_density", new ConfigOptionPercent(density));
    config.set_key_value("bridge_line_travel_extend", new ConfigOptionPercent(extension));
    config.set_key_value("internal_bridge_layers", new ConfigOptionInt(layers));
}

} // namespace

TEST_CASE("Bridge controls have scalar region defaults and closed value ranges", "[BridgeControls][Config]")
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    check_bridge_controls(config, 100., 15., 1);
    for (const auto *key : bridge_control_keys) {
        CAPTURE(key);
        CHECK(std::count(Preset::print_options().begin(), Preset::print_options().end(), key) == 1);
        const auto *definition = print_config_def.get(key);
        REQUIRE(definition != nullptr);
        CHECK(definition->min == 1.);
        CHECK(definition->max == (std::string(key) == "internal_bridge_layers" ? 3. : 100.));
    }

    SECTION("Bridge percentages accept endpoints and fractional intermediate values") {
        for (const auto *key : {"bridge_layer_density", "bridge_line_travel_extend"}) {
            for (const auto *value : {"1%", "37.5%", "100%"}) {
                CAPTURE(key, value);
                config.set_deserialize_strict({{key, value}});
                CHECK(config.validate().count(key) == 0);
            }
        }
    }
    SECTION("Bridge percentages reject values outside the closed interval") {
        for (const auto *key : {"bridge_layer_density", "bridge_line_travel_extend"}) {
            for (const auto *value : {"0%", "0.99%", "100.01%", "101%"}) {
                CAPTURE(key, value);
                config.set_deserialize_strict({{key, value}});
                CHECK(config.validate().count(key) == 1);
            }
        }
    }
    SECTION("Internal bridge layer count accepts one through three only") {
        for (int value = 0; value <= 4; ++value) {
            CAPTURE(value);
            config.set_deserialize_strict({{"internal_bridge_layers", std::to_string(value)}});
            CHECK(config.validate().count("internal_bridge_layers") == (value >= 1 && value <= 3 ? 0 : 1));
        }
    }
}

TEST_CASE("Bridge controls survive process JSON and Bambu project overrides", "[BridgeControls][Config][Persistence]")
{
    BridgeConfigWorkspace workspace;
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"bridge_layer_density", "72.5%"}, {"bridge_line_travel_extend", "25%"}, {"internal_bridge_layers", "2"}
    });

    Preset preset(Preset::TYPE_PRINT, "Bridge controls test");
    preset.file = (workspace.path / "process.json").string();
    preset.config = config;
    REQUIRE(preset.save(nullptr));
    Preset reopened_preset(Preset::TYPE_PRINT, preset.name);
    reopened_preset.file = preset.file;
    reopened_preset.reload(Preset(Preset::TYPE_PRINT, "Default"));
    check_bridge_controls(reopened_preset.config, 72.5, 25., 2);

    Model model;
    auto *object = model.add_object();
    auto *part = object->add_volume(TriangleMesh(its_make_cube(10., 10., 10.)));
    object->add_instance();
    set_bridge_controls(object->config, 50., 40., 3);
    set_bridge_controls(part->config, 33.5, 60.5, 1);

    const auto filename = (workspace.path / "bridge-controls.3mf").string();
    StoreParams params;
    params.path = filename.c_str();
    params.model = &model;
    params.config = &config;
    REQUIRE(store_bbs_3mf(params));
    Model reopened;
    DynamicPrintConfig reopened_config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    bool is_bambu = false;
    Semver version;
    PlateDataPtrs plates;
    std::vector<Preset *> presets;
    const bool loaded = load_bbs_3mf(filename.c_str(), &reopened_config, &substitutions, &reopened,
        &plates, &presets, &is_bambu, &version, nullptr, LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
    for (auto *plate : plates) delete plate;
    for (auto *saved_preset : presets) delete saved_preset;
    REQUIRE(loaded);
    check_bridge_controls(reopened_config, 72.5, 25., 2);
    REQUIRE(reopened.objects.size() == 1);
    const auto *reopened_object = reopened.objects.front();
    REQUIRE(reopened_object->volumes.size() == 1);
    check_bridge_controls(reopened_object->config.get(), 50., 40., 3);
    check_bridge_controls(reopened_object->volumes.front()->config.get(), 33.5, 60.5, 1);

    // Applying the restored inheritance chain must retain the part's scalar overrides.
    DynamicPrintConfig effective = reopened_config;
    effective.apply(reopened_object->config.get());
    effective.apply(reopened_object->volumes.front()->config.get());
    check_bridge_controls(effective, 33.5, 60.5, 1);
}

TEST_CASE("Bridge controls restore with the H2S multi-variant process", "[BridgeControls][Config][ProfileRestore]")
{
    BridgeConfigWorkspace workspace;
    PresetBundle bundle;
    const auto profiles = boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() / "resources" / "profiles";
    bundle.load_vendor_configs_from_json(profiles.string(), "BBL", PresetBundle::LoadSystem,
                                        ForwardCompatibilitySubstitutionRule::Disable);
    bundle.prints.update_user_presets_directory(workspace.path.string(), "process");
    const std::string name = "0.20mm Standard @BBL H2S";
    const auto *base = bundle.prints.find_preset(name, false);
    REQUIRE(base != nullptr);
    REQUIRE(base->config.option<ConfigOptionStrings>("print_extruder_variant") != nullptr);
    REQUIRE(base->config.option<ConfigOptionStrings>("print_extruder_variant")->size() == 3);
    check_bridge_controls(base->config, 100., 15., 1);

    auto saved = base->config;
    saved.set_deserialize_strict({{"inherits", name}, {"bridge_layer_density", "67.5%"},
        {"bridge_line_travel_extend", "42.5%"}, {"internal_bridge_layers", "3"}});
    bool legacy = false;
    SECTION("Ordinary project restore retains one scalar per setting") {}
    SECTION("Reordering nozzle variants does not reinterpret bridge controls as vectors") {
        saved.set_deserialize_strict({
            {"print_extruder_variant", "Direct Drive E3D High Flow;Direct Drive Standard;Direct Drive High Flow"}
        });
    }
    SECTION("Older projects without bridge controls retain the compatible defaults") {
        legacy = true;
        for (const auto *key : bridge_control_keys)
            saved.erase(key);
    }

    const auto restored = bundle.prints.load_external_preset("bridge-controls-test.3mf", "Bridge controls project",
        "Custom H2S bridge process", saved,
        {"bridge_layer_density", "bridge_line_travel_extend", "internal_bridge_layers"},
        PresetCollection::LoadAndSelect::Always, Semver());
    REQUIRE(restored.first != nullptr);
    const auto &restored_config = bundle.prints.get_edited_preset().config;
    check_bridge_controls(restored_config, legacy ? 100. : 67.5, legacy ? 15. : 42.5, legacy ? 1 : 3);
    REQUIRE(restored_config.option<ConfigOptionStrings>("print_extruder_variant")->size() == 3);
    check_bridge_controls(bundle.prints.find_preset(name, false)->config, 100., 15., 1);
}
