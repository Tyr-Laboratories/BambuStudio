#include <catch2/catch.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include <boost/filesystem.hpp>

using namespace Slic3r;

namespace {
struct SlopeConfigWorkspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("slope-wall-config-%%%%-%%%%");
    std::string old_data = data_dir(), old_temp = temporary_dir();
    SlopeConfigWorkspace() {
        boost::filesystem::create_directories(path);
        set_data_dir(path.string());
        set_temporary_dir(path.string());
    }
    ~SlopeConfigWorkspace() {
        set_data_dir(old_data);
        set_temporary_dir(old_temp);
        boost::system::error_code ec;
        boost::filesystem::remove_all(path, ec);
    }
};
}

TEST_CASE("Slope wall angle retains scalar process and model overrides", "[SlopeWalls][Config]")
{
    SlopeConfigWorkspace workspace;
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    REQUIRE(config.has("extra_wall_slope_angle"));
    CHECK(config.opt_float("extra_wall_slope_angle") == 0.);
    REQUIRE(std::count(Preset::print_options().begin(), Preset::print_options().end(), "extra_wall_slope_angle") == 1);
    config.set_deserialize_strict({{"extra_wall_slope_angle", "35.5"}});
    Preset preset(Preset::TYPE_PRINT, "Slope wall test");
    preset.file = (workspace.path / "process.json").string();
    preset.config = config;
    REQUIRE(preset.save(nullptr));
    Preset reopened_preset(Preset::TYPE_PRINT, preset.name);
    reopened_preset.file = preset.file;
    reopened_preset.reload(Preset(Preset::TYPE_PRINT, "Default"));
    CHECK(reopened_preset.config.opt_float("extra_wall_slope_angle") == 35.5);

    Model model;
    auto *object = model.add_object();
    auto *part = object->add_volume(TriangleMesh(its_make_cube(10., 10., 10.)));
    object->add_instance();
    object->config.set("extra_wall_slope_angle", 40.);
    part->config.set("extra_wall_slope_angle", 20.);
    const auto filename = (workspace.path / "sloped-walls.3mf").string();
    StoreParams params;
    params.path = filename.c_str(); params.model = &model; params.config = &config;
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
    CHECK(reopened_config.opt_float("extra_wall_slope_angle") == 35.5);
    REQUIRE(reopened.objects.size() == 1);
    REQUIRE(reopened.objects.front()->volumes.size() == 1);
    CHECK(reopened.objects.front()->config.opt_float("extra_wall_slope_angle") == 40.);
    CHECK(reopened.objects.front()->volumes.front()->config.opt_float("extra_wall_slope_angle") == 20.);
}

TEST_CASE("Slope wall angle restores with the H2S multi-variant process", "[SlopeWalls][Config][ProfileRestore]")
{
    SlopeConfigWorkspace workspace;
    PresetBundle bundle;
    const auto profiles = boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() / "resources" / "profiles";
    bundle.load_vendor_configs_from_json(profiles.string(), "BBL", PresetBundle::LoadSystem,
                                        ForwardCompatibilitySubstitutionRule::Disable);
    bundle.prints.update_user_presets_directory(workspace.path.string(), "process");
    const std::string name = "0.20mm Standard @BBL H2S";
    const auto *base = bundle.prints.find_preset(name, false);
    REQUIRE(base != nullptr);
    REQUIRE(base->config.option<ConfigOptionStrings>("print_extruder_variant")->size() == 3);
    auto saved = base->config;
    saved.set_deserialize_strict({{"inherits", name}, {"extra_wall_slope_angle", "35.5"}});
    const auto restored = bundle.prints.load_external_preset("slope-test.3mf", "Slope wall project", "Custom H2S process",
        saved, {"extra_wall_slope_angle"}, PresetCollection::LoadAndSelect::Always, Semver());
    REQUIRE(restored.first != nullptr);
    CHECK(bundle.prints.get_edited_preset().config.opt_float("extra_wall_slope_angle") == 35.5);
    CHECK(bundle.prints.get_edited_preset().config.option("extra_wall_slope_angle")->type() == coFloat);
    CHECK(bundle.prints.find_preset(name, false)->config.opt_float("extra_wall_slope_angle") == 0.);
}
