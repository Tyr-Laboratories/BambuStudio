#include <catch2/catch.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"

#include <algorithm>
#include <limits>
#include <boost/filesystem.hpp>

using namespace Slic3r;

namespace {

constexpr const char *gap_key = "bridge_aware_cubic_max_gap";

struct BridgeAwareConfigWorkspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("bridge-aware-cubic-config-%%%%-%%%%");
    std::string old_data = data_dir(), old_temp = temporary_dir();

    BridgeAwareConfigWorkspace()
    {
        boost::filesystem::create_directories(path);
        set_data_dir(path.string());
        set_temporary_dir(path.string());
    }

    ~BridgeAwareConfigWorkspace()
    {
        set_data_dir(old_data);
        set_temporary_dir(old_temp);
        boost::system::error_code ec;
        boost::filesystem::remove_all(path, ec);
    }
};

void check_gap(const ConfigBase &config, double expected)
{
    const auto *gap = config.option(gap_key);
    REQUIRE(gap != nullptr);
    REQUIRE(gap->type() == coFloat);
    CHECK(config.option<ConfigOptionFloat>(gap_key)->value == expected);
}

DynamicPrintConfig region_defaults()
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    return config;
}

} // namespace

TEST_CASE("Bridge aware cubic persists as a scalar region option with a compatible default",
          "[BridgeAwareCubic][Config]")
{
    auto config = region_defaults();
    check_gap(config, 5.);
    CHECK(config.opt_serialize("sparse_infill_pattern") == "cubic");
    CHECK(std::count(Preset::print_options().begin(), Preset::print_options().end(), gap_key) == 1);

    config.set_deserialize_strict({{"sparse_infill_pattern", "bridgeawarecubic"}, {gap_key, "3.75"}});
    CHECK(config.opt_serialize("sparse_infill_pattern") == "bridgeawarecubic");
    check_gap(config, 3.75);
    CHECK(config.validate().count("sparse_infill_pattern") == 0);
    CHECK(config.validate().count(gap_key) == 0);
}

TEST_CASE("Bridge aware cubic rejects gaps outside finite representable distances",
          "[BridgeAwareCubic][Config]")
{
    auto config = region_defaults();
    REQUIRE(config.option(gap_key) != nullptr);
    for (double gap : {0., -1., 1e-12, std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
        CAPTURE(gap);
        config.set_key_value(gap_key, new ConfigOptionFloat(gap));
        CHECK(config.validate().count(gap_key) == 1);
    }
    for (double gap : {0.01, 0.5, 5., 75.}) {
        CAPTURE(gap);
        config.set_key_value(gap_key, new ConfigOptionFloat(gap));
        CHECK(config.validate().count(gap_key) == 0);
    }
}

TEST_CASE("Bridge aware cubic rejects active multiline without rewriting it",
          "[BridgeAwareCubic][Config]")
{
    auto config = region_defaults();
    config.set_deserialize_strict({{"sparse_infill_pattern", "bridgeawarecubic"},
                                   {"sparse_infill_density", "5%"}, {"fill_multiline", "2"}});
    CHECK(config.validate().count("fill_multiline") == 1);
    CHECK(config.opt_int("fill_multiline") == 2);
    config.set_deserialize_strict({{"fill_multiline", "1"}});
    CHECK(config.validate().count("fill_multiline") == 0);

    for (const auto *density : {"0%", "100%"}) {
        CAPTURE(density);
        config.set_deserialize_strict({{"sparse_infill_density", density}, {"fill_multiline", "2"}});
        const auto errors = config.validate();
        CHECK(errors.count("fill_multiline") == 0);
        CHECK(errors.count("sparse_infill_pattern") == 0);
    }
    config.set_deserialize_strict({{"sparse_infill_pattern", "cubic"}, {"sparse_infill_density", "5%"}});
    CHECK(config.validate().count("fill_multiline") == 0);
}

TEST_CASE("Bridge aware cubic process and project overrides survive real persistence",
          "[BridgeAwareCubic][Config][Persistence]")
{
    BridgeAwareConfigWorkspace workspace;
    auto config = region_defaults();
    config.set_deserialize_strict({{"sparse_infill_pattern", "bridgeawarecubic"}, {gap_key, "4.5"}});
    Preset preset(Preset::TYPE_PRINT, "Bridge aware cubic test");
    preset.file = (workspace.path / "process.json").string();
    preset.config = config;
    REQUIRE(preset.save(nullptr));
    Preset reopened_preset(Preset::TYPE_PRINT, preset.name);
    reopened_preset.file = preset.file;
    reopened_preset.reload(Preset(Preset::TYPE_PRINT, "Default"));
    check_gap(reopened_preset.config, 4.5);
    CHECK(reopened_preset.config.opt_serialize("sparse_infill_pattern") == "bridgeawarecubic");

    Model model;
    auto *object = model.add_object();
    auto *part = object->add_volume(TriangleMesh(its_make_cube(10., 10., 10.)));
    auto *modifier = object->add_volume(TriangleMesh(its_make_cube(4., 4., 4.)));
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    object->add_instance();
    object->config.set_key_value(gap_key, new ConfigOptionFloat(3.));
    part->config.set_key_value(gap_key, new ConfigOptionFloat(2.));
    modifier->config.set_key_value(gap_key, new ConfigOptionFloat(1.25));

    const auto filename = (workspace.path / "bridge-aware-cubic.3mf").string();
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
    CHECK(reopened_config.opt_serialize("sparse_infill_pattern") == "bridgeawarecubic");
    check_gap(reopened_config, 4.5);
    REQUIRE(reopened.objects.size() == 1);
    const auto *reopened_object = reopened.objects.front();
    REQUIRE(reopened_object->volumes.size() == 2);
    check_gap(reopened_object->config.get(), 3.);
    check_gap(reopened_object->volumes.front()->config.get(), 2.);
    REQUIRE(reopened_object->volumes.back()->is_modifier());
    check_gap(reopened_object->volumes.back()->config.get(), 1.25);

    DynamicPrintConfig effective = reopened_config;
    effective.apply(reopened_object->config.get());
    check_gap(effective, 3.);
    effective.apply(reopened_object->volumes.front()->config.get());
    check_gap(effective, 2.);
    effective.apply(reopened_object->volumes.back()->config.get());
    check_gap(effective, 1.25);
    CHECK(effective.opt_serialize("sparse_infill_pattern") == "bridgeawarecubic");
}

TEST_CASE("Bridge aware cubic restores a scalar gap across H2S nozzle variants",
          "[BridgeAwareCubic][Config][ProfileRestore]")
{
    BridgeAwareConfigWorkspace workspace;
    PresetBundle bundle;
    const auto profiles = boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() / "resources" / "profiles";
    bundle.load_vendor_configs_from_json(profiles.string(), "BBL", PresetBundle::LoadSystem,
                                        ForwardCompatibilitySubstitutionRule::Disable);
    bundle.prints.update_user_presets_directory(workspace.path.string(), "process");
    const std::string name = "0.20mm Standard @BBL H2S";
    const auto *base = bundle.prints.find_preset(name, false);
    REQUIRE(base != nullptr);
    check_gap(base->config, 5.);
    auto saved = base->config;
    saved.set_deserialize_strict({{"inherits", name}, {"sparse_infill_pattern", "bridgeawarecubic"}, {gap_key, "3.25"}});
    bool legacy = false;
    SECTION("Project restoration retains the scalar gap") {}
    SECTION("Reordered nozzle variants retain the scalar gap") {
        saved.set_deserialize_strict({
            {"print_extruder_variant", "Direct Drive E3D High Flow;Direct Drive Standard;Direct Drive High Flow"}
        });
    }
    SECTION("Older projects retain the existing pattern and get an inactive default gap") {
        legacy = true;
        saved.erase(gap_key);
        saved.set_deserialize_strict({{"sparse_infill_pattern", "cubic"}});
    }

    const auto restored = bundle.prints.load_external_preset("bridge-aware-cubic-test.3mf", "Bridge aware cubic project",
        "Custom H2S bridge aware process", saved, {"sparse_infill_pattern", gap_key},
        PresetCollection::LoadAndSelect::Always, Semver());
    REQUIRE(restored.first != nullptr);
    const auto &restored_config = bundle.prints.get_edited_preset().config;
    check_gap(restored_config, legacy ? 5. : 3.25);
    CHECK(restored_config.opt_serialize("sparse_infill_pattern") == (legacy ? "cubic" : "bridgeawarecubic"));
    REQUIRE(restored_config.option<ConfigOptionStrings>("print_extruder_variant")->size() == 3);
    check_gap(bundle.prints.find_preset(name, false)->config, 5.);
}
