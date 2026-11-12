#include <catch2/catch.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>

using namespace Slic3r;

namespace {
struct Workspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("wall-extension-config-%%%%-%%%%");
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
}

TEST_CASE("Wall internal extension defaults to 1 mm and persists in presets and projects", "[WallExtension][Config]")
{
    Workspace workspace;
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    REQUIRE(config.has("wall_internal_extension"));
    CHECK(config.opt_float("wall_internal_extension") == 1.);
    REQUIRE(std::count(Preset::print_options().begin(), Preset::print_options().end(), "wall_internal_extension") == 1);

    config.set_deserialize_strict({{"wall_internal_extension", "2.5"}});
    Preset preset(Preset::TYPE_PRINT, "Wall extension test");
    preset.file = (workspace.path / "process.json").string();
    preset.config = config;
    REQUIRE(preset.save(nullptr));
    Preset reopened_preset(Preset::TYPE_PRINT, preset.name);
    reopened_preset.file = preset.file;
    reopened_preset.reload(Preset(Preset::TYPE_PRINT, "Default"));
    CHECK(reopened_preset.config.opt_float("wall_internal_extension") == 2.5);

    Model model;
    auto *object = model.add_object();
    auto *part = object->add_volume(TriangleMesh(its_make_cube(10., 10., 10.)));
    object->add_instance();
    object->config.set("wall_internal_extension", 0.5);
    part->config.set("wall_internal_extension", 0.);
    const auto filename = (workspace.path / "wall-extension.3mf").string();
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
    CHECK(reopened_config.opt_float("wall_internal_extension") == 2.5);
    REQUIRE(reopened.objects.size() == 1);
    REQUIRE(reopened.objects.front()->volumes.size() == 1);
    CHECK(reopened.objects.front()->config.opt_float("wall_internal_extension") == 0.5);
    CHECK(reopened.objects.front()->volumes.front()->config.opt_float("wall_internal_extension") == 0.);
}

TEST_CASE("Changing wall internal extension regenerates walls but keeps the slices", "[WallExtension][Config]")
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"brim_type", "no_brim"}, {"enable_arc_fitting", "0"},
        {"wall_internal_extension", "1"}
    });
    Model model;
    auto *object = model.add_object();
    object->add_volume(TriangleMesh(its_make_cube(20., 20., 5.)));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));

    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();
    REQUIRE(print.get_object(size_t(0))->is_step_done(posPerimeters));

    REQUIRE(print.get_object(size_t(0))->is_step_done(posSupportMaterial));

    config.set_deserialize_strict({{"wall_internal_extension", "2"}});
    print.apply(model, config);
    const auto *object_after = print.get_object(size_t(0));
    CHECK(object_after->is_step_done(posSlice));
    CHECK_FALSE(object_after->is_step_done(posPerimeters));
    // Support reads bridge areas, which the extension's cut-outs can change.
    CHECK_FALSE(object_after->is_step_done(posSupportMaterial));
}

TEST_CASE("A modifier that only changes wall internal extension adds no walls", "[WallExtension][Config]")
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"}, {"wall_loops", "1"},
        {"wall_generator", "classic"}, {"brim_type", "no_brim"}, {"enable_arc_fitting", "0"},
        {"wall_internal_extension", "1"}
    });
    Model model;
    auto *object = model.add_object();
    object->add_volume(TriangleMesh(its_make_cube(20., 20., 5.)));
    // Covers the left half of the box; differs from the box only in wall_internal_extension.
    TriangleMesh left_half(its_make_cube(12., 24., 7.));
    left_half.translate(-2., -2., -1.);
    auto *modifier = object->add_volume(std::move(left_half));
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    modifier->config.set("wall_internal_extension", 0.);
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));

    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();

    // One outer wall around the box: the modifier's border gets no walls and the outer wall is not split there.
    for (const Layer *layer : print.get_object(size_t(0))->layers()) {
        if (std::abs(layer->print_z - 2.4) > EPSILON)
            continue;
        double outer_wall = 0.;
        for (const LayerRegion *region : layer->regions())
            for (const ExtrusionEntity *island : region->perimeters.entities)
                for (const ExtrusionEntity *entity : static_cast<const ExtrusionEntityCollection *>(island)->entities)
                    if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity))
                        for (const ExtrusionPath &path : loop->paths)
                            if (path.role() == erExternalPerimeter)
                                outer_wall += unscale<double>(path.polyline.length());
        CHECK(outer_wall == Approx(4. * (20. - 0.45)).margin(1.));
    }
}
