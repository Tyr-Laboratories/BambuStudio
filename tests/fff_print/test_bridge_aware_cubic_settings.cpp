#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"

#include "libslic3r/Fill/BridgeAwareCubicPlan.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"

#include <algorithm>
#include <set>
#include <fstream>
#include <string>

using namespace Slic3r;
using namespace BACFixtures;

// S1: option plumbing. The gap is a scalar region option with default 5 in the print options; the pattern is a value of
// sparse_infill_pattern and not of the top surface patterns (which is what refuses it at 100% density, as Cubic is refused).
TEST_CASE("Bridge aware cubic option is a scalar region option with default 5 in the print options", "[BridgeAwareCubic][S1]")
{
    DynamicPrintConfig config = region_defaults();
    check_gap(config, 5.);
    const ConfigOptionDef *def = print_config_def.get(gap_key);
    REQUIRE(def != nullptr);
    CHECK(def->min == 0.01);
    CHECK(def->max == 1000.);
    CHECK(def->category == "Strength");
    CHECK(std::count(Preset::print_options().begin(), Preset::print_options().end(), gap_key) == 1);
    const auto &sparse = print_config_def.get("sparse_infill_pattern")->enum_values;
    const auto &top = print_config_def.get("top_surface_pattern")->enum_values;
    CHECK(std::count(sparse.begin(), sparse.end(), "bridgeawarecubic") == 1);
    CHECK(std::count(top.begin(), top.end(), "bridgeawarecubic") == 0);

    config.set_deserialize_strict({{"sparse_infill_pattern", "bridgeawarecubic"}, {gap_key, "3.75"}, {"sparse_infill_density", "5%"}});
    CHECK(config.opt_serialize("sparse_infill_pattern") == "bridgeawarecubic");
    CHECK(config.option<ConfigOptionEnum<InfillPattern>>("sparse_infill_pattern")->value == ipBridgeAwareCubic);
    check_gap(config, 3.75);
    CHECK(config.validate().count("sparse_infill_pattern") == 0);
    CHECK(config.validate().count(gap_key) == 0);
}

TEST_CASE("Bridge aware cubic option round-trips through a preset, a 3MF and a project", "[BridgeAwareCubic][S1]")
{
    Workspace workspace;
    DynamicPrintConfig config = with(region_defaults(), {{"sparse_infill_pattern", "bridgeawarecubic"}, {gap_key, "4.5"}});

    SECTION("a saved process preset reloads the pattern and the gap") {
        Preset preset(Preset::TYPE_PRINT, "Bridge aware cubic test");
        preset.file = (workspace.path / "process.json").string();
        preset.config = config;
        REQUIRE(preset.save(nullptr));
        Preset reopened(Preset::TYPE_PRINT, preset.name);
        reopened.file = preset.file;
        reopened.reload(Preset(Preset::TYPE_PRINT, "Default"));
        check_gap(reopened.config, 4.5);
        CHECK(reopened.config.opt_serialize("sparse_infill_pattern") == "bridgeawarecubic");
    }

    SECTION("a 3MF keeps the object, part and modifier overrides") {
        Model model;
        ModelObject *object = model.add_object();
        ModelVolume *part = object->add_volume(TriangleMesh(its_make_cube(10., 10., 10.)));
        ModelVolume *modifier = object->add_volume(TriangleMesh(its_make_cube(4., 4., 4.)));
        modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
        object->add_instance();
        object->config.set_key_value(gap_key, new ConfigOptionFloat(3.));
        part->config.set_key_value(gap_key, new ConfigOptionFloat(2.));
        modifier->config.set_key_value(gap_key, new ConfigOptionFloat(1.25));

        const std::string filename = (workspace.path / "bridge-aware-cubic.3mf").string();
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
        const bool loaded = load_bbs_3mf(filename.c_str(), &reopened_config, &substitutions, &reopened, &plates, &presets, &is_bambu,
                                         &version, nullptr, LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
        for (auto *plate : plates) delete plate;
        for (auto *saved : presets) delete saved;
        REQUIRE(loaded);
        CHECK(reopened_config.opt_serialize("sparse_infill_pattern") == "bridgeawarecubic");
        check_gap(reopened_config, 4.5);
        REQUIRE(reopened.objects.size() == 1);
        const ModelObject *reopened_object = reopened.objects.front();
        REQUIRE(reopened_object->volumes.size() == 2);
        check_gap(reopened_object->config.get(), 3.);
        check_gap(reopened_object->volumes.front()->config.get(), 2.);
        REQUIRE(reopened_object->volumes.back()->is_modifier());
        check_gap(reopened_object->volumes.back()->config.get(), 1.25);
    }

    SECTION("a project's process settings restore the gap, and an older project without it gets the default") {
        PresetBundle bundle;
        const auto profiles = boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() / "resources" / "profiles";
        bundle.load_vendor_configs_from_json(profiles.string(), "BBL", PresetBundle::LoadSystem, ForwardCompatibilitySubstitutionRule::Disable);
        bundle.prints.update_user_presets_directory(workspace.path.string(), "process");
        const std::string name = "0.20mm Standard @BBL H2S";
        const Preset *base = bundle.prints.find_preset(name, false);
        REQUIRE(base != nullptr);
        check_gap(base->config, 5.);
        DynamicPrintConfig saved = base->config;
        saved.set_deserialize_strict({{"inherits", name}, {"sparse_infill_pattern", "bridgeawarecubic"}, {gap_key, "3.25"}});
        bool legacy = false;
        SECTION("with the gap") {}
        SECTION("without the gap") {
            legacy = true;
            saved.erase(gap_key);
            saved.set_deserialize_strict({{"sparse_infill_pattern", "cubic"}});
        }
        const auto restored = bundle.prints.load_external_preset("bridge-aware-cubic-test.3mf", "Bridge aware cubic project",
            "Custom H2S bridge aware process", saved, {"sparse_infill_pattern", gap_key}, PresetCollection::LoadAndSelect::Always, Semver());
        REQUIRE(restored.first != nullptr);
        const DynamicPrintConfig &restored_config = bundle.prints.get_edited_preset().config;
        check_gap(restored_config, legacy ? 5. : 3.25);
        CHECK(restored_config.opt_serialize("sparse_infill_pattern") == (legacy ? "cubic" : "bridgeawarecubic"));
        check_gap(bundle.prints.find_preset(name, false)->config, 5.);
    }
}

TEST_CASE("Bridge aware cubic gap change invalidates prepare infill and modifier_ignore_infill restores it", "[BridgeAwareCubic][S1]")
{
    SECTION("a gap change invalidates the infill preparation and keeps the perimeters") {
        Print print;
        Model model = small_cube();
        slice(print, model, fixture_config(5., "bridgeawarecubic"));
        REQUIRE(print.get_object(size_t(0))->is_step_done(posPrepareInfill));
        print.apply(model, fixture_config(6., "bridgeawarecubic"));
        CHECK_FALSE(print.get_object(size_t(0))->is_step_done(posPrepareInfill));
        CHECK(print.get_object(size_t(0))->is_step_done(posPerimeters));
        print.apply(model, fixture_config(5., "bridgeawarecubic"));
        print.process();
        CHECK(print.get_object(size_t(0))->is_step_done(posInfill));
    }

    SECTION("modifier_ignore_infill restores the parent's gap with the other infill keys") {
        for (const bool ignore : {false, true}) {
            CAPTURE(ignore);
            Model model = modifier();
            model.objects.front()->volumes.back()->config.set_key_value("modifier_ignore_infill", new ConfigOptionBool(ignore));
            Print print;
            print.set_status_silent();
            print.auto_assign_extruders(model.objects.front());
            print.apply(model, modifier_config("bridgeawarecubic"));
            const PrintObject &object = *print.get_object(size_t(0));
            size_t regions_at_3 = 0;
            for (size_t i = 0; i < object.num_printing_regions(); ++i) {
                const PrintRegionConfig &region = object.printing_region(i).config();
                if (region.bridge_aware_cubic_max_gap.value == 3.) ++regions_at_3;
                if (ignore) {
                    CHECK(region.bridge_aware_cubic_max_gap.value == 8.);
                    CHECK(region.sparse_infill_density.value == 5.);
                }
            }
            CHECK((regions_at_3 > 0) == ! ignore);
        }
    }
}

// The modifier fixture with the modifier changing the gap only: one lattice key, two gaps, so one group and no seam. (The
// two-lattice modifier carries a seam wall on every layer where the lattices meet, roof or none, by 3.5a; W1 covers it.)
static Model gap_only_modifier()
{
    Model model = modifier();
    ModelVolume *right = model.objects.front()->volumes.back();
    right->config.erase("sparse_infill_density");
    right->config.erase("infill_direction");
    return model;
}

// S2 (open top): where nothing needs refinement, the pattern's fills equal Cubic's as deposited segments within 0.001 mm. A
// part without top shells has no roof over its sparse fill, so the plan has nothing and the group prints as Cubic. The enum's
// position reorders the fill groups and moves their shared edges by Clipper's safety offset, nothing more.
TEST_CASE("Bridge aware cubic prints as Cubic where no roof needs refinement", "[BridgeAwareCubic][S2]")
{
    struct Fixture { const char *name; Model (*model)(); DynamicPrintConfig (*config)(const char *); };
    const Fixture fixtures[] = {
        { "open-top small cube", small_cube, [](const char *p) { return with(fixture_config(5., p), {{"top_shell_layers", "0"}}); } },
        // Not the rib itself: the vertical shells under its ribs are roofs over sparse fill, so its plan has content.
        { "open-top cube with the rib's settings", small_cube, [](const char *p) { return with(rib_config(p), {{"top_shell_layers", "0"}}); } },
        // Not the two-lattice modifier: two lattices meet through a seam wall on every layer (3.5a), roof or none.
        { "open-top modifier changing the gap only", gap_only_modifier, [](const char *p) { return with(modifier_config(p), {{"top_shell_layers", "0"}}); } },
        { "open-top small cube, two lines", small_cube, [](const char *p) { return with(fixture_config(5., p), {{"top_shell_layers", "0"}, {"fill_multiline", "2"}}); } },
    };
    for (const Fixture &fixture : fixtures) {
        INFO(fixture.name);
        Print bac, cubic;
        Model model_bac = fixture.model(), model_cubic = fixture.model();
        slice(bac, model_bac, fixture.config("bridgeawarecubic"));
        slice(cubic, model_cubic, fixture.config("cubic"));
        REQUIRE(sparse_length(cubic) > 0.);
        std::string message;
        const bool same = same_fill_paths(fill_paths(bac), fill_paths(cubic), 0.001, message);
        INFO(message);
        CHECK(same);
    }
}

// S6: setting invariance. Choosing the pattern changes no other setting: the region configs after processing equal Cubic's but
// for the pattern and the gap; arc fitting still fits arcs; an explicit bridge angle reaches every internal bridge; solid fills
// merge as Cubic's away from the plan's layers.
// A 40 x 20 x 10 mm tunnel: two legs 10 mm wide and a 4 mm slab whose underside spans the 20 mm between them over air (an external
// bridge).
Model tunnel()
{
    TriangleMesh body(its_make_cube(10., 20., 6.));
    TriangleMesh right(its_make_cube(10., 20., 6.));
    right.translate(30., 0., 0.);
    TriangleMesh slab(its_make_cube(40., 20., 4.));
    slab.translate(0., 0., 6.);
    body.merge(right);
    body.merge(slab);
    return single_part("Bridge aware cubic tunnel", std::move(body));
}

TEST_CASE("Bridge aware cubic changes no other setting of the part", "[BridgeAwareCubic][S6]")
{
    SECTION("the print, object and region configs equal Cubic's but for the pattern and the gap") {
        Print bac, cubic;
        Model model_bac = modifier(), model_cubic = modifier();
        slice(bac, model_bac, modifier_config("bridgeawarecubic"));
        slice(cubic, model_cubic, modifier_config("cubic"));
        const auto compare = [](const ConfigBase &ca, const ConfigBase &cb, const std::string &what) {
            for (const std::string &key : ca.keys()) {
                if (key == "sparse_infill_pattern" || key == gap_key)
                    continue;
                INFO(what << ": " << key);
                const ConfigOption *oa = ca.option(key), *ob = cb.option(key);
                REQUIRE(oa != nullptr);
                REQUIRE(ob != nullptr);
                CHECK(*oa == *ob);
            }
        };
        compare(bac.config(), cubic.config(), "print");
        const PrintObject &a = *bac.get_object(0), &b = *cubic.get_object(0);
        compare(a.config(), b.config(), "object");
        REQUIRE(a.num_printing_regions() == b.num_printing_regions());
        for (size_t r = 0; r < a.num_printing_regions(); ++r)
            compare(a.printing_region(r).config(), b.printing_region(r).config(), "region " + std::to_string(r));
    }
    SECTION("arc fitting fits arcs as with Cubic, and none when it is off") {
        // A cylinder, whose walls are arcs for either pattern (a box gives Cubic nothing to fit), with a roof for the plan.
        Workspace workspace;
        const auto extruding_arcs = [&workspace](const char *pattern, bool fit, const std::string &name) {
            Print print;
            Model model = single_part("Bridge aware cubic cylinder", TriangleMesh(its_make_cylinder(15., 18.)));
            slice(print, model, with(fixture_config(5., pattern), {{"enable_arc_fitting", fit ? "1" : "0"}}));
            std::ifstream file(print.export_gcode((workspace.path / name).string(), nullptr));
            std::string line;
            size_t arcs = 0;
            while (std::getline(file, line))
                if ((line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0) && line.find(" E") != std::string::npos)
                    ++arcs;
            return arcs;
        };
        CHECK(extruding_arcs("bridgeawarecubic", true, "s6-arcs.gcode") > 0);
        CHECK(extruding_arcs("cubic", true, "s6-arcs-cubic.gcode") > 0);
        CHECK(extruding_arcs("bridgeawarecubic", false, "s6-no-arcs.gcode") == 0);
    }
    SECTION("an explicit bridge angle reaches the external bridges as with Cubic") {
        // The stock pipeline reads `bridge_angle` for the bridges over air (and overhang walls); an internal bridge takes the angle
        // the generator derives from the anchors below, which spec 1.2 lets follow the pattern's lines.
        const auto angles = [](const char *pattern) {
            Print print;
            Model model = tunnel();
            slice(print, model, with(fixture_config(5., pattern), {{"bridge_angle", "30"}}));
            std::vector<double> out;
            for (const Layer *layer : print.get_object(0)->layers())
                for (const LayerRegion *region : layer->regions())
                    for (const Surface &surface : region->fill_surfaces.surfaces)
                        if (surface.surface_type == stBottomBridge)
                            out.push_back(surface.bridge_angle);
            return out;
        };
        for (const char *pattern : { "bridgeawarecubic", "cubic" }) {
            INFO(pattern);
            const std::vector<double> found = angles(pattern);
            CHECK_FALSE(found.empty());
            for (const double angle : found)
                CHECK(angle == Approx(Geometry::deg2rad(30.)).margin(1e-6));
        }
    }
    SECTION("solid fills as Cubic's away from the layers next to its internal bridges") {
        // Next to an internal bridge the stock pipeline answers the pattern's lines (the generator anchors on them, 1.2); elsewhere
        // the solid surfaces of every layer equal Cubic's, area within the outline's length times the resolution.
        Print bac, cubic;
        Model model_bac = small_cube(), model_cubic = small_cube();
        const DynamicPrintConfig settings = fixture_config(5., "bridgeawarecubic");
        slice(bac, model_bac, settings);
        slice(cubic, model_cubic, fixture_config(5., "cubic"));
        const PrintObject &a = *bac.get_object(0), &b = *cubic.get_object(0);
        REQUIRE(a.layers().size() == b.layers().size());
        const double resolution = settings.opt_float("resolution");
        const auto solid = [](const Layer &layer, SurfaceType type) {
            double area = 0., outline = 0.;
            for (const LayerRegion *region : layer.regions())
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == type) {
                        area += surface.expolygon.area() * SCALING_FACTOR * SCALING_FACTOR;
                        for (const Polygon &polygon : to_polygons(surface.expolygon))
                            outline += unscale<double>(polygon.length());
                    }
            return std::make_pair(area, outline);
        };
        std::set<size_t> near_bridges;
        for (size_t l = 0; l < b.layers().size(); ++l)
            if (solid(*b.layers()[l], stInternalBridge).first > 0.)
                for (size_t k = l > 0 ? l - 1 : 0; k <= l + 1; ++k)
                    near_bridges.insert(k);
        REQUIRE_FALSE(near_bridges.empty());
        size_t compared = 0;
        for (size_t l = 0; l < a.layers().size(); ++l) {
            if (near_bridges.count(l))
                continue;
            for (const SurfaceType type : { stInternalSolid, stTop, stBottom }) {
                const auto mine = solid(*a.layers()[l], type), theirs = solid(*b.layers()[l], type);
                INFO("layer " << l << " type " << int(type));
                CHECK(std::abs(mine.first - theirs.first) <= std::max(mine.second, theirs.second) * resolution);
                compared += theirs.first > 0.;
            }
        }
        CHECK(compared > 3);
    }
}

