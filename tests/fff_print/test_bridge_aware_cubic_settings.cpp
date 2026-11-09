#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"
#include "bridge_aware_cubic_observer.hpp"

#include "libslic3r/MeshBoolean.hpp"
#include "libslic3r/Preset.hpp"

#include <algorithm>
#include <cstring>
#include <set>
#include <fstream>
#include <string>

using namespace Slic3r;
using namespace BACFixtures;
using namespace BACObserver;

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
    struct Fixture { const char *name; Model (*model)(); DynamicPrintConfig (*config)(const char *); bool sparse = true; };
    const Fixture fixtures[] = {
        // Settings the pattern honours as Cubic does (3.9): density under the stock limit, conformal infill (Cubic ignores it), a
        // raft, spiral vase (no sparse infill at all), 100 % (no plan: the fill is Cubic's), with and without combination.
        { "open-top small cube at 1 %", small_cube, [](const char *p) { return with(fixture_config(5., p), {{"top_shell_layers", "0"}, {"sparse_infill_density", "1%"}}); } },
        { "open-top small cube, conformal infill", small_cube, [](const char *p) { return with(fixture_config(5., p), {{"top_shell_layers", "0"}, {"conformal_infill", "1"}}); } },
        { "open-top small cube on a raft", small_cube, [](const char *p) { return with(fixture_config(5., p), {{"top_shell_layers", "0"}, {"raft_layers", "3"}}); } },
        { "small cube in spiral vase mode", small_cube, [](const char *p) { return with(fixture_config(5., p), {{"spiral_mode", "1"}, {"wall_loops", "1"}, {"top_shell_layers", "0"}, {"sparse_infill_density", "0%"}}); }, false },
        { "small cube at 100 %", small_cube, [](const char *p) { return with(fixture_config(5., p), {{"sparse_infill_density", "100%"}}); }, false },
        { "small cube at 100 %, combined", small_cube, [](const char *p) { return with(fixture_config(5., p), {{"sparse_infill_density", "100%"}, {"infill_combination", "1"}}); }, false },
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
        if (fixture.sparse)
            REQUIRE(sparse_length(cubic) > 0.);
        std::string message;
        const bool same = same_fill_paths(fill_paths(bac), fill_paths(cubic), 0.001, message);
        INFO(message);
        CHECK(same);
    }
    // At 100 % the configuration check says of the pattern what it says of Cubic (neither is a solid pattern).
    const auto at_full = [](const char *pattern) {
        FullPrintConfig config;
        config.apply(fixture_config(5., pattern), true);
        config.sparse_infill_density.value = 100.;
        std::map<std::string, std::string> errors = validate(config);
        auto it = errors.find("sparse_infill_pattern");
        std::string message = it == errors.end() ? std::string() : it->second;
        if (const size_t at = message.find(pattern); at != std::string::npos)
            message.replace(at, std::strlen(pattern), "<pattern>");
        return message;
    };
    CHECK_FALSE(at_full("cubic").empty());
    CHECK(at_full("bridgeawarecubic") == at_full("cubic"));
}

// S6: setting invariance. Choosing the pattern changes no other setting: the print, object and region configs after processing
// equal Cubic's but for the pattern and the gap, and arc fitting still fits arcs.
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
}

// A block 40 x 40 x 20 mm with a pocket 5 mm deep in its roof, a triangle with corners of 45, 90 and 45 degrees: the floor of the
// pocket is a top surface, whose solid layers below poke the 45 degree corners into the sparse fill.
Model pocket_block()
{
    TriangleMesh block(its_make_cube(40., 40., 20.));
    indexed_triangle_set prism;
    const std::vector<Vec2d> triangle { { 13., 6. }, { 27., 20. }, { 13., 34. } };
    for (const double z : { 15., 21. })
        for (const Vec2d &corner : triangle)
            prism.vertices.emplace_back(float(corner.x()), float(corner.y()), float(z));
    prism.indices = { { 0, 2, 1 }, { 3, 4, 5 }, { 0, 1, 4 }, { 0, 4, 3 }, { 1, 2, 5 }, { 1, 5, 4 }, { 2, 0, 3 }, { 2, 3, 5 } };
    MeshBoolean::cgal::minus(block, TriangleMesh(std::move(prism)));
    return single_part("Bridge aware cubic pocket block", std::move(block));
}

// A block 40 x 40 x 20 mm with a blind pocket 16 x 16 mm, 6 mm deep, in its roof: a roof at 14 mm under the pocket and a roof at
// 20 mm round it.
Model blind_cavity()
{
    TriangleMesh block(its_make_cube(40., 40., 20.));
    TriangleMesh pocket(its_make_cube(16., 16., 7.));
    pocket.translate(12., 12., 14.);
    MeshBoolean::cgal::minus(block, pocket);
    return single_part("Bridge aware cubic blind cavity", std::move(block));
}

// S7: stress settings. The slice completes, every sparse line stays inside the area the stock pipeline gives the infill (the
// measure's control: a bead one bead past the export's rightmost sparse vertex leaves it), and where the contract applies K1 holds:
// at the nozzle-floor gap K1 and K2 are a recorded limitation (docs/bridge-aware-cubic.md), under combination S3 measures K1 on the
// unfolded paths, and S6 reads arc fitting. The control of K1, on every row where Cubic's cells are wider than the gap: Cubic on
// the same part and settings opens wider than the gap somewhere.
TEST_CASE("Bridge aware cubic slices and holds K1 under stress settings", "[BridgeAwareCubic][S7]")
{
    Workspace workspace;
    struct Stress { const char *name; std::initializer_list<std::pair<const std::string, std::string>> settings; double gap; bool k1, control; };
    const std::vector<Stress> stresses = {
        { "gap at the nozzle", {}, 0.4, false, false },
        // The refinement saturates into solid columns, whose cells reach past the island at the walls: on the user's walls (one
        // Arachne wall the infill overlaps by 75 %) a line run on past the island leaves the part.
        { "gap at the nozzle, the user's walls at 3 %", {{"wall_loops", "1"}, {"wall_generator", "arachne"}, {"infill_wall_overlap", "75%"},
                                                         {"sparse_infill_density", "3%"}}, 0.4, false, false },
        { "60 %", {{"sparse_infill_density", "60%"}}, 5., true, false },
        { "0.003 %", {{"sparse_infill_density", "0.003%"}}, 5., true, true },
        { "gap 1000", {}, 1000., true, false },
        { "raft", {{"raft_layers", "3"}}, 5., true, true },
        { "the user's walls and bridges", {{"wall_loops", "1"}, {"wall_generator", "arachne"}, {"infill_wall_overlap", "75%"},
                                          {"detect_narrow_internal_solid_infill", "1"}, {"thick_bridges", "1"}}, 5., true, true },
        { "no walls", {{"wall_loops", "0"}}, 5., true, true },
        { "combination", {{"infill_combination", "1"}}, 5., false, false },
        { "a 0.6 mm sparse bead", {{"sparse_infill_line_width", "0.6"}}, 5., true, true },
    };
    struct Part { const char *name; Model (*model)(); };
    const Part parts[] = { { "small cube", small_cube }, { "pocket block", pocket_block }, { "blind cavity", blind_cavity } };
    for (const Part &part : parts)
        for (const Stress &stress : stresses) {
            INFO(part.name << ", " << stress.name);
            const DynamicPrintConfig settings = with(fixture_config(stress.gap, "bridgeawarecubic"), stress.settings);
            Print print;
            std::vector<EmittedPath> paths;
            REQUIRE_NOTHROW(paths = exported_extrusions(print, workspace, part.model(), settings, "s7.gcode"));
            CHECK_FALSE(paths.empty());
            const std::optional<Vec2d> outside = sparse_outside_fill_area(print, paths);
            INFO("a sparse path leaves the fill area at " << (outside ? *outside : Vec2d::Zero()).transpose());
            CHECK_FALSE(outside.has_value());
            // The measure's control: a bead from the rightmost sparse vertex one bead further out leaves the fill area.
            const EmittedPath *rightmost = nullptr;
            for (const EmittedPath &path : paths)
                if (path.role == erInternalInfill && (rightmost == nullptr || get_extents(path.path).max.x() > get_extents(rightmost->path).max.x()))
                    rightmost = &path;
            if (rightmost != nullptr) {
                const Point far = *std::max_element(rightmost->path.points.begin(), rightmost->path.points.end(), [](const Point &a, const Point &b) { return a.x() < b.x(); });
                std::vector<EmittedPath> injected = paths;
                injected.push_back(*rightmost);
                injected.back().path = Polyline(far, far + Point(coord_t(scale_(rightmost->width)), 0));
                CHECK(sparse_outside_fill_area(print, injected).has_value());
            }
            if (!stress.k1)
                continue;
            const std::vector<PlaneMeasure> planes = measure_planes(paths, 0.4, stress.gap);
            REQUIRE_FALSE(planes.empty());
            for (const PlaneMeasure &plane : planes) {
                INFO("plane below z = " << plane.lower / 1000. << " mm");
                CHECK(plane.widest_opening <= stress.gap);
                CHECK(plane.longest_run <= stress.gap);
            }
            if (stress.control) {
                const std::vector<PlaneMeasure> control = measure_planes(
                    exported_extrusions(workspace, part.model(), with(fixture_config(stress.gap, "cubic"), stress.settings), "s7-cubic.gcode"), 0.4, stress.gap);
                CHECK(std::any_of(control.begin(), control.end(), [&stress](const PlaneMeasure &p) { return p.widest_opening > stress.gap; }));
            }
        }
    SECTION("variable layer height") {
        Model model = small_cube();
        model.objects.front()->layer_height_profile.set(std::vector<coordf_t>{ 0., .2, 3., .2, 5., .12, 12., .12, 18., .24 });
        std::vector<EmittedPath> paths;
        REQUIRE_NOTHROW(paths = exported_extrusions(workspace, std::move(model), fixture_config(5., "bridgeawarecubic"), "s7-variable.gcode"));
        std::set<int> heights;
        for (const EmittedPath &path : paths)
            heights.insert(int(std::lround(path.height * 1000.)));
        CHECK(heights.size() >= 3);
        const std::vector<PlaneMeasure> planes = measure_planes(paths, 0.4, 5.);
        REQUIRE_FALSE(planes.empty());
        for (const PlaneMeasure &plane : planes) {
            INFO("plane below z = " << plane.lower / 1000. << " mm");
            CHECK(plane.widest_opening <= 5.);
            CHECK(plane.longest_run <= 5.);
        }
    }
}
