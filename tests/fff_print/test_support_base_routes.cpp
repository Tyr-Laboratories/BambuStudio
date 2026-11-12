#include <catch2/catch.hpp>

#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/TreeModelVolumes.hpp"
#include "libslic3r/Support/TreeSupport3D.hpp"
#include "libslic3r/Exception.hpp"

using namespace Slic3r;
using namespace Slic3r::TreeSupport3D;

namespace {
struct BaseRouteOptions {
    double platform_width = 20.;
    double bottom_gap = 0.2;
    int raft_layers = 0;
    bool barrier = false;
    double branch_diameter = 2.;
};

struct BaseRouteScene {
    Model model;
    Print print;

    BaseRouteScene(bool paint, bool plate_only = true, bool downward = false, BaseRouteOptions options = {})
    {
        ModelObject *object = model.add_object();
        auto *left = object->add_volume(TriangleMesh(its_make_cube(options.platform_width, 20., 4.)));
        TriangleMesh right_mesh(its_make_cube(20., 20., 4.));
        right_mesh.translate(50., 0., 0.);
        object->add_volume(right_mesh);
        // A separate tall part keeps slicing planes above both top surfaces.
        TriangleMesh mast(its_make_cube(2., 2., 16.));
        mast.translate(34., 0., 0.);
        object->add_volume(mast);
        if (options.barrier) {
            // A wall taller than the sampled route separates the paint from
            // the target, with no route around either end at this height.
            TriangleMesh barrier(its_make_cube(0.5, 120., 16.));
            barrier.translate(20.4, -60., 0.);
            object->add_volume(barrier);
        }
        if (paint) {
            const auto &mesh = left->mesh().its;
            for (size_t i = 0; i < mesh.indices.size(); ++i) {
                const auto &face = mesh.indices[i];
                const Vec3f &a = mesh.vertices[face[0]], &b = mesh.vertices[face[1]], &c = mesh.vertices[face[2]];
                const float nz = (b - a).cross(c - a).z();
                if (downward ? nz < 0.f : nz > 0.f)
                    left->support_base_facets.set_triangle_from_string(i, "4");
            }
        }
        object->add_instance()->set_offset(Vec3d(80., 80., 0.));
        DynamicPrintConfig config;
        config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
        config.set_deserialize_strict({
            {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
            {"enable_support", "1"}, {"support_type", "tree(auto)"},
            {"support_style", "tree_organic"}, {"support_on_build_plate_only", plate_only ? "1" : "0"},
            {"support_bottom_z_distance", std::to_string(options.bottom_gap)}, {"support_top_z_distance", "0.2"},
            {"raft_layers", std::to_string(options.raft_layers)},
            {"support_object_xy_distance", "0.35"}, {"tree_support_branch_angle", "30"},
            {"tree_support_branch_diameter", std::to_string(options.branch_diameter)}, {"brim_type", "no_brim"}
        });
        print.set_status_silent();
        print.auto_assign_extruders(object);
        print.apply(model, config);
        print.get_object(size_t(0))->slice();
    }

    PrintObject &object() { return *print.get_object(size_t(0)); }
    Point point(double x, double y)
    {
        const Vec3d transformed = object().trafo_centered() * Vec3d(x, y, 0.);
        return scaled<coord_t>(Vec2d(transformed.x(), transformed.y()));
    }
    LayerIndex layer(double print_z)
    {
        for (size_t i = 0; i < object().layer_count(); ++i)
            if (std::abs(object().get_layer(i)->print_z - print_z) < EPSILON)
                return LayerIndex(i);
        FAIL("Required slicing layer does not exist");
        return 0;
    }
    BuildVolume bed() { return BuildVolume({{-100., -100.}, {100., -100.}, {100., 100.}, {-100., 100.}}, 256., {}, {}); }
};
}

TEST_CASE("Organic support bases open only painted model contacts at the configured bottom gap", "[SupportBase][Routes]")
{
    BaseRouteScene scene(true);
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes volumes(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const auto base = scene.layer(4.4);
    const coord_t radius = scaled<coord_t>(0.5);
    const Point painted = scene.point(10., 10.);
    const Point unpainted = scene.point(60., 10.);
    CHECK(contains(volumes.getPlaceableAreas(radius, base, [] {}), painted));
    CHECK_FALSE(contains(volumes.getPlaceableAreas(radius, base, [] {}), unpainted));
    CHECK_FALSE(contains(volumes.getPlaceableAreas(radius, base - 1, [] {}), painted));
    CHECK_FALSE(contains(volumes.getPlaceableAreas(radius, base + 1, [] {}), painted));
    // The complete branch footprint has to fit within the painted platform.
    CHECK_FALSE(contains(volumes.getPlaceableAreas(radius, base, [] {}), scene.point(0.1, 10.)));
}

TEST_CASE("Painted base reachability is local and never originates at an unpainted bed edge", "[SupportBase][Routes]")
{
    BaseRouteScene scene(true);
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes volumes(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const coord_t radius = scaled<coord_t>(0.5);
    const auto high = scene.layer(6.);
    const auto &model_avoidance = volumes.getAvoidance(radius, high, TreeModelVolumes::AvoidanceType::Fast, true, true);
    CHECK_FALSE(contains(model_avoidance, scene.point(10., 10.)));
    CHECK(contains(model_avoidance, scene.point(60., 10.)));
    CHECK(contains(model_avoidance, Point(scaled<coord_t>(99.), coord_t(0))));
    // An unrelated bed path stays available; only the to-model route is restricted.
    const Point clear = scene.point(80., 30.);
    CHECK_FALSE(contains(volumes.getAvoidance(radius, high, TreeModelVolumes::AvoidanceType::Fast, false, true), clear));
    CHECK(contains(model_avoidance, clear));
}

TEST_CASE("Without build plate restriction support base painting does not restrict model contacts", "[SupportBase][Routes]")
{
    BaseRouteScene scene(true, false);
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes volumes(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const auto base = scene.layer(4.4);
    const coord_t radius = scaled<coord_t>(0.5);
    CHECK(contains(volumes.getPlaceableAreas(radius, base, [] {}), scene.point(10., 10.)));
    CHECK(contains(volumes.getPlaceableAreas(radius, base, [] {}), scene.point(60., 10.)));

    BaseRouteScene baseline(false, false);
    TreeModelVolumes baseline_volumes(baseline.object(), baseline.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    for (LayerIndex layer : {base, scene.layer(6.)}) {
        const Polygons &before = baseline_volumes.getPlaceableAreas(radius, layer, [] {});
        const Polygons &after = volumes.getPlaceableAreas(radius, layer, [] {});
        CHECK(diff(before, after).empty());
        CHECK(diff(after, before).empty());
        for (bool to_model : {false, true}) {
            const Polygons &route_before = baseline_volumes.getAvoidance(radius, layer, TreeModelVolumes::AvoidanceType::Fast, to_model, true);
            const Polygons &route_after = volumes.getAvoidance(radius, layer, TreeModelVolumes::AvoidanceType::Fast, to_model, true);
            CHECK(diff(route_before, route_after).empty());
            CHECK(diff(route_after, route_before).empty());
        }
    }
}


TEST_CASE("Painted bases smaller than the branch footprint never open a model route", "[SupportBase][Routes]")
{
    BaseRouteOptions options;
    options.platform_width = 0.8;
    BaseRouteScene scene(true, true, false, options);
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes volumes(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const coord_t tip_radius = settings.getRadius(0);
    REQUIRE(2. * unscale<double>(settings.increase_radius_until_radius) > options.platform_width);
    const auto base = scene.layer(4.4);
    // Even an initially thin tip must allow room for its eventual root.
    CHECK(volumes.getPlaceableAreas(tip_radius, base, [] {}).empty());
    CHECK(contains(volumes.getAvoidance(tip_radius, scene.layer(6.), TreeModelVolumes::AvoidanceType::Fast, true, true), scene.point(0.4, 10.)));
    const Point clear = scene.point(80., 30.);
    CHECK_FALSE(contains(volumes.getAvoidance(tip_radius, scene.layer(6.), TreeModelVolumes::AvoidanceType::Fast, false, true), clear));
}

TEST_CASE("Painted routes plan with the widest foot a branch lands with", "[SupportBase][Routes][Footprint]")
{
    // A thin branch is drawn up to the increase-until radius, and one keeping the
    // regular XY distance lands with that distance's difference added. Wherever
    // even the thinnest route may go, that widest foot must fit as well.
    BaseRouteOptions options;
    options.branch_diameter = 5.;
    BaseRouteScene scene(true, true, false, options);
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes volumes(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const coord_t delta = settings.xy_distance - settings.xy_min_distance;
    REQUIRE(delta > 0);
    // The difference crosses a step of the radius cache grid.
    REQUIRE(volumes.ceilRadius(settings.increase_radius_until_radius, false) > volumes.ceilRadius(settings.increase_radius_until_radius, true));
    const auto base = scene.layer(4.4);
    const Polygons &widest = volumes.getPlaceableAreas(settings.increase_radius_until_radius + delta, base, [] {});
    REQUIRE_FALSE(widest.empty());
    CHECK(diff(volumes.getPlaceableAreas(settings.getRadius(0), base, [] {}), widest).empty());
}

TEST_CASE("A branch moved off a slender painted tower never merges into a branch to paint", "[SupportBase][Routes][BedOnly]")
{
    const bool bed_only = GENERATE(false, true);
    BaseRouteScene scene(true);
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes volumes(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    settings.painted_bases_only = volumes.painted_bases_only();
    settings.support_rests_on_model = true;
    REQUIRE(settings.painted_bases_only);
    // Two tips close to the edge of the painted platform: one bound for the
    // paint, one able to reach the bed as well.
    const LayerIndex top = scene.layer(8.);
    std::vector<SupportElements> nodes(top + 1);
    auto add_tip = [&](const Point &point, bool to_bed) {
        SupportElementState state{};
        state.layer_idx = top;
        state.target_height = top;
        state.target_position = point;
        state.next_position = point;
        state.result_on_layer = point;
        state.to_buildplate = to_bed;
        state.to_buildplate_only = to_bed && bed_only;
        state.to_model_gracious = true;
        state.use_min_xy_dist = settings.xy_distance > settings.xy_min_distance;
        Polygon tip = make_circle(scaled<coord_t>(0.01), 25);
        tip.translate(point);
        nodes[top].emplace_back(state, Polygons{tip});
    };
    add_tip(scene.point(17., 10.), false);
    add_tip(scene.point(19., 10.), true);
    REQUIRE_FALSE(contains(volumes.getAvoidance(settings.getRadius(0), top, TreeModelVolumes::AvoidanceType::Fast, false,
        settings.xy_distance > settings.xy_min_distance), scene.point(19., 10.)));
    create_layer_pathing(volumes, settings, nodes, [] {});
    // Merged into the branch to paint, the bed branch would land on the paint too.
    CAPTURE(bed_only, nodes.front().size());
    CHECK(nodes.front().empty() != bed_only);
}

TEST_CASE("Painted support routes cannot pass through a separating model wall", "[SupportBase][Routes]")
{
    const bool barrier = GENERATE(false, true);
    BaseRouteOptions options;
    options.barrier = barrier;
    BaseRouteScene scene(true, true, false, options);
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes volumes(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const coord_t radius = scaled<coord_t>(0.5);
    // The volume cache rounds a 0.5 mm request up to a larger supported
    // radius. Leave enough room beyond the wall and enough vertical travel
    // for an unobstructed route from the painted platform.
    const auto high = scene.layer(14.);
    const Point target = scene.point(23., 10.);
    CAPTURE(barrier);
    // The target itself is free in both cases. Only the route to the paint
    // becomes impossible when the model wall crosses the space between them.
    REQUIRE_FALSE(contains(volumes.getCollision(radius, high, true), target));
    CHECK(contains(volumes.getAvoidance(radius, high, TreeModelVolumes::AvoidanceType::Fast, true, true), target) == barrier);
    CHECK_FALSE(contains(volumes.getAvoidance(radius, high, TreeModelVolumes::AvoidanceType::Fast, false, true), target));
}

TEST_CASE("Painted base contacts retain bottom gaps when raft layers shift the organic indices", "[SupportBase][Routes]")
{
    BaseRouteOptions options;
    options.raft_layers = GENERATE(0, 1, 3);
    options.bottom_gap = GENERATE(0., 0.2, 0.6);
    BaseRouteScene scene(true, true, false, options);
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    const auto &slicing = scene.object().slicing_parameters();
    TreeModelVolumes volumes(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    // Stock slicing treats a zero bottom gap as "use top gap", which is
    // 0.2 mm in this fixture; painting must preserve that rule as well.
    const double effective_gap = options.bottom_gap > 0. ? options.bottom_gap : 0.2;
    CHECK(slicing.gap_object_support == Approx(effective_gap));
    const double first_support_print_z = slicing.object_print_z_min + 4. + effective_gap + 0.2;
    const LayerIndex first = scene.layer(first_support_print_z) + LayerIndex(settings.raft_layers.size());
    const Point painted = scene.point(10., 10.);
    const coord_t radius = scaled<coord_t>(0.5);
    CAPTURE(options.raft_layers, options.bottom_gap, settings.raft_layers.size(), first_support_print_z, first);
    REQUIRE(settings.raft_layers.empty() == (options.raft_layers == 0));
    CHECK(contains(volumes.getPlaceableAreas(radius, first, [] {}), painted));
    CHECK_FALSE(contains(volumes.getPlaceableAreas(radius, first - 1, [] {}), painted));
    CHECK_FALSE(contains(volumes.getPlaceableAreas(radius, first + 1, [] {}), painted));
    CHECK(layer_z(slicing, settings, first) - 0.2 == Approx(slicing.object_print_z_min + 4. + effective_gap).margin(0.00001));
}


TEST_CASE("Invalid painted branches stop slicing instead of leaving unsupported interface roofs", "[SupportBase][Routes][InvalidBranch]")
{
    enum Failure { FootprintTooLarge, NonGracious, TooShort, OrphanTip };
    const auto failure = GENERATE(FootprintTooLarge, NonGracious, TooShort, OrphanTip);
    BaseRouteScene scene(true);
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    settings.painted_bases_only = true;
    settings.support_rests_on_model = true;
    TreeModelVolumes volumes(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const auto base = scene.layer(4.4);
    const Point center = scene.point(failure == FootprintTooLarge ? 1.2 : 10., 10.);
    Polygon footprint({center + Point(-5000, -5000), center + Point(5000, -5000),
                       center + Point(5000, 5000), center + Point(-5000, 5000)});
    std::vector<SupportElements> nodes(scene.object().layer_count());
    SupportElementState state{};
    state.layer_idx = base;
    state.target_height = base + 10;
    state.target_position = center;
    state.next_position = center;
    state.to_model_gracious = failure != NonGracious;
    state.supports_roof = failure != TooShort;
    state.distance_to_top = failure == TooShort ? 0 : uint32_t(settings.increase_radius_until_layer);
    state.effective_radius_height = 0;
    if (failure == FootprintTooLarge) {
        // A thin node's foot is drawn wider than the node and checked at that
        // radius rounded up to the cache grid. The route plans with the same
        // rounded foot, so it never admits a location that foot cannot fit; a
        // branch ending there anyway must still stop slicing.
        REQUIRE(support_element_collision_radius(settings, state) < support_element_radius(settings, state));
        CHECK(intersection(Polygons{footprint}, volumes.getPlaceableAreas(support_element_collision_radius(settings, state), base, [] {})).empty());
        REQUIRE(intersection(Polygons{footprint}, volumes.getPlaceableAreas(support_element_radius(settings, state), base, [] {})).empty());
    } else if (failure == TooShort) {
        REQUIRE(settings.min_dtt_to_model > 0);
    } else if (failure == OrphanTip) {
        state.target_height = base;
        state.result_on_layer = center;
    }
    nodes[base].emplace_back(state, Polygons{footprint});
    CAPTURE(failure);
    CHECK_THROWS_AS(create_nodes_from_area(volumes, settings, nodes, [] {}), SlicingError);
}
