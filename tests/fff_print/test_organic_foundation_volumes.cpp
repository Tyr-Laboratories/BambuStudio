#include <catch2/catch.hpp>

#include <atomic>
#include <stdexcept>

#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/TreeModelVolumes.hpp"

using namespace Slic3r;
using namespace Slic3r::TreeSupport3D;

namespace {
struct FoundationVolumeScene {
    Model model;
    Print print;

    explicit FoundationVolumeScene(bool blocker = false, int raft_layers = 0)
    {
        ModelObject *object = model.add_object();
        object->add_volume(TriangleMesh(its_make_cube(20., 20., 4.)));
        TriangleMesh ceiling(its_make_cube(20., 20., 1.));
        ceiling.translate(0., 0., 10.);
        object->add_volume(std::move(ceiling));
        TriangleMesh mast(its_make_cube(2., 2., 16.));
        mast.translate(34., 0., 0.);
        object->add_volume(std::move(mast));
        if (blocker) {
            TriangleMesh blocked(its_make_cube(3., 3., 2.));
            blocked.translate(8., 8., 5.);
            object->add_volume(std::move(blocked), ModelVolumeType::SUPPORT_BLOCKER);
        }
        object->add_instance()->set_offset(Vec3d(80., 80., 0.));
        DynamicPrintConfig config;
        config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
        config.set_deserialize_strict({
            {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
            {"enable_support", "1"}, {"support_type", "tree(auto)"},
            {"support_style", "tree_organic"}, {"support_on_build_plate_only", "0"},
            {"support_bottom_z_distance", "0.6"}, {"support_top_z_distance", "0.2"},
            {"raft_layers", std::to_string(raft_layers)},
            {"support_object_xy_distance", "0.35"}, {"tree_support_branch_angle", "30"},
            {"tree_support_branch_diameter", "2"}, {"brim_type", "no_brim"}
        });
        print.set_status_silent();
        print.auto_assign_extruders(object);
        print.apply(model, config);
        this->object().slice();
    }

    PrintObject &object() { return *print.get_object(size_t(0)); }
    Point point(double x, double y)
    {
        const Vec3d transformed = object().trafo_centered() * Vec3d(x, y, 0.);
        return scaled<coord_t>(Vec2d(transformed.x(), transformed.y()));
    }
    LayerIndex layer(double z)
    {
        for (size_t i = 0; i < object().layer_count(); ++i)
            if (std::abs(object().get_layer(i)->print_z - z) < EPSILON)
                return LayerIndex(i);
        FAIL("Required slicing layer does not exist");
        return 0;
    }
    Polygons seam(double x = 10., double y = 10.)
    {
        Polygon circle = make_circle(scaled<double>(0.3), scaled<double>(0.01));
        circle.translate(point(x, y));
        return {std::move(circle)};
    }
    BuildVolume bed() { return BuildVolume({{-100., -100.}, {100., -100.}, {100., 100.}, {-100., 100.}}, 256., {}, {}); }
};
}

TEST_CASE("Reversed foundation collisions retain physical top and bottom gaps", "[OrganicFoundation][Volumes]")
{
    FoundationVolumeScene scene;
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes source(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const LayerIndex seam = scene.layer(10.);
    TreeModelVolumes reversed(source, seam, scene.seam());
    const Point axis = scene.point(10., 10.);
    // The 0.6 mm bottom gap still excludes the layer at 4.6; the
    // 0.2 mm top gap excludes 10.0 but leaves 9.8 available.
    CHECK(contains(reversed.getCollision(0, seam - scene.layer(4.6), true), axis));
    CHECK_FALSE(contains(reversed.getCollision(0, seam - scene.layer(4.8), true), axis));
    CHECK_FALSE(contains(reversed.getCollision(0, seam - scene.layer(9.8), true), axis));
    CHECK(contains(reversed.getCollision(0, 0, true), axis));
    for (const coord_t radius : {coord_t(0), scaled<coord_t>(0.4), scaled<coord_t>(1.2)})
        for (const bool minimum_xy : {false, true})
            for (LayerIndex q = 0; q <= seam; ++q) {
                const Polygons &physical = source.getCollision(radius, seam - q, minimum_xy);
                const Polygons &virtual_layer = reversed.getCollision(radius, q, minimum_xy);
                CHECK(diff(physical, virtual_layer).empty());
                CHECK(diff(virtual_layer, physical).empty());
            }
    const auto physical_cache = source.get_collision_lower_bound_area(scene.layer(4.6), scaled<coord_t>(0.5));
    const auto reversed_cache = reversed.get_collision_lower_bound_area(seam - scene.layer(4.6), scaled<coord_t>(0.5));
    REQUIRE(physical_cache);
    REQUIRE(reversed_cache);
    CHECK(physical_cache->first == reversed_cache->first);
    CHECK(diff(physical_cache->second.get(), reversed_cache->second.get()).empty());
}

TEST_CASE("Reversed foundation avoidance can terminate only at its seam", "[OrganicFoundation][Volumes]")
{
    FoundationVolumeScene scene;
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes source(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    TreeModelVolumes reversed(source, scene.layer(8.), scene.seam());
    for (const auto type : {TreeModelVolumes::AvoidanceType::Fast, TreeModelVolumes::AvoidanceType::Slow, TreeModelVolumes::AvoidanceType::FastSafe})
        for (const bool to_model : {false, true}) {
            const Polygons &at_seam = reversed.getAvoidance(scaled<coord_t>(0.4), 0, type, to_model, true);
            CHECK_FALSE(contains(at_seam, scene.point(10., 10.)));
            CHECK(contains(at_seam, scene.point(12., 10.)));
            // Reusing source avoidance would allow a nearby bed route. The
            // reversed route must instead remain within reach of the seam.
            const Polygons &one_layer = reversed.getAvoidance(scaled<coord_t>(0.4), 1, type, to_model, true);
            CHECK_FALSE(contains(one_layer, scene.point(10., 10.)));
            CHECK(contains(one_layer, scene.point(12., 10.)));
            CHECK(contains(one_layer, scene.point(99.99, 10.)));
        }
    // Seam masks constrain centers, not the branch's material footprint.
    CHECK_FALSE(contains(reversed.getCollision(0, 0, true), scene.point(12., 10.)));
    // Supplying a seam inside the model must not override real collisions.
    TreeModelVolumes obstructed(source, scene.layer(8.), scene.seam(35., 1.));
    CHECK(contains(obstructed.getAvoidance(scaled<coord_t>(0.4), 0, TreeModelVolumes::AvoidanceType::Fast, false, true), scene.point(35., 1.)));
}

TEST_CASE("Reversed foundations do not land on intermediate model surfaces", "[OrganicFoundation][Volumes]")
{
    FoundationVolumeScene scene;
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes source(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const LayerIndex seam = scene.layer(8.);
    TreeModelVolumes reversed(source, seam, scene.seam());
    REQUIRE_FALSE(source.getPlaceableAreas(scaled<coord_t>(0.4), scene.layer(4.8), [] {}).empty());
    for (LayerIndex q = 0; q <= seam; ++q) {
        CHECK(reversed.getPlaceableAreas(0, q, [] {}).empty());
        CHECK(reversed.getPlaceableAreas(scaled<coord_t>(0.4), q, [] {}).empty());
    }
}

TEST_CASE("Reversed foundation wall restrictions use the next physical layer above", "[OrganicFoundation][Volumes]")
{
    FoundationVolumeScene scene;
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes source(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const LayerIndex seam = scene.layer(8.);
    TreeModelVolumes reversed(source, seam, scene.seam());
    const coord_t radius = scaled<coord_t>(0.4);
    // At the gap boundary the next physical layer above is free, whereas
    // both layers below that boundary still collide with the platform.
    CHECK_FALSE(contains(reversed.getWallRestriction(radius, seam - scene.layer(4.6), true), scene.point(10., 10.)));
    CHECK(contains(reversed.getWallRestriction(radius, seam - scene.layer(4.4), true), scene.point(10., 10.)));
}

TEST_CASE("Reversed foundations preserve support blocker collisions", "[OrganicFoundation][Volumes]")
{
    const int raft_layers = GENERATE(0, 3);
    FoundationVolumeScene scene(true, raft_layers);
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes source(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const std::vector<Polygons> blockers = scene.object().slice_support_blockers();
    const auto first_blocker = std::find_if(blockers.begin(), blockers.end(), [](const Polygons &p) { return !p.empty(); });
    REQUIRE(first_blocker != blockers.end());
    const LayerIndex physical_layer = LayerIndex(first_blocker - blockers.begin()) + LayerIndex(settings.raft_layers.size()) + 2;
    const LayerIndex seam = physical_layer + 15;
    TreeModelVolumes reversed(source, seam, scene.seam());
    const auto &virtual_layer = reversed.getCollision(0, seam - physical_layer, true);
    CHECK(contains(virtual_layer, scene.point(9., 9.)));
    CHECK_FALSE(contains(virtual_layer, scene.point(13., 9.)));
    CHECK_FALSE(contains(reversed.getCollision(0, 0, true), scene.point(9., 9.)));
    const auto &with_radius = reversed.getCollision(scaled<coord_t>(0.4), seam - physical_layer, true);
    CHECK(contains(with_radius, scene.point(7.9, 9.)));
    const auto smoothing = reversed.get_collision_lower_bound_area(seam - physical_layer, 0);
    REQUIRE(smoothing);
    CHECK(contains(smoothing->second.get(), scene.point(9., 9.)));
}

TEST_CASE("Reversed foundation cache queries remain cancellable during lazy calculations", "[OrganicFoundation][Volumes][Cancellation]")
{
    enum class Query { FastAvoidance, SafeAvoidance, WallRestriction, PhysicalCollision };
    const Query query_kind = GENERATE(Query::FastAvoidance, Query::SafeAvoidance, Query::WallRestriction, Query::PhysicalCollision);
    FoundationVolumeScene scene;
    TreeSupportSettings settings(TreeSupportMeshGroupSettings(scene.object()), scene.object().slicing_parameters());
    TreeModelVolumes source(scene.object(), scene.bed(), settings.maximum_move_distance, settings.maximum_move_distance_slow, 0);
    const LayerIndex seam = scene.layer(8.);
    // Warm physical collisions so the avoidance and wall cases must observe
    // cancellation inside their own new directional cache calculations.
    source.precalculate(scene.object(), seam, [] {});
    std::atomic<unsigned> polls{0};
    std::atomic<bool> cancel{true};
    TreeModelVolumes reversed(source, seam, scene.seam(), [&] {
        if (++polls >= 3 && cancel.load())
            throw std::runtime_error("cancelled reverse cache calculation");
    });
    auto query = [&](const TreeModelVolumes &volumes) -> const Polygons & {
        switch (query_kind) {
        case Query::FastAvoidance:
            return volumes.getAvoidance(settings.min_radius, 10, TreeModelVolumes::AvoidanceType::Fast, false, true);
        case Query::SafeAvoidance:
            return volumes.getAvoidance(settings.min_radius, 10, TreeModelVolumes::AvoidanceType::FastSafe, false, true);
        case Query::WallRestriction:
            return volumes.getWallRestriction(settings.min_radius, 10, true);
        case Query::PhysicalCollision:
            // A large, previously unused radius forces a lazy source collision.
            return volumes.getCollision(scaled<coord_t>(20.), 10, true);
        }
        throw std::logic_error("Unhandled reverse cache query");
    };
    CHECK_THROWS_AS(query(reversed), std::runtime_error);
    CHECK(polls.load() >= 3);
    const unsigned cancelled_polls = polls.load();
    cancel = false;
    CHECK_NOTHROW(query(reversed));
    CHECK(polls.load() > cancelled_polls);
    // Interrupted nested tasks must not publish unfilled cache slots as valid
    // geometry. Retrying must match a view that was never interrupted.
    TreeModelVolumes uninterrupted(source, seam, scene.seam());
    CHECK(diff(query(reversed), query(uninterrupted)).empty());
    CHECK(diff(query(uninterrupted), query(reversed)).empty());
}
