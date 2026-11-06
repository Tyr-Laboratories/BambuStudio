#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"

#include <cmath>

using namespace Slic3r;

TEST_CASE("Bridge aware cubic repairs only the failing strand in a grouped support plane",
          "[BridgeAwareCubic][Integration][RepairLocality]")
{
    Model model;
    auto *object = model.add_object();
    object->name = "Bridge aware cubic local repair fixture";
    object->add_volume(TriangleMesh(its_make_cube(32., 28., 18.)));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig&>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "5%"}, {"sparse_infill_pattern", "bridgeawarecubic"},
        {"bridge_aware_cubic_max_gap", "5"},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
        {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"}
    });
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(object);
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const auto *plan = print.get_object(size_t(0))->bridge_aware_cubic_plan(0);
    REQUIRE(plan != nullptr);

    // At this fixed lattice phase the upper plane has several unsupported
    // intervals, but only its short right-wall tail fails the two-anchor test.
    // Earlier grouped repair selected the passing intervals across the part
    // as well. Inspect actual planned demand, independently of tree depth.
    bool checked_first_repair = false;
    for (const auto &demand : plan->demands()) {
        if (std::abs(demand.z - 10.4) > EPSILON) continue;
        REQUIRE_FALSE(demand.footprint.empty());
        const BoundingBox bounds = get_extents(demand.footprint);
        CHECK(unscale<double>(bounds.min.x()) > 14.8);
        CHECK(unscale<double>(bounds.size().x()) < 1.);
        CHECK(unscale<double>(bounds.size().y()) < 1.);
        checked_first_repair = true;
        break;
    }
    REQUIRE(checked_first_repair);
}
