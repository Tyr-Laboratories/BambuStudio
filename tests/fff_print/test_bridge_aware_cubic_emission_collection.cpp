#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicSupport.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;

TEST_CASE("Emitted construction collection equals the reference and rejects stale Plan bindings",
          "[BridgeAwareCubic][EmissionCollection]")
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "5%"}, {"sparse_infill_pattern", "bridgeawarecubic"},
        {"bridge_aware_cubic_max_gap", "5"},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"skirt_loops", "0"}, {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
        {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"},
        {"override_filament_scarf_seam_setting", "1"}, {"seam_slope_type", "none"},
        {"apply_scarf_seam_on_circles", "0"}, {"spiral_mode", "0"},
        {"use_relative_e_distances", "1"}, {"slow_down_for_layer_cooling", "0"},
        {"machine_start_gcode", ""}, {"machine_end_gcode", ""},
        {"before_layer_change_gcode", ""}, {"layer_change_gcode", ""}
    });
    Model model;
    auto *model_object = model.add_object();
    model_object->add_volume(TriangleMesh(its_make_cube(32., 28., 18.)));
    model_object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model_object);
    print.apply(model, config);
    REQUIRE_THROWS_AS(BAC::audit_construction_collection(*print.get_object(size_t(0))), std::logic_error);
    REQUIRE_NOTHROW(print.process());
    const PrintObject &object = *print.get_object(size_t(0));

    const auto current = BAC::audit_construction_collection(object);
    INFO(current.mismatch);
    REQUIRE(current.equal);
    REQUIRE(current.ledger_emitters > 0);
    // Pure bulk layers do not demonstrate that classification work was saved.
    REQUIRE(current.ledger_construction_paths > 0);

    size_t replaced = 0;
    for (size_t region = 0; region < object.num_printing_regions(); ++region)
        if (const auto *plan = object.bridge_aware_cubic_plan(region)) {
            const auto identity = plan->immutable_identity();
            auto replacement = plan->with_demands(plan->demands());
            REQUIRE(replacement.immutable_identity() != identity);
            // Deliberately leave the live fills in place. Fresh Plan identity
            // must force complete-emitter reference collection until refill.
            *const_cast<BAC::Plan *>(plan) = std::move(replacement);
            ++replaced;
        }
    REQUIRE(replaced > 0);
    const auto stale = BAC::audit_construction_collection(object);
    INFO(stale.mismatch);
    REQUIRE(stale.equal);
    CHECK(stale.ledger_emitters == 0);
    CHECK(stale.ledger_construction_paths == 0);
    CHECK(stale.reference_emitters > 0);
    print.cancel();
    CHECK_THROWS_AS(BAC::audit_construction_collection(object), CanceledException);
}
