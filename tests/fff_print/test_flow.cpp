#include <catch2/catch.hpp>

#include <numeric>
#include <sstream>

#include "test_data.hpp" // get access to init_print, etc

#include "libslic3r/Config.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r::Test;
using namespace Slic3r;

SCENARIO("Extrusion width specifics", "[Flow]") {
    GIVEN("A 20mm cube with a skirt, brim, three walls and one bottom solid layer") {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({
            {"nozzle_diameter", "0.4"},
            {"initial_layer_print_height", "0.3"}, {"layer_height", "0.2"},
            {"initial_layer_line_width", "0.6"}, {"initial_layer_infill_line_width", "0"},
            {"line_width", "0.45"}, {"outer_wall_line_width", "0.45"},
            {"inner_wall_line_width", "0.45"}, {"sparse_infill_line_width", "0.45"},
            {"internal_solid_infill_line_width", "0.45"},
            {"wall_generator", "classic"}, {"wall_loops", "3"},
            {"only_one_wall_first_layer", "0"},
            {"bottom_shell_layers", "1"}, {"bottom_shell_thickness", "0"},
            {"top_shell_layers", "0"}, {"top_shell_thickness", "0"},
            // Monotonic lines retain the requested width instead of widening
            // solid infill lines to fit an integer number across the surface.
            {"bottom_surface_pattern", "monotonicline"},
            {"sparse_infill_density", "40%"}, {"sparse_infill_pattern", "grid"},
            {"initial_layer_flow_ratio", "1"}, {"infill_combination", "0"},
            {"brim_type", "outer_only"}, {"brim_width", "2"},
            {"skirt_loops", "1"}, {"skirt_height", "1"}
        });

        WHEN("the first-layer width is wider than the regular width") {
            Model model;
            Print print;
            init_print({TestMesh::cube_20x20x20}, print, model, config);
            print.process();

            // Inspect generated deposition paths, including nested loops and
            // multipaths, without relying on rounded G-code E coordinates.
            const auto check_role = [](const ExtrusionEntity& entity, ExtrusionRole role,
                                       double width, double height, double mm3_per_mm) {
                size_t count = 0;
                const auto visit = [&](const auto& self, const ExtrusionEntity& current) -> void {
                    if (const auto* path = dynamic_cast<const ExtrusionPath*>(&current)) {
                        // Gap fill has variable width, and wiping paths deposit
                        // nothing; neither is a constant-width extrusion.
                        if (path->role() != role || path->is_force_no_extrusion())
                            return;
                        CAPTURE(role, path->width, path->height);
                        REQUIRE(path->polyline.length() > 0.0);
                        CHECK(path->width == Approx(width).epsilon(1e-6));
                        CHECK(path->height == Approx(height).epsilon(1e-6));
                        CHECK(path->mm3_per_mm == Approx(mm3_per_mm).epsilon(1e-6));
                        ++count;
                    } else if (const auto* loop = dynamic_cast<const ExtrusionLoop*>(&current)) {
                        for (const auto& path : loop->paths)
                            self(self, path);
                    } else if (const auto* multi = dynamic_cast<const ExtrusionMultiPath*>(&current)) {
                        for (const auto& path : multi->paths)
                            self(self, path);
                    } else if (const auto* collection = dynamic_cast<const ExtrusionEntityCollection*>(&current)) {
                        for (const auto* child : collection->entities)
                            self(self, *child);
                    } else {
                        FAIL("Unexpected extrusion entity in constant-width fixture");
                    }
                };
                visit(visit, entity);
                CAPTURE(role);
                REQUIRE(count > 0);
            };

            REQUIRE(print.objects().size() == 1);
            const PrintObject* object = print.get_object(size_t(0));
            REQUIRE(object->layer_count() > 1);

            THEN("first-layer walls, bottom infill, skirt and brim use the first-layer width and flow") {
                const Layer* first = object->get_layer(0);
                REQUIRE(first->print_z == Approx(0.3));
                REQUIRE(first->regions().size() == 1);
                const LayerRegion& region = *first->regions().front();
                // Rounded-rectangle cross section: h * (w - h * (1 - pi/4)).
                constexpr double first_layer_mm3_per_mm = 0.1606858347;
                for (const ExtrusionRole role : {erExternalPerimeter, erPerimeter})
                    check_role(region.perimeters, role, 0.6, 0.3, first_layer_mm3_per_mm);
                check_role(region.fills, erBottomSurface, 0.6, 0.3, first_layer_mm3_per_mm);
                check_role(print.skirt(), erSkirt, 0.6, 0.3, first_layer_mm3_per_mm);
                REQUIRE(print.get_brimMap().size() == 1);
                check_role(print.get_brimMap().begin()->second, erBrim, 0.6, 0.3, first_layer_mm3_per_mm);
            }

            THEN("the next layer returns to the regular wall and sparse infill widths and flow") {
                const Layer* upper = object->get_layer(1);
                REQUIRE(upper->print_z == Approx(0.5));
                REQUIRE(upper->regions().size() == 1);
                const LayerRegion& region = *upper->regions().front();
                constexpr double regular_mm3_per_mm = 0.0814159265;
                for (const ExtrusionRole role : {erExternalPerimeter, erPerimeter})
                    check_role(region.perimeters, role, 0.45, 0.2, regular_mm3_per_mm);
                check_role(region.fills, erInternalInfill, 0.45, 0.2, regular_mm3_per_mm);
            }
        }
    }
}
// needs gcode export
SCENARIO(" Bridge flow specifics.", "[Flow]") {
    GIVEN("A default config with no cooling and a fixed bridge speed, flow ratio and an overhang mesh.") {
        WHEN("bridge_flow_ratio is set to 1.0") {
            THEN("Output flow is as expected.") {
            }
        }
        WHEN("bridge_flow_ratio is set to 0.5") {
            THEN("Output flow is as expected.") {
            }
        }
        WHEN("bridge_flow_ratio is set to 2.0") {
            THEN("Output flow is as expected.") {
            }
        }
    }
    GIVEN("A default config with no cooling and a fixed bridge speed, flow ratio, fixed extrusion width of 0.4mm and an overhang mesh.") {
        WHEN("bridge_flow_ratio is set to 1.0") {
            THEN("Output flow is as expected.") {
            }
        }
        WHEN("bridge_flow_ratio is set to 0.5") {
            THEN("Output flow is as expected.") {
            }
        }
        WHEN("bridge_flow_ratio is set to 2.0") {
            THEN("Output flow is as expected.") {
            }
        }
    }
}

/// Test the expected behavior for auto-width, 
/// spacing, etc
SCENARIO("Flow: Flow math for non-bridges", "[Flow]") {
    GIVEN("Nozzle Diameter of 0.4, a desired width of 1mm and layer height of 0.5") {
        ConfigOptionFloatOrPercent	width(1.0, false);
        float nozzle_diameter	= 0.4f;
        float layer_height		= 0.4f;

        // Spacing for non-bridges is has some overlap
        THEN("External perimeter flow has spacing fixed to 1.125 * nozzle_diameter") {
            auto flow = Flow::new_from_config_width(frExternalPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Approx(1.125 * nozzle_diameter - layer_height * (1.0 - PI / 4.0)));
        }

        THEN("Internal perimeter flow has spacing fixed to 1.125 * nozzle_diameter") {
            auto flow = Flow::new_from_config_width(frPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Approx(1.125 *nozzle_diameter - layer_height * (1.0 - PI / 4.0)));
        }
        THEN("Spacing for supplied width is 0.8927f") {
            auto flow = Flow::new_from_config_width(frExternalPerimeter, width, nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Approx(width.value - layer_height * (1.0 - PI / 4.0)));
            flow = Flow::new_from_config_width(frPerimeter, width, nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Approx(width.value - layer_height * (1.0 - PI / 4.0)));
        }
    }
    /// Check the min/max
    GIVEN("Nozzle Diameter of 0.25") {
        float nozzle_diameter	= 0.25f;
        float layer_height		= 0.5f;
        WHEN("layer height is set to 0.2") {
            layer_height = 0.15f;
            THEN("Max width is set.") {
                auto flow = Flow::new_from_config_width(frPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
                REQUIRE(flow.width() == Approx(1.125 * nozzle_diameter));
            }
        }
        WHEN("Layer height is set to 0.25") {
            layer_height = 0.25f;
            THEN("Min width is set.") {
                auto flow = Flow::new_from_config_width(frPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
                REQUIRE(flow.width() == Approx(1.125 * nozzle_diameter));
            }
        }
    }

#if 0
    /// Check for an edge case in the maths where the spacing could be 0; original
    /// math is 0.99. Slic3r issue #4654
    GIVEN("Input spacing of 0.414159 and a total width of 2") {
        double in_spacing = 0.414159;
        double total_width = 2.0;
        auto flow = Flow::new_from_spacing(1.0, 0.4, 0.3);
        WHEN("solid_spacing() is called") {
            double result = flow.solid_spacing(total_width, in_spacing);
            THEN("Yielded spacing is greater than 0") {
                REQUIRE(result > 0);
            }
        }
    }
#endif    

}

/// Spacing, width calculation for bridge extrusions
SCENARIO("Flow: Flow math for bridges", "[Flow]") {
    GIVEN("Nozzle Diameter of 0.4, a desired width of 1mm and layer height of 0.5") {
		float nozzle_diameter	= 0.4f;
		float bridge_flow		= 1.0f;
        WHEN("Flow role is frExternalPerimeter") {
            auto flow = Flow::bridging_flow(nozzle_diameter * sqrt(bridge_flow), nozzle_diameter);
            THEN("Bridge width is same as nozzle diameter") {
                REQUIRE(flow.width() == Approx(nozzle_diameter));
            }
            THEN("Bridge spacing is same as nozzle diameter + BRIDGE_EXTRA_SPACING") {
                REQUIRE(flow.spacing() == Approx(nozzle_diameter + BRIDGE_EXTRA_SPACING));
            }
        }
    }
}
