#include <catch2/catch.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"

#include "test_data.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

SCENARIO("PrintObject: object layer heights", "[PrintObject]") {
    GIVEN("20mm cube and default initial config, initial layer height of 2mm") {
        const auto layer_config = [](double layer_height, double nozzle_diameter) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(2.));
            config.set_key_value("layer_height", new ConfigOptionFloat(layer_height));
            config.set_key_value("nozzle_diameter", new ConfigOptionFloats{nozzle_diameter});
            config.set_key_value("max_layer_height", new ConfigOptionFloatsNullable{nozzle_diameter});
            // Keep the deliberately large nozzle/layer combinations physically
            // consistent. Printer defaults otherwise retain 0.4 mm line widths.
            for (const char *key : {"line_width", "initial_layer_line_width",
                                   "initial_layer_infill_line_width", "outer_wall_line_width",
                                   "inner_wall_line_width", "sparse_infill_line_width",
                                   "internal_solid_infill_line_width", "top_surface_line_width",
                                   "support_line_width"})
                config.set_key_value(key, new ConfigOptionFloat(nozzle_diameter));
            return config;
        };
        WHEN("generate_object_layers() is called for 2mm layer heights and nozzle diameter of 3mm") {
            Slic3r::Print print;
            Slic3r::Model model;
            Slic3r::Test::init_print({TestMesh::cube_20x20x20}, print, model, layer_config(2., 3.));
            // This fixture tests layer construction, before extrusion widths
            // or toolpaths for these deliberately oversized nozzles matter.
            print.get_object(size_t(0))->slice();
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
            THEN("The output vector has 10 entries") {
                REQUIRE(layers.size() == 10);
            }
            AND_THEN("Each layer is approximately 2mm above the previous Z") {
                coordf_t last = 0.0;
                for (size_t i = 0; i < layers.size(); ++ i) {
                    REQUIRE((layers[i]->print_z - last) == Approx(2.0));
                    last = layers[i]->print_z;
                }
            }
        }
        WHEN("generate_object_layers() is called for 10mm layer heights and nozzle diameter of 11mm") {
            Slic3r::Print print;
            Slic3r::Model model;
            Slic3r::Test::init_print({TestMesh::cube_20x20x20}, print, model, layer_config(10., 11.));
            print.get_object(size_t(0))->slice();
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
			THEN("The output vector has 3 entries") {
                REQUIRE(layers.size() == 3);
            }
            AND_THEN("Layer 0 is at 2mm") {
                REQUIRE(layers.front()->print_z == Approx(2.0));
            }
            AND_THEN("Layer 1 is at 12mm") {
                REQUIRE(layers[1]->print_z == Approx(12.0));
            }
        }
        WHEN("generate_object_layers() is called for 15mm layer heights and nozzle diameter of 16mm") {
            Slic3r::Print print;
            Slic3r::Model model;
            Slic3r::Test::init_print({TestMesh::cube_20x20x20}, print, model, layer_config(15., 16.));
            print.get_object(size_t(0))->slice();
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
			THEN("The output vector has 2 entries") {
                REQUIRE(layers.size() == 2);
            }
            AND_THEN("Layer 0 is at 2mm") {
                REQUIRE(layers[0]->print_z == Approx(2.0));
            }
            AND_THEN("Layer 1 is at 17mm") {
                REQUIRE(layers[1]->print_z == Approx(17.0));
            }
        }
#if 0
        WHEN("generate_object_layers() is called for 15mm layer heights and nozzle diameter of 5mm") {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({TestMesh::cube_20x20x20}, print, {
        		{ "first_layer_height", 2 },
				{ "layer_height", 		15 },
	            { "nozzle_diameter", 	5 }
	        });
			const std::vector<Slic3r::Layer*> &layers = print.objects().front()->layers();
			THEN("The layer height is limited to 5mm.") {
                CHECK(layers.size() == 5);
                coordf_t last = 2.0;
                for (size_t i = 1; i < layers.size(); i++) {
                    REQUIRE((layers[i]->print_z - last) == Approx(5.0));
                    last = layers[i]->print_z;
                }
            }
        }
#endif
    }
}
