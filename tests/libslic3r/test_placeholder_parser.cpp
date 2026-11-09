#include <catch2/catch.hpp>

#include "libslic3r/PlaceholderParser.hpp"
#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;

SCENARIO("Placeholder parser scripting", "[PlaceholderParser]") {
	PlaceholderParser 	parser;
	auto 				config = DynamicPrintConfig::full_print_config();

	config.set_deserialize_strict( {
		{ "printer_notes", "  PRINTER_VENDOR_PRUSA3D  PRINTER_MODEL_MK2  " },
	    { "nozzle_diameter", "0.6,0.6,0.6,0.6" },
	    { "nozzle_temperature", "357,359,363,378" },
        { "layer_height", "0.3" },
        { "inner_wall_line_width", "0.675" },
        { "initial_layer_line_width", "0.9" },
        { "support_object_xy_distance", "0.3375" },
        { "outer_wall_speed", "30" },
        { "infill_wall_overlap", "25%" },
        { "seam_slope_start_height", "50%" },
        { "max_travel_detour_distance", "50%" }
	});

    parser.apply_config(config);
	parser.set("foo", 0);
	parser.set("bar", 2);
	parser.set("num_extruders", 4);

    SECTION("nested config options (legacy syntax)") { REQUIRE(parser.process("[nozzle_temperature_[foo]]") == "357"); }
    SECTION("array reference") { REQUIRE(parser.process("{nozzle_temperature[foo]}") == "357"); }
    SECTION("array reference uses the requested element") { REQUIRE(parser.process("{nozzle_temperature[bar]}") == "363"); }
    SECTION("whitespaces and newlines are maintained") { REQUIRE(parser.process("test [ nozzle_temperature_ [foo] ] \n hu") == "test 357 \n hu"); }

    // Test the math expressions.
    SECTION("math: 2*3") { REQUIRE(parser.process("{2*3}") == "6"); }
    SECTION("math: 2*3/6") { REQUIRE(parser.process("{2*3/6}") == "1"); }
    SECTION("math: 2*3/12") { REQUIRE(parser.process("{2*3/12}") == "0"); }
    SECTION("math: 2.*3/12") { REQUIRE(std::stod(parser.process("{2.*3/12}")) == Approx(0.5)); }
    SECTION("math: 10 % 2.5") { REQUIRE(std::stod(parser.process("{10%2.5}")) == Approx(0.)); }
    SECTION("math: 11 % 2.5") { REQUIRE(std::stod(parser.process("{11%2.5}")) == Approx(1.)); }
    SECTION("math: 2*(3-12)") { REQUIRE(parser.process("{2*(3-12)}") == "-18"); }
    SECTION("math: 2*foo*(3-12)") { REQUIRE(parser.process("{2*foo*(3-12)}") == "0"); }
    SECTION("math: 2*bar*(3-12)") { REQUIRE(parser.process("{2*bar*(3-12)}") == "-36"); }
    SECTION("math: 2.5*bar*(3-12)") { REQUIRE(std::stod(parser.process("{2.5*bar*(3-12)}")) == Approx(-45)); }
    SECTION("math: min(12, 14)") { REQUIRE(parser.process("{min(12, 14)}") == "12"); }
    SECTION("math: max(12, 14)") { REQUIRE(parser.process("{max(12, 14)}") == "14"); }
    SECTION("math: min(13.4, -1238.1)") { REQUIRE(std::stod(parser.process("{min(13.4, -1238.1)}")) == Approx(-1238.1)); }
    SECTION("math: max(13.4, -1238.1)") { REQUIRE(std::stod(parser.process("{max(13.4, -1238.1)}")) == Approx(13.4)); }
    SECTION("math: int(13.4)") { REQUIRE(parser.process("{int(13.4)}") == "13"); }
    SECTION("math: int(-13.4)") { REQUIRE(parser.process("{int(-13.4)}") == "-13"); }
    SECTION("math: round(13.4)") { REQUIRE(parser.process("{round(13.4)}") == "13"); }
    SECTION("math: round(-13.4)") { REQUIRE(parser.process("{round(-13.4)}") == "-13"); }
    SECTION("math: round(13.6)") { REQUIRE(parser.process("{round(13.6)}") == "14"); }
    SECTION("math: round(-13.6)") { REQUIRE(parser.process("{round(-13.6)}") == "-14"); }
    SECTION("math: digits(5, 15)") { REQUIRE(parser.process("{digits(5, 15)}") == "              5"); }
    SECTION("math: digits(5., 15)") { REQUIRE(parser.process("{digits(5., 15)}") == "              5"); }
    SECTION("math: zdigits(5, 15)") { REQUIRE(parser.process("{zdigits(5, 15)}") == "000000000000005"); }
    SECTION("math: zdigits(5., 15)") { REQUIRE(parser.process("{zdigits(5., 15)}") == "000000000000005"); }
    SECTION("math: digits(5, 15, 8)") { REQUIRE(parser.process("{digits(5, 15, 8)}") == "     5.00000000"); }
    SECTION("math: digits(5., 15, 8)") { REQUIRE(parser.process("{digits(5, 15, 8)}") == "     5.00000000"); }
    SECTION("math: zdigits(5, 15, 8)") { REQUIRE(parser.process("{zdigits(5, 15, 8)}") == "000005.00000000"); }
    SECTION("math: zdigits(5., 15, 8)") { REQUIRE(parser.process("{zdigits(5, 15, 8)}") == "000005.00000000"); }
    SECTION("math: digits(13.84375892476, 15, 8)") { REQUIRE(parser.process("{digits(13.84375892476, 15, 8)}") == "    13.84375892"); }
    SECTION("math: zdigits(13.84375892476, 15, 8)") { REQUIRE(parser.process("{zdigits(13.84375892476, 15, 8)}") == "000013.84375892"); }

    // Current Bambu line widths and XY spacing are absolute scalar values.
    SECTION("inner_wall_line_width") { REQUIRE(std::stod(parser.process("{inner_wall_line_width}")) == Approx(0.675)); }
    SECTION("initial_layer_line_width") { REQUIRE(std::stod(parser.process("{initial_layer_line_width}")) == Approx(0.9)); }
    SECTION("support_object_xy_distance") { REQUIRE(std::stod(parser.process("{support_object_xy_distance}")) == Approx(0.3375)); }
    // A one-element per-extruder speed vector is valid in scalar context.
    SECTION("outer_wall_speed") { REQUIRE(std::stod(parser.process("{outer_wall_speed}")) == Approx(30.)); }
    // Percent options remain percentages; float-or-percent options resolve their ratio_over.
    SECTION("infill_wall_overlap") { REQUIRE(std::stod(parser.process("{infill_wall_overlap}")) == Approx(25.)); }
    SECTION("seam_slope_start_height") { REQUIRE(std::stod(parser.process("{seam_slope_start_height}")) == Approx(0.15)); }
    // A travel-distance percentage needs a particular travel path, which the
    // placeholder parser does not have. It must reject an unresolved ratio.
    SECTION("max_travel_detour_distance") { REQUIRE_THROWS(parser.process("{max_travel_detour_distance}")); }

    // Test the boolean expression parser.
    auto boolean_expression = [&parser](const std::string& templ) { return parser.evaluate_boolean_expression(templ, parser.config()); };

    SECTION("boolean expression parser: 12 == 12") { REQUIRE(boolean_expression("12 == 12")); }
    SECTION("boolean expression parser: 12 != 12") { REQUIRE(! boolean_expression("12 != 12")); }
    SECTION("boolean expression parser: regex matches") { REQUIRE(boolean_expression("\"has some PATTERN embedded\" =~ /.*PATTERN.*/")); }
    SECTION("boolean expression parser: regex does not match") { REQUIRE(! boolean_expression("\"has some PATTERN embedded\" =~ /.*PTRN.*/")); }
    SECTION("boolean expression parser: accessing variables, equal") { REQUIRE(boolean_expression("foo + 2 == bar")); }
    SECTION("boolean expression parser: accessing variables, not equal") { REQUIRE(! boolean_expression("foo + 3 == bar")); }
    SECTION("boolean expression parser: (12 == 12) and (13 != 14)") { REQUIRE(boolean_expression("(12 == 12) and (13 != 14)")); }
    SECTION("boolean expression parser: (12 == 12) && (13 != 14)") { REQUIRE(boolean_expression("(12 == 12) && (13 != 14)")); }
    SECTION("boolean expression parser: (12 == 12) or (13 == 14)") { REQUIRE(boolean_expression("(12 == 12) or (13 == 14)")); }
    SECTION("boolean expression parser: (12 == 12) || (13 == 14)") { REQUIRE(boolean_expression("(12 == 12) || (13 == 14)")); }
    SECTION("boolean expression parser: (12 == 12) and not (13 == 14)") { REQUIRE(boolean_expression("(12 == 12) and not (13 == 14)")); }
    SECTION("boolean expression parser: ternary true") { REQUIRE(boolean_expression("(12 == 12) ? (1 - 1 == 0) : (2 * 2 == 3)")); }
    SECTION("boolean expression parser: ternary false") { REQUIRE(! boolean_expression("(12 == 21/2) ? (1 - 1 == 0) : (2 * 2 == 3)")); }
    SECTION("boolean expression parser: ternary false 2") { REQUIRE(boolean_expression("(12 == 13) ? (1 - 1 == 3) : (2 * 2 == 4)")); }
    SECTION("boolean expression parser: ternary true 2") { REQUIRE(! boolean_expression("(12 == 2 * 6) ? (1 - 1 == 3) : (2 * 2 == 4)")); }
    SECTION("boolean expression parser: lower than - false") { REQUIRE(! boolean_expression("12 < 3")); }
    SECTION("boolean expression parser: lower than - true") { REQUIRE(boolean_expression("12 < 22")); }
    SECTION("boolean expression parser: greater than - true") { REQUIRE(boolean_expression("12 > 3")); }
    SECTION("boolean expression parser: greater than - false") { REQUIRE(! boolean_expression("12 > 22")); }
    SECTION("boolean expression parser: lower than or equal- false") { REQUIRE(! boolean_expression("12 <= 3")); }
    SECTION("boolean expression parser: lower than or equal - true") { REQUIRE(boolean_expression("12 <= 22")); }
    SECTION("boolean expression parser: greater than or equal - true") { REQUIRE(boolean_expression("12 >= 3")); }
    SECTION("boolean expression parser: greater than or equal - false") { REQUIRE(! boolean_expression("12 >= 22")); }
    SECTION("boolean expression parser: lower than or equal (same values) - true") { REQUIRE(boolean_expression("12 <= 12")); }
    SECTION("boolean expression parser: greater than or equal (same values) - true") { REQUIRE(boolean_expression("12 >= 12")); }
    SECTION("complex expression") { REQUIRE(boolean_expression("printer_notes=~/.*PRINTER_VENDOR_PRUSA3D.*/ and printer_notes=~/.*PRINTER_MODEL_MK2.*/ and nozzle_diameter[0]==0.6 and num_extruders>1")); }
    SECTION("complex expression2") { REQUIRE(boolean_expression("printer_notes=~/.*PRINTER_VEwerfNDOR_PRUSA3D.*/ or printer_notes=~/.*PRINTertER_MODEL_MK2.*/ or (nozzle_diameter[0]==0.6 and num_extruders>1)")); }
    SECTION("complex expression3") { REQUIRE(! boolean_expression("printer_notes=~/.*PRINTER_VEwerfNDOR_PRUSA3D.*/ or printer_notes=~/.*PRINTertER_MODEL_MK2.*/ or (nozzle_diameter[0]==0.3 and num_extruders>1)")); }
}

TEST_CASE("Placeholder ratios resolve scalar and per-extruder dependencies", "[PlaceholderParser][ratio]")
{
    DynamicConfig external;
    external.set_key_value("filament_map", new ConfigOptionInts{2, 1});
    PlaceholderParser parser(&external);
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("layer_height", "0.2");
    // These sections explicitly exercise the external-map fallback. A local
    // map from full_print_config would correctly take priority over it.
    config.erase("filament_map");
    parser.apply_config(config);

    SECTION("percentage vector resolves a vector speed parent") {
        parser.set("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{{50., true}, {25., true}});
        parser.set("outer_wall_speed", new ConfigOptionFloatsNullable{60., 100.});
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}", 0)) == Approx(25.));
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}", 1)) == Approx(30.));
    }
    SECTION("one-element percentage applies to the active extruder parent") {
        parser.set("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{{50., true}});
        parser.set("outer_wall_speed", new ConfigOptionFloatsNullable{60., 100.});
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}", 0)) == Approx(50.));
    }
    SECTION("absolute vector values do not require ratio resolution") {
        parser.set("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{{20., false}, {35., false}});
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}", 0)) == Approx(35.));
    }
    SECTION("non-nullable percentage vector resolves a scalar parent") {
        parser.set("filament_scarf_height", new ConfigOptionFloatsOrPercents{{50., true}});
        REQUIRE(std::stod(parser.process("{filament_scarf_height}")) == Approx(0.1));
    }
    SECTION("scalar percentage resolves a vector parent") {
        parser.set("seam_slope_gap", new ConfigOptionFloatOrPercent(50., true));
        parser.set("nozzle_diameter", new ConfigOptionFloats{0.4, 0.8});
        REQUIRE(std::stod(parser.process("{seam_slope_gap}", 0)) == Approx(0.4));
        REQUIRE(std::stod(parser.process("{seam_slope_gap}", 1)) == Approx(0.2));
    }
    SECTION("scalar percentage resolves current scalar line-width type") {
        parser.set("sparse_infill_anchor", new ConfigOptionFloatOrPercent(250., true));
        parser.set("sparse_infill_line_width", 0.4);
        REQUIRE(std::stod(parser.process("{sparse_infill_anchor}")) == Approx(1.));
    }
    SECTION("normal one-based mapping selects both scalar and vector ratio parents") {
        external.set_key_value("filament_map", new ConfigOptionInts{1, 2});
        parser.set("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{{50., true}, {25., true}});
        parser.set("outer_wall_speed", new ConfigOptionFloatsNullable{60., 100.});
        parser.set("seam_slope_gap", new ConfigOptionFloatOrPercent(50., true));
        parser.set("nozzle_diameter", new ConfigOptionFloats{0.4, 0.8});
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}", 0)) == Approx(30.));
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}", 1)) == Approx(25.));
        REQUIRE(std::stod(parser.process("{seam_slope_gap}", 0)) == Approx(0.2));
        REQUIRE(std::stod(parser.process("{seam_slope_gap}", 1)) == Approx(0.4));
        REQUIRE(std::stod(parser.process("{outer_wall_speed}", 0)) == Approx(60.));
        REQUIRE(std::stod(parser.process("{outer_wall_speed}", 1)) == Approx(100.));
    }
    SECTION("per-layer parser mapping takes priority over external defaults") {
        parser.set("filament_map", new ConfigOptionInts{1, 2});
        parser.set("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{{50., true}, {25., true}});
        parser.set("outer_wall_speed", new ConfigOptionFloatsNullable{60., 100.});
        parser.set("seam_slope_gap", new ConfigOptionFloatOrPercent(50., true));
        parser.set("nozzle_diameter", new ConfigOptionFloats{0.4, 0.8});
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}", 0)) == Approx(30.));
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}", 1)) == Approx(25.));
        REQUIRE(std::stod(parser.process("{seam_slope_gap}", 0)) == Approx(0.2));
        REQUIRE(std::stod(parser.process("{seam_slope_gap}", 1)) == Approx(0.4));
        REQUIRE(std::stod(parser.process("{outer_wall_speed}", 0)) == Approx(60.));
    }
    SECTION("explicit mapping override takes priority over the parser mapping") {
        parser.set("filament_map", new ConfigOptionInts{1, 2});
        parser.set("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{{50., true}, {25., true}});
        parser.set("outer_wall_speed", new ConfigOptionFloatsNullable{60., 100.});
        parser.set("seam_slope_gap", new ConfigOptionFloatOrPercent(50., true));
        parser.set("nozzle_diameter", new ConfigOptionFloats{0.4, 0.8});
        DynamicConfig overrides;
        overrides.set_key_value("filament_map", new ConfigOptionInts{2, 1});
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}", 0, &overrides)) == Approx(25.));
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}", 1, &overrides)) == Approx(30.));
        REQUIRE(std::stod(parser.process("{seam_slope_gap}", 0, &overrides)) == Approx(0.4));
        REQUIRE(std::stod(parser.process("{seam_slope_gap}", 1, &overrides)) == Approx(0.2));
        REQUIRE(std::stod(parser.process("{outer_wall_speed}", 0, &overrides)) == Approx(100.));
        // An invocation override does not alter the stored per-layer mapping.
        REQUIRE(std::stod(parser.process("{outer_wall_speed}", 0)) == Approx(60.));
    }
    SECTION("local mapping works without an external configuration") {
        PlaceholderParser local_only;
        local_only.set("filament_map", new ConfigOptionInts{2, 1});
        local_only.set("outer_wall_speed", new ConfigOptionFloatsNullable{60., 100.});
        REQUIRE(std::stod(local_only.process("{outer_wall_speed}", 0)) == Approx(100.));
        REQUIRE(std::stod(local_only.process("{outer_wall_speed}", 1)) == Approx(60.));
    }
    SECTION("invalid mapping type safely uses the first extruder") {
        parser.set("filament_map", "invalid");
        parser.set("outer_wall_speed", new ConfigOptionFloatsNullable{60., 100.});
        REQUIRE(std::stod(parser.process("{outer_wall_speed}", 0)) == Approx(60.));
    }
    SECTION("invalid mapping entries safely use the first extruder") {
        external.set_key_value("filament_map", new ConfigOptionInts{0, -1, 99});
        parser.set("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{{50., true}, {25., true}});
        parser.set("outer_wall_speed", new ConfigOptionFloatsNullable{60., 100.});
        for (unsigned int filament_id : {0u, 1u, 2u, 3u}) {
            CAPTURE(filament_id);
            REQUIRE(std::stod(parser.process("{small_perimeter_speed}", filament_id)) == Approx(30.));
        }
    }
    SECTION("missing mapping safely uses the first extruder") {
        external.erase("filament_map");
        parser.set("outer_wall_speed", new ConfigOptionFloatsNullable{60., 100.});
        REQUIRE(std::stod(parser.process("{outer_wall_speed}", 1)) == Approx(60.));
    }
    SECTION("missing external configuration safely uses the first extruder") {
        PlaceholderParser without_external;
        without_external.set("outer_wall_speed", new ConfigOptionFloatsNullable{60., 100.});
        REQUIRE(std::stod(without_external.process("{outer_wall_speed}", 1)) == Approx(60.));
    }
}

TEST_CASE("Placeholder ratios reject unresolved and invalid dependencies", "[PlaceholderParser][ratio]")
{
    PlaceholderParser parser;
    parser.apply_config(DynamicPrintConfig::full_print_config());
    parser.set("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{{50., true}});

    SECTION("missing ratio parent") {
        parser.config_writable().erase("outer_wall_speed");
        REQUIRE_THROWS(parser.process("{small_perimeter_speed}"));
    }
    SECTION("non-numeric ratio parent") {
        parser.set("outer_wall_speed", "invalid");
        REQUIRE_THROWS(parser.process("{small_perimeter_speed}"));
    }
    SECTION("empty ratio parent") {
        parser.set("outer_wall_speed", new ConfigOptionFloatsNullable{});
        REQUIRE_THROWS(parser.process("{small_perimeter_speed}"));
    }
    SECTION("undefined integer parent") {
        parser.set("outer_wall_speed", new ConfigOptionIntsNullable{
            ConfigOptionIntsNullable::nil_value()});
        REQUIRE_THROWS(parser.process("{small_perimeter_speed}"));
    }
    SECTION("empty percentage vector") {
        parser.set("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{});
        REQUIRE_THROWS(parser.process("{small_perimeter_speed}"));
    }
    SECTION("undefined percentage vector element") {
        parser.set("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{
            ConfigOptionFloatsOrPercentsNullable::nil_value()});
        REQUIRE_THROWS(parser.process("{small_perimeter_speed}"));
    }
    SECTION("unregistered percentage has no ratio definition") {
        parser.set("unknown_percentage", new ConfigOptionFloatOrPercent(50., true));
        REQUIRE_THROWS(parser.process("{unknown_percentage}"));
    }
    SECTION("nested percentage advances to the parent's ratio definition") {
        parser.set("filament_scarf_height", new ConfigOptionFloatsOrPercents{{50., true}});
        // layer_height has no ratio_over. Revisiting the child definition
        // would multiply by this parent forever instead of reporting it.
        parser.set("layer_height", new ConfigOptionFloatOrPercent(50., true));
        REQUIRE_THROWS(parser.process("{filament_scarf_height}"));
    }
}
