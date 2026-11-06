#include <catch2/catch.hpp>

#include <string>

#include "libslic3r/GCode/GCodeProcessor.hpp"

using namespace Slic3r;

TEST_CASE("Last Z parsing preserves a complete long double value", "[GCodeProcessor][GCodeZParsing]")
{
    for (const char *command : {"G0", "G1", "G2", "G3", "M9711"}) {
        for (const char *suffix : {"", " E0.5 F1200", "; restore height", "\tF1200 ; restore height"}) {
            const std::string gcode = std::string(command) + " Z125.123456789012" + suffix + "\n";
            CAPTURE(gcode);
            double z = -1.0;

            REQUIRE(GCodeProcessor::get_last_z_from_gcode(gcode, z));
            REQUIRE(z == Approx(125.123456789012).epsilon(0).margin(1e-12));
        }
    }
}

TEST_CASE("Last Z parsing replaces earlier heights with small fractional values", "[GCodeProcessor][GCodeZParsing]")
{
    double z = -1.0;
    REQUIRE(GCodeProcessor::get_last_z_from_gcode(
        "G0 Z5\n"
        "G1 Z-125.123456789012 F1200 ; lower\n"
        "G1 X10 Y20\n"
        "G1 Z0.000000123456789 ; final height\n", z));
    REQUIRE(z == Approx(0.000000123456789).epsilon(0).margin(1e-20));

    REQUIRE_FALSE(GCodeProcessor::get_last_z_from_gcode("G1 X30 Y40 F1200\n", z));
    REQUIRE(z == Approx(0.000000123456789).epsilon(0).margin(1e-20));
}

TEST_CASE("Position parsers preserve fractional signed and exponent tokens", "[GCodeProcessor][GCodeZParsing]")
{
    const struct {
        const char *token;
        double expected_z;
        float expected_position_z;
    } cases[] = {
        {".123456789123", 0.123456789123, 0.123456789123f},
        {"-125.123456789123", -125.123456789123, -125.123456789123f},
        {"1.23456789123e2", 123.456789123, 123.456789123f},
    };

    for (const auto &test : cases) {
        for (const char *suffix : {"", " F1200 ; final height"}) {
            const std::string gcode = std::string("G1 Z") + test.token + suffix + "\n";
            CAPTURE(gcode);
            double z = -1.0;
            Vec3f position(10.0f, 20.0f, -1.0f);

            REQUIRE(GCodeProcessor::get_last_z_from_gcode(gcode, z));
            REQUIRE(z == Approx(test.expected_z).epsilon(0).margin(1e-12));
            REQUIRE(GCodeProcessor::get_last_position_from_gcode(gcode, position));
            REQUIRE(position.z() == Approx(test.expected_position_z).epsilon(1e-7));
            REQUIRE(position.x() == 10.0f);
            REQUIRE(position.y() == 20.0f);
        }
    }
}

TEST_CASE("Last position parsing accepts complete long axis values", "[GCodeProcessor][GCodeZParsing]")
{
    for (const char *command : {"G0", "G1", "G2", "G3", "G387"}) {
        for (const char *suffix : {"", " F1200", "; restore position", "\tF1200 ; restore position"}) {
            const std::string gcode = std::string(command) +
                " X125.123456789012 Y-25.987654321098 Z0.000000123456789" + suffix + "\n";
            CAPTURE(gcode);
            Vec3f position(-1.0f, -1.0f, -1.0f);

            REQUIRE(GCodeProcessor::get_last_position_from_gcode(gcode, position));
            // The public position API stores floats, so compare to float literals.
            REQUIRE(position.x() == 125.123456789012f);
            REQUIRE(position.y() == -25.987654321098f);
            REQUIRE(position.z() == 0.000000123456789f);
        }
    }
}

TEST_CASE("Last position parsing keeps untouched axes and replaces earlier heights", "[GCodeProcessor][GCodeZParsing]")
{
    Vec3f position(10.0f, 20.0f, -1.0f);
    REQUIRE(GCodeProcessor::get_last_position_from_gcode(
        "G0 Z5 F1200\n"
        "G1 Z125.123456789012 E0.5 ; final height\n", position));
    REQUIRE(position.x() == 10.0f);
    REQUIRE(position.y() == 20.0f);
    REQUIRE(position.z() == 125.123456789012f);

    REQUIRE_FALSE(GCodeProcessor::get_last_position_from_gcode("G1 F1200\n", position));
    REQUIRE(position.z() == 125.123456789012f);
}
