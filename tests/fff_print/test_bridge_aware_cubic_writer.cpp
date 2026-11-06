#include <catch2/catch.hpp>

#include "libslic3r/GCodeWriter.hpp"

#include <cmath>
#include <limits>

using namespace Slic3r;

namespace {
double emitted_axis(const std::string &gcode, char axis)
{
    INFO(gcode);
    const size_t pos = gcode.rfind(std::string(" ") + axis);
    REQUIRE(pos != std::string::npos);
    return std::stod(gcode.substr(pos + 2));
}

double emitted_z(const std::string &gcode)
{
    return emitted_axis(gcode, 'Z');
}

void initialize(GCodeWriter &writer)
{
    writer.apply_print_config(PrintConfig{});
    writer.set_extruders({0});
    writer.set_extruder(0, 0);
}

double emitted_e(const std::string &gcode)
{
    return emitted_axis(gcode, 'E');
}
}

TEST_CASE("Bridge aware cubic preserves fractional motion heights", "[bridge_aware_cubic][GCodeWriter]")
{
    GCodeWriter writer;
    initialize(writer);
    writer.set_bridge_aware_cubic_z_precision(true);
    writer.set_xy_offset(125.123, -40.456);

    SECTION("Z-only and combined XYZ travel use the certified serialized height") {
        for (const double z : {.2004, .4008, 125.123456789123, 5368.123456789123}) {
            const double serialized = GCodeWriter::serialized_z(z, true);
            REQUIRE(emitted_z(writer.travel_to_z(z)) == serialized);
            REQUIRE(std::abs(serialized - z) <= 1.e-12);
        }
        REQUIRE(emitted_z(writer.travel_to_xyz(Vec3d(130., 20., .600123456789))) ==
                GCodeWriter::serialized_z(.600123456789, true));
    }

    SECTION("XYZ extrusion preserves Z while ordinary XY formatting stays unchanged") {
        const std::string gcode = writer.extrude_to_xyz(Vec3d(130.123456, 20.654321, .200412345678), .1);
        REQUIRE(emitted_z(gcode) == GCodeWriter::serialized_z(.200412345678, true));
        REQUIRE(gcode.find(" X5 Y61.11") != std::string::npos);
    }

    SECTION("Formatter G1 and G2/G3 Z share the same precision") {
        for (const double z : {-.123456789123, .200412345678, 1234.123456789123}) {
            GCodeG1Formatter linear;
            linear.emit_z(z, writer.z_export_digits());
            REQUIRE(emitted_z(linear.string()) == GCodeWriter::serialized_z(z, true));
            for (const bool ccw : {false, true}) {
                GCodeG2G3Formatter arc(ccw);
                arc.emit_z(z, writer.z_export_digits());
                REQUIRE(emitted_z(arc.string()) == GCodeWriter::serialized_z(z, true));
            }
        }
    }

    SECTION("Adjacent fractional planes retain contact within float-height representation error") {
        const double lower = emitted_z(writer.travel_to_z(.2004));
        const double upper = emitted_z(writer.travel_to_z(.4008));
        const float height = .2004f;
        const double roundoff = .5 * double(std::nextafter(height, std::numeric_limits<float>::infinity()) - height);
        REQUIRE(upper - double(height) <= lower + roundoff);
        REQUIRE(GCodeWriter::serialized_z(.4008, false) - double(height) >
                GCodeWriter::serialized_z(.2004, false) + roundoff);
    }
}

TEST_CASE("Ordinary motion keeps existing three-decimal Z formatting", "[bridge_aware_cubic][GCodeWriter]")
{
    GCodeWriter writer;
    initialize(writer);
    REQUIRE_FALSE(writer.bridge_aware_cubic_z_precision());
    REQUIRE(emitted_z(writer.travel_to_z(.2004)) == .2);
    REQUIRE(emitted_z(writer.travel_to_z(.4008)) == .401);
    writer.set_bridge_aware_cubic_z_precision(true);
    REQUIRE(emitted_z(writer.travel_to_z(.600123456789)) == .600123456789);
    writer.set_bridge_aware_cubic_z_precision(false);
    REQUIRE(emitted_z(writer.travel_to_z(.800456)) == .8);
}

TEST_CASE("Bridge aware cubic Z serialization rejects unrepresentable coordinates", "[bridge_aware_cubic][GCodeWriter]")
{
    for (double z : {std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN(), 1.e12})
        REQUIRE_THROWS(GCodeWriter::serialized_z(z, true));
}

TEST_CASE("Bridge aware cubic retains positive extrusion on tiny serialized XY moves", "[bridge_aware_cubic][GCodeWriter]")
{
    const bool relative = GENERATE(false, true);
    GCodeWriter writer;
    initialize(writer);
    writer.config.use_relative_e_distances.value = relative;
    writer.set_bridge_aware_cubic_z_precision(true);
    writer.travel_to_xy(Vec2d(.00044, 0.));
    const double short_e = .00012 * .0297;
    const std::string short_move = writer.extrude_to_xy(Vec2d(.00056, 0.), short_e, "", false, true);
    REQUIRE(emitted_axis(short_move, 'X') == .001);
    REQUIRE(emitted_axis(short_move, 'Y') == 0.);
    REQUIRE(emitted_e(short_move) > 0.);
    REQUIRE(std::abs(emitted_e(short_move) - short_e) < 5.e-10);

    // Even the smallest nonzero source-grid edge may cross a decimal XY tie.
    writer.reset_e(true);
    writer.travel_to_xy(Vec2d(.00049, 0.));
    const std::string grid_move = writer.extrude_to_xy(Vec2d(.00050, 0.), .00001 * .0297, "", false, true);
    REQUIRE(emitted_axis(grid_move, 'X') == .001);
    REQUIRE(emitted_axis(grid_move, 'Y') == 0.);
    REQUIRE(emitted_e(grid_move) > 0.);
}

TEST_CASE("Bridge aware cubic rejects XY deposition with no positive serialized E", "[bridge_aware_cubic][GCodeWriter]")
{
    const bool relative = GENERATE(false, true);
    GCodeWriter writer;
    initialize(writer);
    writer.config.use_relative_e_distances.value = relative;
    writer.set_bridge_aware_cubic_z_precision(true);
    writer.travel_to_xy(Vec2d(.00044, 0.));
    const Vec3d before = writer.get_position();
    for (double delta : {1.e-12, 0., -.1}) {
        REQUIRE_THROWS_AS(writer.extrude_to_xy(Vec2d(.00056, 0.), delta, "", false, true), std::invalid_argument);
        REQUIRE(writer.get_position() == before);
        REQUIRE(writer.filament()->E() == 0.);
    }
    // A zero-XY command deposits at a point and does not claim a bead segment.
    REQUIRE_NOTHROW(writer.extrude_to_xy(Vec2d(.00045, 0.), 1.e-12, "", false, true));
    // Explicit non-deposition connectors and retracting wipes retain their semantics.
    REQUIRE_NOTHROW(writer.extrude_to_xy(Vec2d(.001, 0.), 0., "", true, true));
    REQUIRE_NOTHROW(writer.extrude_to_xy(Vec2d(.002, 0.), -.1));

    GCodeWriter ordinary;
    initialize(ordinary);
    ordinary.config.use_relative_e_distances.value = relative;
    ordinary.travel_to_xy(Vec2d(.00044, 0.));
    REQUIRE(emitted_e(ordinary.extrude_to_xy(Vec2d(.00056, 0.), .00012 * .0297)) == 0.);
}

TEST_CASE("Bridge aware cubic checks the actual emitted absolute extrusion baseline", "[bridge_aware_cubic][GCodeWriter]")
{
    GCodeWriter writer;
    initialize(writer);
    writer.config.use_relative_e_distances.value = false;
    writer.set_bridge_aware_cubic_z_precision(true);
    REQUIRE(emitted_e(writer.extrude_to_xy(Vec2d(1., 0.), .1, "", false, true)) == .1);
    REQUIRE_FALSE(writer.retract().empty());
    const double unretracted = emitted_e(writer.unretract(.25f));
    REQUIRE(unretracted == Approx(.35));
    // The extra unretraction is emitted but is not included in Extruder::E().
    // Comparing against that internal value would wrongly accept E0.2 here.
    REQUIRE_THROWS_AS(writer.extrude_to_xy(Vec2d(2., 0.), .1, "", false, true), std::invalid_argument);
    REQUIRE(writer.reset_e(true).find("G92 E0") != std::string::npos);
    REQUIRE(emitted_e(writer.extrude_to_xy(Vec2d(2., 0.), .1, "", false, true)) == .1);
    REQUIRE(emitted_e(writer.extrude_to_xyz(Vec3d(3., 0., .2004), .1, "", false, true)) == .2);
    REQUIRE(emitted_e(writer.extrude_arc_to_xy(Vec2d(4., 0.), Vec2d(.5, 0.), .1, true, "", false, true)) == .3);
}

TEST_CASE("Bridge aware cubic restores extrusion mode and coordinate before model paths", "[bridge_aware_cubic][GCodeWriter]")
{
    const bool relative = GENERATE(false, true);
    GCodeWriter writer;
    initialize(writer);
    writer.config.use_relative_e_distances.value = relative;
    REQUIRE(writer.synchronize_extrusion_for_deposition().empty());
    writer.set_bridge_aware_cubic_z_precision(true);
    writer.extrude_to_xy(Vec2d(1., 0.), 100.);
    const std::string baseline = writer.synchronize_extrusion_for_deposition();
    REQUIRE(baseline == (relative ? "M83\n" : "M82\nG92 E0\n"));
    REQUIRE(emitted_e(writer.extrude_to_xy(Vec2d(2., 0.), .000000297, "", false, true)) == .000000297);
    if (!relative) {
        writer.config.gcode_flavor.value = gcfMach3;
        REQUIRE_THROWS_AS(writer.synchronize_extrusion_for_deposition(), std::invalid_argument);
    }
}

TEST_CASE("Bridge aware cubic preserves Z moves smaller than the ordinary travel tolerance", "[bridge_aware_cubic][GCodeWriter]")
{
    GCodeWriter writer;
    initialize(writer);
    writer.set_bridge_aware_cubic_z_precision(true);
    REQUIRE(emitted_z(writer.travel_to_z(.20005)) == .20005);
    REQUIRE(writer.will_move_z(.2));
    REQUIRE(emitted_z(writer.travel_to_z(.2)) == .2);
    REQUIRE(writer.will_move_z(.20005));
    REQUIRE(emitted_z(writer.travel_to_xyz(Vec3d(1., 0., .20005))) == .20005);
    REQUIRE(emitted_z(writer.travel_to_xyz(Vec3d(2., 0., .2))) == .2);
    REQUIRE_FALSE(writer.will_move_z(.2 + 1.e-14));

    SECTION("A small remaining lift is restored before deposition") {
        writer.config.z_hop.values = {.4};
        writer.config.retract_lift_above.values = {0.};
        writer.config.retract_lift_below.values = {1.};
        REQUIRE(emitted_z(writer.eager_lift(LiftType::NormalLift)) == .6);
        REQUIRE(writer.travel_to_z(.59995).empty());
        REQUIRE(emitted_z(writer.unlift()) == .59995);
    }

    SECTION("Ordinary output keeps its existing tolerance") {
        GCodeWriter ordinary;
        initialize(ordinary);
        ordinary.travel_to_z(.20005);
        REQUIRE_FALSE(ordinary.will_move_z(.2));
        REQUIRE(ordinary.travel_to_z(.2).empty());
    }
}

TEST_CASE("Bridge aware cubic extrusion volume follows the serialized XY distance", "[bridge_aware_cubic][GCodeWriter]")
{
    const bool relative = GENERATE(false, true);
    GCodeWriter writer;
    initialize(writer);
    writer.config.use_relative_e_distances.value = relative;
    writer.set_bridge_aware_cubic_z_precision(true);
    const double e_per_mm = .0297;
    const Vec2d start(.00049, 0.), end(.00050, 0.);
    writer.travel_to_xy(start);
    const double output_length = writer.serialized_xy_distance(end);
    REQUIRE(output_length == .001);
    REQUIRE((end - start).norm() == Approx(.00001));
    const std::string command = writer.extrude_to_xy(end, e_per_mm * output_length, "", false, true);
    const double actual_length = std::hypot(emitted_axis(command, 'X'), emitted_axis(command, 'Y'));
    REQUIRE(actual_length == .001);
    REQUIRE(emitted_e(command) / actual_length == Approx(e_per_mm).margin(5.e-7));
    // Source length would deposit only one percent of the certified section.
    REQUIRE(e_per_mm * (end - start).norm() / actual_length < .02 * e_per_mm);
    REQUIRE(writer.serialized_xy_distance(Vec2d(.00051, 0.)) == 0.);
}
