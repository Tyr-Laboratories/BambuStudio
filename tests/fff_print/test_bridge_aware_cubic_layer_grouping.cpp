#include <catch2/catch.hpp>

#include "libslic3r/Exception.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils.hpp"

#include <boost/filesystem.hpp>

using namespace Slic3r;

namespace {
struct GroupingExportWorkspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("bridge-aware-grouping-%%%%-%%%%");
    std::string old_data = data_dir(), old_temp = temporary_dir();
    GroupingExportWorkspace()
    {
        boost::filesystem::create_directories(path);
        set_data_dir(path.string());
        set_temporary_dir(path.string());
    }
    ~GroupingExportWorkspace()
    {
        set_data_dir(old_data);
        set_temporary_dir(old_temp);
        boost::system::error_code error;
        boost::filesystem::remove_all(path, error);
    }
};
}

TEST_CASE("Bridge aware cubic rejects merging distinct serialized layer planes",
          "[BridgeAwareCubic][Integration][LayerGrouping]")
{
    const bool bac = GENERATE(false, true);
    const bool equal_heights = GENERATE(false, true);
    CAPTURE(bac, equal_heights);
    GroupingExportWorkspace workspace;
    Model model;
    for (size_t i = 0; i < 2; ++i) {
        ModelObject *object = model.add_object();
        object->name = i == 0 ? "Ordinary close-height object" : "Certified object";
        // Distinct meshes prevent ordinary control objects sharing layer data.
        object->add_volume(TriangleMesh(its_make_cube(6. + i, 6., 1.)));
        object->add_instance()->set_offset(Vec3d(50. + 40. * i, 50., 0.));
        if (i == 0)
            object->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipCubic));
    }

    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "5%"}, {"sparse_infill_pattern", bac ? "bridgeawarecubic" : "cubic"},
        {"top_shell_layers", "3"}, {"bottom_shell_layers", "3"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"seam_gap", "0"}, {"fill_multiline", "1"}, {"brim_type", "no_brim"},
        {"skirt_loops", "0"}, {"enable_arc_fitting", "0"},
        {"override_filament_scarf_seam_setting", "1"}, {"seam_slope_type", "none"},
        {"machine_start_gcode", ""}, {"machine_end_gcode", ""},
        {"before_layer_change_gcode", ""}, {"layer_change_gcode", ""}
    });
    Print print;
    print.set_status_silent();
    for (ModelObject *object : model.objects) print.auto_assign_extruders(object);
    print.apply(model, config);
    REQUIRE(print.objects().size() == 2);
    REQUIRE(print.objects()[1]->has_bridge_aware_cubic() == bac);
    REQUIRE_NOTHROW(print.process());
    REQUIRE(print.objects()[0]->layers().size() > 1);
    REQUIRE(print.objects()[1]->layers().size() > 1);
    REQUIRE(print.objects()[0]->layers()[1] != print.objects()[1]->layers()[1]);
    if (!equal_heights) {
        // Constant-height slicing normalizes the requested .200025 height.
        // Inject the close ordinary plane at the exporter boundary so this
        // regression exercises actual grouping without modifying BAC geometry.
        const_cast<Layer *>(print.objects()[0]->layers()[1])->print_z += .000025;
    }
    const double first = print.objects()[0]->layers()[1]->print_z;
    const double second = print.objects()[1]->layers()[1]->print_z;
    CAPTURE(first, second);
    REQUIRE(std::abs(first - second) < EPSILON);
    REQUIRE((GCodeWriter::serialized_z(first, true) == GCodeWriter::serialized_z(second, true)) == equal_heights);
    GCodeProcessorResult result;
    const std::string output = (workspace.path / "grouped.gcode").string();

    if (bac && !equal_heights) {
        REQUIRE_THROWS_WITH(print.export_gcode(output, &result), Catch::Matchers::Contains("different serialized layer heights"));
    } else if (bac) {
        SECTION("Matching object planes export successfully") {
            REQUIRE_NOTHROW(print.export_gcode(output, &result));
        }
        SECTION("An ordinary object-support average cannot hide distinct planes") {
            // Model the scheduler's paired ordinary object/support entry. Its
            // average matches BAC, but process_layer selects the object plane.
            PrintObject *ordinary = const_cast<PrintObject *>(print.objects()[0]);
            ordinary->layers()[1]->print_z = second + .000025;
            SupportLayer *support = ordinary->add_support_layer(0, 0, .2, second - .000025);
            ExtrusionPath path(erSupportMaterial, .08, .4f, .2f);
            path.polyline = Polyline{Point(0, 0), Point::new_scale(1., 0.)};
            support->support_fills.append(path);
            REQUIRE(GCodeWriter::serialized_z(.5 * (ordinary->layers()[1]->print_z + support->print_z), true) ==
                    GCodeWriter::serialized_z(second, true));
            REQUIRE_THROWS_WITH(print.export_gcode(output, &result), Catch::Matchers::Contains("different serialized layer heights"));
        }
    } else {
        REQUIRE_NOTHROW(print.export_gcode(output, &result));
    }
}
