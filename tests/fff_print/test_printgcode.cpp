#include <catch2/catch.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Preset.hpp"

#include "test_data.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <vector>

#include "libslic3r/GCode/GCodeProcessor.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

struct ExportedExtrusion
{
    double z;
    double width;
};

using ExportedExtrusions = std::map<ExtrusionRole, std::vector<ExportedExtrusion>>;

// Bambu exports per-path metadata, rather than the retired Prusa width summary
// and per-move role comments. Require an actual depositing move for each role.
ExportedExtrusions exported_extrusions(const std::string &gcode, const GCodeConfig &config)
{
    ExportedExtrusions result;
    ExtrusionRole role = erNone;
    double width = 0.;
    const std::string &role_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role);
    const std::string &width_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width);
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.substr(0, role_tag.size()) == role_tag)
            role = ExtrusionEntity::string_to_role(comment.substr(role_tag.size()));
        if (comment.substr(0, width_tag.size()) == width_tag)
            width = std::stod(std::string(comment.substr(width_tag.size())));
        if (line.extruding(self) && line.dist_XY(self) > 0.)
            result[role].push_back({line.new_Z(self), width});
    });
    return result;
}

void require_exported_widths(const ExportedExtrusions &extrusions, ExtrusionRole role)
{
    auto found = extrusions.find(role);
    REQUIRE(found != extrusions.end());
    REQUIRE_FALSE(found->second.empty());
    REQUIRE(std::all_of(found->second.begin(), found->second.end(), [](const ExportedExtrusion &extrusion) {
        return std::isfinite(extrusion.width) && extrusion.width > 0.;
    }));
}

void configure_test_filaments(DynamicPrintConfig &config, unsigned count)
{
    config.set_num_filaments(count);
    // Each fixture filament uses the same default material profile. Export and
    // tool ordering directly index several arrays not covered by set_num_filaments,
    // including start G-code. Keep profile-compatibility lists as lists of presets.
    const auto &defaults = FullPrintConfig::defaults();
    for (const std::string &key : Preset::filament_options()) {
        if (key.find("compatible_") == 0)
            continue;
        ConfigOption *option = config.option(key);
        if (option != nullptr && option->is_vector())
            static_cast<ConfigOptionVectorBase *>(option)->resize(count, defaults.option(key));
    }
    std::vector<int> filament_indices(count);
    std::iota(filament_indices.begin(), filament_indices.end(), 1);
    config.set_key_value("filament_self_index", new ConfigOptionInts(filament_indices));
    config.set_key_value("filament_map", new ConfigOptionInts(std::vector<int>(count, 1)));
    config.set_key_value("filament_volume_map", new ConfigOptionInts(std::vector<int>(count, int(nvtStandard))));
    config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats(std::vector<double>(count * count, 0.)));
}

struct SpiralRaftGCodeResult
{
    double first_model_extrusion_z { -1. };
    double highest_z_before_model  { 0. };
    double nominal_model_layer_z   { 0. };
    size_t layer_z_restores        { 0 };
    bool   model_z_is_continuous   { true };
};

SpiralRaftGCodeResult spiral_raft_gcode_result(const std::string &change_filament_gcode)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    configure_test_filaments(config, 2);
    config.set_deserialize_strict({
        { "spiral_mode",                           true },
        { "wall_loops",                            1 },
        { "top_shell_layers",                      0 },
        { "bottom_shell_layers",                   1 },
        { "sparse_infill_density",                 "0%" },
        { "raft_layers",                           2 },
        { "support_filament",                      2 },
        { "support_interface_filament",            2 },
        { "wall_filament",                         1 },
        { "sparse_infill_filament",                1 },
        { "solid_infill_filament",                 1 },
        { "retraction_length",                     0 },
        { "skirt_loops",                           0 },
        { "machine_start_gcode",                   "T[initial_tool]\n" },
        { "change_filament_gcode",                 change_filament_gcode }
    });

    Slic3r::Print print;
    Slic3r::Model model;
    Slic3r::Test::init_print({ TestMesh::cube_20x20x20 }, print, model, config);
    std::string gcode = Slic3r::Test::gcode(print);

    SpiralRaftGCodeResult result;
    result.nominal_model_layer_z = print.objects().front()->layers().front()->print_z;
    int                   current_tool = -1;
    bool                  raft_tool_seen = false;
    double                last_model_extrusion_z = -1.;
    GCodeReader           reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&result, &current_tool, &raft_tool_seen, &last_model_extrusion_z](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.cmd().size() > 1 && line.cmd().front() == 'T') {
            current_tool = atoi(line.cmd().data() + 1);
            raft_tool_seen |= current_tool == 1;
        }

        if (result.first_model_extrusion_z < 0.) {
            result.highest_z_before_model = std::max(result.highest_z_before_model, static_cast<double>(line.new_Z(self)));
            // Writer comments are disabled in normal Bambu exports. Verify the
            // actual downward move restoring the model layer after tool change.
            if (raft_tool_seen && current_tool == 0 && (line.cmd_is("G0") || line.cmd_is("G1")) &&
                line.dist_Z(self) < -EPSILON &&
                std::abs(line.new_Z(self) - result.nominal_model_layer_z) < EPSILON)
                ++result.layer_z_restores;
            if (raft_tool_seen && current_tool == 0 && line.extruding(self) && line.dist_XY(self) > 0.)
                result.first_model_extrusion_z = line.new_Z(self);
        }

        if (raft_tool_seen && current_tool == 0 && line.extruding(self) && line.dist_XY(self) > 0.) {
            if (last_model_extrusion_z >= 0.) {
                double z_step = line.new_Z(self) - last_model_extrusion_z;
                result.model_z_is_continuous &= z_step >= -EPSILON && z_step <= 0.21;
            }
            last_model_extrusion_z = line.new_Z(self);
        }
    });
    return result;
}

} // namespace

SCENARIO( "PrintGCode basic functionality", "[PrintGCode]") {
    GIVEN("A default configuration and a print test object") {
        WHEN("the output is executed with no support material") {
            Slic3r::Print print;
            Slic3r::Model model;
            Slic3r::Test::init_print({TestMesh::cube_20x20x20}, print, model, {
                { "layer_height",                          0.2 },
                { "initial_layer_print_height",            0.2 },
                { "initial_layer_line_width",              0 },
                { "machine_start_gcode",                   "" }
                });
            std::string gcode = Slic3r::Test::gcode(print);
            THEN("Some text output is generated.") {
                REQUIRE(gcode.size() > 0);
            }
            THEN("Exported text contains slic3r version") {
                REQUIRE(gcode.find(SLIC3R_VERSION) != std::string::npos);
            }
            //THEN("Exported text contains git commit id") {
            //    REQUIRE(gcode.find("; Git Commit") != std::string::npos);
            //    REQUIRE(gcode.find(SLIC3R_BUILD_ID) != std::string::npos);
            //}
            THEN("Exported paths contain extrusion role and width metadata.") {
                const auto extrusions = exported_extrusions(gcode, GCodeConfig{});
                for (ExtrusionRole role : {erExternalPerimeter, erPerimeter, erInternalInfill, erSolidInfill, erTopSolidInfill})
                    require_exported_widths(extrusions, role);
                REQUIRE(extrusions.count(erSupportMaterial) == 0);
                REQUIRE(extrusions.count(erSupportMaterialInterface) == 0);
            }
            THEN("Exported text does not contain cooling markers (they were consumed)") {
                REQUIRE(gcode.find(";_EXTRUDE_SET_SPEED") == std::string::npos);
            }

            THEN("GCode preamble is emitted.") {
                REQUIRE(gcode.find("\nG21\n") != std::string::npos);
            }

            THEN("Config options emitted for print config, default region config, default object config") {
                REQUIRE(gcode.find("; nozzle_temperature_initial_layer =") != std::string::npos);
                REQUIRE(gcode.find("; layer_height") != std::string::npos);
                REQUIRE(gcode.find("; sparse_infill_density =") != std::string::npos);
            }
            THEN("Infill is emitted.") {
                require_exported_widths(exported_extrusions(gcode, print.config()), erInternalInfill);
            }
            THEN("Perimeters are emitted.") {
                require_exported_widths(exported_extrusions(gcode, print.config()), erPerimeter);
            }
            THEN("Skirt is emitted.") {
                require_exported_widths(exported_extrusions(gcode, print.config()), erSkirt);
            }
            THEN("final Z height is 20mm") {
                double final_z = 0.0;
                GCodeReader reader;
                reader.apply_config(print.config());
                reader.parse_buffer(gcode, [&final_z] (GCodeReader& self, const GCodeReader::GCodeLine& line) {
                    if (line.extruding(self) && line.dist_XY(self) > 0.)
                        final_z = std::max<double>(final_z, line.new_Z(self));
                });
                REQUIRE(final_z == Approx(20.));
            }
        }
        WHEN("output is executed with complete objects and two differently-sized meshes") {
            Slic3r::Print print;
            Slic3r::Model model;
            TriangleMesh short_cube = mesh(TestMesh::cube_20x20x20);
            short_cube.scale(Vec3f(1.f, 1.f, 0.5f));
            Slic3r::Test::init_print({short_cube, mesh(TestMesh::cube_20x20x20)}, print, model, {
                { "initial_layer_line_width",              0 },
                { "initial_layer_print_height",            0.3 },
                { "layer_height",                          0.2 },
                { "enable_support",                        false },
                { "raft_layers",                           0 },
                { "print_sequence",                        "by object" },
                { "printing_by_object_gcode",              "; between-object-gcode" }
                });
            std::string gcode = Slic3r::Test::gcode(print);
            THEN("Some text output is generated.") {
                REQUIRE(gcode.size() > 0);
            }
            THEN("Infill is emitted.") {
                require_exported_widths(exported_extrusions(gcode, print.config()), erInternalInfill);
            }
            THEN("Perimeters are emitted.") {
                require_exported_widths(exported_extrusions(gcode, print.config()), erPerimeter);
            }
            THEN("Skirt is emitted.") {
                require_exported_widths(exported_extrusions(gcode, print.config()), erSkirt);
            }
            THEN("Between-object-gcode is emitted.") {
                REQUIRE(gcode.find("; between-object-gcode") != std::string::npos);
            }
            THEN("final Z height is 20.1mm") {
                double final_z = 0.0;
                GCodeReader reader;
                reader.apply_config(print.config());
                reader.parse_buffer(gcode, [&final_z] (GCodeReader& self, const GCodeReader::GCodeLine& line) {
                    if (line.extruding(self) && line.dist_XY(self) > 0.)
                        final_z = std::max<double>(final_z, line.new_Z(self));
                });
                REQUIRE(final_z == Approx(20.1));
            }
            THEN("Z height resets on object change") {
                double previous_extrusion_z = 0.;
                size_t resets = 0;
                GCodeReader reader;
                reader.apply_config(print.config());
                reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
                    if (!line.extruding(self) || line.dist_XY(self) == 0.)
                        return;
                    const double z = line.new_Z(self);
                    if (z + EPSILON < previous_extrusion_z) {
                        REQUIRE(z == Approx(0.3));
                        ++resets;
                    }
                    previous_extrusion_z = z;
                });
                REQUIRE(resets == 1);
            }
            THEN("The two object heights follow the configured model order") {
                // Sequential printing now follows model order, rather than
                // sorting by height. The first model is 10 mm, the second 20 mm.
                size_t current_object = 0;
                std::vector<double> object_heights(1, 0.);
                GCodeReader reader;
                reader.apply_config(print.config());
                reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
                    if (line.comment() == " between-object-gcode") {
                        ++current_object;
                        object_heights.push_back(0.);
                    }
                    if (line.extruding(self) && line.dist_XY(self) > 0.)
                        object_heights[current_object] = std::max<double>(object_heights[current_object], line.new_Z(self));
                });
                REQUIRE(object_heights.size() == 2);
                REQUIRE(object_heights[0] == Approx(10.1));
                REQUIRE(object_heights[1] == Approx(20.1));
            }
        }
        WHEN("the output is executed with support material") {
            std::string gcode = ::Test::slice({TestMesh::cube_20x20x20}, {
                { "initial_layer_line_width",              0 },
                { "enable_support",                        true },
                { "raft_layers",                           3 },
                });
            THEN("Some text output is generated.") {
                REQUIRE(gcode.size() > 0);
            }
            THEN("Exported paths contain extrusion role and width metadata.") {
                const auto extrusions = exported_extrusions(gcode, GCodeConfig{});
                for (ExtrusionRole role : {erExternalPerimeter, erPerimeter, erInternalInfill, erSolidInfill, erTopSolidInfill})
                    require_exported_widths(extrusions, role);
                require_exported_widths(extrusions, erSupportMaterial);
                require_exported_widths(extrusions, erSupportMaterialInterface);
            }
            THEN("Raft is emitted.") {
                const auto extrusions = exported_extrusions(gcode, GCodeConfig{});
                require_exported_widths(extrusions, erSupportMaterial);
                REQUIRE(extrusions.at(erSupportMaterial).front().z < extrusions.at(erExternalPerimeter).front().z);
            }
        }
        WHEN("the output is executed with a separate first layer extrusion width") {
            std::string gcode = ::Test::slice({ TestMesh::cube_20x20x20 }, {
                { "initial_layer_line_width",              "0.5" },
                { "wall_generator",                        "classic" }
                });
            THEN("Some text output is generated.") {
                REQUIRE(gcode.size() > 0);
            }
            THEN("Exported paths contain extrusion role and width metadata.") {
                const auto extrusions = exported_extrusions(gcode, GCodeConfig{});
                for (ExtrusionRole role : {erExternalPerimeter, erPerimeter, erInternalInfill, erSolidInfill, erTopSolidInfill})
                    require_exported_widths(extrusions, role);
                REQUIRE(extrusions.count(erSupportMaterial) == 0);
                const auto &walls = extrusions.at(erExternalPerimeter);
                const double first_z = walls.front().z;
                for (const ExportedExtrusion &wall : walls)
                    if (std::abs(wall.z - first_z) < EPSILON)
                        REQUIRE(wall.width == Approx(0.5));
                REQUIRE(std::any_of(walls.begin(), walls.end(), [first_z](const ExportedExtrusion &wall) {
                    return wall.z > first_z + EPSILON && std::abs(wall.width - 0.5) > EPSILON;
                }));
            }
        }
        WHEN("Cooling is enabled and the fan is disabled.") {
            std::string gcode = ::Test::slice({ TestMesh::cube_20x20x20 }, {
                { "slow_down_for_layer_cooling",           true },
                { "close_fan_the_first_x_layers",          5 }
                });
            THEN("GCode to disable fan is emitted."){
                REQUIRE(gcode.find("M106 S0") != std::string::npos);
            }
        }
        WHEN("machine_end_gcode contains layer_num and layer_z") {
            std::string gcode = ::Test::slice({ TestMesh::cube_20x20x20 }, {
                { "machine_end_gcode",                     "; Layer_num [layer_num]\n; Layer_z [layer_z]" },
                { "layer_height",                          0.1 },
                { "initial_layer_print_height",            0.1 }
                });
            THEN("layer_num and layer_z are processed in the end gcode") {
                REQUIRE(gcode.find("; Layer_num 199") != std::string::npos);
                REQUIRE(gcode.find("; Layer_z 20") != std::string::npos);
            }
        }
        WHEN("machine_start_gcode contains current_extruder") {
            {
                std::string gcode = ::Test::slice({ TestMesh::cube_20x20x20 }, {
                    { "machine_start_gcode",                   "; Extruder [current_extruder]" }
                });
                THEN("current_extruder is processed in the start gcode and set for first extruder") {
                    REQUIRE(gcode.find("; Extruder 0") != std::string::npos);
                }
            }
            {
                DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
                configure_test_filaments(config, 4);
                config.set_deserialize_strict({
                    { "machine_start_gcode",                   "; Extruder [current_extruder]" },
                    { "sparse_infill_filament",                2 },
                    { "solid_infill_filament",                 2 },
                    { "wall_filament",                         2 },
                    { "support_filament",                      2 },
                    { "support_interface_filament",            2 }
                });
                std::string gcode = Slic3r::Test::slice({TestMesh::cube_20x20x20}, config);
                THEN("current_extruder is processed in the start gcode and set for second extruder") {
                    REQUIRE(gcode.find("; Extruder 1") != std::string::npos);
                }
            }
        }

        WHEN("layer_num represents the layer's index from z=0") {
            std::string gcode = ::Test::slice({ TestMesh::cube_20x20x20, TestMesh::cube_20x20x20 }, {
                { "print_sequence",                        "by object" },
                { "layer_change_gcode",                    ";Layer:[layer_num] ([layer_z] mm)" },
                { "layer_height",                          0.1 },
                { "initial_layer_print_height",            0.1 }
                });
            // End of the 1st object.
            std::string token = ";Layer:199 ";
            size_t pos = gcode.find(token);
            THEN("First and second object last layer is emitted") {
                // First object
                REQUIRE(pos != std::string::npos);
                pos += token.size();
                REQUIRE(pos < gcode.size());
                double z = 0;
                REQUIRE((sscanf(gcode.data() + pos, "(%lf mm)", &z) == 1));
                REQUIRE(z == Approx(20.));
                // Second object
                pos = gcode.find(";Layer:399 ", pos);
                REQUIRE(pos != std::string::npos);
                pos += token.size();
                REQUIRE(pos < gcode.size());
                REQUIRE((sscanf(gcode.data() + pos, "(%lf mm)", &z) == 1));
                REQUIRE(z == Approx(20.));
            }
        }
    }
}

TEST_CASE("Spiral vase restores object layer Z after a raft tool change", "[PrintGCode][SpiralVase]")
{
    SECTION("tool change ends at an elevated clearance Z") {
        SpiralRaftGCodeResult result = spiral_raft_gcode_result("G1 X5 F12000\nG1 Z{max_layer_z + 3.0} F1200\nT[next_extruder]\n");

        REQUIRE(result.first_model_extrusion_z == Approx(result.nominal_model_layer_z));
        REQUIRE(result.highest_z_before_model >= result.nominal_model_layer_z + 2.);
        REQUIRE(result.layer_z_restores == 1);
    }

    SECTION("tool change does not alter Z") {
        SpiralRaftGCodeResult result = spiral_raft_gcode_result("T[next_extruder]\n");

        REQUIRE(result.first_model_extrusion_z == Approx(result.nominal_model_layer_z));
        REQUIRE(result.layer_z_restores == 0);
        REQUIRE(result.model_z_is_continuous);
    }

    SECTION("tool change leaves the XY position unknown at clearance Z") {
        SpiralRaftGCodeResult result = spiral_raft_gcode_result("G1 Z{max_layer_z + 3.0} F1200\nT[next_extruder]\n");

        REQUIRE(result.first_model_extrusion_z == Approx(result.nominal_model_layer_z));
        REQUIRE(result.highest_z_before_model >= result.nominal_model_layer_z + 2.);
        REQUIRE(result.layer_z_restores == 1);
    }
}
