#include <catch2/catch.hpp>
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Fill/Lightning/TreeNode.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/TreeSupport.hpp"
#include "libslic3r/Support/TreeSupportCommon.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/GCode/PostProcessor.hpp"
#include "libslic3r/GCode/GCodeResultSwap.hpp"
#include "libslic3r/GCodeWriter.hpp"

#include <future>
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>

using namespace Slic3r;

TEST_CASE("Shared-nozzle filament state belongs to one plate writer", "[ParallelPlate][Extruder]")
{
    const bool relative = GENERATE(false, true);
    auto prepare = [&](GCodeWriter &writer) {
        writer.config.single_extruder_multi_material.value = true;
        writer.config.use_relative_e_distances.value = relative;
        writer.config.filament_map.values = {1, 1, 2};
        writer.set_extruders({0, 1, 2});
        writer.init_extruder(0, 0);
    };
    GCodeWriter a, b;
    prepare(a);
    a.filament()->extrude(10.);
    REQUIRE(a.filament()->retract(1., 0.) == Approx(1.));
    REQUIRE(a.extruders()[1].get_share_retracted_length() == Approx(1.));
    REQUIRE(a.extruders()[2].get_share_retracted_length() == Approx(0.));
    const double axis_a = a.filament()->E();
    prepare(b);
    b.filament()->extrude(20.);
    b.filament()->retract(2., 0.);
    REQUIRE(a.filament()->E() == Approx(axis_a));
    REQUIRE(a.filament()->get_share_retracted_length() == Approx(1.));
    REQUIRE(a.filament()->unretract() == Approx(1.));
    REQUIRE(b.filament()->get_share_retracted_length() == Approx(2.));
    REQUIRE(b.filament()->unretract() == Approx(2.));
}

TEST_CASE("Interleaved plate processors restore their own flush roles", "[ParallelPlate][Processor]")
{
    GCodeProcessor a, b;
    a.initialize("plate-a.gcode");
    b.initialize("plate-b.gcode");
    const std::string begin = GENERATE(" FLUSH_START", " VFLUSH_START");
    const std::string end = begin == " FLUSH_START" ? " FLUSH_END" : " VFLUSH_END";
    a.process_buffer("G90\nM83\nG1 Z0.2\n; FEATURE: Outer wall\n;" + begin + "\n");
    b.process_buffer("G90\nM83\nG1 Z0.2\n; FEATURE: Support\n;" + begin + "\n");
    a.process_buffer(";" + end + "\nG1 X10 E1 F1200\n");
    b.process_buffer(";" + end + "\nG1 X20 E1 F1200\n");
    REQUIRE(a.get_result().moves.back().extrusion_role == erExternalPerimeter);
    REQUIRE(b.get_result().moves.back().extrusion_role == erSupportMaterial);
}

TEST_CASE("Plate printer context remains local during bed leveling timing", "[ParallelPlate][Processor]")
{
    PrintConfig config;
    config.machine_prepare_compensation_time.value = 20.;
    auto prepare = [&](GCodeProcessor &processor, bool bbl) {
        processor.apply_config(config);
        processor.set_is_bbl_printer(bbl);
        processor.initialize("timing.gcode");
    };
    auto finish = [](GCodeProcessor &processor) {
        processor.process_buffer("G29\nG1 X10 F1200\nG1 X20\nG1 X30\n");
        processor.finalize(false);
        return processor.get_time(PrintEstimatedStatistics::ETimeMode::Normal);
    };
    GCodeProcessor serial_bbl, serial_other, bbl, other;
    prepare(serial_bbl, true);
    const float expected_bbl = finish(serial_bbl);
    prepare(serial_other, false);
    const float expected_other = finish(serial_other);
    REQUIRE(expected_other > expected_bbl + 19.f);
    prepare(bbl, true);
    prepare(other, false);
    REQUIRE(finish(bbl) == Approx(expected_bbl));
    REQUIRE(finish(other) == Approx(expected_other));
}

TEST_CASE("Independent plates slice and export equivalent serial and concurrent G-code", "[ParallelPlate][Native]")
{
    namespace fs = boost::filesystem;
    const fs::path directory = fs::temp_directory_path() / fs::unique_path("parallel-plates-%%%%-%%%%");
    fs::create_directories(directory);
    struct Cleanup { fs::path path; ~Cleanup() { boost::system::error_code ec; fs::remove_all(path, ec); } } cleanup{directory};
    set_data_dir(directory.string());
    set_temporary_dir(directory.string());

    auto make_plate = [](int variant) {
        Model model;
        model.plates_custom_gcodes[0].mode = CustomGCode::SingleExtruder;
        model.plates_custom_gcodes[0].gcodes.push_back(
            {0.6, CustomGCode::Custom, 1, "", variant ? "M117 Plate B" : "M117 Plate A"});
        auto *object = model.add_object();
        object->name = "Parallel plate fixture";
        TriangleMesh mesh(its_make_cube(5., 14., 12.));
        TriangleMesh roof(its_make_cube(22., 14., 1.));
        roof.translate(0., 0., 12.);
        mesh.merge(roof);
        object->add_volume(mesh);
        object->add_instance()->set_offset(Vec3d(80., 80., 0.));
        // Build the same fully typed options used when profiles are loaded.
        // Cloning static vector-enum defaults directly omits their serialization maps.
        DynamicPrintConfig config;
        for (const auto &key : FullPrintConfig::defaults().keys())
            config.set_key_value(key, print_config_def.get(key)->create_default_option());
        config.set_deserialize_strict({
            {"layer_height", "0.3"}, {"initial_layer_print_height", "0.3"},
            {"enable_support", "1"}, {"support_type", "tree(auto)"},
            {"support_style", "tree_organic"}, {"support_on_build_plate_only", "1"},
            {"support_interface_top_layers", "0"}, {"support_interface_bottom_layers", "0"},
            {"sparse_infill_pattern", variant ? "lightning" : "grid"},
            {"sparse_infill_density", "15%"}, {"brim_type", "no_brim"},
            {"enable_prime_tower", "0"}, {"enable_arc_fitting", "0"},
            {"single_extruder_multi_material", "1"},
            {"machine_start_gcode", ""}, {"machine_end_gcode", ""}
        });
        auto print = std::make_unique<Print>();
        print->apply(model, config);
        return print;
    };
    auto slice = [&](Print &print, const std::string &name) {
        print.process();
        GCodeProcessorResult result;
        const std::string path = (directory / (name + ".gcode")).string();
        print.export_gcode(path, &result);
        std::ifstream file(path);
        std::string line, commands;
        while (std::getline(file, line)) {
            // Wall-clock timestamps, output paths and object ids are metadata.
            if (!line.empty() && line.front() != ';' && line.rfind("M486", 0) != 0 && line.rfind("M624", 0) != 0)
                commands += line.substr(0, line.find(';')) + "\n";
        }
        return std::make_pair(commands, result.moves.size());
    };
    auto a = make_plate(0), b = make_plate(1), pa = make_plate(0), pb = make_plate(1);
    const auto serial_a = slice(*a, "serial-a"), serial_b = slice(*b, "serial-b");
    ToolOrdering ordering_a = a->tool_ordering();
    ToolOrdering ordering_b = b->tool_ordering();
    ordering_a.assign_custom_gcodes(*a);
    ordering_b.assign_custom_gcodes(*b);
    auto custom_commands = [](const ToolOrdering &ordering) {
        std::vector<std::string> commands;
        for (const LayerTools &layer : ordering.layer_tools())
            if (layer.custom_gcode) commands.push_back(layer.custom_gcode->extra);
        return commands;
    };
    REQUIRE(custom_commands(ordering_a) == std::vector<std::string>{"M117 Plate A"});
    REQUIRE(custom_commands(ordering_b) == std::vector<std::string>{"M117 Plate B"});
    // A failed finalization must force export again when its private result is
    // discarded, preserving the already computed slice and support geometry.
    REQUIRE(a->finished());
    a->set_gcode_file_invalidated();
    REQUIRE_FALSE(a->finished());
    REQUIRE(a->get_object(0)->is_step_done(posSupportMaterial));
    const auto retried_a = slice(*a, "retry-a");
    REQUIRE(retried_a == serial_a);
    TreeSupport3D::TreeSupportMeshGroupSettings mesh_settings(*a->get_object(0));
    mesh_settings.support_xy_distance = mesh_settings.support_xy_distance_overhang = scale_(0.03);
    mesh_settings.support_top_distance = scale_(0.2);
    const auto &slicing = a->get_object(0)->slicing_parameters();
    TreeSupport3D::TreeSupportSettings soluble(mesh_settings, slicing, true);
    TreeSupport3D::TreeSupportSettings nonsoluble(mesh_settings, slicing);
    REQUIRE(soluble.xy_min_distance == scale_(0.1));
    REQUIRE(nonsoluble.xy_min_distance == mesh_settings.support_xy_distance);
    const ToolOrdering copied_a = ordering_a;
    ordering_a.assign_custom_gcodes(*b);
    REQUIRE(custom_commands(copied_a) == std::vector<std::string>{"M117 Plate A"});
    auto concurrent_a = std::async(std::launch::async, [&] { return slice(*pa, "parallel-a"); });
    auto concurrent_b = std::async(std::launch::async, [&] { return slice(*pb, "parallel-b"); });
    REQUIRE(serial_a.second > 100);
    REQUIRE(serial_b.second > 100);
    REQUIRE(concurrent_a.get() == serial_a);
    REQUIRE(concurrent_b.get() == serial_b);
}

#ifndef _WIN32
TEST_CASE("Post-process scripts receive plate-local environments", "[ParallelPlate][PostProcess]")
{
    namespace fs = boost::filesystem;
    const fs::path directory = fs::temp_directory_path() / fs::unique_path("plate-scripts-%%%%-%%%%");
    fs::create_directories(directory);
    struct Cleanup { fs::path path; ~Cleanup() { boost::system::error_code ec; fs::remove_all(path, ec); } } cleanup{directory};
    const char *before = std::getenv("SLIC3R_LAYER_HEIGHT");
    const std::string saved = before ? before : "<unset>";
    const fs::path script = directory / "check.sh";
    std::ofstream(script.string()) << "sleep 0.1\nprintf '%s' \"$SLIC3R_LAYER_HEIGHT\" > \"$1\"\n";
    auto run = [&](const std::string &height) {
        DynamicPrintConfig config;
        config.set_deserialize_strict({{"layer_height", height}});
        config.set_key_value("post_process", new ConfigOptionStrings({"/bin/sh " + script.string()}));
        std::string path = (directory / (height + ".gcode")).string();
        std::ofstream(path) << "G1 X10\n";
        std::string output = path;
        run_post_process_scripts(path, false, "File", output, config);
        std::ifstream file(path);
        std::string value;
        file >> value;
        return value;
    };
    auto a = std::async(std::launch::async, [&] { return run("0.1"); });
    auto b = std::async(std::launch::async, [&] { return run("0.3"); });
    REQUIRE(a.get() == "0.1");
    REQUIRE(b.get() == "0.3");
    const char *after = std::getenv("SLIC3R_LAYER_HEIGHT");
    REQUIRE(std::string(after ? after : "<unset>") == saved);
}
#endif

TEST_CASE("Lightning conversion does not depend on other plates", "[ParallelPlate][Lightning]")
{
    auto root = FillLightning::Node::create(Point(1000000, 2000000));
    for (int i = 0; i < 7; ++i)
        root->addChild(Point(i * 1000000, 10000000));
    auto convert = [&] {
        Polylines paths;
        root->convertToPolylines(paths, 200000);
        std::vector<Points> result;
        for (const auto &path : paths) result.push_back(path.points);
        return result;
    };
    const auto expected = convert();
    for (int i = 0; i < 8; ++i) {
        auto a = std::async(std::launch::async, convert);
        auto b = std::async(std::launch::async, convert);
        REQUIRE(a.get() == expected);
        REQUIRE(b.get() == expected);
    }
}

TEST_CASE("Publishing a plate preserves nozzle metadata and transfers buffers", "[ParallelPlate][Result]")
{
    GCodeProcessorResult published{}, worker{};
    worker.moves.resize(3);
    const auto *buffer = worker.moves.data();
    worker.nozzle_group_result = std::make_shared<MultiNozzleUtils::LayeredNozzleGroupResult>();
    const auto nozzle_group = worker.nozzle_group_result;
    worker.filament_maps = {2, 1};
    worker.required_nozzle_HRC = {60, 40};
    worker.printer_extruder_variant = {"High Flow", "Standard"};
    worker.filament_vitrification_temperature = {60, 70};
    worker.filament_change_sequence = {0, 1, 0};
    worker.nozzle_change_sequence = {1, 2};
    worker.optimal_assignment = {1, 0};
    worker.extruder_heights = {100., 200.};
    worker.initial_layer_time = 25.;
    worker.filename = "second-plate.gcode";
    swap_gcode_results(published, worker);
    REQUIRE(published.moves.data() == buffer);
    REQUIRE(worker.moves.empty());
    REQUIRE(published.nozzle_group_result == nozzle_group);
    REQUIRE(published.filament_maps == std::vector<int>{2, 1});
    REQUIRE(published.required_nozzle_HRC == std::vector<int>{60, 40});
    REQUIRE(published.printer_extruder_variant == std::vector<std::string>{"High Flow", "Standard"});
    REQUIRE(published.filament_vitrification_temperature == std::vector<int>{60, 70});
    REQUIRE(published.filament_change_sequence == std::vector<unsigned>{0, 1, 0});
    REQUIRE(published.nozzle_change_sequence == std::vector<unsigned>{1, 2});
    REQUIRE(published.optimal_assignment == std::vector<int>{1, 0});
    REQUIRE(published.extruder_heights == std::vector<double>{100., 200.});
    REQUIRE(published.initial_layer_time == Approx(25.));
    REQUIRE(published.filename == "second-plate.gcode");
}

TEST_CASE("Classical tree nodes keep each object's growth setting", "[ParallelPlate][Support]")
{
    TreeSupportData a, b;
    a.diameter_angle_scale_factor = 0.2;
    b.diameter_angle_scale_factor = 0.5;
    auto parent = [](TreeSupportData &data) {
        return data.create_node(Point(0, 0), 0, 5, 0, true, nullptr, 1., 0.2, 0., 1.);
    };
    auto *pa = parent(a), *pb = parent(b);
    auto child = [](TreeSupportData &data, SupportNode *node) {
        return data.create_node(Point(0, 0), 1, 4, 0, true, node, 0.8, 0.2, 0.2, 0.);
    };
    REQUIRE(child(b, pb)->radius == Approx(1.1));
    REQUIRE(child(a, pa)->radius == Approx(1.04));
}
