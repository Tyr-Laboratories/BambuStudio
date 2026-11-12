#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCode/RetractWhenCrossingPerimeters.hpp"

#include <boost/filesystem.hpp>
#include <cmath>
#include <map>
#include <set>

using namespace Slic3r;

namespace {
struct PrintWorkspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("wall-extension-print-%%%%-%%%%");
    std::string old_data = data_dir(), old_temp = temporary_dir();
    PrintWorkspace() {
        boost::filesystem::create_directories(path);
        set_data_dir(path.string());
        set_temporary_dir(path.string());
    }
    ~PrintWorkspace() {
        set_data_dir(old_data);
        set_temporary_dir(old_temp);
        boost::system::error_code ec;
        boost::filesystem::remove_all(path, ec);
    }
};

DynamicPrintConfig extension_config(double extension)
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"}, {"wall_loops", "1"},
        {"top_shell_layers", "4"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "4"}, {"bottom_shell_thickness", "0"},
        {"sparse_infill_density", "10%"}, {"sparse_infill_pattern", "grid"},
        {"wall_generator", "arachne"}, {"brim_type", "no_brim"}, {"enable_arc_fitting", "0"}
    });
    config.set("wall_internal_extension", extension);
    return config;
}

// Boxes given as {size, origin} merged into one part.
Model boxes(std::initializer_list<std::pair<Vec3d, Vec3d>> parts)
{
    TriangleMesh mesh;
    for (const auto &[size, origin] : parts) {
        TriangleMesh box(its_make_cube(size.x(), size.y(), size.z()));
        box.translate(float(origin.x()), float(origin.y()), float(origin.z()));
        mesh.merge(box);
    }
    Model model;
    auto *object = model.add_object();
    object->add_volume(std::move(mesh));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

// A 30x30x12 box with a 10x10x6 tower standing on its top.
Model terrace() { return boxes({ { { 30., 30., 12. }, { 0., 0., 0. } }, { { 10., 10., 6. }, { 10., 10., 12. } } }); }
// A 10x10x6 column carrying a 30x30x4 slab.
Model overhang() { return boxes({ { { 10., 10., 6. }, { 10., 10., 0. } }, { { 30., 30., 4. }, { 0., 0., 6. } } }); }

void slice(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();
}

void collect_paths(const ExtrusionEntity &entity, std::vector<const ExtrusionPath *> &out)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity))
        out.push_back(path);
    else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const ExtrusionPath &path : multipath->paths)
            out.push_back(&path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const ExtrusionPath &path : loop->paths)
            out.push_back(&path);
    else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const ExtrusionEntity *child : collection->entities)
            collect_paths(*child, out);
}

std::vector<const ExtrusionPath *> extension_paths(const Layer &layer)
{
    std::vector<const ExtrusionPath *> all, out;
    for (const LayerRegion *region : layer.regions())
        collect_paths(region->perimeters, all);
    for (const ExtrusionPath *path : all)
        if (path->role() == erWallExtension)
            out.push_back(path);
    return out;
}

// print_z in tenths of a millimetre of every layer holding wall extension lines.
std::set<int> extension_layers(const Print &print)
{
    std::set<int> out;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        if (!extension_paths(*layer).empty())
            out.insert(int(std::lround(layer->print_z * 10.)));
    return out;
}

std::set<int> tenths(int first, int last)
{
    std::set<int> out;
    for (int z = first; z <= last; z += 2)
        out.insert(z);
    return out;
}

const Layer *layer_at(const Print &print, double print_z)
{
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        if (std::abs(layer->print_z - print_z) < EPSILON)
            return layer;
    return nullptr;
}

double area_mm2(const ExPolygons &expolygons)
{
    double area = 0.;
    for (const ExPolygon &expolygon : expolygons)
        area += expolygon.area();
    return area * SCALING_FACTOR * SCALING_FACTOR;
}

double solid_area(const Layer &layer)
{
    double area = 0.;
    for (const LayerRegion *region : layer.regions())
        for (const Surface &surface : region->fill_surfaces.surfaces)
            if (surface.surface_type == stInternalSolid || surface.surface_type == stInternalBridge || surface.surface_type == stTop ||
                surface.surface_type == stBottom || surface.surface_type == stBottomBridge)
                area += surface.expolygon.area();
    return area * SCALING_FACTOR * SCALING_FACTOR;
}
}

TEST_CASE("A tower's outer wall continues 1.8 mm into the box", "[WallExtension][Print]")
{
    PrintWorkspace workspace;
    Model model = terrace();
    Print print;
    slice(print, model, extension_config(1.));
    CHECK(extension_layers(print) == tenths(104, 120));

    const Layer *box_layer = layer_at(print, 6.0), *tower_layer = layer_at(print, 12.2);
    REQUIRE(box_layer != nullptr);
    REQUIRE(tower_layer != nullptr);
    const ExPolygons near_tower = offset_ex(tower_layer->lslices, scale_(1.0 + 0.45 + 0.05));
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const ExtrusionPath *path : extension_paths(*layer)) {
            CAPTURE(layer->print_z);
            for (const Point &point : path->polyline.points) {
                CHECK(std::any_of(box_layer->lslices.begin(), box_layer->lslices.end(), [&](const ExPolygon &e) { return e.contains(point); }));
                CHECK(std::any_of(near_tower.begin(), near_tower.end(), [&](const ExPolygon &e) { return e.contains(point); }));
            }
            // Only the lowest layer hangs over the sparse infill and is printed like a bridge.
            CHECK((path->get_customize_flag() == CustomizeFlag::cfWallExtensionBridge) == (std::abs(layer->print_z - 10.4) < EPSILON));
        }

    const auto filename = (workspace.path / "terrace.gcode").string();
    GCodeProcessorResult result;
    print.export_gcode(filename, &result);
    REQUIRE(result.print_statistics.used_filaments_per_role.count(erWallExtension) == 1);
    CHECK(result.print_statistics.used_filaments_per_role.at(erWallExtension).first > 0.);
}

TEST_CASE("A column's outer wall continues 1.8 mm into the slab", "[WallExtension][Print]")
{
    Model model = overhang();
    Print print;
    slice(print, model, extension_config(1.));
    CHECK(extension_layers(print) == tenths(62, 78));
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const ExtrusionPath *path : extension_paths(*layer))
            CHECK(path->get_customize_flag() != CustomizeFlag::cfWallExtensionBridge);
}

TEST_CASE("Fill never covers wall extension lines", "[WallExtension][Print]")
{
    for (Model model : { terrace(), overhang() }) {
        Print print;
        slice(print, model, extension_config(1.));
        size_t layers_with_area = 0;
        for (const Layer *layer : print.get_object(size_t(0))->layers())
            for (const LayerRegion *region : layer->regions()) {
                if (region->wall_extension_area.empty())
                    continue;
                ++layers_with_area;
                CAPTURE(layer->print_z);
                CHECK(area_mm2(intersection_ex(region->fill_no_overlap_expolygons, region->wall_extension_area)) < 0.01);
                CHECK(intersection_ex(region->fill_expolygons, offset_ex(region->wall_extension_area, -scale_(0.01))).empty());
            }
        CHECK(layers_with_area == 9);
    }
}

TEST_CASE("The wall extension adds no solid infill", "[WallExtension][Print]")
{
    Model model = terrace();
    Print without, with;
    slice(without, model, extension_config(0.));
    slice(with, model, extension_config(1.));
    const auto &layers_without = without.get_object(size_t(0))->layers();
    const auto &layers_with    = with.get_object(size_t(0))->layers();
    REQUIRE(layers_without.size() == layers_with.size());
    for (size_t i = 0; i < layers_with.size(); ++i) {
        CAPTURE(layers_with[i]->print_z);
        CHECK(solid_area(*layers_with[i]) <= solid_area(*layers_without[i]) + 0.05);
    }
}

namespace {
// F (mm/min) of every extruding move under " FEATURE: <feature>", by layer print_z in tenths of a millimetre.
std::map<int, std::set<int>> feature_speeds(const std::string &file, const DynamicPrintConfig &config, const std::string &feature)
{
    std::map<int, std::set<int>> out;
    GCodeReader reader;
    reader.apply_config(config);
    std::string current;
    double      z = 0.;
    REQUIRE(reader.parse_file(file, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.rfind(" Z_HEIGHT: ", 0) == 0)
            z = std::stod(std::string(comment.substr(11)));
        if (const auto tag = comment.find(" FEATURE: "); tag != std::string_view::npos)
            current = std::string(comment.substr(tag + 10));
        if (current == feature && line.extruding(self) && line.dist_XY(self) > 0.)
            out[int(std::lround(z * 10.))].insert(int(std::lround(line.new_F(self))));
    }));
    return out;
}

// Start of every outer wall, by layer print_z in tenths of a millimetre, rounded to 0.01 mm.
std::map<int, std::set<std::pair<long, long>>> outer_wall_starts(const std::string &file, const DynamicPrintConfig &config)
{
    std::map<int, std::set<std::pair<long, long>>> out;
    GCodeReader reader;
    reader.apply_config(config);
    std::string current;
    bool        started = false;
    double      z       = 0.;
    REQUIRE(reader.parse_file(file, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.rfind(" Z_HEIGHT: ", 0) == 0)
            z = std::stod(std::string(comment.substr(11)));
        if (const auto tag = comment.find(" FEATURE: "); tag != std::string_view::npos) {
            current = std::string(comment.substr(tag + 10));
            started = false;
        }
        if (current == "Outer wall" && !started && line.extruding(self) && line.dist_XY(self) > 0.) {
            out[int(std::lround(z * 10.))].insert({ std::lround(self.x() * 100.), std::lround(self.y() * 100.) });
            started = true;
        }
    }));
    return out;
}

DynamicPrintConfig gcode_config(double extension)
{
    DynamicPrintConfig config = extension_config(extension);
    config.set_deserialize_strict({ {"inner_wall_speed", "77"}, {"bridge_speed", "23"}, {"filament_max_volumetric_speed", "100"},
                                    {"slow_down_for_layer_cooling", "0"} });
    return config;
}
}

TEST_CASE("Fine layers still get the extension", "[WallExtension][Print]")
{
    Model model = boxes({ { { 30., 30., 4. }, { 0., 0., 0. } }, { { 10., 10., 2. }, { 10., 10., 4. } } });
    DynamicPrintConfig config = extension_config(1.);
    config.set_deserialize_strict({ {"layer_height", "0.04"} });
    Print print;
    slice(print, model, config);
    CHECK_FALSE(extension_layers(print).empty());
}

TEST_CASE("The lowest extension layer prints at bridge speed in the G-code", "[WallExtension][Print][GCode]")
{
    PrintWorkspace workspace;
    Model model = terrace();
    const DynamicPrintConfig config = gcode_config(1.);
    Print print;
    slice(print, model, config);
    const auto filename = (workspace.path / "bridge.gcode").string();
    GCodeProcessorResult result;
    print.export_gcode(filename, &result);
    const auto speeds = feature_speeds(filename, config, "Wall extension");
    REQUIRE(speeds.count(104) == 1);
    CHECK(speeds.at(104) == std::set<int>{ 23 * 60 });
    for (const auto &[z, f] : speeds)
        if (z != 104) {
            CAPTURE(z);
            CHECK(f == std::set<int>{ 77 * 60 });
        }
}

TEST_CASE("Wall extension lines slow down with height like the walls", "[WallExtension][Print][GCode]")
{
    PrintWorkspace workspace;
    Model model = terrace();
    DynamicPrintConfig config = gcode_config(1.);
    config.set_deserialize_strict({ {"enable_height_slowdown", "1"}, {"slowdown_start_height", "1"}, {"slowdown_end_height", "2"},
                                    {"slowdown_start_speed", "1000"}, {"slowdown_end_speed", "20"},
                                    {"slowdown_start_acc", "100000"}, {"slowdown_end_acc", "100000"} });
    Print print;
    slice(print, model, config);
    const auto filename = (workspace.path / "slowdown.gcode").string();
    GCodeProcessorResult result;
    print.export_gcode(filename, &result);
    const auto speeds = feature_speeds(filename, config, "Wall extension");
    REQUIRE_FALSE(speeds.empty());
    for (const auto &[z, f] : speeds) {
        CAPTURE(z);
        CHECK(*f.rbegin() <= 20 * 60);
    }
}

TEST_CASE("The wall extension does not move outer wall seams", "[WallExtension][Print][GCode]")
{
    PrintWorkspace workspace;
    Model model = terrace();
    std::map<int, std::set<std::pair<long, long>>> starts[2];
    for (int e = 0; e < 2; ++e) {
        const DynamicPrintConfig config = gcode_config(double(e));
        Print print;
        slice(print, model, config);
        const auto filename = (workspace.path / ("seams" + std::to_string(e) + ".gcode")).string();
        GCodeProcessorResult result;
        print.export_gcode(filename, &result);
        starts[e] = outer_wall_starts(filename, config);
    }
    CHECK(starts[0] == starts[1]);
}

TEST_CASE("Travel across wall extension lines does not count as crossing a wall", "[WallExtension][Print]")
{
    Model model = terrace();
    Print print;
    slice(print, model, extension_config(1.));
    const Layer *layer = layer_at(print, 11.0), *tower_layer = layer_at(print, 12.2);
    REQUIRE(layer != nullptr);
    REQUIRE(tower_layer != nullptr);
    REQUIRE_FALSE(extension_paths(*layer).empty());
    // From the middle of the tower's footprint to 9 mm off it: across the extension and its chamfer, inside the box.
    const Point centre = get_extents(tower_layer->lslices).center();
    Polyline travel;
    travel.points = { centre, centre + Point::new_scale(9., 0.) };
    RetractWhenCrossingPerimeters checker;
    CHECK(checker.travel_inside_internal_regions_no_wall_crossing(*layer, travel, 0, 0));
}

TEST_CASE("With slope walls on, the wall extension still adds no solid infill", "[WallExtension][Print]")
{
    // The box, a ridge beside it whose sloped faces get slope walls up to the box's top, and the tower.
    auto ridge_terrace = [] {
        TriangleMesh mesh(its_make_cube(30., 30., 12.));
        TriangleMesh ridge(its_make_prism(10., 30., 14.));
        ridge.translate(34., 15., 0.);
        mesh.merge(ridge);
        TriangleMesh tower(its_make_cube(10., 10., 6.));
        tower.translate(10., 10., 12.);
        mesh.merge(tower);
        Model model;
        auto *object = model.add_object();
        object->add_volume(std::move(mesh));
        object->add_instance()->set_offset(Vec3d(80., 80., 0.));
        return model;
    };
    Model model = ridge_terrace();
    Print without, with;
    DynamicPrintConfig config0 = extension_config(0.), config1 = extension_config(1.);
    config0.set_deserialize_strict({ {"extra_wall_slope_angle", "15"} });
    config1.set_deserialize_strict({ {"extra_wall_slope_angle", "15"} });
    slice(without, model, config0);
    slice(with, model, config1);
    const auto &layers_without = without.get_object(size_t(0))->layers();
    const auto &layers_with    = with.get_object(size_t(0))->layers();
    REQUIRE(layers_without.size() == layers_with.size());
    bool both_on_one_layer = false;
    for (size_t i = 0; i < layers_with.size(); ++i) {
        CAPTURE(layers_with[i]->print_z);
        CHECK(solid_area(*layers_with[i]) <= solid_area(*layers_without[i]) + 0.05);
        both_on_one_layer |= !extension_paths(*layers_with[i]).empty() &&
            std::any_of(layers_with[i]->regions().begin(), layers_with[i]->regions().end(),
                        [](const LayerRegion *region) { return region->has_slope_wall_reinforcement; });
    }
    CHECK(both_on_one_layer);
}

TEST_CASE("Extension 0 or spiral vase adds nothing", "[WallExtension][Print]")
{
    Model model = terrace();
    {
        Print print;
        slice(print, model, extension_config(0.));
        CHECK(extension_layers(print).empty());
        for (const Layer *layer : print.get_object(size_t(0))->layers())
            for (const LayerRegion *region : layer->regions())
                CHECK(region->wall_extension_area.empty());
    }
    {
        DynamicPrintConfig config = extension_config(1.);
        config.set_deserialize_strict({ {"spiral_mode", "1"}, {"sparse_infill_density", "0%"}, {"top_shell_layers", "0"} });
        Print print;
        slice(print, model, config);
        CHECK(extension_layers(print).empty());
    }
}
