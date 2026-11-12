#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"

#include <boost/filesystem.hpp>
#include <cmath>
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
