#include <catch2/catch.hpp>

#include "libslic3r/Model.hpp"
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

// Most cases here build the chamfer, which is off by default.
DynamicPrintConfig extension_config(double extension, bool chamfer = true)
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
    config.set("wall_internal_extension_chamfer", chamfer);
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

double surface_area(const Layer &layer, SurfaceType type)
{
    double area = 0.;
    for (const LayerRegion *region : layer.regions())
        for (const Surface &surface : region->fill_surfaces.surfaces)
            if (surface.surface_type == type)
                area += surface.expolygon.area();
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

// Solid fill area of `layer` other than the internal bridges the wall extension stands on and the stock strips of solid
// infill that bridge_over_infill() adds under their ends, on the layer below a bridge.
double solid_area_besides_extension_bridges(const Layer &layer)
{
    const auto extension_bridges = [](const Layer &on) {
        ExPolygons bases, out;
        for (const LayerRegion *region : on.regions())
            append(bases, region->wall_extension_bridge_base);
        if (!bases.empty())
            for (const LayerRegion *region : on.regions())
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == stInternalBridge && !intersection_ex({ surface.expolygon }, bases).empty())
                        out.push_back(surface.expolygon);
        return out;
    };
    const ExPolygons here  = extension_bridges(layer);
    const ExPolygons above = layer.upper_layer ? extension_bridges(*layer.upper_layer) : ExPolygons{};
    double           area  = solid_area(layer);
    for (const LayerRegion *region : layer.regions())
        for (const Surface &surface : region->fill_surfaces.surfaces)
            if (surface.surface_type == stInternalBridge)
                area -= area_mm2(intersection_ex({ surface.expolygon }, here));
            else if (surface.surface_type == stInternalSolid)
                area -= area_mm2(intersection_ex({ surface.expolygon }, above));
    return area;
}

// Internal bridges of `layer` resting on the layer below (not follow-ups) that overlap `near`.
ExPolygons internal_bridges(const Layer &layer, const ExPolygons &near)
{
    ExPolygons out;
    for (const LayerRegion *region : layer.regions())
        for (const Surface &surface : region->fill_surfaces.surfaces)
            if (surface.surface_type == stInternalBridge && !surface.bridge_followup && !intersection_ex({ surface.expolygon }, near).empty())
                out.push_back(surface.expolygon);
    return out;
}

// Ends of the bridge strands of `layer`'s internal bridges overlapping `near`, and how many of them rest on an extrusion
// of the layer below: {anchored, total}. A strand runs along its surface's bridge direction; the short connectors
// between strands do not. An end rests when the last bead, up to two widths of it, overlaps what is printed below.
std::pair<size_t, size_t> anchored_bridge_ends(const Layer &layer, const ExPolygons &near)
{
    Polygons deposited;
    for (const LayerRegion *region : layer.lower_layer->regions()) {
        region->perimeters.polygons_covered_by_width(deposited, float(SCALED_EPSILON));
        region->fills.polygons_covered_by_width(deposited, float(SCALED_EPSILON));
    }
    deposited = union_(deposited);
    size_t anchored = 0, total = 0;
    for (const LayerRegion *region : layer.regions()) {
        std::vector<const ExtrusionPath *> paths;
        collect_paths(region->fills, paths);
        for (const ExtrusionPath *path : paths) {
            if (path->role() != erBridgeInfill)
                continue;
            for (const Line &line : path->polyline.lines()) {
                const Vec2d direction = (line.b - line.a).cast<double>().normalized();
                bool        strand    = false;
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == stInternalBridge && !surface.bridge_followup && surface.expolygon.contains(line.midpoint()) &&
                        !intersection_ex({ surface.expolygon }, near).empty())
                        strand |= std::abs(direction.dot(Vec2d(std::cos(surface.bridge_angle), std::sin(surface.bridge_angle)))) > 0.9999;
                if (!strand)
                    continue;
                const Point delta = (direction * std::min(line.length() * 0.49, scale_(2. * path->width))).cast<coord_t>();
                const auto  rests = [&](const Point &a, const Point &b) {
                    return area(intersection_ex(offset(Polyline{ a, b }, float(scale_(0.5 * path->width))), deposited)) >
                           double(SCALED_EPSILON) * SCALED_EPSILON;
                };
                anchored += size_t(rests(line.a, line.a + delta)) + size_t(rests(line.b - delta, line.b));
                total += 2;
            }
        }
    }
    return { anchored, total };
}
}

TEST_CASE("Under a top surface the extension stands on an internal bridge anchored like the shell's", "[WallExtension][Print]")
{
    // Bridge Aware Cubic builds its lattice where internal bridges need it, so it must see this bridge as one too.
    const std::string pattern = GENERATE(as<std::string>{}, "grid", "bridgeawarecubic");
    CAPTURE(pattern);
    DynamicPrintConfig config_without = extension_config(0., false), config_with = extension_config(1., false);
    config_without.set_deserialize_strict({ {"sparse_infill_pattern", pattern} });
    config_with.set_deserialize_strict({ {"sparse_infill_pattern", pattern} });
    Model model = terrace();
    Print without, with;
    slice(without, model, config_without);
    slice(with, model, config_with);
    const Layer *bridge_layer = layer_at(with, 10.4), *first_lines = layer_at(with, 10.6), *tower_layer = layer_at(with, 12.2);
    REQUIRE(bridge_layer != nullptr);
    REQUIRE(first_lines != nullptr);
    REQUIRE(tower_layer != nullptr);
    // The deepest layer, 1.8 mm below the box's top, has only the sparse infill below it. Like any solid that starts over
    // sparse infill, it is an internal bridge, and the extension lines start on the layer above it.
    const ExPolygons under_wall = diff_ex(tower_layer->lslices, offset_ex(tower_layer->lslices, -scale_(0.45 + 0.05)));
    const ExPolygons bridges    = internal_bridges(*bridge_layer, under_wall);
    REQUIRE_FALSE(bridges.empty());
    CHECK(extension_paths(*bridge_layer).empty());
    Polylines first;
    for (const ExtrusionPath *path : extension_paths(*first_lines))
        first.push_back(path->polyline);
    REQUIRE_FALSE(first.empty());
    CHECK(total_length(diff_pl(first, offset_ex(bridges, float(SCALED_EPSILON)))) < scale_(with.config().resolution.value));
    // The bridge's strands rest at both ends on what is printed below, as well as those of the box's own internal bridge
    // under its top shell, which the same machinery anchors.
    const Layer *shell_bridge_layer = layer_at(without, 11.4);
    REQUIRE(shell_bridge_layer != nullptr);
    const auto [anchored, ends]             = anchored_bridge_ends(*bridge_layer, under_wall);
    const auto [shell_anchored, shell_ends] = anchored_bridge_ends(*shell_bridge_layer, shell_bridge_layer->lslices);
    CAPTURE(anchored, ends, shell_anchored, shell_ends);
    REQUIRE(ends > 0);
    REQUIRE(shell_ends > 0);
    CHECK(double(anchored) / double(ends) >= double(shell_anchored) / double(shell_ends));
}

TEST_CASE("A tower's outer wall continues 1.8 mm into the box", "[WallExtension][Print]")
{
    PrintWorkspace workspace;
    Model model = terrace();
    Print print;
    slice(print, model, extension_config(1.));
    // Down to 1.8 mm below the box's top, whose deepest layer is the internal bridge the lines stand on; the top layer
    // itself keeps its stock top surface.
    CHECK(extension_layers(print) == tenths(106, 118));

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
        }

    const auto filename = (workspace.path / "terrace.gcode").string();
    GCodeProcessorResult result;
    print.export_gcode(filename, &result);
    REQUIRE(result.print_statistics.used_filaments_per_role.count(erWallExtension) == 1);
    CHECK(result.print_statistics.used_filaments_per_role.at(erWallExtension).first > 0.);
}

TEST_CASE("A column's outer wall continues into the slab above the bridge layer, to 1.8 mm", "[WallExtension][Print]")
{
    Model model = overhang();
    Print print;
    slice(print, model, extension_config(1.));
    // The bridge layer at 6.2 keeps its anchors on the column.
    CHECK(extension_layers(print) == tenths(64, 78));
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const ExtrusionPath *path : extension_paths(*layer))
            CHECK(path->get_customize_flag() != CustomizeFlag::cfWallExtensionBridge);
}

TEST_CASE("Fill never covers wall extension lines", "[WallExtension][Print]")
{
    // Seven layers under the tower, whose surface layer has none and whose deepest layer is a bridge; eight over the
    // column, whose bridge layer has none.
    for (auto [model, expected_layers] : { std::pair<Model, size_t>{ terrace(), 7 }, std::pair<Model, size_t>{ overhang(), 8 } }) {
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
        CHECK(layers_with_area == expected_layers);
    }
}

TEST_CASE("The wall extension adds no solid infill besides the bridge it stands on", "[WallExtension][Print]")
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
        CHECK(solid_area_besides_extension_bridges(*layers_with[i]) <= solid_area(*layers_without[i]) + 0.05);
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

TEST_CASE("Every wall extension line prints at inner wall speed in the G-code", "[WallExtension][Print][GCode]")
{
    PrintWorkspace workspace;
    Model model = terrace();
    const DynamicPrintConfig config = gcode_config(1.);
    Print print;
    slice(print, model, config);
    const auto filename = (workspace.path / "speeds.gcode").string();
    GCodeProcessorResult result;
    print.export_gcode(filename, &result);
    const auto speeds = feature_speeds(filename, config, "Wall extension");
    // The lines start on the bridge at 10.4, which prints as the internal bridge it is.
    CHECK(speeds.count(104) == 0);
    REQUIRE(speeds.count(106) == 1);
    for (const auto &[z, f] : speeds) {
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
        CHECK(solid_area_besides_extension_bridges(*layers_with[i]) <= solid_area(*layers_without[i]) + 0.05);
        // Slope-wall layers expand their top, bottom and bridge surfaces by the depth of their walls; the extension
        // lines are not walls and must not make those surfaces spread over the solid infill.
        for (SurfaceType type : { stTop, stBottom, stBottomBridge }) {
            CAPTURE(int(type));
            CHECK(surface_area(*layers_with[i], type) <= surface_area(*layers_without[i], type) + 0.05);
        }
        both_on_one_layer |= !extension_paths(*layers_with[i]).empty() &&
            std::any_of(layers_with[i]->regions().begin(), layers_with[i]->regions().end(),
                        [](const LayerRegion *region) { return region->has_slope_wall_reinforcement; });
    }
    CHECK(both_on_one_layer);
}

namespace {
Model model_of(TriangleMesh mesh)
{
    Model model;
    auto *object = model.add_object();
    object->add_volume(std::move(mesh));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

// A block along x whose top rises from `low` at x = 0 to `high` at x = length.
TriangleMesh ramp(float length, float width, float low, float high)
{
    indexed_triangle_set its;
    its.vertices = { { 0.f, 0.f, 0.f }, { length, 0.f, 0.f }, { length, width, 0.f }, { 0.f, width, 0.f },
                     { 0.f, 0.f, low }, { length, 0.f, high }, { length, width, high }, { 0.f, width, low } };
    its.indices  = { { 0, 2, 1 }, { 0, 3, 2 }, { 4, 5, 6 }, { 4, 6, 7 }, { 0, 1, 5 }, { 0, 5, 4 },
                     { 3, 7, 6 }, { 3, 6, 2 }, { 0, 4, 7 }, { 0, 7, 3 }, { 1, 2, 6 }, { 1, 6, 5 } };
    return TriangleMesh(its);
}

bool inside(const ExPolygons &area, const Point &point)
{
    return std::any_of(area.begin(), area.end(), [&point](const ExPolygon &e) { return e.contains(point); });
}

// Total height of the layers holding wall extension lines or the internal bridge the extension starts on.
double heights_with_extension(const Print &print)
{
    double sum = 0.;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        if (!extension_paths(*layer).empty() ||
            std::any_of(layer->regions().begin(), layer->regions().end(), [](const LayerRegion *region) { return !region->wall_extension_bridge_base.empty(); }))
            sum += layer->height;
    return sum;
}
}

TEST_CASE("With the chamfer off the tower's extension stays under its wall", "[WallExtension][Print]")
{
    Model model = terrace();
    Print print;
    slice(print, model, extension_config(1., false));
    // As deep as with the chamfer: down to 1.8 mm below the box's top, whose deepest layer is the bridge.
    CHECK(extension_layers(print) == tenths(106, 118));
    const Layer *tower_layer = layer_at(print, 12.2);
    REQUIRE(tower_layer != nullptr);
    // Every line runs right under the tower's wall: inside its outline, within one line width of it.
    const ExPolygons under_wall = diff_ex(tower_layer->lslices, offset_ex(tower_layer->lslices, -scale_(0.45 + 0.05)));
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const ExtrusionPath *path : extension_paths(*layer)) {
            CAPTURE(layer->print_z);
            for (const Point &point : path->polyline.points)
                CHECK(inside(under_wall, point));
        }
}

TEST_CASE("A one-line fin is extended", "[WallExtension][Print]")
{
    Model model = boxes({ { { 30., 30., 6. }, { 0., 0., 0. } }, { { 0.6, 20., 5. }, { 14.7, 5., 6. } } });
    Print print;
    slice(print, model, extension_config(1.));
    // Down to 1.8 mm below the box's top, whose deepest layer is the bridge the lines stand on.
    CHECK(extension_layers(print) == tenths(46, 58));
}

TEST_CASE("A rib standing on a sloped skin is extended along its base", "[WallExtension][Print]")
{
    // A 10 degree skin rising along the rib, from 1 mm to 1 + 30 tan(10) = 6.29 mm; the rib stands 15 mm tall through it.
    TriangleMesh mesh = ramp(30.f, 30.f, 1.f, 6.29f);
    TriangleMesh rib(its_make_cube(30., 1.2, 15.));
    rib.translate(0.f, 14.4f, 0.f);
    mesh.merge(rib);
    Model model = model_of(std::move(mesh));
    Print print;
    slice(print, model, extension_config(1.));
    CHECK(extension_layers(print).size() >= 10);
    const Layer *rib_only = layer_at(print, 10.0);
    REQUIRE(rib_only != nullptr);
    const ExPolygons near_rib = offset_ex(rib_only->lslices, scale_(1.0 + 0.45 + 0.05));
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const ExtrusionPath *path : extension_paths(*layer)) {
            CAPTURE(layer->print_z);
            CHECK(layer->print_z < 6.4);
            for (const Point &point : path->polyline.points)
                CHECK(inside(near_rib, point));
        }
}

TEST_CASE("Only the outer wall position is extended on two-wall parts", "[WallExtension][Print]")
{
    // "all top", the default, gives the box's top layer its second wall only around the area under the tower: one layer
    // of inner wall right under the tower's outer wall, lying on the shell.
    const std::string top_one_wall = GENERATE(as<std::string>{}, "all top", "not apply");
    CAPTURE(top_one_wall);
    Model model = terrace();
    DynamicPrintConfig config = extension_config(1.);
    config.set_deserialize_strict({ {"wall_loops", "2"}, {"top_one_wall_type", top_one_wall} });
    Print print;
    slice(print, model, config);
    const Layer *tower_layer = layer_at(print, 12.2);
    REQUIRE(tower_layer != nullptr);
    const ExPolygons outer_wall_band = diff_ex(offset_ex(tower_layer->lslices, scale_(0.5)), offset_ex(tower_layer->lslices, -scale_(0.5)));
    for (double z : { 11.4, 11.6, 11.8, 12.0 }) {
        CAPTURE(z);
        const Layer *layer = layer_at(print, z);
        REQUIRE(layer != nullptr);
        const std::vector<const ExtrusionPath *> paths = extension_paths(*layer);
        // On the box's top layer that inner wall may already fill the extension's place.
        if (z < 12.0 - EPSILON)
            CHECK(!paths.empty());
        for (const ExtrusionPath *path : paths)
            for (const Point &point : path->polyline.points)
                CHECK(inside(outer_wall_band, point));
    }
}

TEST_CASE("The classic wall generator gets the same extension", "[WallExtension][Print]")
{
    Model model = terrace();
    DynamicPrintConfig config = extension_config(1.);
    config.set_deserialize_strict({ {"wall_generator", "classic"} });
    Print print;
    slice(print, model, config);
    CHECK(extension_layers(print) == tenths(106, 118));
}

TEST_CASE("On a thin plate the extension stops above the bed", "[WallExtension][Print]")
{
    Model model = boxes({ { { 30., 30., 1. }, { 0., 0., 0. } }, { { 10., 10., 5. }, { 10., 10., 1. } } });
    Print print;
    slice(print, model, extension_config(1.));
    // The plate's top layer (1.0) keeps its top surface; layer 0.2 is the first layer.
    CHECK(extension_layers(print) == std::set<int>{ 4, 6, 8 });
}

TEST_CASE("Depth is measured in millimetres with variable layer heights", "[WallExtension][Print]")
{
    Model model = terrace();
    model.objects.front()->layer_config_ranges[{ 9., 12. }].set("layer_height", 0.1);
    Print print;
    slice(print, model, extension_config(1.));
    // Four 0.1 mm top layers make a 0.4 mm shell; the extension goes through it, but for its top layer, which keeps its
    // top surface, and 1 mm further, its deepest layer being the bridge it stands on.
    CHECK(heights_with_extension(print) == Approx(0.4 - 0.1 + 1.0).margin(0.05));
}

TEST_CASE("Two close towers print their shared chamfer once", "[WallExtension][Print]")
{
    Model model = boxes({ { { 30., 30., 12. }, { 0., 0., 0. } }, { { 8., 8., 6. }, { 6., 11., 12. } }, { { 8., 8., 6. }, { 15., 11., 12. } } });
    Print print;
    slice(print, model, extension_config(1.));
    REQUIRE_FALSE(extension_layers(print).empty());
    for (const Layer *layer : print.get_object(size_t(0))->layers()) {
        double   sum = 0.;
        Polygons all;
        for (const ExtrusionPath *path : extension_paths(*layer)) {
            Polygons covered;
            path->polygons_covered_by_spacing(covered, 0.f);
            sum += area_mm2(union_ex(covered));
            append(all, std::move(covered));
        }
        CAPTURE(layer->print_z);
        CHECK(sum <= 1.05 * area_mm2(union_ex(all)) + 0.01);
    }
}

TEST_CASE("A modifier with extension 0 is an off switch for its volume", "[WallExtension][Print]")
{
    Model model = terrace();
    TriangleMesh left_half(its_make_cube(20., 40., 30.));
    left_half.translate(-5.f, -5.f, -1.f);
    auto *modifier = model.objects.front()->add_volume(std::move(left_half));
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    modifier->config.set("wall_internal_extension", 0.);
    Print print;
    slice(print, model, extension_config(1.));
    const Layer *tower_layer = layer_at(print, 12.2);
    REQUIRE(tower_layer != nullptr);
    const coord_t centre = get_extents(tower_layer->lslices).center().x();
    size_t right = 0;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const ExtrusionPath *path : extension_paths(*layer))
            for (const Point &point : path->polyline.points) {
                CAPTURE(layer->print_z);
                CHECK(point.x() >= centre - scale_(0.5));
                right += point.x() > centre + scale_(0.5);
            }
    CHECK(right > 0);
}

TEST_CASE("Continuous outer-wall speed mode still exports", "[WallExtension][Print][GCode]")
{
    PrintWorkspace workspace;
    Model model = terrace();
    DynamicPrintConfig config = extension_config(1.);
    config.set_deserialize_strict({ {"z_direction_outwall_speed_continuous", "1"} });
    Print print;
    slice(print, model, config);
    const auto filename = (workspace.path / "continuous.gcode").string();
    GCodeProcessorResult result;
    print.export_gcode(filename, &result);
    const auto &roles_times = result.print_statistics.modes[0].roles_times;
    const auto  it = std::find_if(roles_times.begin(), roles_times.end(),
                                  [](const std::pair<ExtrusionRole, float> &item) { return item.first == erWallExtension; });
    REQUIRE(it != roles_times.end());
    CHECK(it->second > 0.f);
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

TEST_CASE("Merged chamfers of close fins are printed through and not left hollow", "[WallExtension][Print]")
{
    // Ten one-line fins at 2 mm pitch on a box: their 1 mm chamfers merge into one block under the box's top shell.
    TriangleMesh mesh(its_make_cube(30., 30., 6.));
    for (int i = 0; i < 10; ++i) {
        TriangleMesh fin(its_make_cube(0.6, 20., 5.));
        fin.translate(float(5.7 + 2. * i), 5.f, 6.f);
        mesh.merge(fin);
    }
    Model model = model_of(std::move(mesh));
    Print print;
    slice(print, model, extension_config(1.));
    double widest = 0.;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions()) {
            if (region->wall_extension_area.empty())
                continue;
            // Everything cut out of the fill is printed by extension lines, but for the ends of open lines, which stop half a
            // line short of the area's edge.
            Polygons covered;
            for (const ExtrusionPath *path : extension_paths(*layer))
                path->polygons_covered_by_width(covered, scaled<float>(0.01));
            const double area = area_mm2(region->wall_extension_area);
            widest = std::max(widest, area);
            CAPTURE(layer->print_z, area);
            CHECK(area_mm2(diff_ex(offset_ex(region->wall_extension_area, -scale_(0.25)), union_(covered))) < 0.01 * area);
        }
    CHECK(widest > 300.);
}

TEST_CASE("A bridge over a bottom joint keeps its anchors and direction", "[WallExtension][Print]")
{
    // A tunnel 5 mm wide and 30 mm long under a 3 mm roof. The tunnel's walls end under the roof's first layer, a bridge
    // that spans the 5 mm width anchored on the walls' tops.
    Model model = boxes({ { { 30., 20., 2. }, { 0., 0., 0. } }, { { 30., 7.5, 3. }, { 0., 0., 2. } },
                          { { 30., 7.5, 3. }, { 0., 12.5, 2. } }, { { 30., 20., 3. }, { 0., 0., 5. } } });
    Print without, with;
    slice(without, model, extension_config(0.));
    slice(with, model, extension_config(1.));
    // The tunnel's walls continue into the roof, starting on the layer above the bridge.
    CHECK(extension_layers(with).count(54) == 1);
    const Layer *bridge_without = layer_at(without, 5.2), *bridge_with = layer_at(with, 5.2);
    REQUIRE(bridge_without != nullptr);
    REQUIRE(bridge_with != nullptr);
    CHECK(extension_paths(*bridge_with).empty());
    const auto bridges = [](const Layer &layer) {
        std::vector<std::pair<double, double>> out;
        for (const LayerRegion *region : layer.regions())
            for (const Surface &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == stBottomBridge)
                    out.emplace_back(surface.expolygon.area() * SCALING_FACTOR * SCALING_FACTOR, surface.bridge_angle);
        std::sort(out.begin(), out.end());
        return out;
    };
    const auto expected = bridges(*bridge_without), actual = bridges(*bridge_with);
    REQUIRE_FALSE(expected.empty());
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        CHECK(actual[i].first == Approx(expected[i].first).epsilon(0.01));
        CHECK(actual[i].second == Approx(expected[i].second).margin(1e-3));
    }
}

TEST_CASE("A tube through a thin flange keeps the flange's bridge anchors", "[WallExtension][Print]")
{
    // A 10 x 10 column through a 30 x 30 flange 1 mm thick: the column's wall rising above the flange is a top joint
    // whose extension reaches the flange's first layer, a bridge anchored on the column.
    Model model = boxes({ { { 10., 10., 12. }, { 10., 10., 0. } }, { { 30., 30., 1. }, { 0., 0., 6. } } });
    Print without, with;
    slice(without, model, extension_config(0.));
    slice(with, model, extension_config(1.));
    REQUIRE_FALSE(extension_layers(with).empty());
    const Layer *bridge_without = layer_at(without, 6.2), *bridge_with = layer_at(with, 6.2);
    REQUIRE(bridge_without != nullptr);
    REQUIRE(bridge_with != nullptr);
    CHECK(extension_paths(*bridge_with).empty());
    REQUIRE(surface_area(*bridge_without, stBottomBridge) > 100.);
    CHECK(surface_area(*bridge_with, stBottomBridge) == Approx(surface_area(*bridge_without, stBottomBridge)).epsilon(0.005));
}

TEST_CASE("The top surface around a tower is the same as without the extension", "[WallExtension][Print]")
{
    // The box's top surface runs a little under the tower's wall, as top surfaces do; the extension starts below it.
    Model model = terrace();
    Print without, with;
    slice(without, model, extension_config(0.));
    slice(with, model, extension_config(1.));
    const Layer *top_without = layer_at(without, 12.0), *top_with = layer_at(with, 12.0);
    REQUIRE(top_without != nullptr);
    REQUIRE(top_with != nullptr);
    CHECK(extension_paths(*top_with).empty());
    REQUIRE(surface_area(*top_without, stTop) > 100.);
    CHECK(surface_area(*top_with, stTop) == Approx(surface_area(*top_without, stTop)).epsilon(0.005));
    CHECK_FALSE(extension_paths(*layer_at(with, 11.8)).empty());
}

TEST_CASE("On slope-wall layers the extension lines do not widen top surfaces", "[WallExtension][Print]")
{
    // The terrace with a step on one side whose top lies in the box's top shell, among the extension layers, and beside it
    // a ridge whose sloped faces put slope walls on those layers. Slope-wall layers expand their top surfaces into the
    // neighbouring solid infill by the depth of their walls.
    TriangleMesh mesh(its_make_cube(30., 30., 12.));
    TriangleMesh step(its_make_cube(30., 10., 11.6));
    step.translate(0.f, 30.f, 0.f);
    mesh.merge(step);
    TriangleMesh ridge(its_make_prism(10., 30., 14.));
    ridge.translate(34.f, 15.f, 0.f);
    mesh.merge(ridge);
    TriangleMesh tower(its_make_cube(10., 10., 6.));
    tower.translate(10.f, 10.f, 12.f);
    mesh.merge(tower);
    Model model = model_of(std::move(mesh));
    DynamicPrintConfig config0 = extension_config(0.), config1 = extension_config(1.);
    config0.set_deserialize_strict({ {"extra_wall_slope_angle", "15"} });
    config1.set_deserialize_strict({ {"extra_wall_slope_angle", "15"} });
    Print without, with;
    slice(without, model, config0);
    slice(with, model, config1);
    const Layer *step_top_without = layer_at(without, 11.6), *step_top_with = layer_at(with, 11.6);
    REQUIRE(step_top_without != nullptr);
    REQUIRE(step_top_with != nullptr);
    REQUIRE_FALSE(extension_paths(*step_top_with).empty());
    REQUIRE(std::any_of(step_top_with->regions().begin(), step_top_with->regions().end(),
                        [](const LayerRegion *region) { return region->has_slope_wall_reinforcement; }));
    REQUIRE(surface_area(*step_top_without, stTop) > 100.);
    CHECK(surface_area(*step_top_with, stTop) <= surface_area(*step_top_without, stTop) + 0.05);
}
