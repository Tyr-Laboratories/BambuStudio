#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <algorithm>
#include <array>
#include <cmath>

using namespace Slic3r;

namespace {
const double infill_angle = Geometry::deg2rad(45.);

DynamicPrintConfig sparse_infill_config(const std::string &pattern, const std::string &bridge_aware_density)
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "10%"}, {"sparse_infill_pattern", pattern},
        {"bridge_aware_cubic_density", bridge_aware_density}, {"infill_direction", "45"},
        {"top_shell_layers", "4"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        // Without anchors, each infill line is printed as cut from the lattice.
        {"sparse_infill_anchor", "0"}, {"sparse_infill_anchor_max", "0"},
        {"brim_type", "no_brim"}, {"enable_arc_fitting", "0"}, {"wall_generator", "classic"}
    });
    return config;
}

// A box with a smaller box on top of it: the roof of the lower box closes only a part of its sparse infill.
Model terrace_model()
{
    Model model;
    auto *object = model.add_object();
    TriangleMesh mesh(its_make_cube(48., 48., 24.));
    TriangleMesh tower(its_make_cube(16., 16., 8.));
    tower.translate(6., 16., 24.);
    mesh.merge(tower);
    object->add_volume(mesh);
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

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

std::vector<const ExtrusionPath *> layer_paths(const Layer &layer)
{
    std::vector<const ExtrusionPath *> paths;
    for (const LayerRegion *region : layer.regions()) {
        collect_paths(region->perimeters, paths);
        collect_paths(region->fills, paths);
    }
    return paths;
}

// Which of the three directions of the lattice a straight infill line follows.
int line_direction(const Polyline &line)
{
    const Vec2d v = (line.last_point() - line.first_point()).cast<double>();
    return (int(std::lround((std::atan2(v.y(), v.x()) - infill_angle) / (M_PI / 3.))) % 3 + 3) % 3;
}

// Offsets of the sparse infill lines of a layer in one direction, perpendicular to the lines, sorted, each line once.
std::vector<double> line_offsets(const Layer &layer, int direction)
{
    const double angle = infill_angle + direction * M_PI / 3.;
    const Vec2d  normal(- std::sin(angle), std::cos(angle));
    std::vector<double> offsets;
    for (const ExtrusionPath *path : layer_paths(layer))
        if (path->role() == erInternalInfill && path->polyline.length() > scale_(1.) && line_direction(path->polyline) == direction)
            offsets.push_back(normal.dot(unscaled<double>(path->polyline.first_point())));
    std::sort(offsets.begin(), offsets.end());
    // Pieces of one line.
    offsets.erase(std::unique(offsets.begin(), offsets.end(), [](double a, double b) { return b - a < 0.01; }), offsets.end());
    return offsets;
}

bool has_internal_bridge(const Layer *layer)
{
    for (const LayerRegion *region : layer->regions())
        for (const Surface &surface : region->fill_surfaces.surfaces)
            if (surface.surface_type == stInternalBridge)
                return true;
    return false;
}
}

TEST_CASE("Bridge Aware Cubic prints the lines of Cubic, subdivided below internal bridges", "[BridgeAwareCubic]")
{
    // The density below the internal bridges is reached by doubling the sparse infill density: 10% -> 20% -> 40%.
    const auto [bridge_aware_density, subdivisions] = GENERATE(table<std::string, int>({ { "10%", 0 }, { "20%", 1 }, { "25%", 2 } }));
    CAPTURE(bridge_aware_density, subdivisions);
    Model model = terrace_model();
    Print cubic, bridge_aware;
    slice(cubic, model, sparse_infill_config("cubic", bridge_aware_density));
    slice(bridge_aware, model, sparse_infill_config("bridgeawarecubic", bridge_aware_density));

    // The roof of the lower box is the lowest internal bridge.
    const LayerPtrs &layers = bridge_aware.get_object(0)->layers();
    const auto       roof   = std::find_if(layers.begin(), layers.end(), has_internal_bridge);
    REQUIRE(roof != layers.end());
    const size_t below_roof = (*roof)->id() - 1;
    // Deeper below the roof than the subdivided lattices reach.
    const size_t far_below  = 10;
    REQUIRE(layers[below_roof]->print_z - layers[far_below]->print_z > 20.);

    for (int direction = 0; direction < 3; ++ direction)
        for (size_t layer_id : { far_below, below_roof }) {
            const std::vector<double> lattice = line_offsets(*cubic.get_object(0)->get_layer(layer_id), direction);
            const std::vector<double> lines   = line_offsets(*layers[layer_id], direction);
            CAPTURE(direction, layer_id, lattice, lines);
            REQUIRE(lattice.size() >= 4);
            // The lines of Cubic, right below the roof with the distances between them halved by each subdivision.
            const double distance = (lattice[2] - lattice[1]) / double(layer_id == below_roof ? 1 << subdivisions : 1);
            // The first and the last line of Cubic may be dropped next to the walls.
            std::vector<double> inner;
            std::copy_if(lines.begin(), lines.end(), std::back_inserter(inner),
                [&lattice](double offset) { return offset > lattice[1] - 0.01 && offset < lattice[lattice.size() - 2] + 0.01; });
            REQUIRE(inner.size() == size_t(std::lround((lattice[lattice.size() - 2] - lattice[1]) / distance)) + 1);
            for (size_t i = 0; i < inner.size(); ++ i)
                CHECK(inner[i] == Approx(lattice[1] + double(i) * distance).margin(0.01));
        }
}

TEST_CASE("Bridge Aware Cubic lines end on the layer below or on another extrusion", "[BridgeAwareCubic]")
{
    Model model = terrace_model();
    Print print;
    slice(print, model, sparse_infill_config("bridgeawarecubic", "25%"));

    // The area covered by the extrusions of the layer below.
    Polygons below;
    size_t   ends_checked = 0;
    for (const Layer *layer : print.get_object(0)->layers()) {
        // All the extrusions of the layer, and per direction all of them but the sparse infill lines of that direction.
        std::array<Polylines, 3> sparse;
        std::array<Polygons, 3>  others;
        Polygons                 deposited;
        double                   width = 0.;
        for (const ExtrusionPath *path : layer_paths(*layer)) {
            Polygons covered;
            path->polygons_covered_by_width(covered, float(SCALED_EPSILON));
            const int direction = path->role() == erInternalInfill ? line_direction(path->polyline) : -1;
            if (direction >= 0) {
                sparse[direction].push_back(path->polyline);
                width = path->width;
            }
            for (int i = 0; i < 3; ++ i)
                if (i != direction)
                    append(others[i], covered);
            append(deposited, std::move(covered));
        }
        if (! below.empty())
            for (int direction = 0; direction < 3; ++ direction) {
                // The ends of the lines, 1 um long.
                Polylines ends;
                for (const Polyline &polyline : sparse[direction]) {
                    const double cut = std::max(0., polyline.length() - scale_(0.001));
                    ends.push_back(polyline);
                    ends.back().clip_end(cut);
                    ends.push_back(polyline);
                    ends.back().clip_start(cut);
                }
                ends_checked += ends.size();
                // No line ends in the air: it ends on an extrusion of the layer below, or it touches another extrusion of its layer
                // to which it is anchored, as a line that grows from the crossing of two infill walls.
                CAPTURE(layer->print_z, direction);
                CHECK(diff_pl(ends, union_(below, offset(union_(others[direction]), float(scale_(0.5 * width))))).empty());
            }
        below = union_(deposited);
    }
    CHECK(ends_checked > 2000);
}
