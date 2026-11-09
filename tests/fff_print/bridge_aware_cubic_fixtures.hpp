#ifndef slic3r_tests_bridge_aware_cubic_fixtures_hpp_
#define slic3r_tests_bridge_aware_cubic_fixtures_hpp_

// The fixtures that the Bridge aware cubic tests share: the settings, the benchmark parts (small and large cube, rib, partial
// roof, modifier), the measurements of a sliced print and the config helpers. Header only, so every function is inline.
// Carried over from the branch this pattern replaces (ef9c17704) as the rewrite architecture's 7.2 lists; nothing here reads
// the pattern's own units.

#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"

#include <boost/filesystem.hpp>

#include <algorithm>
#include <functional>
#include <initializer_list>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace BACFixtures {

constexpr const char *gap_key = "bridge_aware_cubic_max_gap";

// A copy of `config` with the settings given, each set strictly.
inline Slic3r::DynamicPrintConfig with(Slic3r::DynamicPrintConfig config,
                                       std::initializer_list<std::pair<const std::string, std::string>> settings)
{
    for (const auto &[key, value] : settings)
        config.set_deserialize_strict(key, value);
    return config;
}

// The region defaults alone, as a dynamic config.
inline Slic3r::DynamicPrintConfig region_defaults()
{
    Slic3r::DynamicPrintConfig config;
    config.apply(static_cast<const Slic3r::PrintRegionConfig &>(Slic3r::FullPrintConfig::defaults()));
    return config;
}

// The settings of every fixture: 0.2 mm layers, 5% sparse infill, three top and three bottom shell layers and two classic
// walls, with the given unsupported gap (mm) and sparse infill pattern ("bridgeawarecubic" or "cubic").
inline Slic3r::DynamicPrintConfig fixture_config(double gap, const char *pattern)
{
    Slic3r::DynamicPrintConfig config = region_defaults();
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "5%"}, {"sparse_infill_pattern", pattern},
        {gap_key, std::to_string(gap)},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
        {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"}
    });
    return config;
}

// A model of one object made of one part, placed at (80, 80, 0).
inline Slic3r::Model single_part(const char *name, Slic3r::TriangleMesh mesh)
{
    using namespace Slic3r;
    Model model;
    ModelObject *object = model.add_object();
    object->name = name;
    object->add_volume(std::move(mesh));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

// 32 x 28 x 18 mm: one roof, at 18 mm.
inline Slic3r::Model small_cube()
{
    return single_part("Bridge aware cubic small cube", Slic3r::TriangleMesh(Slic3r::its_make_cube(32., 28., 18.)));
}

// 64 x 56 x 18 mm: one roof, at 18 mm.
inline Slic3r::Model large_cube()
{
    return single_part("Bridge aware cubic large cube", Slic3r::TriangleMesh(Slic3r::its_make_cube(64., 56., 18.)));
}

// A 40 x 30 x 10 mm body with a 20 x 30 x 4 mm block at x = 20 on its roof: the roof of the left half ends at sparse infill.
inline Slic3r::Model partial_roof()
{
    using namespace Slic3r;
    TriangleMesh body(its_make_cube(40., 30., 10.));
    TriangleMesh block(its_make_cube(20., 30., 4.));
    block.translate(20., 0., 10.);
    body.merge(block);
    return single_part("Bridge aware cubic partial roof", std::move(body));
}

// A 32 x 26 x 10 mm body with two inward ribs on its roof: 5 x 22 x 5 mm at (6, 2) and 4 x 18 x 3.4 mm at (22, 4).
// Slice it with rib_config().
inline Slic3r::Model rib()
{
    using namespace Slic3r;
    TriangleMesh body(its_make_cube(32., 26., 10.));
    TriangleMesh first(its_make_cube(5., 22., 5.));
    first.translate(6., 2., 10.);
    TriangleMesh second(its_make_cube(4., 18., 3.4));
    second.translate(22., 4., 10.);
    body.merge(first);
    body.merge(second);
    return single_part("Bridge aware cubic rib", std::move(body));
}

// fixture_config with 12% sparse infill at 17 degrees, a gap of 5 mm and no vertical shell thickening.
inline Slic3r::DynamicPrintConfig rib_config(const char *pattern)
{
    return with(fixture_config(5., pattern), {
        {"sparse_infill_density", "12%"}, {"infill_direction", "17"},
        {"ensure_vertical_shell_thickness", "disabled"}
    });
}

// A 48 x 28 x 18 mm cube whose right half (24 x 28 mm from x = 24) is a parameter modifier from height z_min to z_max
// (mm), setting 12% sparse infill at 17 degrees and a gap of 3 mm, which Cubic ignores. Slice it with modifier_config().
inline Slic3r::Model modifier_between(double z_min, double z_max)
{
    using namespace Slic3r;
    Model model = single_part("Bridge aware cubic modifier", TriangleMesh(its_make_cube(48., 28., 18.)));
    TriangleMesh mesh(its_make_cube(24., 28., z_max - z_min));
    mesh.translate(24., 0., z_min);
    ModelVolume *right = model.objects.front()->add_volume(std::move(mesh));
    right->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    right->name = "Right half modifier";
    right->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(12.));
    right->config.set("infill_direction", 17.);
    right->config.set(gap_key, 3.);
    return model;
}

// The modifier spans the whole height, so the two halves are two regions with two lattices from the first layer to the last.
inline Slic3r::Model modifier() { return modifier_between(0., 18.); }

// The modifier spans z = 6 to 18 mm: below 6 mm the whole part is one region, so the seam between the two lattices starts
// where the modifier does, over the sparse infill of the object's lattice. Slice it with modifier_config().
inline Slic3r::Model floating_modifier() { return modifier_between(6., 18.); }

// fixture_config with 5% sparse infill at 45 degrees and a gap of 8 mm.
inline Slic3r::DynamicPrintConfig modifier_config(const char *pattern)
{
    return with(fixture_config(8., pattern), {{"infill_direction", "45"}});
}

inline void paths(const Slic3r::ExtrusionEntity &entity, const std::function<void(const Slic3r::ExtrusionPath&)> &visit)
{
    using namespace Slic3r;
    if (const auto *path = dynamic_cast<const ExtrusionPath*>(&entity)) visit(*path);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath*>(&entity))
        for (const auto &path : multi->paths) visit(path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(&entity))
        for (const auto &path : loop->paths) visit(path);
    else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(&entity))
        for (const auto *child : collection->entities) paths(*child, visit);
}

// Applies and processes the model with the config on the plate whose origin is given, the origin of the bed by default.
inline void slice(Slic3r::Print &print, Slic3r::Model &model, const Slic3r::DynamicPrintConfig &config,
                  const Slic3r::Vec3d &plate_origin = Slic3r::Vec3d::Zero())
{
    print.set_status_silent();
    print.set_plate_origin(plate_origin);
    for (Slic3r::ModelObject *object : model.objects)
        print.auto_assign_extruders(object);
    print.apply(model, config);
    print.process();
}

// mm of sparse infill (erInternalInfill) in the first object.
inline double sparse_length(const Slic3r::Print &print)
{
    using namespace Slic3r;
    double result = 0.;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions())
            paths(region->fills, [&](const ExtrusionPath &p) {
                if (p.role() == erInternalInfill) result += unscale<double>(p.length());
            });
    return result;
}

// A path of the fills of a print, as the cases compare them.
struct FillPath {
    Slic3r::ExtrusionRole role;
    Slic3r::Points        points;
    float                 height;

    bool operator==(const FillPath &rhs) const { return role == rhs.role && points == rhs.points && height == rhs.height; }
    bool operator!=(const FillPath &rhs) const { return ! (*this == rhs); }
};

// The paths of the fills of every layer of the first object, in the order that they are printed: those of one role, or all.
inline std::vector<std::vector<FillPath>> fill_paths(const Slic3r::Print &print,
                                                     std::optional<Slic3r::ExtrusionRole> role = std::nullopt)
{
    using namespace Slic3r;
    std::vector<std::vector<FillPath>> layers;
    for (const Layer *layer : print.get_object(size_t(0))->layers()) {
        std::vector<FillPath> &out = layers.emplace_back();
        for (const LayerRegion *region : layer->regions())
            paths(region->fills, [&out, role](const ExtrusionPath &path) {
                if (! role || path.role() == *role)
                    out.push_back({ path.role(), path.polyline.points, path.height });
            });
    }
    return layers;
}

// The layers on which two prints of one object differ in the paths that fill_paths took of them.
inline size_t different_layers(const std::vector<std::vector<FillPath>> &a, const std::vector<std::vector<FillPath>> &b)
{
    size_t different = std::max(a.size(), b.size()) - std::min(a.size(), b.size());
    for (size_t i = 0; i < std::min(a.size(), b.size()); ++i)
        different += a[i] != b[i];
    return different;
}

// The segments a layer's fills deposit: role, height and the two ends (ordered), with collinear vertices within `tolerance_mm`
// removed, so that the same lines compare equal however the fill groups were ordered or their polylines split.
struct FillSegment {
    Slic3r::ExtrusionRole role;
    float                 height;
    Slic3r::Point         a, b;
};

inline std::vector<FillSegment> fill_segments(const std::vector<FillPath> &paths, double tolerance_mm)
{
    using namespace Slic3r;
    std::vector<FillSegment> out;
    for (const FillPath &path : paths) {
        Polyline polyline(path.points);
        polyline.simplify(scale_(tolerance_mm));
        for (size_t i = 1; i < polyline.points.size(); ++i) {
            Point a = polyline.points[i - 1], b = polyline.points[i];
            if (a == b) continue;
            if (std::make_pair(a.x(), a.y()) > std::make_pair(b.x(), b.y())) std::swap(a, b);
            out.push_back({ path.role, path.height, a, b });
        }
    }
    return out;
}

// Whether two prints deposit the same fill segments on every layer, regardless of order: the same count, each segment of one
// matched by an unused segment of the other with the same role and height and both ends within `tolerance_mm`. The message
// names the first layer that differs.
inline bool same_fill_paths(const std::vector<std::vector<FillPath>> &a, const std::vector<std::vector<FillPath>> &b,
                            double tolerance_mm, std::string &message)
{
    using namespace Slic3r;
    if (a.size() != b.size()) { message = "layer counts " + std::to_string(a.size()) + " and " + std::to_string(b.size()); return false; }
    const double tolerance = scale_(tolerance_mm);
    for (size_t i = 0; i < a.size(); ++i) {
        const std::vector<FillSegment> sa = fill_segments(a[i], tolerance_mm), sb = fill_segments(b[i], tolerance_mm);
        if (sa.size() != sb.size()) {
            message = "layer " + std::to_string(i) + ": " + std::to_string(sa.size()) + " against " + std::to_string(sb.size()) + " segments";
            return false;
        }
        std::vector<bool> used(sb.size(), false);
        std::vector<const FillSegment*> unmatched;
        for (const FillSegment &s : sa) {
            bool matched = false;
            for (size_t j = 0; j < sb.size() && !matched; ++j) {
                if (used[j] || sb[j].role != s.role || sb[j].height != s.height) continue;
                const bool same_way = (s.a - sb[j].a).cast<double>().norm() <= tolerance && (s.b - sb[j].b).cast<double>().norm() <= tolerance;
                const bool reversed = (s.a - sb[j].b).cast<double>().norm() <= tolerance && (s.b - sb[j].a).cast<double>().norm() <= tolerance;
                if (same_way || reversed) {
                    used[j] = true;
                    matched = true;
                }
            }
            if (!matched)
                unmatched.push_back(&s);
        }
        if (!unmatched.empty()) {
            const auto describe = [](const FillSegment &s) {
                return "role " + std::to_string(int(s.role)) + " " + std::to_string(unscale<double>(s.a.x())) + "," + std::to_string(unscale<double>(s.a.y())) +
                       " -> " + std::to_string(unscale<double>(s.b.x())) + "," + std::to_string(unscale<double>(s.b.y()));
            };
            message = "layer " + std::to_string(i) + ": " + std::to_string(unmatched.size()) + " segments without counterpart; first:";
            for (size_t k = 0; k < std::min<size_t>(4, unmatched.size()); ++k)
                message += " [" + describe(*unmatched[k]) + "]";
            message += "; unmatched on the other side:";
            size_t shown = 0;
            for (size_t j = 0; j < sb.size() && shown < 4; ++j)
                if (!used[j]) { message += " [" + describe(sb[j]) + "]"; ++shown; }
            return false;
        }
    }
    return true;
}

// Applies the small cube to `print` with `pattern` and one setting changed, `key` = `value`, set on the cube or on a parameter
// modifier of a second cube beside it ("the object", "a modifier of a second object"). Returns the index of the object that
// holds the value, which is the one that a validation error names.
inline size_t apply_with_setting(Slic3r::Print &print, Slic3r::Model &model, const char *pattern, const std::string &key,
                                 const std::string &value, const std::string &holder)
{
    using namespace Slic3r;
    DynamicPrintConfig config = fixture_config(5., pattern);
    DynamicPrintConfig changed;
    changed.set_deserialize_strict({{key, value}});
    size_t holding = 0;
    if (holder == "the object")
        config.apply(changed);
    else {
        ModelObject *second = model.add_object(*model.objects.front());
        second->name = "Bridge aware cubic second cube";
        second->instances.front()->set_offset(Vec3d(140., 80., 0.));
        ModelVolume *modifier = second->add_volume(TriangleMesh(its_make_cube(16., 28., 18.)));
        modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
        modifier->config.apply(changed);
        holding = 1;
    }
    print.set_status_silent();
    for (ModelObject *object : model.objects)
        print.auto_assign_extruders(object);
    print.apply(model, config);
    // The print holds a copy of the model, whose objects keep their ids.
    REQUIRE(print.get_object(holding)->model_object()->id() == model.objects[holding]->id());
    return holding;
}

// Independent interval measurement on emitted toolpaths. For every layer, the paths of its fills other than sparse infill
// that meet the sparse fill of the layer below are measured against what that layer prints, its perimeters and fills, by the
// overhang rule: a bead rests on what the layer below prints within half its width (and 0.01 mm, the measure of touching).
// The longest stretch of a path off that support that meets the sparse fill below is the span.
inline std::pair<size_t, double> roof_gaps(const Slic3r::Print &print)
{
    using namespace Slic3r;
    size_t checked = 0;
    double longest = 0.;
    const auto &layers = print.get_object(size_t(0))->layers();
    for (size_t i = 1; i < layers.size(); ++i) {
        ExPolygons sparse;
        Polygons footprints;
        for (const LayerRegion *region : layers[i - 1]->regions()) {
            for (const Surface &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == stInternal) sparse.push_back(surface.expolygon);
            region->perimeters.polygons_covered_by_width(footprints, 0.f);
            region->fills.polygons_covered_by_width(footprints, 0.f);
        }
        if (sparse.empty()) continue;
        // The support of a bead of each width, made once.
        std::vector<std::pair<float, Polygons>> supports;
        const auto support_of = [&](float width) -> const Polygons& {
            for (const auto &support : supports)
                if (support.first == width) return support.second;
            supports.emplace_back(width, offset(footprints, float(scale_(.5 * width + .01)), ClipperLib::jtRound, scale_(.001)));
            return supports.back().second;
        };
        for (const LayerRegion *region : layers[i]->regions())
            paths(region->fills, [&](const ExtrusionPath &p) {
                if (p.role() == erInternalInfill || intersection_pl(Polylines{p.polyline}, sparse).empty()) return;
                ++checked;
                for (const Polyline &gap : diff_pl(Polylines{p.polyline}, support_of(p.width))) {
                    if (!intersection_pl(Polylines{gap}, sparse).empty())
                        longest = std::max(longest, unscale<double>(gap.length()));
                }
            });
    }
    return {checked, longest};
}

// The ends of the bridge paths (erBridgeInfill) of an object, against what the layer below prints. An end hangs where its
// centre line lies farther than half the path's width and 0.01 mm (the measure of touching) from the footprint of every
// perimeter and fill of the layer below, each bead by its width. A path inside a multipath ends where the multipath does.
// Read from the extrusions as the slice leaves them, on every layer above the first; independent of the plan, so that it
// measures Cubic and Bridge aware cubic alike.
struct BridgeEnds {
    size_t paths = 0;      // bridge paths
    size_t hanging = 0;    // their ends that rest on nothing that the layer below prints
    double area = 0.;      // mm2 that the bridge paths cover, by their width, summed over the layers
    std::vector<std::pair<double, Slic3r::Vec2d>> where;   // print_z and the place (mm, in the frame of the layers) of each hanging end
};

inline BridgeEnds bridge_ends(const Slic3r::PrintObject &object)
{
    using namespace Slic3r;
    BridgeEnds result;
    const auto &layers = object.layers();
    for (size_t i = 1; i < layers.size(); ++i) {
        std::vector<std::pair<Polyline, double>> bridges;
        std::function<void(const ExtrusionEntity&)> collect = [&](const ExtrusionEntity &entity) {
            if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(&entity)) {
                for (const ExtrusionEntity *child : collection->entities)
                    collect(*child);
            } else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath*>(&entity)) {
                if (! multi->paths.empty() && multi->paths.front().role() == erBridgeInfill)
                    bridges.emplace_back(Polyline(multi->first_point(), multi->last_point()), multi->paths.front().width);
            } else if (const auto *path = dynamic_cast<const ExtrusionPath*>(&entity)) {
                if (path->role() == erBridgeInfill)
                    bridges.emplace_back(path->polyline, path->width);
            }
        };
        Polygons covered;
        for (const LayerRegion *region : layers[i]->regions()) {
            collect(region->fills);
            paths(region->fills, [&covered](const ExtrusionPath &path) {
                if (path.role() == erBridgeInfill)
                    path.polygons_covered_by_width(covered, 0.f);
            });
        }
        if (bridges.empty())
            continue;
        result.area += area(union_(covered)) * SCALING_FACTOR * SCALING_FACTOR;
        Polygons below;
        for (const LayerRegion *region : layers[i - 1]->regions()) {
            region->perimeters.polygons_covered_by_width(below, 0.f);
            region->fills.polygons_covered_by_width(below, 0.f);
        }
        std::vector<std::pair<double, ExPolygons>> supports;
        const auto support_of = [&](double width) -> const ExPolygons& {
            for (const auto &support : supports)
                if (support.first == width)
                    return support.second;
            supports.emplace_back(width, offset_ex(below, float(scale_(.5 * width + .01)), ClipperLib::jtRound, scale_(.001)));
            return supports.back().second;
        };
        for (const auto &[polyline, width] : bridges) {
            ++result.paths;
            const ExPolygons &support = support_of(width);
            for (const Point &end : { polyline.first_point(), polyline.last_point() })
                if (std::none_of(support.begin(), support.end(), [&end](const ExPolygon &part) { return part.contains(end); })) {
                    ++result.hanging;
                    result.where.emplace_back(layers[i]->print_z, unscale(end));
                }
        }
    }
    return result;
}

// A temporary data and temporary directory for the tests that save presets, projects or G-code.
struct Workspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("bridge-aware-cubic-%%%%-%%%%");
    std::string old_data = Slic3r::data_dir(), old_temp = Slic3r::temporary_dir();

    Workspace()
    {
        boost::filesystem::create_directories(path);
        Slic3r::set_data_dir(path.string());
        Slic3r::set_temporary_dir(path.string());
    }
    ~Workspace()
    {
        Slic3r::set_data_dir(old_data);
        Slic3r::set_temporary_dir(old_temp);
        boost::system::error_code error;
        boost::filesystem::remove_all(path, error);
    }
};

inline void check_gap(const Slic3r::ConfigBase &config, double expected)
{
    const auto *gap = config.option(gap_key);
    REQUIRE(gap != nullptr);
    REQUIRE(gap->type() == Slic3r::coFloat);
    CHECK(config.option<Slic3r::ConfigOptionFloat>(gap_key)->value == expected);
}

} // namespace BACFixtures

#endif // slic3r_tests_bridge_aware_cubic_fixtures_hpp_
