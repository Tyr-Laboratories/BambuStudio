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
#include <map>
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

// A path of the fills of a print, as the cases compare them: its role, cross-section (height, width, volume per mm), region and
// vertices.
struct FillPath {
    Slic3r::ExtrusionRole role;
    Slic3r::Points        points;
    float                 height;
    float                 width;
    double                mm3_per_mm;
    size_t                region;

    bool operator==(const FillPath &rhs) const
    {
        return role == rhs.role && points == rhs.points && height == rhs.height && width == rhs.width && mm3_per_mm == rhs.mm3_per_mm && region == rhs.region;
    }
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
        for (size_t region = 0; region < layer->regions().size(); ++region)
            paths(layer->regions()[region]->fills, [&out, role, region](const ExtrusionPath &path) {
                if (! role || path.role() == *role)
                    out.push_back({ path.role(), path.polyline.points, path.height, path.width, path.mm3_per_mm, region });
            });
    }
    return layers;
}

// The segments a layer's fills deposit, each with its path's role, cross-section and region.
struct FillSegment {
    Slic3r::ExtrusionRole role;
    float                 height, width;
    double                mm3_per_mm;
    size_t                region;
    Slic3r::Point         a, b;
    bool same_kind(const FillSegment &o) const { return role == o.role && height == o.height && width == o.width && mm3_per_mm == o.mm3_per_mm && region == o.region; }
};

inline std::vector<FillSegment> fill_segments(const std::vector<FillPath> &paths)
{
    std::vector<FillSegment> out;
    for (const FillPath &path : paths)
        for (size_t i = 1; i < path.points.size(); ++i)
            if (path.points[i - 1] != path.points[i])
                out.push_back({ path.role, path.height, path.width, path.mm3_per_mm, path.region, path.points[i - 1], path.points[i] });
    return out;
}

// Whether a segment lies, within `tolerance` (scaled), on segments of the same kind among `others`: both ends of each such
// segment within the tolerance of its line, and their projections covering it. Collinear splits at different vertices compare
// equal this way, whichever side has more of them.
inline bool covered_by(const FillSegment &s, const std::vector<FillSegment> &others, double tolerance)
{
    const Slic3r::Vec2d a = s.a.cast<double>(), d = (s.b - s.a).cast<double>();
    const double length = d.norm();
    if (length < 1.) return true;
    const Slic3r::Vec2d u = d / length;
    std::vector<std::pair<double, double>> intervals;
    for (const FillSegment &o : others) {
        if (!o.same_kind(s)) continue;
        const Slic3r::Vec2d oa = o.a.cast<double>() - a, ob = o.b.cast<double>() - a;
        if (std::abs(oa.x() * u.y() - oa.y() * u.x()) > tolerance || std::abs(ob.x() * u.y() - ob.y() * u.x()) > tolerance) continue;
        double t0 = oa.dot(u), t1 = ob.dot(u);
        if (t0 > t1) std::swap(t0, t1);
        intervals.emplace_back(t0, t1);
    }
    std::sort(intervals.begin(), intervals.end());
    double cursor = 0.;
    for (const auto &interval : intervals) {
        if (interval.first > cursor + tolerance) return false;
        cursor = std::max(cursor, interval.second);
        if (cursor >= length - tolerance) return true;
    }
    return cursor >= length - tolerance;
}

// Whether two prints deposit the same fills on every layer, regardless of the order of paths: every segment of each covered by
// segments of the same role, cross-section and region of the other within `tolerance_mm`, and the same total length of each
// role. The message names the first layer that differs.
inline bool same_fill_paths(const std::vector<std::vector<FillPath>> &a, const std::vector<std::vector<FillPath>> &b,
                            double tolerance_mm, std::string &message)
{
    using namespace Slic3r;
    if (a.size() != b.size()) { message = "layer counts " + std::to_string(a.size()) + " and " + std::to_string(b.size()); return false; }
    const double tolerance = scale_(tolerance_mm);
    for (size_t i = 0; i < a.size(); ++i) {
        // Path counts are not compared: the connector may chain the same lines differently where a fill group's boundary
        // moved by Clipper's safety offset; the segments and their total length per role pin the deposited geometry.
        const std::vector<FillSegment> sa = fill_segments(a[i]), sb = fill_segments(b[i]);
        std::map<int, double> length_a, length_b;
        for (const FillSegment &s : sa) length_a[int(s.role)] += unscale<double>((s.b - s.a).cast<double>().norm());
        for (const FillSegment &s : sb) length_b[int(s.role)] += unscale<double>((s.b - s.a).cast<double>().norm());
        for (const auto &[role, length] : length_a)
            if (std::abs(length - length_b[role]) > tolerance_mm * double(sa.size() + sb.size())) {
                message = "layer " + std::to_string(i) + ": the total length of role " + std::to_string(role) + " differs by " + std::to_string(length - length_b[role]) + " mm";
                return false;
            }
        const auto describe = [](const FillSegment &s) {
            return "role " + std::to_string(int(s.role)) + " " + std::to_string(unscale<double>(s.a.x())) + "," + std::to_string(unscale<double>(s.a.y())) +
                   " -> " + std::to_string(unscale<double>(s.b.x())) + "," + std::to_string(unscale<double>(s.b.y()));
        };
        for (const FillSegment &s : sa)
            if (!covered_by(s, sb, tolerance)) { message = "layer " + std::to_string(i) + ": uncovered in the second print: " + describe(s); return false; }
        for (const FillSegment &s : sb)
            if (!covered_by(s, sa, tolerance)) { message = "layer " + std::to_string(i) + ": uncovered in the first print: " + describe(s); return false; }
    }
    return true;
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
