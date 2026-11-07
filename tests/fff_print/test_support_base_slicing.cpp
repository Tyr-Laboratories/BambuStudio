#include <catch2/catch.hpp>
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/TriangleSelector.hpp"

using namespace Slic3r;

namespace {
DynamicPrintConfig support_base_config(bool bed_only = true)
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"wall_loops", "2"}, {"sparse_infill_density", "0%"},
        {"top_shell_layers", "2"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "2"}, {"bottom_shell_thickness", "0"},
        {"brim_type", "no_brim"}, {"enable_arc_fitting", "0"},
        {"enable_support", "1"}, {"support_type", "tree(auto)"},
        {"support_style", "tree_organic"}, {"support_threshold_angle", "30"},
        {"support_on_build_plate_only", bed_only ? "1" : "0"},
        {"support_top_z_distance", "0.2"}, {"support_bottom_z_distance", "0.2"},
        {"support_interface_top_layers", "2"}, {"support_interface_bottom_layers", "2"},
        {"support_object_xy_distance", "0.35"}, {"tree_support_branch_angle", "35"},
        {"tree_support_branch_diameter", "2"}, {"support_critical_regions_only", "0"}
    });
    return config;
}

void paint_faces(ModelVolume &volume, const std::function<bool(const Vec3f &, const Vec3f &)> &choose)
{
    const auto &mesh = volume.mesh().its;
    const auto transform = volume.get_matrix();
    TriangleSelector selector(volume.mesh());
    for (size_t i = 0; i < mesh.indices.size(); ++i) {
        const auto &face = mesh.indices[i];
        const Vec3f a = (transform * mesh.vertices[face[0]].cast<double>()).cast<float>();
        const Vec3f b = (transform * mesh.vertices[face[1]].cast<double>()).cast<float>();
        const Vec3f c = (transform * mesh.vertices[face[2]].cast<double>()).cast<float>();
        if (choose((a+b+c)/3.f, (b-a).cross(c-a).normalized()))
            selector.set_facet(int(i), EnforcerBlockerType::ENFORCER);
    }
    volume.support_base_facets.set(selector);
}

Model platform_model(bool painted = false, bool downward = false)
{
    // Two C-shaped fixtures joined at the bed. Their raised platforms are
    // genuinely solid, so any support below z=14 serves the upper cantilever.
    Model model;
    auto *object = model.add_object();
    TriangleMesh mesh(its_make_cube(88., 12., 1.));
    for (double x : {0., 70.}) {
        TriangleMesh platform(its_make_cube(18., 12., 14.));
        platform.translate(x, 0., 0.); mesh.merge(platform);
        TriangleMesh upright(its_make_cube(3., 12., 26.));
        upright.translate(x, 0., 0.); mesh.merge(upright);
        TriangleMesh roof(its_make_cube(18., 12., 1.));
        roof.translate(x, 0., 25.); mesh.merge(roof);
    }
    auto *volume = object->add_volume(mesh);
    object->add_instance()->set_offset(Vec3d(110., 100., 0.));
    if (painted)
        paint_faces(*volume, [downward](const Vec3f &point, const Vec3f &normal) {
            return point.x() < 30.f && (downward ? normal.z() < -0.9f && point.z() > 24.f :
                normal.z() > 0.9f && std::abs(point.z()-14.f) < 0.001f);
        });
    REQUIRE(volume->support_base_facets.empty() == !painted);
    return model;
}

void slice_supports(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.get_object(size_t(0))->generate_support_preview();
}

struct SupportStats {
    double low_length[2]{};
    double high_length[2]{};
    double roof_length[2]{};
    double first_over_platform = std::numeric_limits<double>::max();
};

SupportStats support_stats(const PrintObject &object)
{
    SupportStats stats;
    const auto trafo = object.trafo_centered();
    const Vec3d split = trafo * Vec3d(44., 0., 0.);
    const Vec3d corner = trafo * Vec3d(0., 0., 0.);
    for (const auto *layer : object.support_layers()) {
        for (const auto &polyline : layer->support_fills.as_polylines()) {
            for (const Line &line : polyline.lines()) {
                const Point center = line.midpoint();
                const size_t side = center.x() < scale_(split.x()) ? 0 : 1;
                (layer->print_z < 14. ? stats.low_length[side] : stats.high_length[side]) += unscale<double>(line.length());
                if (layer->print_z > 24.5)
                    stats.roof_length[side] += unscale<double>(line.length());
                const Vec2d local = center.cast<double>() * SCALING_FACTOR - corner.head<2>();
                if (local.x() > 4. && local.x() < 17. && local.y() > 0.5 && local.y() < 11.5)
                    stats.first_over_platform = std::min(stats.first_over_platform, layer->bottom_z());
            }
        }
    }
    return stats;
}
}

TEST_CASE("Support base projections select upward paint in print orientation", "[SupportBase][Projection]")
{
    Model model;
    auto *object = model.add_object();
    auto *volume = object->add_volume(TriangleMesh(its_make_cube(10., 10., 2.)));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    const bool down = GENERATE(false, true);
    const bool mirrored = GENERATE(false, true);
    const bool rotated = GENERATE(false, true);
    const double scale = GENERATE(1., 1.5);
    paint_faces(*volume, [down](const Vec3f &, const Vec3f &normal) { return down ? normal.z() < -0.9f : normal.z() > 0.9f; });
    if (mirrored) object->instances.front()->set_mirror(Vec3d(-1., 1., 1.));
    object->instances.front()->set_scaling_factor(Vec3d(scale, 1., 1.));
    if (rotated) object->instances.front()->set_rotation(Vec3d(0., 0., 0.4));
    Print print;
    auto config = support_base_config();
    print.set_status_silent(); print.auto_assign_extruders(object); print.apply(model, config);
    auto &sliced = *print.get_object(size_t(0));
    sliced.slice();
    REQUIRE(sliced.has_support_base_painting());
    const auto projected = sliced.project_support_base_facets();
    double projected_area = 0.;
    for (const auto &layer : projected) projected_area += area(union_ex(layer)) * SCALING_FACTOR * SCALING_FACTOR;
    CHECK(projected_area == Approx(down ? 0. : 100. * scale).margin(0.001));
}

TEST_CASE("Painted organic bases shorten nearby towers and retain distant bed supports", "[SupportBase][Slicing]")
{
    auto config = support_base_config();
    const int bottom_interfaces = GENERATE(0, 2);
    config.set_deserialize_strict({{"support_interface_bottom_layers", std::to_string(bottom_interfaces)}});
    auto baseline_model = platform_model();
    Print baseline; slice_supports(baseline, baseline_model, config);
    const auto before = support_stats(*baseline.get_object(size_t(0)));
    REQUIRE(before.low_length[0] > 10.);
    REQUIRE(before.low_length[1] > 10.);

    auto painted_model = platform_model(true);
    Print painted; slice_supports(painted, painted_model, config);
    const auto after = support_stats(*painted.get_object(size_t(0)));
    CAPTURE(before.low_length[0], before.low_length[1], after.low_length[0], after.low_length[1],
        before.high_length[0], after.high_length[0], after.first_over_platform);
    CHECK(after.low_length[0] < before.low_length[0] * 0.15);
    CHECK(after.high_length[0] > 10.);
    CHECK(after.low_length[1] > before.low_length[1] * 0.8);
    for (size_t i = 0; i < 2; ++i) {
        REQUIRE(before.roof_length[i] > 10.);
        CHECK(after.roof_length[i] > before.roof_length[i] * 0.9);
    }
    CHECK(after.first_over_platform == Approx(14.2).margin(0.01));

    // Removing the permission must rebuild supports all the way to the bed.
    painted_model.objects.front()->volumes.front()->support_base_facets.reset();
    slice_supports(painted, painted_model, config);
    const auto erased = support_stats(*painted.get_object(size_t(0)));
    for (size_t i = 0; i < 2; ++i) {
        CHECK(erased.low_length[i] == Approx(before.low_length[i]).margin(0.01));
        CHECK(erased.high_length[i] == Approx(before.high_length[i]).margin(0.01));
    }
}

TEST_CASE("Base paint leaves unrestricted organic support paths unchanged", "[SupportBase][Slicing]")
{
    auto config = support_base_config(false);
    auto baseline_model = platform_model();
    Print baseline; slice_supports(baseline, baseline_model, config);
    auto painted_model = platform_model(true);
    Print painted; slice_supports(painted, painted_model, config);
    const auto before = support_stats(*baseline.get_object(size_t(0)));
    const auto after = support_stats(*painted.get_object(size_t(0)));
    for (size_t i = 0; i < 2; ++i) {
        CHECK(after.low_length[i] == Approx(before.low_length[i]).margin(0.01));
        CHECK(after.high_length[i] == Approx(before.high_length[i]).margin(0.01));
    }
    CHECK(after.first_over_platform == Approx(before.first_over_platform).margin(0.01));
}

TEST_CASE("Wrong-facing support base paint does not change organic supports", "[SupportBase][Slicing]")
{
    auto config = support_base_config();
    auto baseline_model = platform_model();
    Print baseline; slice_supports(baseline, baseline_model, config);
    auto painted_model = platform_model(true, true);
    Print painted; slice_supports(painted, painted_model, config);
    const auto before = support_stats(*baseline.get_object(size_t(0)));
    const auto after = support_stats(*painted.get_object(size_t(0)));
    for (size_t i = 0; i < 2; ++i) {
        CHECK(after.low_length[i] == Approx(before.low_length[i]).margin(0.01));
        CHECK(after.high_length[i] == Approx(before.high_length[i]).margin(0.01));
    }
}
