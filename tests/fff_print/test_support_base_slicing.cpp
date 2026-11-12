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

// Height of a platform top tilted by slope_deg, at x along the fixture.
double platform_top(double x, double slope_deg) { return 14. + std::tan(slope_deg * M_PI / 180.) * (x - 9.); }

Model platform_model(bool painted = false, bool downward = false, double slope_deg = 0.)
{
    // Two C-shaped fixtures joined at the bed. Their raised platforms are
    // genuinely solid, so any support below z=14 serves the upper cantilever.
    // A slope tilts both platform tops about their centre line, rising along X.
    Model model;
    auto *object = model.add_object();
    TriangleMesh mesh(its_make_cube(88., 12., 1.));
    for (double x : {0., 70.}) {
        indexed_triangle_set block = its_make_cube(18., 12., 14.);
        for (Vec3f &vertex : block.vertices)
            if (vertex.z() > 7.f)
                vertex.z() = float(platform_top(vertex.x(), slope_deg));
        TriangleMesh platform(std::move(block));
        platform.translate(x, 0., 0.); mesh.merge(platform);
        TriangleMesh upright(its_make_cube(3., 12., 26.));
        upright.translate(x, 0., 0.); mesh.merge(upright);
        TriangleMesh roof(its_make_cube(18., 12., 1.));
        roof.translate(x, 0., 25.); mesh.merge(roof);
    }
    auto *volume = object->add_volume(mesh);
    object->add_instance()->set_offset(Vec3d(110., 100., 0.));
    if (painted)
        paint_faces(*volume, [downward, slope_deg](const Vec3f &point, const Vec3f &normal) {
            const float top = float(platform_top(point.x(), slope_deg));
            return point.x() < 30.f && (downward ? normal.z() < -0.9f && point.z() > 24.f :
                normal.z() > 0.5f && std::abs(point.z() - top) < 0.001f);
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

// Support printed below low_z on either fixture cannot stand on its platform.
SupportStats support_stats(const PrintObject &object, double low_z = 14.)
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
                (layer->print_z < low_z ? stats.low_length[side] : stats.high_length[side]) += unscale<double>(line.length());
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

struct HangingSupport {
    double support = 0.; // support line length over the left platform, mm
    double hanging = 0.; // the part of it resting on neither support nor the platform
};

// Support over the left platform below its cantilever, and how much of it hangs
// in the air: a line rests on the support of the previous layer if it lies
// within one line width of it, or on the platform if no more than two layers
// clear the bottom gap above it. The roof interface (z=25) bridges between tips
// by design and is skipped.
HangingSupport hanging_over_left_platform(const PrintObject &object, double slope_deg)
{
    const double bottom_gap = object.config().support_bottom_z_distance.value;
    const double layer_height = object.config().layer_height.value;
    const Vec3d corner = object.trafo_centered() * Vec3d(0., 0., 0.);
    HangingSupport out;
    Polygons below;
    for (const SupportLayer *layer : object.support_layers()) {
        if (layer->print_z > 24.)
            break;
        const ExtrusionEntityCollection flat = layer->support_fills.flatten();
        std::vector<const ExtrusionPath *> paths;
        for (const ExtrusionEntity *entity : flat.entities)
            if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity))
                paths.push_back(path);
            else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(entity))
                for (const ExtrusionPath &part : multi->paths) paths.push_back(&part);
            else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity))
                for (const ExtrusionPath &part : loop->paths) paths.push_back(&part);
        float width = 0.f;
        for (const ExtrusionPath *path : paths)
            width = std::max(width, scaled<float>(path->width));
        const Polygons supported = offset(below, width);
        Polygons here;
        for (const ExtrusionPath *path : paths) {
            append(here, offset(path->polyline, 0.5f * scaled<float>(path->width)));
            for (const Line &line : path->polyline.lines()) {
                const double length = unscale<double>(line.length());
                const int samples = std::max(1, int(std::ceil(length / 0.1)));
                for (int i = 0; i < samples; ++i) {
                    const Point point = line.a + ((line.b - line.a).cast<double>() * ((i + 0.5) / samples)).cast<coord_t>();
                    const Vec2d local = point.cast<double>() * SCALING_FACTOR - corner.head<2>();
                    if (local.x() < 4. || local.x() > 17. || local.y() < 0.5 || local.y() > 11.5)
                        continue;
                    out.support += length / samples;
                    const double clearance = layer->bottom_z() - platform_top(local.x(), slope_deg);
                    if (clearance > bottom_gap + 2. * layer_height + EPSILON && !contains(supported, point))
                        out.hanging += length / samples;
                }
            }
        }
        below = union_(here);
    }
    return out;
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

TEST_CASE("Automatic organic supports handle enforcer paint ending below the model top", "[SupportBase][Slicing][EnforcerBounds]")
{
    Model model;
    auto *model_object = model.add_object();
    TriangleMesh fixture(its_make_cube(3., 12., 26.));
    for (double z : {10., 25.}) {
        TriangleMesh shelf(its_make_cube(18., 12., 1.));
        shelf.translate(0., 0., z);
        fixture.merge(shelf);
    }
    ModelVolume &volume = *model_object->add_volume(fixture);
    model_object->add_instance()->set_offset(Vec3d(110., 100., 0.));
    TriangleSelector selector(volume.mesh());
    const auto &mesh = volume.mesh().its;
    const auto transform = volume.get_matrix();
    for (size_t i = 0; i < mesh.indices.size(); ++i) {
        const auto &face = mesh.indices[i];
        const Vec3f a = (transform * mesh.vertices[face[0]].cast<double>()).cast<float>();
        const Vec3f b = (transform * mesh.vertices[face[1]].cast<double>()).cast<float>();
        const Vec3f c = (transform * mesh.vertices[face[2]].cast<double>()).cast<float>();
        const Vec3f center = (a + b + c) / 3.f;
        if (std::abs(center.z() - 10.f) < 0.001f && (b-a).cross(c-a).z() < 0.f)
            selector.set_facet(int(i), EnforcerBlockerType::ENFORCER);
    }
    volume.supported_facets.set(selector);
    REQUIRE_FALSE(volume.supported_facets.empty());

    auto config = support_base_config(false);
    config.set_deserialize_strict({{"support_interface_top_layers", "0"}});
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    PrintObject &object = *print.get_object(size_t(0));
    object.slice();
    auto enforcers = object.slice_support_enforcers();
    object.project_and_append_custom_facets(false, EnforcerBlockerType::ENFORCER, enforcers);
    while (!enforcers.empty() && enforcers.back().empty())
        enforcers.pop_back();
    REQUIRE_FALSE(enforcers.empty());
    REQUIRE(enforcers.size() < object.layer_count() / 2);

    object.generate_support_preview();
    double lower_contact = 0., upper_contact = 0.;
    for (const auto *layer : object.support_layers())
        for (const auto &path : layer->support_fills.as_polylines()) {
            if (layer->print_z > 9.5 && layer->print_z < 10.)
                lower_contact += unscale<double>(path.length());
            if (layer->print_z > 24.5 && layer->print_z < 25.)
                upper_contact += unscale<double>(path.length());
        }
    // Automatic detection continues above the painted lower shelf.
    CHECK(lower_contact > 10.);
    CHECK(upper_contact > 10.);
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

TEST_CASE("Painted organic bases on sloped platforms carry the trees above them", "[SupportBase][Slicing][Slope]")
{
    // Tips are sampled from each layer's overhang band however thin a curved
    // overhang makes it. A painted base sloping under the trees must serve them
    // the same way, although no single layer's band can hold a whole branch foot.
    const double slope = GENERATE(10., 25.);
    auto config = support_base_config();
    // Lower than the lowest point of either tilted platform.
    const double low_z = platform_top(0., slope) - 0.5;
    auto baseline_model = platform_model(false, false, slope);
    Print baseline; slice_supports(baseline, baseline_model, config);
    const auto before = support_stats(*baseline.get_object(size_t(0)), low_z);
    REQUIRE(before.low_length[0] > 10.);
    REQUIRE(before.low_length[1] > 10.);

    auto painted_model = platform_model(true, false, slope);
    Print painted; slice_supports(painted, painted_model, config);
    const PrintObject &object = *painted.get_object(size_t(0));
    const auto after = support_stats(object, low_z);

    const auto hanging_before = hanging_over_left_platform(*baseline.get_object(size_t(0)), slope);
    const auto hanging_after = hanging_over_left_platform(object, slope);
    CAPTURE(slope, before.low_length[0], after.low_length[0], before.low_length[1], after.low_length[1],
        hanging_before.support, hanging_before.hanging, hanging_after.support, hanging_after.hanging);
    // The trees above the painted slope stand on it; the unpainted twin keeps its bed route.
    CHECK(after.low_length[0] < before.low_length[0] * 0.15);
    CHECK(after.low_length[1] > before.low_length[1] * 0.8);
    for (size_t i = 0; i < 2; ++i)
        CHECK(after.roof_length[i] > before.roof_length[i] * 0.9);
    REQUIRE(hanging_after.support > hanging_before.support);
    // Their feet follow the slope down instead of stopping where their uphill
    // edges first meet it: painting adds no more hanging support than organic
    // branches over the same platform already show without it.
    CHECK(hanging_after.hanging < hanging_before.hanging + 0.005 * hanging_after.support);
}

TEST_CASE("Painted towers too slender for their anchor are built from the bed", "[SupportBase][Slicing][Slender]")
{
    // A cantilever 35 mm above a painted 4 x 4 mm pad. Standing on the pad, a
    // tower would rise higher than the branch diameter angle's taper allows
    // for so small an anchor, so the cantilever has to be held from the bed.
    // 4 mm above the pad, the same anchor is wide enough and the pad holds it.
    // On a steep pad no layer's band of paint is wider than the XY distance;
    // the foot stepping down it is the anchor all the same.
    const double slope = GENERATE(0., 35.);
    const double height = GENERATE(35., 4.);
    const bool   slender = height > 10.;
    const auto pad_top = [slope](double x) { return 10. + std::tan(slope * M_PI / 180.) * (x - 12.); };
    Model model;
    auto *object = model.add_object();
    TriangleMesh mesh(its_make_cube(3., 12., 11. + height));
    TriangleMesh roof(its_make_cube(18., 12., 1.));
    roof.translate(0., 0., 10. + height); mesh.merge(roof);
    indexed_triangle_set pad_block = its_make_cube(4., 4., 10.);
    for (Vec3f &vertex : pad_block.vertices)
        if (vertex.z() > 5.f)
            vertex.z() = float(pad_top(vertex.x() + 10.));
    TriangleMesh pad(std::move(pad_block));
    pad.translate(10., 4., 0.); mesh.merge(pad);
    auto *volume = object->add_volume(mesh);
    object->add_instance()->set_offset(Vec3d(110., 100., 0.));
    paint_faces(*volume, [&pad_top](const Vec3f &point, const Vec3f &normal) {
        return normal.z() > 0.5f && point.x() > 9.f && std::abs(point.z() - pad_top(point.x())) < 0.001f;
    });
    REQUIRE_FALSE(volume->support_base_facets.empty());
    Print print;
    slice_supports(print, model, support_base_config());
    const PrintObject &sliced = *print.get_object(size_t(0));
    const Vec3d corner = sliced.trafo_centered() * Vec3d(0., 0., 0.);
    double below_pad = 0., roof_contact = 0., on_pad = 0.;
    for (const SupportLayer *layer : sliced.support_layers())
        for (const Polyline &polyline : layer->support_fills.as_polylines()) {
            // Below the lowest point of the pad's top.
            if (layer->print_z < pad_top(10.) - 0.5)
                below_pad += unscale<double>(polyline.length());
            if (layer->print_z > 9.5 + height)
                roof_contact += unscale<double>(polyline.length());
            // Just above the pad, inside its outline: support standing on it.
            for (const Line &line : polyline.lines()) {
                const Vec2d local = line.midpoint().cast<double>() * SCALING_FACTOR - corner.head<2>();
                if (local.x() > 10.5 && local.x() < 13.5 && local.y() > 4.5 && local.y() < 7.5 &&
                    layer->print_z < pad_top(local.x()) + 1.5)
                    on_pad += unscale<double>(line.length());
            }
        }
    CAPTURE(slope, height, below_pad, roof_contact, on_pad);
    CHECK(roof_contact > 10.);
    if (slender) {
        // Only the bed can hold the cantilever, so support stands on it.
        CHECK(below_pad > 10.);
        CHECK(on_pad == 0.);
    } else
        CHECK(on_pad > 5.);
}

TEST_CASE("A roof tip resting on paint at its own layer does not stop slicing", "[SupportBase][Slicing][LowOverhang]")
{
    // A cantilever 1 mm above a painted shelf leaves room for the top gap, two
    // roof layers, one layer of tips and the bottom gap: the tips touch the
    // paint where they are placed.
    Model model;
    auto *object = model.add_object();
    TriangleMesh mesh(its_make_cube(3., 12., 13.));
    TriangleMesh shelf(its_make_cube(18., 12., 10.));
    mesh.merge(shelf);
    TriangleMesh cantilever(its_make_cube(18., 12., 1.));
    cantilever.translate(0., 0., 11.); mesh.merge(cantilever);
    auto *volume = object->add_volume(mesh);
    object->add_instance()->set_offset(Vec3d(110., 100., 0.));
    paint_faces(*volume, [](const Vec3f &point, const Vec3f &normal) {
        return normal.z() > 0.9f && std::abs(point.z() - 10.f) < 0.001f;
    });
    REQUIRE_FALSE(volume->support_base_facets.empty());
    Print print;
    REQUIRE_NOTHROW(slice_supports(print, model, support_base_config()));
    double roof = 0.;
    for (const SupportLayer *layer : print.get_object(size_t(0))->support_layers())
        if (layer->print_z > 10.3 && layer->print_z < 11.)
            for (const Polyline &polyline : layer->support_fills.as_polylines())
                roof += unscale<double>(polyline.length());
    CHECK(roof > 10.);
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
