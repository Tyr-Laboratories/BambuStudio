#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

using namespace Slic3r;

namespace {
constexpr double platform_z = 14.;
constexpr double roof_z = 46.;

DynamicPrintConfig foundation_config()
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
        {"support_on_build_plate_only", "1"},
        {"support_top_z_distance", "0.2"}, {"support_bottom_z_distance", "0.2"},
        {"support_interface_top_layers", "2"}, {"support_interface_bottom_layers", "2"},
        {"support_object_xy_distance", "0.35"}, {"tree_support_branch_angle", "40"},
        {"tree_support_branch_diameter", "2"}, {"tree_support_branch_distance", "5"},
        {"support_critical_regions_only", "0"}
    });
    return config;
}

Model foundation_model(bool painted = true, bool mixed = false, double roof_level = roof_z, double width = 28., bool multiple_roofs = false)
{
    Model model;
    auto *object = model.add_object();
    TriangleMesh mesh(its_make_cube(width, 18., platform_z));
    TriangleMesh upright(its_make_cube(3., 18., roof_level + 1.));
    mesh.merge(upright);
    TriangleMesh roof(its_make_cube(width, 18., 1.));
    roof.translate(0., 0., roof_level);
    mesh.merge(roof);
    if (multiple_roofs) {
        TriangleMesh lower_roof(its_make_cube(12., 18., 1.));
        lower_roof.translate(0., 0., 26.);
        mesh.merge(lower_roof);
    }
    if (mixed) {
        TriangleMesh control = mesh;
        control.translate(80., 0., 0.);
        mesh.merge(control);
        TriangleMesh connector(its_make_cube(width + 80., 18., 1.));
        mesh.merge(connector);
    }
    auto *volume = object->add_volume(mesh);
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    if (painted) {
        TriangleSelector selector(volume->mesh());
        const auto &its = volume->mesh().its;
        const auto transform = volume->get_matrix();
        for (size_t i = 0; i < its.indices.size(); ++i) {
            const auto &face = its.indices[i];
            const Vec3f a = (transform * its.vertices[face[0]].cast<double>()).cast<float>();
            const Vec3f b = (transform * its.vertices[face[1]].cast<double>()).cast<float>();
            const Vec3f c = (transform * its.vertices[face[2]].cast<double>()).cast<float>();
            if ((b-a).cross(c-a).z() > 0.f && std::abs(a.z() - platform_z) < 0.001 && a.x() < 40.)
                selector.set_facet(int(i), EnforcerBlockerType::ENFORCER);
        }
        volume->support_base_facets.set(selector);
        REQUIRE_FALSE(volume->support_base_facets.empty());
    }
    return model;
}

void slice_foundation(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.get_object(size_t(0))->generate_support_preview();
}

Polygons model_rectangle(const PrintObject &object, double left, double front, double right, double back)
{
    Points corners;
    for (const auto &corner : {Vec3d(left, front, 0.), Vec3d(right, front, 0.), Vec3d(right, back, 0.), Vec3d(left, back, 0.)}) {
        const Vec3d transformed = object.trafo_centered() * corner;
        corners.emplace_back(scale_(transformed.x()), scale_(transformed.y()));
    }
    return {Polygon(std::move(corners))};
}

Polygons fixture_region(const PrintObject &object, bool control = false)
{
    return control ? model_rectangle(object, 60., -40., 140., 60.) : model_rectangle(object, 3.5, 0., 28., 18.);
}

ExPolygons printed_section(const PrintObject &object, const SupportLayer &layer, bool control = false)
{
    return intersection_ex(layer.support_fills.polygons_covered_by_width(), fixture_region(object, control));
}

size_t branch_count_at(const PrintObject &object, double print_z)
{
    for (const SupportLayer *layer : object.support_layers()) {
        if (std::abs(layer->print_z - print_z) < EPSILON) {
            const auto sections = printed_section(object, *layer);
            return size_t(std::count_if(sections.begin(), sections.end(), [](const ExPolygon &section) {
                return section.area() * SCALING_FACTOR * SCALING_FACTOR > 0.05;
            }));
        }
    }
    FAIL("Expected support layer is missing");
    return 0;
}
}

TEST_CASE("Model-root organic trees have separated foundation feet below their midpoint join", "[OrganicFoundation][Slicing]")
{
    auto model = foundation_model();
    Print print;
    slice_foundation(print, model, foundation_config());
    const PrintObject &object = *print.get_object(size_t(0));
    const size_t lower = branch_count_at(object, 15.4);
    const size_t middle = branch_count_at(object, 30.);
    const size_t upper = branch_count_at(object, 44.4);
    CAPTURE(lower, middle, upper);
    CHECK(lower >= 2);
    CHECK(lower > middle);
    CHECK(middle >= 1);
    CHECK(upper > middle);

    // Counted feet must form printable paths through the join, not isolated
    // islands drawn only at the three sampled heights.
    Polygons previous;
    double previous_z = 0.;
    size_t checked_layers = 0;
    // Printed footprints undergo half-line-width offsets and contour
    // regularization after the branch geometry is accepted. Allow half a line
    // width plus both contours' resolution as a conservative diagnostic
    // envelope, not an exact bound on branch-center movement.
    const double contour_allowance = 0.5 * support_material_flow(&object).width() +
                                     2. * print.config().resolution.value;
    for (const SupportLayer *layer : object.support_layers()) {
        if (layer->print_z < 15.2 || layer->print_z > 44.4 + EPSILON)
            continue;
        const auto section = printed_section(object, *layer);
        CAPTURE(layer->print_z);
        REQUIRE_FALSE(section.empty());
        if (!previous.empty()) {
            CHECK(layer->print_z - previous_z == Approx(0.2).margin(0.00001));
            const Polygons supported = offset(previous, scaled<coord_t>(0.01));
            for (const ExPolygon &branch : section)
                if (branch.area() * SCALING_FACTOR * SCALING_FACTOR > 0.05)
                    CHECK_FALSE(intersection(to_polygons(branch), supported).empty());
            if (layer->print_z >= 29.6 - EPSILON && layer->print_z <= 30.2 + EPSILON) {
                // The configured slope and radius growth allow less than
                // 0.2 mm per layer before normal contour processing. Check
                // the printed walls: filled disks alone can conceal a large
                // unsupported hollow-perimeter shoulder at the midpoint.
                const double unsupported_area = area(diff_ex(to_polygons(section), offset(previous, scaled<coord_t>(0.2 + contour_allowance)))) *
                                                SCALING_FACTOR * SCALING_FACTOR;
                CAPTURE(unsupported_area);
                CHECK(unsupported_area < 0.001);
            }
        }
        previous = to_polygons(section);
        previous_z = layer->print_z;
        ++checked_layers;
    }
    CHECK(checked_layers > 140);

    double first_support_bottom = std::numeric_limits<double>::max();
    for (const SupportLayer *layer : object.support_layers())
        if (!printed_section(object, *layer).empty())
            first_support_bottom = std::min(first_support_bottom, layer->bottom_z());
    CHECK(first_support_bottom == Approx(14.2).margin(0.01));
}

TEST_CASE("Organic foundation generation preserves build-plate trees in a mixed scene", "[OrganicFoundation][BuildPlate]")
{
    auto painted_model = foundation_model(true, true);
    auto config = foundation_config();
    Print painted;
    slice_foundation(painted, painted_model, config);
    const auto &after = *painted.get_object(size_t(0));
    double first_model_bottom = std::numeric_limits<double>::max();
    double first_bed_bottom = std::numeric_limits<double>::max();
    double bed_support_area = 0., upper_contact_area = 0.;
    for (const SupportLayer *layer : after.support_layers()) {
        if (!printed_section(after, *layer).empty())
            first_model_bottom = std::min(first_model_bottom, layer->bottom_z());
        const auto control = printed_section(after, *layer, true);
        if (!control.empty()) {
            first_bed_bottom = std::min(first_bed_bottom, layer->bottom_z());
            if (layer->print_z < platform_z)
                bed_support_area += area(control) * SCALING_FACTOR * SCALING_FACTOR;
            if (layer->print_z > roof_z - 0.6)
                upper_contact_area += area(control) * SCALING_FACTOR * SCALING_FACTOR;
        }
    }
    REQUIRE(bed_support_area > 10.);
    CHECK(upper_contact_area > 10.);
    CHECK(first_model_bottom == Approx(14.2).margin(0.01));
    CHECK(first_bed_bottom == Approx(0.));
}

TEST_CASE("Organic trees without model foundations stay rooted on the build plate", "[OrganicFoundation][BuildPlateBaseline]")
{
    auto model = foundation_model(false);
    Print print;
    slice_foundation(print, model, foundation_config());
    const auto &object = *print.get_object(size_t(0));
    REQUIRE_FALSE(object.support_layers().empty());
    CHECK(object.support_layers().front()->bottom_z() == Approx(0.));
}

TEST_CASE("Unrestricted organic model roots grow foundations without base painting", "[OrganicFoundation][Unpainted]")
{
    auto model = foundation_model(false);
    // Close the other three sides so a tree cannot escape around the platform
    // to the build plate. Its only permitted foundation is the unpainted floor.
    auto add_wall = [&](double x, double y, double width, double depth) {
        TriangleMesh wall(its_make_cube(width, depth, roof_z + 1.));
        wall.translate(x, y, 0.);
        model.objects.front()->add_volume(wall);
    };
    add_wall(25., 0., 3., 18.);
    add_wall(0., 0., 28., 3.);
    add_wall(0., 15., 28., 3.);
    auto config = foundation_config();
    config.set_deserialize_strict({{"support_on_build_plate_only", "0"}, {"bridge_no_support", "0"}});
    Print print;
    slice_foundation(print, model, config);
    const auto &object = *print.get_object(size_t(0));
    REQUIRE_FALSE(object.support_layers().empty());
    REQUIRE(object.support_layers().front()->bottom_z() == Approx(14.2).margin(0.01));
    const size_t feet = branch_count_at(object, 15.4);
    const size_t middle = branch_count_at(object, 30.);
    CAPTURE(feet, middle);
    CHECK(feet >= 2);
    CHECK(feet > middle);
    double first_bottom = std::numeric_limits<double>::max();
    for (const SupportLayer *layer : object.support_layers())
        if (!printed_section(object, *layer).empty())
            first_bottom = std::min(first_bottom, layer->bottom_z());
    CHECK(first_bottom == Approx(14.2).margin(0.01));
}

TEST_CASE("Model foundation bottom interfaces do not change distant build-plate top interfaces", "[OrganicFoundation][InterfacePhase]")
{
    auto without_model_bottom_model = foundation_model(true, true);
    auto with_model_bottom_model = foundation_model(true, true);
    auto config = foundation_config();
    Print without_model_bottom, with_model_bottom;
    config.set_deserialize_strict({{"support_interface_bottom_layers", "0"}});
    slice_foundation(without_model_bottom, without_model_bottom_model, config);
    config.set_deserialize_strict({{"support_interface_bottom_layers", "2"}});
    slice_foundation(with_model_bottom, with_model_bottom_model, config);
    const auto &before = *without_model_bottom.get_object(size_t(0));
    const auto &after = *with_model_bottom.get_object(size_t(0));
    const auto after_layers = after.support_layers();
    size_t checked_layers = 0;
    for (const SupportLayer *layer : before.support_layers()) {
        if (layer->print_z < roof_z - 0.6 - EPSILON)
            continue;
        const auto expected = printed_section(before, *layer, true);
        if (expected.empty())
            continue;
        const auto found = std::find_if(after_layers.begin(), after_layers.end(), [&](const SupportLayer *candidate) {
            return std::abs(candidate->print_z - layer->print_z) < EPSILON;
        });
        REQUIRE(found != after_layers.end());
        const auto actual = printed_section(after, **found, true);
        CAPTURE(layer->print_z);
        // The stock slicer produces sub-micron clipping slivers when bottom
        // interfaces change. A changed top-interface phase differs by tens of
        // square millimeters, well above this geometric rounding tolerance.
        CHECK(area(diff_ex(expected, actual)) * SCALING_FACTOR * SCALING_FACTOR < 0.001);
        CHECK(area(diff_ex(actual, expected)) * SCALING_FACTOR * SCALING_FACTOR < 0.001);
        ++checked_layers;
    }
    REQUIRE(checked_layers >= 2);
}

TEST_CASE("Organic foundation feet use the existing tree branch distance setting", "[OrganicFoundation][Settings]")
{
    std::array<size_t, 2> lower{}, upper{};
    for (size_t i = 0; i < 2; ++i) {
        auto config = foundation_config();
        config.set_deserialize_strict({
            {"tree_support_branch_distance", i == 0 ? "3" : "8"},
            {"support_interface_top_layers", "0"}, {"support_interface_bottom_layers", "0"}
        });
        auto model = foundation_model();
        Print print;
        slice_foundation(print, model, config);
        const auto &object = *print.get_object(size_t(0));
        lower[i] = branch_count_at(object, 15.4);
        upper[i] = branch_count_at(object, 44.4);
    }
    CAPTURE(lower[0], lower[1], upper[0], upper[1]);
    REQUIRE(upper[0] > upper[1]);
    CHECK(lower[0] > lower[1]);
}

TEST_CASE("Organic foundations respond to the branch angle and retain contacts when routing is too restrictive", "[OrganicFoundation][Angle]")
{
    std::array<ExPolygons, 2> lower_sections;
    for (size_t i = 0; i < 3; ++i) {
        const char *angle = i == 0 ? "35" : i == 1 ? "50" : "25";
        auto config = foundation_config();
        config.set_deserialize_strict({{"tree_support_branch_angle", angle}});
        auto model = foundation_model();
        Print print;
        slice_foundation(print, model, config);
        const auto &object = *print.get_object(size_t(0));
        CAPTURE(angle);
        if (i < 2) {
            const size_t feet = branch_count_at(object, 15.4);
            const size_t middle = branch_count_at(object, 30.);
            CAPTURE(feet, middle);
            CHECK(feet > middle);
            CHECK(middle >= 1);
        }
        double first_bottom = std::numeric_limits<double>::max();
        double last_top = 0.;
        for (const SupportLayer *layer : object.support_layers()) {
            const auto section = printed_section(object, *layer);
            if (!section.empty()) {
                first_bottom = std::min(first_bottom, layer->bottom_z());
                last_top = std::max(last_top, layer->print_z);
            }
            if (i < 2 && std::abs(layer->print_z - 18.) < EPSILON)
                lower_sections[i] = section;
        }
        // A restrictive angle may safely retain the original trunk instead
        // of accepting a foundation, but neither contact may be lost.
        CHECK(first_bottom == Approx(14.2).margin(0.01));
        CHECK(last_top == Approx(45.8).margin(0.01));
    }
    REQUIRE_FALSE(lower_sections[0].empty());
    REQUIRE_FALSE(lower_sections[1].empty());
    // Different angle limits must affect the actual lower branch paths. The
    // generator does not promise a monotonic number or span of merged feet.
    const double changed_area = (area(diff_ex(lower_sections[0], lower_sections[1])) +
                                 area(diff_ex(lower_sections[1], lower_sections[0]))) * SCALING_FACTOR * SCALING_FACTOR;
    CHECK(changed_area > 1.);
}

TEST_CASE("Organic foundations retain support contacts in short and narrow model gaps", "[OrganicFoundation][Fallback]")
{
    const bool narrow = GENERATE(false, true);
    const double roof_level = narrow ? roof_z : 18.;
    auto model = foundation_model(true, false, roof_level, narrow ? 8. : 28.);
    Print print;
    slice_foundation(print, model, foundation_config());
    const auto &object = *print.get_object(size_t(0));
    double first_bottom = std::numeric_limits<double>::max();
    double last_top = 0.;
    for (const SupportLayer *layer : object.support_layers()) {
        if (!printed_section(object, *layer).empty()) {
            first_bottom = std::min(first_bottom, layer->bottom_z());
            last_top = std::max(last_top, layer->print_z);
        }
    }
    CAPTURE(narrow, first_bottom, last_top);
    CHECK(first_bottom == Approx(14.2).margin(0.01));
    CHECK(last_top == Approx(roof_level - 0.2).margin(0.01));
}

TEST_CASE("Organic foundation replacement retains lower and upper overhang contacts", "[OrganicFoundation][MultiLevel]")
{
    auto model = foundation_model(true, false, roof_z, 28., true);
    Print print;
    slice_foundation(print, model, foundation_config());
    const auto &object = *print.get_object(size_t(0));
    for (double contact_z : {25.8, 45.8}) {
        const auto layers = object.support_layers();
        const auto found = std::find_if(layers.begin(), layers.end(), [contact_z](const SupportLayer *layer) {
            return std::abs(layer->print_z - contact_z) < EPSILON;
        });
        CAPTURE(contact_z);
        REQUIRE(found != layers.end());
        // Check the actual undersides separately. The tall roof's trunks may
        // cross the lower height elsewhere and must not hide a lost shelf tip.
        const auto contact_region = contact_z < 30. ? model_rectangle(object, 4., 0.5, 11.5, 17.5) :
                                                       model_rectangle(object, 14., 0.5, 27., 17.5);
        const auto contact = intersection_ex((*found)->support_fills.polygons_covered_by_width(), contact_region);
        CHECK(area(contact) * SCALING_FACTOR * SCALING_FACTOR > 10.);
    }
}
