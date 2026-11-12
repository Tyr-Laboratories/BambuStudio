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

Model foundation_model(bool painted = true, bool mixed = false, double roof_level = roof_z, double width = 28., bool multiple_roofs = false, double depth = 18.)
{
    Model model;
    auto *object = model.add_object();
    TriangleMesh mesh(its_make_cube(width, depth, platform_z));
    TriangleMesh upright(its_make_cube(3., depth, roof_level + 1.));
    mesh.merge(upright);
    TriangleMesh roof(its_make_cube(width, depth, 1.));
    roof.translate(0., 0., roof_level);
    mesh.merge(roof);
    if (multiple_roofs) {
        TriangleMesh lower_roof(its_make_cube(12., depth, 1.));
        lower_roof.translate(0., 0., 26.);
        mesh.merge(lower_roof);
    }
    if (mixed) {
        TriangleMesh control = mesh;
        control.translate(80., 0., 0.);
        mesh.merge(control);
        TriangleMesh connector(its_make_cube(width + 80., depth, 1.));
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
            if ((b-a).cross(c-a).z() > 0.f && std::abs(a.z() - platform_z) < 0.001 && a.x() < (mixed ? 40. : width + 1.))
                selector.set_facet(int(i), EnforcerBlockerType::ENFORCER);
        }
        volume->support_base_facets.set(selector);
        REQUIRE_FALSE(volume->support_base_facets.empty());
    }
    return model;
}

// The foundation fixture with its platform top tilted about its centre line,
// rising along X. The paint stops short of the slope's upper edge, which has no
// layer above it and so counts as flat whatever the slope.
constexpr double sloped_paint_end_x = 24.;
double sloped_platform_top(double x, double slope_deg) { return platform_z + std::tan(slope_deg * M_PI / 180.) * (x - 14.); }

Model sloped_foundation_model(double slope_deg)
{
    const auto top = [slope_deg](double x) { return sloped_platform_top(x, slope_deg); };
    Model model;
    auto *object = model.add_object();
    // Two blocks meeting at the paint's end, so a triangle edge lies there.
    TriangleMesh mesh;
    for (const auto &[from, to] : {std::make_pair(0., sloped_paint_end_x), std::make_pair(sloped_paint_end_x, 28.)}) {
        indexed_triangle_set block = its_make_cube(to - from, 18., platform_z);
        for (Vec3f &vertex : block.vertices) {
            vertex.x() += float(from);
            if (vertex.z() > 0.5f * float(platform_z))
                vertex.z() = float(top(vertex.x()));
        }
        mesh.merge(TriangleMesh(std::move(block)));
    }
    TriangleMesh upright(its_make_cube(3., 18., roof_z + 1.));
    mesh.merge(upright);
    TriangleMesh roof(its_make_cube(28., 18., 1.));
    roof.translate(0., 0., roof_z);
    mesh.merge(roof);
    auto *volume = object->add_volume(mesh);
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    TriangleSelector selector(volume->mesh());
    const auto &its = volume->mesh().its;
    const auto transform = volume->get_matrix();
    for (size_t i = 0; i < its.indices.size(); ++i) {
        const auto &face = its.indices[i];
        const Vec3f a = (transform * its.vertices[face[0]].cast<double>()).cast<float>();
        const Vec3f b = (transform * its.vertices[face[1]].cast<double>()).cast<float>();
        const Vec3f c = (transform * its.vertices[face[2]].cast<double>()).cast<float>();
        const Vec3f center = (a + b + c) / 3.f;
        if ((b - a).cross(c - a).z() > 0.f && std::abs(center.z() - float(top(center.x()))) < 0.001f &&
            std::max({a.x(), b.x(), c.x()}) < float(sloped_paint_end_x) + 0.001f)
            selector.set_facet(int(i), EnforcerBlockerType::ENFORCER);
    }
    volume->support_base_facets.set(selector);
    REQUIRE_FALSE(volume->support_base_facets.empty());
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

TEST_CASE("Painted slopes flatter than the support threshold angle carry foundation feet", "[OrganicFoundation][Slope]")
{
    // Tips stand on overhangs flatter than the support threshold angle, however
    // thin a slope leaves each layer's band. Mirrored, so do the feet of a
    // foundation on painted bases, on the slope itself. Steeper paint carries no
    // support, as steeper overhangs get no tips. The fixture's 30 degrees count as 31.
    const double slope = GENERATE(15., 25., 45.);
    auto model = sloped_foundation_model(slope);
    Print print;
    slice_foundation(print, model, foundation_config());
    const PrintObject &object = *print.get_object(size_t(0));
    const Vec3d corner = object.trafo_centered() * Vec3d(0., 0., 0.);
    // A foot is a support island with no support under it, starting just above
    // the painted slope, clear of the slope's ends.
    size_t feet = 0;
    Polygons below;
    for (const SupportLayer *layer : object.support_layers()) {
        const Polygons here = layer->support_fills.polygons_covered_by_width();
        for (const ExPolygon &island : union_ex(here)) {
            if (island.area() * SCALING_FACTOR * SCALING_FACTOR <= 0.05 ||
                ! intersection(to_polygons(island), offset(below, scaled<float>(0.01))).empty())
                continue;
            const Vec2d local = island.contour.centroid().cast<double>() * SCALING_FACTOR - corner.head<2>();
            const double clearance = layer->bottom_z() - sloped_platform_top(local.x(), slope);
            if (local.x() > 4. && local.x() < sloped_paint_end_x - 1. && local.y() > 1. && local.y() < 17. &&
                clearance > 0. && clearance < 1.)
                ++ feet;
        }
        below = here;
    }
    const size_t middle = branch_count_at(object, 32.);
    CAPTURE(slope, feet, middle);
    if (slope < 31.)
        CHECK(feet > middle);
    else
        CHECK(feet == 0);
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

TEST_CASE("Equal-width thin ledges retain substantial lower organic branching", "[OrganicFoundation][ThinLedge]")
{
    // Both contact surfaces are 80 x 4 mm. These dimensions and settings
    // reproduce a long, thin painted ledge whose original foundation stayed
    // a single stem until a tiny fork immediately above the contact.
    auto model = foundation_model(true, false, 63., 80., false, 4.);
    auto config = foundation_config();
    config.set_deserialize_strict({
        {"support_top_z_distance", "0.3"}, {"support_bottom_z_distance", "0.3"},
        {"support_interface_top_layers", "0"}, {"support_interface_bottom_layers", "2"},
        {"support_line_width", "0.35"}, {"support_object_xy_distance", "0.2"},
        {"tree_support_branch_angle", "35"}, {"tree_support_branch_diameter", "2.5"},
        {"tree_support_branch_diameter_angle", "4"}, {"tree_support_branch_distance", "2"},
        {"resolution", "0.012"}, {"slice_closing_radius", "0.012"}, {"bridge_no_support", "0"},
        {"support_threshold_angle", "35"}
    });
    Print print;
    slice_foundation(print, model, config);
    const auto &object = *print.get_object(size_t(0));
    const auto region = model_rectangle(object, 3.5, -4., 80.5, 8.);
    struct Section { size_t branches = 0; double span = 0.; };
    auto section_at = [&](double z, const Polygons &window) {
        Section result;
        double left = std::numeric_limits<double>::max();
        double right = std::numeric_limits<double>::lowest();
        const auto layers = object.support_layers();
        const auto found = std::find_if(layers.begin(), layers.end(), [z](const SupportLayer *layer) {
            return std::abs(layer->print_z - z) < EPSILON;
        });
        REQUIRE(found != layers.end());
        for (const auto &part : intersection_ex((*found)->support_fills.polygons_covered_by_width(), window)) {
            if (part.area() * SCALING_FACTOR * SCALING_FACTOR <= 0.05)
                continue;
            ++result.branches;
            const auto bounds = get_extents(part.contour);
            left = std::min(left, unscale<double>(bounds.min.x()));
            right = std::max(right, unscale<double>(bounds.max.x()));
        }
        REQUIRE(result.branches > 0);
        result.span = right - left;
        return result;
    };
    const auto lower = section_at(18., region);
    const auto lower_quarter = section_at(24., region);
    const auto middle = section_at(38., region);
    const auto upper = section_at(62., region);
    CAPTURE(lower.branches, lower_quarter.branches, middle.branches, upper.branches,
            lower.span, middle.span);
    // Require sustained branching at two heights, well beyond the contact
    // tip layers. A tiny near-contact fork on a long stem must not pass.
    CHECK(lower.branches > middle.branches);
    CHECK(lower_quarter.branches > middle.branches);
    CHECK(lower.span > middle.span);
    CHECK(upper.branches > middle.branches);
    // One successful foundation must not conceal the other forest falling
    // back to its original stem. These outboard windows exclude feet from
    // the neighboring forest that cross the center of the ledge.
    for (const auto bounds : {std::array<double, 2>{4., 25.}, std::array<double, 2>{55., 80.}}) {
        const auto outboard = model_rectangle(object, bounds[0], -4., bounds[1], 8.);
        const auto outboard_lower = section_at(18., outboard);
        const auto outboard_quarter = section_at(24., outboard);
        const auto outboard_middle = section_at(38., outboard);
        CAPTURE(bounds[0], bounds[1], outboard_lower.branches, outboard_quarter.branches, outboard_middle.branches);
        CHECK(outboard_lower.branches > outboard_middle.branches);
        // Ordinary branches may already have merged to the number of seam
        // stems here; require two actual branches, not monotonic counts.
        CHECK(outboard_quarter.branches >= 2);
    }

    // The clear outer-left trunk is unbranched across the artificial midpoint.
    // Its printed walls should follow the chord between its fixed endpoints,
    // rather than preserving the several-millimeter bend from two separate runs.
    const auto trunk_window = model_rectangle(object, 4., -4., 18., 8.);
    std::vector<std::pair<double, Vec2d>> trunk_centroids;
    for (const SupportLayer *layer : object.support_layers()) {
        if (layer->print_z < 27. - EPSILON || layer->print_z > 49. + EPSILON)
            continue;
        const auto section = intersection_ex(layer->support_fills.polygons_covered_by_width(), trunk_window);
        REQUIRE(section.size() == 1);
        // Signed hole areas matter: measure the actual hollow extrusion footprint,
        // not the solid outer contour or a branch-center implementation detail.
        Vec2d weighted_center = Vec2d::Zero();
        double signed_area = 0.;
        for (const Polygon &ring : to_polygons(section)) {
            const double ring_area = ring.area();
            weighted_center += ring.centroid().cast<double>() * ring_area;
            signed_area += ring_area;
        }
        REQUIRE(signed_area > 0.);
        trunk_centroids.emplace_back(layer->print_z, weighted_center * (SCALING_FACTOR / signed_area));
    }
    REQUIRE(trunk_centroids.size() > 100);
    double maximum_departure = 0.;
    const auto &first = trunk_centroids.front();
    const auto &last = trunk_centroids.back();
    for (const auto &[z, center] : trunk_centroids) {
        const Vec2d chord = first.second + (last.second - first.second) * ((z - first.first) / (last.first - first.first));
        maximum_departure = std::max(maximum_departure, (center - chord).norm());
    }
    CAPTURE(maximum_departure);
    CHECK(maximum_departure < 0.2);

    // Preserve the original layer-quantized contact interval, and require
    // every substantial printed component to stay attached through the join.
    REQUIRE_FALSE(object.support_layers().empty());
    CHECK(object.support_layers().front()->bottom_z() == Approx(14.2).margin(0.01));
    CHECK(object.support_layers().back()->print_z == Approx(62.8).margin(0.01));
    Polygons previous;
    for (const SupportLayer *layer : object.support_layers()) {
        const auto section = intersection_ex(layer->support_fills.polygons_covered_by_width(), region);
        if (layer->print_z < 15.)
            continue;
        REQUIRE_FALSE(section.empty());
        if (!previous.empty()) {
            CAPTURE(layer->print_z);
            for (const auto &part : section)
                if (part.area() * SCALING_FACTOR * SCALING_FACTOR > 0.05)
                    CHECK_FALSE(intersection(to_polygons(part), previous).empty());
        }
        previous = to_polygons(section);
    }
}

TEST_CASE("Organic foundation bottom interfaces inherit the top interface count", "[OrganicFoundation][SameAsTop]")
{
    auto explicit_model = foundation_model();
    auto inherited_model = foundation_model();
    auto config = foundation_config();
    Print explicit_count, inherited_count;
    slice_foundation(explicit_count, explicit_model, config);
    config.set_deserialize_strict({{"support_interface_bottom_layers", "-1"}});
    slice_foundation(inherited_count, inherited_model, config);
    const auto &expected = *explicit_count.get_object(size_t(0));
    const auto &actual = *inherited_count.get_object(size_t(0));
    const auto actual_layers = actual.support_layers();
    size_t checked_layers = 0;
    for (const auto *layer : expected.support_layers()) {
        if (layer->print_z > platform_z + 1. + EPSILON)
            continue;
        const auto found = std::find_if(actual_layers.begin(), actual_layers.end(), [&](const SupportLayer *candidate) {
            return std::abs(candidate->print_z - layer->print_z) < EPSILON;
        });
        REQUIRE(found != actual_layers.end());
        const auto before = printed_section(expected, *layer);
        const auto after = printed_section(actual, **found);
        const double difference = (area(diff_ex(before, after)) + area(diff_ex(after, before))) *
                                  SCALING_FACTOR * SCALING_FACTOR;
        CAPTURE(layer->print_z, difference);
        // -1 means the same count as the two top-interface layers. It must
        // preserve their bottom geometry, including actual foundation tips.
        CHECK(difference < 0.001);
        ++checked_layers;
    }
    REQUIRE(checked_layers >= 3);
}

TEST_CASE("Organic foundation tips stay supported with asymmetric interface counts", "[OrganicFoundation][BottomZero][AsymmetricInterfaces]")
{
    const auto counts = GENERATE(std::array<int, 2>{2, 0}, std::array<int, 2>{4, 1}, std::array<int, 2>{4, 2});
    CAPTURE(counts[0], counts[1]);
    auto model = foundation_model();
    auto config = foundation_config();
    config.set_deserialize_strict({
        {"support_interface_top_layers", std::to_string(counts[0])},
        {"support_interface_bottom_layers", std::to_string(counts[1])}
    });
    Print print;
    slice_foundation(print, model, config);
    const auto &object = *print.get_object(size_t(0));
    Polygons previous;
    size_t checked_layers = 0;
    for (const SupportLayer *layer : object.support_layers()) {
        if (layer->print_z > platform_z + 2. + EPSILON)
            break;
        const auto section = printed_section(object, *layer);
        REQUIRE_FALSE(section.empty());
        CAPTURE(layer->print_z);
        if (!previous.empty())
            for (const auto &part : section)
                if (part.area() * SCALING_FACTOR * SCALING_FACTOR > 0.05)
                    CHECK_FALSE(intersection(to_polygons(part), previous).empty());
        previous = to_polygons(section);
        ++checked_layers;
    }
    REQUIRE(checked_layers >= 4);
}

TEST_CASE("Organic foundation interfaces follow the existing top-interface enable switch", "[OrganicFoundation][InterfaceRoles]")
{
    const bool short_gap = GENERATE(false, true);
    const double roof_level = short_gap ? 18. : roof_z;
    CAPTURE(short_gap);
    auto without_model = foundation_model(true, false, roof_level);
    auto with_model = foundation_model(true, false, roof_level);
    auto config = foundation_config();
    config.set_deserialize_strict({{"support_interface_top_layers", "0"}, {"support_interface_bottom_layers", "0"}});
    Print without_bottom, with_bottom;
    slice_foundation(without_bottom, without_model, config);
    config.set_deserialize_strict({{"support_interface_bottom_layers", "2"}});
    slice_foundation(with_bottom, with_model, config);
    const auto &before = *without_bottom.get_object(size_t(0));
    const auto &after = *with_bottom.get_object(size_t(0));
    if (!short_gap) {
        REQUIRE(branch_count_at(before, 15.4) > branch_count_at(before, 30.));
        REQUIRE(branch_count_at(after, 15.4) > branch_count_at(after, 30.));
    }
    REQUIRE_FALSE(before.support_layers().empty());
    CHECK(before.support_layers().front()->bottom_z() == Approx(platform_z + 0.2).margin(0.01));
    CHECK(before.support_layers().back()->print_z == Approx(roof_level - 0.2).margin(0.01));
    const auto after_layers = after.support_layers();
    REQUIRE(before.support_layers().size() == after_layers.size());
    size_t interface_entities = 0;
    for (const SupportLayer *layer : before.support_layers()) {
        const auto found = std::find_if(after_layers.begin(), after_layers.end(), [&](const SupportLayer *candidate) {
            return std::abs(candidate->print_z - layer->print_z) < EPSILON;
        });
        REQUIRE(found != after_layers.end());
        const auto expected = printed_section(before, *layer);
        const auto actual = printed_section(after, **found);
        const double difference = (area(diff_ex(expected, actual)) + area(diff_ex(actual, expected))) *
                                  SCALING_FACTOR * SCALING_FACTOR;
        CAPTURE(layer->print_z, difference);
        CHECK(difference < 0.001);
        const auto flat = (*found)->support_fills.flatten();
        for (const ExtrusionEntity *entity : flat.entities)
            interface_entities += entity->role() == erSupportMaterialInterface;
    }
    // A zero top-interface count disables interfaces in the existing renderer.
    // A stored bottom count must not create interface bars above a base layer,
    // including original support retained when a short gap cannot form a foundation.
    CHECK(interface_entities == 0);

    auto enabled_model = foundation_model(true, false, roof_level);
    config.set_deserialize_strict({{"support_interface_top_layers", "2"}});
    Print enabled;
    slice_foundation(enabled, enabled_model, config);
    const auto &object = *enabled.get_object(size_t(0));
    REQUIRE_FALSE(object.support_layers().empty());
    const SupportLayer &first = *object.support_layers().front();
    CHECK(first.bottom_z() == Approx(platform_z + 0.2).margin(0.01));
    Polygons interface_footprint;
    const auto flat = first.support_fills.flatten();
    for (const ExtrusionEntity *entity : flat.entities)
        if (entity->role() == erSupportMaterialInterface)
            append(interface_footprint, entity->polygons_covered_by_width());
    CHECK(area(intersection_ex(interface_footprint, fixture_region(object))) *
          SCALING_FACTOR * SCALING_FACTOR > 0.05);
}
