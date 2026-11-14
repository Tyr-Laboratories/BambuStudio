#include <catch2/catch.hpp>

#include "libslic3r/WallInternalExtension.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <algorithm>
#include <cmath>

using namespace Slic3r;
using namespace Slic3r::WallInternalExtension;

namespace {
constexpr double line_w = 0.45;
// Spacing of 0.45 mm lines at 0.2 mm layers (Flow: width - height * (1 - pi / 4)).
const double line_spacing_w = line_w - 0.2 * (1. - 0.25 * M_PI);

ExPolygon rect(double x0, double y0, double x1, double y1)
{
    Polygon p;
    p.points = { Point::new_scale(x0, y0), Point::new_scale(x1, y0), Point::new_scale(x1, y1), Point::new_scale(x0, y1) };
    return ExPolygon(p);
}

ExPolygon centred_square(double cx, double cy, double half) { return rect(cx - half, cy - half, cx + half, cy + half); }

// One LayerGeometry per outline, `h` mm thick, with walls covering the band within walls * line_w of the outline.
std::vector<LayerGeometry> stack(const std::vector<ExPolygons> &outlines, double h = 0.2, int walls = 1)
{
    std::vector<LayerGeometry> layers;
    for (size_t i = 0; i < outlines.size(); ++i) {
        LayerGeometry g;
        g.bottom_z = h * double(i);
        g.print_z  = h * double(i + 1);
        g.outline  = outlines[i];
        g.walls    = diff_ex(outlines[i], offset_ex(outlines[i], -scale_(walls * line_w)));
        layers.emplace_back(std::move(g));
    }
    return layers;
}

// Layers with the given heights, all with the same outline.
std::vector<LayerGeometry> stack_of_heights(const std::vector<double> &heights)
{
    std::vector<LayerGeometry> layers;
    double z = 0.;
    for (double h : heights) {
        LayerGeometry g;
        g.bottom_z = z;
        g.print_z  = z + h;
        g.outline  = { centred_square(10., 10., 10.) };
        z += h;
        layers.emplace_back(std::move(g));
    }
    return layers;
}

std::vector<ExPolygons> repeat(const ExPolygons &outline, size_t count) { return std::vector<ExPolygons>(count, outline); }

std::vector<ExPolygons> concat(std::vector<ExPolygons> a, const std::vector<ExPolygons> &b)
{
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

// Closed centrelines of the outer wall of `outline`.
Polylines outer_lines(const ExPolygons &outline) { return to_polylines(offset_ex(outline, -scale_(line_w / 2.))); }

JointSettings settings(double extension = 1.)
{
    JointSettings s;
    s.extension            = extension;
    s.top_shell_layers     = 4;
    s.bottom_shell_layers  = 4;
    s.line_width           = scaled<coord_t>(line_w);
    s.line_spacing         = scaled<coord_t>(line_spacing_w);
    s.tolerance            = scaled<coord_t>(0.012);
    // These cases build the chamfer; the setting itself is off by default.
    s.chamfer              = true;
    return s;
}

double length_mm(const Polylines &lines) { return unscale<double>(total_length(lines)); }

const ExPolygons box20   { rect(0., 0., 20., 20.) };
const ExPolygons tower10 { centred_square(10., 10., 5.) };
}

TEST_CASE("A tower rising out of a flat top surface is a top joint", "[WallExtension][Joints]")
{
    const auto layers = stack(concat(repeat(box20, 15), repeat(tower10, 15)));
    const auto joints = find_joints(layers, 15, outer_lines(tower10), settings());
    REQUIRE(joints.size() == 1);
    CHECK(joints.front().side == JointSide::Top);
    CHECK(joints.front().layer == 15);
    CHECK(length_mm(joints.front().lines) == Approx(4. * (10. - line_w)).margin(0.5));
    CHECK(joints.front().shell_depth == Approx(0.8));
    CHECK(joints.front().extension == 1.);
    CHECK(find_joints(layers, 16, outer_lines(tower10), settings()).empty());
    CHECK(find_joints(layers, 14, outer_lines(box20), settings()).empty());
}

TEST_CASE("A column under a wider slab is a bottom joint", "[WallExtension][Joints]")
{
    const auto layers = stack(concat(repeat(tower10, 15), repeat(box20, 15)));
    const auto joints = find_joints(layers, 14, outer_lines(tower10), settings());
    REQUIRE(joints.size() == 1);
    CHECK(joints.front().side == JointSide::Bottom);
    CHECK(length_mm(joints.front().lines) == Approx(4. * (10. - line_w)).margin(0.5));
    CHECK(joints.front().shell_depth == Approx(0.8));
    CHECK(find_joints(layers, 15, outer_lines(box20), settings()).empty());
}

TEST_CASE("A pocket's walls rising from its floor are a top joint", "[WallExtension][Joints]")
{
    ExPolygon pocket = rect(0., 0., 20., 20.);
    pocket.holes.emplace_back(centred_square(10., 10., 3.).contour);
    pocket.holes.back().reverse();
    const ExPolygons with_pocket { pocket };
    const auto layers = stack(concat(repeat(box20, 15), repeat(with_pocket, 15)));
    const auto joints = find_joints(layers, 15, outer_lines(with_pocket), settings());
    REQUIRE(joints.size() == 1);
    CHECK(joints.front().side == JointSide::Top);
    CHECK(length_mm(joints.front().lines) == Approx(4. * (6. + line_w)).margin(0.5));
}

TEST_CASE("Only walls strictly steeper than 45 degrees and at least E tall qualify", "[WallExtension][Joints]")
{
    // Change of the tower's half-width per 0.2 mm layer, number of tower layers, whether the base is a joint.
    const auto [step, count, expected] = GENERATE(table<double, int, bool>({
        { 0.,      15, true  },   // vertical
        { -0.1155, 15, true  },   // leaning in at 60 degrees
        { 0.1155,  15, true  },   // leaning out at 60 degrees
        { -0.2,    15, false },   // 45 degree ramp above the step
        { -0.55,   15, false },   // 20 degree slope: every layer steps in again
        { 0.,      2,  false },   // 0.4 mm lip
        { 0.,      4,  false },   // 0.8 mm: one layer short of E
        { 0.,      5,  true  },   // exactly 1.0 mm tall
    }));
    CAPTURE(step, count, expected);
    std::vector<ExPolygons> outlines = repeat(box20, 15);
    for (int k = 0; k < count; ++k)
        outlines.push_back({ centred_square(10., 10., 5. + step * k) });
    const auto layers = stack(outlines);
    CHECK(find_joints(layers, 15, outer_lines(outlines[15]), settings()).empty() != expected);
}

TEST_CASE("An extension of one layer still needs the wall above to be steep", "[WallExtension][Joints]")
{
    // A 15 degree skin: every layer steps in by 0.2 / tan(15) = 0.746 mm, so every layer's wall stands on the one below.
    std::vector<ExPolygons> skin;
    for (int k = 0; k < 20; ++k)
        skin.push_back({ centred_square(15., 15., 15. - 0.746 * k) });
    const auto layers = stack(skin);
    for (size_t l = 1; l + 1 < layers.size(); ++l) {
        CAPTURE(l);
        CHECK(find_joints(layers, l, outer_lines(skin[l]), settings(0.2)).empty());
    }
    // A vertical tower is still a joint with such a short extension.
    const auto tower = stack(concat(repeat(box20, 15), repeat(tower10, 15)));
    CHECK(find_joints(tower, 15, outer_lines(tower10), settings(0.2)).size() == 1);
}

TEST_CASE("A loop that only partly qualifies stays one line across its seam", "[WallExtension][Joints]")
{
    // The tower is flush with the box's left side, so only its other three sides stand on the box's top surface.
    const ExPolygons flush_tower { rect(0., 5., 10., 15.) };
    const auto layers = stack(concat(repeat(box20, 15), repeat(flush_tower, 15)));
    // The loop starts at a corner inside the qualifying part, as a wall's seam often does.
    Polyline loop;
    loop.points = { Point::new_scale(9.775, 5.225), Point::new_scale(9.775, 14.775), Point::new_scale(0.225, 14.775),
                    Point::new_scale(0.225, 5.225), Point::new_scale(9.775, 5.225) };
    const auto joints = find_joints(layers, 15, Polylines{ loop }, settings());
    REQUIRE(joints.size() == 1);
    CHECK(joints.front().lines.size() == 1);
}

TEST_CASE("A one-line fin gets its extension and both chamfers as one area", "[WallExtension][Joints]")
{
    // A 0.6 mm fin printed as one line along its middle.
    const ExPolygons fin { rect(9.7, 5., 10.3, 15.) };
    const auto layers = stack(concat(repeat(box20, 30), repeat(fin, 15)));
    Polyline middle;
    middle.points = { Point::new_scale(10., 5.3), Point::new_scale(10., 14.7) };
    const auto joints = find_joints(layers, 30, Polylines{ middle }, settings());
    REQUIRE(joints.size() == 1);
    // Layer 25 is just under the shell: a 1 mm chamfer on each side of the 0.45 mm line, with nothing between them.
    const ExPolygons area = extension_area(layers, joints, 25).all;
    CHECK(area.size() == 1);
    const double across = [&] {
        double a = 0.;
        for (const ExPolygon &e : intersection_ex(area, ExPolygons{ rect(5., 9.9, 15., 10.1) }))
            a += e.area();
        return a * SCALING_FACTOR * SCALING_FACTOR / 0.2;
    }();
    CHECK(across == Approx(line_w + 2. * 1.0).margin(0.03));
}

TEST_CASE("A wall line deep inside the outline is not an outer wall", "[WallExtension][Joints]")
{
    const auto layers = stack(concat(repeat(box20, 15), repeat(tower10, 15)));
    const Polylines deep = to_polylines(ExPolygons{ centred_square(10., 10., 2.) });
    CHECK(find_joints(layers, 15, deep, settings()).empty());
}

TEST_CASE("On a two-wall lower part a line standing on the inner wall is not a joint", "[WallExtension][Joints]")
{
    const ExPolygons offset_tower { rect(0.6, 5., 10.6, 15.) };
    const auto layers = stack(concat(repeat(box20, 15), repeat(offset_tower, 15)), 0.2, 2);
    const auto joints = find_joints(layers, 15, outer_lines(offset_tower), settings());
    REQUIRE(joints.size() == 1);
    // The left side stands on the box's inner wall and is dropped; the rest stands on the infill. A line beside the inner
    // wall may overlap it as neighbouring lines do.
    const ExPolygons near_box_walls =
        diff_ex(box20, offset_ex(box20, -scale_(2. * line_w + line_w / 2. - (line_w - line_spacing_w) - 0.012 - 0.012)));
    CHECK(intersection_pl(joints.front().lines, near_box_walls).empty());
    CHECK(length_mm(joints.front().lines) > 25.);
}

TEST_CASE("A wall line below stops a joint only where it overlaps more than neighbouring lines do", "[WallExtension][Joints]")
{
    // "Only one wall on top surfaces" leaves the box's top layer one wall at the box's edge and puts its second wall
    // around the area under the tower; `inset` is how far inside the tower's side that wall's footprint starts.
    const auto with_wall_under_tower = [](double inset) {
        auto layers = stack(concat(repeat(box20, 15), repeat(tower10, 15)));
        ExPolygons top_walls = layers[14].walls;
        append(top_walls, diff_ex(offset_ex(tower10, -scale_(inset)), offset_ex(tower10, -scale_(inset + line_w))));
        layers[14].walls = union_ex(top_walls);
        return layers;
    };
    // Beside the tower's outer wall, overlapping its footprint as neighbouring lines of one layer do: the joint stands.
    const auto beside = with_wall_under_tower(line_spacing_w);
    const auto joints = find_joints(beside, 15, outer_lines(tower10), settings());
    REQUIRE(joints.size() == 1);
    CHECK(joints.front().side == JointSide::Top);
    CHECK(length_mm(joints.front().lines) == Approx(4. * (10. - line_w)).margin(0.5));
    // The tower's own first-layer wall still carries the wall above it.
    CHECK(find_joints(beside, 16, outer_lines(tower10), settings()).empty());
    // Right under the outer wall: the wall carries on and there is no joint.
    CHECK(find_joints(with_wall_under_tower(0.), 15, outer_lines(tower10), settings()).empty());
}

TEST_CASE("Shell depth follows the shell settings in millimetres", "[WallExtension][Joints]")
{
    const auto uniform = stack_of_heights(std::vector<double>(20, 0.2));
    CHECK(top_shell_depth(uniform, 10, 4, 0.) == Approx(0.8));
    CHECK(top_shell_depth(uniform, 10, 4, 1.0) == Approx(1.0));
    CHECK(top_shell_depth(uniform, 10, 0, 0.) == Approx(0.));
    CHECK(bottom_shell_depth(uniform, 5, 4, 0.) == Approx(0.8));

    std::vector<double> heights(10, 0.2);
    heights.insert(heights.end(), 10, 0.1);
    const auto mixed = stack_of_heights(heights);
    CHECK(top_shell_depth(mixed, 15, 4, 0.) == Approx(0.4));
    CHECK(top_shell_depth(mixed, 15, 4, 0.6) == Approx(0.6));
    CHECK(bottom_shell_depth(mixed, 10, 4, 0.) == Approx(0.4));
}

namespace {
double area_mm2(const ExPolygons &expolygons)
{
    double area = 0.;
    for (const ExPolygon &expolygon : expolygons)
        area += expolygon.area();
    return area * SCALING_FACTOR * SCALING_FACTOR;
}

// Width in mm of `area` along a 0.2 mm tall strip crossing the right side (x = 15) of the 10 mm tower: exact on a
// straight side, unlike an area divided by a perimeter, which the corners distort.
double width_across_right_side(const ExPolygons &area)
{
    return area_mm2(intersection_ex(area, ExPolygons{ rect(13., 9.9, 18., 10.1) })) / 0.2;
}
}

TEST_CASE("Under a tower the extension goes through the shell and 1 mm below with a widening chamfer", "[WallExtension][Area]")
{
    const auto layers = stack(concat(repeat(box20, 30), repeat(tower10, 15)));
    const auto joints = find_joints(layers, 30, outer_lines(tower10), settings());
    REQUIRE(joints.size() == 1);
    REQUIRE(joints.front().shell_depth == Approx(0.8));
    for (size_t l = 0; l < layers.size(); ++l) {
        CAPTURE(l);
        const ExtensionArea a = extension_area(layers, joints, l);
        CHECK(a.all.empty() == (l < 21 || l > 29));
        CHECK(a.hanging.empty() == (l != 21));
        if (a.all.empty())
            continue;
        // The extension line lies inside the tower's outline, right under the wall; only the chamfer lies outside it.
        CHECK(area_mm2(intersection_ex(a.all, tower10)) == Approx(4. * (10. - line_w) * line_w).epsilon(0.05));
        const double chamfer = l >= 26 ? 0. : 0.2 * double(l - 20);
        CHECK(width_across_right_side(a.all) == Approx(line_w + chamfer).margin(0.02));
        if (l == 21)
            CHECK(area_mm2(a.hanging) == Approx(area_mm2(a.all)));
    }
}

TEST_CASE("Without the chamfer the extension stays one line wide", "[WallExtension][Area]")
{
    const auto layers = stack(concat(repeat(box20, 30), repeat(tower10, 15)));
    JointSettings without_chamfer = settings();
    without_chamfer.chamfer = false;
    const auto joints = find_joints(layers, 30, outer_lines(tower10), without_chamfer);
    REQUIRE(joints.size() == 1);
    for (size_t l = 0; l < layers.size(); ++l) {
        CAPTURE(l);
        const ExtensionArea a = extension_area(layers, joints, l);
        // The same reach as with the chamfer: through the shell and 1 mm below it, the lowest layer hanging.
        CHECK(a.all.empty() == (l < 21 || l > 29));
        CHECK(a.hanging.empty() == (l != 21));
        if (a.all.empty())
            continue;
        // Only the line, right under the wall, on every layer.
        CHECK(width_across_right_side(a.all) == Approx(line_w).margin(0.02));
        CHECK(area_mm2(diff_ex(a.all, tower10)) < 0.01);
    }
}

TEST_CASE("Over a column the extension rises through the bottom shell and 1 mm above", "[WallExtension][Area]")
{
    const auto layers = stack(concat(repeat(tower10, 30), repeat(box20, 30)));
    const auto joints = find_joints(layers, 29, outer_lines(tower10), settings());
    REQUIRE(joints.size() == 1);
    REQUIRE(joints.front().side == JointSide::Bottom);
    for (size_t l = 0; l < layers.size(); ++l) {
        CAPTURE(l);
        const ExtensionArea a = extension_area(layers, joints, l);
        // The area reaches layer 30, the bridge layer; apply() keeps it out of the bridge's anchor zone there.
        CHECK(a.all.empty() == (l < 30 || l > 38));
        CHECK(a.hanging.empty());
        if (a.all.empty())
            continue;
        // Narrows going up: 1.0 mm just above the shell (layer 34), 0.2 mm on the last layer (38).
        const double chamfer = l <= 33 ? 0. : 0.2 * double(39 - l);
        CHECK(width_across_right_side(a.all) == Approx(line_w + chamfer).margin(0.02));
    }
}

TEST_CASE("Overlapping extensions form one area", "[WallExtension][Area]")
{
    const ExPolygon  left = rect(6., 11., 14., 19.), right = rect(15., 11., 23., 19.);
    const ExPolygons both { left, right };
    const auto layers = stack(concat(repeat(ExPolygons{ rect(0., 0., 30., 30.) }, 30), repeat(both, 15)));
    const auto joints_left  = find_joints(layers, 30, outer_lines({ left }), settings());
    const auto joints_right = find_joints(layers, 30, outer_lines({ right }), settings());
    REQUIRE(joints_left.size() == 1);
    REQUIRE(joints_right.size() == 1);
    const std::vector<Joint> joints { joints_left.front(), joints_right.front() };
    // Layer 25 is just under the shell, where each chamfer is 1.0 mm wide; they meet across the 1.0 mm gap.
    const ExPolygons together = extension_area(layers, joints, 25).all;
    double sum = 0.;
    for (const ExPolygon &expolygon : together)
        sum += area_mm2({ expolygon });
    CHECK(sum == Approx(area_mm2(union_ex(together))));
    CHECK(area_mm2(together) < area_mm2(extension_area(layers, joints_left, 25).all) +
                               area_mm2(extension_area(layers, joints_right, 25).all) - 1.);
}
