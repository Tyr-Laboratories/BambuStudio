#include <catch2/catch.hpp>

#include "libslic3r/WallInternalExtension.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <algorithm>
#include <cmath>

using namespace Slic3r;
using namespace Slic3r::WallInternalExtension;

namespace {
constexpr double line_w = 0.45;

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
    s.tolerance            = scaled<coord_t>(0.012);
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
        { 0.,      5,  true  },   // exactly 1.0 mm tall
    }));
    CAPTURE(step, count, expected);
    std::vector<ExPolygons> outlines = repeat(box20, 15);
    for (int k = 0; k < count; ++k)
        outlines.push_back({ centred_square(10., 10., 5. + step * k) });
    const auto layers = stack(outlines);
    CHECK(find_joints(layers, 15, outer_lines(outlines[15]), settings()).empty() != expected);
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
    // The left side stands on the box's inner wall and is dropped; the rest stands on the infill.
    const ExPolygons near_box_walls = diff_ex(box20, offset_ex(box20, -scale_(2. * line_w + line_w / 2. - 0.012)));
    CHECK(intersection_pl(joints.front().lines, near_box_walls).empty());
    CHECK(length_mm(joints.front().lines) > 25.);
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
