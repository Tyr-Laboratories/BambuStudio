#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <cmath>
#include <limits>

using namespace Slic3r;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
ExPolygon rectangle(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon({Point::new_scale(x0, y0), Point::new_scale(x1, y0),
                              Point::new_scale(x1, y1), Point::new_scale(x0, y1)}));
}

Polyline path(std::initializer_list<Vec2d> points)
{
    return Polyline::new_scale(std::vector<Vec2d>(points));
}

ExPolygons frame(double side)
{
    return {rectangle(0., 0., side, .5), rectangle(0., side - .5, side, side),
            rectangle(0., 0., .5, side), rectangle(side - .5, 0., side, side)};
}
}

TEST_CASE("Bridge aware validation measures real two-ended air spans", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({},
        {rectangle(0., 0., 1., 2.), rectangle(4., 0., 5., 2.)},
        {path({{.5, 1.}, {4.5, 1.}})}, {rectangle(0., 0., 5., 2.)}, 3.1, .001);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs[0].kind == Validation::UnsupportedRunKind::TwoEndedBridge);
    CHECK(result.unsupported_runs[0].begin_mm == Approx(.5));
    CHECK(result.unsupported_runs[0].end_mm == Approx(3.5));
    CHECK(result.unsupported_runs[0].length_mm == Approx(3.));
    CHECK(result.max_unsupported_mm == Approx(3.));
    CHECK(result.passed());
}

TEST_CASE("Bridge aware validation never adds tolerance to the requested gap", "[BridgeAwareCubic][Validation]")
{
    const double gap = GENERATE(2.999, 3., 3.0005);
    const auto result = Validation::validate({},
        {rectangle(0., 0., 1., 2.), rectangle(4., 0., 5., 2.)},
        {path({{.5, 1.}, {4.5, 1.}})}, {rectangle(0., 0., 5., 2.)}, gap, .001);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs[0].exceeds_gap);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation reports short one-sided paths separately", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({}, {rectangle(0., 0., 1., 2.)},
        {path({{.5, 1.}, {2., 1.}})}, {rectangle(1., 0., 2.1, 2.)}, 5., .001);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs[0].kind == Validation::UnsupportedRunKind::OneSidedOverhang);
    CHECK_FALSE(result.unsupported_runs[0].exceeds_gap);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation rejects a tiny wholly floating island", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({}, {}, {path({{.01, .01}, {.02, .01}})},
        {rectangle(0., 0., .03, .03)}, 5., .001);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs[0].kind == Validation::UnsupportedRunKind::Floating);
    CHECK(result.unsupported_runs[0].length_mm == Approx(.01));
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation does not accept point or boundary contact as anchors", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({}, {rectangle(0., 0., 1., 1.)},
        {path({{-1., 1.}, {2., 1.}})}, {rectangle(-1., .5, 2., 1.5)}, 5., .001);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs[0].kind == Validation::UnsupportedRunKind::Floating);
    CHECK(result.unsupported_runs[0].length_mm == Approx(3.));
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation uses arclength through a bent path", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({},
        {rectangle(0., 0., 1., 4.), rectangle(4., 0., 5., 4.)},
        {path({{.5, 1.}, {2., 1.}, {2., 3.}, {4.5, 3.}})},
        {rectangle(1.9, 1.9, 2.1, 2.1)}, 4.5, .001);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs[0].kind == Validation::UnsupportedRunKind::TwoEndedBridge);
    CHECK(result.unsupported_runs[0].length_mm == Approx(5.));
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation retains full runs across demand boundaries", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({},
        {rectangle(0., 0., 1., 2.), rectangle(9., 0., 10., 2.)},
        {path({{.5, 1.}, {9.5, 1.}})}, {rectangle(2.4, .9, 2.6, 1.1)}, 3., .001);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs[0].begin_mm == Approx(.5));
    CHECK(result.unsupported_runs[0].end_mm == Approx(8.5));
    CHECK(result.unsupported_runs[0].length_mm == Approx(8.));
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation includes a run touching only the demand boundary", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({}, {}, {path({{0., 0.}, {1., 1.}, {2., 0.}})},
        {rectangle(.9, 1., 1.1, 2.)}, 5., .001);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs[0].length_mm == Approx(std::sqrt(8.)));
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation ignores unsupported paths outside demand", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({}, {rectangle(0., 0., 5., 2.)},
        {path({{.5, 1.}, {4.5, 1.}}), path({{0., 3.}, {100., 3.}})},
        {rectangle(0., 0., 5., 2.)}, 1., .001);
    CHECK(result.unsupported_runs.empty());
    CHECK(result.max_unsupported_mm == 0.);
    CHECK(result.passed());
}

TEST_CASE("Bridge aware validation measures the entire selected cell", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({rectangle(0., 0., 5., 5.)}, frame(5.), {},
        {rectangle(.6, .6, .7, .7)}, 5., .001);
    REQUIRE(result.void_cells.size() == 1);
    CHECK(result.void_cells[0].closed);
    CHECK(result.void_cells[0].diameter_mm == Approx(std::sqrt(32.)));
    CHECK(result.void_cells[0].exceeds_gap);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation joins voids across a missing partition", "[BridgeAwareCubic][Validation]")
{
    ExPolygons support = {rectangle(0., 0., 10., .25), rectangle(0., 3.75, 10., 4.),
                          rectangle(0., 0., .25, 4.), rectangle(9.75, 0., 10., 4.)};
    const auto missing = Validation::validate({rectangle(0., 0., 10., 4.)}, support, {},
        {rectangle(1., 1., 2., 2.)}, 6., .001);
    REQUIRE(missing.void_cells.size() == 1);
    CHECK(missing.void_cells[0].diameter_mm == Approx(std::sqrt(102.5)));
    CHECK_FALSE(missing.passed());
    support.push_back(rectangle(4.75, .25, 5.25, 3.75));
    const auto intact = Validation::validate({rectangle(0., 0., 10., 4.)}, support, {},
        {rectangle(1., 1., 2., 2.)}, 6., .001);
    REQUIRE(intact.void_cells.size() == 1);
    CHECK(intact.void_cells[0].diameter_mm == Approx(std::sqrt(32.5)));
    CHECK(intact.void_cells[0].closed);
    CHECK(intact.passed());
}

TEST_CASE("Bridge aware validation identifies open model boundaries", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({rectangle(0., 0., 2., 2.)}, {}, {},
        {rectangle(.1, .1, .2, .2)}, 5., .001);
    REQUIRE(result.void_cells.size() == 1);
    CHECK_FALSE(result.void_cells[0].closed);
    CHECK_FALSE(result.void_cells[0].exceeds_gap);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation does not fill or measure an intentional cavity", "[BridgeAwareCubic][Validation]")
{
    auto domain = rectangle(0., 0., 5., 5.);
    auto hole = rectangle(1., 1., 4., 4.).contour;
    hole.make_clockwise();
    domain.holes.push_back(hole);
    const auto result = Validation::validate({domain}, {}, {}, {rectangle(2., 2., 3., 3.)}, 1., .001);
    CHECK(result.void_cells.empty());
    CHECK(result.passed());
}

TEST_CASE("Bridge aware validation rejects invalid numeric limits", "[BridgeAwareCubic][Validation]")
{
    const double gap = GENERATE(0., -1., std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::quiet_NaN(), 1.e30);
    const auto result = Validation::validate({}, {}, {}, {}, gap, .001);
    CHECK_FALSE(result.input_valid);
    CHECK_FALSE(result.passed());
    CHECK_FALSE(result.failure_reason.empty());
}

TEST_CASE("Bridge aware validation rejects invalid tolerances", "[BridgeAwareCubic][Validation]")
{
    const double tolerance = GENERATE(-.001, 5., std::numeric_limits<double>::infinity(),
                                      std::numeric_limits<double>::quiet_NaN());
    const auto result = Validation::validate({}, {}, {}, {}, 5., tolerance);
    CHECK_FALSE(result.input_valid);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation keeps revisited positions distinct in path order", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({},
        {rectangle(0., 0., 1., 2.), rectangle(4., 0., 5., 2.)},
        {path({{.5, 1.}, {4.5, 1.}, {.5, 1.}})}, {rectangle(2., .9, 2.1, 1.1)}, 3.1, .001);
    REQUIRE(result.unsupported_runs.size() == 2);
    CHECK(result.unsupported_runs[0].begin_mm == Approx(.5));
    CHECK(result.unsupported_runs[1].begin_mm == Approx(4.5));
    CHECK(result.unsupported_runs[0].length_mm == Approx(3.));
    CHECK(result.unsupported_runs[1].length_mm == Approx(3.));
    CHECK(result.passed());
}

TEST_CASE("Bridge aware validation selects only demanded runs on a shared path", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({},
        {rectangle(0., 0., 1., 2.), rectangle(3., 0., 4., 2.), rectangle(7., 0., 8., 2.)},
        {path({{.5, 1.}, {7.5, 1.}})}, {rectangle(5., .9, 5.1, 1.1)}, 3.1, .001);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs[0].begin_mm == Approx(3.5));
    CHECK(result.unsupported_runs[0].length_mm == Approx(3.));
    CHECK(result.passed());
}

TEST_CASE("Bridge aware validation removes artificial support polygon seams", "[BridgeAwareCubic][Validation]")
{
    const double overlap = GENERATE(0., .5);
    const auto result = Validation::validate({},
        {rectangle(0., 0., 2., 2.), rectangle(2. - overlap, 0., 4., 2.)},
        {path({{2., .5}, {2., 1.5}})}, {rectangle(1.9, 0., 2.1, 2.)}, 1., .001);
    CHECK(result.unsupported_runs.empty());
    CHECK(result.passed());
}

TEST_CASE("Bridge aware validation rejects a narrow physical partition opening", "[BridgeAwareCubic][Validation]")
{
    ExPolygons support = {rectangle(0., 0., 10., .25), rectangle(0., 3.75, 10., 4.),
                          rectangle(0., 0., .25, 4.), rectangle(9.75, 0., 10., 4.),
                          rectangle(4.75, .25, 5.25, 1.99), rectangle(4.75, 2.01, 5.25, 3.75)};
    const auto result = Validation::validate({rectangle(0., 0., 10., 4.)}, support, {},
        {rectangle(1., 1., 2., 2.)}, 6., .001);
    REQUIRE(result.void_cells.size() == 1);
    CHECK(result.void_cells[0].diameter_mm == Approx(std::sqrt(102.5)));
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation detects a void reaching a cavity boundary", "[BridgeAwareCubic][Validation]")
{
    auto domain = rectangle(0., 0., 6., 6.);
    auto hole = rectangle(2.5, 2.5, 3.5, 3.5).contour;
    hole.make_clockwise();
    domain.holes.push_back(hole);
    const auto result = Validation::validate({domain}, frame(6.), {}, {rectangle(1., 1., 1.1, 1.1)}, 10., .001);
    REQUIRE(result.void_cells.size() == 1);
    CHECK_FALSE(result.void_cells[0].closed);
    CHECK_FALSE(result.void_cells[0].exceeds_gap);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation tolerates repeated points without inventing gaps", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({},
        {rectangle(0., 0., 1., 2.), rectangle(4., 0., 5., 2.)},
        {path({{.5, 1.}, {.5, 1.}, {2., 1.}, {2., 1.}, {4.5, 1.}})},
        {rectangle(0., 0., 5., 2.)}, 3.1, .001);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs[0].length_mm == Approx(3.));
    CHECK(result.passed());
}

TEST_CASE("Bridge aware validation rejects fitted arc chords", "[BridgeAwareCubic][Validation]")
{
    auto curved = path({{0., 0.}, {1., 0.}});
    PathFittingData arc{};
    arc.start_point_index = 0;
    arc.end_point_index = 1;
    arc.path_type = EMovePathType::Arc_move_ccw;
    curved.fitting_result.push_back(arc);
    const auto result = Validation::validate({}, {}, {curved}, {rectangle(-1., -1., 2., 2.)}, 5., .001);
    CHECK_FALSE(result.input_valid);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation has no demanded work for an empty mask", "[BridgeAwareCubic][Validation]")
{
    const auto result = Validation::validate({rectangle(0., 0., 5., 5.)}, {},
        {path({{0., 0.}, {5., 5.}})}, {}, 1., .001);
    CHECK(result.unsupported_runs.empty());
    CHECK(result.void_cells.empty());
    CHECK(result.passed());
}

TEST_CASE("Bridge aware validation guards coordinate and input allocation bounds", "[BridgeAwareCubic][Validation]")
{
    SECTION("Unsafe coordinate") {
        Polyline oversized({Point(0, 0), Point(std::numeric_limits<coord_t>::max(), 0)});
        const auto result = Validation::validate({}, {}, {oversized}, {}, 5., .001);
        CHECK_FALSE(result.input_valid);
        CHECK_FALSE(result.passed());
    }
    SECTION("Too many vertices") {
        Polyline oversized(Points(200001, Point(0, 0)));
        const auto result = Validation::validate({}, {}, {oversized}, {}, 5., .001);
        CHECK_FALSE(result.input_valid);
        CHECK_FALSE(result.passed());
    }
}

TEST_CASE("Bridge aware validation reports work exhaustion as incomplete", "[BridgeAwareCubic][Validation]")
{
    ExPolygons support;
    for (int i = 0; i < 400; ++i)
        support.push_back(rectangle(2. * i, 0., 2. * i + 1., 2.));
    const Polylines paths(1000, path({{.5, 1.}, {798.5, 1.}}));
    const auto result = Validation::validate({}, support, paths, {rectangle(0., 0., 800., 2.)}, 1000., .001);
    CHECK(result.input_valid);
    CHECK_FALSE(result.completed);
    CHECK_FALSE(result.failure_reason.empty());
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware validation retains subresolution tails in local arclength", "[BridgeAwareCubic][Validation]")
{
    constexpr coord_t extent = 500000000;
    const ExPolygon support(Polygon({Point(0, 0), Point(0, -extent), Point(extent, -extent), Point(extent, extent - 1)}));
    Polyline long_path;
    for (size_t i = 0; i <= 1000; ++i)
        long_path.points.emplace_back(i % 2 ? extent - 1 : 1, -1);
    long_path.points.push_back(long_path.points.back());
    long_path.points.emplace_back(1, 1);
    long_path.points.push_back(long_path.points.back());
    if (GENERATE(false, true))
        long_path.reverse();
    const ExPolygon demand(Polygon({Point(0, 0), Point(2, 0), Point(2, 2), Point(0, 2)}));
    const auto result = Validation::validate({}, {support}, {long_path}, {demand}, 5., .001);
    CHECK(result.input_valid);
    CHECK(result.completed);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::OneSidedOverhang);
    CHECK(result.unsupported_runs.front().length_mm > 0.);
    CHECK(result.unsupported_runs.front().length_mm == Approx(2.e-14).epsilon(.001));
    CHECK_FALSE(result.passed());
    CHECK_FALSE(result.failure_reason.empty());
}

TEST_CASE("Bridge aware validation accepts a real bridge across a subresolution clipping sliver", "[BridgeAwareCubic][Validation]")
{
    constexpr coord_t extent = 1000000;
    ExPolygon support(Polygon({Point(-100, -100), Point(100, -100), Point(100, extent + 1), Point(-100, extent + 1)}));
    Polygon notch({Point(0, 0), Point(2, extent), Point(1, extent)});
    notch.make_clockwise();
    support.holes.push_back(notch);
    const Polyline upper({Point(-50, 1), Point(50, 1)});
    const ExPolygon demand(Polygon({Point(-60, 0), Point(60, 0), Point(60, 2), Point(-60, 2)}));
    const auto result = Validation::validate({}, {support}, {upper}, {demand}, 5., .001);
    CHECK(result.completed);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::TwoEndedBridge);
    CHECK(result.unsupported_runs.front().length_mm == Approx(1.e-11).epsilon(.001));
    CHECK(result.passed());
}

TEST_CASE("Bridge aware validation bounds boolean intersections before allocating cells", "[BridgeAwareCubic][Validation]")
{
    ExPolygons support;
    for (int i = 0; i < 2000; ++i) {
        support.push_back(rectangle(i * .02, 0., i * .02 + .01, 40.));
        support.push_back(rectangle(0., i * .02, 40., i * .02 + .01));
    }
    const auto result = Validation::validate({rectangle(0., 0., 40., 40.)}, support, {},
        {rectangle(.001, .001, .002, .002)}, 5., .001);
    CHECK(result.input_valid);
    CHECK_FALSE(result.completed);
    CHECK_FALSE(result.passed());
    CHECK_FALSE(result.failure_reason.empty());
}

TEST_CASE("Bridge aware validation does not analyze uncertain support outside demand", "[BridgeAwareCubic][Validation]")
{
    constexpr coord_t extent = 500000000;
    const ExPolygon support(Polygon({Point(0, 0), Point(0, -extent), Point(extent, -extent), Point(extent, extent - 1)}));
    const Polyline unrelated({Point(1, -1), Point(1, 1)});
    const auto result = Validation::validate({}, {support}, {unrelated},
        {rectangle(10., 10., 11., 11.)}, 5., .001);
    CHECK(result.unsupported_runs.empty());
    CHECK(result.completed);
    CHECK(result.passed());
}
