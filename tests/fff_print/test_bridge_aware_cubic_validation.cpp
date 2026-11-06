#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <cmath>
#include <limits>

using namespace Slic3r;
namespace Contact = Slic3r::BridgeAwareCubicContact;
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
    Contact::PathContact contact;
    contact.segments.push_back({0, support, {}});
    const auto validate = [&](Validation::WorkBudget &budget) {
        return Validation::validate_contacts({}, {}, {path({{.5, 1.}, {798.5, 1.}})},
            {rectangle(0., 0., 800., 2.)}, 1000., .001, {contact}, &budget);
    };
    // Use an explicitly insufficient allowance for real crossing queries,
    // rather than requiring an inefficient implementation to spend 50M work.
    // Calibration keeps the guard meaningful as exact lookups improve.
    Validation::WorkBudget probe;
    const size_t initial_work = probe.remaining_work;
    const auto complete = validate(probe);
    REQUIRE(complete.passed());
    REQUIRE(complete.unsupported_runs.size() == 399);
    const size_t required_work = initial_work - probe.remaining_work;
    REQUIRE(required_work > 1);
    Validation::WorkBudget limited{required_work / 2};
    const auto result = validate(limited);
    CHECK(result.input_valid);
    CHECK_FALSE(result.completed);
    CHECK(result.resource_limited);
    CHECK(limited.remaining_work == 0);
    CHECK(result.failure_reason.find("aggregate geometry work limit") != std::string::npos);
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

TEST_CASE("Bridge aware prepared geometry selects complete cells for each demand", "[BridgeAwareCubic][Validation]")
{
    const ExPolygons domain{rectangle(0., 0., 12., 4.)};
    const ExPolygons support{rectangle(0., 0., 12., .25), rectangle(0., 3.75, 12., 4.),
                            rectangle(0., 0., .25, 4.), rectangle(11.75, 0., 12., 4.),
                            rectangle(4.75, .25, 5.25, 3.75)};
    const auto prepared = Validation::prepare_geometry(domain, support);
    REQUIRE(prepared);

    const auto left = Validation::validate_prepared_contacts(*prepared, {},
        {rectangle(1., 1., 1.1, 1.1)}, 8., .001, {});
    REQUIRE(left.input_valid);
    REQUIRE(left.completed);
    REQUIRE(left.void_cells.size() == 1);
    CHECK(left.void_cells.front().closed);
    CHECK(left.void_cells.front().diameter_mm == Approx(std::sqrt(32.5)));
    CHECK(left.max_void_diameter_mm == Approx(std::sqrt(32.5)));
    CHECK(left.passed());

    const auto right = Validation::validate_prepared_contacts(*prepared, {},
        {rectangle(9., 1., 9.1, 1.1)}, 8., .001, {});
    REQUIRE(right.input_valid);
    REQUIRE(right.completed);
    REQUIRE(right.void_cells.size() == 1);
    CHECK(right.void_cells.front().closed);
    CHECK(right.void_cells.front().diameter_mm == Approx(std::sqrt(54.5)));
    CHECK(right.max_void_diameter_mm == Approx(std::sqrt(54.5)));
    CHECK(right.passed());
}

TEST_CASE("Bridge aware prepared geometry applies each call's gap", "[BridgeAwareCubic][Validation]")
{
    const auto prepared = Validation::prepare_geometry({rectangle(0., 0., 5., 5.)}, frame(5.));
    REQUIRE(prepared);
    const ExPolygons demand{rectangle(.6, .6, .7, .7)};

    const auto loose = Validation::validate_prepared_contacts(*prepared, {}, demand, 6., .001, {});
    REQUIRE(loose.completed);
    REQUIRE(loose.void_cells.size() == 1);
    CHECK(loose.passed());

    const auto tight = Validation::validate_prepared_contacts(*prepared, {}, demand, 5., .001, {});
    REQUIRE(tight.completed);
    REQUIRE(tight.void_cells.size() == 1);
    CHECK(tight.void_cells.front().diameter_mm == Approx(std::sqrt(32.)));
    CHECK(tight.void_cells.front().exceeds_gap);
    CHECK_FALSE(tight.passed());

    const auto loose_again = Validation::validate_prepared_contacts(*prepared, {}, demand, 6., .001, {});
    CHECK(loose_again.completed);
    CHECK(loose_again.passed());
}

TEST_CASE("Bridge aware contact validation shares work limits across calls", "[BridgeAwareCubic][Validation]")
{
    const bool use_prepared = GENERATE(false, true);
    const ExPolygons domain{rectangle(0., 0., 5., 5.)};
    const ExPolygons support = frame(5.);
    const ExPolygons demand{rectangle(.6, .6, .7, .7)};
    const auto prepared = Validation::prepare_geometry(domain, support);
    REQUIRE(prepared);
    const auto validate = [&](Validation::WorkBudget &budget) {
        if (use_prepared)
            return Validation::validate_prepared_contacts(*prepared, {}, demand, 6., .001, {}, &budget);
        return Validation::validate_contacts(domain, support, {}, demand, 6., .001, {}, &budget);
    };

    // Calibrate one successful call without tying the regression to a private
    // operation count. The shared allowance covers one call but not two.
    Validation::WorkBudget probe;
    const size_t initial_work = probe.remaining_work;
    REQUIRE(validate(probe).passed());
    const size_t one_call = initial_work - probe.remaining_work;
    REQUIRE(one_call > 0);
    Validation::WorkBudget shared{one_call + one_call / 2};
    const auto first = validate(shared);
    REQUIRE(first.completed);
    REQUIRE(first.passed());
    CHECK(shared.remaining_work < one_call);

    const auto second = validate(shared);
    CHECK(second.input_valid);
    CHECK_FALSE(second.completed);
    CHECK(second.resource_limited);
    CHECK(shared.remaining_work == 0);
    CHECK_FALSE(second.passed());
    CHECK(second.failure_reason.find("work") != std::string::npos);
    CHECK(second.failure_reason.find("limit") != std::string::npos);
}

TEST_CASE("Bridge aware contact validation skips unused raw support unions without a domain", "[BridgeAwareCubic][Validation]")
{
    ExPolygons raw_support;
    for (int i = 0; i < 2000; ++i) {
        raw_support.push_back(rectangle(i * .02, 0., i * .02 + .01, 40.));
        raw_support.push_back(rectangle(0., i * .02, 40., i * .02 + .01));
    }
    const Polylines upper{path({{0., 0.}, {10., 0.}})};
    Contact::PathContact contact;
    contact.segments.push_back({0, {rectangle(-1., -1., 11., 1.)}, {}});
    const auto result = Validation::validate_contacts({}, raw_support, upper,
        {rectangle(-1., -1., 11., 1.)}, 1., .001, {contact});
    CHECK(result.input_valid);
    CHECK(result.completed);
    CHECK(result.unsupported_runs.empty());
    CHECK(result.void_cells.empty());
    CHECK(result.passed());
}

TEST_CASE("Bridge aware contact validation still validates unused raw support geometry", "[BridgeAwareCubic][Validation]")
{
    const Polylines upper{path({{0., 0.}, {10., 0.}})};
    Contact::PathContact contact;
    contact.segments.push_back({0, {rectangle(-1., -1., 11., 1.)}, {}});
    ExPolygons invalid;
    SECTION("Invalid contour") {
        invalid.emplace_back(Polygon({Point(0, 0), Point(1, 0)}));
    }
    SECTION("Unsafe coordinate") {
        invalid.emplace_back(Polygon({Point(0, 0), Point(std::numeric_limits<coord_t>::max(), 0), Point(0, 1)}));
    }
    const auto result = Validation::validate_contacts({}, invalid, upper,
        {rectangle(-1., -1., 11., 1.)}, 1., .001, {contact});
    CHECK_FALSE(result.input_valid);
    CHECK_FALSE(result.passed());
    CHECK_FALSE(result.failure_reason.empty());
}

TEST_CASE("Bridge aware contact validation bounds disjoint-hole classification work", "[BridgeAwareCubic][Validation]")
{
    auto mask = rectangle(0., 0., 40., 40.);
    const double pi = std::acos(-1.);
    for (size_t i = 0; i < 100; ++i) {
        const double x = 10. + 3. * double(i % 10);
        const double y = 10. + 3. * double(i / 10);
        Polygon hole;
        for (size_t vertex = 0; vertex < 32; ++vertex) {
            const double angle = 2. * pi * double(vertex) / 32.;
            hole.points.emplace_back(coord_t(std::llround((x + .75 * std::cos(angle)) / SCALING_FACTOR)),
                                     coord_t(std::llround((y + .75 * std::sin(angle)) / SCALING_FACTOR)));
        }
        hole.make_clockwise();
        mask.holes.push_back(std::move(hole));
    }
    Contact::PathContact contact;
    contact.segments.push_back({0, {mask}, {}});

    // The 3,204 mask vertices require input validation and one boundary scan.
    // This allowance also covers hole bounds, but not another full scan of
    // every remote hole merely to classify the supported path's midpoint.
    Validation::WorkBudget budget{8000};
    const auto supported = Validation::validate_contacts({}, {}, {path({{1., 1.}, {2., 1.}})},
        {rectangle(.5, .5, 2.5, 1.5)}, 1., .001, {contact}, &budget);
    REQUIRE(supported.input_valid);
    REQUIRE(supported.completed);
    CHECK_FALSE(supported.resource_limited);
    CHECK(supported.unsupported_runs.empty());
    CHECK(supported.passed());
    CHECK(budget.remaining_work > 0);
    CHECK(budget.remaining_work < 8000);

    // The first hole has a horizontal diameter of 1.5 mm. Its containing
    // bounds must still trigger the exact hole test rather than grant support.
    const Polyline crossing_path(Points{Point(900000, 1000000), Point(1100000, 1000000)});
    const auto crossing = Validation::validate_contacts({}, {}, {crossing_path},
        {rectangle(8.5, 9.5, 11.5, 10.5)}, 1., .001, {contact});
    REQUIRE(crossing.input_valid);
    REQUIRE(crossing.completed);
    REQUIRE(crossing.unsupported_runs.size() == 1);
    CHECK(crossing.unsupported_runs.front().kind == Validation::UnsupportedRunKind::TwoEndedBridge);
    CHECK(crossing.unsupported_runs.front().length_mm == Approx(1.5));
    CHECK(crossing.unsupported_runs.front().exceeds_gap);
    CHECK_FALSE(crossing.passed());
}

TEST_CASE("Bridge aware indexed rings preserve boundary and vertex predicates", "[BridgeAwareCubic][Validation]")
{
    const auto subdivided_rectangle = [](coord_t low, coord_t high) {
        Polygon ring;
        const coord_t step = (high - low) / 32;
        for (coord_t i = 0; i < 32; ++i) ring.points.emplace_back(low + i * step, low);
        for (coord_t i = 0; i < 32; ++i) ring.points.emplace_back(high, low + i * step);
        for (coord_t i = 0; i < 32; ++i) ring.points.emplace_back(high - i * step, high);
        for (coord_t i = 0; i < 32; ++i) ring.points.emplace_back(low, high - i * step);
        return ring;
    };
    ExPolygon indexed(subdivided_rectangle(0, 1000000));
    ExPolygon direct(Polygon(Points{Point(0, 0), Point(1000000, 0), Point(1000000, 1000000), Point(0, 1000000)}));
    const bool with_hole = GENERATE(false, true);
    if (with_hole) {
        Polygon indexed_hole = subdivided_rectangle(200000, 800000);
        indexed_hole.make_clockwise();
        indexed.holes.push_back(std::move(indexed_hole));
        Polygon direct_hole(Points{Point(200000, 200000), Point(800000, 200000),
                                  Point(800000, 800000), Point(200000, 800000)});
        direct_hole.make_clockwise();
        direct.holes.push_back(std::move(direct_hole));
    }
    const Polylines paths{
        Polyline(Points{Point(100000, 500000), Point(900000, 500000)}), // Inside / crossing the hole.
        Polyline(Points{Point(-200000, 500000), Point(-100000, 500000)}), // Outside.
        Polyline(Points{Point(100000, 0), Point(900000, 0)}), // Horizontal outer boundary.
        Polyline(Points{Point(-100000, 100000), Point(100000, -100000)}), // Touches one vertex only.
        Polyline(Points{Point(500000, -100000), Point(500000, 1100000)}), // Crosses horizontal edges.
        Polyline(Points{Point(-100000, -100000), Point(1100000, 1100000)}), // Crosses exact vertices.
        Polyline(Points{Point(300000, 200000), Point(700000, 200000)}), // Horizontal hole boundary.
        Polyline(Points{Point(0, 0), Point(1000000, 0)}) // Endpoint vertices and a horizontal edge.
    };
    for (size_t index = 0; index < paths.size(); ++index) {
        INFO("path " << index << ", with hole " << with_hole);
        Contact::PathContact indexed_contact, direct_contact;
        indexed_contact.segments.push_back({0, {indexed}, {}});
        direct_contact.segments.push_back({0, {direct}, {}});
        const ExPolygons demand{rectangle(-3., -3., 13., 13.)};
        const auto expected = Validation::validate_contacts({}, {}, {paths[index]}, demand, 2., .001, {direct_contact});
        const auto actual = Validation::validate_contacts({}, {}, {paths[index]}, demand, 2., .001, {indexed_contact});
        REQUIRE(expected.input_valid);
        REQUIRE(expected.completed);
        REQUIRE(actual.input_valid);
        REQUIRE(actual.completed);
        CHECK(actual.passed() == expected.passed());
        CHECK(actual.max_unsupported_mm == Approx(expected.max_unsupported_mm));
        REQUIRE(actual.unsupported_runs.size() == expected.unsupported_runs.size());
        for (size_t run = 0; run < expected.unsupported_runs.size(); ++run) {
            CHECK(actual.unsupported_runs[run].kind == expected.unsupported_runs[run].kind);
            CHECK(actual.unsupported_runs[run].begin_mm == Approx(expected.unsupported_runs[run].begin_mm));
            CHECK(actual.unsupported_runs[run].end_mm == Approx(expected.unsupported_runs[run].end_mm));
            CHECK(actual.unsupported_runs[run].length_mm == Approx(expected.unsupported_runs[run].length_mm));
            CHECK(actual.unsupported_runs[run].exceeds_gap == expected.unsupported_runs[run].exceeds_gap);
        }
        // A true polygon edge or isolated vertex never becomes a physical
        // anchor merely because the candidate edge order changed in the tree.
        if (index == 2 || index == 3 || index == 7 || (with_hole && index == 6)) {
            REQUIRE(actual.unsupported_runs.size() == 1);
            CHECK(actual.unsupported_runs.front().kind == Validation::UnsupportedRunKind::Floating);
        }
    }
}

TEST_CASE("Bridge aware indexed demand bounds repeated long-zigzag classification", "[BridgeAwareCubic][Validation]")
{
    // Non-collinear shallow teeth survive union normalization. All path points
    // are well inside, so scanning this entire ring for every midpoint and
    // endpoint is redundant; its Y index can discard the remote teeth exactly.
    Polygon ring;
    for (coord_t i = 0; i <= 1000; ++i)
        ring.points.emplace_back(i * 10000, (i % 2) * 1000);
    for (coord_t i = 0; i <= 1000; ++i)
        ring.points.emplace_back((1000 - i) * 10000, 1000000 - (i % 2) * 1000);
    Polyline upper;
    Contact::PathContact contact;
    const ExPolygon support = rectangle(0., 1., 100., 9.);
    for (size_t i = 0; i <= 200; ++i) {
        upper.points.emplace_back(i % 2 == 0 ? 100000 : 9900000, i % 2 == 0 ? 200000 : 800000);
        if (i < 200)
            contact.segments.push_back({i, {support}, {}});
    }
    // One million units cover index construction, all boundary scans, and
    // actual candidate visits. Repeated full-ring classifications exceed it.
    Validation::WorkBudget budget{1000000};
    const auto result = Validation::validate_contacts({}, {}, {upper}, {ExPolygon(std::move(ring))},
        1., .001, {contact}, &budget);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    CHECK_FALSE(result.resource_limited);
    CHECK(result.unsupported_runs.empty());
    CHECK(result.passed());
    CHECK(budget.remaining_work > 0);
}

TEST_CASE("Bridge aware indexed polygon sets bound disconnected-demand queries", "[BridgeAwareCubic][Validation]")
{
    ExPolygons demand;
    for (size_t i = 0; i < 4096; ++i)
        demand.push_back(rectangle(.5 * i, -.1, .5 * i + .2, .1));
    const Polyline upper = path({{.05, 0.}, {2047.7, 0.}});
    Contact::PathContact contact;
    contact.segments.push_back({0, {}, {}});
    // Every rectangle is genuinely crossed. The exact event list remains
    // complete, but each midpoint needs only its nearby polygon candidates.
    // A repeated scan of all 4,096 contour boxes exhausts this allowance.
    Validation::WorkBudget budget{2000000};
    const auto result = Validation::validate_contacts({}, {}, {upper}, demand, 5., 0., {contact}, &budget);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    CHECK_FALSE(result.resource_limited);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::Floating);
    CHECK(result.unsupported_runs.front().length_mm == Approx(2047.65));
    CHECK(result.unsupported_runs.front().exceeds_gap);
    CHECK_FALSE(result.passed());
    CHECK(budget.remaining_work > 0);
}

TEST_CASE("Bridge aware indexed cell selection skips disjoint demand polygons", "[BridgeAwareCubic][Validation]")
{
    ExPolygons domain, support;
    for (size_t i = 0; i < 100; ++i) {
        const double x = 4. * i;
        domain.push_back(rectangle(x, 0., x + 2., 2.));
        auto walls = frame(2.);
        for (auto &wall : walls) wall.translate(scale_(x), 0);
        append(support, std::move(walls));
    }
    const auto prepared = Validation::prepare_geometry(domain, support);
    ExPolygons selection{rectangle(.75, .75, 1.25, 1.25)};
    for (size_t i = 0; i < 3000; ++i)
        selection.push_back(rectangle(.5 * i, 10., .5 * i + .2, 10.2));
    // The cached cells are unchanged. Remote demand must not consume a full
    // edge scan for each of their boundaries or create additional cells.
    Validation::WorkBudget budget{1000000};
    const auto result = Validation::validate_prepared_contacts(*prepared, {}, selection, 2., 0., {}, &budget);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    CHECK_FALSE(result.resource_limited);
    REQUIRE(result.void_cells.size() == 1);
    CHECK(result.void_cells.front().closed);
    CHECK(result.void_cells.front().diameter_mm == Approx(std::sqrt(2.)));
    CHECK(result.passed());
    CHECK(budget.remaining_work > 0);
}

TEST_CASE("Bridge aware polygon box index preserves exact boundary and hole selection", "[BridgeAwareCubic][Validation]")
{
    ExPolygons local{rectangle(0., 0., 10., 10.)};
    Polygon hole = rectangle(2., 2., 8., 8.).contour;
    hole.make_clockwise();
    local.front().holes.push_back(std::move(hole));
    ExPolygons indexed = local;
    for (size_t i = 0; i < 100; ++i)
        indexed.push_back(rectangle(20. + i, 20., 20.5 + i, 20.5));
    const Polylines paths{
        path({{1., 1.}, {9., 1.}}),
        path({{-1., 0.}, {11., 0.}}),
        path({{-1., 1.}, {1., -1.}}),
        path({{1., 5.}, {9., 5.}}),
        path({{3., 2.}, {7., 2.}}),
        path({{3., 3.}, {7., 3.}}),
        path({{-2., -2.}, {-1., -1.}})
    };
    const bool as_support = GENERATE(false, true);
    for (const Polyline &upper : paths) {
        Contact::PathContact direct_contact, indexed_contact;
        direct_contact.segments.push_back({0, as_support ? local : ExPolygons{}, {}});
        indexed_contact.segments.push_back({0, as_support ? indexed : ExPolygons{}, {}});
        const ExPolygons whole{rectangle(-3., -3., 13., 13.)};
        const auto expected = Validation::validate_contacts({}, {}, {upper}, as_support ? whole : local,
            3., 0., {direct_contact});
        const auto actual = Validation::validate_contacts({}, {}, {upper}, as_support ? whole : indexed,
            3., 0., {indexed_contact});
        REQUIRE(expected.completed);
        REQUIRE(actual.completed);
        REQUIRE(actual.input_valid);
        CHECK(actual.passed() == expected.passed());
        REQUIRE(actual.unsupported_runs.size() == expected.unsupported_runs.size());
        for (size_t i = 0; i < expected.unsupported_runs.size(); ++i) {
            CHECK(actual.unsupported_runs[i].kind == expected.unsupported_runs[i].kind);
            CHECK(actual.unsupported_runs[i].begin_mm == Approx(expected.unsupported_runs[i].begin_mm));
            CHECK(actual.unsupported_runs[i].end_mm == Approx(expected.unsupported_runs[i].end_mm));
            CHECK(actual.unsupported_runs[i].length_mm == Approx(expected.unsupported_runs[i].length_mm));
            CHECK(actual.unsupported_runs[i].exceeds_gap == expected.unsupported_runs[i].exceeds_gap);
        }
    }
}

TEST_CASE("Bridge aware polygon box index preserves overlapping-mask order", "[BridgeAwareCubic][Validation]")
{
    const bool boundary_first = GENERATE(false, true);
    ExPolygons masks{rectangle(0., 0., 10., 1.), rectangle(0., -1., 10., 1.)};
    if (!boundary_first) std::swap(masks[0], masks[1]);
    Contact::PathContact direct, indexed;
    direct.segments.push_back({0, masks, {}});
    for (size_t i = 0; i < 100; ++i)
        masks.push_back(rectangle(-100. + i, 20., -99.5 + i, 20.5));
    indexed.segments.push_back({0, masks, {}});
    const Polylines upper{path({{1., 0.}, {9., 0.}})};
    const ExPolygons demand{rectangle(0., -2., 10., 2.)};
    const auto expected = Validation::validate_contacts({}, {}, upper, demand, 9., 0., {direct});
    const auto actual = Validation::validate_contacts({}, {}, upper, demand, 9., 0., {indexed});
    REQUIRE(expected.completed);
    REQUIRE(actual.completed);
    CHECK(actual.passed() == expected.passed());
    REQUIRE(actual.unsupported_runs.size() == expected.unsupported_runs.size());
    CHECK(actual.max_unsupported_mm == Approx(expected.max_unsupported_mm));
}

TEST_CASE("Bridge aware prepared cell index preserves complete ordered cells and exact selection", "[BridgeAwareCubic][Validation]")
{
    ExPolygons domain{rectangle(0., 0., 10., 10.)};
    ExPolygons support = frame(10.);
    support.push_back(rectangle(4., 4., 6., 6.)); // A physical island creates a hole in the first void.
    for (size_t i = 0; i < 100; ++i) {
        const double x = 20. + 4. * double(i % 10), y = 20. + 4. * double(i / 10);
        // Scale locally and translate domain and walls identically. Scaling
        // x+side separately can differ by a coordinate unit from their sum,
        // creating real unintended boundary voids in this closed-cell fixture.
        const Point origin = Point::new_scale(x, y);
        auto box = rectangle(0., 0., 2., 2.);
        box.translate(origin);
        domain.push_back(std::move(box));
        auto walls = frame(2.);
        for (auto &wall : walls) wall.translate(origin);
        append(support, std::move(walls));
    }
    const auto prepared = Validation::prepare_geometry(domain, support);
    REQUIRE(prepared);
    auto surrounding = rectangle(-1., -1., 60., 60.);
    Polygon excluded = rectangle(-.25, -.25, 10.25, 10.25).contour;
    excluded.make_clockwise();
    surrounding.holes.push_back(std::move(excluded));
    const std::vector<ExPolygons> demands{
        {rectangle(1., 1., 1.1, 1.1)},                         // Demand wholly inside a larger cell.
        {rectangle(-1., -1., 11., 11.)},                       // Demand contains that entire cell.
        {rectangle(4.5, 4.5, 5.5, 5.5)},                       // Wholly inside the cell's physical island.
        {rectangle(4.5, 4., 5.5, 4.25)},                       // Touches the island boundary only.
        {rectangle(-1., 2., .5, 3.)},                          // Outer cell edge contact only.
        {rectangle(-1., -1., .5, .5)},                         // Outer cell vertex contact only.
        {rectangle(10., 10., 11., 11.)},                       // Disjoint despite nearby model bounds.
        {rectangle(1., 1., 1.1, 1.1), rectangle(20.75, 20.75, 21., 21.),
         rectangle(56.75, 56.75, 57., 57.)},                  // Separate, spatially distant candidates.
        {rectangle(-1., -1., 60., 60.)},                       // Every cell, in original output order.
        {surrounding}                                        // Demand hole excludes the first cell.
    };
    const size_t expected_counts[] = {1, 1, 0, 1, 1, 1, 0, 3, 101, 100};
    const auto check_points = [](const Points &actual, const Points &expected) {
        REQUIRE(actual.size() == expected.size());
        for (size_t i = 0; i < expected.size(); ++i) {
            CHECK(actual[i].x() == expected[i].x());
            CHECK(actual[i].y() == expected[i].y());
        }
    };
    for (size_t i = 0; i < demands.size(); ++i) {
        INFO("demand case " << i);
        const auto expected = Validation::validate_contacts(domain, support, {}, demands[i], 15., 0., {});
        const auto actual = Validation::validate_prepared_contacts(*prepared, {}, demands[i], 15., 0., {});
        REQUIRE(expected.input_valid);
        REQUIRE(expected.completed);
        REQUIRE(actual.input_valid);
        REQUIRE(actual.completed);
        REQUIRE(expected.void_cells.size() == expected_counts[i]);
        REQUIRE(actual.void_cells.size() == expected.void_cells.size());
        CHECK(actual.passed() == expected.passed());
        CHECK(actual.max_void_diameter_mm == expected.max_void_diameter_mm);
        for (size_t j = 0; j < expected.void_cells.size(); ++j) {
            const auto &actual_cell = actual.void_cells[j];
            const auto &expected_cell = expected.void_cells[j];
            CHECK(actual_cell.closed == expected_cell.closed);
            CHECK(actual_cell.exceeds_gap == expected_cell.exceeds_gap);
            CHECK(actual_cell.diameter_mm == expected_cell.diameter_mm);
            check_points(actual_cell.geometry.contour.points, expected_cell.geometry.contour.points);
            REQUIRE(actual_cell.geometry.holes.size() == expected_cell.geometry.holes.size());
            for (size_t k = 0; k < expected_cell.geometry.holes.size(); ++k)
                check_points(actual_cell.geometry.holes[k].points, expected_cell.geometry.holes[k].points);
        }
        if (i == 0) {
            CHECK(actual.void_cells.front().diameter_mm == Approx(std::sqrt(162.)));
            REQUIRE(actual.void_cells.front().geometry.holes.size() == 1);
        }
    }
}

TEST_CASE("Bridge aware prepared cell index bounds repeated sparse queries and fails closed", "[BridgeAwareCubic][Validation]")
{
    ExPolygons domain, support;
    for (size_t i = 0; i < 1024; ++i) {
        const double x = 4. * double(i);
        const Point origin = Point::new_scale(x, 0.);
        auto box = rectangle(0., 0., 2., 2.);
        box.translate(origin);
        domain.push_back(std::move(box));
        auto walls = frame(2.);
        for (auto &wall : walls) wall.translate(origin);
        append(support, std::move(walls));
    }
    Validation::WorkBudget measured_preparation;
    const size_t initial_preparation_work = measured_preparation.remaining_work;
    const auto prepared = Validation::prepare_geometry(domain, support, &measured_preparation);
    REQUIRE(prepared);
    // One immutable index serves all queries. A full 1,024-cell scan per
    // target cannot fit this allowance; exact visits to nearby cells can.
    Validation::WorkBudget budget{20000};
    for (size_t i = 0; i < 64; ++i) {
        const double x = 4. * double((i * 97) % 1024);
        const auto result = Validation::validate_prepared_contacts(*prepared, {},
            {rectangle(x + .75, .75, x + 1.25, 1.25)}, 2., 0., {}, &budget);
        INFO("query " << i);
        REQUIRE(result.input_valid);
        REQUIRE(result.completed);
        CHECK_FALSE(result.resource_limited);
        REQUIRE(result.void_cells.size() == 1);
        CHECK(result.void_cells.front().closed);
        CHECK(result.void_cells.front().diameter_mm == Approx(std::sqrt(2.)));
        CHECK(result.passed());
    }
    CHECK(budget.remaining_work > 0);
    CHECK(budget.remaining_work < 20000);

    const ExPolygons demand{rectangle(.75, .75, 1.25, 1.25)};
    const size_t preparation_work = initial_preparation_work - measured_preparation.remaining_work;
    REQUIRE(preparation_work > 1);
    // Index construction is the final preparation step. One less work unit
    // exercises failure after geometry and most of the immutable index exist.
    Validation::WorkBudget late_preparation_budget{preparation_work - 1};
    const auto late_prepared = Validation::prepare_geometry(domain, support, &late_preparation_budget);
    REQUIRE(late_prepared);
    const auto late_cached = Validation::validate_prepared_contacts(*late_prepared, {}, demand, 2., 0., {});
    CHECK(late_cached.input_valid);
    CHECK_FALSE(late_cached.completed);
    CHECK(late_cached.resource_limited);
    CHECK_FALSE(late_cached.passed());
    CHECK(late_cached.void_cells.empty());
    CHECK_FALSE(late_cached.failure_reason.empty());
    CHECK(late_preparation_budget.remaining_work == 0);

    Validation::WorkBudget exhausted{0};
    const auto incomplete = Validation::validate_prepared_contacts(*prepared, {}, demand, 2., 0., {}, &exhausted);
    CHECK(incomplete.input_valid);
    CHECK_FALSE(incomplete.completed);
    CHECK(incomplete.resource_limited);
    CHECK_FALSE(incomplete.passed());
    CHECK_FALSE(incomplete.failure_reason.empty());

    ExPolygons invalid_demand = demand;
    invalid_demand.emplace_back(Polygon({Point(500000000, 0), Point(500000001, 0)}));
    const auto invalid = Validation::validate_prepared_contacts(*prepared, {}, invalid_demand, 2., 0., {});
    CHECK_FALSE(invalid.input_valid);
    CHECK_FALSE(invalid.passed());

    ExPolygons invalid_domain = domain;
    invalid_domain.emplace_back(Polygon({Point(500000000, 0), Point(500000001, 0)}));
    const auto invalid_prepared = Validation::prepare_geometry(invalid_domain, support);
    REQUIRE(invalid_prepared);
    const auto invalid_cached = Validation::validate_prepared_contacts(*invalid_prepared, {}, demand, 2., 0., {});
    CHECK_FALSE(invalid_cached.input_valid);
    CHECK_FALSE(invalid_cached.passed());

    Validation::WorkBudget preparation_budget{0};
    const auto limited_prepared = Validation::prepare_geometry(domain, support, &preparation_budget);
    REQUIRE(limited_prepared);
    const auto limited_cached = Validation::validate_prepared_contacts(*limited_prepared, {}, demand, 2., 0., {});
    CHECK(limited_cached.input_valid);
    CHECK_FALSE(limited_cached.completed);
    CHECK(limited_cached.resource_limited);
    CHECK_FALSE(limited_cached.passed());
}

namespace {
void same_trial_decision(const Validation::Result &trial, const Validation::Result &full,
                         bool preserve_maximum = false)
{
    CHECK(trial.input_valid == full.input_valid);
    CHECK(trial.completed == full.completed);
    CHECK(trial.resource_limited == full.resource_limited);
    CHECK(trial.failure_reason == full.failure_reason);
    CHECK(trial.passed() == full.passed());
    CHECK(trial.max_unsupported_mm == full.max_unsupported_mm);
    REQUIRE(trial.unsupported_runs.size() == full.unsupported_runs.size());
    for (size_t i = 0; i < full.unsupported_runs.size(); ++i) {
        const auto &actual = trial.unsupported_runs[i], &expected = full.unsupported_runs[i];
        CHECK(actual.path_index == expected.path_index);
        CHECK(actual.begin_mm == expected.begin_mm);
        CHECK(actual.end_mm == expected.end_mm);
        CHECK(actual.length_mm == expected.length_mm);
        CHECK(actual.uncertainty_mm == expected.uncertainty_mm);
        CHECK(actual.kind == expected.kind);
        CHECK(actual.exceeds_gap == expected.exceeds_gap);
    }
    size_t failing = 0;
    double maximum = 0.;
    for (const auto &expected : full.void_cells) {
        if (expected.closed && !expected.exceeds_gap) continue;
        REQUIRE(failing < trial.void_cells.size());
        const auto &actual = trial.void_cells[failing++];
        CHECK(actual.closed == expected.closed);
        CHECK(actual.exceeds_gap == expected.exceeds_gap);
        CHECK(actual.diameter_mm == expected.diameter_mm);
        CHECK(actual.geometry.contour.points == expected.geometry.contour.points);
        REQUIRE(actual.geometry.holes.size() == expected.geometry.holes.size());
        for (size_t j = 0; j < expected.geometry.holes.size(); ++j)
            CHECK(actual.geometry.holes[j].points == expected.geometry.holes[j].points);
        maximum = std::max(maximum, expected.diameter_mm);
    }
    CHECK(trial.void_cells.size() == failing);
    CHECK(trial.max_void_diameter_mm == (preserve_maximum ? full.max_void_diameter_mm : maximum));
}
}

TEST_CASE("Bridge aware trial validation preserves every selected failing whole cell", "[BridgeAwareCubic][Validation]")
{
    const size_t small_count = GENERATE(size_t(1), size_t(70));
    const double gap = GENERATE(2., 15.);
    ExPolygons domain{rectangle(0., 0., 10., 10.), rectangle(14., 0., 18., 4.)};
    ExPolygons support = frame(10.);
    support.push_back(rectangle(4., 4., 6., 6.)); // Hole in the closed large cell.
    for (size_t i = 0; i < small_count; ++i) {
        const Point origin = Point::new_scale(24. + 4. * double(i), 0.);
        auto box = rectangle(0., 0., 2., 2.);
        box.translate(origin);
        domain.push_back(std::move(box));
        auto walls = frame(2.);
        for (auto &wall : walls) wall.translate(origin);
        append(support, std::move(walls));
    }
    const auto prepared = Validation::prepare_geometry(domain, support);
    auto surrounding = rectangle(-1., -1., 310., 11.);
    Polygon hole = rectangle(-.25, -.25, 20.25, 10.25).contour;
    hole.make_clockwise();
    surrounding.holes.push_back(std::move(hole));
    const std::vector<ExPolygons> demands{
        {rectangle(1., 1., 1.1, 1.1)},                // Inside the large cell.
        {rectangle(-1., -1., 11., 11.)},              // Contains the whole cell.
        {rectangle(4.5, 4.5, 5.5, 5.5)},              // Inside its physical island.
        {rectangle(4.5, 4., 5.5, 4.25)},              // Touches the island boundary.
        {rectangle(-1., 2., .5, 3.)},                 // Outer edge touch.
        {rectangle(-1., -1., .5, .5)},                // Outer vertex touch.
        {rectangle(15., 1., 16., 2.)},                // Whole unclosed void selected.
        {rectangle(24.75, .75, 25.25, 1.25)},         // Safe cell only.
        {rectangle(-1., -1., 310., 11.)},             // Mixed cells in original order.
        {surrounding},                               // Demand hole excludes bad cells.
        {rectangle(-5., -5., -4., -4.)},              // Disjoint selection.
        {}
    };
    for (size_t i = 0; i < demands.size(); ++i) {
        INFO("selection " << i << ", small cells " << small_count << ", gap " << gap);
        const auto full = Validation::validate_prepared_contacts(*prepared, {}, demands[i], gap, 0., {});
        const auto trial = Validation::validate_prepared_contacts_for_trial(*prepared, {}, demands[i], gap, 0., {});
        REQUIRE(full.input_valid);
        REQUIRE(full.completed);
        same_trial_decision(trial, full);
        same_trial_decision(Validation::validate_prepared_contact_failures(*prepared, {}, demands[i], gap, 0., {}), full, true);
        if (i == 6) {
            REQUIRE(trial.void_cells.size() == 1);
            CHECK_FALSE(trial.void_cells.front().closed);
            CHECK_FALSE(trial.passed());
        }
        if (i == 7) {
            REQUIRE(full.void_cells.size() == 1);
            CHECK(full.passed());
            CHECK(trial.void_cells.empty());
        }
    }
}

TEST_CASE("Bridge aware trial validation keeps contacts and exact gap boundaries", "[BridgeAwareCubic][Validation]")
{
    const auto prepared = Validation::prepare_geometry({rectangle(0., 0., 2., 2.)}, frame(2.));
    const ExPolygons demand{rectangle(-1., -1., 12., 3.)};
    const Polylines paths{path({{0., 0.}, {10., 0.}})};
    for (int contact_kind = 0; contact_kind < 4; ++contact_kind) {
        Contact::PathContact contact;
        ExPolygons masks;
        if (contact_kind <= 2) masks.push_back(rectangle(-1., -1., 1., 1.));
        if (contact_kind <= 1) masks.push_back(rectangle(9., -1., 11., 1.));
        contact.segments.push_back({0, masks, {}});
        if (contact_kind == 0) contact.segments.front().line_contacts.push_back({1, 2, 1, 2, .0002});
        contact.coordinate_uncertainty_mm = .0001;
        for (double gap : {4., 9.}) {
            const auto full = Validation::validate_prepared_contacts(*prepared, paths, demand, gap, .001, {contact});
            REQUIRE(full.input_valid);
            REQUIRE(full.completed);
            REQUIRE_FALSE(full.unsupported_runs.empty());
            same_trial_decision(Validation::validate_prepared_contacts_for_trial(*prepared, paths, demand, gap, .001, {contact}), full);
            same_trial_decision(Validation::validate_prepared_contact_failures(*prepared, paths, demand, gap, .001, {contact}), full, true);
            if (contact_kind <= 1 && gap == 9.) CHECK(full.passed());
        }
    }
    const auto baseline = Validation::validate_prepared_contacts(*prepared, {}, demand, 2., 0., {});
    REQUIRE(baseline.void_cells.size() == 1);
    const double tolerance = .001;
    const double boundary = baseline.void_cells.front().diameter_mm + tolerance;
    for (double gap : {std::nextafter(boundary, 0.), boundary,
                       std::nextafter(boundary, std::numeric_limits<double>::infinity())}) {
        const auto full = Validation::validate_prepared_contacts(*prepared, {}, demand, gap, tolerance, {});
        same_trial_decision(Validation::validate_prepared_contacts_for_trial(*prepared, {}, demand, gap, tolerance, {}), full);
        same_trial_decision(Validation::validate_prepared_contact_failures(*prepared, {}, demand, gap, tolerance, {}), full, true);
    }
}

TEST_CASE("Bridge aware trial validation retains invalid inputs and preparation failures", "[BridgeAwareCubic][Validation]")
{
    const ExPolygons domain{rectangle(0., 0., 2., 2.)}, support = frame(2.);
    const auto prepared = Validation::prepare_geometry(domain, support);
    const ExPolygons demand{rectangle(.75, .75, 1.25, 1.25)};
    for (double gap : {0., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        same_trial_decision(Validation::validate_prepared_contacts_for_trial(*prepared, {}, demand, gap, 0., {}),
                            Validation::validate_prepared_contacts(*prepared, {}, demand, gap, 0., {}));
    ExPolygons invalid = demand;
    invalid.front().contour.points.resize(2);
    same_trial_decision(Validation::validate_prepared_contacts_for_trial(*prepared, {}, invalid, 2., 0., {}),
                        Validation::validate_prepared_contacts(*prepared, {}, invalid, 2., 0., {}));
    const Polylines paths{path({{.75, 1.}, {1.25, 1.}})};
    Contact::PathContact bad_contact;
    bad_contact.segments.push_back({1, {}, {}}); // Wrong original segment index.
    same_trial_decision(Validation::validate_prepared_contacts_for_trial(*prepared, paths, demand, 2., 0., {bad_contact}),
                        Validation::validate_prepared_contacts(*prepared, paths, demand, 2., 0., {bad_contact}));
    const auto invalid_prepared = Validation::prepare_geometry(invalid, support);
    same_trial_decision(Validation::validate_prepared_contacts_for_trial(*invalid_prepared, {}, demand, 2., 0., {}),
                        Validation::validate_prepared_contacts(*invalid_prepared, {}, demand, 2., 0., {}));
    Validation::WorkBudget empty{0};
    const auto failed = Validation::prepare_geometry(domain, support, &empty);
    const auto trial = Validation::validate_prepared_contacts_for_trial(*failed, {}, demand, 2., 0., {});
    CHECK_FALSE(trial.completed);
    CHECK(trial.resource_limited);
    same_trial_decision(trial, Validation::validate_prepared_contacts(*failed, {}, demand, 2., 0., {}));
}

TEST_CASE("Bridge aware trial validation charges remaining work without passing cell copies", "[BridgeAwareCubic][Validation]")
{
    ExPolygons domain, support;
    for (size_t i = 0; i < 100; ++i) {
        const Point origin = Point::new_scale(4. * double(i), 0.);
        auto box = rectangle(0., 0., 2., 2.);
        box.translate(origin);
        domain.push_back(std::move(box));
        auto walls = frame(2.);
        for (auto &wall : walls) wall.translate(origin);
        append(support, std::move(walls));
    }
    const auto prepared = Validation::prepare_geometry(domain, support);
    const ExPolygons demand{rectangle(-1., -1., 400., 3.)};
    Validation::WorkBudget full_budget, trial_budget;
    const auto full = Validation::validate_prepared_contacts(*prepared, {}, demand, 2., 0., {}, &full_budget);
    const auto trial = Validation::validate_prepared_contacts_for_trial(*prepared, {}, demand, 2., 0., {}, &trial_budget);
    REQUIRE(full.passed());
    REQUIRE(full.void_cells.size() == 100);
    same_trial_decision(trial, full);
    CHECK(trial_budget.remaining_work > full_budget.remaining_work);
    CHECK(trial_budget.remaining_work < 50000000);
    const size_t trial_work = 50000000 - trial_budget.remaining_work;
    Validation::WorkBudget exact{trial_work};
    CHECK(Validation::validate_prepared_contacts_for_trial(*prepared, {}, demand, 2., 0., {}, &exact).passed());
    CHECK(exact.remaining_work == 0);
    Validation::WorkBudget short_budget{trial_work - 1};
    const auto incomplete = Validation::validate_prepared_contacts_for_trial(*prepared, {}, demand, 2., 0., {}, &short_budget);
    CHECK_FALSE(incomplete.completed);
    CHECK(incomplete.resource_limited);
    CHECK_FALSE(incomplete.passed());
    CHECK(short_budget.remaining_work == 0);
    Validation::WorkBudget exhausted{0};
    CHECK_FALSE(Validation::validate_prepared_contacts_for_trial(*prepared, {}, demand, 2., 0., {}, &exhausted).completed);
}

TEST_CASE("Bridge aware failure reports preserve maxima from larger safe selected cells", "[BridgeAwareCubic][Validation]")
{
    const bool descending = GENERATE(false, true);
    ExPolygons domain, support;
    for (size_t i = 0; i < 4; ++i) {
        const double side = descending ? 5. - double(i) : 2. + double(i);
        const Point origin = Point::new_scale(10. * double(i), 0.);
        auto box = rectangle(0., 0., side, side);
        box.translate(origin);
        domain.push_back(std::move(box));
        auto walls = frame(side);
        for (auto &wall : walls) wall.translate(origin);
        append(support, std::move(walls));
    }
    domain.push_back(rectangle(40., 0., 42., 2.)); // Smaller unclosed failing cell.
    const auto prepared = Validation::prepare_geometry(domain, support);
    auto with_hole = rectangle(-1., -1., 43., 6.);
    const double largest_x = descending ? 0. : 30.;
    Polygon hole = rectangle(largest_x - .25, -.25, largest_x + 5.25, 5.25).contour;
    hole.make_clockwise();
    with_hole.holes.push_back(std::move(hole));
    for (const ExPolygons &demand : std::vector<ExPolygons>{{rectangle(-1., -1., 43., 6.)}, {with_hole}}) {
        const auto full = Validation::validate_prepared_contacts(*prepared, {}, demand, 10., 0., {});
        const auto failures = Validation::validate_prepared_contact_failures(*prepared, {}, demand, 10., 0., {});
        REQUIRE(full.input_valid);
        REQUIRE(full.completed);
        same_trial_decision(failures, full, true);
        REQUIRE(failures.void_cells.size() == 1);
        CHECK_FALSE(failures.void_cells.front().closed);
        CHECK(failures.max_void_diameter_mm > failures.void_cells.front().diameter_mm);
    }
    // A grouped target's failing path need not observe its largest opening.
    // Preserve the maximum from passing paths too, before caller aggregation.
    const ExPolygons failing_demand{rectangle(40.5, .5, 41.5, 1.5)};
    const ExPolygons passing_demand{rectangle(largest_x + .75, .75, largest_x + 1.25, 1.25)};
    const auto full_failure = Validation::validate_prepared_contacts(*prepared, {}, failing_demand, 10., 0., {});
    const auto full_passing = Validation::validate_prepared_contacts(*prepared, {}, passing_demand, 10., 0., {});
    const auto failures = Validation::validate_prepared_contact_failures(*prepared, {}, failing_demand, 10., 0., {});
    const auto passing = Validation::validate_prepared_contact_failures(*prepared, {}, passing_demand, 10., 0., {});
    REQUIRE_FALSE(full_failure.passed());
    REQUIRE(full_passing.passed());
    CHECK(std::max(failures.max_void_diameter_mm, passing.max_void_diameter_mm) ==
          std::max(full_failure.max_void_diameter_mm, full_passing.max_void_diameter_mm));
    CHECK(passing.max_void_diameter_mm > failures.max_void_diameter_mm);
    CHECK(passing.void_cells.empty());
}

TEST_CASE("Bridge aware failure reports preserve invalid and incomplete states", "[BridgeAwareCubic][Validation]")
{
    const ExPolygons domain{rectangle(0., 0., 2., 2.)}, support = frame(2.);
    const auto prepared = Validation::prepare_geometry(domain, support);
    const ExPolygons demand{rectangle(.75, .75, 1.25, 1.25)};
    for (double gap : {0., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        same_trial_decision(Validation::validate_prepared_contact_failures(*prepared, {}, demand, gap, 0., {}),
                            Validation::validate_prepared_contacts(*prepared, {}, demand, gap, 0., {}), true);
    ExPolygons invalid = demand;
    invalid.front().contour.points.resize(2);
    same_trial_decision(Validation::validate_prepared_contact_failures(*prepared, {}, invalid, 2., 0., {}),
                        Validation::validate_prepared_contacts(*prepared, {}, invalid, 2., 0., {}), true);
    const Polylines paths{path({{.75, 1.}, {1.25, 1.}})};
    Contact::PathContact contact;
    contact.segments.push_back({1, {}, {}});
    same_trial_decision(Validation::validate_prepared_contact_failures(*prepared, paths, demand, 2., 0., {contact}),
                        Validation::validate_prepared_contacts(*prepared, paths, demand, 2., 0., {contact}), true);
    const auto bad_prepared = Validation::prepare_geometry(invalid, support);
    same_trial_decision(Validation::validate_prepared_contact_failures(*bad_prepared, {}, demand, 2., 0., {}),
                        Validation::validate_prepared_contacts(*bad_prepared, {}, demand, 2., 0., {}), true);
    Validation::WorkBudget empty{0};
    const auto failed = Validation::prepare_geometry(domain, support, &empty);
    const auto result = Validation::validate_prepared_contact_failures(*failed, {}, demand, 2., 0., {});
    REQUIRE_FALSE(result.completed);
    CHECK(result.resource_limited);
    same_trial_decision(result, Validation::validate_prepared_contacts(*failed, {}, demand, 2., 0., {}), true);
}

TEST_CASE("Bridge aware failure metrics skip dominated safe cell work within shared limits", "[BridgeAwareCubic][Validation]")
{
    ExPolygons domain, support;
    for (size_t i = 0; i < 100; ++i) {
        const Point origin = Point::new_scale(4. * double(i), 0.);
        auto box = rectangle(0., 0., 2., 2.);
        box.translate(origin);
        domain.push_back(std::move(box));
        auto walls = frame(2.);
        for (auto &wall : walls) wall.translate(origin);
        append(support, std::move(walls));
    }
    const auto prepared = Validation::prepare_geometry(domain, support);
    const ExPolygons demand{rectangle(-1., -1., 400., 3.)};
    Validation::WorkBudget full_budget, failure_budget, trial_budget;
    const auto full = Validation::validate_prepared_contacts(*prepared, {}, demand, 2., 0., {}, &full_budget);
    const auto failures = Validation::validate_prepared_contact_failures(*prepared, {}, demand, 2., 0., {}, &failure_budget);
    REQUIRE(full.passed());
    same_trial_decision(failures, full, true);
    REQUIRE(Validation::validate_prepared_contacts_for_trial(*prepared, {}, demand, 2., 0., {}, &trial_budget).passed());
    CHECK(failure_budget.remaining_work > full_budget.remaining_work);
    CHECK(failure_budget.remaining_work < trial_budget.remaining_work);
    const size_t work = 50000000 - failure_budget.remaining_work;
    Validation::WorkBudget exact{work};
    same_trial_decision(Validation::validate_prepared_contact_failures(*prepared, {}, demand, 2., 0., {}, &exact), full, true);
    CHECK(exact.remaining_work == 0);
    Validation::WorkBudget short_budget{work - 1};
    const auto incomplete = Validation::validate_prepared_contact_failures(*prepared, {}, demand, 2., 0., {}, &short_budget);
    CHECK_FALSE(incomplete.completed);
    CHECK(incomplete.resource_limited);
    CHECK_FALSE(incomplete.passed());
    CHECK(short_budget.remaining_work == 0);
}
