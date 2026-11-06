#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/BridgeAwareCubicAttachment.hpp"
#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSerialization.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace Slic3r;
namespace Attachment = Slic3r::BridgeAwareCubicAttachment;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {

constexpr double width = .4;
constexpr double tolerance = .001;

ExPolygon rectangle(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon({Point::new_scale(x0, y0), Point::new_scale(x1, y0),
                              Point::new_scale(x1, y1), Point::new_scale(x0, y1)}));
}

Polyline path(std::initializer_list<Vec2d> points)
{
    return Polyline::new_scale(std::vector<Vec2d>(points));
}

ExPolygons broad_domain()
{
    return {rectangle(-2., -2., 12., 2.)};
}

void check_complete_attachment(const Polyline &original, const Polyline &attached,
                               const ExPolygons &support, const ExPolygons &domain,
                               double gap)
{
    // Replacing, clipping, or simplifying the original points would conceal
    // the unsupported material instead of adding an actual attachment.
    REQUIRE(attached.points.size() > original.points.size());
    CHECK(std::search(attached.points.begin(), attached.points.end(),
                      original.points.begin(), original.points.end()) != attached.points.end());
    CHECK(attached.length() > original.length());
    const Polygons bead = offset(attached, float(scale_(width / 2.)));
    const Polygons original_bead = offset(original, float(scale_(width / 2.)));
    CHECK(diff_ex(diff_ex(bead, original_bead), domain).empty());
    const auto result = Validation::validate({}, support, {attached},
        broad_domain(), gap, tolerance);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    CHECK(result.passed());
}

} // namespace

TEST_CASE("Bridge aware attachment extends a short free end into real prior material",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {-.25, 0.}, {.025, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, 1., .5)};
    const auto saved_points = original.points;
    const auto saved_support = support;
    const auto domain = broad_domain();
    const auto attached = Attachment::attach_endpoints(original, support, domain, width, .1, tolerance);
    REQUIRE(attached.has_value());
    CHECK(attached->first_point() == original.first_point());
    CHECK(attached->last_point().x() > scale_(.05));
    CHECK(attached->last_point().x() < scale_(1.));
    CHECK(attached->last_point().y() == 0);
    check_complete_attachment(original, *attached, support, domain, .1);
    CHECK(original.points == saved_points);
    CHECK(support == saved_support);
}

TEST_CASE("Bridge aware attachment extends a short free start along its terminal tangent",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{.025, 0.}, {-.25, 0.}, {-.5, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, 1., .5)};
    const auto domain = broad_domain();
    const auto attached = Attachment::attach_endpoints(original, support, domain, width, .1, tolerance);
    REQUIRE(attached.has_value());
    CHECK(attached->last_point() == original.last_point());
    CHECK(attached->first_point().x() > scale_(.05));
    CHECK(attached->first_point().x() < scale_(1.));
    CHECK(attached->first_point().y() == 0);
    check_complete_attachment(original, *attached, support, domain, .1);
}

TEST_CASE("Bridge aware attachment counts the original air run and extension together",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {1.975, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(2., -.5, 3., .5)};
    // The extension is only about .025 mm, but the resulting air span is 2 mm.
    CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, .1, tolerance));
    CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, 2., tolerance));
}

TEST_CASE("Bridge aware attachment can rescue a floating strand only by attaching both ends",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{.5, 0.}, {1.5, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(2., -.5, 3., .5)};
    const auto domain = broad_domain();
    const auto attached = Attachment::attach_endpoints(original, support, domain, width, 2.1, tolerance);
    REQUIRE(attached.has_value());
    CHECK(attached->first_point().x() < 0);
    CHECK(attached->first_point().x() > scale_(-1.));
    CHECK(attached->last_point().x() > scale_(2.));
    CHECK(attached->last_point().x() < scale_(3.));
    CHECK(attached->first_point().y() == 0);
    CHECK(attached->last_point().y() == 0);
    check_complete_attachment(original, *attached, support, domain, 2.1);
}

TEST_CASE("Bridge aware attachment rejects a floating strand with only one reachable anchor",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{.5, 0.}, {1.5, 0.}});
    const ExPolygons support{rectangle(2., -.5, 3., .5)};
    CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, 2.1, tolerance));
}

TEST_CASE("Bridge aware attachment does not conceal an excessive span elsewhere in the path",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {3.5, 0.}, {4.5, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(3., -.5, 4., .5),
                             rectangle(5., -.5, 6., .5)};
    // Attaching the final .5 mm tail cannot repair the earlier 3 mm air span.
    CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, 1.1, tolerance));
}

TEST_CASE("Bridge aware attachment preserves a bent original path and its final direction",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {-.25, .25}, {.025, .25}});
    const ExPolygons support{rectangle(-1., -.5, 0., .75), rectangle(.05, -.5, 1., .75)};
    const auto domain = broad_domain();
    const auto attached = Attachment::attach_endpoints(original, support, domain, width, .1, tolerance);
    REQUIRE(attached.has_value());
    CHECK(attached->last_point().x() > scale_(.05));
    CHECK(attached->last_point().y() == original.last_point().y());
    check_complete_attachment(original, *attached, support, domain, .1);
}

TEST_CASE("Bridge aware attachment cannot cross a cavity to reach prior support",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {.5, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(2., -.5, 3., .5)};
    ExPolygon allowed = rectangle(-2., -2., 4., 2.);
    Polygon cavity = rectangle(.75, -.5, 1.75, .5).contour;
    cavity.make_clockwise();
    allowed.holes.push_back(cavity);
    CHECK_FALSE(Attachment::attach_endpoints(original, support, {allowed}, width, 2.1, tolerance));
}

TEST_CASE("Bridge aware attachment cannot enter another region to reach support",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {.025, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, 1., .5)};
    const ExPolygons owner_domain{rectangle(-2., -1., .04, 1.)};
    CHECK_FALSE(Attachment::attach_endpoints(original, support, owner_domain, width, .1, tolerance));
}

TEST_CASE("Bridge aware attachment keeps the entire bead inside the allowed domain",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {.025, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, 1., .5)};
    // The endpoint ray is inside this corridor, but the .4 mm bead is not.
    const ExPolygons domain{rectangle(-2., -1., .03, 1.), rectangle(.03, -.1, 1., .1)};
    CHECK_FALSE(Attachment::attach_endpoints(original, support, domain, width, .1, tolerance));
}

TEST_CASE("Bridge aware attachment rejects boundary tangency and unusably narrow anchor intervals",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {.025, 0.}});
    ExPolygons support{rectangle(-1., -.5, 0., .5)};
    SECTION("Ray only follows the anchor boundary") {
        support.push_back(rectangle(.05, 0., 1., .5));
    }
    SECTION("Anchor interior is thinner than the coordinate margin") {
        support.push_back(rectangle(.05, -.5, .05 + SCALING_FACTOR, .5));
    }
    CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, .1, tolerance));
}

TEST_CASE("Bridge aware attachment skips unusable intervals before a real anchor",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {.025, 0.}});
    ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.07, -.5, 1., .5)};
    SECTION("Microscopic interval precedes the anchor") {
        support.push_back(rectangle(.05, -.5, .05 + SCALING_FACTOR, .5));
    }
    SECTION("Boundary tangency precedes the anchor") {
        support.push_back(rectangle(.05, 0., .06, .5));
    }
    const auto domain = broad_domain();
    const auto attached = Attachment::attach_endpoints(original, support, domain, width, .1, tolerance);
    REQUIRE(attached.has_value());
    CHECK(attached->last_point().x() > scale_(.07));
    CHECK(attached->last_point().x() < scale_(1.));
    check_complete_attachment(original, *attached, support, domain, .1);
}

TEST_CASE("Bridge aware attachment constrains added material while preserving existing overlap",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {.025, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, 1., .5)};
    // The caller allows new material only beyond the old endpoint. Existing
    // extrusion outside that mask must remain unchanged, not be clipped away.
    const ExPolygons domain{rectangle(.025, -.3, 1., .3)};
    const auto attached = Attachment::attach_endpoints(original, support, domain, width, .1, tolerance);
    REQUIRE(attached.has_value());
    CHECK(attached->first_point() == original.first_point());
    CHECK(attached->last_point().x() > scale_(.05));
    check_complete_attachment(original, *attached, support, domain, .1);
}

TEST_CASE("Bridge aware attachment finds interior support after a connected boundary prefix",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {.025, 0.}});
    // These two anchor polygons meet: the ray first lies on the boundary
    // from .05 to .07, then enters usable interior without an intervening gap.
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, 0., .07, .5),
                             rectangle(.07, -.5, 1., .5)};
    const auto domain = broad_domain();
    const auto attached = Attachment::attach_endpoints(original, support, domain, width, .1, tolerance);
    REQUIRE(attached.has_value());
    CHECK(attached->last_point().x() > scale_(.07));
    CHECK(attached->last_point().x() < scale_(1.));
    CHECK(attached->last_point().y() == 0);
    check_complete_attachment(original, *attached, support, domain, .1);
}

TEST_CASE("Bridge aware attachment leaves already supported paths unchanged",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {.5, 0.}});
    ExPolygons support;
    SECTION("Continuous support") { support = {rectangle(-1., -.5, 1., .5)}; }
    SECTION("A valid two-ended bridge") {
        support = {rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, 1., .5)};
    }
    CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, .1, tolerance));
}

TEST_CASE("Bridge aware attachment is idempotent and independent of path reversal",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {-.25, 0.}, {.025, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, 1., .5)};
    const auto domain = broad_domain();
    const auto attached = Attachment::attach_endpoints(original, support, domain, width, .1, tolerance);
    REQUIRE(attached.has_value());
    CHECK_FALSE(Attachment::attach_endpoints(*attached, support, domain, width, .1, tolerance));
    Polyline reversed = original;
    reversed.reverse();
    auto attached_reversed = Attachment::attach_endpoints(reversed, support, domain, width, .1, tolerance);
    REQUIRE(attached_reversed.has_value());
    attached_reversed->reverse();
    CHECK(attached_reversed->points == attached->points);
}

TEST_CASE("Bridge aware attachment rejects paths without a valid free terminal tangent",
          "[BridgeAwareCubic][Attachment]")
{
    Polyline original;
    SECTION("Empty") {}
    SECTION("One point") { original = path({{.025, 0.}}); }
    SECTION("Closed") { original = path({{-.5, 0.}, {.025, 0.}, {-.5, 0.}}); }
    SECTION("Duplicate end") { original = path({{-.5, 0.}, {.025, 0.}, {.025, 0.}}); }
    SECTION("Duplicate start") { original = path({{.025, 0.}, {.025, 0.}, {-.5, 0.}}); }
    SECTION("Fitted arc") {
        original = path({{-.5, 0.}, {.025, 0.}});
        PathFittingData arc{};
        arc.start_point_index = 0;
        arc.end_point_index = 1;
        arc.path_type = EMovePathType::Arc_move_ccw;
        original.fitting_result.push_back(arc);
    }
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, 1., .5)};
    CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, .1, tolerance));
}

TEST_CASE("Bridge aware attachment rejects invalid numeric limits and missing material domains",
          "[BridgeAwareCubic][Attachment]")
{
    const Polyline original = path({{-.5, 0.}, {.025, 0.}});
    ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, 1., .5)};
    auto domain = broad_domain();
    double requested_width = width;
    double gap = .1;
    double error = tolerance;
    SECTION("Zero width") { requested_width = 0.; }
    SECTION("Negative width") { requested_width = -.4; }
    SECTION("Nonfinite width") { requested_width = std::numeric_limits<double>::quiet_NaN(); }
    SECTION("Zero gap") { gap = 0.; }
    SECTION("Nonfinite gap") { gap = std::numeric_limits<double>::infinity(); }
    SECTION("Negative tolerance") { error = -.001; }
    SECTION("Nonfinite tolerance") { error = std::numeric_limits<double>::quiet_NaN(); }
    SECTION("Tolerance consumes the gap") { error = gap; }
    SECTION("No allowed material") { domain.clear(); }
    SECTION("No prior material") { support.clear(); }
    CHECK_FALSE(Attachment::attach_endpoints(original, support, domain, requested_width, gap, error));
}

TEST_CASE("Bridge aware attachment bounds path complexity before generating a bead footprint",
          "[BridgeAwareCubic][Attachment]")
{
    Polyline original;
    for (size_t i = 0; i < 1025; ++i)
        original.points.push_back(Point::new_scale(-.5 + .525 * double(i) / 1024., 0.));
    const auto saved_points = original.points;
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, 1., .5)};
    // The endpoint has an otherwise valid attachment. Excessive input work
    // must fail closed without simplifying or discarding original extrusion.
    CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, .1, tolerance));
    CHECK(original.points == saved_points);
}

TEST_CASE("Bridge aware attachment centers a thin real bridge anchor before serialized validation",
          "[BridgeAwareCubic][Attachment][Contact][Serialization]")
{
    namespace Contact = BridgeAwareCubicContact;
    const auto make_bead = [](const Polyline &line, float bead_width, float height) {
        ExtrusionPath bead(erBridgeInfill, .08, bead_width, height);
        bead.polyline = line;
        return bead;
    };
    // Perpendicular bridge layers leave a 3.41 micrometre top flat. The
    // ordinary conservative search erases that real interval entirely.
    const auto left = make_bead(path({{-.5, -1.}, {-.5, 1.}}), .40341f, .4f);
    const auto right = make_bead(path({{.0517, -1.}, {.0517, 1.}}), .40341f, .4f);
    const auto original = make_bead(path({{-.5, 0.}, {.025, 0.}}), .40215f, .2f);
    const std::vector<Contact::Bead> prior{{&left, .4}, {&right, .4}};
    const auto conservative = Contact::footprint(prior, original, .6, .00216);
    REQUIRE(conservative.input_valid);
    REQUIRE(conservative.completed);
    REQUIRE(conservative.segments.size() == 1);
    CHECK(conservative.segments.front().centerline_support.empty());

    const auto exact = Contact::footprint(prior, original, .6, 0.);
    REQUIRE(exact.input_valid);
    REQUIRE(exact.completed);
    REQUIRE(exact.segments.size() == 1);
    const auto support = union_ex(exact.segments.front().centerline_support);
    REQUIRE_FALSE(support.empty());
    const auto shallow = Attachment::attach_endpoints(original.polyline, support,
        broad_domain(), original.width, .6, 0.);
    REQUIRE(shallow.has_value());
    const auto attached = Attachment::attach_endpoints(original.polyline, support,
        broad_domain(), original.width, .6, 0., .00432);
    REQUIRE(attached.has_value());
    CHECK(unscale<double>(attached->last_point().x()) == Approx(.0517).margin(2. * SCALING_FACTOR));
    CHECK(std::search(attached->points.begin(), attached->points.end(),
                      original.polyline.points.begin(), original.polyline.points.end()) != attached->points.end());

    const auto serialized_passes = [&](const Polyline &candidate, double phase) {
        BridgeAwareCubic::SerializedXY serialize;
        serialize.shift = Vec2d(80. + phase, 80. - phase);
        auto emitted_left = left, emitted_right = right, emitted_upper = original;
        emitted_left.polyline = serialize.emitted_polyline(left.polyline);
        emitted_right.polyline = serialize.emitted_polyline(right.polyline);
        emitted_upper.polyline = serialize.emitted_polyline(candidate);
        const auto contact = Contact::footprint({{&emitted_left, .4}, {&emitted_right, .4}},
                                                emitted_upper, .6, 0.);
        const auto demand = union_ex(offset(Polylines{emitted_upper.polyline}, scale_(.5 * original.width)));
        return Validation::validate_contacts({}, contact.physical_footprint, {emitted_upper.polyline},
                                              demand, .6, 0., {contact}).passed();
    };
    // Merely retaining the narrow interval is insufficient: its former
    // near-boundary candidate rounds outside the physical lower bead.
    CHECK_FALSE(serialized_passes(*shallow, 0.));
    for (double phase : {-.0004, 0., .0004})
        CHECK(serialized_passes(*attached, phase));
}

TEST_CASE("Bridge aware attachment preferred inset remains bounded by the actual anchor",
          "[BridgeAwareCubic][Attachment]")
{
    const auto original = path({{-.5, 0.}, {.025, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, .0534, .5)};
    const auto attached = Attachment::attach_endpoints(original, support, broad_domain(), width, .1, 0., 10.);
    REQUIRE(attached.has_value());
    CHECK(unscale<double>(attached->last_point().x()) == Approx(.0517).margin(SCALING_FACTOR));
    CHECK(Validation::validate({}, support, {*attached}, broad_domain(), .1, 0.).passed());
    for (double invalid : {-1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
        CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, .1, 0., invalid));
}

TEST_CASE("Bridge aware attachment skips a real thin interval lost during serialized output",
          "[BridgeAwareCubic][Attachment][Contact][Serialization]")
{
    namespace Contact = BridgeAwareCubicContact;
    const bool reversed = GENERATE(false, true);
    ExtrusionPath upper(erSolidInfill, .08, .400240391493f, .2f);
    upper.polyline = path({{8.33707, -8.66078}, {7.57203, -8.89468}});
    if (reversed) upper.polyline.reverse();
    const Points original_points = upper.polyline.points;
    // Retain the actual rib's 17-degree terminal and the first two lower
    // bridge strands. A normal lower bead seats the opposite endpoint, making
    // this a complete reduced path whose entire attachment can be verified.
    ExtrusionPaths lower;
    lower.emplace_back(erBridgeInfill, .08, .40061f, .4f);
    lower.back().polyline = path({{2.42873, 12.09295}, {9.82310, -12.09294}});
    lower.emplace_back(erBridgeInfill, .08, .40061f, .4f);
    lower.back().polyline = path({{10.29430, -12.09294}, {2.89993, 12.09294}});
    lower.emplace_back(erSolidInfill, .08, .4f, .2f);
    lower.back().polyline = path({{7.57203, -9.4}, {7.57203, -8.4}});
    std::vector<Contact::Bead> prior;
    for (const ExtrusionPath &bead : lower) prior.push_back({&bead, 9.6});
    const ExPolygons domain{rectangle(6., -14., 12., -6.)};
    const auto original_contact = Contact::footprint(prior, upper, 9.8, 0.);
    const auto original_result = Validation::validate_contacts({}, original_contact.physical_footprint,
        {upper.polyline}, domain, 5., 0., {original_contact});
    REQUIRE(original_result.input_valid);
    REQUIRE(original_result.completed);
    REQUIRE(original_result.unsupported_runs.size() == 1);
    REQUIRE(original_result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::OneSidedOverhang);
    Attachment::TerminalAirLengths air;
    (reversed ? air.end_mm : air.start_mm) = original_result.unsupported_runs.front().length_mm;

    ExtrusionPath probe = upper;
    const Vec2d outward = (Point::new_scale(8.33707, -8.66078) - Point::new_scale(7.57203, -8.89468)).cast<double>().normalized();
    const Vec2d far = Vec2d(8.33707, -8.66078) + 5. * outward;
    probe.polyline = path({far, {7.57203, -8.89468}});
    const auto probe_contact = Contact::footprint(prior, probe, 9.8, 0.);
    REQUIRE(probe_contact.input_valid);
    REQUIRE(probe_contact.completed);
    REQUIRE(probe_contact.segments.size() == 1);
    const ExPolygons support = union_ex(probe_contact.segments.front().centerline_support);
    const auto unchanged_support = support;
    const auto attach = [&](const Attachment::TerminalAnchorIntervals &intervals, double gap = 5., double inset = .00432) {
        return Attachment::attach_endpoints(upper.polyline, support, domain, upper.width, gap, 0., inset,
                                            &support, &support, &air, &intervals);
    };
    const Attachment::TerminalAnchorIntervals first;
    const auto nearest = attach(first);
    REQUIRE(nearest.has_value());
    const auto deeper_same_interval = attach(first, 5., 5.);
    REQUIRE(deeper_same_interval.has_value());
    CHECK(deeper_same_interval->points == nearest->points);
    Attachment::TerminalAnchorIntervals later;
    (reversed ? later.end_skip : later.start_skip) = 1;
    const auto attached = attach(later);
    REQUIRE(attached.has_value());
    const Point repaired_endpoint = reversed ? attached->last_point() : attached->first_point();
    CHECK(unscale<double>(repaired_endpoint.x()) == Approx(9.16737).margin(.0001));
    CHECK(unscale<double>(repaired_endpoint.y()) == Approx(-8.40693).margin(.0001));
    CHECK(std::search(attached->points.begin(), attached->points.end(), original_points.begin(), original_points.end()) != attached->points.end());
    CHECK(diff_ex(diff_ex(offset(*attached, scale_(.5 * upper.width)),
                          offset(upper.polyline, scale_(.5 * upper.width))), domain).empty());
    CHECK(upper.polyline.points == original_points);
    CHECK(support == unchanged_support);

    const auto emitted_result = [&](const Polyline &candidate) {
        BridgeAwareCubic::SerializedXY serialize;
        serialize.shift = Vec2d(80., 80.);
        ExtrusionPath emitted_upper = upper;
        emitted_upper.polyline = serialize.emitted_polyline(candidate);
        auto emitted_lower = lower;
        std::vector<Contact::Bead> deposited;
        for (ExtrusionPath &bead : emitted_lower) {
            bead.polyline = serialize.emitted_polyline(bead.polyline);
            deposited.push_back({&bead, 9.6});
        }
        const auto actual = Contact::footprint(deposited, emitted_upper, 9.8, 0.);
        const ExPolygons demand = union_ex(offset(Polylines{emitted_upper.polyline}, scale_(.5 * upper.width)));
        return Validation::validate_contacts({}, actual.physical_footprint, {emitted_upper.polyline}, demand, 5., 0., {actual});
    };
    const auto rejected = emitted_result(*nearest);
    REQUIRE(rejected.input_valid);
    REQUIRE(rejected.completed);
    CHECK_FALSE(rejected.passed());
    CHECK(std::any_of(rejected.unsupported_runs.begin(), rejected.unsupported_runs.end(), [](const auto &run) {
        return run.kind == Validation::UnsupportedRunKind::OneSidedOverhang;
    }));
    const auto accepted = emitted_result(*attached);
    REQUIRE(accepted.input_valid);
    REQUIRE(accepted.completed);
    CHECK(accepted.passed());
    CHECK(accepted.max_unsupported_mm <= 5.);
    REQUIRE(attach(first, 1.2).has_value());
    CHECK_FALSE(attach(later, 1.2)); // Original terminal air still spends G.
    (reversed ? later.end_skip : later.start_skip) = 2;
    CHECK_FALSE(attach(later)); // No third real anchor in this bounded ray.
}

TEST_CASE("Bridge aware attachment searches deeper when translated output rotates a short extension",
          "[BridgeAwareCubic][Attachment][Contact][Serialization]")
{
    namespace Contact = BridgeAwareCubicContact;
    ExtrusionPath upper(erInternalInfill, .08, .4f, .2f);
    upper.polyline = path({{-4.00149, 12.74812}, {9.83945, 9.03945}});
    const Points original_points = upper.polyline.points;
    // The actual failed export used this 45-degree solid-fill family below
    // a -15-degree cubic strand. Preserve its bead dimensions, next anchor,
    // complete upper path and translated output phase in this focused case.
    ExtrusionPaths lower;
    lower.reserve(37);
    for (size_t i = 0; i < 37; ++i) {
        const double shift = .506825 * double(i);
        lower.emplace_back(erSolidInfill, .08, .401300370693f, .2f);
        lower.back().polyline = path({{-12.12558 - shift, -13.13942}, {14.15326 - shift, 13.13942}});
    }
    std::vector<Contact::Bead> prior;
    for (const auto &bead : lower) prior.push_back({&bead, .6});
    const auto contact = Contact::footprint(prior, upper, .8, 0.);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    REQUIRE(contact.segments.size() == 1);
    const auto support = union_ex(contact.segments.front().centerline_support);
    const auto unchanged_support = support;
    const ExPolygons domain{rectangle(-20., -20., 20., 20.)};
    const auto shallow = Attachment::attach_endpoints(upper.polyline, support, domain, upper.width, 5., 0., .00432);
    const auto deeper = Attachment::attach_endpoints(upper.polyline, support, domain, upper.width, 5., 0., .5 * upper.width);
    const auto midpoint = Attachment::attach_endpoints(upper.polyline, support, domain, upper.width, 5., 0., 5.);
    REQUIRE(shallow.has_value());
    REQUIRE(deeper.has_value());
    REQUIRE(midpoint.has_value());
    CHECK(deeper->points == midpoint->points);
    CHECK(unscale<double>(deeper->last_point().x()) == Approx(10.00810).margin(.0001));
    CHECK(unscale<double>(deeper->last_point().y()) == Approx(8.99426).margin(.0001));
    CHECK(std::search(deeper->points.begin(), deeper->points.end(), original_points.begin(), original_points.end()) !=
          deeper->points.end());
    CHECK(upper.polyline.points == original_points);
    CHECK(support == unchanged_support);
    CHECK(diff_ex(diff_ex(offset(*deeper, scale_(.5 * upper.width)),
                          offset(upper.polyline, scale_(.5 * upper.width))), domain).empty());

    const auto serialized_result = [&](const Polyline &candidate, const Vec2d &phase) {
        BridgeAwareCubic::SerializedXY serialize;
        serialize.shift = Vec2d(80., 80.) + phase;
        auto emitted_upper = upper;
        emitted_upper.polyline = serialize.emitted_polyline(candidate);
        auto emitted_lower = lower;
        std::vector<Contact::Bead> deposited;
        for (auto &bead : emitted_lower) {
            bead.polyline = serialize.emitted_polyline(bead.polyline);
            deposited.push_back({&bead, .6});
        }
        const auto actual = Contact::footprint(deposited, emitted_upper, .8, 0.);
        const auto demand = union_ex(offset(Polylines{emitted_upper.polyline}, scale_(.5 * upper.width)));
        return Validation::validate_contacts({}, actual.physical_footprint, {emitted_upper.polyline},
                                              demand, 5., 0., {actual});
    };
    CHECK(serialized_result(*shallow, Vec2d::Zero()).passed());
    const Vec2d translated_phase(.00024, .00032);
    const auto rejected = serialized_result(*shallow, translated_phase);
    REQUIRE(rejected.input_valid);
    REQUIRE(rejected.completed);
    CHECK_FALSE(rejected.passed());
    CHECK(std::any_of(rejected.unsupported_runs.begin(), rejected.unsupported_runs.end(), [](const auto &run) {
        return run.kind == Validation::UnsupportedRunKind::OneSidedOverhang;
    }));
    CHECK(serialized_result(*deeper, Vec2d::Zero()).passed());
    CHECK(serialized_result(*deeper, translated_phase).passed());
}

TEST_CASE("Bridge aware attachment preferred inset falls back when only shallow material is permitted",
          "[BridgeAwareCubic][Attachment]")
{
    const auto original = path({{-.5, 0.}, {.025, 0.}});
    const ExPolygons support{rectangle(-1., -.5, 0., .5), rectangle(.05, -.5, .2, .5)};
    const ExPolygons allowed{rectangle(-2., -1., .053, 1.)};
    const auto shallow = Attachment::attach_endpoints(original, support, allowed, width, .1, 0.);
    REQUIRE(shallow.has_value());
    const auto preferred = Attachment::attach_endpoints(original, support, allowed, width, .1, 0., .00432);
    REQUIRE(preferred.has_value());
    CHECK(preferred->points == shallow->points);
    CHECK(diff_ex(diff_ex(offset(*preferred, scale_(.5 * width)), offset(original, scale_(.5 * width))), allowed).empty());
}

TEST_CASE("Bridge aware attachment uses the terminal direction for need and anchor search",
          "[BridgeAwareCubic][Attachment][Contact]")
{
    const auto original = path({{0., .5}, {0., 0.}, {1., 0.}});
    const ExPolygons terminal_support{rectangle(-.1, -.1, 1.1, .1), rectangle(-.1, .8, .1, .9)};
    ExPolygons all_directions = terminal_support;
    SECTION("Another segment direction falsely supports the original endpoint") {
        all_directions.push_back(rectangle(-.1, .45, .1, .55));
    }
    SECTION("Another segment direction offers a nearer false anchor") {
        all_directions.push_back(rectangle(-.1, .6, .1, .7));
    }
    const Attachment::TerminalAirLengths air{.4, std::nullopt};
    const auto attached = Attachment::attach_endpoints(original, union_ex(all_directions), broad_domain(),
        width, 1., 0., .00432, &terminal_support, nullptr, &air);
    REQUIRE(attached.has_value());
    CHECK(attached->first_point().y() > scale_(.8));
    CHECK(attached->first_point().y() < scale_(.9));
    CHECK(attached->first_point().x() == 0);
    CHECK(attached->last_point() == original.last_point());
    CHECK(std::search(attached->points.begin(), attached->points.end(),
                      original.points.begin(), original.points.end()) != attached->points.end());
    CHECK(Validation::validate({}, terminal_support, {*attached}, broad_domain(), 1., 0.).passed());
}

TEST_CASE("Bridge aware attachment reuses the verified complete air budget across a bend",
          "[BridgeAwareCubic][Attachment][Contact]")
{
    const auto original = path({{0., .5}, {0., 0.}, {1., 0.}});
    const ExPolygons start_support{rectangle(-.1, .8, .1, .9)};
    const ExPolygons other_support{rectangle(-.1, -.1, 1.1, .1)};
    const auto all_support = union_ex(start_support, other_support);
    // The terminal segment has .5 mm of air before its bend. The next
    // segment is supported by its own direction's mask; it must not turn the
    // entire 1.5 mm path into a floating strand under the terminal-only mask.
    const Attachment::TerminalAirLengths air{.5, std::nullopt};
    const auto attached = Attachment::attach_endpoints(original, all_support, broad_domain(),
        width, 1., 0., .00432, &start_support, nullptr, &air);
    REQUIRE(attached.has_value());
    CHECK(attached->first_point().y() > scale_(.8));
    CHECK(attached->first_point().y() < scale_(.9));
    CHECK(attached->last_point() == original.last_point());
    CHECK(Validation::validate({}, all_support, {*attached}, broad_domain(), 1., 0.).passed());
}

TEST_CASE("Bridge aware directional attachment requires valid terminal classification",
          "[BridgeAwareCubic][Attachment][Contact]")
{
    const auto original = path({{0., .5}, {0., 0.}, {1., 0.}});
    const ExPolygons support{rectangle(-.1, -.1, 1.1, .1), rectangle(-.1, .8, .1, .9)};
    CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, 1., 0.,
                                            .00432, &support, nullptr));
    for (double invalid : {-1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        const Attachment::TerminalAirLengths air{invalid, std::nullopt};
        CHECK_FALSE(Attachment::attach_endpoints(original, support, broad_domain(), width, 1., 0.,
                                                .00432, &support, nullptr, &air));
    }
}

namespace {

BridgeAwareCubic::LayerAttachment decision(const Polyline &original, const Polyline &replacement)
{
    BridgeAwareCubic::LayerAttachment result;
    result.z = .8;
    result.emitting_region = 2;
    result.original = original;
    result.replacement = replacement;
    result.width = width;
    result.height = .2;
    return result;
}

} // namespace

TEST_CASE("Bridge aware attachment chains preserve unmatched paths and inputs",
          "[BridgeAwareCubic][Attachment][AttachmentChain]")
{
    const Polyline original = path({{0., 0.}, {1., 0.}});
    const auto result = Attachment::resolve_chain(original, width, .2, 2, {});
    CHECK(result.polyline.points == original.points);
    CHECK(result.applied_indices.empty());
}

TEST_CASE("Bridge aware attachment chains resolve independently of storage order with exact lineage",
          "[BridgeAwareCubic][Attachment][AttachmentChain]")
{
    const Polyline a = path({{0., 0.}, {1., 0.}});
    const Polyline b = path({{0., 0.}, {1., 0.}, {2., 0.}});
    const Polyline c = path({{-1., 0.}, {0., 0.}, {1., 0.}, {2., 0.}});
    const Polyline d = path({{-1., 0.}, {0., 0.}, {1., 0.}, {2., 0.}, {3., 0.}});
    std::vector<BridgeAwareCubic::LayerAttachment> records{
        decision(b, c), decision(c, d), decision(a, b)};
    const auto result = Attachment::resolve_chain(a, width, .2, 2, records);
    CHECK(result.polyline.points == d.points);
    CHECK((result.applied_indices == std::vector<size_t>{2, 0, 1}));
    CHECK(records[2].original.points == a.points);
    CHECK(records[2].replacement.points == b.points);

    std::reverse(records.begin(), records.end());
    const auto reordered = Attachment::resolve_chain(a, width, .2, 2, records);
    CHECK(reordered.polyline.points == d.points);
    CHECK((reordered.applied_indices == std::vector<size_t>{0, 2, 1}));
    CHECK(Attachment::resolve_chain(d, width, .2, 2, records).applied_indices.empty());
}

TEST_CASE("Bridge aware attachment chains require exact emitter dimensions and oriented point identity",
          "[BridgeAwareCubic][Attachment][AttachmentChain]")
{
    const Polyline original = path({{0., 0.}, {1., 0.}});
    auto record = decision(original, path({{0., 0.}, {1., 0.}, {2., 0.}}));
    SECTION("Other emitting region") { record.emitting_region = 1; }
    SECTION("Different width even within geometric tolerance") { record.width = std::nextafter(width, 1.); }
    SECTION("Different height even within geometric tolerance") { record.height = std::nextafter(.2, 1.); }
    SECTION("Reversed path") { record.original.reverse(); }
    SECTION("Equivalent geometry with different point identity") { record.original = path({{0., 0.}, {.5, 0.}, {1., 0.}}); }
    const auto result = Attachment::resolve_chain(original, width, .2, 2, {record});
    CHECK(result.polyline.points == original.points);
    CHECK(result.applied_indices.empty());
}

TEST_CASE("Bridge aware attachment chains reject ambiguous replacement or metadata lineage",
          "[BridgeAwareCubic][Attachment][AttachmentChain]")
{
    const Polyline a = path({{0., 0.}, {1., 0.}});
    const auto first = decision(a, path({{0., 0.}, {1., 0.}, {2., 0.}}));
    auto second = first;
    SECTION("Different replacement") { second.replacement.points.back().x() += 1; }
    SECTION("Equal geometry with different source owner") { second.owners.push_back({{{1, 0, 0}}, 1}); }
    SECTION("Duplicate decisions are not silently assigned arbitrary lineage") {}
    CHECK_THROWS_AS(Attachment::resolve_chain(a, width, .2, 2, {first, second}), std::invalid_argument);
    CHECK_THROWS_AS(Attachment::resolve_chain(a, width, .2, 2, {second, first}), std::invalid_argument);
}

TEST_CASE("Bridge aware attachment chains reject no-op decisions and cycles",
          "[BridgeAwareCubic][Attachment][AttachmentChain]")
{
    const Polyline a = path({{0., 0.}, {1., 0.}});
    const Polyline b = path({{0., 0.}, {1., 0.}, {2., 0.}});
    std::vector<BridgeAwareCubic::LayerAttachment> records;
    SECTION("No-op") { records = {decision(a, a)}; }
    SECTION("Cycle") { records = {decision(b, a), decision(a, b)}; }
    SECTION("Cycle following a valid prefix") {
        const Polyline c = path({{0., 0.}, {1., 0.}, {2., 0.}, {3., 0.}});
        records = {decision(a, b), decision(b, c), decision(c, b)};
    }
    CHECK_THROWS_AS(Attachment::resolve_chain(a, width, .2, 2, records), std::invalid_argument);
}

TEST_CASE("Bridge aware attachment chains never return a prefix before a later ambiguity",
          "[BridgeAwareCubic][Attachment][AttachmentChain]")
{
    const Polyline a = path({{0., 0.}, {1., 0.}});
    const Polyline b = path({{0., 0.}, {1., 0.}, {2., 0.}});
    const Polyline c = path({{0., 0.}, {1., 0.}, {2., 0.}, {3., 0.}});
    const Polyline d = path({{0., 0.}, {1., 0.}, {2., 0.}, {4., 0.}});
    std::vector<BridgeAwareCubic::LayerAttachment> records{decision(b, c), decision(a, b), decision(b, d)};
    CHECK_THROWS_AS(Attachment::resolve_chain(a, width, .2, 2, records), std::invalid_argument);
    std::reverse(records.begin(), records.end());
    CHECK_THROWS_AS(Attachment::resolve_chain(a, width, .2, 2, records), std::invalid_argument);
    CHECK(a.points == path({{0., 0.}, {1., 0.}}).points);
}

TEST_CASE("Bridge aware attachment chains ignore unrelated malformed decision graphs",
          "[BridgeAwareCubic][Attachment][AttachmentChain]")
{
    const Polyline a = path({{0., 0.}, {1., 0.}});
    const Polyline b = path({{0., 0.}, {1., 0.}, {2., 0.}});
    const Polyline elsewhere = path({{0., 1.}, {1., 1.}});
    const auto result = Attachment::resolve_chain(a, width, .2, 2,
        {decision(elsewhere, elsewhere), decision(elsewhere, elsewhere), decision(a, b)});
    CHECK(result.polyline.points == b.points);
    CHECK(result.applied_indices == std::vector<size_t>{2});
}

TEST_CASE("Bridge aware attachment chains bound matching and point comparison work without partial results",
          "[BridgeAwareCubic][Attachment][AttachmentChain]")
{
    const Polyline a = path({{0., 0.}, {1., 0.}});
    const Polyline b = path({{0., 0.}, {1., 0.}, {2., 0.}});
    CHECK_THROWS_AS(Attachment::resolve_chain(a, width, .2, 2, {}, 0), std::runtime_error);
    CHECK_THROWS_AS(Attachment::resolve_chain(a, width, .2, 2, {decision(a, b)}, 1), std::runtime_error);
    // Eleven units reach the replacement but cannot copy the completed result.
    CHECK_THROWS_AS(Attachment::resolve_chain(a, width, .2, 2, {decision(a, b)}, 11), std::runtime_error);
    CHECK(Attachment::resolve_chain(a, width, .2, 2, {decision(a, b)}, 100).polyline.points == b.points);

    Polyline many_points;
    for (int i = 0; i < 100; ++i) many_points.points.emplace_back(i, 0);
    CHECK_THROWS_AS(Attachment::resolve_chain(many_points, width, .2, 2, {}, 50), std::runtime_error);
}

TEST_CASE("Bridge aware attachment chains reject invalid dimensions and matched replacement geometry",
          "[BridgeAwareCubic][Attachment][AttachmentChain]")
{
    const Polyline a = path({{0., 0.}, {1., 0.}});
    const Polyline b = path({{0., 0.}, {1., 0.}, {2., 0.}});
    double target_width = width, target_height = .2;
    auto record = decision(a, b);
    SECTION("Nonfinite width") { target_width = std::numeric_limits<double>::infinity(); }
    SECTION("Zero height") { target_height = 0.; }
    SECTION("Missing replacement geometry") { record.replacement.clear(); }
    SECTION("Replacement arc cannot be identified by points alone") {
        PathFittingData arc{};
        arc.path_type = EMovePathType::Arc_move_ccw;
        record.replacement.fitting_result.push_back(arc);
    }
    SECTION("Saved original arc cannot be identified by points alone") {
        PathFittingData arc{};
        arc.path_type = EMovePathType::Arc_move_ccw;
        record.original.fitting_result.push_back(arc);
    }
    CHECK_THROWS_AS(Attachment::resolve_chain(a, target_width, target_height, 2, {record}), std::invalid_argument);
}

TEST_CASE("Bridge aware attachment chains preserve unmatched arcs but reject matched arc queries",
          "[BridgeAwareCubic][Attachment][AttachmentChain]")
{
    Polyline original = path({{0., 0.}, {1., 0.}});
    const auto record = decision(original, path({{0., 0.}, {1., 0.}, {2., 0.}}));
    PathFittingData arc{};
    arc.path_type = EMovePathType::Arc_move_ccw;
    original.fitting_result.push_back(arc);
    CHECK_THROWS_AS(Attachment::resolve_chain(original, width, .2, 2, {record}), std::invalid_argument);
    const auto untouched = Attachment::resolve_chain(original, width, .2, 1, {record});
    CHECK(untouched.polyline.points == original.points);
    REQUIRE(untouched.polyline.fitting_result.size() == 1);
    CHECK(untouched.polyline.fitting_result.front().path_type == EMovePathType::Arc_move_ccw);
    CHECK(untouched.applied_indices.empty());
}
