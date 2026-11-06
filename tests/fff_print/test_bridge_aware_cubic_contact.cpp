#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSerialization.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace Slic3r;
namespace Contact = Slic3r::BridgeAwareCubicContact;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
ExtrusionPath bead(std::initializer_list<Vec2d> points, float width = .45f, float height = .2f)
{
    ExtrusionPath result(erInternalInfill, .08, width, height);
    result.polyline = Polyline::new_scale(std::vector<Vec2d>(points));
    return result;
}
ExPolygon contact_demand()
{
    return ExPolygon(Polygon({Point::new_scale(-2., -2.), Point::new_scale(12., -2.),
                              Point::new_scale(12., 2.), Point::new_scale(-2., 2.)}));
}
Validation::Result check_contact(const ExtrusionPath &upper, const Contact::PathContact &contact,
                                 double gap = 20., const ExPolygons &domain = {})
{
    return Validation::validate_contacts(domain, contact.physical_footprint, {upper.polyline},
                                         {contact_demand()}, gap, .0001, {contact});
}
}

TEST_CASE("Bridge aware contact uses actual rounded bead sections", "[BridgeAwareCubic][Contact]")
{
    CHECK(Contact::section_width_mm(.45, .2, .4, .4) == Approx(.25));
    CHECK(Contact::section_width_mm(.45, .2, .4, .2) == Approx(.25));
    CHECK(Contact::section_width_mm(.45, .2, .4, .3) == Approx(.45));
    CHECK(Contact::section_width_mm(.4, .4, .4, .4) == 0.);
    CHECK(Contact::section_width_mm(.4, .4, .4, .2) == Approx(.4));
    CHECK(Contact::section_width_mm(.45, .2, .4, .4001) == 0.);
    CHECK(std::isnan(Contact::section_width_mm(.1, .2, .4, .3)));
}

TEST_CASE("Bridge aware contact allows actual overlap of shifted normal beads", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{-1., 0.}, {11., 0.}});
    const auto upper = bead({{0., .2 / std::sqrt(2.)}, {10., .2 / std::sqrt(2.)}});
    const auto contact = Contact::footprint({{&lower, .2}}, upper, .4, .0001);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    CHECK(contact.contact_z == Approx(.2));
    CHECK(check_contact(upper, contact).passed());
    // Upper bead width participates in anchor contact, but not physical walls.
    CHECK_FALSE(Validation::validate({}, contact.physical_footprint, {upper.polyline},
                                    {contact_demand()}, 20., .0001).passed());
}

TEST_CASE("Bridge aware contact rejects full-width projection and true vertical gaps", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{-1., 0.}, {11., 0.}});
    const auto upper = bead({{0., .3}, {10., .3}});
    CHECK_FALSE(check_contact(upper, Contact::footprint({{&lower, .2}}, upper, .4, .0001)).passed());
    const auto aligned = bead({{0., 0.}, {10., 0.}});
    // The much larger XY tolerance must not excuse even this small Z gap.
    const auto separated = Contact::footprint({{&lower, .2}}, aligned, .40001, .002);
    REQUIRE(separated.input_valid);
    REQUIRE(separated.completed);
    CHECK_FALSE(check_contact(aligned, separated).passed());
}

TEST_CASE("Bridge aware contact accepts finite circular-top line anchors", "[BridgeAwareCubic][Contact]")
{
    auto lower_left = bead({{0., -1.}, {0., 1.}}, .4f, .4f);
    auto lower_right = bead({{10., -1.}, {10., 1.}}, .4f, .4f);
    auto upper = bead({{0., 0.}, {10., 0.}});
    if (GENERATE(false, true)) {
        std::reverse(upper.polyline.points.begin(), upper.polyline.points.end());
        std::reverse(lower_left.polyline.points.begin(), lower_left.polyline.points.end());
    }
    const auto contact = Contact::footprint({{&lower_left, .4}, {&lower_right, .4}}, upper, .6, 0.);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    REQUIRE(contact.segments.size() == 1);
    CHECK(contact.segments.front().line_contacts.size() == 2);
    CHECK(contact.physical_footprint.empty());
    CHECK(contact.physical_lines.size() == 2);
    const auto result = check_contact(upper, contact, 10.1);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::TwoEndedBridge);
    CHECK(result.unsupported_runs.front().length_mm == Approx(10.));
    CHECK(result.passed());
    // Contact certificates cannot create an artificial physical cell wall.
    CHECK_FALSE(check_contact(upper, contact, 20., {contact_demand()}).passed());
}

TEST_CASE("Bridge aware contact distinguishes finite line from isolated point", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{5., -1.}, {5., 1.}}, .4f, .4f);
    const auto upper = bead({{0., 0.}, {10., 0.}}, .4f, .4f);
    const auto contact = Contact::footprint({{&lower, .4}}, upper, .8, .0001);
    REQUIRE(contact.completed);
    REQUIRE(contact.segments.size() == 1);
    CHECK(contact.segments.front().line_contacts.empty());
    const auto result = check_contact(upper, contact);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::Floating);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware circular-top parallel contact respects upper flat width", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{-1., 0.}, {11., 0.}}, .4f, .4f);
    const double shift = GENERATE(.1, .2);
    const auto upper = bead({{0., shift}, {10., shift}});
    const auto contact = Contact::footprint({{&lower, .4}}, upper, .6, .0001);
    CHECK(check_contact(upper, contact).passed() == (shift == .1));
}

TEST_CASE("Bridge aware contact uses one real plane through variable height beads", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{-1., 0.}, {11., 0.}}, .4f, .4f);
    const auto upper = bead({{0., .1}, {10., .1}}, .5f, .3f);
    const auto contact = Contact::footprint({{&lower, .4}}, upper, .6, .0001);
    REQUIRE(contact.completed);
    CHECK(contact.contact_z > .3);
    CHECK(contact.contact_z < .4);
    CHECK_FALSE(contact.physical_footprint.empty());
    CHECK(check_contact(upper, contact).passed());
}

TEST_CASE("Bridge aware contact fails closed on unsupported or nonfinite bead dimensions", "[BridgeAwareCubic][Contact]")
{
    auto lower = bead({{-1., 0.}, {11., 0.}});
    const auto upper = bead({{0., 0.}, {10., 0.}});
    lower.width = GENERATE(0.f, std::numeric_limits<float>::infinity());
    const auto contact = Contact::footprint({{&lower, .2}}, upper, .4, .0001);
    CHECK_FALSE(contact.input_valid);
    CHECK_FALSE(check_contact(upper, contact).passed());
}

TEST_CASE("Bridge aware contact excludes unsupported lower shapes without discarding real support", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{-1., 0.}, {11., 0.}});
    const auto narrow = bead({{-1., 0.}, {11., 0.}}, .1f, .2f);
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const auto supported = Contact::footprint({{&lower, .2}, {&narrow, .2}}, upper, .4, .0001);
    REQUIRE(supported.input_valid);
    REQUIRE(supported.completed);
    CHECK(check_contact(upper, supported).passed());
    const auto unsupported = Contact::footprint({{&narrow, .2}}, upper, .4, .0001);
    REQUIRE(unsupported.input_valid);
    REQUIRE(unsupported.completed);
    CHECK(unsupported.physical_footprint.empty());
    CHECK_FALSE(check_contact(upper, unsupported).passed());
    const auto invalid_upper = Contact::footprint({{&lower, .2}}, narrow, .4, .0001);
    CHECK_FALSE(invalid_upper.input_valid);
}

TEST_CASE("Bridge aware contact preserves genuine collinear seams but not small air gaps", "[BridgeAwareCubic][Contact]")
{
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const auto split_path = bead({{-1., 0.}, {5., 0.}, {11., 0.}});
    const auto joined = Contact::footprint({{&split_path, .2}}, upper, .4, .0001);
    CHECK(check_contact(upper, joined).unsupported_runs.empty());
    const auto left = bead({{-1., 0.}, {5., 0.}});
    const auto right = bead({{5., 0.}, {11., 0.}});
    const auto separate_joined = Contact::footprint({{&left, .2}, {&right, .2}}, upper, .4, .0001);
    CHECK(check_contact(upper, separate_joined).unsupported_runs.empty());
    const auto right_gap = bead({{5.0001, 0.}, {11., 0.}});
    const auto separated = Contact::footprint({{&left, .2}, {&right_gap, .2}}, upper, .4, .0001);
    REQUIRE(check_contact(upper, separated).unsupported_runs.size() == 1);
    CHECK(check_contact(upper, separated).unsupported_runs.front().length_mm >= .0001);
}

TEST_CASE("Bridge aware qualified line anchors retain one-sided endpoint classification", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{0., -1.}, {0., 1.}}, .4f, .4f);
    auto upper = bead({{0., 0.}, {10., 0.}});
    if (GENERATE(false, true))
        std::reverse(upper.polyline.points.begin(), upper.polyline.points.end());
    const auto contact = Contact::footprint({{&lower, .4}}, upper, .6, 0.);
    const auto result = check_contact(upper, contact);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::OneSidedOverhang);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware endpoint line contact must survive the XY rounding reserve", "[BridgeAwareCubic][Contact]")
{
    auto lower = bead({{0., 0.}, {1., 0.}}, .4f, .4f);
    auto upper = bead({{0., 0.}, {3., 1.}});
    // Exact represented directions are perpendicular. Export at .001 mm can
    // move the lower line just behind the original upper starting butt plane.
    lower.polyline.points = {Point(-16667, 50001), Point(33334, -100002)};
    upper.polyline.points = {Point(0, 0), Point(300000, 100000)};
    const auto geometric = Contact::footprint({{&lower, .4}}, upper, .6, 0.);
    const auto reserved = Contact::footprint({{&lower, .4}}, upper, .6, .002);
    REQUIRE(geometric.completed);
    REQUIRE(reserved.completed);
    REQUIRE(geometric.segments.size() == 1);
    REQUIRE(reserved.segments.size() == 1);
    CHECK(geometric.segments.front().line_contacts.size() == 1);
    CHECK(reserved.segments.front().line_contacts.empty());
    CHECK_FALSE(check_contact(upper, reserved).passed());
}

TEST_CASE("Bridge aware zero-width collinearity is not robust to coordinate rounding", "[BridgeAwareCubic][Contact]")
{
    auto lower = bead({{0., 0.}, {1., 0.}}, .4f, .4f);
    auto upper = bead({{0., 0.}, {3., 1.}}, .4f, .4f);
    lower.polyline.points = {Point(50001, 16667), Point(150003, 50001)};
    upper.polyline.points = {Point(0, 0), Point(300000, 100000)};
    const auto geometric = Contact::footprint({{&lower, .4}}, upper, .8, 0.);
    const auto reserved = Contact::footprint({{&lower, .4}}, upper, .8, .002);
    REQUIRE(geometric.segments.size() == 1);
    REQUIRE(reserved.segments.size() == 1);
    CHECK(geometric.segments.front().line_contacts.size() == 1);
    CHECK(reserved.segments.front().line_contacts.empty());
}

TEST_CASE("Bridge aware physical walls retain a real near-collinear shared vertex", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{-1., -1.}, {5., -1.}, {10., -.99999},
                             {10., 1.}, {-1., 1.}, {-1., -1.}});
    const auto upper = bead({{0., 0.}, {9., 0.}});
    const auto contact = Contact::footprint({{&lower, .2}}, upper, .4, .0001);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    const ExPolygons domain{ExPolygon(Polygon({Point::new_scale(-2., -2.), Point::new_scale(11., -2.),
                                                Point::new_scale(11., 2.), Point::new_scale(-2., 2.)}))};
    const ExPolygons demand{ExPolygon(Polygon({Point::new_scale(0., -.1), Point::new_scale(9., -.1),
                                                Point::new_scale(9., .1), Point::new_scale(0., .1)}))};
    const auto result = Validation::validate(domain, contact.physical_footprint, {}, demand, 20., .0001);
    REQUIRE(result.completed);
    REQUIRE(result.void_cells.size() == 1);
    CHECK(result.void_cells.front().closed);
    CHECK(result.passed());
}

TEST_CASE("Bridge aware contact correspondence preserves repeated original vertices", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{5., -1.}, {5., 1.}}, .4f, .4f);
    const auto upper = bead({{0., 0.}, {5., 0.}, {5., 0.}, {5., 0.}, {10., 0.}});
    auto contact = Contact::footprint({{&lower, .4}}, upper, .6, 0.);
    REQUIRE(contact.segments.size() == 2);
    CHECK(contact.segments[0].segment_index == 0);
    CHECK(contact.segments[1].segment_index == 3);
    const auto valid = check_contact(upper, contact);
    REQUIRE(valid.input_valid);
    REQUIRE(valid.unsupported_runs.size() == 2);
    CHECK(valid.unsupported_runs[0].kind == Validation::UnsupportedRunKind::OneSidedOverhang);
    CHECK(valid.unsupported_runs[1].kind == Validation::UnsupportedRunKind::OneSidedOverhang);
    contact.segments[1].segment_index = 1;
    CHECK_FALSE(check_contact(upper, contact).input_valid);
}

TEST_CASE("Bridge aware contact bounds qualified line-event output", "[BridgeAwareCubic][Contact]")
{
    std::vector<ExtrusionPath> lower;
    lower.reserve(400);
    for (size_t i = 0; i < 400; ++i) {
        const double x = .1 + .002 * double(i);
        lower.push_back(bead({{x, -1.}, {x, 1.}}, .4f, .4f));
    }
    std::vector<Contact::Bead> deposited;
    for (const auto &path : lower)
        deposited.push_back({&path, .4});
    auto upper = bead({{0., 0.}, {1., 0.}});
    upper.polyline.points.clear();
    for (size_t i = 0; i <= 1000; ++i)
        upper.polyline.points.push_back(Point::new_scale(i % 2 == 0 ? 0. : 1., 0.));
    const auto contact = Contact::footprint(deposited, upper, .6, .0001);
    CHECK(contact.input_valid);
    CHECK_FALSE(contact.completed);
    CHECK_FALSE(contact.failure_reason.empty());
    CHECK(contact.failure_reason.find("vertex allocation limit") != std::string::npos);
}

TEST_CASE("Bridge aware near-perpendicular thin sections retain robust finite line anchors", "[BridgeAwareCubic][Contact]")
{
    const float lower_width = GENERATE(.4f, .40127f);
    const auto left = bead({{-1., 0.}, {1., 0.}});
    const auto right = bead({{9., 0.}, {11., 0.}});
    auto first = bead({{2., -1.}, {2., 1.}}, lower_width, .4f);
    auto second = bead({{8., -1.}, {8., 1.}}, lower_width, .4f);
    // Integer rounding makes these nearly, rather than exactly, perpendicular.
    first.polyline.points = {Point(200000, -100000), Point(200001, 100000)};
    second.polyline.points = {Point(800000, -100000), Point(800001, 100000)};
    auto upper = bead({{0., 0.}, {10., 0.}});
    upper.polyline.points = {Point(0, 0), Point(1000000, 0)};
    const auto contact = Contact::footprint({{&left, .4}, {&right, .4}, {&first, .4}, {&second, .4}},
                                           upper, .6, .00216);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    REQUIRE(contact.segments.size() == 1);
    CHECK(contact.segments.front().line_contacts.size() >= 2);
    const auto result = check_contact(upper, contact, 6.1);
    REQUIRE_FALSE(result.unsupported_runs.empty());
    const auto longest = std::max_element(result.unsupported_runs.begin(), result.unsupported_runs.end(),
        [](const Validation::UnsupportedRun &a, const Validation::UnsupportedRun &b) { return a.length_mm < b.length_mm; });
    CHECK(longest->length_mm - longest->uncertainty_mm == Approx(6.));
    CHECK(longest->uncertainty_mm > .00432);
    CHECK(result.passed());
    // Surviving anchors may move apart. Their nominal six-millimetre spacing
    // alone cannot certify a gap only three microns larger.
    CHECK_FALSE(check_contact(upper, contact, 6.003).passed());
}

TEST_CASE("Bridge aware contact bounds cumulative rounding along a multi-segment air run", "[BridgeAwareCubic][Contact]")
{
    auto upper = bead({{0., 0.}, {1., 1.}});
    upper.polyline.points.clear();
    Contact::PathContact contact;
    contact.coordinate_uncertainty_mm = .001;
    for (size_t i = 0; i <= 100; ++i) {
        upper.polyline.points.emplace_back(i % 2 == 0 ? 0 : 100000, coord_t(i * 100000));
        if (i < 100)
            contact.segments.push_back({i, {}, {}});
    }
    contact.segments.front().line_contacts.push_back({0, 1, 0, 1});
    contact.segments.back().line_contacts.push_back({1, 1, 1, 1});
    const double nominal = 100. * std::sqrt(2.);
    const auto result = check_contact(upper, contact, nominal + .1);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().uncertainty_mm == Approx(.2));
    CHECK(result.unsupported_runs.front().length_mm == Approx(nominal + .2));
    CHECK(result.unsupported_runs.front().exceeds_gap);
    CHECK_FALSE(result.passed());
}

TEST_CASE("Bridge aware contacts-only footprints preserve exact contact certificates", "[BridgeAwareCubic][Contact]")
{
    const double tolerance = GENERATE(0., .0001);
    const auto positive_section = bead({{-1., 0.}, {4., 0.}, {4., 2.}, {11., 2.}});
    const auto first_line = bead({{2., -1.}, {2., 1.}}, .4f, .4f);
    const auto second_line = bead({{3., 1.}, {5., 1.}}, .4f, .4f);
    const auto third_line = bead({{8., 1.}, {8., 3.}}, .4f, .4f);
    const auto upper = bead({{0., 0.}, {4., 0.}, {4., 0.}, {4., 2.}, {10., 2.}});
    const std::vector<Contact::Bead> deposited{{&positive_section, .4}, {&first_line, .4},
                                              {&second_line, .4}, {&third_line, .4}};
    // The existing four-argument entry point must continue to produce Full.
    const auto full = Contact::footprint(deposited, upper, .6, tolerance);
    const auto contacts_only = Contact::footprint(deposited, upper, .6, tolerance,
                                                 Contact::FootprintMode::ContactsOnly);
    REQUIRE(full.input_valid);
    REQUIRE(full.completed);
    REQUIRE(contacts_only.input_valid);
    REQUIRE(contacts_only.completed);
    CHECK_FALSE(full.resource_limited);
    CHECK_FALSE(contacts_only.resource_limited);
    CHECK(contacts_only.contact_z == full.contact_z);
    CHECK(contacts_only.coordinate_uncertainty_mm == full.coordinate_uncertainty_mm);
    CHECK(contacts_only.failure_reason == full.failure_reason);
    CHECK_FALSE(full.physical_footprint.empty());
    CHECK(full.physical_lines.size() >= 3);
    CHECK(contacts_only.physical_footprint.empty());
    CHECK(contacts_only.physical_lines.empty());

    REQUIRE(full.segments.size() == 3);
    REQUIRE(contacts_only.segments.size() == full.segments.size());
    const size_t expected_indices[] = {0, 2, 3};
    size_t polygon_count = 0, line_count = 0;
    const auto check_points = [](const Points &expected, const Points &actual) {
        REQUIRE(actual.size() == expected.size());
        for (size_t i = 0; i < expected.size(); ++i) {
            CHECK(actual[i].x() == expected[i].x());
            CHECK(actual[i].y() == expected[i].y());
        }
    };
    for (size_t i = 0; i < full.segments.size(); ++i) {
        const auto &expected = full.segments[i];
        const auto &actual = contacts_only.segments[i];
        CHECK(expected.segment_index == expected_indices[i]);
        CHECK(actual.segment_index == expected.segment_index);
        REQUIRE(actual.centerline_support.size() == expected.centerline_support.size());
        polygon_count += expected.centerline_support.size();
        for (size_t j = 0; j < expected.centerline_support.size(); ++j) {
            const auto &expected_polygon = expected.centerline_support[j];
            const auto &actual_polygon = actual.centerline_support[j];
            check_points(expected_polygon.contour.points, actual_polygon.contour.points);
            REQUIRE(actual_polygon.holes.size() == expected_polygon.holes.size());
            for (size_t k = 0; k < expected_polygon.holes.size(); ++k)
                check_points(expected_polygon.holes[k].points, actual_polygon.holes[k].points);
        }
        REQUIRE(actual.line_contacts.size() == expected.line_contacts.size());
        line_count += expected.line_contacts.size();
        for (size_t j = 0; j < expected.line_contacts.size(); ++j) {
            const auto &expected_line = expected.line_contacts[j];
            const auto &actual_line = actual.line_contacts[j];
            CHECK(actual_line.begin_numerator == expected_line.begin_numerator);
            CHECK(actual_line.begin_denominator == expected_line.begin_denominator);
            CHECK(actual_line.end_numerator == expected_line.end_numerator);
            CHECK(actual_line.end_denominator == expected_line.end_denominator);
            CHECK(actual_line.position_uncertainty_mm == expected_line.position_uncertainty_mm);
        }
    }
    CHECK(polygon_count > 0);
    CHECK(line_count >= 3);
}

TEST_CASE("Bridge aware contact footprint exhausts a shared budget across calls", "[BridgeAwareCubic][Contact]")
{
    const auto mode = GENERATE(Contact::FootprintMode::Full, Contact::FootprintMode::ContactsOnly);
    const auto lower = bead({{-1., 0.}, {11., 0.}});
    const auto upper = bead({{0., 0.}, {5., 0.}, {5., 0.}, {10., 0.}});
    const std::vector<Contact::Bead> deposited{{&lower, .2}};
    Validation::WorkBudget probe;
    const size_t initial_work = probe.remaining_work;
    const auto measured = Contact::footprint(deposited, upper, .4, .0001, mode, &probe);
    REQUIRE(measured.input_valid);
    REQUIRE(measured.completed);
    REQUIRE(check_contact(upper, measured).passed());
    const size_t one_call = initial_work - probe.remaining_work;
    REQUIRE(one_call > 0);

    // Permit one successful call, then require the same shared allowance to
    // stop the next call rather than silently resetting its work budget.
    Validation::WorkBudget shared{one_call + one_call / 2};
    const size_t shared_initial = shared.remaining_work;
    const auto first = Contact::footprint(deposited, upper, .4, .0001, mode, &shared);
    REQUIRE(first.input_valid);
    REQUIRE(first.completed);
    CHECK_FALSE(first.resource_limited);
    CHECK(shared.remaining_work < one_call);
    const auto second = Contact::footprint(deposited, upper, .4, .0001, mode, &shared);
    CHECK(second.input_valid);
    CHECK_FALSE(second.completed);
    CHECK(second.resource_limited);
    CHECK(shared.remaining_work == 0);
    CHECK_FALSE(second.failure_reason.empty());
    size_t charged = 0;
    for (size_t count : shared.charged_work) charged += count;
    CHECK(charged + shared.unspent_at_limit == shared_initial);
    CHECK(shared.unspent_at_limit < shared.rejected_request);
    CHECK(second.failure_reason.find("deposited_paths=1") != std::string::npos);
    CHECK(second.failure_reason.find("prepared_lower_sections=1") != std::string::npos);

    const auto rejected = check_contact(upper, second);
    CHECK(rejected.input_valid);
    CHECK_FALSE(rejected.completed);
    CHECK(rejected.resource_limited);
    CHECK_FALSE(rejected.passed());
}

TEST_CASE("Bridge aware stationary lower deposition contributes no support evidence", "[BridgeAwareCubic][Contact]")
{
    // Acceptance stack clipping leaves a genuine 0.00026 mm source edge.
    // The exporter emits E for it, but both serialized XY endpoints coincide.
    auto stationary = bead({{0., 0.}, {1., 0.}}, .4f, .2f);
    stationary.polyline.points = {Point(-1301768, 481539), Point(-1301794, 481539)};
    stationary.polyline = BridgeAwareCubic::SerializedXY{}.emitted_polyline(stationary.polyline);
    REQUIRE(stationary.polyline.size() == 2);
    REQUIRE(stationary.polyline.first_point() == stationary.polyline.last_point());

    const auto lower = bead({{-1., 0.}, {11., 0.}});
    const auto upper = bead({{0., 0.}, {10., 0.}});
    // A later stationary deposit must not select a higher contact plane and
    // thereby hide the real, usable lower bead at Z=0.2.
    const auto supported = Contact::footprint({{&lower, .2}, {&stationary, .3}}, upper, .4, .0001);
    REQUIRE(supported.input_valid);
    REQUIRE(supported.completed);
    CHECK(supported.contact_z == Approx(.2));
    CHECK(check_contact(upper, supported).passed());

    const auto empty = Contact::footprint({{&stationary, .3}}, upper, .4, .0001);
    REQUIRE(empty.input_valid);
    REQUIRE(empty.completed);
    CHECK(empty.physical_footprint.empty());
    CHECK(empty.physical_lines.empty());
    REQUIRE(empty.segments.size() == 1);
    CHECK(empty.segments.front().centerline_support.empty());
    CHECK(empty.segments.front().line_contacts.empty());
    const auto floating = check_contact(upper, empty);
    REQUIRE(floating.unsupported_runs.size() == 1);
    CHECK(floating.unsupported_runs.front().kind == Validation::UnsupportedRunKind::Floating);
    CHECK_FALSE(floating.passed());

    // Upper E-only deposition has no supported finite-path model here.
    const auto invalid_upper = Contact::footprint({{&lower, .2}}, stationary, .4, .0001);
    CHECK_FALSE(invalid_upper.input_valid);
    CHECK(invalid_upper.failure_reason.find("no positive-length XY segment") != std::string::npos);
    // Exclusion is not permission to accept malformed lower extrusion data.
    stationary.mm3_per_mm = 0.;
    const auto invalid_lower = Contact::footprint({{&lower, .2}, {&stationary, .3}}, upper, .4, .0001);
    CHECK_FALSE(invalid_lower.input_valid);
    CHECK(invalid_lower.failure_reason.find("extrusion amount") != std::string::npos);
}

TEST_CASE("Bridge aware diagonal contact lookup omits provably disjoint masks", "[BridgeAwareCubic][Contact]")
{
    const auto mode = GENERATE(Contact::FootprintMode::Full, Contact::FootprintMode::ContactsOnly);
    std::vector<ExtrusionPath> lower;
    lower.reserve(201);
    for (size_t i = 0; i < 200; ++i) {
        const double shift = .5 * (double(i) - 100.);
        lower.push_back(bead({{-40., -40. + shift}, {40., 40. + shift}}));
    }
    // Retain a second real, close parallel contact as well as the aligned one.
    lower.push_back(bead({{-40., -39.75}, {40., 40.25}}));
    std::vector<Contact::Bead> deposited;
    for (const auto &path : lower) deposited.push_back({&path, .2});
    auto upper = bead({{-30., -30.}, {30., 30.}});
    upper.polyline.points.clear();
    for (size_t i = 0; i <= 400; ++i)
        upper.polyline.points.emplace_back(i % 2 == 0 ? -3000000 : 3000000,
                                          i % 2 == 0 ? -3000000 : 3000000);

    // All long diagonal boxes overlap. Building masks for every pair exceeds
    // the unchanged 200,000-vertex limit despite only two true nearby beads.
    const auto contact = Contact::footprint(deposited, upper, .4, .0001, mode);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    CHECK_FALSE(contact.resource_limited);
    REQUIRE(contact.segments.size() == 400);
    for (const auto &segment : contact.segments) {
        CHECK(segment.centerline_support.size() == 2);
        CHECK(segment.line_contacts.size() <= 2);
    }
    CHECK(check_contact(upper, contact).passed());
    if (mode == Contact::FootprintMode::Full)
        CHECK(contact.physical_footprint.size() == lower.size());
    else
        CHECK(contact.physical_footprint.empty());
}

TEST_CASE("Bridge aware normal projection retains crossings and near contact", "[BridgeAwareCubic][Contact]")
{
    const auto upper = bead({{-5., -5.}, {5., 5.}});
    SECTION("Crossing finite sections") {
        const auto lower = bead({{-10., 10.}, {10., -10.}});
        const auto contact = Contact::footprint({{&lower, .2}}, upper, .4, .0001);
        REQUIRE(contact.input_valid);
        REQUIRE(contact.completed);
        REQUIRE(contact.segments.size() == 1);
        CHECK_FALSE(contact.segments.front().centerline_support.empty());
        CHECK_FALSE(contact.segments.front().line_contacts.empty());
    }
    SECTION("Parallel sections with a small positive overlap") {
        const auto lower = bead({{-10., -9.66}, {10., 10.34}});
        const auto contact = Contact::footprint({{&lower, .2}}, upper, .4, .0001);
        REQUIRE(contact.input_valid);
        REQUIRE(contact.completed);
        CHECK(check_contact(upper, contact).passed());
    }
    SECTION("Parallel sections just beyond their physical reach") {
        const auto lower = bead({{-10., -9.64}, {10., 10.36}});
        const auto contact = Contact::footprint({{&lower, .2}}, upper, .4, .0001);
        REQUIRE(contact.input_valid);
        REQUIRE(contact.completed);
        REQUIRE(contact.segments.size() == 1);
        CHECK(contact.segments.front().centerline_support.empty());
        CHECK(contact.segments.front().line_contacts.empty());
        CHECK_FALSE(check_contact(upper, contact).passed());
    }
}

TEST_CASE("Bridge aware shared work diagnostics account only for charged operations", "[BridgeAwareCubic][Contact]")
{
    const auto mode = GENERATE(Contact::FootprintMode::Full, Contact::FootprintMode::ContactsOnly);
    const auto lower = bead({{-1., 0.}, {11., 0.}});
    const auto upper = bead({{0., 0.}, {5., 0.}, {10., 0.}});
    Validation::WorkBudget budget;
    const size_t initial = budget.remaining_work;
    const auto contact = Contact::footprint({{&lower, .2}}, upper, .4, 0., mode, &budget);
    REQUIRE(contact.completed);
    REQUIRE(contact.input_valid);
    const auto total_charged = [](const Validation::WorkBudget &work) {
        size_t total = 0;
        for (size_t count : work.charged_work) total += count;
        return total;
    };
    CHECK(total_charged(budget) == initial - budget.remaining_work);
    CHECK(budget.charged_work[size_t(Validation::WorkCategory::ContactInput)] > 0);
    CHECK(budget.charged_work[size_t(Validation::WorkCategory::ContactSections)] > 0);
    CHECK(budget.charged_work[size_t(Validation::WorkCategory::ContactCandidates)] > 0);
    CHECK(budget.charged_work[size_t(Validation::WorkCategory::ContactMasks)] > 0);
    CHECK(budget.charged_work[size_t(Validation::WorkCategory::Validation)] == 0);
    const auto result = Validation::validate_contacts({}, {}, {upper.polyline},
        {contact_demand()}, 1., 0., {contact}, &budget);
    REQUIRE(result.passed());
    CHECK(budget.charged_work[size_t(Validation::WorkCategory::Validation)] > 0);
    CHECK(total_charged(budget) == initial - budget.remaining_work);

    // An operation larger than the remaining allowance is never charged as
    // executed work. Preserve both its request and the unspent remainder.
    Validation::WorkBudget limited{1};
    const auto incomplete = Contact::footprint({{&lower, .2}}, upper, .4, 0., mode, &limited);
    REQUIRE(incomplete.input_valid);
    CHECK_FALSE(incomplete.completed);
    CHECK(incomplete.resource_limited);
    CHECK(limited.remaining_work == 0);
    CHECK(total_charged(limited) == 0);
    CHECK(limited.unspent_at_limit == 1);
    CHECK(limited.rejected_request == upper.polyline.size());
    CHECK(incomplete.failure_reason.find("contact_input=0") != std::string::npos);
    CHECK(incomplete.failure_reason.find("unspent=1") != std::string::npos);
    CHECK(incomplete.failure_reason.find("deposited_paths=1") != std::string::npos);
    CHECK(incomplete.failure_reason.find("prepared_lower_sections=not_prepared") != std::string::npos);
}

namespace {
void same_contact(const Contact::PathContact &actual, const Contact::PathContact &expected)
{
    CHECK(actual.input_valid == expected.input_valid);
    CHECK(actual.completed == expected.completed);
    CHECK(actual.resource_limited == expected.resource_limited);
    CHECK(actual.failure_reason == expected.failure_reason);
    CHECK(actual.contact_z == expected.contact_z);
    CHECK(actual.coordinate_uncertainty_mm == expected.coordinate_uncertainty_mm);
    REQUIRE(actual.physical_footprint.size() == expected.physical_footprint.size());
    for (size_t i = 0; i < actual.physical_footprint.size(); ++i) {
        CHECK(actual.physical_footprint[i].contour.points == expected.physical_footprint[i].contour.points);
        REQUIRE(actual.physical_footprint[i].holes.size() == expected.physical_footprint[i].holes.size());
        for (size_t j = 0; j < actual.physical_footprint[i].holes.size(); ++j)
            CHECK(actual.physical_footprint[i].holes[j].points == expected.physical_footprint[i].holes[j].points);
    }
    REQUIRE(actual.physical_lines.size() == expected.physical_lines.size());
    for (size_t i = 0; i < actual.physical_lines.size(); ++i)
        CHECK(actual.physical_lines[i].points == expected.physical_lines[i].points);
    REQUIRE(actual.segments.size() == expected.segments.size());
    for (size_t i = 0; i < actual.segments.size(); ++i) {
        const auto &a = actual.segments[i], &e = expected.segments[i];
        CHECK(a.segment_index == e.segment_index);
        REQUIRE(a.centerline_support.size() == e.centerline_support.size());
        for (size_t j = 0; j < a.centerline_support.size(); ++j)
            CHECK(a.centerline_support[j].contour.points == e.centerline_support[j].contour.points);
        REQUIRE(a.line_contacts.size() == e.line_contacts.size());
        for (size_t j = 0; j < a.line_contacts.size(); ++j) {
            CHECK(a.line_contacts[j].begin_numerator == e.line_contacts[j].begin_numerator);
            CHECK(a.line_contacts[j].begin_denominator == e.line_contacts[j].begin_denominator);
            CHECK(a.line_contacts[j].end_numerator == e.line_contacts[j].end_numerator);
            CHECK(a.line_contacts[j].end_denominator == e.line_contacts[j].end_denominator);
            CHECK(a.line_contacts[j].position_uncertainty_mm == e.line_contacts[j].position_uncertainty_mm);
        }
    }
}
}

TEST_CASE("Bridge aware prepared lower sections exactly preserve contact outputs", "[BridgeAwareCubic][Contact]")
{
    const auto mode = GENERATE(Contact::FootprintMode::Full, Contact::FootprintMode::ContactsOnly);
    const double tolerance = GENERATE(0., .0001);
    const double upper_z = GENERATE(.4, .35, .4001);
    std::vector<ExtrusionPath> lower{
        bead({{-1., 0.}, {5., 0.}}), bead({{11., 0.}, {5., 0.}}),
        bead({{2., -2.}, {2., 2.}}, .2f, .2f),
        bead({{-5., 12.}, {15., 12.}}),
        bead({{0., 1.}, {10., 1.}}, .1f, .2f),
        bead({{0., 0.}, {0., 0.}})};
    std::vector<Contact::Bead> deposited;
    for (const auto &path : lower) deposited.push_back({&path, .2});
    const auto prepared = Contact::prepare_lower(deposited, upper_z, .2f);
    REQUIRE(prepared);
    for (auto upper : {bead({{0., 0.}, {5., 0.}, {5., 0.}, {10., 0.}}),
                       bead({{5., 3.}, {5., -3.}}, .6f),
                       bead({{10., 0.}, {0., 0.}}, .2f)}) {
        const auto expected = Contact::footprint(deposited, upper, upper_z, tolerance, mode);
        REQUIRE(expected.input_valid);
        REQUIRE(expected.completed);
        same_contact(Contact::footprint(*prepared, upper, upper_z, tolerance, mode), expected);
    }
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const auto owned = Contact::footprint(*prepared, upper, upper_z, tolerance, mode);
    lower.clear(); // The prepared entry must own every section and corner.
    deposited.clear();
    same_contact(Contact::footprint(*prepared, upper, upper_z, tolerance, mode), owned);
}

TEST_CASE("Bridge aware prepared contact keys include elevation and exact float height", "[BridgeAwareCubic][Contact]")
{
    const auto first = bead({{-1., 0.}, {11., 0.}}, .8f, .5f);
    const auto later = bead({{5., -2.}, {5., 2.}}, 1.f, .75f);
    const std::vector<Contact::Bead> lower{{&first, .25}, {&later, .5}};
    const auto upper_a = bead({{0., 0.}, {10., 0.}}, .8f, .5f);
    const auto upper_b = bead({{0., 0.}, {10., 0.}}, 1.5f, 1.25f);
    const auto prepared_a = Contact::prepare_lower(lower, .5, upper_a.height);
    const auto prepared_b = Contact::prepare_lower(lower, 1., upper_b.height);
    const auto a = Contact::footprint(*prepared_a, upper_a, .5, 0.);
    const auto b = Contact::footprint(*prepared_b, upper_b, 1., 0.);
    REQUIRE(a.completed);
    REQUIRE(b.completed);
    REQUIRE(a.input_valid);
    REQUIRE(b.input_valid);
    CHECK(a.contact_z == b.contact_z);
    CHECK(a.physical_footprint.size() == 1);
    CHECK(b.physical_footprint.size() == 2);
    CHECK_FALSE(Contact::footprint(*prepared_a, upper_b, 1., 0.).input_valid);
    auto adjacent_height = upper_a;
    adjacent_height.height = std::nextafter(upper_a.height, 1.f);
    CHECK_FALSE(Contact::footprint(*prepared_a, adjacent_height, .5, 0.).input_valid);
    CHECK_FALSE(Contact::footprint(*prepared_a, upper_a, std::nextafter(.5, 1.), 0.).input_valid);
    auto invalid_upper = upper_a;
    invalid_upper.mm3_per_mm = 0.;
    CHECK_FALSE(Contact::footprint(*prepared_a, invalid_upper, .5, 0.).input_valid);
    same_contact(Contact::footprint(*prepared_a, upper_a, .5, 0.), a);
}

TEST_CASE("Bridge aware preparation validates excluded lower input and never publishes partial evidence", "[BridgeAwareCubic][Contact]")
{
    const auto good = bead({{-1., 0.}, {11., 0.}});
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const int excluded = GENERATE(0, 1, 2, 3);
    auto bad = bead({{1., 0.}, {2., 0.}});
    double top = .2;
    if (excluded == 0) top = .8; // Future material is still validated.
    if (excluded == 1) top = -.8;
    if (excluded == 2) bad.width = .1f;
    if (excluded == 3) bad.polyline.points.back() = bad.polyline.points.front();
    bad.mm3_per_mm = std::numeric_limits<double>::quiet_NaN();
    const auto prepared = Contact::prepare_lower({{&good, .2}, {&bad, top}}, .4, upper.height);
    REQUIRE(prepared);
    const auto rejected = Contact::footprint(*prepared, upper, .4, 0.);
    CHECK_FALSE(rejected.input_valid);
    CHECK(rejected.physical_footprint.empty());
    CHECK(rejected.physical_lines.empty());
    CHECK(rejected.segments.empty());
    CHECK(rejected.failure_reason.find("extrusion amount") != std::string::npos);

    Validation::WorkBudget limited{1};
    const auto incomplete = Contact::prepare_lower({{&good, .2}}, .4, upper.height, &limited);
    const auto no_partial = Contact::footprint(*incomplete, upper, .4, 0.);
    CHECK_FALSE(no_partial.completed);
    CHECK(no_partial.resource_limited);
    CHECK(no_partial.physical_footprint.empty());
    CHECK(no_partial.segments.empty());

    Validation::WorkBudget measured;
    Contact::prepare_lower({{&good, .2}}, .4, upper.height, &measured);
    Validation::WorkBudget late_limit{50000000 - measured.remaining_work - 1};
    const auto partly_constructed = Contact::prepare_lower({{&good, .2}}, .4, upper.height, &late_limit);
    const auto discarded = Contact::footprint(*partly_constructed, upper, .4, 0.);
    CHECK_FALSE(discarded.completed);
    CHECK(discarded.resource_limited);
    CHECK(discarded.physical_footprint.empty());
    CHECK(discarded.segments.empty());
}

TEST_CASE("Bridge aware prepared contact charges preparation once and each query", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{-1., 0.}, {11., 0.}});
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const auto mode = GENERATE(Contact::FootprintMode::Full, Contact::FootprintMode::ContactsOnly);
    Validation::WorkBudget measured;
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height, &measured);
    const size_t preparation_work = 50000000 - measured.remaining_work;
    const size_t before_query = measured.remaining_work;
    const auto expected = Contact::footprint(*prepared, upper, .4, 0., mode, &measured);
    REQUIRE(expected.input_valid);
    REQUIRE(expected.completed);
    const size_t query_work = before_query - measured.remaining_work;
    REQUIRE(preparation_work > 0);
    REQUIRE(query_work > 0);
    Validation::WorkBudget raw_budget;
    same_contact(Contact::footprint({{&lower, .2}}, upper, .4, 0., mode, &raw_budget), expected);
    CHECK(50000000 - raw_budget.remaining_work == preparation_work + query_work);
    same_contact(Contact::footprint(*prepared, upper, .4, 0., mode, &measured), expected);
    CHECK(50000000 - measured.remaining_work == preparation_work + 2 * query_work);

    Validation::WorkBudget limited{preparation_work + 2 * query_work - 1};
    const auto cached = Contact::prepare_lower({{&lower, .2}}, .4, upper.height, &limited);
    REQUIRE(Contact::footprint(*cached, upper, .4, 0., mode, &limited).completed);
    const auto exhausted = Contact::footprint(*cached, upper, .4, 0., mode, &limited);
    CHECK_FALSE(exhausted.completed);
    CHECK(exhausted.resource_limited);
    CHECK(limited.remaining_work == 0);
}

TEST_CASE("Bridge aware prepared residents share the query allocation bound", "[BridgeAwareCubic][Contact]")
{
    // Each disjoint section owns its axis and four real corners. Preparation
    // fits, but retaining it together with a large query exceeds 200k units.
    std::vector<ExtrusionPath> paths;
    paths.reserve(20000);
    for (size_t i = 0; i < 20000; ++i)
        paths.push_back(bead({{double(i % 200) * 2., double(i / 200) * 2.},
                              {double(i % 200) * 2. + 1., double(i / 200) * 2.}}));
    std::vector<Contact::Bead> lower;
    for (const auto &path : paths) lower.push_back({&path, .2});
    const auto prepared = Contact::prepare_lower(lower, .4, .2f);
    auto small = bead({{-10., -10.}, {-9., -10.}});
    const auto admitted = Contact::footprint(*prepared, small, .4, 0., Contact::FootprintMode::ContactsOnly);
    REQUIRE(admitted.input_valid);
    REQUIRE(admitted.completed);
    auto large = small;
    large.polyline.points.resize(40001, large.polyline.points.back());
    const auto rejected = Contact::footprint(*prepared, large, .4, 0., Contact::FootprintMode::ContactsOnly);
    CHECK_FALSE(rejected.completed);
    CHECK(rejected.resource_limited);
    CHECK(rejected.failure_reason.find("vertex allocation limit") != std::string::npos);
}

TEST_CASE("Bridge aware cached queries preserve combined excluded-input admission", "[BridgeAwareCubic][Contact]")
{
    auto ignored = bead({{0., 0.}, {0., 0.}});
    ignored.polyline.points.resize(199999, ignored.polyline.points.front());
    const auto upper = bead({{0., 0.}, {1., 0.}});
    const auto prepared = Contact::prepare_lower({{&ignored, .2}}, .4, upper.height);
    const auto cached = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    const auto uncached = Contact::footprint({{&ignored, .2}}, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    CHECK_FALSE(cached.completed);
    CHECK(cached.resource_limited);
    CHECK_FALSE(uncached.completed);
    CHECK(uncached.resource_limited);
    CHECK(cached.failure_reason.find("vertex allocation limit") != std::string::npos);
    CHECK(uncached.failure_reason.find("vertex allocation limit") != std::string::npos);
}

TEST_CASE("Bridge aware indexed sections preserve independent contact geometry and ordering", "[BridgeAwareCubic][Contact]")
{
    const auto mode = GENERATE(Contact::FootprintMode::Full, Contact::FootprintMode::ContactsOnly);
    const int direction = GENERATE(-1, 0, 1);
    const int query = GENERATE(0, 1, 2);
    std::vector<ExtrusionPath> lower;
    lower.reserve(97);
    for (int i = -48; i <= 48; ++i) {
        const double y = .3 * i;
        lower.push_back(bead({{-10., y - 10. * direction}, {10., y + 10. * direction}},
                             i % 3 == 0 ? .2f : .45f, .2f));
        lower.back().polyline.points = {Point(-1000000, 30000 * i - 1000000 * direction),
                                         Point(1000000, 30000 * i + 1000000 * direction)};
        if (i % 2 == 0) std::reverse(lower.back().polyline.points.begin(), lower.back().polyline.points.end());
    }
    const auto upper = query == 0 ? bead({{-8., -8. * direction}, {8., 8. * direction}}) :
                       query == 1 ? bead({{10., -15.}, {10., 15.}}) :
                                    bead({{10.125, -15.}, {10.125, 15.}});
    std::vector<Contact::Bead> deposited;
    for (const auto &path : lower) deposited.push_back({&path, .2});
    const auto prepared = Contact::prepare_lower(deposited, .4, upper.height);
    const auto actual = Contact::footprint(*prepared, upper, .4, 0., mode);
    REQUIRE(actual.input_valid);
    REQUIRE(actual.completed);
    // Each single-section call uses the original direct scan. Input is in
    // canonical section-key order; the independent output concatenation must
    // equal the indexed result, including irrelevant masks and finite lines.
    Contact::PathContact expected;
    for (size_t i = 0; i < lower.size(); ++i) {
        auto single = Contact::footprint({{&lower[i], .2}}, upper, .4, 0., mode);
        REQUIRE(single.input_valid);
        REQUIRE(single.completed);
        if (i == 0) {
            expected = std::move(single);
            continue;
        }
        expected.physical_footprint.insert(expected.physical_footprint.end(),
            single.physical_footprint.begin(), single.physical_footprint.end());
        expected.physical_lines.insert(expected.physical_lines.end(), single.physical_lines.begin(), single.physical_lines.end());
        REQUIRE(expected.segments.size() == single.segments.size());
        for (size_t j = 0; j < expected.segments.size(); ++j) {
            auto &target = expected.segments[j];
            const auto &source = single.segments[j];
            target.centerline_support.insert(target.centerline_support.end(), source.centerline_support.begin(), source.centerline_support.end());
            target.line_contacts.insert(target.line_contacts.end(), source.line_contacts.begin(), source.line_contacts.end());
        }
    }
    same_contact(actual, expected);
}

TEST_CASE("Bridge aware diagonal section index bounds work without changing contacts", "[BridgeAwareCubic][Contact]")
{
    const bool reversed = GENERATE(false, true);
    std::vector<ExtrusionPath> lower;
    lower.reserve(4096);
    for (int i = -2048; i < 2048; ++i) {
        lower.push_back(bead({{-40., -40. + .5 * i}, {40., 40. + .5 * i}}));
        lower.back().polyline.points = {Point(-4000000, -4000000 + 50000 * i),
                                         Point(4000000, 4000000 + 50000 * i)};
    }
    std::vector<Contact::Bead> deposited;
    for (const auto &path : lower) deposited.push_back({&path, .2});
    auto upper = bead({{-30., -30.}, {30., 30.}});
    upper.polyline.points.clear();
    for (size_t i = 0; i <= 1000; ++i)
        upper.polyline.points.emplace_back((i % 2 == 0) != reversed ? -3000000 : 3000000,
                                          (i % 2 == 0) != reversed ? -3000000 : 3000000);
    // Every diagonal XY box overlaps the query for hundreds of lower strands.
    // The previous scan visits 4,096,000 pairs before any narrow predicates.
    // The node diagonal bound is exactly an existing upper-normal rejection.
    Validation::WorkBudget budget{1000000};
    const auto prepared = Contact::prepare_lower(deposited, .4, upper.height, &budget);
    const auto result = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, &budget);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    CHECK_FALSE(result.resource_limited);
    REQUIRE(result.segments.size() == 1000);
    for (const auto &segment : result.segments) {
        REQUIRE(segment.centerline_support.size() == 1);
        CHECK(segment.line_contacts.size() <= 1);
    }
    CHECK(budget.remaining_work > 0);
}

TEST_CASE("Bridge aware section indexes retain invalid input and aggregate limits", "[BridgeAwareCubic][Contact]")
{
    std::vector<ExtrusionPath> lower;
    lower.reserve(100);
    for (size_t i = 0; i < 100; ++i)
        lower.push_back(bead({{0., double(i)}, {10., double(i)}}));
    std::vector<Contact::Bead> deposited;
    for (const auto &path : lower) deposited.push_back({&path, .2});
    const auto upper = bead({{1., 0.}, {9., 0.}});
    Validation::WorkBudget measured;
    const auto prepared = Contact::prepare_lower(deposited, .4, upper.height, &measured);
    REQUIRE(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly).completed);
    const size_t preparation_work = 50000000 - measured.remaining_work;
    REQUIRE(preparation_work > 1);
    Validation::WorkBudget exhausted{preparation_work - 1};
    const auto incomplete = Contact::prepare_lower(deposited, .4, upper.height, &exhausted);
    const auto rejected = Contact::footprint(*incomplete, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    CHECK_FALSE(rejected.completed);
    CHECK(rejected.resource_limited);
    CHECK(rejected.segments.empty());
    CHECK(exhausted.remaining_work == 0);
    lower.back().mm3_per_mm = 0.;
    deposited.back().print_z = .8; // Excluded future input is still validated.
    const auto invalid = Contact::prepare_lower(deposited, .4, upper.height);
    CHECK_FALSE(Contact::footprint(*invalid, upper, .4, 0., Contact::FootprintMode::ContactsOnly).input_valid);
}

TEST_CASE("Bridge aware exact mask cache preserves every contact output", "[BridgeAwareCubic][Contact]")
{
    const auto mode = GENERATE(Contact::FootprintMode::Full, Contact::FootprintMode::ContactsOnly);
    const double tolerance = GENERATE(0., .0001);
    const float lower_width = GENERATE(.45f, .2f, .201f);
    const auto lower = bead({{-2., 0.}, {12., 0.}}, lower_width, .2f);
    const auto crossing = bead({{5., -2.}, {5., 2.}}, lower_width, .2f);
    const auto prepared = Contact::prepare_lower({{&lower, .2}, {&crossing, .2}}, .4, .2f);
    Contact::QueryCache cache(prepared);
    for (int repeat = 0; repeat < 2; ++repeat)
        for (auto upper : {bead({{0., 0.}, {10., 0.}}), bead({{10., 0.}, {0., 0.}}),
                           bead({{0., 0.}, {5., 0.}}), bead({{0., .125}, {10., .125}}),
                           bead({{0., 0.}, {10., 0.}}, .6f), bead({{5., -1.}, {5., 1.}})}) {
            const auto expected = Contact::footprint(*prepared, upper, .4, tolerance, mode);
            const auto actual = Contact::footprint(*prepared, upper, .4, tolerance, mode, nullptr, &cache);
            REQUIRE(expected.input_valid);
            REQUIRE(expected.completed);
            same_contact(actual, expected);
        }
    CHECK(cache.statistics().hits > 0);
    CHECK(cache.statistics().misses > 0);
    CHECK(cache.statistics().resident_units <= 8192);
}

TEST_CASE("Bridge aware mask cache keeps exact reserve and snapshot identities", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{-2., 0.}, {12., 0.}});
    auto upper = bead({{0., 0.}, {10., 0.}});
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    Contact::QueryCache cache(prepared);
    const auto query = [&](const ExtrusionPath &path, double tolerance) {
        same_contact(Contact::footprint(*prepared, path, .4, tolerance, Contact::FootprintMode::ContactsOnly, nullptr, &cache),
                     Contact::footprint(*prepared, path, .4, tolerance, Contact::FootprintMode::ContactsOnly));
    };
    query(upper, .0001);
    CHECK(cache.statistics().misses == 1);
    query(upper, .0001);
    CHECK(cache.statistics().hits == 1);
    auto shorter = bead({{0., 0.}, {5., 0.}});
    query(shorter, .0001);
    CHECK(cache.statistics().misses == 2); // Same transverse, different final reserve.
    query(upper, .0002);
    CHECK(cache.statistics().misses == 3);
    upper.width = .6f;
    query(upper, .0001);
    CHECK(cache.statistics().misses == 4);

    const auto different = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    CHECK_FALSE(Contact::footprint(*different, upper, .4, .0001, Contact::FootprintMode::ContactsOnly, nullptr, &cache).input_valid);
    const size_t hits = cache.statistics().hits;
    upper.mm3_per_mm = 0.;
    CHECK_FALSE(Contact::footprint(*prepared, upper, .4, .0001, Contact::FootprintMode::ContactsOnly, nullptr, &cache).input_valid);
    CHECK(cache.statistics().hits == hits);
    REQUIRE_THROWS_AS(Contact::QueryCache(Contact::PreparedLowerPtr{}), std::invalid_argument);
}

TEST_CASE("Bridge aware mask cache bounds eviction including empty masks", "[BridgeAwareCubic][Contact]")
{
    const bool empty_masks = GENERATE(false, true);
    const float width = empty_masks ? .2f : .45f;
    const auto left = bead({{2., -1.}, {2., 1.}}, width, .2f);
    const auto right = bead({{8., -1.}, {8., 1.}}, width, .2f);
    const auto upper = bead({{0., 0.}, {10., 0.}}, width, .2f);
    const auto prepared = Contact::prepare_lower({{&left, .2}, {&right, .2}}, .4, upper.height);
    const size_t capacity = empty_masks ? 1 : 5;
    Contact::QueryCache cache(prepared, capacity);
    for (int repeat = 0; repeat < 3; ++repeat)
        same_contact(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, nullptr, &cache),
                     Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly));
    CHECK(cache.statistics().evictions > 0);
    CHECK(cache.statistics().resident_units <= capacity);
    CHECK(cache.statistics().resident_units > 0);
}

TEST_CASE("Bridge aware cached masks charge copied output and preserve shared failures", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{-1., 0.}, {11., 0.}});
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    Contact::QueryCache cache(prepared);
    const auto expected = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    Validation::WorkBudget first, second, uncached;
    same_contact(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, &first, &cache), expected);
    same_contact(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, &second, &cache), expected);
    same_contact(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, &uncached), expected);
    CHECK(second.remaining_work > uncached.remaining_work);
    CHECK(second.remaining_work < 50000000);
    const size_t hit_work = 50000000 - second.remaining_work;
    REQUIRE(hit_work > 1);
    Validation::WorkBudget limited{hit_work - 1};
    const auto incomplete = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, &limited, &cache);
    CHECK_FALSE(incomplete.completed);
    CHECK(incomplete.resource_limited);
    CHECK(limited.remaining_work == 0);
    same_contact(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, nullptr, &cache), expected);
    // Exhaust every stage of this hit, including map comparisons. Each next
    // successful query must use its own live budget after exception unwinding.
    for (size_t allowance = 0; allowance < hit_work; ++allowance) {
        Validation::WorkBudget interrupted{allowance};
        const auto rejected = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, &interrupted, &cache);
        CHECK_FALSE(rejected.completed);
        CHECK(rejected.resource_limited);
        CHECK(interrupted.remaining_work == 0);
        same_contact(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, nullptr, &cache), expected);
    }
}

TEST_CASE("Bridge aware optional mask residency cannot reject an admissible query", "[BridgeAwareCubic][Contact]")
{
    const auto lower = bead({{-1., 0.}, {11., 0.}});
    auto upper = bead({{0., 0.}, {10., 0.}});
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    Contact::QueryCache cache(prepared);
    REQUIRE(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, nullptr, &cache).completed);
    REQUIRE(cache.statistics().resident_units == 5);
    // Eight lower input/axis/corner units, N upper input units, one segment,
    // four mask vertices and one line interval exactly fill the 200k bound.
    // The cached mask must transfer ownership before its output allocation.
    upper.polyline.points.resize(199986, upper.polyline.points.back());
    const auto expected = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    REQUIRE(expected.input_valid);
    REQUIRE(expected.completed);
    const size_t hits = cache.statistics().hits;
    same_contact(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, nullptr, &cache), expected);
    CHECK(cache.statistics().hits == hits + 1);
    CHECK(cache.statistics().evictions == 1);
    CHECK(cache.statistics().resident_units == 0);
    upper.polyline.points.push_back(upper.polyline.points.back());
    const auto rejected = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, nullptr, &cache);
    CHECK_FALSE(rejected.completed);
    CHECK(rejected.resource_limited);
}

TEST_CASE("Bridge aware physical sections equal Full material without path certificates", "[BridgeAwareCubic][Contact]")
{
    const float width = GENERATE(.45f, .2f, .201f);
    const double upper_z = GENERATE(.4, .35, .4001);
    const auto lower = bead({{-1., 0.}, {11., 0.}}, width, .2f);
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, upper_z, upper.height);
    Contact::QueryCache cache(prepared);
    auto expected = Contact::footprint(*prepared, upper, upper_z, 0.);
    expected.segments.clear();
    const auto physical = Contact::physical_sections(*prepared, nullptr, &cache);
    same_contact(physical, expected);
    CHECK_FALSE(check_contact(upper, physical).input_valid);
    Validation::WorkBudget empty{0};
    const auto limited = Contact::physical_sections(*prepared, &empty, &cache);
    if (!expected.physical_footprint.empty() || !expected.physical_lines.empty()) {
        CHECK_FALSE(limited.completed);
        CHECK(limited.resource_limited);
        CHECK(limited.physical_footprint.empty());
        CHECK(limited.physical_lines.empty());
    }
    auto invalid_lower = lower;
    invalid_lower.mm3_per_mm = 0.;
    const auto invalid = Contact::prepare_lower({{&invalid_lower, .2}}, upper_z, upper.height);
    CHECK_FALSE(Contact::physical_sections(*invalid).input_valid);
    Contact::QueryCache invalid_cache(invalid);
    same_contact(Contact::physical_sections(*invalid, nullptr, &invalid_cache), Contact::physical_sections(*invalid));
    Validation::WorkBudget no_preparation{0};
    const auto incomplete = Contact::prepare_lower({{&lower, .2}}, upper_z, upper.height, &no_preparation);
    Contact::QueryCache incomplete_cache(incomplete);
    const auto failed = Contact::physical_sections(*incomplete, nullptr, &incomplete_cache);
    CHECK_FALSE(failed.completed);
    CHECK(failed.resource_limited);
    CHECK(failed.physical_footprint.empty());
    CHECK(failed.physical_lines.empty());
    same_contact(failed, Contact::physical_sections(*incomplete));
}

TEST_CASE("Bridge aware indexed candidate scratch follows its actual lifetime", "[BridgeAwareCubic][Contact]")
{
    const bool cached_masks = GENERATE(false, true);
    std::vector<ExtrusionPath> lower;
    lower.reserve(201);
    for (int i = -100; i <= 100; ++i) {
        auto strand = bead({{-100., -50.}, {100., 50.}});
        strand.polyline.points = {Point(-10000000, -5000000 + 50000 * i),
                                  Point(10000000, 5000000 + 50000 * i)};
        lower.push_back(std::move(strand));
    }
    std::vector<Contact::Bead> deposited;
    for (const auto &strand : lower) deposited.push_back({&strand, .2});
    auto upper = bead({{-1., -.5}, {1., .5}});
    upper.polyline.points.clear();
    for (size_t i = 0; i <= 1000; ++i)
        upper.polyline.points.emplace_back(i % 2 == 0 ? -100000 : 100000,
                                          i % 2 == 0 ? -50000 : 50000);
    // Every long lower box meets this short slope-1/2 query. The exact normal
    // tests retain only the central strand. Repeated candidate arrays are
    // scratch, not 201,000 simultaneous output or resident vertices.
    const auto expected = Contact::footprint({{&lower[100], .2}}, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    REQUIRE(expected.input_valid);
    REQUIRE(expected.completed);
    REQUIRE(expected.segments.size() == 1000);
    Validation::WorkBudget budget{10000000};
    const auto prepared = Contact::prepare_lower(deposited, .4, upper.height, &budget);
    Contact::QueryCache cache(prepared);
    const auto actual = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly,
                                          &budget, cached_masks ? &cache : nullptr);
    INFO(actual.failure_reason);
    REQUIRE(actual.input_valid);
    REQUIRE(actual.completed);
    CHECK_FALSE(actual.resource_limited);
    same_contact(actual, expected);
    CHECK(budget.remaining_work > 0);
}

TEST_CASE("Bridge aware whole-path proof certifies every segment without contact masks",
          "[BridgeAwareCubic][Contact][WholePathSupport]")
{
    const bool reversed = GENERATE(false, true);
    const bool cached = GENERATE(false, true);
    std::vector<ExtrusionPath> lower{bead({{0., 0.}, {5., 0.}}), bead({{10., 0.}, {5., 0.}}),
        bead({{10., 0.}, {10., 1.}, {0., 1.}, {0., 0.}})};
    auto upper = bead({{0., 0.}, {10., 0.}, {10., 0.}, {10., 1.}, {0., 1.}, {0., 0.}});
    if (reversed) std::reverse(upper.polyline.points.begin(), upper.polyline.points.end());
    std::vector<Contact::Bead> deposited;
    for (const auto &path : lower) deposited.push_back({&path, .2});
    const auto prepared = Contact::prepare_lower(deposited, .4, upper.height);
    Contact::QueryCache cache(prepared);
    Validation::WorkBudget budget;
    const auto proof = Contact::footprint_for_support(*prepared, upper, .4, 0., &budget, cached ? &cache : nullptr);
    REQUIRE(proof.contact.input_valid);
    REQUIRE(proof.contact.completed);
    REQUIRE(proof.entire_path_supported);
    REQUIRE(proof.contact.segments.size() == 4);
    for (const auto &segment : proof.contact.segments) CHECK(segment.centerline_support.empty());
    CHECK(proof.contact.physical_footprint.empty());
    CHECK(proof.contact.physical_lines.empty());
    CHECK(budget.charged_work[size_t(Validation::WorkCategory::ContactMasks)] == 0);
    CHECK(cache.statistics().misses == 0);
    const auto checked = check_contact(upper, proof.contact);
    REQUIRE(checked.passed());
    CHECK(checked.unsupported_runs.empty());
    const auto exhaustive = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    const auto reference = check_contact(upper, exhaustive);
    REQUIRE(reference.passed());
    CHECK(reference.unsupported_runs.empty());
}

TEST_CASE("Bridge aware whole-path proof never mistakes a passing bridge for continuous support",
          "[BridgeAwareCubic][Contact][WholePathSupport]")
{
    const auto left = bead({{-1., 0.}, {2., 0.}});
    const auto right = bead({{8., 0.}, {11., 0.}});
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const auto prepared = Contact::prepare_lower({{&left, .2}, {&right, .2}}, .4, upper.height);
    const auto proof = Contact::footprint_for_support(*prepared, upper, .4, 0.);
    CHECK_FALSE(proof.entire_path_supported);
    same_contact(proof.contact, Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly));
    const auto checked = check_contact(upper, proof.contact);
    REQUIRE(checked.passed());
    REQUIRE(checked.unsupported_runs.size() == 1);
    CHECK(checked.unsupported_runs.front().kind == Validation::UnsupportedRunKind::TwoEndedBridge);
}

TEST_CASE("Bridge aware whole-path support is not a physical cell certificate",
          "[BridgeAwareCubic][Contact][WholePathSupport]")
{
    // A tangent circular bead supplies a finite contact line along the whole
    // upper strand, but no positive-area wall enclosing the model's void.
    const auto lower = bead({{0., 0.}, {10., 0.}}, .2f, .2f);
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    const auto proof = Contact::footprint_for_support(*prepared, upper, .4, 0.);
    REQUIRE(proof.entire_path_supported);
    const auto physical = Contact::physical_sections(*prepared);
    REQUIRE(physical.input_valid);
    REQUIRE(physical.completed);
    CHECK(physical.physical_footprint.empty());
    REQUIRE(physical.physical_lines.size() == 1);

    const ExPolygons demand{contact_demand()};
    const auto span = Validation::validate_contacts({}, physical.physical_footprint,
        {upper.polyline}, demand, 20., .0001, {proof.contact});
    REQUIRE(span.passed());
    CHECK(span.unsupported_runs.empty());
    const auto model = Validation::validate_contacts(demand, physical.physical_footprint,
        {upper.polyline}, demand, 20., .0001, {proof.contact});
    REQUIRE(model.input_valid);
    REQUIRE(model.completed);
    CHECK(model.unsupported_runs.empty());
    REQUIRE_FALSE(model.void_cells.empty());
    CHECK_FALSE(model.passed());
}

TEST_CASE("Bridge aware whole-path misses preserve exhaustive contact output",
          "[BridgeAwareCubic][Contact][WholePathSupport]")
{
    const int condition = GENERATE(0, 1, 2, 3, 4, 5, 6, 7);
    std::vector<ExtrusionPath> lower{bead({{0., 0.}, {10., 0.}})};
    auto upper = bead({{0., 0.}, {10., 0.}});
    double tolerance = 0., upper_z = .4;
    if (condition == 0) tolerance = .0001;
    if (condition == 1) upper.polyline.translate(Point::new_scale(0., .01));
    if (condition == 2) lower.front().polyline.points.front().x() += 1;
    if (condition == 3) upper.polyline.points.push_back(Point::new_scale(10., 1.));
    if (condition == 4) lower = {bead({{0., 0.}, {5., 0.}}), bead({{5.00001, 0.}, {10., 0.}})};
    if (condition == 5) lower.front().width = .1f; // Valid, unmodeled lower gap fill.
    if (condition == 6) lower.front().polyline.points.back() = lower.front().polyline.points.front();
    if (condition == 7) upper_z = .4001;
    std::vector<Contact::Bead> deposited;
    for (const auto &path : lower) deposited.push_back({&path, .2});
    const auto prepared = Contact::prepare_lower(deposited, upper_z, upper.height);
    Contact::QueryCache cache(prepared);
    Contact::QueryCache reference_cache(prepared);
    Validation::WorkBudget budget, reference_budget;
    const auto proof = Contact::footprint_for_support(*prepared, upper, upper_z, tolerance, &budget, &cache);
    REQUIRE_FALSE(proof.entire_path_supported);
    const auto reference = Contact::footprint(*prepared, upper, upper_z, tolerance,
        Contact::FootprintMode::ContactsOnly, &reference_budget, &reference_cache);
    same_contact(proof.contact, reference);
    // A failed probe never constructs masks, including when an earlier upper
    // segment matched and a later segment did not. Only fallback does so.
    CHECK(budget.charged_work[size_t(Validation::WorkCategory::ContactMasks)] ==
          reference_budget.charged_work[size_t(Validation::WorkCategory::ContactMasks)]);
}

TEST_CASE("Bridge aware whole-path proof follows actual serialized axes and circular line contact",
          "[BridgeAwareCubic][Contact][WholePathSupport]")
{
    const float width = GENERATE(.45f, .2f);
    const double phase = GENERATE(.00049, .00051);
    auto lower = bead({{0., 0.}, {5., .5}, {10., 0.}}, width);
    BridgeAwareCubic::SerializedXY serialization;
    serialization.shift = Vec2d(80. + phase, 80. - phase);
    serialization.plate_origin = Vec2d(80., 80.);
    lower.polyline = serialization.emitted_polyline(lower.polyline);
    auto upper = lower;
    std::reverse(upper.polyline.points.begin(), upper.polyline.points.end());
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    const auto proof = Contact::footprint_for_support(*prepared, upper, .4, 0.);
    REQUIRE(proof.entire_path_supported);
    const auto result = check_contact(upper, proof.contact);
    REQUIRE(result.passed());
    CHECK(result.unsupported_runs.empty());
    const auto exhaustive = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    CHECK(check_contact(upper, exhaustive).unsupported_runs.empty());
}

TEST_CASE("Bridge aware whole-path proof uses the fully prepared common plane",
          "[BridgeAwareCubic][Contact][WholePathSupport]")
{
    const auto aligned = bead({{0., 0.}, {10., 0.}});
    const auto higher = bead({{20., 1.}, {30., 1.}}, .45f, .05f);
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const auto prepared = Contact::prepare_lower({{&aligned, .2}, {&higher, .3}}, .4, upper.height);
    const auto proof = Contact::footprint_for_support(*prepared, upper, .4, 0.);
    REQUIRE(proof.contact.input_valid);
    REQUIRE(proof.contact.completed);
    CHECK_FALSE(proof.entire_path_supported);
    CHECK(proof.contact.contact_z == Approx(.275));
    CHECK_FALSE(check_contact(upper, proof.contact).passed());
    same_contact(proof.contact, Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly));
}

TEST_CASE("Bridge aware whole-path proof retains query and preparation guards",
          "[BridgeAwareCubic][Contact][WholePathSupport]")
{
    const int invalid = GENERATE(0, 1, 2, 3, 4, 5, 6, 7, 8);
    const auto lower = bead({{0., 0.}, {10., 0.}});
    auto excluded = bead({{100., 100.}, {110., 100.}});
    if (invalid == 4) excluded.mm3_per_mm = std::numeric_limits<double>::quiet_NaN();
    auto upper = bead({{0., 0.}, {10., 0.}});
    const auto prepared = Contact::prepare_lower({{&lower, .2}, {&excluded, .8}}, .4, upper.height);
    Contact::QueryCache cache(prepared);
    const auto other = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    Contact::QueryCache wrong_cache(other);
    double upper_z = .4, tolerance = 0.;
    if (invalid == 0) upper.mm3_per_mm = 0.;
    if (invalid == 1) upper.height = 0.;
    if (invalid == 2) upper.width = .1f;
    if (invalid == 3) {
        PathFittingData arc{};
        arc.start_point_index = 0;
        arc.end_point_index = 1;
        arc.path_type = EMovePathType::Arc_move_ccw;
        upper.polyline.fitting_result.push_back(arc);
    }
    if (invalid == 5) upper_z = std::nextafter(.4, 1.);
    if (invalid == 6) upper.height = std::nextafter(upper.height, 1.f);
    if (invalid == 7) tolerance = std::numeric_limits<double>::quiet_NaN();
    Contact::QueryCache *query_cache = invalid == 8 ? &wrong_cache : &cache;
    const auto proof = Contact::footprint_for_support(*prepared, upper, upper_z, tolerance, nullptr, query_cache);
    CHECK_FALSE(proof.entire_path_supported);
    CHECK_FALSE(proof.contact.input_valid);
    same_contact(proof.contact, Contact::footprint(*prepared, upper, upper_z, tolerance,
        Contact::FootprintMode::ContactsOnly, nullptr, query_cache));
}

TEST_CASE("Bridge aware whole-path proof preserves finite budgets before publication",
          "[BridgeAwareCubic][Contact][WholePathSupport]")
{
    const auto lower = bead({{0., 0.}, {10., 0.}, {10., 1.}});
    const auto upper = lower;
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    Contact::QueryCache cache(prepared);
    Validation::WorkBudget measured;
    REQUIRE(Contact::footprint_for_support(*prepared, upper, .4, 0., &measured, &cache).entire_path_supported);
    const size_t work = 50000000 - measured.remaining_work;
    REQUIRE(work > 0);
    for (size_t allowance = 0; allowance < work; ++allowance) {
        Validation::WorkBudget limited{allowance};
        const auto failed = Contact::footprint_for_support(*prepared, upper, .4, 0., &limited, &cache);
        CHECK_FALSE(failed.entire_path_supported);
        CHECK_FALSE(failed.contact.completed);
        CHECK(failed.contact.resource_limited);
        CHECK(limited.remaining_work == 0);
    }
    Validation::WorkBudget no_preparation{0};
    const auto incomplete = Contact::prepare_lower({{&lower, .2}}, .4, upper.height, &no_preparation);
    const auto failed = Contact::footprint_for_support(*incomplete, upper, .4, 0.);
    CHECK_FALSE(failed.entire_path_supported);
    CHECK_FALSE(failed.contact.completed);
    CHECK(failed.contact.resource_limited);
    CHECK(failed.contact.segments.empty());
    REQUIRE(Contact::footprint_for_support(*prepared, upper, .4, 0., nullptr, &cache).entire_path_supported);
}

TEST_CASE("Bridge aware whole-path proof retains combined resident and output admission",
          "[BridgeAwareCubic][Contact][WholePathSupport]")
{
    const auto lower = bead({{0., 0.}, {10., 0.}});
    auto upper = lower;
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    Contact::QueryCache cache(prepared);
    REQUIRE(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly, nullptr, &cache).completed);
    // Eight prepared units plus upper input and two proof-output units fill
    // the existing allowance. Optional cached masks must yield their storage.
    upper.polyline.points.resize(199990, upper.polyline.points.back());
    const auto admitted = Contact::footprint_for_support(*prepared, upper, .4, 0., nullptr, &cache);
    REQUIRE(admitted.entire_path_supported);
    CHECK(cache.statistics().resident_units == 0);
    upper.polyline.points.push_back(upper.polyline.points.back());
    const auto rejected = Contact::footprint_for_support(*prepared, upper, .4, 0., nullptr, &cache);
    CHECK_FALSE(rejected.entire_path_supported);
    CHECK_FALSE(rejected.contact.completed);
    CHECK(rejected.contact.resource_limited);
}


namespace {
void same_mask_sign_contact(const Contact::PathContact &actual, const Contact::PathContact &expected)
{
    same_contact(actual, expected);
    // The existing helper compares contours and exact line witnesses. Include
    // every mask hole too, so reuse requires complete ordered output equality.
    for (size_t i = 0; i < actual.segments.size(); ++i)
        for (size_t j = 0; j < actual.segments[i].centerline_support.size(); ++j) {
            const auto &a = actual.segments[i].centerline_support[j];
            const auto &e = expected.segments[i].centerline_support[j];
            REQUIRE(a.holes.size() == e.holes.size());
            for (size_t k = 0; k < a.holes.size(); ++k)
                CHECK(a.holes[k].points == e.holes[k].points);
        }
}

void require_reverse_mask_reuse(const Contact::PreparedLowerPtr &prepared, ExtrusionPath upper,
                                double tolerance, Contact::FootprintMode mode)
{
    Contact::QueryCache cache(prepared);
    const auto forward = Contact::footprint(*prepared, upper, .4, tolerance, mode);
    REQUIRE(forward.input_valid);
    REQUIRE(forward.completed);
    same_mask_sign_contact(Contact::footprint(*prepared, upper, .4, tolerance, mode, nullptr, &cache), forward);
    const auto warmed = cache.statistics();
    REQUIRE(warmed.misses > 0);
    REQUIRE(warmed.resident_units > 0);
    REQUIRE(warmed.evictions == 0);

    upper.reverse();
    // A reversed path has its own arclength/anchor coordinates. Compare with
    // an independently computed query in THIS orientation, not with forward.
    const auto reverse = Contact::footprint(*prepared, upper, .4, tolerance, mode);
    REQUIRE(reverse.input_valid);
    REQUIRE(reverse.completed);
    same_mask_sign_contact(Contact::footprint(*prepared, upper, .4, tolerance, mode, nullptr, &cache), reverse);
    CHECK(cache.statistics().hits > warmed.hits);
    CHECK(cache.statistics().misses == warmed.misses);
    CHECK(cache.statistics().evictions == warmed.evictions);
    CHECK(cache.statistics().resident_units == warmed.resident_units);
}
}

TEST_CASE("Bridge aware mask sign keys reuse exact reverse queries across rounded geometry",
          "[BridgeAwareCubic][Contact][MaskSign]")
{
    const int geometry = GENERATE(0, 1, 2, 3, 4, 5);
    const float lower_width = GENERATE(.2f, .45f);
    const double tolerance = GENERATE(0., .0001);
    const auto mode = GENERATE(Contact::FootprintMode::ContactsOnly, Contact::FootprintMode::Full);
    INFO("geometry=" << geometry << ", lower_width=" << lower_width << ", tolerance=" << tolerance);
    auto lower = bead({{-6., 0.}, {6., 0.}}, lower_width, .2f);
    auto upper = bead({{-5., 0.}, {5., 0.}});
    if (geometry == 1) {
        lower.polyline.points = {Point(0, -600000), Point(0, 600000)};
        upper.polyline.points = {Point(0, -500000), Point(0, 500000)};
    } else if (geometry == 2) {
        lower.polyline.points = {Point(-600001, -600001), Point(600001, 600001)};
        upper.polyline.points = {Point(-500000, -500000), Point(500000, 500000)};
    } else if (geometry == 3) {
        // Almost parallel axes retain both small nonzero transverse components.
        upper.polyline.points = {Point(-500000, 1), Point(500000, 2)};
    } else if (geometry >= 4) {
        // Adjacent float widths put the upper transverse displacement on
        // opposite sides of half a coordinate unit, at negative/positive Y.
        upper.width = geometry == 4 ? .20001f : std::nextafter(.20001f, 1.f);
        const double radius_units = .5 * (double(upper.width) - double(upper.height)) / SCALING_FACTOR;
        if (geometry == 4) REQUIRE(radius_units < .5);
        else REQUIRE(radius_units > .5);
        REQUIRE(std::abs(radius_units - .5) < .01);
        const coord_t y = geometry == 4 ? -200007 : 200007;
        lower.polyline.points = {Point(-600003, y), Point(600003, y)};
        upper.polyline.points = {Point(-500003, y), Point(500003, y)};
    }
    // Horizontal and vertical directions produce the signed-zero components;
    // a circular lower top has radius zero but still produces cacheable masks.
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    require_reverse_mask_reuse(prepared, upper, tolerance, mode);
}

TEST_CASE("Bridge aware mask sign keys retain exact output near coordinate bounds",
          "[BridgeAwareCubic][Contact][MaskSign]")
{
    const int sign = GENERATE(-1, 1);
    const float lower_width = GENERATE(.2f, .45f);
    const coord_t safe_limit = std::numeric_limits<coord_t>::max() / 4;
    const coord_t origin = sign * (safe_limit - 1000000);
    auto lower = bead({{-6., -2.}, {6., 2.}}, lower_width, .2f);
    auto upper = bead({{-5., -1.}, {5., 1.}});
    lower.polyline.points = {Point(origin - 600000, origin - 200000),
                             Point(origin + 600000, origin + 200000)};
    upper.polyline.points = {Point(origin - 500000, origin - 166666),
                             Point(origin + 500000, origin + 166666)};
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    require_reverse_mask_reuse(prepared, upper, 0., Contact::FootprintMode::Full);
}

TEST_CASE("Bridge aware reverse mask hits transfer ownership at the actual admission boundary",
          "[BridgeAwareCubic][Contact][MaskSign]")
{
    const auto lower = bead({{-1., 0.}, {11., 0.}});
    auto upper = bead({{0., 0.}, {10., 0.}});
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    const auto baseline = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    REQUIRE(baseline.input_valid);
    REQUIRE(baseline.completed);
    REQUIRE_FALSE(baseline.segments.empty());
    REQUIRE_FALSE(baseline.segments.front().centerline_support.empty());
    size_t output_units = baseline.segments.size();
    for (const auto &segment : baseline.segments) {
        output_units += segment.line_contacts.size();
        for (const auto &polygon : segment.centerline_support) {
            output_units += polygon.contour.points.size();
            for (const auto &hole : polygon.holes) output_units += hole.points.size();
        }
    }
    const size_t prepared_units = Contact::prepared_lower_contact_units(*prepared);
    REQUIRE(prepared_units + output_units + 2 < 200000);
    Contact::QueryCache cache(prepared);
    same_mask_sign_contact(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly,
                                             nullptr, &cache), baseline);
    const auto warmed = cache.statistics();
    REQUIRE(warmed.resident_units > 0);

    upper.reverse();
    upper.polyline.points.resize(200000 - prepared_units - output_units, upper.polyline.points.back());
    const auto expected = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    REQUIRE(expected.input_valid);
    REQUIRE(expected.completed);
    same_mask_sign_contact(Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly,
                                             nullptr, &cache), expected);
    CHECK(cache.statistics().hits > warmed.hits);
    CHECK(cache.statistics().misses == warmed.misses);
    CHECK(cache.statistics().evictions > warmed.evictions);
    CHECK(cache.statistics().resident_units == 0);

    upper.polyline.points.push_back(upper.polyline.points.back());
    const auto uncached_rejected = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    const auto cached_rejected = Contact::footprint(*prepared, upper, .4, 0., Contact::FootprintMode::ContactsOnly,
                                                    nullptr, &cache);
    CHECK_FALSE(uncached_rejected.completed);
    CHECK(uncached_rejected.resource_limited);
    CHECK_FALSE(cached_rejected.completed);
    CHECK(cached_rejected.resource_limited);
}

TEST_CASE("Bridge aware mask sign reuse preserves exact reserve identity",
          "[BridgeAwareCubic][Contact][MaskSign]")
{
    const auto lower = bead({{-1., 0.}, {11., 0.}});
    auto upper = bead({{0., 0.}, {10., 0.}});
    const auto prepared = Contact::prepare_lower({{&lower, .2}}, .4, upper.height);
    Contact::QueryCache cache(prepared);
    const auto query = [&](const ExtrusionPath &path, double tolerance) {
        const auto expected = Contact::footprint(*prepared, path, .4, tolerance, Contact::FootprintMode::ContactsOnly);
        REQUIRE(expected.input_valid);
        REQUIRE(expected.completed);
        same_mask_sign_contact(Contact::footprint(*prepared, path, .4, tolerance, Contact::FootprintMode::ContactsOnly,
                                                 nullptr, &cache), expected);
    };
    query(upper, .0001);
    const auto warmed = cache.statistics();
    REQUIRE(warmed.misses > 0);
    upper.reverse();
    query(upper, .0001);
    CHECK(cache.statistics().hits > warmed.hits);
    CHECK(cache.statistics().misses == warmed.misses);

    const auto reused = cache.statistics();
    auto shorter = upper;
    shorter.polyline.points.back() = Point::new_scale(5., 0.);
    query(shorter, .0001); // Same transverse, different length-dependent reserve.
    CHECK(cache.statistics().misses > reused.misses);
    const auto shortened = cache.statistics();
    query(upper, .0002);
    CHECK(cache.statistics().misses > shortened.misses);
}
