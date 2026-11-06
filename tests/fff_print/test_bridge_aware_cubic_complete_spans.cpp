#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"
#include "libslic3r/Fill/BridgeAwareCubicProofShadow.hpp"

#include <algorithm>
#include <limits>

using namespace Slic3r;
namespace Contact = Slic3r::BridgeAwareCubicContact;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
Polyline path(std::initializer_list<Vec2d> points)
{
    return Polyline::new_scale(std::vector<Vec2d>(points));
}
ExPolygon rectangle(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon({Point::new_scale(x0, y0), Point::new_scale(x1, y0),
                              Point::new_scale(x1, y1), Point::new_scale(x0, y1)}));
}
Contact::PathContact masked(const Polyline &line, const ExPolygons &masks = {})
{
    Contact::PathContact result;
    result.contact_z = .2;
    for (size_t i = 1; i < line.points.size(); ++i)
        if (line.points[i - 1] != line.points[i])
            result.segments.push_back({i - 1, masks, {}});
    return result;
}
Validation::Result complete(const Polyline &line, const Contact::PathContact &contact,
                            double gap = 5., double tolerance = .001,
                            Validation::WorkBudget *budget = nullptr)
{
    return Validation::validate_complete_contact_spans({line}, gap, tolerance, {contact}, budget);
}
Validation::Result enclosing_reference(const Polylines &paths,
    const std::vector<Contact::PathContact> &contacts, double gap, double tolerance)
{
    // Every valid fixture in the differential tests is strictly inside this
    // selection. Empty material_domain disables independent physical cells.
    return Validation::validate_contacts({}, {}, paths, {rectangle(-100., -100., 100., 100.)},
                                          gap, tolerance, contacts);
}
void same_spans(const Validation::Result &actual, const Validation::Result &expected)
{
    CHECK(actual.input_valid == expected.input_valid);
    CHECK(actual.completed == expected.completed);
    CHECK(actual.resource_limited == expected.resource_limited);
    CHECK(actual.passed() == expected.passed());
    CHECK(actual.max_unsupported_mm == expected.max_unsupported_mm);
    CHECK(actual.void_cells.empty());
    CHECK(actual.max_void_diameter_mm == 0.);
    REQUIRE(actual.unsupported_runs.size() == expected.unsupported_runs.size());
    for (size_t i = 0; i < actual.unsupported_runs.size(); ++i) {
        const auto &a = actual.unsupported_runs[i];
        const auto &b = expected.unsupported_runs[i];
        CHECK(a.path_index == b.path_index);
        CHECK(a.begin_mm == b.begin_mm);
        CHECK(a.end_mm == b.end_mm);
        CHECK(a.length_mm == b.length_mm);
        CHECK(a.uncertainty_mm == b.uncertainty_mm);
        CHECK(a.kind == b.kind);
        CHECK(a.exceeds_gap == b.exceeds_gap);
    }
}
}

TEST_CASE("Complete contact spans accept short anchored bridges and continuous support",
          "[BridgeAwareCubic][CompleteSpans]")
{
    const auto line = path({{.5, 0.}, {4.5, 0.}});
    ExPolygons masks{rectangle(-1., -1., 1., 1.), rectangle(4., -1., 6., 1.)};
    bool continuous = false;
    SECTION("two ended air span") {}
    SECTION("continuous material") { masks = {rectangle(-1., -1., 6., 1.)}; continuous = true; }
    const auto contact = masked(line, masks);
    const auto result = complete(line, contact, 3.1);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    REQUIRE(result.passed());
    CHECK(result.unsupported_runs.size() == (continuous ? 0 : 1));
    CHECK(result.max_unsupported_mm == Approx(continuous ? 0. : 3.));
    same_spans(result, enclosing_reference({line}, {contact}, 3.1, .001));
}

TEST_CASE("Complete contact spans accept real zero width finite anchors",
          "[BridgeAwareCubic][CompleteSpans]")
{
    ExtrusionPath upper(erInternalInfill, .08, .45f, .2f);
    upper.polyline = path({{0., 0.}, {4., 0.}});
    ExtrusionPath left(erPerimeter, .08, .2f, .2f), right = left;
    left.polyline = path({{0., -1.}, {0., 1.}});
    right.polyline = path({{4., -1.}, {4., 1.}});
    const auto contact = Contact::footprint({{&left, .2}, {&right, .2}}, upper, .4, 0.);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    REQUIRE(contact.physical_footprint.empty());
    REQUIRE(contact.physical_lines.size() == 2);
    const auto result = complete(upper.polyline, contact, 4.001, 0.);
    REQUIRE(result.passed());
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::TwoEndedBridge);
    CHECK(result.unsupported_runs.front().length_mm == Approx(4.));
    same_spans(result, enclosing_reference({upper.polyline}, {contact}, 4.001, 0.));
    CHECK_FALSE(complete(upper.polyline, contact, 4., 0.).passed());
}

TEST_CASE("Complete contact spans never add tolerance to the permitted gap",
          "[BridgeAwareCubic][CompleteSpans]")
{
    const double gap = GENERATE(2.999, 3., 3.0005);
    const auto line = path({{.5, 0.}, {4.5, 0.}});
    const auto contact = masked(line, {rectangle(-1., -1., 1., 1.), rectangle(4., -1., 6., 1.)});
    const auto result = complete(line, contact, gap);
    REQUIRE(result.completed);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().exceeds_gap);
    CHECK_FALSE(result.passed());
    same_spans(result, enclosing_reference({line}, {contact}, gap, .001));
}

TEST_CASE("Complete contact spans reject short one sided and floating extrusions",
          "[BridgeAwareCubic][CompleteSpans]")
{
    const bool anchored = GENERATE(false, true);
    const auto line = path({{0., 0.}, {1., 0.}});
    const auto contact = masked(line, anchored ? ExPolygons{rectangle(-1., -1., .1, 1.)} : ExPolygons{});
    const auto result = complete(line, contact, 100.);
    REQUIRE(result.completed);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().kind == (anchored ? Validation::UnsupportedRunKind::OneSidedOverhang :
                                                                            Validation::UnsupportedRunKind::Floating));
    CHECK_FALSE(result.unsupported_runs.front().exceeds_gap);
    CHECK_FALSE(result.passed());
    same_spans(result, enclosing_reference({line}, {contact}, 100., .001));
}

TEST_CASE("Complete contact spans retain joints reversals and original repeated vertices",
          "[BridgeAwareCubic][CompleteSpans]")
{
    auto line = path({{.5, 0.}, {2., 0.}, {2., 0.}, {2., 2.}, {4.5, 2.}, {4.5, 2.}});
    if (GENERATE(false, true)) line.reverse();
    const auto contact = masked(line, {rectangle(-1., -1., 1., 1.), rectangle(4., 1., 6., 3.)});
    const auto result = complete(line, contact, 5.1);
    REQUIRE(result.passed());
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().length_mm == Approx(5.));
    same_spans(result, enclosing_reference({line}, {contact}, 5.1, .001));
    CHECK_FALSE(complete(line, contact, 4.9).passed());
}

TEST_CASE("Complete contact spans preserve exact anchors at original segment joints",
          "[BridgeAwareCubic][CompleteSpans]")
{
    auto line = path({{0., 0.}, {2., 0.}, {2., 0.}, {4., 0.}});
    if (GENERATE(false, true)) line.reverse();
    auto contact = masked(line);
    REQUIRE(contact.segments.size() == 2);
    contact.segments.front().line_contacts = {{1, 1, 1, 1}, {0, 1, 0, 1}}; // Deliberately unsorted.
    contact.segments.back().line_contacts = {{1, 1, 1, 1}};
    const auto result = complete(line, contact, 2.1);
    REQUIRE(result.passed());
    REQUIRE(result.unsupported_runs.size() == 2);
    CHECK(result.unsupported_runs[0].length_mm == Approx(2.));
    CHECK(result.unsupported_runs[1].length_mm == Approx(2.));
    same_spans(result, enclosing_reference({line}, {contact}, 2.1, .001));
}

TEST_CASE("Complete contact spans never borrow anchors across a closed path seam",
          "[BridgeAwareCubic][CompleteSpans]")
{
    auto line = path({{0., 0.}, {4., 0.}, {4., 4.}, {0., 4.}, {0., 0.}});
    if (GENERATE(false, true)) line.reverse();
    const auto contact = masked(line, {rectangle(1., -1., 3., 1.)});
    const auto result = complete(line, contact, 100.);
    REQUIRE(result.completed);
    REQUIRE(result.unsupported_runs.size() == 2);
    for (const auto &run : result.unsupported_runs)
        CHECK(run.kind == Validation::UnsupportedRunKind::OneSidedOverhang);
    CHECK_FALSE(result.passed());
    same_spans(result, enclosing_reference({line}, {contact}, 100., .001));
}

TEST_CASE("Complete contact spans preserve disjoint support and mask holes",
          "[BridgeAwareCubic][CompleteSpans]")
{
    const auto line = path({{-.5, 0.}, {10.5, 0.}});
    auto first = rectangle(-1., -1., 3., 1.);
    auto hole = rectangle(1., -.5, 2., .5).contour;
    hole.make_clockwise();
    first.holes.push_back(hole);
    const auto contact = masked(line, {first, rectangle(7., -1., 11., 1.)});
    const auto result = complete(line, contact, 4.1);
    REQUIRE(result.passed());
    REQUIRE(result.unsupported_runs.size() == 2);
    CHECK(result.unsupported_runs[0].length_mm == Approx(1.));
    CHECK(result.unsupported_runs[1].length_mm == Approx(4.));
    same_spans(result, enclosing_reference({line}, {contact}, 4.1, .001));
    CHECK_FALSE(complete(line, contact, 3.9).passed());
}

TEST_CASE("Complete contact spans do not turn arbitrary polygon boundaries into anchors",
          "[BridgeAwareCubic][CompleteSpans]")
{
    const auto line = path({{0., 0.}, {4., 0.}});
    const auto contact = masked(line, {rectangle(-1., 0., 5., 2.)});
    const auto result = complete(line, contact, 10.);
    REQUIRE(result.completed);
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().kind == Validation::UnsupportedRunKind::Floating);
    CHECK_FALSE(result.passed());
    same_spans(result, enclosing_reference({line}, {contact}, 10., .001));
}

TEST_CASE("Complete contact spans include anchor and traversed segment uncertainty",
          "[BridgeAwareCubic][CompleteSpans]")
{
    const auto line = path({{0., 0.}, {2., 0.}, {4., 0.}});
    auto contact = masked(line);
    contact.coordinate_uncertainty_mm = .05;
    contact.segments.front().line_contacts = {{0, 1, 0, 1, .1}};
    contact.segments.back().line_contacts = {{1, 1, 1, 1, .2}};
    const auto result = complete(line, contact, 4.6);
    REQUIRE(result.passed());
    REQUIRE(result.unsupported_runs.size() == 1);
    CHECK(result.unsupported_runs.front().uncertainty_mm == Approx(.5));
    CHECK(result.unsupported_runs.front().length_mm == Approx(4.5));
    same_spans(result, enclosing_reference({line}, {contact}, 4.6, .001));
    CHECK_FALSE(complete(line, contact, 4.5).passed());
}

TEST_CASE("Complete contact spans reject invalid incomplete and resource limited certificates",
          "[BridgeAwareCubic][CompleteSpans]")
{
    const auto line = path({{0., 0.}, {4., 0.}});
    auto contact = masked(line, {rectangle(-1., -1., 5., 1.)});
    std::vector<Contact::PathContact> contacts{contact};
    SECTION("invalid contact") { contacts.front().input_valid = false; }
    SECTION("incomplete contact") { contacts.front().completed = false; }
    SECTION("resource limit") { contacts.front().completed = false; contacts.front().resource_limited = true; }
    SECTION("missing contact") { contacts.clear(); }
    SECTION("extra contact") { contacts.push_back(contact); }
    SECTION("wrong original segment index") { contacts.front().segments.front().segment_index = 1; }
    SECTION("missing positive segment") { contacts.front().segments.clear(); }
    SECTION("extra segment") { contacts.front().segments.push_back({1, {}, {}}); }
    SECTION("nonfinite contact plane") { contacts.front().contact_z = std::numeric_limits<double>::quiet_NaN(); }
    SECTION("negative coordinate uncertainty") { contacts.front().coordinate_uncertainty_mm = -.1; }
    SECTION("nonfinite coordinate uncertainty") { contacts.front().coordinate_uncertainty_mm = std::numeric_limits<double>::infinity(); }
    SECTION("invalid mask") { contacts.front().segments.front().centerline_support = {ExPolygon(Polygon({Point(0, 0), Point(1, 0)}))}; }
    SECTION("invalid exact line denominator") { contacts.front().segments.front().line_contacts = {{0, 0, 1, 1}}; }
    SECTION("reversed exact line interval") { contacts.front().segments.front().line_contacts = {{1, 1, 0, 1}}; }
    SECTION("negative anchor uncertainty") { contacts.front().segments.front().line_contacts = {{0, 1, 1, 1, -.1}}; }
    SECTION("nonfinite anchor uncertainty") { contacts.front().segments.front().line_contacts = {{0, 1, 1, 1, std::numeric_limits<double>::quiet_NaN()}}; }
    const auto result = Validation::validate_complete_contact_spans({line}, 5., .001, contacts);
    CHECK_FALSE(result.passed());
    CHECK_FALSE(result.failure_reason.empty());
    same_spans(result, enclosing_reference({line}, contacts, 5., .001));
}

TEST_CASE("Complete contact spans retain path numeric and input bounds",
          "[BridgeAwareCubic][CompleteSpans]")
{
    auto line = path({{0., 0.}, {4., 0.}});
    SECTION("too short") { line.points.resize(1); }
    SECTION("stationary") { line.points.back() = line.points.front(); }
    SECTION("unsafe coordinate") { line.points.back().x() = std::numeric_limits<coord_t>::max(); }
    SECTION("too many vertices") { line.points.assign(200001, Point(0, 0)); }
    SECTION("fitted arc") {
        PathFittingData arc{};
        arc.start_point_index = 0; arc.end_point_index = 1;
        arc.path_type = EMovePathType::Arc_move_ccw;
        line.fitting_result.push_back(arc);
    }
    const auto result = complete(line, masked(line, {rectangle(-1., -1., 5., 1.)}));
    CHECK_FALSE(result.input_valid);
    CHECK_FALSE(result.passed());
    CHECK_FALSE(result.failure_reason.empty());
}

TEST_CASE("Complete contact spans retain gap and tolerance input guards",
          "[BridgeAwareCubic][CompleteSpans]")
{
    double gap = 5., tolerance = .001;
    SECTION("zero gap") { gap = 0.; }
    SECTION("negative gap") { gap = -1.; }
    SECTION("nonfinite gap") { gap = std::numeric_limits<double>::quiet_NaN(); }
    SECTION("unrepresentable gap") { gap = 1.e30; }
    SECTION("negative tolerance") { tolerance = -.001; }
    SECTION("tolerance equals gap") { tolerance = gap; }
    SECTION("nonfinite tolerance") { tolerance = std::numeric_limits<double>::infinity(); }
    const auto line = path({{0., 0.}, {4., 0.}});
    const auto contact = masked(line, {rectangle(-1., -1., 5., 1.)});
    const auto result = complete(line, contact, gap, tolerance);
    CHECK_FALSE(result.input_valid);
    CHECK_FALSE(result.passed());
    same_spans(result, enclosing_reference({line}, {contact}, gap, tolerance));
}

TEST_CASE("Complete contact spans can fail while a selected demand subset passes",
          "[BridgeAwareCubic][CompleteSpans]")
{
    Polylines paths;
    std::vector<Contact::PathContact> contacts;
    ExPolygons selected;
    SECTION("another complete path is floating") {
        paths = {path({{0., 0.}, {1., 0.}}), path({{10., 0.}, {11., 0.}})};
        contacts = {masked(paths[0], {rectangle(-1., -1., 2., 1.)}), masked(paths[1])};
        selected = {rectangle(-1., -1., 2., 1.)};
    }
    SECTION("one path has an unselected one sided tail") {
        paths = {path({{0., 0.}, {10., 0.}})};
        contacts = {masked(paths[0], {rectangle(-1., -1., 1., 1.), rectangle(4., -1., 5., 1.),
                                     rectangle(8., -1., 9., 1.)})};
        selected = {rectangle(2., -.5, 3., .5)};
    }
    const auto subset = Validation::validate_contacts({}, {}, paths, selected, 3.1, .001, contacts);
    REQUIRE(subset.passed());
    const auto all = Validation::validate_complete_contact_spans(paths, 3.1, .001, contacts);
    REQUIRE(all.completed);
    CHECK_FALSE(all.passed());
    REQUIRE_FALSE(all.unsupported_runs.empty());
    same_spans(all, enclosing_reference(paths, contacts, 3.1, .001));
}

TEST_CASE("Complete contact spans never certify physical cells",
          "[BridgeAwareCubic][CompleteSpans]")
{
    const auto line = path({{1., 1.}, {2., 1.}});
    const auto contact = masked(line, {rectangle(0., 0., 3., 2.)});
    const auto spans = complete(line, contact, 1.);
    REQUIRE(spans.passed());
    CHECK(spans.void_cells.empty());
    CHECK(spans.max_void_diameter_mm == 0.);
    const auto cells = Validation::validate_contacts({rectangle(0., 0., 20., 20.)}, {}, {line},
        {rectangle(0., 0., 3., 2.)}, 1., .001, {contact});
    REQUIRE(cells.completed);
    REQUIRE_FALSE(cells.void_cells.empty());
    CHECK_FALSE(cells.passed());
}

TEST_CASE("Complete contact spans share work and report exhaustion without refund",
          "[BridgeAwareCubic][CompleteSpans]")
{
    const auto line = path({{0., 0.}, {10., 0.}});
    const auto contact = masked(line, {rectangle(-1., -1., 1., 1.), rectangle(4., -1., 6., 1.),
                                      rectangle(9., -1., 11., 1.)});
    Validation::WorkBudget probe;
    const size_t initial = probe.remaining_work;
    const auto expected = complete(line, contact, 3.1, .001, &probe);
    REQUIRE(expected.passed());
    const size_t one_call = initial - probe.remaining_work;
    REQUIRE(one_call > 1);
    CHECK(probe.charged_work[size_t(Validation::WorkCategory::Validation)] == one_call);
    Validation::WorkBudget exact{one_call};
    same_spans(complete(line, contact, 3.1, .001, &exact), expected);
    CHECK(exact.remaining_work == 0);
    Validation::WorkBudget short_budget{one_call - 1};
    const auto limited = complete(line, contact, 3.1, .001, &short_budget);
    CHECK(limited.input_valid);
    CHECK_FALSE(limited.completed);
    CHECK(limited.resource_limited);
    CHECK_FALSE(limited.passed());
    CHECK(short_budget.remaining_work == 0);
    CHECK_FALSE(limited.failure_reason.empty());
    Validation::WorkBudget shared{one_call + one_call / 2};
    REQUIRE(complete(line, contact, 3.1, .001, &shared).passed());
    const auto second = complete(line, contact, 3.1, .001, &shared);
    CHECK_FALSE(second.completed);
    CHECK(second.resource_limited);
    CHECK(shared.remaining_work == 0);
}

TEST_CASE("Complete contact spans accept an empty collection without inventing cells",
          "[BridgeAwareCubic][CompleteSpans]")
{
    const auto result = Validation::validate_complete_contact_spans({}, 5., .001, {});
    CHECK(result.input_valid);
    CHECK(result.completed);
    CHECK(result.passed());
    CHECK(result.unsupported_runs.empty());
    CHECK(result.void_cells.empty());
    CHECK(result.max_unsupported_mm == 0.);
    CHECK(result.max_void_diameter_mm == 0.);
}

TEST_CASE("Proof shadow distinguishes actual line support from mask-only support",
          "[BridgeAwareCubic][ProofShadow]")
{
    namespace S = BridgeAwareCubic::ProofShadow;
    const auto line = path({{0., 0.}, {4., 0.}});
    auto c = masked(line, {rectangle(-1., -1., 5., 1.)});
    REQUIRE(complete(line, c, 4.1, 0.).passed());
    size_t work = 1000000;
    const auto noop = [] {};
    CHECK(S::lines(line, c, 4.1, work, noop) == S::Outcome::Fail);
    ExtrusionPath upper(erInternalInfill, .08, .45f, .2f); upper.polyline = line;
    ExtrusionPath left(erPerimeter, .08, .2f, .2f), right = left;
    left.polyline = path({{0., -1.}, {0., 1.}}); right.polyline = path({{4., -1.}, {4., 1.}});
    const auto prep = Contact::prepare_lower({{&left,.2},{&right,.2}}, .4, upper.height);
    c = Contact::footprint(*prep, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
    REQUIRE(c.input_valid); REQUIRE(c.completed);
    CHECK(S::lines(line, c, 4.1, work, noop) == S::Outcome::Pass);
    CHECK(S::lines(line, c, 4., work, noop) == S::Outcome::Fail);
    CHECK(work < 1000000);
    size_t exhausted = 0;
    CHECK(S::lines(line, c, 4.1, exhausted, noop) == S::Outcome::Unavailable);
    CHECK_THROWS_AS(S::lines(line, c, 4.1, work, [] { throw std::runtime_error("shadow canceled"); }), std::runtime_error);
    c.resource_limited = true;
    CHECK(S::lines(line, c, 4.1, work, noop) == S::Outcome::Unavailable);
}

TEST_CASE("Proof shadow only labels fully observed valid empty layer visits",
          "[BridgeAwareCubic][ProofShadow]")
{
    namespace S = BridgeAwareCubic::ProofShadow;
    S::Row r;
    Validation::Result pass;
    { S::Layer v{&r, false}; v.begin(3, 2); v.note(true, pass, S::Clock::now()); v.note(true, pass, S::Clock::now()); }
    { S::Layer v{&r, false}; v.begin(3, 2); v.note(true, pass, S::Clock::now()); }
    { S::Layer v{&r, false}; v.begin(3, 1); v.note(false, pass, S::Clock::now()); }
    { S::Layer v{&r, true}; v.begin(3, 1); v.note(true, pass, S::Clock::now()); }
    { S::Layer v{&r, false}; v.begin(3, 1); Validation::Result bad; bad.completed = false; v.note(true, bad, S::Clock::now()); }
    { S::Layer v{&r, false, false}; v.begin(3, 1); v.note(true, pass, S::Clock::now()); }
    CHECK(r.layer_unqualified == 1);
    CHECK(r.layer_empty == 1);
    CHECK(r.layer_unknown == 2);
    CHECK(r.layer_nonempty == 1);
    CHECK(r.layer_probe == 1);
    CHECK(r.direct_empty_pass == 5);
    CHECK(r.direct_invalid == 1);
}
