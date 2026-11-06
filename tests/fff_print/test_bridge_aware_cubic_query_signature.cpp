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
ExtrusionPath bead(std::initializer_list<Vec2d> points, float width = .4f, float height = .2f)
{
    ExtrusionPath path(erInternalInfill, .08, width, height);
    path.polyline = Polyline::new_scale(std::vector<Vec2d>(points));
    return path;
}

Contact::PreparedLowerPtr prepare(const std::vector<ExtrusionPath> &paths,
                                 const ExtrusionPath &upper, double upper_z = .4, double lower_z = .2)
{
    std::vector<Contact::Bead> deposited;
    for (const auto &path : paths) deposited.push_back({&path, lower_z});
    return Contact::prepare_lower(deposited, upper_z, upper.height);
}

Contact::PathContact contacts(const Contact::PreparedLower &lower, const ExtrusionPath &upper,
                              double upper_z = .4, double tolerance = 0.)
{
    return Contact::footprint(lower, upper, upper_z, tolerance, Contact::FootprintMode::ContactsOnly);
}

void same_contacts(const Contact::PathContact &a, const Contact::PathContact &b)
{
    REQUIRE(a.input_valid);
    REQUIRE(b.input_valid);
    REQUIRE(a.completed);
    REQUIRE(b.completed);
    CHECK(a.contact_z == b.contact_z);
    CHECK(a.coordinate_uncertainty_mm == b.coordinate_uncertainty_mm);
    CHECK(a.physical_footprint.empty());
    CHECK(b.physical_footprint.empty());
    REQUIRE(a.segments.size() == b.segments.size());
    for (size_t i = 0; i < a.segments.size(); ++i) {
        const auto &x = a.segments[i], &y = b.segments[i];
        CHECK(x.segment_index == y.segment_index);
        REQUIRE(x.centerline_support.size() == y.centerline_support.size());
        for (size_t j = 0; j < x.centerline_support.size(); ++j) {
            CHECK(x.centerline_support[j].contour.points == y.centerline_support[j].contour.points);
            REQUIRE(x.centerline_support[j].holes.size() == y.centerline_support[j].holes.size());
            for (size_t k = 0; k < x.centerline_support[j].holes.size(); ++k)
                CHECK(x.centerline_support[j].holes[k].points == y.centerline_support[j].holes[k].points);
        }
        REQUIRE(x.line_contacts.size() == y.line_contacts.size());
        for (size_t j = 0; j < x.line_contacts.size(); ++j) {
            CHECK(x.line_contacts[j].begin_numerator == y.line_contacts[j].begin_numerator);
            CHECK(x.line_contacts[j].begin_denominator == y.line_contacts[j].begin_denominator);
            CHECK(x.line_contacts[j].end_numerator == y.line_contacts[j].end_numerator);
            CHECK(x.line_contacts[j].end_denominator == y.line_contacts[j].end_denominator);
            CHECK(x.line_contacts[j].position_uncertainty_mm == y.line_contacts[j].position_uncertainty_mm);
        }
    }
}

Validation::Result spans(const ExtrusionPath &upper, const Contact::PathContact &contact)
{
    const ExPolygons demand{ExPolygon(Polygon({Point::new_scale(-1., -2.), Point::new_scale(11., -2.),
        Point::new_scale(11., 2.), Point::new_scale(-1., 2.)}))};
    return Validation::validate_contacts({}, {}, {upper.polyline}, demand, 5., .0001, {contact});
}
}

TEST_CASE("Contact query signatures ignore only fresh noncontributing sections",
          "[BridgeAwareCubic][QuerySignature]")
{
    const auto upper = bead({{0., 0.}, {10., 0.}});
    std::vector<ExtrusionPath> lower{bead({{0., -1.}, {0., 1.}}), bead({{4., -1.}, {4., 1.}}),
        bead({{8., -1.}, {8., 1.}}), bead({{10., -1.}, {10., 1.}})};
    // Exercise indexed enumeration and index renumbering, not only a tiny scan.
    for (int i = 0; i < 80; ++i) lower.push_back(bead({{double(i), 20.}, {double(i), 21.}}));
    const auto first = prepare(lower, upper);
    const auto signature = Contact::capture_query_signature(*first, upper, .4, 0.);
    REQUIRE(signature);
    CHECK(Contact::matches_query_signature(*signature, *first, upper, .4, 0.));
    const auto expected = contacts(*first, upper);
    REQUIRE(spans(upper, expected).passed());

    lower.back() = bead({{-20., 30.}, {-20., 31.}});
    std::reverse(lower.begin(), lower.end());
    lower.push_back(bead({{-30., -20.}, {-30., -19.}}));
    const auto unrelated = prepare(lower, upper);
    CHECK(Contact::matches_query_signature(*signature, *unrelated, upper, .4, 0.));
    same_contacts(contacts(*unrelated, upper), expected);

    // A fresh candidate appearing in formerly empty space must invalidate too.
    lower.push_back(bead({{2., -1.}, {2., 1.}}));
    CHECK_FALSE(Contact::matches_query_signature(*signature, *prepare(lower, upper), upper, .4, 0.));
}

TEST_CASE("Contact query signatures detect changed contributors and canonical merges",
          "[BridgeAwareCubic][QuerySignature]")
{
    const auto upper = bead({{0., 0.}, {10., 0.}});
    std::vector<ExtrusionPath> lower{bead({{0., -1.}, {0., 1.}}), bead({{4., -1.}, {4., 1.}}),
        bead({{8., -1.}, {8., 1.}}), bead({{10., -1.}, {10., 1.}})};
    const auto first = prepare(lower, upper);
    const auto signature = Contact::capture_query_signature(*first, upper, .4, 0.);
    REQUIRE(signature);
    REQUIRE(spans(upper, contacts(*first, upper)).passed());
    SECTION("moving a real anchor changes the complete span verdict") {
        lower[1] = bead({{7., -1.}, {7., 1.}});
        const auto changed = prepare(lower, upper);
        CHECK_FALSE(Contact::matches_query_signature(*signature, *changed, upper, .4, 0.));
        CHECK_FALSE(spans(upper, contacts(*changed, upper)).passed());
    }
    SECTION("one float step in actual radius is not an approximate match") {
        lower[1].width = std::nextafter(lower[1].width, 1.f);
        CHECK_FALSE(Contact::matches_query_signature(*signature, *prepare(lower, upper), upper, .4, 0.));
    }
    SECTION("outside extension changes the canonical merged contributor") {
        lower.push_back(bead({{4., -10.}, {4., -1.}}));
        CHECK_FALSE(Contact::matches_query_signature(*signature, *prepare(lower, upper), upper, .4, 0.));
    }
    // Preparation and signature own their state; later source edits cannot alter it.
    CHECK(Contact::matches_query_signature(*signature, *first, upper, .4, 0.));
}

TEST_CASE("Contact query signatures retain tangent and globally selected plane identity",
          "[BridgeAwareCubic][QuerySignature]")
{
    const auto upper = bead({{0., 0.}, {10., 0.}});
    SECTION("zero-width physical section is an exact contact input") {
        std::vector<ExtrusionPath> lower{bead({{5., -1.}, {5., 1.}}, .4f, .4f)};
        const auto first = prepare(lower, upper, .6, .4);
        const auto signature = Contact::capture_query_signature(*first, upper, .6, 0.);
        REQUIRE(signature);
        REQUIRE_FALSE(contacts(*first, upper, .6).segments.front().line_contacts.empty());
        lower.front().width = .4001f;
        CHECK_FALSE(Contact::matches_query_signature(*signature, *prepare(lower, upper, .6, .4), upper, .6, 0.));
    }
    SECTION("distant bead height can change the common physical plane") {
        std::vector<ExtrusionPath> lower{bead({{5., -1.}, {5., 1.}}), bead({{20., 20.}, {21., 20.}})};
        const auto first = prepare(lower, upper, .35);
        const auto signature = Contact::capture_query_signature(*first, upper, .35, 0.);
        REQUIRE(signature);
        lower.back().height = .01f;
        const auto changed = prepare(lower, upper, .35);
        REQUIRE(contacts(*first, upper, .35).contact_z != contacts(*changed, upper, .35).contact_z);
        CHECK_FALSE(Contact::matches_query_signature(*signature, *changed, upper, .35, 0.));
    }
}

TEST_CASE("Contact query signatures retain original segment order and serialized coordinates",
          "[BridgeAwareCubic][QuerySignature]")
{
    auto upper = bead({{0., 0.}, {5., 0.}, {5., 0.}, {10., 0.}});
    const std::vector<ExtrusionPath> lower{bead({{-1., 0.}, {11., 0.}})};
    const auto first = prepare(lower, upper);
    const auto signature = Contact::capture_query_signature(*first, upper, .4, 0.);
    REQUIRE(signature);
    SECTION("reversal changes directed original segment identity") {
        std::reverse(upper.polyline.points.begin(), upper.polyline.points.end());
        CHECK_FALSE(Contact::matches_query_signature(*signature, *first, upper, .4, 0.));
    }
    SECTION("duplicate removal changes original segment indices") {
        upper.polyline.points.erase(upper.polyline.points.begin() + 2);
        CHECK_FALSE(Contact::matches_query_signature(*signature, *first, upper, .4, 0.));
    }
    SECTION("serialized output phase changes a short terminal direction") {
        const auto source = bead({{9.83945, 9.03945}, {9.84428, 9.03816}});
        BridgeAwareCubic::SerializedXY phase;
        auto a = source, b = source;
        a.polyline = phase.emitted_polyline(source.polyline);
        phase.shift = Vec2d(.00024, .00032);
        b.polyline = phase.emitted_polyline(source.polyline);
        REQUIRE(a.polyline.points != b.polyline.points);
        const auto prepared = prepare({}, a);
        const auto phase_signature = Contact::capture_query_signature(*prepared, a, .4, 0.);
        REQUIRE(phase_signature);
        CHECK_FALSE(Contact::matches_query_signature(*phase_signature, *prepared, b, .4, 0.));
        // Separate phase components remain an adapter key even when output coincides.
    }
}

TEST_CASE("Contact query signature lookup freshly validates all normal query guards",
          "[BridgeAwareCubic][QuerySignature]")
{
    const auto good = bead({{0., 0.}, {10., 0.}});
    std::vector<ExtrusionPath> lower{bead({{-1., 0.}, {11., 0.}})};
    const auto first = prepare(lower, good);
    const auto signature = Contact::capture_query_signature(*first, good, .4, 0.);
    REQUIRE(signature);
    auto upper = good;
    double top = .4, tolerance = 0.;
    const int invalid = GENERATE(0, 1, 2, 3, 4, 5, 6, 7, 8, 9);
    if (invalid == 0) upper.mm3_per_mm = 0.;
    if (invalid == 1) upper.set_force_no_extrusion(true);
    if (invalid == 2) upper.width = .1f;
    if (invalid == 3) upper.height = std::nextafter(upper.height, 1.f);
    if (invalid == 4) top = std::nextafter(top, 1.);
    if (invalid == 5) tolerance = std::numeric_limits<double>::quiet_NaN();
    if (invalid == 6) upper.polyline.points[1] = upper.polyline.points[0];
    if (invalid == 7) upper.polyline.points.resize(1);
    if (invalid == 8) upper.polyline.points[1].x() = std::numeric_limits<coord_t>::max();
    if (invalid == 9) {
        PathFittingData arc{};
        arc.start_point_index = 0;
        arc.end_point_index = 1;
        arc.path_type = EMovePathType::Arc_move_ccw;
        upper.polyline.fitting_result.push_back(arc);
    }
    CHECK_FALSE(Contact::capture_query_signature(*first, upper, top, tolerance));
    CHECK_FALSE(Contact::matches_query_signature(*signature, *first, upper, top, tolerance));

    // Invalid excluded/future material is still rejected by fresh preparation.
    lower.push_back(bead({{100., 100.}, {101., 100.}}));
    lower.back().mm3_per_mm = 0.;
    const auto invalid_lower = Contact::prepare_lower({{&lower[0], .2}, {&lower[1], .8}}, .4, good.height);
    CHECK_FALSE(Contact::capture_query_signature(*invalid_lower, good, .4, 0.));
    CHECK_FALSE(Contact::matches_query_signature(*signature, *invalid_lower, good, .4, 0.));
}

TEST_CASE("Contact query signatures compare valid scalar inputs exactly",
          "[BridgeAwareCubic][QuerySignature]")
{
    const auto original = bead({{0., 0.}, {10., 0.}});
    const std::vector<ExtrusionPath> lower{bead({{-1., 0.}, {11., 0.}})};
    const auto prepared = prepare(lower, original);
    const auto signature = Contact::capture_query_signature(*prepared, original, .4, 0.);
    REQUIRE(signature);
    auto changed = original;
    double top = .4, tolerance = 0.;
    const int input = GENERATE(0, 1, 2, 3, 4, 5);
    if (input == 0) changed.width = std::nextafter(changed.width, 1.f);
    if (input == 1) changed.mm3_per_mm = std::nextafter(changed.mm3_per_mm, 1.);
    if (input == 2) changed.set_extrusion_role(erBridgeInfill);
    if (input == 3) tolerance = std::numeric_limits<double>::epsilon();
    if (input == 4) top = std::nextafter(top, 1.);
    if (input == 5) changed.height = std::nextafter(changed.height, 1.f);
    const auto fresh = prepare(lower, changed, top);
    REQUIRE(contacts(*fresh, changed, top, tolerance).input_valid);
    CHECK_FALSE(Contact::matches_query_signature(*signature, *fresh, changed, top, tolerance));
}

TEST_CASE("Contact query signatures share live mask cache admission and reject foreign caches",
          "[BridgeAwareCubic][QuerySignature]")
{
    const auto upper = bead({{0., 0.}, {10., 0.}});
    const std::vector<ExtrusionPath> lower{bead({{0., -1.}, {0., 1.}}), bead({{5., -1.}, {5., 1.}}),
        bead({{10., -1.}, {10., 1.}})};
    const auto first = prepare(lower, upper);
    Contact::QueryCache masks(first);
    const auto reference = Contact::footprint(*first, upper, .4, 0., Contact::FootprintMode::ContactsOnly, nullptr, &masks);
    REQUIRE(reference.completed);
    const size_t resident = masks.statistics().resident_units;
    REQUIRE(resident > 0);
    const auto signature = Contact::capture_query_signature(*first, upper, .4, 0., nullptr, 8192, {}, &masks);
    REQUIRE(signature);
    CHECK(Contact::matches_query_signature(*signature, *first, upper, .4, 0., nullptr, {}, &masks));
    CHECK(masks.statistics().resident_units == resident);
    same_contacts(Contact::footprint(*first, upper, .4, 0., Contact::FootprintMode::ContactsOnly, nullptr, &masks), reference);

    const auto equivalent = prepare(lower, upper);
    Contact::QueryCache foreign(equivalent);
    CHECK_FALSE(Contact::capture_query_signature(*first, upper, .4, 0., nullptr, 8192, {}, &foreign));
    CHECK_FALSE(Contact::matches_query_signature(*signature, *first, upper, .4, 0., nullptr, {}, &foreign));
    CHECK(Contact::matches_query_signature(*signature, *equivalent, upper, .4, 0., nullptr, {}, &foreign));
}

TEST_CASE("Contact query signatures have bounded owned storage and never refund work",
          "[BridgeAwareCubic][QuerySignature]")
{
    const auto upper = bead({{0., 0.}, {5., 0.}, {10., 0.}});
    const std::vector<ExtrusionPath> lower{bead({{0., -1.}, {0., 1.}}), bead({{5., -1.}, {5., 1.}}),
        bead({{10., -1.}, {10., 1.}})};
    const auto first = prepare(lower, upper);
    Validation::WorkBudget budget;
    const size_t initial = budget.remaining_work;
    const auto signature = Contact::capture_query_signature(*first, upper, .4, 0., &budget);
    REQUIRE(signature);
    const size_t units = Contact::query_signature_units(*signature);
    REQUIRE(units > upper.polyline.size());
    REQUIRE(units <= 8192);
    REQUIRE(budget.remaining_work < initial);
    const size_t after_capture = budget.remaining_work;
    CHECK(Contact::matches_query_signature(*signature, *first, upper, .4, 0., &budget));
    CHECK(budget.remaining_work < after_capture);
    CHECK(Contact::capture_query_signature(*first, upper, .4, 0., &budget, units));
    const size_t before_cap = budget.remaining_work;
    // A smaller cap can choose smaller vector growth, so use a cap too small
    // even for metadata plus the complete original upper points.
    CHECK_FALSE(Contact::capture_query_signature(*first, upper, .4, 0., &budget, upper.polyline.size()));
    CHECK(budget.remaining_work < before_cap);
    CHECK_FALSE(Contact::capture_query_signature(*first, upper, .4, 0., &budget, 0));
    CHECK(budget.remaining_work < initial);
    Validation::WorkBudget exhausted{0};
    CHECK_FALSE(Contact::capture_query_signature(*first, upper, .4, 0., &exhausted));
    CHECK_FALSE(Contact::matches_query_signature(*signature, *first, upper, .4, 0., &exhausted));
    CHECK(exhausted.remaining_work == 0);
}

TEST_CASE("Contact query signature cancellation propagates during capture and warm lookup",
          "[BridgeAwareCubic][QuerySignature]")
{
    auto upper = bead({{0., 0.}, {10., 0.}});
    for (int i = 0; i < 128; ++i) upper.polyline.points.push_back(Point::new_scale(i % 2 ? 10. : 0., 0.));
    const std::vector<ExtrusionPath> lower{bead({{-1., 0.}, {11., 0.}})};
    const auto first = prepare(lower, upper);
    const auto signature = Contact::capture_query_signature(*first, upper, .4, 0.);
    REQUIRE(signature);
    size_t calls = 0;
    const auto cancel = [&] { if (++calls == 2) throw std::runtime_error("query-signature cancelled"); };
    Validation::WorkBudget budget;
    CHECK_THROWS_AS(Contact::capture_query_signature(*first, upper, .4, 0., &budget, 8192, cancel), std::runtime_error);
    REQUIRE(calls == 2);
    CHECK(budget.remaining_work < 50000000);
    calls = 0;
    CHECK_THROWS_AS(Contact::matches_query_signature(*signature, *first, upper, .4, 0., &budget, cancel), std::runtime_error);
    CHECK(calls == 2);
    const auto allocation_named_cancel = [] { throw std::bad_alloc(); };
    CHECK_THROWS_AS(Contact::capture_query_signature(*first, upper, .4, 0., &budget, 8192, allocation_named_cancel), std::bad_alloc);
    CHECK_THROWS_AS(Contact::matches_query_signature(*signature, *first, upper, .4, 0., &budget, allocation_named_cancel), std::bad_alloc);
}
