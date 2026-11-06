#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSpanProofCache.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Contact = Slic3r::BridgeAwareCubicContact;
namespace Validation = Slic3r::BridgeAwareCubicValidation;
using Cache = BAC::SpanProofCache;

namespace {
ExtrusionPath bead(double first, double last, double y = 0.)
{
    ExtrusionPath result(erInternalInfill, .08, .45f, .2f);
    result.polyline = Polyline(Point::new_scale(first, y), Point::new_scale(last, y));
    return result;
}

struct Fixture {
    ExtrusionPath upper = bead(0., 10.);
    ExtrusionPath left = bead(-1., 2.), right, remote = bead(0., 10., 100.);
    BAC::SerializedXY phase;
    ExPolygons demand {ExPolygon(Polygon(Points{Point::new_scale(-1., -1.), Point::new_scale(11., -1.),
                                                Point::new_scale(11., 1.), Point::new_scale(-1., 1.)}))};
    Contact::PreparedLowerPtr lower;
    double gap = 8., selection_width = double(upper.width);

    explicit Fixture(double right_start = 8.) : right(bead(right_start, 11.)) { prepare(); }
    void prepare() { lower = Contact::prepare_lower({{&left, .2}, {&right, .2}, {&remote, .2}}, .4, upper.height); }
    Cache::Query query(Cache::Kind kind = Cache::Kind::Construction, Contact::QueryCache *masks = nullptr) const
    {
        return {kind, phase, gap, selection_width, demand, *lower, upper, .4, masks};
    }
    Validation::Result check() const
    {
        const auto contact = Contact::footprint(*lower, upper, .4, 0., Contact::FootprintMode::ContactsOnly);
        Polylines unsupported;
        for (const auto &segment : contact.segments) {
            const size_t i = segment.segment_index;
            append(unsupported, diff_pl(Polylines{Polyline(upper.polyline.points[i], upper.polyline.points[i + 1])},
                                       segment.centerline_support));
        }
        if (!contact.input_valid || !contact.completed || contact.segments.empty()) unsupported = {upper.polyline};
        const ExPolygons selected = intersection_ex(demand, union_ex(offset(unsupported, scale_(.5 * selection_width))));
        return Validation::validate_contacts({}, contact.physical_footprint, {upper.polyline}, selected, gap, 0., {contact});
    }
};
}

TEST_CASE("Bridge aware span cache publishes only complete passing local span facts", "[BridgeAwareCubic][SpanProofCache]")
{
    Fixture fixture;
    const auto checked = fixture.check();
    REQUIRE(checked.passed());
    REQUIRE_FALSE(checked.unsupported_runs.empty());
    REQUIRE(checked.max_unsupported_mm > 0.);
    Cache cache;
    CHECK_FALSE(cache.matches(fixture.query()));
    REQUIRE(cache.publish(fixture.query(), checked));
    CHECK(cache.matches(fixture.query()));
    CHECK(cache.statistics().entries == 1);
    CHECK(cache.statistics().units > 0);
    CHECK(cache.statistics().publications == 1);
    CHECK(cache.statistics().hits == 1);
    for (int invalid = 0; invalid < 7; ++invalid) {
        CAPTURE(invalid);
        Cache rejected;
        auto result = checked;
        if (invalid == 0) result.input_valid = false;
        if (invalid == 1) result.completed = false;
        if (invalid == 2) result.resource_limited = true;
        if (invalid == 3) result.failure_reason = "incomplete optional proof";
        if (invalid == 4) result.unsupported_runs.front().exceeds_gap = true;
        if (invalid == 5) result.void_cells.push_back({fixture.demand.front(), 1., false, true});
        if (invalid == 6) result.max_void_diameter_mm = .1;
        CHECK_FALSE(rejected.publish(fixture.query(), result));
        CHECK_FALSE(rejected.matches(fixture.query()));
        CHECK(rejected.statistics().entries == 0);
    }
    CHECK_FALSE(cache.publish(fixture.query(Cache::Kind::Ineligible), checked));
    CHECK_FALSE(cache.matches(fixture.query(Cache::Kind::Ineligible)));
    CHECK_FALSE(cache.matches(fixture.query(Cache::Kind::BridgeFollowup)));
    REQUIRE(cache.publish(fixture.query(Cache::Kind::BridgeFollowup), checked));
    CHECK(cache.matches(fixture.query(Cache::Kind::BridgeFollowup)));
}

TEST_CASE("Bridge aware span cache compares demand gap width and each exact phase component", "[BridgeAwareCubic][SpanProofCache]")
{
    Fixture fixture;
    Cache cache;
    REQUIRE(cache.publish(fixture.query(), fixture.check()));
    for (int mutation = 0; mutation < 8; ++mutation) {
        CAPTURE(mutation);
        Fixture changed;
        if (mutation == 0) changed.gap = std::nextafter(changed.gap, 20.);
        if (mutation == 1) changed.selection_width = std::nextafter(changed.selection_width, 1.);
        if (mutation == 2) changed.demand.front().contour.points.front().x() += 1;
        if (mutation == 3) {
            auto &points = changed.demand.front().contour.points;
            std::rotate(points.begin(), points.begin() + 1, points.end());
        }
        if (mutation == 4) { changed.phase.shift.x() += 1.; changed.phase.plate_origin.x() += 1.; }
        if (mutation == 5) { changed.phase.shift.y() += 1.; changed.phase.nozzle_offset.y() += 1.; }
        if (mutation == 6) changed.upper.polyline.points.back().x() -= 1;
        if (mutation == 7) changed.upper.mm3_per_mm = 0.;
        CHECK_FALSE(cache.matches(changed.query()));
        CHECK(cache.matches(fixture.query()));
    }
}

TEST_CASE("Bridge aware span cache accepts only fresh exact local contact candidates", "[BridgeAwareCubic][SpanProofCache]")
{
    Fixture fixture;
    Cache cache;
    REQUIRE(cache.publish(fixture.query(), fixture.check()));
    fixture.remote.polyline.translate(Point::new_scale(0., 100.));
    fixture.prepare();
    REQUIRE(cache.matches(fixture.query())); // Unrelated valid material may change.
    fixture.left.polyline.translate(Point::new_scale(0., 2.));
    fixture.prepare();
    CHECK_FALSE(cache.matches(fixture.query()));
    REQUIRE_FALSE(fixture.check().passed());
    fixture.left.polyline.translate(Point::new_scale(0., -2.));
    fixture.remote.mm3_per_mm = std::numeric_limits<double>::quiet_NaN();
    fixture.prepare();
    CHECK_FALSE(cache.matches(fixture.query())); // Even unrelated invalid deposition must be caught.
    Fixture valid;
    Contact::QueryCache wrong_snapshot(fixture.lower);
    CHECK_FALSE(cache.matches(valid.query(Cache::Kind::Construction, &wrong_snapshot)));
    CHECK_FALSE(cache.publish(valid.query(Cache::Kind::Construction, &wrong_snapshot), valid.check()));
}

TEST_CASE("Bridge aware span cache refuses malformed keys without replacing valid facts", "[BridgeAwareCubic][SpanProofCache]")
{
    Fixture fixture;
    Cache cache;
    REQUIRE(cache.publish(fixture.query(), fixture.check()));
    const size_t retained = cache.statistics().units;
    for (int invalid = 0; invalid < 13; ++invalid) {
        CAPTURE(invalid);
        Fixture changed;
        if (invalid == 0) changed.gap = 0.;
        if (invalid == 1) changed.gap = -1.;
        if (invalid == 2) changed.gap = std::numeric_limits<double>::infinity();
        if (invalid == 3) changed.selection_width = 0.;
        if (invalid == 4) changed.selection_width = -1.;
        if (invalid == 5) changed.selection_width = std::numeric_limits<double>::quiet_NaN();
        if (invalid == 6) changed.phase.shift.x() = std::numeric_limits<double>::infinity();
        if (invalid == 7) changed.phase.nozzle_offset.y() = std::numeric_limits<double>::quiet_NaN();
        if (invalid == 8) changed.phase.plate_origin.x() = std::numeric_limits<double>::infinity();
        if (invalid == 9) changed.demand.front().contour.points.resize(2);
        if (invalid == 10) changed.demand.front().contour.reverse();
        if (invalid == 11) changed.demand.front().contour.points.front().x() =
            std::numeric_limits<coord_t>::max() / 4 + 1;
        if (invalid == 12) changed.demand.front().contour.points.resize(200001, Point(0, 0));
        CHECK_FALSE(cache.matches(changed.query()));
        CHECK_FALSE(Cache::valid_policy(changed.query()));
        // A default Result cannot override malformed query admission.
        CHECK_FALSE(cache.publish(changed.query(), Validation::Result{}));
        CHECK(cache.matches(fixture.query()));
        CHECK(cache.statistics().entries == 1);
        CHECK(cache.statistics().units == retained);
    }
}

TEST_CASE("Bridge aware span cache retains two bounded versions for trial rollback", "[BridgeAwareCubic][SpanProofCache]")
{
    Fixture first(8.), second(7.), third(6.);
    Cache cache;
    REQUIRE(cache.publish(first.query(), first.check()));
    REQUIRE(cache.publish(second.query(), second.check()));
    CHECK(cache.matches(first.query()));
    CHECK(cache.matches(second.query()));
    CHECK(cache.statistics().entries == 2);
    REQUIRE(cache.publish(third.query(), third.check()));
    CHECK_FALSE(cache.matches(first.query()));
    CHECK(cache.matches(second.query()));
    CHECK(cache.matches(third.query()));
    CHECK(cache.statistics().entries == 2);
    CHECK(cache.statistics().evictions >= 1);
}

TEST_CASE("Bridge aware span cache owns its evidence and bounds all retained publication state", "[BridgeAwareCubic][SpanProofCache]")
{
    Cache cache;
    {
        Fixture temporary;
        REQUIRE(cache.publish(temporary.query(), temporary.check()));
    }
    Fixture fixture;
    REQUIRE(cache.matches(fixture.query()));
    const size_t one_entry = cache.statistics().units;
    REQUIRE(one_entry > 1);
    Cache bounded(2 * one_entry - 1, 2);
    REQUIRE(bounded.publish(fixture.query(), fixture.check()));
    Fixture alternate(7.);
    CHECK_FALSE(bounded.publish(alternate.query(), alternate.check()));
    CHECK(bounded.matches(fixture.query())); // Failed staging never discards the old fact.
    CHECK(bounded.statistics().evictions == 0);
    CHECK(bounded.statistics().units <= 2 * one_entry - 1);
    CHECK(bounded.statistics().entries <= 2);
    Cache no_space(0, 0);
    CHECK_FALSE(no_space.publish(fixture.query(), fixture.check()));
    CHECK(no_space.statistics().units == 0);
    Cache entry_cap(200000, 2);
    for (int phase = 0; phase < 3; ++phase) {
        fixture.phase.shift.x() = phase;
        REQUIRE(entry_cap.publish(fixture.query(), fixture.check()));
    }
    CHECK(entry_cap.statistics().entries == 2);
    fixture.phase.shift.x() = 0.;
    CHECK_FALSE(entry_cap.matches(fixture.query()));
    fixture.phase.shift.x() = 2.;
    CHECK(entry_cap.matches(fixture.query()));

    Cache turnover(16 * one_entry, 20000);
    for (int phase = 0; phase < 40; ++phase) {
        fixture.phase.shift.x() = phase;
        REQUIRE(turnover.publish(fixture.query(), fixture.check()));
        CHECK(turnover.statistics().units <= 16 * one_entry);
    }
    CHECK(turnover.statistics().evictions > 0);
    CHECK(turnover.matches(fixture.query()));
    fixture.phase.shift.x() = 0.;
    CHECK_FALSE(turnover.matches(fixture.query()));
}

TEST_CASE("Bridge aware span cache falls back without publishing partial or canceled evidence", "[BridgeAwareCubic][SpanProofCache]")
{
    Fixture fixture;
    const auto checked = fixture.check();
    Cache cache;
    Validation::WorkBudget absent{0};
    CHECK_FALSE(cache.publish(fixture.query(), checked, &absent));
    CHECK(cache.statistics().entries == 0);
    REQUIRE(cache.publish(fixture.query(), checked));
    CHECK_FALSE(cache.matches(fixture.query(), &absent));
    CHECK(cache.matches(fixture.query()));
    Validation::WorkBudget partial{3};
    Fixture changed(7.);
    CHECK_FALSE(cache.publish(changed.query(), changed.check(), &partial));
    CHECK(partial.remaining_work < 3);
    CHECK_FALSE(cache.matches(changed.query()));
    CHECK(cache.statistics().entries <= 1);
    const auto canceled = [] { throw std::runtime_error("span cache canceled"); };
    REQUIRE_THROWS_WITH(cache.matches(fixture.query(), nullptr, canceled), "span cache canceled");
    REQUIRE_THROWS_WITH(cache.publish(changed.query(), changed.check(), nullptr, canceled), "span cache canceled");
    CHECK_FALSE(cache.matches(changed.query()));

    // Cancel at the last publication poll, after private node/key allocation.
    // Even allocation-shaped cancellation must propagate rather than fallback.
    size_t polls = 0;
    Cache counted;
    REQUIRE(counted.publish(fixture.query(), checked, nullptr, [&] { ++polls; }));
    REQUIRE(polls > 1);
    Cache interrupted;
    size_t current = 0;
    REQUIRE_THROWS_AS(interrupted.publish(fixture.query(), checked, nullptr, [&] {
        if (++current == polls) throw std::bad_alloc();
    }), std::bad_alloc);
    CHECK(interrupted.statistics().entries == 0);
    CHECK(interrupted.statistics().units == 0);
}

TEST_CASE("Complete span probe policy guards preserve budgets and cached facts", "[BridgeAwareCubic][CompleteSpans][SpanProofCache]")
{
    Fixture fixture;
    Cache cache;
    REQUIRE(cache.publish(fixture.query(), fixture.check()));
    const auto before = cache.statistics();
    Validation::WorkBudget work;
    const size_t initial = work.remaining_work;
    REQUIRE(Cache::valid_policy(fixture.query(), &work));
    const size_t used = initial - work.remaining_work;
    REQUIRE(used > 0);
    Validation::WorkBudget exact{used};
    CHECK(Cache::valid_policy(fixture.query(), &exact));
    CHECK(exact.remaining_work == 0);
    Validation::WorkBudget insufficient{used - 1};
    CHECK_FALSE(Cache::valid_policy(fixture.query(), &insufficient));
    CHECK(insufficient.remaining_work == 0);
    size_t polls = 0;
    struct PolicyCanceled {};
    CHECK_THROWS_AS(Cache::valid_policy(fixture.query(), nullptr, [&] {
        if (++polls == 2) throw PolicyCanceled{};
    }), PolicyCanceled);
    const auto after = cache.statistics();
    CHECK(after.entries == before.entries);
    CHECK(after.units == before.units);
    CHECK(after.hits == before.hits);
    CHECK(after.misses == before.misses);
    CHECK(after.publications == before.publications);
    CHECK(cache.matches(fixture.query()));
    CHECK_FALSE(Cache::valid_policy(fixture.query(Cache::Kind::Ineligible)));
}
