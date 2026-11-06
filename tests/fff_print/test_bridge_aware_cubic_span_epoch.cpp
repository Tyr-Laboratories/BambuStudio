#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSpanProofCache.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Contact = Slic3r::BridgeAwareCubicContact;
namespace Validation = Slic3r::BridgeAwareCubicValidation;
using Cache = BAC::SpanProofCache;

namespace {
constexpr Contact::EpochLimits epoch_limits{2 * 1024 * 1024, 0, 0};

ExtrusionPath span_bead(Vec2d a, Vec2d b, ExtrusionRole role = erInternalInfill)
{
    ExtrusionPath result(role, .08, .45f, .2f);
    result.polyline = Polyline(Point::new_scale(a.x(), a.y()), Point::new_scale(b.x(), b.y()));
    return result;
}

struct SpanFixture {
    BAC::SerializedXY phase;
    double upper_z = .4, gap = 8., selection_width = double(.45f);
    ExtrusionPath upper = span_bead({0., 0.}, {10., 0.});
    std::vector<ExtrusionPath> deposited{
        span_bead({-1., 0.}, {2., 0.}, erPerimeter),
        span_bead({8., 0.}, {11., 0.}, erSolidInfill),
        span_bead({0., 100.}, {10., 100.}, erGapFill)};
    ExPolygons demand{ExPolygon(Polygon(Points{
        Point::new_scale(-1., -1.), Point::new_scale(11., -1.),
        Point::new_scale(11., 1.), Point::new_scale(-1., 1.)}))};
    Contact::PreparedLowerPtr prepared;

    SpanFixture() { rebuild(); }
    void rebuild()
    {
        std::vector<Contact::Bead> beads;
        for (const auto &path : deposited) beads.push_back({&path, .2});
        prepared = Contact::prepare_lower(beads, upper_z, upper.height);
    }
    Contact::CanonicalLowerEpochPtr epoch() const
    { return Contact::capture_lower_epoch(prepared, phase, epoch_limits); }
    Cache::Query query(const Cache::EpochContext *epochs = nullptr,
                       Contact::QueryCache *masks = nullptr, bool audit = false) const
    {
        return {Cache::Kind::Construction, phase, gap, selection_width, demand,
                *prepared, upper, upper_z, masks, epochs, audit};
    }
    Validation::Result check() const
    {
        const auto contact = Contact::footprint(*prepared, upper, upper_z, 0., Contact::FootprintMode::ContactsOnly);
        Polylines unsupported;
        for (const auto &segment : contact.segments) {
            const size_t i = segment.segment_index;
            append(unsupported, diff_pl(Polylines{Polyline(upper.polyline.points[i], upper.polyline.points[i + 1])},
                                       segment.centerline_support));
        }
        if (!contact.input_valid || !contact.completed || contact.segments.empty()) unsupported = {upper.polyline};
        const auto selected = intersection_ex(demand, union_ex(offset(unsupported, scale_(.5 * selection_width))));
        return Validation::validate_contacts({}, contact.physical_footprint, {upper.polyline}, selected,
                                             gap, 0., {contact});
    }
};

Cache::EpochContext context(const SpanFixture &fixture, const Contact::CanonicalLowerEpochPtr &epoch,
                            Contact::CanonicalLowerDeltaPtr transition = {})
{ return {fixture.prepared, epoch, std::move(transition), epoch_limits}; }

Contact::CanonicalLowerDeltaPtr difference(const Contact::CanonicalLowerEpochPtr &before,
                                          const Contact::CanonicalLowerEpochPtr &after)
{
    auto result = Contact::compare_lower_epochs(before, after, epoch_limits);
    REQUIRE(result);
    REQUIRE(Contact::lower_epoch_before(*result) == before);
    REQUIRE(Contact::lower_epoch_after(*result) == after);
    return result;
}

void admit(Cache &cache, const std::vector<Contact::CanonicalLowerEpochPtr> &epochs)
{ REQUIRE(cache.admit_epochs(epochs, epoch_limits)); }

void publish(Cache &cache, const SpanFixture &fixture, const Cache::EpochContext &current)
{
    const auto checked = fixture.check();
    REQUIRE(checked.passed());
    REQUIRE_FALSE(checked.unsupported_runs.empty());
    REQUIRE(checked.max_unsupported_mm > 0.);
    REQUIRE(checked.void_cells.empty());
    REQUIRE(cache.publish(fixture.query(&current), checked));
}
}

TEST_CASE("Span epoch keeps exact span facts while remote canonical material changes",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture before, after;
    after.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    after.rebuild();
    auto first = before.epoch(), second = after.epoch();
    REQUIRE(first);
    REQUIRE(second);
    auto changed = difference(first, second);
    REQUIRE(Contact::lower_epoch_relation(*changed) == Contact::EpochRelation::Changed);
    auto current = context(before, first), next = context(after, second, changed);
    Cache cache;
    admit(cache, {first, second});
    publish(cache, before, current);
    const auto old = cache.statistics();
    REQUIRE(cache.matches(after.query(&next)));
    CHECK(cache.statistics().advances == old.advances + 1);
    CHECK(cache.statistics().epoch_hits == old.epoch_hits + 1);
    CHECK(cache.statistics().raw_checks == old.raw_checks);
    CHECK(cache.statistics().bound_handles == 2);
    REQUIRE(cache.matches(after.query(&next)));
    CHECK(cache.statistics().advances == old.advances + 1);
    CHECK(cache.statistics().raw_checks == old.raw_checks);
    REQUIRE(after.check().passed());
}

TEST_CASE("Span epoch rejects contributor removal newly occupied absence and changed merged endpoints",
          "[BridgeAwareCubic][SpanEpoch]")
{
    const int mutation = GENERATE(0, 1, 2);
    SpanFixture before, after;
    if (mutation == 0) after.deposited.erase(after.deposited.begin());
    if (mutation == 1) after.deposited.push_back(span_bead({5., -2.}, {5., 2.}));
    if (mutation == 2) after.deposited.push_back(span_bead({2., 0.}, {2.001, 0.}));
    after.rebuild();
    auto first = before.epoch(), second = after.epoch();
    REQUIRE(first);
    REQUIRE(second);
    auto current = context(before, first), next = context(after, second, difference(first, second));
    Cache cache;
    admit(cache, {first, second});
    publish(cache, before, current);
    CHECK_FALSE(cache.matches(after.query(&next)));
    CHECK(cache.statistics().advances == 0);
    CHECK(cache.statistics().bound_handles == 1);
    if (mutation == 0) {
        REQUIRE_FALSE(after.check().passed());
        CHECK_FALSE(cache.publish(after.query(&next), after.check()));
    } else {
        // Added material still needs its new exact identity and actual check.
        REQUIRE(after.check().passed());
        REQUIRE(cache.publish(after.query(&next), after.check()));
        CHECK(cache.matches(after.query(&next)));
    }
    CHECK(cache.matches(before.query(&current)));
}

TEST_CASE("Span epoch A to B to C requires each exact before binding and retires old material",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture a, b, c;
    b.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    c.deposited.back().polyline.translate(Point::new_scale(0., 40.));
    b.rebuild(); c.rebuild();
    auto ea = a.epoch(), eb = b.epoch(), ec = c.epoch();
    REQUIRE(ea); REQUIRE(eb); REQUIRE(ec);
    auto ab = difference(ea, eb), bc = difference(eb, ec);
    auto qa = context(a, ea), qb = context(b, eb, ab), qc = context(c, ec, bc);
    Cache cache;
    admit(cache, {ea, eb});
    publish(cache, a, qa);
    REQUIRE(cache.matches(b.query(&qb)));
    REQUIRE(cache.retire_epochs({ea}));
    CHECK(cache.statistics().bound_handles == 1);
    CHECK(cache.statistics().admitted_epochs == 1);
    admit(cache, {ec});
    REQUIRE(cache.matches(c.query(&qc)));
    CHECK(cache.statistics().advances == 2);
    REQUIRE(cache.retire_epochs({eb}));
    CHECK(cache.statistics().bound_handles == 1);
    CHECK(cache.statistics().admitted_epochs == 1);
    CHECK(cache.statistics().epoch_resident_bytes == Contact::lower_epoch_resident_bytes(*ec));
    const size_t discovery = cache.statistics().raw_checks;
    REQUIRE(cache.matches(c.query(&qc)));
    CHECK(cache.statistics().raw_checks == discovery);
}

TEST_CASE("Span epoch observed rejected material transitions back to a fresh restored source",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture original, rejected, restored;
    rejected.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    rejected.rebuild();
    auto ea = original.epoch(), eb = rejected.epoch(), restored_epoch = restored.epoch();
    REQUIRE(ea); REQUIRE(eb); REQUIRE(restored_epoch);
    REQUIRE(ea != restored_epoch);
    auto qa = context(original, ea), qb = context(rejected, eb, difference(ea, eb));
    Cache cache;
    admit(cache, {ea, eb});
    publish(cache, original, qa);
    REQUIRE(cache.matches(rejected.query(&qb)));
    REQUIRE(cache.retire_epochs({ea}));
    auto rollback = difference(eb, restored_epoch);
    auto qr = context(restored, restored_epoch, rollback);
    admit(cache, {restored_epoch});
    REQUIRE(cache.matches(restored.query(&qr)));
    CHECK(cache.statistics().advances == 2);
    REQUIRE(cache.retire_epochs({eb}));
    CHECK(cache.statistics().admitted_epochs == 1);
    CHECK(cache.statistics().bound_handles == 1);
    CHECK(Contact::lower_epoch_source_matches(*restored_epoch, restored.prepared, restored.phase));
    CHECK_FALSE(Contact::lower_epoch_source_matches(*ea, restored.prepared, restored.phase));
}

TEST_CASE("Span epoch demotion releases retired generations while retaining raw fallback",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture fixture;
    auto epoch = fixture.epoch();
    REQUIRE(epoch);
    std::weak_ptr<const Contact::CanonicalLowerEpoch> weak_epoch = epoch;
    auto current = context(fixture, epoch);
    Cache cache;
    admit(cache, {epoch});
    publish(cache, fixture, current);
    CHECK(cache.statistics().bound_handles == 1);
    REQUIRE(cache.retire_epochs({epoch}));
    CHECK(cache.statistics().bound_handles == 0);
    CHECK(cache.statistics().bound_units == 0);
    CHECK(cache.statistics().admitted_epochs == 0);
    CHECK(cache.statistics().epoch_resident_bytes == 0);
    CHECK(cache.statistics().entries == 1);
    current = {};
    epoch.reset();
    CHECK(weak_epoch.expired());
    const size_t discovery = cache.statistics().raw_checks;
    REQUIRE(cache.matches(fixture.query()));
    CHECK(cache.statistics().raw_checks == discovery + 1);
    auto rebound_epoch = fixture.epoch();
    REQUIRE(rebound_epoch);
    auto rebound = context(fixture, rebound_epoch);
    admit(cache, {rebound_epoch});
    REQUIRE(cache.matches(fixture.query(&rebound))); // Fresh raw equality, then new bound capture.
    CHECK(cache.statistics().bound_handles == 1);
    const size_t rebind_discovery = cache.statistics().raw_checks;
    REQUIRE(cache.matches(fixture.query(&rebound)));
    CHECK(cache.statistics().raw_checks == rebind_discovery);
    fixture.deposited.front().polyline.translate(Point::new_scale(0., 2.));
    fixture.rebuild();
    CHECK_FALSE(cache.matches(fixture.query()));
    REQUIRE_FALSE(fixture.check().passed());
}

TEST_CASE("Span epoch still compares the complete current upper and every policy component",
          "[BridgeAwareCubic][SpanEpoch]")
{
    const int mutation = GENERATE(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);
    SpanFixture fixture;
    auto epoch = fixture.epoch();
    REQUIRE(epoch);
    auto current = context(fixture, epoch);
    Cache cache;
    admit(cache, {epoch});
    publish(cache, fixture, current);
    if (mutation == 0) fixture.upper.polyline.points.back().x() -= 1;
    if (mutation == 1) fixture.upper.mm3_per_mm = std::nextafter(fixture.upper.mm3_per_mm, 1.);
    if (mutation == 2) fixture.upper.height = std::nextafter(fixture.upper.height, 1.f);
    if (mutation == 3) fixture.upper_z = std::nextafter(fixture.upper_z, 1.);
    if (mutation == 4) fixture.gap = std::nextafter(fixture.gap, 20.);
    if (mutation == 5) fixture.selection_width = std::nextafter(fixture.selection_width, 1.);
    if (mutation == 6) fixture.demand.front().contour.points.front().x() += 1;
    if (mutation == 7) {
        auto &points = fixture.demand.front().contour.points;
        std::rotate(points.begin(), points.begin() + 1, points.end());
    }
    if (mutation == 8) { fixture.phase.shift.x() += 1.; fixture.phase.plate_origin.x() += 1.; }
    if (mutation == 9) { fixture.phase.shift.y() += 1.; fixture.phase.nozzle_offset.y() += 1.; }
    if (mutation == 10) std::reverse(fixture.upper.polyline.points.begin(), fixture.upper.polyline.points.end());
    if (mutation == 11) fixture.upper.width = std::nextafter(fixture.upper.width, 1.f);
    CHECK_FALSE(cache.matches(fixture.query(&current)));
    CHECK(cache.statistics().epoch_hits == 0);
    CHECK(cache.statistics().advances == 0);
}

TEST_CASE("Span epoch foreign provenance never authorizes an epoch hit or unadmitted retention",
          "[BridgeAwareCubic][SpanEpoch]")
{
    const int mutation = GENERATE(0, 1, 2, 3, 4);
    SpanFixture a, b, foreign;
    b.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    b.rebuild();
    auto ea = a.epoch(), eb = b.epoch(), unrelated = a.epoch();
    REQUIRE(ea); REQUIRE(eb); REQUIRE(unrelated);
    auto qa = context(a, ea), qb = context(b, eb, difference(ea, eb));
    Cache cache;
    admit(cache, {ea});
    if (mutation != 0) admit(cache, {eb}); // 0: unknown current epoch.
    publish(cache, a, qa);
    if (mutation == 1) qb.prepared = foreign.prepared;
    if (mutation == 2) qb.transition = difference(unrelated, eb);
    if (mutation == 3) qb.transition = difference(eb, ea);
    if (mutation == 4) qb.current = unrelated;
    const size_t bindings = cache.statistics().bound_handles;
    // The ordinary fresh-discovery result is still true for this unchanged
    // local contact. A bad epoch context must fall back, not claim a fast hit.
    REQUIRE(cache.matches(b.query(&qb)));
    CHECK(cache.statistics().epoch_hits == 0);
    CHECK(cache.statistics().advances == 0);
    CHECK(cache.statistics().bound_handles == bindings);
    CHECK(cache.statistics().raw_checks > 0);
    Contact::QueryCache wrong_source(foreign.prepared);
    CHECK_FALSE(cache.matches(b.query(&qb, &wrong_source)));
}

TEST_CASE("Span epoch admission counts distinct retained generations and other byte-pool storage",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture a, b;
    b.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    b.rebuild();
    auto ea = a.epoch(), eb = b.epoch();
    REQUIRE(ea); REQUIRE(eb);
    const size_t a_bytes = Contact::lower_epoch_resident_bytes(*ea);
    const size_t b_bytes = Contact::lower_epoch_resident_bytes(*eb);
    REQUIRE(a_bytes > 0); REQUIRE(b_bytes > 0);
    Cache cache;
    CHECK_FALSE(cache.admit_epochs({ea}, {a_bytes - 1, 0, 0}));
    CHECK(cache.statistics().admitted_epochs == 0);
    REQUIRE(cache.admit_epochs({ea, ea}, {a_bytes, 0, 0}));
    CHECK(cache.statistics().admitted_epochs == 1);
    CHECK(cache.statistics().epoch_resident_bytes == a_bytes);
    CHECK_FALSE(cache.admit_epochs({eb}, {a_bytes + b_bytes - 1, 0, 0}));
    CHECK_FALSE(cache.admit_epochs({eb}, {a_bytes + b_bytes, 1, 0}));
    CHECK(cache.statistics().admitted_epochs == 1);
    REQUIRE(cache.admit_epochs({ea, eb, eb}, {a_bytes + b_bytes, 0, 0}));
    CHECK(cache.statistics().admitted_epochs == 2);
    CHECK(cache.statistics().epoch_resident_bytes == a_bytes + b_bytes);
    Cache no_metadata(0, 0);
    CHECK_FALSE(no_metadata.admit_epochs({ea}, epoch_limits));
    CHECK(no_metadata.statistics().units == 0);
}

TEST_CASE("Span epoch staged promotion failure and canceled retirement preserve old evidence and allowance",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture a, b;
    b.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    b.rebuild();
    auto ea = a.epoch(), eb = b.epoch();
    REQUIRE(ea); REQUIRE(eb);
    auto qa = context(a, ea), qb = context(b, eb, difference(ea, eb));
    Cache cache;
    admit(cache, {ea, eb});
    publish(cache, a, qa);
    const auto saved = cache.statistics();
    Validation::WorkBudget exhausted{0}, partial{3};
    CHECK_FALSE(cache.matches(b.query(&qb), &partial));
    CHECK(partial.remaining_work < 3);
    CHECK(cache.statistics().advances == saved.advances);
    CHECK(cache.statistics().bound_handles == saved.bound_handles);
    CHECK(cache.statistics().units == saved.units);
    CHECK_FALSE(cache.retire_epochs({ea}, &exhausted));
    CHECK(cache.statistics().admitted_epochs == saved.admitted_epochs);
    REQUIRE_THROWS_WITH(cache.retire_epochs({ea}, nullptr, [] { throw std::runtime_error("retire canceled"); }),
                        "retire canceled");
    REQUIRE_THROWS_AS(cache.matches(b.query(&qb), nullptr, [] { throw std::bad_alloc(); }), std::bad_alloc);
    CHECK(cache.statistics().bound_handles == saved.bound_handles);
    REQUIRE(cache.matches(a.query(&qa)));

    // Count the complete successful lookup, then cancel its last poll in an
    // independent cache. A staged after handle may never leak into its facts.
    Cache counted;
    admit(counted, {ea, eb}); publish(counted, a, qa);
    size_t polls = 0;
    REQUIRE(counted.matches(b.query(&qb), nullptr, [&] { ++polls; }));
    REQUIRE(polls > 1);
    Cache interrupted;
    admit(interrupted, {ea, eb}); publish(interrupted, a, qa);
    const auto before = interrupted.statistics();
    size_t current_poll = 0;
    REQUIRE_THROWS_AS(interrupted.matches(b.query(&qb), nullptr, [&] {
        if (++current_poll == polls) throw std::bad_alloc();
    }), std::bad_alloc);
    CHECK(interrupted.statistics().bound_handles == before.bound_handles);
    CHECK(interrupted.statistics().advances == before.advances);
    CHECK(interrupted.statistics().units == before.units);
    REQUIRE(interrupted.matches(a.query(&qa)));
}

TEST_CASE("Span epoch binding staging respects the existing fact allowance",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture a, b;
    b.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    b.rebuild();
    auto ea = a.epoch(), eb = b.epoch();
    REQUIRE(ea); REQUIRE(eb);
    auto qa = context(a, ea), qb = context(b, eb, difference(ea, eb));
    Cache measured;
    admit(measured, {ea, eb}); publish(measured, a, qa);
    const size_t exact_resident = measured.statistics().units;
    REQUIRE(exact_resident > 0);
    Cache bounded(exact_resident);
    admit(bounded, {ea, eb}); publish(bounded, a, qa);
    const auto saved = bounded.statistics();
    // No room for a simultaneous bound handle: fresh raw discovery can still
    // prove this query, but the old binding cannot be evicted to stage a hit.
    REQUIRE(bounded.matches(b.query(&qb)));
    CHECK(bounded.statistics().advances == 0);
    CHECK(bounded.statistics().bound_handles == saved.bound_handles);
    CHECK(bounded.statistics().units == saved.units);
    CHECK(bounded.statistics().raw_checks > saved.raw_checks);
    REQUIRE(bounded.matches(a.query(&qa)));
}

TEST_CASE("Span epoch publication excludes incomplete failed and physical-cell results",
          "[BridgeAwareCubic][SpanEpoch]")
{
    const int invalid = GENERATE(0, 1, 2, 3, 4, 5);
    SpanFixture fixture;
    auto epoch = fixture.epoch();
    REQUIRE(epoch);
    auto current = context(fixture, epoch);
    auto checked = fixture.check();
    REQUIRE(checked.passed());
    REQUIRE_FALSE(checked.unsupported_runs.empty());
    if (invalid == 0) checked.input_valid = false;
    if (invalid == 1) checked.completed = false;
    if (invalid == 2) checked.resource_limited = true;
    if (invalid == 3) checked.failure_reason = "incomplete actual proof";
    if (invalid == 4) checked.unsupported_runs.front().exceeds_gap = true;
    if (invalid == 5) checked.void_cells.push_back({fixture.demand.front(), 1., false, true});
    Cache cache;
    admit(cache, {epoch});
    CHECK_FALSE(cache.publish(fixture.query(&current), checked));
    CHECK(cache.statistics().bound_handles == 0);
    CHECK(cache.statistics().entries == 0);
    CHECK_FALSE(cache.matches(fixture.query(&current)));
}

TEST_CASE("Span epoch fast hits support a same-budget complete-signature differential audit",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture a, b;
    b.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    b.rebuild();
    auto ea = a.epoch(), eb = b.epoch();
    REQUIRE(ea); REQUIRE(eb);
    auto qa = context(a, ea), qb = context(b, eb, difference(ea, eb));
    Cache cache;
    admit(cache, {ea, eb}); publish(cache, a, qa);
    Validation::WorkBudget budget;
    const size_t initial = budget.remaining_work;
    REQUIRE(cache.matches(b.query(&qb, nullptr, true), &budget));
    CHECK(budget.remaining_work < initial);
    CHECK(cache.statistics().epoch_hits == 1);
    CHECK(cache.statistics().epoch_audits == 1);
    CHECK(cache.statistics().epoch_audit_mismatches == 0);
    CHECK(cache.statistics().raw_checks == 1);
    REQUIRE(cache.matches(b.query(&qb)));
    CHECK(cache.statistics().epoch_audits == 1);
    CHECK(cache.statistics().raw_checks == 1);
}

TEST_CASE("Span epoch third observed generation cannot displace either retained binding while staging",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture a, b, c;
    b.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    c.deposited.back().polyline.translate(Point::new_scale(0., 40.));
    b.rebuild(); c.rebuild();
    auto ea = a.epoch(), eb = b.epoch(), ec = c.epoch();
    REQUIRE(ea); REQUIRE(eb); REQUIRE(ec);
    auto qa = context(a, ea), qb = context(b, eb, difference(ea, eb));
    auto qc = context(c, ec, difference(eb, ec));
    Cache cache;
    admit(cache, {ea, eb, ec});
    publish(cache, a, qa);
    REQUIRE(cache.matches(b.query(&qb)));
    REQUIRE(cache.statistics().bound_handles == 2);
    const auto saved = cache.statistics();
    REQUIRE(cache.matches(c.query(&qc))); // Complete raw discovery remains valid.
    CHECK(cache.statistics().raw_checks == saved.raw_checks + 1);
    CHECK(cache.statistics().advances == saved.advances);
    CHECK(cache.statistics().bound_handles == 2);
    CHECK(cache.statistics().units == saved.units);
    REQUIRE(cache.matches(a.query(&qa)));
    REQUIRE(cache.matches(b.query(&qb)));
    REQUIRE(cache.retire_epochs({ea}));
    REQUIRE(cache.matches(c.query(&qc)));
    CHECK(cache.statistics().advances == saved.advances + 1);
    CHECK(cache.statistics().bound_handles == 2);
}

TEST_CASE("Span epoch incomplete differential audit throws before publishing a staged advance",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture a, b;
    b.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    b.rebuild();
    auto ea = a.epoch(), eb = b.epoch();
    REQUIRE(ea); REQUIRE(eb);
    auto qa = context(a, ea), qb = context(b, eb, difference(ea, eb));
    Cache measured;
    admit(measured, {ea, eb}); publish(measured, a, qa);
    Validation::WorkBudget complete;
    const size_t initial = complete.remaining_work;
    REQUIRE(measured.matches(b.query(&qb), &complete));
    const size_t fast_work = initial - complete.remaining_work;
    REQUIRE(fast_work > 0);

    Cache audited;
    admit(audited, {ea, eb}); publish(audited, a, qa);
    const auto before = audited.statistics();
    Validation::WorkBudget insufficient_reference{fast_work};
    REQUIRE_THROWS_WITH(audited.matches(b.query(&qb, nullptr, true), &insufficient_reference),
                        "Span epoch audit mismatch or unavailable raw-signature reference");
    CHECK(insufficient_reference.remaining_work == 0);
    CHECK(audited.statistics().epoch_audits == 1);
    CHECK(audited.statistics().epoch_audit_mismatches == 1);
    CHECK(audited.statistics().advances == before.advances);
    CHECK(audited.statistics().bound_handles == before.bound_handles);
    CHECK(audited.statistics().units == before.units);
    REQUIRE(audited.matches(a.query(&qa)));
    REQUIRE(audited.matches(b.query(&qb, nullptr, true)));
    CHECK(audited.statistics().advances == before.advances + 1);
}

TEST_CASE("Span epoch shared raw payload uses wrapper-only promotion but full fresh-rebind staging",
          "[BridgeAwareCubic][SpanEpoch]")
{
    SpanFixture a, b;
    b.deposited.back().polyline.translate(Point::new_scale(0., 20.));
    b.rebuild();
    auto ea = a.epoch(), eb = b.epoch();
    REQUIRE(ea); REQUIRE(eb);
    auto qa = context(a, ea), qb = context(b, eb, difference(ea, eb));
    auto bound = Contact::capture_epoch_query_signature(ea, a.prepared, a.phase, a.upper, a.upper_z, 0.);
    REQUIRE(bound);
    const auto &raw = Contact::epoch_query_signature_raw(*bound);
    REQUIRE(raw);
    const size_t raw_units = Contact::query_signature_units(*raw);
    const size_t bound_units = Contact::epoch_query_signature_units(*bound);
    REQUIRE(bound_units > raw_units);
    const size_t wrapper = bound_units - raw_units;

    Cache measured, registry;
    admit(registry, {ea});
    const size_t registry_units = registry.statistics().units;
    admit(measured, {ea, eb}); publish(measured, a, qa);
    const size_t resident = measured.statistics().units;
    REQUIRE(measured.statistics().bound_units == wrapper);

    Cache exact(resident + wrapper);
    admit(exact, {ea, eb}); publish(exact, a, qa);
    REQUIRE(exact.statistics().units == resident);
    REQUIRE(exact.matches(b.query(&qb)));
    CHECK(exact.statistics().advances == 1);
    CHECK(exact.statistics().raw_checks == 0);
    CHECK(exact.statistics().units == resident + wrapper);
    CHECK(exact.statistics().bound_units == 2 * wrapper);
    REQUIRE(exact.retire_epochs({ea}));
    CHECK(exact.statistics().units == resident - registry_units);
    CHECK(exact.statistics().bound_units == wrapper);
    REQUIRE(exact.matches(b.query(&qb)));

    // Demote and readmit the same source under a cap leaving one wrapper's
    // room. A fresh capture owns a NEW raw payload while the old is still
    // retained, so that room is insufficient despite identical geometry.
    Cache rebind(resident);
    admit(rebind, {ea, eb}); publish(rebind, a, qa);
    REQUIRE(rebind.retire_epochs({ea}));
    admit(rebind, {ea});
    REQUIRE(rebind.statistics().units == resident - wrapper);
    REQUIRE(rebind.matches(a.query(&qa)));
    CHECK(rebind.statistics().raw_checks == 1);
    CHECK(rebind.statistics().bound_handles == 0);
    CHECK(rebind.statistics().bound_units == 0);
    CHECK(rebind.statistics().units == resident - wrapper);
    CHECK(rebind.statistics().entries == 1);
}
