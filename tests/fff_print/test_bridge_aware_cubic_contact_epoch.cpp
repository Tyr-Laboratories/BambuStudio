#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSerialization.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>

using namespace Slic3r;
namespace Contact = Slic3r::BridgeAwareCubicContact;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
constexpr size_t capacity = 1024 * 1024;
constexpr Contact::EpochLimits limits{capacity, 0, 0};

ExtrusionPath bead(Vec2d a, Vec2d b, ExtrusionRole role = erInternalInfill, float width = .45f)
{
    ExtrusionPath result(role, .08, width, .2f);
    result.polyline = Polyline(Point::new_scale(a.x(), a.y()), Point::new_scale(b.x(), b.y()));
    return result;
}

struct Fixture {
    BAC::SerializedXY phase;
    double lower_z = .2, upper_z = .4;
    ExtrusionPath upper = bead({0., 0.}, {10., 0.});
    std::vector<ExtrusionPath> lower_paths{
        bead({-1., 0.}, {2., 0.}, erPerimeter),
        bead({8., 0.}, {11., 0.}, erSolidInfill),
        bead({0., 100.}, {10., 100.}, erGapFill)};
    Contact::PreparedLowerPtr prepared;

    Fixture() { rebuild(); }
    void rebuild(Validation::WorkBudget *budget = nullptr)
    {
        std::vector<Contact::Bead> beads;
        for (const auto &path : lower_paths) beads.push_back({&path, lower_z});
        prepared = Contact::prepare_lower(beads, upper_z, upper.height, budget);
    }
    Contact::CanonicalLowerEpochPtr epoch(Contact::EpochLimits quota = limits,
        Validation::WorkBudget *budget = nullptr) const
    { return Contact::capture_lower_epoch(prepared, phase, quota, budget); }
    Contact::EpochQuerySignaturePtr signature(const Contact::CanonicalLowerEpochPtr &epoch) const
    { return Contact::capture_epoch_query_signature(epoch, prepared, phase, upper, upper_z, 0.); }
};

Contact::CanonicalLowerDeltaPtr delta(const Contact::CanonicalLowerEpochPtr &before,
                                     const Contact::CanonicalLowerEpochPtr &after)
{
    auto result = Contact::compare_lower_epochs(before, after, limits);
    REQUIRE(result);
    REQUIRE(Contact::lower_epoch_before(*result) == before);
    REQUIRE(Contact::lower_epoch_after(*result) == after);
    return result;
}

Contact::EpochQueryAdvance advance(const Contact::EpochQuerySignaturePtr &signature,
    const Contact::CanonicalLowerDeltaPtr &difference, const Fixture &current)
{
    return Contact::advance_epoch_query_signature(*signature, *difference,
        current.prepared, current.phase, limits);
}

bool matches(const Contact::EpochQuerySignaturePtr &signature, const Fixture &current)
{
    return Contact::matches_epoch_query_signature(*signature, current.prepared, current.phase,
        current.upper, current.upper_z, 0.);
}
}

TEST_CASE("Contact epoch owns a complete canonical physical identity and exact prepared binding",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture fixture;
    const auto epoch = fixture.epoch();
    REQUIRE(epoch);
    CHECK(Contact::lower_epoch_source_matches(*epoch, fixture.prepared, fixture.phase));
    CHECK(Contact::lower_epoch_owned_bytes(*epoch) > 0);
    CHECK(Contact::lower_epoch_owned_bytes(*epoch) <= capacity);
    CHECK(Contact::lower_epoch_resident_bytes(*epoch) == Contact::lower_epoch_owned_bytes(*epoch));
    CHECK(Contact::prepared_lower_contact_units(*fixture.prepared) > 0);
    const size_t epoch_bytes = Contact::lower_epoch_owned_bytes(*epoch);
    CHECK(Contact::lower_epoch_contact_units(*epoch) ==
          epoch_bytes / sizeof(Point) + (epoch_bytes % sizeof(Point) != 0));
    const auto signature = fixture.signature(epoch);
    REQUIRE(signature);
    CHECK(Contact::query_signature_epoch(*signature) == epoch);
    REQUIRE(Contact::epoch_query_signature_raw(*signature));
    CHECK(Contact::epoch_query_signature_units(*signature) >
          Contact::query_signature_units(*Contact::epoch_query_signature_raw(*signature)));
    CHECK(matches(signature, fixture));

    Fixture identical_but_distinct;
    CHECK_FALSE(Contact::lower_epoch_source_matches(*epoch, identical_but_distinct.prepared, fixture.phase));
    CHECK_FALSE(identical_but_distinct.signature(epoch));
    CHECK_FALSE(matches(signature, identical_but_distinct));
    const Contact::PreparedLowerPtr foreign_owner(fixture.prepared.get(), [](const Contact::PreparedLower *) {});
    CHECK_FALSE(Contact::lower_epoch_source_matches(*epoch, foreign_owner, fixture.phase));
    CHECK_FALSE(Contact::capture_epoch_query_signature(epoch, foreign_owner, fixture.phase,
        fixture.upper, fixture.upper_z, 0.));
    Contact::QueryCache foreign(identical_but_distinct.prepared);
    CHECK_FALSE(Contact::capture_lower_epoch(fixture.prepared, fixture.phase, limits, nullptr, {}, &foreign));
    CHECK_FALSE(Contact::capture_epoch_query_signature(epoch, fixture.prepared, fixture.phase,
        fixture.upper, fixture.upper_z, 0., nullptr, 8192, {}, &foreign));
    const std::weak_ptr<const Contact::PreparedLower> old_source = fixture.prepared;
    fixture.prepared.reset();
    CHECK(old_source.expired());
    CHECK_FALSE(Contact::lower_epoch_source_matches(*epoch, fixture.prepared, fixture.phase));
}

TEST_CASE("Contact epoch canonical equality includes exact merging and consumes source multiplicity",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture before;
    before.lower_paths = {bead({-10., 0.}, {10., 0.}), bead({0., 20.}, {10., 20.})};
    before.rebuild();
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    after.lower_paths = {bead({10., 20.}, {0., 20.}), bead({0., 0.}, {10., 0.}),
                         bead({-10., 0.}, {0., 0.}), bead({-10., 0.}, {0., 0.})};
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    const auto difference = delta(first, second);
    CHECK(Contact::lower_epoch_relation(*difference) == Contact::EpochRelation::Identical);
    const auto transported = advance(signature, difference, after);
    REQUIRE(transported.impact == Contact::QueryImpact::Unaffected);
    REQUIRE(transported.signature);
    CHECK(Contact::query_signature_epoch(*transported.signature) == second);
    CHECK(matches(transported.signature, after));
    CHECK(Contact::matches_query_signature(*Contact::epoch_query_signature_raw(*signature),
        *after.prepared, after.upper, after.upper_z, 0.));
}

TEST_CASE("Contact epoch validates all deposition roles and detects newly occupied candidate space",
          "[BridgeAwareCubic][ContactEpoch]")
{
    const ExtrusionRole role = GENERATE(erPerimeter, erExternalPerimeter, erSolidInfill, erGapFill,
                                       erInternalInfill, erSupportMaterial);
    Fixture before;
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    after.lower_paths.push_back(bead({5., -2.}, {5., 2.}, role));
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    const auto difference = delta(first, second);
    CHECK(Contact::lower_epoch_relation(*difference) == Contact::EpochRelation::Changed);
    CHECK_FALSE(Contact::matches_query_signature(*Contact::epoch_query_signature_raw(*signature),
        *after.prepared, after.upper, after.upper_z, 0.));
    const auto transported = advance(signature, difference, after);
    CHECK(transported.impact == Contact::QueryImpact::Changed);
    CHECK_FALSE(transported.signature);
    CHECK(matches(signature, before));
}

TEST_CASE("Contact epoch advancement binds both sides and rejects reversed or unrelated provenance",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture before;
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    after.lower_paths.back().polyline.translate(Point::new_scale(0., 10.));
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    const auto difference = delta(first, second);
    REQUIRE(Contact::lower_epoch_relation(*difference) == Contact::EpochRelation::Changed);
    const auto transported = advance(signature, difference, after);
    REQUIRE(transported.impact == Contact::QueryImpact::Unaffected);
    REQUIRE(transported.signature);
    CHECK(matches(transported.signature, after));
    CHECK_FALSE(matches(signature, after));

    SECTION("reversed before binding") {
        const auto reversed = delta(second, first);
        const auto rejected = advance(signature, reversed, before);
        CHECK(rejected.impact == Contact::QueryImpact::Unavailable);
        CHECK_FALSE(rejected.signature);
    }
    SECTION("equivalent independently captured before handle") {
        const auto unrelated_before = before.epoch();
        REQUIRE(unrelated_before);
        const auto unrelated = delta(unrelated_before, second);
        const auto rejected = advance(signature, unrelated, after);
        CHECK(rejected.impact == Contact::QueryImpact::Unavailable);
        CHECK_FALSE(rejected.signature);
    }
    SECTION("current source is before instead of after") {
        const auto rejected = advance(signature, difference, before);
        CHECK(rejected.impact == Contact::QueryImpact::Unavailable);
        CHECK_FALSE(rejected.signature);
    }
    SECTION("current source has same values but different ownership") {
        Fixture unrelated_after = after;
        unrelated_after.rebuild();
        const auto rejected = advance(signature, difference, unrelated_after);
        CHECK(rejected.impact == Contact::QueryImpact::Unavailable);
        CHECK_FALSE(rejected.signature);
    }
    SECTION("before prepared geometry has already been released") {
        const std::weak_ptr<const Contact::PreparedLower> old_source = before.prepared;
        before.prepared.reset();
        CHECK(old_source.expired());
        const auto independent_difference = delta(first, second);
        const auto retained = advance(signature, independent_difference, after);
        REQUIRE(retained.impact == Contact::QueryImpact::Unaffected);
        REQUIRE(retained.signature);
        CHECK(matches(retained.signature, after));
        CHECK(Contact::epoch_query_signature_raw(*retained.signature) ==
              Contact::epoch_query_signature_raw(*signature));
    }
    SECTION("legitimate second promotion uses the transported epoch") {
        Fixture third = after;
        third.lower_paths.back().polyline.translate(Point::new_scale(0., 10.));
        third.rebuild();
        const auto third_epoch = third.epoch();
        REQUIRE(third_epoch);
        const auto next = delta(second, third_epoch);
        const auto next_signature = advance(transported.signature, next, third);
        REQUIRE(next_signature.impact == Contact::QueryImpact::Unaffected);
        REQUIRE(next_signature.signature);
        CHECK(matches(next_signature.signature, third));
        CHECK(advance(signature, next, third).impact == Contact::QueryImpact::Unavailable);
    }
}

TEST_CASE("Contact epoch phase and contact plane compatibility are exact",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture before;
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    SECTION("shift and nozzle changes with unchanged net translation") {
        after.phase.shift.x() = .5;
        after.phase.nozzle_offset.x() = .5;
    }
    SECTION("shift and plate changes with unchanged net translation") {
        after.phase.shift.y() = .5;
        after.phase.plate_origin.y() = .5;
    }
    SECTION("upper Z") { after.upper_z += .001; }
    SECTION("upper float height") { after.upper.height = std::nextafter(after.upper.height, 1.f); }
    SECTION("selected contact plane") { after.lower_z = .22; }
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    const auto difference = delta(first, second);
    CHECK(Contact::lower_epoch_relation(*difference) == Contact::EpochRelation::Incompatible);
    const auto transported = advance(signature, difference, after);
    CHECK(transported.impact == Contact::QueryImpact::Changed);
    CHECK_FALSE(transported.signature);
}

TEST_CASE("Contact epoch distinguishes complete empty material from failed preparation",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture empty;
    empty.lower_paths.clear();
    empty.rebuild();
    const auto first = empty.epoch();
    REQUIRE(first);
    const auto identical = delta(first, first);
    CHECK(Contact::lower_epoch_relation(*identical) == Contact::EpochRelation::Identical);
    CHECK(Contact::lower_delta_resident_bytes(*identical) ==
          Contact::lower_delta_owned_bytes(*identical) + Contact::lower_epoch_owned_bytes(*first));
    SECTION("invalid remote bead still declines the entire epoch") {
        Fixture invalid;
        invalid.lower_paths.back().height = -1.f;
        invalid.rebuild();
        CHECK_FALSE(invalid.epoch());
    }
    SECTION("out of plane invalid bead still declines") {
        Fixture invalid;
        invalid.lower_z = 10.;
        invalid.lower_paths.front().mm3_per_mm = 0.;
        invalid.rebuild();
        CHECK_FALSE(invalid.epoch());
    }
    SECTION("resource limited preparation is not empty geometry") {
        Fixture exhausted;
        Validation::WorkBudget work;
        work.remaining_work = 0;
        exhausted.rebuild(&work);
        CHECK_FALSE(exhausted.epoch());
    }
    SECTION("missing source") {
        CHECK_FALSE(Contact::capture_lower_epoch({}, empty.phase, limits));
    }
    SECTION("nonfinite phase") {
        empty.phase.shift.x() = std::numeric_limits<double>::quiet_NaN();
        CHECK_FALSE(empty.epoch());
    }
}

TEST_CASE("Contact epoch dirty queries retain zero width axes and endpoint anchors",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture before;
    SECTION("transverse tangent line at an upper endpoint") {
        before.lower_paths = {bead({0., -1.}, {0., 1.}, erPerimeter, .2f)};
    }
    SECTION("collinear tangent axes") {
        before.upper.width = before.upper.height;
        before.lower_paths = {bead({-1., 0.}, {11., 0.}, erPerimeter, .2f)};
    }
    before.rebuild();
    const auto contact = Contact::footprint(*before.prepared, before.upper, before.upper_z, 0.,
                                           Contact::FootprintMode::ContactsOnly);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    REQUIRE(std::any_of(contact.segments.begin(), contact.segments.end(),
        [](const auto &segment) { return !segment.line_contacts.empty(); }));
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    after.lower_paths.clear();
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    const auto transported = advance(signature, delta(first, second), after);
    CHECK(transported.impact == Contact::QueryImpact::Changed);
    CHECK_FALSE(transported.signature);
}

TEST_CASE("Contact epoch dirties whole old and new merged axes beyond a source change",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture before;
    before.upper = bead({-90., 0.}, {-80., 0.});
    before.lower_paths = {bead({-100., 0.}, {0., 0.}), bead({0., 0.}, {100., 0.})};
    before.rebuild();
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    SECTION("remote extension") { after.lower_paths.push_back(bead({100., 0.}, {110., 0.})); }
    SECTION("remote split") {
        after.lower_paths.back() = bead({1., 0.}, {100., 0.});
    }
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    const auto difference = delta(first, second);
    CHECK(Contact::lower_epoch_relation(*difference) == Contact::EpochRelation::Changed);
    CHECK_FALSE(Contact::matches_query_signature(*Contact::epoch_query_signature_raw(*signature),
        *after.prepared, after.upper, after.upper_z, 0.));
    const auto transported = advance(signature, difference, after);
    CHECK(transported.impact == Contact::QueryImpact::Changed);
    CHECK_FALSE(transported.signature);
}

TEST_CASE("Contact epoch examines every segment of the complete upper path",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture before;
    before.upper.polyline = Polyline(Points{Point::new_scale(0., 0.), Point::new_scale(10., 0.),
        Point::new_scale(10., 10.), Point::new_scale(20., 10.)});
    before.rebuild();
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    after.lower_paths.push_back(bead({19., 9.}, {19., 11.}, erSolidInfill));
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    const auto transported = advance(signature, delta(first, second), after);
    CHECK(transported.impact == Contact::QueryImpact::Changed);
    CHECK_FALSE(transported.signature);
}

TEST_CASE("Contact epoch unchanged decisions agree with the original candidate signature oracle",
          "[BridgeAwareCubic][ContactEpoch]")
{
    const double y = GENERATE(-100., -.25004, -.25, -.24996, 0., .24996, .25, .25004, 100.);
    const bool diagonal = GENERATE(false, true);
    Fixture before;
    if (diagonal) before.upper = bead({0., 0.}, {10., 10.});
    before.lower_paths = {bead({-2., -100.}, {12., -100.})};
    before.rebuild();
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    after.lower_paths.push_back(diagonal ? bead({0., y}, {10., 10. + y}) : bead({0., y}, {10., y}));
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    const bool unchanged = Contact::matches_query_signature(*Contact::epoch_query_signature_raw(*signature),
        *after.prepared, after.upper, after.upper_z, 0.);
    const auto transported = advance(signature, delta(first, second), after);
    CHECK(transported.impact == (unchanged ? Contact::QueryImpact::Unaffected : Contact::QueryImpact::Changed));
    CHECK(bool(transported.signature) == unchanged);
    if (transported.signature) CHECK(matches(transported.signature, after));
}

TEST_CASE("Contact epoch query lookup preserves current upper guards after promotion",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture fixture;
    const auto epoch = fixture.epoch();
    REQUIRE(epoch);
    const auto signature = fixture.signature(epoch);
    REQUIRE(signature);
    REQUIRE(matches(signature, fixture));
    SECTION("point") { fixture.upper.polyline.points.back().x() += 1; }
    SECTION("width") { fixture.upper.width = std::nextafter(fixture.upper.width, 1.f); }
    SECTION("flow") { fixture.upper.mm3_per_mm = std::nextafter(fixture.upper.mm3_per_mm, 1.); }
    SECTION("role") { fixture.upper.set_extrusion_role(erPerimeter); }
    SECTION("suppressed extrusion") { fixture.upper.set_force_no_extrusion(true); }
    SECTION("fitted representation") { fixture.upper.polyline.fitting_result.emplace_back(); }
    SECTION("phase component") { fixture.phase.plate_origin.x() = .001; }
    CHECK_FALSE(matches(signature, fixture));
}

TEST_CASE("Contact epoch dirty checks preserve normal rejection inside overlapping axis boxes",
          "[BridgeAwareCubic][ContactEpoch]")
{
    const double rise = GENERATE(6., 10.);
    Fixture before;
    before.upper = bead({0., 0.}, {10., rise});
    before.lower_paths = {bead({-2., -100.}, {12., -100.})};
    before.rebuild();
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    // The XY boxes overlap substantially; only the finite normal-strip test
    // rejects this parallel line. Include a direction without the 45deg index shortcut.
    after.lower_paths.push_back(bead({0., 1.}, {10., rise + 1.}));
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    const auto difference = delta(first, second);
    REQUIRE(Contact::lower_epoch_relation(*difference) == Contact::EpochRelation::Changed);
    REQUIRE(Contact::matches_query_signature(*Contact::epoch_query_signature_raw(*signature),
        *after.prepared, after.upper, after.upper_z, 0.));
    const auto transported = advance(signature, difference, after);
    REQUIRE(transported.impact == Contact::QueryImpact::Unaffected);
    REQUIRE(transported.signature);
    CHECK(matches(transported.signature, after));
}

TEST_CASE("Contact epoch dirty checks agree with indexed candidate order above 64 sections",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture before;
    before.lower_paths.clear();
    for (size_t i = 0; i < 96; ++i)
        before.lower_paths.push_back(bead({.25 * i, -1.}, {.25 * i, 1.}));
    before.rebuild();
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    bool unchanged = true;
    SECTION("remote change") { after.lower_paths.back().polyline.translate(Point::new_scale(0., 5.)); }
    SECTION("source order reversed") { std::reverse(after.lower_paths.begin(), after.lower_paths.end()); }
    SECTION("new nearby section") {
        after.lower_paths.push_back(bead({4.9, -.5}, {5.1, .5}));
        unchanged = false;
    }
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    REQUIRE(Contact::matches_query_signature(*Contact::epoch_query_signature_raw(*signature),
        *after.prepared, after.upper, after.upper_z, 0.) == unchanged);
    const auto transported = advance(signature, delta(first, second), after);
    CHECK(transported.impact == (unchanged ? Contact::QueryImpact::Unaffected : Contact::QueryImpact::Changed));
    CHECK(bool(transported.signature) == unchanged);
    if (transported.signature) CHECK(matches(transported.signature, after));
}

TEST_CASE("Contact epoch quota and work failures cannot publish partial advancement",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture before;
    const auto first = before.epoch();
    REQUIRE(first);
    const auto signature = before.signature(first);
    REQUIRE(signature);
    Fixture after = before;
    after.lower_paths.back().polyline.translate(Point::new_scale(0., 10.));
    after.rebuild();
    const auto second = after.epoch();
    REQUIRE(second);
    const auto difference = delta(first, second);
    CHECK(Contact::lower_delta_resident_bytes(*difference) == Contact::lower_delta_owned_bytes(*difference) +
          Contact::lower_epoch_owned_bytes(*first) + Contact::lower_epoch_owned_bytes(*second));
    const size_t delta_bytes = Contact::lower_delta_owned_bytes(*difference);
    CHECK(Contact::lower_delta_owned_contact_units(*difference) ==
          delta_bytes / sizeof(Point) + (delta_bytes % sizeof(Point) != 0));
    CHECK_FALSE(before.epoch({0, 0, 0}));
    CHECK_FALSE(before.epoch({Contact::lower_epoch_owned_bytes(*first) - 1, 0, 0}));
    CHECK_FALSE(before.epoch({capacity, capacity + 1, 0}));
    CHECK_FALSE(before.epoch({capacity, 0, std::numeric_limits<size_t>::max()}));
    CHECK_FALSE(Contact::compare_lower_epochs(first, second, {1, 0, 0}));
    CHECK_FALSE(Contact::compare_lower_epochs(first, second,
        {Contact::lower_delta_resident_bytes(*difference) - 1, 0, 0}));
    CHECK_FALSE(Contact::compare_lower_epochs(first, second, {capacity, capacity + 1, 0}));
    CHECK_FALSE(Contact::compare_lower_epochs(first, second,
        {capacity, 0, std::numeric_limits<size_t>::max()}));
    REQUIRE(Contact::compare_lower_epochs(first, second,
        {capacity, 0, Contact::prepared_lower_contact_units(*after.prepared)}));
    const auto unavailable = Contact::advance_epoch_query_signature(*signature, *difference,
        after.prepared, after.phase, {1, 0, 0});
    CHECK(unavailable.impact == Contact::QueryImpact::Unavailable);
    CHECK_FALSE(unavailable.signature);

    Validation::WorkBudget exhausted;
    exhausted.remaining_work = 0;
    CHECK_FALSE(before.epoch(limits, &exhausted));
    CHECK_FALSE(Contact::compare_lower_epochs(first, second, limits, &exhausted));
    CHECK_FALSE(Contact::capture_epoch_query_signature(first, before.prepared, before.phase,
        before.upper, before.upper_z, 0., &exhausted));
    const auto limited = Contact::advance_epoch_query_signature(*signature, *difference,
        after.prepared, after.phase, limits, &exhausted);
    CHECK(limited.impact == Contact::QueryImpact::Unavailable);
    CHECK_FALSE(limited.signature);
    CHECK(exhausted.remaining_work == 0);
    CHECK(Contact::query_signature_epoch(*signature) == first);
    CHECK(matches(signature, before));
    const auto retried = advance(signature, difference, after);
    REQUIRE(retried.impact == Contact::QueryImpact::Unaffected);
    REQUIRE(retried.signature);
    CHECK(matches(retried.signature, after));
}

TEST_CASE("Contact epoch cancellation preserves old identity and propagates allocation-shaped exceptions",
          "[BridgeAwareCubic][ContactEpoch]")
{
    Fixture fixture;
    const auto epoch = fixture.epoch();
    REQUIRE(epoch);
    const auto signature = fixture.signature(epoch);
    REQUIRE(signature);
    const auto difference = delta(epoch, epoch);
    const auto canceled = [] { throw std::runtime_error("epoch canceled"); };
    CHECK_THROWS_WITH(Contact::capture_lower_epoch(fixture.prepared, fixture.phase, limits, nullptr, canceled), "epoch canceled");
    CHECK_THROWS_WITH(Contact::compare_lower_epochs(epoch, epoch, limits, nullptr, canceled), "epoch canceled");
    CHECK_THROWS_WITH(Contact::capture_epoch_query_signature(epoch, fixture.prepared, fixture.phase,
        fixture.upper, fixture.upper_z, 0., nullptr, 8192, canceled), "epoch canceled");
    CHECK_THROWS_WITH(Contact::advance_epoch_query_signature(*signature, *difference, fixture.prepared,
        fixture.phase, limits, nullptr, canceled), "epoch canceled");
    CHECK_THROWS_WITH(Contact::matches_epoch_query_signature(*signature, fixture.prepared, fixture.phase,
        fixture.upper, fixture.upper_z, 0., nullptr, canceled), "epoch canceled");
    const auto bad_alloc_cancel = [] { throw std::bad_alloc(); };
    CHECK_THROWS_AS(Contact::capture_lower_epoch(fixture.prepared, fixture.phase, limits, nullptr, bad_alloc_cancel), std::bad_alloc);
    CHECK_THROWS_AS(Contact::advance_epoch_query_signature(*signature, *difference, fixture.prepared,
        fixture.phase, limits, nullptr, bad_alloc_cancel), std::bad_alloc);
    Validation::WorkBudget spent;
    const size_t initial_work = spent.remaining_work;
    size_t polls = 0;
    const auto late_cancel = [&] { if (++polls == 2) throw std::runtime_error("late epoch cancellation"); };
    CHECK_THROWS_WITH(Contact::capture_lower_epoch(fixture.prepared, fixture.phase, limits, &spent, late_cancel),
        "late epoch cancellation");
    CHECK(polls == 2);
    CHECK(spent.remaining_work < initial_work);
    const size_t remaining_after_capture = spent.remaining_work;
    polls = 0;
    CHECK_THROWS_WITH(Contact::advance_epoch_query_signature(*signature, *difference, fixture.prepared,
        fixture.phase, limits, &spent, late_cancel), "late epoch cancellation");
    CHECK(polls == 2);
    CHECK(spent.remaining_work < remaining_after_capture);
    CHECK(Contact::query_signature_epoch(*signature) == epoch);
    CHECK(matches(signature, fixture));
}
