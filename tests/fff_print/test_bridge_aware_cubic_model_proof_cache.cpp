#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicModelProofCache.hpp"

#include <string>
#include <type_traits>
#include <vector>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Contact = Slic3r::BridgeAwareCubicContact;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
using Cache = BAC::ModelLayerProofCache;
static_assert(!std::is_copy_constructible_v<Cache> && !std::is_move_constructible_v<Cache>,
              "Live proof transactions borrow their cache owner");

ExtrusionPath proof_bead(double y, float width = .45f, float height = .2f, bool lower = false)
{
    ExtrusionPath path(erInternalInfill, .08, width, height);
    path.polyline = Polyline::new_scale({Vec2d(lower ? -1. : 0., y), Vec2d(lower ? 11. : 10., y)});
    return path;
}

ExPolygons proof_demand()
{
    return {ExPolygon(Polygon({Point::new_scale(-2., -1.), Point::new_scale(12., -1.),
                               Point::new_scale(12., 1.), Point::new_scale(-2., 1.)}))};
}

struct ProofFixture {
    int frozen_context = 0;
    size_t layer = 7;
    const std::vector<size_t> targets{11, 17};
    ExtrusionPath lower = proof_bead(0., .45f, .2f, true);
    ExtrusionPath first = proof_bead(.1), last = proof_bead(.2);
    Cache::Sources sources{&lower}; // This vector outlives every transaction.
    double lower_z = .2, upper_z = .4;
    BAC::SerializedXY phase;

    Cache cache(size_t max_vertices = 200000, size_t max_items = 20000) const
    {
        return Cache(&frozen_context, {{layer, targets}}, {phase}, max_vertices, max_items);
    }

    bool matches(Cache &cache, const BAC::SerializedXY &output,
                 Validation::WorkBudget *budget = nullptr, const Cache::Cancel &cancel = {}) const
    {
        return cache.matches(&frozen_context, layer, targets, output, lower_z, upper_z, sources, budget, cancel);
    }

    Cache::Proof begin(Cache &cache, const BAC::SerializedXY &output,
                       Validation::WorkBudget *budget = nullptr, const Cache::Cancel &cancel = {}) const
    {
        return cache.begin(&frozen_context, layer, targets, output, lower_z, upper_z, sources, budget, cancel);
    }

    Validation::Result verify(const ExtrusionPath &source, const BAC::SerializedXY &output,
                              Validation::WorkBudget *budget = nullptr) const
    {
        const Point origin = output(Point(0, 0));
        const auto emitted = [&](const ExtrusionPath &path) {
            ExtrusionPath result = path;
            result.polyline = output.emitted_polyline(path.polyline);
            result.polyline.translate(-origin);
            return result;
        };
        ExtrusionPaths deposited;
        for (const ExtrusionPath *path : sources) deposited.push_back(emitted(*path));
        std::vector<Contact::Bead> beads;
        for (const ExtrusionPath &path : deposited) beads.push_back({&path, lower_z});
        const ExtrusionPath upper = emitted(source);
        const auto contact = Contact::footprint(beads, upper, upper_z, 0.);
        REQUIRE(contact.input_valid);
        REQUIRE(contact.completed);
        // These are complete path-only model groups, as used by follow-up
        // bridges. No artificial material boundary supplies a cell or anchor.
        return Validation::validate_contacts({}, contact.physical_footprint, {upper.polyline},
                                               proof_demand(), 5., 0., {contact}, budget);
    }

    void record_passing(Cache::Proof &transaction, const BAC::SerializedXY &output) const
    {
        const auto a = verify(first, output), b = verify(last, output);
        REQUIRE(a.passed());
        REQUIRE(b.passed());
        transaction.record(targets[0], a);
        transaction.record(targets[1], b);
    }

    void publish(Cache &cache, const BAC::SerializedXY &output) const
    {
        auto transaction = begin(cache, output);
        record_passing(transaction, output);
        REQUIRE(transaction.commit());
        REQUIRE(matches(cache, output));
    }
};

void require_real_failure(const Validation::Result &result)
{
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    REQUIRE_FALSE(result.passed());
    REQUIRE_FALSE(result.unsupported_runs.empty());
    CHECK(result.max_unsupported_mm > 0.);
}
} // namespace

TEST_CASE("Bridge aware model proof cache publishes only a complete passing layer",
          "[BridgeAwareCubic][ModelProofCache]")
{
    ProofFixture fixture;
    auto cache = fixture.cache();
    // Combined tangent-section reach falls from .25 to .175 mm. The first
    // group remains supported while the last group really becomes floating.
    fixture.lower.width = .30f;
    const auto first = fixture.verify(fixture.first, fixture.phase);
    const auto last = fixture.verify(fixture.last, fixture.phase);
    REQUIRE(first.passed());
    require_real_failure(last);
    {
        auto transaction = fixture.begin(cache, fixture.phase);
        transaction.record(fixture.targets[0], first);
        CHECK_FALSE(fixture.matches(cache, fixture.phase));
        transaction.record(fixture.targets[1], last);
        CHECK_FALSE(transaction.commit());
    }
    CHECK_FALSE(fixture.matches(cache, fixture.phase));
    fixture.lower.width = .45f;
    fixture.publish(cache, fixture.phase);
}

TEST_CASE("Bridge aware model proof cache requires exact ordered complete group coverage",
          "[BridgeAwareCubic][ModelProofCache]")
{
    const std::string variant = GENERATE("missing", "out of order", "duplicate", "incomplete", "invalid");
    CAPTURE(variant);
    ProofFixture fixture;
    auto cache = fixture.cache();
    const auto first = fixture.verify(fixture.first, fixture.phase);
    const auto last = fixture.verify(fixture.last, fixture.phase);
    REQUIRE(first.passed());
    REQUIRE(last.passed());
    {
        auto transaction = fixture.begin(cache, fixture.phase);
        if (variant == "out of order") {
            transaction.record(fixture.targets[1], last);
            transaction.record(fixture.targets[0], first);
        } else {
            transaction.record(fixture.targets[0], first);
            if (variant == "duplicate") transaction.record(fixture.targets[0], first);
            if (variant == "incomplete") {
                Validation::WorkBudget exhausted;
                exhausted.remaining_work = 0;
                const auto incomplete = fixture.verify(fixture.last, fixture.phase, &exhausted);
                REQUIRE_FALSE(incomplete.completed);
                REQUIRE(incomplete.resource_limited);
                transaction.record(fixture.targets[1], incomplete);
            } else if (variant == "invalid") {
                Polyline fitted = fixture.last.polyline;
                fitted.reset_to_linear_move();
                REQUIRE_FALSE(fitted.fitting_result.empty());
                fitted.fitting_result.front().path_type = EMovePathType::Arc_move_cw;
                const auto invalid = Validation::validate({}, {}, {fitted}, proof_demand(), 5., 0.);
                REQUIRE_FALSE(invalid.input_valid);
                transaction.record(fixture.targets[1], invalid);
            } else if (variant != "missing") transaction.record(fixture.targets[1], last);
        }
        CHECK_FALSE(transaction.commit());
    }
    CHECK_FALSE(fixture.matches(cache, fixture.phase));
    fixture.publish(cache, fixture.phase);
}

TEST_CASE("Bridge aware model proof cache rechecks destructive predecessor mutations",
          "[BridgeAwareCubic][ModelProofCache]")
{
    const std::string mutation = GENERATE("points", "width", "height", "Z", "removed");
    CAPTURE(mutation);
    ProofFixture fixture;
    auto cache = fixture.cache();
    fixture.publish(cache, fixture.phase);
    if (mutation == "points") fixture.lower.polyline.translate(Point::new_scale(0., -.1));
    else if (mutation == "width") fixture.lower.width = .30f;
    else if (mutation == "height") fixture.lower.height = .4f; // Valid w >= h, but less top contact.
    else if (mutation == "Z") fixture.lower_z = .199;
    else fixture.sources.clear();
    REQUIRE_FALSE(fixture.matches(cache, fixture.phase));
    const auto first = fixture.verify(fixture.first, fixture.phase);
    const auto last = fixture.verify(fixture.last, fixture.phase);
    require_real_failure(last);
    auto transaction = fixture.begin(cache, fixture.phase);
    transaction.record(fixture.targets[0], first);
    transaction.record(fixture.targets[1], last);
    CHECK_FALSE(transaction.commit());
    CHECK_FALSE(fixture.matches(cache, fixture.phase));
}

TEST_CASE("Bridge aware model proof cache does not inherit another phase from an old source version",
          "[BridgeAwareCubic][ModelProofCache]")
{
    ProofFixture fixture;
    fixture.lower = proof_bead(.0004, .4498f, .2f, true);
    fixture.first = proof_bead(.1, .4498f);
    fixture.last = proof_bead(.2494, .4498f);
    BAC::SerializedXY a, b;
    a.shift.y() = -.0004;
    b.shift.y() = .0004;
    Cache cache(&fixture.frozen_context, {{fixture.layer, fixture.targets}}, {a, b});
    fixture.publish(cache, a);
    fixture.publish(cache, b);
    REQUIRE(fixture.matches(cache, a));
    REQUIRE(fixture.matches(cache, b));
    const Polyline old_a = a.emitted_polyline(fixture.lower.polyline);
    const Polyline old_b = b.emitted_polyline(fixture.lower.polyline);
    fixture.lower = proof_bead(0., .4498f, .2f, true);
    REQUIRE(old_a.points == a.emitted_polyline(fixture.lower.polyline).points);
    REQUIRE(old_b.points != b.emitted_polyline(fixture.lower.polyline).points);
    REQUIRE_FALSE(fixture.matches(cache, a));
    fixture.publish(cache, a);
    REQUIRE_FALSE(fixture.matches(cache, b));
    // Old B had .249 mm lateral separation; new B has .250 mm, exceeding
    // the two actual flat sections' combined reach of about .2498 mm.
    const auto first = fixture.verify(fixture.first, b), last = fixture.verify(fixture.last, b);
    REQUIRE(first.passed());
    require_real_failure(last);
    auto transaction = fixture.begin(cache, b);
    transaction.record(fixture.targets[0], first);
    transaction.record(fixture.targets[1], last);
    CHECK_FALSE(transaction.commit());
    CHECK(fixture.matches(cache, a));
    CHECK_FALSE(fixture.matches(cache, b));
}

TEST_CASE("Bridge aware model proof cache rejects sources changed during a proof",
          "[BridgeAwareCubic][ModelProofCache]")
{
    ProofFixture fixture;
    auto cache = fixture.cache();
    {
        auto transaction = fixture.begin(cache, fixture.phase);
        fixture.record_passing(transaction, fixture.phase);
        fixture.lower.width = .30f;
        require_real_failure(fixture.verify(fixture.last, fixture.phase));
        CHECK_FALSE(transaction.commit());
    }
    CHECK_FALSE(fixture.matches(cache, fixture.phase));
    fixture.lower.width = .45f;
    fixture.publish(cache, fixture.phase);
}

TEST_CASE("Bridge aware model proof cache rejects a stale overlapping transaction",
          "[BridgeAwareCubic][ModelProofCache]")
{
    ProofFixture fixture;
    auto cache = fixture.cache();
    auto old = fixture.begin(cache, fixture.phase);
    fixture.record_passing(old, fixture.phase);
    fixture.lower.width = .5f;
    fixture.publish(cache, fixture.phase);
    // Restore the exact old source so content comparison alone would pass;
    // the older transaction must still not replace the newer publication.
    fixture.lower.width = .45f;
    REQUIRE(fixture.verify(fixture.last, fixture.phase).passed());
    CHECK_FALSE(old.commit());
    CHECK_FALSE(fixture.matches(cache, fixture.phase));
    fixture.lower.width = .5f;
    CHECK(fixture.matches(cache, fixture.phase));
}

TEST_CASE("Bridge aware model proof cache declines incomplete identity and unfitted input",
          "[BridgeAwareCubic][ModelProofCache]")
{
    const std::string mutation = GENERATE("context", "layer", "targets", "phase", "upper Z", "fitted source",
                                          "oriented source", "sloped source");
    CAPTURE(mutation);
    ProofFixture fixture;
    auto cache = fixture.cache();
    fixture.publish(cache, fixture.phase);
    int other_context = 0;
    const void *context = &fixture.frozen_context;
    size_t layer = fixture.layer;
    auto targets = fixture.targets;
    auto phase = fixture.phase;
    ExtrusionPathOriented oriented(fixture.lower.role(), fixture.lower.mm3_per_mm,
                                   fixture.lower.width, fixture.lower.height);
    static_cast<ExtrusionPath &>(oriented) = fixture.lower;
    ExtrusionPathSloped sloped(fixture.lower, {.5, .5, 0.}, {1., 1., 0.});
    if (mutation == "context") context = &other_context;
    else if (mutation == "layer") ++layer;
    else if (mutation == "targets") targets.pop_back();
    else if (mutation == "phase") phase.plate_origin.y() = .0001;
    else if (mutation == "upper Z") fixture.upper_z += .001;
    else if (mutation == "oriented source") fixture.sources = {&oriented};
    else if (mutation == "sloped source") fixture.sources = {&sloped};
    else fixture.lower.polyline.reset_to_linear_move();
    CHECK_FALSE(cache.matches(context, layer, targets, phase, fixture.lower_z, fixture.upper_z, fixture.sources));
    if (mutation == "upper Z") require_real_failure(fixture.verify(fixture.last, fixture.phase));
    if (mutation == "oriented source" || mutation == "sloped source") {
        // Reusing valid base-path results must not make derived source state
        // cacheable: its extra extrusion behavior is outside this key's scope.
        const auto derived_sources = fixture.sources;
        fixture.sources = {&fixture.lower};
        const auto first = fixture.verify(fixture.first, fixture.phase), last = fixture.verify(fixture.last, fixture.phase);
        REQUIRE(first.passed());
        REQUIRE(last.passed());
        fixture.sources = derived_sources;
        auto transaction = fixture.begin(cache, fixture.phase);
        transaction.record(fixture.targets[0], first);
        transaction.record(fixture.targets[1], last);
        CHECK_FALSE(transaction.commit());
        fixture.sources = {&fixture.lower};
        CHECK(fixture.matches(cache, fixture.phase));
    }
}

TEST_CASE("Bridge aware model proof cache falls back under work and aggregate storage limits",
          "[BridgeAwareCubic][ModelProofCache]")
{
    ProofFixture fixture;
    const auto first = fixture.verify(fixture.first, fixture.phase), last = fixture.verify(fixture.last, fixture.phase);
    REQUIRE(first.passed());
    REQUIRE(last.passed());
    SECTION("insufficient copy capacity cannot publish a passing result") {
        const bool vertices = GENERATE(false, true);
        const size_t vertex_cap = vertices ? 1 : 200000, item_cap = vertices ? 20000 : 6;
        auto cache = fixture.cache(vertex_cap, item_cap);
        auto transaction = fixture.begin(cache, fixture.phase);
        transaction.record(fixture.targets[0], first);
        transaction.record(fixture.targets[1], last);
        CHECK_FALSE(transaction.commit());
        CHECK_FALSE(fixture.matches(cache, fixture.phase));
        CHECK(cache.statistics().vertices <= vertex_cap);
        CHECK(cache.statistics().items <= item_cap);
    }
    SECTION("outstanding transactions share the same retained allocation limit") {
        auto cache = fixture.cache(2, 7); // 4 metadata items plus one 3-item snapshot.
        auto admitted = fixture.begin(cache, fixture.phase);
        {
            auto excess = fixture.begin(cache, fixture.phase);
            excess.record(fixture.targets[0], first);
            excess.record(fixture.targets[1], last);
            CHECK_FALSE(excess.commit());
        }
        admitted.record(fixture.targets[0], first);
        admitted.record(fixture.targets[1], last);
        REQUIRE(admitted.commit());
        CHECK(fixture.matches(cache, fixture.phase));
        CHECK(cache.statistics().vertices <= 2);
        CHECK(cache.statistics().items <= 7);
    }
    SECTION("a zero lookup budget falls back without destroying the valid fact") {
        auto cache = fixture.cache();
        fixture.publish(cache, fixture.phase);
        Validation::WorkBudget budget;
        budget.remaining_work = 0;
        CHECK_FALSE(fixture.matches(cache, fixture.phase, &budget));
        REQUIRE(fixture.verify(fixture.last, fixture.phase).passed());
        CHECK(fixture.matches(cache, fixture.phase));
    }
    SECTION("work exhausted between capture and commit cannot publish") {
        auto cache = fixture.cache();
        Validation::WorkBudget budget;
        const auto before_capture = budget.remaining_work;
        auto transaction = fixture.begin(cache, fixture.phase, &budget);
        REQUIRE(budget.remaining_work < before_capture);
        transaction.record(fixture.targets[0], first);
        transaction.record(fixture.targets[1], last);
        budget.remaining_work = 0;
        CHECK_FALSE(transaction.commit());
        CHECK_FALSE(fixture.matches(cache, fixture.phase));
    }
}

TEST_CASE("Bridge aware model proof cache bounds and charges complete phase admission",
          "[BridgeAwareCubic][ModelProofCache]")
{
    ProofFixture fixture;
    std::vector<BAC::SerializedXY> phases(Cache::max_phases);
    REQUIRE(phases.size() == 64);
    for (size_t i = 0; i < phases.size(); ++i) phases[i].shift.x() = double(i);
    SECTION("the last admitted phase needs its own proof and bounded lookup work") {
        Cache cache(&fixture.frozen_context, {{fixture.layer, fixture.targets}}, phases);
        fixture.publish(cache, phases.front());
        REQUIRE_FALSE(fixture.matches(cache, phases.back()));
        fixture.publish(cache, phases.back());
        REQUIRE(fixture.matches(cache, phases.front()));
        Validation::WorkBudget small;
        // Enough for the two-point bead, but not all components of 64 phases.
        small.remaining_work = phases.size();
        CHECK_FALSE(fixture.matches(cache, phases.back(), &small));
        REQUIRE(fixture.verify(fixture.last, phases.back()).passed());
        CHECK(fixture.matches(cache, phases.back()));
    }
    SECTION("a 65th phase declines caching while the actual proof still passes") {
        phases.emplace_back();
        phases.back().shift.x() = 64.;
        Cache cache(&fixture.frozen_context, {{fixture.layer, fixture.targets}}, phases);
        auto transaction = fixture.begin(cache, phases.back());
        fixture.record_passing(transaction, phases.back());
        CHECK_FALSE(transaction.commit());
        CHECK_FALSE(fixture.matches(cache, phases.back()));
        CHECK(cache.statistics().publications == 0);
    }
}

TEST_CASE("Bridge aware model proof cache honors cancellation without publishing partial work",
          "[BridgeAwareCubic][ModelProofCache]")
{
    struct Cancelled {};
    ProofFixture fixture;
    auto cache = fixture.cache();
    bool cancelled = true;
    const Cache::Cancel cancel = [&] { if (cancelled) throw Cancelled{}; };
    CHECK_THROWS_AS(fixture.matches(cache, fixture.phase, nullptr, cancel), Cancelled);
    CHECK_THROWS_AS(fixture.begin(cache, fixture.phase, nullptr, cancel), Cancelled);
    cancelled = false;
    {
        auto transaction = fixture.begin(cache, fixture.phase, nullptr, cancel);
        fixture.record_passing(transaction, fixture.phase);
        cancelled = true;
        CHECK_THROWS_AS(transaction.commit(), Cancelled);
    }
    cancelled = false;
    CHECK_FALSE(fixture.matches(cache, fixture.phase));
    fixture.publish(cache, fixture.phase);
}
