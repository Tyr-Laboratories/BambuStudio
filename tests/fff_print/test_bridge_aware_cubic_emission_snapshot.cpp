#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicEmissionLedger.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <new>
#include <stdexcept>
#include <type_traits>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
constexpr size_t capacity = 512 * 1024;
constexpr double tolerance = .002 + 16. * SCALING_FACTOR;

BAC::CellId owner(int64_t x)
{
    BAC::CellId result;
    result.depth = 2;
    result.address = {{x, 2, -1}};
    return result;
}

ExtrusionPath bead(std::initializer_list<Vec2d> points)
{
    ExtrusionPath result(erInternalInfill, .08, .4f, .2f);
    result.polyline = Polyline::new_scale(std::vector<Vec2d>(points));
    return result;
}

struct Fixture {
    size_t emitter = 0;
    double z = .4;
    BAC::EmissionReference reference{BoundingBox(Point::new_scale(-20., -20.),
                                                Point::new_scale(20., 20.)), tolerance};
    std::vector<BAC::EmissionPlanBinding> plans{{0, std::make_shared<const int>(1)},
                                               {1, std::make_shared<const int>(2)}};
    std::vector<ExtrusionPath> paths{bead({{-10., 0.}, {10., 0.}}), bead({{0., -2.}, {0., 2.}})};
    std::vector<BAC::EmissionTransition> active;
    BAC::TaggedLineBatch batch;

    Fixture()
    {
        batch.sources = {{BAC::LineSource::Kind::Partition, paths[1].first_point(), paths[1].last_point(),
                          {{owner(1), 0., 1.}}},
                         {BAC::LineSource::Kind::Bulk, paths[0].first_point(), paths[0].last_point(), {}}};
        batch.source_for_path = {1, 0};
        batch.complete = true;
        sync();
    }
    void sync()
    {
        batch.paths.clear();
        for (const auto &path : paths) batch.paths.push_back(path.polyline);
    }
    std::vector<const ExtrusionPath*> children() const
    {
        std::vector<const ExtrusionPath*> result;
        for (const auto &path : paths) result.push_back(&path);
        return result;
    }
    std::vector<BAC::EmissionPathView> live() const
    {
        std::vector<BAC::EmissionPathView> result;
        for (const auto &path : paths) result.push_back({&path, true});
        return result;
    }
    BAC::EmissionLedgerBuilder builder(BAC::EmissionLedgerLimits limits = {capacity, 0}) const
    {
        return BAC::EmissionLedgerBuilder(emitter, z, reference, plans, active, limits);
    }
    BAC::ReferenceLineBatch broad() const
    {
        BAC::ReferenceLineBatch result;
        result.plan_identity = plans[0].identity;
        result.z = z;
        result.bounds = reference.classification_bounds;
        result.sources = {batch.sources[0]};
        result.paths = {paths[1].polyline};
        result.source_for_path = {0};
        result.partitions = {{batch.sources[0].producers[0].owner, result.paths}};
        result.complete = true;
        return result;
    }
};

BAC::EmissionSnapshotPtr capture(const Fixture &fixture,
    BAC::EmissionLedgerLimits limits = {capacity, 0}, Validation::WorkBudget *work = nullptr)
{
    auto builder = fixture.builder(limits);
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children(), work));
    auto snapshot = builder.seal_snapshot(fixture.live(), fixture.plans, work);
    REQUIRE(snapshot);
    return snapshot;
}

BAC::EmissionTransition transition(const Fixture &fixture, const Polyline &before,
                                  const Polyline &after, BAC::CellId producer)
{
    BAC::LayerAttachment value;
    value.z = fixture.z;
    value.emitting_region = fixture.emitter;
    value.original = before;
    value.replacement = after;
    value.width = fixture.paths[1].width;
    value.height = fixture.paths[1].height;
    value.owners = {producer};
    return {0, std::move(value)};
}
}

TEST_CASE("Emission snapshot seals final survival before broad reference classification",
          "[BridgeAwareCubic][EmissionSnapshot]")
{
    Fixture fixture;
    // This valid raw source crosses the reference box. The existing no-reference
    // classifier must decline; raw survival capture must still be available.
    fixture.batch.sources[0].canonical_a = Point::new_scale(0., -30.);
    fixture.batch.sources[0].canonical_b = Point::new_scale(0., 30.);
    auto eager = fixture.builder();
    REQUIRE(eager.bind_batch(fixture.batch, fixture.children()));
    REQUIRE_FALSE(eager.seal(fixture.live(), fixture.plans));

    Validation::WorkBudget work;
    const size_t start = work.remaining_work;
    const auto snapshot = capture(fixture, {capacity, 0}, &work);
    CHECK(work.remaining_work < start);
    REQUIRE(snapshot->records().size() == 2);
    CHECK(snapshot->emitting_region() == fixture.emitter);
    CHECK(snapshot->print_z() == fixture.z);
    CHECK(snapshot->records()[1].path.polyline == fixture.paths[1].polyline);
    CHECK(snapshot->sources()[snapshot->records()[1].source_index].canonical_a == Point::new_scale(0., -30.));
    CHECK(snapshot->matches(fixture.emitter, fixture.z, fixture.reference, fixture.plans, fixture.live(), &work));
    CHECK(snapshot->resident_bytes() == snapshot->owned_bytes());

    auto reference = fixture.broad();
    reference.paths = {Polyline::new_scale({{0., -20.}, {0., 20.}})};
    reference.partitions = {{owner(1), reference.paths}};
    const auto projected = BAC::project_emissions(snapshot, std::move(reference), {capacity, 0}, &work);
    REQUIRE(projected);
    CHECK(projected->snapshot() == snapshot);
    CHECK(projected->records()[1].construction);
    fixture.paths[1].polyline.points.back().x() += 1;
    CHECK_FALSE(snapshot->matches(fixture.emitter, fixture.z, fixture.reference, fixture.plans, fixture.live(), &work));
    CHECK(projected->snapshot()->records()[1].path.polyline.points.back().x() == 0);
}

TEST_CASE("Emission snapshot rejects missing final leaves and owns ordered attachment events",
          "[BridgeAwareCubic][EmissionSnapshot]")
{
    Fixture fixture;
    auto missing = fixture.builder();
    REQUIRE(missing.bind_batch(fixture.batch, fixture.children()));
    auto incomplete = fixture.live();
    incomplete.pop_back();
    CHECK_FALSE(missing.seal_snapshot(incomplete, fixture.plans));
    CHECK_FALSE(missing.seal_snapshot(fixture.live(), fixture.plans));
    auto surviving = fixture.builder();
    REQUIRE(surviving.bind_batch(fixture.batch, fixture.children()));
    REQUIRE(surviving.erased(fixture.paths[0]));
    const auto survivor = surviving.seal_snapshot({{&fixture.paths[1], true}}, fixture.plans);
    REQUIRE(survivor);
    REQUIRE(survivor->records().size() == 1);
    CHECK(survivor->records()[0].path.polyline == fixture.paths[1].polyline);

    const Polyline original = fixture.paths[1].polyline;
    const auto middle = Polyline::new_scale({{0., -3.}, {0., -2.}, {0., 2.}});
    const auto final = Polyline::new_scale({{0., -3.}, {0., -2.}, {0., 2.}, {0., 3.}});
    const auto first = transition(fixture, original, middle, owner(1));
    auto second = transition(fixture, middle, final, owner(9));
    second.storage_region = 1;
    fixture.active = {second, first};
    auto builder = fixture.builder();
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children()));
    fixture.paths[1].polyline = final;
    REQUIRE(builder.applied(fixture.paths[1], {first, second}));
    const auto snapshot = builder.seal_snapshot(fixture.live(), fixture.plans);
    REQUIRE(snapshot);
    REQUIRE(snapshot->records()[1].transitions.size() == 2);
    CHECK(snapshot->records()[1].transitions[0].storage_region == 0);
    CHECK(snapshot->records()[1].transitions[1].storage_region == 1);
    REQUIRE(snapshot->active_attachments().size() == 2);
    CHECK(snapshot->active_attachments()[0].storage_region == 1);

    auto reference = fixture.broad();
    reference.paths = {original};
    reference.partitions = {{owner(1), reference.paths}};
    const auto projection = BAC::project_emissions(snapshot, std::move(reference), {capacity, 0});
    REQUIRE(projection);
    CHECK_FALSE(projection->records()[1].mask_full_path);
    CHECK(projection->records()[1].construction);
    CHECK(projection->records()[1].partition_owners.empty());
    CHECK(projection->records()[1].attachment_indices == std::vector<size_t>{0, 1});
}

TEST_CASE("Emission projection matches eager classification and retains raw paths without copying",
          "[BridgeAwareCubic][EmissionProjection]")
{
    Fixture fixture;
    // A large unchanged ordinary path makes a hidden second raw-path copy
    // exceed the deliberately small derived-only projection allowance.
    fixture.paths[0].polyline.points.clear();
    for (int i = 0; i <= 4000; ++i)
        fixture.paths[0].polyline.points.push_back(Point::new_scale(-10. + .005 * i, 0.));
    fixture.sync();
    auto eager = fixture.builder();
    REQUIRE(eager.bind_reference(fixture.broad()));
    REQUIRE(eager.bind_batch(fixture.batch, fixture.children()));
    const auto legacy = eager.seal(fixture.live(), fixture.plans);
    REQUIRE(legacy);
    const auto snapshot = capture(fixture);
    const size_t projected_limit = snapshot->resident_bytes() + 16 * 1024;
    const auto projection = BAC::project_emissions(snapshot, fixture.broad(), {projected_limit, 0});
    REQUIRE(projection);
    REQUIRE(projection->records().size() == legacy->records().size());
    for (size_t i = 0; i < projection->records().size(); ++i) {
        const auto &actual = projection->records()[i];
        const auto &expected = legacy->records()[i];
        CHECK(actual.construction == expected.construction);
        CHECK(actual.mask_full_path == expected.mask_full_path);
        CHECK(actual.partition_owners == expected.partition_owners);
        CHECK(actual.attachment_indices == expected.attachment_indices);
    }
    CHECK(projection->snapshot() == snapshot);
    CHECK(&projection->snapshot()->records()[0].path == &snapshot->records()[0].path);
    CHECK(projection->owned_bytes() < 16 * 1024);
    CHECK(projection->resident_bytes() == snapshot->resident_bytes() + projection->owned_bytes());
}

TEST_CASE("Emission projection requires exact current reference and complete alternate lineage",
          "[BridgeAwareCubic][EmissionProjection]")
{
    Fixture fixture;
    const auto snapshot = capture(fixture);
    for (int mutation = 0; mutation != 5; ++mutation) {
        auto reference = fixture.broad();
        if (mutation == 0) reference.plan_identity = std::make_shared<const int>(1);
        if (mutation == 1) reference.z += .2;
        if (mutation == 2) reference.bounds.max.x() += 1;
        if (mutation == 3) reference.sources[0].producers[0].last = .9;
        if (mutation == 4) reference.complete = false;
        CHECK_FALSE(BAC::project_emissions(snapshot, std::move(reference), {capacity, 0}));
    }
    REQUIRE(BAC::project_emissions(snapshot, fixture.broad(), {capacity, 0}));

    const Polyline original = fixture.paths[1].polyline;
    const auto alternate = Polyline::new_scale({{0., 2.}, {0., 4.}});
    const auto final = Polyline::new_scale({{0., -2.}, {0., 2.}, {0., 4.}});
    const auto actual = transition(fixture, original, final, owner(1));
    const auto other = transition(fixture, alternate, final, owner(9));
    fixture.active = {actual, other};
    auto builder = fixture.builder();
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children()));
    fixture.paths[1].polyline = final;
    REQUIRE(builder.applied(fixture.paths[1], {actual}));
    const auto convergent = builder.seal_snapshot(fixture.live(), fixture.plans);
    REQUIRE(convergent); // Valid actual survival is separate from classifier availability.
    auto reference = fixture.broad();
    reference.paths = {original};
    reference.partitions = {{owner(1), reference.paths}};
    CHECK_FALSE(BAC::project_emissions(convergent, std::move(reference), {capacity, 0}));
}

TEST_CASE("Emission delta preserves exact duplicate multiplicity and reports order separately",
          "[BridgeAwareCubic][EmissionDelta]")
{
    Fixture before;
    before.paths.push_back(before.paths[0]);
    before.batch.source_for_path.push_back(1);
    before.sync();
    const auto base = capture(before);
    Fixture after = before;
    after.paths = {before.paths[0], before.paths[0], before.paths[1]};
    after.batch.source_for_path = {1, 1, 0};
    after.sync();
    const auto candidate = capture(after);
    const auto delta = BAC::compare_emissions(base, candidate, {capacity, 0});
    REQUIRE(delta);
    CHECK(delta->before() == base);
    CHECK(delta->after() == candidate);
    CHECK(delta->removed().empty());
    CHECK(delta->added().empty());
    REQUIRE(delta->matches().size() == 3);
    CHECK(delta->matches()[0].before == 0);
    CHECK(delta->matches()[0].after == 0);
    CHECK(delta->matches()[1].before == 2);
    CHECK(delta->matches()[1].after == 1);
    CHECK(delta->matches()[2].before == 1);
    CHECK(delta->matches()[2].after == 2);
    CHECK(delta->order_changed());
    CHECK_FALSE(delta->classification_context_changed());
    for (const auto &match : delta->matches()) CHECK_FALSE(match.provenance_changed);

    after.paths.erase(after.paths.begin());
    after.batch.source_for_path.erase(after.batch.source_for_path.begin());
    after.sync();
    const auto one_removed = BAC::compare_emissions(base, capture(after), {capacity, 0});
    REQUIRE(one_removed);
    CHECK(one_removed->removed() == std::vector<size_t>{2});
    CHECK(one_removed->added().empty());
}

TEST_CASE("Emission delta reports geometry flow and elevation changes without inventing a match",
          "[BridgeAwareCubic][EmissionDelta]")
{
    Fixture before;
    const auto base = capture(before);
    for (int mutation = 0; mutation != 5; ++mutation) {
        Fixture after = before;
        if (mutation == 0) after.paths[1].polyline.points.back().x() += 1;
        if (mutation == 1) after.paths[1].width = .41f;
        if (mutation == 2) after.paths[1].height = .21f;
        if (mutation == 3) after.paths[1].mm3_per_mm = .081;
        if (mutation == 4) std::reverse(after.paths[1].polyline.points.begin(), after.paths[1].polyline.points.end());
        after.sync();
        const auto delta = BAC::compare_emissions(base, capture(after), {capacity, 0});
        REQUIRE(delta);
        CHECK(delta->removed() == std::vector<size_t>{1});
        CHECK(delta->added() == std::vector<size_t>{1});
        REQUIRE(delta->matches().size() == 1);
        CHECK(delta->matches()[0].before == 0);
        CHECK(delta->matches()[0].after == 0);
    }
    Fixture higher = before;
    higher.z += .2;
    const auto elevated = BAC::compare_emissions(base, capture(higher), {capacity, 0});
    REQUIRE(elevated);
    CHECK(elevated->matches().empty());
    CHECK(elevated->removed() == std::vector<size_t>{0, 1});
    CHECK(elevated->added() == std::vector<size_t>{0, 1});
    CHECK(elevated->classification_context_changed());

    Fixture appended = before;
    appended.paths.push_back(bead({{-5., 5.}, {5., 5.}}));
    appended.batch.sources.push_back({BAC::LineSource::Kind::Bulk,
        appended.paths.back().first_point(), appended.paths.back().last_point(), {}});
    appended.batch.source_for_path.push_back(appended.batch.sources.size() - 1);
    appended.sync();
    const auto addition = BAC::compare_emissions(base, capture(appended), {capacity, 0});
    REQUIRE(addition);
    CHECK(addition->removed().empty());
    CHECK(addition->added() == std::vector<size_t>{2});
    CHECK(addition->matches().size() == 2);
}

TEST_CASE("Emission delta distinguishes source renumbering from owner and generation changes",
          "[BridgeAwareCubic][EmissionDelta]")
{
    Fixture before;
    const auto base = capture(before);
    Fixture renumbered = before;
    std::swap(renumbered.batch.sources[0], renumbered.batch.sources[1]);
    renumbered.batch.source_for_path = {0, 1};
    const auto same = BAC::compare_emissions(base, capture(renumbered), {capacity, 0});
    REQUIRE(same);
    REQUIRE(same->matches().size() == 2);
    CHECK_FALSE(same->classification_context_changed());
    for (const auto &match : same->matches()) CHECK_FALSE(match.provenance_changed);

    Fixture different_owner = before;
    different_owner.batch.sources[0].producers[0].owner = owner(8);
    const auto ownership = BAC::compare_emissions(base, capture(different_owner), {capacity, 0});
    REQUIRE(ownership);
    CHECK(ownership->removed().empty());
    CHECK(ownership->added().empty());
    REQUIRE(ownership->matches().size() == 2);
    CHECK_FALSE(ownership->matches()[0].provenance_changed);
    CHECK(ownership->matches()[1].provenance_changed);
    CHECK(ownership->classification_context_changed());

    Fixture replacement_plan = before;
    replacement_plan.plans[0].identity = std::make_shared<const int>(1);
    const auto context = BAC::compare_emissions(base, capture(replacement_plan), {capacity, 0});
    REQUIRE(context);
    CHECK(context->removed().empty());
    CHECK(context->added().empty());
    CHECK(context->classification_context_changed());
    for (const auto &match : context->matches()) CHECK_FALSE(match.provenance_changed);

    for (int mutation = 0; mutation != 2; ++mutation) {
        Fixture different_source = before;
        if (mutation == 0) different_source.batch.sources[0].canonical_a.y() -= 1;
        else different_source.batch.sources[0].producers[0].first = .1;
        const auto changed = BAC::compare_emissions(base, capture(different_source), {capacity, 0});
        REQUIRE(changed);
        CHECK(changed->removed().empty());
        CHECK(changed->added().empty());
        CHECK(changed->matches()[1].provenance_changed);
        CHECK(changed->classification_context_changed());
    }

    Fixture new_active = before;
    new_active.active.push_back(transition(new_active,
        Polyline::new_scale({{0., 2.}, {0., 4.}}),
        Polyline::new_scale({{0., -2.}, {0., 2.}, {0., 4.}}), owner(9)));
    const auto changed_active = BAC::compare_emissions(base, capture(new_active), {capacity, 0});
    REQUIRE(changed_active);
    CHECK(changed_active->removed().empty());
    CHECK(changed_active->added().empty());
    CHECK(changed_active->classification_context_changed());
}

TEST_CASE("Emission duplicate matching retains provenance changes after owner order swaps",
          "[BridgeAwareCubic][EmissionDelta]")
{
    Fixture before;
    before.paths[0] = before.paths[1];
    before.batch.sources[1] = before.batch.sources[0];
    before.batch.sources[1].producers[0].owner = owner(2);
    before.batch.source_for_path = {0, 1};
    before.sync();
    const auto base = capture(before);
    Fixture after = before;
    after.batch.source_for_path = {1, 0};
    const auto swapped = BAC::compare_emissions(base, capture(after), {capacity, 0});
    REQUIRE(swapped);
    CHECK(swapped->removed().empty());
    CHECK(swapped->added().empty());
    REQUIRE(swapped->matches().size() == 2);
    CHECK(swapped->matches()[0].before == 0);
    CHECK(swapped->matches()[1].before == 1);
    CHECK(swapped->matches()[0].provenance_changed);
    CHECK(swapped->matches()[1].provenance_changed);
    CHECK_FALSE(swapped->order_changed()); // Physical multiplicity/order is identical.
}

TEST_CASE("Emission snapshot rollback matching uses restored values and complete context",
          "[BridgeAwareCubic][EmissionSnapshot]")
{
    Fixture original;
    const auto base = capture(original);
    Fixture regenerated = original;
    REQUIRE(&regenerated.paths[0] != &original.paths[0]);
    CHECK(base->matches(regenerated.emitter, regenerated.z, regenerated.reference,
                        regenerated.plans, regenerated.live()));
    regenerated.paths[1].mm3_per_mm += .001;
    CHECK_FALSE(base->matches(regenerated.emitter, regenerated.z, regenerated.reference,
                              regenerated.plans, regenerated.live()));
    regenerated = original;
    std::swap(regenerated.paths[0], regenerated.paths[1]);
    CHECK_FALSE(base->matches(regenerated.emitter, regenerated.z, regenerated.reference,
                              regenerated.plans, regenerated.live()));
    regenerated = original;
    regenerated.plans[0].identity = std::make_shared<const int>(1);
    CHECK_FALSE(base->matches(regenerated.emitter, regenerated.z, regenerated.reference,
                              regenerated.plans, regenerated.live()));
    CHECK_FALSE(base->matches(original.emitter, original.z + .2, original.reference,
                              original.plans, original.live()));
    auto nonfree = original.live();
    nonfree[1].free_ended = false;
    CHECK_FALSE(base->matches(original.emitter, original.z, original.reference, original.plans, nonfree));
    auto missing = original.live();
    missing.pop_back();
    CHECK_FALSE(base->matches(original.emitter, original.z, original.reference, original.plans, missing));
    auto extra = original.live();
    extra.push_back(extra.front());
    CHECK_FALSE(base->matches(original.emitter, original.z, original.reference, original.plans, extra));
}

TEST_CASE("Emission snapshot projection and delta charge retained peaks without partial publication",
          "[BridgeAwareCubic][EmissionSnapshot][EmissionProjection][EmissionDelta]")
{
    Fixture fixture;
    const auto base = capture(fixture);
    auto blocked = fixture.builder({base->resident_bytes(), base->resident_bytes()});
    CHECK_FALSE(blocked.bind_batch(fixture.batch, fixture.children()));
    CHECK_FALSE(blocked.seal_snapshot(fixture.live(), fixture.plans));
    const auto candidate = capture(fixture, {capacity, base->resident_bytes()});
    const size_t raw_total = base->resident_bytes() + candidate->resident_bytes();
    CHECK(raw_total <= capacity);

    Validation::WorkBudget work;
    const size_t initial = work.remaining_work;
    CHECK_FALSE(BAC::project_emissions(candidate, fixture.broad(), {raw_total, base->resident_bytes()}, &work));
    CHECK(work.remaining_work < initial);
    const auto projection = BAC::project_emissions(candidate, fixture.broad(), {capacity, base->resident_bytes()}, &work);
    REQUIRE(projection);
    CHECK(base->resident_bytes() + projection->resident_bytes() <= capacity);
    CHECK_FALSE(BAC::compare_emissions(base, candidate, {raw_total, 0}, &work));
    const auto delta = BAC::compare_emissions(base, candidate, {capacity, projection->owned_bytes()}, &work);
    REQUIRE(delta);
    CHECK(delta->resident_bytes() == raw_total + delta->owned_bytes());
    CHECK(delta->resident_bytes() + projection->owned_bytes() <= capacity);

    // Reusing one immutable input twice must count its allocation once.
    const auto identical = BAC::compare_emissions(base, base, {capacity, 0}, &work);
    REQUIRE(identical);
    CHECK(identical->resident_bytes() == base->resident_bytes() + identical->owned_bytes());
    work.remaining_work = 0;
    CHECK_FALSE(BAC::project_emissions(candidate, fixture.broad(), {capacity, 0}, &work));
    CHECK_FALSE(BAC::compare_emissions(base, candidate, {capacity, 0}, &work));
    CHECK(work.remaining_work == 0);
}

TEST_CASE("Emission immutable operations propagate cancellation including allocation-shaped callbacks",
          "[BridgeAwareCubic][EmissionSnapshot][EmissionProjection][EmissionDelta]")
{
    Fixture fixture;
    auto builder = fixture.builder();
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children()));
    CHECK_THROWS_AS(builder.seal_snapshot(fixture.live(), fixture.plans, nullptr,
        [] { throw std::runtime_error("cancel snapshot"); }), std::runtime_error);
    CHECK_FALSE(builder.seal_snapshot(fixture.live(), fixture.plans));
    const auto snapshot = capture(fixture);
    CHECK_THROWS_AS(snapshot->matches(fixture.emitter, fixture.z, fixture.reference,
        fixture.plans, fixture.live(), nullptr, [] { throw std::bad_alloc(); }), std::bad_alloc);
    CHECK_THROWS_AS(BAC::project_emissions(snapshot, fixture.broad(), {capacity, 0}, nullptr,
        [] { throw std::bad_alloc(); }), std::bad_alloc);
    CHECK_THROWS_AS(BAC::compare_emissions(snapshot, snapshot, {capacity, 0}, nullptr,
        [] { throw std::runtime_error("cancel delta"); }), std::runtime_error);
    REQUIRE(BAC::project_emissions(snapshot, fixture.broad(), {capacity, 0}));
    REQUIRE(BAC::compare_emissions(snapshot, snapshot, {capacity, 0}));
}

TEST_CASE("Emission immutable operations decline null or foreign emitters instead of empty complete deltas",
          "[BridgeAwareCubic][EmissionSnapshot][EmissionProjection][EmissionDelta]")
{
    Fixture fixture;
    const auto snapshot = capture(fixture);
    CHECK_FALSE(BAC::project_emissions({}, fixture.broad(), {capacity, 0}));
    CHECK_FALSE(BAC::compare_emissions({}, {}, {capacity, 0}));
    CHECK_FALSE(BAC::compare_emissions(snapshot, {}, {capacity, 0}));
    CHECK_FALSE(BAC::compare_emissions({}, snapshot, {capacity, 0}));
    Fixture foreign = fixture;
    foreign.emitter = 1;
    CHECK_FALSE(BAC::compare_emissions(snapshot, capture(foreign), {capacity, 0}));
    CHECK_FALSE(BAC::project_emissions(snapshot, fixture.broad(), {capacity, 0}, nullptr, {}, capacity));
    REQUIRE(BAC::project_emissions(snapshot, fixture.broad(), {capacity, 0}));
    const auto same = BAC::compare_emissions(snapshot, snapshot, {capacity, 0});
    REQUIRE(same);
    CHECK(same->removed().empty());
    CHECK(same->added().empty());
    CHECK(same->matches().size() == fixture.paths.size());
}

TEST_CASE("Emission reference and foreign Plan context invalidate reuse without moving paths",
          "[BridgeAwareCubic][EmissionSnapshot][EmissionDelta]")
{
    Fixture before;
    const auto snapshot = capture(before);
    for (int mutation = 0; mutation != 3; ++mutation) {
        Fixture after = before;
        if (mutation == 0) after.reference.tolerance_mm += SCALING_FACTOR;
        if (mutation == 1) after.reference.classification_bounds.max.x() += 1;
        if (mutation == 2) after.plans[1].identity = std::make_shared<const int>(2);
        CHECK_FALSE(snapshot->matches(after.emitter, after.z, after.reference, after.plans, after.live()));
        const auto delta = BAC::compare_emissions(snapshot, capture(after), {capacity, 0});
        REQUIRE(delta);
        CHECK(delta->removed().empty());
        CHECK(delta->added().empty());
        CHECK(delta->classification_context_changed());
        CHECK(delta->matches().size() == before.paths.size());
    }
}

TEST_CASE("Emission delta resolves identical fingerprint buckets by complete metadata",
          "[BridgeAwareCubic][EmissionDelta]")
{
    Fixture before;
    before.paths[0] = before.paths[1];
    before.batch.source_for_path = {0, 0};
    // curve_degree is intentionally absent from the lookup fingerprint but
    // present in exact path metadata. These records necessarily share a hash.
    before.paths[0].curve_degree = 3;
    before.paths[1].curve_degree = 7;
    before.sync();
    const auto base = capture(before);
    Fixture after = before;
    std::swap(after.paths[0], after.paths[1]);
    after.sync();
    const auto delta = BAC::compare_emissions(base, capture(after), {capacity, 0});
    REQUIRE(delta);
    CHECK(delta->removed().empty());
    CHECK(delta->added().empty());
    REQUIRE(delta->matches().size() == 2);
    CHECK(delta->matches()[0].before == 1);
    CHECK(delta->matches()[0].after == 0);
    CHECK(delta->matches()[1].before == 0);
    CHECK(delta->matches()[1].after == 1);
    CHECK(delta->order_changed());
    CHECK_FALSE(delta->classification_context_changed());
    for (const auto &match : delta->matches()) CHECK_FALSE(match.provenance_changed);
}

TEST_CASE("Emission projection outlives its reference input and retains exactly its snapshot",
          "[BridgeAwareCubic][EmissionSnapshot][EmissionProjection]")
{
    Fixture fixture;
    auto snapshot = capture(fixture);
    std::weak_ptr<const BAC::EmissionSnapshot> lifetime = snapshot;
    BAC::EmissionProjectionPtr projection;
    {
        auto reference = fixture.broad();
        projection = BAC::project_emissions(snapshot, std::move(reference), {capacity, 0});
        REQUIRE(projection);
        // Destroy every reference buffer before observing the projection. No
        // borrowed axis, source row, owner fragment or input wrapper may remain.
        reference = BAC::ReferenceLineBatch{};
    }
    snapshot.reset();
    CHECK_FALSE(lifetime.expired());
    REQUIRE(projection->records().size() == 2);
    CHECK_FALSE(projection->records()[0].construction);
    CHECK(projection->records()[1].construction);
    CHECK(projection->records()[1].partition_owners == std::vector<BAC::CellId>{owner(1)});
    CHECK(projection->snapshot()->records()[1].path.polyline == fixture.paths[1].polyline);
    CHECK(projection->snapshot()->matches(fixture.emitter, fixture.z, fixture.reference,
                                          fixture.plans, fixture.live()));
    projection.reset();
    CHECK(lifetime.expired());
}
