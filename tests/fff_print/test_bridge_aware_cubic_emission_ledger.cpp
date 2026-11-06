#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicEmissionLedger.hpp"
#include "libslic3r/Fill/BridgeAwareCubicAttachment.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSupport.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
constexpr double tolerance = .002 + 16. * SCALING_FACTOR;
constexpr size_t capacity = 64 * 1024;

BAC::CellId owner(int64_t x)
{
    BAC::CellId id;
    id.address = {{x, 2, -1}};
    id.depth = 2;
    return id;
}

ExtrusionPath bead(std::initializer_list<Vec2d> points)
{
    ExtrusionPath result(erInternalInfill, .08, .4f, .2f);
    result.polyline = Polyline::new_scale(std::vector<Vec2d>(points));
    return result;
}

BAC::LineSource source(const ExtrusionPath &path, BAC::LineSource::Kind kind,
                       std::vector<BAC::ProducerInterval> producers = {})
{
    return {kind, path.first_point(), path.last_point(), std::move(producers)};
}

struct Fixture {
    ExtrusionPath bulk = bead({{-10., 0.}, {10., 0.}});
    ExtrusionPath partition = bead({{0., -2.}, {0., 2.}});
    BAC::EmissionReference reference{BoundingBox(Point::new_scale(-20., -20.),
                                                Point::new_scale(20., 20.)), tolerance};
    // Retained opaque tokens exercise exact owned identity without constructing
    // a second solver. Plan's immutable-token lifecycle has separate tests.
    std::vector<BAC::EmissionPlanBinding> plans{{0, std::make_shared<const int>(1)},
                                               {1, std::make_shared<const int>(2)}};
    std::vector<BAC::EmissionTransition> active;
    BAC::TaggedLineBatch batch;

    Fixture()
    {
        batch.paths = {bulk.polyline, partition.polyline};
        batch.source_for_path = {0, 1};
        batch.sources = {source(bulk, BAC::LineSource::Kind::Bulk),
            source(partition, BAC::LineSource::Kind::Partition, {{owner(1), 0., 1.}})};
        batch.complete = true;
    }
    std::vector<const ExtrusionPath*> children() const { return {&bulk, &partition}; }
    std::vector<BAC::EmissionPathView> live() const { return {{&bulk, true}, {&partition, true}}; }
    BAC::EmissionLedgerBuilder builder(size_t limit = capacity, size_t retained = 0) const
    {
        return BAC::EmissionLedgerBuilder(0, .4, reference, plans, active, {limit, retained});
    }
};

BAC::EmissionLedgerPtr seal(Fixture &fixture)
{
    auto builder = fixture.builder();
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children()));
    auto ledger = builder.seal(fixture.live(), fixture.plans);
    REQUIRE(ledger);
    return ledger;
}

BAC::EmissionTransition transition(size_t storage, const Polyline &before,
                                  const Polyline &after, const ExtrusionPath &path,
                                  BAC::CellId producer)
{
    BAC::LayerAttachment attachment;
    attachment.z = .4;
    attachment.emitting_region = 0;
    attachment.original = before;
    attachment.replacement = after;
    attachment.width = path.width;
    attachment.height = path.height;
    attachment.owners = {producer};
    return {storage, std::move(attachment)};
}

BAC::ReferenceLineBatch clipped_reference(Fixture &fixture)
{
    fixture.partition = bead({{0., -5.}, {0., 5.}});
    fixture.batch.paths[1] = fixture.partition.polyline;
    fixture.batch.sources[1] = {BAC::LineSource::Kind::Partition,
        Point::new_scale(0., -30.), Point::new_scale(0., 30.),
        {{owner(1), .2, .5}, {owner(2), .5, .8}, {owner(3), 0., 1.}}};
    BAC::ReferenceLineBatch result;
    result.plan_identity = fixture.plans[0].identity;
    result.z = .4;
    result.bounds = fixture.reference.classification_bounds;
    result.paths = {Polyline::new_scale({{0., -20.}, {0., 20.}})};
    result.source_for_path = {0};
    result.sources = {fixture.batch.sources[1]};
    result.partitions = {
        {owner(1), {Polyline::new_scale({{0., -18.}, {0., 0.}})}},
        {owner(2), {Polyline::new_scale({{0., 0.}, {0., 18.}})}},
        {owner(3), result.paths}};
    result.complete = true;
    return result;
}
}

TEST_CASE("Emission ledger admits a long ordinary crossing and binds filtered child ordinals",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    // These are the real old classifier predicates, not the new ledger's
    // assertions about its own tags. A crossing has positive added-mask
    // overlap but cannot classify this complete twenty-millimetre bulk path.
    const auto mask = offset(Polylines{fixture.partition.polyline}, scale_(tolerance));
    const auto crossed = intersection_pl(Polylines{fixture.bulk.polyline}, mask);
    REQUIRE_FALSE(crossed.empty());
    REQUIRE_FALSE(BAC::complete_construction_match(fixture.bulk.polyline, crossed, tolerance));
    REQUIRE(BAC::complete_construction_match(fixture.partition.polyline,
        intersection_pl(Polylines{fixture.partition.polyline}, mask), tolerance));

    // FillBase omits invalid polylines, retaining valid output order. Metadata
    // must follow exactly that ordinal rule, rather than a geometric rematch.
    fixture.batch.paths.insert(fixture.batch.paths.begin() + 1, Polyline(Points{Point(5, 5)}));
    fixture.batch.source_for_path.insert(fixture.batch.source_for_path.begin() + 1, 0);
    auto builder = fixture.builder();
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children()));
    auto ledger = builder.seal(fixture.live(), fixture.plans);
    REQUIRE(ledger);
    REQUIRE(ledger->records().size() == 2);
    CHECK_FALSE(ledger->records()[0].construction);
    CHECK_FALSE(ledger->records()[0].mask_full_path);
    CHECK(ledger->records()[0].partition_owners.empty());
    CHECK(ledger->records()[1].construction);
    CHECK(ledger->records()[1].mask_full_path);
    CHECK(ledger->records()[1].partition_owners == std::vector<BAC::CellId>{owner(1)});
    CHECK(ledger->records()[0].path.polyline == fixture.bulk.polyline);
    CHECK(ledger->records()[1].path.polyline == fixture.partition.polyline);
    CHECK(ledger->matches(0, .4, fixture.reference, fixture.plans, fixture.live()));
    CHECK(ledger->resident_bytes() > 0);
    CHECK(ledger->resident_bytes() <= capacity);
}

TEST_CASE("Emission ledger compares owned final values and exact classification context",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    const auto ledger = seal(fixture);
    ExtrusionPath copied_bulk = fixture.bulk, copied_partition = fixture.partition;
    std::vector<BAC::EmissionPathView> copied{{&copied_bulk, true}, {&copied_partition, true}};
    REQUIRE(ledger->matches(0, .4, fixture.reference, fixture.plans, copied));

    SECTION("live coordinates and all consumed flow fields") {
        const auto changes = std::vector<std::function<void(ExtrusionPath&)>>{
            [](auto &path) { ++path.polyline.points[0].x(); },
            [](auto &path) { path.width = std::nextafter(path.width, 1.f); },
            [](auto &path) { path.height = std::nextafter(path.height, 1.f); },
            [](auto &path) { path.mm3_per_mm = std::nextafter(path.mm3_per_mm, 1.); },
            [](auto &path) { path.set_force_no_extrusion(true); },
            [](auto &path) { path.set_extrusion_role(erSolidInfill); },
            [](auto &path) { path.set_reverse(); },
            [](auto &path) { path.polyline.fitting_result.push_back(PathFittingData{}); }};
        for (const auto &change : changes) {
            copied_partition = fixture.partition;
            change(copied_partition);
            CHECK_FALSE(ledger->matches(0, .4, fixture.reference, fixture.plans, copied));
        }
        copied_partition = fixture.partition;
        CHECK(ledger->matches(0, .4, fixture.reference, fixture.plans, copied));
    }
    SECTION("complete traversal and free-path identity") {
        copied.back().free_ended = false;
        CHECK_FALSE(ledger->matches(0, .4, fixture.reference, fixture.plans, copied));
        copied.back().free_ended = true;
        std::reverse(copied.begin(), copied.end());
        CHECK_FALSE(ledger->matches(0, .4, fixture.reference, fixture.plans, copied));
        copied = fixture.live();
        copied.pop_back();
        CHECK_FALSE(ledger->matches(0, .4, fixture.reference, fixture.plans, copied));
        copied = fixture.live();
        copied.push_back({&copied_partition, true});
        CHECK_FALSE(ledger->matches(0, .4, fixture.reference, fixture.plans, copied));
    }
    SECTION("emitter elevation tolerance and clipped-reference bounds") {
        CHECK_FALSE(ledger->matches(1, .4, fixture.reference, fixture.plans, copied));
        CHECK_FALSE(ledger->matches(0, std::nextafter(.4, 1.), fixture.reference, fixture.plans, copied));
        auto changed = fixture.reference;
        ++changed.classification_bounds.max.x();
        CHECK_FALSE(ledger->matches(0, .4, changed, fixture.plans, copied));
        changed = fixture.reference;
        changed.tolerance_mm = std::nextafter(tolerance, 1.);
        CHECK_FALSE(ledger->matches(0, .4, changed, fixture.plans, copied));
    }
    SECTION("retained tokens and storage namespaces") {
        auto changed = fixture.plans;
        // Equal contents in a replacement implementation are not the same
        // immutable Plan token, even when the Plan wrapper address survives.
        changed[0].identity = std::make_shared<const int>(1);
        CHECK_FALSE(ledger->matches(0, .4, fixture.reference, changed, copied));
        changed = fixture.plans;
        changed[0].storage_region = 2;
        CHECK_FALSE(ledger->matches(0, .4, fixture.reference, changed, copied));
        changed = fixture.plans;
        std::reverse(changed.begin(), changed.end());
        CHECK_FALSE(ledger->matches(0, .4, fixture.reference, changed, copied));
    }
}

TEST_CASE("Emission ledger retains finite source ownership without combining alternative producers",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    fixture.partition = bead({{0., -5.}, {0., 5.}});
    fixture.batch.paths[1] = fixture.partition.polyline;
    fixture.batch.sources[1] = source(fixture.partition, BAC::LineSource::Kind::Partition,
        {{owner(1), 0., .5}, {owner(2), .5, 1.}, {owner(3), 0., 1.}});
    const Polyline first_half = Polyline::new_scale({{0., -5.}, {0., 0.}});
    const Polyline last_half = Polyline::new_scale({{0., 0.}, {0., 5.}});
    REQUIRE_FALSE(BAC::complete_construction_match(fixture.partition.polyline,
        BAC::matching_partition_segments({fixture.partition.polyline}, {first_half}), 2. * SCALING_FACTOR));
    REQUIRE_FALSE(BAC::complete_construction_match(fixture.partition.polyline,
        BAC::matching_partition_segments({fixture.partition.polyline}, {last_half}), 2. * SCALING_FACTOR));
    const auto ledger = seal(fixture);
    REQUIRE(ledger->records().size() == 2);
    CHECK(ledger->records()[1].partition_owners == std::vector<BAC::CellId>{owner(3)});
    const size_t index = ledger->records()[1].source_index;
    REQUIRE(index < ledger->sources().size());
    const auto &saved = ledger->sources()[index];
    REQUIRE(saved.producers.size() == 3);
    CHECK(saved.canonical_a == fixture.partition.first_point());
    CHECK(saved.canonical_b == fixture.partition.last_point());
    CHECK(saved.producers[0].first == 0.);
    CHECK(saved.producers[0].last == .5);
    CHECK(saved.producers[1].first == .5);
    CHECK(saved.producers[1].last == 1.);
    // Changing borrowed metadata afterwards cannot change owned provenance.
    fixture.batch.sources[1].producers.clear();
    CHECK(saved.producers.size() == 3);
}

TEST_CASE("Emission ledger seals ordered opposite-terminal transitions in separate owner namespaces",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    const auto original = fixture.partition.polyline;
    const auto middle = Polyline::new_scale({{0., -2.5}, {0., -2.}, {0., 2.}});
    const auto final = Polyline::new_scale({{0., -2.5}, {0., -2.}, {0., 2.}, {0., 2.5}});
    const auto first = transition(0, original, middle, fixture.partition, owner(1));
    const auto second = transition(1, middle, final, fixture.partition, owner(9));
    // Active-table order is deliberately different from actual replay order.
    fixture.active = {second, first};
    auto builder = fixture.builder();
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children()));
    fixture.partition.polyline = final;
    REQUIRE(builder.applied(fixture.partition, {first, second}));
    const auto ledger = builder.seal(fixture.live(), fixture.plans);
    REQUIRE(ledger);
    REQUIRE(ledger->records().size() == 2);
    const auto &record = ledger->records()[1];
    CHECK(record.construction);
    CHECK_FALSE(record.mask_full_path);
    CHECK(record.path.polyline == final);
    CHECK(record.partition_owners.empty()); // Original finite axis does not own either extension.
    REQUIRE(record.transitions.size() == 2);
    CHECK(record.transitions[0].storage_region == 0);
    CHECK(record.transitions[1].storage_region == 1);
    CHECK(record.transitions[0].attachment.original == original);
    CHECK(record.transitions[0].attachment.replacement == middle);
    CHECK(record.transitions[1].attachment.original == middle);
    CHECK(record.transitions[1].attachment.replacement == final);
    CHECK(record.transitions[0].attachment.owners == std::vector<BAC::CellId>{owner(1)});
    CHECK(record.transitions[1].attachment.owners == std::vector<BAC::CellId>{owner(9)});
    CHECK(record.attachment_indices == std::vector<size_t>{0, 1});
    auto changed = fixture.plans;
    changed[0].identity = std::make_shared<const int>(3);
    CHECK_FALSE(ledger->matches(0, .4, fixture.reference, changed, fixture.live()));

    Fixture unreachable;
    unreachable.active = {first, second};
    auto broken = unreachable.builder();
    REQUIRE(broken.bind_batch(unreachable.batch, unreachable.children()));
    unreachable.partition.polyline = final;
    CHECK_FALSE(broken.applied(unreachable.partition, {second}));
    CHECK_FALSE(broken.seal(unreachable.live(), unreachable.plans));
}

TEST_CASE("Emission ledger declines alternate active lineages converging on the same final path",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    const auto original = fixture.partition.polyline;
    const auto alternate = Polyline::new_scale({{0., 2.}, {0., 4.}});
    const auto final = Polyline::new_scale({{0., -2.}, {0., 2.}, {0., 4.}});
    const auto actual = transition(0, original, final, fixture.partition, owner(1));
    const auto other = transition(1, alternate, final, fixture.partition, owner(9));
    fixture.active = {actual, other};
    const std::vector<BAC::LayerAttachment> old_table{actual.attachment, other.attachment};
    const auto from_actual = BridgeAwareCubicAttachment::resolve_chain(original,
        fixture.partition.width, fixture.partition.height, 0, old_table);
    const auto from_alternate = BridgeAwareCubicAttachment::resolve_chain(alternate,
        fixture.partition.width, fixture.partition.height, 0, old_table);
    REQUIRE(from_actual.polyline == final);
    REQUIRE(from_alternate.polyline == final);
    REQUIRE(from_actual.applied_indices == std::vector<size_t>{0});
    REQUIRE(from_alternate.applied_indices == std::vector<size_t>{1});
    auto builder = fixture.builder();
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children()));
    fixture.partition.polyline = final;
    // The actual A->C event cannot erase the legacy collector's applicable
    // B->C owner. First scope falls back, preserving the old full classifier.
    builder.applied(fixture.partition, {actual});
    CHECK_FALSE(builder.seal(fixture.live(), fixture.plans));
}

TEST_CASE("Emission ledger canonicalizes repeated raw source tables across fill batches",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    ExtrusionPath first = bead({{0., -2.}, {0., 0.}});
    ExtrusionPath second = bead({{0., 0.}, {0., 2.}});
    auto first_batch = fixture.batch;
    first_batch.paths[1] = first.polyline;
    auto second_batch = fixture.batch;
    second_batch.paths = {second.polyline};
    second_batch.source_for_path = {1};
    auto builder = fixture.builder();
    REQUIRE(builder.bind_batch(first_batch, {&fixture.bulk, &first}));
    REQUIRE(builder.bind_batch(second_batch, {&second}));
    const auto ledger = builder.seal({{&fixture.bulk, true}, {&first, true}, {&second, true}}, fixture.plans);
    REQUIRE(ledger);
    REQUIRE(ledger->records().size() == 3);
    CHECK_FALSE(ledger->records()[0].construction);
    CHECK(ledger->records()[1].partition_owners == std::vector<BAC::CellId>{owner(1)});
    CHECK(ledger->records()[2].partition_owners == std::vector<BAC::CellId>{owner(1)});
    CHECK(ledger->records()[1].source_index == ledger->records()[2].source_index);
    CHECK(std::count_if(ledger->sources().begin(), ledger->sources().end(), [](const auto &row) {
        return row.kind == BAC::LineSource::Kind::Partition;
    }) == 1);
}

TEST_CASE("Emission ledger seals only final complete survival and declines unsupported emitters",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    auto builder = fixture.builder();
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children()));
    SECTION("explicit actual deletion preserves the remaining exact record") {
        REQUIRE(builder.erased(fixture.partition));
        const auto ledger = builder.seal({{&fixture.bulk, true}}, fixture.plans);
        REQUIRE(ledger);
        REQUIRE(ledger->records().size() == 1);
        CHECK_FALSE(ledger->records()[0].construction);
    }
    SECTION("silent deletion cannot publish partial provenance") {
        CHECK_FALSE(builder.seal({{&fixture.bulk, true}}, fixture.plans));
    }
    SECTION("a surviving untagged leaf declines the entire emitter") {
        const auto unknown = bead({{4., 5.}, {6., 5.}});
        auto view = fixture.live();
        view.push_back({&unknown, true});
        CHECK_FALSE(builder.seal(view, fixture.plans));
    }
    SECTION("a later incomplete batch cannot leave earlier tags reusable") {
        auto incomplete = fixture.batch;
        incomplete.complete = false;
        CHECK_FALSE(builder.bind_batch(incomplete, fixture.children()));
        CHECK_FALSE(builder.seal(fixture.live(), fixture.plans));
    }
    SECTION("solids closures or unsupported tree handling explicitly decline") {
        builder.decline();
        CHECK_FALSE(builder.seal(fixture.live(), fixture.plans));
    }
}

TEST_CASE("Emission ledger intrinsically declines unresolved legacy classification and clipping ambiguity",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    SECTION("a short true bulk source can still be legacy construction") {
        fixture.bulk = bead({{-.0005, 0.}, {.0005, 0.}});
        fixture.batch.paths[0] = fixture.bulk.polyline;
        fixture.batch.sources[0] = source(fixture.bulk, BAC::LineSource::Kind::Bulk);
        const auto mask = offset(Polylines{fixture.partition.polyline}, scale_(tolerance));
        REQUIRE(BAC::complete_construction_match(fixture.bulk.polyline,
            intersection_pl(Polylines{fixture.bulk.polyline}, mask), tolerance));
    }
    SECTION("a near-coincident unreferenced source can supply an extra tolerant owner") {
        auto near = fixture.batch.sources[1];
        ++near.canonical_a.x();
        ++near.canonical_b.x();
        near.producers = {{owner(7), 0., 1.}};
        const Polyline near_line(Points{near.canonical_a, near.canonical_b});
        REQUIRE(BAC::complete_construction_match(fixture.partition.polyline,
            BAC::matching_partition_segments({fixture.partition.polyline}, {near_line}), 2. * SCALING_FACTOR));
        fixture.batch.sources.push_back(std::move(near)); // Deliberately no output references this row.
    }
    SECTION("raw positive source at the old clipping edge has no interior locality proof") {
        fixture.reference.classification_bounds.max.y() = fixture.partition.last_point().y();
    }
    SECTION("an invalid source id cannot silently omit a path") {
        fixture.batch.source_for_path[1] = fixture.batch.sources.size();
    }
    SECTION("nonfinite or out-of-range finite ownership is unavailable") {
        fixture.batch.sources[1].producers[0].last = std::numeric_limits<double>::infinity();
    }
    SECTION("derived path metadata is outside the first eligible scope") {
        ExtrusionPathOriented derived(erInternalInfill, .08, .4f, .2f);
        derived.polyline = fixture.partition.polyline;
        auto builder = fixture.builder();
        CHECK_FALSE(builder.bind_batch(fixture.batch, {&fixture.bulk, &derived}));
        CHECK_FALSE(builder.seal({{&fixture.bulk, true}, {&derived, true}}, fixture.plans));
        return;
    }
    auto builder = fixture.builder();
    builder.bind_batch(fixture.batch, fixture.children()); // Either bind or seal may decline.
    CHECK_FALSE(builder.seal(fixture.live(), fixture.plans));
}

TEST_CASE("Emission ledger bounds retained plus staged state and never replenishes work",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    Validation::WorkBudget work;
    const size_t before = work.remaining_work;
    auto builder = fixture.builder();
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children(), &work));
    CHECK(builder.staged_bytes() > 0);
    CHECK(builder.staged_bytes() <= capacity);
    const auto ledger = builder.seal(fixture.live(), fixture.plans, &work);
    REQUIRE(ledger);
    CHECK(work.remaining_work < before);
    CHECK(ledger->resident_bytes() <= capacity);
    const size_t after = work.remaining_work;
    REQUIRE(ledger->matches(0, .4, fixture.reference, fixture.plans, fixture.live(), &work));
    CHECK(work.remaining_work < after);

    auto no_room = fixture.builder(ledger->resident_bytes(), ledger->resident_bytes());
    CHECK_FALSE(no_room.bind_batch(fixture.batch, fixture.children(), &work));
    CHECK_FALSE(no_room.seal(fixture.live(), fixture.plans, &work));
    CHECK(work.remaining_work <= after);
    auto tiny = fixture.builder(1);
    CHECK_FALSE(tiny.bind_batch(fixture.batch, fixture.children()));
    CHECK_FALSE(tiny.seal(fixture.live(), fixture.plans));
    auto scratch = fixture.builder();
    REQUIRE(scratch.available_scratch_bytes() > 0);
    const size_t free = scratch.available_scratch_bytes();
    CHECK_FALSE(scratch.bind_batch(fixture.batch, fixture.children(), nullptr, {}, free));
    CHECK(scratch.available_scratch_bytes() == 0);
    CHECK(std::string(scratch.decline_reason()) == "work_or_capacity");
    scratch.decline();
    CHECK_FALSE(scratch.seal(fixture.live(), fixture.plans));
    CHECK(std::string(scratch.decline_reason()) == "work_or_capacity");

    work.remaining_work = 0;
    CHECK_FALSE(ledger->matches(0, .4, fixture.reference, fixture.plans, fixture.live(), &work));
    auto exhausted = fixture.builder();
    CHECK_FALSE(exhausted.bind_batch(fixture.batch, fixture.children(), &work));
    CHECK_FALSE(exhausted.seal(fixture.live(), fixture.plans, &work));
    CHECK(work.remaining_work == 0);
}

TEST_CASE("Emission ledger preserves finite owner classification under rotation translation and source reversal",
          "[BridgeAwareCubic][EmissionLedger]")
{
    for (double angle : {0., .29670597283903605, .7504915783575616}) {
        for (const Vec2d shift : {Vec2d(0., 0.), Vec2d(.12345, -.24681)}) {
            for (bool reverse : {false, true}) {
                CAPTURE(angle, shift.x(), shift.y(), reverse);
                Fixture fixture;
                const auto transformed = [&](double distance) {
                    return Point::new_scale(shift.x() + std::cos(angle) * distance,
                                            shift.y() + std::sin(angle) * distance);
                };
                Point first = transformed(-5.12347), last = transformed(5.87653);
                if (reverse) std::swap(first, last);
                const auto at = [&](double t) {
                    const Vec2d p = first.cast<double>() + (last - first).cast<double>() * t;
                    return Point(coord_t(std::llround(p.x())), coord_t(std::llround(p.y())));
                };
                fixture.partition.polyline = Polyline(Points{at(.35), at(.65)});
                fixture.batch.paths = {fixture.partition.polyline};
                fixture.batch.source_for_path = {0};
                fixture.batch.sources = {{BAC::LineSource::Kind::Partition, first, last,
                    {{owner(1), .137, .863}, {owner(2), 0., 1.}}}};
                const Polyline axis(Points{first, last}), finite(Points{at(.137), at(.863)});
                const auto mask = offset(Polylines{axis}, scale_(tolerance));
                REQUIRE(BAC::complete_construction_match(fixture.partition.polyline,
                    intersection_pl(Polylines{fixture.partition.polyline}, mask), tolerance));
                std::vector<BAC::CellId> expected;
                for (const auto &owned : std::vector<std::pair<BAC::CellId, Polyline>>{
                         {owner(1), finite}, {owner(2), axis}}) {
                    if (BAC::complete_construction_match(fixture.partition.polyline,
                        BAC::matching_partition_segments({fixture.partition.polyline}, {owned.second}),
                        2. * SCALING_FACTOR)) expected.push_back(owned.first);
                }
                REQUIRE(expected == std::vector<BAC::CellId>{owner(1), owner(2)});
                auto builder = fixture.builder();
                REQUIRE(builder.bind_batch(fixture.batch, {&fixture.partition}));
                const auto ledger = builder.seal({{&fixture.partition, true}}, fixture.plans);
                REQUIRE(ledger);
                REQUIRE(ledger->records().size() == 1);
                CHECK(ledger->records()[0].mask_full_path);
                CHECK(ledger->records()[0].partition_owners == expected);
            }
        }
    }
}

TEST_CASE("Emission ledger declines transverse paths on a source butt boundary",
          "[BridgeAwareCubic][EmissionLedger]")
{
    for (bool reverse : {false, true}) {
        CAPTURE(reverse);
        Fixture fixture;
        auto axis = fixture.partition.polyline;
        if (reverse) axis.reverse();
        fixture.partition = bead({{-.0005, 2.}, {.0005, 2.}});
        fixture.batch.paths = {fixture.partition.polyline};
        fixture.batch.source_for_path = {0};
        // Keep finite ownership far from the tested endpoint. This isolates
        // the mask classifier instead of declining on a nearby owner edge.
        fixture.batch.sources = {{BAC::LineSource::Kind::Partition,
            axis.first_point(), axis.last_point(),
            {{owner(1), reverse ? .75 : 0., reverse ? 1. : .25}}}};
        const auto mask = offset(Polylines{axis}, scale_(tolerance));
        auto inside = fixture.partition.polyline, outside = fixture.partition.polyline;
        for (auto &p : inside.points) --p.y();
        for (auto &p : outside.points) ++p.y();
        REQUIRE(BAC::complete_construction_match(inside,
            intersection_pl(Polylines{inside}, mask), tolerance));
        REQUIRE_FALSE(BAC::complete_construction_match(outside,
            intersection_pl(Polylines{outside}, mask), tolerance));
        auto builder = fixture.builder();
        REQUIRE(builder.bind_batch(fixture.batch, {&fixture.partition}));
        CHECK_FALSE(builder.seal({{&fixture.partition, true}}, fixture.plans));
    }
    Fixture clipped;
    clipped.reference.classification_bounds.max.y() = clipped.partition.last_point().y();
    auto builder = clipped.builder();
    REQUIRE(builder.bind_batch(clipped.batch, clipped.children()));
    CHECK_FALSE(builder.seal(clipped.live(), clipped.plans));
    CHECK(std::string(builder.decline_reason()) == "reference_clip");
    builder.decline();
    CHECK_FALSE(builder.bind_batch(clipped.batch, clipped.children()));
    CHECK(std::string(builder.decline_reason()) == "reference_clip");
}

TEST_CASE("Emission ledger cancellation propagates on optional capture and warm lookup",
          "[BridgeAwareCubic][EmissionLedger]")
{
    struct Stop : std::runtime_error { Stop() : std::runtime_error("stop ledger") {} };
    Fixture fixture;
    const auto cancel = [] { throw Stop(); };
    auto builder = fixture.builder();
    CHECK_THROWS_AS(builder.bind_batch(fixture.batch, fixture.children(), nullptr, cancel), Stop);
    auto successful = fixture.builder();
    REQUIRE(successful.bind_batch(fixture.batch, fixture.children()));
    CHECK_THROWS_AS(successful.seal(fixture.live(), fixture.plans, nullptr, cancel), Stop);
    const auto ledger = seal(fixture);
    CHECK_THROWS_AS(ledger->matches(0, .4, fixture.reference, fixture.plans,
                                   fixture.live(), nullptr, cancel), Stop);
    // Allocation-shaped cancellation must not be swallowed as optional cache
    // allocation failure. The cancellation callback is external authority.
    CHECK_THROWS_AS(ledger->matches(0, .4, fixture.reference, fixture.plans,
        fixture.live(), nullptr, [] { throw std::bad_alloc(); }), std::bad_alloc);
}

TEST_CASE("Emission ledger uses exact broad clipping and independent finite owners",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    auto reference = clipped_reference(fixture);
    const auto added = reference.paths;
    const auto partitions = reference.partitions;
    auto builder = fixture.builder();
    REQUIRE(builder.needs_reference());
    CHECK(builder.reference_bounds().min == reference.bounds.min);
    CHECK(builder.reference_bounds().max == reference.bounds.max);
    REQUIRE(builder.bind_reference(std::move(reference)));
    CHECK_FALSE(builder.needs_reference());
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children()));
    const auto ledger = builder.seal(fixture.live(), fixture.plans);
    REQUIRE(ledger);
    const auto mask = offset(added, scale_(tolerance));
    for (size_t i = 0; i < fixture.live().size(); ++i) {
        const auto &path = *fixture.live()[i].path;
        const bool full = BAC::complete_construction_match(path.polyline,
            intersection_pl(Polylines{path.polyline}, mask), tolerance);
        CHECK(ledger->records()[i].mask_full_path == full);
        CHECK(ledger->records()[i].construction == full);
        std::vector<BAC::CellId> expected;
        if (full) for (const auto &group : partitions)
            if (BAC::complete_construction_match(path.polyline,
                BAC::matching_partition_segments({path.polyline}, group.second), 2. * SCALING_FACTOR))
                expected.push_back(group.first);
        CHECK(ledger->records()[i].partition_owners == expected);
    }
    REQUIRE(ledger->records()[1].partition_owners == std::vector<BAC::CellId>{owner(3)});
    CHECK(ledger->matches(0, .4, fixture.reference, fixture.plans, fixture.live()));
    // The exact reference permits this boundary-clipped source; raw tags
    // alone still cannot authorize the previous interior-only certificate.
    auto without = fixture.builder();
    REQUIRE(without.bind_batch(fixture.batch, fixture.children()));
    CHECK_FALSE(without.seal(fixture.live(), fixture.plans));
    CHECK(std::string(without.decline_reason()) == "reference_clip");
}

TEST_CASE("Emission ledger rejects mismatched or partial broad reference state",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    auto reference = clipped_reference(fixture);
    SECTION("replacement identity") { reference.plan_identity = std::make_shared<const int>(1); }
    SECTION("elevation") { reference.z = std::nextafter(reference.z, 1.); }
    SECTION("clipping bounds") { ++reference.bounds.max.x(); }
    SECTION("incomplete") { reference.complete = false; }
    SECTION("missing source") { reference.source_for_path[0] = reference.sources.size(); }
    SECTION("joined reference has no isolated butt offset proof") {
        reference.paths[0].points.insert(reference.paths[0].points.begin() + 1, Point(1, 1));
    }
    SECTION("group order is significant") { std::reverse(reference.partitions.begin(), reference.partitions.end()); }
    SECTION("different later canonical table") {
        auto builder = fixture.builder();
        REQUIRE(builder.bind_reference(std::move(reference)));
        ++fixture.batch.sources[1].canonical_a.x();
        CHECK_FALSE(builder.bind_batch(fixture.batch, fixture.children()));
        CHECK_FALSE(builder.seal(fixture.live(), fixture.plans));
        return;
    }
    SECTION("different later finite owner") {
        auto builder = fixture.builder();
        REQUIRE(builder.bind_reference(std::move(reference)));
        fixture.batch.sources[1].producers[0].last = .49;
        CHECK_FALSE(builder.bind_batch(fixture.batch, fixture.children()));
        CHECK_FALSE(builder.seal(fixture.live(), fixture.plans));
        return;
    }
    auto builder = fixture.builder();
    CHECK_FALSE(builder.bind_reference(std::move(reference)));
    CHECK_FALSE(builder.needs_reference());
    CHECK_FALSE(builder.bind_batch(fixture.batch, fixture.children()));
    CHECK_FALSE(builder.seal(fixture.live(), fixture.plans));
}

TEST_CASE("Emission ledger preserves the exact tiny-owner shortcut before exclusion bounds",
          "[BridgeAwareCubic][EmissionLedger]")
{
    for (bool partial : {false, true}) {
        CAPTURE(partial);
        Fixture fixture;
        fixture.partition.polyline = Polyline(Points{Point(0, 10), Point(0, 1010)});
        if (partial) fixture.partition.polyline.points.push_back(Point(1, 1010));
        const Polyline axis(Points{Point(0, -2000), Point(0, 2000)});
        const Polyline tiny(Points{Point(0, 0), Point(1, 0)});
        fixture.batch.paths = {fixture.partition.polyline};
        fixture.batch.source_for_path = {0};
        fixture.batch.sources = {
            {BAC::LineSource::Kind::Partition, axis.front(), axis.back(), {{owner(1), 0., 1.}}},
            {BAC::LineSource::Kind::Partition, tiny.front(), tiny.back(), {{owner(9), 0., 1.}}}};
        BAC::ReferenceLineBatch reference;
        reference.plan_identity = fixture.plans[0].identity;
        reference.z = .4;
        reference.bounds = fixture.reference.classification_bounds;
        reference.paths = {axis, tiny};
        reference.source_for_path = {0, 1};
        reference.sources = fixture.batch.sources;
        reference.partitions = {{owner(1), {axis}}, {owner(9), {tiny}}};
        reference.complete = true;
        REQUIRE(BAC::complete_construction_match(fixture.partition.polyline,
            BAC::matching_partition_segments({fixture.partition.polyline}, {tiny}), 2. * SCALING_FACTOR));
        auto builder = fixture.builder();
        REQUIRE(builder.bind_reference(std::move(reference)));
        REQUIRE(builder.bind_batch(fixture.batch, {&fixture.partition}));
        const auto ledger = builder.seal({{&fixture.partition, true}}, fixture.plans);
        if (partial) {
            // The first long segment is wholly owned through the nonlocal
            // shortcut, and the remaining one-grid segment is within legacy
            // total tolerance. Rectangle-only exclusion would lose owner(9).
            CHECK_FALSE(ledger);
        } else {
            REQUIRE(ledger);
            CHECK(ledger->records()[0].partition_owners == std::vector<BAC::CellId>{owner(1), owner(9)});
        }
    }
}

TEST_CASE("Emission ledger reference capture consumes quota and cancellation without partial publication",
          "[BridgeAwareCubic][EmissionLedger]")
{
    Fixture fixture;
    auto reference = clipped_reference(fixture);
    auto builder = fixture.builder();
    Validation::WorkBudget work;
    const size_t before = work.remaining_work;
    REQUIRE(builder.bind_reference(std::move(reference), &work));
    REQUIRE(work.remaining_work < before);
    REQUIRE(builder.bind_batch(fixture.batch, fixture.children(), &work));
    auto incomplete = fixture.batch;
    incomplete.complete = false;
    CHECK_FALSE(builder.bind_batch(incomplete, fixture.children(), &work));
    CHECK_FALSE(builder.seal(fixture.live(), fixture.plans, &work));
    CHECK(builder.staged_bytes() == 0);
    CHECK(work.remaining_work < before);
    reference = clipped_reference(fixture);
    auto capped = fixture.builder();
    CHECK_FALSE(capped.bind_reference(std::move(reference), nullptr, {}, capped.available_scratch_bytes()));
    CHECK(capped.staged_bytes() == 0);
    reference = clipped_reference(fixture);
    auto exhausted = fixture.builder();
    work.remaining_work = 0;
    CHECK_FALSE(exhausted.bind_reference(std::move(reference), &work));
    CHECK(work.remaining_work == 0);
    reference = clipped_reference(fixture);
    auto canceled = fixture.builder();
    CHECK_THROWS_AS(canceled.bind_reference(std::move(reference), nullptr,
        [] { throw std::bad_alloc(); }), std::bad_alloc);
    CHECK(canceled.staged_bytes() == 0);
}

TEST_CASE("Emission ledger accepts exact production endpoints without admitting points beyond a cap",
          "[BridgeAwareCubic][EmissionLedger][EmissionEndpoint]")
{
    struct Witness { Point raw_a, raw_b, ref_a, ref_b, path_a, path_b; };
    const std::vector<Witness> witnesses{
        {Point(-400149, 1274812), Point(983945, 903945),
         Point(-400149, 1274812), Point(983945, 903945),
         Point(-400149, 1274812), Point(983945, 903945)},
        {Point(-2189475, -434514), Point(-710740, -830740),
         Point(-1599999, -592464), Point(-710740, -830740),
         Point(-1531794, -610739), Point(-710740, -830740)}};
    for (size_t index = 0; index < witnesses.size(); ++index) {
        for (bool reverse : {false, true}) {
            for (bool beyond_cap : {false, true}) {
                CAPTURE(index, reverse, beyond_cap);
                auto witness = witnesses[index];
                if (reverse) {
                    std::swap(witness.raw_a, witness.raw_b);
                    std::swap(witness.ref_a, witness.ref_b);
                    std::swap(witness.path_a, witness.path_b);
                }
                Fixture fixture;
                fixture.reference.classification_bounds = BoundingBox(Point(-1600000, -1400000), Point(1600000, 1400000));
                fixture.partition.polyline = Polyline(Points{witness.path_a, witness.path_b});
                if (beyond_cap) {
                    const Point direction = witness.ref_b - witness.ref_a;
                    fixture.partition.polyline.points.back() = witness.ref_b +
                        Point(direction.x() > 0 ? 1 : -1, direction.y() > 0 ? 1 : -1);
                    const Point displacement = fixture.partition.last_point() - witness.ref_a;
                    const int64_t dot = int64_t(displacement.x()) * direction.x() + int64_t(displacement.y()) * direction.y();
                    const int64_t squared = int64_t(direction.x()) * direction.x() + int64_t(direction.y()) * direction.y();
                    REQUIRE(dot > squared);
                }
                fixture.batch.paths = {fixture.partition.polyline};
                fixture.batch.source_for_path = {0};
                fixture.batch.sources = {{BAC::LineSource::Kind::Partition, witness.raw_a, witness.raw_b,
                                          {{owner(1), 0., 1.}}}};
                BAC::ReferenceLineBatch reference;
                reference.plan_identity = fixture.plans[0].identity;
                reference.z = .4;
                reference.bounds = fixture.reference.classification_bounds;
                reference.paths = {Polyline(Points{witness.ref_a, witness.ref_b})};
                reference.source_for_path = {0};
                reference.sources = fixture.batch.sources;
                reference.partitions = {{owner(1), reference.paths}};
                reference.complete = true;
                if (!beyond_cap) {
                    const auto mask = offset(reference.paths, scale_(tolerance));
                    REQUIRE(BAC::complete_construction_match(fixture.partition.polyline,
                        intersection_pl(Polylines{fixture.partition.polyline}, mask), tolerance));
                    REQUIRE(BAC::complete_construction_match(fixture.partition.polyline,
                        BAC::matching_partition_segments({fixture.partition.polyline}, reference.paths), 2. * SCALING_FACTOR));
                }
                auto builder = fixture.builder();
                REQUIRE(builder.bind_reference(std::move(reference)));
                REQUIRE(builder.bind_batch(fixture.batch, {&fixture.partition}));
                const auto ledger = builder.seal({{&fixture.partition, true}}, fixture.plans);
                if (beyond_cap) {
                    CHECK_FALSE(ledger);
                } else {
                    CHECK(ledger);
                    if (ledger) {
                        CHECK(ledger->records()[0].mask_full_path);
                        CHECK(ledger->records()[0].partition_owners == std::vector<BAC::CellId>{owner(1)});
                    }
                }
            }
        }
    }
}
