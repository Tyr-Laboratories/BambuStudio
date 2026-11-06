#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubic.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <new>
#include <stdexcept>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;

namespace {
ExPolygon rectangle(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon({Point::new_scale(x0, y0), Point::new_scale(x1, y0),
                              Point::new_scale(x1, y1), Point::new_scale(x0, y1)}));
}

BAC::Parameters parameters()
{
    BAC::Parameters result;
    result.bounds = BoundingBoxf3(Vec3d(-24., -14., 0.), Vec3d(24., 14., 20.));
    result.flow_spacing = double(float(.4)) - double(float(.2)) * (1. - M_PI / 4.);
    result.density = .12;
    result.angle = 17. * M_PI / 180.;
    return result;
}

template<class Batch> std::vector<const BAC::LineSource *> partition_sources(const Batch &batch)
{
    std::vector<const BAC::LineSource *> result;
    for (const auto &source : batch.sources)
        if (source.kind == BAC::LineSource::Kind::Partition) result.push_back(&source);
    return result;
}

template<class First, class Second> void same_sources(const First &first, const Second &second)
{
    const auto a = partition_sources(first), b = partition_sources(second);
    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i]->canonical_a == b[i]->canonical_a);
        CHECK(a[i]->canonical_b == b[i]->canonical_b);
        REQUIRE(a[i]->producers.size() == b[i]->producers.size());
        for (size_t j = 0; j < a[i]->producers.size(); ++j) {
            CHECK(a[i]->producers[j].owner == b[i]->producers[j].owner);
            CHECK(a[i]->producers[j].first == b[i]->producers[j].first);
            CHECK(a[i]->producers[j].last == b[i]->producers[j].last);
        }
    }
}

void complete_batch(const BAC::TaggedLineBatch &batch, const Polylines &ordinary)
{
    // Geometry equivalence is point-for-point and order-sensitive, not a
    // buffered geometric approximation or total-length comparison.
    CHECK(batch.paths == ordinary);
    REQUIRE(batch.complete);
    REQUIRE(batch.source_for_path.size() == batch.paths.size());
    for (size_t source : batch.source_for_path) REQUIRE(source < batch.sources.size());
}

void finite_sources(const BAC::Plan &plan, const BAC::TaggedLineBatch &batch, double z)
{
    const auto &p = plan.parameters();
    const double angle = std::remainder(p.angle, 2. * M_PI) + .5 * M_PI;
    Matrix3d rotation;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const double azimuth = angle + axis * 2. * M_PI / 3.;
        rotation.col(axis) = Vec3d(std::sqrt(2. / 3.) * std::cos(azimuth),
                                   std::sqrt(2. / 3.) * std::sin(azimuth), 1. / std::sqrt(3.));
    }
    const Vec3d origin(.5 * (p.bounds.min.x() + p.bounds.max.x()),
                       .5 * (p.bounds.min.y() + p.bounds.max.y()), 0.);
    const auto local = [&](const Vec2d &point) -> Vec3d {
        return rotation.transpose() * (Vec3d(point.x() * SCALING_FACTOR,
                                             point.y() * SCALING_FACTOR, z) - origin);
    };
    // Only the known endpoint-to-integer conversion is allowed here. This
    // checks source metadata; it is not a physical contact/gap allowance.
    const double rounding = 2. * SCALING_FACTOR;
    for (const auto &source : batch.sources) {
        REQUIRE(source.canonical_a != source.canonical_b);
        if (source.kind == BAC::LineSource::Kind::Bulk) {
            CHECK(source.producers.empty());
            continue;
        }
        REQUIRE_FALSE(source.producers.empty());
        const Vec2d a = source.canonical_a.cast<double>(), b = source.canonical_b.cast<double>();
        for (const auto &producer : source.producers) {
            REQUIRE(std::isfinite(producer.first));
            REQUIRE(std::isfinite(producer.last));
            REQUIRE(producer.first >= 0.);
            REQUIRE(producer.last <= 1.);
            REQUIRE(producer.first < producer.last);
            REQUIRE(std::any_of(plan.refinements().begin(), plan.refinements().end(), [&](const auto &r) {
                return !r.solid && r.id == producer.owner;
            }));
            const double edge = std::ldexp(plan.bulk_cell_edge(), -int(producer.owner.depth));
            const Vec3d low(double(producer.owner.address[0]) * edge,
                             double(producer.owner.address[1]) * edge,
                             double(producer.owner.address[2]) * edge);
            const Vec3d source_a = local(a), source_b = local(b);
            bool on_midplane = false;
            for (unsigned axis = 0; axis < 3; ++axis)
                on_midplane = on_midplane ||
                    (std::abs(source_a[axis] - low[axis] - .5 * edge) <= rounding &&
                     std::abs(source_b[axis] - low[axis] - .5 * edge) <= rounding);
            CHECK(on_midplane);
            for (double parameter : {producer.first, producer.last}) {
                const Vec3d endpoint = local(a + (b - a) * parameter);
                for (unsigned axis = 0; axis < 3; ++axis) {
                    CHECK(endpoint[axis] >= low[axis] - rounding);
                    CHECK(endpoint[axis] <= low[axis] + edge + rounding);
                }
            }
        }
    }
}

std::vector<std::pair<BAC::CellId, Polylines>> owned_fragments(const BAC::TaggedLineBatch &batch)
{
    // Evaluate ranges on the original axis against the actual clipped output,
    // as the established partition_lines API does. Keeping this operation in
    // the source frame is essential before later pointwise XY serialization.
    std::map<BAC::CellId, Polylines> by_owner;
    for (size_t index = 0; index < batch.paths.size(); ++index) {
        const auto &source = batch.sources.at(batch.source_for_path.at(index));
        if (source.kind == BAC::LineSource::Kind::Bulk) continue;
        const auto &path = batch.paths[index];
        const Vec2d direction = (source.canonical_b - source.canonical_a).cast<double>();
        const double squared = direction.squaredNorm();
        REQUIRE(squared > 0.);
        for (size_t i = 1; i < path.size(); ++i) {
            const Point &start = path.points[i - 1], &finish = path.points[i];
            const double from = direction.dot((start - source.canonical_a).cast<double>()) / squared;
            const double to = direction.dot((finish - source.canonical_a).cast<double>()) / squared;
            if (from == to) continue;
            for (const auto &producer : source.producers) {
                const double low = std::max(producer.first, std::min(from, to));
                const double high = std::min(producer.last, std::max(from, to));
                if (high <= low) continue;
                const auto point_at = [&](double position) {
                    if (position == from) return start;
                    if (position == to) return finish;
                    const Vec2d point = start.cast<double>() +
                        (finish - start).cast<double>() * ((position - from) / (to - from));
                    return Point(coord_t(std::llround(point.x())), coord_t(std::llround(point.y())));
                };
                const Point a = point_at(low), b = point_at(high);
                if (a != b) by_owner[producer.owner].emplace_back(a, b);
            }
        }
    }
    return {by_owner.begin(), by_owner.end()};
}

ExPolygon reference_domain(const BoundingBox &bounds)
{
    return ExPolygon(Polygon{Points{bounds.min, Point(bounds.max.x(), bounds.min.y()), bounds.max,
                                    Point(bounds.min.x(), bounds.max.y())}});
}

void exact_reference(const BAC::Plan &plan, double z, const BoundingBox &bounds,
                     const BAC::ReferenceLineBatch &reference)
{
    REQUIRE(reference.complete);
    CHECK(reference.plan_identity == plan.immutable_identity());
    CHECK(reference.z == z);
    CHECK(reference.bounds.defined == bounds.defined);
    CHECK(reference.bounds.min == bounds.min);
    CHECK(reference.bounds.max == bounds.max);
    const ExPolygon domain = reference_domain(bounds);
    CHECK(reference.paths == plan.refinement_lines(z, domain));
    CHECK(reference.partitions == plan.partition_lines(z, domain));
    REQUIRE(reference.source_for_path.size() == reference.paths.size());
    for (size_t index : reference.source_for_path) {
        REQUIRE(index < reference.sources.size());
        CHECK(reference.sources[index].kind == BAC::LineSource::Kind::Partition);
    }
    const auto tagged = plan.tagged_lines(z, domain);
    REQUIRE(tagged.complete);
    same_sources(reference, tagged);
}
}

TEST_CASE("Bridge aware reference batches preserve exact broad clipping and owner order", "[BridgeAwareCubic][TaggedLines][ReferenceLines]")
{
    auto p = parameters();
    p.angle = GENERATE(0., 17. * M_PI / 180., M_PI / 4.);
    p.bounds.translate(Vec3d(.12345, -.54321, 0.));
    const ExPolygon whole = rectangle(-24., -14., 24., 14.);
    const BAC::Plan plan(p, {{{whole}, 13., 6.}});
    const BoundingBox bounds(Point::new_scale(-12.123, -8.456), Point::new_scale(15.789, 9.234));
    bool clipped_axis = false;
    for (double z : {12.8, 13., 13.2}) {
        CAPTURE(p.angle, z);
        const auto reference = plan.reference_lines(z, bounds);
        exact_reference(plan, z, bounds, reference);
        REQUIRE_FALSE(reference.paths.empty());
        for (size_t i = 0; i < reference.paths.size(); ++i) {
            const auto &source = reference.sources[reference.source_for_path[i]];
            clipped_axis |= !bounds.contains(source.canonical_a) || !bounds.contains(source.canonical_b);
        }
    }
    REQUIRE(clipped_axis);
}

TEST_CASE("Bridge aware reference batches preserve a reversed horizontal clipping witness", "[BridgeAwareCubic][TaggedLines][ReferenceLines]")
{
    auto p = parameters();
    p.angle = 0.;
    p.bounds.translate(Vec3d(.12345, -.54321, 0.));
    const ExPolygon whole = rectangle(-24., -14., 24., 14.);
    const BAC::Plan plan(p, {{{whole}, 13., 6.}});
    const BoundingBox bounds(Point::new_scale(-12.123, -8.456), Point::new_scale(15.789, 9.234));
    bool reversed_horizontal = false;
    for (double z : {12.8, 13., 13.2}) {
        const auto reference = plan.reference_lines(z, bounds);
        exact_reference(plan, z, bounds, reference);
        for (size_t i = 0; i < reference.paths.size(); ++i) {
            const auto &path = reference.paths[i];
            const auto &source = reference.sources[reference.source_for_path[i]];
            if (source.canonical_a.y() != source.canonical_b.y()) continue;
            // Horizontal scan conversion can reverse an actual source; this
            // is not a promise that every nonhorizontal angle also reverses.
            reversed_horizontal |= (path.back() - path.front()).cast<double>().dot(
                (source.canonical_b - source.canonical_a).cast<double>()) < 0.;
        }
    }
    REQUIRE(reversed_horizontal);
}

TEST_CASE("Bridge aware reference batches retain tiny fragments and clipped out source rows", "[BridgeAwareCubic][TaggedLines][ReferenceLines]")
{
    const auto p = parameters();
    const ExPolygon whole = rectangle(-24., -14., 24., 14.);
    const BAC::Plan plan(p, {{{whole}, 13., 6.}});
    const auto broad = plan.reference_lines(13., BoundingBox(Point::new_scale(-24., -14.), Point::new_scale(24., 14.)));
    REQUIRE(broad.complete);
    REQUIRE_FALSE(broad.paths.empty());
    const Polyline &axis = broad.paths.front();
    const Point middle = axis.front() + (axis.back() - axis.front()) / 2;
    const BoundingBox tiny(middle - Point(2, 2), middle + Point(2, 2));
    const auto reference = plan.reference_lines(13., tiny);
    exact_reference(plan, 13., tiny, reference);
    same_sources(reference, broad);
    REQUIRE_FALSE(reference.paths.empty());
    for (const Polyline &path : reference.paths) CHECK(path.length() <= 8.);
    REQUIRE(std::any_of(reference.partitions.begin(), reference.partitions.end(), [](const auto &entry) {
        return std::any_of(entry.second.begin(), entry.second.end(), [](const Polyline &path) {
            return path.length() > 0. && path.length() <= 8.;
        });
    }));
    REQUIRE(reference.sources.size() > reference.paths.size());
    const BoundingBox empty(Point::new_scale(100., 100.), Point::new_scale(101., 101.));
    const auto clipped_out = plan.reference_lines(13., empty);
    exact_reference(plan, 13., empty, clipped_out);
    CHECK(clipped_out.paths.empty());
    CHECK(clipped_out.partitions.empty());
    same_sources(clipped_out, broad);
}

TEST_CASE("Bridge aware reference admission is bounded and cancellation is never optional", "[BridgeAwareCubic][TaggedLines][ReferenceLines]")
{
    auto p = parameters();
    bool canceled = false;
    size_t calls = 0, allocation_cancel_at = 0;
    p.throw_if_canceled = [&] {
        ++calls;
        if (canceled) throw std::runtime_error("reference canceled");
        if (allocation_cancel_at && calls == allocation_cancel_at) throw std::bad_alloc();
    };
    const ExPolygon whole = rectangle(-24., -14., 24., 14.);
    const BAC::Plan plan(p, {{{whole}, 13., 6.}});
    const BoundingBox bounds(Point::new_scale(-24., -14.), Point::new_scale(24., 14.));
    const auto empty = [](const BAC::ReferenceLineBatch &reference) {
        CHECK_FALSE(reference.complete);
        CHECK_FALSE(reference.plan_identity);
        CHECK(reference.paths.empty()); CHECK(reference.sources.empty());
        CHECK(reference.source_for_path.empty()); CHECK(reference.partitions.empty());
    };
    for (size_t capacity : {size_t(0), size_t(1), size_t(64)})
        empty(plan.reference_lines(13., bounds, capacity));
    BridgeAwareCubicValidation::WorkBudget absent{0}, partial{100};
    empty(plan.reference_lines(13., bounds, 200000, &absent));
    empty(plan.reference_lines(13., bounds, 200000, &partial));
    CHECK(partial.remaining_work < 100);
    empty(plan.reference_lines(13., BoundingBox{}));
    constexpr coord_t limit = coord_t(std::min<int64_t>(std::numeric_limits<coord_t>::max() / 4,
                                                       std::numeric_limits<int32_t>::max() / 4));
    empty(plan.reference_lines(13., BoundingBox(Point(-limit - 1, -limit), Point(limit, limit))));
    empty(plan.reference_lines(13., BoundingBox(Point(-limit, -limit), Point(limit, limit + 1))));
    empty(plan.reference_lines(13., BoundingBox(Point(std::numeric_limits<coord_t>::min(), std::numeric_limits<coord_t>::min()),
                                               Point(std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max()))));
    const BoundingBox admissible(Point(-limit, -limit), Point(limit, limit));
    const auto admitted = plan.reference_lines(13., admissible);
    REQUIRE(admitted.complete);
    CHECK(admitted.paths == plan.refinement_lines(13., reference_domain(admissible)));
    CHECK(admitted.partitions == plan.partition_lines(13., reference_domain(admissible)));
    canceled = true;
    REQUIRE_THROWS_WITH(plan.reference_lines(13., bounds), "reference canceled");
    canceled = false; calls = 0;
    REQUIRE(plan.reference_lines(13., bounds).complete);
    allocation_cancel_at = calls;
    REQUIRE(allocation_cancel_at > 1);
    calls = 0;
    REQUIRE_THROWS_AS(plan.reference_lines(13., bounds), std::bad_alloc);
}

TEST_CASE("Bridge aware tagged bulk preserves exact ordinary geometry and order", "[BridgeAwareCubic][TaggedLines]")
{
    auto p = parameters();
    p.angle = GENERATE(0., 17. * M_PI / 180.);
    p.bounds.translate(Vec3d(.12345, -.54321, 0.));
    const BAC::Plan plan(p, {});
    const ExPolygon domain = rectangle(-20.123, -12.456, 22.789, 11.234);
    for (double z : {1.3, 8.7, 13.}) {
        CAPTURE(p.angle, z);
        const auto batch = plan.tagged_lines(z, domain);
        complete_batch(batch, plan.lines(z, domain));
        REQUIRE_FALSE(batch.paths.empty());
        for (const auto &source : batch.sources) {
            CHECK(source.kind == BAC::LineSource::Kind::Bulk);
            CHECK(source.producers.empty());
        }
        finite_sources(plan, batch, z);
    }
}

TEST_CASE("Bridge aware tagged partitions preserve canonical axes and finite owner coverage", "[BridgeAwareCubic][TaggedLines]")
{
    auto p = parameters();
    p.angle = GENERATE(0., 17. * M_PI / 180., M_PI / 4.);
    const ExPolygon domain = rectangle(-24., -14., 24., 14.);
    const double z = 13.;
    const BAC::Plan plan(p, {{{domain}, z, 6.}}), bulk(p, {});
    REQUIRE(plan.statistics().splits > 0);
    REQUIRE(plan.solid_area(z).empty());
    const auto batch = plan.tagged_lines(z, domain);
    complete_batch(batch, plan.lines(z, domain));
    const auto ordinary_bulk = bulk.lines(z, domain);
    const auto added = plan.refinement_lines(z, domain);
    REQUIRE(batch.paths.size() == ordinary_bulk.size() + added.size());
    for (size_t i = 0; i < batch.paths.size(); ++i) {
        const auto &source = batch.sources[batch.source_for_path[i]];
        CHECK(source.kind == (i < ordinary_bulk.size() ? BAC::LineSource::Kind::Bulk : BAC::LineSource::Kind::Partition));
        CHECK(batch.paths[i] == (i < ordinary_bulk.size() ? ordinary_bulk[i] : added[i - ordinary_bulk.size()]));
    }
    finite_sources(plan, batch, z);
    CHECK(owned_fragments(batch) == plan.partition_lines(z, domain));
    bool has_shared_source = false, has_finite_subinterval = false;
    for (const auto *source : partition_sources(batch)) {
        has_shared_source = has_shared_source || source->producers.size() > 1;
        for (const auto &producer : source->producers)
            has_finite_subinterval = has_finite_subinterval || producer.first > 0. || producer.last < 1.;
    }
    REQUIRE(has_shared_source);
    REQUIRE(has_finite_subinterval); // A touched-owner set cannot replace finite ranges.
}

TEST_CASE("Bridge aware tagged source tables survive holes and wholly clipped sources", "[BridgeAwareCubic][TaggedLines]")
{
    const auto p = parameters();
    const ExPolygon whole = rectangle(-24., -14., 24., 14.);
    const BAC::Plan plan(p, {{{whole}, 13., 6.}});
    const auto broad = plan.tagged_lines(13., whole);
    complete_batch(broad, plan.lines(13., whole));
    ExPolygon holed = rectangle(-22., -12., 22., 12.);
    Polygon hole = rectangle(-7., -5., 7., 5.).contour;
    hole.reverse();
    holed.holes.push_back(std::move(hole));
    const auto split = plan.tagged_lines(13., holed);
    complete_batch(split, plan.lines(13., holed));
    same_sources(broad, split);
    CHECK(owned_fragments(split) == plan.partition_lines(13., holed));
    std::map<size_t, size_t> occurrences;
    for (size_t source : split.source_for_path)
        if (split.sources[source].kind == BAC::LineSource::Kind::Partition) ++occurrences[source];
    REQUIRE(std::any_of(occurrences.begin(), occurrences.end(), [](const auto &item) { return item.second > 1; }));
    const ExPolygon narrow = rectangle(20., 10., 22., 12.);
    const auto clipped = plan.tagged_lines(13., narrow);
    complete_batch(clipped, plan.lines(13., narrow));
    same_sources(broad, clipped); // Complete includes output-unreferenced raw sources.
    size_t unused = 0;
    for (size_t i = 0; i < clipped.sources.size(); ++i)
        if (clipped.sources[i].kind == BAC::LineSource::Kind::Partition &&
            std::find(clipped.source_for_path.begin(), clipped.source_for_path.end(), i) == clipped.source_for_path.end())
            ++unused;
    REQUIRE(unused > 0);
}

TEST_CASE("Bridge aware tagged ownership drops collapsed producers without changing its source plan", "[BridgeAwareCubic][TaggedLines]")
{
    const auto p = parameters();
    const ExPolygon domain = rectangle(-24., -14., 24., 14.);
    const BAC::Plan plan(p, {{{domain}, 13., 6.}});
    const auto original = plan.tagged_lines(13., domain);
    complete_batch(original, plan.lines(13., domain));
    const auto sources = partition_sources(original);
    REQUIRE_FALSE(sources.empty());
    REQUIRE_FALSE(sources.front()->producers.empty());
    const BAC::CellId removed = sources.front()->producers.front().owner;
    const BAC::Plan trial = plan.collapsed(removed);
    const auto changed = trial.tagged_lines(13., domain);
    complete_batch(changed, trial.lines(13., domain));
    finite_sources(trial, changed, 13.);
    CHECK(owned_fragments(changed) == trial.partition_lines(13., domain));
    for (const auto &source : changed.sources)
        for (const auto &producer : source.producers) CHECK_FALSE(producer.owner == removed);
    const BoundingBox bounds(Point::new_scale(-24., -14.), Point::new_scale(24., 14.));
    const auto reference = trial.reference_lines(13., bounds);
    exact_reference(trial, 13., bounds, reference);
    CHECK(reference.plan_identity != plan.immutable_identity());
    for (const auto &owner : reference.partitions) CHECK_FALSE(owner.first == removed);
    const auto unchanged = plan.tagged_lines(13., domain);
    CHECK(unchanged.paths == original.paths);
    CHECK(unchanged.source_for_path == original.source_for_path);
    same_sources(original, unchanged);
}

TEST_CASE("Bridge aware tagged admission falls back to unchanged geometry and propagates cancellation", "[BridgeAwareCubic][TaggedLines]")
{
    auto p = parameters();
    bool canceled = false;
    size_t cancellation_calls = 0, allocation_cancel_at = 0;
    p.throw_if_canceled = [&] {
        ++cancellation_calls;
        if (canceled) throw std::runtime_error("tagged lines canceled");
        if (allocation_cancel_at != 0 && cancellation_calls == allocation_cancel_at) throw std::bad_alloc();
    };
    const ExPolygon domain = rectangle(-24., -14., 24., 14.);
    const BAC::Plan plan(p, {{{domain}, 13., 6.}});
    const auto ordinary = plan.lines(13., domain);
    for (size_t capacity : {size_t(0), size_t(1)}) {
        const auto unavailable = plan.tagged_lines(13., domain, capacity);
        CHECK_FALSE(unavailable.complete);
        CHECK(unavailable.paths == ordinary);
        CHECK(unavailable.sources.empty());
        CHECK(unavailable.source_for_path.empty());
    }
    BridgeAwareCubicValidation::WorkBudget absent{0}, partial{3};
    for (auto *budget : {&absent, &partial}) {
        const auto unavailable = plan.tagged_lines(13., domain, 200000, budget);
        CHECK_FALSE(unavailable.complete);
        CHECK(unavailable.paths == ordinary);
        CHECK(unavailable.sources.empty());
        CHECK(unavailable.source_for_path.empty());
    }
    CHECK(partial.remaining_work < 3); // Optional work is never refunded.
    canceled = true;
    REQUIRE_THROWS_WITH(plan.tagged_lines(13., domain), "tagged lines canceled");
    canceled = false;
    cancellation_calls = 0;
    REQUIRE(plan.tagged_lines(13., domain).complete);
    allocation_cancel_at = cancellation_calls;
    REQUIRE(allocation_cancel_at > 1);
    cancellation_calls = 0;
    // The final cancellation poll occurs after complete metadata construction;
    // an allocation-shaped callback exception must not become a fallback.
    REQUIRE_THROWS_AS(plan.tagged_lines(13., domain), std::bad_alloc);
}

TEST_CASE("Bridge aware tagged initial scope declines active solid and closure layers", "[BridgeAwareCubic][TaggedLines]")
{
    auto p = parameters();
    p.bounds = BoundingBoxf3(Vec3d(-12., -12., 0.), Vec3d(12., 12., 12.));
    p.density = .5;
    p.allow_solid_fallback = true;
    const ExPolygon domain = rectangle(-11., -11., 11., 11.);
    const double z = 5.5 * std::sqrt(2.) * p.flow_spacing / p.density;
    const BAC::Plan saturated(p, {{{domain}, z, 1.}});
    REQUIRE_FALSE(saturated.solid_area(z).empty());
    const BAC::LayerClosure closure{z, {Polyline(Point::new_scale(-3., 0.), Point::new_scale(3., 0.))},
                                       {rectangle(-3., -.3, 3., .3)}};
    const BAC::Plan closed(p, {}, {closure});
    REQUIRE_FALSE(closed.closures(z).empty());
    for (const BAC::Plan *plan : {&saturated, &closed}) {
        for (double level : {z, z + .1, -1., 13.})
            CHECK(plan->has_closures(level) == !plan->closures(level).empty());
        const auto unavailable = plan->tagged_lines(z, domain);
        CHECK_FALSE(unavailable.complete);
        CHECK(unavailable.paths == plan->lines(z, domain));
        CHECK(unavailable.sources.empty());
        CHECK(unavailable.source_for_path.empty());
        const auto reference = plan->reference_lines(z, BoundingBox(Point::new_scale(-11., -11.), Point::new_scale(11., 11.)));
        CHECK_FALSE(reference.complete);
        CHECK_FALSE(reference.plan_identity);
        CHECK(reference.paths.empty());
        CHECK(reference.sources.empty());
        CHECK(reference.source_for_path.empty());
        CHECK(reference.partitions.empty());
    }
}

TEST_CASE("Bridge aware attachment visits preserve active storage order without copying", "[BridgeAwareCubic][TaggedLines]")
{
    auto p = parameters();
    bool canceled = false;
    p.throw_if_canceled = [&] { if (canceled) throw std::runtime_error("attachment visit canceled"); };
    const ExPolygon domain = rectangle(-24., -14., 24., 14.);
    const std::vector<BAC::Demand> demands{{{domain}, 13., 6.}};
    const BAC::Plan initial(p, demands);
    REQUIRE_FALSE(initial.refinements().empty());
    const auto owner = initial.refinements().front().id;
    BAC::LayerAttachment direct{13., 2, Polyline(Point::new_scale(-1., 0.), Point::new_scale(1., 0.)),
        Polyline(Point::new_scale(-1.1, 0.), Point::new_scale(1.1, 0.)), .45, .2};
    BAC::LayerAttachment active = direct, inactive = direct, other_layer = direct;
    active.emitting_region = 3;
    active.owners = {owner};
    inactive.emitting_region = 4;
    inactive.owners = {owner};
    inactive.owners.front().address[0] += 1000;
    other_layer.z = 12.;
    const BAC::Plan plan(p, demands, {}, {inactive, direct, other_layer, active});
    const auto copied = plan.attachments(13.);
    REQUIRE(copied.size() == 2);
    std::vector<const BAC::LayerAttachment *> visited;
    REQUIRE(plan.visit_attachments(13., [&](const auto &attachment) { visited.push_back(&attachment); return true; }));
    REQUIRE(visited.size() == copied.size());
    for (size_t i = 0; i < copied.size(); ++i) {
        CHECK(visited[i]->emitting_region == copied[i].emitting_region);
        CHECK(visited[i]->original == copied[i].original);
        CHECK(visited[i]->replacement == copied[i].replacement);
        CHECK(visited[i]->width == copied[i].width);
        CHECK(visited[i]->height == copied[i].height);
        CHECK(visited[i]->owners == copied[i].owners);
    }
    size_t count = 0;
    CHECK_FALSE(plan.visit_attachments(13., [&](const auto &attachment) {
        CHECK(&attachment == visited.front()); ++count; return false;
    }));
    CHECK(count == 1);
    CHECK(plan.visit_attachments(19., [&](const auto &) { ++count; return false; }));
    CHECK(count == 1);
    canceled = true;
    REQUIRE_THROWS_WITH(plan.visit_attachments(13., [](const auto &) { return true; }), "attachment visit canceled");
    REQUIRE_THROWS_WITH(plan.has_closures(13.), "attachment visit canceled");
}
