#include "BridgeAwareCubicEmissionLedger.hpp"
#include "BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <new>
#include <typeinfo>

namespace Slic3r { namespace BridgeAwareCubic {
namespace {
constexpr size_t max_bytes = 32 * 1024 * 1024, max_items = 20000, max_vertices = 200000;
constexpr int64_t coordinate_limit = std::min<int64_t>(std::numeric_limits<coord_t>::max() / 4,
                                                       std::numeric_limits<int32_t>::max() / 4);
struct Unavailable { const char *reason = nullptr; };
struct CallbackError { std::exception_ptr exception; };

struct Work {
    BridgeAwareCubicValidation::WorkBudget local;
    BridgeAwareCubicValidation::WorkBudget &budget;
    const EmissionCancel &cancel;
    size_t polls = 0;
    Work(BridgeAwareCubicValidation::WorkBudget *shared, const EmissionCancel &callback)
        : budget(shared ? *shared : local), cancel(callback) { poll(); }
    void poll() const
    {
        if (cancel) try { cancel(); } catch (...) { throw CallbackError{std::current_exception()}; }
    }
    void charge(size_t count = 1)
    {
        if (count > budget.remaining_work) throw Unavailable{"work_or_capacity"};
        budget.remaining_work -= count;
        budget.charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += count;
        if (count >= 256 || (++polls & 255) == 0) poll();
    }
};

bool safe_point(const Point &p)
{
    return p.x() >= -coordinate_limit && p.x() <= coordinate_limit &&
           p.y() >= -coordinate_limit && p.y() <= coordinate_limit;
}
bool same_points(const Points &a, const Points &b, Work &work)
{
    work.charge();
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        work.charge();
        if (a[i] != b[i] || !safe_point(a[i])) return false;
    }
    return true;
}
bool linear(const Polyline &line, Work &work)
{
    work.charge();
    if (line.size() < 2 || line.size() > max_vertices || !line.fitting_result.empty()) return false;
    bool moving = false;
    for (size_t i = 0; i < line.size(); ++i) {
        work.charge();
        if (!safe_point(line.points[i])) return false;
        moving |= i != 0 && line.points[i] != line.points[i - 1];
    }
    return moving;
}
bool same_line(const Polyline &a, const Polyline &b, Work &work)
{
    return a.fitting_result.empty() && b.fitting_result.empty() && same_points(a.points, b.points, work);
}
bool eligible_path(const ExtrusionPath &path, Work &work)
{
    return typeid(path) == typeid(ExtrusionPath) && path.role() == erInternalInfill &&
        !path.is_force_no_extrusion() && std::isfinite(path.width) && path.width > 0.f &&
        std::isfinite(path.height) && path.height > 0.f &&
        std::isfinite(path.mm3_per_mm) && path.mm3_per_mm > 0. &&
        std::isfinite(path.overhang_degree) && std::isfinite(path.smooth_speed) && linear(path.polyline, work);
}
bool same_path_metadata(const ExtrusionPath &a, const ExtrusionPath &b)
{
    return typeid(a) == typeid(ExtrusionPath) && typeid(b) == typeid(ExtrusionPath) &&
        a.role() == b.role() && a.width == b.width && a.height == b.height && a.mm3_per_mm == b.mm3_per_mm &&
        a.overhang_degree == b.overhang_degree && a.curve_degree == b.curve_degree && a.smooth_speed == b.smooth_speed &&
        a.is_force_no_extrusion() == b.is_force_no_extrusion() && a.can_reverse() == b.can_reverse() &&
        a.get_customize_flag() == b.get_customize_flag() && a.get_cooling_node() == b.get_cooling_node();
}
bool same_path(const ExtrusionPath &a, const ExtrusionPath &b, Work &work)
{
    work.charge();
    return same_path_metadata(a, b) && same_line(a.polyline, b.polyline, work);
}
bool same_owners(const std::vector<CellId> &a, const std::vector<CellId> &b, Work &work)
{
    work.charge();
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) { work.charge(); if (!(a[i] == b[i])) return false; }
    return true;
}
bool same_transition(const EmissionTransition &a, const EmissionTransition &b, Work &work)
{
    work.charge();
    const auto &x = a.attachment, &y = b.attachment;
    // Solid-cut/finite-selection emitters are deliberately unavailable here.
    return a.storage_region == b.storage_region && x.z == y.z && x.emitting_region == y.emitting_region &&
        x.width == y.width && x.height == y.height && !x.solid_cut && !y.solid_cut &&
        x.demand_selection.empty() && y.demand_selection.empty() &&
        same_owners(x.owners, y.owners, work) && same_line(x.original, y.original, work) &&
        same_line(x.replacement, y.replacement, work);
}
bool same_source(const LineSource &a, const LineSource &b, Work &work)
{
    work.charge();
    if (a.kind != b.kind || a.canonical_a != b.canonical_a || a.canonical_b != b.canonical_b ||
        a.producers.size() != b.producers.size()) return false;
    for (size_t i = 0; i < a.producers.size(); ++i) {
        work.charge();
        if (!(a.producers[i].owner == b.producers[i].owner) ||
            a.producers[i].first != b.producers[i].first || a.producers[i].last != b.producers[i].last) return false;
    }
    return true;
}
bool same_bindings(const std::vector<EmissionPlanBinding> &a,
                   const std::vector<EmissionPlanBinding> &b, Work &work)
{
    work.charge();
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        work.charge();
        if (a[i].storage_region != b[i].storage_region || a[i].identity != b[i].identity) return false;
    }
    return true;
}

struct Measure {
    Work &work;
    size_t limit, bytes = 0, items = 0, vertices = 0;
    void add(size_t count, size_t size = 1)
    {
        if (count > (limit - bytes) / size) throw Unavailable{"work_or_capacity"};
        bytes += count * size;
    }
    template<class T> void array(const std::vector<T> &values, bool points = false)
    {
        work.charge(1 + values.size());
        size_t &total = points ? vertices : items;
        const size_t cap = points ? max_vertices : max_items;
        if (values.size() > cap - total) throw Unavailable{"work_or_capacity"};
        total += values.size();
        add(values.capacity(), sizeof(T));
    }
    void line(const Polyline &line)
    {
        if (!line.fitting_result.empty()) throw Unavailable{};
        array(line.points, true);
    }
    void transitions(const std::vector<EmissionTransition> &entries)
    {
        array(entries);
        for (const auto &entry : entries) {
            line(entry.attachment.original); line(entry.attachment.replacement);
            array(entry.attachment.owners);
            if (entry.attachment.solid_cut || !entry.attachment.demand_selection.empty()) throw Unavailable{};
        }
    }
    void sources(const std::vector<LineSource> &sources)
    {
        array(sources);
        for (const auto &source : sources) array(source.producers);
    }
    void batch(const TaggedLineBatch &batch)
    {
        array(batch.paths); array(batch.source_for_path); sources(batch.sources);
        for (const auto &line : batch.paths) this->line(line);
    }
    void lines(const Polylines &paths)
    {
        array(paths);
        for (const auto &path : paths) line(path);
    }
    void partitions(const std::vector<std::pair<CellId, Polylines>> &groups)
    {
        array(groups);
        for (const auto &group : groups) lines(group.second);
    }
};
} // namespace

struct EmissionLedgerState {
    size_t emitter = 0;
    double z = 0.;
    EmissionReference reference;
    std::vector<EmissionPlanBinding> plans;
    std::vector<EmissionTransition> active;
    std::vector<LineSource> sources;
    std::vector<size_t> partition_table;
    Polylines reference_paths;
    std::vector<size_t> reference_sources;
    std::vector<std::pair<CellId, Polylines>> reference_partitions;
    bool reference_bound = false;
    std::vector<EmissionRecord> records;
    // These event handles never survive sealing and never authorize lookup.
    std::vector<const ExtrusionPath*> handles;
    bool bound = false;
    size_t bytes = 0;
};

struct EmissionSnapshotState {
    size_t emitter = 0;
    double z = 0.;
    EmissionReference reference;
    std::vector<EmissionPlanBinding> plans;
    std::vector<EmissionTransition> active;
    std::vector<LineSource> sources;
    std::vector<size_t> partition_table;
    std::vector<EmissionSnapshotRecord> records;
    size_t bytes = 0, items = 0, vertices = 0;
};

struct EmissionProjectionState {
    EmissionSnapshotPtr snapshot;
    std::vector<EmissionClassification> records;
    size_t bytes = 0;
};

struct EmissionDeltaState {
    EmissionSnapshotPtr before, after;
    std::vector<EmissionMatch> matches;
    std::vector<size_t> removed, added;
    bool order_changed = false, context_changed = false;
    size_t bytes = 0;
};

namespace {
// Views let eager and lazy projection share exactly the same classifier and
// lineage predicates. No raw path, source or attachment payload is copied.
struct ProjectedRecord {
    const ExtrusionPath &path;
    size_t source_index;
    const std::vector<EmissionTransition> &transitions;
    bool &construction, &mask_full_path;
    std::vector<CellId> &partition_owners;
    std::vector<size_t> &attachment_indices;
    ProjectedRecord(const EmissionSnapshotRecord &raw, EmissionClassification &derived)
        : path(raw.path), source_index(raw.source_index), transitions(raw.transitions),
          construction(derived.construction), mask_full_path(derived.mask_full_path),
          partition_owners(derived.partition_owners), attachment_indices(derived.attachment_indices) {}
};

struct ProjectionView {
    const EmissionSnapshotState &raw;
    EmissionProjectionState &output;
    const ReferenceLineBatch &batch;
    const size_t emitter;
    const EmissionReference &reference;
    const std::vector<EmissionTransition> &active;
    const std::vector<LineSource> &sources;
    const std::vector<size_t> &partition_table;
    const Polylines &reference_paths;
    const std::vector<size_t> &reference_sources;
    const std::vector<std::pair<CellId, Polylines>> &reference_partitions;
    const bool reference_bound = true;
    std::vector<ProjectedRecord> records;
    size_t bytes = 0;
    ProjectionView(const EmissionSnapshotState &input, EmissionProjectionState &result,
                   const ReferenceLineBatch &broad)
        : raw(input), output(result), batch(broad), emitter(input.emitter), reference(input.reference),
          active(input.active), sources(input.sources), partition_table(input.partition_table),
          reference_paths(broad.paths), reference_sources(broad.source_for_path), reference_partitions(broad.partitions) {}
};

size_t measure(const EmissionLedgerState &state, Work &work, size_t limit, size_t *items = nullptr)
{
    Measure m{work, limit};
    // Include object and shared-control overhead conservatively, including
    // the temporary wrapper while unique state ownership transfers at seal.
    m.add(sizeof(EmissionLedgerState) + sizeof(EmissionLedger) + 128);
    m.array(state.plans); m.transitions(state.active); m.sources(state.sources);
    // Identity tokens retain their separate tiny nonce/control allocations,
    // not an entire Plan implementation. Count a conservative supported-stdlib
    // allowance even when another live holder shares that allocation.
    m.add(state.plans.size(), 128);
    m.array(state.partition_table); m.array(state.records); m.array(state.handles);
    m.lines(state.reference_paths); m.array(state.reference_sources); m.partitions(state.reference_partitions);
    for (const auto &record : state.records) {
        m.line(record.path.polyline); m.array(record.partition_owners);
        m.transitions(record.transitions); m.array(record.attachment_indices);
    }
    if (items) *items = m.items;
    return m.bytes;
}

size_t measure_snapshot(EmissionSnapshotState &state, Work &work, size_t limit)
{
    Measure m{work, limit};
    m.add(sizeof(EmissionSnapshotState) + sizeof(EmissionSnapshot) + 128);
    m.array(state.plans); m.add(state.plans.size(), 128);
    m.transitions(state.active); m.sources(state.sources); m.array(state.partition_table); m.array(state.records);
    for (const auto &record : state.records) {
        m.line(record.path.polyline); m.transitions(record.transitions);
    }
    state.items = m.items;
    state.vertices = m.vertices;
    return m.bytes;
}

void measure_reference(Measure &m, const ReferenceLineBatch &batch)
{
    m.add(sizeof(ReferenceLineBatch) + 128);
    m.lines(batch.paths); m.array(batch.source_for_path); m.sources(batch.sources); m.partitions(batch.partitions);
}

void measure_projection(Measure &m, const EmissionProjectionState &state)
{
    m.add(sizeof(EmissionProjectionState) + sizeof(EmissionProjection) + 128);
    m.array(state.records);
    for (const auto &record : state.records) {
        m.array(record.partition_owners); m.array(record.attachment_indices);
    }
}

size_t measure(const ProjectionView &state, Work &work, size_t limit, size_t *items = nullptr)
{
    Measure m{work, limit};
    m.add(state.raw.bytes);
    m.items = state.raw.items; m.vertices = state.raw.vertices;
    measure_reference(m, state.batch); measure_projection(m, state.output);
    m.add(sizeof(ProjectionView)); m.array(state.records);
    if (items) *items = m.items;
    return m.bytes;
}
size_t remaining(size_t limit, size_t used)
{
    if (used > limit) throw Unavailable{"work_or_capacity"};
    return limit - used;
}
size_t allowance(const EmissionLedgerLimits &limits)
{
    if (limits.capacity_bytes > max_bytes || limits.retained_bytes > limits.capacity_bytes)
        throw Unavailable{"work_or_capacity"};
    return limits.capacity_bytes - limits.retained_bytes;
}
void admitted(EmissionLedgerState &state, size_t scratch, size_t limit, Work &work)
{
    state.bytes = measure(state, work, remaining(limit, scratch));
}
bool interior(const Point &p, const BoundingBox &box, long double margin)
{
    return p.x() > static_cast<long double>(box.min.x()) + margin &&
           p.x() < static_cast<long double>(box.max.x()) - margin &&
           p.y() > static_cast<long double>(box.min.y()) + margin &&
           p.y() < static_cast<long double>(box.max.y()) - margin;
}
long double length(const Point &a, const Point &b)
{
    return std::hypot(static_cast<long double>(b.x()) - a.x(), static_cast<long double>(b.y()) - a.y());
}

// Overbound intersection with the finite axis's oriented rectangle. Both
// longitudinal and transverse padding are outward. No polygon is made and
// this rectangle supplies classification bounds only, never physical support.
long double possible(const Point &a, const Point &b, const Point &s, const Point &t, long double radius)
{
    const long double dx = static_cast<long double>(t.x()) - s.x(), dy = static_cast<long double>(t.y()) - s.y();
    const long double axis_length = std::hypot(dx, dy), path_length = length(a, b);
    if (axis_length <= 0. || path_length <= 0.) return 0.;
    const auto along = [&](const Point &p) { return ((static_cast<long double>(p.x()) - s.x()) * dx +
                                                   (static_cast<long double>(p.y()) - s.y()) * dy) / axis_length; };
    const auto across = [&](const Point &p) { return ((static_cast<long double>(p.x()) - s.x()) * dy -
                                                    (static_cast<long double>(p.y()) - s.y()) * dx) / axis_length; };
    long double lo = 0., hi = 1.;
    const auto clip = [&](long double first, long double last, long double lower, long double upper) {
        const long double delta = last - first;
        if (delta == 0.) return first >= lower && first <= upper;
        const long double x = (lower - first) / delta, y = (upper - first) / delta;
        lo = std::max(lo, std::min(x, y)); hi = std::min(hi, std::max(x, y));
        return hi >= lo;
    };
    if (!clip(along(a), along(b), -radius, axis_length + radius) ||
        !clip(across(a), across(b), -radius, radius)) return 0.;
    return std::max(0.L, hi - lo) * path_length + 1.;
}
bool contained(const Polyline &path, const Point &a, const Point &b, long double radius, Work &work,
               bool strict_caps = false)
{
    // Every source/reference/path point has passed safe_point(). Differences
    // are at most 2*coordinate_limit, so dot/squared/cross sums are bounded by
    // 8*coordinate_limit^2 < INT64_MAX. Exact longitudinal bounds preserve an
    // endpoint even when squared/hypot rounds just above hypot.
    const int64_t ax = int64_t(b.x()) - a.x(), ay = int64_t(b.y()) - a.y();
    const int64_t squared = ax * ax + ay * ay;
    const long double dx = ax, dy = ay;
    const long double size = std::hypot(dx, dy);
    if (size <= 8. || radius <= 0.) return false;
    for (const Point &p : path.points) {
        work.charge();
        const int64_t px = int64_t(p.x()) - a.x(), py = int64_t(p.y()) - a.y();
        const int64_t dot = px * ax + py * ay;
        const long double x = px, y = py;
        const long double projected = (x * dx + y * dy) / size;
        if (dot < 0 || dot > squared || std::abs(x * dy - y * dx) >= radius * size) return false;
        if (strict_caps && (projected < 8. || projected > size - 8.)) {
            // The original exact axis (including its own endpoints) remains
            // covered. A transverse path on a rounded butt boundary has no
            // strict positive containment proof, even when within the strip.
            if (px * ay != py * ax) return false;
        }
    }
    return true;
}

// This is just the exact, inexpensive whole-segment shortcut of the legacy
// matcher on a KNOWN finite source. No reverse source inference, offset,
// intersection or tolerance expansion is performed here.
bool wholly_owned(const Point &a, const Point &b, const Point &s, const Point &t)
{
    const Vec2d axis = (b - a).cast<double>(), other = (t - s).cast<double>();
    const double l = axis.norm(), m = other.norm();
    if (l <= 0. || m <= 0.) return false;
    const auto cross = [](const Vec2d &x, const Vec2d &y) { return x.x() * y.y() - x.y() * y.x(); };
    const Vec2d reference = l >= m ? axis : other;
    const Point origin = l >= m ? a : s, first = l >= m ? s : a, last = l >= m ? t : b;
    const double begin = other.dot((a - s).cast<double>()), end = other.dot((b - s).cast<double>());
    return std::abs(cross(reference, (first - origin).cast<double>())) <= 2. * std::max(l, m) &&
           std::abs(cross(reference, (last - origin).cast<double>())) <= 2. * std::max(l, m) &&
           std::min(begin, end) >= 0. && std::max(begin, end) <= other.squaredNorm();
}

struct FiniteOwner { CellId owner; Point a, b, reverse_a, reverse_b; };
Point at(const LineSource &source, double position, bool reverse)
{
    if (position == 0.) return source.canonical_a;
    if (position == 1.) return source.canonical_b;
    const Vec2d a = source.canonical_a.cast<double>(), b = source.canonical_b.cast<double>();
    const Vec2d point = reverse ? b + (a - b) * (1. - position) : a + (b - a) * position;
    return Point(coord_t(std::llround(point.x())), coord_t(std::llround(point.y())));
}

template<class State>
void classify(State &state, Work &work, size_t state_and_temporary_limit)
{
    const auto &box = state.reference.classification_bounds;
    const long double mask_radius = state.reference.tolerance_mm / SCALING_FACTOR;
    std::vector<FiniteOwner> owners;
    size_t count = 0;
    if (!state.reference_bound) for (size_t index : state.partition_table) {
        work.charge();
        const auto &source = state.sources.at(index);
        if (source.producers.size() > max_items - count) throw Unavailable{"work_or_capacity"};
        count += source.producers.size();
        // Synthetic callers without the exact reference retain the original
        // conservative locality proof. Production binds broad clipping first.
        if (!interior(source.canonical_a, box, mask_radius + 16.) ||
            !interior(source.canonical_b, box, mask_radius + 16.)) throw Unavailable{"reference_clip"};
    }
    size_t items = 0;
    state.bytes = measure(state, work, state_and_temporary_limit, &items);
    const size_t temporary_limit = remaining(state_and_temporary_limit, state.bytes);
    if (count > temporary_limit / sizeof(FiniteOwner)) throw Unavailable{"work_or_capacity"};
    owners.reserve(count);
    if (owners.capacity() > temporary_limit / sizeof(FiniteOwner)) throw Unavailable{"work_or_capacity"};
    if (!state.reference_bound) for (size_t index : state.partition_table) {
        const auto &source = state.sources[index];
        for (const auto &producer : source.producers) {
            work.charge();
            FiniteOwner owner{producer.owner, at(source, producer.first, false), at(source, producer.last, false),
                at(source, producer.first, true), at(source, producer.last, true)};
            // Without exact broad clipping, short reconstructed references are
            // ambiguous. Exact reference fragments below keep their shortcut.
            if (length(owner.a, owner.b) <= 16. || length(owner.reverse_a, owner.reverse_b) <= 16.)
                throw Unavailable{"owner_raw_short"};
            owners.push_back(owner);
        }
    }
    std::sort(owners.begin(), owners.end(), [&](const FiniteOwner &a, const FiniteOwner &b) {
        work.charge(); return a.owner < b.owner;
    });
    const size_t owners_bytes = owners.capacity() * sizeof(FiniteOwner);
    for (auto &record : state.records) {
        work.poll();
        const auto &path = record.path.polyline;
        work.charge(path.size());
        const double full_length = path.length();
        bool mask_full = false;
        if (state.sources[record.source_index].kind == LineSource::Kind::Partition) {
            if (state.reference_bound) {
                for (size_t i = 0; i < state.reference_paths.size(); ++i) {
                    work.charge();
                    if (state.reference_sources[i] != record.source_index) continue;
                    const auto &line = state.reference_paths[i];
                    mask_full |= contained(path, line.front(), line.back(), mask_radius - 8., work, true);
                }
            } else {
                const auto &source = state.sources[record.source_index];
                mask_full = contained(path, source.canonical_a, source.canonical_b, mask_radius - 8., work, true);
            }
        }
        if (!mask_full) {
            long double upper = 0.;
            const auto include_axis = [&](const Point &a, const Point &b) {
                for (size_t i = 1; i < path.size(); ++i) {
                    work.charge();
                    upper += possible(path.points[i - 1], path.points[i], a, b, mask_radius + 8.);
                }
            };
            if (state.reference_bound) {
                for (const auto &line : state.reference_paths) include_axis(line.front(), line.back());
            } else {
                for (size_t index : state.partition_table) {
                    const auto &source = state.sources[index];
                    include_axis(source.canonical_a, source.canonical_b);
                }
            }
            if (upper + mask_radius + 16. >= full_length)
                throw Unavailable{state.sources[record.source_index].kind == LineSource::Kind::Bulk ? "mask_bulk" :
                    record.attachment_indices.empty() ? "mask_partition" : "mask_attachment"};
        }
        record.mask_full_path = mask_full;
        record.construction = mask_full || !record.attachment_indices.empty();
        record.partition_owners.clear();
        if (!record.construction) continue;
        // Each independent owner is evaluated on the complete final path.
        // Tiny actual fragments retain the legacy nonlocal whole shortcut;
        // when it fires, even a partial-path upper bound counts that FULL
        // candidate segment rather than its local rectangle intersection.
        const auto classify_owner = [&](const CellId &id, const auto &visit) {
            bool complete = true;
            long double upper = 0.;
            for (size_t i = 1; i < path.size(); ++i) {
                bool segment_complete = false;
                long double partial = 0.;
                visit([&](const Point &a, const Point &b, const Point &reverse_a, const Point &reverse_b) {
                    work.charge();
                    segment_complete |= wholly_owned(path.points[i - 1], path.points[i], a, b) &&
                        wholly_owned(path.points[i - 1], path.points[i], reverse_a, reverse_b);
                    partial += possible(path.points[i - 1], path.points[i], a, b, 8.);
                });
                complete &= segment_complete;
                upper += segment_complete ? length(path.points[i - 1], path.points[i]) + 1. : partial;
            }
            if (complete) {
                if (items == max_items) throw Unavailable{"work_or_capacity"};
                const size_t wanted = record.partition_owners.size() + 1;
                const size_t free = remaining(remaining(state_and_temporary_limit, owners_bytes), state.bytes);
                if (wanted > free / sizeof(CellId)) throw Unavailable{"work_or_capacity"};
                const size_t previous_capacity = record.partition_owners.capacity();
                work.charge(wanted);
                record.partition_owners.reserve(wanted);
                if (record.partition_owners.capacity() > free / sizeof(CellId)) throw Unavailable{"work_or_capacity"};
                state.bytes += (record.partition_owners.capacity() - previous_capacity) * sizeof(CellId);
                record.partition_owners.push_back(id);
                ++items;
            } else if (upper + 18. >= full_length) throw Unavailable{"owner_ambiguous"};
        };
        if (state.reference_bound) {
            for (const auto &group : state.reference_partitions) {
                work.charge();
                classify_owner(group.first, [&](const auto &visit) {
                    for (const auto &line : group.second) visit(line.front(), line.back(), line.front(), line.back());
                });
            }
        } else {
            for (size_t owner_index = 0; owner_index < owners.size();) {
                work.charge();
                size_t end = owner_index + 1;
                while (end < owners.size() && owners[end].owner == owners[owner_index].owner) { work.charge(); ++end; }
                classify_owner(owners[owner_index].owner, [&](const auto &visit) {
                    for (size_t j = owner_index; j < end; ++j) {
                        const auto &owner = owners[j];
                        visit(owner.a, owner.b, owner.reverse_a, owner.reverse_b);
                    }
                });
                owner_index = end;
            }
            // Unreferenced raw sources can create additional tolerant owners
            // when no exact broad owner table is available.
            for (size_t index : state.partition_table) {
                if (index == record.source_index) continue;
                const auto &source = state.sources[index];
                if (contained(path, source.canonical_a, source.canonical_b, 8., work)) throw Unavailable{"owner_raw_near"};
            }
        }
    }
    work.poll();
}

// Compare the complete legacy applicable-index union against actual replay.
// Starting at every active original is necessary: a convergent alternate
// chain need not have been the path that the filler emitted.
template<class State, class Record>
void lineage(const State &state, Record &record, Work &work)
{
    std::vector<size_t> applicable;
    for (size_t start = 0; start < state.active.size(); ++start) {
        const auto &initial = state.active[start].attachment;
        if (initial.emitting_region != state.emitter || initial.width != record.path.width ||
            initial.height != record.path.height) continue;
        const Polyline *current = &initial.original;
        std::vector<size_t> chain;
        for (size_t step = 0; ; ++step) {
            if (step > state.active.size()) throw Unavailable{"lineage_cycle"};
            size_t found = state.active.size();
            for (size_t i = 0; i < state.active.size(); ++i) {
                work.charge();
                const auto &a = state.active[i].attachment;
                if (a.emitting_region != state.emitter || a.width != record.path.width || a.height != record.path.height ||
                    !same_line(*current, a.original, work)) continue;
                if (found != state.active.size()) throw Unavailable{"lineage_ambiguous"};
                found = i;
            }
            if (found == state.active.size()) break;
            work.charge(chain.size());
            if (std::find(chain.begin(), chain.end(), found) != chain.end()) throw Unavailable{"lineage_cycle"};
            chain.push_back(found);
            current = &state.active[found].attachment.replacement;
        }
        if (!chain.empty() && same_line(*current, record.path.polyline, work))
            applicable.insert(applicable.end(), chain.begin(), chain.end());
    }
    const auto less = [&](size_t a, size_t b) { work.charge(); return a < b; };
    const auto equal = [&](size_t a, size_t b) { work.charge(); return a == b; };
    std::sort(applicable.begin(), applicable.end(), less);
    applicable.erase(std::unique(applicable.begin(), applicable.end(), equal), applicable.end());
    std::vector<size_t> actual;
    for (const auto &transition : record.transitions) {
        size_t found = state.active.size();
        for (size_t i = 0; i < state.active.size(); ++i) {
            if (!same_transition(transition, state.active[i], work)) continue;
            if (found != state.active.size()) throw Unavailable{"lineage_ambiguous"};
            found = i;
        }
        if (found == state.active.size()) throw Unavailable{"lineage_missing"};
        actual.push_back(found);
    }
    std::sort(actual.begin(), actual.end(), less);
    actual.erase(std::unique(actual.begin(), actual.end(), equal), actual.end());
    work.charge(1 + actual.size() + applicable.size());
    if (actual != applicable) throw Unavailable{"lineage_union"};
    record.attachment_indices = std::move(actual);
}
} // namespace

EmissionLedger::EmissionLedger(std::unique_ptr<EmissionLedgerState> state) : m_state(std::move(state)) {}
EmissionLedger::~EmissionLedger() = default;
const std::vector<EmissionRecord> &EmissionLedger::records() const noexcept { return m_state->records; }
const std::vector<LineSource> &EmissionLedger::sources() const noexcept { return m_state->sources; }
size_t EmissionLedger::resident_bytes() const noexcept { return m_state->bytes; }
bool EmissionLedger::matches(size_t emitter, double z, const EmissionReference &reference,
    const std::vector<EmissionPlanBinding> &plans, const std::vector<EmissionPathView> &paths,
    BridgeAwareCubicValidation::WorkBudget *budget, const EmissionCancel &cancel) const
{
    try {
        Work work(budget, cancel);
        work.charge();
        const auto &s = *m_state;
        if (emitter != s.emitter || z != s.z || reference.tolerance_mm != s.reference.tolerance_mm ||
            reference.classification_bounds.defined != s.reference.classification_bounds.defined ||
            reference.classification_bounds.min != s.reference.classification_bounds.min ||
            reference.classification_bounds.max != s.reference.classification_bounds.max ||
            paths.size() != s.records.size() || !same_bindings(plans, s.plans, work)) return false;
        for (size_t i = 0; i < paths.size(); ++i)
            if (!paths[i].path || !paths[i].free_ended || !same_path(*paths[i].path, s.records[i].path, work)) return false;
        work.poll();
        return true;
    } catch (const CallbackError &e) { std::rethrow_exception(e.exception); }
      catch (const Unavailable &) { return false; }
}

EmissionLedgerBuilder::EmissionLedgerBuilder(size_t emitter, double z, const EmissionReference &reference,
    const std::vector<EmissionPlanBinding> &plans, const std::vector<EmissionTransition> &active,
    EmissionLedgerLimits limits, BridgeAwareCubicValidation::WorkBudget *budget, const EmissionCancel &cancel)
    : m_limits(limits)
{
    try {
        Work work(budget, cancel);
        const size_t limit = allowance(limits);
        if (!std::isfinite(z) || !reference.classification_bounds.defined ||
            reference.classification_bounds.min.x() >= reference.classification_bounds.max.x() ||
            reference.classification_bounds.min.y() >= reference.classification_bounds.max.y() ||
            !safe_point(reference.classification_bounds.min) || !safe_point(reference.classification_bounds.max) ||
            !std::isfinite(reference.tolerance_mm) || reference.tolerance_mm <= 8. * SCALING_FACTOR ||
            reference.tolerance_mm > 1.) throw Unavailable{};
        Measure input{work, limit}; input.array(plans); input.transitions(active); input.add(plans.size(), 128);
        if (plans.empty()) throw Unavailable{};
        bool own = false;
        for (size_t i = 0; i < plans.size(); ++i) {
            work.charge();
            if (!plans[i].identity || (i && plans[i - 1].storage_region >= plans[i].storage_region)) throw Unavailable{};
            own |= plans[i].storage_region == emitter;
        }
        if (!own) throw Unavailable{};
        for (const auto &entry : active) {
            work.charge();
            const auto &a = entry.attachment;
            bool storage = false;
            for (const auto &p : plans) { work.charge(); storage |= p.storage_region == entry.storage_region; }
            if (!storage || a.z != z || !std::isfinite(a.width) || a.width <= 0. ||
                !std::isfinite(a.height) || a.height <= 0. || !linear(a.original, work) ||
                !linear(a.replacement, work) || same_line(a.original, a.replacement, work)) throw Unavailable{};
        }
        if (input.bytes > limit / 2 || sizeof(EmissionLedgerState) + 256 > limit - 2 * input.bytes) throw Unavailable{};
        auto state = std::make_unique<EmissionLedgerState>();
        state->emitter = emitter; state->z = z; state->reference = reference;
        state->plans = plans; state->active = active;
        state->bytes = measure(*state, work, limit - input.bytes);
        work.poll();
        m_state = std::move(state);
    } catch (const CallbackError &e) { std::rethrow_exception(e.exception); }
      catch (const Unavailable &e) { m_decline_reason = e.reason ? e.reason : "capture"; }
      catch (const std::bad_alloc &) { m_decline_reason = "allocation"; }
}
EmissionLedgerBuilder::~EmissionLedgerBuilder() = default;
EmissionLedgerBuilder::EmissionLedgerBuilder(EmissionLedgerBuilder &&) noexcept = default;
EmissionLedgerBuilder &EmissionLedgerBuilder::operator=(EmissionLedgerBuilder &&) noexcept = default;
void EmissionLedgerBuilder::decline() noexcept
{
    if (!m_decline_reason) m_decline_reason = "declined";
    m_state.reset();
}
const char *EmissionLedgerBuilder::decline_reason() const noexcept
{
    return m_decline_reason ? m_decline_reason : "none";
}
size_t EmissionLedgerBuilder::staged_bytes() const noexcept { return m_state ? m_state->bytes : 0; }
size_t EmissionLedgerBuilder::available_scratch_bytes() const noexcept
{
    return m_state ? m_limits.capacity_bytes - m_limits.retained_bytes - m_state->bytes : 0;
}
bool EmissionLedgerBuilder::needs_reference() const noexcept
{
    return m_state && !m_state->reference_bound;
}
BoundingBox EmissionLedgerBuilder::reference_bounds() const noexcept
{
    return m_state ? m_state->reference.classification_bounds : BoundingBox();
}

bool EmissionLedgerBuilder::bind_reference(ReferenceLineBatch &&batch,
    BridgeAwareCubicValidation::WorkBudget *budget, const EmissionCancel &cancel, size_t additional_scratch_bytes)
{
    try {
        Work work(budget, cancel);
        if (!m_state) return false;
        auto &state = *m_state;
        if (state.bound || state.reference_bound || !state.sources.empty()) throw Unavailable{"reference"};
        const size_t limit = allowance(m_limits);
        Measure input{work, remaining(limit, state.bytes)};
        input.add(additional_scratch_bytes); input.add(sizeof(ReferenceLineBatch) + 128);
        input.lines(batch.paths); input.array(batch.source_for_path); input.sources(batch.sources);
        input.partitions(batch.partitions);
        const auto &box = state.reference.classification_bounds;
        if (!batch.complete || batch.z != state.z || !batch.bounds.defined ||
            batch.bounds.min != box.min || batch.bounds.max != box.max ||
            batch.paths.size() != batch.source_for_path.size()) throw Unavailable{"reference"};
        bool identity = false;
        for (const auto &binding : state.plans) {
            work.charge();
            identity |= binding.storage_region == state.emitter && binding.identity == batch.plan_identity;
        }
        if (!identity) throw Unavailable{"reference"};
        for (const auto &source : batch.sources) {
            work.charge();
            if (source.kind != LineSource::Kind::Partition || !safe_point(source.canonical_a) ||
                !safe_point(source.canonical_b) || source.canonical_a == source.canonical_b || source.producers.empty())
                throw Unavailable{"reference"};
            for (const auto &producer : source.producers) {
                work.charge();
                if (!std::isfinite(producer.first) || !std::isfinite(producer.last) || producer.first < 0. ||
                    producer.last > 1. || producer.first >= producer.last) throw Unavailable{"reference"};
            }
        }
        for (size_t i = 0; i < batch.paths.size(); ++i) {
            work.charge();
            if (batch.source_for_path[i] >= batch.sources.size() || batch.paths[i].size() != 2 ||
                !linear(batch.paths[i], work)) throw Unavailable{"reference_shape"};
        }
        for (size_t i = 0; i < batch.partitions.size(); ++i) {
            work.charge();
            if ((i && !(batch.partitions[i - 1].first < batch.partitions[i].first)) || batch.partitions[i].second.empty())
                throw Unavailable{"reference"};
            for (const auto &line : batch.partitions[i].second)
                if (line.size() != 2 || !linear(line, work)) throw Unavailable{"reference_shape"};
        }
        // The reference is optional, wholly private input. Admit its live
        // borrowed storage plus the one new index vector before moving it;
        // no old-state copy or duplicate reference allocation is necessary.
        Measure growth{work, remaining(remaining(limit, state.bytes), input.bytes)};
        growth.add(batch.sources.size(), sizeof(size_t));
        state.partition_table.reserve(batch.sources.size());
        for (size_t i = 0; i < batch.sources.size(); ++i) { work.charge(); state.partition_table.push_back(i); }
        state.sources = std::move(batch.sources);
        state.reference_paths = std::move(batch.paths);
        state.reference_sources = std::move(batch.source_for_path);
        state.reference_partitions = std::move(batch.partitions);
        state.reference_bound = true;
        admitted(state, additional_scratch_bytes + sizeof(ReferenceLineBatch) + 128, limit, work);
        work.poll();
        return true;
    } catch (const CallbackError &e) { decline(); std::rethrow_exception(e.exception); }
      catch (const Unavailable &e) { m_decline_reason = e.reason ? e.reason : "reference"; }
      catch (const std::bad_alloc &) { m_decline_reason = "allocation"; }
    decline(); return false;
}

bool EmissionLedgerBuilder::bind_batch(const TaggedLineBatch &batch, const std::vector<const ExtrusionPath*> &children,
    BridgeAwareCubicValidation::WorkBudget *budget, const EmissionCancel &cancel, size_t additional_scratch_bytes)
{
    try {
        Work work(budget, cancel);
        if (!m_state) return false;
        const size_t limit = allowance(m_limits);
        Measure input{work, remaining(limit, m_state->bytes)};
        input.add(additional_scratch_bytes); input.batch(batch); input.array(children);
        if (!batch.complete || batch.paths.size() != batch.source_for_path.size()) throw Unavailable{};
        auto *next = m_state.get();
        // Admit a conservative peak before allocation, including old and new
        // outer capacities during reallocation and all possible copied payloads.
        const size_t available = remaining(remaining(limit, next->bytes), input.bytes);
        Measure growth{work, available};
        if (children.size() > max_items - next->records.size() ||
            batch.sources.size() > max_items - next->sources.size()) throw Unavailable{};
        growth.add(next->records.size() + children.size(), sizeof(EmissionRecord));
        growth.add(next->handles.size() + children.size(), sizeof(const ExtrusionPath*));
        growth.add(next->sources.size() + batch.sources.size(), sizeof(LineSource));
        growth.add(2 * batch.sources.size(), sizeof(size_t));
        for (const auto &source : batch.sources) growth.array(source.producers);
        for (const auto *child : children) {
            if (!child || !eligible_path(*child, work)) throw Unavailable{};
            growth.line(child->polyline);
        }
        next->records.reserve(next->records.size() + children.size());
        next->handles.reserve(next->handles.size() + children.size());
        next->sources.reserve(next->sources.size() + batch.sources.size());
        std::vector<size_t> mapping, partition_table;
        mapping.reserve(batch.sources.size()); partition_table.reserve(batch.sources.size());
        for (const auto &source : batch.sources) {
            work.charge();
            if (!safe_point(source.canonical_a) || !safe_point(source.canonical_b) || source.canonical_a == source.canonical_b ||
                (source.kind != LineSource::Kind::Bulk && source.kind != LineSource::Kind::Partition) ||
                (source.kind == LineSource::Kind::Bulk && !source.producers.empty()) ||
                (source.kind == LineSource::Kind::Partition && source.producers.empty())) throw Unavailable{};
            for (const auto &p : source.producers) {
                work.charge();
                if (!std::isfinite(p.first) || !std::isfinite(p.last) || p.first < 0. || p.last > 1. || p.last <= p.first)
                    throw Unavailable{};
            }
            size_t found = 0;
            for (; found < next->sources.size(); ++found) if (same_source(source, next->sources[found], work)) break;
            if (found == next->sources.size()) next->sources.push_back(source);
            mapping.push_back(found);
            if (source.kind == LineSource::Kind::Partition) {
                work.charge(partition_table.size());
                if (std::find(partition_table.begin(), partition_table.end(), found) != partition_table.end()) throw Unavailable{};
                partition_table.push_back(found);
            }
        }
        work.charge(1 + partition_table.size());
        if ((next->bound || next->reference_bound) && next->partition_table != partition_table) throw Unavailable{};
        if (!next->bound && !next->reference_bound) next->partition_table = std::move(partition_table);
        size_t child = 0;
        for (size_t i = 0; i < batch.paths.size(); ++i) {
            work.charge();
            if (batch.source_for_path[i] >= mapping.size()) throw Unavailable{};
            if (!batch.paths[i].is_valid()) continue;
            if (child == children.size() || !children[child] || !eligible_path(*children[child], work) ||
                !same_line(batch.paths[i], children[child]->polyline, work)) throw Unavailable{};
            for (const auto *handle : next->handles) { work.charge(); if (handle == children[child]) throw Unavailable{}; }
            EmissionRecord record;
            record.path = *children[child]; record.source_index = mapping[batch.source_for_path[i]];
            next->records.push_back(std::move(record)); next->handles.push_back(children[child++]);
        }
        if (child != children.size()) throw Unavailable{};
        next->bound = true;
        // mapping/table are simultaneous optional scratch too.
        const size_t scratch = input.bytes + (mapping.capacity() + partition_table.capacity()) * sizeof(size_t);
        admitted(*next, scratch, limit, work);
        work.poll(); return true;
    } catch (const CallbackError &e) { decline(); std::rethrow_exception(e.exception); }
      catch (const Unavailable &e) { m_decline_reason = e.reason ? e.reason : "bind"; }
      catch (const std::bad_alloc &) { m_decline_reason = "allocation"; }
    decline(); return false;
}

bool EmissionLedgerBuilder::applied(const ExtrusionPath &path, const std::vector<EmissionTransition> &transitions,
    BridgeAwareCubicValidation::WorkBudget *budget, const EmissionCancel &cancel)
{
    try {
        Work work(budget, cancel);
        if (!m_state) return false;
        const size_t limit = allowance(m_limits);
        Measure input{work, remaining(limit, m_state->bytes)}; input.transitions(transitions);
        if (transitions.empty() || !eligible_path(path, work)) throw Unavailable{};
        size_t index = 0;
        for (; index < m_state->handles.size(); ++index) { work.charge(); if (m_state->handles[index] == &path) break; }
        if (index == m_state->handles.size() || !same_path_metadata(path, m_state->records[index].path)) throw Unavailable{};
        auto *next = m_state.get();
        auto &record = next->records[index];
        if (record.transitions.size() > next->active.size() ||
            transitions.size() > next->active.size() - record.transitions.size()) throw Unavailable{};
        const Polyline *current = &record.path.polyline;
        for (const auto &transition : transitions) {
            const auto &a = transition.attachment;
            if (a.z != next->z || a.emitting_region != next->emitter || a.width != path.width || a.height != path.height ||
                !same_line(*current, a.original, work)) throw Unavailable{};
            size_t matches = 0;
            for (const auto &active : next->active) matches += same_transition(transition, active, work);
            if (matches != 1) throw Unavailable{};
            current = &a.replacement;
        }
        if (!same_line(*current, path.polyline, work)) throw Unavailable{};
        Measure growth{work, remaining(remaining(limit, next->bytes), input.bytes)};
        growth.line(path.polyline);
        growth.transitions(transitions);
        growth.add(record.transitions.size() + transitions.size(), sizeof(EmissionTransition));
        record.transitions.reserve(record.transitions.size() + transitions.size());
        record.path = path;
        record.transitions.insert(record.transitions.end(), transitions.begin(), transitions.end());
        admitted(*next, input.bytes, limit, work);
        work.poll(); return true;
    } catch (const CallbackError &e) { decline(); std::rethrow_exception(e.exception); }
      catch (const Unavailable &e) { m_decline_reason = e.reason ? e.reason : "attachment"; }
      catch (const std::bad_alloc &) { m_decline_reason = "allocation"; }
    decline(); return false;
}

bool EmissionLedgerBuilder::erased(const ExtrusionPath &path,
    BridgeAwareCubicValidation::WorkBudget *budget, const EmissionCancel &cancel)
{
    try {
        Work work(budget, cancel);
        if (!m_state) return false;
        for (size_t i = 0; i < m_state->handles.size(); ++i) {
            work.charge();
            if (m_state->handles[i] != &path) continue;
            if (!same_path(path, m_state->records[i].path, work)) throw Unavailable{};
            m_state->handles[i] = nullptr; work.poll(); return true;
        }
    } catch (const CallbackError &e) { decline(); std::rethrow_exception(e.exception); }
      catch (const Unavailable &e) { m_decline_reason = e.reason ? e.reason : "erase"; }
    decline(); return false;
}

EmissionLedgerPtr EmissionLedgerBuilder::seal(const std::vector<EmissionPathView> &paths,
    const std::vector<EmissionPlanBinding> &plans, BridgeAwareCubicValidation::WorkBudget *budget,
    const EmissionCancel &cancel)
{
    try {
        Work work(budget, cancel);
        if (!m_state) return {};
        const size_t limit = allowance(m_limits);
        Measure input{work, remaining(limit, m_state->bytes)}; input.array(paths); input.array(plans);
        if (!m_state->bound || !same_bindings(plans, m_state->plans, work)) throw Unavailable{"seal_context"};
        auto *next = m_state.get();
        size_t live = 0;
        for (size_t i = 0; i < next->records.size(); ++i) {
            if (next->handles[i] == nullptr) continue;
            if (live == paths.size() || !paths[live].path || !paths[live].free_ended ||
                paths[live].path != next->handles[i] || !same_path(*paths[live].path, next->records[i].path, work))
                throw Unavailable{"seal_survival"};
            if (live != i) next->records[live] = std::move(next->records[i]);
            ++live;
        }
        if (live != paths.size()) throw Unavailable{"seal_leaf_count"};
        next->records.resize(live);
        std::vector<const ExtrusionPath*>().swap(next->handles);
        // Bound all simultaneous lineage scratch (one chain per start, union,
        // actual indices, sort workspace) before any vector grows.
        const size_t active_count = next->active.size();
        if (active_count > max_items || active_count > 1024) throw Unavailable{"lineage_capacity"};
        const size_t lineage_scratch = (active_count * active_count + 4 * active_count + 16) * sizeof(size_t) * 2;
        admitted(*next, input.bytes + lineage_scratch, limit, work);
        for (auto &record : next->records) {
            lineage(*next, record, work);
            admitted(*next, input.bytes + lineage_scratch, limit, work);
        }
        admitted(*next, input.bytes + lineage_scratch, limit, work);
        classify(*next, work, remaining(limit, input.bytes));
        admitted(*next, input.bytes, limit, work);
        work.poll();
        EmissionLedgerPtr ledger(new EmissionLedger(std::move(m_state)));
        return ledger;
    } catch (const CallbackError &e) { decline(); std::rethrow_exception(e.exception); }
      catch (const Unavailable &e) { m_decline_reason = e.reason ? e.reason : "seal"; }
      catch (const std::bad_alloc &) { m_decline_reason = "allocation"; }
    decline(); return {};
}

EmissionSnapshot::EmissionSnapshot(std::unique_ptr<EmissionSnapshotState> state) : m_state(std::move(state)) {}
EmissionSnapshot::~EmissionSnapshot() = default;
bool EmissionSnapshot::matches(size_t emitter, double z, const EmissionReference &reference,
    const std::vector<EmissionPlanBinding> &plans, const std::vector<EmissionPathView> &paths,
    BridgeAwareCubicValidation::WorkBudget *budget, const EmissionCancel &cancel) const
{
    try {
        Work work(budget, cancel);
        work.charge();
        const auto &s = *m_state;
        if (emitter != s.emitter || z != s.z || reference.tolerance_mm != s.reference.tolerance_mm ||
            reference.classification_bounds.defined != s.reference.classification_bounds.defined ||
            reference.classification_bounds.min != s.reference.classification_bounds.min ||
            reference.classification_bounds.max != s.reference.classification_bounds.max ||
            paths.size() != s.records.size() || !same_bindings(plans, s.plans, work)) return false;
        for (size_t i = 0; i < paths.size(); ++i)
            if (!paths[i].path || !paths[i].free_ended || !same_path(*paths[i].path, s.records[i].path, work)) return false;
        work.poll();
        return true;
    } catch (const CallbackError &e) { std::rethrow_exception(e.exception); }
      catch (const Unavailable &) { return false; }
}
size_t EmissionSnapshot::emitting_region() const noexcept { return m_state->emitter; }
double EmissionSnapshot::print_z() const noexcept { return m_state->z; }
const EmissionReference &EmissionSnapshot::reference() const noexcept { return m_state->reference; }
const std::vector<EmissionPlanBinding> &EmissionSnapshot::plan_bindings() const noexcept { return m_state->plans; }
const std::vector<EmissionSnapshotRecord> &EmissionSnapshot::records() const noexcept { return m_state->records; }
const std::vector<LineSource> &EmissionSnapshot::sources() const noexcept { return m_state->sources; }
const std::vector<EmissionTransition> &EmissionSnapshot::active_attachments() const noexcept { return m_state->active; }
size_t EmissionSnapshot::owned_bytes() const noexcept { return m_state->bytes; }
size_t EmissionSnapshot::resident_bytes() const noexcept { return m_state->bytes; }

EmissionProjection::EmissionProjection(std::unique_ptr<EmissionProjectionState> state) : m_state(std::move(state)) {}
EmissionProjection::~EmissionProjection() = default;
const EmissionSnapshotPtr &EmissionProjection::snapshot() const noexcept { return m_state->snapshot; }
const std::vector<EmissionClassification> &EmissionProjection::records() const noexcept { return m_state->records; }
size_t EmissionProjection::owned_bytes() const noexcept { return m_state->bytes; }
size_t EmissionProjection::resident_bytes() const noexcept { return m_state->bytes + m_state->snapshot->resident_bytes(); }

EmissionDelta::EmissionDelta(std::unique_ptr<EmissionDeltaState> state) : m_state(std::move(state)) {}
EmissionDelta::~EmissionDelta() = default;
const EmissionSnapshotPtr &EmissionDelta::before() const noexcept { return m_state->before; }
const EmissionSnapshotPtr &EmissionDelta::after() const noexcept { return m_state->after; }
const std::vector<EmissionMatch> &EmissionDelta::matches() const noexcept { return m_state->matches; }
const std::vector<size_t> &EmissionDelta::removed() const noexcept { return m_state->removed; }
const std::vector<size_t> &EmissionDelta::added() const noexcept { return m_state->added; }
bool EmissionDelta::order_changed() const noexcept { return m_state->order_changed; }
bool EmissionDelta::classification_context_changed() const noexcept { return m_state->context_changed; }
size_t EmissionDelta::owned_bytes() const noexcept { return m_state->bytes; }
size_t EmissionDelta::resident_bytes() const noexcept
{
    return m_state->bytes + m_state->before->resident_bytes() +
        (m_state->before == m_state->after ? 0 : m_state->after->resident_bytes());
}

EmissionProjectionPtr project_emissions(const EmissionSnapshotPtr &snapshot, ReferenceLineBatch &&batch,
    EmissionLedgerLimits limits, BridgeAwareCubicValidation::WorkBudget *budget,
    const EmissionCancel &cancel, size_t additional_scratch_bytes)
{
    try {
        Work work(budget, cancel);
        work.charge();
        if (!snapshot) throw Unavailable{};
        const auto &raw = *snapshot->m_state;
        const size_t limit = allowance(limits);
        Measure input{work, limit};
        input.add(raw.bytes); input.items = raw.items; input.vertices = raw.vertices;
        input.add(additional_scratch_bytes); measure_reference(input, batch);
        const auto &box = raw.reference.classification_bounds;
        if (!batch.complete || batch.z != raw.z || !batch.bounds.defined || batch.bounds.min != box.min ||
            batch.bounds.max != box.max || batch.sources.size() != raw.partition_table.size() ||
            batch.paths.size() != batch.source_for_path.size()) throw Unavailable{"reference"};
        bool identity = false;
        for (const auto &binding : raw.plans) {
            work.charge();
            identity |= binding.storage_region == raw.emitter && binding.identity == batch.plan_identity;
        }
        if (!identity) throw Unavailable{"reference"};
        for (size_t i = 0; i < batch.sources.size(); ++i)
            if (!same_source(batch.sources[i], raw.sources.at(raw.partition_table[i]), work))
                throw Unavailable{"reference"};
        for (size_t i = 0; i < batch.paths.size(); ++i) {
            work.charge();
            if (batch.source_for_path[i] >= batch.sources.size() || batch.paths[i].size() != 2 ||
                !linear(batch.paths[i], work)) throw Unavailable{"reference_shape"};
        }
        for (size_t i = 0; i < batch.partitions.size(); ++i) {
            work.charge();
            if ((i && !(batch.partitions[i - 1].first < batch.partitions[i].first)) || batch.partitions[i].second.empty())
                throw Unavailable{"reference"};
            for (const auto &line : batch.partitions[i].second)
                if (line.size() != 2 || !linear(line, work)) throw Unavailable{"reference_shape"};
        }
        const size_t active_count = raw.active.size();
        if (active_count > 1024) throw Unavailable{"lineage_capacity"};
        const size_t lineage_scratch = (active_count * active_count + 4 * active_count + 16) * sizeof(size_t) * 2;
        // Admit the entire initial simultaneous peak before allocating. Later
        // derived-vector growth is checked by the same eager classifier.
        input.add(sizeof(EmissionProjectionState) + sizeof(EmissionProjection) + 128 + sizeof(ProjectionView));
        input.add(raw.records.size(), sizeof(EmissionClassification) + sizeof(ProjectedRecord));
        input.add(lineage_scratch);
        auto state = std::make_unique<EmissionProjectionState>();
        state->snapshot = snapshot;
        state->records.resize(raw.records.size());
        for (size_t &source : batch.source_for_path) { work.charge(); source = raw.partition_table[source]; }
        ProjectionView view(raw, *state, batch);
        view.records.reserve(raw.records.size());
        for (size_t i = 0; i < raw.records.size(); ++i) {
            work.charge(); view.records.emplace_back(raw.records[i], state->records[i]);
        }
        const size_t available = remaining(limit, additional_scratch_bytes);
        view.bytes = measure(view, work, remaining(available, lineage_scratch));
        for (auto &record : view.records) {
            lineage(view, record, work);
            view.bytes = measure(view, work, remaining(available, lineage_scratch));
        }
        classify(view, work, available);
        view.bytes = measure(view, work, available);
        Measure own{work, remaining(limit, raw.bytes)};
        measure_projection(own, *state);
        state->bytes = own.bytes;
        work.poll();
        return EmissionProjectionPtr(new EmissionProjection(std::move(state)));
    } catch (const CallbackError &e) { std::rethrow_exception(e.exception); }
      catch (const Unavailable &) { return {}; }
      catch (const std::bad_alloc &) { return {}; }
}

namespace {
bool same_context(const EmissionSnapshotState &a, const EmissionSnapshotState &b, Work &work)
{
    work.charge();
    if (a.emitter != b.emitter || a.z != b.z ||
        a.reference.tolerance_mm != b.reference.tolerance_mm ||
        a.reference.classification_bounds.defined != b.reference.classification_bounds.defined ||
        a.reference.classification_bounds.min != b.reference.classification_bounds.min ||
        a.reference.classification_bounds.max != b.reference.classification_bounds.max ||
        !same_bindings(a.plans, b.plans, work) || a.partition_table.size() != b.partition_table.size() ||
        a.active.size() != b.active.size()) return false;
    for (size_t i = 0; i < a.partition_table.size(); ++i)
        if (!same_source(a.sources.at(a.partition_table[i]), b.sources.at(b.partition_table[i]), work)) return false;
    for (size_t i = 0; i < a.active.size(); ++i)
        if (!same_transition(a.active[i], b.active[i], work)) return false;
    return true;
}
bool same_provenance(const EmissionSnapshotRecord &a, const EmissionSnapshotState &before,
                     const EmissionSnapshotRecord &b, const EmissionSnapshotState &after, Work &work)
{
    work.charge();
    if (!same_source(before.sources.at(a.source_index), after.sources.at(b.source_index), work) ||
        a.transitions.size() != b.transitions.size()) return false;
    for (size_t i = 0; i < a.transitions.size(); ++i)
        if (!same_transition(a.transitions[i], b.transitions[i], work)) return false;
    return true;
}
struct PathFingerprint { size_t hash = 0, index = 0; };
size_t fingerprint(const ExtrusionPath &path, Work &work)
{
    size_t value = 0;
    const auto mix = [&](size_t part) { value ^= part + size_t(0x9e3779b9) + (value << 6) + (value >> 2); };
    work.charge(5);
    mix(std::hash<float>{}(path.width)); mix(std::hash<float>{}(path.height));
    mix(std::hash<double>{}(path.mm3_per_mm)); mix(size_t(path.role())); mix(path.polyline.size());
    for (const Point &point : path.polyline.points) {
        work.charge(); mix(std::hash<coord_t>{}(point.x())); mix(std::hash<coord_t>{}(point.y()));
    }
    // Remaining metadata is deliberately resolved by exact same_path below.
    // A hash collision can only spend work, never establish correspondence.
    return value;
}
void measure_delta(Measure &m, const EmissionDeltaState &state)
{
    m.add(sizeof(EmissionDeltaState) + sizeof(EmissionDelta) + 128);
    m.array(state.matches); m.array(state.removed); m.array(state.added);
}
}

EmissionDeltaPtr compare_emissions(const EmissionSnapshotPtr &before, const EmissionSnapshotPtr &after,
    EmissionLedgerLimits limits, BridgeAwareCubicValidation::WorkBudget *budget, const EmissionCancel &cancel)
{
    try {
        Work work(budget, cancel);
        work.charge();
        if (!before || !after || before->emitting_region() != after->emitting_region()) throw Unavailable{};
        const auto &a = *before->m_state, &b = *after->m_state;
        const size_t limit = allowance(limits);
        Measure input{work, limit};
        input.add(a.bytes); input.items = a.items; input.vertices = a.vertices;
        if (before != after) {
            input.add(b.bytes);
            if (b.items > max_items - input.items || b.vertices > max_vertices - input.vertices) throw Unavailable{};
            input.items += b.items; input.vertices += b.vertices;
        }
        const size_t raw_bytes = input.bytes;
        input.add(sizeof(EmissionDeltaState) + sizeof(EmissionDelta) + 128);
        input.add(a.records.size(), sizeof(PathFingerprint) + sizeof(unsigned char));
        input.add(a.records.size() + b.records.size(), sizeof(size_t));
        input.add(std::min(a.records.size(), b.records.size()), sizeof(EmissionMatch));
        auto state = std::make_unique<EmissionDeltaState>();
        state->before = before; state->after = after;
        state->matches.reserve(std::min(a.records.size(), b.records.size()));
        state->removed.reserve(a.records.size()); state->added.reserve(b.records.size());
        std::vector<PathFingerprint> index;
        std::vector<unsigned char> used(a.records.size(), 0);
        index.reserve(a.records.size());
        for (size_t i = 0; i < a.records.size(); ++i) index.push_back({fingerprint(a.records[i].path, work), i});
        const auto less = [&](const PathFingerprint &x, const PathFingerprint &y) {
            work.charge(); return x.hash < y.hash || (x.hash == y.hash && x.index < y.index);
        };
        std::sort(index.begin(), index.end(), less);
        // Allocators may reserve more than requested; verify actual capacities
        // while all buffers are live, before interpreting any correspondence.
        Measure live{work, limit};
        live.add(raw_bytes); live.items = input.items; live.vertices = input.vertices;
        measure_delta(live, *state); live.array(index); live.array(used);
        state->context_changed = !same_context(a, b, work);
        bool have_previous = false;
        size_t previous = 0;
        for (size_t i = 0; i < b.records.size(); ++i) {
            const size_t hash = fingerprint(b.records[i].path, work);
            size_t found = a.records.size();
            if (a.z == b.z) {
                auto entry = std::lower_bound(index.begin(), index.end(), PathFingerprint{hash, 0}, less);
                for (; entry != index.end() && entry->hash == hash; ++entry) {
                    work.charge();
                    if (!used[entry->index] && same_path(a.records[entry->index].path, b.records[i].path, work)) {
                        found = entry->index; break;
                    }
                }
            }
            if (found == a.records.size()) state->added.push_back(i);
            else {
                used[found] = 1;
                state->order_changed |= have_previous && found < previous;
                previous = found; have_previous = true;
                state->matches.push_back({found, i, !same_provenance(a.records[found], a, b.records[i], b, work)});
            }
        }
        for (size_t i = 0; i < used.size(); ++i) { work.charge(); if (!used[i]) state->removed.push_back(i); }
        Measure final_live{work, limit};
        final_live.add(raw_bytes); final_live.items = input.items; final_live.vertices = input.vertices;
        measure_delta(final_live, *state); final_live.array(index); final_live.array(used);
        Measure own{work, remaining(limit, raw_bytes)};
        measure_delta(own, *state);
        state->bytes = own.bytes;
        work.poll();
        return EmissionDeltaPtr(new EmissionDelta(std::move(state)));
    } catch (const CallbackError &e) { std::rethrow_exception(e.exception); }
      catch (const Unavailable &) { return {}; }
      catch (const std::bad_alloc &) { return {}; }
}

EmissionSnapshotPtr EmissionLedgerBuilder::seal_snapshot(const std::vector<EmissionPathView> &paths,
    const std::vector<EmissionPlanBinding> &plans, BridgeAwareCubicValidation::WorkBudget *budget,
    const EmissionCancel &cancel)
{
    try {
        Work work(budget, cancel);
        if (!m_state) return {};
        const size_t limit = allowance(m_limits);
        Measure input{work, remaining(limit, m_state->bytes)};
        input.array(paths); input.array(plans);
        if (!m_state->bound || !same_bindings(plans, m_state->plans, work)) throw Unavailable{"seal_context"};
        auto &old = *m_state;
        size_t live = 0;
        for (size_t i = 0; i < old.records.size(); ++i) {
            work.charge();
            if (old.handles[i] == nullptr) continue;
            if (live == paths.size() || !paths[live].path || !paths[live].free_ended ||
                paths[live].path != old.handles[i] || !same_path(*paths[live].path, old.records[i].path, work))
                throw Unavailable{"seal_survival"};
            if (live != i) old.records[live] = std::move(old.records[i]);
            ++live;
        }
        if (live != paths.size()) throw Unavailable{"seal_leaf_count"};
        old.records.resize(live);
        // Only outer record storage is new; paths/transitions and common tables
        // move once. Admit old + new outer buffers before that private transfer.
        Measure growth{work, remaining(remaining(limit, old.bytes), input.bytes)};
        growth.add(sizeof(EmissionSnapshotState) + sizeof(EmissionSnapshot) + 128);
        growth.add(live, sizeof(EmissionSnapshotRecord));
        auto state = std::make_unique<EmissionSnapshotState>();
        state->records.reserve(live);
        if (state->records.capacity() > remaining(growth.limit,
                sizeof(EmissionSnapshotState) + sizeof(EmissionSnapshot) + 128) / sizeof(EmissionSnapshotRecord))
            throw Unavailable{"work_or_capacity"};
        state->emitter = old.emitter; state->z = old.z; state->reference = old.reference;
        state->plans = std::move(old.plans); state->active = std::move(old.active);
        state->sources = std::move(old.sources); state->partition_table = std::move(old.partition_table);
        for (auto &record : old.records) {
            work.charge();
            state->records.push_back({std::move(record.path), record.source_index, std::move(record.transitions)});
        }
        // Discard event handles and any eager reference geometry before final
        // publication. No classification or lineage resolver is called here.
        m_state.reset();
        state->bytes = measure_snapshot(*state, work, remaining(limit, input.bytes));
        work.poll();
        return EmissionSnapshotPtr(new EmissionSnapshot(std::move(state)));
    } catch (const CallbackError &e) { decline(); std::rethrow_exception(e.exception); }
      catch (const Unavailable &e) { m_decline_reason = e.reason ? e.reason : "seal"; }
      catch (const std::bad_alloc &) { m_decline_reason = "allocation"; }
    decline(); return {};
}

}} // namespace Slic3r::BridgeAwareCubic
