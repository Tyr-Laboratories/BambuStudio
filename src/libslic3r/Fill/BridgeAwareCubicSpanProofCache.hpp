#ifndef slic3r_BridgeAwareCubicSpanProofCache_hpp_
#define slic3r_BridgeAwareCubicSpanProofCache_hpp_

#include "BridgeAwareCubicContact.hpp"
#include "BridgeAwareCubicSerialization.hpp"
#include "BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <functional>
#include <iterator>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <new>

namespace Slic3r { namespace BridgeAwareCubic {

// Optional facts about complete passed span-only checks during coarsening.
// Model-cell, final and repair proofs must never consult this cache. Every
// operation borrows Query inputs only for its duration; retained facts own
// their exact key and the Contact query signature, without plan/path pointers.
class SpanProofCache {
public:
    enum class Kind { Ineligible, Construction, BridgeFollowup };
    using Cancel = std::function<void()>;
    struct Query {
        Kind kind;
        const SerializedXY &phase;
        double gap, selection_width;
        const ExPolygons &demand;
        const BridgeAwareCubicContact::PreparedLower &lower;
        const ExtrusionPath &upper;
        double upper_z;
        BridgeAwareCubicContact::QueryCache *contact_cache = nullptr;
    };
    struct Statistics {
        size_t hits = 0, misses = 0, publications = 0, evictions = 0;
        size_t entries = 0, units = 0;
    };

    explicit SpanProofCache(size_t max_units = 200000, size_t max_entries = 20000)
        : m_max_units(std::min(max_units, size_t(200000))),
          m_max_entries(std::min(max_entries, size_t(20000))) {}
    SpanProofCache(const SpanProofCache &) = delete;
    SpanProofCache &operator=(const SpanProofCache &) = delete;

    bool matches(const Query &query, BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                 const Cancel &cancel = {})
    {
        Work work(budget, cancel);
        try {
            work.poll();
            const Description description = describe(query, work);
            auto bucket = find(description.hash, work);
            if (bucket != m_index.end()) {
                for (size_t i = 0; i < bucket->second.size; ++i) {
                    const Entry &entry = *bucket->second.entries[i];
                    if (same_key(entry, query, description, work) &&
                        BridgeAwareCubicContact::matches_query_signature(*entry.signature,
                            query.lower, query.upper, query.upper_z, 0., work.budget(),
                            [&] { work.poll(); }, query.contact_cache)) {
                        ++m_statistics.hits;
                        return true;
                    }
                }
            }
        } catch (const Canceled &canceled) {
            std::rethrow_exception(canceled.exception);
        } catch (const Unavailable &) {
        } catch (const std::bad_alloc &) {
        }
        ++m_statistics.misses;
        return false;
    }
    // The caller must supply the ACTUAL complete span-only validation result
    // for precisely this Query, with zero contact and validation tolerances.
    // Result has no input provenance: its flags alone do not turn a default,
    // generic or independent cell result into evidence for this path.
    // A later failed path in the same trial does not invalidate this local fact.
    // Failed/canceled admission leaves all stored facts intact. If no room is
    // available for a bounded staged candidate, decline without evicting.
    bool publish(const Query &query, const BridgeAwareCubicValidation::Result &checked,
                 BridgeAwareCubicValidation::WorkBudget *budget = nullptr, const Cancel &cancel = {})
    {
        Work work(budget, cancel);
        try {
            work.poll();
            const Description description = describe(query, work);
            if (!complete_span(checked, work) || m_max_entries == 0 ||
                description.key_units >= m_max_units) return false;

            // Reserve the full key and a bounded signature before allocating
            // either. Admission includes the staged entry as well as all old
            // facts; no allocation failure can publish a partially copied key.
            const size_t available = m_max_units - m_statistics.units;
            if (description.key_units >= available) return false;
            const size_t signature_capacity = std::min(size_t(8192), available - description.key_units);
            auto signature = BridgeAwareCubicContact::capture_query_signature(query.lower,
                query.upper, query.upper_z, 0., work.budget(), signature_capacity,
                [&] { work.poll(); }, query.contact_cache);
            if (!signature) return false;
            const size_t signature_units = BridgeAwareCubicContact::query_signature_units(*signature);
            if (signature_units > signature_capacity) return false;

            Entry staged(query, description);
            staged.signature = std::move(signature);
            staged.units = description.key_units + signature_units;
            copy_key(staged, query, work);
            auto bucket = find(description.hash, work);
            Iterator victim = m_entries.end();
            if (bucket != m_index.end() && bucket->second.size == 2)
                victim = bucket->second.entries[0];
            else if (m_entries.size() >= m_max_entries)
                victim = m_entries.begin();

            // Allocate both nodes privately and charge every commit operation
            // before eviction. Transfer node ownership without allocation;
            // cancellation/work/allocation failure therefore preserves all
            // existing facts, including the useful pre-trial version.
            work.charge(metadata_units);
            Entries staged_entries;
            staged_entries.push_back(std::move(staged));
            Index staged_index;
            staged_index.emplace(description.hash, Bucket{});
            if (victim != m_entries.end()) {
                work.charge(victim->units); // Owned-array/signature destruction.
                charge_lookup(work);
            }
            // Keep ordinary staging room after successful publication so a
            // full cache can still accept later versions. Large custom keys
            // may still decline; never evict on a failed attempt to stage one.
            // Tiny custom capacities may retain a single fact above this
            // target, provided retained + staged admission itself fitted.
            const size_t retained_target = m_max_units - std::min(size_t(9216), m_max_units / 8);
            size_t retained_units = m_statistics.units + staged_entries.front().units;
            size_t retained_entries = m_entries.size() + 1, prefix_evictions = 0;
            if (victim != m_entries.end()) { retained_units -= victim->units; --retained_entries; }
            for (auto entry = m_entries.begin(); retained_units > retained_target &&
                 retained_entries > 1 && entry != m_entries.end(); ++entry) {
                if (entry == victim) continue;
                work.charge(entry->units);
                charge_lookup(work);
                retained_units -= entry->units;
                --retained_entries;
                ++prefix_evictions;
            }
            charge_lookup(work);
            charge_lookup(work);
            work.poll();
            if (victim != m_entries.end()) evict(victim);
            for (size_t i = 0; i < prefix_evictions; ++i) evict(m_entries.begin());
            bucket = m_index.find(description.hash);
            if (bucket == m_index.end())
                bucket = m_index.insert(staged_index.extract(staged_index.begin())).position;
            m_entries.splice(m_entries.end(), staged_entries);
            auto added = std::prev(m_entries.end());
            bucket->second.entries[bucket->second.size++] = added;
            m_statistics.units += added->units;
            m_statistics.entries = m_entries.size();
            ++m_statistics.publications;
            return true;
        } catch (const Canceled &canceled) {
            std::rethrow_exception(canceled.exception);
        } catch (const Unavailable &) {
        } catch (const std::bad_alloc &) {
        }
        return false;
    }
    Statistics statistics() const { return m_statistics; }

private:
    static constexpr size_t max_vertices = 200000, max_items = 20000;
    // Conservative metadata allowance for the key, signature handle, list
    // node and a whole two-version map node (even when two facts share it).
    static constexpr size_t metadata_units = 64;
    static constexpr coord_t coordinate_limit = std::numeric_limits<coord_t>::max() / 4;
    struct Unavailable {};
    struct Canceled { std::exception_ptr exception; };

    class Work {
    public:
        Work(BridgeAwareCubicValidation::WorkBudget *budget, const Cancel &cancel)
            : m_budget(budget == nullptr ? &m_local : budget), m_cancel(cancel) {}
        void poll() const
        {
            if (m_cancel) {
                try { m_cancel(); }
                catch (...) { throw Canceled{std::current_exception()}; }
            }
        }
        void charge(size_t count = 1)
        {
            if (count > m_budget->remaining_work) throw Unavailable{};
            m_budget->remaining_work -= count;
            m_budget->charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += count;
            m_since_poll += count;
            if (m_since_poll >= 256) { m_since_poll = 0; poll(); }
        }
        BridgeAwareCubicValidation::WorkBudget *budget() { return m_budget; }
    private:
        BridgeAwareCubicValidation::WorkBudget m_local;
        BridgeAwareCubicValidation::WorkBudget *m_budget;
        const Cancel &m_cancel;
        size_t m_since_poll = 0;
    };

    struct Description {
        size_t hash = 0, points = 0, shapes = 1, items = 0, key_units = 0;
    };
    struct Entry {
        Kind kind;
        SerializedXY phase;
        double gap, selection_width;
        size_t hash, point_count, shape_count, units = 0;
        // Exact-sized arrays avoid unaccounted nested-vector spare capacity.
        // Shapes store polygon count, then contour size / hole count / hole
        // sizes per polygon. All original ring and point order is preserved.
        std::unique_ptr<Point[]> points;
        std::unique_ptr<size_t[]> shapes;
        BridgeAwareCubicContact::QuerySignaturePtr signature;
        Entry(const Query &query, const Description &description)
            : kind(query.kind), phase(query.phase), gap(query.gap), selection_width(query.selection_width),
              hash(description.hash), point_count(description.points), shape_count(description.shapes) {}
    };
    using Entries = std::list<Entry>;
    using Iterator = Entries::iterator;
    struct Bucket { std::array<Iterator, 2> entries; size_t size = 0; };
    using Index = std::map<size_t, Bucket>;
    size_t m_max_units, m_max_entries;
    Entries m_entries;
    Index m_index;
    Statistics m_statistics;

    template<class T> static void hash_value(size_t &hash, const T &value, Work &work)
    {
        work.charge();
        // Hashes are lookup hints only. std::hash gives equal numeric zeros
        // the same hash; every positive result still compares exact values.
        hash ^= std::hash<T>{}(value) + size_t(0x9e3779b9) + (hash << 6) + (hash >> 2);
    }
    static bool safe(const Point &point)
    {
        return point.x() >= -coordinate_limit && point.x() <= coordinate_limit &&
               point.y() >= -coordinate_limit && point.y() <= coordinate_limit;
    }
    static void describe_ring(const Polygon &ring, bool hole, Description &description, Work &work)
    {
        work.charge();
        if (ring.size() < 3 || ring.size() > max_vertices - description.points ||
            ++description.items > max_items) throw Unavailable{};
        description.points += ring.size();
        hash_value(description.hash, ring.size(), work);
        for (const Point &point : ring.points) {
            if (!safe(point)) throw Unavailable{};
            hash_value(description.hash, point.x(), work);
            hash_value(description.hash, point.y(), work);
        }
        work.charge(ring.size()); // The orientation calculation visits every point.
        const double area = ring.area();
        if (!std::isfinite(area) || (hole ? area >= 0. : area <= 0.)) throw Unavailable{};
    }
    static Description describe(const Query &query, Work &work)
    {
        work.charge();
        if ((query.kind != Kind::Construction && query.kind != Kind::BridgeFollowup) ||
            !std::isfinite(query.gap) || query.gap <= 0. ||
            !std::isfinite(query.selection_width) || query.selection_width <= 0. ||
            query.demand.size() > max_items) throw Unavailable{};
        Description description;
        hash_value(description.hash, unsigned(query.kind), work);
        hash_value(description.hash, query.gap, work);
        hash_value(description.hash, query.selection_width, work);
        for (unsigned axis = 0; axis < 2; ++axis) {
            for (double value : {query.phase.shift[axis], query.phase.nozzle_offset[axis], query.phase.plate_origin[axis]}) {
                if (!std::isfinite(value)) throw Unavailable{};
                hash_value(description.hash, value, work);
            }
        }
        hash_value(description.hash, query.demand.size(), work);
        for (const ExPolygon &polygon : query.demand) {
            if (polygon.holes.size() > max_items - description.items) throw Unavailable{};
            description.shapes += 2 + polygon.holes.size();
            describe_ring(polygon.contour, false, description, work);
            hash_value(description.hash, polygon.holes.size(), work);
            for (const Polygon &hole : polygon.holes) describe_ring(hole, true, description, work);
        }
        // Upper geometry is included in the lookup hint but owned and checked
        // by Contact's signature. No prepared pointer or lower version is a key.
        if (!std::isfinite(query.upper_z) || !std::isfinite(query.upper.width) ||
            !std::isfinite(query.upper.height) || !std::isfinite(query.upper.mm3_per_mm) ||
            query.upper.polyline.size() > max_vertices || !query.upper.polyline.fitting_result.empty())
            throw Unavailable{};
        hash_value(description.hash, query.upper_z, work);
        hash_value(description.hash, query.upper.width, work);
        hash_value(description.hash, query.upper.height, work);
        hash_value(description.hash, query.upper.mm3_per_mm, work);
        hash_value(description.hash, unsigned(query.upper.role()), work);
        hash_value(description.hash, query.upper.polyline.size(), work);
        for (const Point &point : query.upper.polyline.points) {
            if (!safe(point)) throw Unavailable{};
            hash_value(description.hash, point.x(), work);
            hash_value(description.hash, point.y(), work);
        }
        description.key_units = metadata_units + description.points + description.shapes;
        return description;
    }
    static bool complete_span(const BridgeAwareCubicValidation::Result &checked, Work &work)
    {
        work.charge();
        if (!checked.input_valid || !checked.completed || checked.resource_limited ||
            !checked.failure_reason.empty() || !checked.void_cells.empty() || checked.max_void_diameter_mm != 0. ||
            !std::isfinite(checked.max_unsupported_mm) || checked.max_unsupported_mm < 0. ||
            checked.unsupported_runs.size() > max_vertices) return false;
        for (const auto &run : checked.unsupported_runs) {
            work.charge();
            if (run.kind != BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge || run.exceeds_gap ||
                run.path_index != 0 || !std::isfinite(run.begin_mm) || !std::isfinite(run.end_mm) ||
                !std::isfinite(run.length_mm) || !std::isfinite(run.uncertainty_mm) ||
                run.begin_mm < 0. || run.end_mm < run.begin_mm || run.length_mm < 0. || run.uncertainty_mm < 0.)
                return false;
        }
        return true;
    }
    static void copy_key(Entry &entry, const Query &query, Work &work)
    {
        work.charge(entry.point_count + entry.shape_count); // Array initialization.
        entry.points.reset(new Point[entry.point_count]);
        entry.shapes.reset(new size_t[entry.shape_count]);
        size_t point_index = 0, shape_index = 0;
        const auto copy_ring = [&](const Polygon &ring) {
            work.charge();
            entry.shapes[shape_index++] = ring.size();
            for (const Point &point : ring.points) { work.charge(); entry.points[point_index++] = point; }
        };
        entry.shapes[shape_index++] = query.demand.size();
        for (const ExPolygon &polygon : query.demand) {
            copy_ring(polygon.contour);
            work.charge();
            entry.shapes[shape_index++] = polygon.holes.size();
            for (const Polygon &hole : polygon.holes) copy_ring(hole);
        }
    }
    static bool same_key(const Entry &entry, const Query &query, const Description &description, Work &work)
    {
        work.charge();
        if (entry.kind != query.kind || entry.gap != query.gap || entry.selection_width != query.selection_width ||
            entry.point_count != description.points || entry.shape_count != description.shapes) return false;
        for (unsigned axis = 0; axis < 2; ++axis) {
            work.charge();
            if (entry.phase.shift[axis] != query.phase.shift[axis] ||
                entry.phase.nozzle_offset[axis] != query.phase.nozzle_offset[axis] ||
                entry.phase.plate_origin[axis] != query.phase.plate_origin[axis]) return false;
        }
        size_t point_index = 0, shape_index = 0;
        const auto same_ring = [&](const Polygon &ring) {
            work.charge();
            if (entry.shapes[shape_index++] != ring.size()) return false;
            for (const Point &point : ring.points) {
                work.charge();
                if (entry.points[point_index++] != point) return false;
            }
            return true;
        };
        if (entry.shapes[shape_index++] != query.demand.size()) return false;
        for (const ExPolygon &polygon : query.demand) {
            if (!same_ring(polygon.contour)) return false;
            work.charge();
            if (entry.shapes[shape_index++] != polygon.holes.size()) return false;
            for (const Polygon &hole : polygon.holes) if (!same_ring(hole)) return false;
        }
        return true;
    }
    void charge_lookup(Work &work) const
    {
        work.charge();
        // Red-black tree lookup/insertion requires at most twice this depth.
        for (size_t size = m_index.size(); size > 0; size >>= 1) work.charge(2);
    }
    Index::iterator find(size_t hash, Work &work)
    {
        charge_lookup(work);
        return m_index.find(hash);
    }
    void evict(Iterator entry)
    {
        // Work and all potentially throwing operations were completed before
        // this commit. The integral comparator and owned destructors cannot throw.
        auto bucket = m_index.find(entry->hash);
        for (size_t i = 0; i < bucket->second.size; ++i) {
            if (bucket->second.entries[i] == entry) {
                if (i + 1 < bucket->second.size) bucket->second.entries[i] = bucket->second.entries[i + 1];
                --bucket->second.size;
                break;
            }
        }
        if (bucket->second.size == 0) m_index.erase(bucket);
        m_statistics.units -= entry->units;
        m_entries.erase(entry);
        m_statistics.entries = m_entries.size();
        ++m_statistics.evictions;
    }
};

}} // namespace Slic3r::BridgeAwareCubic

#endif
