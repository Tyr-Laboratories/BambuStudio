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
#include <vector>

namespace Slic3r { namespace BridgeAwareCubic {

// Optional facts about complete passed span-only checks during coarsening.
// Model-cell, final and repair proofs must never consult this cache. Every
// operation borrows Query inputs only for its duration; retained facts own
// their exact key and the Contact query signature, without plan/path pointers.
class SpanProofCache {
public:
    enum class Kind { Ineligible, Construction, BridgeFollowup };
    using Cancel = std::function<void()>;
    // Borrowed by one lookup/publication. The epoch must already have been
    // admitted below. Prepared ownership is checked against Query::lower;
    // neither this context nor an emission delta can relabel an old fact.
    struct EpochContext {
        BridgeAwareCubicContact::PreparedLowerPtr prepared;
        BridgeAwareCubicContact::CanonicalLowerEpochPtr current;
        BridgeAwareCubicContact::CanonicalLowerDeltaPtr transition;
        BridgeAwareCubicContact::EpochLimits limits;
    };
    struct Query {
        Kind kind;
        const SerializedXY &phase;
        double gap, selection_width;
        const ExPolygons &demand;
        const BridgeAwareCubicContact::PreparedLower &lower;
        const ExtrusionPath &upper;
        double upper_z;
        BridgeAwareCubicContact::QueryCache *contact_cache = nullptr;
        const EpochContext *epochs = nullptr;
        // Hidden differential audit only: every epoch hit must also pass the
        // original fresh raw-signature query with this same remaining budget.
        // False reference comparison throws (mismatch OR unavailable reference);
        // it cannot turn an incomplete audit into a silently accepted hit.
        bool audit_epoch_hits = false;
    };
    struct Statistics {
        size_t hits = 0, misses = 0, publications = 0, evictions = 0;
        size_t entries = 0, units = 0;
        size_t epoch_hits = 0, advances = 0, raw_checks = 0;
        size_t epoch_audits = 0, epoch_audit_mismatches = 0;
        size_t admitted_epochs = 0, epoch_resident_bytes = 0;
        // Wrapper/control storage only; each entry's shared raw payload is
        // counted once in units, even when two bound handles reference it.
        size_t bound_handles = 0, bound_units = 0;
    };

    explicit SpanProofCache(size_t max_units = 200000, size_t max_entries = 20000)
        : m_max_units(std::min(max_units, size_t(200000))),
          m_max_entries(std::min(max_entries, size_t(20000))) {}
    SpanProofCache(const SpanProofCache &) = delete;
    SpanProofCache &operator=(const SpanProofCache &) = delete;

    // Atomically retain distinct epochs from the caller's EXISTING metadata
    // allowance. Count the union of previously admitted and supplied epochs
    // once; limits.other_retained_bytes excludes that union but includes all
    // other live layer/epoch/delta storage. Registry metadata and all staged
    // binding handles remain under the existing fact-unit allowance. Admission
    // never grants support, and publication may retain at most two epoch
    // bindings for one raw signature (last observed and active transition).
    bool admit_epochs(const std::vector<BridgeAwareCubicContact::CanonicalLowerEpochPtr> &epochs,
                      BridgeAwareCubicContact::EpochLimits limits,
                      BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                      const Cancel &cancel = {})
    {
        Work work(budget, cancel);
        try {
            work.poll();
            work.charge();
            if (epochs.size() > max_items || m_max_entries == 0 ||
                limits.other_retained_bytes > limits.capacity_bytes ||
                m_statistics.epoch_resident_bytes > limits.capacity_bytes - limits.other_retained_bytes)
                return false;
            size_t retained_bytes = m_statistics.epoch_resident_bytes;
            size_t staged_units = 0;
            Epochs staged;
            for (const auto &epoch : epochs) {
                work.charge();
                if (!epoch) return false;
                if (contains_epoch(m_epochs, epoch, work) || contains_epoch(staged, epoch, work)) continue;
                const size_t bytes = BridgeAwareCubicContact::lower_epoch_resident_bytes(*epoch);
                if (bytes == 0 || bytes > limits.capacity_bytes - limits.other_retained_bytes - retained_bytes ||
                    m_epochs.size() + staged.size() >= m_max_entries ||
                    epoch_registry_units > m_max_units - m_statistics.units - staged_units)
                    return false;
                // Metadata is pre-admitted before allocating a node; epoch
                // arrays are shared and counted once in the byte-pool union.
                work.charge(epoch_registry_units);
                staged.push_back({epoch, bytes});
                staged_units += epoch_registry_units;
                retained_bytes += bytes;
            }
            work.poll();
            m_epochs.splice(m_epochs.end(), staged);
            m_statistics.units += staged_units;
            m_statistics.admitted_epochs = m_epochs.size();
            m_statistics.epoch_resident_bytes = retained_bytes;
            return true;
        } catch (const Canceled &canceled) {
            std::rethrow_exception(canceled.exception);
        } catch (const Unavailable &) {
        } catch (const std::bad_alloc &) {
        }
        return false;
    }

    // Precharge a nonallocating scan, then atomically remove these registry
    // references and their optional bindings. Exact raw signatures/policy keys
    // survive for ordinary fresh-discovery fallback. Failure/cancellation
    // changes nothing: the caller MUST keep old bytes accounted until this
    // succeeds (or destroys the complete optional cache at a safe boundary).
    bool retire_epochs(const std::vector<BridgeAwareCubicContact::CanonicalLowerEpochPtr> &epochs,
                       BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                       const Cancel &cancel = {})
    {
        Work work(budget, cancel);
        try {
            work.poll();
            work.charge();
            if (epochs.size() > max_items) return false;
            for (const auto &epoch : epochs) { work.charge(); if (!epoch) return false; }
            // Charge both traversals before changing any handles. The second
            // traversal performs only fixed comparisons and noexcept releases;
            // no cancellation, allocation or work failure can interrupt it.
            for (const auto &entry : m_entries) {
                work.charge(2);
                for (const auto &bound : entry.bindings) {
                    work.charge(2);
                    if (bound && requested_epoch(BridgeAwareCubicContact::query_signature_epoch(*bound), epochs, &work))
                        work.charge(4); // Wrapper releases; entry still owns the complete raw payload.
                }
            }
            for (const auto &admitted : m_epochs) {
                work.charge(2);
                if (requested_epoch(admitted.epoch, epochs, &work)) work.charge(epoch_registry_units);
            }
            work.poll();
            for (auto &entry : m_entries) {
                const size_t old_contact_units = contact_units(entry);
                for (auto &bound : entry.bindings) {
                    if (bound && requested_epoch(BridgeAwareCubicContact::query_signature_epoch(*bound), epochs)) {
                        const size_t units = BridgeAwareCubicContact::epoch_query_signature_units(*bound) - entry.raw_units;
                        entry.binding_units -= units;
                        m_statistics.bound_units -= units;
                        --m_statistics.bound_handles;
                        bound.reset();
                    }
                }
                const size_t released = old_contact_units - contact_units(entry);
                entry.units -= released;
                m_statistics.units -= released;
            }
            for (auto it = m_epochs.begin(); it != m_epochs.end();) {
                if (requested_epoch(it->epoch, epochs)) {
                    m_statistics.epoch_resident_bytes -= it->bytes;
                    m_statistics.units -= epoch_registry_units;
                    it = m_epochs.erase(it);
                } else ++it;
            }
            m_statistics.admitted_epochs = m_epochs.size();
            return true;
        } catch (const Canceled &canceled) {
            std::rethrow_exception(canceled.exception);
        } catch (const Unavailable &) {
        }
        return false;
    }

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
                    Entry &entry = *bucket->second.entries[i];
                    if (!same_key(entry, query, description, work)) continue;
                    if (matches_epoch(entry, query, work)) {
                        ++m_statistics.epoch_hits;
                        ++m_statistics.hits;
                        return true;
                    }
                    ++m_statistics.raw_checks;
                    if (BridgeAwareCubicContact::matches_query_signature(*entry.signature,
                            query.lower, query.upper, query.upper_z, 0., work.budget(),
                            [&] { work.poll(); }, query.contact_cache)) {
                        // After retirement an exact raw fact can be bound to
                        // the new admitted source, but only by fresh capture.
                        // The resulting raw identity replaces the old one in
                        // the same atomic operation; no mismatched pair lives.
                        bind_after_raw_match(entry, query, work);
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
            BridgeAwareCubicContact::EpochQuerySignaturePtr bound;
            BridgeAwareCubicContact::QuerySignaturePtr signature;
            if (valid_epoch_context(query, work)) {
                const auto &epochs = *query.epochs;
                bound = BridgeAwareCubicContact::capture_epoch_query_signature(epochs.current,
                    epochs.prepared, query.phase, query.upper, query.upper_z, 0., work.budget(),
                    signature_capacity, [&] { work.poll(); }, query.contact_cache);
                if (bound) signature = BridgeAwareCubicContact::epoch_query_signature_raw(*bound);
            }
            if (!signature) signature = BridgeAwareCubicContact::capture_query_signature(query.lower,
                query.upper, query.upper_z, 0., work.budget(), signature_capacity,
                [&] { work.poll(); }, query.contact_cache);
            if (!signature) return false;
            const size_t signature_units = BridgeAwareCubicContact::query_signature_units(*signature);
            const size_t retained_signature_units = bound ?
                BridgeAwareCubicContact::epoch_query_signature_units(*bound) : signature_units;
            if ((bound && retained_signature_units <= signature_units) || retained_signature_units > signature_capacity)
                return false;

            Entry staged(query, description);
            staged.signature = std::move(signature);
            staged.raw_units = signature_units;
            staged.units = description.key_units + retained_signature_units;
            if (bound) {
                staged.binding_units = retained_signature_units - signature_units;
                staged.bindings[0] = std::move(bound);
            }
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
            m_statistics.bound_units += added->binding_units;
            for (const auto &binding : added->bindings) if (binding) ++m_statistics.bound_handles;
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
    static constexpr size_t epoch_registry_units = 16;
    static constexpr coord_t coordinate_limit = std::numeric_limits<coord_t>::max() / 4;
    struct Unavailable {};
    struct Canceled { std::exception_ptr exception; };
    struct AuditFailure : std::exception {
        const char *what() const noexcept override
        { return "Span epoch audit mismatch or unavailable raw-signature reference"; }
    };

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
        size_t hash, point_count, shape_count, units = 0, raw_units = 0, binding_units = 0;
        // Exact-sized arrays avoid unaccounted nested-vector spare capacity.
        // Shapes store polygon count, then contour size / hole count / hole
        // sizes per polygon. All original ring and point order is preserved.
        std::unique_ptr<Point[]> points;
        std::unique_ptr<size_t[]> shapes;
        BridgeAwareCubicContact::QuerySignaturePtr signature;
        std::array<BridgeAwareCubicContact::EpochQuerySignaturePtr, 2> bindings;
        Entry(const Query &query, const Description &description)
            : kind(query.kind), phase(query.phase), gap(query.gap), selection_width(query.selection_width),
              hash(description.hash), point_count(description.points), shape_count(description.shapes) {}
    };
    using Entries = std::list<Entry>;
    using Iterator = Entries::iterator;
    struct Bucket { std::array<Iterator, 2> entries; size_t size = 0; };
    using Index = std::map<size_t, Bucket>;
    struct AdmittedEpoch {
        BridgeAwareCubicContact::CanonicalLowerEpochPtr epoch;
        size_t bytes;
    };
    using Epochs = std::list<AdmittedEpoch>;
    static_assert(sizeof(Entry) + sizeof(Index::value_type) + 10 * sizeof(void *) <= metadata_units * sizeof(Point),
                  "Span entry and container metadata must fit their admitted units");
    static_assert(sizeof(AdmittedEpoch) + 6 * sizeof(void *) <= epoch_registry_units * sizeof(Point),
                  "Epoch registry node and allocation allowance must fit their admitted units");
    size_t m_max_units, m_max_entries;
    Entries m_entries;
    Index m_index;
    Epochs m_epochs;
    Statistics m_statistics;

    static size_t contact_units(const Entry &entry) noexcept
    {
        return entry.raw_units + entry.binding_units;
    }
    static bool contains_epoch(const Epochs &epochs,
        const BridgeAwareCubicContact::CanonicalLowerEpochPtr &wanted, Work &work)
    {
        for (const auto &entry : epochs) { work.charge(); if (entry.epoch == wanted) return true; }
        return false;
    }
    static bool requested_epoch(const BridgeAwareCubicContact::CanonicalLowerEpochPtr &wanted,
        const std::vector<BridgeAwareCubicContact::CanonicalLowerEpochPtr> &epochs, Work *work = nullptr)
    {
        for (const auto &epoch : epochs) {
            if (work) work->charge(2); // This preflight and the identical commit scan.
            if (epoch == wanted) return true;
        }
        return false;
    }
    bool valid_epoch_context(const Query &query, Work &work) const
    {
        if (query.epochs == nullptr) return false;
        work.charge();
        return query.epochs->current && query.epochs->prepared &&
            query.epochs->prepared.get() == &query.lower &&
            contains_epoch(m_epochs, query.epochs->current, work) &&
            BridgeAwareCubicContact::lower_epoch_source_matches(*query.epochs->current,
                query.epochs->prepared, query.phase);
    }
    void audit_epoch(const Entry &entry, const Query &query, Work &work)
    {
        if (!query.audit_epoch_hits) return;
        ++m_statistics.epoch_audits;
        ++m_statistics.raw_checks;
        if (!BridgeAwareCubicContact::matches_query_signature(*entry.signature,
                query.lower, query.upper, query.upper_z, 0., work.budget(),
                [&] { work.poll(); }, query.contact_cache)) {
            ++m_statistics.epoch_audit_mismatches;
            throw AuditFailure{};
        }
    }
    bool matches_epoch(Entry &entry, const Query &query, Work &work)
    {
        if (!valid_epoch_context(query, work)) return false;
        const auto &epochs = *query.epochs;
        for (const auto &bound : entry.bindings) {
            work.charge();
            if (bound && BridgeAwareCubicContact::query_signature_epoch(*bound) == epochs.current &&
                BridgeAwareCubicContact::epoch_query_signature_raw(*bound) == entry.signature &&
                BridgeAwareCubicContact::matches_epoch_query_signature(*bound, epochs.prepared,
                    query.phase, query.upper, query.upper_z, 0., work.budget(),
                    [&] { work.poll(); }, query.contact_cache)) {
                work.poll();
                audit_epoch(entry, query, work);
                work.poll();
                return true;
            }
        }
        if (!epochs.transition || BridgeAwareCubicContact::lower_epoch_after(*epochs.transition) != epochs.current)
            return false;
        const auto &before = BridgeAwareCubicContact::lower_epoch_before(*epochs.transition);
        if (!before || !contains_epoch(m_epochs, before, work)) return false;
        size_t available_slot = entry.bindings.size();
        for (size_t i = 0; i < entry.bindings.size(); ++i) {
            work.charge();
            if (!entry.bindings[i]) { available_slot = i; break; }
        }
        // Never displace the last-observed binding merely to attempt a new
        // optional query. Plane retirement makes room at an explicit boundary.
        if (available_slot == entry.bindings.size()) return false;
        for (const auto &bound : entry.bindings) {
            work.charge();
            if (!bound || BridgeAwareCubicContact::query_signature_epoch(*bound) != before ||
                BridgeAwareCubicContact::epoch_query_signature_raw(*bound) != entry.signature) continue;
            const size_t bound_units = BridgeAwareCubicContact::epoch_query_signature_units(*bound);
            if (bound_units <= entry.raw_units) return false;
            // Advance shares this exact raw object and allocates only another
            // wrapper of the same type. Admit its storage before the call;
            // never charge the already retained raw arrays a second time.
            const size_t reserve = bound_units - entry.raw_units;
            if (reserve > m_max_units - m_statistics.units) return false;
            auto advanced = BridgeAwareCubicContact::advance_epoch_query_signature(*bound,
                *epochs.transition, epochs.prepared, query.phase, epochs.limits, work.budget(),
                [&] { work.poll(); }, query.contact_cache);
            if (advanced.impact != BridgeAwareCubicContact::QueryImpact::Unaffected || !advanced.signature)
                return false;
            const size_t advanced_units = BridgeAwareCubicContact::epoch_query_signature_units(*advanced.signature);
            if (advanced_units <= entry.raw_units || advanced_units - entry.raw_units > reserve ||
                BridgeAwareCubicContact::query_signature_epoch(*advanced.signature) != epochs.current ||
                BridgeAwareCubicContact::epoch_query_signature_raw(*advanced.signature) != entry.signature ||
                !BridgeAwareCubicContact::matches_epoch_query_signature(*advanced.signature,
                    epochs.prepared, query.phase, query.upper, query.upper_z, 0., work.budget(),
                    [&] { work.poll(); }, query.contact_cache)) return false;
            const size_t units = advanced_units - entry.raw_units;
            work.charge(); // Nonthrowing handle/statistics commit below.
            work.poll();
            audit_epoch(entry, query, work);
            work.poll();
            entry.bindings[available_slot] = std::move(advanced.signature);
            entry.binding_units += units;
            entry.units += units;
            m_statistics.units += units;
            m_statistics.bound_units += units;
            ++m_statistics.bound_handles;
            ++m_statistics.advances;
            return true;
        }
        return false;
    }
    void bind_after_raw_match(Entry &entry, const Query &query, Work &work)
    {
        try {
            if (entry.binding_units != 0 || !valid_epoch_context(query, work)) return;
            const size_t capacity = std::min(size_t(8192), m_max_units - m_statistics.units);
            if (capacity == 0) return;
            const auto &epochs = *query.epochs;
            auto bound = BridgeAwareCubicContact::capture_epoch_query_signature(epochs.current,
                epochs.prepared, query.phase, query.upper, query.upper_z, 0., work.budget(), capacity,
                [&] { work.poll(); }, query.contact_cache);
            if (!bound) return;
            const auto &raw = BridgeAwareCubicContact::epoch_query_signature_raw(*bound);
            const size_t units = BridgeAwareCubicContact::epoch_query_signature_units(*bound);
            if (!raw || units <= BridgeAwareCubicContact::query_signature_units(*raw) || units > capacity ||
                BridgeAwareCubicContact::query_signature_epoch(*bound) != epochs.current) return;
            work.charge(entry.raw_units + 1);
            work.poll();
            const size_t old_units = entry.raw_units;
            // Fresh raw equality above proved this same policy fact; capture
            // describes that same immutable query. Replace raw and binding
            // together so every retained binding shares entry.signature.
            entry.signature = raw;
            entry.raw_units = BridgeAwareCubicContact::query_signature_units(*raw);
            entry.bindings[0] = std::move(bound);
            entry.binding_units = units - entry.raw_units;
            entry.units = entry.units - old_units + units;
            m_statistics.units = m_statistics.units - old_units + units;
            m_statistics.bound_units += entry.binding_units;
            ++m_statistics.bound_handles;
        } catch (const Unavailable &) {
            // Optional rebinding cannot undo the already complete raw match.
        } catch (const std::bad_alloc &) {
        }
    }

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
        m_statistics.bound_units -= entry->binding_units;
        for (const auto &bound : entry->bindings) if (bound) --m_statistics.bound_handles;
        m_entries.erase(entry);
        m_statistics.entries = m_entries.size();
        ++m_statistics.evictions;
    }
};

}} // namespace Slic3r::BridgeAwareCubic

#endif
