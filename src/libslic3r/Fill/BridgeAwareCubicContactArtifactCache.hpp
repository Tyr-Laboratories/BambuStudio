#ifndef slic3r_BridgeAwareCubicContactArtifactCache_hpp_
#define slic3r_BridgeAwareCubicContactArtifactCache_hpp_

#include "BridgeAwareCubicContact.hpp"
#include "BridgeAwareCubicSerialization.hpp"
#include "BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <new>
#include <stdexcept>

namespace Slic3r { namespace BridgeAwareCubic {

// Optional coarsening-local computational reuse for direct roofs. Artifacts
// contain no support verdict: every hit still requires the current complete
// physical-cell check. Repair, final/export, probes and exhaustive reference
// checks must not consult this cache. No lower epoch or Plan is retained.
class ContactArtifactCache {
    struct Accounting { std::atomic<size_t> bytes{0}; };
    struct Lease {
        std::shared_ptr<Accounting> account;
        size_t bytes = 0;
        ~Lease() { if (account) account->bytes.fetch_sub(bytes, std::memory_order_relaxed); }
    };
public:
    enum class Purpose { Ineligible, DirectRoof };
    using Cancel = std::function<void()>;
    struct Query {
        Purpose purpose;
        const SerializedXY &phase;
        double gap, selection_width;
        const ExPolygons &demand;
        const BridgeAwareCubicContact::PreparedLower &lower;
        const ExtrusionPath &upper;
        double upper_z;
        BridgeAwareCubicContact::QueryCache *contact_cache = nullptr;
    };

    class Artifact {
    public:
        const BridgeAwareCubicContact::PathContact &contact() const noexcept { return m_contact; }
        const ExPolygons &selected() const noexcept { return m_selected; }
        size_t owned_bytes() const noexcept { return m_owned_bytes; }
    private:
        friend class ContactArtifactCache;
        // Destroy the lease LAST, after every charged allocation is released.
        Lease m_lease;
        BridgeAwareCubicContact::PathContact m_contact;
        ExPolygons m_selected;
        size_t m_owned_bytes = 0;
        SerializedXY m_phase;
        double m_gap = 0., m_selection_width = 0., m_upper_z = 0., m_flow = 0.;
        float m_width = 0., m_height = 0.;
        unsigned m_role = 0;
        size_t m_hash = 0, m_point_count = 0, m_shape_count = 0, m_upper_count = 0;
        // Demand rings followed by the exact upper points. Shape sizes retain
        // contour/hole/order identity without nested key-vector spare capacity.
        std::unique_ptr<Point[]> m_points;
        std::unique_ptr<size_t[]> m_shapes;
        BridgeAwareCubicContact::QuerySignaturePtr m_signature;
    };
    using Handle = std::shared_ptr<const Artifact>;
    struct Statistics {
        size_t hits = 0, misses = 0, signature_misses = 0, publications = 0, evictions = 0, entries = 0;
        // Actual nested capacities plus conservative control/index/node
        // allowances, including simultaneous staged publication. Escaped
        // handles remain charged after eviction until their last owner dies.
        size_t resident_bytes = 0, peak_bytes = 0, staged_peak_bytes = 0;
    };

    explicit ContactArtifactCache(size_t max_bytes = 4 * 1024 * 1024, size_t max_entries = 20000)
        : m_max_bytes(std::min(max_bytes, size_t(4 * 1024 * 1024))),
          m_max_entries(std::min(max_entries, size_t(20000))) {}
    ContactArtifactCache(const ContactArtifactCache &) = delete;
    ContactArtifactCache &operator=(const ContactArtifactCache &) = delete;
    ~ContactArtifactCache()
    {
        if (m_account) m_account->bytes.fetch_sub(m_entries.size() * node_bytes, std::memory_order_relaxed);
    }

    // Exact policy/upper equality followed by fresh Contact matching AND
    // current active-output admission. XY tolerance is exactly zero, as in
    // the direct-roof computation. Cancellation propagates unchanged.
    Handle find(const Query &query, BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                const Cancel &cancel = {})
    {
        Work work(budget, cancel);
        try {
            work.poll();
            const Description description = describe(query, work);
            lookup_work(work);
            const auto range = m_index.equal_range(description.hash);
            for (auto it = range.first; it != range.second; ++it) {
                const auto &artifact = *it->second;
                if (!same_key(*artifact, query, description, work)) continue;
                if (BridgeAwareCubicContact::matches_query_signature_with_contact(*artifact->m_signature,
                        query.lower, query.upper, query.upper_z, 0., artifact->m_contact,
                        work.budget(), [&] { work.poll(); }, query.contact_cache)) {
                    work.poll();
                    ++m_statistics.hits;
                    return artifact;
                }
                ++m_statistics.signature_misses;
            }
        } catch (const Canceled &canceled) {
            std::rethrow_exception(canceled.exception);
        } catch (const Unavailable &) {
        } catch (const std::bad_alloc &) {
        } catch (const std::length_error &) {
        }
        ++m_statistics.misses;
        return {};
    }

    // Caller contract: these are the ACTUAL completed footprint(...,
    // ContactsOnly) and nonempty unsupported-selection Boolean outputs for
    // precisely this Query. A default/forged result is not such provenance.
    // This stores no physical sections, support verdict or cell certificate.
    bool publish(const Query &query, const BridgeAwareCubicContact::PathContact &contact,
                 const ExPolygons &selected, BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                 const Cancel &cancel = {})
    {
        Work work(budget, cancel);
        try {
            work.poll();
            const Description description = describe(query, work);
            if (!m_max_entries || !m_max_bytes) return false;
            const size_t payload_bytes = payload_storage(contact, selected, work);
            const size_t key_bytes = checked_add(checked_mul(description.points, sizeof(Point)),
                                                  checked_mul(description.shapes, sizeof(size_t)));
            const size_t fixed = checked_add(artifact_bytes + node_bytes, checked_add(payload_bytes, key_bytes));
            // The first attempt is small and deterministic, independent of free
            // space. Larger signatures retry up to the existing optional bound;
            // every attempt retains all shared work already spent.
            const size_t first_capture = BridgeAwareCubicContact::query_signature_capture_bytes(64);
            const size_t context = context_bytes();
            if (!first_capture || fixed > m_max_bytes || first_capture > m_max_bytes - fixed ||
                (!m_account && context > m_max_bytes - fixed - first_capture)) return false;
            if (!m_account) {
                work.charge();
                m_account = std::make_shared<Accounting>();
                m_account->bytes.store(context, std::memory_order_relaxed);
                observe(0);
            }
            Reservation staged(*this);
            BridgeAwareCubicContact::QuerySignaturePtr signature;
            size_t capture_bytes = 0;
            for (size_t capacity = 64; capacity <= 8192; capacity *= 2) {
                capture_bytes = BridgeAwareCubicContact::query_signature_capture_bytes(capacity);
                if (!capture_bytes) return false;
                staged.resize(checked_add(fixed, capture_bytes));
                signature = BridgeAwareCubicContact::capture_query_signature(query.lower, query.upper,
                    query.upper_z, 0., work.budget(), capacity, [&] { work.poll(); }, query.contact_cache);
                if (signature) break;
            }
            if (!signature) return false;
            const size_t signature_bytes = BridgeAwareCubicContact::query_signature_resident_bytes(*signature);
            if (!signature_bytes || signature_bytes > capture_bytes) return false;
            if (!BridgeAwareCubicContact::matches_query_signature_with_contact(*signature, query.lower,
                    query.upper, query.upper_z, 0., contact, work.budget(), [&] { work.poll(); }, query.contact_cache))
                return false;
            const size_t staging_peak = staged.bytes;
            // Capture's relocation allowance is no longer live. All subsequent
            // copied buffers have their final capacities pre-admitted below.
            staged.resize(checked_add(fixed, signature_bytes));
            work.charge();
            auto candidate = std::shared_ptr<Artifact>(new Artifact);
            candidate->m_signature = std::move(signature);
            copy_key(*candidate, query, description, work);
            copy_payload(*candidate, contact, selected, work);
            const size_t actual_payload = payload_storage(candidate->m_contact, candidate->m_selected, work);
            if (actual_payload > payload_bytes) throw Unavailable{};
            candidate->m_owned_bytes = checked_add(artifact_bytes,
                checked_add(key_bytes, checked_add(actual_payload, signature_bytes)));
            staged.resize(candidate->m_owned_bytes + node_bytes);

            // Private nodes are fully allocated before touching the live index.
            Entries pending;
            Index pending_index;
            work.charge(2);
            pending.push_back(candidate);
            pending_index.emplace(description.hash, pending.begin());
            Artifact *const staged_artifact = candidate.get();
            candidate.reset(); // The pending list is the sole owner until commit.

            lookup_work(work);
            const auto range = m_index.equal_range(description.hash);
            auto version_victim = m_entries.end();
            size_t versions = 0;
            for (auto it = range.first; it != range.second; ++it) {
                if (same_key(**it->second, query, description, work)) {
                    if (!versions) version_victim = it->second;
                    ++versions;
                }
            }
            if (versions < 2) version_victim = m_entries.end();
            // Reserve staging headroom only when the quota can comfortably
            // hold it. Tiny custom quotas may retain one/two entries and decline
            // a later publication, instead of discarding facts before admission.
            const size_t headroom = staging_peak <= m_max_bytes / 4 ?
                std::max(m_headroom, staging_peak) : m_headroom;
            const size_t retained_limit = m_max_bytes - std::min(headroom, m_max_bytes / 2);
            size_t projected = resident();
            size_t count = m_entries.size() + 1;
            if (version_victim != m_entries.end()) {
                work.charge(2);
                projected -= reclaimable(version_victim);
                --count;
            }
            auto prefix_end = m_entries.begin();
            while (prefix_end != m_entries.end() &&
                   (count > m_max_entries || (count > 1 && projected > retained_limit))) {
                work.charge(2);
                if (prefix_end != version_victim) {
                    projected -= reclaimable(prefix_end);
                    --count;
                }
                ++prefix_end;
            }
            // Precharge both index searches and all owned releases. Neither
            // allocation nor cancellation/work failure is allowed after this.
            if (version_victim != m_entries.end()) eviction_work(version_victim, work);
            for (auto it = m_entries.begin(); it != prefix_end; ++it)
                if (it != version_victim) eviction_work(it, work);
            lookup_work(work);
            work.poll();

            // No-throw commit. Numeric comparator, node transfer, shared-pointer
            // moves and owned destructors perform no user callbacks/allocation.
            bool victim_in_prefix = false;
            for (auto it = m_entries.begin(); it != prefix_end; ++it)
                if (it == version_victim) victim_in_prefix = true;
            while (m_entries.begin() != prefix_end) evict(m_entries.begin());
            if (!victim_in_prefix && version_victim != m_entries.end()) evict(version_victim);
            auto &accepted = *staged_artifact;
            accepted.m_lease.account = m_account;
            accepted.m_lease.bytes = accepted.m_owned_bytes;
            m_entries.splice(m_entries.end(), pending);
            m_index.insert(pending_index.extract(pending_index.begin()));
            staged.transfer();
            m_headroom = headroom;
            ++m_statistics.publications;
            return true;
        } catch (const Canceled &canceled) {
            std::rethrow_exception(canceled.exception);
        } catch (const Unavailable &) {
        } catch (const std::bad_alloc &) {
        } catch (const std::length_error &) {
        }
        return false;
    }

    Statistics statistics() const noexcept
    {
        Statistics result = m_statistics;
        result.entries = m_entries.size();
        result.resident_bytes = resident();
        return result;
    }

private:
    static constexpr size_t max_vertices = 200000, max_items = 20000;
    static constexpr coord_t coordinate_limit = std::numeric_limits<coord_t>::max() / 4;
    struct Unavailable {};
    struct Canceled { std::exception_ptr exception; };
    class Work {
    public:
        Work(BridgeAwareCubicValidation::WorkBudget *budget, const Cancel &cancel)
            : m_budget(budget ? budget : &m_local), m_cancel(cancel) {}
        void poll() const
        {
            if (m_cancel) try { m_cancel(); }
            catch (const Canceled &) { throw; }
            catch (...) { throw Canceled{std::current_exception()}; }
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
    struct Description { size_t hash = 0, points = 0, shapes = 1, items = 0; };
    using Entries = std::list<Handle>;
    using Iterator = Entries::iterator;
    using Index = std::multimap<size_t, Iterator>;
    // Allowance includes allocator/control metadata and the two tree/list node
    // link sets. Payload arrays/signature vectors are counted separately.
    static constexpr size_t node_bytes = sizeof(Handle) + sizeof(Index::value_type) + 16 * sizeof(void *);
    static constexpr size_t artifact_bytes = sizeof(Artifact) + 128;
    static size_t context_bytes() { return sizeof(Accounting) + sizeof(ContactArtifactCache) + 256; }
    size_t m_max_bytes, m_max_entries, m_headroom = 0;
    std::shared_ptr<Accounting> m_account;
    Entries m_entries;
    Index m_index;
    Statistics m_statistics;

    size_t resident() const noexcept
    { return m_account ? m_account->bytes.load(std::memory_order_relaxed) : 0; }
    void observe(size_t staged)
    {
        m_statistics.peak_bytes = std::max(m_statistics.peak_bytes, resident());
        m_statistics.staged_peak_bytes = std::max(m_statistics.staged_peak_bytes, staged);
    }
    struct Reservation {
        ContactArtifactCache &owner;
        size_t bytes = 0;
        explicit Reservation(ContactArtifactCache &value) : owner(value) {}
        ~Reservation() { if (bytes) owner.m_account->bytes.fetch_sub(bytes, std::memory_order_relaxed); }
        void resize(size_t wanted)
        {
            if (wanted > bytes) {
                const size_t more = wanted - bytes;
                if (more > owner.m_max_bytes - owner.resident()) throw Unavailable{};
                owner.m_account->bytes.fetch_add(more, std::memory_order_relaxed);
            } else owner.m_account->bytes.fetch_sub(bytes - wanted, std::memory_order_relaxed);
            bytes = wanted;
            owner.observe(bytes);
        }
        void transfer() noexcept { bytes = 0; }
    };
    static size_t checked_add(size_t a, size_t b)
    {
        if (b > std::numeric_limits<size_t>::max() - a) throw Unavailable{};
        return a + b;
    }
    static size_t checked_mul(size_t a, size_t b)
    {
        if (a && b > std::numeric_limits<size_t>::max() / a) throw Unavailable{};
        return a * b;
    }
    template<class T> static void hash_value(size_t &hash, const T &value, Work &work)
    {
        work.charge();
        hash ^= std::hash<T>{}(value) + size_t(0x9e3779b9) + (hash << 6) + (hash >> 2);
    }
    static bool safe(const Point &p)
    { return p.x() >= -coordinate_limit && p.x() <= coordinate_limit &&
             p.y() >= -coordinate_limit && p.y() <= coordinate_limit; }
    static void ring(const Polygon &polygon, bool hole, Description &d, Work &work)
    {
        work.charge();
        if (polygon.size() < 3 || polygon.size() > max_vertices - d.points || ++d.items > max_items)
            throw Unavailable{};
        d.points += polygon.size();
        hash_value(d.hash, polygon.size(), work);
        for (const Point &point : polygon.points) {
            if (!safe(point)) throw Unavailable{};
            hash_value(d.hash, point.x(), work); hash_value(d.hash, point.y(), work);
        }
        work.charge(polygon.size());
        const double area = polygon.area();
        if (!std::isfinite(area) || (hole ? area >= 0. : area <= 0.)) throw Unavailable{};
    }
    static void polygons(const ExPolygons &polygons, Description &d, Work &work)
    {
        if (polygons.size() > max_items) throw Unavailable{};
        hash_value(d.hash, polygons.size(), work);
        for (const ExPolygon &polygon : polygons) {
            if (polygon.holes.size() > max_items - d.items) throw Unavailable{};
            d.shapes += 2 + polygon.holes.size();
            ring(polygon.contour, false, d, work);
            hash_value(d.hash, polygon.holes.size(), work);
            for (const Polygon &hole : polygon.holes) ring(hole, true, d, work);
        }
    }
    static Description describe(const Query &q, Work &work)
    {
        work.charge();
        if (q.purpose != Purpose::DirectRoof || !std::isfinite(q.gap) || q.gap <= 0. ||
            !std::isfinite(q.selection_width) || q.selection_width <= 0. || q.demand.empty() ||
            !std::isfinite(q.upper_z) || !std::isfinite(q.upper.width) || q.upper.width <= 0. ||
            !std::isfinite(q.upper.height) || q.upper.height <= 0. ||
            !std::isfinite(q.upper.mm3_per_mm) || q.upper.mm3_per_mm <= 0. ||
            q.upper.polyline.size() < 2 || !q.upper.polyline.fitting_result.empty()) throw Unavailable{};
        Description d;
        hash_value(d.hash, q.gap, work); hash_value(d.hash, q.selection_width, work);
        for (unsigned axis = 0; axis < 2; ++axis)
            for (double value : {q.phase.shift[axis], q.phase.nozzle_offset[axis], q.phase.plate_origin[axis]}) {
                if (!std::isfinite(value)) throw Unavailable{};
                hash_value(d.hash, value, work);
            }
        polygons(q.demand, d, work);
        if (q.upper.polyline.size() > max_vertices - d.points) throw Unavailable{};
        d.points += q.upper.polyline.size();
        hash_value(d.hash, q.upper_z, work); hash_value(d.hash, q.upper.width, work);
        hash_value(d.hash, q.upper.height, work); hash_value(d.hash, q.upper.mm3_per_mm, work);
        hash_value(d.hash, unsigned(q.upper.role()), work); hash_value(d.hash, q.upper.polyline.size(), work);
        bool nonzero = false;
        for (size_t i = 0; i < q.upper.polyline.size(); ++i) {
            const Point &p = q.upper.polyline.points[i];
            if (!safe(p)) throw Unavailable{};
            nonzero |= i && p != q.upper.polyline.points[i - 1];
            hash_value(d.hash, p.x(), work); hash_value(d.hash, p.y(), work);
        }
        if (!nonzero) throw Unavailable{};
        return d;
    }
    static size_t polygon_storage(const ExPolygons &polygons, Work &work)
    {
        size_t result = checked_mul(polygons.capacity(), sizeof(ExPolygon));
        for (const ExPolygon &polygon : polygons) {
            work.charge();
            result = checked_add(result, checked_mul(polygon.contour.points.capacity(), sizeof(Point)));
            result = checked_add(result, checked_mul(polygon.holes.capacity(), sizeof(Polygon)));
            for (const Polygon &hole : polygon.holes) {
                work.charge();
                result = checked_add(result, checked_mul(hole.points.capacity(), sizeof(Point)));
            }
        }
        return result;
    }
    static size_t payload_storage(const BridgeAwareCubicContact::PathContact &contact,
                                  const ExPolygons &selected, Work &work)
    {
        work.charge();
        if (!contact.input_valid || !contact.completed || contact.resource_limited ||
            !contact.failure_reason.empty() || !contact.physical_footprint.empty() ||
            !contact.physical_lines.empty() || contact.segments.empty() ||
            contact.segments.size() > max_items || selected.empty()) throw Unavailable{};
        Description d;
        polygons(selected, d, work);
        size_t bytes = checked_add(contact.failure_reason.capacity(), 1);
        bytes = checked_add(bytes, checked_mul(contact.segments.capacity(), sizeof(BridgeAwareCubicContact::SegmentContact)));
        bytes = checked_add(bytes, checked_mul(contact.physical_lines.capacity(), sizeof(Polyline)));
        bytes = checked_add(bytes, polygon_storage(contact.physical_footprint, work));
        bytes = checked_add(bytes, polygon_storage(selected, work));
        for (const auto &segment : contact.segments) {
            work.charge();
            polygons(segment.centerline_support, d, work);
            if (segment.line_contacts.size() > max_vertices - d.points) throw Unavailable{};
            d.points += segment.line_contacts.size();
            bytes = checked_add(bytes, polygon_storage(segment.centerline_support, work));
            bytes = checked_add(bytes, checked_mul(segment.line_contacts.capacity(), sizeof(BridgeAwareCubicContact::ExactInterval)));
        }
        return bytes;
    }
    static void copy_key(Artifact &a, const Query &q, const Description &d, Work &work)
    {
        a.m_phase = q.phase; a.m_gap = q.gap; a.m_selection_width = q.selection_width;
        a.m_upper_z = q.upper_z; a.m_flow = q.upper.mm3_per_mm;
        a.m_width = q.upper.width; a.m_height = q.upper.height; a.m_role = unsigned(q.upper.role());
        a.m_hash = d.hash; a.m_point_count = d.points; a.m_shape_count = d.shapes;
        a.m_upper_count = q.upper.polyline.size();
        work.charge(d.points + d.shapes);
        a.m_points.reset(new Point[d.points]); a.m_shapes.reset(new size_t[d.shapes]);
        size_t p = 0, s = 0;
        const auto copy_ring = [&](const Polygon &polygon) {
            a.m_shapes[s++] = polygon.size();
            for (const Point &point : polygon.points) { work.charge(); a.m_points[p++] = point; }
        };
        a.m_shapes[s++] = q.demand.size();
        for (const ExPolygon &polygon : q.demand) {
            copy_ring(polygon.contour); work.charge(); a.m_shapes[s++] = polygon.holes.size();
            for (const Polygon &hole : polygon.holes) copy_ring(hole);
        }
        for (const Point &point : q.upper.polyline.points) { work.charge(); a.m_points[p++] = point; }
    }
    static bool same_key(const Artifact &a, const Query &q, const Description &d, Work &work)
    {
        work.charge();
        if (a.m_gap != q.gap || a.m_selection_width != q.selection_width || a.m_upper_z != q.upper_z ||
            a.m_flow != q.upper.mm3_per_mm || a.m_width != q.upper.width || a.m_height != q.upper.height ||
            a.m_role != unsigned(q.upper.role()) || a.m_point_count != d.points || a.m_shape_count != d.shapes ||
            a.m_upper_count != q.upper.polyline.size()) return false;
        for (unsigned axis = 0; axis < 2; ++axis) {
            work.charge();
            if (a.m_phase.shift[axis] != q.phase.shift[axis] || a.m_phase.nozzle_offset[axis] != q.phase.nozzle_offset[axis] ||
                a.m_phase.plate_origin[axis] != q.phase.plate_origin[axis]) return false;
        }
        size_t p = 0, s = 0;
        const auto same_ring = [&](const Polygon &polygon) {
            work.charge();
            if (a.m_shapes[s++] != polygon.size()) return false;
            for (const Point &point : polygon.points) { work.charge(); if (a.m_points[p++] != point) return false; }
            return true;
        };
        if (a.m_shapes[s++] != q.demand.size()) return false;
        for (const ExPolygon &polygon : q.demand) {
            if (!same_ring(polygon.contour)) return false;
            work.charge(); if (a.m_shapes[s++] != polygon.holes.size()) return false;
            for (const Polygon &hole : polygon.holes) if (!same_ring(hole)) return false;
        }
        for (const Point &point : q.upper.polyline.points) { work.charge(); if (a.m_points[p++] != point) return false; }
        return true;
    }
    template<class T> static void reserve(std::vector<T> &output, size_t count, Work &work)
    {
        work.charge();
        output.reserve(count);
        // Standard libraries used by this project reserve exactly here. A
        // different allocator's larger capacity cannot be silently retained.
        if (output.capacity() > count) throw Unavailable{};
    }
    static void copy_polygons(ExPolygons &output, const ExPolygons &input, Work &work)
    {
        reserve(output, input.capacity(), work);
        for (const ExPolygon &polygon : input) {
            work.charge(); output.emplace_back();
            auto &target = output.back();
            reserve(target.contour.points, polygon.contour.points.capacity(), work);
            for (const Point &point : polygon.contour.points) { work.charge(); target.contour.points.push_back(point); }
            reserve(target.holes, polygon.holes.capacity(), work);
            for (const Polygon &hole : polygon.holes) {
                work.charge(); target.holes.emplace_back();
                reserve(target.holes.back().points, hole.points.capacity(), work);
                for (const Point &point : hole.points) { work.charge(); target.holes.back().points.push_back(point); }
            }
        }
    }
    static void copy_payload(Artifact &a, const BridgeAwareCubicContact::PathContact &contact,
                             const ExPolygons &selected, Work &work)
    {
        a.m_contact.input_valid = contact.input_valid; a.m_contact.completed = contact.completed;
        a.m_contact.resource_limited = contact.resource_limited;
        a.m_contact.contact_z = contact.contact_z;
        a.m_contact.coordinate_uncertainty_mm = contact.coordinate_uncertainty_mm;
        copy_polygons(a.m_contact.physical_footprint, contact.physical_footprint, work);
        reserve(a.m_contact.physical_lines, contact.physical_lines.capacity(), work);
        reserve(a.m_contact.segments, contact.segments.capacity(), work);
        for (const auto &segment : contact.segments) {
            work.charge(); a.m_contact.segments.emplace_back();
            auto &target = a.m_contact.segments.back();
            target.segment_index = segment.segment_index;
            copy_polygons(target.centerline_support, segment.centerline_support, work);
            reserve(target.line_contacts, segment.line_contacts.capacity(), work);
            for (const auto &interval : segment.line_contacts) { work.charge(); target.line_contacts.push_back(interval); }
        }
        copy_polygons(a.m_selected, selected, work);
    }
    void lookup_work(Work &work) const
    {
        work.charge();
        for (size_t n = m_index.size(); n; n >>= 1) work.charge(2);
    }
    static size_t reclaimable(Iterator it)
    { return node_bytes + (it->use_count() == 1 ? (*it)->m_owned_bytes : 0); }
    void eviction_work(Iterator it, Work &work) const
    {
        lookup_work(work);
        const auto range = m_index.equal_range((*it)->m_hash);
        for (auto index = range.first; index != range.second; ++index) {
            work.charge(2);
            if (index->second == it) break;
        }
        // Destruction visits all owned vector elements. Overcharging bytes
        // bounds that traversal without walking payloads again after commit.
        work.charge((*it)->m_owned_bytes);
    }
    void evict(Iterator it) noexcept
    {
        const auto range = m_index.equal_range((*it)->m_hash);
        for (auto index = range.first; index != range.second; ++index)
            if (index->second == it) { m_index.erase(index); break; }
        m_entries.erase(it);
        m_account->bytes.fetch_sub(node_bytes, std::memory_order_relaxed);
        ++m_statistics.evictions;
    }
};

}} // namespace Slic3r::BridgeAwareCubic

#endif
