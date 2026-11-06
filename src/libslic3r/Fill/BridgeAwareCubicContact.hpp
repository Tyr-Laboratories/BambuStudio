#ifndef slic3r_BridgeAwareCubicContact_hpp_
#define slic3r_BridgeAwareCubicContact_hpp_

#include "../ExtrusionEntity.hpp"
#include "../ExPolygon.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubicValidation { struct WorkBudget; }}
namespace Slic3r { namespace BridgeAwareCubic { struct SerializedXY; }}

namespace Slic3r { namespace BridgeAwareCubicContact {

enum class FootprintMode { Full, ContactsOnly };

struct Bead {
    const ExtrusionPath *path = nullptr;
    double print_z = 0.;
};

// Parameters on the original segment, in [0,1]. Equal endpoints represent a
// finite transverse contact line, not an isolated physical contact point.
struct ExactInterval {
    int64_t begin_numerator = 0;
    int64_t begin_denominator = 1;
    int64_t end_numerator = 0;
    int64_t end_denominator = 1;
    // Maximum movement along the upper path relative to this nominal event.
    // Nonzero only for a contact witness certified through XY perturbations.
    double position_uncertainty_mm = 0.;
};

struct SegmentContact {
    size_t segment_index = 0;
    ExPolygons centerline_support;
    std::vector<ExactInterval> line_contacts;
};

struct PathContact {
    bool input_valid = true;
    bool completed = true;
    bool resource_limited = false;
    std::string failure_reason;
    double contact_z = 0.;
    // Per original emitted endpoint; the verifier bounds accumulated segment
    // length changes separately from line-anchor position uncertainty.
    double coordinate_uncertainty_mm = 0.;
    // Undilated lower-bead sections at contact_z. These alone describe cell
    // walls. Zero-width sections remain lines, never inflated material.
    ExPolygons physical_footprint;
    Polylines physical_lines;
    // One entry for each original positive-length upper segment. These are
    // direction-dependent contact certificates and must never close cells.
    std::vector<SegmentContact> segments;
};

struct SupportQuery {
    PathContact contact;
    // A complete, demand-independent proof of continuous support. An ordinary
    // passing bridge with air between its anchors does not set this flag.
    // This proves path spans only: it never certifies physical cells or
    // discharges an independent, nonempty model-cell demand.
    bool entire_path_supported = false;
};

// Rounded-rectangle bead section, using actual emitted width and height.
// The top and bottom flat widths are width-height; width==height is circular.
// Returns zero outside the closed Z interval; invalid dimensions return NaN.
double section_width_mm(double width, double height, double print_z, double z);

// Immutable owned lower sections for one exact (upper_print_z, upper_height)
// pair. Validate every deposited bead before selecting eligible sections. A
// failed preparation retains only its failure status, never partial evidence.
// Keep at most one prepared entry alive per checking pass: each query admits
// its allocations together with this entry's owned geometry and conservative
// validated-input admission under the existing vertex bound. Preparation and
// every query share the pass budget; input admission is not charged twice.
struct PreparedLower;
using PreparedLowerPtr = std::shared_ptr<const PreparedLower>;
PreparedLowerPtr prepare_lower(const std::vector<Bead> &deposited,
                               double upper_print_z, float upper_height,
                               BridgeAwareCubicValidation::WorkBudget *budget = nullptr);
// Existing validated-input plus owned-geometry admission, without recharging
// preparation work. Epoch comparison has no prepared/cache argument; include
// this and live cache resident_units in its other_contact_units when retained
// by the caller across that comparison. Other epoch operations admit them.
size_t prepared_lower_contact_units(const PreparedLower &) noexcept;

// Optional identity for one complete ContactsOnly query, not a support proof.
// Own the exact upper inputs and the ordered canonical lower sections used by
// each positive segment. Unrelated sections need not match, but full prepared
// status, plane and ordinary upper-input guards are checked on every lookup.
// Capture and lookup share the caller's work budget; cap/allocation/validation
// failure returns null/false and must fall back to ordinary verification with
// that SAME remaining budget. Cancellation exceptions propagate unchanged.
// The eventual proof adapter must additionally key demand, gap and phase, and
// publish only after a complete passing span-only check. No cell proof follows
// from matching this signature. Storage units include metadata, upper points,
// segment records and owned section records, including spare vector capacity;
// the signature owns no lower plan. Supply the live query-mask cache, if any,
// for shared temporary-allocation admission; foreign caches are ineligible.
struct QuerySignature;
class QueryCache;
using QuerySignaturePtr = std::shared_ptr<const QuerySignature>;
using QuerySignatureCancel = std::function<void()>;
QuerySignaturePtr capture_query_signature(const PreparedLower &prepared,
                                           const ExtrusionPath &upper,
                                           double upper_print_z, double xy_tolerance_mm,
                                           BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                                           size_t capacity_units = 8192,
                                           const QuerySignatureCancel &cancel = {},
                                           QueryCache *cache = nullptr);
bool matches_query_signature(const QuerySignature &signature, const PreparedLower &prepared,
                             const ExtrusionPath &upper, double upper_print_z, double xy_tolerance_mm,
                             BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                             const QuerySignatureCancel &cancel = {},
                             QueryCache *cache = nullptr);
size_t query_signature_units(const QuerySignature &signature) noexcept;
// Owned object/control allowance and allocated vector capacities, distinct
// from the logical Contact units above. Zero means unavailable and must not be
// admitted by a byte-bounded owner. No transitive prepared geometry is retained.
size_t query_signature_resident_bytes(const QuerySignature &signature) noexcept;
// Conservative peak signature-storage bytes at the given logical-unit
// allowance, including old/new vector storage during growth. Ordinary Contact
// preparation/query scratch remains under its existing per-operation budget.
// Zero is unavailable (zero, oversized or overflowing allowance). Byte owners
// reserve this before capture and replace it with resident_bytes on return.
size_t query_signature_capture_bytes(size_t capacity_units) noexcept;

// Exact fresh raw-signature lookup with active immutable ContactsOnly output
// admitted in the SAME Contact budget as current preparation, upper input,
// candidate scratch and live evictable mask cache. Derive units from retained
// segments, line intervals and every mask contour/hole point; no asserted unit
// count is accepted. Reject failed/incomplete/mismatched contact metadata,
// incorrect positive original-segment correspondence, invalid finite bounds and any
// physical output. This does not recompute or independently certify retained:
// the caller must own the actual successful ContactsOnly result from the exact
// original signature query, and retain its provenance and policy separately.
// A false result is an optional miss; ordinary computation uses the SAME
// remaining shared work. Cancellation propagates and retained is not modified.
bool matches_query_signature_with_contact(const QuerySignature &signature,
    const PreparedLower &prepared, const ExtrusionPath &upper,
    double upper_print_z, double xy_tolerance_mm, const PathContact &retained,
    BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
    const QuerySignatureCancel &cancel = {}, QueryCache *cache = nullptr);

// Optional coarsening-local identities, never deposited-support proofs. The
// caller must prepare the COMPLETE emitted lower layer, across all roles and
// regions, in this exact serialized phase. Source-frame emission deltas cannot
// substitute for that preparation. Unknown/invalid/incomplete preparation is
// unavailable, not a valid empty physical epoch.
struct CanonicalLowerEpoch;
struct CanonicalLowerDelta;
struct EpochQuerySignature;
using CanonicalLowerEpochPtr = std::shared_ptr<const CanonicalLowerEpoch>;
using CanonicalLowerDeltaPtr = std::shared_ptr<const CanonicalLowerDelta>;
using EpochQuerySignaturePtr = std::shared_ptr<const EpochQuerySignature>;

struct EpochLimits {
    // A caller-granted share of the EXISTING object metadata allowance, not
    // a new allowance. Arguments are charged once per distinct owned epoch;
    // other_retained_bytes excludes those arguments but includes other live
    // epoch/delta allocations charged to this share. Staging/scratch also fit.
    size_t capacity_bytes = 0;
    size_t other_retained_bytes = 0;
    // OTHER epoch geometry actively used by this operation, in existing
    // Contact vertex units; excludes argument epochs charged internally.
    // Do not pass the whole retained registry or unrelated source metadata:
    // ordinary Contact admission remains per preparation/query.
    size_t other_contact_units = 0;
};

enum class EpochRelation { Identical, Changed, Incompatible };
enum class QueryImpact { Unaffected, Changed, Unavailable };
struct EpochQueryAdvance {
    QueryImpact impact = QueryImpact::Unavailable;
    // Present ONLY after a complete unaffected check and exact before/after
    // provenance checks. This newly bound identity is still not a span proof.
    EpochQuerySignaturePtr signature;
};

// Owns exact canonical endpoints, numeric radii and copied derived corners,
// along with exact phase components/plane keys. The prepared source is weakly
// identified by shared ownership, so old corners/index storage is not retained.
CanonicalLowerEpochPtr capture_lower_epoch(const PreparedLowerPtr &prepared,
    const BridgeAwareCubic::SerializedXY &phase, EpochLimits limits,
    BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
    const QuerySignatureCancel &cancel = {}, QueryCache *cache = nullptr);
bool lower_epoch_source_matches(const CanonicalLowerEpoch &, const PreparedLowerPtr &,
    const BridgeAwareCubic::SerializedXY &) noexcept;
size_t lower_epoch_owned_bytes(const CanonicalLowerEpoch &) noexcept;
size_t lower_epoch_resident_bytes(const CanonicalLowerEpoch &) noexcept;
// Conservative Contact admission units: ceil(owned_bytes / sizeof(Point)).
// Byte counts include object/control allowances, the weak source's retained
// make_shared object/control allocation, and allocated vector capacity. They
// do not claim allocator bookkeeping or total process-memory accounting.
size_t lower_epoch_contact_units(const CanonicalLowerEpoch &) noexcept;

// Different exact phase/upper/contact-plane keys produce Incompatible. For
// compatible epochs compare WHOLE canonical sections, including copied corners.
// Both removed and added complete axes participate in dirty-query discovery;
// never clip them to a source fragment, changed bbox, demand or cell.
CanonicalLowerDeltaPtr compare_lower_epochs(const CanonicalLowerEpochPtr &before,
    const CanonicalLowerEpochPtr &after, EpochLimits limits,
    BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
    const QuerySignatureCancel &cancel = {});
EpochRelation lower_epoch_relation(const CanonicalLowerDelta &) noexcept;
const CanonicalLowerEpochPtr &lower_epoch_before(const CanonicalLowerDelta &) noexcept;
const CanonicalLowerEpochPtr &lower_epoch_after(const CanonicalLowerDelta &) noexcept;
size_t lower_delta_owned_bytes(const CanonicalLowerDelta &) noexcept;
size_t lower_delta_resident_bytes(const CanonicalLowerDelta &) noexcept;
// Only delta-owned records; add each distinct argument epoch exactly once.
size_t lower_delta_owned_contact_units(const CanonicalLowerDelta &) noexcept;

// Capture the ordinary whole-query signature from the SAME immutable prepared
// source bound to epoch. The proof adapter invokes this only after that exact
// complete span-only check passes, and separately owns gap/demand/kind policy.
EpochQuerySignaturePtr capture_epoch_query_signature(const CanonicalLowerEpochPtr &epoch,
    const PreparedLowerPtr &prepared, const BridgeAwareCubic::SerializedXY &phase,
    const ExtrusionPath &upper, double upper_print_z, double xy_tolerance_mm,
    BridgeAwareCubicValidation::WorkBudget *budget = nullptr, size_t capacity_units = 8192,
    const QuerySignatureCancel &cancel = {}, QueryCache *cache = nullptr);
const CanonicalLowerEpochPtr &query_signature_epoch(const EpochQuerySignature &) noexcept;
// Allows safe demotion to the original exhaustive-signature lookup when an
// epoch is retired. The immutable raw signature cannot relabel a bound epoch.
const QuerySignaturePtr &epoch_query_signature_raw(const EpochQuerySignature &) noexcept;
// Raw signature plus binding wrapper/control allowance, excluding transitive
// canonical epoch geometry charged once by the caller's shared byte budget.
// Advancement shares the identical raw signature. Callers may conservatively
// charge this full count again, or charge only units - raw.units after proving
// exact shared raw ownership; initial/fresh capture must admit its new raw copy.
size_t epoch_query_signature_units(const EpochQuerySignature &) noexcept;

// Requires bound.epoch == delta.before, and the supplied current prepared
// ownership/phase to match delta.after. Tests every retained positive upper
// segment against old AND new dirty sections, including zero-radius axes and
// endpoints. No caller can directly relabel a bound signature's epoch.
// The current upper may differ later: matches_epoch_query_signature must still
// compare it freshly before any use of the transported span fact.
// Failure never mutates the old handle. Caller admits the staged binding under
// its existing span-fact allowance separately from EpochLimits geometry bytes.
EpochQueryAdvance advance_epoch_query_signature(const EpochQuerySignature &bound,
    const CanonicalLowerDelta &delta, const PreparedLowerPtr &current_prepared,
    const BridgeAwareCubic::SerializedXY &phase, EpochLimits limits,
    BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
    const QuerySignatureCancel &cancel = {}, QueryCache *cache = nullptr);
bool matches_epoch_query_signature(const EpochQuerySignature &bound,
    const PreparedLowerPtr &current_prepared, const BridgeAwareCubic::SerializedXY &phase,
    const ExtrusionPath &upper, double upper_print_z, double xy_tolerance_mm,
    BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
    const QuerySignatureCancel &cancel = {}, QueryCache *cache = nullptr);

struct QueryCacheData;
struct QueryCacheStatistics {
    size_t hits = 0, misses = 0, evictions = 0, resident_units = 0;
};

// Optional, caller-owned acceleration for sequential queries on one immutable
// prepared snapshot. Exact computed transverse/reserve values identify masks;
// line-contact witnesses are always recomputed. Capacity includes an entry
// unit even for empty masks. Entries are evicted before they can prevent an
// otherwise admissible query. Destroy this cache before replacing its snapshot.
class QueryCache {
public:
    explicit QueryCache(PreparedLowerPtr prepared, size_t capacity_units = 8192);
    ~QueryCache();
    QueryCache(const QueryCache &) = delete;
    QueryCache &operator=(const QueryCache &) = delete;
    QueryCacheStatistics statistics() const;
private:
    std::unique_ptr<QueryCacheData> m_data;
    friend PathContact footprint(const PreparedLower &, const ExtrusionPath &, double, double,
                                 FootprintMode, BridgeAwareCubicValidation::WorkBudget *, QueryCache *);
    friend SupportQuery footprint_for_support(const PreparedLower &, const ExtrusionPath &, double, double,
                                              BridgeAwareCubicValidation::WorkBudget *, QueryCache *);
    friend PathContact physical_sections(const PreparedLower &, BridgeAwareCubicValidation::WorkBudget *, QueryCache *);
    friend QuerySignaturePtr capture_query_signature(const PreparedLower &, const ExtrusionPath &, double, double,
        BridgeAwareCubicValidation::WorkBudget *, size_t, const QuerySignatureCancel &, QueryCache *);
    friend bool matches_query_signature(const QuerySignature &, const PreparedLower &, const ExtrusionPath &, double, double,
        BridgeAwareCubicValidation::WorkBudget *, const QuerySignatureCancel &, QueryCache *);
    friend bool matches_query_signature_with_contact(const QuerySignature &, const PreparedLower &, const ExtrusionPath &,
        double, double, const PathContact &, BridgeAwareCubicValidation::WorkBudget *, const QuerySignatureCancel &, QueryCache *);
    friend CanonicalLowerEpochPtr capture_lower_epoch(const PreparedLowerPtr &,
        const BridgeAwareCubic::SerializedXY &, EpochLimits, BridgeAwareCubicValidation::WorkBudget *,
        const QuerySignatureCancel &, QueryCache *);
    friend EpochQuerySignaturePtr capture_epoch_query_signature(const CanonicalLowerEpochPtr &,
        const PreparedLowerPtr &, const BridgeAwareCubic::SerializedXY &, const ExtrusionPath &, double, double,
        BridgeAwareCubicValidation::WorkBudget *, size_t, const QuerySignatureCancel &, QueryCache *);
    friend EpochQueryAdvance advance_epoch_query_signature(const EpochQuerySignature &, const CanonicalLowerDelta &,
        const PreparedLowerPtr &, const BridgeAwareCubic::SerializedXY &, EpochLimits,
        BridgeAwareCubicValidation::WorkBudget *, const QuerySignatureCancel &, QueryCache *);
    friend bool matches_epoch_query_signature(const EpochQuerySignature &, const PreparedLowerPtr &,
        const BridgeAwareCubic::SerializedXY &, const ExtrusionPath &, double, double,
        BridgeAwareCubicValidation::WorkBudget *, const QuerySignatureCancel &, QueryCache *);
};

// The upper elevation and float height must exactly match the prepared key.
// Upper path validity, width, direction and XY tolerance remain per query.
PathContact footprint(const PreparedLower &deposited,
                      const ExtrusionPath &upper,
                      double upper_print_z,
                      double xy_tolerance_mm,
                      FootprintMode mode = FootprintMode::Full,
                      BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                      QueryCache *cache = nullptr);

// ContactsOnly query with an optional exact whole-path proof before exhaustive
// contact generation. The shortcut requires zero XY tolerance and an exact
// finite collinear lower witness covering every original positive segment.
// Failed probes allocate no masks and retain the ordinary exhaustive output;
// the spent probe work remains charged. Preparation, query guards and shared
// limits are identical to footprint(). A successful certificate contains only
// full-span line contacts, not physical material usable for cell boundaries.
// Model-cell obligations still require their full physical geometry check even
// when entire_path_supported is true (including purely linear lower contact).
SupportQuery footprint_for_support(const PreparedLower &deposited,
                                   const ExtrusionPath &upper,
                                   double upper_print_z,
                                   double xy_tolerance_mm,
                                   BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                                   QueryCache *cache = nullptr);

// Exact Full-mode physical material only, without a redundant upper-path
// contact query. Status and plane come from the prepared key; no segments are
// returned, so this is not a path-contact certificate. Supply the live query
// cache, if any, to include its residency in the same allocation allowance.
PathContact physical_sections(const PreparedLower &prepared,
                              BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                              QueryCache *cache = nullptr);

// All physical material in one result belongs to one common plane. Heights
// differing only by float representation error are snapped at a tangent plane;
// xy_tolerance_mm is never used to bridge a vertical separation. The supplied
// paths must already have been emitted before upper, and are not retained.
// Positive tolerance requires line events to survive endpoint rounding: they
// must be interior to an upper segment, and zero-width collinearity alone is
// insufficient. Zero tolerance exposes the exact geometric line-contact rule.
// Piecewise straight, finite stadium/circular sections are supported. Bounds
// and work exhaustion fail closed. Upper w<h and fitted arcs fail closed;
// valid lower gap-fill beads with w<h and stationary serialized deposition
// contribute no support evidence and cannot select the contact plane.
// ContactsOnly preserves every contact certificate but omits physical wall
// polygons/lines; use only when those outputs will not participate in cells.
// A shared budget bounds a complete collection/check pass across all paths.
PathContact footprint(const std::vector<Bead> &deposited,
                      const ExtrusionPath &upper,
                      double upper_print_z,
                      double xy_tolerance_mm,
                      FootprintMode mode = FootprintMode::Full,
                      BridgeAwareCubicValidation::WorkBudget *budget = nullptr);

}} // namespace Slic3r::BridgeAwareCubicContact

#endif
