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
