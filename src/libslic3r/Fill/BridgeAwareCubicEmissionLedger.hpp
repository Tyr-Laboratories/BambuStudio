#ifndef slic3r_BridgeAwareCubicEmissionLedger_hpp_
#define slic3r_BridgeAwareCubicEmissionLedger_hpp_

#include "BridgeAwareCubic.hpp"
#include "../ExtrusionEntity.hpp"

#include <functional>
#include <memory>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubic {

// These are exact retained Plan::immutable_identity() tokens, ordered by the
// storage-region namespace. Capture every plan which can supply this emitter's
// source or attachment records. Tokens alone never identify emitted paths.
struct EmissionPlanBinding {
    size_t storage_region = 0;
    std::shared_ptr<const void> identity;
};

struct EmissionPathView {
    const ExtrusionPath *path = nullptr;
    bool free_ended = true;
};

struct EmissionTransition {
    size_t storage_region = 0;
    LayerAttachment attachment;
};

struct EmissionReference {
    // The old collector clips added and independently owned axes to this
    // exact object bounding box before constructing its classification masks.
    // Raw tagged axes alone do not establish positive coverage at its edge.
    BoundingBox classification_bounds;
    double tolerance_mm = 0.;
};

struct EmissionRecord {
    ExtrusionPath path;
    bool construction = false;
    // Exact legacy ordinary_full_path result, kept separate from an
    // attachment-only construction classification and its selection payload.
    bool mask_full_path = false;
    // Independent owners covering the complete FINAL emitted source.
    // Jointly covering finite intervals never become interchangeable owners.
    std::vector<CellId> partition_owners;
    size_t source_index = 0;
    // Preserve application order and separate terminal owner namespaces.
    std::vector<EmissionTransition> transitions;
    // Sorted unique indices into the complete active table, preserving the
    // old collector's owner/selection order independently of replay order.
    std::vector<size_t> attachment_indices;
};

struct EmissionLedgerLimits {
    // The integration carves a deterministic per-layer quota from one fixed
    // object allowance, then divides that quota among emitting regions. All
    // owned arrays/capacities, retained tokens and staging count against it.
    size_t capacity_bytes = 0;
    size_t retained_bytes = 0;
};

using EmissionCancel = std::function<void()>;
class EmissionLedger;
using EmissionLedgerPtr = std::shared_ptr<const EmissionLedger>;

// A raw final-survival record has no construction or support verdict. Source
// indices are local lookup indices, never persistent emitted-path identities.
struct EmissionSnapshotRecord {
    ExtrusionPath path;
    size_t source_index = 0;
    std::vector<EmissionTransition> transitions;
};

// Classification owns only derived values. Paths and carried provenance stay
// in the one immutable snapshot retained by its projection.
struct EmissionClassification {
    bool construction = false;
    bool mask_full_path = false;
    std::vector<CellId> partition_owners;
    std::vector<size_t> attachment_indices;
};

struct EmissionMatch {
    size_t before = 0, after = 0;
    bool provenance_changed = false;
};

class EmissionSnapshot;
class EmissionProjection;
class EmissionDelta;
using EmissionSnapshotPtr = std::shared_ptr<const EmissionSnapshot>;
using EmissionProjectionPtr = std::shared_ptr<const EmissionProjection>;
using EmissionDeltaPtr = std::shared_ptr<const EmissionDelta>;

// These operations return unavailable on optional admission/allocation failure;
// a missing value is never an empty complete result or a passing certificate.
// They charge the distinct snapshot arguments, borrowed reference capacities,
// output capacities and simultaneous scratch. limits.retained_bytes describes
// OTHER live storage only, e.g. a retained base projection's owned_bytes().
// Cancellation propagates and all performed work remains charged on fallback.
// The reference must be the exact complete broad-reference batch for the
// snapshot's emitter Plan, elevation, box and complete partition source table.
EmissionProjectionPtr project_emissions(const EmissionSnapshotPtr &, ReferenceLineBatch &&,
    EmissionLedgerLimits, BridgeAwareCubicValidation::WorkBudget * = nullptr,
    const EmissionCancel &cancel = {}, size_t additional_scratch_bytes = 0);

// Exact SOURCE-FRAME final-emission changes, not serialized Contact-section
// changes. Output phases, canonical merged lower axes, current policy/selection
// and complete-cell proof remain obligations of the caller. Equal physical
// records are paired in after order with the first unmatched equal before
// record; duplicate multiplicity is preserved. Reversed paths do not match.
// Z changes make every record physically removed/added. Different emitter
// namespaces are unavailable, not a cross-region identity correspondence.
EmissionDeltaPtr compare_emissions(const EmissionSnapshotPtr &before,
    const EmissionSnapshotPtr &after, EmissionLedgerLimits,
    BridgeAwareCubicValidation::WorkBudget * = nullptr, const EmissionCancel &cancel = {});

struct EmissionSnapshotState;
class EmissionSnapshot {
public:
    ~EmissionSnapshot();
    EmissionSnapshot(const EmissionSnapshot &) = delete;
    EmissionSnapshot &operator=(const EmissionSnapshot &) = delete;
    bool matches(size_t, double, const EmissionReference &,
                 const std::vector<EmissionPlanBinding> &, const std::vector<EmissionPathView> &,
                 BridgeAwareCubicValidation::WorkBudget * = nullptr,
                 const EmissionCancel &cancel = {}) const;
    size_t emitting_region() const noexcept;
    double print_z() const noexcept;
    const EmissionReference &reference() const noexcept;
    const std::vector<EmissionPlanBinding> &plan_bindings() const noexcept;
    const std::vector<EmissionSnapshotRecord> &records() const noexcept;
    const std::vector<LineSource> &sources() const noexcept;
    const std::vector<EmissionTransition> &active_attachments() const noexcept;
    // Snapshot storage is wholly owned, so these two counts are equal. Counts
    // include capacities, wrapper/control overhead and bounded Plan nonces.
    size_t owned_bytes() const noexcept;
    size_t resident_bytes() const noexcept;
private:
    explicit EmissionSnapshot(std::unique_ptr<EmissionSnapshotState>);
    std::unique_ptr<EmissionSnapshotState> m_state;
    friend class EmissionLedgerBuilder;
    friend EmissionProjectionPtr project_emissions(const EmissionSnapshotPtr &, ReferenceLineBatch &&,
        EmissionLedgerLimits, BridgeAwareCubicValidation::WorkBudget *, const EmissionCancel &, size_t);
    friend EmissionDeltaPtr compare_emissions(const EmissionSnapshotPtr &, const EmissionSnapshotPtr &,
        EmissionLedgerLimits, BridgeAwareCubicValidation::WorkBudget *, const EmissionCancel &);
};

struct EmissionProjectionState;
class EmissionProjection {
public:
    ~EmissionProjection();
    EmissionProjection(const EmissionProjection &) = delete;
    EmissionProjection &operator=(const EmissionProjection &) = delete;
    const EmissionSnapshotPtr &snapshot() const noexcept;
    const std::vector<EmissionClassification> &records() const noexcept;
    size_t owned_bytes() const noexcept; // Derived allocation only.
    size_t resident_bytes() const noexcept; // Owned + retained snapshot, once.
private:
    explicit EmissionProjection(std::unique_ptr<EmissionProjectionState>);
    std::unique_ptr<EmissionProjectionState> m_state;
    friend EmissionProjectionPtr project_emissions(const EmissionSnapshotPtr &, ReferenceLineBatch &&,
        EmissionLedgerLimits, BridgeAwareCubicValidation::WorkBudget *, const EmissionCancel &, size_t);
};

struct EmissionDeltaState;
class EmissionDelta {
public:
    ~EmissionDelta();
    EmissionDelta(const EmissionDelta &) = delete;
    EmissionDelta &operator=(const EmissionDelta &) = delete;
    const EmissionSnapshotPtr &before() const noexcept;
    const EmissionSnapshotPtr &after() const noexcept;
    const std::vector<EmissionMatch> &matches() const noexcept; // Ordered by after index.
    const std::vector<size_t> &removed() const noexcept; // Ordered before indices.
    const std::vector<size_t> &added() const noexcept; // Ordered after indices.
    // Relative order of surviving matched physical records changed.
    bool order_changed() const noexcept;
    // Plan bindings, Z/reference, complete partition table or complete active
    // attachment table changed. Unchanged path pairs cannot erase this global
    // classification invalidation. Source-index renumbering alone is ignored.
    bool classification_context_changed() const noexcept;
    size_t owned_bytes() const noexcept; // Correspondence allocation only.
    size_t resident_bytes() const noexcept; // Owned + each distinct snapshot once.
private:
    explicit EmissionDelta(std::unique_ptr<EmissionDeltaState>);
    std::unique_ptr<EmissionDeltaState> m_state;
    friend EmissionDeltaPtr compare_emissions(const EmissionSnapshotPtr &, const EmissionSnapshotPtr &,
        EmissionLedgerLimits, BridgeAwareCubicValidation::WorkBudget *, const EmissionCancel &);
};

// Source-frame provenance, never a support certificate or serialized fragment
// proof. Construct finite owner fragment endpoints in this original frame
// before any later pointwise output transform. Do not interpolate transformed
// canonical endpoints to reconstruct an interior owner boundary.
struct EmissionLedgerState;
class EmissionLedger {
public:
    ~EmissionLedger();
    EmissionLedger(const EmissionLedger &) = delete;
    EmissionLedger &operator=(const EmissionLedger &) = delete;
    bool matches(size_t, double, const EmissionReference &,
                 const std::vector<EmissionPlanBinding> &,
                 const std::vector<EmissionPathView> &,
                 BridgeAwareCubicValidation::WorkBudget * = nullptr,
                 const EmissionCancel &cancel = {}) const;
    const std::vector<EmissionRecord> &records() const noexcept;
    const std::vector<LineSource> &sources() const noexcept;
    size_t resident_bytes() const noexcept;
private:
    explicit EmissionLedger(std::unique_ptr<EmissionLedgerState>);
    std::unique_ptr<EmissionLedgerState> m_state;
    friend class EmissionLedgerBuilder;
};

// One builder per emitter during one Layer::make_fills call. Raw path pointers
// are transient event handles only; seal owns the exact surviving path/flow
// values and lookup compares live values plus current Plan bindings. Binding
// is the base fill_surface_extrusion ordinal mapping after is_valid filtering,
// not a geometric search. No pointer-only lookup is permitted after seal.
//
// First eligible scope excludes saturated solids, closures, fitted/derived
// paths and unsupported tree shapes. A Bulk source is NOT automatic exclusion:
// legacy added-mask tolerance can classify short ordinary fragments. Eager
// seal or a later lazy projection must intrinsically establish equivalent
// full-path classification and independent complete-owner coverage, otherwise
// decline the WHOLE emitter's classification. Raw survival remains distinct.
// The complete
// raw partition source table includes rows which emitted no output locally.
// Long bulk crossing a short added mask is admissible when full coverage is
// provably impossible; a crossing alone must not force fallback.
// Production seals a raw snapshot after surface batches and requests the exact
// broad reference only at lazy projection. The eager bind_reference/seal pair
// remains available for compatibility. Eager synthetic callers without that
// reference must establish strict interior clipping locality or decline.
// The reference box and tolerance are owned
// exact inputs and are checked again on lookup, not inferred from Plan tokens.
// Repeated fill batches can carry the same complete raw partition table:
// canonical rows must be deduplicated exactly or the whole emitter declined.
// Duplicate rows must never increase covered length or owner alternatives.
//
// The constructor receives the COMPLETE ordered active attachment table from
// every retained storage Plan. Actual replay events alone are insufficient:
// the old collector also includes alternate chains reaching the final path.
// This first scope declines an emitter if that applicable-index union differs
// from its recorded ordered replay. The complete table is owned and bounded;
// live Plan bindings guard its immutable source on subsequent matches.
//
// The caller seals only at END of apply_layer_paths, after both actual
// attachment replays, closure handling and stationary removal. The final view
// must contain every surviving active internal-infill leaf in traversal order,
// including unknown sources, so missing or partial tags cannot omit a target.
// A declined, capped or failed builder falls back for the complete emitter.
// Cancellation propagates; optional failures never replenish shared work.
class EmissionLedgerBuilder {
public:
    EmissionLedgerBuilder(size_t, double, const EmissionReference &,
                          const std::vector<EmissionPlanBinding> &,
                          const std::vector<EmissionTransition> &,
                          EmissionLedgerLimits,
                          BridgeAwareCubicValidation::WorkBudget * = nullptr,
                          const EmissionCancel &cancel = {});
    ~EmissionLedgerBuilder();
    EmissionLedgerBuilder(const EmissionLedgerBuilder &) = delete;
    EmissionLedgerBuilder &operator=(const EmissionLedgerBuilder &) = delete;
    EmissionLedgerBuilder(EmissionLedgerBuilder &&) noexcept;
    EmissionLedgerBuilder &operator=(EmissionLedgerBuilder &&) noexcept;

    // Request once, before pending filler/tag buffers exist. Consuming this
    // reference transfers its exact broad geometry without a second copy.
    bool needs_reference() const noexcept;
    BoundingBox reference_bounds() const noexcept;
    bool bind_reference(ReferenceLineBatch &&,
                        BridgeAwareCubicValidation::WorkBudget * = nullptr,
                        const EmissionCancel &cancel = {}, size_t additional_scratch_bytes = 0);

    // The passed batch and child-vector capacities are counted automatically
    // during capture. additional_scratch_bytes counts only OTHER simultaneous
    // optional filler buffers; it must not repeat this batch's storage.
    bool bind_batch(const TaggedLineBatch &, const std::vector<const ExtrusionPath*> &,
                    BridgeAwareCubicValidation::WorkBudget * = nullptr,
                    const EmissionCancel &cancel = {}, size_t additional_scratch_bytes = 0);
    // Called immediately after the real replay updates this bound path. Each
    // transition includes the exact original/replacement, width/height and
    // owner payload returned by that replay, not a flattened inferred union.
    bool applied(const ExtrusionPath &, const std::vector<EmissionTransition> &,
                 BridgeAwareCubicValidation::WorkBudget * = nullptr,
                 const EmissionCancel &cancel = {});
    // Called before an actual bound entity is destroyed. This only tracks a
    // real deletion; it neither authorizes nor performs stationary removal.
    bool erased(const ExtrusionPath &, BridgeAwareCubicValidation::WorkBudget * = nullptr,
                const EmissionCancel &cancel = {});
    void decline() noexcept;
    EmissionLedgerPtr seal(const std::vector<EmissionPathView> &,
                           const std::vector<EmissionPlanBinding> &,
                           BridgeAwareCubicValidation::WorkBudget * = nullptr,
                           const EmissionCancel &cancel = {});
    // Alternative one-shot sealing operation: validate complete final survival
    // and exact context, own the raw values, and discard every event handle.
    // It neither requires broad reference geometry nor runs lineage resolution
    // or analytic classification. Only a later complete projection may supply
    // construction classifications to the collector. Failure consumes/declines
    // the whole builder; no partial snapshot or mutable cache escapes.
    EmissionSnapshotPtr seal_snapshot(const std::vector<EmissionPathView> &,
                           const std::vector<EmissionPlanBinding> &,
                           BridgeAwareCubicValidation::WorkBudget * = nullptr,
                           const EmissionCancel &cancel = {});
    size_t staged_bytes() const noexcept;
    size_t available_scratch_bytes() const noexcept;
    // Fixed diagnostic literal, retained after decline; no logging/allocation.
    const char *decline_reason() const noexcept;
private:
    std::unique_ptr<EmissionLedgerState> m_state;
    EmissionLedgerLimits m_limits;
    const char *m_decline_reason = nullptr;
};

}} // namespace Slic3r::BridgeAwareCubic
#endif
