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
// legacy added-mask tolerance can classify short ordinary fragments. Seal must
// intrinsically establish equivalent full-path classification and independent
// complete-owner coverage, otherwise decline the WHOLE emitter. The complete
// raw partition source table includes rows which emitted no output locally.
// Long bulk crossing a short added mask is admissible when full coverage is
// provably impossible; a crossing alone must not force fallback.
// Production binds exact broad-reference clipping before surface batches.
// Without that optional reference, synthetic callers must establish strict
// interior clipping locality relative to EmissionReference, or decline.
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
