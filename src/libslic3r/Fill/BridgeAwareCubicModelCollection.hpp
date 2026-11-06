#ifndef slic3r_BridgeAwareCubicModelCollection_hpp_
#define slic3r_BridgeAwareCubicModelCollection_hpp_

#include <cstddef>
#include <functional>
#include <memory>
#include <set>

namespace Slic3r {
class PrintObject;
namespace BridgeAwareCubicValidation { struct WorkBudget; }
namespace BridgeAwareCubic {

// Exact source-frame inputs to collect_targets(false, false), NOT a support
// certificate. The owner keeps the corresponding complete model Targets
// immutable and live. A Match permits reusing those Targets for checked_layers;
// every physical check remains current. Different/Unavailable must fall back to
// the original collector, never directly accept or reject a coarsening trial.
class ModelCollectionInputs final {
public:
    using Cancel = std::function<void()>;
    enum class Match { Equal, Different, Unavailable };
    struct Usage { size_t bytes = 0, vertices = 0, items = 0; };
    struct Limits {
        // Aggregate caps: BOTH the existing PreparedModelTargets storage and
        // this identity (including staged capacity) must fit. The caller gives
        // its actual already-retained capacities in other_retained. This does
        // not allocate another pool or subtract from emission storage.
        size_t bytes = 512 * 1024;
        size_t vertices = 200000;
        size_t items = 20000;
        Usage other_retained;
    };

    // Capture only while the same source state used by the complete collector
    // is still live. Owns ordered policy/surface inputs for every layer and the
    // complete included model-path event stream, including unselected paths.
    // Unsupported/derived/fitted trees, caps or optional allocation failure
    // return null. Cancellation, including callback-shaped bad_alloc, escapes.
    static std::unique_ptr<const ModelCollectionInputs> capture(
        const PrintObject &, const Limits &, BridgeAwareCubicValidation::WorkBudget * = nullptr,
        const Cancel & = {});
    static std::unique_ptr<const ModelCollectionInputs> capture(
        const PrintObject &object, BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
        const Cancel &cancel = {})
    { return capture(object, Limits{}, budget, cancel); }

    // Rechecks the complete live policy/surface context plus every model path
    // on each requested layer, including layers with no collected Targets.
    // Tree traversal/inclusion/free-ended membership follow the collector.
    // No pointer/hash-only equality, and no concurrent mutation is permitted.
    // The same consumed budget MUST continue into collection on a miss.
    Match matches(const PrintObject &, const std::set<size_t> &checked_layers,
                  BridgeAwareCubicValidation::WorkBudget * = nullptr, const Cancel & = {}) const;
    Usage owned_usage() const noexcept;
    // Capture high water includes its temporary free-membership index.
    Usage capture_peak_usage() const noexcept;

    ~ModelCollectionInputs();
    ModelCollectionInputs(const ModelCollectionInputs &) = delete;
    ModelCollectionInputs &operator=(const ModelCollectionInputs &) = delete;
private:
    struct Impl;
    explicit ModelCollectionInputs(std::unique_ptr<const Impl>);
    std::unique_ptr<const Impl> m_impl;
};

} // namespace BridgeAwareCubic
} // namespace Slic3r
#endif
