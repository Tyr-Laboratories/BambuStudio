#ifndef slic3r_BridgeAwareCubicCertificate_hpp_
#define slic3r_BridgeAwareCubicCertificate_hpp_

#include "BridgeAwareCubicSerialization.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

namespace Slic3r {
class PrintObject;
namespace BridgeAwareCubic {

// An exact, owned description of inputs to a deposition proof. Capturing this
// record DOES NOT perform or certify a proof. Only a caller that has completed
// the full serialization preflight and both model/construction proofs may use
// it to retain that successful result.
// The proof owner must keep live state stable from proof through capture, and
// during comparison; this helper does not synchronize concurrent mutations.
class DepositionStateSnapshot final
{
public:
    using Cancel = std::function<void()>;
    struct Limits {
        // Snapshot-owned payload and containers only. Strong immutable Plan
        // handles are count-bounded by items; their shared Impl allocations
        // belong to Plan and are not charged as newly copied snapshot bytes.
        // The owner clears its record before ordinary Plan replacement/reset.
        size_t bytes = 64 * 1024 * 1024;
        size_t items = 4 * 1024 * 1024;
        size_t phases = 64;
        size_t tree_depth = 64;
    };

    // Phases must be freshly resolved by the proof caller, including the full
    // common-nozzle-offset guard. Each component is retained separately.
    // Unsupported state, exhausted admission limits, or optional allocation
    // failure returns null. Cancellation and other errors propagate.
    static std::shared_ptr<const DepositionStateSnapshot> capture(
        const PrintObject &geometry, const PrintObject &caller,
        const std::vector<SerializedXY> &phases, const Cancel &cancel,
        const Limits &limits);
    static std::shared_ptr<const DepositionStateSnapshot> capture(
        const PrintObject &geometry, const PrintObject &caller,
        const std::vector<SerializedXY> &phases, const Cancel &cancel = {})
    { return capture(geometry, caller, phases, cancel, Limits{}); }

    // Performs an allocation-free exact comparison, with cancellation before
    // any possible hit and periodically during traversal. A subset of retained
    // phases may hit; a new phase cannot inherit an old proof.
    bool matches(const PrintObject &geometry, const PrintObject &caller,
                 const std::vector<SerializedXY> &phases, const Cancel &cancel = {}) const;
    // Payload/container capacity accounting, excluding allocator bookkeeping,
    // shared_ptr control blocks and the shared immutable Plan implementations.
    size_t retained_bytes() const noexcept;
    ~DepositionStateSnapshot();

    DepositionStateSnapshot(const DepositionStateSnapshot &) = delete;
    DepositionStateSnapshot &operator=(const DepositionStateSnapshot &) = delete;

private:
    struct Impl;
    explicit DepositionStateSnapshot(std::unique_ptr<const Impl> impl);
    std::unique_ptr<const Impl> m_impl;
};

} // namespace BridgeAwareCubic
} // namespace Slic3r

#endif
