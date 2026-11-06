#ifndef slic3r_BridgeAwareCubicFailureReplay_hpp_
#define slic3r_BridgeAwareCubicFailureReplay_hpp_

#include "BridgeAwareCubic.hpp"
#include "BridgeAwareCubicSerialization.hpp"
#include "BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <map>
#include <new>
#include <optional>

namespace Slic3r { namespace BridgeAwareCubic {

// Run-local search hints, never certificates. A hint only chooses which full
// model path to verify first after actual/frozen target equality. Its previous
// outcome cannot reject a trial; current deposition must fail a fresh proof.
class ModelFailureReplay {
public:
    using Key = std::pair<size_t, CellId>; // Region namespaces the lattice ID.
    enum class Kind { Run, Cell, RunAndCell };
    struct Hint {
        size_t frozen_target = 0, path = 0;
        SerializedXY phase;
        Kind kind = Kind::Run;
    };
    struct Probe {
        size_t target = 0, path = 0; // Index in the complete current target view.
        SerializedXY phase;

        bool includes_phase(const SerializedXY &other) const noexcept
        {
            const auto same = [](const Vec2d &a, const Vec2d &b) { return a.x() == b.x() && a.y() == b.y(); };
            return same(phase.shift, other.shift) && same(phase.nozzle_offset, other.nozzle_offset) &&
                same(phase.plate_origin, other.plate_origin);
        }
    };

    explicit ModelFailureReplay(size_t capacity = 4096) : m_capacity(std::min(capacity, size_t(4096))) {}

    std::optional<Hint> find(const Key &key) const
    {
        const auto found = m_hints.find(key);
        return found == m_hints.end() ? std::nullopt : std::optional<Hint>(found->second);
    }
    void erase(const Key &key) { m_hints.erase(key); }
    size_t size() const noexcept { return m_hints.size(); }
    bool remember(const Key &key, const Hint &hint)
    {
        const auto found = m_hints.find(key);
        if (found != m_hints.end()) { found->second = hint; return true; }
        if (m_hints.size() == m_capacity) return false;
        try { return m_hints.emplace(key, hint).second; }
        catch (const std::bad_alloc &) { return false; }
    }

    // Mapping is optional work but shares the verification allowance. Failure
    // to resolve falls back to the full check without refunding performed work.
    template<class PathCount, class Cancel>
    static std::optional<Probe> resolve(const Hint &hint, const std::vector<size_t> &frozen_indices,
                                        PathCount path_count, BridgeAwareCubicValidation::WorkBudget &budget, Cancel cancel)
    {
        for (size_t index = 0; index < frozen_indices.size(); ++index) {
            if ((index & 63) == 0) cancel();
            if (budget.remaining_work == 0) return {};
            --budget.remaining_work;
            ++budget.charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)];
            if (frozen_indices[index] == hint.frozen_target) {
                if (hint.path >= path_count(index)) return {};
                return Probe{index, hint.path, hint.phase};
            }
        }
        return {};
    }

    static std::optional<Kind> failure_kind(const BridgeAwareCubicValidation::Result &result)
    {
        if (!result.input_valid || !result.completed || result.resource_limited) return {};
        const bool run = std::any_of(result.unsupported_runs.begin(), result.unsupported_runs.end(), [](const auto &entry) {
            return entry.kind != BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge || entry.exceeds_gap;
        });
        const bool cell = std::any_of(result.void_cells.begin(), result.void_cells.end(), [](const auto &entry) {
            return !entry.closed || entry.exceeds_gap;
        });
        if (!run && !cell) return {};
        return run && cell ? Kind::RunAndCell : run ? Kind::Run : Kind::Cell;
    }

private:
    size_t m_capacity;
    std::map<Key, Hint> m_hints;
};

}} // namespace Slic3r::BridgeAwareCubic

#endif
