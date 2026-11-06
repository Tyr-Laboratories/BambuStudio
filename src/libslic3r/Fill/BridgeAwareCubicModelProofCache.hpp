#ifndef slic3r_BridgeAwareCubicModelProofCache_hpp_
#define slic3r_BridgeAwareCubicModelProofCache_hpp_

#include "BridgeAwareCubicSerialization.hpp"
#include "BridgeAwareCubicValidation.hpp"
#include "../ExtrusionEntity.hpp"

#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <typeinfo>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubic {

// Optional coarsening-local facts about complete model-layer proofs. The
// immutable frozen_context must outlive this cache and identify the full
// model inputs (including all layer topology demands). Plan identity is not
// part of a physical model proof. Construction and final proofs stay separate.
class ModelLayerProofCache {
public:
    static constexpr size_t max_phases = 64;
    struct Coverage { size_t layer; std::vector<size_t> targets; };
    using Sources = std::vector<const ExtrusionPath *>;
    using Cancel = std::function<void()>;
    struct Statistics {
        size_t hits = 0, misses = 0, publications = 0, evictions = 0;
        size_t vertices = 0, items = 0;
    };
private:
    struct StoredPath { ExtrusionPath path; bool reverse; };
    struct Snapshot {
        double lower_z, upper_z;
        std::vector<StoredPath> paths;
        std::vector<bool> phases;
        size_t vertices = 0, items = 0;
    };
    struct LayerState {
        std::vector<size_t> targets;
        std::unique_ptr<Snapshot> snapshot;
        size_t version = 0;
    };
    const void *m_context;
    std::vector<SerializedXY> m_phases;
    std::map<size_t, LayerState> m_layers;
    size_t m_max_vertices, m_max_items, m_staged_vertices = 0, m_staged_items = 0;
    Statistics m_stats;
    bool m_enabled = true;

    static bool same_phase(const SerializedXY &a, const SerializedXY &b) {
        return a.shift == b.shift && a.nozzle_offset == b.nozzle_offset && a.plate_origin == b.plate_origin;
    }
    size_t phase_index(const SerializedXY &phase, BridgeAwareCubicValidation::WorkBudget *budget) const {
        for (size_t i = 0; i < m_phases.size(); ++i) {
            if (!spend(budget, 6)) return m_phases.size();
            if (same_phase(m_phases[i], phase)) return i;
        }
        return m_phases.size();
    }
    struct Eligibility { LayerState *entry = nullptr; size_t phase = 0; };
    Eligibility eligible(const void *context, size_t layer, const std::vector<size_t> &targets,
                         const SerializedXY &phase, double lower_z, double upper_z,
                         BridgeAwareCubicValidation::WorkBudget *budget, const Cancel &cancel) {
        if (!m_enabled || !spend(budget, 1) || context != m_context ||
            !std::isfinite(lower_z) || !std::isfinite(upper_z)) return {};
        const size_t output_phase = phase_index(phase, budget);
        if (output_phase == m_phases.size()) return {};
        auto found = m_layers.find(layer);
        if (found == m_layers.end() || targets.empty() || found->second.targets.size() != targets.size()) return {};
        for (size_t i = 0; i < targets.size(); ++i) {
            if (cancel && i % 256 == 0) cancel();
            if (!spend(budget, 1) || found->second.targets[i] != targets[i]) return {};
        }
        return {&found->second, output_phase};
    }
    static bool spend(BridgeAwareCubicValidation::WorkBudget *budget, size_t amount) {
        if (budget == nullptr) return true;
        if (amount > budget->remaining_work) return false;
        budget->remaining_work -= amount;
        budget->charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += amount;
        return true;
    }
    static bool same_sources(const Snapshot &snapshot, double lower_z, double upper_z,
                             const Sources &sources, BridgeAwareCubicValidation::WorkBudget *budget,
                             const Cancel &cancel) {
        if (cancel) cancel();
        if (snapshot.lower_z != lower_z || snapshot.upper_z != upper_z || snapshot.paths.size() != sources.size()) return false;
        for (size_t i = 0; i < sources.size(); ++i) {
            if (cancel) cancel();
            const ExtrusionPath *live = sources[i];
            if (live == nullptr || typeid(*live) != typeid(ExtrusionPath) || !live->polyline.fitting_result.empty() ||
                !spend(budget, live->polyline.size()) || !spend(budget, 1)) return false;
            const StoredPath &saved = snapshot.paths[i];
            const ExtrusionPath &path = saved.path;
            if (path.polyline.points != live->polyline.points || path.role() != live->role() ||
                path.width != live->width || path.height != live->height || path.mm3_per_mm != live->mm3_per_mm ||
                path.is_force_no_extrusion() != live->is_force_no_extrusion() || saved.reverse != live->can_reverse() ||
                path.overhang_degree != live->overhang_degree || path.curve_degree != live->curve_degree ||
                path.smooth_speed != live->smooth_speed || path.get_customize_flag() != live->get_customize_flag() ||
                path.get_cooling_node() != live->get_cooling_node()) return false;
        }
        if (cancel) cancel();
        return true;
    }
    void discard(LayerState &layer) {
        if (layer.snapshot) {
            m_stats.vertices -= layer.snapshot->vertices;
            m_stats.items -= layer.snapshot->items;
            layer.snapshot.reset();
            ++layer.version;
            ++m_stats.evictions;
        }
    }
    void release_stage(const Snapshot &snapshot) {
        m_staged_vertices -= snapshot.vertices;
        m_staged_items -= snapshot.items;
    }
public:
    // Transactions borrow this cache, Sources (including its pointed-to paths)
    // and WorkBudget. All must remain alive and stationary until destruction.
    class Proof {
        friend class ModelLayerProofCache;
        ModelLayerProofCache *m_owner = nullptr;
        LayerState *m_layer = nullptr;
        const Sources *m_sources = nullptr;
        std::unique_ptr<Snapshot> m_snapshot;
        BridgeAwareCubicValidation::WorkBudget *m_budget = nullptr;
        Cancel m_cancel;
        size_t m_version = 0, m_phase = 0, m_next = 0;
        bool m_valid = true;
        void release() {
            if (m_owner && m_snapshot) m_owner->release_stage(*m_snapshot);
            m_snapshot.reset();
            m_owner = nullptr;
        }
    public:
        Proof() = default;
        Proof(const Proof &) = delete;
        Proof &operator=(const Proof &) = delete;
        Proof(Proof &&other) noexcept { *this = std::move(other); }
        Proof &operator=(Proof &&other) noexcept {
            if (this != &other) {
                release();
                m_owner = other.m_owner; m_layer = other.m_layer; m_sources = other.m_sources;
                m_snapshot = std::move(other.m_snapshot); m_budget = other.m_budget; m_cancel = std::move(other.m_cancel);
                m_version = other.m_version; m_phase = other.m_phase; m_next = other.m_next; m_valid = other.m_valid;
                other.m_owner = nullptr;
            }
            return *this;
        }
        ~Proof() { release(); }
        void record(size_t target, const BridgeAwareCubicValidation::Result &result) {
            if (!m_owner) return;
            if (m_next >= m_layer->targets.size() || m_layer->targets[m_next] != target || !result.passed()) m_valid = false;
            ++m_next;
        }
        bool commit() {
            if (!m_owner || !m_valid || m_next != m_layer->targets.size() || m_version != m_layer->version ||
                !same_sources(*m_snapshot, m_snapshot->lower_z, m_snapshot->upper_z, *m_sources, m_budget, m_cancel)) return false;
            // Preserve other phases only if they proved this exact source
            // version. A changed source atomically starts with no coverage.
            if (m_layer->snapshot && same_sources(*m_layer->snapshot, m_snapshot->lower_z, m_snapshot->upper_z,
                                                 *m_sources, m_budget, m_cancel))
                for (size_t i = 0; i < m_snapshot->phases.size(); ++i)
                    m_snapshot->phases[i] = m_layer->snapshot->phases[i];
            m_snapshot->phases[m_phase] = true;
            m_owner->discard(*m_layer);
            m_owner->release_stage(*m_snapshot);
            m_owner->m_stats.vertices += m_snapshot->vertices;
            m_owner->m_stats.items += m_snapshot->items;
            m_layer->snapshot = std::move(m_snapshot);
            ++m_layer->version;
            ++m_owner->m_stats.publications;
            m_owner = nullptr;
            return true;
        }
    };

    ModelLayerProofCache(const void *frozen_context, const std::vector<Coverage> &coverage,
                        const std::vector<SerializedXY> &phases, size_t max_vertices = 200000, size_t max_items = 20000)
        : m_context(frozen_context), m_max_vertices(max_vertices), m_max_items(max_items) {
        m_enabled = false;
        if (frozen_context == nullptr || phases.empty() || phases.size() > max_phases || phases.size() > max_items) return;
        size_t items = phases.size();
        // Admission precedes every owned fixed-key copy, including callers
        // other than the slicing adapter which also pre-counts its temporaries.
        for (const auto &layer : coverage) {
            if (layer.targets.size() >= max_items || items > max_items - layer.targets.size() - 1) return;
            items += layer.targets.size() + 1;
        }
        m_phases = phases;
        for (const auto &layer : coverage)
            if (!m_layers.emplace(layer.layer, LayerState{layer.targets, {}, 0}).second) return;
        m_stats.items = items;
        m_enabled = true;
    }
    ModelLayerProofCache(const ModelLayerProofCache &) = delete;
    ModelLayerProofCache &operator=(const ModelLayerProofCache &) = delete;
    ModelLayerProofCache(ModelLayerProofCache &&) = delete;
    ModelLayerProofCache &operator=(ModelLayerProofCache &&) = delete;
    Statistics statistics() const { return m_stats; }

    bool matches(const void *context, size_t layer, const std::vector<size_t> &targets, const SerializedXY &phase,
                 double lower_z, double upper_z, const Sources &sources,
                 BridgeAwareCubicValidation::WorkBudget *budget = nullptr, const Cancel &cancel = {}) {
        if (cancel) cancel();
        const auto allowed = eligible(context, layer, targets, phase, lower_z, upper_z, budget, cancel);
        const bool hit = allowed.entry && allowed.entry->snapshot && allowed.entry->snapshot->phases[allowed.phase] &&
            same_sources(*allowed.entry->snapshot, lower_z, upper_z, sources, budget, cancel);
        hit ? ++m_stats.hits : ++m_stats.misses;
        return hit;
    }

    Proof begin(const void *context, size_t layer, const std::vector<size_t> &targets, const SerializedXY &phase,
                double lower_z, double upper_z, const Sources &sources,
                BridgeAwareCubicValidation::WorkBudget *budget = nullptr, const Cancel &cancel = {}) {
        if (cancel) cancel();
        const auto allowed = eligible(context, layer, targets, phase, lower_z, upper_z, budget, cancel);
        LayerState *entry = allowed.entry;
        if (!entry || sources.size() >= m_max_items || m_phases.size() > m_max_items - sources.size() - 1) return {};
        const size_t items = sources.size() + m_phases.size() + 1;
        size_t vertices = 0;
        for (const ExtrusionPath *path : sources) {
            if (cancel) cancel();
            if (!path || typeid(*path) != typeid(ExtrusionPath) || !path->polyline.fitting_result.empty() || path->polyline.size() > m_max_vertices - vertices ||
                !spend(budget, path->polyline.size()) || !spend(budget, 1)) return {};
            vertices += path->polyline.size();
        }
        const auto fits = [&] {
            return vertices <= m_max_vertices - m_stats.vertices - m_staged_vertices &&
                items <= m_max_items - m_stats.items - m_staged_items;
        };
        for (auto &candidate : m_layers) {
            if (fits()) break;
            discard(candidate.second);
        }
        if (!fits()) return {};
        Proof proof;
        try {
            auto snapshot = std::make_unique<Snapshot>();
            snapshot->lower_z = lower_z; snapshot->upper_z = upper_z;
            snapshot->vertices = vertices; snapshot->items = items;
            snapshot->phases.resize(m_phases.size(), false);
            snapshot->paths.reserve(sources.size());
            for (const ExtrusionPath *path : sources) {
                if (cancel) cancel();
                snapshot->paths.push_back({*path, path->can_reverse()});
            }
            proof.m_cancel = cancel;
            proof.m_owner = this; proof.m_layer = entry; proof.m_sources = &sources; proof.m_budget = budget;
            proof.m_version = entry->version; proof.m_phase = allowed.phase;
            proof.m_snapshot = std::move(snapshot);
            m_staged_vertices += vertices; m_staged_items += items;
        } catch (const std::bad_alloc &) { return {}; }
        return proof;
    }
};

}} // namespace Slic3r::BridgeAwareCubic
#endif
