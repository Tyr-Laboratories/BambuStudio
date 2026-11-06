#include "Print.hpp"
#include "Layer.hpp"
#include "ClipperUtils.hpp"
#include "Exception.hpp"
#include "GCodeWriter.hpp"
#include "Fill/BridgeAwareCubicValidation.hpp"
#include "Fill/BridgeAwareCubicSupport.hpp"
#include "Fill/BridgeAwareCubicAttachment.hpp"
#include "Fill/BridgeAwareCubicContact.hpp"
#include "Fill/BridgeAwareCubicClipMask.hpp"
#include "Fill/BridgeAwareCubicDependencies.hpp"
#include "Fill/BridgeAwareCubicSerialization.hpp"
#include "Fill/BridgeAwareCubicModelProofCache.hpp"
#include "Fill/BridgeAwareCubicModelCollection.hpp"
#include "Fill/BridgeAwareCubicSpanProofCache.hpp"
#include "Fill/BridgeAwareCubicEmissionLedger.hpp"
#include "Fill/BridgeAwareCubicFailureReplay.hpp"
#include "Fill/BridgeAwareCubicContainment.hpp"
#include "Fill/BridgeAwareCubicCertificate.hpp"
#include "Fill/BridgeAwareCubicOwnerBounds.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <type_traits>
#include <exception>
#include <fstream>
#include <iomanip>
#include <map>
#include <limits>
#include <numeric>
#include <mutex>
#include <sstream>
#include <set>
#include <string_view>
#include <tuple>

namespace Slic3r {
namespace BridgeAwareCubic {
// Only explicit scalar values and ordered points enter the byte stream. No
// padding, live pointers, Plan identities or support verdicts are retained.
struct ModelCollectionInputs::Impl {
    struct Unavailable {};
    struct Different {};
    struct Canceled { std::exception_ptr exception; };
    struct Range { size_t begin = 0, end = 0; };
    struct Work {
        BridgeAwareCubicValidation::WorkBudget local;
        BridgeAwareCubicValidation::WorkBudget *budget;
        const Cancel &cancel;
        size_t since_poll = 0;
        Work(BridgeAwareCubicValidation::WorkBudget *given, const Cancel &callback)
            : budget(given ? given : &local), cancel(callback) {}
        void poll() const {
            if (cancel) try { cancel(); } catch (...) { throw Canceled{std::current_exception()}; }
        }
        void charge(size_t amount = 1) {
            if (amount > budget->remaining_work) throw Unavailable{};
            budget->remaining_work -= amount;
            budget->charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += amount;
            since_poll += amount;
            if (since_poll >= 128) { since_poll = 0; poll(); }
        }
    };
    enum class Mode { Count, Write, Compare };
    struct Stream {
        Impl &owner;
        Work &work;
        Mode mode;
        size_t position = 0, end = 0;
        const unsigned char *compared = nullptr;
        void bytes(const void *value, size_t size) {
            work.charge();
            if (mode == Mode::Count) {
                if (size > owner.limits.bytes || position > owner.limits.bytes - size) throw Unavailable{};
                position += size;
                owner.admit(position, 0);
                return;
            }
            if (size > end - position) {
                if (mode == Mode::Compare) throw Different{};
                throw Unavailable{};
            }
            if (mode == Mode::Compare) {
                if (std::memcmp(compared + position, value, size) != 0) throw Different{};
            } else std::memcpy(owner.data.get() + position, value, size);
            position += size;
        }
        template<class T> void scalar(T value) {
            static_assert(std::is_arithmetic_v<T> || std::is_enum_v<T>, "Only explicit scalar encodings");
            if constexpr (std::is_floating_point_v<T>) if (!std::isfinite(value)) throw Unavailable{};
            bytes(&value, sizeof(value));
        }
        void items(size_t amount = 1) {
            work.charge(amount);
            if (mode == Mode::Count) {
                if (amount > owner.limits.items - owner.usage.items) throw Unavailable{};
                owner.usage.items += amount;
                owner.admit(position, 0);
            }
        }
        void points(const Points &points) {
            scalar(points.size());
            if (mode == Mode::Count) {
                if (points.size() > owner.limits.vertices - owner.usage.vertices) throw Unavailable{};
                owner.usage.vertices += points.size();
                owner.admit(position, 0);
            }
            for (const Point &point : points) { scalar(point.x()); scalar(point.y()); }
        }
        void polygon(const ExPolygon &polygon) {
            items(1 + polygon.holes.size());
            points(polygon.contour.points);
            scalar(polygon.holes.size());
            for (const Polygon &hole : polygon.holes) points(hole.points);
        }
    };
    Limits limits;
    Usage usage;
    size_t rows_count = 0, context_end = 0, data_size = 0, fixed_bytes = 0, scratch_peak = 0, scratch_items_peak = 0;
    std::unique_ptr<Range[]> rows;
    std::unique_ptr<unsigned char[]> data;

    void admit(size_t encoded, size_t scratch, size_t scratch_items = 0) const {
        const auto fits = [](size_t cap, size_t a, size_t b) { return a <= cap && b <= cap - a; };
        if (!fits(limits.vertices, limits.other_retained.vertices, usage.vertices) ||
            !fits(limits.items, limits.other_retained.items, usage.items) ||
            !fits(limits.bytes, limits.other_retained.bytes, fixed_bytes)) throw Unavailable{};
        if (scratch_items > limits.items - limits.other_retained.items - usage.items) throw Unavailable{};
        size_t remaining = limits.bytes - limits.other_retained.bytes - fixed_bytes;
        if (encoded > remaining || scratch > remaining - encoded) throw Unavailable{};
    }
    static void binding(const PrintObject &object, const Layer &layer, size_t region, Work &work) {
        work.charge();
        if (layer.object() != &object || region >= object.num_printing_regions() ||
            layer.get_region(int(region))->layer() != &layer ||
            &layer.get_region(int(region))->region() != &object.printing_region(region)) throw Unavailable{};
    }
    static void context(const PrintObject &object, Stream &stream) {
        stream.work.poll();
        stream.scalar(object.layers().size());
        stream.scalar(object.num_printing_regions());
        for (const Layer *layer : object.layers()) {
            stream.items();
            if (!layer || layer->object() != &object) throw Unavailable{};
            stream.scalar(layer->print_z);
            stream.scalar(layer->regions().size());
            for (size_t r = 0; r < layer->regions().size(); ++r) {
                binding(object, *layer, r, stream.work);
                stream.items();
                const LayerRegion &region = *layer->get_region(int(r));
                const auto &config = region.region().config();
                stream.scalar(config.sparse_infill_pattern.value);
                stream.scalar(config.sparse_infill_density.value);
                stream.scalar(config.bridge_aware_cubic_max_gap.value);
                stream.scalar(region.fill_surfaces.surfaces.size());
                for (const Surface &surface : region.fill_surfaces.surfaces) {
                    stream.items();
                    stream.scalar(surface.surface_type);
                    stream.scalar(surface.bridge_followup);
                    stream.polygon(surface.expolygon);
                }
                stream.scalar(region.slices.surfaces.size());
                for (const Surface &surface : region.slices.surfaces) {
                    stream.items();
                    stream.polygon(surface.expolygon);
                }
            }
        }
    }
    template<class Visitor> static void walk(const ExtrusionEntity &entity, bool free_only,
                                              Visitor &visitor, Work &work, size_t depth = 0) {
        work.charge();
        if (depth >= 64) throw Unavailable{};
        if (typeid(entity) == typeid(ExtrusionPath)) {
            const auto &path = static_cast<const ExtrusionPath &>(entity);
            if (!path.is_force_no_extrusion() && path.mm3_per_mm > 0. && path.polyline.size() >= 2)
                visitor(path);
        } else if (typeid(entity) == typeid(ExtrusionEntityCollection)) {
            for (const ExtrusionEntity *child : static_cast<const ExtrusionEntityCollection &>(entity).entities) {
                if (!child) throw Unavailable{};
                walk(*child, free_only, visitor, work, depth + 1);
            }
        } else if (typeid(entity) == typeid(ExtrusionMultiPath)) {
            if (!free_only) for (const auto &path : static_cast<const ExtrusionMultiPath &>(entity).paths)
                walk(path, false, visitor, work, depth + 1);
        } else if (typeid(entity) == typeid(ExtrusionLoop)) {
            if (!free_only) for (const auto &path : static_cast<const ExtrusionLoop &>(entity).paths)
                walk(path, false, visitor, work, depth + 1);
        } else throw Unavailable{};
    }
    static void path_value(const ExtrusionPath &path, Stream &stream) {
        if (!path.polyline.fitting_result.empty()) throw Unavailable{};
        stream.items();
        stream.scalar(path.role()); stream.scalar(path.width); stream.scalar(path.height);
        stream.scalar(path.mm3_per_mm); stream.scalar(path.is_force_no_extrusion());
        stream.scalar(path.can_reverse());
        stream.scalar(path.get_customize_flag());
        stream.scalar(path.get_cooling_node()); stream.scalar(path.overhang_degree);
        stream.scalar(path.curve_degree); stream.scalar(path.smooth_speed);
        stream.points(path.polyline.points);
    }
    void model_layer(const Layer &layer, Stream &stream) {
        stream.work.poll();
        for (size_t r = 0; r < layer.regions().size(); ++r) {
            const auto &region = *layer.get_region(int(r));
            // Preserve actual free-set membership, including a path occurring
            // through more than one tree route. Pointers are temporary only.
            size_t free_count = 0;
            auto count = [&](const ExtrusionPath &) {
                stream.work.charge();
                if (++free_count > 20000) throw Unavailable{};
            };
            walk(region.fills, true, count, stream.work);
            const size_t scratch = free_count ? free_count * sizeof(const ExtrusionPath *) + 64 : 0;
            scratch_peak = std::max(scratch_peak, scratch);
            scratch_items_peak = std::max(scratch_items_peak, free_count);
            admit(stream.mode == Mode::Count ? stream.position : data_size, scratch, free_count);
            std::unique_ptr<const ExtrusionPath *[]> free;
            if (free_count) free.reset(new const ExtrusionPath *[free_count]);
            size_t filled = 0;
            auto remember = [&](const ExtrusionPath &path) {
                stream.work.charge();
                if (filled == free_count) throw Unavailable{};
                free[filled++] = &path;
            };
            walk(region.fills, true, remember, stream.work);
            if (filled != free_count) throw Unavailable{};
            const auto less = [&](const ExtrusionPath *a, const ExtrusionPath *b) {
                stream.work.charge();
                return std::less<const ExtrusionPath *>{}(a, b);
            };
            if (free_count > 1) std::sort(free.get(), free.get() + free_count, less);
            auto model = [&](const ExtrusionPath &path) {
                if (path.role() == erInternalInfill || path.role() == erIroning) return;
                stream.scalar(uint8_t(1));
                stream.scalar(r);
                const bool free_ended = free_count && std::binary_search(free.get(), free.get() + free_count, &path, less);
                stream.scalar(free_ended);
                path_value(path, stream);
            };
            walk(region.perimeters, false, model, stream.work);
            walk(region.fills, false, model, stream.work);
        }
        stream.scalar(uint8_t(0)); // Empty rows and ordered stream termination.
    }
};

ModelCollectionInputs::ModelCollectionInputs(std::unique_ptr<const Impl> impl) : m_impl(std::move(impl)) {}
ModelCollectionInputs::~ModelCollectionInputs() = default;
std::unique_ptr<const ModelCollectionInputs> ModelCollectionInputs::capture(
    const PrintObject &object, const Limits &limits, BridgeAwareCubicValidation::WorkBudget *budget, const Cancel &cancel)
{
    Impl::Work work(budget, cancel);
    try {
        work.poll(); work.charge();
        const size_t rows = object.layers().size();
        if (rows > 20000 || limits.other_retained.bytes > limits.bytes ||
            limits.other_retained.vertices > limits.vertices || limits.other_retained.items > limits.items)
            throw Impl::Unavailable{};
        const size_t fixed = sizeof(ModelCollectionInputs) + sizeof(Impl) + 192 + rows * sizeof(Impl::Range);
        const size_t byte_cap = std::min(limits.bytes, size_t(512 * 1024));
        if (limits.other_retained.bytes > byte_cap || fixed > byte_cap - limits.other_retained.bytes ||
            limits.other_retained.items > std::min(limits.items, size_t(20000)) ||
            rows + 2 > std::min(limits.items, size_t(20000)) - limits.other_retained.items ||
            limits.other_retained.vertices > std::min(limits.vertices, size_t(200000))) throw Impl::Unavailable{};
        auto state = std::make_unique<Impl>();
        state->limits = limits;
        state->limits.bytes = std::min(limits.bytes, size_t(512 * 1024));
        state->limits.vertices = std::min(limits.vertices, size_t(200000));
        state->limits.items = std::min(limits.items, size_t(20000));
        state->rows_count = rows;
        state->fixed_bytes = fixed;
        state->usage.items = rows + 2;
        state->admit(0, 0);
        if (rows) state->rows.reset(new Impl::Range[rows]);
        Impl::Stream count{*state, work, Impl::Mode::Count};
        Impl::context(object, count);
        state->context_end = count.position;
        for (size_t layer = 1; layer < rows; ++layer) {
            state->rows[layer].begin = count.position;
            state->model_layer(*object.layers()[layer], count);
            state->rows[layer].end = count.position;
        }
        state->data_size = count.position;
        state->admit(state->data_size, state->scratch_peak, state->scratch_items_peak);
        state->usage.bytes = state->fixed_bytes + state->data_size;
        state->data.reset(new unsigned char[state->data_size]);
        Impl::Stream write{*state, work, Impl::Mode::Write, 0, state->data_size};
        Impl::context(object, write);
        if (write.position != state->context_end) throw Impl::Unavailable{};
        for (size_t layer = 1; layer < rows; ++layer) {
            state->model_layer(*object.layers()[layer], write);
            if (write.position != state->rows[layer].end) throw Impl::Unavailable{};
        }
        work.poll();
        return std::unique_ptr<const ModelCollectionInputs>(new ModelCollectionInputs(std::move(state)));
    } catch (const Impl::Canceled &canceled) { std::rethrow_exception(canceled.exception); }
      catch (const Impl::Unavailable &) {} catch (const std::bad_alloc &) {} catch (const std::length_error &) {}
    return {};
}
ModelCollectionInputs::Match ModelCollectionInputs::matches(
    const PrintObject &object, const std::set<size_t> &checked, BridgeAwareCubicValidation::WorkBudget *budget, const Cancel &cancel) const
{
    Impl::Work work(budget, cancel);
    try {
        work.poll(); work.charge();
        if (!m_impl) throw Impl::Unavailable{};
        for (size_t layer : checked) {
            work.charge();
            if (layer == 0 || layer >= object.layers().size() || layer >= m_impl->rows_count) throw Impl::Unavailable{};
        }
        // The only mutable calculation is scratch high-water accounting. Keep
        // it in a local view; the retained identity itself remains immutable.
        Impl view;
        view.limits = m_impl->limits; view.usage = m_impl->usage;
        view.fixed_bytes = m_impl->fixed_bytes; view.data_size = m_impl->data_size;
        Impl::Stream compare{view, work, Impl::Mode::Compare, 0, m_impl->context_end, m_impl->data.get()};
        Impl::context(object, compare);
        if (compare.position != m_impl->context_end) throw Impl::Different{};
        for (size_t layer : checked) {
            compare.position = m_impl->rows[layer].begin;
            compare.end = m_impl->rows[layer].end;
            view.model_layer(*object.layers()[layer], compare);
            if (compare.position != compare.end) throw Impl::Different{};
        }
        work.poll();
        return Match::Equal;
    } catch (const Impl::Canceled &canceled) { std::rethrow_exception(canceled.exception); }
      catch (const Impl::Different &) { return Match::Different; }
      catch (const Impl::Unavailable &) {} catch (const std::bad_alloc &) {} catch (const std::length_error &) {}
    return Match::Unavailable;
}
ModelCollectionInputs::Usage ModelCollectionInputs::owned_usage() const noexcept
{ return m_impl ? m_impl->usage : Usage{}; }
ModelCollectionInputs::Usage ModelCollectionInputs::capture_peak_usage() const noexcept
{
    if (!m_impl) return {};
    Usage peak = m_impl->usage;
    peak.bytes += m_impl->scratch_peak;
    peak.items += m_impl->scratch_items_peak;
    return peak;
}
} // namespace BridgeAwareCubic
namespace {
namespace Dependency = BridgeAwareCubic::Dependencies;
namespace Contact = BridgeAwareCubicContact;

// Opt-in, aggregate-only profiling. Set BAC_PROFILE_FILE to a unique JSONL
// destination; BAC_PROFILE_STAGE is an optional caller label (e.g. export).
// BAC_PROFILE_FULL_TRIAL_CHECKS=1 is the exhaustive reference: rejected trials,
// prepared cells and final checks run fully, without optional proof caches.
// Timings are inclusive, so nested metrics must not be summed as wall time.
// No clocks, containers or output are used by inner probes when disabled.
enum class ProfilePhase {
    PlanBuild, PlanClone, PlanCollapse, PlanPayload, Prune,
    GenerateFull, GenerateChanged, GenerateRollback,
    CollectModel, CollectConstruction, CheckModel, CheckConstruction, CheckMixed,
    PrepareModelTargets, CompareFinalProof, CaptureFinalProof, FinalProof, Sweep,
    ConstructionSections, ConstructionMatch, ConstructionOwners, ConstructionAdd, ModelReplay,
    ModelInputCapture, ModelInputMatch, Count
};

struct ProfileMetric {
    uint64_t calls = 0, nanoseconds = 0, work = 0, items = 0;
    uint64_t layers = 0, unique_layers = 0, unique_planes = 0;
    uint64_t upper_segments = 0, lower_segments = 0;
};

struct ProfileState {
    using Clock = std::chrono::steady_clock;
    static constexpr size_t entry_limit = 65536;
    std::array<ProfileMetric, size_t(ProfilePhase::Count)> metrics{};
    std::set<std::pair<ProfilePhase, size_t>> layers;
    std::set<std::tuple<ProfilePhase, size_t, size_t, double>> planes;
    std::set<std::tuple<size_t, unsigned, std::array<int64_t, 3>>> candidates;
    uint64_t trials = 0, accepted = 0, rejected = 0, repeated = 0, closure_trials = 0;
    uint64_t early_trial_rejections = 0;
    uint64_t sparse_probe_attempts = 0, sparse_probe_rejections = 0, sparse_probe_audits = 0, sparse_probe_audit_mismatches = 0;
    uint64_t sparse_probe_saved_generation_layers = 0, sparse_probe_saved_rollback_layers = 0;
    uint64_t sparse_probe_would_skip_generation_layers = 0, sparse_probe_would_skip_rollback_layers = 0;
    uint64_t model_replay_probes = 0, model_replay_rejections = 0, model_replay_passes = 0;
    uint64_t model_replay_unavailable = 0, model_replay_stores = 0, model_replay_entries = 0;
    uint64_t construction_containment_indices = 0, construction_containment_queries = 0, construction_containment_proofs = 0;
    uint64_t emission_ledger_hits = 0, emission_ledger_fallbacks = 0, emission_ledger_construction_paths = 0;
    uint64_t emission_projection_builds = 0, emission_projection_reuses = 0;
    uint64_t emission_rollback_restored = 0, emission_rollback_mismatches = 0;
    uint64_t emission_rejected_without_projection = 0;
    uint64_t emission_collection_audits = 0;
    std::array<std::pair<const char *, uint64_t>, 32> emission_capture_reasons{};
    uint64_t whole_support_queries = 0, whole_support_proofs = 0, whole_support_segments = 0;
    uint64_t complete_span_queries = 0, complete_span_proofs = 0, complete_span_audits = 0;
    uint64_t complete_span_audit_mismatches = 0;
    uint64_t model_cache_hits = 0, model_cache_fallbacks = 0, model_cache_vertices = 0, model_cache_items = 0;
    uint64_t model_input_hits = 0, model_input_misses = 0, model_input_unavailable = 0, model_input_audits = 0;
    uint64_t model_input_captures = 0, model_input_capture_declines = 0, model_input_bytes = 0;
    uint64_t model_input_prepared_bytes = 0, model_input_combined_bytes = 0, model_input_capture_peak_bytes = 0;
    uint64_t model_input_combined_vertices = 0, model_input_combined_items = 0;
    uint64_t model_layer_proof_hits = 0, model_layer_proof_misses = 0, model_layer_proof_publications = 0;
    uint64_t model_layer_proof_vertices = 0, model_layer_proof_items = 0;
    uint64_t span_proof_hits = 0, span_proof_misses = 0, span_proof_publications = 0, span_proof_evictions = 0;
    uint64_t span_proof_entries = 0, span_proof_units = 0;
    uint64_t contact_epoch_captures = 0, contact_epoch_transitions = 0, contact_epoch_fallbacks = 0;
    uint64_t contact_epoch_peak_bytes = 0;
    uint64_t span_epoch_hits = 0, span_epoch_advances = 0, span_epoch_audits = 0, span_epoch_audit_mismatches = 0;
    uint64_t span_epoch_raw_checks = 0, contact_epoch_peak_slots = 0;
    uint64_t final_proof_hits = 0, final_proof_misses = 0, final_proof_publications = 0, final_proof_bytes = 0;
    bool full_trial_checks = false;
    bool audit_emission_collection = false;
    bool audit_contact_epochs = false;
    bool audit_complete_spans = false;
    bool audit_sparse_rejections = false;
    bool truncated = false;
    std::string file, stage;
    const char *session;
    uint64_t sequence, object_id;
    Clock::time_point started = Clock::now();
    int exceptions = std::uncaught_exceptions();

    void emission_capture(const char *reason) noexcept {
        reason = reason ? reason : "unavailable";
        for (auto &entry : emission_capture_reasons)
            if (entry.first == nullptr || std::string_view(entry.first) == reason) {
                entry.first = reason;
                ++entry.second;
                return;
            }
        truncated = true;
    }

    void layer(ProfilePhase phase, size_t index) noexcept {
        auto &metric = metrics[size_t(phase)];
        ++metric.layers;
        try {
            if (layers.size() < entry_limit) metric.unique_layers += layers.emplace(phase, index).second;
            else truncated = true;
        } catch (...) { truncated = true; }
    }
    void plane(ProfilePhase phase, size_t layer, size_t output_phase, double z) noexcept {
        if (!std::isfinite(z)) return;
        try {
            if (planes.size() < entry_limit)
                metrics[size_t(phase)].unique_planes += planes.emplace(phase, layer, output_phase, z).second;
            else truncated = true;
        } catch (...) { truncated = true; }
    }
    void trial(size_t region, const BridgeAwareCubic::CellId *cell) noexcept {
        ++trials;
        if (cell == nullptr) { ++closure_trials; return; }
        try {
            if (candidates.size() < entry_limit)
                repeated += !candidates.emplace(region, cell->depth, cell->address).second;
            else truncated = true;
        } catch (...) { truncated = true; }
    }
    void write(const char *record, uint64_t elapsed, uint64_t sweep = 0,
               uint64_t attempted = 0, uint64_t passed = 0, uint64_t failed = 0,
               uint64_t retried = 0, uint64_t closure_attempted = 0) const noexcept {
        try {
            static std::mutex output_mutex;
            std::lock_guard<std::mutex> lock(output_mutex);
            std::ofstream out(file, std::ios::app);
            if (!out) return;
            out << "{\"record\":\"" << record << "\",\"session\":\"" << session
                << "\",\"stage\":\"" << stage << "\",\"sequence\":" << sequence
                << ",\"object\":" << object_id << ",\"elapsed_ns\":" << elapsed
                << ",\"incomplete\":" << (std::uncaught_exceptions() > exceptions ? "true" : "false");
            if (sweep != 0) {
                out << ",\"sweep\":" << sweep << ",\"trials\":" << attempted
                    << ",\"accepted\":" << passed << ",\"rejected\":" << failed
                    << ",\"repeated_cell_trials\":" << retried << ",\"closure_trials\":" << closure_attempted;
            } else {
                out << ",\"trials\":" << trials << ",\"accepted\":" << accepted
                    << ",\"rejected\":" << rejected << ",\"repeated_cell_trials\":" << repeated
                    << ",\"closure_trials\":" << closure_trials
                    << ",\"early_trial_rejections\":" << early_trial_rejections
                    << ",\"sparse_probe_attempts\":" << sparse_probe_attempts
                    << ",\"sparse_probe_rejections\":" << sparse_probe_rejections
                    << ",\"sparse_probe_audits\":" << sparse_probe_audits
                    << ",\"sparse_probe_audit_mismatches\":" << sparse_probe_audit_mismatches
                    << ",\"sparse_probe_saved_generation_layers\":" << sparse_probe_saved_generation_layers
                    << ",\"sparse_probe_saved_rollback_layers\":" << sparse_probe_saved_rollback_layers
                    << ",\"sparse_probe_would_skip_generation_layers\":" << sparse_probe_would_skip_generation_layers
                    << ",\"sparse_probe_would_skip_rollback_layers\":" << sparse_probe_would_skip_rollback_layers
                    << ",\"model_replay_probes\":" << model_replay_probes
                    << ",\"model_replay_rejections\":" << model_replay_rejections
                    << ",\"model_replay_passes\":" << model_replay_passes
                    << ",\"model_replay_unavailable\":" << model_replay_unavailable
                    << ",\"model_replay_stores\":" << model_replay_stores
                    << ",\"model_replay_entries\":" << model_replay_entries
                    << ",\"construction_containment_indices\":" << construction_containment_indices
                    << ",\"construction_containment_queries\":" << construction_containment_queries
                    << ",\"construction_containment_proofs\":" << construction_containment_proofs
                    << ",\"emission_ledger_hits\":" << emission_ledger_hits
                    << ",\"emission_ledger_fallbacks\":" << emission_ledger_fallbacks
                    << ",\"emission_ledger_construction_paths\":" << emission_ledger_construction_paths
                    << ",\"emission_projection_builds\":" << emission_projection_builds
                    << ",\"emission_projection_reuses\":" << emission_projection_reuses
                    << ",\"emission_rollback_restored\":" << emission_rollback_restored
                    << ",\"emission_rollback_mismatches\":" << emission_rollback_mismatches
                    << ",\"emission_rejected_without_projection\":" << emission_rejected_without_projection
                    << ",\"emission_collection_audits\":" << emission_collection_audits
                    << ",\"full_trial_checks\":" << (full_trial_checks ? "true" : "false")
                    << ",\"model_cache_hits\":" << model_cache_hits << ",\"model_cache_fallbacks\":" << model_cache_fallbacks
                    << ",\"model_cache_vertices\":" << model_cache_vertices << ",\"model_cache_items\":" << model_cache_items
                    << ",\"model_input_hits\":" << model_input_hits
                    << ",\"model_input_misses\":" << model_input_misses
                    << ",\"model_input_unavailable\":" << model_input_unavailable
                    << ",\"model_input_audits\":" << model_input_audits
                    << ",\"model_input_captures\":" << model_input_captures
                    << ",\"model_input_capture_declines\":" << model_input_capture_declines
                    << ",\"model_input_bytes\":" << model_input_bytes
                    << ",\"model_input_prepared_bytes\":" << model_input_prepared_bytes
                    << ",\"model_input_combined_bytes\":" << model_input_combined_bytes
                    << ",\"model_input_capture_peak_bytes\":" << model_input_capture_peak_bytes
                    << ",\"model_input_combined_vertices\":" << model_input_combined_vertices
                    << ",\"model_input_combined_items\":" << model_input_combined_items
                    << ",\"model_layer_proof_hits\":" << model_layer_proof_hits
                    << ",\"model_layer_proof_misses\":" << model_layer_proof_misses
                    << ",\"model_layer_proof_publications\":" << model_layer_proof_publications
                    << ",\"model_layer_proof_vertices\":" << model_layer_proof_vertices
                    << ",\"model_layer_proof_items\":" << model_layer_proof_items
                    << ",\"span_proof_hits\":" << span_proof_hits
                    << ",\"span_proof_misses\":" << span_proof_misses
                    << ",\"span_proof_publications\":" << span_proof_publications
                    << ",\"span_proof_evictions\":" << span_proof_evictions
                    << ",\"span_proof_entries\":" << span_proof_entries
                    << ",\"span_proof_units\":" << span_proof_units
                    << ",\"contact_epoch_captures\":" << contact_epoch_captures
                    << ",\"contact_epoch_transitions\":" << contact_epoch_transitions
                    << ",\"contact_epoch_fallbacks\":" << contact_epoch_fallbacks
                    << ",\"contact_epoch_peak_bytes\":" << contact_epoch_peak_bytes
                    << ",\"span_epoch_hits\":" << span_epoch_hits
                    << ",\"span_epoch_advances\":" << span_epoch_advances
                    << ",\"span_epoch_audits\":" << span_epoch_audits
                    << ",\"span_epoch_audit_mismatches\":" << span_epoch_audit_mismatches
                    << ",\"span_epoch_raw_checks\":" << span_epoch_raw_checks
                    << ",\"contact_epoch_peak_slots\":" << contact_epoch_peak_slots
                    << ",\"complete_span_queries\":" << complete_span_queries
                    << ",\"complete_span_proofs\":" << complete_span_proofs
                    << ",\"complete_span_audits\":" << complete_span_audits
                    << ",\"complete_span_audit_mismatches\":" << complete_span_audit_mismatches
                    << ",\"whole_support_queries\":" << whole_support_queries
                    << ",\"whole_support_proofs\":" << whole_support_proofs
                    << ",\"whole_support_segments\":" << whole_support_segments
                    << ",\"final_proof_hits\":" << final_proof_hits << ",\"final_proof_misses\":" << final_proof_misses
                    << ",\"final_proof_publications\":" << final_proof_publications << ",\"final_proof_bytes\":" << final_proof_bytes
                    << ",\"identity_counts_truncated\":" << (truncated ? "true" : "false")
                    << ",\"emission_capture_reasons\":{";
                bool first_reason = true;
                for (const auto &entry : emission_capture_reasons) if (entry.first != nullptr) {
                    if (!first_reason) out << ',';
                    first_reason = false;
                    out << '"' << entry.first << "\":" << entry.second;
                }
                out << "},\"metrics\": [";
                static constexpr const char *names[] = {"plan_build", "plan_clone", "plan_collapse", "plan_payload", "dependency_prune",
                    "generate_full", "generate_changed", "generate_rollback", "collect_model", "collect_construction",
                    "check_model", "check_construction", "check_mixed", "prepare_model_targets", "compare_final_proof",
                    "capture_final_proof", "final_proof", "coarsening_sweep", "construction_sections",
                    "construction_match", "construction_owners", "construction_add", "model_failure_replay",
                    "model_input_capture", "model_input_match"};
                bool first = true;
                for (size_t i = 0; i < metrics.size(); ++i) {
                    const auto &metric = metrics[i];
                    if (metric.calls == 0) continue;
                    if (!first) out << ',';
                    first = false;
                    out << "{\"name\":\"" << names[i] << "\",\"calls\":" << metric.calls
                        << ",\"elapsed_ns\":" << metric.nanoseconds << ",\"work\":" << metric.work
                        << ",\"items\":" << metric.items << ",\"layer_visits\":" << metric.layers
                        << ",\"unique_layers\":" << metric.unique_layers << ",\"unique_planes\":" << metric.unique_planes
                        << ",\"upper_segment_slots\":" << metric.upper_segments << ",\"lower_segment_slots\":" << metric.lower_segments << '}';
                }
                out << ']';
            }
            out << "}\n";
        } catch (...) { /* Diagnostics must not change slicing or exception flow. */ }
    }
};

thread_local ProfileState *active_profile = nullptr;

class ProfileSession {
    std::unique_ptr<ProfileState> state;
    ProfileState *previous = active_profile;
public:
    ProfileSession(const char *session, uint64_t object_id) noexcept {
        const char *file = std::getenv("BAC_PROFILE_FILE");
        if (file == nullptr || *file == '\0') return;
        try {
            state = std::make_unique<ProfileState>();
            state->file = file;
            state->session = session;
            state->object_id = object_id;
            if (const char *full = std::getenv("BAC_PROFILE_FULL_TRIAL_CHECKS"))
                state->full_trial_checks = full[0] == '1' && full[1] == '\0';
            if (const char *audit = std::getenv("BAC_PROFILE_AUDIT_EMISSION_COLLECTION"))
                state->audit_emission_collection = audit[0] == '1' && audit[1] == '\0';
            if (const char *audit = std::getenv("BAC_PROFILE_AUDIT_CONTACT_EPOCHS"))
                state->audit_contact_epochs = audit[0] == '1' && audit[1] == '\0';
            if (const char *audit = std::getenv("BAC_PROFILE_AUDIT_SPARSE_REJECTIONS"))
                state->audit_sparse_rejections = audit[0] == '1' && audit[1] == '\0';
            if (const char *audit = std::getenv("BAC_PROFILE_AUDIT_COMPLETE_SPANS"))
                state->audit_complete_spans = audit[0] == '1' && audit[1] == '\0';
            static std::atomic<uint64_t> sequence{0};
            state->sequence = ++sequence;
            if (const char *stage = std::getenv("BAC_PROFILE_STAGE"))
                for (size_t i = 0; stage[i] != '\0' && i < 64; ++i) {
                    const char c = stage[i];
                    state->stage += ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.') ? c : '_';
                }
            active_profile = state.get();
        } catch (...) { state.reset(); }
    }
    ~ProfileSession() {
        if (state) {
            state->write("total", uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                ProfileState::Clock::now() - state->started).count()));
            active_profile = previous;
        }
    }
};

class ProfileScope {
    ProfileState *state = active_profile;
    ProfilePhase phase;
    ProfileState::Clock::time_point started;
    const BridgeAwareCubicValidation::WorkBudget *budget;
    size_t initial_work = 0;
public:
    explicit ProfileScope(ProfilePhase phase, const BridgeAwareCubicValidation::WorkBudget *budget = nullptr)
        : phase(phase), budget(budget) {
        if (state) {
            started = ProfileState::Clock::now();
            ++state->metrics[size_t(phase)].calls;
            if (budget) initial_work = budget->remaining_work;
        }
    }
    ~ProfileScope() {
        if (state) {
            auto &metric = state->metrics[size_t(phase)];
            metric.nanoseconds += uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(ProfileState::Clock::now() - started).count());
            if (budget) metric.work += initial_work - budget->remaining_work;
        }
    }
    void layer(size_t index) { if (state) state->layer(phase, index); }
    void plane(size_t layer, size_t output_phase, double z) { if (state) state->plane(phase, layer, output_phase, z); }
    void upper(size_t segments) { if (state) state->metrics[size_t(phase)].upper_segments += segments; }
    void lower(size_t segments) { if (state) state->metrics[size_t(phase)].lower_segments += segments; }
    void items(size_t count) { if (state) state->metrics[size_t(phase)].items += count; }
};

class ProfileSweep {
    ProfileState *state = active_profile;
    ProfileScope timing{ProfilePhase::Sweep};
    ProfileState::Clock::time_point started;
    uint64_t number, trials = 0, accepted = 0, rejected = 0, repeated = 0, closures = 0;
public:
    explicit ProfileSweep(uint64_t number) : number(number) {
        if (state) {
            started = ProfileState::Clock::now();
            trials = state->trials; accepted = state->accepted; rejected = state->rejected;
            repeated = state->repeated; closures = state->closure_trials;
        }
    }
    ~ProfileSweep() {
        if (state) state->write("sweep", uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
            ProfileState::Clock::now() - started).count()), number, state->trials - trials,
            state->accepted - accepted, state->rejected - rejected, state->repeated - repeated,
            state->closure_trials - closures);
    }
};

// No path simplification is allowed after contact-qualified verification.
// This allowance covers polygon operations plus paired XYZ export rounding:
// 0.001 mm output can displace two relative XY endpoints by sqrt(2)*0.001.
constexpr double geometry_tolerance = .002 + 16. * SCALING_FACTOR;

double deposition_z(const Layer &layer)
{
    return GCodeWriter::serialized_z(layer.print_z, true);
}

bool participates(const PrintRegionConfig &config)
{
    return config.sparse_infill_pattern == ipBridgeAwareCubic &&
        config.sparse_infill_density.value > 0. && config.sparse_infill_density.value < 100.;
}

template<class Visitor> void visit_paths(const ExtrusionEntity &entity, const Visitor &visitor)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath*>(&entity)) {
        if (!path->is_force_no_extrusion() && path->mm3_per_mm > 0. && path->polyline.size() >= 2)
            visitor(*path);
    } else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath*>(&entity)) {
        for (const ExtrusionPath &path : multi->paths) visit_paths(path, visitor);
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(&entity)) {
        for (const ExtrusionPath &path : loop->paths) visit_paths(path, visitor);
    } else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(&entity)) {
        for (const ExtrusionEntity *child : collection->entities) visit_paths(*child, visitor);
    }
}

template<class Visitor> void visit_layer(const Layer &layer, const Visitor &visitor)
{
    for (const LayerRegion *region : layer.regions()) {
        visit_paths(region->perimeters, visitor);
        visit_paths(region->fills, visitor);
    }
}

// Only independent open entities have free endpoints. The ends of a loop path
// or an interior multipath component are joins and must never be extended.
template<class Entity, class Visitor> void visit_free_paths(Entity &entity, const Visitor &visitor)
{
    using Path = std::conditional_t<std::is_const_v<Entity>, const ExtrusionPath, ExtrusionPath>;
    using Collection = std::conditional_t<std::is_const_v<Entity>, const ExtrusionEntityCollection, ExtrusionEntityCollection>;
    if (auto *path = dynamic_cast<Path*>(&entity)) {
        if (!path->is_force_no_extrusion() && path->mm3_per_mm > 0. && path->polyline.size() >= 2)
            visitor(*path);
    } else if (auto *collection = dynamic_cast<Collection*>(&entity)) {
        for (auto *child : collection->entities) visit_free_paths(*child, visitor);
    }
}

bool has_circle_scarf(const ExtrusionEntity &entity)
{
    if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(&entity))
        return loop->get_customize_flag() == cfCircleCompensation && !loop->has_overhang_paths();
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(&entity))
        for (const ExtrusionEntity *child : collection->entities)
            if (has_circle_scarf(*child)) return true;
    return false;
}

ExPolygons sparse_domain(const LayerRegion &region)
{
    ExPolygons result;
    for (const Surface &surface : region.fill_surfaces.surfaces)
        if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
            result.push_back(surface.expolygon);
    return union_ex(result);
}

bool closure_compatible(const LayerRegion &emitter, const LayerRegion &candidate)
{
    const auto &config = emitter.region().config();
    const auto &other = candidate.region().config();
    if (!participates(config) || !participates(other) || config.sparse_infill_filament != other.sparse_infill_filament)
        return false;
    // Closure courses use ordinary sparse flow. Density, gap and lattice
    // angle may differ across a participating seam; physical material/flow
    // ownership may not. Other extrusion roles do not enter this comparison.
    const Flow own = emitter.flow(frInfill), adjacent = candidate.flow(frInfill);
    return own.width() == adjacent.width() && own.height() == adjacent.height() && own.mm3_per_mm() == adjacent.mm3_per_mm();
}

ExPolygons closure_domain(const Layer &layer, size_t emitting_region)
{
    ExPolygons allowed;
    if (emitting_region >= layer.regions().size()) return allowed;
    const LayerRegion &emitter = *layer.get_region(int(emitting_region));
    if (!participates(emitter.region().config()) || emitter.slices.empty()) return allowed;
    for (const LayerRegion *region : layer.regions()) {
        if (!closure_compatible(emitter, *region)) continue;
        // Shared modifier faces are permitted only between compatible printable
        // participating regions. Existing ordinary material may supply contact
        // evidence, but it cannot become a destination for added BAC closure.
        for (const Surface &surface : region->slices.surfaces) allowed.push_back(surface.expolygon);
    }
    return intersection_ex(union_ex(allowed), layer.lslices);
}

bool closure_paths_owned(const Layer &layer, size_t emitting_region, const Polylines &paths)
{
    if (emitting_region >= layer.regions().size()) return false;
    const LayerRegion &emitter = *layer.get_region(int(emitting_region));
    if (!participates(emitter.region().config()) || emitter.slices.empty()) return false;
    ExPolygons own;
    for (const Surface &surface : emitter.slices.surfaces) own.push_back(surface.expolygon);
    // A shared-boundary course has one actual emitter. This allowance belongs
    // only to integer ownership clipping; the full bead must separately fit
    // the undilated participating material domain below.
    if (!diff_pl(paths, offset_ex(union_ex(own), 2.)).empty()) return false;
    const Flow flow = emitter.flow(frInfill);
    return diff_ex(union_ex(offset(paths, scale_(.5 * flow.width()))), closure_domain(layer, emitting_region)).empty();
}

bool same_polyline(const Polyline &a, const Polyline &b)
{
    return a.points == b.points;
}

struct ActiveAttachments {
    std::vector<BridgeAwareCubic::LayerAttachment> entries;
    std::vector<size_t> storage_regions;
};

ActiveAttachments active_attachments(const Layer &layer)
{
    ActiveAttachments active;
    for (size_t owner = 0; owner < layer.regions().size(); ++owner)
        if (const auto *plan = layer.object()->bridge_aware_cubic_plan(owner))
            for (auto &attachment : plan->attachments(layer.print_z)) {
                active.storage_regions.push_back(owner);
                active.entries.push_back(std::move(attachment));
            }
    return active;
}

// Divide the optional metadata allowance by every layer and emitting region,
// including layers currently regenerated in parallel. A second equal allowance
// bounds transient caller views/copies; builder and filler scratch share the
// first allowance with retained ledgers. No global mutable accounting is used.
// Canonical material identities borrow two MiB from this existing allowance;
// neither coarsening nor parallel generation receives another metadata pool.
constexpr size_t contact_epoch_allowance = 2 * 1024 * 1024;
constexpr size_t emission_allowance = 24 * 1024 * 1024 - contact_epoch_allowance;
constexpr size_t emission_context_slot_bytes = 512;

size_t emission_quota(const Layer &layer)
{
    const size_t share = emission_allowance / std::max(size_t(1), layer.object()->layers().size()) /
        std::max(size_t(1), layer.object()->num_printing_regions());
    // Includes current/base handles, row capacities and retained-byte views in
    // the one coarsening-local context. No second allowance for a trial state.
    return share > emission_context_slot_bytes ? share - emission_context_slot_bytes : 0;
}

bool emission_charge(const PrintObject &object, BridgeAwareCubicValidation::WorkBudget &budget, size_t amount = 1)
{
    if (object.print()->canceled()) throw CanceledException();
    if (amount > budget.remaining_work) return false;
    budget.remaining_work -= amount;
    budget.charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += amount;
    return true;
}

bool emission_bindings(const Layer &layer, std::vector<BridgeAwareCubic::EmissionPlanBinding> &bindings,
    size_t byte_limit, BridgeAwareCubicValidation::WorkBudget &budget)
{
    const PrintObject &object = *layer.object();
    const size_t count = object.num_printing_regions();
    if (count > byte_limit / sizeof(BridgeAwareCubic::EmissionPlanBinding) ||
        !emission_charge(object, budget, count)) return false;
    bindings.reserve(count);
    if (bindings.capacity() > byte_limit / sizeof(BridgeAwareCubic::EmissionPlanBinding)) return false;
    for (size_t owner = 0; owner < count; ++owner)
        if (const auto *plan = object.bridge_aware_cubic_plan(owner))
            bindings.push_back({owner, plan->immutable_identity()});
    return true;
}

// Read every active internal-infill leaf in the same traversal order as the
// reference collector. Unknown tree shapes and non-free leaves are unavailable
// in the first capture scope; they cannot disappear from a partial ledger.
bool emission_views(const Layer &layer, size_t emitter,
    std::vector<BridgeAwareCubic::EmissionPathView> &views, size_t byte_limit,
    BridgeAwareCubicValidation::WorkBudget &budget)
{
    const size_t limit = std::min(size_t(20000), byte_limit / sizeof(BridgeAwareCubic::EmissionPathView));
    const auto visit = [&](auto &&self, const ExtrusionEntity &entity, size_t depth) -> bool {
        if (depth > 64 || !emission_charge(*layer.object(), budget)) return false;
        if (typeid(entity) == typeid(ExtrusionPath)) {
            const auto &path = static_cast<const ExtrusionPath &>(entity);
            if (path.role() != erInternalInfill || path.is_force_no_extrusion() ||
                !(path.mm3_per_mm > 0.) || path.polyline.size() < 2) return true;
            if (views.size() == limit) return false;
            if (views.size() == views.capacity()) {
                views.reserve(std::min(limit, std::max(size_t(8), 2 * views.capacity())));
                if (views.capacity() > limit) return false;
            }
            views.push_back({&path, true});
            return true;
        }
        if (typeid(entity) == typeid(ExtrusionEntityCollection)) {
            for (const ExtrusionEntity *child : static_cast<const ExtrusionEntityCollection &>(entity).entities)
                if (child == nullptr || !self(self, *child, depth + 1)) return false;
            return true;
        }
        // Existing loops/multipaths may carry model solid fill, but an active
        // internal-infill component is outside the standalone-path scope.
        const ExtrusionPaths *joined = nullptr;
        if (typeid(entity) == typeid(ExtrusionMultiPath)) joined = &static_cast<const ExtrusionMultiPath &>(entity).paths;
        else if (typeid(entity) == typeid(ExtrusionLoop)) joined = &static_cast<const ExtrusionLoop &>(entity).paths;
        else return false;
        for (const auto &path : *joined) {
            if (!emission_charge(*layer.object(), budget)) return false;
            if (path.role() == erInternalInfill && !path.is_force_no_extrusion() &&
                path.mm3_per_mm > 0. && path.polyline.size() >= 2) return false;
        }
        return true;
    };
    return emitter < layer.regions().size() && visit(visit, layer.get_region(int(emitter))->fills, 0);
}

bool emission_attachment_bytes(const BridgeAwareCubic::LayerAttachment &attachment, size_t &remaining,
    const PrintObject &object, BridgeAwareCubicValidation::WorkBudget &budget)
{
    const auto charge = [&](size_t count, size_t size) {
        if (count > remaining / size || !emission_charge(object, budget, count)) return false;
        remaining -= count * size;
        return true;
    };
    // Reserve twice the exact copied elements, leaving room for vector control
    // blocks/capacity rounding. Callers reserve the outer vector once.
    if (!charge(1, 2 * sizeof(BridgeAwareCubic::EmissionTransition)) ||
        !charge(attachment.original.size(), 2 * sizeof(Point)) ||
        !charge(attachment.replacement.size(), 2 * sizeof(Point)) ||
        !charge(attachment.original.fitting_result.size(), 2 * sizeof(PathFittingData)) ||
        !charge(attachment.replacement.fitting_result.size(), 2 * sizeof(PathFittingData)) ||
        !charge(attachment.owners.size(), 2 * sizeof(BridgeAwareCubic::CellId)) ||
        !charge(attachment.demand_selection.size(), 2 * sizeof(ExPolygon))) return false;
    for (const ExPolygon &polygon : attachment.demand_selection) {
        if (!charge(polygon.contour.size(), 2 * sizeof(Point)) ||
            !charge(polygon.holes.size(), 2 * sizeof(Polygon))) return false;
        for (const Polygon &hole : polygon.holes)
            if (!charge(hole.size(), 2 * sizeof(Point))) return false;
    }
    return true;
}

BridgeAwareCubicAttachment::ResolvedChain resolve_attachments(const Layer &layer, const Polyline &path,
    double width, double height, size_t emitter, const ActiveAttachments &active)
{
    try {
        return BridgeAwareCubicAttachment::resolve_chain(path, width, height, emitter, active.entries);
    } catch (const std::exception &error) {
        throw SlicingError("Bridge aware cubic attachment replay failed at layer " + std::to_string(layer.id() + 1) +
            ": " + error.what(), layer.object()->model_object()->id().id);
    }
}

// Remove only coincident, parallel deposition. Crossing partition faces are
// both retained; subtracting their finite-width footprints would open a joint.
Polylines remove_collinear_duplicates(Polylines candidates, const Polylines &existing, bool matching_only = false)
{
    Polylines unique;
    for (const Polyline &candidate : candidates) {
        if (candidate.size() > 2) {
            for (size_t i = 1; i < candidate.size(); ++i)
                append(unique, remove_collinear_duplicates({Polyline(Points{candidate.points[i - 1], candidate.points[i]})},
                                                            existing, matching_only));
            continue;
        }
        Polylines overlaps;
        bool completely_owned = false;
        if (candidate.size() == 2) {
            const Vec2d axis = (candidate.last_point() - candidate.first_point()).cast<double>();
            const double length = axis.norm();
            for (const Polyline &line : existing) for (size_t i = 1; i < line.size(); ++i) {
                const Vec2d other = (line.points[i] - line.points[i - 1]).cast<double>();
                const Vec2d delta = (line.points[i - 1] - candidate.first_point()).cast<double>();
                const auto cross = [](const Vec2d &a, const Vec2d &b) { return a.x()*b.y()-a.y()*b.x(); };
                bool coincident = false;
                const double other_length = other.norm();
                if (length > 0. && other_length > 0.) {
                    if (matching_only) {
                        // Both clipped endpoints can move by a coordinate unit.
                        // Test the short interval against the long reference;
                        // extrapolating its rounded direction to a distant owner
                        // endpoint magnifies that error and loses true producers.
                        const Vec2d reference = length >= other_length ? axis : other;
                        const Point reference_start = length >= other_length ? candidate.first_point() : line.points[i - 1];
                        const Point first = length >= other_length ? line.points[i - 1] : candidate.first_point();
                        const Point last = length >= other_length ? line.points[i] : candidate.last_point();
                        const double limit = 2. * std::max(length, other_length);
                        coincident = std::abs(cross(reference, (first - reference_start).cast<double>())) <= limit &&
                                     std::abs(cross(reference, (last - reference_start).cast<double>())) <= limit;
                        // Preserve a completely owned source fragment exactly.
                        // Re-clipping a short rounded segment through a thin
                        // offset can move its endpoints or direction and leave
                        // an artificial wedge in its finite-width ownership.
                        const double begin = other.dot((candidate.first_point() - line.points[i - 1]).cast<double>());
                        const double end = other.dot((candidate.last_point() - line.points[i - 1]).cast<double>());
                        completely_owned |= coincident && std::min(begin, end) >= 0. &&
                                            std::max(begin, end) <= other.squaredNorm();
                    } else {
                        // Deposition deduplication retains its stricter rule;
                        // the broader clipping bound is only provenance data.
                        coincident = std::abs(cross(axis, other)) <= std::max(length, other_length) &&
                                     std::abs(cross(axis, delta)) <= length;
                    }
                }
                if (coincident) overlaps.emplace_back(Points{line.points[i - 1], line.points[i]});
            }
        }
        if (matching_only) {
            if (completely_owned) unique.push_back(candidate);
            else if (!overlaps.empty()) append(unique, intersection_pl(Polylines{candidate}, offset(overlaps, 2.)));
        } else append(unique, overlaps.empty() ? Polylines{candidate} : diff_pl(Polylines{candidate}, offset(overlaps, 2.)));
    }
    return unique;
}

struct Target {
    size_t upper_layer = 0;
    size_t supporting_region = 0;
    double gap = 0.;
    double bottom_z = 0.;
    bool dependency = false;
    bool bridge_followup = false;
    Polylines paths;
    std::vector<double> path_widths;
    ExPolygons footprint;
    ExtrusionPaths extrusions;
    std::vector<size_t> emitting_regions;
    std::vector<bool> free_ended;
    std::vector<std::vector<BridgeAwareCubic::CellId>> producers;
    std::vector<bool> solid_cuts;
    std::vector<ExPolygons> path_footprints;
};

const ExPolygons &path_demand(const Target &target, size_t index)
{
    if (!target.path_footprints.empty()) return target.path_footprints.at(index);
    // Synthetic attachment proposals historically supply one complete path.
    // A grouped target must retain each path's exact selected demand identity.
    if (target.paths.size() != 1 || index != 0)
        throw std::logic_error("Bridge aware cubic lost an individual path's demand footprint.");
    return target.footprint;
}

std::unique_ptr<BridgeAwareCubicContact::QueryCache> make_contact_query_cache(
    const BridgeAwareCubicContact::PreparedLowerPtr &prepared)
{
    try {
        return std::make_unique<BridgeAwareCubicContact::QueryCache>(prepared);
    } catch (const std::bad_alloc &) {
        // This allocation is optional acceleration. Keep the exact prepared
        // geometry and query without reuse until the next prepared-key change.
        return {};
    }
}

// Group equal contact planes, but retain complete emitted paths and endpoints.
// Only the demand mask is clipped to the supplying region; it never closes a
// cell or supplies an artificial endpoint to the validator.
enum class ConstructionCollection { Reference, Emitted };

// One sequential coarsening transaction owns both versions. Parallel final
// generation neither accesses this context nor mutates immutable snapshots.
struct EmissionReuseEntry {
    BridgeAwareCubic::EmissionProjectionPtr projection;
    BridgeAwareCubic::EmissionSnapshotPtr base;
    BridgeAwareCubic::EmissionProjectionPtr base_projection;
};

struct EmissionReuseRow {
    std::vector<EmissionReuseEntry> entries;
    std::vector<size_t> retained;
};

bool current_emission_matches(const Layer &layer, size_t emitter,
    const BridgeAwareCubic::EmissionSnapshotPtr &snapshot, size_t other_bytes,
    BridgeAwareCubicValidation::WorkBudget &budget,
    const std::vector<BridgeAwareCubic::EmissionPlanBinding> *restored_bindings = nullptr)
{
    const size_t quota = emission_quota(layer);
    if (!snapshot || other_bytes > quota || snapshot->resident_bytes() > quota - other_bytes) return false;
    const size_t scratch = quota - other_bytes - snapshot->resident_bytes();
    try {
        std::vector<BridgeAwareCubic::EmissionPlanBinding> bindings;
        std::vector<BridgeAwareCubic::EmissionPathView> views;
        // Vector growth may temporarily retain the old views allocation.
        if ((!restored_bindings && !emission_bindings(layer, bindings, scratch / 4, budget)) ||
            !emission_views(layer, emitter, views, scratch / 3, budget)) return false;
        return snapshot->matches(emitter, layer.print_z, {layer.object()->bounding_box(), geometry_tolerance},
            restored_bindings ? *restored_bindings : bindings, views, &budget,
            [&] { if (layer.object()->print()->canceled()) throw CanceledException(); });
    } catch (const std::bad_alloc &) {}
    return false;
}

// A rejection may generate one real predecessor out of order only when its
// emission is independent of earlier changed layers. This initial case has
// one participating and correctly bound region, only ordinary internal fill,
// no narrow-solid detection/LockedZag surface edits or other fill generators,
// and no free perimeter path on which either attachment replay can write.
bool sparse_probe_layer_eligible(const Layer &layer, const EmissionReuseRow *saved,
                                 BridgeAwareCubicValidation::WorkBudget &budget)
{
    const PrintObject &object = *layer.object();
    if (!saved || saved->entries.size() != 1 || saved->retained.size() != 1 ||
        saved->retained[0] || saved->entries[0].base || saved->entries[0].base_projection ||
        object.num_printing_regions() != 1 || layer.regions().size() != 1 ||
        layer.bridge_aware_emission_budget || !layer.bridge_aware_emission_builders.empty() ||
        layer.bridge_aware_emission_snapshots.size() > 1 ||
        object.config().detect_narrow_internal_solid_infill.value) return false;
    const LayerRegion &region = *layer.get_region(0);
    const auto &cfg = region.region().config();
    if (region.layer() != &layer || &region.region() != &object.printing_region(0) ||
        !participates(cfg) || !object.bridge_aware_cubic_plan(0) || cfg.conformal_infill.value ||
        cfg.fill_multiline.value != 1 || !region.thin_fills.entities.empty() ||
        region.fill_surfaces.surfaces.empty()) return false;
    for (const Surface &surface : region.fill_surfaces.surfaces)
        if (!emission_charge(object, budget) || surface.surface_type != stInternal) return false;
    const auto joined_perimeters = [&](auto &&self, const ExtrusionEntity &entity, size_t depth) -> bool {
        if (depth >= 64 || !emission_charge(object, budget)) return false;
        if (typeid(entity) == typeid(ExtrusionLoop) || typeid(entity) == typeid(ExtrusionMultiPath)) return true;
        if (typeid(entity) != typeid(ExtrusionEntityCollection)) return false;
        for (const auto *child : static_cast<const ExtrusionEntityCollection &>(entity).entities)
            if (!child || !self(self, *child, depth + 1)) return false;
        return true;
    };
    if (!joined_perimeters(joined_perimeters, region.perimeters, 0)) return false;

    // Stash only the original fill's owned child tree. Its extra live payload,
    // all snapshots and any projection share the existing emitter allowance.
    // Unknown/derived trees cannot be measured conservatively: use full trial.
    size_t remaining = emission_quota(layer);
    const auto admit = [&](size_t count, size_t size) {
        if (count > remaining / size) return false;
        remaining -= count * size;
        return true;
    };
    if (!admit(1, 512) || !admit(layer.bridge_aware_emission_snapshots.capacity(), sizeof(BridgeAwareCubic::EmissionSnapshotPtr)))
        return false;
    const auto &projection = saved->entries[0].projection;
    const auto snapshot = layer.bridge_aware_emission_snapshots.empty() ? nullptr : layer.bridge_aware_emission_snapshots.front();
    if (snapshot && !admit(snapshot->resident_bytes(), 1)) return false;
    if (projection && !admit(projection->snapshot() == snapshot ? projection->owned_bytes() : projection->resident_bytes(), 1))
        return false;
    const auto measure = [&](auto &&self, const ExtrusionEntity &entity, size_t depth) -> bool {
        if (depth >= 64 || !emission_charge(object, budget)) return false;
        if (typeid(entity) == typeid(ExtrusionPath)) {
            const auto &path = static_cast<const ExtrusionPath &>(entity);
            return path.polyline.fitting_result.empty() && admit(1, sizeof(ExtrusionPath) + 192) &&
                admit(path.polyline.points.capacity(), sizeof(Point)) &&
                admit(path.polyline.fitting_result.capacity(), sizeof(PathFittingData));
        }
        if (typeid(entity) != typeid(ExtrusionEntityCollection)) return false;
        const auto &collection = static_cast<const ExtrusionEntityCollection &>(entity);
        if (!admit(1, sizeof(ExtrusionEntityCollection) + 128) ||
            !admit(collection.entities.capacity(), sizeof(ExtrusionEntity *))) return false;
        for (const auto *child : collection.entities)
            if (!child || !self(self, *child, depth + 1)) return false;
        return true;
    };
    return measure(measure, region.fills, 0);
}

class EmissionReuseContext {
public:
    explicit EmissionReuseContext(const PrintObject &object)
    {
        if (active_profile && active_profile->full_trial_checks) return;
        const size_t layers = object.layers().size(), regions = object.num_printing_regions();
        if (layers == 0 || regions == 0 || regions > emission_allowance / emission_context_slot_bytes ||
            layers > emission_allowance / emission_context_slot_bytes / regions ||
            emission_quota(*object.layers().front()) < 2048) return;
        const size_t reserved = layers * regions * emission_context_slot_bytes;
        static_assert(sizeof(EmissionReuseRow) + sizeof(EmissionReuseEntry) + sizeof(size_t) < emission_context_slot_bytes);
        try {
            m_rows.resize(layers);
            size_t used = m_rows.capacity() * sizeof(EmissionReuseRow);
            if (used > reserved) { clear(); return; }
            for (auto &row : m_rows) {
                row.entries.resize(regions);
                row.retained.resize(regions);
                const size_t remaining = reserved - used;
                if (row.entries.capacity() > remaining / sizeof(EmissionReuseEntry)) { clear(); return; }
                used += row.entries.capacity() * sizeof(EmissionReuseEntry);
                if (row.retained.capacity() > (reserved - used) / sizeof(size_t)) { clear(); return; }
                used += row.retained.capacity() * sizeof(size_t);
            }
            // One optional diagnostic receipt, outside all per-emitter
            // quotas, may borrow only already reserved, unused row bytes.
            m_audit_slack = reserved - used > 512 ? reserved - used - 512 : 0;
        } catch (const std::bad_alloc &) { clear(); }
    }

    size_t audit_slack() const noexcept { return std::min(m_audit_slack, size_t(32 * 1024)); }
    EmissionReuseRow *row(size_t index) noexcept { return index < m_rows.size() ? &m_rows[index] : nullptr; }

    void save(const PrintObject &object, const std::set<size_t> &changed,
              const std::vector<std::unique_ptr<BridgeAwareCubic::Plan>> &previous_plans)
    {
        BridgeAwareCubicValidation::WorkBudget budget;
        for (size_t index : changed) {
            auto *saved = row(index);
            if (!saved) continue;
            const Layer &layer = *object.layers()[index];
            for (size_t emitter = 0; emitter < saved->entries.size(); ++emitter) {
                auto &entry = saved->entries[emitter];
                entry.base = emitter < layer.bridge_aware_emission_snapshots.size() ?
                    layer.bridge_aware_emission_snapshots[emitter] : nullptr;
                if (entry.projection && entry.projection->snapshot() == entry.base)
                    entry.base_projection = std::move(entry.projection);
                else { entry.projection.reset(); entry.base_projection.reset(); }
                const size_t raw = entry.base ? entry.base->resident_bytes() : 0;
                const size_t derived = entry.base_projection ? entry.base_projection->owned_bytes() : 0;
                const size_t quota = emission_quota(layer);
                // The live fills still belong to the previous Plans. A
                // snapshot left untouched by an earlier accepted trial may
                // carry an older Plan token: do not save it as a usable base.
                // Check every previous namespace without copying its table.
                bool baseline = entry.base != nullptr;
                size_t binding = 0;
                if (baseline) {
                    const auto &bindings = entry.base->plan_bindings();
                    for (size_t owner = 0; owner < previous_plans.size(); ++owner) if (previous_plans[owner]) {
                        if (!emission_charge(object, budget) || binding == bindings.size() ||
                            bindings[binding].storage_region != owner ||
                            bindings[binding].identity != previous_plans[owner]->immutable_identity()) {
                            baseline = false;
                            break;
                        }
                        ++binding;
                    }
                    baseline = baseline && binding == bindings.size() &&
                        current_emission_matches(layer, emitter, entry.base, derived, budget, &bindings);
                }
                if (!baseline || raw > quota || derived > quota - raw) {
                    entry.base.reset(); entry.base_projection.reset(); saved->retained[emitter] = 0;
                } else saved->retained[emitter] = raw + derived;
            }
        }
    }

    void accept(const std::set<size_t> &changed) noexcept
    {
        for (size_t index : changed) if (auto *saved = row(index))
            for (size_t emitter = 0; emitter < saved->entries.size(); ++emitter) {
                saved->entries[emitter].base.reset();
                saved->entries[emitter].base_projection.reset();
                saved->retained[emitter] = 0;
            }
    }

    void discard_candidate(const std::set<size_t> &changed) noexcept
    {
        for (size_t index : changed) if (auto *saved = row(index))
            for (auto &entry : saved->entries) entry.projection.reset();
    }

    // Call only after physical regeneration under restored Plans and payloads.
    // A failure publishes no saved metadata; ordinary reference collection is
    // always available. No optional handle authorizes a deposition change.
    void restore(Layer &layer, size_t index)
    {
        auto *saved = row(index);
        if (!saved) return;
        try {
            layer.bridge_aware_emission_snapshots.resize(layer.regions().size());
            BridgeAwareCubicValidation::WorkBudget budget;
            for (size_t emitter = 0; emitter < saved->entries.size(); ++emitter) {
                auto &entry = saved->entries[emitter];
                if (!entry.base || emitter >= layer.regions().size()) continue;
                const size_t derived = entry.base_projection ? entry.base_projection->owned_bytes() : 0;
                if (current_emission_matches(layer, emitter, entry.base, derived, budget)) {
                    layer.bridge_aware_emission_snapshots[emitter] = entry.base;
                    entry.projection = std::move(entry.base_projection);
                    if (active_profile) ++active_profile->emission_rollback_restored;
                } else if (active_profile) ++active_profile->emission_rollback_mismatches;
            }
        } catch (const std::bad_alloc &) {
            layer.bridge_aware_emission_snapshots.clear();
            for (auto &entry : saved->entries) entry.projection.reset();
        }
    }
private:
    std::vector<EmissionReuseRow> m_rows;
    size_t m_audit_slack = 0;
    void clear() noexcept { std::vector<EmissionReuseRow>().swap(m_rows); }
};

BridgeAwareCubic::EmissionProjectionPtr current_emission_projection(const Layer &layer, size_t emitter,
    BridgeAwareCubicValidation::WorkBudget &budget, EmissionReuseRow *reuse)
{
    if (emitter >= layer.bridge_aware_emission_snapshots.size()) return {};
    const auto &snapshot = layer.bridge_aware_emission_snapshots[emitter];
    if (!snapshot) return {};
    auto *entry = reuse && emitter < reuse->entries.size() ? &reuse->entries[emitter] : nullptr;
    const size_t retained = entry ? reuse->retained[emitter] : 0;
    // A stale projection retains another raw allocation. Evict before either
    // live matching or generating a replacement reference; saved base storage
    // remains explicitly charged by retained.
    if (entry && entry->projection && entry->projection->snapshot() != snapshot) entry->projection.reset();
    const size_t derived = entry && entry->projection ? entry->projection->owned_bytes() : 0;
    const size_t quota = emission_quota(layer);
    if (retained > quota || derived > quota - retained ||
        !current_emission_matches(layer, emitter, snapshot, retained + derived, budget)) return {};
    if (entry && entry->projection) {
        if (active_profile) ++active_profile->emission_projection_reuses;
        return entry->projection;
    }
    try {
        const auto *plan = layer.object()->bridge_aware_cubic_plan(emitter);
        if (!plan || snapshot->resident_bytes() > quota - retained) return {};
        // Fresh live-view buffers have already been released. Reference and
        // projection staging must fit beside BOTH retained raw generations.
        const size_t available = quota - retained - snapshot->resident_bytes();
        auto reference = plan->reference_lines(layer.print_z, snapshot->reference().classification_bounds,
            available / sizeof(Point), &budget);
        if (!reference.complete) return {};
        auto projection = BridgeAwareCubic::project_emissions(snapshot, std::move(reference), {quota, retained}, &budget,
            [&] { if (layer.object()->print()->canceled()) throw CanceledException(); });
        if (projection) {
            if (active_profile) ++active_profile->emission_projection_builds;
            if (entry) entry->projection = projection;
        }
        return projection;
    } catch (const std::bad_alloc &) {}
    return {};
}

std::vector<Target> collect_targets(const PrintObject &object, bool dependencies, bool unsupported_only = true,
                                    const std::set<size_t> *selected_layers = nullptr,
                                    ConstructionCollection construction = ConstructionCollection::Reference,
                                    BridgeAwareCubic::EmissionCollectionAudit *audit = nullptr,
                                    EmissionReuseContext *reuse = nullptr,
                                    BridgeAwareCubicValidation::WorkBudget *supplied_budget = nullptr)
{
    std::vector<Target> targets;
    BridgeAwareCubicValidation::WorkBudget local_budget;
    auto &contact_budget = supplied_budget ? *supplied_budget : local_budget;
    ProfileScope profile(dependencies ? ProfilePhase::CollectConstruction : ProfilePhase::CollectModel, &contact_budget);
    const auto &layers = object.layers();
    const BoundingBox bounds = object.bounding_box();
    const ExPolygon broad_domain(Polygon{Points{
        bounds.min, Point(bounds.max.x(), bounds.min.y()), bounds.max,
        Point(bounds.min.x(), bounds.max.y())}});
    for (size_t index = 1; index < layers.size(); ++index) {
        if (selected_layers != nullptr && selected_layers->count(index) == 0) continue;
        profile.layer(index);
        if (object.print()->canceled()) throw CanceledException();
        const Layer &upper = *layers[index];
        const Layer &lower = *layers[index - 1];
        const ActiveAttachments active = active_attachments(upper);
        std::vector<BridgeAwareCubicContact::Bead> prior_beads;
        visit_layer(lower, [&](const ExtrusionPath &path) {
            profile.lower(path.polyline.size() - 1);
            prior_beads.push_back({&path, deposition_z(lower)});
        });
        // Collection visits many upper strands above the same immutable lower
        // deposition. Reuse one owned section snapshot at its exact top/height
        // key; replacing it first preserves the helper's combined resident and
        // query allocation allowance. Rebuilds charge this collection's budget.
        BridgeAwareCubicContact::PreparedLowerPtr prepared_lower;
        std::unique_ptr<BridgeAwareCubicContact::QueryCache> contact_queries;
        std::optional<std::pair<double, float>> prepared_key;
        const double upper_z = deposition_z(upper);
        std::vector<ExPolygons> eligible(lower.regions().size());
        std::vector<ExPolygons> bridge_eligible(lower.regions().size());
        for (size_t region_id = 0; region_id < lower.regions().size(); ++region_id) {
            if (participates(lower.get_region(int(region_id))->region().config()))
                eligible[region_id] = sparse_domain(*lower.get_region(int(region_id)));
        }
        if (!dependencies) {
            // Carry the originating sparse region's policy through bridge
            // planes, including across a modifier ending below a bridge. The
            // bridge's own region is not necessarily the support owner.
            const auto charge = [&](size_t amount = 1) {
                if (object.print()->canceled()) throw CanceledException();
                if (amount > contact_budget.remaining_work)
                    throw SlicingError("Bridge aware cubic bridge ancestry exceeded its geometry work limit.",
                                       object.model_object()->id().id);
                contact_budget.remaining_work -= amount;
            };
            const auto charge_polygons = [&](const ExPolygons &polygons) {
                for (const ExPolygon &polygon : polygons) {
                    charge(polygon.contour.size() + 1);
                    for (const Polygon &hole : polygon.holes) charge(hole.size() + 1);
                }
            };
            ExPolygons bridge_components;
            for (const LayerRegion *region : lower.regions())
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == stInternalBridge)
                        bridge_components.push_back(surface.expolygon);
            charge_polygons(bridge_components);
            bridge_components = union_ex(bridge_components);
            std::vector<ExPolygons> upper_footprints;
            if (!bridge_components.empty())
                visit_layer(upper, [&](const ExtrusionPath &path) {
                    if (path.role() == erInternalInfill || path.role() == erIroning) return;
                    charge(path.polyline.size() + 1);
                    upper_footprints.push_back(union_ex(offset(Polylines{path.polyline}, scale_(.5 * path.width))));
                    charge_polygons(upper_footprints.back());
                });
            for (const ExPolygon &component : bridge_components) {
                // Expand policy selection to real connected upper paths;
                // deposited support and physical contact remain unchanged.
                ExPolygons coverage{component};
                for (const ExPolygons &footprint : upper_footprints) {
                    charge_polygons(footprint);
                    charge(component.contour.size() + 1);
                    if (!intersection_ex(footprint, ExPolygons{component}).empty()) append(coverage, footprint);
                }
                charge_polygons(coverage);
                coverage = union_ex(coverage);
                ExPolygons projection{component};
                for (size_t bridge_index = index - 1; bridge_index > 0 && !projection.empty(); --bridge_index) {
                    ExPolygons bases, continuing;
                    for (const LayerRegion *region : layers[bridge_index]->regions())
                        for (const Surface &surface : region->fill_surfaces.surfaces) {
                            charge();
                            if (surface.surface_type != stInternalBridge) continue;
                            charge_polygons(projection);
                            charge_polygons(ExPolygons{surface.expolygon});
                            if (!intersection_ex(surface.expolygon, projection).empty())
                                (surface.bridge_followup ? continuing : bases).push_back(surface.expolygon);
                        }
                    if (!bases.empty()) {
                        const Layer &beneath_base = *layers[bridge_index - 1];
                        for (size_t region_id = 0; region_id < beneath_base.regions().size(); ++region_id) {
                            const LayerRegion &source = *beneath_base.get_region(int(region_id));
                            if (!participates(source.region().config())) continue;
                            const ExPolygons sparse = sparse_domain(source);
                            charge_polygons(bases);
                            charge_polygons(sparse);
                            if (intersection_ex(bases, sparse).empty()) continue;
                            // Sparse interiors establish ownership, not the
                            // extent of an inherited path's anchor obligation.
                            // Keep the source's complete material section so
                            // wall/anchor fringes survive, including when a
                            // modifier ends below the reconstructed stack.
                            ExPolygons material;
                            for (const Surface &surface : source.slices.surfaces) material.push_back(surface.expolygon);
                            charge_polygons(material);
                            charge_polygons(coverage);
                            append(bridge_eligible[region_id], intersection_ex(coverage, union_ex(material)));
                        }
                    }
                    charge_polygons(continuing);
                    projection = union_ex(continuing);
                }
            }
        }
        for (ExPolygons &domain : eligible) domain = union_ex(domain);
        for (ExPolygons &domain : bridge_eligible) domain = union_ex(domain);

        std::map<std::tuple<size_t, double, bool>, size_t> groups;
        auto add = [&](const ExtrusionPath &path, bool dependency, size_t emitting_region, bool free_ended,
                       const std::vector<BridgeAwareCubic::CellId> &producers,
                       const ExPolygons *selection = nullptr, bool solid_cut = false) {
            std::optional<ProfileScope> detail_profile;
            if (dependency && active_profile) detail_profile.emplace(ProfilePhase::ConstructionAdd);
            profile.upper(path.polyline.size() - 1);
            // A swept bead edge alone is not an unsupported extrusion. Select
            // actual air runs first and then use the real target bead width.
            // The original path below retains all endpoints and identities.
            Polylines unsupported;
            if (unsupported_only) {
                const bool finite_key = std::isfinite(upper_z) && std::isfinite(path.height);
                const auto key = std::make_pair(upper_z, path.height);
                if (!prepared_lower || !finite_key || !prepared_key || *prepared_key != key) {
                    contact_queries.reset();
                    prepared_lower.reset();
                    prepared_key.reset();
                    prepared_lower = BridgeAwareCubicContact::prepare_lower(prior_beads, upper_z, path.height, &contact_budget);
                    if (finite_key) {
                        prepared_key = key;
                        contact_queries = make_contact_query_cache(prepared_lower);
                    }
                }
                const auto contact = BridgeAwareCubicContact::footprint(*prepared_lower, path, upper_z, geometry_tolerance,
                    BridgeAwareCubicContact::FootprintMode::ContactsOnly, &contact_budget, contact_queries.get());
                profile.plane(index, 0, contact.contact_z);
                if (contact.resource_limited)
                    throw SlicingError("Bridge aware cubic target collection is incomplete at layer " + std::to_string(upper.id() + 1) +
                        ": " + contact.failure_reason, object.model_object()->id().id);
                if (contact.input_valid && contact.completed) {
                    for (const auto &segment : contact.segments) {
                        const size_t i = segment.segment_index;
                        append(unsupported, diff_pl(Polylines{Polyline(Points{path.polyline.points[i], path.polyline.points[i + 1]})},
                                                    segment.centerline_support));
                    }
                    if (contact.segments.empty()) unsupported.push_back(path.polyline);
                } else unsupported.push_back(path.polyline);
            } else unsupported.push_back(path.polyline);
            if (unsupported.empty()) return;
            ExPolygons footprint = union_ex(offset(unsupported, scale_(.5 * path.width)));
            if (selection != nullptr) footprint = intersection_ex(footprint, *selection);
            if (footprint.empty()) return;
            for (size_t region_id = 0; region_id < eligible.size(); ++region_id) {
                for (bool followup : {false, true}) {
                    const ExPolygons &domain = followup ? bridge_eligible[region_id] : eligible[region_id];
                    if (domain.empty()) continue;
                    ExPolygons demand = intersection_ex(footprint, domain);
                    if (demand.empty()) continue;
                    const double bottom_z = upper.print_z - path.height;
                    const auto key = std::make_tuple(region_id, bottom_z, followup);
                    auto found = groups.find(key);
                    if (found == groups.end()) {
                        const auto &config = lower.get_region(int(region_id))->region().config();
                        found = groups.emplace(key, targets.size()).first;
                        targets.push_back({index, region_id, config.bridge_aware_cubic_max_gap.value,
                                           bottom_z, dependency, followup, {}, {}, {}});
                    }
                    Target &target = targets[found->second];
                    target.paths.push_back(path.polyline);
                    target.path_widths.push_back(path.width);
                    target.extrusions.push_back(path);
                    target.emitting_regions.push_back(emitting_region);
                    target.free_ended.push_back(free_ended);
                    target.producers.push_back(producers);
                    target.solid_cuts.push_back(solid_cut);
                    target.path_footprints.push_back(demand);
                    append(target.footprint, std::move(demand));
                }
            }
            // An added strand outside every eligible lower domain must still
            // have real prior material. A forbidden region is never filled to
            // hide an infeasible construction.
            if (dependency) {
                ExPolygons all_eligible;
                for (const ExPolygons &domain : eligible) append(all_eligible, domain);
                ExPolygons outside = diff_ex(footprint, all_eligible);
                if (!outside.empty()) {
                    const auto &config = upper.get_region(int(emitting_region))->region().config();
                    targets.push_back({index, emitting_region, config.bridge_aware_cubic_max_gap.value,
                                       upper.print_z - path.height, true, false, {path.polyline},
                                       {double(path.width)}, std::move(outside)});
                    Target &target = targets.back();
                    target.extrusions.push_back(path);
                    target.emitting_regions.push_back(emitting_region);
                    target.free_ended.push_back(free_ended);
                    target.producers.push_back(producers);
                    target.solid_cuts.push_back(solid_cut);
                    target.path_footprints.push_back(target.footprint);
                }
            }
        };

        if (!dependencies) {
            for (size_t region_id = 0; region_id < upper.regions().size(); ++region_id) {
                const LayerRegion &region = *upper.get_region(int(region_id));
                std::set<const ExtrusionPath*> free;
                // Model perimeters are frozen before this preparation step
                // and can survive an infill-only reslice. Repair their support
                // below; never retain a path extension in that persistent tree.
                visit_free_paths(region.fills, [&](const ExtrusionPath &path) { free.insert(&path); });
                const auto model_path = [&](const ExtrusionPath &path) {
                    if (path.role() != erInternalInfill && path.role() != erIroning)
                        add(path, false, region_id, free.count(&path) != 0, {});
                };
                visit_paths(region.perimeters, model_path);
                visit_paths(region.fills, model_path);
            }
        } else {
            for (size_t region_id = 0; region_id < upper.regions().size(); ++region_id) {
                const auto *plan = object.bridge_aware_cubic_plan(region_id);
                if (plan == nullptr) continue;
                if (construction == ConstructionCollection::Emitted &&
                    !(active_profile && active_profile->full_trial_checks)) {
                    if (const auto projection = current_emission_projection(upper, region_id, contact_budget,
                            reuse ? reuse->row(index) : nullptr)) {
                        if (audit) ++audit->ledger_emitters;
                        if (active_profile) ++active_profile->emission_ledger_hits;
                        const auto &raw = projection->snapshot()->records();
                        for (size_t path_index = 0; path_index < projection->records().size(); ++path_index) {
                            const auto &record = projection->records()[path_index];
                            if (!record.construction) continue;
                            if (audit) ++audit->ledger_construction_paths;
                            if (active_profile) ++active_profile->emission_ledger_construction_paths;
                            std::vector<BridgeAwareCubic::CellId> producers = record.partition_owners;
                            const std::set<size_t> lineage(record.attachment_indices.begin(), record.attachment_indices.end());
                            for (size_t id : lineage)
                                if (active.storage_regions.at(id) == region_id)
                                    append(producers, active.entries.at(id).owners);
                            const auto selection = BridgeAwareCubic::merged_attachment_selection(
                                record.mask_full_path, active.entries, lineage);
                            add(raw.at(path_index).path, true, region_id, true, producers,
                                selection ? &*selection : nullptr, selection.has_value());
                        }
                        // Closure layers are excluded during capture, including
                        // foreign storage/emitter arrangements. All other
                        // policy and physical checking still uses add() above.
                        continue;
                    }
                    if (audit) ++audit->reference_emitters;
                    if (active_profile) ++active_profile->emission_ledger_fallbacks;
                }
                std::optional<ProfileScope> sections_profile;
                if (active_profile) sections_profile.emplace(ProfilePhase::ConstructionSections);
                const Polylines added = plan->refinement_lines(upper.print_z, broad_domain);
                if (added.empty() && plan->solid_area(upper.print_z).empty() && plan->attachments(upper.print_z).empty()) continue;
                const ExPolygons solid_area = plan->solid_area(upper.print_z);
                Polygons added_mask = offset(added, scale_(geometry_tolerance));
                append(added_mask, to_polygons(solid_area));
                std::optional<BridgeAwareCubic::StrictMaskContainment> strict_mask;
                if (!(active_profile && active_profile->full_trial_checks)) {
                    strict_mask.emplace(added_mask, &contact_budget,
                        [&] { if (object.print()->canceled()) throw CanceledException(); });
                    if (active_profile && strict_mask->available()) ++active_profile->construction_containment_indices;
                }
                const BridgeAwareCubic::BoundedClipMask added_clip_mask(std::move(added_mask));
                const auto &saved_attachments = active.entries;
                const auto solid_sections = plan->solid_sections(upper.print_z);
                const auto owner_sections = plan->partition_lines(upper.print_z, broad_domain);
                std::vector<BridgeAwareCubic::PartitionOwnerBounds> owner_bounds;
                // Optional broad-phase metadata is admitted as one bounded
                // layer-local batch. Allocation/size/work fallback retains the
                // original exact owner scan; no owner is lost to cache limits.
                constexpr size_t max_owner_bounds = 20000, max_owner_vertices = 200000;
                size_t owner_vertices = 0;
                bool bounded = owner_sections.size() <= max_owner_bounds;
                for (const auto &owner : owner_sections) {
                    if (!bounded) break;
                    for (const Polyline &line : owner.second) {
                        if (line.size() > max_owner_vertices - owner_vertices) { bounded = false; break; }
                        owner_vertices += line.size();
                    }
                }
                if (bounded && owner_vertices <= contact_budget.remaining_work) {
                    try {
                        owner_bounds.reserve(owner_sections.size());
                        for (const auto &owner : owner_sections) {
                            if (object.print()->canceled()) throw CanceledException();
                            owner_bounds.emplace_back(owner.second);
                        }
                        contact_budget.remaining_work -= owner_vertices;
                        contact_budget.charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += owner_vertices;
                    } catch (const std::bad_alloc &) { owner_bounds.clear(); }
                }
                sections_profile.reset();
                visit_paths(upper.get_region(int(region_id))->fills, [&](const ExtrusionPath &path) {
                    if (path.role() != erInternalInfill) return;
                    std::optional<ProfileScope> match_profile;
                    if (active_profile) match_profile.emplace(ProfilePhase::ConstructionMatch);
                    // This classifies the complete original strand only. The
                    // actual mask's winding and strict boundary clearance can
                    // establish full containment without clipping; it supplies
                    // no deposited-material support or cell-wall evidence.
                    bool full_containment = false;
                    if (strict_mask && strict_mask->available()) {
                        if (active_profile) ++active_profile->construction_containment_queries;
                        full_containment = strict_mask->proves(path.polyline, &contact_budget,
                            [&] { if (object.print()->canceled()) throw CanceledException(); });
                        if (active_profile && full_containment) ++active_profile->construction_containment_proofs;
                    }
                    const Polylines matched = full_containment ? Polylines{} :
                        intersection_pl(Polylines{path.polyline}, added_clip_mask.candidates(path.polyline));
                    // Full emitted strand identity, not a clipped fragment.
                    bool attachment_match = false;
                    std::set<size_t> attachment_lineage;
                    for (const auto &attachment : saved_attachments)
                        if (attachment.emitting_region == region_id) {
                            const auto chain = resolve_attachments(upper, attachment.original, path.width, path.height, region_id, active);
                            if (!chain.applied_indices.empty() && same_polyline(path.polyline, chain.polyline)) {
                                attachment_match = true;
                                attachment_lineage.insert(chain.applied_indices.begin(), chain.applied_indices.end());
                            }
                        }
                    const bool ordinary_full_path = full_containment ||
                        BridgeAwareCubic::complete_construction_match(path.polyline, matched, geometry_tolerance);
                    match_profile.reset();
                    if (attachment_match || ordinary_full_path) {
                        std::optional<ProfileScope> owners_profile;
                        if (active_profile) owners_profile.emplace(ProfilePhase::ConstructionOwners);
                        std::vector<BridgeAwareCubic::CellId> producers;
                        const bool use_bounds = owner_bounds.size() == owner_sections.size() &&
                            path.polyline.size() <= contact_budget.remaining_work;
                        const auto path_bounds = use_bounds ?
                            BridgeAwareCubic::PartitionCandidateBounds(path.polyline) :
                            BridgeAwareCubic::PartitionCandidateBounds(Polyline{});
                        if (use_bounds) {
                            contact_budget.remaining_work -= path.polyline.size();
                            contact_budget.charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += path.polyline.size();
                        }
                        for (size_t owner_index = 0; owner_index < owner_sections.size(); ++owner_index) {
                            if (use_bounds && !owner_bounds[owner_index].may_match(path_bounds)) continue;
                            const auto &[owner_id, owner_lines] = owner_sections[owner_index];
                            const auto pieces = remove_collinear_duplicates({path.polyline}, owner_lines, true);
                            if (BridgeAwareCubic::complete_construction_match(path.polyline, pieces, 2. * SCALING_FACTOR))
                                producers.push_back(owner_id);
                        }
                        for (const auto &solid : solid_sections)
                            if (diff_pl(Polylines{path.polyline}, ExPolygons{ExPolygon(solid.polygon)}).empty())
                                producers.push_back(solid.cell_id);
                        for (size_t id : attachment_lineage)
                            if (active.storage_regions[id] == region_id)
                                append(producers, saved_attachments[id].owners);
                        const auto attachment_selection = BridgeAwareCubic::merged_attachment_selection(
                            ordinary_full_path, saved_attachments, attachment_lineage);
                        owners_profile.reset();
                        add(path, true, region_id, true, producers,
                            attachment_selection ? &*attachment_selection : nullptr, attachment_selection.has_value());
                    } else if (!solid_area.empty()) {
                        // Replacing cubic with a solid patch also cuts the
                        // inherited bulk lines. A new terminal point is a
                        // construction change even though the remaining line
                        // was not itself an added midplane. Select only this
                        // terminal neighbourhood; the validator retains the
                        // entire intersecting air run and both real endpoints.
                        std::vector<BridgeAwareCubic::CellId> producers;
                        ExPolygons terminal_selection;
                        for (const auto &solid : solid_sections) {
                            ExPolygons selected = BridgeAwareCubic::solid_cut_endpoint_demand(path.polyline,
                                {ExPolygon(solid.polygon)}, path.width, geometry_tolerance);
                            if (!selected.empty()) {
                                append(terminal_selection, std::move(selected));
                                producers.push_back(solid.cell_id);
                            }
                        }
                        if (!terminal_selection.empty()) {
                            terminal_selection = union_ex(terminal_selection);
                            add(path, true, region_id, true, producers, &terminal_selection, true);
                        }
                    }
                });
                // A closure may be stored with its demand owner while a
                // different compatible region emits a predecessor course.
                for (const auto &closure : plan->closures(upper.print_z)) {
                    const size_t emitter = closure.emitting_region == std::numeric_limits<size_t>::max() ? region_id : closure.emitting_region;
                    if (emitter == region_id || emitter >= upper.regions().size()) continue;
                    Polylines emitted_closure = closure.paths;
                    const Flow closure_flow = upper.get_region(int(emitter))->flow(frInfill);
                    for (Polyline &line : emitted_closure)
                        line = resolve_attachments(upper, line, float(closure_flow.width()), float(closure_flow.height()), emitter, active).polyline;
                    const Polygons closure_mask = offset(emitted_closure, scale_(geometry_tolerance));
                    visit_paths(upper.get_region(int(emitter))->fills, [&](const ExtrusionPath &path) {
                        if (path.role() != erInternalInfill) return;
                        const auto matched = intersection_pl(Polylines{path.polyline}, closure_mask);
                        if (BridgeAwareCubic::complete_construction_match(path.polyline, matched, geometry_tolerance))
                            // Cell IDs belong to the storage lattice. A course
                            // emitted elsewhere retains its dependency through
                            // the actual active closure path (ClosureRoot), not
                            // by interpreting foreign IDs in the emitter plan.
                            add(path, true, emitter, true, {});
                    });
                }
            }
        }
        for (const auto &group : groups)
            targets[group.second].footprint = union_ex(targets[group.second].footprint);
    }
    profile.items(targets.size());
    return targets;
}

struct Failure {
    Target target;
    BridgeAwareCubicValidation::Result result;
    Polylines verified_paths;
};

enum class TargetCheck { CollectFailures, RejectTrial };
struct PreparedModelView;

// Sequential coarsening only. Each exact plane/phase slot owns its last
// observed complete material, which may belong to a rejected trial. A later
// observation must compare that material with freshly prepared actual fills;
// restoring Plans never relabels a proof. Only the active plane retains a
// previous epoch and delta. Unvisited old bindings are demoted before release.
class ContactEpochReuse {
    using Epoch = Contact::CanonicalLowerEpochPtr;
    using Cache = BridgeAwareCubic::SpanProofCache;
    using Key = std::tuple<size_t, double, double, double, double, double, double, double, float>;
    using Slots = std::map<Key, Epoch>;
    static constexpr size_t context_bytes = 512, slot_bytes = 256;
    const PrintObject &m_object;
    Cache &m_cache;
    Slots m_slots;
    Epoch m_previous;
    Contact::CanonicalLowerDeltaPtr m_transition;

    size_t metadata(size_t extra_slots = 0) const noexcept
    { return context_bytes + (m_slots.size() + extra_slots) * slot_bytes; }

    std::optional<Cache::EpochContext> unavailable() const noexcept
    {
        if (active_profile) ++active_profile->contact_epoch_fallbacks;
        return {};
    }

    void note_peak(size_t bytes) const noexcept
    {
        if (active_profile)
            active_profile->contact_epoch_peak_bytes = std::max<uint64_t>(active_profile->contact_epoch_peak_bytes, bytes);
    }

public:
    ContactEpochReuse(const PrintObject &object, Cache &cache) : m_object(object), m_cache(cache)
    {
        // Include tree-node links/allocator overhead and temporary node/handle
        // views conservatively. Registry nodes live in the existing fact cap.
        static_assert(sizeof(Slots::value_type) + 8 * sizeof(void *) <= slot_bytes);
        static_assert(sizeof(ContactEpochReuse) + sizeof(Cache::EpochContext) + 8 * sizeof(Epoch) <= context_bytes);
    }

    std::optional<Cache::EpochContext> observe(size_t lower_layer,
        const BridgeAwareCubic::SerializedXY &phase, double upper_z, float upper_height,
        const Contact::PreparedLowerPtr &prepared, Contact::QueryCache *queries,
        BridgeAwareCubicValidation::WorkBudget &budget)
    {
        const auto cancel = [&] { if (m_object.print()->canceled()) throw CanceledException(); };
        try {
            cancel();
            if (!prepared || !std::isfinite(upper_z) || !std::isfinite(upper_height) ||
                !phase.shift.allFinite() || !phase.nozzle_offset.allFinite() || !phase.plate_origin.allFinite())
                return unavailable();
            // Retirement precharges its complete nonallocating scan. If it
            // cannot finish, retain both registry entries and their accounting;
            // do not accumulate a second pending transition.
            if (m_previous && !m_cache.retire_epochs({m_previous}, &budget, cancel)) return unavailable();
            m_previous.reset();
            m_transition.reset();

            size_t levels = 1;
            for (size_t count = m_slots.size(); count > 0; count >>= 1) ++levels;
            // Bound red-black-tree comparisons, complete tuple keys, staging
            // and commit before any optional mutation.
            if (!emission_charge(m_object, budget, 32 * (2 * levels + 2))) return unavailable();
            const Key key{lower_layer, phase.shift.x(), phase.shift.y(), phase.nozzle_offset.x(),
                phase.nozzle_offset.y(), phase.plate_origin.x(), phase.plate_origin.y(), upper_z, upper_height};
            auto found = m_slots.find(key);
            const bool new_slot = found == m_slots.end();
            if (new_slot && m_slots.size() >= (contact_epoch_allowance - context_bytes) / slot_bytes)
                return unavailable();
            const size_t meta = metadata(new_slot ? 1 : 0);
            const size_t retained = m_cache.statistics().epoch_resident_bytes;
            if (retained > contact_epoch_allowance - meta) return unavailable();

            const Epoch before = new_slot ? Epoch{} : found->second;
            if (before && Contact::lower_epoch_source_matches(*before, prepared, phase))
                return Cache::EpochContext{prepared, before, {},
                    {contact_epoch_allowance, meta + retained - Contact::lower_epoch_resident_bytes(*before), 0}};

            // Every role/region has already participated in prepare_lower.
            // The capture admits new storage beside every old registered epoch.
            Epoch after = Contact::capture_lower_epoch(prepared, phase,
                {contact_epoch_allowance, meta + retained, 0}, &budget, cancel, queries);
            if (!after) return unavailable();
            if (active_profile) ++active_profile->contact_epoch_captures;
            const size_t after_bytes = Contact::lower_epoch_resident_bytes(*after);
            const size_t before_bytes = before ? Contact::lower_epoch_resident_bytes(*before) : 0;
            if (before_bytes > retained || after_bytes > contact_epoch_allowance - meta - retained)
                return unavailable();
            note_peak(meta + retained + after_bytes);
            Contact::CanonicalLowerDeltaPtr difference;
            if (before) {
                const size_t other_contact = Contact::prepared_lower_contact_units(*prepared) +
                    (queries ? queries->statistics().resident_units : 0);
                difference = Contact::compare_lower_epochs(before, after,
                    {contact_epoch_allowance, meta + retained - before_bytes, other_contact}, &budget, cancel);
                // An unavailable difference grants no transport. Fresh passed
                // checks may still bind to the new complete material identity.
                if (difference && active_profile) ++active_profile->contact_epoch_transitions;
            }
            const size_t delta_bytes = difference ? Contact::lower_delta_owned_bytes(*difference) : 0;
            if (delta_bytes > contact_epoch_allowance - meta - retained - after_bytes) return unavailable();
            note_peak(meta + retained + after_bytes + delta_bytes);

            // Allocate a missing tree node privately before registry admission.
            // The final transfer/replacement and handle publication cannot
            // allocate or poll after the registry has committed its reference.
            Slots staged;
            if (new_slot) staged.emplace(key, after);
            if (!m_cache.admit_epochs({after},
                    {contact_epoch_allowance, meta + delta_bytes, 0}, &budget, cancel)) return unavailable();
            if (new_slot) m_slots.insert(staged.extract(staged.begin()));
            else found->second = after;
            if (active_profile)
                active_profile->contact_epoch_peak_slots = std::max<uint64_t>(active_profile->contact_epoch_peak_slots, m_slots.size());
            m_previous = before;
            m_transition = difference;
            // Epoch operations charge their own before/after arguments once;
            // unrelated retained slots occupy only the object metadata share.
            // Without a delta there is no argument owning the still-retained
            // previous epoch, so it remains part of the OTHER storage.
            const size_t query_other = meta + retained - (difference ? before_bytes : 0);
            return Cache::EpochContext{prepared, std::move(after), std::move(difference),
                {contact_epoch_allowance, query_other, 0}};
        } catch (const std::bad_alloc &) {
            return unavailable();
        }
    }
};

struct TargetCheckContext {
    BridgeAwareCubicValidation::WorkBudget *budget = nullptr;
    const BridgeAwareCubic::ModelFailureReplay::Probe *probe = nullptr;
    const std::vector<size_t> *frozen_indices = nullptr;
    std::optional<BridgeAwareCubic::ModelFailureReplay::Hint> *first_failure = nullptr;
    bool *probe_checked = nullptr;
    BridgeAwareCubic::SpanProofCache *span_proofs = nullptr;
    ContactEpochReuse *contact_epochs = nullptr;
};

std::vector<Failure> check_targets(const PrintObject &object, const std::vector<Target> &targets,
                                   const PrintObject *phase_object = nullptr,
                                   TargetCheck mode = TargetCheck::CollectFailures,
                                   const PreparedModelView *prepared_model = nullptr,
                                   const TargetCheckContext *context = nullptr);

[[noreturn]] void fail(const PrintObject &object, const Failure &failure);

ExPolygons repair_footprint(const Failure &failure, bool complete_selected_runs = false)
{
    std::vector<ExPolygons> synthetic_demand;
    if (failure.target.path_footprints.empty() && !failure.target.paths.empty())
        synthetic_demand.push_back(path_demand(failure.target, 0));
    ExPolygons affected = BridgeAwareCubic::select_unsupported_run_demands(failure.target.paths,
        failure.verified_paths, failure.target.path_widths,
        failure.target.path_footprints.empty() ? synthetic_demand : failure.target.path_footprints,
        failure.result.unsupported_runs, geometry_tolerance, complete_selected_runs);
    ExPolygons cells;
    for (const auto &cell : failure.result.void_cells)
        if (!cell.closed || cell.exceeds_gap)
            append(cells, offset_ex(cell.geometry, scale_(geometry_tolerance)));
    // The grouped target may contain many unrelated passing paths. Only a
    // violating run or void selects repair. A selected complete air run may
    // require support across a region seam, while void-cell selection remains
    // local to the original demand and never turns a logical seam into a wall.
    append(affected, intersection_ex(union_ex(cells), failure.target.footprint));
    return complete_selected_runs ? union_ex(affected) :
        intersection_ex(union_ex(affected), failure.target.footprint);
}

std::vector<BridgeAwareCubicContact::Bead> deposited_beads(const Layer &layer)
{
    std::vector<BridgeAwareCubicContact::Bead> beads;
    visit_layer(layer, [&](const ExtrusionPath &path) { beads.push_back({&path, deposition_z(layer)}); });
    return beads;
}

bool repair_attachments(const PrintObject &object, const Failure &failure,
                        std::vector<std::vector<BridgeAwareCubic::LayerAttachment>> &attachments,
                        const ExPolygons *same_layer_walls = nullptr)
{
    const Layer &upper = *object.layers()[failure.target.upper_layer];
    const Layer &lower = *object.layers()[failure.target.upper_layer - 1];
    const auto prior = deposited_beads(lower);
    std::set<size_t> candidates;
    for (const auto &run : failure.result.unsupported_runs)
        if (run.kind != BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge)
            candidates.insert(run.path_index);
    bool repaired = false;
    for (size_t index : candidates) {
        if (!failure.target.free_ended.at(index)) continue;
        const size_t emitter = failure.target.emitting_regions.at(index);
        const ExtrusionPath &original = failure.target.extrusions.at(index);
        BridgeAwareCubicAttachment::TerminalAirLengths terminal_air;
        for (const auto &run : failure.result.unsupported_runs) {
            if (run.path_index != index || run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge) continue;
            if (run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::Floating || run.begin_mm == 0.)
                terminal_air.start_mm = std::max(terminal_air.start_mm.value_or(0.), run.length_mm);
            if (run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::Floating || run.begin_mm != 0.)
                terminal_air.end_mm = std::max(terminal_air.end_mm.value_or(0.), run.length_mm);
        }
        ExPolygons support, start_support, end_support;
        if (same_layer_walls != nullptr) {
            // A cell-closing extension starts in this already emitted strand
            // and terminates in real wall material on the same contact plane.
            // This is only a candidate-search mask. The replacement must still
            // pass the actual preceding-layer proof below.
            support = union_ex(offset(Polylines{original.polyline}, scale_(.5 * original.width)));
            start_support = end_support = *same_layer_walls;
        } else {
            const auto contact = BridgeAwareCubicContact::footprint(prior, original, deposition_z(upper), 0.,
                BridgeAwareCubicContact::FootprintMode::ContactsOnly);
            if (!contact.input_valid || !contact.completed) {
                continue;
            }
            for (const auto &segment : contact.segments) {
                append(support, segment.centerline_support);
                if (segment.segment_index == 0) start_support = segment.centerline_support;
                if (segment.segment_index + 2 == original.polyline.size()) end_support = segment.centerline_support;
            }
            // A contact mask is bounded to its queried segment. Query the actual
            // terminal direction over the permitted search distance so a real
            // prior strand just beyond the original endpoint is considered. The
            // probe is not deposited and supplies no material to the certificate.
            const auto probe_support = [&](bool start) {
                ExtrusionPath probe = original;
                const Point endpoint = start ? original.polyline.first_point() : original.polyline.last_point();
                const Point neighbour = start ? original.polyline.points[1] : original.polyline.points[original.polyline.size() - 2];
                const Vec2d direction = (endpoint - neighbour).cast<double>();
                const double length = direction.norm();
                if (length == 0.) return ExPolygons{};
                const Vec2d far = endpoint.cast<double>() + direction * (scale_(failure.target.gap) / length);
                constexpr double coordinate_limit = double(std::numeric_limits<coord_t>::max() / 4);
                if (!far.allFinite() || far.cwiseAbs().maxCoeff() > coordinate_limit) return ExPolygons{};
                const Point point(far.cast<coord_t>());
                probe.polyline = start ? Polyline(Points{point, neighbour}) : Polyline(Points{neighbour, point});
                const auto queried = BridgeAwareCubicContact::footprint(prior, probe, deposition_z(upper), 0.,
                    BridgeAwareCubicContact::FootprintMode::ContactsOnly);
                ExPolygons masks;
                if (queried.input_valid && queried.completed)
                    for (const auto &segment : queried.segments) append(masks, segment.centerline_support);
                return union_ex(masks);
            };
            if (terminal_air.start_mm) start_support = probe_support(true);
            if (terminal_air.end_mm) end_support = probe_support(false);
        }
        append(support, start_support);
        append(support, end_support);
        const ExPolygons qualified_support = union_ex(support);
        const ExPolygons allowed_domain = BridgeAwareCubic::endpoint_attachment_domain(upper, emitter);
        std::optional<Polyline> replacement;
        Polylines attempted;
        BridgeAwareCubicAttachment::TerminalAnchorIntervals interval_search;
        unsigned skipped_intervals = 0;
        std::optional<Failure> last_interval_failure;
        // The nearest source anchor can yield a very short new segment whose
        // direction changes after XY serialization. Try deeper points inside
        // the same real contact intervals; the helper caps each preference at
        // the interval midpoint and every proposal receives the full proof.
        for (;;) {
        bool retry_start = false, retry_end = false;
        for (double inset : {2. * geometry_tolerance, .5 * double(original.width), failure.target.gap}) {
            auto candidate = BridgeAwareCubicAttachment::attach_endpoints(original.polyline, qualified_support,
                allowed_domain, original.width, failure.target.gap, 0., inset,
                &start_support, &end_support, &terminal_air, &interval_search);
            if (!candidate) continue;
            if (std::any_of(attempted.begin(), attempted.end(), [&](const Polyline &previous) {
                    return same_polyline(previous, *candidate);
                })) continue;
            attempted.push_back(*candidate);
            ExtrusionPath extended = original;
            extended.polyline = *candidate;
            Target proposed;
            proposed.upper_layer = failure.target.upper_layer;
            proposed.supporting_region = failure.target.supporting_region;
            proposed.gap = failure.target.gap;
            proposed.dependency = true; // Complete path/anchor check; no new cell demand.
            proposed.paths = {*candidate};
            proposed.path_widths = {original.width};
            proposed.extrusions = {extended};
            proposed.footprint = union_ex(offset(Polylines{*candidate}, scale_(.5 * original.width)));
            proposed.path_footprints = {proposed.footprint};
            // The final proof uses the actual serialized path in every instance,
            // including each newly added terminal segment.
            const auto rejected = check_targets(object, {proposed});
            if (!rejected.empty()) {
                if (!rejected.front().result.input_valid || !rejected.front().result.completed)
                    fail(object, rejected.front());
                for (const auto &run : rejected.front().result.unsupported_runs) {
                    if (run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge) continue;
                    if (terminal_air.start_mm && (run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::Floating || run.begin_mm == 0.))
                        retry_start = true;
                    if (terminal_air.end_mm && (run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::Floating || run.begin_mm != 0.))
                        retry_end = true;
                }
                last_interval_failure = rejected.front();
                continue;
            }
            replacement = std::move(candidate);
            break;
        }
        if (replacement || (!retry_start && !retry_end)) break;
        const unsigned advance = unsigned(retry_start) + unsigned(retry_end);
        if (skipped_intervals + advance > 8) {
            last_interval_failure->result.completed = false;
            last_interval_failure->result.resource_limited = true;
            last_interval_failure->result.failure_reason =
                "Endpoint attachment exhausted its bounded real-anchor interval search before serialization-qualified support was found.";
            fail(object, *last_interval_failure);
        }
        // A thin first physical contact interval may disappear after output
        // rounding. Advance only endpoints rejected by the complete proof;
        // later real intervals remain subject to the original gap/domain.
        interval_search.start_skip += unsigned(retry_start);
        interval_search.end_skip += unsigned(retry_end);
        skipped_intervals += advance;
        }
        if (!replacement) {
            continue;
        }
        const bool changed_start = replacement->first_point() != original.polyline.first_point();
        const bool changed_end = replacement->last_point() != original.polyline.last_point();
        // Storage follows the producing lattice, while the saved emitter is
        // the region whose immutable ordinary path will be replayed.
        size_t owner = emitter;
        if (object.bridge_aware_cubic_plan(owner) == nullptr) owner = failure.target.supporting_region;
        auto &saved = attachments.at(owner);
        const bool duplicate = std::any_of(saved.begin(), saved.end(), [&](const auto &entry) {
            return entry.z == upper.print_z && entry.emitting_region == emitter && same_polyline(entry.original, original.polyline);
        });
        if (duplicate) continue;
        // A floating construction can require both terminals. Keep two exact
        // replay records instead of assigning either addition to the opposite
        // endpoint's alternative producers. Nothing is published until both
        // records are valid and the complete replacement has passed above.
        const auto make_record = [&](const Polyline &before, const Polyline &after)
                -> std::optional<BridgeAwareCubic::LayerAttachment> {
            BridgeAwareCubic::LayerAttachment attachment;
            attachment.z = upper.print_z;
            attachment.emitting_region = emitter;
            attachment.original = before;
            attachment.replacement = after;
            attachment.width = original.width;
            attachment.height = original.height;
            if (failure.target.dependency && owner == emitter) {
                // A merged emitted face may span several refined cells, so no
                // single cell owns the entire original line. Endpoint additions
                // inherit only producers whose actual interval reaches that end.
                if (const auto *plan = object.bridge_aware_cubic_plan(owner)) {
                    Polylines terminals;
                    if (attachment.original.first_point() != attachment.replacement.first_point())
                        terminals.emplace_back(Points{attachment.original.points[1], attachment.original.first_point()});
                    if (attachment.original.last_point() != attachment.replacement.last_point())
                        terminals.emplace_back(Points{attachment.original.points[attachment.original.size() - 2], attachment.original.last_point()});
                    const BoundingBox bounds = object.bounding_box();
                    const ExPolygon domain(Polygon{Points{bounds.min, Point(bounds.max.x(), bounds.min.y()), bounds.max,
                        Point(bounds.min.x(), bounds.max.y())}});
                    for (const auto &[owner_id, owner_lines] : plan->partition_lines(upper.print_z, domain)) {
                        for (const Polyline &terminal : terminals)
                            for (const Polyline &piece : remove_collinear_duplicates({terminal}, owner_lines, true))
                                if ((piece.first_point() - terminal.last_point()).cast<double>().norm() <= 2. ||
                                    (piece.last_point() - terminal.last_point()).cast<double>().norm() <= 2.)
                                    attachment.owners.push_back(owner_id);
                    }
                    for (const auto &cell : plan->solid_sections(upper.print_z))
                        for (const Polyline &terminal : terminals)
                            if (cell.polygon.contains(terminal.last_point())) attachment.owners.push_back(cell.cell_id);
                }
                std::sort(attachment.owners.begin(), attachment.owners.end());
                attachment.owners.erase(std::unique(attachment.owners.begin(), attachment.owners.end()), attachment.owners.end());
                if (attachment.owners.empty()) {
                    // Only an explicit persistent closure can have no cell owner.
                    // Do not turn an unrecognized construction extension into a
                    // permanent model demand merely to retain its support.
                    double residual = 0.;
                    for (const Polyline &piece : remove_collinear_duplicates({attachment.original},
                            BridgeAwareCubic::layer_closure_lines(upper))) residual += piece.length();
                    if (residual > scale_(geometry_tolerance)) {
                        return std::nullopt;
                    }
                }
            }
            attachment.solid_cut = failure.target.solid_cuts.at(index);
            if (attachment.solid_cut) {
                const ExPolygons original_bead = union_ex(offset(Polylines{original.polyline}, scale_(.5 * original.width)));
                const ExPolygons replacement_bead = union_ex(offset(Polylines{attachment.replacement}, scale_(.5 * original.width)));
                attachment.demand_selection = union_ex(intersection_ex(path_demand(failure.target, index), original_bead),
                                                       diff_ex(replacement_bead, original_bead));
            }
            return attachment;
        };
        std::vector<BridgeAwareCubic::LayerAttachment> staged;
        if (failure.target.dependency && changed_start && changed_end) {
            Polyline intermediate = original.polyline;
            intermediate.append_before(replacement->first_point());
            auto first = make_record(original.polyline, intermediate);
            auto second = make_record(intermediate, *replacement);
            if (!first || !second) continue;
            staged.push_back(std::move(*first));
            staged.push_back(std::move(*second));
        } else {
            auto entry = make_record(original.polyline, *replacement);
            if (!entry) continue;
            staged.push_back(std::move(*entry));
        }
        bool duplicate_stage = false;
        for (const auto &entry : staged)
            duplicate_stage = duplicate_stage || std::any_of(saved.begin(), saved.end(), [&](const auto &existing) {
                return existing.z == entry.z && existing.emitting_region == entry.emitting_region &&
                    same_polyline(existing.original, entry.original);
            });
        if (duplicate_stage) continue;
        append(saved, std::move(staged));
        repaired = true;
    }
    return repaired;
}

bool repair_wall_connections(const PrintObject &object, const Failure &failure,
                             std::vector<std::vector<BridgeAwareCubic::LayerAttachment>> &attachments)
{
    if (failure.target.dependency || failure.target.bridge_followup ||
        failure.target.upper_layer < 2 || failure.target.extrusions.empty()) return false;
    ExPolygons cells;
    for (const auto &cell : failure.result.void_cells)
        if (!cell.closed || cell.exceeds_gap) cells.push_back(cell.geometry);
    if (cells.empty()) return false;

    const size_t layer_index = failure.target.upper_layer - 1;
    const Layer &layer = *object.layers()[layer_index];
    const size_t emitter = failure.target.supporting_region;
    const LayerRegion &region = *layer.get_region(int(emitter));
    if (!participates(region.region().config())) return false;
    std::vector<BridgeAwareCubicContact::Bead> wall_beads;
    visit_paths(region.perimeters, [&](const ExtrusionPath &path) {
        if (path.role() == erPerimeter || path.role() == erExternalPerimeter)
            wall_beads.push_back({&path, deposition_z(layer)});
    });
    if (wall_beads.empty()) return false;
    // Query the same physical plane that exposed the failed model cell. A
    // surface boundary supplies no support; only these actual wall beads do.
    const auto contact = BridgeAwareCubicContact::footprint(deposited_beads(layer), failure.target.extrusions.front(),
        deposition_z(*object.layers()[failure.target.upper_layer]), 0.,
        BridgeAwareCubicContact::FootprintMode::ContactsOnly);
    if (contact.resource_limited)
        throw SlicingError("Bridge aware cubic wall-connection search is incomplete: " + contact.failure_reason,
            object.model_object()->id().id);
    if (!contact.input_valid || !contact.completed) return false;
    Polygons wall_sections;
    for (const auto &bead : wall_beads) {
        const double width = BridgeAwareCubicContact::section_width_mm(bead.path->width, bead.path->height,
            bead.print_z, contact.contact_z);
        if (!std::isfinite(width) || width <= 0.) continue;
        // Construct each straight wall section on the common plane, with a
        // two-grid inward reserve for polygon rounding. Never replace the
        // actual section by the bead's maximum XY width or a logical contour.
        const double radius = std::floor(scale_(.5 * width)) - 2.;
        if (radius <= 0.) continue;
        float offset_radius = float(radius);
        if (double(offset_radius) > radius)
            offset_radius = std::nextafter(offset_radius, -std::numeric_limits<float>::infinity());
        for (size_t i = 1; i < bead.path->polyline.size(); ++i) {
            const Point &a = bead.path->polyline.points[i - 1], &b = bead.path->polyline.points[i];
            if (a != b) append(wall_sections, offset(Polyline(Points{a, b}), offset_radius,
                ClipperLib::jtSquare, 2., ClipperLib::etOpenButt));
        }
    }
    const ExPolygons walls = union_ex(wall_sections);
    if (walls.empty()) return false;

    const ExPolygons affected = union_ex(cells);
    const auto contains = [](const ExPolygons &polygons, const Point &point) {
        return std::any_of(polygons.begin(), polygons.end(), [&](const ExPolygon &polygon) { return polygon.contains(point); });
    };
    bool repaired = false;
    size_t searched = 0;
    std::map<double, ExPolygons> nearby_by_width;
    visit_free_paths(region.fills, [&](const ExtrusionPath &path) {
        if (path.role() != erInternalInfill || path.polyline.size() < 2) return;
        if (object.print()->canceled()) throw CanceledException();
        auto nearby_entry = nearby_by_width.find(path.width);
        if (nearby_entry == nearby_by_width.end())
            nearby_entry = nearby_by_width.emplace(path.width,
                offset_ex(affected, scale_(.5 * path.width + geometry_tolerance))).first;
        const ExPolygons &nearby = nearby_entry->second;
        const bool start = contains(nearby, path.polyline.first_point()) &&
            !contains(walls, path.polyline.first_point());
        const bool end = contains(nearby, path.polyline.last_point()) &&
            !contains(walls, path.polyline.last_point());
        if (!start && !end) return;
        if (++searched > 4096)
            throw SlicingError("Bridge aware cubic wall-connection search exceeded its path work limit.",
                object.model_object()->id().id);
        Failure request;
        Target &target = request.target;
        target.upper_layer = layer_index;
        target.supporting_region = emitter;
        target.gap = std::min(failure.target.gap, region.region().config().bridge_aware_cubic_max_gap.value);
        target.bottom_z = layer.print_z - path.height;
        target.dependency = true;
        target.paths = {path.polyline};
        target.path_widths = {path.width};
        target.extrusions = {path};
        target.footprint = union_ex(offset(Polylines{path.polyline}, scale_(.5 * path.width)));
        target.path_footprints = {target.footprint};
        target.emitting_regions = {emitter};
        target.free_ended = {true};
        target.producers.resize(1);
        target.solid_cuts = {false};
        BridgeAwareCubicValidation::UnsupportedRun run;
        run.path_index = 0;
        run.kind = start && end ? BridgeAwareCubicValidation::UnsupportedRunKind::Floating :
                                 BridgeAwareCubicValidation::UnsupportedRunKind::OneSidedOverhang;
        run.begin_mm = start ? 0. : unscale<double>(path.polyline.length());
        run.end_mm = run.begin_mm;
        // The original strand is already emitted on this plane. Only the
        // bounded extension spends the search gap; actual predecessor support
        // for its complete replacement is independently certified afterward.
        run.length_mm = 0.;
        request.result.unsupported_runs.push_back(run);
        repaired |= repair_attachments(object, request, attachments, &walls);
    });
    return repaired;
}

bool repair_interfaces(const PrintObject &object, const Failure &failure,
                       std::vector<std::vector<BridgeAwareCubic::LayerClosure>> &closures,
                       const std::optional<Dependency::SourceOwner> &producer = std::nullopt)
{
    if (failure.target.bridge_followup) return false;
    if (failure.target.dependency && !producer) {
        // A dense fallback can still leave a free endpoint between its actual
        // strands. Propose a landing only for one complete, exactly owned
        // source at a time; alternative owners from unrelated paths must not
        // keep an orphaned interface course alive.
        std::set<size_t> paths;
        for (const auto &run : failure.result.unsupported_runs)
            if (run.kind != BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge)
                paths.insert(run.path_index);
        bool repaired = false;
        for (size_t index : paths) {
            if (!failure.target.free_ended.at(index) || failure.target.producers.at(index).size() != 1) continue;
            const size_t emitter = failure.target.emitting_regions.at(index);
            const auto *plan = object.bridge_aware_cubic_plan(emitter);
            const auto &cell = failure.target.producers[index].front();
            if (plan == nullptr || std::none_of(plan->refinements().begin(), plan->refinements().end(),
                    [&](const auto &refinement) { return refinement.id == cell; })) continue;
            Failure selected;
            selected.target.upper_layer = failure.target.upper_layer;
            selected.target.supporting_region = failure.target.supporting_region;
            selected.target.gap = failure.target.gap;
            selected.target.dependency = true;
            selected.target.paths = {failure.target.paths[index]};
            selected.target.path_widths = {failure.target.path_widths[index]};
            selected.target.extrusions = {failure.target.extrusions[index]};
            selected.target.footprint = path_demand(failure.target, index);
            selected.target.path_footprints = {selected.target.footprint};
            if (!failure.verified_paths.empty()) selected.verified_paths = {failure.verified_paths[index]};
            for (auto run : failure.result.unsupported_runs)
                if (run.path_index == index && run.kind != BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge) {
                    run.path_index = 0;
                    selected.result.unsupported_runs.push_back(run);
                }
            repaired |= repair_interfaces(object, selected, closures, Dependency::SourceOwner{emitter, cell});
        }
        return repaired;
    }
    ExPolygons failing_cells;
    Lines terminal_rays;
    std::set<size_t> closed_seams;
    if (failure.target.dependency) {
        const Polyline &path = failure.target.paths.front();
        if (path.size() < 2) return false;
        for (const auto &run : failure.result.unsupported_runs) {
            const double remaining = failure.target.gap - run.length_mm - geometry_tolerance;
            if (!(remaining > 0.)) continue;
            for (bool start : {true, false}) {
                if (run.kind != BridgeAwareCubicValidation::UnsupportedRunKind::Floating && start != (run.begin_mm == 0.)) continue;
                const Point endpoint = start ? path.first_point() : path.last_point();
                const Point neighbour = start ? path.points[1] : path.points[path.size() - 2];
                const Vec2d direction = (endpoint - neighbour).cast<double>();
                if (direction.squaredNorm() == 0.) continue;
                const Vec2d far = endpoint.cast<double>() + direction.normalized() * scale_(remaining);
                constexpr double limit = double(std::numeric_limits<coord_t>::max() / 4);
                if (!far.allFinite() || far.cwiseAbs().maxCoeff() > limit) continue;
                terminal_rays.emplace_back(endpoint, Point(far.cast<coord_t>()));
            }
        }
        Polylines probes;
        for (const Line &ray : terminal_rays) probes.emplace_back(ray.a, ray.b);
        // This is only a bounded interface-search window. It supplies no
        // deposited support; actual ray/interface intersection is required.
        failing_cells = union_ex(offset(probes, scale_(geometry_tolerance)));
    } else {
        for (const auto &cell : failure.result.void_cells)
            if (!cell.closed || cell.exceeds_gap) failing_cells.push_back(cell.geometry);
        for (const auto &run : failure.result.unsupported_runs) {
            if (run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge) continue;
            const size_t index = run.path_index;
            const Polyline &path = failure.target.paths.at(index);
            if (path.size() < 3 || path.first_point() != path.last_point() ||
                failure.target.free_ended.at(index) || !is_perimeter(failure.target.extrusions.at(index).role())) continue;
            const Polyline &verified = failure.verified_paths.empty() ? path : failure.verified_paths.at(index);
            if (run.begin_mm == 0. || run.end_mm >= unscale<double>(verified.length()) - geometry_tolerance)
                closed_seams.insert(index);
        }
    }
    if (failing_cells.empty() && closed_seams.empty()) return false;
    const size_t support_index = failure.target.upper_layer - 1;
    const Layer &top = *object.layers()[support_index];
    const size_t owner = failure.target.supporting_region;
    const size_t storage_owner = producer ? producer->region : owner;
    const LayerRegion &owner_region = *top.get_region(int(owner));
    const Flow top_flow = owner_region.flow(frInfill);
    ExPolygons owner_slice;
    for (const Surface &surface : owner_region.slices.surfaces) owner_slice.push_back(surface.expolygon);
    Polylines interfaces;
    for (size_t neighbour = 0; neighbour < top.regions().size(); ++neighbour) {
        if (neighbour == owner) continue;
        const LayerRegion &region = *top.get_region(int(neighbour));
        if (!closure_compatible(owner_region, region)) continue;
        ExPolygons neighbour_slice;
        for (const Surface &surface : region.slices.surfaces) neighbour_slice.push_back(surface.expolygon);
        Polylines shared = intersection_pl(to_polylines(union_ex(owner_slice)), offset_ex(union_ex(neighbour_slice), 2.));
        shared = intersection_pl(shared, offset_ex(union_ex(failing_cells), scale_(top_flow.width())));
        shared = intersection_pl(shared, offset_ex(sparse_domain(owner_region), scale_(top_flow.width())));
        append(interfaces, std::move(shared));
    }
    // A real model bridge or solid patch can leave its open strand slots
    // connected to a demanded sparse fringe. Propose deposited closure only
    // on the selected shared fill interface; the interface itself is never
    // counted as support. Existing material containment, predecessor courses
    // and final contact/cell verification apply to these courses unchanged.
    const auto append_fill_interfaces = [&](const LayerRegion &region) {
        ExPolygons solid_surfaces;
        for (const Surface &surface : region.fill_surfaces.surfaces)
            if (surface.is_solid()) solid_surfaces.push_back(surface.expolygon);
        if (solid_surfaces.empty()) return;
        // Ordinary surface differencing leaves ClipperSafetyOffset between
        // adjacent sparse and solid classifications. Recognize that known
        // clearance when proposing a real course on the unchanged boundary;
        // it never expands deposited support or the allowed material domain.
        Polylines shared = intersection_pl(to_polylines(union_ex(solid_surfaces)),
            offset_ex(sparse_domain(region), ClipperSafetyOffset + 2.f));
        shared = intersection_pl(shared, offset_ex(union_ex(failing_cells), scale_(top_flow.width())));
        append(interfaces, std::move(shared));
    };
    append_fill_interfaces(owner_region);
    if (producer) {
        // An added partition may begin above an ordinary bridge which has
        // already consumed the lower sparse island. Its exact upper fill
        // interface remains a bounded place to propose a real landing below.
        // Only the failing terminal rays select it; the projected boundary
        // supplies no contact evidence or permission to deposit material.
        const Layer &upper = *object.layers()[failure.target.upper_layer];
        if (producer->region < upper.regions().size())
            append_fill_interfaces(*upper.get_region(int(producer->region)));
    }
    Polylines candidates;
    const ExPolygons centers = offset_ex(closure_domain(top, owner), -scale_(.5 * top_flow.width()));
    for (size_t index : closed_seams) {
        // A fixed model loop must start on actual material. A tangent course
        // beside its seam does not establish that contact, and joining the
        // initial/final air runs would conceal the unsupported start. Deposit
        // one short landing through the unchanged source seam instead.
        const Polyline &path = failure.target.paths[index];
        const Point seam = path.first_point();
        const auto next = std::find_if(path.points.begin() + 1, path.points.end(),
            [&](const Point &point) { return point != seam; });
        if (next == path.points.end() || std::none_of(centers.begin(), centers.end(),
                [&](const ExPolygon &polygon) { return polygon.contains(seam, true); })) continue;
        const Vec2d half = (*next - seam).cast<double>().normalized() * scale_(top_flow.width());
        const Vec2d begin = seam.cast<double>() - half, end = seam.cast<double>() + half;
        constexpr double limit = double(std::numeric_limits<coord_t>::max() / 4);
        if (!begin.allFinite() || !end.allFinite() || begin.cwiseAbs().maxCoeff() > limit || end.cwiseAbs().maxCoeff() > limit) continue;
        append(candidates, intersection_pl(Polylines{Polyline(Point(begin.cast<coord_t>()), Point(end.cast<coord_t>()))}, centers));
    }
    for (const Polyline &line : interfaces) for (size_t i = 1; i < line.size(); ++i) {
        Polyline segment(Points{line.points[i - 1], line.points[i]});
        if (segment.length() <= scale_(geometry_tolerance)) continue;
        if (failure.target.dependency) {
            const Line boundary_line(segment.first_point(), segment.last_point());
            const Vec2d direction = (boundary_line.b - boundary_line.a).cast<double>().normalized();
            for (const Line &ray : terminal_rays) {
                Point crossing;
                if (!boundary_line.intersection(ray, &crossing)) continue;
                const Vec2d half = direction * scale_(top_flow.width());
                const Vec2d begin = crossing.cast<double>() - half, end = crossing.cast<double>() + half;
                constexpr double limit = double(std::numeric_limits<coord_t>::max() / 4);
                if (!begin.allFinite() || !end.allFinite() || begin.cwiseAbs().maxCoeff() > limit || end.cwiseAbs().maxCoeff() > limit) continue;
                append(candidates, intersection_pl(Polylines{Polyline(Point(begin.cast<coord_t>()), Point(end.cast<coord_t>()))}, centers));
            }
            continue;
        }
        segment.extend_start(scale_(top_flow.width()));
        segment.extend_end(scale_(top_flow.width()));
        append(candidates, intersection_pl(Polylines{std::move(segment)}, centers));
    }
    Polylines already;
    const auto *storage_plan = object.bridge_aware_cubic_plan(storage_owner);
    for (const auto &stored : closures[storage_owner]) {
        if (std::abs(stored.z - top.print_z) > EPSILON) continue;
        const bool active = stored.owners.empty() || (storage_plan != nullptr &&
            std::any_of(stored.owners.begin(), stored.owners.end(), [&](const auto &id) {
                return std::any_of(storage_plan->refinements().begin(), storage_plan->refinements().end(),
                    [&](const auto &refinement) { return refinement.id == id; });
            }));
        if (active) append(already, stored.paths);
    }
    candidates = remove_collinear_duplicates(std::move(candidates), already);
    if (candidates.empty()) {
        return false;
    }

    std::vector<BridgeAwareCubic::LayerClosure> accepted;
    for (const Polyline &candidate : candidates) {
    const Polylines course{candidate};
    std::vector<BridgeAwareCubic::LayerClosure> construction;
    bool printable = true;
    for (size_t layer_index = support_index + 1; layer_index-- > 0;) {
        if (object.print()->canceled()) throw CanceledException();
        const Layer &layer = *object.layers()[layer_index];
        size_t emitter = owner;
        const auto fits = [&](size_t region_id) {
            return closure_compatible(*layer.get_region(int(owner)), *layer.get_region(int(region_id))) &&
                closure_paths_owned(layer, region_id, course);
        };
        if (!fits(emitter)) {
            emitter = 0;
            while (emitter < layer.regions().size() && !fits(emitter)) ++emitter;
            if (emitter == layer.regions().size()) {
                printable = false;
                break;
            }
        }
        const Flow flow = layer.get_region(int(emitter))->flow(frInfill);
        BridgeAwareCubic::LayerClosure closure;
        closure.z = layer.print_z;
        closure.paths = course;
        closure.reservation = union_ex(offset(course, scale_(.5 * flow.width() + geometry_tolerance)));
        closure.emitting_region = emitter;
        if (producer) closure.owners = {producer->cell};
        construction.push_back(std::move(closure));
        if (layer_index == 0) break;
        const auto prior = deposited_beads(*object.layers()[layer_index - 1]);
        ExtrusionPath path(erInternalInfill, flow.mm3_per_mm(), float(flow.width()), float(flow.height()));
        path.polyline = candidate;
        const auto contact = BridgeAwareCubicContact::footprint(prior, path, deposition_z(layer), geometry_tolerance,
            BridgeAwareCubicContact::FootprintMode::ContactsOnly);
        const auto checked = BridgeAwareCubicValidation::validate_contacts({}, contact.physical_footprint, course,
            construction.back().reservation, failure.target.gap, geometry_tolerance, {contact});
        if (checked.passed()) break;
    }
    // One unowned extension must not discard another legitimate interface
    // course. Publish only complete predecessor stacks, with each layer's
    // actual emitter and whole-bead containment established independently.
    if (printable) append(accepted, std::move(construction));
    }
    if (accepted.empty()) return false;
    append(closures[storage_owner], std::move(accepted));
    return true;
}

Vec2d common_nozzle_offset(const PrintObject &object)
{
    const auto &offsets = object.print()->config().extruder_offset.values;
    const Vec2d common = offsets.empty() ? Vec2d::Zero() : offsets.front();
    if (!common.allFinite() || std::any_of(offsets.begin(), offsets.end(), [&](const Vec2d &offset) {
            return !offset.allFinite() || offset != common;
        }))
        throw SlicingError("Bridge aware cubic currently requires identical nozzle XY offsets because its support certificate does not resolve per-path physical nozzle changes.", object.model_object()->id().id);
    return common;
}

std::vector<BridgeAwareCubic::SerializedXY> serialization_phases(const PrintObject &geometry,
                                                                const PrintObject *phase_object)
{
    std::vector<BridgeAwareCubic::SerializedXY> phases;
    const Vec2d nozzle_offset = common_nozzle_offset(geometry);
    const Vec2d plate_origin = geometry.print()->get_plate_origin().head<2>();
    const auto append_object = [&](const PrintObject &object) {
        for (const PrintInstance &instance : object.instances()) {
            BridgeAwareCubic::SerializedXY phase;
            phase.shift = unscale(Vec2crd(instance.shift));
            phase.nozzle_offset = nozzle_offset;
            phase.plate_origin = plate_origin;
            phases.push_back(std::move(phase));
        }
    };
    if (phase_object != nullptr) append_object(*phase_object);
    else for (const PrintObject *object : geometry.print()->objects())
        if (object == &geometry || object->get_shared_object() == &geometry) append_object(*object);
    if (phases.empty()) phases.push_back({Vec2d::Zero(), nozzle_offset, plate_origin});
    return phases;
}

void final_proof_preamble(const PrintObject &geometry, const PrintObject &caller,
                         const std::vector<BridgeAwareCubic::SerializedXY> &phases)
{
    // Untargeted emitted paths can precede a required model path. Retain the
    // complete representation/range scan before publishing a successful proof.
    for (const auto &phase : phases)
        for (const Layer *layer : geometry.layers()) {
            if (caller.print()->canceled()) throw CanceledException();
            visit_layer(*layer, [&](const ExtrusionPath &path) {
                try {
                    phase.validate_path(path.polyline);
                } catch (const std::logic_error &error) {
                    throw SlicingError("Bridge aware cubic cannot certify serialized extrusion geometry for object \"" +
                        caller.model_object()->name + "\", layer " + std::to_string(layer->id() + 1) + ": " + error.what(),
                        caller.model_object()->id().id);
                }
            });
        }
    for (size_t region_id = 0; region_id < caller.num_printing_regions(); ++region_id) {
        if (!participates(caller.printing_region(region_id).config()) || caller.bridge_aware_cubic_plan(region_id) != nullptr)
            continue;
        for (const Layer *layer : geometry.layers())
            if (!sparse_domain(*layer->get_region(int(region_id))).empty())
                throw SlicingError("Bridge aware cubic support plan is missing for object \"" + caller.model_object()->name +
                    "\". Reslice the object to rebuild and verify its support geometry.", caller.model_object()->id().id);
    }
}

std::shared_ptr<const BridgeAwareCubic::DepositionStateSnapshot> capture_final_proof(
    const PrintObject &geometry, const PrintObject &caller, const std::vector<BridgeAwareCubic::SerializedXY> &phases)
{
    if (active_profile && active_profile->full_trial_checks) return {};
    ProfileScope profile(ProfilePhase::CaptureFinalProof);
    auto snapshot = BridgeAwareCubic::DepositionStateSnapshot::capture(geometry, caller, phases, [&] {
        if (caller.print()->canceled()) throw CanceledException();
    });
    if (active_profile && snapshot) {
        ++active_profile->final_proof_publications;
        active_profile->final_proof_bytes = snapshot->retained_bytes();
    }
    return snapshot;
}

void remove_stationary_construction(Layer &layer, const ActiveAttachments &active)
{
    const PrintObject &object = *layer.object();
    bool has_plan = false;
    for (size_t emitter = 0; emitter < layer.regions().size(); ++emitter)
        has_plan |= object.bridge_aware_cubic_plan(emitter) != nullptr;
    if (!has_plan) return;

    const auto phases = serialization_phases(object, nullptr);
    const BoundingBox bounds = object.bounding_box();
    const ExPolygon broad_domain(Polygon{Points{bounds.min, Point(bounds.max.x(), bounds.min.y()),
        bounds.max, Point(bounds.min.x(), bounds.max.y())}});
    struct Sources {
        bool prepared = false;
        Polylines original_bulk;
        Polylines partitions;
        Polylines closures;
        ExPolygons solids;
    };
    std::vector<Sources> sources(layer.regions().size());
    const auto prepare_sources = [&](size_t emitter) -> const Sources& {
        Sources &source = sources[emitter];
        if (source.prepared) return source;
        source.prepared = true;
        if (const auto *plan = object.bridge_aware_cubic_plan(emitter)) {
            // The active filler emits these straight original scaffold axes
            // without ordinary Cubic boundary hooks. A larger domain retains
            // their finite source extents before region/material clipping.
            // Keep ambiguous microscopic crossings of bulk and added faces;
            // two-grid provenance proximity alone cannot distinguish them.
            BridgeAwareCubic::Plan original(plan->parameters(), {});
            source.original_bulk = original.lines(layer.print_z, broad_domain);
            for (auto &owned : plan->partition_lines(layer.print_z, broad_domain))
                append(source.partitions, std::move(owned.second));
            source.solids = plan->solid_area(layer.print_z);
        }
        for (size_t owner = 0; owner < layer.regions().size(); ++owner)
            if (const auto *plan = object.bridge_aware_cubic_plan(owner))
                for (const auto &closure : plan->closures(layer.print_z)) {
                    const size_t region = closure.emitting_region == std::numeric_limits<size_t>::max() ?
                        owner : closure.emitting_region;
                    if (region != emitter) continue;
                    const Flow flow = layer.get_region(int(emitter))->flow(frInfill);
                    for (const Polyline &line : closure.paths)
                        source.closures.push_back(resolve_attachments(layer, line, float(flow.width()),
                            float(flow.height()), emitter, active).polyline);
                }
        return source;
    };
    const auto is_stationary = [&](const ExtrusionPath &path) {
        if (path.role() != erInternalInfill || path.is_force_no_extrusion() ||
            !(path.mm3_per_mm > 0.) || path.polyline.size() < 2 || !path.polyline.fitting_result.empty())
            return false;
        for (const auto &phase : phases) {
            const Point first = phase(path.polyline.first_point());
            if (std::all_of(path.polyline.points.begin() + 1, path.polyline.points.end(),
                    [&](const Point &point) { return phase(point) == first; }))
                return true;
        }
        return false;
    };
    const auto is_construction = [&](const ExtrusionPath &path, size_t emitter) {
        const Sources &source = prepare_sources(emitter);
        if (!remove_collinear_duplicates({path.polyline}, source.original_bulk, true).empty())
            return false;
        const auto completely_matched = [&](const Polylines &reference) {
            return BridgeAwareCubic::complete_construction_match(path.polyline,
                remove_collinear_duplicates({path.polyline}, reference, true), 2. * SCALING_FACTOR);
        };
        if (completely_matched(source.partitions) || completely_matched(source.closures)) return true;
        if (!source.solids.empty() && BridgeAwareCubic::complete_construction_match(path.polyline,
                intersection_pl(Polylines{path.polyline}, source.solids), 2. * SCALING_FACTOR))
            return true;
        for (const auto &attachment : active.entries) {
            if (attachment.emitting_region != emitter || attachment.owners.empty()) continue;
            const auto chain = resolve_attachments(layer, attachment.original, path.width, path.height, emitter, active);
            if (!chain.applied_indices.empty() && same_polyline(path.polyline, chain.polyline)) return true;
        }
        return false;
    };
    for (size_t emitter = 0; emitter < layer.regions().size(); ++emitter) {
        const auto remove = [&](auto &&self, ExtrusionEntityCollection &collection) -> void {
            for (auto it = collection.entities.begin(); it != collection.entities.end();) {
                if (object.print()->canceled()) throw CanceledException();
                ExtrusionEntity *entity = *it;
                bool discard = false;
                if (const auto *path = dynamic_cast<const ExtrusionPath*>(entity)) {
                    try {
                        discard = is_stationary(*path) && is_construction(*path, emitter);
                    } catch (const std::logic_error &error) {
                        throw SlicingError("Bridge aware cubic cannot generate serialized construction at layer " +
                            std::to_string(layer.id() + 1) + ": " + error.what(), object.model_object()->id().id);
                    }
                } else if (auto *nested = dynamic_cast<ExtrusionEntityCollection*>(entity)) {
                    const bool was_empty = nested->empty();
                    self(self, *nested);
                    discard = !was_empty && nested->empty();
                }
                // Only standalone paths are removed. A loop or multipath join
                // is retained intact, as is every path with moving output in
                // every actual instance.
                if (discard) {
                    if (const auto *path = dynamic_cast<const ExtrusionPath*>(entity))
                        if (emitter < layer.bridge_aware_emission_builders.size())
                            if (const auto &builder = layer.bridge_aware_emission_builders[emitter])
                                builder->erased(*path, layer.bridge_aware_emission_budget.get(),
                                    [&] { if (object.print()->canceled()) throw CanceledException(); });
                    delete entity;
                    it = collection.entities.erase(it);
                } else ++it;
            }
        };
        remove(remove, layer.get_region(int(emitter))->fills);
    }
}

ExtrusionPath serialized_extrusion(const PrintObject &object, const ExtrusionPath &source,
                                  const BridgeAwareCubic::SerializedXY &phase, const Point &origin)
{
    ExtrusionPath emitted = source;
    try {
        emitted.polyline = phase.emitted_polyline(source.polyline);
    } catch (const std::logic_error &error) {
        throw SlicingError("Bridge aware cubic cannot certify serialized extrusion geometry for object \"" +
            object.model_object()->name + "\": " + error.what(), object.model_object()->id().id);
    }
    emitted.polyline.translate(-origin);
    return emitted;
}

// This is a coarsening-local optimization of immutable model inputs, not a
// support certificate. Each use follows exact actual/frozen target equality;
// lower deposition, contact and complete-cell geometry are always checked anew.
struct PreparedModelTargets {
    struct Phase {
        BridgeAwareCubic::SerializedXY transform;
        std::vector<std::vector<ExPolygons>> path_demands;
        std::vector<ExtrusionPaths> extrusions;
        std::map<size_t, ExPolygons> layer_demands;
    };
    std::vector<Phase> phases;
    size_t vertices = 0, items = 0;
};

struct PreparedModelView {
    const PreparedModelTargets *snapshot;
    // Each actual target maps to the equal frozen target in this snapshot.
    const std::vector<size_t> &frozen_indices;
    BridgeAwareCubic::ModelLayerProofCache *proofs = nullptr;
    // Only a traversal-filtered rejection probe may pass the complete frozen
    // vector itself. Its target indices are already the prepared indices.
    const std::vector<Target> *complete_frozen_probe = nullptr;
};

std::unique_ptr<const BridgeAwareCubic::ModelCollectionInputs> prepare_model_collection_inputs(
    const PrintObject &object, const PreparedModelTargets *prepared)
{
    using Inputs = BridgeAwareCubic::ModelCollectionInputs;
    if (!prepared) return {};
    BridgeAwareCubicValidation::WorkBudget budget;
    ProfileScope profile(ProfilePhase::ModelInputCapture, &budget);
    Inputs::Limits limits;
    auto &usage = limits.other_retained;
    struct Unavailable {};
    const auto charge = [&](size_t amount = 1) {
        if (object.print()->canceled()) throw CanceledException();
        if (amount > budget.remaining_work) throw Unavailable{};
        budget.remaining_work -= amount;
        budget.charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += amount;
    };
    const auto add = [](size_t &value, size_t amount) {
        if (amount > std::numeric_limits<size_t>::max() - value) throw Unavailable{};
        value += amount;
    };
    const auto allocation = [&](size_t capacity, size_t size, bool vertices = false) {
        charge();
        if (capacity > std::numeric_limits<size_t>::max() / size) throw Unavailable{};
        add(usage.bytes, capacity * size);
        if (capacity) add(usage.bytes, 64); // Conservative allocator backing.
        if (vertices) { add(usage.vertices, capacity); add(usage.items, 1); }
        else add(usage.items, capacity);
    };
    const auto polygons = [&](const ExPolygons &polygons) {
        allocation(polygons.capacity(), sizeof(ExPolygon));
        for (const ExPolygon &polygon : polygons) {
            allocation(polygon.contour.points.capacity(), sizeof(Point), true);
            allocation(polygon.holes.capacity(), sizeof(Polygon));
            for (const Polygon &hole : polygon.holes) allocation(hole.points.capacity(), sizeof(Point), true);
        }
    };
    try {
        usage.bytes = sizeof(PreparedModelTargets) + 64;
        usage.items = 1;
        allocation(prepared->phases.capacity(), sizeof(PreparedModelTargets::Phase));
        for (const auto &phase : prepared->phases) {
            allocation(phase.path_demands.capacity(), sizeof(std::vector<ExPolygons>));
            for (const auto &group : phase.path_demands) {
                allocation(group.capacity(), sizeof(ExPolygons));
                for (const auto &demand : group) polygons(demand);
            }
            allocation(phase.extrusions.capacity(), sizeof(ExtrusionPaths));
            for (const auto &group : phase.extrusions) {
                allocation(group.capacity(), sizeof(ExtrusionPath));
                for (const auto &path : group) {
                    allocation(path.polyline.points.capacity(), sizeof(Point), true);
                    allocation(path.polyline.fitting_result.capacity(), sizeof(decltype(path.polyline.fitting_result)::value_type));
                }
            }
            allocation(phase.layer_demands.size(), sizeof(decltype(phase.layer_demands)::value_type) + 64);
            for (const auto &demand : phase.layer_demands) polygons(demand.second);
        }
        if (active_profile) active_profile->model_input_prepared_bytes = usage.bytes;
        auto result = Inputs::capture(object, limits, &budget,
            [&] { if (object.print()->canceled()) throw CanceledException(); });
        if (active_profile) {
            if (result) {
                ++active_profile->model_input_captures;
                const auto own = result->owned_usage();
                active_profile->model_input_bytes = own.bytes;
                active_profile->model_input_combined_bytes = own.bytes + usage.bytes;
                active_profile->model_input_capture_peak_bytes = result->capture_peak_usage().bytes + usage.bytes;
                active_profile->model_input_combined_vertices = own.vertices + usage.vertices;
                active_profile->model_input_combined_items = own.items + usage.items;
            } else ++active_profile->model_input_capture_declines;
        }
        return result;
    } catch (const Unavailable &) {
        if (active_profile) ++active_profile->model_input_capture_declines;
        return {};
    }
}

std::unique_ptr<BridgeAwareCubic::ModelLayerProofCache> prepare_model_proofs(
    const PrintObject &object, const PreparedModelTargets *snapshot, const std::vector<Target> &targets)
{
    if (snapshot == nullptr || snapshot->phases.size() > BridgeAwareCubic::ModelLayerProofCache::max_phases) return {};
    // Count fixed keys before copying them. The optional cache and every
    // in-flight owned source snapshot share these aggregate admission bounds.
    constexpr size_t max_items = 20000, max_vertices = 200000;
    size_t items = snapshot->phases.size();
    if (items > max_items || targets.size() > max_items - items) return {};
    items += targets.size();
    size_t layers = 0, previous_layer = size_t(-1);
    for (const Target &target : targets) {
        if (object.print()->canceled()) throw CanceledException();
        if (target.dependency || (previous_layer != size_t(-1) && target.upper_layer < previous_layer)) return {};
        if (target.upper_layer != previous_layer) { ++layers; previous_layer = target.upper_layer; }
    }
    if (layers > max_items - items) return {};
    try {
        std::vector<BridgeAwareCubic::ModelLayerProofCache::Coverage> coverage;
        std::vector<BridgeAwareCubic::SerializedXY> phases;
        coverage.reserve(layers);
        phases.reserve(snapshot->phases.size());
        for (const auto &phase : snapshot->phases) phases.push_back(phase.transform);
        for (size_t index = 0; index < targets.size(); ++index) {
            if (object.print()->canceled()) throw CanceledException();
            if (coverage.empty() || coverage.back().layer != targets[index].upper_layer)
                coverage.push_back({targets[index].upper_layer, {}});
            coverage.back().targets.push_back(index);
        }
        return std::make_unique<BridgeAwareCubic::ModelLayerProofCache>(snapshot,
            std::move(coverage), std::move(phases), max_vertices, max_items);
    } catch (const std::bad_alloc &) { return {}; }
}

std::unique_ptr<const PreparedModelTargets> prepare_model_targets(const PrintObject &object,
                                                                 const std::vector<Target> &targets)
{
    ProfileScope profile(ProfilePhase::PrepareModelTargets);
    // Aggregate admission covers every retained instance phase, vector/header,
    // path metadata object, polygon/ring and geometry vertex. A cache miss uses
    // the original one-phase-at-a-time preparation; it never omits verification.
    constexpr size_t max_vertices = 200000, max_items = 20000;
    struct CacheFull {};
    std::unique_ptr<PreparedModelTargets> result;
    const auto charge = [](size_t &used, size_t amount, size_t maximum) {
        if (amount > maximum - used) throw CacheFull{};
        used += amount;
    };
    const auto polygons = [&](const ExPolygons &geometry) {
        charge(result->items, geometry.size(), max_items);
        for (const ExPolygon &polygon : geometry) {
            charge(result->items, 1, max_items);
            charge(result->items, polygon.holes.size(), max_items);
            charge(result->vertices, polygon.contour.size(), max_vertices);
            for (const Polygon &hole : polygon.holes) charge(result->vertices, hole.size(), max_vertices);
        }
    };
    try {
        result = std::make_unique<PreparedModelTargets>();
        for (const auto &transform : serialization_phases(object, nullptr)) {
            if (object.print()->canceled()) throw CanceledException();
            charge(result->items, 1, max_items);
            charge(result->items, targets.size(), max_items);
            charge(result->items, targets.size(), max_items);
            result->phases.emplace_back();
            auto &phase = result->phases.back();
            phase.transform = transform;
            phase.path_demands.resize(targets.size());
            phase.extrusions.resize(targets.size());
            const Point origin = transform(Point(0, 0));
            for (size_t index = 0; index < targets.size(); ++index) {
                if (object.print()->canceled()) throw CanceledException();
                const Target &target = targets[index];
                if (target.dependency) throw std::logic_error("Construction targets cannot use frozen model preparation.");
                try {
                    ExPolygons demand = BridgeAwareCubic::serialized_demand(target.footprint, target.paths, transform);
                    if (!target.bridge_followup) {
                        polygons(demand);
                        if (phase.layer_demands.count(target.upper_layer) == 0) charge(result->items, 1, max_items);
                        append(phase.layer_demands[target.upper_layer], std::move(demand));
                    }
                    charge(result->items, target.paths.size(), max_items);
                    phase.path_demands[index].reserve(target.paths.size());
                    for (size_t path = 0; path < target.paths.size(); ++path) {
                        ExPolygons selection = BridgeAwareCubic::serialized_demand(path_demand(target, path), {target.paths[path]}, transform);
                        polygons(selection);
                        phase.path_demands[index].push_back(std::move(selection));
                    }
                } catch (const std::logic_error &error) {
                    throw SlicingError("Bridge aware cubic cannot preserve output demand for object \"" +
                        object.model_object()->name + "\": " + error.what(), object.model_object()->id().id);
                }
                charge(result->items, target.extrusions.size(), max_items);
                phase.extrusions[index].reserve(target.extrusions.size());
                for (const ExtrusionPath &source : target.extrusions) {
                    charge(result->vertices, source.polyline.size(), max_vertices);
                    phase.extrusions[index].push_back(serialized_extrusion(object, source, transform, origin));
                }
            }
        }
    } catch (const CacheFull &) {
        if (active_profile) ++active_profile->model_cache_fallbacks;
        return {};
    } catch (const std::bad_alloc &) {
        if (active_profile) ++active_profile->model_cache_fallbacks;
        return {};
    }
    if (active_profile) {
        active_profile->model_cache_vertices = result->vertices;
        active_profile->model_cache_items = result->items;
    }
    return result;
}

std::vector<Failure> check_targets(const PrintObject &object, const std::vector<Target> &targets,
                                   const PrintObject *phase_object, TargetCheck mode,
                                   const PreparedModelView *prepared_model, const TargetCheckContext *context)
{
    std::vector<Failure> failures;
    BridgeAwareCubicValidation::WorkBudget local_budget;
    auto &budget = context && context->budget ? *context->budget : local_budget;
    const auto *probe = context && mode == TargetCheck::RejectTrial ? context->probe : nullptr;
    auto *span_proofs = context && !probe && mode == TargetCheck::RejectTrial &&
        !(active_profile && active_profile->full_trial_checks) ? context->span_proofs : nullptr;
    auto *contact_epochs = span_proofs ? context->contact_epochs : nullptr;
    const auto profile_span_proofs = [&] {
        if (active_profile && span_proofs) {
            const auto stats = span_proofs->statistics();
            active_profile->span_proof_hits = stats.hits;
            active_profile->span_proof_misses = stats.misses;
            active_profile->span_proof_publications = stats.publications;
            active_profile->span_proof_evictions = stats.evictions;
            active_profile->span_proof_entries = stats.entries;
            active_profile->span_proof_units = stats.units;
            active_profile->span_epoch_hits = stats.epoch_hits;
            active_profile->span_epoch_advances = stats.advances;
            active_profile->span_epoch_audits = stats.epoch_audits;
            active_profile->span_epoch_audit_mismatches = stats.epoch_audit_mismatches;
            active_profile->span_epoch_raw_checks = stats.raw_checks;
        }
    };
    if (context && context->probe_checked) *context->probe_checked = false;
    // The filter controls traversal only. All model groups remain present for
    // per-path selection and the full physical support-complement window.
    if (probe && (probe->target >= targets.size() || targets[probe->target].dependency ||
                  probe->path >= targets[probe->target].paths.size())) return failures;
    ProfilePhase kind = !targets.empty() && targets.front().dependency ? ProfilePhase::CheckConstruction : ProfilePhase::CheckModel;
    if (active_profile && std::any_of(targets.begin(), targets.end(), [&](const auto &target) {
        return target.dependency != (kind == ProfilePhase::CheckConstruction);
    })) kind = ProfilePhase::CheckMixed;
    ProfileScope profile(kind, &budget);
    profile.items(targets.size());
    const auto phases = serialization_phases(object, phase_object);
    const bool complete_frozen_probe = probe && prepared_model && !prepared_model->proofs &&
        prepared_model->complete_frozen_probe == &targets;
    bool use_model_cache = prepared_model != nullptr && prepared_model->snapshot != nullptr && mode == TargetCheck::RejectTrial &&
        (complete_frozen_probe || prepared_model->frozen_indices.size() == targets.size()) && prepared_model->snapshot->phases.size() == phases.size() &&
        std::none_of(targets.begin(), targets.end(), [](const Target &target) { return target.dependency; });
    if (use_model_cache) {
        const auto equal_offset = [](const Vec2d &a, const Vec2d &b) { return a.x() == b.x() && a.y() == b.y(); };
        for (size_t i = 0; i < phases.size(); ++i) {
            const auto &saved = prepared_model->snapshot->phases[i].transform;
            if (!equal_offset(saved.shift, phases[i].shift) || !equal_offset(saved.nozzle_offset, phases[i].nozzle_offset) ||
                !equal_offset(saved.plate_origin, phases[i].plate_origin)) { use_model_cache = false; break; }
        }
    }
    if (complete_frozen_probe && !use_model_cache) return failures;
    if (active_profile && prepared_model) {
        if (use_model_cache) ++active_profile->model_cache_hits;
        else ++active_profile->model_cache_fallbacks;
    }
    size_t output_phase = 0;
    // Quantize whole original paths in the exact writer operation order. An
    // integer translation back near the object origin keeps all shared joints
    // exact while allowing failed cells to select their original local demand.
    // Source-coordinate tolerance is only used by repair discovery; emitted
    // geometry itself is checked without a hypothetical XY displacement.
    for (const auto &phase : phases) {
        ++output_phase;
        if (probe && !probe->includes_phase(phase)) continue;
        const Point origin = phase(Point(0, 0));
        const auto *cached_phase = use_model_cache ? &prepared_model->snapshot->phases[output_phase - 1] : nullptr;
        const auto emitted_path = [&](const ExtrusionPath &source) {
            return serialized_extrusion(object, source, phase, origin);
        };
        std::vector<ExPolygons> preserved_demands;
        std::vector<std::vector<ExPolygons>> preserved_path_demands;
        std::map<size_t, ExPolygons> layer_demands;
        if (cached_phase == nullptr) {
            preserved_demands.reserve(targets.size());
            preserved_path_demands.reserve(targets.size());
            for (const Target &target : targets) {
                try {
                    preserved_demands.push_back(BridgeAwareCubic::serialized_demand(target.footprint, target.paths, phase));
                    std::vector<ExPolygons> paths;
                    paths.reserve(target.paths.size());
                    for (size_t index = 0; index < target.paths.size(); ++index)
                        paths.push_back(BridgeAwareCubic::serialized_demand(path_demand(target, index), {target.paths[index]}, phase));
                    preserved_path_demands.push_back(std::move(paths));
                } catch (const std::logic_error &error) {
                    throw SlicingError("Bridge aware cubic cannot preserve output demand for object \"" +
                        object.model_object()->name + "\": " + error.what(), object.model_object()->id().id);
                }
                if (!target.dependency && !target.bridge_followup)
                    append(layer_demands[target.upper_layer], preserved_demands.back());
            }
        }
        const auto &physical_demands = cached_phase ? cached_phase->layer_demands : layer_demands;
        size_t cached_lower = size_t(-1);
        ExtrusionPaths prior_paths;
        std::vector<BridgeAwareCubicContact::Bead> prior_beads;
        std::map<double, BridgeAwareCubicValidation::PreparedGeometryPtr> prepared;
        std::map<std::pair<double, float>, BridgeAwareCubicContact::PreparedLowerPtr> prepared_lower;
        std::unique_ptr<BridgeAwareCubicContact::QueryCache> contact_queries;
        std::optional<BridgeAwareCubic::SpanProofCache::EpochContext> span_epoch;
        bool epoch_attempted = false;
        // The source pointers and budget outlive the transaction. Publication
        // is possible only after every ordered frozen group on this layer has
        // passed; a partial view or rejected path cannot produce a cache hit.
        BridgeAwareCubic::ModelLayerProofCache::Sources proof_sources;
        std::vector<size_t> proof_targets;
        BridgeAwareCubic::ModelLayerProofCache::Proof layer_proof;
        size_t proof_layer_end = 0;
        const auto cancel = [&] { if (object.print()->canceled()) throw CanceledException(); };
        for (size_t target_index = 0; target_index < targets.size(); ++target_index) {
            if (probe && probe->target != target_index) continue;
            const Target &target = targets[target_index];
            const size_t frozen_index = complete_frozen_probe ? target_index :
                cached_phase ? prepared_model->frozen_indices.at(target_index) : 0;
            if (object.print()->canceled()) throw CanceledException();
            const Layer &lower = *object.layers()[target.upper_layer - 1];
            if (!probe && cached_phase && prepared_model->proofs && target_index >= proof_layer_end) {
                layer_proof = {};
                proof_sources.clear();
                proof_targets.clear();
                proof_layer_end = target_index + 1;
                while (proof_layer_end < targets.size() && targets[proof_layer_end].upper_layer == target.upper_layer)
                    ++proof_layer_end;
                constexpr size_t max_source_items = 20000;
                try {
                    bool admitted = proof_layer_end - target_index <= max_source_items;
                    if (admitted) {
                        proof_targets.reserve(proof_layer_end - target_index);
                        for (size_t index = target_index; index < proof_layer_end; ++index)
                            proof_targets.push_back(prepared_model->frozen_indices.at(index));
                        visit_layer(lower, [&](const ExtrusionPath &path) {
                            cancel();
                            if (!admitted) return;
                            if (proof_sources.size() == max_source_items || budget.remaining_work == 0) {
                                admitted = false;
                                return;
                            }
                            --budget.remaining_work;
                            ++budget.charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)];
                            if (proof_sources.size() == proof_sources.capacity())
                                proof_sources.reserve(std::min(max_source_items,
                                    std::max(size_t(64), 2 * proof_sources.capacity())));
                            proof_sources.push_back(&path);
                        });
                    }
                    if (admitted) {
                        const double lower_z = deposition_z(lower);
                        const double upper_z = deposition_z(*object.layers()[target.upper_layer]);
                        if (prepared_model->proofs->matches(prepared_model->snapshot, target.upper_layer,
                                proof_targets, phase, lower_z, upper_z, proof_sources, &budget, cancel)) {
                            target_index = proof_layer_end - 1;
                            continue;
                        }
                        layer_proof = prepared_model->proofs->begin(prepared_model->snapshot, target.upper_layer,
                            proof_targets, phase, lower_z, upper_z, proof_sources, &budget, cancel);
                    }
                } catch (const std::bad_alloc &) {
                    // Allocation is optional acceleration only. Keep all
                    // already charged work and perform the original proof.
                    layer_proof = {};
                }
            }
            if (cached_lower != target.upper_layer - 1) {
                profile.layer(target.upper_layer);
                // The prepared sections own their snapshot, but clearing all
                // lower-geometry caches before replacing it makes the lifetime
                // and output-phase boundary explicit.
                span_epoch.reset();
                epoch_attempted = false;
                contact_queries.reset();
                prepared_lower.clear();
                prepared.clear();
                prior_paths.clear();
                visit_layer(lower, [&](const ExtrusionPath &path) {
                    auto emitted = emitted_path(path);
                    if (emitted.polyline.size() >= 2) profile.lower(emitted.polyline.size() - 1);
                    if (emitted.polyline.size() >= 2) prior_paths.push_back(std::move(emitted));
                });
                prior_beads.clear();
                for (const ExtrusionPath &path : prior_paths) prior_beads.push_back({&path, deposition_z(lower)});
                cached_lower = target.upper_layer - 1;
            }
            BridgeAwareCubicValidation::Result result;
            Polylines verified_paths(target.paths.size());
            for (size_t index = 0; index < target.paths.size(); ++index) {
                if (probe && probe->path != index) continue;
                if (probe && context->probe_checked) *context->probe_checked = true;
                ExtrusionPath uncached_upper;
                const ExtrusionPath &upper = cached_phase ? cached_phase->extrusions.at(frozen_index).at(index) :
                    (uncached_upper = emitted_path(target.extrusions.at(index)));
                verified_paths[index] = upper.polyline;
                // A source path made entirely of repeated points emits no
                // segment and supplies neither a demand nor support.
                if (upper.polyline.size() < 2) continue;
                profile.upper(upper.polyline.size() - 1);
                const double upper_z = deposition_z(*object.layers()[target.upper_layer]);
                BridgeAwareCubicContact::PreparedLowerPtr lower_contact;
                if (std::isfinite(upper_z) && std::isfinite(upper.height)) {
                    const auto key = std::make_pair(upper_z, upper.height);
                    auto found = prepared_lower.find(key);
                    if (found == prepared_lower.end()) {
                        // Eviction affects reuse only. Every miss performs the
                        // same bounded preparation and charges the shared pass
                        // budget; no geometry is omitted to fit the cache.
                        // Keep one resident snapshot so the helper's combined
                        // prepared-plus-query allocation admission remains the
                        // complete allowance, rather than one per cached key.
                        span_epoch.reset();
                        epoch_attempted = false;
                        contact_queries.reset();
                        prepared_lower.clear();
                        found = prepared_lower.emplace(key, BridgeAwareCubicContact::prepare_lower(
                            prior_beads, upper_z, upper.height, &budget)).first;
                        contact_queries = make_contact_query_cache(found->second);
                    }
                    lower_contact = found->second;
                } else {
                    // Invalid floating values must not enter an ordered key.
                    // Retain the contact helper's normal invalid-input result.
                    span_epoch.reset();
                    epoch_attempted = false;
                    contact_queries.reset();
                    prepared_lower.clear();
                    lower_contact = BridgeAwareCubicContact::prepare_lower(prior_beads, upper_z, upper.height, &budget);
                }
                const ExPolygons &path_selection = cached_phase ? cached_phase->path_demands.at(frozen_index).at(index) :
                    preserved_path_demands[target_index][index];
                const auto span_kind = target.dependency ? BridgeAwareCubic::SpanProofCache::Kind::Construction :
                    target.bridge_followup ? BridgeAwareCubic::SpanProofCache::Kind::BridgeFollowup :
                    BridgeAwareCubic::SpanProofCache::Kind::Ineligible;
                const bool reuse_span = span_proofs && span_kind != BridgeAwareCubic::SpanProofCache::Kind::Ineligible;
                if (reuse_span && contact_epochs && !epoch_attempted) {
                    epoch_attempted = true;
                    span_epoch = contact_epochs->observe(target.upper_layer - 1, phase, upper_z, upper.height,
                        lower_contact, contact_queries.get(), budget);
                }
                const BridgeAwareCubic::SpanProofCache::Query span_query{span_kind, phase, target.gap,
                    target.path_widths[index], path_selection, *lower_contact, upper, upper_z, contact_queries.get(),
                    span_epoch ? &*span_epoch : nullptr, active_profile && active_profile->audit_contact_epochs};
                if (reuse_span) {
                    // The complete lower transition includes newly occupied
                    // absence regions and whole merged axes. Only a proven
                    // unaffected complete query can bypass fresh discovery;
                    // every unavailable/changed identity retains that fallback.
                    const bool matched = span_proofs->matches(span_query, &budget, cancel);
                    profile_span_proofs();
                    if (matched) continue;
                }
                // A continuous contact line proves the path span, but it is
                // not a physical wall bounding the independent model-cell
                // obligation. Only span-only construction/follow-up targets
                // may omit the normal physical-demand and cell checks.
                const bool allow_whole_support = (target.dependency || target.bridge_followup) &&
                    !(active_profile && active_profile->full_trial_checks);
                const auto support_query = !allow_whole_support ?
                    BridgeAwareCubicContact::SupportQuery{BridgeAwareCubicContact::footprint(*lower_contact, upper, upper_z, 0.,
                        BridgeAwareCubicContact::FootprintMode::ContactsOnly, &budget, contact_queries.get()), false} :
                    BridgeAwareCubicContact::footprint_for_support(*lower_contact, upper, upper_z, 0.,
                        &budget, contact_queries.get());
                const auto &contact = support_query.contact;
                if (active_profile && allow_whole_support) {
                    ++active_profile->whole_support_queries;
                    if (support_query.entire_path_supported) {
                        ++active_profile->whole_support_proofs;
                        active_profile->whole_support_segments += contact.segments.size();
                    }
                }
                profile.plane(target.upper_layer, output_phase, contact.contact_z);
                if (contact.resource_limited) {
                    result.completed = false;
                    result.resource_limited = true;
                    result.failure_reason = contact.failure_reason;
                    break;
                }
                BridgeAwareCubicValidation::Result checked;
                bool complete_span_proved = false;
                // Only a positive proof for every run can bypass construction
                // of the selected subset. Failure of this stricter scope says
                // nothing about that subset: retain the original check below
                // with the same already-consumed allowance. Direct roofs,
                // repair, final/export and exhaustive checks never use it.
                if (reuse_span && !support_query.entire_path_supported &&
                    BridgeAwareCubic::SpanProofCache::valid_policy(span_query, &budget, cancel)) {
                    if (active_profile) ++active_profile->complete_span_queries;
                    checked = BridgeAwareCubicValidation::validate_complete_contact_spans(
                        {upper.polyline}, target.gap, 0., {contact}, &budget);
                    complete_span_proved = checked.passed();
                    if (complete_span_proved && active_profile) ++active_profile->complete_span_proofs;
                }
                const bool audit_complete_span = complete_span_proved && active_profile &&
                    active_profile->audit_complete_spans;
                if (support_query.entire_path_supported) {
                    // Contact proved every original segment continuously
                    // supported at the common plane. Keep the validator's
                    // input, gap and certificate guards without constructing
                    // an empty air selection or unrelated physical cells.
                    checked = BridgeAwareCubicValidation::validate_contacts({}, {},
                        {upper.polyline}, {}, target.gap, 0., {contact}, &budget);
                } else if (!complete_span_proved || audit_complete_span) {
                    Polylines unsupported;
                    for (const auto &segment : contact.segments) {
                        const size_t i = segment.segment_index;
                        append(unsupported, diff_pl(Polylines{Polyline(Points{upper.polyline.points[i], upper.polyline.points[i + 1]})},
                                                    segment.centerline_support));
                    }
                    if (!contact.input_valid || !contact.completed || contact.segments.empty()) unsupported = {upper.polyline};
                    const ExPolygons selected = intersection_ex(path_selection,
                        union_ex(offset(unsupported, scale_(.5 * target.path_widths[index]))));
                    if (target.bridge_followup || target.dependency || selected.empty() || !contact.input_valid || !contact.completed) {
                        // Construction and follow-up paths require actual prior
                        // anchors and bounded complete air runs. They do not turn
                        // long slots between bridge strands into cubic cells.
                        checked = BridgeAwareCubicValidation::validate_contacts({}, contact.physical_footprint,
                            {upper.polyline}, selected, target.gap, 0., {contact}, &budget);
                    } else {
                        // Raw occupied material is independent of upper direction
                        // at the same selected contact plane. Reuse only immutable
                        // geometry from this exact lower layer and output phase.
                        auto found = prepared.find(contact.contact_z);
                        if (found == prepared.end()) {
                            // Physical sections are identical for every target at
                            // this plane. Construct them only on a cache miss;
                            // segment contact predicates above remain per-path.
                            const auto physical = BridgeAwareCubicContact::physical_sections(*lower_contact, &budget, contact_queries.get());
                            if (!physical.input_valid || !physical.completed || physical.contact_z != contact.contact_z) {
                                result.input_valid = physical.input_valid && physical.contact_z == contact.contact_z;
                                result.completed = physical.completed;
                                result.resource_limited = physical.resource_limited;
                                result.failure_reason = physical.failure_reason.empty() ?
                                    "Physical support section changed during immutable contact verification." : physical.failure_reason;
                                break;
                            }
                            // Compute the actual support complement in a padded
                            // window containing every demand for this layer. No
                            // rounded model seam may erase a cell or act as a wall.
                            const ExPolygons domain = BridgeAwareCubic::support_topology_domain(
                                physical.physical_footprint, physical_demands.at(target.upper_layer));
                            found = prepared.emplace(contact.contact_z,
                                BridgeAwareCubicValidation::prepare_geometry(domain, physical.physical_footprint, &budget)).first;
                        }
                        if (active_profile && active_profile->full_trial_checks)
                            checked = BridgeAwareCubicValidation::validate_prepared_contacts(*found->second,
                                {upper.polyline}, selected, target.gap, 0., {contact}, &budget);
                        else if (mode == TargetCheck::RejectTrial)
                            checked = BridgeAwareCubicValidation::validate_prepared_contacts_for_trial(*found->second,
                                {upper.polyline}, selected, target.gap, 0., {contact}, &budget);
                        else
                            checked = BridgeAwareCubicValidation::validate_prepared_contact_failures(*found->second,
                                {upper.polyline}, selected, target.gap, 0., {contact}, &budget);
                    }
                }
                if (audit_complete_span) {
                    ++active_profile->complete_span_audits;
                    // An incomplete reference cannot certify an audit hit.
                    // Result::passed includes both input validity and completion.
                    if (!checked.passed()) {
                        ++active_profile->complete_span_audit_mismatches;
                        throw std::runtime_error("Bridge aware cubic complete-span audit failed against selected validation.");
                    }
                }
                if (reuse_span && checked.passed()) {
                    // Only the actual complete span-only check above can
                    // publish a fact. A later failed path in this trial does
                    // not change the exact inputs of this local result.
                    span_proofs->publish(span_query, checked, &budget, cancel);
                    profile_span_proofs();
                }
                // Inspect only this path before moving its vectors. Rescanning
                // the growing aggregate after every successful path would make
                // accepted trials quadratic in their target-group size.
                const bool path_rejects_trial = mode == TargetCheck::RejectTrial && !checked.passed();
                if (path_rejects_trial && !target.dependency && context && context->first_failure &&
                    !context->first_failure->has_value() && context->frozen_indices &&
                    context->frozen_indices->size() == targets.size()) {
                    if (const auto failure_kind = BridgeAwareCubic::ModelFailureReplay::failure_kind(checked))
                        *context->first_failure = BridgeAwareCubic::ModelFailureReplay::Hint{
                            context->frozen_indices->at(target_index), index, phase, *failure_kind};
                }
                result.input_valid &= checked.input_valid;
                result.completed &= checked.completed;
                result.resource_limited |= checked.resource_limited;
                if (!checked.failure_reason.empty() && result.failure_reason.empty()) result.failure_reason = checked.failure_reason;
                result.max_unsupported_mm = std::max(result.max_unsupported_mm, checked.max_unsupported_mm);
                result.max_void_diameter_mm = std::max(result.max_void_diameter_mm, checked.max_void_diameter_mm);
                for (auto &run : checked.unsupported_runs) run.path_index = index;
                append(result.unsupported_runs, std::move(checked.unsupported_runs));
                append(result.void_cells, std::move(checked.void_cells));
                if (!result.input_valid || !result.completed) break;
                // A discarded coarsening candidate needs only one complete
                // counterexample. Successful candidates, repair discovery and
                // final/export verification still check their entire scope.
                // Incomplete results above retain their normal fatal handling.
                if (path_rejects_trial) {
                    if (active_profile) ++active_profile->early_trial_rejections;
                    failures.push_back({target, std::move(result), std::move(verified_paths)});
                    return failures;
                }
            }
            if (!probe) layer_proof.record(frozen_index, result);
            if (!result.passed()) {
                const bool incomplete = !result.input_valid || !result.completed;
                failures.push_back({target, std::move(result), std::move(verified_paths)});
                if (incomplete) return failures;
            }
            if (!probe && target_index + 1 == proof_layer_end) {
                layer_proof.commit();
                layer_proof = {};
            }
        }
        // Repair one output phase at a time, preserving original path identity.
        // All phases are checked again after the resulting geometry changes.
        if (!failures.empty()) break;
    }
    return failures;
}


[[noreturn]] void fail(const PrintObject &object, const Failure &failure)
{
    const Layer &layer = *object.layers()[failure.target.upper_layer];
    if (failure.result.resource_limited) {
        std::ostringstream incomplete;
        incomplete << "Bridge aware cubic support verification is incomplete for object \"" << object.model_object()->name
                   << "\", layer " << layer.id() + 1 << " (Z " << layer.print_z << " mm): "
                   << failure.result.failure_reason;
        throw SlicingError(incomplete.str(), object.model_object()->id().id);
    }
    std::ostringstream message;
    message << "Bridge aware cubic cannot satisfy the requested support gap for object \""
            << object.model_object()->name << "\", layer " << layer.id() + 1
            << " (Z " << std::fixed << std::setprecision(3) << layer.print_z << " mm), region "
            << failure.target.supporting_region + 1 << ": requested " << failure.target.gap
            << " mm; longest unsupported run " << failure.result.max_unsupported_mm << " mm";
    if (!failure.target.dependency && !failure.target.bridge_followup)
        message << ", largest opening " << failure.result.max_void_diameter_mm << " mm";
    message << ". ";
    double longest_one_sided = 0., longest_floating = 0.;
    for (const auto &run : failure.result.unsupported_runs) {
        if (run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::OneSidedOverhang)
            longest_one_sided = std::max(longest_one_sided, run.length_mm);
        else if (run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::Floating)
            longest_floating = std::max(longest_floating, run.length_mm);
    }
    if (longest_one_sided > 0.)
        message << "Longest one-sided run: " << longest_one_sided << " mm. ";
    if (longest_floating > 0.)
        message << "Longest run without either end anchored: " << longest_floating << " mm. ";
    if (!failure.result.failure_reason.empty()) {
        message << failure.result.failure_reason;
        const char last = failure.result.failure_reason.back();
        if (last != '.' && last != '!' && last != '?') message << '.';
        message << ' ';
    }
    message << (failure.target.dependency ? "An added partition lacks valid preceding-layer support. " :
                failure.target.bridge_followup ? "A follow-up bridge layer lacks adequately anchored preceding-layer support. " :
                "The final model extrusion lacks a closed, adequately anchored support cell. ");
    if (failure.target.bridge_followup)
        message << "Check Bridge Layer Density and bridge overlap, or provide additional printable support beneath this feature.";
    else
        message << "Increase Unsupported gap size or provide additional printable support beneath this feature.";
    throw SlicingError(message.str(), object.model_object()->id().id);
}

// prepare_infill rebuilds these surfaces from typed slices and reruns ordinary
// bridge reconstruction before entering this solve. Infill-only regeneration
// does not call this helper, so its accepted angles remain frozen rather than
// becoming the starting point of another rotation on the same surfaces.
void orient_automatic_bridges_impl(PrintObject &object, const std::function<void()> &generate)
{
    using RunKind = BridgeAwareCubicValidation::UnsupportedRunKind;
    using Score = std::tuple<size_t, double, double, double>;
    struct Evaluation { Score score{}; std::vector<Failure> failures; };
    size_t work = 0;
    const auto checkpoint = [&](size_t amount = 1) {
        if (object.print()->canceled()) throw CanceledException();
        constexpr size_t max_work = 1000000;
        if (amount > max_work - work)
            throw SlicingError("Bridge aware cubic automatic orientation exceeded its work limit.", object.model_object()->id().id);
        work += amount;
    };
    const auto normalize = [](double angle) {
        angle = std::fmod(angle, M_PI);
        return angle < 0. ? angle + M_PI : angle;
    };
    const auto charge_polygon = [&](const ExPolygon &polygon) {
        checkpoint(polygon.contour.size() + 1);
        for (const Polygon &hole : polygon.holes) checkpoint(hole.size() + 1);
    };
    const auto compatible_region = [](const LayerRegion &supplier, const LayerRegion &candidate) {
        const auto &config = supplier.region().config();
        const auto &other = candidate.region().config();
        if (!participates(other) || config.wall_filament != other.wall_filament ||
            config.solid_infill_filament != other.solid_infill_filament || config.sparse_infill_filament != other.sparse_infill_filament ||
            config.outer_wall_line_width != other.outer_wall_line_width || config.inner_wall_line_width != other.inner_wall_line_width ||
            config.sparse_infill_line_width != other.sparse_infill_line_width || config.skin_infill_line_width != other.skin_infill_line_width ||
            config.skeleton_infill_line_width != other.skeleton_infill_line_width ||
            config.internal_solid_infill_line_width != other.internal_solid_infill_line_width ||
            config.top_surface_line_width != other.top_surface_line_width || config.bridge_flow != other.bridge_flow)
            return false;
        const auto equal_flow = [](const Flow &a, const Flow &b) {
            return a.width() == b.width() && a.height() == b.height() && a.mm3_per_mm() == b.mm3_per_mm();
        };
        for (FlowRole role : {frExternalPerimeter, frPerimeter, frInfill, frSolidInfill, frTopSolidInfill})
            if (!equal_flow(supplier.flow(role), candidate.flow(role)) ||
                !equal_flow(supplier.bridging_flow(role, false), candidate.bridging_flow(role, false)) ||
                !equal_flow(supplier.bridging_flow(role, true), candidate.bridging_flow(role, true))) return false;
        return true;
    };
    const auto evaluate = [&] {
        checkpoint();
        auto targets = collect_targets(object, false, false);
        targets.erase(std::remove_if(targets.begin(), targets.end(), [&](const Target &target) {
            return !target.bridge_followup || !participates(object.printing_region(target.supporting_region).config());
        }), targets.end());
        Evaluation evaluation;
        evaluation.failures = check_targets(object, targets);
        size_t over_gap = 0;
        double excess = 0., maximum = 0., unanchored = 0.;
        for (const Failure &failure : evaluation.failures) {
            if (!failure.result.input_valid || !failure.result.completed) fail(object, failure);
            const double gap = failure.target.gap;
            if (!std::isfinite(gap) || gap <= 0.)
                throw SlicingError("Bridge aware cubic automatic orientation requires a finite positive gap.", object.model_object()->id().id);
            for (const auto &run : failure.result.unsupported_runs) {
                checkpoint();
                // A lost anchor must not improve the score merely by turning
                // an overlong two-ended interval into a floating interval.
                if (run.exceeds_gap) {
                    ++over_gap;
                    const double ratio = std::max(0., run.length_mm / gap - 1.);
                    excess += std::min(1000., ratio);
                    maximum = std::max(maximum, ratio);
                }
                if (run.kind != RunKind::TwoEndedBridge)
                    unanchored += (run.kind == RunKind::Floating ? 1. : 0.) + std::min(1., run.length_mm / gap);
            }
        }
        evaluation.score = {over_gap, excess, maximum, unanchored};
        return evaluation;
    };

    Evaluation initial = evaluate();
    const bool has_interior_failure = std::any_of(initial.failures.begin(), initial.failures.end(), [](const Failure &failure) {
        return std::any_of(failure.result.unsupported_runs.begin(), failure.result.unsupported_runs.end(), [](const auto &run) {
            return run.kind == RunKind::TwoEndedBridge && run.exceeds_gap;
        });
    });
    if (!has_interior_failure) return;

    struct Slice { size_t layer, region; Surface *surface; };
    std::vector<Slice> slices;
    std::map<std::pair<size_t, size_t>, std::vector<size_t>> by_plane;
    for (size_t layer = 0; layer < object.layers().size(); ++layer)
        for (size_t region = 0; region < object.layers()[layer]->regions().size(); ++region)
            for (Surface &surface : object.layers()[layer]->get_region(int(region))->fill_surfaces.surfaces) {
                checkpoint();
                if (surface.surface_type != stInternalBridge) continue;
                by_plane[{layer, region}].push_back(slices.size());
                slices.push_back({layer, region, &surface});
            }
    std::vector<size_t> group_for_slice(slices.size(), size_t(-1));
    struct Group { std::vector<size_t> slices; bool automatic = true; };
    std::vector<Group> groups;
    const auto overlaps = [&](const ExPolygon &first, const ExPolygon &second) {
        charge_polygon(first);
        charge_polygon(second);
        return !intersection_ex(first, second).empty();
    };
    for (size_t start = 0; start < slices.size(); ++start) {
        if (group_for_slice[start] != size_t(-1)) continue;
        const size_t id = groups.size();
        groups.push_back({{start}, true});
        group_for_slice[start] = id;
        for (size_t head = 0; head < groups[id].slices.size(); ++head) {
            checkpoint();
            const Slice &slice = slices[groups[id].slices[head]];
            const auto &config = object.layers()[slice.layer]->get_region(int(slice.region))->region().config();
            groups[id].automatic &= participates(config) && config.bridge_angle.value == 0. && std::isfinite(slice.surface->bridge_angle) &&
                slice.surface->bridge_angle >= 0.;
            for (bool upward : {false, true}) {
                if ((!upward && (slice.layer == 0 || !slice.surface->bridge_followup)) ||
                    (upward && slice.layer + 1 >= object.layers().size())) continue;
                const size_t layer = upward ? slice.layer + 1 : slice.layer - 1;
                const auto found = by_plane.find({layer, slice.region});
                if (found == by_plane.end()) continue;
                for (size_t next : found->second) {
                    checkpoint();
                    if (group_for_slice[next] != size_t(-1) || (upward && !slices[next].surface->bridge_followup)) continue;
                    if (overlaps(slice.surface->expolygon, slices[next].surface->expolygon)) {
                        group_for_slice[next] = id;
                        groups[id].slices.push_back(next);
                    }
                }
            }
        }
    }

    struct Direction { double weight = 0., x = 0., y = 0.; };
    struct Seed { size_t slice = 0; double ratio = 0.; std::map<int, Direction> directions; };
    std::map<size_t, Seed> seeds;
    for (const Failure &failure : initial.failures) {
        std::map<size_t, std::vector<BridgeAwareCubicValidation::UnsupportedRun>> runs_by_path;
        for (const auto &run : failure.result.unsupported_runs)
            if (run.kind == RunKind::TwoEndedBridge && run.exceeds_gap) runs_by_path[run.path_index].push_back(run);
        for (const auto &entry : runs_by_path) {
            checkpoint();
            const size_t path_index = entry.first;
            const Polyline &verified = failure.verified_paths.empty() ? failure.target.paths[path_index] : failure.verified_paths[path_index];
            Polylines intervals;
            double ratio = 0.;
            for (const auto &run : entry.second) {
                checkpoint(verified.size() + 1);
                ratio = std::max(ratio, run.length_mm / failure.target.gap);
                Polyline interval = verified;
                const double total = unscale<double>(interval.length());
                interval.clip_end(scale_(std::max(0., total - run.end_mm)));
                if (interval.size() >= 2) interval.clip_start(scale_(run.begin_mm));
                if (interval.size() >= 2) intervals.push_back(std::move(interval));
            }
            if (intervals.empty()) continue;
            const size_t lower = failure.target.upper_layer - 1;
            const LayerRegion &supplier = *object.layers()[lower]->get_region(int(failure.target.supporting_region));
            for (size_t region = 0; region < object.layers()[lower]->regions().size(); ++region) {
                // Complete air runs may cross a modifier seam. Their unrelated
                // portions cannot authorize changing another material policy.
                if (!compatible_region(supplier, *object.layers()[lower]->get_region(int(region)))) continue;
                const auto found = by_plane.find({lower, region});
                if (found == by_plane.end()) continue;
                for (size_t slice_index : found->second) {
                    const size_t group = group_for_slice[slice_index];
                    if (!groups[group].automatic) continue;
                    charge_polygon(slices[slice_index].surface->expolygon);
                    const Polylines local = intersection_pl(intervals, ExPolygons{slices[slice_index].surface->expolygon});
                    if (local.empty()) continue;
                    Seed candidate;
                    candidate.slice = slice_index;
                    candidate.ratio = ratio;
                    for (const Polyline &line : local) for (size_t i = 1; i < line.size(); ++i) {
                        checkpoint();
                        const Vec2d direction = (line.points[i] - line.points[i - 1]).cast<double>();
                        const double length = direction.norm();
                        if (length < scale_(std::max(.5, 2. * failure.target.path_widths[path_index]))) continue;
                        const double angle = normalize(std::atan2(direction.y(), direction.x()));
                        const int bin = int(std::floor(angle * 180. / M_PI + .5)) % 180;
                        auto &value = candidate.directions[bin];
                        value.weight += length;
                        value.x += length * std::cos(2. * angle);
                        value.y += length * std::sin(2. * angle);
                    }
                    if (candidate.directions.empty()) continue;
                    auto current = seeds.find(group);
                    // One deterministic worst failing contact plane supplies
                    // proposals; every follow-up plane participates in scoring.
                    if (current == seeds.end() || candidate.ratio > current->second.ratio)
                        seeds[group] = std::move(candidate);
                }
            }
        }
    }
    if (seeds.size() > 32)
        throw SlicingError("Bridge aware cubic automatic orientation exceeded its bridge-group limit.", object.model_object()->id().id);

    Evaluation current = std::move(initial);
    for (const auto &entry : seeds) {
        checkpoint();
        if (std::get<0>(current.score) == 0) break;
        const Group &group = groups[entry.first];
        const Seed &seed = entry.second;
        const auto dominant = std::max_element(seed.directions.begin(), seed.directions.end(), [](const auto &a, const auto &b) {
            return a.second.weight < b.second.weight;
        });
        const double upper_angle = normalize(.5 * std::atan2(dominant->second.y, dominant->second.x));
        double separation = normalize(slices[seed.slice].surface->bridge_angle - upper_angle);
        if (separation > .5 * M_PI) separation -= M_PI;
        const double direction = separation < 0. ? -1. : 1.;
        const double magnitude = std::abs(separation);
        const double margin = M_PI / 180.;
        const double suggested = std::min(.5 * M_PI, std::max(magnitude + margin,
            std::asin(std::min(1., seed.ratio * std::sin(magnitude))) + margin));
        std::vector<double> deltas;
        for (double target : {suggested, M_PI / 12., M_PI / 4., M_PI / 2.}) {
            if (target <= magnitude + 1.e-9) continue;
            const double delta = direction * (target - magnitude);
            if (std::none_of(deltas.begin(), deltas.end(), [&](double existing) { return std::abs(existing - delta) < 1.e-9; }))
                deltas.push_back(delta);
        }
        std::sort(deltas.begin(), deltas.end(), [](double a, double b) { return std::abs(a) < std::abs(b); });
        std::vector<double> original_angles;
        for (size_t index : group.slices) original_angles.push_back(slices[index].surface->bridge_angle);
        const auto apply = [&](double delta) {
            for (size_t i = 0; i < group.slices.size(); ++i)
                slices[group.slices[i]].surface->bridge_angle = delta == 0. ? original_angles[i] : normalize(original_angles[i] + delta);
        };
        double best_delta = 0.;
        Evaluation best = std::move(current);
        for (double delta : deltas) {
            checkpoint();
            apply(delta);
            generate();
            Evaluation trial = evaluate();
            // Losing intermediate contacts can merge several short failures
            // into one longer run. A smaller count alone must not prefer it.
            if (trial.score < best.score && std::get<2>(trial.score) <= std::get<2>(best.score)) {
                best_delta = delta;
                best = std::move(trial);
            }
        }
        apply(best_delta);
        // Only the complete regenerated geometry enters the authoritative
        // solve. The score is a bounded proposal, never a support certificate.
        if (!deltas.empty()) generate();
        current = std::move(best);
    }
}

} // namespace

namespace BridgeAwareCubic {

void orient_automatic_bridges(PrintObject &object, const std::function<void()> &regenerate)
{
    orient_automatic_bridges_impl(object, regenerate);
}

ExPolygons endpoint_attachment_domain(const Layer &layer, size_t emitting_region)
{
    const LayerRegion &emitter = *layer.get_region(int(emitting_region));
    const auto &config = emitter.region().config();
    const auto compatible = [&](const LayerRegion &candidate) {
        const auto &other = candidate.region().config();
        if (!participates(other) || config.wall_filament != other.wall_filament ||
            config.solid_infill_filament != other.solid_infill_filament || config.sparse_infill_filament != other.sparse_infill_filament ||
            config.outer_wall_line_width != other.outer_wall_line_width || config.inner_wall_line_width != other.inner_wall_line_width ||
            config.sparse_infill_line_width != other.sparse_infill_line_width || config.skin_infill_line_width != other.skin_infill_line_width ||
            config.skeleton_infill_line_width != other.skeleton_infill_line_width ||
            config.internal_solid_infill_line_width != other.internal_solid_infill_line_width ||
            config.top_surface_line_width != other.top_surface_line_width || config.bridge_flow != other.bridge_flow)
            return false;
        const auto equal_flow = [](const Flow &a, const Flow &b) {
            return a.width() == b.width() && a.height() == b.height() && a.mm3_per_mm() == b.mm3_per_mm();
        };
        for (FlowRole role : {frExternalPerimeter, frPerimeter, frInfill, frSolidInfill, frTopSolidInfill})
            if (!equal_flow(emitter.flow(role), candidate.flow(role)) ||
                !equal_flow(emitter.bridging_flow(role, false), candidate.bridging_flow(role, false)) ||
                !equal_flow(emitter.bridging_flow(role, true), candidate.bridging_flow(role, true))) return false;
        return true;
    };
    ExPolygons allowed;
    for (const LayerRegion *region : layer.regions())
        if (region == &emitter || compatible(*region))
            for (const Surface &surface : region->slices.surfaces) allowed.push_back(surface.expolygon);
    return intersection_ex(union_ex(allowed), layer.lslices);
}

Polylines matching_partition_segments(const Polylines &candidates, const Polylines &partitions)
{
    return remove_collinear_duplicates(candidates, partitions, true);
}

bool complete_construction_match(const Polyline &path, const Polylines &matched, double tolerance_mm)
{
    if (!std::isfinite(tolerance_mm) || tolerance_mm < 0.)
        throw std::invalid_argument("Bridge aware cubic construction matching requires a finite nonnegative tolerance.");
    double covered = 0.;
    for (const Polyline &piece : matched) covered += piece.length();
    return covered > 0. && covered + scale_(tolerance_mm) >= path.length();
}

ExPolygons partition_provenance_footprint(const Polylines &owned, double width_mm, double tolerance_mm,
                                         const Polylines *source_segments)
{
    if (!std::isfinite(width_mm) || width_mm <= 0. || !std::isfinite(tolerance_mm) || tolerance_mm < 0.)
        throw std::invalid_argument("Bridge aware cubic requires finite partition provenance dimensions.");
    ExPolygons footprint;
    if (source_segments == nullptr) {
        footprint = union_ex(offset(owned, scale_(.5 * width_mm + tolerance_mm)));
    } else {
        constexpr size_t max_vertices = 200000, max_work = 50000000;
        constexpr coord_t coordinate_limit = std::numeric_limits<coord_t>::max() / 4;
        size_t owned_points = 0, source_points = 0;
        const auto checked_size = [&](const Polylines &paths, size_t &count) {
            for (const Polyline &path : paths) {
                if (path.size() > max_vertices - count)
                    throw std::runtime_error("Bridge aware cubic partition provenance exceeds its vertex work limit.");
                count += path.size();
                for (const Point &point : path.points)
                    if (point.x() < -coordinate_limit || point.x() > coordinate_limit ||
                            point.y() < -coordinate_limit || point.y() > coordinate_limit)
                        throw std::invalid_argument("Bridge aware cubic partition provenance exceeds finite coordinate bounds.");
            }
        };
        checked_size(owned, owned_points);
        checked_size(*source_segments, source_points);
        if (owned_points != 0 && source_points > max_work / owned_points)
            throw std::runtime_error("Bridge aware cubic partition provenance exceeds its matching work limit.");
        const long double radius = (.5L * width_mm + tolerance_mm) / SCALING_FACTOR;
        if (!std::isfinite(radius) || radius > coordinate_limit)
            throw std::invalid_argument("Bridge aware cubic partition provenance exceeds finite width bounds.");
        const auto point = [&](long double x, long double y) {
            if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x) > coordinate_limit || std::abs(y) > coordinate_limit)
                throw std::invalid_argument("Bridge aware cubic partition provenance exceeds finite coordinate bounds.");
            return Point(coord_t(std::llround(x)), coord_t(std::llround(y)));
        };
        for (const Polyline &source : *source_segments) for (size_t i = 1; i < source.size(); ++i) {
            const Point &a = source.points[i - 1], &b = source.points[i];
            const long double dx = static_cast<long double>(b.x()) - a.x();
            const long double dy = static_cast<long double>(b.y()) - a.y();
            const long double squared = dx * dx + dy * dy;
            if (squared == 0.) continue;
            const long double length = std::sqrt(squared);
            const long double nx = -dy * radius / length, ny = dx * radius / length;
            const Polylines matches = remove_collinear_duplicates({Polyline(a, b)}, owned, true);
            for (const Polyline &match : matches) for (size_t j = 1; j < match.size(); ++j) {
                const auto projection = [&](const Point &p) {
                    return std::clamp(((static_cast<long double>(p.x()) - a.x()) * dx +
                                       (static_cast<long double>(p.y()) - a.y()) * dy) / squared, 0.L, 1.L);
                };
                long double begin = projection(match.points[j - 1]), end = projection(match.points[j]);
                if (begin > end) std::swap(begin, end);
                if (begin == end) continue;
                if (footprint.size() >= max_vertices / 4)
                    throw std::runtime_error("Bridge aware cubic partition provenance exceeds its output work limit.");
                // Keep both slab caps perpendicular to the ORIGINAL segment.
                // Rounding projected endpoints before buffering would rotate a
                // short interval and amplify that error over half a bead width.
                const long double x0 = a.x() + begin * dx, y0 = a.y() + begin * dy;
                const long double x1 = a.x() + end * dx, y1 = a.y() + end * dy;
                Polygon slab(Points{point(x0 - nx, y0 - ny), point(x1 - nx, y1 - ny),
                                    point(x1 + nx, y1 + ny), point(x0 + nx, y0 + ny)});
                if (slab.area() > 0.) footprint.emplace_back(std::move(slab));
            }
        }
        footprint = union_ex(footprint);
    }
    // Boolean intersections round their vertices to the integer grid. A
    // selected vertex can therefore lie fractionally beyond a source butt
    // cap even when selection was intersected with that exact source bead.
    // Widening the stroke alone cannot cover this longitudinal roundoff.
    return offset_ex(footprint, 2.);
}

ExPolygons serialized_demand(const ExPolygons &source, const Polylines &paths, const SerializedXY &phase)
{
    if (source.empty()) return {};
    constexpr coord_t limit = std::numeric_limits<coord_t>::max() / 4;
    const auto bounded = [&](const Point &point) {
        if (point.x() < -limit || point.x() > limit || point.y() < -limit || point.y() > limit)
            throw std::out_of_range("Bridge aware cubic demand displacement exceeds the safe geometry range.");
    };
    const Point origin = phase(Point(0, 0));
    bounded(origin);
    coord_t margin = 0;
    for (const Polyline &path : paths) for (const Point &point : path.points) {
        const Point serialized = phase(point);
        bounded(point);
        bounded(serialized);
        // Each subtraction is bounded by 3*limit. The integer L1 norm is an
        // exact upper bound on Euclidean displacement, without sqrt rounding.
        const coord_t dx = std::abs(serialized.x() - origin.x() - point.x());
        const coord_t dy = std::abs(serialized.y() - origin.y() - point.y());
        if (dx > limit || dy > limit || dx > limit - dy)
            throw std::out_of_range("Bridge aware cubic demand displacement exceeds the safe geometry range.");
        margin = std::max(margin, dx + dy);
    }
    constexpr coord_t boolean_allowance = 2;
    if (margin > limit - boolean_allowance)
        throw std::out_of_range("Bridge aware cubic demand envelope exceeds the safe geometry range.");
    const coord_t integer_envelope = margin + boolean_allowance;
    float envelope = float(integer_envelope);
    if (envelope >= float(limit))
        throw std::out_of_range("Bridge aware cubic demand envelope exceeds the safe geometry range.");
    // Clipper's offset entry point takes float. Compare in the integer domain
    // so even a large int64 bound cannot silently round down through double.
    if (coord_t(envelope) < integer_envelope)
        envelope = std::nextafter(envelope, std::numeric_limits<float>::infinity());
    if (envelope >= float(limit))
        throw std::out_of_range("Bridge aware cubic demand envelope exceeds the safe geometry range.");
    const coord_t source_limit = limit - coord_t(envelope);
    const auto bounded_source = [&](const Point &point) {
        if (point.x() < -source_limit || point.x() > source_limit || point.y() < -source_limit || point.y() > source_limit)
            throw std::out_of_range("Bridge aware cubic demand envelope exceeds the safe geometry range.");
    };
    for (const ExPolygon &polygon : source) {
        for (const Point &point : polygon.contour.points) bounded_source(point);
        for (const Polygon &hole : polygon.holes) for (const Point &point : hole.points) bounded_source(point);
    }
    return offset_ex(source, envelope);
}

ExPolygons support_topology_domain(const ExPolygons &support, const ExPolygons &all_demands)
{
    BoundingBox bounds;
    for (const auto *polygons : {&support, &all_demands})
        for (const ExPolygon &polygon : *polygons) {
            bounds.merge(polygon.contour.points);
            for (const Polygon &hole : polygon.holes) bounds.merge(hole.points);
        }
    if (!bounds.defined)
        throw std::invalid_argument("Bridge aware cubic support topology has no finite verification bounds.");
    const coord_t pad = scale_(1.);
    constexpr coord_t limit = std::numeric_limits<coord_t>::max() / 4;
    if (bounds.min.x() < -limit + pad || bounds.min.y() < -limit + pad ||
        bounds.max.x() > limit - pad || bounds.max.y() > limit - pad)
        throw std::out_of_range("Bridge aware cubic support topology exceeds the safe geometry range.");
    bounds.offset(pad);
    return {ExPolygon(Polygon{Points{bounds.min, Point(bounds.max.x(), bounds.min.y()), bounds.max,
                                    Point(bounds.min.x(), bounds.max.y())}})};
}

std::optional<ExPolygons> merged_attachment_selection(bool ordinary_full_path,
    const std::vector<LayerAttachment> &active, const std::set<size_t> &applicable_lineage)
{
    if (ordinary_full_path || applicable_lineage.empty()) return std::nullopt;
    ExPolygons selected;
    for (size_t index : applicable_lineage) {
        const LayerAttachment &attachment = active.at(index);
        // Empty is the core payload's explicit whole-path sentinel. Neither
        // a convergent cut-only chain nor storage ordering may narrow it.
        if (!attachment.solid_cut || attachment.demand_selection.empty()) return std::nullopt;
        append(selected, attachment.demand_selection);
    }
    return union_ex(selected);
}

ExPolygons unsupported_interval_demand(const Polyline &verified_path, double begin_mm, double end_mm,
                                       double width_mm, double tolerance_mm, const ExPolygons &original_demand,
                                       const Polyline *source_path)
{
    if (verified_path.size() < 2) return {};
    if (!std::isfinite(begin_mm) || !std::isfinite(end_mm) || !std::isfinite(width_mm) ||
        !std::isfinite(tolerance_mm) || begin_mm < 0. || end_mm < begin_mm || width_mm <= 0. || tolerance_mm < 0.)
        throw std::invalid_argument("Bridge aware cubic requires finite unsupported-interval geometry.");
    const double total = unscale<double>(verified_path.length());
    const double begin = std::max(0., begin_mm - tolerance_mm);
    const double end = std::min(total, end_mm + tolerance_mm);
    if (begin >= end) return {};
    Polyline interval = verified_path;
    interval.clip_end(scale_(std::max(0., total - end)));
    if (interval.size() >= 2) interval.clip_start(scale_(begin));
    if (interval.size() < 2) return {};
    const ExPolygons selected = source_path == nullptr ? original_demand :
        intersection_ex(original_demand, union_ex(offset(Polylines{*source_path}, scale_(.5 * width_mm))));
    return intersection_ex(union_ex(offset(Polylines{std::move(interval)}, scale_(.5 * width_mm + tolerance_mm))), selected);
}

ExPolygons select_unsupported_run_demands(const Polylines &source_paths, const Polylines &verified_paths,
    const std::vector<double> &widths_mm, const std::vector<ExPolygons> &path_demands,
    const std::vector<BridgeAwareCubicValidation::UnsupportedRun> &runs, double tolerance_mm,
    bool complete_selected_runs)
{
    if (source_paths.size() != widths_mm.size() || source_paths.size() != path_demands.size() ||
        (!verified_paths.empty() && verified_paths.size() != source_paths.size()))
        throw std::invalid_argument("Bridge aware cubic repair requires matching per-path demand identities.");
    ExPolygons affected;
    for (const auto &run : runs) {
        if (!run.exceeds_gap && run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge)
            continue;
        const Polyline &source = source_paths.at(run.path_index);
        const Polyline &verified = verified_paths.empty() ? source : verified_paths.at(run.path_index);
        const double width = widths_mm.at(run.path_index);
        ExPolygons selected = unsupported_interval_demand(verified, run.begin_mm, run.end_mm,
            width, tolerance_mm, path_demands.at(run.path_index), &source);
        if (selected.empty()) continue;
        if (complete_selected_runs) {
            const ExPolygons bead = union_ex(offset(Polylines{source}, scale_(.5 * width)));
            selected = unsupported_interval_demand(verified, run.begin_mm, run.end_mm,
                width, tolerance_mm, bead, &source);
        }
        append(affected, std::move(selected));
    }
    return affected;
}

ExPolygons solid_cut_endpoint_demand(const Polyline &path, const ExPolygons &solid_area,
                                     double width_mm, double tolerance_mm)
{
    ExPolygons selected;
    if (path.size() < 2 || solid_area.empty()) return selected;
    if (!std::isfinite(width_mm) || width_mm <= 0. || !std::isfinite(tolerance_mm) || tolerance_mm < 0.)
        throw std::invalid_argument("Bridge aware cubic requires finite solid-endpoint geometry.");
    const ExPolygons near = offset_ex(solid_area, scale_(tolerance_mm));
    const coord_t radius = scale_(.5 * width_mm + tolerance_mm);
    for (const Point &endpoint : {path.first_point(), path.last_point()})
        if (std::any_of(near.begin(), near.end(), [&](const ExPolygon &polygon) { return polygon.contains(endpoint, true); }))
            selected.emplace_back(Polygon{Points{
                endpoint + Point(-radius, -radius), endpoint + Point(radius, -radius),
                endpoint + Point(radius, radius), endpoint + Point(-radius, radius)}});
    return union_ex(selected);
}

ExPolygons solid_cut_provenance_footprint(const Polyline &path, const ExPolygons &selection, double width_mm)
{
    constexpr size_t max_vertices = 200000;
    constexpr coord_t coordinate_limit = std::numeric_limits<coord_t>::max() / 4;
    if (!std::isfinite(width_mm) || width_mm <= 0. ||
        .5L * width_mm / SCALING_FACTOR > coordinate_limit)
        throw std::invalid_argument("Bridge aware cubic requires finite solid-cut provenance width.");
    size_t vertices = 0;
    const auto check_points = [&](const Points &points) {
        if (points.size() > max_vertices - vertices)
            throw std::runtime_error("Bridge aware cubic solid-cut provenance exceeds its vertex work limit.");
        vertices += points.size();
        for (const Point &point : points)
            if (point.x() < -coordinate_limit || point.x() > coordinate_limit ||
                point.y() < -coordinate_limit || point.y() > coordinate_limit)
                throw std::invalid_argument("Bridge aware cubic solid-cut provenance exceeds finite coordinate bounds.");
    };
    check_points(path.points);
    for (const ExPolygon &polygon : selection) {
        check_points(polygon.contour.points);
        for (const Polygon &hole : polygon.holes) check_points(hole.points);
        if (!polygon.is_valid())
            throw std::invalid_argument("Bridge aware cubic requires valid solid-cut provenance selections.");
    }
    if (path.size() < 2 || selection.empty()) return {};
    const ExPolygons bead = union_ex(offset(Polylines{path}, scale_(.5 * width_mm)));
    // Bound each original constraint before intersecting. Offsetting an
    // already rounded intersection spends that vertex roundoff a second time
    // and can extend beyond the source or selector's two-unit envelope.
    return intersection_ex(offset_ex(bead, 2.), offset_ex(selection, 2.));
}

Polylines layer_closure_lines(const Layer &layer)
{
    Polylines result;
    const ActiveAttachments active = active_attachments(layer);
    for (size_t owner = 0; owner < layer.regions().size(); ++owner) {
        const Plan *plan = layer.object()->bridge_aware_cubic_plan(owner);
        if (plan == nullptr) continue;
        for (const LayerClosure &closure : plan->closures(layer.print_z)) {
            const size_t emitter = closure.emitting_region == std::numeric_limits<size_t>::max() ? owner : closure.emitting_region;
            if (emitter >= layer.regions().size())
                throw SlicingError("Bridge aware cubic closure has no printable owning region.", layer.object()->model_object()->id().id);
            if (!closure_paths_owned(layer, emitter, closure.paths))
                throw SlicingError("Bridge aware cubic closure has no compatible participating material owner at Z " +
                    std::to_string(layer.print_z) + " mm.", layer.object()->model_object()->id().id);
            const Flow flow = layer.get_region(int(emitter))->flow(frInfill);
            for (const Polyline &line : closure.paths)
                result.push_back(resolve_attachments(layer, line, float(flow.width()), float(flow.height()), emitter, active).polyline);
        }
    }
    return result;
}

void begin_layer_emissions(Layer &layer, bool capture, const std::vector<size_t> *retained_bytes)
{
    // Drop every old address handle before Layer::make_fills destroys paths.
    layer.bridge_aware_emission_builders.clear();
    layer.bridge_aware_emission_snapshots.clear();
    layer.bridge_aware_emission_budget.reset();
    const PrintObject &object = *layer.object();
    if (!capture || !object.has_bridge_aware_cubic()) return;
    const size_t quota = emission_quota(layer);
    if (quota < 2048 || layer.regions().size() > 20000) {
        if (active_profile) active_profile->emission_capture("init_quota");
        return;
    }
    if (retained_bytes && (retained_bytes->size() != object.num_printing_regions() ||
            retained_bytes->size() < layer.regions().size())) {
        if (active_profile) active_profile->emission_capture("retained_namespace");
        return;
    }
    size_t available = quota;
    if (retained_bytes)
        for (size_t emitter = 0; emitter < layer.regions().size(); ++emitter)
            if (object.bridge_aware_cubic_plan(emitter) != nullptr) {
                const size_t held = retained_bytes->at(emitter);
                if (held > quota) return;
                available = std::min(available, quota - held);
            }
    if (object.print()->canceled()) throw CanceledException();
    try {
        auto work = std::make_shared<BridgeAwareCubicValidation::WorkBudget>();
        std::vector<EmissionPlanBinding> bindings;
        if (!emission_bindings(layer, bindings, available / 4, *work)) {
            if (active_profile) active_profile->emission_capture("init_bindings");
            return;
        }
        if (bindings.empty()) return;
        size_t remaining = available / 2, active_count = 0;
        for (size_t owner = 0; owner < layer.regions().size(); ++owner) {
            const Plan *plan = object.bridge_aware_cubic_plan(owner);
            if (plan == nullptr) continue;
            // Cross-region closure storage requires the reference collector's
            // separate visit. No emitter on a closure layer is eligible yet.
            if (plan->has_closures(layer.print_z)) {
                if (active_profile) active_profile->emission_capture("init_closures");
                return;
            }
            if (!plan->visit_attachments(layer.print_z, [&](const LayerAttachment &attachment) {
                    if (!emission_attachment_bytes(attachment, remaining, object, *work)) return false;
                    ++active_count;
                    return active_count <= 20000;
                })) {
                    if (active_profile) active_profile->emission_capture("init_attachments");
                    return;
                }
        }
        std::vector<EmissionTransition> active;
        active.reserve(active_count);
        if (active.capacity() > available / (4 * sizeof(EmissionTransition))) return;
        for (size_t owner = 0; owner < layer.regions().size(); ++owner)
            if (const Plan *plan = object.bridge_aware_cubic_plan(owner))
                if (!plan->visit_attachments(layer.print_z, [&](const LayerAttachment &attachment) {
                        if (!emission_charge(object, *work)) return false;
                        active.push_back({owner, attachment});
                        return true;
                    })) return;
        const EmissionReference reference{object.bounding_box(), geometry_tolerance};
        layer.bridge_aware_emission_builders.resize(layer.regions().size());
        layer.bridge_aware_emission_snapshots.resize(layer.regions().size());
        layer.bridge_aware_emission_budget = std::move(work);
        for (size_t emitter = 0; emitter < layer.regions().size(); ++emitter)
            if (object.bridge_aware_cubic_plan(emitter) != nullptr)
                layer.bridge_aware_emission_builders[emitter] = std::make_shared<EmissionLedgerBuilder>(
                    emitter, layer.print_z, reference, bindings, active,
                    EmissionLedgerLimits{quota, retained_bytes ? retained_bytes->at(emitter) : 0},
                    layer.bridge_aware_emission_budget.get(),
                    [&] { if (object.print()->canceled()) throw CanceledException(); });
    } catch (const std::bad_alloc &) {
        if (active_profile) active_profile->emission_capture("init_allocation");
        layer.bridge_aware_emission_builders.clear();
        layer.bridge_aware_emission_snapshots.clear();
        layer.bridge_aware_emission_budget.reset();
    }
}

void apply_layer_paths(Layer &layer)
{
    const PrintObject &object = *layer.object();
    // Replay only immutable original-path identities. This hook also runs in
    // the parallel final fill pass and never reads another layer's mutable fills.
    const ActiveAttachments active = active_attachments(layer);
    for (const auto &attachment : active.entries)
        if (attachment.emitting_region >= layer.regions().size())
            throw SlicingError("Bridge aware cubic attachment has no printable owning region.", object.model_object()->id().id);
    const auto replay_attachments = [&] {
        for (size_t emitter = 0; emitter < layer.regions().size(); ++emitter) {
            LayerRegion &region = *layer.get_region(int(emitter));
            const auto replay = [&](ExtrusionPath &path) {
                auto resolved = resolve_attachments(layer, path.polyline, path.width, path.height, emitter, active);
                path.polyline = std::move(resolved.polyline);
                if (path.role() == erInternalInfill && !resolved.applied_indices.empty() &&
                    emitter < layer.bridge_aware_emission_builders.size()) {
                    const auto &builder = layer.bridge_aware_emission_builders[emitter];
                    if (!builder) return;
                    try {
                        size_t remaining = builder->available_scratch_bytes();
                        for (size_t id : resolved.applied_indices)
                            if (!emission_attachment_bytes(active.entries.at(id), remaining, object,
                                    *layer.bridge_aware_emission_budget)) {
                                builder->decline();
                                return;
                            }
                        std::vector<EmissionTransition> transitions;
                        transitions.reserve(resolved.applied_indices.size());
                        // The pre-copy walk reserved two outer slots per
                        // transition along with all nested payload capacity.
                        if (transitions.capacity() > 2 * resolved.applied_indices.size()) {
                            builder->decline();
                            return;
                        }
                        for (size_t id : resolved.applied_indices)
                            transitions.push_back({active.storage_regions.at(id), active.entries.at(id)});
                        builder->applied(path, transitions, layer.bridge_aware_emission_budget.get(),
                            [&] { if (object.print()->canceled()) throw CanceledException(); });
                    } catch (const std::bad_alloc &) { builder->decline(); }
                }
            };
            visit_free_paths(region.perimeters, replay);
            visit_free_paths(region.fills, replay);
        }
    };
    replay_attachments();

    Polylines deposited;
    visit_layer(layer, [&](const ExtrusionPath &path) { deposited.push_back(path.polyline); });
    for (size_t owner = 0; owner < layer.regions().size(); ++owner) {
        const Plan *plan = object.bridge_aware_cubic_plan(owner);
        if (plan == nullptr) continue;
        for (const LayerClosure &closure : plan->closures(layer.print_z)) {
            const size_t emitter = closure.emitting_region == std::numeric_limits<size_t>::max() ? owner : closure.emitting_region;
            if (emitter >= layer.regions().size())
                throw SlicingError("Bridge aware cubic closure has no printable owning region.", object.model_object()->id().id);
            if (!closure_paths_owned(layer, emitter, closure.paths))
                throw SlicingError("Bridge aware cubic closure has no compatible participating material owner at Z " +
                    std::to_string(layer.print_z) + " mm.", object.model_object()->id().id);
            LayerRegion &region = *layer.get_region(int(emitter));
            const Flow flow = region.flow(frInfill);
            Polylines paths = remove_collinear_duplicates(closure.paths, deposited);
            ExtrusionPaths extrusions;
            extrusion_paths_append(extrusions, paths, erInternalInfill, flow.mm3_per_mm(), float(flow.width()), float(flow.height()));
            Polygons footprint;
            for (const auto &path : extrusions) path.polygons_covered_by_width(footprint, 0.f);
            if (!diff_ex(union_ex(footprint), closure_domain(layer, emitter)).empty())
                throw SlicingError("Bridge aware cubic closure would leave permitted printable material at Z " +
                    std::to_string(layer.print_z) + " mm.", object.model_object()->id().id);
            if (!extrusions.empty()) {
                auto *collection = new ExtrusionEntityCollection;
                collection->no_sort = true;
                collection->append(std::move(extrusions));
                region.fills.entities.push_back(collection);
                append(deposited, std::move(paths));
            }
        }
    }
    // Saved attachments may also belong to a newly emitted closure course.
    // The exact identity check makes this second replay idempotent for paths
    // already generated by ordinary fill groups.
    replay_attachments();
    // A microscopic clipped partition can emit E while every output XY point
    // coincides. Remove that complete construction entity from the actual
    // generated material for all instances, before support collection. This
    // runs on every trial and final fill; the remaining model, cell and
    // dependency geometry must still pass the ordinary final certificate.
    remove_stationary_construction(layer, active);
    // Capture only the final surviving complete emitter after both replay
    // passes and stationary removal. Raw survival owns values, never live
    // entity addresses. Classification/reference work is deferred until a
    // construction collector actually requests it after the trial model gate.
    if (layer.bridge_aware_emission_budget) {
        auto &budget = *layer.bridge_aware_emission_budget;
        size_t available = emission_quota(layer);
        bool staged = false;
        for (const auto &builder : layer.bridge_aware_emission_builders)
            if (builder && builder->staged_bytes() != 0) {
                staged = true;
                available = std::min(available, builder->available_scratch_bytes());
            }
        if (!staged) available = 0;
        try {
            std::vector<EmissionPlanBinding> bindings;
            if (emission_bindings(layer, bindings, available / 4, budget))
                for (size_t emitter = 0; emitter < layer.bridge_aware_emission_builders.size(); ++emitter) {
                    const auto &builder = layer.bridge_aware_emission_builders[emitter];
                    if (!builder) continue;
                    std::vector<EmissionPathView> views;
                    // Old and new view buffers may coexist during growth;
                    // two thirds plus the binding quarter stays in quota.
                    if (emission_views(layer, emitter, views, builder->available_scratch_bytes() / 3, budget)) {
                        layer.bridge_aware_emission_snapshots[emitter] = builder->seal_snapshot(views, bindings, &budget,
                            [&] { if (object.print()->canceled()) throw CanceledException(); });
                        if (active_profile) active_profile->emission_capture(
                            layer.bridge_aware_emission_snapshots[emitter] ? "captured" : builder->decline_reason());
                    } else if (active_profile) active_profile->emission_capture("final_views");
                }
        } catch (const std::bad_alloc &) {
            for (auto &snapshot : layer.bridge_aware_emission_snapshots) snapshot.reset();
        }
    }
    layer.bridge_aware_emission_builders.clear();
    layer.bridge_aware_emission_budget.reset();
}

static std::string construction_collection_difference(const PrintObject &object,
    const std::vector<Target> &actual, const std::vector<Target> &reference)
{
    const auto mismatch = [&](size_t target, const std::string &field) {
        return "Construction target " + std::to_string(target) + " differs in " + field + ".";
    };
    const auto same_polyline_state = [](const Polyline &a, const Polyline &b) {
        if (a.points != b.points || a.fitting_result.size() != b.fitting_result.size()) return false;
        for (size_t i = 0; i < a.fitting_result.size(); ++i) {
            const auto &left = a.fitting_result[i], &right = b.fitting_result[i];
            if (left.start_point_index != right.start_point_index || left.end_point_index != right.end_point_index ||
                left.path_type != right.path_type) return false;
            const auto &x = left.arc_data, &y = right.arc_data;
            if (x.center != y.center || x.radius != y.radius || x.is_arc != y.is_arc || x.length != y.length ||
                x.angle_radians != y.angle_radians || x.polar_start_theta != y.polar_start_theta ||
                x.polar_end_theta != y.polar_end_theta || x.start_point != y.start_point ||
                x.end_point != y.end_point || x.direction != y.direction) return false;
        }
        return true;
    };
    if (actual.size() != reference.size()) return mismatch(0, "target count");
    for (size_t i = 0; i < actual.size(); ++i) {
        if (object.print()->canceled()) throw CanceledException();
        const Target &a = actual[i], &b = reference[i];
        if (a.upper_layer != b.upper_layer) return mismatch(i, "layer");
        if (a.supporting_region != b.supporting_region) return mismatch(i, "supporting region");
        if (a.gap != b.gap) return mismatch(i, "gap policy");
        if (a.bottom_z != b.bottom_z) return mismatch(i, "contact elevation");
        if (a.dependency != b.dependency || a.bridge_followup != b.bridge_followup) return mismatch(i, "target kind");
        if (a.paths.size() != b.paths.size()) return mismatch(i, "path count");
        for (size_t j = 0; j < a.paths.size(); ++j)
            if (!same_polyline_state(a.paths[j], b.paths[j])) return mismatch(i, "path " + std::to_string(j));
        if (a.path_widths != b.path_widths) return mismatch(i, "selection widths");
        if (a.footprint != b.footprint) return mismatch(i, "complete footprint");
        if (a.path_footprints != b.path_footprints) return mismatch(i, "individual footprints");
        if (a.emitting_regions != b.emitting_regions) return mismatch(i, "emitting regions");
        if (a.free_ended != b.free_ended) return mismatch(i, "free endpoints");
        if (a.producers != b.producers) return mismatch(i, "ordered producers");
        if (a.solid_cuts != b.solid_cuts) return mismatch(i, "solid-cut selection");
        if (a.extrusions.size() != b.extrusions.size()) return mismatch(i, "extrusion count");
        for (size_t j = 0; j < a.extrusions.size(); ++j) {
            const ExtrusionPath &x = a.extrusions[j], &y = b.extrusions[j];
            if (!same_polyline_state(x.polyline, y.polyline) || x.role() != y.role() || x.width != y.width ||
                x.height != y.height || x.mm3_per_mm != y.mm3_per_mm || x.can_reverse() != y.can_reverse() ||
                x.is_force_no_extrusion() != y.is_force_no_extrusion() || x.overhang_degree != y.overhang_degree ||
                x.curve_degree != y.curve_degree || x.smooth_speed != y.smooth_speed ||
                x.get_customize_flag() != y.get_customize_flag() || x.get_cooling_node() != y.get_cooling_node())
                return mismatch(i, "extrusion " + std::to_string(j));
        }
    }
    return {};
}

EmissionCollectionAudit audit_construction_collection(const PrintObject &object)
{
    if (!object.is_step_done(posInfill) || object.layers().empty())
        throw std::logic_error("Construction collection audit requires completed infill.");
    if (object.print()->canceled()) throw CanceledException();
    EmissionCollectionAudit audit;
    EmissionReuseContext reuse(object);
    const auto actual = collect_targets(object, true, false, nullptr, ConstructionCollection::Emitted, &audit, &reuse);
    const auto reference = collect_targets(object, true, false, nullptr, ConstructionCollection::Reference);
    audit.mismatch = construction_collection_difference(object, actual, reference);
    audit.equal = audit.mismatch.empty();
    return audit;
}

} // namespace BridgeAwareCubic

bool PrintObject::has_bridge_aware_cubic() const
{
    for (size_t region_id = 0; region_id < num_printing_regions(); ++region_id)
        if (participates(printing_region(region_id).config())) return true;
    return false;
}

const BridgeAwareCubic::Plan *PrintObject::bridge_aware_cubic_plan(size_t region_id) const
{
    if (m_shared_object != nullptr) return m_shared_object->bridge_aware_cubic_plan(region_id);
    return region_id < m_bridge_aware_cubic_plans.size() ? m_bridge_aware_cubic_plans[region_id].get() : nullptr;
}

const ExPolygons &PrintObject::bridge_aware_cubic_reservations(size_t layer_id, size_t region_id) const
{
    if (m_shared_object != nullptr) return m_shared_object->bridge_aware_cubic_reservations(layer_id, region_id);
    static const ExPolygons empty;
    return layer_id < m_bridge_aware_cubic_reservations.size() &&
        region_id < m_bridge_aware_cubic_reservations[layer_id].size() ?
        m_bridge_aware_cubic_reservations[layer_id][region_id] : empty;
}

void PrintObject::prepare_bridge_aware_cubic(bool has_restored_sparse)
{
    clear_bridge_aware_cubic_certificate();
    m_bridge_aware_cubic_plans.clear();
    m_bridge_aware_cubic_plans.resize(num_printing_regions());
    m_bridge_aware_cubic_reservations.clear();
    if (!has_bridge_aware_cubic() || m_layers.empty()) return;
    ProfileSession profile_session("prepare", model_object()->id().id);
    common_nozzle_offset(*this);

    if (print()->config().spiral_mode.value)
        throw SlicingError("Bridge aware cubic requires Spiral vase disabled because its support certificate uses planar layers.", model_object()->id().id);

    if (config().seam_gap.value != 0.)
        throw SlicingError("Bridge aware cubic currently requires Seam gap = 0 because support validation precedes seam placement.", model_object()->id().id);
    for (const PrintRegion &region : all_regions()) {
        const auto &cfg = region.config();
        const bool scarf = cfg.override_filament_scarf_seam_setting.value ?
            cfg.seam_slope_type.value != SeamScarfType::None :
            std::any_of(print()->config().filament_scarf_seam_type.values.begin(),
                        print()->config().filament_scarf_seam_type.values.end(),
                        [](int value) { return value != int(SeamScarfType::None); });
        if (scarf)
            throw SlicingError("Bridge aware cubic currently requires Scarf seam type = None because support validation precedes scarf generation.", model_object()->id().id);
    }
    if (config().apply_scarf_seam_on_circles.value)
        for (const Layer *layer : m_layers)
            for (const LayerRegion *region : layer->regions())
                if (has_circle_scarf(region->perimeters))
                    throw SlicingError("Bridge aware cubic currently requires Apply scarf seam on circles disabled for compensated circular walls.", model_object()->id().id);

    const auto generate = [this] {
        ProfileScope profile(ProfilePhase::GenerateFull);
        for (Layer *layer : m_layers) {
            throw_if_canceled();
            profile.layer(layer->id());
            layer->make_fills(m_adaptive_fill_octrees.first.get(), m_adaptive_fill_octrees.second.get(), m_lightning_generator.get());
        }
    };
    // Freeze the actual bridge angles/anchor geometry selected by the ordinary
    // bridge passes. The hierarchy retains the bulk scaffold and only adds
    // partitions; these exact upper paths are the planning targets.
    generate();
    BridgeAwareCubic::orient_automatic_bridges(*this, generate);
    std::vector<Target> roofs = collect_targets(*this, false);
    std::vector<std::vector<BridgeAwareCubic::Demand>> demands(num_printing_regions());
    std::vector<std::vector<BridgeAwareCubic::LayerClosure>> closures(num_printing_regions());
    std::vector<std::vector<BridgeAwareCubic::LayerAttachment>> attachments(num_printing_regions());
    std::vector<Dependency::DemandKey> direct_roots;
    struct ClosureRoot { Dependency::DemandKey demand; double z; Polyline producer; };
    std::vector<ClosureRoot> closure_roots;
    std::vector<Dependency::Obligation> obligations;
    std::vector<std::vector<size_t>> demand_indices(num_printing_regions());
    for (const Target &target : roofs) {
        if (target.bridge_followup) continue;
        const Layer &lower = *m_layers[target.upper_layer - 1];
        ExPolygons refinable = intersection_ex(target.footprint,
            sparse_domain(*lower.get_region(int(target.supporting_region))));
        if (!refinable.empty()) {
            direct_roots.push_back({target.supporting_region, demands[target.supporting_region].size()});
            demands[target.supporting_region].push_back({std::move(refinable), lower.print_z,
                                                        target.gap - geometry_tolerance});
        }
    }

    const BoundingBox bounds = bounding_box();
    std::vector<BridgeAwareCubic::Parameters> parameters(num_printing_regions());
    for (size_t region_id = 0; region_id < num_printing_regions(); ++region_id) {
        const auto &config = printing_region(region_id).config();
        if (!participates(config)) continue;
        if (config.fill_multiline.value != 1 || config.conformal_infill.value)
            throw SlicingError("Bridge aware cubic requires Fill multiline = 1 and Conformal infill disabled.", model_object()->id().id);
        bool has_sparse = false;
        for (const Layer *layer : m_layers)
            if (!sparse_domain(*layer->get_region(int(region_id))).empty()) {
                has_sparse = true;
                break;
            }
        if (!has_sparse) continue;
        const double spacing = printing_region(region_id).flow(*this, frInfill, this->config().layer_height, false).spacing();
        auto &params = parameters[region_id];
        params.bounds = BoundingBoxf3(Vec3d(unscale<double>(bounds.min.x()), unscale<double>(bounds.min.y()), m_layers.front()->bottom_z()),
                                     Vec3d(unscale<double>(bounds.max.x()), unscale<double>(bounds.max.y()), m_layers.back()->print_z));
        params.flow_spacing = spacing;
        params.density = config.sparse_infill_density.value * .01;
        params.angle = Geometry::deg2rad(config.infill_direction.value);
        params.throw_if_canceled = [this] { throw_if_canceled(); };
        params.allow_solid_fallback = true;
    }

    // A new predecessor can be discovered one layer below an accepted repair.
    // Its initial request must not inherit tightening from unrelated earlier
    // layers. Scope escalation to this receiving layer, target and exact
    // producer/source-path identity; reversing a path preserves its identity.
    using PathIdentity = std::vector<std::pair<coord_t, coord_t>>;
    using MaskIdentity = std::vector<PathIdentity>;
    using RepairIdentity = std::tuple<size_t, size_t, size_t, double, bool,
        std::vector<Dependency::SourceOwner>, PathIdentity, MaskIdentity>;
    std::map<RepairIdentity, unsigned> local_repairs;
    unsigned refinement_limit = 0;
    for (const auto &params : parameters)
        if (params.flow_spacing > 0.) refinement_limit = std::max(refinement_limit, params.max_depth);
    const size_t per_layer_limit = size_t(refinement_limit) + 1;
    if (m_layers.size() > (std::numeric_limits<size_t>::max() - 1) / per_layer_limit)
        throw SlicingError("Bridge aware cubic propagation exceeds its finite layer work range.", model_object()->id().id);
    // Each construction chain descends through actual model layers, with at
    // most the configured finite refinement depth spent at any one layer.
    const size_t propagation_limit = m_layers.size() * per_layer_limit;

    // Cell identities are scoped to their lattice. Resolve construction
    // dependencies globally before rebuilding any local Plan, so a coincident
    // address in another modifier cannot impersonate the true producer.
    const auto prune_dependencies = [&] {
        ProfileScope profile(ProfilePhase::Prune);
        Dependency::Limits limits;
        limits.throw_if_canceled = [this] { throw_if_canceled(); };
        for (size_t iteration = 0;; ++iteration) {
            if (iteration > demands.size() + obligations.size() + 1)
                throw SlicingError("Bridge aware cubic dependency pruning did not converge.", model_object()->id().id);
            std::vector<Dependency::Producer> producers;
            for (size_t region = 0; region < m_bridge_aware_cubic_plans.size(); ++region)
                if (const auto *plan = bridge_aware_cubic_plan(region))
                    for (const auto &refinement : plan->refinements()) {
                        Dependency::Producer producer;
                        producer.owner = {region, refinement.id};
                        for (size_t index : refinement.demand_indices)
                            producer.demand_indices.push_back(demand_indices[region].at(index));
                        producers.push_back(std::move(producer));
                    }
            auto roots = direct_roots;
            std::map<double, Polylines> closure_preview;
            for (const ClosureRoot &root : closure_roots) {
                auto found = closure_preview.find(root.z);
                if (found == closure_preview.end()) {
                    Polylines paths;
                    for (const Layer *layer : m_layers)
                        if (std::abs(layer->print_z - root.z) <= EPSILON) {
                            paths = BridgeAwareCubic::layer_closure_lines(*layer);
                            break;
                        }
                    found = closure_preview.emplace(root.z, std::move(paths)).first;
                }
                double unmatched = 0.;
                for (const auto &piece : remove_collinear_duplicates({root.producer}, found->second)) unmatched += piece.length();
                // The allowance may complete a real finite overlap, never
                // keep a microscopic course alive after its producer vanished.
                // Use the strict deposition matcher above, not the broader
                // proximity matcher used for rounded ownership fragments.
                if (unmatched < root.producer.length() && unmatched <= scale_(geometry_tolerance))
                    roots.push_back(root.demand);
            }
            const auto reachable = Dependency::resolve(roots, producers, obligations, limits);
            bool changed = false;
            for (size_t region = 0; region < m_bridge_aware_cubic_plans.size(); ++region) {
                auto &plan = m_bridge_aware_cubic_plans[region];
                if (plan == nullptr) continue;
                std::vector<size_t> selected;
                std::vector<BridgeAwareCubic::Demand> active;
                for (size_t index = 0; index < demands[region].size(); ++index)
                    if (reachable.is_active(Dependency::DemandKey{region, index})) {
                        selected.push_back(index);
                        active.push_back(demands[region][index]);
                    }
                if (selected != demand_indices[region]) {
                    ProfileScope build(ProfilePhase::PlanBuild);
                    plan = std::make_unique<BridgeAwareCubic::Plan>(plan->with_demands(std::move(active)));
                    demand_indices[region] = std::move(selected);
                    changed = true;
                }
            }
            if (!changed) break;
        }
    };

    std::vector<Failure> failures;
    // Every accepted repair batch receives a fresh deposition/verification
    // pass, including the final bounded batch. Never report the stale failure
    // that preceded a repair which has not yet been regenerated.
    for (size_t pass = 0; pass <= propagation_limit; ++pass) {
        throw_if_canceled();
        for (size_t region_id = 0; region_id < num_printing_regions(); ++region_id) {
            if (parameters[region_id].flow_spacing <= 0.) continue;
            const auto contextual_error = [&](const std::exception &error) {
                std::ostringstream support_context;
                if (!demands[region_id].empty()) {
                    const auto z_range = std::minmax_element(demands[region_id].begin(), demands[region_id].end(),
                        [](const auto &a, const auto &b) { return a.z < b.z; });
                    const auto support_layer = [this](double z) {
                        auto found = std::lower_bound(m_layers.begin(), m_layers.end(), z - EPSILON,
                            [](const Layer *layer, double value) { return layer->print_z < value; });
                        return found == m_layers.end() ? m_layers.back()->id() + 1 : (*found)->id() + 1;
                    };
                    support_context << ", support layers " << support_layer(z_range.first->z)
                                    << '-' << support_layer(z_range.second->z) << " (Z "
                                    << std::fixed << std::setprecision(3) << z_range.first->z
                                    << '-' << z_range.second->z << " mm)";
                }
                return SlicingError("Bridge aware cubic for object \"" + model_object()->name + "\", region " +
                    std::to_string(region_id + 1) + support_context.str() + ", requested gap " +
                    std::to_string(printing_region(region_id).config().bridge_aware_cubic_max_gap.value) +
                    " mm: " + error.what(), model_object()->id().id);
            };
            try {
                ProfileScope build(ProfilePhase::PlanBuild);
                build.items(demands[region_id].size());
                m_bridge_aware_cubic_plans[region_id] = std::make_unique<BridgeAwareCubic::Plan>(parameters[region_id], demands[region_id],
                    closures[region_id], attachments[region_id]);
                demand_indices[region_id].resize(demands[region_id].size());
                std::iota(demand_indices[region_id].begin(), demand_indices[region_id].end(), size_t(0));
            } catch (const std::logic_error &error) {
                throw contextual_error(error);
            } catch (const std::runtime_error &error) {
                throw contextual_error(error);
            }
        }
        prune_dependencies();
        generate();
        // Attachments alter the actual model path. Preserve the original
        // demand selection while following each committed replay identity, so
        // later validation and coarsening never certify abandoned endpoints.
        for (Target &target : roofs) {
            const Layer &layer = *m_layers[target.upper_layer];
            const ActiveAttachments active = active_attachments(layer);
            for (size_t index = 0; index < target.paths.size(); ++index) {
                const ExtrusionPath &extrusion = target.extrusions[index];
                auto chain = resolve_attachments(layer, target.paths[index], extrusion.width, extrusion.height,
                                                 target.emitting_regions[index], active);
                target.paths[index] = std::move(chain.polyline);
                target.extrusions[index].polyline = target.paths[index];
            }
        }
        failures = check_targets(*this, roofs);
        const auto dependencies = collect_targets(*this, true);
        auto dependency_failures = check_targets(*this, dependencies);
        failures.insert(failures.end(), std::make_move_iterator(dependency_failures.begin()),
                        std::make_move_iterator(dependency_failures.end()));
        if (failures.empty()) {
            // The initial mask selects only air found before construction.
            // Changed deposition can expose another part of the same required
            // model path. Check the complete current obligations before
            // support reduction; a newly exposed seam or strand must receive
            // ordinary bounded repair rather than fail only after coarsening.
            roofs = collect_targets(*this, false, false);
            failures = check_targets(*this, roofs);
            auto complete_dependencies = check_targets(*this, collect_targets(*this, true, false));
            failures.insert(failures.end(), std::make_move_iterator(complete_dependencies.begin()),
                            std::make_move_iterator(complete_dependencies.end()));
        }
        if (failures.empty()) {
            // Freeze all final model paths and their supplying-region masks.
            // Every trial recomputes which portions are actually unsupported,
            // so removing a partition can expose a new target without either
            // retaining an abandoned endpoint or demanding a cell beside an
            // already fully supported perimeter.
            // Test removal against the exact deposited paths. Ideal zero-width
            // cell sizing is only a construction proposal; bead width may make
            // a coarser parent sufficient. A changed cell affects its own layer
            // paths and the next layer's contacts, not every layer in the part.
            const auto prepared_roofs = active_profile && active_profile->full_trial_checks ? nullptr :
                prepare_model_targets(*this, roofs);
            const auto model_inputs = prepare_model_collection_inputs(*this, prepared_roofs.get());
            const auto model_proofs = prepare_model_proofs(*this, prepared_roofs.get(), roofs);
            BridgeAwareCubic::ModelFailureReplay model_replay;
            BridgeAwareCubic::SpanProofCache span_proofs;
            ContactEpochReuse contact_epochs(*this, span_proofs);
            EmissionReuseContext emission_reuse(*this);
            size_t trials = 0;
            const auto changed_plan_layers = [&](const auto &previous, const auto &previous_indices) {
                std::set<size_t> changed;
                const auto mark_range = [&](const BridgeAwareCubic::Refinement &refinement) {
                    for (size_t index = 0; index < m_layers.size(); ++index)
                        if (m_layers[index]->print_z >= refinement.bottom_z - EPSILON &&
                            m_layers[index]->print_z <= refinement.top_z + EPSILON) changed.insert(index);
                };
                const auto witnesses = [](const BridgeAwareCubic::Refinement &refinement, const auto &indices) {
                    std::vector<size_t> result;
                    result.reserve(refinement.demand_indices.size());
                    for (size_t index : refinement.demand_indices) result.push_back(indices.at(index));
                    std::sort(result.begin(), result.end());
                    return result;
                };
                const auto same_closure = [](const auto &a, const auto &b) {
                    return a.z == b.z && a.paths == b.paths && a.reservation == b.reservation &&
                        a.owners == b.owners && a.emitting_region == b.emitting_region;
                };
                const auto same_attachment = [](const auto &a, const auto &b) {
                    return a.z == b.z && a.emitting_region == b.emitting_region &&
                        a.original == b.original && a.replacement == b.replacement &&
                        a.width == b.width && a.height == b.height && a.owners == b.owners &&
                        a.solid_cut == b.solid_cut && a.demand_selection == b.demand_selection;
                };
                for (size_t region = 0; region < previous.size(); ++region) {
                    const auto *before = previous[region].get();
                    const auto *after = m_bridge_aware_cubic_plans[region].get();
                    if (before == nullptr || after == nullptr) {
                        if (before != after)
                            for (size_t index = 0; index < m_layers.size(); ++index) changed.insert(index);
                        continue;
                    }
                    std::map<BridgeAwareCubic::CellId, const BridgeAwareCubic::Refinement*> remaining;
                    for (const auto &refinement : after->refinements()) remaining.emplace(refinement.id, &refinement);
                    for (const auto &refinement : before->refinements()) {
                        const auto found = remaining.find(refinement.id);
                        if (found == remaining.end()) { mark_range(refinement); continue; }
                        const auto &replacement = *found->second;
                        // Local demand indices can shift when the global graph
                        // prunes another obligation. Compare their stable source
                        // identities, not counts or reindexed vector positions.
                        if (refinement.solid != replacement.solid || refinement.bottom_z != replacement.bottom_z ||
                            refinement.top_z != replacement.top_z ||
                            witnesses(refinement, previous_indices[region]) != witnesses(replacement, demand_indices[region])) {
                            mark_range(refinement);
                            mark_range(replacement);
                        }
                        remaining.erase(found);
                    }
                    for (const auto &entry : remaining) mark_range(*entry.second);
                    for (size_t index = 0; index < m_layers.size(); ++index) {
                        throw_if_canceled();
                        const double z = m_layers[index]->print_z;
                        const auto old_closures = before->closures(z), new_closures = after->closures(z);
                        const auto old_attachments = before->attachments(z), new_attachments = after->attachments(z);
                        // Geometry, emitter, owner and target-mask changes all
                        // matter even when the number of saved paths is equal.
                        if (old_closures.size() != new_closures.size() ||
                            !std::equal(old_closures.begin(), old_closures.end(), new_closures.begin(), same_closure) ||
                            old_attachments.size() != new_attachments.size() ||
                            !std::equal(old_attachments.begin(), old_attachments.end(), new_attachments.begin(), same_attachment))
                            changed.insert(index);
                    }
                }
                // Floating solid fill may use the preceding layer's sparse
                // preview (including closures), even if that layer has no
                // remaining sparse BAC surface group. Its preview is rebuilt
                // from plans/surfaces, not from emitted paths, so this direct
                // successor is the complete generation dependency.
                const auto deposition_changed = changed;
                for (size_t index : deposition_changed)
                    if (index + 1 < m_layers.size()) changed.insert(index + 1);
                return changed;
            };
            const auto same_model_targets = [](const std::vector<Target> &a, const std::vector<Target> &b) {
                if (a.size() != b.size()) return false;
                for (size_t i = 0; i < a.size(); ++i) {
                    const Target &left = a[i], &right = b[i];
                    if (left.upper_layer != right.upper_layer || left.supporting_region != right.supporting_region ||
                        left.gap != right.gap || left.bottom_z != right.bottom_z || left.dependency != right.dependency ||
                        left.bridge_followup != right.bridge_followup || left.paths != right.paths ||
                        left.path_widths != right.path_widths || left.footprint != right.footprint ||
                        left.emitting_regions != right.emitting_regions || left.free_ended != right.free_ended ||
                        left.producers != right.producers || left.solid_cuts != right.solid_cuts ||
                        left.path_footprints != right.path_footprints || left.extrusions.size() != right.extrusions.size()) return false;
                    // Polyline equality compares points only. Frozen emitted
                    // preparation may be reused only while both actual and
                    // frozen paths retain the straight-segment representation
                    // that serialization validated before caching them.
                    for (const Polyline &path : left.paths) if (!path.fitting_result.empty()) return false;
                    for (const Polyline &path : right.paths) if (!path.fitting_result.empty()) return false;
                    for (size_t index = 0; index < left.extrusions.size(); ++index) {
                        const ExtrusionPath &before = left.extrusions[index], &after = right.extrusions[index];
                        if (!before.polyline.fitting_result.empty() || !after.polyline.fitting_result.empty() ||
                            before.polyline != after.polyline || before.role() != after.role() ||
                            before.width != after.width || before.height != after.height || before.mm3_per_mm != after.mm3_per_mm ||
                            before.is_force_no_extrusion() != after.is_force_no_extrusion()) return false;
                    }
                }
                return true;
            };
            using Replay = BridgeAwareCubic::ModelFailureReplay;
            struct SparseWitness { Replay::Hint hint; Replay::Kind failure; Replay::ExactWitness evidence; };
            const auto probe_frozen_witness = [&](const Replay::Hint &hint, BridgeAwareCubicValidation::WorkBudget &budget,
                    Replay::ExactWitness *capture, const Replay::ExactWitness *expected)
                    -> std::optional<Replay::Kind> {
                if (!prepared_roofs || hint.frozen_target >= roofs.size()) return {};
                const Target &target = roofs[hint.frozen_target];
                if (target.dependency || hint.path >= target.paths.size()) return {};
                // Full frozen vector, all same-plane demand, complete original
                // path and exact output transform. No positive cache may read
                // or publish through this non-accepting probe.
                const Replay::Probe probe{hint.frozen_target, hint.path, hint.phase};
                const std::vector<size_t> no_remap;
                const PreparedModelView view{prepared_roofs.get(), no_remap, nullptr, &roofs};
                bool checked = false;
                const TargetCheckContext context{&budget, &probe, nullptr, nullptr, &checked};
                // Admit the full vector's candidate/role scans, though the
                // verifier reads actual geometry from only the selected path.
                const auto charge = [&](size_t n) { return emission_charge(*this, budget, n); };
                if (roofs.size() > budget.remaining_work / 4 || !charge(4 * roofs.size())) return {};
                auto failures = check_targets(*this, roofs, nullptr, TargetCheck::RejectTrial, &view, &context);
                if (!checked || failures.size() != 1) return {};
                auto &result = failures.front().result;
                if (!charge(result.unsupported_runs.size()) || !charge(result.void_cells.size())) return {};
                const auto kind = Replay::failure_kind(result);
                if (!kind || (expected && !expected->matches(result, charge)) ||
                    (capture && !capture->take(result, emission_reuse.audit_slack(), charge))) return {};
                return kind;
            };
            const auto check_changed_layers = [&](const std::set<size_t> &changed,
                                                   const Replay::Key *candidate_key = nullptr,
                                                   BridgeAwareCubicValidation::WorkBudget *trial_model_budget = nullptr,
                                                   const SparseWitness *sparse_audit = nullptr,
                                                   bool *audit_matched = nullptr) {
                std::set<size_t> checked_layers;
                {
                    ProfileScope profile(ProfilePhase::GenerateChanged);
                    for (size_t index : changed) {
                        profile.layer(index);
                        const auto *saved = emission_reuse.row(index);
                        m_layers[index]->make_fills(m_adaptive_fill_octrees.first.get(), m_adaptive_fill_octrees.second.get(),
                            m_lightning_generator.get(), true, saved ? &saved->retained : nullptr);
                        if (index > 0) checked_layers.insert(index);
                        if (index + 1 < m_layers.size()) checked_layers.insert(index + 1);
                    }
                }
                BridgeAwareCubicValidation::WorkBudget local_model_budget;
                auto &model_budget = trial_model_budget ? *trial_model_budget : local_model_budget;
                if (sparse_audit) {
                    // Recheck exactly the sparse witness with actual fully
                    // generated predecessor material, not an unrelated failure.
                    // Normal model collection and the full trial still follow.
                    // Defer reporting any mismatch until the ordinary trial
                    // rollback has restored Plans, fills and emission handles.
                    *audit_matched = probe_frozen_witness(sparse_audit->hint, model_budget,
                        nullptr, &sparse_audit->evidence) == sparse_audit->failure;
                    // A missing witness is already an audit failure. Do not
                    // spend a possibly exhausted budget before restoring base.
                    if (!*audit_matched) return false;
                }
                std::vector<Target> selected;
                std::vector<size_t> frozen_indices;
                for (size_t index = 0; index < roofs.size(); ++index)
                    if (checked_layers.count(roofs[index].upper_layer)) {
                        selected.push_back(roofs[index]);
                        frozen_indices.push_back(index);
                    }
                BridgeAwareCubicValidation::WorkBudget collection_budget;
                bool reuse_model_targets = false;
                if (model_inputs) {
                    ProfileScope input_profile(ProfilePhase::ModelInputMatch, &collection_budget);
                    const auto matched = model_inputs->matches(*this, checked_layers, &collection_budget,
                        [&] { throw_if_canceled(); });
                    reuse_model_targets = matched == BridgeAwareCubic::ModelCollectionInputs::Match::Equal;
                    if (active_profile) {
                        if (reuse_model_targets) ++active_profile->model_input_hits;
                        else if (matched == BridgeAwareCubic::ModelCollectionInputs::Match::Different)
                            ++active_profile->model_input_misses;
                        else ++active_profile->model_input_unavailable;
                    }
                }
                std::vector<Target> recollected;
                const auto &actual = reuse_model_targets ? selected :
                    (recollected = collect_targets(*this, false, false, &checked_layers,
                        ConstructionCollection::Reference, nullptr, nullptr, &collection_budget));
                if (reuse_model_targets && active_profile && active_profile->audit_emission_collection) {
                    const auto reference = collect_targets(*this, false, false, &checked_layers,
                        ConstructionCollection::Reference, nullptr, nullptr, &collection_budget);
                    // Pre-admit the complete comparison, including nested
                    // rings, paths and owner vectors. This audit shares the
                    // capture/reference allowance and cannot report success
                    // after exhausting it. No geometry is copied to count it.
                    const auto charge_audit = [&](size_t count, size_t factor = 1) {
                        throw_if_canceled();
                        if (count > collection_budget.remaining_work / factor)
                            throw std::logic_error("Bridge aware cubic model input audit exhausted its comparison budget.");
                        const size_t work = count * factor;
                        collection_budget.remaining_work -= work;
                        collection_budget.charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += work;
                    };
                    const auto audit_polygons = [&](const ExPolygons &polygons) {
                        charge_audit(polygons.size(), 4);
                        for (const auto &polygon : polygons) {
                            charge_audit(polygon.contour.points.size(), 4);
                            charge_audit(polygon.holes.size(), 4);
                            for (const auto &hole : polygon.holes) charge_audit(hole.points.size(), 4);
                        }
                    };
                    const auto audit_targets = [&](const std::vector<Target> &targets) {
                        charge_audit(targets.size(), 32);
                        for (const Target &target : targets) {
                            charge_audit(target.paths.size(), 4);
                            for (const auto &path : target.paths) charge_audit(path.points.size(), 4);
                            charge_audit(target.path_widths.size());
                            charge_audit(target.emitting_regions.size());
                            charge_audit(target.free_ended.size());
                            charge_audit(target.solid_cuts.size());
                            charge_audit(target.producers.size(), 2);
                            for (const auto &owners : target.producers) charge_audit(owners.size(), 8);
                            audit_polygons(target.footprint);
                            charge_audit(target.path_footprints.size(), 2);
                            for (const auto &demand : target.path_footprints) audit_polygons(demand);
                            charge_audit(target.extrusions.size(), 24);
                            for (const auto &path : target.extrusions) charge_audit(path.polyline.points.size(), 4);
                        }
                    };
                    audit_targets(selected);
                    audit_targets(reference);
                    bool equal = same_model_targets(selected, reference);
                    // The normal rejection predicate intentionally concerns
                    // physical model inputs. This audit additionally compares
                    // every copied path metadata field without changing that
                    // production predicate on a cache miss.
                    for (size_t group = 0; equal && group < selected.size(); ++group)
                        for (size_t path = 0; equal && path < selected[group].extrusions.size(); ++path) {
                            const auto &a = selected[group].extrusions[path], &b = reference[group].extrusions[path];
                            equal = a.can_reverse() == b.can_reverse() && a.get_customize_flag() == b.get_customize_flag() &&
                                a.get_cooling_node() == b.get_cooling_node() && a.overhang_degree == b.overhang_degree &&
                                a.curve_degree == b.curve_degree && a.smooth_speed == b.smooth_speed;
                        }
                    if (!equal)
                        throw std::logic_error("Bridge aware cubic model input reuse differed from complete target collection.");
                    throw_if_canceled();
                    ++active_profile->model_input_audits;
                }
                const auto reject_model = [&] {
                    // No reference/projection is requested during generation
                    // or model checks. This trial never reached classification.
                    if (active_profile) ++active_profile->emission_rejected_without_projection;
                    return false;
                };
                // Coarsening may remove support, never adopt a different
                // required model path to avoid its frozen support obligation.
                if (!same_model_targets(selected, actual)) {
                    if (candidate_key) model_replay.erase(*candidate_key);
                    if (active_profile) active_profile->model_replay_entries = model_replay.size();
                    return reject_model();
                }
                const TargetCheck mode = active_profile && active_profile->full_trial_checks ?
                    TargetCheck::CollectFailures : TargetCheck::RejectTrial;
                const PreparedModelView prepared_view{prepared_roofs.get(), frozen_indices, model_proofs.get()};
                std::optional<BridgeAwareCubic::ModelFailureReplay::Hint> first_model_failure;
                TargetCheckContext context{&model_budget, nullptr, &frozen_indices, &first_model_failure, nullptr,
                    &span_proofs, &contact_epochs};
                const bool replay_enabled = candidate_key && mode == TargetCheck::RejectTrial;
                const auto remember_failure = [&] {
                    if (replay_enabled && first_model_failure && model_replay.remember(*candidate_key, *first_model_failure) && active_profile)
                        ++active_profile->model_replay_stores;
                    if (active_profile) active_profile->model_replay_entries = model_replay.size();
                };
                std::vector<Failure> rejected;
                if (replay_enabled) {
                    const auto previous = model_replay.find(*candidate_key);
                    // A failed old hint must not survive a passing new model
                    // check and then masquerade as a construction rejection.
                    model_replay.erase(*candidate_key);
                    if (previous) {
                        const auto probe = BridgeAwareCubic::ModelFailureReplay::resolve(*previous, frozen_indices,
                            [&](size_t index) { return actual[index].paths.size(); }, model_budget,
                            [&] { throw_if_canceled(); });
                        if (probe) {
                            bool checked = false;
                            TargetCheckContext probe_context{&model_budget, &*probe, &frozen_indices, &first_model_failure, &checked};
                            // Same full target view and physical domain, but no
                            // successful-layer proof lookup or publication.
                            const PreparedModelView probe_view{prepared_roofs.get(), frozen_indices, nullptr};
                            if (active_profile) ++active_profile->model_replay_probes;
                            {
                                ProfileScope replay_profile(ProfilePhase::ModelReplay, &model_budget);
                                rejected = check_targets(*this, actual, nullptr, mode,
                                    prepared_roofs ? &probe_view : nullptr, &probe_context);
                            }
                            for (const Failure &failure : rejected)
                                if (!failure.result.input_valid || !failure.result.completed) fail(*this, failure);
                            if (!rejected.empty()) {
                                if (active_profile) ++active_profile->model_replay_rejections;
                                remember_failure();
                                return reject_model();
                            }
                            if (active_profile) checked ? ++active_profile->model_replay_passes : ++active_profile->model_replay_unavailable;
                        } else if (active_profile) ++active_profile->model_replay_unavailable;
                    }
                }
                // A passing/missing probe proves nothing about other paths.
                // Perform the original complete model check with the same
                // remaining budget; no performed probe work is refunded.
                rejected = check_targets(*this, actual, nullptr, mode, prepared_roofs ? &prepared_view : nullptr, &context);
                if (active_profile && model_proofs) {
                    const auto stats = model_proofs->statistics();
                    active_profile->model_layer_proof_hits = stats.hits;
                    active_profile->model_layer_proof_misses = stats.misses;
                    active_profile->model_layer_proof_publications = stats.publications;
                    active_profile->model_layer_proof_vertices = stats.vertices;
                    active_profile->model_layer_proof_items = stats.items;
                }
                for (const Failure &failure : rejected)
                    if (!failure.result.input_valid || !failure.result.completed) fail(*this, failure);
                if (!rejected.empty()) {
                    remember_failure();
                    return reject_model();
                }
                else {
                    TargetCheckContext construction_context;
                    construction_context.span_proofs = &span_proofs;
                    construction_context.contact_epochs = &contact_epochs;
                    const auto construction_targets = collect_targets(*this, true, false, &checked_layers,
                        ConstructionCollection::Emitted, nullptr, &emission_reuse);
                    if (active_profile && active_profile->audit_emission_collection) {
                        const auto reference = collect_targets(*this, true, false, &checked_layers);
                        const std::string mismatch = BridgeAwareCubic::construction_collection_difference(*this,
                            construction_targets, reference);
                        if (!mismatch.empty()) throw std::logic_error(mismatch);
                        // Deliberately read the same generation twice in the
                        // diagnostic audit to exercise exact cache-hit checks.
                        // A normal passing trial requests it only once.
                        const auto repeated = collect_targets(*this, true, false, &checked_layers,
                            ConstructionCollection::Emitted, nullptr, &emission_reuse);
                        const std::string repeat_mismatch = BridgeAwareCubic::construction_collection_difference(*this,
                            repeated, reference);
                        if (!repeat_mismatch.empty()) throw std::logic_error(repeat_mismatch);
                        ++active_profile->emission_collection_audits;
                    }
                    rejected = check_targets(*this, construction_targets, nullptr, mode,
                        nullptr, &construction_context);
                    for (const Failure &failure : rejected)
                        if (!failure.result.input_valid || !failure.result.completed) fail(*this, failure);
                    if (active_profile) active_profile->model_replay_entries = model_replay.size();
                }
                return rejected.empty();
            };
            bool accepted;
            uint64_t sweep_number = 0;
            do {
                ProfileSweep profile_sweep(++sweep_number);
                accepted = false;
                struct Candidate { size_t region; BridgeAwareCubic::Refinement refinement; };
                std::vector<Candidate> candidates;
                for (size_t region = 0; region < m_bridge_aware_cubic_plans.size(); ++region)
                    if (const auto *plan = bridge_aware_cubic_plan(region))
                        for (const auto &refinement : plan->refinements()) candidates.push_back({region, refinement});
                std::sort(candidates.begin(), candidates.end(), [](const auto &a, const auto &b) {
                    if (a.refinement.top_z != b.refinement.top_z) return a.refinement.top_z > b.refinement.top_z;
                    if (a.refinement.id.depth != b.refinement.id.depth) return a.refinement.id.depth > b.refinement.id.depth;
                    return std::tie(a.region, a.refinement.id) < std::tie(b.region, b.refinement.id);
                });
                for (const Candidate &candidate : candidates) {
                    throw_if_canceled();
                    auto &current = m_bridge_aware_cubic_plans[candidate.region];
                    if (std::none_of(current->refinements().begin(), current->refinements().end(), [&](const auto &entry) {
                        return entry.id == candidate.refinement.id;
                    })) continue;
                    if (++trials > 10000)
                        throw SlicingError("Bridge aware cubic exceeded the final coarsening work limit; no minimal support certificate was produced.", model_object()->id().id);
                    if (active_profile) active_profile->trial(candidate.region, &candidate.refinement.id);
                    std::vector<std::unique_ptr<BridgeAwareCubic::Plan>> previous(m_bridge_aware_cubic_plans.size());
                    {
                        ProfileScope profile(ProfilePhase::PlanClone);
                        for (size_t region = 0; region < previous.size(); ++region)
                            if (m_bridge_aware_cubic_plans[region]) {
                                profile.items(1);
                                previous[region] = std::make_unique<BridgeAwareCubic::Plan>(m_bridge_aware_cubic_plans[region]->clone());
                            }
                    }
                    auto previous_indices = demand_indices;
                    {
                        ProfileScope profile(ProfilePhase::PlanCollapse);
                        current = std::make_unique<BridgeAwareCubic::Plan>(current->collapsed(candidate.refinement.id));
                    }
                    prune_dependencies();
                    const auto changed = changed_plan_layers(previous, previous_indices);
                    const Replay::Key candidate_key{candidate.region, candidate.refinement.id};
                    BridgeAwareCubicValidation::WorkBudget trial_budget;
                    std::optional<SparseWitness> sparse;
                    try {
                        if (prepared_roofs && changed.size() > 1 && !(active_profile && active_profile->full_trial_checks)) {
                            const auto old = model_replay.find(candidate_key);
                            if (old && old->frozen_target < roofs.size()) {
                                const size_t upper = roofs[old->frozen_target].upper_layer;
                                if (upper > 0 && upper < m_layers.size() && changed.count(upper - 1)) {
                                    const size_t index = upper - 1;
                                    Layer &lower = *m_layers[index];
                                    if (lower.object() == this &&
                                        sparse_probe_layer_eligible(lower, emission_reuse.row(index), trial_budget)) {
                                        if (active_profile) ++active_profile->sparse_probe_attempts;
                                        Replay::ScopedLowerStage stage(lower.get_region(0)->fills,
                                            lower.bridge_aware_emission_snapshots);
                                        lower.make_fills(m_adaptive_fill_octrees.first.get(), m_adaptive_fill_octrees.second.get(),
                                            m_lightning_generator.get(), false);
                                        sparse.emplace();
                                        sparse->hint = *old;
                                        const bool capture = active_profile && active_profile->audit_sparse_rejections;
                                        if (auto failed = probe_frozen_witness(*old, trial_budget,
                                                capture ? &sparse->evidence : nullptr, nullptr))
                                            sparse->failure = *failed;
                                        else sparse.reset();
                                    }
                                }
                            }
                        }
                    } catch (const std::bad_alloc &) {
                        // Both original and candidate fills are temporarily
                        // resident. The ordinary route frees the original first.
                        sparse.reset();
                    } catch (const SlicingError &) {
                        sparse.reset();
                        // A hypothetical frozen model can be invalid while
                        // the real one would simply differ. Original route
                        // decides this after restoring untouched base geometry.
                    } catch (...) {
                        m_bridge_aware_cubic_plans = std::move(previous);
                        demand_indices.swap(previous_indices);
                        throw;
                    }
                    const bool audit = sparse && active_profile && active_profile->audit_sparse_rejections;
                    if (sparse) {
                        // If any actual checked Target differs from its frozen
                        // version the existing model gate rejects. Otherwise
                        // this fresh failure is on precisely the required path.
                        if (active_profile) {
                            ++active_profile->sparse_probe_rejections;
                            active_profile->sparse_probe_would_skip_generation_layers += changed.size() - 1;
                            active_profile->sparse_probe_would_skip_rollback_layers += changed.size();
                        }
                        if (!audit) {
                            if (active_profile) {
                                ++active_profile->rejected;
                                ++active_profile->emission_rejected_without_projection;
                                active_profile->sparse_probe_saved_generation_layers += changed.size() - 1;
                                active_profile->sparse_probe_saved_rollback_layers += changed.size();
                            }
                            m_bridge_aware_cubic_plans = std::move(previous);
                            demand_indices.swap(previous_indices);
                            continue;
                        }
                    }
                    emission_reuse.save(*this, changed, previous);
                    bool audit_matched = false;
                    bool ordinary_verified = false;
                    std::exception_ptr audit_error;
                    try {
                        ordinary_verified = check_changed_layers(changed, &candidate_key, &trial_budget,
                            audit ? &*sparse : nullptr, &audit_matched);
                    } catch (...) {
                        if (!audit) throw;
                        // Diagnostic generation/checks run only to challenge a
                        // known rejection. Route their exceptions through the
                        // same rollback before propagating the original error.
                        audit_error = std::current_exception();
                    }
                    const bool bad_audit = audit && (audit_error || !audit_matched || ordinary_verified);
                    if (audit && !bad_audit) ++active_profile->sparse_probe_audits;
                    const bool verified = ordinary_verified && !bad_audit;
                    if (active_profile) verified ? ++active_profile->accepted : ++active_profile->rejected;
                    if (verified) {
                        accepted = true;
                        emission_reuse.accept(changed);
                    }
                    else {
                        m_bridge_aware_cubic_plans = std::move(previous);
                        demand_indices.swap(previous_indices);
                        emission_reuse.discard_candidate(changed);
                        ProfileScope profile(ProfilePhase::GenerateRollback);
                        for (size_t index : changed) {
                            profile.layer(index);
                            m_layers[index]->make_fills(m_adaptive_fill_octrees.first.get(), m_adaptive_fill_octrees.second.get(),
                                m_lightning_generator.get(), false);
                            emission_reuse.restore(*m_layers[index], index);
                        }
                        emission_reuse.accept(changed);
                    }
                    if (bad_audit) {
                        if (audit_error) std::rethrow_exception(audit_error);
                        ++active_profile->sparse_probe_audit_mismatches;
                        throw std::logic_error("Sparse rejection and full candidate disagree on the same model witness.");
                    }
                }
                // A shared interface partition is a complete construction
                // stack, not an immortal direct-demand root. Trial-remove all
                // of its identical XY courses together, release their support
                // obligations, and certify the remaining final deposition.
                const auto same_paths = [](const Polylines &a, const Polylines &b) {
                    if (a.size() != b.size()) return false;
                    for (size_t i = 0; i < a.size(); ++i) if (!same_polyline(a[i], b[i])) return false;
                    return true;
                };
                for (size_t owner = 0; owner < closures.size(); ++owner) {
                    if (!m_bridge_aware_cubic_plans[owner]) continue;
                    std::vector<Polylines> stacks;
                    for (const auto &closure : closures[owner])
                        if (std::none_of(stacks.begin(), stacks.end(), [&](const auto &paths) { return same_paths(paths, closure.paths); }))
                            stacks.push_back(closure.paths);
                    for (const auto &stack : stacks) {
                        if (++trials > 10000)
                            throw SlicingError("Bridge aware cubic exceeded the final closure coarsening work limit.", model_object()->id().id);
                        if (active_profile) active_profile->trial(owner, nullptr);
                        std::vector<std::unique_ptr<BridgeAwareCubic::Plan>> previous(m_bridge_aware_cubic_plans.size());
                        {
                            ProfileScope profile(ProfilePhase::PlanClone);
                            for (size_t region = 0; region < previous.size(); ++region)
                                if (m_bridge_aware_cubic_plans[region]) {
                                    profile.items(1);
                                    previous[region] = std::make_unique<BridgeAwareCubic::Plan>(m_bridge_aware_cubic_plans[region]->clone());
                                }
                        }
                        const auto previous_indices = demand_indices;
                        const auto previous_closures = closures[owner];
                        auto &remaining = closures[owner];
                        remaining.erase(std::remove_if(remaining.begin(), remaining.end(), [&](const auto &closure) {
                            return same_paths(stack, closure.paths);
                        }), remaining.end());
                        {
                            ProfileScope profile(ProfilePhase::PlanPayload);
                            m_bridge_aware_cubic_plans[owner] = std::make_unique<BridgeAwareCubic::Plan>(
                                m_bridge_aware_cubic_plans[owner]->with_payloads(remaining, attachments[owner]));
                        }
                        prune_dependencies();
                        const auto changed = changed_plan_layers(previous, previous_indices);
                        emission_reuse.save(*this, changed, previous);
                        const bool verified = check_changed_layers(changed);
                        if (active_profile) verified ? ++active_profile->accepted : ++active_profile->rejected;
                        if (verified) {
                            accepted = true;
                            emission_reuse.accept(changed);
                        }
                        else {
                            m_bridge_aware_cubic_plans = std::move(previous);
                            demand_indices = previous_indices;
                            closures[owner] = previous_closures;
                            emission_reuse.discard_candidate(changed);
                            ProfileScope profile(ProfilePhase::GenerateRollback);
                            for (size_t index : changed) {
                                profile.layer(index);
                                m_layers[index]->make_fills(m_adaptive_fill_octrees.first.get(), m_adaptive_fill_octrees.second.get(),
                                    m_lightning_generator.get(), false);
                                emission_reuse.restore(*m_layers[index], index);
                            }
                            emission_reuse.accept(changed);
                        }
                    }
                }
            } while (accepted);
            // Trimming consumes only the actual restored sparse fragments.
            // Combination decides its layer windows later, so preserve the
            // full reservation when it is enabled for any participating plan.
            bool need_reservations = has_restored_sparse;
            for (size_t r = 0; !need_reservations && r < num_printing_regions(); ++r)
                need_reservations = bridge_aware_cubic_plan(r) != nullptr && printing_region(r).config().infill_combination.value;

            const bool reference_reservations = need_reservations && active_profile && active_profile->full_trial_checks;
            std::map<std::pair<size_t, size_t>, ExPolygons> selected;
            const auto stage = [&](std::vector<Target> &targets) {
                for (Target &target : targets) {
                    throw_if_canceled();
                    append(selected[{target.upper_layer - 1, target.supporting_region}], std::move(target.footprint));
                }
            };
            const auto final_phases = serialization_phases(*this, nullptr);
            {
                ProfileScope profile(ProfilePhase::FinalProof);
                final_proof_preamble(*this, *this, final_phases);
                {
                    auto model = collect_targets(*this, false, false);
                    if (!same_model_targets(roofs, model))
                        throw SlicingError("Bridge aware cubic could not preserve the required model paths during final support reduction.", model_object()->id().id);
                    failures = check_targets(*this, model);
                    if (!failures.empty()) fail(*this, failures.front());
                    if (need_reservations && !reference_reservations) stage(model);
                }
                auto construction = collect_targets(*this, true, false);
                failures = check_targets(*this, construction);
                if (!failures.empty()) fail(*this, failures.front());
                if (need_reservations && !reference_reservations) stage(construction);
            }
            if (need_reservations) {
                // An already-supported added strand still needs its actual
                // preceding scaffold protected from trimming or combination.
                if (reference_reservations) {
                    // Independent oracle: recollect both ordered target sets,
                    // as before, instead of consuming the just-checked ones.
                    {
                        auto model = collect_targets(*this, false, false);
                        stage(model);
                    }
                    auto construction = collect_targets(*this, true, false);
                    stage(construction);
                }
                throw_if_canceled();
                m_bridge_aware_cubic_reservations.resize(m_layers.size());
                for (auto &regions : m_bridge_aware_cubic_reservations)
                    regions.resize(num_printing_regions());
                for (auto &entry : selected) {
                    throw_if_canceled();
                    const size_t layer_index = entry.first.first;
                    const size_t region_id = entry.first.second;
                    const auto *plan = bridge_aware_cubic_plan(region_id);
                    if (plan == nullptr) continue;
                    const ExPolygons footprint = union_ex(entry.second);
                    ExPolygons &reserved = m_bridge_aware_cubic_reservations[layer_index][region_id];
                    for (const auto &cell : plan->leaf_sections(m_layers[layer_index]->print_z))
                        if (!intersection_ex(ExPolygon(cell.polygon), footprint).empty())
                            reserved.emplace_back(cell.polygon);
                    reserved = union_ex(reserved);
                }
            }
            throw_if_canceled();
            // Reservations are part of the owned state too. They do not alter
            // the just-proven emitted paths, and must be captured after their
            // final values are installed. Capture alone never asserts a proof.
            std::atomic_store(&m_bridge_aware_cubic_certificate, capture_final_proof(*this, *this, final_phases));
            return;
        }

        if (pass == propagation_limit) {
            failures.front().result.completed = false;
            failures.front().result.resource_limited = true;
            failures.front().result.failure_reason =
                "The bounded support-repair work limit was reached before a complete support certificate could be produced.";
            fail(*this, failures.front());
        }
        bool repaired = false;
        // Plans remain immutable during this repair pass. Reuse each layer's
        // exact clipped emission batch across paths and producing-cell queries.
        std::map<std::pair<size_t, size_t>,
            std::vector<std::pair<BridgeAwareCubic::CellId, Polylines>>> partition_batches;
        for (const Failure &failure : failures) {
            if (!failure.result.input_valid || !failure.result.completed) fail(*this, failure);
            // A short free tail can be repaired only by extending real upper
            // deposition into qualified prior material inside its own region.
            if (repair_attachments(*this, failure, attachments)) { repaired = true; continue; }
            // A physically open strip beside a wall is closed only by actual
            // deposition joining that wall, never by the logical fill boundary.
            if (repair_wall_connections(*this, failure, attachments)) { repaired = true; continue; }
            // Independent modifier lattices have no implicit wall at their
            // logical boundary. Construct an owned physical partition and its
            // preceding courses before asking either lattice to split again.
            if (repair_interfaces(*this, failure, closures)) { repaired = true; continue; }
            if (failure.target.bridge_followup) fail(*this, failure);
            const size_t lower_index = failure.target.upper_layer - 1;
            const Layer &lower = *m_layers[lower_index];
            struct Fragment {
                ExPolygons footprint;
                std::vector<Dependency::SourceOwner> sources;
                Polyline closure_source;
                Polyline source_path;
            };
            std::vector<Fragment> fragments;
            if (failure.target.dependency) {
                // Distinct emitted strands are separate obligations. OR over
                // producers of different fragments would keep an orphan after
                // only one fragment's producer survived.
                const Layer &construction_layer = *m_layers[failure.target.upper_layer];
                const ActiveAttachments active = active_attachments(construction_layer);
                for (size_t index = 0; index < failure.target.paths.size(); ++index) {
                    Failure selected;
                    selected.target.upper_layer = failure.target.upper_layer;
                    selected.target.supporting_region = failure.target.supporting_region;
                    selected.target.gap = failure.target.gap;
                    selected.target.dependency = true;
                    selected.target.paths = {failure.target.paths[index]};
                    selected.target.path_widths = {failure.target.path_widths[index]};
                    selected.target.extrusions = {failure.target.extrusions[index]};
                    selected.target.footprint = path_demand(failure.target, index);
                    selected.target.path_footprints = {selected.target.footprint};
                    if (!failure.verified_paths.empty()) selected.verified_paths = {failure.verified_paths[index]};
                    for (auto run : failure.result.unsupported_runs)
                        if (run.path_index == index && (run.exceeds_gap ||
                            run.kind != BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge)) {
                            run.path_index = 0;
                            selected.result.unsupported_runs.push_back(run);
                        }
                    if (selected.result.unsupported_runs.empty()) continue;
                    const ExPolygons failed = repair_footprint(selected);
                    const size_t emitter = failure.target.emitting_regions[index];
                    const auto *producer_plan = bridge_aware_cubic_plan(emitter);
                    ExPolygons covered;
                    if (producer_plan != nullptr) {
                        const double z = m_layers[failure.target.upper_layer]->print_z;
                        const ExPolygon domain(Polygon{Points{bounds.min, Point(bounds.max.x(), bounds.min.y()), bounds.max,
                            Point(bounds.min.x(), bounds.max.y())}});
                        const auto key = std::make_pair(failure.target.upper_layer, emitter);
                        auto cached = partition_batches.find(key);
                        if (cached == partition_batches.end())
                            cached = partition_batches.emplace(key, producer_plan->partition_lines(z, domain)).first;
                        const auto owner_lines = [&](const BridgeAwareCubic::CellId &owner) -> const Polylines& {
                            const auto &batch = cached->second;
                            const auto found = std::lower_bound(batch.begin(), batch.end(), owner,
                                [](const auto &entry, const auto &id) { return entry.first < id; });
                            static const Polylines empty;
                            return found != batch.end() && found->first == owner ? found->second : empty;
                        };
                        Polylines segments;
                        const Polyline &path = failure.target.paths[index];
                        for (size_t i = 1; i < path.size(); ++i)
                            segments.emplace_back(Points{path.points[i - 1], path.points[i]});
                        std::set<size_t> lineage;
                        for (const auto &attachment : active.entries)
                            if (attachment.emitting_region == emitter) {
                                const auto chain = resolve_attachments(construction_layer, attachment.original,
                                    failure.target.extrusions[index].width, failure.target.extrusions[index].height, emitter, active);
                                if (!chain.applied_indices.empty() && same_polyline(path, chain.polyline))
                                    lineage.insert(chain.applied_indices.begin(), chain.applied_indices.end());
                            }
                        const auto solid_sections = producer_plan->solid_sections(z);
                        for (const auto &refinement : producer_plan->refinements()) {
                            ExPolygons produced;
                            if (refinement.solid) {
                                for (const auto &cell : solid_sections)
                                    if (cell.cell_id == refinement.id) {
                                        if (failure.target.solid_cuts[index]) {
                                            if (std::find(failure.target.producers[index].begin(), failure.target.producers[index].end(), refinement.id) !=
                                                failure.target.producers[index].end()) {
                                                ExPolygons selection = BridgeAwareCubic::solid_cut_endpoint_demand(path,
                                                    {ExPolygon(cell.polygon)}, failure.target.path_widths[index], geometry_tolerance);
                                                for (size_t id : lineage) {
                                                    const auto &attachment = active.entries[id];
                                                    if (active.storage_regions[id] == emitter && attachment.solid_cut &&
                                                        std::find(attachment.owners.begin(), attachment.owners.end(), refinement.id) !=
                                                            attachment.owners.end())
                                                        append(selection, attachment.demand_selection);
                                                }
                                                // The collector selects a square terminal
                                                // neighbourhood, whose diagonal corner need
                                                // not lie in an offset of the solid cell.
                                                // Reuse that exact selector on the real bead.
                                                append(produced, BridgeAwareCubic::solid_cut_provenance_footprint(
                                                    path, union_ex(selection), failure.target.path_widths[index]));
                                            }
                                        } else {
                                            // Solid fill is finite-width deposition. Its
                                            // owning cell contains the centreline, while
                                            // the printed edge can extend beyond that
                                            // section by half the real bead width.
                                            const auto centre = intersection_pl(Polylines{path}, ExPolygons{ExPolygon(cell.polygon)});
                                            append(produced, BridgeAwareCubic::partition_provenance_footprint(centre,
                                                failure.target.path_widths[index], 2. * geometry_tolerance, &segments));
                                        }
                                    }
                            } else {
                                const Polylines &lines = owner_lines(refinement.id);
                                const auto matching = remove_collinear_duplicates(segments, lines, true);
                                produced = BridgeAwareCubic::partition_provenance_footprint(
                                    matching, failure.target.path_widths[index], geometry_tolerance, &segments);
                            }
                            for (size_t id : lineage) {
                                const auto &attachment = active.entries[id];
                                if (active.storage_regions[id] != emitter ||
                                    std::find(attachment.owners.begin(), attachment.owners.end(), refinement.id) == attachment.owners.end()) continue;
                                // Buffer only the real terminal addition. A
                                // widened replacement minus a narrower original
                                // creates a false halo along unchanged material.
                                Polylines added;
                                if (attachment.original.first_point() != attachment.replacement.first_point())
                                    added.emplace_back(Points{attachment.replacement.first_point(), attachment.original.first_point()});
                                if (attachment.original.last_point() != attachment.replacement.last_point())
                                    added.emplace_back(Points{attachment.original.last_point(), attachment.replacement.last_point()});
                                append(produced, BridgeAwareCubic::partition_provenance_footprint(added,
                                    failure.target.path_widths[index], 2. * geometry_tolerance, &segments));
                            }
                            ExPolygons local = intersection_ex(failed, produced);
                            if (local.empty()) continue;
                            // Keep the original provenance envelope for this
                            // coverage proof. Re-unioning rounded intersections
                            // can manufacture a second sliver along failed's edge.
                            append(covered, produced);
                            // The local selected fragment identifies the exact
                            // producer that creates this obligation. Its complete
                            // failing air run can require a landing in another
                            // participating region, including beyond an attached
                            // endpoint's original modifier boundary.
                            ExPolygons complete_run = BridgeAwareCubic::select_unsupported_run_demands(
                                selected.target.paths, selected.verified_paths, selected.target.path_widths,
                                {local}, selected.result.unsupported_runs, geometry_tolerance, true);
                            fragments.push_back({std::move(complete_run), {{emitter, refinement.id}}, {}, path});
                        }
                    }
                    // Persistent interface courses are explicit direct
                    // obligations. Any residual comes from their actual saved
                    // path, never from a perpendicular crossing lattice face.
                    ExPolygons residual = diff_ex(failed, union_ex(covered));
                    if (!residual.empty()) {
                        ExPolygons explicit_courses;
                        const double z = m_layers[failure.target.upper_layer]->print_z;
                        const auto matching = remove_collinear_duplicates({failure.target.paths[index]},
                            BridgeAwareCubic::layer_closure_lines(*m_layers[failure.target.upper_layer]), true);
                        const Polylines source{failure.target.paths[index]};
                        append(explicit_courses, BridgeAwareCubic::partition_provenance_footprint(matching,
                            failure.target.path_widths[index], geometry_tolerance, &source));
                        if (!diff_ex(residual, union_ex(explicit_courses)).empty()) {
                            throw SlicingError("Bridge aware cubic could not identify the exact producer of a required construction fragment for object \"" +
                                model_object()->name + "\", layer " + std::to_string(m_layers[failure.target.upper_layer]->id() + 1) +
                                " (Z " + std::to_string(z) + " mm).", model_object()->id().id);
                        }
                        ExPolygons complete_run = BridgeAwareCubic::select_unsupported_run_demands(
                            selected.target.paths, selected.verified_paths, selected.target.path_widths,
                            {residual}, selected.result.unsupported_runs, geometry_tolerance, true);
                        fragments.push_back({std::move(complete_run), {}, failure.target.paths[index], failure.target.paths[index]});
                    }
                }
            } else fragments.push_back({repair_footprint(failure, true), {}});
            // Propagate the specific failed footprint into allowed sparse
            // material. Do not refine a roof's bounding box or a whole ancestor.
            for (const Fragment &fragment : fragments) for (size_t region_id = 0; region_id < lower.regions().size(); ++region_id) {
                if (parameters[region_id].flow_spacing <= 0.) continue;
                ExPolygons local = intersection_ex(fragment.footprint, sparse_domain(*lower.get_region(int(region_id))));
                if (local.empty()) continue;
                PathIdentity path_identity;
                path_identity.reserve(fragment.source_path.size());
                for (const Point &point : fragment.source_path.points)
                    path_identity.emplace_back(point.x(), point.y());
                const PathIdentity reversed(path_identity.rbegin(), path_identity.rend());
                if (reversed < path_identity) path_identity = reversed;
                auto sources = fragment.sources;
                std::sort(sources.begin(), sources.end());
                sources.erase(std::unique(sources.begin(), sources.end()), sources.end());
                MaskIdentity mask_identity;
                if (!failure.target.dependency) {
                    // Distinct direct failures in one roof group must not
                    // inherit each other's refinement strength. Polygon
                    // winding is retained while cyclic starts/order normalize.
                    for (const Polygon &ring : to_polygons(local)) {
                        PathIdentity points;
                        points.reserve(ring.size());
                        for (const Point &point : ring.points) points.emplace_back(point.x(), point.y());
                        if (!points.empty()) std::rotate(points.begin(), std::min_element(points.begin(), points.end()), points.end());
                        mask_identity.push_back(std::move(points));
                    }
                    std::sort(mask_identity.begin(), mask_identity.end());
                }
                const RepairIdentity identity{lower_index, region_id, failure.target.supporting_region,
                    failure.target.bottom_z, failure.target.dependency, std::move(sources), std::move(path_identity), std::move(mask_identity)};
                unsigned &attempt = local_repairs[identity];
                const double gap = std::ldexp(std::min(failure.target.gap,
                    printing_region(region_id).config().bridge_aware_cubic_max_gap.value),
                    -int(attempt + (failure.target.dependency ? 0 : 1)));
                // A sub-spacing gap may be feasible with a supported solid
                // patch. The emitted solid geometry, not nominal line spacing,
                // decides feasibility in the next authoritative check.
                if (attempt >= parameters[region_id].max_depth || gap <= geometry_tolerance) {
                    Failure incomplete = failure;
                    incomplete.result.completed = false;
                    incomplete.result.resource_limited = true;
                    incomplete.result.failure_reason =
                        "A local support obligation exhausted the finite refinement depth or geometry resolution before verification completed.";
                    fail(*this, incomplete);
                }
                const Dependency::DemandKey key{region_id, demands[region_id].size()};
                if (!fragment.sources.empty()) obligations.push_back({key, fragment.sources});
                else if (fragment.closure_source.size() >= 2)
                    closure_roots.push_back({key, m_layers[failure.target.upper_layer]->print_z, fragment.closure_source});
                else direct_roots.push_back(key);
                demands[region_id].push_back({std::move(local), lower.print_z, gap - geometry_tolerance});
                ++attempt;
                repaired = true;
            }
        }
        if (!repaired) break;
    }
    if (!failures.empty()) fail(*this, failures.front());
}

void PrintObject::clear_bridge_aware_cubic_certificate() const
{
    std::atomic_store(&m_bridge_aware_cubic_certificate,
        std::shared_ptr<const BridgeAwareCubic::DepositionStateSnapshot>{});
}

void PrintObject::validate_bridge_aware_cubic() const
{
    if (!has_bridge_aware_cubic()) return;
    ProfileSession profile_session("validate", model_object()->id().id);
    const PrintObject &geometry = m_shared_object == nullptr ? *this : *m_shared_object;
    // Cancellation and the complete physical-nozzle guard are always fresh.
    // Separate phase components preserve writer operation order; new instance
    // phases cannot inherit a proof established only at an older placement.
    print()->throw_if_canceled();
    const auto phases = serialization_phases(geometry, this);
    auto certificate = std::atomic_load(&geometry.m_bridge_aware_cubic_certificate);
    bool hit = false;
    if (certificate && !(active_profile && active_profile->full_trial_checks)) {
        ProfileScope profile(ProfilePhase::CompareFinalProof);
        hit = certificate->matches(geometry, *this, phases, [&] { print()->throw_if_canceled(); });
    }
    if (hit) {
        if (active_profile) {
            ++active_profile->final_proof_hits;
            active_profile->final_proof_bytes = certificate->retained_bytes();
        }
        return;
    }
    if (active_profile) ++active_profile->final_proof_misses;
    // Release retired Plan tokens before proving/capturing replacement state.
    // The exact comparison, rather than invalidation notifications, is the
    // authority: all public mutable path/config/surface trees may change.
    geometry.clear_bridge_aware_cubic_certificate();
    certificate.reset();
    final_proof_preamble(geometry, *this, phases);
    auto failures = check_targets(geometry, collect_targets(geometry, false, false), this);
    if (!failures.empty()) fail(geometry, failures.front());
    failures = check_targets(geometry, collect_targets(geometry, true, false), this);
    if (!failures.empty()) fail(geometry, failures.front());
    std::atomic_store(&geometry.m_bridge_aware_cubic_certificate, capture_final_proof(geometry, *this, phases));
}

} // namespace Slic3r
