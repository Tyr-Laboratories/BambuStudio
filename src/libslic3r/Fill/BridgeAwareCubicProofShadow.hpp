#ifndef slic3r_BridgeAwareCubicProofShadow_hpp_
#define slic3r_BridgeAwareCubicProofShadow_hpp_
#include "BridgeAwareCubicContact.hpp"
#include "BridgeAwareCubicValidation.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <ostream>

namespace Slic3r { namespace BridgeAwareCubic { namespace ProofShadow {
using Clock = std::chrono::steady_clock;
namespace V = BridgeAwareCubicValidation;
namespace C = BridgeAwareCubicContact;
enum class Stage { Initial, FirstSweep, LaterSweep, Final, Export, Count };
enum class Outcome { Pass, Fail, Unavailable };
struct Row {
    uint64_t direct_started = 0, direct_empty_pass = 0, direct_empty_fail = 0, direct_nonempty_pass = 0, direct_nonempty_fail = 0;
    uint64_t direct_invalid = 0, direct_empty_ns = 0, direct_nonempty_ns = 0, direct_cell_ns = 0;
    uint64_t layer_empty = 0, layer_nonempty = 0, layer_unknown = 0, layer_probe = 0, layer_unqualified = 0;
    uint64_t line_pass = 0, line_fail = 0, line_unknown = 0, line_pass_reference_fail = 0;
    uint64_t line_pass_reference_ns = 0, line_fail_reference_ns = 0;
    uint64_t line_fail_reference_pass = 0, span_preproved = 0, span_shortcut = 0, line_ns = 0;
    void write(std::ostream &out) const {
#define BAC_SHADOW_FIELD(name) out << ",\"" #name "\":" << name
        out << "{\"schema\":1";
        BAC_SHADOW_FIELD(direct_started); BAC_SHADOW_FIELD(direct_empty_pass); BAC_SHADOW_FIELD(direct_empty_fail);
        BAC_SHADOW_FIELD(direct_nonempty_pass); BAC_SHADOW_FIELD(direct_nonempty_fail);
        BAC_SHADOW_FIELD(direct_invalid); BAC_SHADOW_FIELD(direct_empty_ns);
        BAC_SHADOW_FIELD(direct_nonempty_ns); BAC_SHADOW_FIELD(direct_cell_ns);
        BAC_SHADOW_FIELD(layer_empty); BAC_SHADOW_FIELD(layer_nonempty);
        BAC_SHADOW_FIELD(layer_unknown); BAC_SHADOW_FIELD(layer_probe); BAC_SHADOW_FIELD(layer_unqualified);
        BAC_SHADOW_FIELD(line_pass); BAC_SHADOW_FIELD(line_fail); BAC_SHADOW_FIELD(line_unknown);
        BAC_SHADOW_FIELD(line_pass_reference_fail); BAC_SHADOW_FIELD(line_fail_reference_pass);
        BAC_SHADOW_FIELD(line_pass_reference_ns); BAC_SHADOW_FIELD(line_fail_reference_ns);
        BAC_SHADOW_FIELD(span_preproved); BAC_SHADOW_FIELD(span_shortcut); BAC_SHADOW_FIELD(line_ns);
        out << '}';
#undef BAC_SHADOW_FIELD
    }
};

inline uint64_t ns(Clock::time_point start) {
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count());
}

struct Timer {
    uint64_t *destination;
    Clock::time_point start;
    explicit Timer(uint64_t *out) : destination(out), start(out ? Clock::now() : Clock::time_point{}) {}
    ~Timer() { if (destination) *destination += ns(start); }
};

// Counts visited contiguous layer/phase views only. A model-layer cache hit,
// no-op path, incompletely checked target, or early rejection cannot count as
// an all-empty proof. A single-path replay is always an incomplete view.
struct Layer {
    Row *row;
    bool probe, qualified;
    size_t id = size_t(-1), expected = 0, observed = 0, empty_passes = 0;
    bool saw_nonempty = false, unknown = false;
    explicit Layer(Row *r, bool p, bool q = true) : row(r), probe(p), qualified(q) {}
    void finish() {
        if (row && expected) {
            if (probe) ++row->layer_probe;
            else if (!qualified) ++row->layer_unqualified;
            else if (unknown || observed != expected) ++row->layer_unknown;
            else if (empty_passes == expected) ++row->layer_empty;
            else if (saw_nonempty) ++row->layer_nonempty;
            else ++row->layer_unknown;
        }
    }
    ~Layer() { finish(); }
    void begin(size_t layer, size_t n) {
        finish(); id = layer; expected = n; observed = empty_passes = 0; saw_nonempty = unknown = false;
    }
    void note(bool empty, const V::Result &result, Clock::time_point start) {
        ++observed;
        if (!result.input_valid || !result.completed || result.resource_limited) {
            ++row->direct_invalid; unknown = true; return;
        }
        const bool pass = result.passed();
        if (empty) {
            (pass ? row->direct_empty_pass : row->direct_empty_fail)++;
            row->direct_empty_ns += ns(start);
            if (pass) ++empty_passes; else unknown = true;
        } else {
            saw_nonempty = true;
            (pass ? row->direct_nonempty_pass : row->direct_nonempty_fail)++;
            row->direct_nonempty_ns += ns(start);
        }
    }
};

struct Metrics {
    Stage stage = Stage::Initial;
    std::array<Row, size_t(Stage::Count)> rows{};
    static constexpr size_t work_limit = 500000000;
    size_t remaining = work_limit;
    Row &at() { return rows[size_t(stage)]; }
    void write(std::ostream &out) const {
        static constexpr const char *names[] = {"initial","first_sweep","later_sweep","final","export"};
        out << ",\"shadow_work_remaining\":" << remaining << ",\"shadow_proofs\":{";
        for (size_t i = 0; i < rows.size(); ++i) {
            if (i) out << ',';
            out << '"' << names[i] << "\":"; rows[i].write(out);
        }
        out << '}';
    }
};

struct RestoreStage {
    Metrics *m;
    Stage old;
    RestoreStage(Metrics *metrics, Stage next) : m(metrics), old(m ? m->stage : Stage::Initial) {
        if (m) m->stage = next;
    }
    ~RestoreStage() { if (m) m->stage = old; }
};

// Only fresh real ContactsOnly path output; the synthetic whole-path shortcut
// and cache hits do not supply raw line witnesses. No mask input is copied.
// Fixed geometry ceilings bound both the independent inputs and the verifier's
// line-only scratch (intervals, segments, and at most one extra final run).
template<class Cancel>
Outcome lines(const Polyline &path, const C::PathContact &raw, double gap, size_t &remaining, Cancel &&cancel)
{
    cancel();
    constexpr size_t max_points = 512, max_events = 1024, max_work = 50000;
    if (!raw.input_valid || !raw.completed || raw.resource_limited || !raw.failure_reason.empty() ||
        !std::isfinite(raw.contact_z) || raw.coordinate_uncertainty_mm != 0. ||
        !raw.physical_lines.empty() || !raw.physical_footprint.empty() || !path.fitting_result.empty() ||
        path.size() < 2 || path.size() > max_points || raw.segments.size() >= max_points)
        return Outcome::Unavailable;
    size_t events = 0;
    // Charge observable admission work even on a later decline.
    const size_t metadata = 1 + raw.segments.size();
    if (remaining < metadata) { remaining = 0; return Outcome::Unavailable; }
    remaining -= metadata;
    for (const auto &s : raw.segments) {
        if (s.line_contacts.size() > max_events - events) return Outcome::Unavailable;
        events += s.line_contacts.size();
    }
    // Account input copies and a conservative upper bound for the validator's
    // unmetered interval sort (bounded n log n); same session does not refill.
    const size_t extra = path.size() + events * 32 + raw.segments.size();
    if (remaining < extra) { remaining = 0; return Outcome::Unavailable; }
    remaining -= extra;
    const size_t allow = std::min(max_work, remaining);
    if (!allow) return Outcome::Unavailable;
    V::WorkBudget work{allow};
    Outcome answer = Outcome::Unavailable;
    try {
        Polylines paths(1); paths.front().points = path.points;
        std::vector<C::PathContact> lowered(1);
        auto &thin = lowered.front();
        thin.contact_z = raw.contact_z;
        thin.segments.reserve(raw.segments.size());
        for (const auto &s : raw.segments)
            thin.segments.push_back({s.segment_index, {}, s.line_contacts});
        const auto r = V::validate_complete_contact_spans(paths, gap, 0., lowered, &work);
        if (r.input_valid && r.completed && !r.resource_limited)
            answer = r.passed() ? Outcome::Pass : Outcome::Fail;
    } catch (const std::bad_alloc &) {} catch (const std::length_error &) {}
    remaining -= allow - work.remaining_work;
    cancel(); // Outside allocation catches: cancellation always propagates unchanged.
    return answer;
}
}}}
#endif
