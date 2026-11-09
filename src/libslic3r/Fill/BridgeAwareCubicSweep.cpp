// Bridge aware cubic: the support sweep (rewrite architecture, section 3.7).
//
// The sweep holds every strand the refinement laid, from the layer below, with lattice columns: a small hold rule and a greedy
// placement, no search. It goes down each group from its highest layer that has a strand to layer 0, one layer at a time; at
// layer m it reads what prints at m − 1: the bulk lines inside the fill boundary, the strands of m − 1 and the material that
// surely prints. Solid-column courses are held by section 3.6c and are not swept.

#include "BridgeAwareCubicPlan.hpp"

#include "../ClipperUtils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace Slic3r {
namespace BridgeAwareCubic {

namespace {

constexpr double MU            = 0.1;      // the planning margin: G_b = G − μ
constexpr double EPSILON_MM    = 0.01;     // the touching measure of the checker and the verifier
constexpr int    LOOK_AHEAD    = 3;        // layers below a holder in which a column must have room
constexpr double LOOK_ALONG    = 20.;      // mm along its trace in which a column looks for material
constexpr int    PLANE_LEVELS  = 16;       // the plane through a point: levels 1 to 16 (plane_through)
constexpr double SIN_60        = 0.86602540378443864676;
constexpr float  ARC_TOLERANCE = float(scale_(0.001));

using Interval  = std::pair<double, double>;
using Intervals = std::vector<Interval>;

// The union of intervals, sorted and merged.
Intervals merged(Intervals intervals)
{
    std::sort(intervals.begin(), intervals.end());
    Intervals out;
    for (const Interval &i : intervals) {
        if (i.second < i.first)
            continue;
        if (!out.empty() && i.first <= out.back().second)
            out.back().second = std::max(out.back().second, i.second);
        else
            out.push_back(i);
    }
    return out;
}

// [t0, t1] minus the (merged) intervals.
Intervals complement(const Intervals &held, double t0, double t1)
{
    Intervals out;
    double cursor = t0;
    for (const Interval &h : held) {
        if (h.second < cursor)
            continue;
        if (h.first > t1)
            break;
        if (h.first > cursor)
            out.emplace_back(cursor, h.first);
        cursor = std::max(cursor, h.second);
    }
    if (cursor < t1)
        out.emplace_back(cursor, t1);
    return out;
}

bool covers(const Intervals &held, double t)
{
    for (const Interval &h : held)
        if (h.first <= t && t <= h.second)
            return true;
    return false;
}

// Where the trace of plane J at height zj crosses the trace of K at height zk, as a parameter along K; nothing when parallel.
std::optional<double> cross_parameter(const Trace &k, const Trace &j)
{
    const Vec2d nj(-j.dir.y(), j.dir.x());
    const double denominator = nj.dot(k.dir);
    if (std::abs(denominator) < 1e-9)
        return std::nullopt;
    return (nj.dot(j.origin) - nj.dot(k.origin)) / denominator;
}

// One layer's print set, as the sweep reads it.
struct LayerPrint
{
    size_t                                      layer = 0;
    double                                      z = 0.;
    const LayerBoundary                        *boundary = nullptr;
    std::unique_ptr<AreaIndex>                  material_reach;   // offset_round(M⁻, w/2)
    std::map<PlaneKey, Intervals>               strands_by_plane; // every strand of the layer, by plane
    std::vector<std::pair<PlaneKey, Interval>>  strands_of_family[3];
};

class Sweep
{
public:
    Sweep(GroupPlan &group, ObjectPlan &plan, const std::function<void()> &cancel)
        : m_group(group), m_plan(plan), m_cancel(cancel), m_lattice(group.lattice())
    {
        m_w     = m_lattice.width;
        m_c     = m_w / std::sqrt(3.);
        m_cN    = m_c + (m_lattice.multiline - 1) * m_lattice.spacing / (2. * std::sqrt(3.));
        m_lambda = m_w;
        m_tau   = (m_c - 0.5 * m_w) * SIN_60 / 4.;
        // Taken off the reaches (ObjectPlan::vertex_margin, path_margin): a point held at exactly α, c or w/2 in the plan would read
        // as free on the export after path simplification and coordinate rounding. The material reach w/2 gets the path margin
        // less the checker's touching measure ε, which the checker grants material.
        m_vertex_margin   = plan.vertex_margin();
        m_path_margin     = plan.path_margin();
        m_material_margin = std::max(0., m_path_margin - EPSILON_MM);
        m_layers.resize(plan.layer_count());
    }

    void run()
    {
        // From the highest layer that has a swept strand down to layer 1 (layer 0 rests on the bed or the raft).
        size_t top = 0;
        bool any = false;
        for (size_t l = 0; l < m_group.layer_count(); ++l) {
            for (const Strand &s : m_group.strands(l))
                if (swept(s)) { top = l; any = true; }
            for (const size_t region_id : m_group.regions())
                if (!m_plan.boundary_courses_at(l, region_id).empty()) { top = l; any = true; }
        }
        if (!any)
            return;
        for (size_t m = top; m >= 1; --m) {
            m_cancel();
            process_layer(m);
        }
    }

private:
    // Every strand is an item, the solid columns' faces included: inside a column's window a face is held by the face below
    // (rule (i)); at the window's lowest layer nothing is below it, and the column's courses end on the faces, so the faces are
    // what holds the column.
    static bool swept(const Strand &) { return true; }

    // α_m with the disc of reach shrunk by the vertex margin: the reach of a plan line along its own plane that survives the
    // export's rounding (the margin is taken off the radius w/2, where the rounding acts, not off the reach along the trace).
    double alpha(size_t m) { return alpha_of_radius(m, 0.5 * m_w - m_vertex_margin); }
    // The same for a stock path below (a bulk line), which the simplifier may move by the path margin.
    double alpha_path(size_t m) { return alpha_of_radius(m, 0.5 * m_w - m_path_margin); }

    double alpha_of_radius(size_t m, double radius)
    {
        const double s = m_plan.layer_height(m) / std::sqrt(2.);
        if (s >= 0.5 * m_w) {
            std::vector<size_t> &thick = m_plan.statistics_mutable().thick_layers;
            if (std::find(thick.begin(), thick.end(), m) == thick.end())
                thick.push_back(m);
            return 1e-6;
        }
        if (s >= radius)
            return 1e-6;
        return std::sqrt(radius * radius - s * s);
    }
    // c_N with the disc shrunk by a margin: the reach along a line from a crossing with a line of another family.
    double crossing_reach(double margin) const
    {
        return std::max(1e-6, (0.5 * m_w - margin) / SIN_60 + (m_lattice.multiline - 1) * m_lattice.spacing / (4. * SIN_60));
    }
    // What a cover piece surely holds either side of the point it was aimed at: its plane is found within τ of the point, so its
    // crossing lies up to τ / sin 60° away along the item.
    double cover_reach() const { return std::max(1e-6, crossing_reach(m_vertex_margin) - m_tau / SIN_60); }

    const LayerPrint& print_set(size_t l)
    {
        LayerPrint &lp = m_layers[l];
        if (lp.boundary != nullptr)
            return lp;
        lp.layer    = l;
        lp.z        = m_plan.print_z(l);
        lp.boundary = &m_group.boundary(l);
        lp.material_reach = std::make_unique<AreaIndex>(offset_ex(m_plan.material(l).surely_prints, float(scale_(0.5 * m_w - m_material_margin)), ClipperLib::jtRound, ARC_TOLERANCE));
        rebuild_strands(lp);
        return lp;
    }

    void rebuild_strands(LayerPrint &lp)
    {
        lp.strands_by_plane.clear();
        for (auto &family : lp.strands_of_family)
            family.clear();
        for (const Strand &s : m_group.strands(lp.layer)) {
            lp.strands_by_plane[s.plane].emplace_back(s.t0, s.t1);
            lp.strands_of_family[s.plane.family].emplace_back(s.plane, Interval(s.t0, s.t1));
        }
        for (auto &entry : lp.strands_by_plane)
            entry.second = merged(entry.second);
    }

    void add_strand(size_t l, const Strand &s)
    {
        m_group.add_strand(l, s);
        if (m_layers[l].boundary != nullptr) {
            m_layers[l].strands_by_plane[s.plane].push_back(Interval(s.t0, s.t1));
            m_layers[l].strands_by_plane[s.plane] = merged(m_layers[l].strands_by_plane[s.plane]);
            m_layers[l].strands_of_family[s.plane.family].emplace_back(s.plane, Interval(s.t0, s.t1));
        }
    }

    // The printed intervals of plane K on layer l: the bulk line inside the fill boundary (a stock path) and the strands of K
    // (plan lines), kept apart because their margins differ.
    struct PrintedOnPlane { Intervals bulk, strands; };
    PrintedOnPlane printed_on_plane(const PlaneKey &K, size_t l, double t_lo, double t_hi)
    {
        const LayerPrint &lp = print_set(l);
        PrintedOnPlane out;
        if (K.depth == 0 && !lp.boundary->index.empty()) {
            const Trace t = trace(m_lattice, K, lp.z);
            const Point a = m_lattice.scaled(t.point(t_lo)), b = m_lattice.scaled(t.point(t_hi));
            for (const Interval &i : lp.boundary->index.segment_inside(a, b))
                out.bulk.emplace_back(t_lo + i.first * (t_hi - t_lo), t_lo + i.second * (t_hi - t_lo));
        }
        auto it = lp.strands_by_plane.find(K);
        if (it != lp.strands_by_plane.end())
            out.strands = it->second;
        return out;
    }

    // Hold rule R: the parts of [t0, t1] on plane K at layer m held by the print set of m − 1.
    Intervals held(const PlaneKey &K, double t0, double t1, size_t m)
    {
        const size_t below = m - 1;
        const LayerPrint &lp = print_set(below);
        const double a = alpha(m), a_path = alpha_path(m);
        const Trace tk = trace(m_lattice, K, m_plan.print_z(m));
        Intervals out;
        // (i) own plane.
        const PrintedOnPlane on_plane = printed_on_plane(K, below, t0 - a - 1., t1 + a + 1.);
        for (const Interval &i : on_plane.bulk)
            out.emplace_back(i.first - a_path, i.second + a_path);
        for (const Interval &i : on_plane.strands)
            out.emplace_back(i.first - a, i.second + a);
        // (ii) crossings: bulk lines of the other families that print there (the path margin), and the strands of the other
        // families (the vertex margin); the reach c_N less the margin.
        const double cN_path = crossing_reach(m_path_margin), cN = crossing_reach(m_vertex_margin);
        for (int j = 0; j < 3; ++j) {
            if (j == K.family)
                continue;
            // Bulk planes (j, 0, q): the crossing parameter is linear in q.
            const Trace tj0 = trace(m_lattice, PlaneKey{ j, 0, 0 }, lp.z), tj1 = trace(m_lattice, PlaneKey{ j, 0, 1 }, lp.z);
            const auto c0 = cross_parameter(tk, tj0), c1 = cross_parameter(tk, tj1);
            if (c0 && c1 && std::abs(*c1 - *c0) > 1e-9) {
                const double step = *c1 - *c0;
                const double q_lo = std::floor((std::min(t0, t1) - cN_path - *c0) / step), q_hi = std::ceil((std::max(t0, t1) + cN_path - *c0) / step);
                const int64_t qa = int64_t(std::min(q_lo, q_hi)), qb = int64_t(std::max(q_lo, q_hi));
                if (qb - qa < 10000)
                    for (int64_t q = qa; q <= qb; ++q) {
                        const double tc = *c0 + q * step;
                        if (tc < t0 - cN_path || tc > t1 + cN_path)
                            continue;
                        if (lp.boundary->index.empty())
                            continue;
                        const Vec2d crossing_point = tk.point(tc);
                        if (!lp.boundary->index.contains(m_lattice.scaled(crossing_point)))
                            continue;
                        // The simplifier moves a path's vertices, not the interior of a straight segment: where the bulk line runs
                        // straight through the boundary a bead either side of the crossing, only the rounding margin applies.
                        const Trace tj = trace(m_lattice, PlaneKey{ j, 0, q }, lp.z);
                        const double uj = tj.parameter(crossing_point);
                        const Intervals straight = lp.boundary->index.segment_inside(m_lattice.scaled(tj.point(uj - m_w)), m_lattice.scaled(tj.point(uj + m_w)));
                        const bool interior = straight.size() == 1 && straight.front().first <= 1e-9 && straight.front().second >= 1. - 1e-9;
                        const double reach = interior ? cN : cN_path;
                        out.emplace_back(tc - reach, tc + reach);
                    }
            }
            for (const auto &[plane, interval] : lp.strands_of_family[j]) {
                const Trace tj = trace(m_lattice, plane, lp.z);
                const auto tc = cross_parameter(tk, tj);
                if (!tc || *tc < t0 - cN || *tc > t1 + cN)
                    continue;
                const double along_j = tj.parameter(tk.point(*tc));
                if (along_j < interval.first - 1e-9 || along_j > interval.second + 1e-9)
                    continue;
                // A finite piece: beyond its ends the reach along K is set by the disc about the end, not by the line. On the side
                // of K that runs toward an end e away, the reach is c when e ≥ c cos 60°, else e cos 60° + √(r² − e² sin² 60°).
                const double r = 0.5 * m_w - m_vertex_margin;
                const auto reach_toward = [&](double e) {
                    if (e >= cN * 0.5)
                        return cN;
                    const double under = r * r - e * e * SIN_60 * SIN_60;
                    return under <= 0. ? 0. : std::min(cN, 0.5 * e + std::sqrt(under));
                };
                const bool same_sense = tk.dir.dot(tj.dir) > 0.;    // +t along K moves toward interval.second along J
                const double e_plus = same_sense ? interval.second - along_j : along_j - interval.first;
                const double e_minus = same_sense ? along_j - interval.first : interval.second - along_j;
                out.emplace_back(*tc - reach_toward(e_minus), *tc + reach_toward(e_plus));
            }
        }
        // (iii) material within w/2.
        if (!lp.material_reach->empty() && t1 > t0) {
            const Point pa = m_lattice.scaled(tk.point(t0)), pb = m_lattice.scaled(tk.point(t1));
            for (const Interval &i : lp.material_reach->segment_inside(pa, pb))
                out.emplace_back(t0 + i.first * (t1 - t0), t0 + i.second * (t1 - t0));
        }
        return merged(out);
    }

    // Rule (iv), the anchors' consequence (user decision of 9 November 2026): an end of a plan line that meets another printed
    // line of its own layer (a bulk line inside the boundary or a strand of another family, within half a bead) is fused to it by
    // the filler's nub and counts as a held point. The met line is itself held or bridging.
    bool anchored_end(const PlaneKey &K, double t, size_t m)
    {
        if (!m_group.anchors())
            return false;    // the region's anchor settings make no nub: ends hold only from below
        const LayerPrint &lp = print_set(m);
        const Trace tk = trace(m_lattice, K, lp.z);
        const Vec2d p = tk.point(t);
        const double reach = 0.5 * m_w - m_vertex_margin;
        const double pitch = unscale<double>(m_lattice.pitch);
        for (int j = 0; j < 3; ++j) {
            if (j == K.family)
                continue;
            // The nearest bulk line of family j: n_j · p + σ_j z / √2 = q · L.
            const Vec2d n = m_lattice.normal(j);
            const double v = n.dot(p) + m_lattice.sigma[j] * lp.z / std::sqrt(2.);
            const double off = v - std::round(v / pitch) * pitch;
            if (std::abs(off) <= reach && !lp.boundary->index.empty() && lp.boundary->index.contains(m_lattice.scaled(p - off * n)))
                return true;
            for (const auto &[plane, interval] : lp.strands_of_family[j]) {
                const Trace tj = trace(m_lattice, plane, lp.z);
                if (std::abs(n.dot(p - tj.origin)) > reach)
                    continue;
                // In the interior of the met strand: a nub needs the met bead on both sides of the junction.
                const double along = tj.parameter(p);
                if (along >= interval.first + 0.5 * m_w - 1e-6 && along <= interval.second - 0.5 * m_w + 1e-6)
                    return true;
            }
        }
        return false;
    }

    bool held_whole(const PlaneKey &K, double t0, double t1, size_t m)
    {
        const Intervals h = held(K, t0, t1, m);
        return complement(h, t0, t1).empty();
    }

    // Whether a piece [y − λ/2, y + λ/2] on K lies whole inside the fill boundary of layer l.
    bool inside_boundary(const PlaneKey &K, double y, size_t l, double half_length)
    {
        const LayerPrint &lp = print_set(l);
        if (lp.boundary->index.empty())
            return false;
        const Trace t = trace(m_lattice, K, lp.z);
        const Point a = m_lattice.scaled(t.point(y - half_length)), b = m_lattice.scaled(t.point(y + half_length));
        const Intervals in = lp.boundary->index.segment_inside(a, b);
        return in.size() == 1 && in.front().first <= 1e-9 && in.front().second >= 1. - 1e-9;
    }

    // The nearest position to y within [y − reach, y + reach] whose piece lies inside the boundary of layer l, or nothing.
    std::optional<double> nearest_inside(const PlaneKey &K, double y, size_t l, double reach, double half_length)
    {
        if (inside_boundary(K, y, l, half_length))
            return y;
        const LayerPrint &lp = print_set(l);
        if (lp.boundary->index.empty())
            return std::nullopt;
        const Trace t = trace(m_lattice, K, lp.z);
        const double lo = y - reach - half_length, hi = y + reach + half_length;
        const Point a = m_lattice.scaled(t.point(lo)), b = m_lattice.scaled(t.point(hi));
        std::optional<double> best;
        for (const Interval &i : lp.boundary->index.segment_inside(a, b)) {
            const double il = lo + i.first * (hi - lo), ih = lo + i.second * (hi - lo);
            if (ih - il < 2. * half_length - 1e-9)
                continue;
            const double candidate = std::min(std::max(y, il + half_length), ih - half_length);
            if (std::abs(candidate - y) <= reach + 1e-9 && (!best || std::abs(candidate - y) < std::abs(*best - y)))
                best = candidate;
        }
        return best;
    }

    // A piece [y − λ/2, y + λ/2] on K at layer l is admissible when it lies whole inside the fill boundary at l and a chain of
    // pieces within α of each other continues through the boundaries of the three layers below it, or lands sooner.
    bool admissible(const PlaneKey &K, double y, size_t l, double half_length)
    {
        if (!inside_boundary(K, y, l, half_length))
            return false;
        double pos = y;
        for (int step = 1; step <= LOOK_AHEAD; ++step) {
            if (l < size_t(step))
                return true;    // the bed
            const size_t layer = l - step + 1;      // the piece at `layer` is held by the print set of layer − 1
            if (held_whole(K, pos - half_length, pos + half_length, layer))
                return true;    // lands
            const std::optional<double> next = nearest_inside(K, pos, layer - 1, alpha(layer), half_length);
            if (!next)
                return false;
            pos = *next;
        }
        return true;
    }

    // Recorded in every build (a few places per object at most); only the warnings of section 5.2 depend on the development flag.
    void record_unheld(size_t layer, const PlaneKey &K, double t, const char *reason)
    {
        const Trace tr = trace(m_lattice, K, m_plan.print_z(layer));
        const Point abs = m_lattice.scaled(tr.point(t));
        m_plan.statistics_mutable().unheld.push_back(Place{ layer, Vec2d(unscale<double>(abs.x()), unscale<double>(abs.y())), reason });
    }

    // An item of layer m: a merged interval of swept strands on one plane.
    struct Item { PlaneKey plane; double t0, t1, gap; bool column; };

    std::vector<Item> items_of(size_t m)
    {
        std::map<PlaneKey, std::vector<const Strand*>> by_plane;
        for (const Strand &s : m_group.strands(m))
            if (swept(s))
                by_plane[s.plane].push_back(&s);
        std::vector<Item> out;
        for (auto &[plane, strands] : by_plane) {
            std::sort(strands.begin(), strands.end(), [](const Strand *a, const Strand *b) { return a->t0 < b->t0; });
            for (const Strand *s : strands) {
                if (!out.empty() && out.back().plane == plane && s->t0 <= out.back().t1 + m_lambda && out.back().column == (s->kind == StrandKind::Column || s->kind == StrandKind::Cover)) {
                    out.back().t1  = std::max(out.back().t1, s->t1);
                    out.back().gap = std::min(out.back().gap, s->gap);
                } else
                    out.push_back(Item{ plane, s->t0, s->t1, s->gap, s->kind == StrandKind::Column || s->kind == StrandKind::Cover });
            }
        }
        return out;
    }

    void process_layer(size_t m)
    {
        for (Item item : items_of(m)) {
            if (item.column)
                continue_column(item, m);
            else
                hold_strand(item, m);
        }
        hold_course_ends(m);
    }

    // The ends of the courses of layer m (seam walls and rails, 3.5; a solid column's courses end on its faces): an end that the
    // print set of m − 1 does not hold gets a holder, a column piece of some family through the end point at m − 1. A course's
    // length is held by the demand its bead raises on the plane beneath it (3.4).
    void hold_course_ends(size_t m)
    {
        const size_t below = m - 1;
        for (const size_t region_id : m_group.regions()) {
            const Polylines &courses = m_plan.boundary_courses_at(m, region_id);
            if (courses.empty())
                continue;
            const double gap = m_plan.gap_of_region(region_id);
            for (const Polyline &course : courses) {
                if (course.points.size() < 2 || course.first_point() == course.last_point())
                    continue;    // a closed course has no end
                for (const Point &end : { course.first_point(), course.last_point() }) {
                    const Vec2d p = m_lattice.relative(end);
                    if (point_held(p, m))
                        continue;
                    bool placed = false;
                    for (int j = 0; j < 3 && !placed; ++j) {
                        const std::optional<PlaneKey> plane = plane_through(m_lattice, j, p, m_plan.print_z(below), m_tau);
                        if (!plane)
                            continue;
                        const double y = trace(m_lattice, *plane, m_plan.print_z(below)).parameter(p), half = 0.5 * m_lambda;
                        // Centred on the end, or shifted along the trace as far as the piece still covers the end's projection.
                        std::optional<double> centre;
                        if (admissible(*plane, y, below, half))
                            centre = y;
                        else if (const std::optional<double> shifted = nearest_inside(*plane, y, below, half - m_vertex_margin, half); shifted && admissible(*plane, *shifted, below, half))
                            centre = shifted;
                        if (!centre)
                            continue;
                        add_strand(below, Strand{ *plane, *centre - half, *centre + half, StrandKind::Column, gap });
                        placed = true;
                    }
                    if (!placed)
                        m_plan.statistics_mutable().unheld.push_back(Place{ m, Vec2d(unscale<double>(end.x()), unscale<double>(end.y())), "no admissible piece under a course's end" });
                }
            }
        }
    }

    // Whether a point of layer m (mm, relative to the lattice centre) lies over a bead of the print set of m − 1: the material
    // within w/2, a bulk line inside the boundary, or a strand, each less its margin.
    bool point_held(const Vec2d &p, size_t m)
    {
        const LayerPrint &lp = print_set(m - 1);
        if (!lp.material_reach->empty() && lp.material_reach->contains(m_lattice.scaled(p)))
            return true;
        const double pitch = unscale<double>(m_lattice.pitch);
        const double band  = 0.5 * (m_lattice.multiline - 1) * m_lattice.spacing;
        for (int j = 0; j < 3; ++j) {
            const Vec2d  n   = m_lattice.normal(j);
            const double v   = n.dot(p) + m_lattice.sigma[j] * lp.z / std::sqrt(2.);
            const double off = v - std::round(v / pitch) * pitch;
            if (std::abs(off) <= band + 0.5 * m_w - m_path_margin && !lp.boundary->index.empty() && lp.boundary->index.contains(m_lattice.scaled(p - off * n)))
                return true;
            for (const auto &[plane, interval] : lp.strands_of_family[j]) {
                const Trace tj = trace(m_lattice, plane, lp.z);
                if (std::abs(n.dot(p - tj.origin)) > band + 0.5 * m_w - m_vertex_margin)
                    continue;
                const double along = tj.parameter(p);
                if (along >= interval.first - 1e-9 && along <= interval.second + 1e-9)
                    return true;
            }
        }
        return false;
    }

    // Steps 1 to 4 for a needed strand (with its extensions) at layer m.
    void hold_strand(Item item, size_t m)
    {
        const double gb = item.gap - MU;
        const LayerPrint &lp_m = print_set(m);
        for (int round = 0; round < 4; ++round) {
            Intervals h = held(item.plane, item.t0, item.t1, m);
            Intervals gaps = complement(h, item.t0, item.t1);
            if (gaps.empty())
                return;
            // The extension of a free end (step 3): along the trace inside the boundary at m to the first held point within G_b,
            // when the whole run from the last held point to it is at most G_b. Returns the point reached.
            const auto try_extend = [&](bool forward, double end, double last_held) -> std::optional<double> {
                if (lp_m.boundary->index.empty())
                    return std::nullopt;
                const Trace t = trace(m_lattice, item.plane, lp_m.z);
                const double far = forward ? end + gb : end - gb;
                const Point a = m_lattice.scaled(t.point(std::min(end, far))), b = m_lattice.scaled(t.point(std::max(end, far)));
                for (const Interval &i : lp_m.boundary->index.segment_inside(a, b)) {
                    const double il = std::min(end, far) + i.first * gb, ih = std::min(end, far) + i.second * gb;
                    if (il > end + 1e-6 || ih < end - 1e-6)
                        continue;    // only the boundary interval that continues the strand's end
                    const double lo = forward ? end : il, hi = forward ? ih : end;
                    if (hi - lo < 1e-9)
                        return std::nullopt;
                    // The first held point beyond the end (the rules return every interval of the plane below, some behind it).
                    std::optional<double> first_held;
                    for (const Interval &h2 : held(item.plane, lo, hi, m)) {
                        if (forward && h2.second >= end + 1e-9) {
                            const double point = std::max(h2.first, end);
                            if (!first_held || point < *first_held) first_held = point;
                        } else if (!forward && h2.first <= end - 1e-9) {
                            const double point = std::min(h2.second, end);
                            if (!first_held || point > *first_held) first_held = point;
                        }
                    }
                    if (first_held && std::abs(*first_held - last_held) <= gb && std::abs(*first_held - end) <= gb && std::abs(*first_held - end) > 1e-9)
                        return first_held;
                    return std::nullopt;
                }
                return std::nullopt;
            };
            bool extended = false;
            for (const Interval &gap : gaps) {
                // A gap reaches an end of the item when that end is not a held point (an end that an extension reached is held).
                const bool at_start = gap.first <= item.t0 + 1e-9 && !covers(h, item.t0);
                const bool at_end   = gap.second >= item.t1 - 1e-9 && !covers(h, item.t1);
                // An end fused to another line of the layer is a held point too (rule iv).
                const bool lo_held = !at_start || anchored_end(item.plane, item.t0, m);
                const bool hi_held = !at_end || anchored_end(item.plane, item.t1, m);
                if (lo_held && hi_held) {
                    if (gap.second - gap.first <= gb)
                        continue;    // a bridge between held points
                    place_holders(item, gap.first, gap.second, m, true, true);
                    continue;
                }
                // A free end: an extension first; the item grows and the gaps are recomputed.
                if (!lo_held)
                    if (const std::optional<double> reach = try_extend(false, item.t0, gap.second)) {
                        add_strand(m, Strand{ item.plane, *reach, item.t0, StrandKind::Extension, item.gap });
                        item.t0 = *reach;
                        extended = true;
                        break;
                    }
                if (!hi_held)
                    if (const std::optional<double> reach = try_extend(true, item.t1, gap.first)) {
                        add_strand(m, Strand{ item.plane, item.t1, *reach, StrandKind::Extension, item.gap });
                        item.t1 = *reach;
                        extended = true;
                        break;
                    }
                // Holders from each free end inward.
                place_holders(item, gap.first, gap.second, m, lo_held, hi_held);
            }
            if (!extended)
                return;
        }
    }

    // Column pieces of length λ on the item's plane at m − 1 under the gap [g0, g1], greedily from each free end inward so that
    // every stretch between held points is at most G_b. `held_lo` / `held_hi` say whether the gap's ends are held points.
    void place_holders(const Item &item, double g0, double g1, size_t m, bool held_lo, bool held_hi)
    {
        const size_t below = m - 1;
        const double gb = item.gap - MU, a = alpha(m), half = 0.5 * m_lambda;
        const double reach = half + a;             // a piece at y holds [y − λ/2 − α, y + λ/2 + α]
        // Coverage advances from the low end. A held low end covers up to g0; a free end needs a holder near it.
        double covered = held_lo ? g0 : -std::numeric_limits<double>::infinity();
        if (!held_lo) {
            std::optional<double> y = first_holder(item.plane, g0, below, +1, half, a);
            if (!y) {
                if (!cover(item, g0, m))
                    record_unheld(m, item.plane, g0, item.column ? "no admissible piece at a column's free end" : "no admissible piece at a strand's free end");
                covered = g0;
            } else {
                add_strand(below, Strand{ item.plane, *y - half, *y + half, StrandKind::Column, item.gap });
                covered = *y + reach;
            }
        }
        int guard = 0;
        while (covered < g1 - 1e-9 && ++guard < 10000) {
            const double remaining = g1 - covered;
            if (remaining <= gb) {
                if (held_hi)
                    break;    // the last stretch is a bridge to the held end
                // The free high end: a holder as near the end as an admissible position allows; the stretch before it is a bridge.
                std::optional<double> y = first_holder(item.plane, g1, below, -1, half, a);
                if (!y) {
                    if (!cover(item, g1, m))
                        record_unheld(m, item.plane, g1, item.column ? "no admissible piece at a column's free end" : "no admissible piece at a strand's free end");
                } else
                    add_strand(below, Strand{ item.plane, *y - half, *y + half, StrandKind::Column, item.gap });
                break;
            }
            // The next holder as far along as keeps the stretch within G_b (its reach must start by covered + G_b), under the item.
            const double farthest = std::min(covered + gb + reach, g1);
            std::optional<double> y;
            for (double candidate = farthest; candidate > covered - reach + 1e-6; candidate -= 0.5 * a) {
                if (admissible(item.plane, candidate, below, half)) { y = candidate; break; }
                if (a <= 1e-6) break;
            }
            if (!y) {
                const double point = std::min(covered + gb, g1);
                if (!cover(item, point, m))
                    record_unheld(m, item.plane, point, "no admissible piece in the window");
                covered = point + cover_reach();    // a cover holds c either side of its crossing, less the plane search drift
                continue;
            }
            add_strand(below, Strand{ item.plane, *y - half, *y + half, StrandKind::Column, item.gap });
            covered = *y + reach;
        }
    }

    // The first holder near a free end at `end`: as near the end as an admissible position allows within λ/2 + α of it, searched
    // inward (direction +1 from a low end, −1 from a high end) first, then beyond the end.
    std::optional<double> first_holder(const PlaneKey &K, double end, size_t below, int direction, double half, double a)
    {
        const double reach = half + a;
        const double step = std::max(0.25 * a, 1e-3);
        for (double d = 0.; d <= reach + 1e-9; d += step) {
            const double y = end + direction * d;
            if (admissible(K, y, below, half))
                return y;
        }
        for (double d = step; d <= reach + 1e-9; d += step) {
            const double y = end - direction * d;
            if (admissible(K, y, below, half))
                return y;
        }
        return std::nullopt;
    }

    // A cover piece of another family at m − 1 through the point of the item at parameter t: on the plane of that family through
    // the point (levels 1 to 16, within τ), centred on its crossing with the item's trace; holds the item there by rule (ii).
    bool cover(const Item &item, double t, size_t m)
    {
        const size_t below = m - 1;
        const Trace tk = trace(m_lattice, item.plane, m_plan.print_z(m));
        const Vec2d p = tk.point(t);
        const double half = 0.5 * m_lambda + (m_lattice.multiline - 1) * m_lattice.spacing / (2. * SIN_60);
        for (int j = 0; j < 3; ++j) {
            if (j == item.plane.family)
                continue;
            const std::optional<PlaneKey> plane = plane_through(m_lattice, j, p, m_plan.print_z(below), m_tau);
            if (!plane)
                continue;
            const Trace tj = trace(m_lattice, *plane, m_plan.print_z(below));
            const auto tc = cross_parameter(tk, tj);
            if (!tc)
                continue;
            const double y = tj.parameter(tk.point(*tc));
            if (!admissible(*plane, y, below, half))
                continue;
            add_strand(below, Strand{ *plane, y - half, y + half, StrandKind::Cover, item.gap });
            return true;
        }
        return false;
    }

    // Step 5: a column piece (or a joined interval) at layer m lands, merges or continues at m − 1.
    void continue_column(const Item &item, size_t m)
    {
        if (m == 0)
            return;
        const size_t below = m - 1;
        const double a = alpha(m), half = 0.5 * m_lambda;
        const Intervals h = held(item.plane, item.t0, item.t1, m);
        if (complement(h, item.t0, item.t1).empty())
            return;    // lands, or rests on a planned interval of its own plane
        const double length = item.t1 - item.t0;
        const double y = 0.5 * (item.t0 + item.t1);
        if (length > m_lambda + 1e-9) {
            // A joined interval continues narrowed by α at each end, until it is shorter than a piece.
            const double lo = item.t0 + a, hi = item.t1 - a;
            if (hi - lo >= m_lambda) {
                if (inside_boundary_interval(item.plane, lo, hi, below)) {
                    add_strand(below, Strand{ item.plane, lo, hi, StrandKind::Column, item.gap });
                    return;
                }
                // The narrowed interval does not fit inside the boundary: covers along the item, or the places are recorded.
                const double c = cover_reach();
                bool all = true;
                for (double t = item.t0 + c; ; t += 2. * c) {
                    const double point = std::min(t, item.t1 - c);
                    if (!cover(item, point, m))
                        all = false;
                    if (point >= item.t1 - c - 1e-9)
                        break;
                }
                if (!all)
                    record_unheld(m, item.plane, y, "no room for the joined interval");
                return;
            }
        }
        // A single piece at y′ holds [y′ − λ/2 − α, y′ + λ/2 + α]: for an item of this length it may shift from the centre by at most
        // λ/2 + α − length/2 (zero slack when the item is as long as a piece plus two reaches).
        const double slack = std::max(0., half + a - 0.5 * length);
        // Merge: a planned interval of the plane at m − 1 within λ joins the column. The interval holds what it reaches; only the
        // part of the piece's footprint it does not cover is added, as a piece of its own, and the column goes on from there.
        {
            const LayerPrint &lp = print_set(below);
            auto it = lp.strands_by_plane.find(item.plane);
            if (it != lp.strands_by_plane.end())
                for (const Interval &i : it->second)
                    if (i.first <= y + m_lambda && i.second >= y - m_lambda) {
                        const double need_lo = item.t0 + a, need_hi = item.t1 - a;    // what must print below to hold the whole item
                        Intervals uncovered = complement(Intervals{ i }, need_lo, need_hi);
                        bool placed = true;
                        for (const Interval &u : uncovered) {
                            // A piece of λ at the uncovered part, abutting the interval, inside the boundary.
                            double lo = u.first <= need_lo + 1e-9 ? u.second - m_lambda : u.first;
                            double hi = lo + m_lambda;
                            if (inside_boundary_interval(item.plane, lo, hi, below))
                                add_strand(below, Strand{ item.plane, lo, hi, StrandKind::Column, item.gap });
                            else
                                placed = false;
                        }
                        if (placed)
                            return;
                    }
        }
        // Continue: a new piece within α along the trace, steered onto the quickest landing. A piece lands only when a crossing
        // covers it whole (c exceeds λ/2 by a few hundredths), so the next piece is aimed exactly at where the crossing will be
        // one layer further down, as far as α allows; a fixed menu of shifts missed that window and the columns never landed.
        const double target = steering_target(item.plane, y, m);
        const double step = std::min(a, slack);
        const double move = std::max(-step, std::min(step, target - y));
        // The exact move first, then the positions within the window ordered by their distance to the target (a target already
        // reached, or none in sight, must still leave the admissible shifts in play).
        std::vector<double> preferences = { move, step, -step, 0.5 * step, -0.5 * step, 0. };
        std::stable_sort(preferences.begin(), preferences.end(), [&](double p, double q) { return std::abs(y + p - target) < std::abs(y + q - target); });
        preferences.erase(std::unique(preferences.begin(), preferences.end(), [](double p, double q) { return std::abs(p - q) < 1e-9; }), preferences.end());
        for (const double shift : preferences) {
            const double candidate = y + shift;
            if (admissible(item.plane, candidate, below, half)) {
                add_strand(below, Strand{ item.plane, candidate - half, candidate + half, StrandKind::Column, item.gap });
                return;
            }
        }
        // Trapped (its trace runs into the boundary, typically at a wall): the column changes family through cover pieces of
        // another family, each holding the item within the crossing reach c of its crossing (step 6's trapped case, among the
        // three lattice directions only); an item longer than 2c takes several, c apart from each end.
        {
            const double c = cover_reach();
            bool all = true;
            if (length <= 2. * c - 1e-9)
                all = cover(item, y, m);
            else
                for (double t = item.t0 + c; ; t += 2. * c) {
                    const double point = std::min(t, item.t1 - c);
                    if (!cover(item, point, m))
                        all = false;
                    if (point >= item.t1 - c - 1e-9)
                        break;
                }
            if (all)
                return;
        }
        record_unheld(m, item.plane, y, "no admissible piece for the column");
    }

    bool inside_boundary_interval(const PlaneKey &K, double lo, double hi, size_t l)
    {
        return inside_boundary(K, 0.5 * (lo + hi), l, 0.5 * (hi - lo));
    }

    // The centre of the next piece (at m − 1) that lands soonest: on a crossing with a bulk line of another family where that
    // crossing will be at m − 2 (so that the piece lies whole within the crossing's reach c there), inside the material reach along
    // the trace within 20 mm, or the current centre when nothing is in sight. The caller moves toward it by at most α a layer.
    double steering_target(const PlaneKey &K, double y, size_t m)
    {
        const size_t below = m - 1;
        const double a = alpha(m), h = m_plan.layer_height(m);
        const LayerPrint &lp = print_set(below);
        // The piece we place lies on layer m − 1 and is held by the print set of m − 2: its crossings are those of K's trace at
        // m − 1 with the bulk traces at m − 2 (as rule (ii) measures them for a piece and the layer below it), and the boundary
        // and material that hold it are those of m − 2.
        const LayerPrint &lp2 = below >= 1 ? print_set(below - 1) : lp;
        const double z_k = lp.z, z_j = below >= 1 ? m_plan.print_z(below - 1) : lp.z - h;
        const Trace tk = trace(m_lattice, K, z_k);
        double best_k = double(m);     // layers to the bed
        double target = y;
        for (int j = 0; j < 3; ++j) {
            if (j == K.family)
                continue;
            const Trace tj0 = trace(m_lattice, PlaneKey{ j, 0, 0 }, z_j), tj1 = trace(m_lattice, PlaneKey{ j, 0, 1 }, z_j);
            const auto c0 = cross_parameter(tk, tj0), c1 = cross_parameter(tk, tj1);
            if (!c0 || !c1 || std::abs(*c1 - *c0) < 1e-9)
                continue;
            const double step = *c1 - *c0;
            // The crossings' drift per layer going down, for the count of layers to the landing.
            const auto drift = crossing(m_lattice, K, PlaneKey{ j, 0, 0 }, z_k);
            const double v = drift ? -drift->dt_dz * h : 0.;
            for (int side = -1; side <= 1; side += 2) {
                const double q = side > 0 ? std::ceil((y - *c0) / step) : std::floor((y - *c0) / step);
                const double tc = *c0 + q * step;
                const double g = std::abs(tc - y);
                const bool approaching = (tc - y) * v < 0.;
                if (g > a && !approaching && std::abs(v) >= a)
                    continue;    // it runs away faster than a column can follow
                if (!lp2.boundary->index.empty() && !lp2.boundary->index.contains(m_lattice.scaled(tk.point(tc))))
                    continue;
                const double closing = approaching ? std::abs(v) + a : std::max(1e-6, a - std::abs(v));
                const double k = g <= a ? 1. : 1. + std::ceil((g - a) / closing);
                if (k < best_k) { best_k = k; target = tc; }
            }
        }
        if (!lp2.material_reach->empty()) {
            const Point pa = m_lattice.scaled(tk.point(y - LOOK_ALONG)), pb = m_lattice.scaled(tk.point(y + LOOK_ALONG));
            for (const Interval &i : lp2.material_reach->segment_inside(pa, pb)) {
                const double lo = y - LOOK_ALONG + i.first * 2. * LOOK_ALONG, hi = y - LOOK_ALONG + i.second * 2. * LOOK_ALONG;
                if (hi - lo < m_lambda)
                    continue;
                const double inside = std::min(std::max(y, lo + 0.5 * m_lambda), hi - 0.5 * m_lambda);
                const double g = std::abs(inside - y);
                const double k = g <= a ? 1. : 1. + std::ceil((g - a) / a);
                if (k < best_k) { best_k = k; target = inside; }
            }
        }
        return target;
    }

    GroupPlan                   &m_group;
    ObjectPlan                  &m_plan;
    const std::function<void()> &m_cancel;
    const LatticeParams         &m_lattice;
    std::vector<LayerPrint>      m_layers;
    double m_w = 0., m_c = 0., m_cN = 0., m_lambda = 0., m_tau = 0.;
    double m_vertex_margin = 0., m_path_margin = 0., m_material_margin = 0.;
};

} // namespace

void sweep(GroupPlan &group, ObjectPlan &plan, const std::function<void()> &cancel)
{
    Sweep s(group, plan, cancel);
    s.run();
}

} // namespace BridgeAwareCubic
} // namespace Slic3r
