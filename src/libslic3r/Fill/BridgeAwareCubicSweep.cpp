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
constexpr double K_MIN         = 6.;       // step 6: a column that would stay this many layers or more evaluates a family change
constexpr double LOOK_ALONG    = 20.;      // mm along its trace in which a column looks for material
constexpr int    PLANE_LEVELS  = 16;       // the plane through a point: levels 1 to 16 (plane_through)
constexpr double SIN_60        = 0.86602540378443864676;
constexpr float  ARC_TOLERANCE = float(scale_(0.001));

using Interval  = std::pair<double, double>;
using Intervals = std::vector<Interval>;

// The intersection of two sorted, merged interval lists.
Intervals intersected(const Intervals &a, const Intervals &b)
{
    Intervals out;
    size_t i = 0, j = 0;
    while (i < a.size() && j < b.size()) {
        const double lo = std::max(a[i].first, b[j].first), hi = std::min(a[i].second, b[j].second);
        if (lo < hi)
            out.emplace_back(lo, hi);
        if (a[i].second < b[j].second) ++i; else ++j;
    }
    return out;
}

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
        m_offsets = m_lattice.subline_offsets();
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

    // The stretch of the line a + t u (u a unit vector) inside the capsule of radius r about the segment b0 b1: the discs about its
    // ends and the band along it, whose union is that one stretch (the capsule is convex).
    static std::optional<Interval> capsule_stretch(const Vec2d &a, const Vec2d &u, const Vec2d &b0, const Vec2d &b1, double r)
    {
        std::optional<Interval> out;
        const auto add = [&out](double lo, double hi) {
            if (lo <= hi)
                out = out ? Interval{ std::min(out->first, lo), std::max(out->second, hi) } : Interval{ lo, hi };
        };
        for (const Vec2d &b : { b0, b1 }) {
            const double tb = u.dot(b - a), d2 = (b - (a + tb * u)).squaredNorm();
            if (d2 <= r * r)
                add(tb - std::sqrt(r * r - d2), tb + std::sqrt(r * r - d2));
        }
        const double length = (b1 - b0).norm();
        if (length > 1e-12) {
            const Vec2d along = (b1 - b0) / length, across(-along.y(), along.x());
            double lo = -std::numeric_limits<double>::infinity(), hi = std::numeric_limits<double>::infinity();
            // v(t) = v0 + v1 t kept within [v_lo, v_hi].
            const auto keep = [&lo, &hi](double v0, double v1, double v_lo, double v_hi) {
                if (std::abs(v1) < 1e-12) {
                    if (v0 < v_lo || v0 > v_hi)
                        hi = -std::numeric_limits<double>::infinity();
                    return;
                }
                const double ta = (v_lo - v0) / v1, tb = (v_hi - v0) / v1;
                lo = std::max(lo, std::min(ta, tb));
                hi = std::min(hi, std::max(ta, tb));
            };
            keep(across.dot(a - b0), across.dot(u), -r, r);
            keep(along.dot(a - b0), along.dot(u), 0., length);
            add(lo, hi);
        }
        return out;
    }

    // Rule (ii) for a band crossing a finite piece of a band of family j (3.9b): held where each of its sub-lines lies within r of
    // one of the piece's sub-lines. For whole lines this is the reach c_N; near a piece's end the sub-lines part ways, and the
    // reach about the bands' centre lines would claim a sub-line the piece does not reach.
    Intervals band_hold(const Trace &tk, int k, const Trace &tj, int j, const Interval &piece, double r) const
    {
        const std::vector<double> &offsets = m_offsets;
        const Vec2d nk = m_lattice.normal(k), nj = m_lattice.normal(j);
        Intervals all;
        bool first = true;
        for (const double oi : offsets) {
            Intervals any;
            for (const double oj : offsets)
                if (const std::optional<Interval> in = capsule_stretch(tk.origin + oi * nk, tk.dir, tj.point(piece.first) + oj * nj, tj.point(piece.second) + oj * nj, r))
                    any.push_back(*in);
            all = first ? merged(std::move(any)) : intersected(all, merged(std::move(any)));
            first = false;
            if (all.empty())
                break;
        }
        return all;
    }

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
                        const bool interior = lp.boundary->index.segment_inside_whole(m_lattice.scaled(tj.point(uj - m_w)), m_lattice.scaled(tj.point(uj + m_w)));
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
                if (m_lattice.multiline > 1) {
                    append(out, band_hold(tk, K.family, tj, j, interval, r));
                    continue;
                }
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
        // (iii) material within w/2: of every sub-line of the band (3.9b; one sub-line at N = 1).
        if (!lp.material_reach->empty() && t1 > t0) {
            const Vec2d normal = m_lattice.normal(K.family);
            Intervals material;
            bool first = true;
            for (const double o : m_offsets) {
                const Point pa = m_lattice.scaled(tk.point(t0) + o * normal), pb = m_lattice.scaled(tk.point(t1) + o * normal);
                Intervals one;
                for (const Interval &i : lp.material_reach->segment_inside(pa, pb))
                    one.emplace_back(t0 + i.first * (t1 - t0), t0 + i.second * (t1 - t0));
                one = merged(one);
                material = first ? one : intersected(material, one);
                first = false;
            }
            append(out, material);
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
        const std::vector<double> &offsets = m_offsets;
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
                // Every sub-line end of the band on a sub-line of the met strand (3.9b; at N = 1 the line's end on the strand's
                // line), in the interior of the met strand: a nub needs the met bead on both sides of the junction.
                bool fused = true;
                for (const double oi : offsets) {
                    const Vec2d pi = p + oi * m_lattice.normal(K.family);
                    const double across = n.dot(pi - tj.origin);
                    const double along = tj.parameter(pi);
                    if (std::none_of(offsets.begin(), offsets.end(), [&](double oj) { return std::abs(across - oj) <= reach; }) ||
                        along < interval.first + 0.5 * m_w - 1e-6 || along > interval.second - 0.5 * m_w + 1e-6) {
                        fused = false;
                        break;
                    }
                }
                if (fused)
                    return true;
            }
        }
        // A boundary course of the layer (a seam wall, a rail): the line ends inside its bead (3.8) and the filler's nub fuses it,
        // in the course's interior (half a bead from its ends; a closed course has none); every sub-line end of the band.
        const double reach_scaled = scale_(reach), end_margin = scale_(0.5 * m_w);
        const auto on_course = [&](const Point &P) {
            for (const size_t region_id : m_group.regions())
                for (const Polyline &course : m_plan.boundary_courses_at(m, region_id)) {
                    if (course.points.size() < 2)
                        continue;
                    const bool   course_closed = course.first_point() == course.last_point();
                    const double total = double(course.length());
                    double arc = 0.;
                    for (size_t i = 1; i < course.points.size(); ++i) {
                        const Line   seg(course.points[i - 1], course.points[i]);
                        const double seg_length = seg.length();
                        if (seg.distance_to(P) <= reach_scaled) {
                            const Vec2d  a = seg.a.cast<double>(), d = (seg.b - seg.a).cast<double>();
                            const double t    = seg_length > 0. ? std::min(1., std::max(0., (P.cast<double>() - a).dot(d) / (seg_length * seg_length))) : 0.;
                            const double foot = arc + t * seg_length;
                            if (course_closed || (foot >= end_margin && total - foot >= end_margin))
                                return true;
                        }
                        arc += seg_length;
                    }
                }
            return false;
        };
        return std::all_of(offsets.begin(), offsets.end(), [&](double o) { return on_course(m_lattice.scaled(p + o * m_lattice.normal(K.family))); });
    }

    bool held_whole(const PlaneKey &K, double t0, double t1, size_t m)
    {
        const Intervals h = held(K, t0, t1, m);
        return complement(h, t0, t1).empty();
    }

    // Whether a piece [y − λ/2, y + λ/2] on K lies whole inside the fill boundary of layer l.
    // Every sub-line of the band (3.9b; the one line at N = 1), since the filler clips each on its own.
    bool inside_boundary(const PlaneKey &K, double y, size_t l, double half_length)
    {
        const LayerPrint &lp = print_set(l);
        if (lp.boundary->index.empty())
            return false;
        const Trace t = trace(m_lattice, K, lp.z);
        const Vec2d normal = m_lattice.normal(K.family);
        for (const double o : m_offsets)
            if (!lp.boundary->index.segment_inside_whole(m_lattice.scaled(t.point(y - half_length) + o * normal), m_lattice.scaled(t.point(y + half_length) + o * normal)))
                return false;
        return true;
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
        // Where every sub-line lies inside the boundary.
        const Vec2d normal = m_lattice.normal(K.family);
        Intervals inside;
        bool first = true;
        for (const double o : m_offsets) {
            const Point a = m_lattice.scaled(t.point(lo) + o * normal), b = m_lattice.scaled(t.point(hi) + o * normal);
            Intervals one;
            for (const Interval &i : lp.boundary->index.segment_inside(a, b))
                one.emplace_back(lo + i.first * (hi - lo), lo + i.second * (hi - lo));
            inside = first ? std::move(one) : intersected(inside, one);
            first = false;
        }
        std::optional<double> best;
        for (const Interval &i : inside) {
            const double il = i.first, ih = i.second;
            if (ih - il < 2. * half_length - 1e-9)
                continue;
            const double candidate = std::min(std::max(y, il + half_length), ih - half_length);
            if (std::abs(candidate - y) <= reach + 1e-9 && (!best || std::abs(candidate - y) < std::abs(*best - y)))
                best = candidate;
        }
        return best;
    }

    // A piece [y − λ/2, y + λ/2] on K at layer l is admissible when it lies whole inside the fill boundary at l and a chain of
    // pieces within α of each other continues through the boundaries of the three layers below it (`look_ahead` layers), or
    // lands sooner.
    bool admissible(const PlaneKey &K, double y, size_t l, double half_length, int look_ahead = LOOK_AHEAD)
    {
        if (!inside_boundary(K, y, l, half_length))
            return false;
        double pos = y;
        for (int step = 1; step <= look_ahead; ++step) {
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
                if (course.points.size() < 2)
                    continue;
                const bool closed = course.first_point() == course.last_point();
                // The points to hold: both ends of an open course, and along every course the points where a stretch over the
                // void would otherwise exceed G_b (the roof measure bounds the openings under a course within G, which leaves its
                // bead's own stretches between held points up to G plus the band's width; rule R asks for G_b).
                std::vector<Point> to_hold;
                if (!closed) {
                    to_hold.push_back(course.first_point());
                    to_hold.push_back(course.last_point());
                }
                {
                    const double gb = gap - MU, step_mm = 0.5 * m_lambda;
                    const double length = unscale<double>(course.length());
                    // Stations every half bead; a closed course's last station stops short of its start point, which is station 0.
                    std::vector<double> stations;
                    for (double st = 0.; st < length - (closed ? 0.5 * step_mm : -1e-9); st += step_mm)
                        stations.push_back(std::min(st, length));
                    if (!closed && (stations.empty() || stations.back() < length - 1e-9))
                        stations.push_back(length);
                    const size_t count = stations.size();
                    if (count > 0) {
                        std::vector<char> held_at(count, 0);
                        for (size_t i = 0; i < count; ++i)
                            held_at[i] = point_held(m_lattice.relative(point_at_arc_length(course, stations[i])), m) ? 1 : 0;
                        // A stretch over the void runs from the last held station before it to the first held station after it
                        // (the transitions lie between stations, so a stretch is never undercounted); an open course's ends bound
                        // it; a closed course is walked from a held station round to itself, so that a stretch through its start
                        // point is one stretch, and with no held station it is one stretch of its whole length.
                        size_t start = 0;
                        if (closed) {
                            const auto first_held = std::find(held_at.begin(), held_at.end(), char(1));
                            start = first_held == held_at.end() ? 0 : size_t(first_held - held_at.begin());
                        }
                        const auto place_along = [&](double from, double to) {
                            // Holders every G_b at most over a stretch longer than G_b (arc positions past the length wrap round).
                            if (to - from <= gb)
                                return;
                            const int n = int(std::ceil((to - from) / gb));
                            for (int k = 1; k < n; ++k) {
                                double st = from + (to - from) * double(k) / double(n);
                                if (closed && st >= length)
                                    st -= length;
                                to_hold.push_back(point_at_arc_length(course, st));
                            }
                        };
                        double last_held = closed ? stations[start] : 0.;
                        bool   in_unheld = !closed && held_at.front() == 0;
                        const size_t steps = closed ? count + 1 : count;
                        for (size_t j = closed ? 1 : 0; j < steps; ++j) {
                            const size_t i   = (start + j) % count;
                            const double arc = stations[i] + (closed && start + j >= count ? length : 0.);
                            if (held_at[i]) {
                                if (in_unheld)
                                    place_along(last_held, arc);
                                last_held = arc;
                                in_unheld = false;
                            } else
                                in_unheld = true;
                        }
                        const bool any_held = std::find(held_at.begin(), held_at.end(), char(1)) != held_at.end();
                        if (closed && !any_held) {
                            // A loop with no held station: n holders round it, station 0 among them, so that no stretch exceeds
                            // G_b (one holder for a loop no longer than G_b).
                            const int n = std::max(1, int(std::ceil(length / gb)));
                            for (int k = 0; k < n; ++k)
                                to_hold.push_back(point_at_arc_length(course, length * double(k) / double(n)));
                        } else if (in_unheld)    // an open course ends over the void
                            place_along(last_held, closed ? stations[start] + length : length);
                    }
                }
                for (const Point &end : to_hold) {
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

    // The point of a polyline at an arc length from its start (scaled), clamped to its ends.
    static Point point_at_arc_length(const Polyline &polyline, double along_mm)
    {
        double remaining = scale_(along_mm);
        for (size_t i = 1; i < polyline.points.size(); ++i) {
            const Vec2d a = polyline.points[i - 1].cast<double>(), d = (polyline.points[i] - polyline.points[i - 1]).cast<double>();
            const double length = d.norm();
            if (remaining <= length || i + 1 == polyline.points.size()) {
                const double t = length > 0. ? std::min(1., std::max(0., remaining / length)) : 0.;
                return Point(coord_t(std::llround(a.x() + t * d.x())), coord_t(std::llround(a.y() + t * d.y())));
            }
            remaining -= length;
        }
        return polyline.points.front();
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
            // A copy: add_strand() reassigns the plane's intervals, and a partial placement goes on to the next interval.
            const Intervals planned = it != lp.strands_by_plane.end() ? it->second : Intervals();
            for (const Interval &i : planned)
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
        const Steering stay = steering(item.plane, y, m);
        const std::optional<double> staying = nearest_admissible(item.plane, y, std::min(a, slack), stay.target, below, half);
        // Step 6: a single piece that would stay long, or is trapped, moves to another family when that lands sooner.
        if (length <= m_lambda + 1e-9 && (!staying || stay.layers >= K_MIN) &&
            change_family(item, y, m, staying ? stay.layers : std::numeric_limits<double>::infinity()))
            return;
        if (staying) {
            add_strand(below, Strand{ item.plane, *staying - half, *staying + half, StrandKind::Column, item.gap });
            return;
        }
        // Trapped (its trace runs into the boundary, typically at a wall) and too long to change family as one piece: cover pieces
        // of another family, each holding the item within the crossing reach c of its crossing (among the three lattice directions
        // only); an item longer than 2c takes several, c apart from each end.
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

    // The admissible centre for a piece at layer l within `step` of y, nearest the target: the exact move toward it first, then the
    // positions of the window ordered by their distance to the target (a target already reached, or none in sight, must still
    // leave the admissible shifts in play).
    std::optional<double> nearest_admissible(const PlaneKey &K, double y, double step, double target, size_t l, double half)
    {
        const double move = std::max(-step, std::min(step, target - y));
        std::vector<double> preferences = { move, step, -step, 0.5 * step, -0.5 * step, 0. };
        std::stable_sort(preferences.begin(), preferences.end(), [&](double p, double q) { return std::abs(y + p - target) < std::abs(y + q - target); });
        preferences.erase(std::unique(preferences.begin(), preferences.end(), [](double p, double q) { return std::abs(p - q) < 1e-9; }), preferences.end());
        for (const double shift : preferences)
            if (admissible(K, y + shift, l, half))
                return y + shift;
        return std::nullopt;
    }

    // Step 6, the family change: the single piece at m centred at x on K goes on, at m − 1, on the plane of another family through
    // its centre (the plane search of step 4; never a bulk plane, which would have landed it), at the admissible point nearest that
    // plane's steering target within r_sw = (λ − c)/2 of the crossing, where the crossing holds the piece above whole by (ii). It
    // changes when the estimate there, k_sw = 1 + the layers to land from the new piece, is at most k_stay − 1 (always when trapped,
    // k_stay infinite): the smallest k_sw, then material over bulk over bed, then the lower family index.
    bool change_family(const Item &item, double x, size_t m, double k_stay)
    {
        const size_t below = m - 1;
        // A switched piece is λ_sw(N) = λ + (N − 1) s_f / sin 60° long, and the window r_sw = (λ − c)/2 is the single line's
        // (3.9b): the band's crossing then holds every sub-line of the piece above whole.
        const double half = 0.5 * m_lambda + (m_lattice.multiline - 1) * m_lattice.spacing / (2. * SIN_60);
        const double r_sw = std::max(0., 0.5 * (m_lambda - std::max(1e-6, (0.5 * m_w - m_vertex_margin) / SIN_60)));
        const Trace tk = trace(m_lattice, item.plane, m_plan.print_z(m));
        const Vec2d centre = tk.point(x);
        struct Candidate { PlaneKey plane; double y, layers; int landing; };
        std::optional<Candidate> best;
        for (int j = 0; j < 3; ++j) {
            if (j == item.plane.family)
                continue;
            const std::optional<PlaneKey> plane = plane_through(m_lattice, j, centre, m_plan.print_z(below), m_tau);
            if (!plane)
                continue;
            const Trace tj = trace(m_lattice, *plane, m_plan.print_z(below));
            const auto tc = cross_parameter(tk, tj);
            if (!tc)
                continue;
            const double xj = tj.parameter(tk.point(*tc));
            const std::optional<double> y = nearest_admissible(*plane, xj, r_sw, steering(*plane, xj, m).target, below, half);
            if (!y)
                continue;
            Candidate candidate{ *plane, *y, 1., 0 };    // a piece held whole at m − 2 lands at once
            if (below >= 1 && !held_whole(*plane, *y - half, *y + half, below)) {
                const Steering from = steering(*plane, *y, below);
                candidate.layers  = 1. + from.layers;
                candidate.landing = from.landing;
                // A change by choice must leave the column room for the descent it promises: a plane whose trace drifts into the
                // boundary within those layers traps the column there (a change out of a trap needs only the usual room).
                if (std::isfinite(k_stay) && candidate.layers > LOOK_AHEAD + 1 &&
                    !admissible(*plane, *y, below, half, int(std::min(candidate.layers, double(below)))))
                    continue;
            }
            if (!best || candidate.layers < best->layers || (candidate.layers == best->layers && candidate.landing < best->landing))
                best = candidate;
        }
        if (!best || best->layers > k_stay - 1.)
            return false;
        add_strand(below, Strand{ best->plane, best->y - half, best->y + half, StrandKind::Column, item.gap });
        return true;
    }

    // Where a column steers, and the estimated layers to its landing there (step 5's closed form), by what it lands on: material,
    // a bulk line, or the bed (the preference order of step 6's ties).
    struct Steering { double target, layers; int landing; };
    enum Landing { OnMaterial = 0, OnBulk = 1, OnBed = 2 };

    // The centre of the next piece (at m − 1) that lands soonest: on a crossing with a bulk line of another family where that
    // crossing will be at m − 2 (so that the piece lies whole within the crossing's reach c there), inside the material reach along
    // the trace within 20 mm, or the current centre when nothing is in sight; a target counts only where the piece fits inside the
    // fill boundary of its own layer (beside a wall the material's reach and the boundary overlap by less than a piece, and a
    // column steered at the wall rode along it). The caller moves toward it by at most α a layer.
    Steering steering(const PlaneKey &K, double y, size_t m)
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
        int    landing = OnBed;
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
                if (!inside_boundary(K, tc, below, 0.5 * m_lambda))
                    continue;
                const double closing = approaching ? std::abs(v) + a : std::max(1e-6, a - std::abs(v));
                const double k = g <= a ? 1. : 1. + std::ceil((g - a) / closing);
                if (k < best_k) { best_k = k; target = tc; landing = OnBulk; }
            }
        }
        if (!lp2.material_reach->empty() && !lp.boundary->index.empty()) {
            const Point pa = m_lattice.scaled(tk.point(y - LOOK_ALONG)), pb = m_lattice.scaled(tk.point(y + LOOK_ALONG));
            for (const Interval &i : intersected(merged(lp2.material_reach->segment_inside(pa, pb)), merged(lp.boundary->index.segment_inside(pa, pb)))) {
                const double lo = y - LOOK_ALONG + i.first * 2. * LOOK_ALONG, hi = y - LOOK_ALONG + i.second * 2. * LOOK_ALONG;
                if (hi - lo < m_lambda)
                    continue;
                const double inside = std::min(std::max(y, lo + 0.5 * m_lambda), hi - 0.5 * m_lambda);
                const double g = std::abs(inside - y);
                const double k = g <= a ? 1. : 1. + std::ceil((g - a) / a);
                if (k < best_k) { best_k = k; target = inside; landing = OnMaterial; }
            }
        }
        return Steering{ target, best_k, landing };
    }

    GroupPlan                   &m_group;
    ObjectPlan                  &m_plan;
    const std::function<void()> &m_cancel;
    const LatticeParams         &m_lattice;
    std::vector<LayerPrint>      m_layers;
    double m_w = 0., m_c = 0., m_cN = 0., m_lambda = 0., m_tau = 0.;
    std::vector<double> m_offsets;    // the band's sub-line offsets (3.9b; one, 0, at N = 1)
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
