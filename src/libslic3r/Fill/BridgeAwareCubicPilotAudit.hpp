#ifndef slic3r_BridgeAwareCubicPilotAudit_hpp_
#define slic3r_BridgeAwareCubicPilotAudit_hpp_
#include "BridgeAwareCubicPilot.hpp"
#include "BridgeAwareCubicAttachment.hpp"
#include "../Layer.hpp"
#include "../Exception.hpp"
#include "../ExtrusionEntityCollection.hpp"
#include <map>
#include <numeric>
#include <set>

namespace Slic3r { namespace BridgeAwareCubic {
struct PilotReplayRoot
{
    size_t              region;
    ExtrusionRole       role;
    float               width, height;
    Polyline            original;
    std::vector<size_t> transitions;
};
struct PilotReplayProof
{
    std::shared_ptr<const void>  identity;
    std::vector<PilotReplayRoot> roots;
};
namespace Pilot {
// Use the raw writer operation order to obtain command coordinates; invert its
// phase ONLY on the exact geometry grid. Never round or expand material masks.
inline std::array<int64_t, 2> grid_translation(const SerializedXY &p)
{
    const auto exact = [](double x) {
        if (!std::isfinite(x) || std::abs(x) > 5000.) throw SlicingError("Canonical pilot phase is outside the admitted grid range");
        const int64_t n = std::llround(x / SCALING_FACTOR);
        if (double(n) * SCALING_FACTOR != x) throw SlicingError("Canonical pilot post-serialization material proof requires a grid-exact placement");
        return n;
    };
    std::array<int64_t, 2> result{};
    for (int i = 0; i < 2; ++i) result[i] = exact(p.shift[i]) - exact(p.nozzle_offset[i]) - exact(p.plate_origin[i]);
    return result;
}
inline Point local_point(const Point &cmd, const std::array<int64_t, 2> &offset)
{
    Point p;
    for (int i = 0; i < 2; ++i) {
        const int64_t n = int64_t(cmd[i]) - offset[i];
        if (n < -int64_t(std::numeric_limits<int32_t>::max() / 4) || n > std::numeric_limits<int32_t>::max() / 4)
            throw SlicingError("Canonical pilot inverse placement is outside the clipping grid");
        p[i] = coord_t(n);
    }
    return p;
}
// Exact integer line interval event counts. Reversal, order and positive
// collinear subdivisions do not change the completed multiplicity profile.
using LineKey = std::tuple<int64_t, int64_t, int64_t>;
using Profile = std::map<LineKey, std::map<coord_t, int>>;
inline void event(Profile &profile, const Point &a, const Point &b)
{
    if (a == b) return;
    int64_t       dx = int64_t(b.x()) - a.x(), dy = int64_t(b.y()) - a.y();
    const int64_t gcd = std::gcd(dx, dy);
    dx /= gcd;
    dy /= gcd;
    if (dx < 0 || (dx == 0 && dy < 0)) {
        dx = -dx;
        dy = -dy;
    }
    // All input command coordinates are int32 multiples of 100; the
    // primitive direction and determinant fit the signed 64-bit key.
    const int64_t c = dx * int64_t(a.y()) - dy * int64_t(a.x());
    const coord_t u = dx ? a.x() : a.y(), v = dx ? b.x() : b.y();
    auto         &count = profile[{dx, dy, c}];
    for (auto [at, n] : {std::pair<coord_t, int>{std::min(u, v), 1}, {std::max(u, v), -1}})
        if ((count[at] += n) == 0) count.erase(at);
}
struct Coverage
{
    Profile all, part;
    void    add(const Point &a, const Point &b, bool partition)
    {
        event(all, a, b);
        if (partition) event(part, a, b);
    }
    void verify() const
    {
        for (const auto &[key, pe] : part) {
            const auto &ae = all.at(key);
            auto        p  = pe.begin();
            auto        a  = ae.begin();
            int         nc = 0, na = 0;
            while (p != pe.end() || a != ae.end()) {
                const auto x = p == pe.end() ? a->first : a == ae.end() ? p->first : std::min(p->first, a->first);
                if (p != pe.end() && p->first == x) {
                    nc += p->second;
                    ++p;
                }
                if (a != ae.end() && a->first == x) {
                    na += a->second;
                    ++a;
                }
                if (nc > 0 && na > 1) throw SlicingError("Canonical pilot would deposit a duplicate moving course");
            }
        }
    }
};
// Unknown/deforming path kinds cannot be projected as trustworthy planar motion.
template<class Visit> void walk(const ExtrusionEntity &entity, const Visit &visit, bool free_only = false, size_t depth = 0)
{
    if (depth > 64) throw SlicingError("Canonical pilot entity nesting limit");
    const auto &type = typeid(entity);
    if (type == typeid(ExtrusionPath) || type == typeid(ExtrusionPathOriented) || type == typeid(PilotPath)) {
        const auto &p = static_cast<const ExtrusionPath &>(entity);
        if (p.is_force_no_extrusion()) return;
        if (!std::isfinite(p.mm3_per_mm)) throw SlicingError("Canonical pilot non-finite actual flow");
        if (p.mm3_per_mm <= 0. || p.polyline.size() < 2) return;
        if (!p.polyline.fitting_result.empty()) throw SlicingError("Canonical pilot cannot audit fitted extrusion");
        visit(p);
        return;
    }
    if (type == typeid(ExtrusionEntityCollection)) {
        for (const auto *child : static_cast<const ExtrusionEntityCollection &>(entity).entities) {
            if (!child) throw SlicingError("Null model extrusion");
            walk(*child, visit, free_only, depth + 1);
        }
        return;
    }
    if (type == typeid(ExtrusionLoop)) {
        if (!free_only)
            for (const auto &p : static_cast<const ExtrusionLoop &>(entity).paths) walk(p, visit, false, depth + 1);
        return;
    }
    if (type == typeid(ExtrusionMultiPath)) {
        if (!free_only)
            for (const auto &p : static_cast<const ExtrusionMultiPath &>(entity).paths) walk(p, visit, false, depth + 1);
        return;
    }
    throw SlicingError("Canonical pilot cannot audit an unknown model extrusion type");
}
struct AttachmentCounts
{
    std::vector<unsigned> hits;
    explicit AttachmentCounts(size_t count) : hits(count, 0) {}
    void consume(size_t i, const LayerAttachment &a, const PilotPath *path)
    {
        if (i >= hits.size() || ++hits[i] != 1) throw SlicingError("Canonical attachment consumed more than once");
        if (path && !path->partition()) throw SlicingError("Canonical bulk attachment unsupported");
        if (!path) {
            if (!a.owners.empty() || a.solid_cut) throw SlicingError("Canonical model attachment cannot claim a partition owner");
            return;
        }
        const auto &before = a.original.points, &after = a.replacement.points;
        if (before.size() < 2 || after.size() != before.size() + 1 || a.owners.size() != 1 || a.solid_cut)
            throw SlicingError("Canonical partition attachment must extend exactly one terminal with one owner");
        const bool front = std::equal(before.begin(), before.end(), after.begin() + 1), back = std::equal(before.begin(), before.end(), after.begin());
        if (front == back || !(a.owners[0] == (front ? path->source.atoms.front() : path->source.atoms.back()).source.owner))
            throw SlicingError("Canonical attachment does not belong to its actual terminal owner");
    }
    void finish() const
    {
        if (std::any_of(hits.begin(), hits.end(), [](unsigned n) { return n != 1; })) throw SlicingError("Canonical active attachment has no unique live replay root");
    }
};
struct UnusedPartitionRepair : SlicingError
{
    UnusedPartitionRepair(std::shared_ptr<const void> plan, const LayerAttachment &attachment)
        : SlicingError("Canonical active attachment has no unique live replay root"), plan(std::move(plan)), attachment(attachment)
    {}
    std::shared_ptr<const void> plan;
    LayerAttachment             attachment;
};

inline void validate_roots(const Layer &layer, const std::vector<LayerAttachment> &active, const PilotReplayProof &proof)
{
    if (active.size() > 20000 || proof.roots.size() > active.size()) throw SlicingError("Canonical replay accounting limit");
    AttachmentCounts                counts(active.size());
    std::set<const ExtrusionPath *> claimed;
    size_t                          work = 5000000;
    for (const auto &r : proof.roots) {
        if (r.region >= layer.regions().size()) throw SlicingError("Canonical replay root has no emitter");
        const auto result = BridgeAwareCubicAttachment::resolve_chain(r.original, r.width, r.height, r.region, active);
        if (result.applied_indices != r.transitions || r.transitions.empty()) throw SlicingError("Canonical attachment chain no longer matches its generated replay");
        const ExtrusionPath *found = nullptr;
        const auto           match = [&](const ExtrusionPath &p) {
            if (work == 0) throw SlicingError("Canonical replay matching work limit");
            --work;
            if (p.role() != r.role || p.width != r.width || p.height != r.height || p.polyline.size() != result.polyline.size()) return;
            if (p.polyline.size() > work) throw SlicingError("Canonical replay geometry work limit");
            work -= p.polyline.size();
            if (p.polyline.points != result.polyline.points) return;
            if (found) throw SlicingError("Canonical attachment final path is ambiguous");
            found = &p;
        };
        const auto *region = layer.get_region(int(r.region));
        walk(region->perimeters, match, true);
        walk(region->fills, match, true);
        if (!found || !claimed.insert(found).second) throw SlicingError("Canonical attachment lost its unique emitted path");
        const auto *p = dynamic_cast<const PilotPath *>(found);
        if (p && (p->source.points != r.original.points || p->plan_identity != proof.identity)) throw SlicingError("Canonical attachment source is stale");
        for (size_t id : r.transitions) counts.consume(id, active.at(id), p);
    }
    // All surviving free paths and their complete replay chains were visited.
    // Only the planner may remove a proven unused partition repair and retry
    // generation. Export and independent calls must reject a stale payload.
    for (size_t i = 0; i < active.size(); ++i) {
        if (counts.hits[i] == 0 && active[i].owners.size() == 1 && !active[i].solid_cut) throw UnusedPartitionRepair(proof.identity, active[i]);
    }
    counts.finish();
}
} // namespace Pilot
}} // namespace Slic3r::BridgeAwareCubic
#endif
