#ifndef slic3r_BridgeAwareCubicOwnerBounds_hpp_
#define slic3r_BridgeAwareCubicOwnerBounds_hpp_

#include "../Polyline.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace Slic3r { namespace BridgeAwareCubic {

namespace OwnerBoundsDetail {
struct Bounds {
    static constexpr int64_t coordinate_limit = std::min<int64_t>(
        std::numeric_limits<coord_t>::max() / 4, std::numeric_limits<int32_t>::max() / 4);
    int64_t min_x = 0, min_y = 0, max_x = 0, max_y = 0;
    bool populated = false;

    bool add(const Point &point) noexcept
    {
        const int64_t x = point.x(), y = point.y();
        if (x < -coordinate_limit || x > coordinate_limit ||
            y < -coordinate_limit || y > coordinate_limit)
            return false;
        if (!populated) {
            min_x = max_x = x;
            min_y = max_y = y;
            populated = true;
        } else {
            min_x = std::min(min_x, x);
            min_y = std::min(min_y, y);
            max_x = std::max(max_x, x);
            max_y = std::max(max_y, y);
        }
        return true;
    }
};
} // namespace OwnerBoundsDetail

class PartitionOwnerBounds;

// Compute once for the complete emitted path, including every original bend.
// Empty, stationary or unsafe inputs retain the legacy matching path.
class PartitionCandidateBounds {
public:
    explicit PartitionCandidateBounds(const Polyline &path) noexcept
    {
        if (path.size() < 2) return;
        bool moving = false;
        for (size_t i = 0; i < path.size(); ++i) {
            if (!m_bounds.add(path.points[i])) return;
            moving |= i > 0 && path.points[i] != path.points[i - 1];
        }
        m_checked = moving;
    }

private:
    OwnerBoundsDetail::Bounds m_bounds;
    bool m_checked = false;
    friend class PartitionOwnerBounds;
};

// Optional broad phase for matching_partition_segments only. It never assigns
// an owner: every retained owner must still run the unchanged exact matcher.
// Store these in original owner order and compute each owner's bounds once.
class PartitionOwnerBounds {
public:
    explicit PartitionOwnerBounds(const Polylines &lines) noexcept
    {
        if (lines.empty()) return;
        for (const Polyline &line : lines) {
            if (line.size() < 2) return;
            for (size_t i = 0; i < line.size(); ++i) {
                if (!m_bounds.add(line.points[i])) return;
                if (i == 0) continue;
                const int64_t dx = int64_t(line.points[i].x()) - line.points[i - 1].x();
                const int64_t dy = int64_t(line.points[i].y()) - line.points[i - 1].y();
                // Do not square large differences. This exact test also keeps
                // duplicate points and every segment of length <= 8 units.
                if (dx >= -8 && dx <= 8 && dy >= -8 && dy <= 8 && dx * dx + dy * dy <= 64)
                    return;
            }
        }
        m_checked = true;
    }

    bool may_match(const PartitionCandidateBounds &candidate) const noexcept
    {
        if (!m_checked || !candidate.m_checked) return true;
        const auto &query = candidate.m_bounds;
        // The matching-only legacy shortcut tests the shorter segment against
        // the longer infinite line, then projects candidate endpoints onto the
        // owner. Tiny perpendicular owners can therefore own distant paths:
        // owner (0,0)->(1,0), candidate (0,10)->(0,20). Keep them above.
        //
        // For owner length L>8, the candidate-longer branch gives
        // |sin(theta)|<=4/L and |cos(theta)|>sqrt(3)/2. Projecting a candidate
        // endpoint onto the owner then bounds its distance by
        // 2/|cos(theta)|<4/sqrt(3)<2.31 units. The owner-longer branch bounds
        // that distance directly by 2. Within the checked 32-bit coordinate range,
        // Point subtraction is safe and binary64 cross/dot/norm errors are
        // below 1e-5 units of distance; these bounds remain below 2.31.
        //
        // The other branch clips against offset(two-point overlaps,2): butt
        // caps and no interior joins, hence no miter extension. Rounded offset
        // vertices fit inside the owner box + 3; intersection adds no safety
        // offset. Four units conservatively covers both branches. Inclusive
        // overlap retains boundary touches; widen before any bound arithmetic.
        constexpr int64_t margin = 4;
        return !(query.max_x < m_bounds.min_x - margin || query.min_x > m_bounds.max_x + margin ||
                 query.max_y < m_bounds.min_y - margin || query.min_y > m_bounds.max_y + margin);
    }

private:
    OwnerBoundsDetail::Bounds m_bounds;
    bool m_checked = false;
};

}} // namespace Slic3r::BridgeAwareCubic

#endif
