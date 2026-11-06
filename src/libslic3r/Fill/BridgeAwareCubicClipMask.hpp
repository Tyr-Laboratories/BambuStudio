#ifndef slic3r_BridgeAwareCubicClipMask_hpp_
#define slic3r_BridgeAwareCubicClipMask_hpp_

#include "../BoundingBox.hpp"
#include "../Polyline.hpp"

#include <utility>

namespace Slic3r { namespace BridgeAwareCubic {

// One immutable mask, queried by complete paths or closed cell sections.
// A ring whose bounds are disjoint from the subject bounds has winding zero
// everywhere on the subject. Excluding only those rings preserves the exact
// nonzero-fill intersection, including containing contours and holes.
class BoundedClipMask {
public:
    explicit BoundedClipMask(Polygons rings) : m_rings(std::move(rings))
    {
        m_bounds.reserve(m_rings.size());
        for (const Polygon &ring : m_rings)
            m_bounds.push_back(get_extents<true>(ring.points));
    }

    Polygons candidates(const Polyline &subject) const
    {
        // IncludeBoundary is essential for horizontal and vertical paths.
        return candidates(get_extents<true>(subject.points));
    }

    Polygons candidates(const Polygon &subject) const
    {
        return candidates(get_extents<true>(subject.points));
    }

    Polygons candidates(const BoundingBox &bounds) const
    {
        Polygons result;
        if (!bounds.defined) return result;
        for (size_t i = 0; i < m_rings.size(); ++i)
            if (m_bounds[i].defined && m_bounds[i].overlap(bounds))
                result.push_back(m_rings[i]);
        return result;
    }

private:
    Polygons m_rings;
    std::vector<BoundingBox> m_bounds;
};

}} // namespace Slic3r::BridgeAwareCubic

#endif
