#ifndef slic3r_RoundHoleDetection_hpp_
#define slic3r_RoundHoleDetection_hpp_

#include "Point.hpp"
#include <functional>
#include <vector>

struct indexed_triangle_set;

namespace Slic3r::WallPainting {

struct RoundHole {
    std::vector<int> facets;
    Vec3d axis;
    Vec3d center;
    double radius;
};

// Finds complete inward-facing cylindrical wall patches. Facets refer to the
// original mesh; geometry is measured after transform. Does not modify the mesh.
// progress is in [0, 1] and may throw to cancel the operation.
std::vector<RoundHole> detect_round_internal_holes(
    const indexed_triangle_set &mesh,
    const Transform3d &transform = Transform3d::Identity(),
    const std::function<void(float)> &progress = {});

} // namespace Slic3r::WallPainting
#endif
