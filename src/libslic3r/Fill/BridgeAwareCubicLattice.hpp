#ifndef slic3r_BridgeAwareCubicLattice_hpp_
#define slic3r_BridgeAwareCubicLattice_hpp_

// Bridge aware cubic: Cubic's lattice in closed form (rewrite architecture, section 3.2 and appendix A).
//
// The three families of FillCubic's lines are the traces of three mutually orthogonal families of planes of a cubic lattice
// whose body diagonal is vertical. Family k has the normal n_k = (cos θ_k, sin θ_k), θ_k = infill direction + π/2 + k·π/3, and
// the sign σ_k ∈ (+1, −1, +1). Its lines at height z are { p : n_k · p + σ_k · z / √2 = m · L } in the object-centred frame of
// the filler (p in mm relative to the bounding box centre, L the pitch). Cube coordinates u_k = √(2/3) (n_k · p) + σ_k z / √3
// make the cell (i_0, i_1, i_2) at depth δ the box u_k ∈ [i_k a_δ, (i_k + 1) a_δ], a_δ = a_0 / 2^δ, a_0 = L √(2/3).
//
// bulk_lines() reproduces FillCubic's lines, with FillCubic's float-rounded angles and float-truncated pattern shift, so that
// the pattern's bulk output equals Cubic's; the plane geometry (traces, sections, crossings) uses exact double angles and the
// exact shift. The difference is below one scaled unit.

#include "../libslic3r.h"
#include "../BoundingBox.hpp"
#include "../Line.hpp"
#include "../Point.hpp"
#include "../Polygon.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace Slic3r {
namespace BridgeAwareCubic {

struct LatticeParams
{
    double  spacing   = 0.;     // s_f, mm: the sparse flow's spacing
    float   density   = 0.f;    // d, the fraction FillParams::density carries
    double  width     = 0.;     // w, mm: the sparse bead width
    int     multiline = 1;      // N
    coord_t pitch     = 0;      // L_N, scaled; clamped to half the coord_t range
    double  a0        = 0.;     // the cube edge, mm: unscale(pitch) · √(2/3)
    double  theta[3]  = { 0., 0., 0. };    // θ_k, exact: the plane geometry
    double  theta_fill[3] = { 0., 0., 0. };   // θ_k as FillCubic's float arithmetic yields them: the bulk lines
    int     sigma[3]  = { +1, -1, +1 };
    Point   center;             // the filler's reference point: the object's bounding box centre, scaled

    // `density` is the fraction FillParams::density carries (float, 0.01 × the percent), `spacing_mm` the sparse flow's spacing,
    // `multiline` the fill_multiline count; `object_bbox` is what Fill::bounding_box holds, whose centre is the reference point.
    static LatticeParams make(double infill_direction_deg, float density, double spacing_mm, double width_mm, int multiline,
                              const BoundingBox &object_bbox);

    Vec2d  normal(int k) const { return Vec2d(std::cos(theta[k]), std::sin(theta[k])); }       // n_k
    Vec2d  direction(int k) const { return Vec2d(-std::sin(theta[k]), std::cos(theta[k])); }   // d_k
    double band_width() const { return (multiline - 1) * spacing + width; }                    // W_N
    std::vector<double> subline_offsets() const;                                               // o_i = (i − (N − 1)/2) s_f

    // Cube coordinate of a point (mm, relative to the centre) at height z, and back.
    double cube_coordinate(int k, const Vec2d &p, double z) const;
    // The point and height of the cube coordinates (u_0, u_1, u_2).
    std::pair<Vec2d, double> from_cube(const std::array<double, 3> &u) const;

    // Between the filler's scaled frame and the lattice frame (mm, relative to the centre).
    Point scaled(const Vec2d &relative_mm) const;
    Vec2d relative(const Point &scaled) const;
};

// The plane u_family = numerator · a_0 / 2^depth. Depth 0 planes are Cubic's bulk planes; a refinement plane has an odd numerator
// at its depth and therefore never coincides with a plane of a smaller depth.
struct PlaneKey
{
    int     family    = 0;
    int     depth     = 0;
    int64_t numerator = 0;

    bool operator==(const PlaneKey &rhs) const { return family == rhs.family && depth == rhs.depth && numerator == rhs.numerator; }
    bool operator!=(const PlaneKey &rhs) const { return !(*this == rhs); }
    bool operator<(const PlaneKey &rhs) const
    {
        return family != rhs.family ? family < rhs.family : depth != rhs.depth ? depth < rhs.depth : numerator < rhs.numerator;
    }
};

// The cell u_k ∈ [index_k · a_depth, (index_k + 1) · a_depth], k = 0, 1, 2.
struct Cell
{
    int64_t index[3] = { 0, 0, 0 };
    int     depth    = 0;
};

// A plane's trace on a layer: the line point(t) = origin + t · dir, mm, relative to the centre; dir is the family's d_k.
struct Trace
{
    Vec2d origin;
    Vec2d dir;
    Vec2d point(double t) const { return origin + t * dir; }
    // The parameter of the point of the line nearest to p.
    double parameter(const Vec2d &p) const { return dir.dot(p - origin); }
};

// Where the trace of another plane crosses this one: the parameter along this trace and its drift per mm of height.
struct Crossing
{
    double t;
    double dt_dz;
};

Trace                    trace(const LatticeParams &lattice, const PlaneKey &plane, double z);
std::pair<double, double> height_range(const LatticeParams &lattice, const Cell &cell);
// The section of a cell at height z: the convex hull of the plane's intersections with the cube's twelve edges, scaled and
// absolute; empty when z lies outside the cell's height range.
Polygon                  section(const LatticeParams &lattice, const Cell &cell, double z);
// The crossing of plane J's trace with plane K's, along K's trace at height z; nothing when the planes are of one family.
std::optional<Crossing>  crossing(const LatticeParams &lattice, const PlaneKey &K, const PlaneKey &J, double z);
// FillCubic's lines of every family and sub-line at height z across `bbox` (scaled), as make_fill_lines lays them, the extreme
// lines included.
Lines                    bulk_lines(const LatticeParams &lattice, double z, const BoundingBox &bbox);
// The depth-0 cells whose cube meets `bbox` (scaled) at height z.
std::vector<Cell>        roots(const LatticeParams &lattice, const BoundingBox &bbox, double z);
std::array<Cell, 8>      children(const Cell &cell);
// The three planes through the cell's centre, one of each family: odd numerators at depth + 1.
std::array<PlaneKey, 3>  split_planes(const Cell &cell);
// The cell's lower (upper) face of the family.
PlaneKey                 face(const Cell &cell, int family, bool upper);
// The refinement plane of the family through a point (mm, relative to the centre) at height z: the coarsest of the levels 1 to
// 16 whose trace passes within `tolerance` of the point (never a bulk plane); nothing when no level does.
std::optional<PlaneKey>  plane_through(const LatticeParams &lattice, int family, const Vec2d &point, double z, double tolerance);

} // namespace BridgeAwareCubic
} // namespace Slic3r

#endif // slic3r_BridgeAwareCubicLattice_hpp_
