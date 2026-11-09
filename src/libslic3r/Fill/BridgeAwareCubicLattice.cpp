#include "BridgeAwareCubicLattice.hpp"

#include "../Geometry/ConvexHull.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace Slic3r {
namespace BridgeAwareCubic {

namespace {

constexpr double SQRT_HALF         = 0.70710678118654752440;   // 1/√2
constexpr double SQRT_TWO_THIRDS   = 0.81649658092772603273;   // √(2/3)
constexpr double SQRT_THREE_HALVES = 1.22474487139158904909;   // √(3/2)
constexpr double SQRT_THIRD        = 0.57735026918962576451;   // 1/√3

double cell_edge(const LatticeParams &lattice, int depth) { return lattice.a0 / double(int64_t(1) << depth); }

// The matrix of the cube coordinates: u = M (p.x, p.y, z).
Eigen::Matrix3d to_cube_matrix(const LatticeParams &lattice)
{
    Eigen::Matrix3d m;
    for (int k = 0; k < 3; ++k) {
        const Vec2d n = lattice.normal(k);
        m.row(k) << SQRT_TWO_THIRDS * n.x(), SQRT_TWO_THIRDS * n.y(), lattice.sigma[k] * SQRT_THIRD;
    }
    return m;
}

} // namespace

LatticeParams LatticeParams::make(double infill_direction_deg, float density, double spacing_mm, double width_mm, int multiline,
                                  const BoundingBox &object_bbox)
{
    LatticeParams p;
    p.spacing   = spacing_mm;
    p.density   = density;
    p.width     = width_mm;
    p.multiline = std::max(1, multiline);
    p.center    = object_bbox.center();
    // FillCubic: params.density /= 3 (float), line_spacing = coord_t(scale_(spacing) * multiline / params.density).
    const float density_per_sweep = float(double(density) / 3.);
    const double pitch = scale_(spacing_mm) * p.multiline / double(density_per_sweep);
    const double max_pitch = double(std::numeric_limits<coord_t>::max() / 2);
    p.pitch = coord_t(std::min(std::max(pitch, 1.), max_pitch));
    p.a0    = unscale<double>(p.pitch) * SQRT_TWO_THIRDS;
    // Fill::_infill_direction: float angle = float(deg2rad(direction)) + float(π/2); then + the sweep's float angle base.
    const float base = float(infill_direction_deg * M_PI / 180.) + float(M_PI / 2.);
    const float sweeps[3] = { 0.f, float(M_PI / 3.), float(M_PI * 2. / 3.) };
    for (int k = 0; k < 3; ++k) {
        p.theta_fill[k] = double(base + sweeps[k]);
        p.theta[k]      = infill_direction_deg * M_PI / 180. + M_PI / 2. + k * M_PI / 3.;
    }
    return p;
}

std::vector<double> LatticeParams::subline_offsets() const
{
    std::vector<double> out;
    out.reserve(multiline);
    for (int i = 0; i < multiline; ++i)
        out.push_back((i - (multiline - 1) / 2.) * spacing);
    return out;
}

double LatticeParams::cube_coordinate(int k, const Vec2d &p, double z) const
{
    return SQRT_TWO_THIRDS * normal(k).dot(p) + sigma[k] * z * SQRT_THIRD;
}

std::pair<Vec2d, double> LatticeParams::from_cube(const std::array<double, 3> &u) const
{
    const Eigen::Vector3d x = to_cube_matrix(*this).inverse() * Eigen::Vector3d(u[0], u[1], u[2]);
    return { Vec2d(x(0), x(1)), x(2) };
}

Point LatticeParams::scaled(const Vec2d &relative_mm) const
{
    return Point(coord_t(std::llround(scale_(relative_mm.x()))) + center.x(), coord_t(std::llround(scale_(relative_mm.y()))) + center.y());
}

Vec2d LatticeParams::relative(const Point &p) const
{
    return Vec2d(unscale<double>(p.x() - center.x()), unscale<double>(p.y() - center.y()));
}

Trace trace(const LatticeParams &lattice, const PlaneKey &plane, double z)
{
    // n_k · p = q · a_δ · √(3/2) − σ_k z / √2
    const double c = plane.numerator * cell_edge(lattice, plane.depth) * SQRT_THREE_HALVES - lattice.sigma[plane.family] * z * SQRT_HALF;
    return Trace{ c * lattice.normal(plane.family), lattice.direction(plane.family) };
}

std::pair<double, double> height_range(const LatticeParams &lattice, const Cell &cell)
{
    // z = Σ_k σ_k u_k / √3; the extreme corners take the lower face where σ_k < 0 and the upper where σ_k > 0, and the reverse.
    const double a = cell_edge(lattice, cell.depth);
    double lo = 0., hi = 0.;
    for (int k = 0; k < 3; ++k) {
        const double u0 = cell.index[k] * a, u1 = u0 + a;
        if (lattice.sigma[k] > 0) { lo += u0; hi += u1; } else { lo -= u1; hi -= u0; }
    }
    return { lo * SQRT_THIRD, hi * SQRT_THIRD };
}

Polygon section(const LatticeParams &lattice, const Cell &cell, double z)
{
    const auto range = height_range(lattice, cell);
    if (z < range.first || z > range.second)
        return Polygon();
    const double a = cell_edge(lattice, cell.depth);
    const Eigen::Matrix3d from_cube = to_cube_matrix(lattice).inverse();
    std::array<Eigen::Vector3d, 8> corners;
    for (int c = 0; c < 8; ++c)
        corners[c] = from_cube * Eigen::Vector3d((cell.index[0] + (c & 1)) * a, (cell.index[1] + ((c >> 1) & 1)) * a, (cell.index[2] + ((c >> 2) & 1)) * a);
    Points points;
    for (int c = 0; c < 8; ++c)
        for (int bit = 0; bit < 3; ++bit) {
            const int d = c | (1 << bit);
            if (d == c)
                continue;
            const Eigen::Vector3d &p = corners[c], &q = corners[d];
            const double zp = p(2), zq = q(2);
            if ((zp <= z && zq >= z) || (zq <= z && zp >= z)) {
                const double f = std::abs(zq - zp) < 1e-12 ? 0. : (z - zp) / (zq - zp);
                const Eigen::Vector3d at = p + f * (q - p);
                points.push_back(lattice.scaled(Vec2d(at(0), at(1))));
            }
        }
    if (points.size() < 3)
        return Polygon();
    return Geometry::convex_hull(points);
}

std::optional<Crossing> crossing(const LatticeParams &lattice, const PlaneKey &K, const PlaneKey &J, double z)
{
    if (K.family == J.family)
        return std::nullopt;
    const Trace tk = trace(lattice, K, z);
    const Vec2d  nj = lattice.normal(J.family);
    const double denominator = nj.dot(tk.dir);
    if (std::abs(denominator) < 1e-9)
        return std::nullopt;
    const double cj = J.numerator * cell_edge(lattice, J.depth) * SQRT_THREE_HALVES - lattice.sigma[J.family] * z * SQRT_HALF;
    const double t = (cj - nj.dot(tk.origin)) / denominator;
    // d c_J / dz = −σ_j / √2; d origin_K / dz = −σ_k / √2 · n_k.
    const double dcj = -lattice.sigma[J.family] * SQRT_HALF;
    const double dorigin = -lattice.sigma[K.family] * SQRT_HALF * nj.dot(lattice.normal(K.family));
    return Crossing{ t, (dcj - dorigin) / denominator };
}

Lines bulk_lines(const LatticeParams &lattice, double z, const BoundingBox &bbox)
{
    Lines out;
    if (lattice.pitch <= 0 || !bbox.defined)
        return out;
    const coord_t line_spacing = lattice.pitch;
    const coord_t line_width   = coord_t(scale_(lattice.spacing));
    const int     n            = lattice.multiline;
    const float   dx           = float(SQRT_HALF * z);                  // FillCubic: coordf_t dx = sqrt(0.5) * z; passed as float(dx)
    const std::array<Point, 4> corners = { bbox.min, Point(bbox.max.x(), bbox.min.y()), bbox.max, Point(bbox.min.x(), bbox.max.y()) };
    for (int k = 0; k < 3; ++k) {
        const float  angle = float(lattice.theta_fill[k]);
        const double cos_a = std::cos(angle), sin_a = std::sin(angle);
        // The vertical frame of make_fill_lines: rotate by −angle; refpt is the reference point in that frame.
        const Point refpt_rotated = lattice.center.rotated(-angle);
        coord_t x_lo = std::numeric_limits<coord_t>::max(), x_hi = std::numeric_limits<coord_t>::min();
        coord_t y_lo = std::numeric_limits<coord_t>::max(), y_hi = std::numeric_limits<coord_t>::min();
        for (const Point &corner : corners) {
            const Point r = corner.rotated(-angle);
            x_lo = std::min(x_lo, r.x()); x_hi = std::max(x_hi, r.x());
            y_lo = std::min(y_lo, r.y()); y_hi = std::max(y_hi, r.y());
        }
        const float shift_sign = lattice.sigma[k] > 0 ? dx : -dx;       // {0, dx}, {π/3, −dx}, {2π/3, dx}
        for (int i = 0; i < n; ++i) {
            const coord_t group_offset    = i * line_spacing;
            const coord_t internal_offset = coord_t((i - (n - 1) / 2.0f) * line_width);
            const coord_t total_offset    = group_offset + internal_offset;
            const coord_t pattern_shift   = coord_t(scale_(double(shift_sign) + unscale_(total_offset)));
            const coord_t shift           = pattern_shift % line_spacing;
            const coord_t ref             = refpt_rotated.x() - (shift >= 0 ? shift : line_spacing + shift);
            // Every line x ≡ ref (mod L) inside [x_lo, x_hi].
            const int64_t m_lo = int64_t(std::floor(double(x_lo - ref) / double(line_spacing)));
            const int64_t m_hi = int64_t(std::ceil(double(x_hi - ref) / double(line_spacing)));
            for (int64_t m = m_lo; m <= m_hi; ++m) {
                const int64_t x = int64_t(ref) + m * int64_t(line_spacing);
                if (x < x_lo || x > x_hi)
                    continue;
                out.emplace_back(Point(coord_t(x), y_lo).rotated(cos_a, sin_a), Point(coord_t(x), y_hi).rotated(cos_a, sin_a));
            }
        }
    }
    return out;
}

std::vector<Cell> roots(const LatticeParams &lattice, const BoundingBox &bbox, double z)
{
    std::vector<Cell> out;
    if (!bbox.defined || lattice.a0 <= 0.)
        return out;
    const std::array<Point, 4> corners = { bbox.min, Point(bbox.max.x(), bbox.min.y()), bbox.max, Point(bbox.min.x(), bbox.max.y()) };
    int64_t lo[3], hi[3];
    for (int k = 0; k < 3; ++k) { lo[k] = std::numeric_limits<int64_t>::max(); hi[k] = std::numeric_limits<int64_t>::min(); }
    for (const Point &corner : corners) {
        const Vec2d p = lattice.relative(corner);
        for (int k = 0; k < 3; ++k) {
            const int64_t i = int64_t(std::floor(lattice.cube_coordinate(k, p, z) / lattice.a0));
            lo[k] = std::min(lo[k], i);
            hi[k] = std::max(hi[k], i);
        }
    }
    for (int64_t i0 = lo[0]; i0 <= hi[0]; ++i0)
        for (int64_t i1 = lo[1]; i1 <= hi[1]; ++i1)
            for (int64_t i2 = lo[2]; i2 <= hi[2]; ++i2) {
                Cell cell{ { i0, i1, i2 }, 0 };
                const auto range = height_range(lattice, cell);
                if (z < range.first || z > range.second)
                    continue;
                const Polygon p = section(lattice, cell, z);
                if (p.points.empty() || !p.bounding_box().overlap(bbox))
                    continue;
                out.push_back(cell);
            }
    return out;
}

std::array<Cell, 8> children(const Cell &cell)
{
    std::array<Cell, 8> out;
    for (int c = 0; c < 8; ++c) {
        out[c].depth = cell.depth + 1;
        for (int k = 0; k < 3; ++k)
            out[c].index[k] = 2 * cell.index[k] + ((c >> k) & 1);
    }
    return out;
}

std::array<PlaneKey, 3> split_planes(const Cell &cell)
{
    std::array<PlaneKey, 3> out;
    for (int k = 0; k < 3; ++k)
        out[k] = PlaneKey{ k, cell.depth + 1, 2 * cell.index[k] + 1 };
    return out;
}

PlaneKey face(const Cell &cell, int family, bool upper)
{
    return PlaneKey{ family, cell.depth, cell.index[family] + (upper ? 1 : 0) };
}

std::optional<PlaneKey> plane_through(const LatticeParams &lattice, int family, const Vec2d &point, double z, double tolerance)
{
    const double u = lattice.cube_coordinate(family, point, z);
    for (int depth = 1; depth <= 16; ++depth) {
        const double a = cell_edge(lattice, depth);
        const double q = u / a;
        // The odd numerator nearest to q.
        int64_t odd = int64_t(std::floor(q));
        if (odd % 2 == 0)
            ++odd;
        if (std::abs(q - double(odd - 2)) < std::abs(q - double(odd)))
            odd -= 2;
        // The trace's distance from the point in the XY plane is √(3/2) times the cube-coordinate distance.
        if (SQRT_THREE_HALVES * std::abs(u - odd * a) <= tolerance)
            return PlaneKey{ family, depth, odd };
    }
    return std::nullopt;
}

} // namespace BridgeAwareCubic
} // namespace Slic3r
