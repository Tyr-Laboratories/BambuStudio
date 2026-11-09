#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicLattice.hpp"
#include "libslic3r/Fill/FillRectilinear.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Surface.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::BridgeAwareCubic;

namespace {

// A segment with its ends ordered, so that two sets of segments compare regardless of direction and order.
struct Segment {
    Point a, b;
    Segment(Point p, Point q) : a(p), b(q) { if (std::make_pair(a.x(), a.y()) > std::make_pair(b.x(), b.y())) std::swap(a, b); }
    bool operator<(const Segment &rhs) const
    {
        return std::make_tuple(a.x(), a.y(), b.x(), b.y()) < std::make_tuple(rhs.a.x(), rhs.a.y(), rhs.b.x(), rhs.b.y());
    }
};

std::vector<Segment> segments_of(const Polylines &polylines)
{
    std::vector<Segment> out;
    for (const Polyline &polyline : polylines)
        for (size_t i = 1; i < polyline.points.size(); ++i)
            out.emplace_back(polyline.points[i - 1], polyline.points[i]);
    std::sort(out.begin(), out.end());
    return out;
}

// The family of a segment: the direction d_k it is parallel to.
int family_of(const LatticeParams &lattice, const Line &line)
{
    const Vec2d dir = (line.b - line.a).cast<double>().normalized();
    int best = 0;
    double best_dot = -1.;
    for (int k = 0; k < 3; ++k) {
        const double dot = std::abs(dir.dot(lattice.direction(k)));
        if (dot > best_dot) { best_dot = dot; best = k; }
    }
    return best;
}

// The corners of a cell's cube in (x, y, z), mm, from its cube coordinates, by brute force: solve the three linear equations
// u_k = sqrt(2/3) (n_k . p) + sigma_k z / sqrt(3) for p and z (least squares over the three normals, which are not orthogonal).
struct Corner { Vec2d p; double z; };
std::array<Corner, 8> corners_of(const LatticeParams &lattice, const Cell &cell)
{
    const double a = lattice.a0 / double(int64_t(1) << cell.depth);
    std::array<Corner, 8> out;
    for (int c = 0; c < 8; ++c) {
        Eigen::Matrix3d A;
        Eigen::Vector3d u;
        for (int k = 0; k < 3; ++k) {
            const Vec2d n = lattice.normal(k);
            A.row(k) << std::sqrt(2. / 3.) * n.x(), std::sqrt(2. / 3.) * n.y(), lattice.sigma[k] / std::sqrt(3.);
            u(k) = (cell.index[k] + ((c >> k) & 1)) * a;
        }
        const Eigen::Vector3d x = A.colPivHouseholderQr().solve(u);
        out[c] = { Vec2d(x(0), x(1)), x(2) };
    }
    return out;
}

double cube_coordinate(const LatticeParams &lattice, int k, const Vec2d &p, double z)
{
    return std::sqrt(2. / 3.) * lattice.normal(k).dot(p) + lattice.sigma[k] * z / std::sqrt(3.);
}

} // namespace

// L1: the pattern's bulk lines are FillCubic's, point for point, for one, two and three lines, several directions (two outside
// [0, 360)), densities and heights, and a reference point off the origin; except within one line width of the rotated bounding
// box's extremes, which make_fill_lines drops and the pattern keeps (R5).
TEST_CASE("Bridge aware cubic bulk lines equal FillCubic's point for point", "[BridgeAwareCubic][L1]")
{
    const double s_f = 0.357, w = 0.4;
    for (const int N : {1, 2, 3})
    for (const double direction : {0., 45., 17., 405., -30.})
    for (const double density : {0.05, 0.12, 0.03})
    for (const double z : {0.2, 3.4, 17.8})
    for (const Vec2d center : {Vec2d(0., 0.), Vec2d(2., 8.)}) {
        CAPTURE(N, direction, density, z, center.x(), center.y());
        const BoundingBox bbox(Point::new_scale(center.x() - 32., center.y() - 28.), Point::new_scale(center.x() + 32., center.y() + 28.));
        const ExPolygon rectangle(Polygon({ Point::new_scale(center.x() - 30., center.y() - 22.), Point::new_scale(center.x() + 30., center.y() - 22.),
                                            Point::new_scale(center.x() + 30., center.y() + 22.), Point::new_scale(center.x() - 30., center.y() + 22.) }));

        FillCubic fill;
        fill.spacing  = s_f;
        fill.angle    = float(Geometry::deg2rad(direction));
        fill.z        = z;
        fill.layer_id = 0;
        fill.set_bounding_box(bbox);
        FillParams params;
        params.density           = float(density);
        params.multiline         = N;
        params.anchor_length     = 0.f;
        params.anchor_length_max = 0.f;
        Surface surface(stInternal, rectangle);
        const std::vector<Segment> cubic = segments_of(fill.fill_surface(&surface, params));
        REQUIRE_FALSE(cubic.empty());

        const LatticeParams lattice = LatticeParams::make(direction, density, s_f, w, N, bbox);
        CHECK(lattice.center == bbox.center());
        const Lines lines = bulk_lines(lattice, z, get_extents(rectangle));
        // Clip as fill_surface_by_multilines does, and drop the extreme lines as make_fill_lines does.
        const coord_t aoffset = coord_t(float(scale_(-0.5 * s_f)));
        const Polygons boundary = offset(to_polygons(rectangle), float(aoffset), ClipperLib::jtMiter, 3.);
        const coord_t x_margin = coord_t(scale_(s_f)) + coord_t(SCALED_EPSILON);
        Polylines kept;
        for (const Line &line : lines) {
            const int k = family_of(lattice, line);
            const double angle = lattice.theta_fill[k];
            Polygon rotated = rectangle.contour;
            rotated.rotate(-angle);
            const BoundingBox rotated_bbox = rotated.bounding_box();
            const double x = lattice.normal(k).dot(line.a.cast<double>());
            if (x < rotated_bbox.min.x() + x_margin || x > rotated_bbox.max.x() - x_margin)
                continue;
            kept.emplace_back(Points{ line.a, line.b });
        }
        // Clip one line at a time: Clipper splits open paths clipped together where they cross each other.
        Polylines clipped;
        for (const Polyline &line : kept)
            append(clipped, intersection_pl(Polylines{ line }, boundary));
        const std::vector<Segment> mine = segments_of(clipped);
        REQUIRE(mine.size() == cubic.size());
        // Point for point: every segment of mine has a counterpart within five scaled units at both ends.
        std::vector<bool> used(cubic.size(), false);
        for (size_t i = 0; i < mine.size(); ++i) {
            CAPTURE(i, unscale<double>(mine[i].a.x()), unscale<double>(mine[i].a.y()));
            bool matched = false;
            for (size_t j = 0; j < cubic.size() && !matched; ++j)
                if (!used[j] && (mine[i].a - cubic[j].a).cast<double>().norm() <= 5. && (mine[i].b - cubic[j].b).cast<double>().norm() <= 5.) {
                    used[j] = true;
                    matched = true;
                }
            CHECK(matched);
        }
    }
}

// L2: cell sections, height ranges, traces and crossings agree with brute-force plane–cube clipping and line intersection;
// refinement plane keys never alias a bulk plane or a face of their cell.
TEST_CASE("Bridge aware cubic lattice geometry agrees with brute force", "[BridgeAwareCubic][L2]")
{
    std::mt19937 rng(20261108);
    std::uniform_int_distribution<int> depth_of(0, 4), index_of(-6, 6), family_of_(0, 2);
    std::uniform_real_distribution<double> unit(0., 1.);
    const BoundingBox bbox(Point::new_scale(-32., -28.), Point::new_scale(32., 28.));
    for (const double density : {0.05, 0.03}) {
        const LatticeParams lattice = LatticeParams::make(45., density, 0.357, 0.4, 1, bbox);
        CHECK(lattice.a0 == Approx(unscale<double>(lattice.pitch) * std::sqrt(2. / 3.)));
        for (int trial = 0; trial < 200; ++trial) {
            Cell cell{ { index_of(rng), index_of(rng), index_of(rng) }, depth_of(rng) };
            CAPTURE(trial, cell.index[0], cell.index[1], cell.index[2], cell.depth);
            const auto corners = corners_of(lattice, cell);
            double z_min = corners[0].z, z_max = corners[0].z;
            for (const Corner &corner : corners) { z_min = std::min(z_min, corner.z); z_max = std::max(z_max, corner.z); }
            const auto range = height_range(lattice, cell);
            CHECK(range.first == Approx(z_min).margin(1e-9));
            CHECK(range.second == Approx(z_max).margin(1e-9));

            const double z = z_min + unit(rng) * (z_max - z_min);
            Points brute;
            for (int c = 0; c < 8; ++c)
                for (int bit = 0; bit < 3; ++bit) {
                    const int d = c | (1 << bit);
                    if (d == c) continue;
                    const Corner &p = corners[c], &q = corners[d];
                    if ((p.z <= z && q.z >= z) || (q.z <= z && p.z >= z)) {
                        const double f = std::abs(q.z - p.z) < 1e-12 ? 0. : (z - p.z) / (q.z - p.z);
                        const Vec2d point = p.p + f * (q.p - p.p);
                        brute.emplace_back(Point::new_scale(point.x(), point.y()) + lattice.center);
                    }
                }
            const Polygon hull = Geometry::convex_hull(brute);
            const Polygon sect = section(lattice, cell, z);
            // Vertices are rounded to scaled units: the areas differ by at most the perimeter times a unit.
            CHECK(std::abs(sect.area() - hull.area()) <= 2. * hull.length() + 1.);
            for (const Point &vertex : sect.points) {
                double nearest = std::numeric_limits<double>::max();
                for (const Point &candidate : hull.points)
                    nearest = std::min(nearest, (vertex - candidate).cast<double>().norm());
                CHECK(nearest <= 2.);    // two scaled units: the rounding of two computations
            }
            CHECK(section(lattice, cell, z_max + 1.).points.empty());

            // Traces: points of the trace lie on the plane.
            const int k = family_of_(rng);
            const PlaneKey plane{ k, cell.depth + 1, 2 * cell.index[k] + 1 };
            const Trace trace_k = trace(lattice, plane, z);
            const double a_plane = lattice.a0 / double(int64_t(1) << plane.depth);
            for (const double t : {-7., 0., 11.})
                CHECK(cube_coordinate(lattice, k, trace_k.origin + t * trace_k.dir, z) == Approx(plane.numerator * a_plane).margin(1e-9));

            // Crossings against a plane of another family: the brute-force intersection of the two traces, and its drift.
            const int j = (k + 1 + (trial % 2)) % 3;
            const PlaneKey other{ j, cell.depth, cell.index[j] };
            const auto cross = crossing(lattice, plane, other, z);
            REQUIRE(cross.has_value());
            const Trace trace_j = trace(lattice, other, z);
            const Vec2d at = trace_k.origin + cross->t * trace_k.dir;
            CHECK(std::abs(lattice.normal(j).dot(at - trace_j.origin)) <= 1e-9);
            const auto cross_up = crossing(lattice, plane, other, z + 1e-4);
            REQUIRE(cross_up.has_value());
            CHECK(cross->dt_dz == Approx((cross_up->t - cross->t) / 1e-4).margin(1e-6));
            CHECK(std::abs(cross->dt_dz) == Approx(std::sqrt(1.5)).margin(1e-9));

            // Refinement planes never alias a bulk plane or a face of their cell; children cover the parent.
            for (const PlaneKey &split : split_planes(cell)) {
                CHECK(split.depth == cell.depth + 1);
                CHECK(split.numerator % 2 != 0);
                for (const bool upper : {false, true}) {
                    const PlaneKey f = face(cell, split.family, upper);
                    CHECK_FALSE((f.depth == split.depth && f.numerator == split.numerator));
                }
            }
            double lo = std::numeric_limits<double>::max(), hi = std::numeric_limits<double>::lowest();
            for (const Cell &child : children(cell)) {
                const auto child_range = height_range(lattice, child);
                lo = std::min(lo, child_range.first);
                hi = std::max(hi, child_range.second);
            }
            CHECK(lo == Approx(range.first).margin(1e-9));
            CHECK(hi == Approx(range.second).margin(1e-9));
        }

        // The plane through a point: a refinement plane of the family within the tolerance, found at every level.
        for (int trial = 0; trial < 50; ++trial) {
            const Vec2d point(-20. + 40. * unit(rng), -15. + 30. * unit(rng));
            const double z = 18. * unit(rng);
            const int k = family_of_(rng);
            const auto found = plane_through(lattice, k, point, z, 0.0069);
            REQUIRE(found.has_value());
            CHECK(found->family == k);
            CHECK(found->depth >= 1);
            CHECK(found->depth <= 16);
            CHECK(found->numerator % 2 != 0);
            const Trace t = trace(lattice, *found, z);
            CHECK(std::abs(lattice.normal(k).dot(point - t.origin)) <= 0.0069 + 1e-9);
        }
    }
}
