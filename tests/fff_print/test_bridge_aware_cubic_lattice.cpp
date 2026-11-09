#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicLattice.hpp"
#include "libslic3r/Fill/FillRectilinear.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Surface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <set>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::BridgeAwareCubic;

namespace {

// A segment with its ends ordered, so that two sets of segments compare regardless of direction and order.
struct Segment {
    Point a, b;
    Segment(Point p, Point q) : a(p), b(q) { if (std::make_pair(a.x(), a.y()) > std::make_pair(b.x(), b.y())) std::swap(a, b); }
};

std::vector<Segment> segments_of(const Polylines &polylines)
{
    std::vector<Segment> out;
    for (const Polyline &polyline : polylines)
        for (size_t i = 1; i < polyline.points.size(); ++i)
            out.emplace_back(polyline.points[i - 1], polyline.points[i]);
    return out;
}

// The oracle's own view of the lattice, from FillCubic's definitions and nothing of the unit: θ_k = direction + π/2 + kπ/3,
// σ = (+1, −1, +1) for FillCubic's sweeps {0, +dx}, {π/3, −dx}, {2π/3, +dx}, cube coordinates u_k = √(2/3)(n_k · p) + σ_k z/√3.
struct Oracle {
    double theta[3];
    int    sigma[3] = { +1, -1, +1 };
    double a;          // cube edge, mm
    Vec2d  normal(int k) const { return Vec2d(std::cos(theta[k]), std::sin(theta[k])); }
    double u(int k, const Vec2d &p, double z) const { return std::sqrt(2. / 3.) * normal(k).dot(p) + sigma[k] * z / std::sqrt(3.); }
    Oracle(double direction_deg, double pitch_mm)
    {
        for (int k = 0; k < 3; ++k) theta[k] = Geometry::deg2rad(direction_deg) + M_PI / 2. + k * M_PI / 3.;
        a = pitch_mm * std::sqrt(2. / 3.);
    }
};

// The family of a segment: the direction it is parallel to.
int family_of(const Oracle &oracle, const Line &line)
{
    const Vec2d dir = (line.b - line.a).cast<double>().normalized();
    int best = 0;
    double best_dot = -1.;
    for (int k = 0; k < 3; ++k) {
        const Vec2d d(-std::sin(oracle.theta[k]), std::cos(oracle.theta[k]));
        const double dot = std::abs(dir.dot(d));
        if (dot > best_dot) { best_dot = dot; best = k; }
    }
    return best;
}

// A convex polygon (mm) clipped by the half plane n · p ≤ c.
std::vector<Vec2d> clip(const std::vector<Vec2d> &polygon, const Vec2d &n, double c)
{
    std::vector<Vec2d> out;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const Vec2d &p = polygon[i], &q = polygon[(i + 1) % polygon.size()];
        const double dp = n.dot(p) - c, dq = n.dot(q) - c;
        if (dp <= 0.) out.push_back(p);
        if ((dp <= 0.) != (dq <= 0.)) out.push_back(p + (dp / (dp - dq)) * (q - p));
    }
    return out;
}

// The section of a cell at height z by brute force: a large square clipped by the six half planes i_k a ≤ u_k ≤ (i_k + 1) a.
std::vector<Vec2d> brute_section(const Oracle &oracle, const Cell &cell, double z)
{
    const double a = oracle.a / double(int64_t(1) << cell.depth);
    std::vector<Vec2d> polygon = { Vec2d(-1000., -1000.), Vec2d(1000., -1000.), Vec2d(1000., 1000.), Vec2d(-1000., 1000.) };
    for (int k = 0; k < 3; ++k) {
        const Vec2d n = oracle.normal(k);
        const double scale = std::sqrt(2. / 3.), shift = oracle.sigma[k] * z / std::sqrt(3.);
        polygon = clip(polygon, n, ((cell.index[k] + 1) * a - shift) / scale);
        if (polygon.empty()) break;
        polygon = clip(polygon, -n, -(cell.index[k] * a - shift) / scale);
        if (polygon.empty()) break;
    }
    return polygon;
}

double polygon_area(const std::vector<Vec2d> &polygon)
{
    double twice = 0.;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const Vec2d &p = polygon[i], &q = polygon[(i + 1) % polygon.size()];
        twice += p.x() * q.y() - q.x() * p.y();
    }
    return std::abs(0.5 * twice);
}

} // namespace

// L1: the pattern's bulk lines are FillCubic's, point for point, for one, two and three lines, several directions (two outside
// [0, 360)), densities and heights, and a reference point off the origin; except within one line width of the rotated bounding
// box's extremes, which make_fill_lines drops and the pattern keeps (R5). Each of Cubic's lines also lies on the trace of a
// bulk plane of the unit, so the plane geometry is pinned to Cubic's lines too.
TEST_CASE("Bridge aware cubic bulk lines equal FillCubic's point for point", "[BridgeAwareCubic][L1]")
{
    const double s_f = 0.357, w = 0.4;
    for (const int N : {1, 2, 3})
    for (const double direction : {0., 45., 17., 405., -30.})
    for (const float density : {0.05f, 0.12f, 0.03f})
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
        params.density           = density;
        params.multiline         = N;
        params.anchor_length     = 0.f;
        params.anchor_length_max = 0.f;
        Surface surface(stInternal, rectangle);
        const std::vector<Segment> cubic = segments_of(fill.fill_surface(&surface, params));
        REQUIRE_FALSE(cubic.empty());

        const LatticeParams lattice = LatticeParams::make(direction, density, s_f, w, N, bbox);
        const Lines lines = bulk_lines(lattice, z, get_extents(rectangle));
        // FillCubic's pitch, as its code computes it, for the oracle.
        const double pitch_mm = unscale<double>(coord_t(scale_(s_f) * N / double(float(double(density) / 3.))));
        const Oracle oracle(direction, pitch_mm);
        // Clip one line at a time, as fill_surface_by_multilines does, and drop the extreme lines as make_fill_lines does, in
        // the frame make_fill_lines uses (rotated by the float angle).
        const coord_t aoffset = coord_t(float(scale_(-0.5 * s_f)));
        const Polygons boundary = offset(to_polygons(rectangle), float(aoffset), ClipperLib::jtMiter, 3.);
        const coord_t x_margin = coord_t(scale_(s_f)) + coord_t(SCALED_EPSILON);
        Polylines clipped;
        for (const Line &line : lines) {
            const int k = family_of(oracle, line);
            const float angle = float(lattice.theta_fill[k]);
            Polygon rotated = rectangle.contour;
            rotated.rotate(-angle);
            const BoundingBox rotated_bbox = rotated.bounding_box();
            const coord_t x = line.a.rotated(-angle).x();
            if (x < rotated_bbox.min.x() + x_margin || x > rotated_bbox.max.x() - x_margin)
                continue;
            append(clipped, intersection_pl(Polylines{ Polyline(line.a, line.b) }, boundary));
        }
        const std::vector<Segment> mine = segments_of(clipped);
        REQUIRE(mine.size() == cubic.size());
        // Point for point: every segment of mine has a counterpart within five scaled units at both ends, either way round.
        std::vector<bool> used(cubic.size(), false);
        for (size_t i = 0; i < mine.size(); ++i) {
            CAPTURE(i, unscale<double>(mine[i].a.x()), unscale<double>(mine[i].a.y()));
            bool matched = false;
            for (size_t j = 0; j < cubic.size() && !matched; ++j) {
                if (used[j]) continue;
                const bool same = (mine[i].a - cubic[j].a).cast<double>().norm() <= 5. && (mine[i].b - cubic[j].b).cast<double>().norm() <= 5.;
                const bool swapped = (mine[i].a - cubic[j].b).cast<double>().norm() <= 5. && (mine[i].b - cubic[j].a).cast<double>().norm() <= 5.;
                if (same || swapped) { used[j] = true; matched = true; }
            }
            CHECK(matched);
        }
        // Every Cubic line lies on the trace of a bulk plane, shifted by a sub-line offset: n_k · p + σ_k z/√2 − o_i = m L for an
        // integer m, within three units (FillCubic truncates the float shift and each sub-line's offset toward zero, and rounds
        // the endpoints) plus the float angles' drift over the distance from the centre. Adjacent lines of one family are a
        // pitch apart.
        std::array<std::vector<double>, 3> positions;
        for (const Segment &s : cubic) {
            const Line line(s.a, s.b);
            const int k = family_of(oracle, line);
            const Vec2d mid = 0.5 * (s.a + s.b).cast<double>();
            const Vec2d rel = (mid - bbox.center().cast<double>()) * SCALING_FACTOR;
            const double value = oracle.normal(k).dot(rel) + oracle.sigma[k] * z / std::sqrt(2.);
            double best = std::numeric_limits<double>::max();
            for (const double offset : lattice.subline_offsets()) {
                const double m = std::round((value - offset) / pitch_mm);
                best = std::min(best, std::abs(value - offset - m * pitch_mm));
            }
            CHECK(best <= 3. * SCALING_FACTOR + rel.norm() * 1e-6);
            positions[k].push_back(oracle.normal(k).dot(rel));
        }
        if (N == 1)
            for (int k = 0; k < 3; ++k) {
                std::sort(positions[k].begin(), positions[k].end());
                for (size_t i = 1; i < positions[k].size(); ++i) {
                    const double gap = positions[k][i] - positions[k][i - 1];
                    if (gap > 1e-6)
                        CHECK(std::abs(gap - std::round(gap / pitch_mm) * pitch_mm) <= 2. * SCALING_FACTOR + 60. * 1e-6);
                }
            }
    }
}

// L2: cell sections, height ranges, traces, crossings, roots and the plane through a point agree with brute force from
// FillCubic's definitions; refinement plane keys never alias a bulk plane or a face of their cell.
TEST_CASE("Bridge aware cubic lattice geometry agrees with brute force", "[BridgeAwareCubic][L2]")
{
    std::mt19937 rng(20261108);
    std::uniform_int_distribution<int> depth_of(0, 4), index_of(-6, 6), family_of_(0, 2);
    std::uniform_real_distribution<double> unit(0., 1.);
    const BoundingBox bbox(Point::new_scale(-32., -28.), Point::new_scale(32., 28.));
    const double w = 0.4, c = w / std::sqrt(3.), tau = (c - 0.5 * w) * std::sin(M_PI / 3.) / 4.;
    for (const float density : {0.05f, 0.03f}) {
        const LatticeParams lattice = LatticeParams::make(45., density, 0.357, w, 1, bbox);
        const Oracle oracle(45., unscale<double>(lattice.pitch));
        for (int trial = 0; trial < 200; ++trial) {
            Cell cell{ { index_of(rng), index_of(rng), index_of(rng) }, depth_of(rng) };
            CAPTURE(trial, cell.index[0], cell.index[1], cell.index[2], cell.depth);
            // Height range: the extreme heights of the eight corners, from z = Σ σ_k u_k / √3.
            const double a = oracle.a / double(int64_t(1) << cell.depth);
            double z_min = std::numeric_limits<double>::max(), z_max = std::numeric_limits<double>::lowest();
            for (int corner = 0; corner < 8; ++corner) {
                double z = 0.;
                for (int k = 0; k < 3; ++k) z += oracle.sigma[k] * (cell.index[k] + ((corner >> k) & 1)) * a / std::sqrt(3.);
                z_min = std::min(z_min, z);
                z_max = std::max(z_max, z);
            }
            const auto range = height_range(lattice, cell);
            CHECK(range.first == Approx(z_min).margin(1e-9));
            CHECK(range.second == Approx(z_max).margin(1e-9));

            // Section: the half-plane clipping against the unit's polygon.
            const double z = z_min + (0.05 + 0.9 * unit(rng)) * (z_max - z_min);
            const std::vector<Vec2d> brute = brute_section(oracle, cell, z);
            const Polygon sect = section(lattice, cell, z);
            REQUIRE(brute.size() >= 3);
            REQUIRE(sect.points.size() >= 3);
            CHECK(std::abs(sect.area() * SCALING_FACTOR * SCALING_FACTOR - polygon_area(brute)) <= 2. * SCALING_FACTOR * unscale<double>(sect.length()) + 1e-12);
            for (const Point &vertex : sect.points) {
                const Vec2d v = (vertex - lattice.center).cast<double>() * SCALING_FACTOR;
                double nearest = std::numeric_limits<double>::max();
                for (const Vec2d &b : brute) nearest = std::min(nearest, (v - b).norm());
                CHECK(nearest <= 2. * SCALING_FACTOR);
            }
            for (const Vec2d &b : brute) {
                double nearest = std::numeric_limits<double>::max();
                for (const Point &vertex : sect.points) nearest = std::min(nearest, (b - (vertex - lattice.center).cast<double>() * SCALING_FACTOR).norm());
                CHECK(nearest <= 2. * SCALING_FACTOR);
            }
            CHECK(section(lattice, cell, z_max + 1.).points.empty());

            // Traces: points of the trace lie on the plane, by the oracle's cube coordinate.
            const int k = family_of_(rng);
            const PlaneKey plane{ k, cell.depth + 1, 2 * cell.index[k] + 1 };
            const Trace trace_k = trace(lattice, plane, z);
            const double a_plane = oracle.a / double(int64_t(1) << plane.depth);
            for (const double t : {-7., 0., 11.})
                CHECK(oracle.u(k, trace_k.point(t), z) == Approx(plane.numerator * a_plane).margin(1e-9));

            // Crossings against a plane of another family: the brute-force intersection of the two traces, and its drift.
            const int j = (k + 1 + (trial % 2)) % 3;
            const PlaneKey other{ j, cell.depth, cell.index[j] };
            const auto cross = crossing(lattice, plane, other, z);
            REQUIRE(cross.has_value());
            const Vec2d at = trace_k.point(cross->t);
            CHECK(oracle.u(j, at, z) == Approx(other.numerator * a).margin(1e-9));
            const auto cross_up = crossing(lattice, plane, other, z + 1e-4);
            REQUIRE(cross_up.has_value());
            CHECK(cross->dt_dz == Approx((cross_up->t - cross->t) / 1e-4).margin(1e-6));
            CHECK(std::abs(cross->dt_dz) == Approx(std::sqrt(1.5)).margin(1e-9));

            // Children cover the parent.
            double lo = std::numeric_limits<double>::max(), hi = std::numeric_limits<double>::lowest();
            for (const Cell &child : children(cell)) {
                const auto child_range = height_range(lattice, child);
                lo = std::min(lo, child_range.first);
                hi = std::max(hi, child_range.second);
            }
            CHECK(lo == Approx(range.first).margin(1e-9));
            CHECK(hi == Approx(range.second).margin(1e-9));
        }

        // Refinement planes never alias: 1,000 random cells; the split planes are odd one level down and lie midway between the
        // cell's faces, whose canonical keys compare by the plane they name.
        for (int trial = 0; trial < 1000; ++trial) {
            Cell cell{ { index_of(rng), index_of(rng), index_of(rng) }, depth_of(rng) };
            const double a_child = oracle.a / double(int64_t(1) << (cell.depth + 1));
            for (const PlaneKey &split : split_planes(cell)) {
                CHECK(split.depth == cell.depth + 1);
                CHECK(split.numerator % 2 != 0);
                const double u_split = split.numerator * a_child;
                for (const bool upper : {false, true}) {
                    const PlaneKey f = face(cell, split.family, upper);
                    CHECK((f.depth == 0 || f.numerator % 2 != 0));    // canonical: a face of a coarser plane carries that plane's key
                    const double u_face = f.numerator * oracle.a / double(int64_t(1) << f.depth);
                    CHECK(std::abs(u_face - u_split) == Approx(a_child).margin(1e-9));
                }
            }
        }

        // Roots: every cell that a point of the box at height z lies in is a root.
        for (const double z : {0.2, 7.3, 17.8}) {
            const std::vector<Cell> found = roots(lattice, bbox, z);
            std::set<std::tuple<int64_t, int64_t, int64_t>> root_set;
            for (const Cell &cell : found) root_set.emplace(cell.index[0], cell.index[1], cell.index[2]);
            for (int ix = 0; ix <= 20; ++ix)
                for (int iy = 0; iy <= 20; ++iy) {
                    const Vec2d p(-32. + 64. * ix / 20., -28. + 56. * iy / 20.);
                    std::tuple<int64_t, int64_t, int64_t> cell{ int64_t(std::floor(oracle.u(0, p, z) / oracle.a)), int64_t(std::floor(oracle.u(1, p, z) / oracle.a)), int64_t(std::floor(oracle.u(2, p, z) / oracle.a)) };
                    CHECK(root_set.count(cell) == 1);
                }
        }

        // The plane through a point: the coarsest refinement plane of the family within τ, never a bulk plane; nothing when a
        // bulk plane is the one within τ.
        for (int trial = 0; trial < 200; ++trial) {
            const Vec2d point(-20. + 40. * unit(rng), -15. + 30. * unit(rng));
            const double z = 18. * unit(rng);
            const int k = family_of_(rng);
            const double u = oracle.u(k, point, z);
            const auto found = plane_through(lattice, k, point, z, tau);
            // The nearest plane of each level, coarsest first: the first within τ decides.
            std::optional<std::pair<int, int64_t>> expected;
            for (int depth = 0; depth <= 40 && !expected; ++depth) {
                const double a_level = oracle.a / double(int64_t(1) << depth);
                const int64_t q = int64_t(std::llround(u / a_level));
                if (std::sqrt(1.5) * std::abs(u - q * a_level) <= tau) expected = std::make_pair(depth, q);
            }
            REQUIRE(expected.has_value());
            if (expected->first == 0) {
                CHECK_FALSE(found.has_value());
            } else {
                REQUIRE(found.has_value());
                CHECK(found->family == k);
                CHECK(found->depth == expected->first);
                CHECK(found->numerator == expected->second);
                CHECK(found->numerator % 2 != 0);
            }
        }
    }
}
