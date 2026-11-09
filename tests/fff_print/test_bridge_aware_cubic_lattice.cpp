#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicLattice.hpp"
#include "libslic3r/Fill/FillRectilinear.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Surface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
