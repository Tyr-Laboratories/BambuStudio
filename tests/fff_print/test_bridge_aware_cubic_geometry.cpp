#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/BridgeAwareCubic.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Surface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;

namespace {

ExPolygon rectangle(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon({ Point::new_scale(x0, y0), Point::new_scale(x1, y0),
                               Point::new_scale(x1, y1), Point::new_scale(x0, y1) }));
}

BAC::Parameters parameters()
{
    BAC::Parameters p;
    p.bounds = BoundingBoxf3(Vec3d(-60., -60., 0.), Vec3d(60., 60., 50.));
    p.flow_spacing = .45;
    p.density = .05;
    return p;
}

double diameter(const Polygon &polygon)
{
    double diameter = 0.;
    for (const Point &a : polygon.points)
        for (const Point &b : polygon.points)
            diameter = std::max(diameter, unscale<double>((a - b).cast<double>().norm()));
    return diameter;
}

double length(const Polylines &lines)
{
    double value = 0.;
    for (const Polyline &line : lines)
        value += unscale<double>(line.length());
    return value;
}

void check_demanded_cells(const BAC::Plan &plan, const BAC::Demand &demand)
{
    size_t touched = 0;
    for (const BAC::LeafSection &leaf : plan.leaf_sections(demand.z)) {
        if (!intersection_ex(ExPolygon(leaf.polygon), demand.footprint).empty()) {
            ++touched;
            REQUIRE(diameter(leaf.polygon) <= demand.gap);
        }
    }
    REQUIRE(touched > 0);
}

} // namespace

TEST_CASE("Bridge aware cubic bulk retains the ordinary cubic pitch and phase", "[bridge_aware_cubic][geometry]")
{
    const BAC::Plan plan(parameters(), {});
    const ExPolygon domain = rectangle(-60., -60., 60., 60.);
    REQUIRE(plan.statistics().splits == 0);
    REQUIRE(plan.bulk_cell_edge() == Approx(22.045407685));
    for (const double z : { 1.3, 8.7, 26.1 }) {
        const Polylines lines = plan.lines(z, domain);
        REQUIRE_FALSE(lines.empty());
        REQUIRE(plan.refinement_lines(z, domain).empty());
        REQUIRE(plan.refinement_area(z).empty());
        std::vector<double> horizontal;
        for (const Polyline &line : lines) {
            REQUIRE(line.points.size() == 2);
            if (std::abs(line.front().y() - line.back().y()) <= 2)
                horizontal.push_back(unscale<double>(line.front().y()));
        }
        std::sort(horizontal.begin(), horizontal.end());
        REQUIRE(horizontal.size() >= 4);
        for (size_t i = 1; i < horizontal.size(); ++i)
            REQUIRE(horizontal[i] - horizontal[i - 1] == Approx(27.).margin(.000005));
        for (double y : horizontal)
            REQUIRE(std::remainder(y + z / std::sqrt(2.), 27.) == Approx(0.).margin(2. * SCALING_FACTOR));
    }
}

TEST_CASE("Bridge aware cubic refines each child against the line footprint", "[bridge_aware_cubic][geometry]")
{
    const BAC::Demand line { { rectangle(-25., 4.8, 25., 5.2) }, 21., 3.5 };
    const BAC::Demand roof { { rectangle(-25., -25., 25., 25.) }, 21., 3.5 };
    const BAC::Plan local(parameters(), { line });
    const BAC::Plan uniform(parameters(), { roof });
    check_demanded_cells(local, line);
    REQUIRE(local.statistics().splits > 0);
    REQUIRE(local.statistics().splits < uniform.statistics().splits / 2);
    REQUIRE(length(local.refinement_lines(21., roof.footprint.front())) <
            length(uniform.refinement_lines(21., roof.footprint.front())));
    bool coarse_sibling = false;
    for (const BAC::LeafSection &leaf : local.leaf_sections(21.))
        if (leaf.depth == 1 && diameter(leaf.polygon) > line.gap &&
            intersection_ex(ExPolygon(leaf.polygon), line.footprint).empty())
            coarse_sibling = true;
    REQUIRE(coarse_sibling);
}

TEST_CASE("Bridge aware cubic preserves the space between separate demands", "[bridge_aware_cubic][geometry]")
{
    const BAC::Demand left { { rectangle(-20.2, -22., -19.8, 22.) }, 20., 3.5 };
    const BAC::Demand right { { rectangle(19.8, -22., 20.2, 22.) }, 20., 3.5 };
    const BAC::Plan plan(parameters(), { left, right });
    const BAC::Plan filled(parameters(), { { { rectangle(-20.2, -22., 20.2, 22.) }, 20., 3.5 } });
    check_demanded_cells(plan, left);
    check_demanded_cells(plan, right);
    REQUIRE(plan.statistics().splits < filled.statistics().splits / 2);
    bool coarse_middle = false;
    for (const BAC::LeafSection &leaf : plan.leaf_sections(20.))
        if (leaf.polygon.contains(Point::new_scale(0., 0.)) && diameter(leaf.polygon) > 3.5)
            coarse_middle = true;
    REQUIRE(coarse_middle);
}

TEST_CASE("Bridge aware cubic stops at a small actual triangular section", "[bridge_aware_cubic][geometry]")
{
    const BAC::Demand demand { { rectangle(-.05, -.05, .05, .05) }, 1., 5. };
    const BAC::Plan plan(parameters(), { demand });
    REQUIRE(plan.bulk_cell_edge() * std::sqrt(2.) > demand.gap);
    REQUIRE(plan.statistics().splits == 0);
    check_demanded_cells(plan, demand);
}

TEST_CASE("Bridge aware cubic rebuild releases obsolete branches and follows rotated demand", "[bridge_aware_cubic][geometry]")
{
    BAC::Parameters p = parameters();
    p.angle = .37;
    const BAC::Demand first { { rectangle(-20., -10.2, 20., -9.8) }, 17.4, 4. };
    const BAC::Demand last { { rectangle(-20., 9.8, 20., 10.2) }, 23.7, 4. };
    const BAC::Plan old(p, { first });
    const BAC::Plan changed(p, { last });
    const BAC::Plan fresh(p, { last });
    const BAC::Plan cleared(p, {});
    check_demanded_cells(old, first);
    check_demanded_cells(changed, last);
    REQUIRE(changed.statistics().nodes == fresh.statistics().nodes);
    const Polylines a = changed.lines(20., rectangle(-30., -30., 30., 30.));
    const Polylines b = fresh.lines(20., rectangle(-30., -30., 30., 30.));
    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i)
        REQUIRE(a[i].points == b[i].points);
    REQUIRE(cleared.statistics().splits == 0);
}

TEST_CASE("Bridge aware cubic partitions retain their natural births and domain holes", "[bridge_aware_cubic][geometry]")
{
    const BAC::Demand demand { { rectangle(1.9, .9, 2.1, 1.1) }, 19., 10. };
    const BAC::Plan plan(parameters(), { demand });
    const ExPolygon domain = rectangle(-30., -30., 30., 30.);
    REQUIRE(plan.statistics().splits > 0);
    // The first split of [0,a]^3 begins at a/(2 sqrt(3)) = 6.363961 mm.
    REQUIRE(plan.refinement_lines(6.3, domain).empty());
    REQUIRE_FALSE(plan.refinement_lines(6.5, domain).empty());
    REQUIRE_FALSE(plan.refinement_area(6.5).empty());
    REQUIRE(plan.refinement_area(6.3).empty());
    REQUIRE(length(plan.refinement_lines(19., domain)) > 10.);
    REQUIRE_FALSE(diff_pl(plan.refinement_lines(19., domain), demand.footprint).empty());
    ExPolygon holed = domain;
    Polygon hole = rectangle(-4., -4., 4., 4.).contour;
    hole.make_clockwise();
    holed.holes.push_back(hole);
    REQUIRE(intersection_pl(plan.lines(19., holed), rectangle(-3.9, -3.9, 3.9, 3.9)).empty());
}

TEST_CASE("Bridge aware cubic rejects invalid and unbounded construction", "[bridge_aware_cubic][geometry]")
{
    BAC::Parameters p = parameters();
    p.density = 0.;
    REQUIRE_THROWS_AS(BAC::Plan(p, {}), std::invalid_argument);
    p = parameters();
    p.angle = std::numeric_limits<double>::infinity();
    REQUIRE_THROWS_AS(BAC::Plan(p, {}), std::invalid_argument);
    p = parameters();
    REQUIRE_THROWS_AS(BAC::Plan(p, { { { rectangle(1., 1., 2., 2.) }, 10., 0. } }), std::invalid_argument);
    REQUIRE_THROWS_AS(BAC::Plan(p, { { {}, 10., std::numeric_limits<double>::max() } }), std::invalid_argument);
    REQUIRE_THROWS_AS(BAC::Plan(p, { { {}, p.bounds.max.z() + SCALING_FACTOR, 5. } }), std::invalid_argument);
    REQUIRE_THROWS_AS(BAC::Plan(p, { { {}, p.bounds.min.z() - SCALING_FACTOR, 5. } }), std::invalid_argument);
    p.max_nodes = 1;
    REQUIRE_THROWS_AS(BAC::Plan(p, {}), std::runtime_error);
    p = parameters();
    p.max_depth = 1;
    REQUIRE_THROWS_AS(BAC::Plan(p, { { { rectangle(-10., -10., 10., 10.) }, 20., .01 } }), std::runtime_error);
    p = parameters();
    p.throw_if_canceled = [] { throw std::runtime_error("cancel geometry"); };
    REQUIRE_THROWS_WITH(BAC::Plan(p, {}), "cancel geometry");
}

TEST_CASE("Bridge aware cubic bulk agrees with ordinary cubic after rotation and translation", "[bridge_aware_cubic][geometry]")
{
    BAC::Parameters p = parameters();
    p.bounds.translate(Vec3d(17., -11., 0.));
    const ExPolygon domain = rectangle(-43., -71., 77., 49.);
    const ExPolygon interior = rectangle(-33., -61., 67., 39.);
    Surface surface(stInternal, domain);
    std::unique_ptr<Fill> ordinary(Fill::new_from_type(ipCubic));
    ordinary->bounding_box = get_extents(domain);
    ordinary->spacing = p.flow_spacing;
    ordinary->overlap = .5 * p.flow_spacing;
    FillParams fill;
    fill.density = float(p.density);
    fill.anchor_length_max = 0.;
    fill.anchor_length = 0.;
    for (double angle : { 0., .37, 1.2 }) {
        p.angle = angle;
        ordinary->angle = float(angle);
        const BAC::Plan plan(p, {});
        for (double z : { 1.3, 8.7, 26.1 }) {
            ordinary->z = z;
            const Polylines legacy = intersection_pl(ordinary->fill_surface(&surface, fill), interior);
            const Polylines current = plan.lines(z, interior);
            CAPTURE(angle, z, legacy.size(), current.size());
            REQUIRE_FALSE(legacy.empty());
            REQUIRE_FALSE(current.empty());
            // Legacy sweeps round their angles and pitch through float and
            // scaled integer arithmetic; allow 1 micron for that difference
            // in every direction, including at clipped boundary endpoints.
            // Butt caps would impose zero longitudinal endpoint tolerance.
            const Polylines legacy_missing = diff_pl(legacy, offset(current, float(scale_(.001)),
                ClipperLib::jtRound, 0., ClipperLib::etOpenRound));
            const Polylines current_missing = diff_pl(current, offset(legacy, float(scale_(.001)),
                ClipperLib::jtRound, 0., ClipperLib::etOpenRound));
            CAPTURE(length(legacy_missing), length(current_missing));
            std::ostringstream geometry;
            if (!legacy_missing.empty() || !current_missing.empty()) {
                for (const Polyline &line : legacy_missing)
                    geometry << "legacy missing " << line.front() << " -> " << line.back() << '\n';
                for (const Polyline &line : current_missing)
                    geometry << "current missing " << line.front() << " -> " << line.back() << '\n';
            }
            INFO(geometry.str());
            REQUIRE(legacy_missing.empty());
            REQUIRE(current_missing.empty());
        }
    }
}

TEST_CASE("Bridge aware cubic follows diagonal and curved finite-width footprints", "[bridge_aware_cubic][geometry]")
{
    ExPolygon diagonal = rectangle(-25., -.2, 25., .2);
    diagonal.rotate(.6, Point(0, 0));
    Polyline arc;
    for (unsigned i = 0; i <= 12; ++i) {
        const double angle = .3 + i * .1;
        arc.points.push_back(Point::new_scale(20. * std::cos(angle), 20. * std::sin(angle)));
    }
    const BAC::Demand diagonal_demand { { diagonal }, 16.7, 3.5 };
    const BAC::Demand curved_demand { union_ex(offset(arc, float(scale_(.2)))), 24.1, 3.5 };
    const BAC::Plan local(parameters(), { diagonal_demand, curved_demand });
    check_demanded_cells(local, diagonal_demand);
    check_demanded_cells(local, curved_demand);
    const BAC::Plan envelope(parameters(), { { { rectangle(-25., -25., 25., 25.) }, 16.7, 3.5 },
                                             { { rectangle(-25., -25., 25., 25.) }, 24.1, 3.5 } });
    REQUIRE(local.statistics().splits < envelope.statistics().splits / 2);
}

TEST_CASE("Bridge aware cubic rejects dense saturation instead of overlapping line walls", "[bridge_aware_cubic][geometry]")
{
    BAC::Parameters p = parameters();
    p.density = .2;
    const BAC::Demand demand { { rectangle(-10., -10., 10., 10.) }, 20., 1.5 };
    REQUIRE_THROWS_WITH(BAC::Plan(p, { demand }), Catch::Contains("dense saturation"));
}

TEST_CASE("Bridge aware cubic clips bulk before scaled coordinate conversion", "[bridge_aware_cubic][geometry]")
{
    BAC::Parameters p = parameters();
    p.bounds = BoundingBoxf3(Vec3d(21400., -30., 0.), Vec3d(21460., 30., 50.));
    const ExPolygon domain = rectangle(21400., -30., 21460., 30.);
    const BAC::Plan plan(p, {});
    const Polylines lines = plan.lines(10., domain);
    REQUIRE_FALSE(lines.empty());
    REQUIRE(diff_pl(lines, domain).empty());
}

TEST_CASE("Bridge aware cubic validates support footprint holes", "[bridge_aware_cubic][geometry]")
{
    ExPolygon footprint = rectangle(-25., -25., 25., 25.);
    Polygon hole = rectangle(-20., -20., 20., 20.).contour;
    footprint.holes.push_back(hole);
    REQUIRE_THROWS_AS(BAC::Plan(parameters(), { { { footprint }, 20., 5. } }), std::invalid_argument);
    footprint.holes.front().make_clockwise();
    const BAC::Plan valid(parameters(), { { { footprint }, 20., 5. } });
    bool coarse_inside_hole = false;
    for (const BAC::LeafSection &leaf : valid.leaf_sections(20.))
        if (leaf.polygon.contains(Point::new_scale(0., 0.)) && diameter(leaf.polygon) > 5.)
            coarse_inside_hole = true;
    REQUIRE(coarse_inside_hole);
    footprint.holes.front().points.resize(2);
    REQUIRE_THROWS_AS(BAC::Plan(parameters(), { { { footprint }, 20., 5. } }), std::invalid_argument);
}

TEST_CASE("Bridge aware cubic merges shared partition intervals without crossing holes", "[bridge_aware_cubic][geometry]")
{
    const BAC::Plan plan(parameters(), { { { rectangle(-30., -30., 30., 30.) }, 20., 8. } });
    ExPolygon domain = rectangle(-40., -40., 40., 40.);
    Polygon hole = rectangle(-3., -3., 3., 3.).contour;
    hole.make_clockwise();
    domain.holes.push_back(hole);
    const Polylines lines = plan.refinement_lines(20., domain);
    REQUIRE_FALSE(lines.empty());
    REQUIRE(intersection_pl(lines, rectangle(-2.9, -2.9, 2.9, 2.9)).empty());
    for (size_t i = 0; i < lines.size(); ++i) {
        const Vec2d a = unscaled(lines[i].front());
        const Vec2d b = unscaled(lines[i].back());
        const double size = (b - a).norm();
        if (size < .001)
            continue;
        const Vec2d along = (b - a) / size;
        const Vec2d normal(-along.y(), along.x());
        for (size_t j = i + 1; j < lines.size(); ++j) {
            const Vec2d c = unscaled(lines[j].front());
            const Vec2d d = unscaled(lines[j].back());
            if (std::abs(normal.dot(c - a)) > 2. * SCALING_FACTOR ||
                    std::abs(normal.dot(d - a)) > 2. * SCALING_FACTOR)
                continue;
            double first = along.dot(c - a);
            double last = along.dot(d - a);
            if (first > last)
                std::swap(first, last);
            // Shared endpoints on the same uninterrupted plane should have
            // been merged as well as any coincident positive-length interval.
            const double overlap = std::min(size, last) - std::max(0., first);
            REQUIRE(overlap < -2. * SCALING_FACTOR);
        }
    }
}

TEST_CASE("Bridge aware cubic keeps each gap local to its own demand elevation", "[bridge_aware_cubic][geometry]")
{
    const BAC::Demand coarse { { rectangle(-35., -10., -25., 10.) }, 15., 8. };
    const BAC::Demand fine { { rectangle(25., -10., 35., 10.) }, 30., 3.5 };
    const BAC::Plan plan(parameters(), { coarse, fine });
    check_demanded_cells(plan, coarse);
    check_demanded_cells(plan, fine);
    bool stopped_above_unrelated_gap = false;
    for (const BAC::LeafSection &leaf : plan.leaf_sections(coarse.z))
        if (diameter(leaf.polygon) > fine.gap &&
                !intersection_ex(ExPolygon(leaf.polygon), coarse.footprint).empty())
            stopped_above_unrelated_gap = true;
    REQUIRE(stopped_above_unrelated_gap);
}
