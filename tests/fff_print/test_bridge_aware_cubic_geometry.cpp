#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/BridgeAwareCubic.hpp"
#include "libslic3r/Fill/BridgeAwareCubicClipMask.hpp"
#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Fill/FillBridgeAwareCubic.hpp"
#include "libslic3r/Surface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
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

TEST_CASE("Bridge aware cubic exposes only demanded saturated leaves for solid construction", "[bridge_aware_cubic][geometry]")
{
    BAC::Parameters p = parameters();
    p.allow_solid_fallback = true;
    const BAC::Demand demand { { rectangle(1.9, .9, 2.1, 1.1) }, 19., .2 };
    const BAC::Plan local(p, { demand });
    const BAC::Plan envelope(p, { { { rectangle(-20., -20., 20., 20.) }, 19., .2 } });
    REQUIRE(local.statistics().solid_cells > 0);
    REQUIRE(local.statistics().solid_cells < envelope.statistics().solid_cells / 2);
    for (const BAC::Refinement &refinement : local.refinements()) {
        REQUIRE(std::ldexp(p.density, int(refinement.id.depth)) < 1.);
        REQUIRE_FALSE(refinement.demand_indices.empty());
        if (refinement.solid) {
            REQUIRE(refinement.bottom_z < refinement.top_z);
            REQUIRE_FALSE(local.solid_area(.5 * (refinement.bottom_z + refinement.top_z)).empty());
        }
    }
    const auto solids = local.solid_sections(demand.z);
    REQUIRE_FALSE(solids.empty());
    for (const auto &solid : solids) REQUIRE(solid.solid);
    const ExPolygons mask = local.solid_area(demand.z);
    REQUIRE_FALSE(intersection_ex(mask, demand.footprint).empty());
    REQUIRE(intersection_pl(local.lines(demand.z, rectangle(-40., -40., 40., 40.)),
                            offset_ex(mask, -1.f)).empty());
}

TEST_CASE("Bridge aware cubic partition ownership follows merged emission at solid boundaries",
          "[bridge_aware_cubic][geometry][partition_ownership]")
{
    BAC::Parameters p;
    p.bounds = BoundingBoxf3(Vec3d(-14., -12., 0.), Vec3d(14., 12., 14.));
    p.flow_spacing = double(float(.4)) - double(float(.2)) * (1. - M_PI / 4.);
    p.density = .12;
    p.angle = 17. * M_PI / 180.;
    p.allow_solid_fallback = true;
    const ExPolygon domain = rectangle(-14., -12., 14., 12.);
    const double z = 8.6;
    // A saturated child shares a face with its parent's added partition.
    // The long merged face and independently rounded parent interval can lie
    // on opposite sides of that same solid boundary by less than a grid unit.
    const BAC::Plan plan(p, {{ {domain}, z, 7. },
        { {rectangle(-.653, 2.133, -.652, 2.135)}, z, .05 }});
    REQUIRE(plan.statistics().solid_cells > 0);
    const Polylines emitted = plan.refinement_lines(z, domain);
    REQUIRE_FALSE(emitted.empty());
    Polylines owned;
    for (const BAC::Refinement &refinement : plan.refinements()) {
        if (refinement.solid) continue;
        const Polylines section = plan.partition_lines(refinement.id, z, domain);
        for (const Polyline &line : section) owned.push_back(line);
        // An owner must never manufacture a segment removed from emission.
        CHECK(diff_pl(section, offset(emitted, 2., ClipperLib::jtRound, 0.,
                                      ClipperLib::etOpenRound)).empty());
    }
    REQUIRE_FALSE(owned.empty());
    // All surviving added material retains its actual producing cell, even
    // when a local solid-fill reservation touches its inherited boundary.
    CHECK(diff_pl(emitted, offset(owned, 2., ClipperLib::jtRound, 0.,
                                 ClipperLib::etOpenRound)).empty());
}

TEST_CASE("Bridge aware cubic solid clipping is independent of the caller fill domain",
          "[bridge_aware_cubic][geometry][partition_ownership]")
{
    BAC::Parameters p;
    p.bounds = BoundingBoxf3(Vec3d(-24., -14., 0.), Vec3d(24., 14., 20.));
    p.flow_spacing = double(float(.4)) - double(float(.2)) * (1. - M_PI / 4.);
    p.density = .15;
    p.angle = M_PI / 4.;
    p.allow_solid_fallback = true;
    const double z = 13.4;
    const ExPolygon whole = rectangle(-24., -14., 24., 14.);
    ExPolygon local = rectangle(-21.31795, -7.31795, -4.68205, 7.31795);
    Polygon hole = rectangle(-17.68205, -3.68205, -10.31795, 3.68205).contour;
    hole.reverse();
    local.holes.push_back(std::move(hole));
    const BAC::Plan plan(p, {{{whole}, z, 6.},
        {{rectangle(-10.089, -5.914, -10.088, -5.913)}, z, .05}});
    REQUIRE(plan.statistics().solid_cells > 0);
    const Polylines broad = plan.refinement_lines(z, whole);
    const Polylines restricted = intersection_pl(broad, local);
    const Polylines emitted = plan.refinement_lines(z, local);
    REQUIRE_FALSE(emitted.empty());
    // A rounded domain endpoint must not change which side of an almost
    // collinear solid boundary a complete refinement face occupies.
    CHECK(diff_pl(emitted, offset(restricted, 2., ClipperLib::jtRound, 0., ClipperLib::etOpenRound)).empty());
    CHECK(diff_pl(restricted, offset(emitted, 2., ClipperLib::jtRound, 0., ClipperLib::etOpenRound)).empty());
    Polylines owners;
    for (const BAC::Refinement &refinement : plan.refinements()) {
        if (refinement.solid) continue;
        const Polylines section = plan.partition_lines(refinement.id, z, whole);
        owners.insert(owners.end(), section.begin(), section.end());
    }
    CHECK(diff_pl(emitted, offset(owners, 2., ClipperLib::jtRound, 0., ClipperLib::etOpenRound)).empty());
}

TEST_CASE("Bridge aware cubic batch ownership partitions the exact emitted paths",
          "[bridge_aware_cubic][geometry][partition_ownership]")
{
    BAC::Parameters p;
    p.bounds = BoundingBoxf3(Vec3d(-24., -14., 0.), Vec3d(24., 14., 20.));
    p.flow_spacing = double(float(.4)) - double(float(.2)) * (1. - M_PI / 4.);
    p.density = GENERATE(.12, .15);
    p.angle = GENERATE(17. * M_PI / 180., M_PI / 4.);
    p.allow_solid_fallback = true;
    const ExPolygon whole = rectangle(-24., -14., 24., 14.);
    // Several touching saturated descendants make simultaneous open-line
    // clipping materially different from independently clipping each face.
    std::vector<BAC::Demand> demands{{{whole}, 13., 6.}};
    for (const Vec2d &point : {Vec2d(-20.85, 1.05), Vec2d(-20.6, .4), Vec2d(-10.09, -5.91),
                             Vec2d(9.05, 7.19), Vec2d(9.6, 7.35)})
        demands.push_back({{rectangle(point.x() - .001, point.y() - .001,
                                     point.x() + .001, point.y() + .001)}, 13., .05});
    const BAC::Plan plan(p, demands);
    REQUIRE(plan.statistics().solid_cells > 0);
    for (double z : {12.6, 12.8, 13., 13.2, 13.4}) {
        CAPTURE(p.density, p.angle, z);
        const Polylines emitted = plan.refinement_lines(z, whole);
        const auto partitions = plan.partition_lines(z, whole);
        Polylines owned;
        for (const auto &[id, lines] : partitions) {
            CHECK(std::any_of(plan.refinements().begin(), plan.refinements().end(),
                [&](const BAC::Refinement &r) { return !r.solid && r.id == id; }));
            CHECK(diff_pl(lines, offset(emitted, 2., ClipperLib::jtRound, 0., ClipperLib::etOpenRound)).empty());
            owned.insert(owned.end(), lines.begin(), lines.end());
        }
        CHECK(diff_pl(emitted, offset(owned, 2., ClipperLib::jtRound, 0., ClipperLib::etOpenRound)).empty());
    }
}

TEST_CASE("Closed cell clipping retains exact nonzero demand geometry after bounds filtering",
          "[bridge_aware_cubic][geometry][ClipFilter]")
{
    Polygon hole = rectangle(-3., -3., 3., 3.).contour;
    hole.reverse();
    const Polygons rings{rectangle(-10., -10., 10., 10.).contour, hole,
        rectangle(-1., -1., 1., 1.).contour, rectangle(10., -2., 14., 2.).contour,
        rectangle(50., 50., 60., 60.).contour, rectangle(50., -10., 60., 10.).contour, Polygon{}};
    const BAC::BoundedClipMask mask(rings);
    const auto canonical = [](const ExPolygons &polygons) {
        std::vector<std::vector<std::pair<coord_t, coord_t>>> result;
        for (const Polygon &ring : to_polygons(polygons)) {
            std::vector<std::pair<coord_t, coord_t>> points;
            for (const Point &point : ring.points) points.emplace_back(point.x(), point.y());
            if (!points.empty()) std::rotate(points.begin(), std::min_element(points.begin(), points.end()), points.end());
            result.push_back(std::move(points));
        }
        std::sort(result.begin(), result.end());
        return result;
    };
    for (const ExPolygon &subject : {rectangle(-8., -8., -6., -6.), rectangle(-2.5, -2.5, -1.5, -1.5),
            rectangle(-.5, -.5, .5, .5), rectangle(-4., -4., 4., 4.), rectangle(9., -1., 11., 1.),
            rectangle(10., 10., 11., 11.), rectangle(45., 45., 55., 55.), rectangle(20., 20., 21., 21.),
            rectangle(9.99999, -.00001, 10.00001, .00001), rectangle(2.99999, -2., 3.00001, -1.)}) {
        const Polygons filtered = mask.candidates(subject.contour);
        CHECK(canonical(intersection_ex(subject, filtered)) == canonical(intersection_ex(subject, rings)));
        CHECK(canonical(intersection_ex(subject, mask.candidates(get_extents<true>(subject.contour.points)))) ==
              canonical(intersection_ex(subject, rings)));
    }
    // Entire containing rings and their orientation survive; disjoint rings
    // and empty bounds are the only exclusions, including a point-only touch.
    CHECK(mask.candidates(rectangle(-8., -8., -6., -6.).contour).size() == 1);
    const auto in_hole = mask.candidates(rectangle(-2.5, -2.5, -1.5, -1.5).contour);
    REQUIRE(in_hole.size() == 2);
    CHECK(in_hole.back().points == hole.points);
    CHECK(mask.candidates(rectangle(10., 10., 11., 11.).contour).size() == 1);
    CHECK(mask.candidates(Polygon{}).empty());
    CHECK(mask.candidates(BoundingBox{}).empty());
}

TEST_CASE("Filtering disconnected demand rings preserves recursive cubic output",
          "[bridge_aware_cubic][geometry][ClipFilter]")
{
    BAC::Parameters p;
    p.bounds = BoundingBoxf3(Vec3d(-30., -24., 0.), Vec3d(30., 24., 20.));
    p.flow_spacing = .4;
    p.density = .08;
    p.angle = GENERATE(0., 17. * M_PI / 180.);
    const ExPolygons pieces{rectangle(-24., -18., -22., -16.), rectangle(-2., -1., 1., 2.),
                            rectangle(18., 14., 20., 16.)};
    const BAC::Plan grouped(p, {{pieces, 11.7, 5.5}});
    std::vector<BAC::Demand> isolated;
    for (const ExPolygon &piece : pieces) isolated.push_back({{piece}, 11.7, 5.5});
    // Each reference demand has one ring, retained in full whenever its
    // existing whole-demand bounds test passes. Its clipping is unchanged.
    const BAC::Plan reference(p, isolated);
    REQUIRE(grouped.statistics().max_depth > 1);
    CHECK(grouped.statistics().nodes == reference.statistics().nodes);
    CHECK(grouped.statistics().splits == reference.statistics().splits);
    REQUIRE(grouped.refinements().size() == reference.refinements().size());
    for (size_t i = 0; i < grouped.refinements().size(); ++i) {
        CHECK(grouped.refinements()[i].id == reference.refinements()[i].id);
        CHECK(grouped.refinements()[i].solid == reference.refinements()[i].solid);
    }
    const ExPolygon domain = rectangle(-30., -24., 30., 24.);
    for (double z : {5.4, 11.7, 14.2}) {
        const Polylines actual = grouped.lines(z, domain), expected = reference.lines(z, domain);
        REQUIRE(actual.size() == expected.size());
        for (size_t i = 0; i < actual.size(); ++i) CHECK(actual[i].points == expected[i].points);
        const auto leaves = grouped.leaf_sections(z), reference_leaves = reference.leaf_sections(z);
        REQUIRE(leaves.size() == reference_leaves.size());
        for (size_t i = 0; i < leaves.size(); ++i) {
            CHECK(leaves[i].cell_id == reference_leaves[i].cell_id);
            CHECK(leaves[i].polygon.points == reference_leaves[i].polygon.points);
        }
    }
}

TEST_CASE("Open clipping retains source identity at a one-grid family crossing",
          "[bridge_aware_cubic][geometry][partition_ownership][OpenSourceIdentity]")
{
    const bool reverse = GENERATE(false, true);
    ClipperLib::Paths subjects{{Point(-1000000, -577350), Point(1000000, 577350)},
                              {Point(1, -100), Point(1, 100)}};
    if (reverse) for (auto &path : subjects) std::reverse(path.begin(), path.end());
    const ClipperLib::Path strip{Point(-100, 101), Point(100, -99), Point(100, -98), Point(-100, 102)};
    ClipperLib::Clipper clipper;
    REQUIRE(clipper.AddPaths(subjects, ClipperLib::ptSubject, false));
    REQUIRE(clipper.AddPath(strip, ClipperLib::ptClip, true));
    ClipperLib::PolyTree result;
    REQUIRE(clipper.Execute(ClipperLib::ctIntersection, result, ClipperLib::pftNonZero, ClipperLib::pftNonZero));
    REQUIRE(result.ChildCount() == 2);
    std::set<size_t> sources;
    for (const auto *node : result.Childs) {
        REQUIRE(node->IsOpen());
        REQUIRE(node->Contour.size() == 2);
        CHECK(((node->Contour.front() == Point(1, 0) && node->Contour.back() == Point(1, 1)) ||
               (node->Contour.front() == Point(1, 1) && node->Contour.back() == Point(1, 0))));
        // Both fragments round to the vertical source's exact axis. A nearest
        // axis lookup would give source 1 twice and lose source 0's producer.
        sources.insert(node->OpenPathIndex());
    }
    CHECK(sources == std::set<size_t>{0, 1});
}

TEST_CASE("Open clipping retains source identity through splits and repeated operations",
          "[bridge_aware_cubic][geometry][partition_ownership][OpenSourceIdentity]")
{
    const ClipperLib::Paths subjects{{}, {Point(-100, 0), Point(100, 0)},
                                    {Point(-100, 20), Point(0, 40), Point(100, 20)}};
    const ClipperLib::Path hole{Point(-10, -50), Point(10, -50), Point(10, 50), Point(-10, 50)};
    ClipperLib::Clipper difference;
    for (unsigned repeat = 0; repeat < 2; ++repeat) {
        difference.Clear();
        REQUIRE(difference.AddPaths(subjects, ClipperLib::ptSubject, false));
        REQUIRE(difference.AddPath(hole, ClipperLib::ptClip, true));
        ClipperLib::PolyTree pieces;
        REQUIRE(difference.Execute(ClipperLib::ctDifference, pieces, ClipperLib::pftNonZero, ClipperLib::pftNonZero));
        REQUIRE(pieces.ChildCount() == 4);
        std::multiset<size_t> source_ids;
        ClipperLib::Paths split;
        std::vector<size_t> original_ids;
        for (const auto *node : pieces.Childs) {
            REQUIRE(node->IsOpen());
            source_ids.insert(node->OpenPathIndex());
            split.push_back(node->Contour);
            original_ids.push_back(node->OpenPathIndex());
        }
        CHECK(source_ids == std::multiset<size_t>{1, 1, 2, 2});
        ClipperLib::Clipper intersection;
        REQUIRE(intersection.AddPaths(split, ClipperLib::ptSubject, false));
        REQUIRE(intersection.AddPath({Point(-50, -50), Point(50, -50), Point(50, 50), Point(-50, 50)},
                                     ClipperLib::ptClip, true));
        ClipperLib::PolyTree final;
        REQUIRE(intersection.Execute(ClipperLib::ctIntersection, final, ClipperLib::pftNonZero, ClipperLib::pftNonZero));
        source_ids.clear();
        for (const auto *node : final.Childs) {
            REQUIRE(node->OpenPathIndex() < original_ids.size());
            source_ids.insert(original_ids[node->OpenPathIndex()]);
        }
        CHECK(source_ids == std::multiset<size_t>{1, 1, 2, 2});
    }
    difference.Clear();
    REQUIRE(difference.AddPath(subjects[1], ClipperLib::ptSubject, false));
    ClipperLib::PolyTree reset;
    REQUIRE(difference.Execute(ClipperLib::ctDifference, reset, ClipperLib::pftNonZero, ClipperLib::pftNonZero));
    REQUIRE(reset.ChildCount() == 1);
    CHECK(reset.Childs.front()->OpenPathIndex() == 0);
}

TEST_CASE("Bridge aware saturated deposition closes top-plane cells at one nominal layer volume",
          "[bridge_aware_cubic][geometry][SaturatedDeposition]")
{
    // Parallel 100% fill leaves long slots at the top of rounded beads. Two
    // full-density sweeps would close them by depositing twice the material.
    // Exercise the emitted paths to reject both failures, including a bridge
    // hint that must not collapse the two construction directions into one.
    const float height = GENERATE(.195046857f, .24f);
    const double angle = GENERATE(0., .296705972839036); // 0 and 17 degrees.
    const Flow flow(.4f, height, .4f);
    const ExPolygon domain = rectangle(-10., -10., 10., 10.);
    const ExPolygons material{domain};
    const ExPolygon interior = rectangle(-7., -7., 7., 7.);
    BAC::Parameters p;
    p.bounds = BoundingBoxf3(Vec3d(-12., -12., 0.), Vec3d(12., 12., 8.));
    p.flow_spacing = flow.spacing();
    p.density = .5;
    p.angle = angle;
    p.allow_solid_fallback = true;
    // Halfway between cubic vertices, every intersecting root has a section
    // larger than 1 mm and reaches saturation before another subdivision.
    const double z = 5.5 * std::sqrt(2.) * p.flow_spacing / p.density;
    const BAC::Plan plan(p, {{{rectangle(-11., -11., 11., 11.)}, z, 1.}});
    REQUIRE(plan.statistics().solid_cells > 0);
    REQUIRE(diff_ex(material, plan.solid_area(z)).empty());
    REQUIRE(plan.lines(z, domain).empty());

    FillBridgeAwareCubic filler;
    filler.plan = &plan;
    filler.material_domain = &material;
    filler.extrusion_width = flow.width();
    filler.bounding_box = get_extents(domain);
    filler.angle = float(angle);
    filler.spacing = flow.spacing();
    filler.overlap = 0.;
    filler.layer_id = 3;
    filler.z = z;
    FillParams params;
    params.density = float(p.density);
    params.flow = flow;
    params.using_internal_flow = true;
    params.extrusion_role = erInternalInfill;
    params.multiline = 3; // The saturated construction must use single courses.
    Surface surface(stInternal, domain);
    surface.bridge_angle = .61;
    ExtrusionEntityCollection emitted;
    filler.fill_surface_extrusion(&surface, params, emitted.entities);
    REQUIRE(emitted.entities.size() == 1);
    const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(emitted.entities.front());
    REQUIRE(collection != nullptr);
    REQUIRE_FALSE(collection->entities.empty());

    Polylines paths;
    std::vector<BridgeAwareCubicContact::Bead> beads;
    double deposited_volume = 0.;
    std::array<double, 2> directional_length{{0., 0.}};
    const Vec2d first_direction(std::cos(angle), std::sin(angle));
    for (const ExtrusionEntity *entity : collection->entities) {
        const auto *path = dynamic_cast<const ExtrusionPath *>(entity);
        REQUIRE(path != nullptr);
        CHECK(path->role() == erInternalInfill);
        CHECK(path->width == flow.width());
        CHECK(path->height == flow.height());
        CHECK(path->mm3_per_mm == Approx(flow.mm3_per_mm()));
        paths.push_back(path->polyline);
        beads.push_back({path, z});
        const Polylines measured = intersection_pl(Polylines{path->polyline}, interior);
        deposited_volume += length(measured) * path->mm3_per_mm;
        for (const Polyline &line : measured) for (size_t i = 1; i < line.size(); ++i) {
            const Vec2d delta = (line.points[i] - line.points[i - 1]).cast<double>() * SCALING_FACTOR;
            const double segment_length = delta.norm();
            REQUIRE(segment_length > 0.);
            const double alignment = std::abs(delta.dot(first_direction) / segment_length);
            REQUIRE((alignment < .0001 || alignment > .9999));
            directional_length[alignment > .5 ? 0 : 1] += segment_length;
        }
    }
    REQUIRE(directional_length[0] > 0.);
    REQUIRE(directional_length[1] > 0.);
    // Finite-window line counting may add/remove one course per family. The
    // 14 mm square is large enough to keep that boundary effect below 6%.
    CHECK(deposited_volume / (14. * 14. * height) == Approx(1.).margin(.06));
    CHECK(diff_pl(paths, offset_ex(material, -scale_(.5 * flow.width()))).empty());

    ExtrusionPath upper(erInternalInfill, flow.mm3_per_mm(), flow.width(), flow.height());
    upper.polyline = Polyline(Point::new_scale(-7., 0.), Point::new_scale(7., 0.));
    const auto contact = BridgeAwareCubicContact::footprint(beads, upper, z + height, 0.);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    CHECK(contact.contact_z == Approx(z));
    const auto cells = BridgeAwareCubicValidation::validate(material, contact.physical_footprint,
                                                           {}, {interior}, 1., 0.);
    INFO(cells.failure_reason);
    REQUIRE(cells.passed());
    REQUIRE_FALSE(cells.void_cells.empty());
    // For these two heights, pitch = 2*flow spacing and the real top width is
    // width-height: the square-hole diagonals are about 0.723 and 0.759 mm.
    CHECK(cells.max_void_diameter_mm < .8);
}

TEST_CASE("Bridge aware cubic collapse removes orphan dependency cycles and preserves earlier removals", "[bridge_aware_cubic][geometry]")
{
    const BAC::CellId a {{ 0, 0, 0 }, 0};
    const BAC::CellId b {{ 1, 0, 0 }, 0};
    const BAC::CellId c {{ -1, 0, 2 }, 0};
    const BAC::Demand direct_a { { rectangle(-.1, -.1, .1, .1) }, 19., 20. };
    const BAC::Demand direct_b { { rectangle(-.1, 17.9, .1, 18.1) }, 31.727922061, 20. };
    const BAC::Demand dependent_c { { rectangle(29.9, -30.1, 30.1, -29.9) }, 20., 3.5, true, { a, b } };
    const BAC::Demand dependent_a { direct_a.footprint, direct_a.z, 3.5, true, { c } };
    const BAC::Plan orphan_cycle(parameters(), { dependent_c, dependent_a });
    REQUIRE(orphan_cycle.statistics().splits == 0);
    REQUIRE(orphan_cycle.active_demands().empty());
    const BAC::Plan rooted(parameters(), { direct_a, direct_b, dependent_c, dependent_a });
    REQUIRE(rooted.active_demands().size() == 4);
    const size_t original_splits = rooted.statistics().splits;
    const BAC::Plan one = rooted.collapsed(a);
    REQUIRE(one.active_demands().size() == 4); // b is still an alternative producer.
    const BAC::Plan both = one.collapsed(b);
    REQUIRE(both.active_demands().size() == 2);
    REQUIRE(both.statistics().splits == 0);
    REQUIRE(rooted.statistics().splits == original_splits);
    REQUIRE_THROWS_AS(both.collapsed(a), std::invalid_argument);
}

TEST_CASE("Bridge aware cubic coarsening uses deposited widths rather than ideal cell sizing", "[bridge_aware_cubic][geometry]")
{
    const BAC::Demand demand { { rectangle(-.1, -.1, .1, .1) }, 19., 30.9 };
    const BAC::Plan initial(parameters(), { demand });
    REQUIRE(initial.statistics().splits == 1);
    size_t validations = 0;
    const auto actual_openings_pass = [&](const BAC::Plan &trial) {
        ++validations;
        const Polygons material = offset(trial.lines(demand.z, rectangle(-40., -40., 40., 40.)),
                                          float(scale_(.225)), ClipperLib::jtRound, 0., ClipperLib::etOpenRound);
        for (const BAC::LeafSection &leaf : trial.leaf_sections(demand.z)) {
            if (intersection_ex(ExPolygon(leaf.polygon), demand.footprint).empty()) continue;
            for (const ExPolygon &opening : diff_ex(ExPolygon(leaf.polygon), material))
                if (!intersection_ex(opening, demand.footprint).empty() && diameter(opening.contour) > demand.gap)
                    return false;
        }
        return true;
    };
    const BAC::Plan coarsened = initial.coarsened(actual_openings_pass);
    REQUIRE(validations >= 2);
    REQUIRE(coarsened.statistics().splits == 0);
    REQUIRE(actual_openings_pass(coarsened));
    REQUIRE(initial.statistics().splits == 1);
}

TEST_CASE("Bridge aware cubic rejected coarsening and work limits cannot certify a trial", "[bridge_aware_cubic][geometry]")
{
    const BAC::Plan initial(parameters(), { { { rectangle(-25., 4.8, 25., 5.2) }, 21., 3.5 } });
    const size_t splits = initial.statistics().splits;
    const BAC::Plan unchanged = initial.coarsened([&](const BAC::Plan &trial) {
        return trial.statistics().splits == splits;
    });
    REQUIRE(unchanged.statistics().splits == splits);
    REQUIRE_THROWS_WITH(initial.coarsened([](const BAC::Plan &) { return true; }, 1),
                        Catch::Contains("coarsening work limit"));
    REQUIRE_THROWS_WITH(initial.coarsened([](const BAC::Plan &) -> bool {
        throw std::runtime_error("cancel exact trial");
    }), "cancel exact trial");
}

TEST_CASE("Bridge aware cubic immutable closure and attachment payloads survive validated trials", "[bridge_aware_cubic][geometry]")
{
    const BAC::Demand demand { { rectangle(-.1, -.1, .1, .1) }, 19., 20. };
    const BAC::CellId owner {{ 0, 0, 0 }, 0};
    const BAC::LayerClosure closure { 10., { Polyline(Point::new_scale(-3., 0.), Point::new_scale(3., 0.)) },
                                      { rectangle(-3., -.3, 3., .3) }, {}, 9 };
    const BAC::LayerClosure dependent { 11., closure.paths, closure.reservation, { owner } };
    BAC::LayerAttachment attachment;
    attachment.z = 20.;
    attachment.emitting_region = 7;
    attachment.original = Polyline(Point::new_scale(-2., 0.), Point::new_scale(2., 0.));
    attachment.replacement = closure.paths.front();
    attachment.width = .45;
    attachment.height = .2;
    attachment.solid_cut = true;
    attachment.demand_selection = { rectangle(1.8, -.2, 2.2, .2) };
    BAC::LayerAttachment dependent_attachment = attachment;
    dependent_attachment.z = 21.;
    dependent_attachment.owners = { owner };
    const BAC::Plan initial(parameters(), { demand }, { closure, dependent }, { attachment, dependent_attachment });
    REQUIRE(length(initial.closure_lines(10.)) == Approx(6.));
    REQUIRE(length(initial.closure_lines(11.)) == Approx(6.));
    REQUIRE(initial.closures(10.).size() == 1);
    REQUIRE(initial.closures(10.).front().emitting_region == 9);
    const ExPolygon domain = rectangle(-40., -40., 40., 40.);
    REQUIRE(length(initial.refinement_lines(10., domain)) - length(initial.partition_lines(owner, 10., domain)) == Approx(6.));
    REQUIRE(initial.attachments(21.).size() == 1);
    REQUIRE(initial.closure_lines(10.00001).empty());
    const BAC::Plan trial = initial.collapsed(owner);
    REQUIRE(length(trial.closure_lines(10.)) == Approx(6.));
    REQUIRE(trial.closure_lines(11.).empty());
    REQUIRE(trial.attachments(20.).size() == 1);
    REQUIRE(trial.attachments(20.).front().emitting_region == 7);
    REQUIRE(trial.attachments(20.).front().replacement.points == attachment.replacement.points);
    REQUIRE(trial.attachments(20.).front().solid_cut);
    REQUIRE(trial.attachments(20.).front().demand_selection.front().contour.points ==
            attachment.demand_selection.front().contour.points);
    REQUIRE(trial.attachments(21.).empty());
    REQUIRE_FALSE(intersection_ex(trial.refinement_area(10.), closure.reservation).empty());
}

TEST_CASE("Bridge aware cubic bounds and cancels dependency ownership work", "[bridge_aware_cubic][geometry]")
{
    BAC::Parameters p = parameters();
    p.max_nodes = 1000;
    const std::vector<BAC::CellId> excessive(16001, BAC::CellId{{0, 0, 0}, 0});
    const BAC::Demand demand { { rectangle(-.1, -.1, .1, .1) }, 19., 5., true, excessive };
    REQUIRE_THROWS_WITH(BAC::Plan(p, { demand }), Catch::Contains("ownership reference work limit"));
    const BAC::LayerClosure closure { 10., {}, {}, excessive };
    REQUIRE_THROWS_WITH(BAC::Plan(p, {}, { closure }), Catch::Contains("ownership reference work limit"));
    BAC::LayerAttachment attachment;
    attachment.z = 20.;
    attachment.original = attachment.replacement = Polyline(Point(0, 0), Point(100, 100));
    attachment.width = .45;
    attachment.height = .2;
    attachment.owners = excessive;
    REQUIRE_THROWS_WITH(BAC::Plan(p, {}, {}, { attachment }), Catch::Contains("ownership reference work limit"));
    BAC::Demand bounded = demand;
    bounded.owners.resize(1000);
    unsigned checks = 0;
    p.throw_if_canceled = [&] { if (++checks == 2) throw std::runtime_error("owner canceled"); };
    REQUIRE_THROWS_WITH(BAC::Plan(p, { bounded }), "owner canceled");
}

TEST_CASE("Bridge aware cubic trial copies preserve collapsed cells during global dependency filtering", "[bridge_aware_cubic][geometry]")
{
    const BAC::CellId a {{ 0, 0, 0 }, 0};
    const BAC::CellId b {{ 1, 0, 0 }, 0};
    const BAC::Demand direct_a { { rectangle(-.1, -.1, .1, .1) }, 19., 20. };
    const BAC::Demand direct_b { { rectangle(-.1, 17.9, .1, 18.1) }, 31.727922061, 20. };
    const BAC::LayerClosure closure { 10., { Polyline(Point::new_scale(-3., 0.), Point::new_scale(3., 0.)) },
                                      { rectangle(-3., -.3, 3., .3) }, {}, 9 };
    BAC::LayerAttachment attachment;
    attachment.z = 20.;
    attachment.original = attachment.replacement = closure.paths.front();
    attachment.width = .45;
    attachment.height = .2;
    attachment.solid_cut = true;
    attachment.demand_selection = { rectangle(2.8, -.2, 3.2, .2) };
    const BAC::Plan original(parameters(), { direct_a }, { closure }, { attachment });
    const BAC::Plan collapsed = original.collapsed(a);
    REQUIRE(collapsed.refinements().empty());
    const BAC::Plan copy = collapsed.clone();
    REQUIRE(copy.refinements().empty());
    REQUIRE(copy.demands().size() == 1);
    REQUIRE(copy.closures(10.).front().emitting_region == 9);
    REQUIRE(copy.attachments(20.).size() == 1);
    REQUIRE(copy.attachments(20.).front().solid_cut);
    REQUIRE(copy.attachments(20.).front().demand_selection.front().contour.points ==
            attachment.demand_selection.front().contour.points);

    BAC::Demand stronger_a = direct_a;
    stronger_a.gap = 3.5;
    const BAC::Plan filtered = copy.with_demands({ stronger_a, direct_b });
    REQUIRE(filtered.demands().size() == 2);
    REQUIRE(filtered.refinements().size() == 1);
    REQUIRE(filtered.refinements().front().id == b);
    REQUIRE(filtered.attachments(20.).size() == 1);
    REQUIRE(filtered.attachments(20.).front().solid_cut);
    REQUIRE(filtered.attachments(20.).front().demand_selection.front().contour.points ==
            attachment.demand_selection.front().contour.points);
    const BAC::Plan payload_rebuilt = filtered.with_payloads({}, {attachment});
    REQUIRE(payload_rebuilt.attachments(20.).front().solid_cut);
    REQUIRE(payload_rebuilt.attachments(20.).front().demand_selection.front().contour.points ==
            attachment.demand_selection.front().contour.points);
    REQUIRE(length(filtered.closure_lines(10.)) == Approx(6.));
    const BAC::Plan payload_filtered = filtered.with_payloads({}, {});
    REQUIRE(payload_filtered.refinements().size() == 1);
    REQUIRE(payload_filtered.refinements().front().id == b);
    REQUIRE(payload_filtered.closure_lines(10.).empty());
    REQUIRE(payload_filtered.attachments(20.).empty());
    const BAC::Plan finally_collapsed = payload_filtered.collapsed(b).with_demands({ stronger_a, direct_b });
    REQUIRE(finally_collapsed.refinements().empty());
    REQUIRE(original.refinements().size() == 1);
    REQUIRE(original.refinements().front().id == a);
    REQUIRE(filtered.closures(10.).size() == 1);
}

TEST_CASE("Bridge aware cubic validates and bounds attachment demand selection", "[bridge_aware_cubic][geometry]")
{
    BAC::Parameters p = parameters();
    p.max_nodes = 1000;
    BAC::LayerAttachment attachment;
    attachment.z = 20.;
    attachment.original = attachment.replacement = Polyline(Point::new_scale(-2., 0.), Point::new_scale(2., 0.));
    attachment.width = .45;
    attachment.height = .2;
    REQUIRE_NOTHROW(BAC::Plan(p, {}, {}, {attachment})); // Empty means the full actual path.
    attachment.solid_cut = true;
    attachment.demand_selection = { rectangle(-2.1, -.2, -1.9, .2) };
    REQUIRE_NOTHROW(BAC::Plan(p, {}, {}, {attachment}));
    attachment.demand_selection.front().holes = { rectangle(-2.05, -.1, -1.95, .1).contour };
    REQUIRE_THROWS_AS(BAC::Plan(p, {}, {}, {attachment}), std::invalid_argument);
    attachment.demand_selection.front().holes.front().make_clockwise();
    REQUIRE_NOTHROW(BAC::Plan(p, {}, {}, {attachment}));
    attachment.demand_selection.front().holes.front().points.resize(2);
    REQUIRE_THROWS_AS(BAC::Plan(p, {}, {}, {attachment}), std::invalid_argument);
    attachment.demand_selection = { rectangle(-2.1, -.2, -1.9, .2) };
    attachment.demand_selection.front().contour.points.front().x() = std::numeric_limits<coord_t>::max();
    REQUIRE_THROWS_WITH(BAC::Plan(p, {}, {}, {attachment}), Catch::Contains("coordinate bounds"));
    attachment.demand_selection.assign(4000, rectangle(-2.1, -.2, -1.9, .2));
    // Selection vertices and both path identities share one payload budget.
    REQUIRE_THROWS_WITH(BAC::Plan(p, {}, {}, {attachment}), Catch::Contains("payload vertex work limit"));
    attachment.demand_selection.resize(1000);
    unsigned checks = 0;
    p.throw_if_canceled = [&] { if (++checks == 2) throw std::runtime_error("selection canceled"); };
    REQUIRE_THROWS_WITH(BAC::Plan(p, {}, {}, {attachment}), "selection canceled");
}
