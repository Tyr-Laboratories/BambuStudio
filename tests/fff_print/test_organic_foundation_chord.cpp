#include <catch2/catch.hpp>

#include <cmath>
#include <stdexcept>

#include "libslic3r/Support/TreeSupport3D.hpp"
#include "libslic3r/ClipperUtils.hpp"

using namespace Slic3r;
using namespace Slic3r::TreeSupport3D;

namespace {
coord_t units(double value) { return coord_t(std::llround(value / SCALING_FACTOR)); }
Point at(double x, double y = 0.) { return Point(units(x), units(y)); }
Polygon rectangle(double x0, double y0, double x1, double y1)
{
    return Polygon{Points{at(x0, y0), at(x1, y0), at(x1, y1), at(x0, y1)}};
}
}

TEST_CASE("A clear organic degree-two stem takes its fixed endpoint chord", "[OrganicFoundation][Chord]")
{
    const std::vector<Point> centers{at(0.), at(0.5, 0.4), at(1., 0.7), at(1.5, 0.4), at(2.)};
    const std::vector<coord_t> radii{units(0.4), units(0.6),
        units(0.8), units(0.7), units(0.5)};
    const auto old_centers = centers;
    const auto old_radii = radii;
    std::vector<Point> result;
    size_t checks = 0;
    int polls = 0;
    for (size_t i = 1; i < centers.size(); ++i)
        REQUIRE((centers[i] - centers[i - 1]).cast<double>().norm() <= units(1.));
    REQUIRE(organic_branch_chord(centers, radii, 17, units(1.),
        [&](const Point& point, coord_t radius, LayerIndex layer) {
            REQUIRE(layer == LayerIndex(17 + checks));
            REQUIRE(radius == radii[checks]);
            REQUIRE(point == at(double(checks) * 0.5));
            ++checks;
            return true;
        }, result, [&] { ++polls; }));
    const std::vector<Point> expected{at(0.), at(0.5), at(1.), at(1.5), at(2.)};
    REQUIRE(result == expected);
    REQUIRE(result.front() == centers.front());
    REQUIRE(result.back() == centers.back());
    REQUIRE(centers == old_centers);
    REQUIRE(radii == old_radii);
    REQUIRE(checks == centers.size());
    REQUIRE(polls > 0);
}

TEST_CASE("An unsafe organic chord leaves the existing stem intact", "[OrganicFoundation][Chord]")
{
    const std::vector<Point> centers{at(0.), at(0., 2.), at(1., 2.), at(2., 2.), at(2.)};
    const std::vector<coord_t> radii(centers.size(), units(0.5));
    std::vector<Point> result = centers;
    for (size_t i = 1; i < centers.size(); ++i)
        REQUIRE((centers[i] - centers[i - 1]).cast<double>().norm() <= units(2.));
    SECTION("A radius-aware obstacle blocks the chord") {
        REQUIRE_FALSE(organic_branch_chord(centers, radii, 17, units(2.),
            [](const Point& point, coord_t radius, LayerIndex layer) {
                return layer != 19 || (point - at(1., 0.4)).cast<double>().norm() > double(radius);
            }, result, [] {}));
    }
    SECTION("The chord exceeds the configured layer movement") {
        REQUIRE_FALSE(organic_branch_chord(centers, radii, 17, units(0.49),
            [](const Point&, coord_t, LayerIndex) { return true; }, result, [] {}));
    }
    SECTION("A fixed endpoint is forbidden") {
        REQUIRE_FALSE(organic_branch_chord(centers, radii, 17, units(2.),
            [](const Point&, coord_t, LayerIndex layer) { return layer != 17; }, result, [] {}));
    }
    SECTION("Cancellation preserves the existing result") {
        REQUIRE_THROWS_AS(organic_branch_chord(centers, radii, 17, units(2.),
            [](const Point&, coord_t, LayerIndex) { return true; }, result,
            [] { throw std::runtime_error("cancel"); }), std::runtime_error);
    }
    SECTION("Mismatched radius data is rejected") {
        REQUIRE_FALSE(organic_branch_chord(centers, {radii.front()}, 17, units(0.5),
            [](const Point&, coord_t, LayerIndex) { return true; }, result, [] {}));
    }
    REQUIRE(result == centers);
}

TEST_CASE("Organic chord validation handles short paths", "[OrganicFoundation][Chord]")
{
    const auto clear = [](const Point&, coord_t, LayerIndex) { return true; };
    std::vector<Point> result{at(9.)};
    REQUIRE_FALSE(organic_branch_chord({}, {}, 0, units(1.), clear, result, [] {}));
    REQUIRE_FALSE(organic_branch_chord({at(0.)}, {units(0.5)}, 0,
        units(1.), clear, result, [] {}));
    REQUIRE(result == std::vector<Point>{at(9.)});
    const std::vector<Point> pair{at(0.), at(1.)};
    REQUIRE(organic_branch_chord(pair, {units(0.5), units(0.5)}, 0,
        units(1.), clear, result, [] {}));
    REQUIRE(result == pair);
}

TEST_CASE("Organic chord shell validation scopes unchanged sibling relationships", "[OrganicFoundation][Chord][Sheath]")
{
    const double tolerance = double(units(0.01)) * units(0.01);
    const Polygons old_above{rectangle(0., 0., 1., 1.), rectangle(10., 0., 11., 1.)};
    const Polygons old_below{rectangle(10., 0., 11., 1.)};
    SECTION("An unchanged unsupported sibling does not veto a supported shortcut") {
        const Polygons changed_above{old_above.front(), rectangle(10.1, 0., 11.1, 1.)};
        REQUIRE(organic_changed_sheath_supported(changed_above, old_below,
            old_above, old_below, units(0.2), tolerance));
    }
    SECTION("A newly unsupported component is rejected") {
        Polygons changed_above = old_above;
        changed_above.push_back(rectangle(20., 0., 21., 1.));
        REQUIRE_FALSE(organic_changed_sheath_supported(changed_above, old_below,
            old_above, old_below, units(0.2), tolerance));
    }
    SECTION("Changed direct support sets must be checked even when both overlap") {
        const Polygons above{rectangle(0., 0., 2., 1.)};
        const Polygons before{rectangle(0., 0., 1., 1.)};
        const Polygons after{rectangle(0., 0., 0.5, 1.)};
        REQUIRE_FALSE(organic_changed_sheath_supported(above, after, above, before, 0, tolerance));
    }
    SECTION("Changed movement-envelope support must also be checked") {
        const Polygons above{rectangle(0., 0., 2., 1.)};
        const Polygons before{rectangle(0., 0., 0.5, 1.), rectangle(2.1, 0., 3., 1.)};
        const Polygons after{before.front()};
        REQUIRE(diff(intersection(above, before), intersection(above, after)).empty());
        REQUIRE(diff(intersection(above, after), intersection(above, before)).empty());
        REQUIRE_FALSE(organic_changed_sheath_supported(above, after, above, before, units(0.2), tolerance));
    }
    SECTION("All changed components share one unsupported-area budget") {
        const Polygons above{rectangle(0., 0., 1., 1.), rectangle(10., 0., 11., 1.)};
        const Polygons below{rectangle(0., 0., 0.99992, 1.), rectangle(10., 0., 10.99992, 1.)};
        REQUIRE(organic_changed_sheath_supported({above.front()}, {below.front()},
            {below.front()}, {below.front()}, 0, tolerance));
        REQUIRE_FALSE(organic_changed_sheath_supported(above, below, below, below, 0, tolerance));
    }
}
