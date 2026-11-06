#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/BridgeAwareCubicContainment.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSupport.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
Polygon box(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    return Polygon(Points{Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1)});
}

Polyline path(std::initializer_list<Point> points) { return Polyline(Points(points)); }

bool original_match(const Polyline &line, const Polygons &rings)
{
    return BAC::complete_construction_match(line, intersection_pl(Polylines{line}, rings), .002 + 16. * SCALING_FACTOR);
}

bool check_shortcut(const BAC::StrictMaskContainment &index, const Polyline &line, const Polygons &rings)
{
    const bool proven = index.proves(line);
    const bool expected = original_match(line, rings);
    if (proven) CHECK(expected);
    CHECK((proven || original_match(line, rings)) == expected);
    return proven;
}
}

TEST_CASE("Bridge aware strict containment proves only complete interior paths", "[BridgeAwareCubic][Containment]")
{
    const Polygons rings {box(-1000, -1000, 1000, 1000)};
    Validation::WorkBudget budget;
    const BAC::StrictMaskContainment index(rings, &budget);
    REQUIRE(index.available());
    CHECK(budget.remaining_work < 50000000);
    const auto before = budget.remaining_work;
    REQUIRE(index.proves(path({Point(-800, 0), Point(0, 500), Point(800, 0)}), &budget));
    CHECK(budget.remaining_work < before);
    REQUIRE(check_shortcut(index, path({Point(-800, 0), Point(-800, 0), Point(800, 0)}), rings));
    REQUIRE(check_shortcut(index, path({Point(-800, 0), Point(800, 0), Point(-800, 0),
                                        Point(800, 500), Point(-800, 500), Point(800, 0)}), rings));
    REQUIRE_FALSE(index.proves(path({Point(0, 0), Point(0, 0)})));
    REQUIRE_FALSE(index.proves(Polyline{}));
    REQUIRE_FALSE(index.proves(path({Point(0, 0)})));
    REQUIRE_FALSE(check_shortcut(index, path({Point(-1200, 0), Point(1200, 0)}), rings));
    REQUIRE_FALSE(check_shortcut(index, path({Point(-800, 0), Point(0, 1200), Point(800, 0)}), rings));
}

TEST_CASE("Bridge aware strict containment retains exact nonzero winding semantics", "[BridgeAwareCubic][Containment]")
{
    const auto middle = path({Point(-100, 0), Point(100, 0)});
    const Polygon outer = box(-1000, -1000, 1000, 1000);
    Polygon reversed = outer;
    reversed.reverse();
    for (const Polygons &rings : {Polygons{outer}, Polygons{reversed}, Polygons{outer, outer}, Polygons{reversed, reversed}}) {
        const BAC::StrictMaskContainment index(rings);
        REQUIRE(index.available());
        REQUIRE(check_shortcut(index, middle, rings));
    }
    const Polygons canceled {outer, reversed};
    const BAC::StrictMaskContainment zero(canceled);
    REQUIRE_FALSE(check_shortcut(zero, middle, canceled));

    Polygon hole = box(-200, -200, 200, 200);
    hole.reverse();
    const Polygons rings {outer, hole, box(1800, -500, 2800, 500)};
    const BAC::StrictMaskContainment index(rings);
    REQUIRE(check_shortcut(index, path({Point(-800, 500), Point(800, 500)}), rings));
    REQUIRE_FALSE(check_shortcut(index, middle, rings));
    REQUIRE_FALSE(check_shortcut(index, path({Point(-800, 0), Point(800, 0)}), rings));
    REQUIRE(check_shortcut(index, path({Point(1900, 0), Point(2700, 0)}), rings));
    REQUIRE_FALSE(check_shortcut(index, path({Point(500, 500), Point(2200, 0)}), rings));
}

TEST_CASE("Bridge aware strict containment declines boundary and clearance uncertainty", "[BridgeAwareCubic][Containment]")
{
    const Polygons rings {box(-1000, -1000, 1000, 1000)};
    const BAC::StrictMaskContainment index(rings);
    for (coord_t separation : {0, 1, 2, 3, 4}) {
        CAPTURE(separation);
        REQUIRE_FALSE(check_shortcut(index,
            path({Point(-800, -1000 + separation), Point(800, -1000 + separation)}), rings));
    }
    REQUIRE(check_shortcut(index, path({Point(-800, -995), Point(800, -995)}), rings));
    REQUIRE_FALSE(check_shortcut(index, path({Point(0, 0), Point(1000, 1000)}), rings));
    REQUIRE_FALSE(check_shortcut(index, path({Point(-800, 0), Point(1000, 0), Point(-800, 500)}), rings));
    // Actual overlap edges remain obstacles even when nonzero winding on both
    // sides would let the original predicate classify the path successfully.
    const Polygons overlap {box(-1000, -1000, 400, 1000), box(-400, -1000, 1000, 1000)};
    const BAC::StrictMaskContainment conservative(overlap);
    const auto full = path({Point(-800, 0), Point(800, 0)});
    REQUIRE(original_match(full, overlap));
    REQUIRE_FALSE(check_shortcut(conservative, full, overlap));
}

TEST_CASE("Bridge aware strict containment handles self intersections and exact distant bounds", "[BridgeAwareCubic][Containment]")
{
    const Polygons crossing {Polygon(Points{Point(-1000, -1000), Point(1000, 1000),
                                           Point(-1000, 1000), Point(1000, -1000)})};
    const BAC::StrictMaskContainment index(crossing);
    REQUIRE(check_shortcut(index, path({Point(-100, 700), Point(100, 700)}), crossing));
    REQUIRE_FALSE(check_shortcut(index, path({Point(-100, 700), Point(100, -700)}), crossing));
    constexpr coord_t limit = std::numeric_limits<int32_t>::max() / 4;
    for (coord_t center : {-limit + 2000, limit - 2000}) {
        const Polygons rings {box(center - 1000, center - 1000, center + 1000, center + 1000)};
        const BAC::StrictMaskContainment distant(rings);
        REQUIRE(distant.available());
        REQUIRE(check_shortcut(distant, path({Point(center - 700, center), Point(center + 700, center)}), rings));
        REQUIRE_FALSE(check_shortcut(distant,
            path({Point(center - 700, center - 997), Point(center + 700, center - 997)}), rings));
    }
    const BAC::StrictMaskContainment unsafe({box(limit - 100, -100, limit + 100, 100)});
    CHECK_FALSE(unsafe.available());
    CHECK_FALSE(index.proves(path({Point(0, 700), Point(limit + 1, 700)})));
}

TEST_CASE("Bridge aware strict containment bounded queries agree with independent clipping", "[BridgeAwareCubic][Containment]")
{
    Polygons rings;
    for (coord_t i = 0; i < 80; ++i) rings.push_back(box(i * 3000, -1000, i * 3000 + 2000, 1000));
    Polygon hole = box(500, -300, 1500, 300);
    hole.reverse();
    rings.push_back(hole);
    const BAC::StrictMaskContainment index(rings);
    REQUIRE(index.available());
    uint32_t state = 0x6d2b79f5;
    const auto coordinate = [&](coord_t low, coord_t span) {
        state = state * 1664525u + 1013904223u;
        return coord_t(low + state % uint32_t(span));
    };
    size_t successes = 0;
    for (size_t i = 0; i < 500; ++i) {
        const coord_t tile = coord_t(i % 80) * 3000;
        Polyline line = path({Point(coordinate(tile - 200, 2400), coordinate(-1200, 2400)),
                              Point(coordinate(tile - 200, 2400), coordinate(-1200, 2400))});
        successes += check_shortcut(index, line, rings);
        line.reverse();
        successes += check_shortcut(index, line, rings);
    }
    REQUIRE(successes > 0);
}

TEST_CASE("Bridge aware strict containment falls back on caps and preserves cancellation", "[BridgeAwareCubic][Containment]")
{
    const Polygons rings {box(-1000, -1000, 1000, 1000)};
    const auto line = path({Point(-500, 0), Point(500, 0)});
    Validation::WorkBudget none;
    none.remaining_work = 0;
    const BAC::StrictMaskContainment declined(rings, &none);
    CHECK_FALSE(declined.available());
    CHECK_FALSE(declined.proves(line));
    const BAC::StrictMaskContainment index(rings);
    CHECK_FALSE(index.proves(line, &none));
    CHECK(none.remaining_work == 0);
    const Polygons excessive(20001, rings.front());
    CHECK_FALSE(BAC::StrictMaskContainment(excessive).available());
    const Polygons excessive_vertices {Polygon(Points(100001, Point(0, 0)))};
    CHECK_FALSE(BAC::StrictMaskContainment(excessive_vertices).available());
    Polyline long_path;
    long_path.points.reserve(4097);
    for (size_t i = 0; i < 4097; ++i) long_path.points.emplace_back(i % 2 ? 500 : -500, 0);
    CHECK_FALSE(index.proves(long_path));
    long_path.points.pop_back();
    REQUIRE(check_shortcut(index, long_path, rings));
    Validation::WorkBudget partial_build;
    partial_build.remaining_work = 5;
    CHECK_FALSE(BAC::StrictMaskContainment(rings, &partial_build).available());
    CHECK(partial_build.remaining_work == 0);
    CHECK(partial_build.charged_work[size_t(Validation::WorkCategory::Validation)] == 5);
    Validation::WorkBudget partial_query;
    partial_query.remaining_work = 3;
    CHECK_FALSE(index.proves(line, &partial_query));
    CHECK(partial_query.remaining_work == 0);
    CHECK(partial_query.charged_work[size_t(Validation::WorkCategory::Validation)] == 3);
    const auto canceled = [] { throw std::runtime_error("containment canceled"); };
    REQUIRE_THROWS_WITH(BAC::StrictMaskContainment(rings, nullptr, canceled), "containment canceled");
    REQUIRE_THROWS_WITH(index.proves(line, nullptr, canceled), "containment canceled");
    size_t calls = 0;
    const auto cancel_work = [&] { if (++calls == 2) throw std::runtime_error("containment work canceled"); };
    REQUIRE_THROWS_WITH(BAC::StrictMaskContainment(Polygons(100, rings.front()), nullptr, cancel_work),
                        "containment work canceled");
    calls = 0;
    REQUIRE_THROWS_WITH(index.proves(long_path, nullptr, cancel_work), "containment work canceled");
}
