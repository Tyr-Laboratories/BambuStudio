#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicOwnerBounds.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSupport.hpp"

#include <algorithm>
#include <initializer_list>
#include <limits>
#include <utility>
#include <vector>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;

namespace {
Polyline line(std::initializer_list<Point> points)
{
    return Polyline(Points(points));
}

void same_pieces(const Polylines &actual, const Polylines &expected)
{
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        CHECK(actual[i].points == expected[i].points);
}

using Owner = std::pair<size_t, Polylines>;

// Exercise the actual public wrapper of the legacy matcher, including its
// clipping and complete-owned shortcut. No copy of that predicate lives here.
size_t check_owners(const Polyline &candidate, const std::vector<Owner> &owners)
{
    const BAC::PartitionCandidateBounds bounds(candidate);
    std::vector<size_t> expected_producers, actual_producers;
    std::vector<size_t> expected_matches, actual_matches;
    size_t skipped = 0;
    for (const auto &[id, lines] : owners) {
        const Polylines expected = BAC::matching_partition_segments({candidate}, lines);
        const bool retain = BAC::PartitionOwnerBounds(lines).may_match(bounds);
        const Polylines actual = retain ? BAC::matching_partition_segments({candidate}, lines) : Polylines{};
        skipped += !retain;
        same_pieces(actual, expected);
        if (!expected.empty()) expected_matches.push_back(id);
        if (!actual.empty()) actual_matches.push_back(id);
        if (BAC::complete_construction_match(candidate, expected, 2. * SCALING_FACTOR)) expected_producers.push_back(id);
        if (BAC::complete_construction_match(candidate, actual, 2. * SCALING_FACTOR)) actual_producers.push_back(id);
    }
    CHECK(actual_matches == expected_matches);
    CHECK(actual_producers == expected_producers);
    return skipped;
}
}

TEST_CASE("Bridge aware owner bounds preserve distant tiny perpendicular ownership", "[BridgeAwareCubic][OwnerBounds]")
{
    const auto owner = line({Point(0, 0), Point(1, 0)});
    const auto candidate = line({Point(0, 10), Point(0, 20)});
    const auto expected = BAC::matching_partition_segments({candidate}, {owner});
    REQUIRE(expected.size() == 1);
    CHECK(expected.front().points == candidate.points);
    CHECK(BAC::PartitionOwnerBounds({owner}).may_match(BAC::PartitionCandidateBounds(candidate)));
    CHECK(check_owners(candidate, {{7, {owner}}, {3, {line({Point(100, 0), Point(200, 0)})}}}) == 1);
}

TEST_CASE("Bridge aware owner bounds preserve exact pieces and producer order", "[BridgeAwareCubic][OwnerBounds]")
{
    const std::vector<Owner> owners{
        {17, {line({Point(100, 0), Point(0, 0)})}},
        {2, {line({Point(500, 500), Point(600, 500)})}},
        {11, {line({Point(20, 0), Point(80, 0)})}},
        {5, {line({Point(0, 2), Point(100, 2)})}},
        {23, {line({Point(0, 0), Point(100, 0)}), line({Point(100, 0), Point(100, 100)})}},
        {4, {line({Point(0, 0), Point(100, 100)})}},
        {8, {line({Point(50, -100), Point(50, 100)})}},
        {1, {line({Point(0, 0), Point(1, 0)})}}
    };
    for (auto candidate : {
             line({Point(0, 0), Point(100, 0)}),
             line({Point(-20, 0), Point(120, 0)}),
             line({Point(25, 0), Point(75, 0)}),
             line({Point(0, 2), Point(100, 2)}),
             line({Point(0, 3), Point(100, 3)}),
             line({Point(100, 0), Point(100, 100)}),
             line({Point(0, 0), Point(100, 0), Point(100, 100)}),
             line({Point(10, 10), Point(90, 90)})}) {
        CHECK(check_owners(candidate, owners) > 0);
        std::reverse(candidate.points.begin(), candidate.points.end());
        CHECK(check_owners(candidate, owners) > 0);
    }
}

TEST_CASE("Bridge aware owner bounds retain threshold and inclusive boundary cases", "[BridgeAwareCubic][OwnerBounds]")
{
    const auto distant = line({Point(0, 100), Point(0, 110)});
    const BAC::PartitionCandidateBounds bounds(distant);
    CHECK(BAC::PartitionOwnerBounds({line({Point(0, 0), Point(8, 0)})}).may_match(bounds));
    CHECK_FALSE(BAC::PartitionOwnerBounds({line({Point(0, 0), Point(8, 1)})}).may_match(bounds));
    CHECK_FALSE(BAC::PartitionOwnerBounds({line({Point(0, 0), Point(9, 0)})}).may_match(bounds));
    check_owners(distant, {{0, {line({Point(0, 0), Point(8, 0)})}},
                           {1, {line({Point(0, 0), Point(8, 1)})}},
                           {2, {line({Point(0, 0), Point(9, 0)})}}});

    const Polylines owner{line({Point(0, 0), Point(100, 0)})};
    const BAC::PartitionOwnerBounds owner_bounds(owner);
    for (const auto candidate : {line({Point(0, 4), Point(100, 4)}),
                                 line({Point(104, 0), Point(120, 0)})}) {
        CHECK(owner_bounds.may_match(BAC::PartitionCandidateBounds(candidate)));
        check_owners(candidate, {{0, owner}});
    }
    for (const auto candidate : {line({Point(0, 5), Point(100, 5)}),
                                 line({Point(105, 0), Point(120, 0)})}) {
        CHECK_FALSE(owner_bounds.may_match(BAC::PartitionCandidateBounds(candidate)));
        CHECK(check_owners(candidate, {{0, owner}}) == 1);
    }
}

TEST_CASE("Bridge aware owner bounds use checked wide coordinates and conservative fallbacks", "[BridgeAwareCubic][OwnerBounds]")
{
    constexpr coord_t limit = coord_t(std::min<int64_t>(
        std::numeric_limits<coord_t>::max() / 4, std::numeric_limits<int32_t>::max() / 4));
    for (const coord_t origin : {-limit, coord_t(limit - 200)}) {
        const auto owner = line({Point(origin, origin), Point(origin + 100, origin)});
        const auto overlapping = line({Point(origin + 20, origin + 2), Point(origin + 80, origin + 2)});
        const auto distant = line({Point(origin + 20, origin + 20), Point(origin + 80, origin + 20)});
        CHECK(check_owners(overlapping, {{5, {owner}}}) == 0);
        CHECK(check_owners(distant, {{5, {owner}}}) == 1);
    }
    const auto candidate = line({Point(0, 100), Point(100, 100)});
    const BAC::PartitionCandidateBounds bounds(candidate);
    for (const Polylines owner : {Polylines{}, Polylines{Polyline{}},
                                 Polylines{line({Point(0, 0)})},
                                 Polylines{line({Point(0, 0), Point(0, 0), Point(100, 0)})},
                                 Polylines{line({Point(limit + 1, 0), Point(limit + 100, 0)})}})
        CHECK(BAC::PartitionOwnerBounds(owner).may_match(bounds));

    const BAC::PartitionOwnerBounds normal({line({Point(0, 0), Point(100, 0)})});
    for (const auto query : {Polyline{}, line({Point(1000, 1000)}),
                             line({Point(1000, 1000), Point(1000, 1000)}),
                             line({Point(limit + 1, 0), Point(limit + 100, 0)}),
                             line({Point(std::numeric_limits<coord_t>::min(), 0),
                                   Point(std::numeric_limits<coord_t>::max(), 0)})})
        CHECK(normal.may_match(BAC::PartitionCandidateBounds(query)));
}
