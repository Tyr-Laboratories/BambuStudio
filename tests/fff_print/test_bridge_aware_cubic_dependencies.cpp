#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicDependencies.hpp"

#include <algorithm>
#include <stdexcept>

namespace Dependencies = Slic3r::BridgeAwareCubic::Dependencies;

namespace {

Dependencies::SourceOwner owner(size_t region, int64_t address)
{
    return {region, {{{address, 0, 0}}, 1}};
}

} // namespace

TEST_CASE("Bridge aware ownership follows rooted support across region lattices",
          "[BridgeAwareCubic][Dependencies]")
{
    const auto upper = owner(0, 10), lower = owner(1, 20), base = owner(2, 30);
    const std::vector<Dependencies::Producer> producers{
        {upper, {7}}, {lower, {4}}, {base, {9}}};
    const std::vector<Dependencies::Obligation> obligations{
        {{1, 4}, {upper}}, {{2, 9}, {lower}}};
    const auto result = Dependencies::resolve({{0, 7}}, producers, obligations);
    const std::vector<Dependencies::DemandKey> expected{{0, 7}, {1, 4}, {2, 9}};
    CHECK(result.active_demands == expected);
    CHECK(result.is_active(upper));
    CHECK(result.is_active(lower));
    CHECK(result.is_active(base));
    CHECK_FALSE(result.is_active(Dependencies::DemandKey{0, 4}));
}

TEST_CASE("Bridge aware ownership discards unrooted cross-region cycles",
          "[BridgeAwareCubic][Dependencies]")
{
    const auto first = owner(0, 1), second = owner(1, 1);
    const std::vector<Dependencies::Producer> producers{{first, {3}}, {second, {5}}};
    const std::vector<Dependencies::Obligation> obligations{
        {{0, 3}, {second}}, {{1, 5}, {first}}};
    const auto orphaned = Dependencies::resolve({}, producers, obligations);
    CHECK(orphaned.active_demands.empty());
    CHECK(orphaned.active_sources.empty());

    // The same cycle is legitimate while a surviving producer is required by
    // a model target. Removing that root must release both plans together.
    const auto rooted = Dependencies::resolve({{0, 3}}, producers, obligations);
    CHECK(rooted.active_demands.size() == 2);
    CHECK(rooted.active_sources.size() == 2);
}

TEST_CASE("Bridge aware ownership only follows currently surviving producers",
          "[BridgeAwareCubic][Dependencies]")
{
    const auto collapsed = owner(0, 1), lower = owner(1, 2), base = owner(2, 3);
    const std::vector<Dependencies::Producer> producers{{lower, {2}}, {base, {3}}};
    const std::vector<Dependencies::Obligation> obligations{
        {{1, 2}, {collapsed}}, {{2, 3}, {lower}}, {{1, 2 + 10}, {base}}};
    const auto result = Dependencies::resolve({{0, 1}}, producers, obligations);
    CHECK(result.active_demands.size() == 1);
    CHECK(result.active_sources.empty());
    CHECK_FALSE(result.is_active(Dependencies::DemandKey{1, 2}));
}

TEST_CASE("Bridge aware ownership treats complete fragment producers as alternatives",
          "[BridgeAwareCubic][Dependencies]")
{
    const auto removed = owner(0, 1), retained = owner(1, 1), support = owner(2, 1);
    const std::vector<Dependencies::Producer> producers{{retained, {4}}, {support, {8}}};
    const auto result = Dependencies::resolve({{1, 4}}, producers,
        {{{2, 8}, {removed, retained}}});
    CHECK(result.is_active(Dependencies::DemandKey{2, 8}));
    CHECK(result.is_active(support));
    CHECK_FALSE(result.is_active(removed));
}

TEST_CASE("Bridge aware ownership never substitutes an ancestor or an empty owner list",
          "[BridgeAwareCubic][Dependencies]")
{
    auto ancestor = owner(0, 0), removed_child = ancestor;
    ancestor.cell.depth = 0;
    removed_child.cell.depth = 1;
    const auto result = Dependencies::resolve({{0, 0}}, {{ancestor, {0}}},
        {{{1, 1}, {removed_child}}, {{1, 2}, {}}});
    CHECK(result.is_active(ancestor));
    CHECK_FALSE(result.is_active(removed_child));
    CHECK_FALSE(result.is_active(Dependencies::DemandKey{1, 1}));
    CHECK_FALSE(result.is_active(Dependencies::DemandKey{1, 2}));
}

TEST_CASE("Bridge aware ownership never aliases identical cell addresses in other regions",
          "[BridgeAwareCubic][Dependencies]")
{
    const auto first = owner(0, 3), unrelated = owner(1, 3), lower = owner(2, 4);
    const auto result = Dependencies::resolve({{0, 0}},
        {{first, {0}}, {unrelated, {0}}, {lower, {1}}},
        {{{2, 1}, {unrelated}}});
    CHECK(result.is_active(first));
    CHECK_FALSE(result.is_active(unrelated));
    CHECK_FALSE(result.is_active(lower));
    CHECK_FALSE(result.is_active(Dependencies::DemandKey{2, 1}));
}

TEST_CASE("Bridge aware ownership preserves sparse stable demand keys and deterministic ordering",
          "[BridgeAwareCubic][Dependencies]")
{
    const auto first = owner(9, 8), second = owner(2, -4);
    std::vector<Dependencies::Producer> producers{{first, {500, 500}}, {second, {900}}};
    std::vector<Dependencies::Obligation> obligations{{{2, 900}, {first, first}}};
    const auto result = Dependencies::resolve({{9, 500}, {9, 500}}, producers, obligations);
    std::reverse(producers.begin(), producers.end());
    const auto reordered = Dependencies::resolve({{9, 500}}, producers, obligations);
    CHECK(result.active_demands == reordered.active_demands);
    CHECK(result.active_sources == reordered.active_sources);
    const std::vector<Dependencies::DemandKey> expected_demands{{2, 900}, {9, 500}};
    const std::vector<Dependencies::SourceOwner> expected_sources{second, first};
    CHECK(result.active_demands == expected_demands);
    CHECK(result.active_sources == expected_sources);
}

TEST_CASE("Bridge aware ownership rejects ambiguous duplicate graph records",
          "[BridgeAwareCubic][Dependencies]")
{
    const auto first = owner(0, 1), second = owner(1, 1);
    CHECK_THROWS_AS(Dependencies::resolve({}, {{first, {1}}, {first, {2}}}, {}),
                    std::invalid_argument);
    // Separate fragments must receive separate demand keys. Treating these as
    // alternatives would allow one supported fragment to retain the other.
    CHECK_THROWS_AS(Dependencies::resolve({}, {},
        {{{2, 0}, {first}}, {{2, 0}, {second}}}), std::invalid_argument);
}

TEST_CASE("Bridge aware ownership work limits fail without a partial reachability result",
          "[BridgeAwareCubic][Dependencies]")
{
    Dependencies::Limits limits;
    limits.max_nodes = 0;
    limits.max_edges = 0;
    CHECK(Dependencies::resolve({}, {}, {}, limits).active_demands.empty());
    limits.max_nodes = 1;
    const auto root_only = Dependencies::resolve({{0, 0}}, {}, {}, limits);
    CHECK(root_only.is_active(Dependencies::DemandKey{0, 0}));
    CHECK(root_only.active_sources.empty());
    CHECK_THROWS_AS(Dependencies::resolve({{0, 0}}, {{owner(0, 1), {0}}}, {}, limits),
                    std::runtime_error);
    limits.max_nodes = 2;
    limits.max_edges = 1;
    CHECK(Dependencies::resolve({{0, 0}}, {{owner(0, 1), {0}}}, {}, limits).active_sources.size() == 1);
    limits.max_nodes = 10;
    limits.max_edges = 1;
    CHECK_THROWS_AS(Dependencies::resolve({{0, 0}}, {{owner(0, 1), {0}}},
        {{{1, 1}, {owner(0, 1)}}}, limits), std::runtime_error);
    // Even absent and repeated source references consume the input-work cap.
    CHECK_THROWS_AS(Dependencies::resolve({}, {},
        {{{1, 1}, {owner(0, 1), owner(0, 1)}}}, limits), std::runtime_error);
}

TEST_CASE("Bridge aware ownership propagates cancellation including empty graphs",
          "[BridgeAwareCubic][Dependencies]")
{
    struct Canceled {};
    Dependencies::Limits limits;
    limits.throw_if_canceled = [] { throw Canceled{}; };
    CHECK_THROWS_AS(Dependencies::resolve({}, {}, {}, limits), Canceled);

    size_t checkpoints = 0;
    limits.throw_if_canceled = [&] { if (++checkpoints == 2) throw Canceled{}; };
    std::vector<Dependencies::Producer> producers;
    for (int64_t i = 0; i < 2000; ++i)
        producers.push_back({owner(0, i), {0}});
    CHECK_THROWS_AS(Dependencies::resolve({{0, 0}}, producers, {}, limits), Canceled);
    CHECK(checkpoints == 2);
}
