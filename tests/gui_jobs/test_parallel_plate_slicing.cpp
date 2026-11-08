#include <catch2/catch.hpp>
#include "slic3r/GUI/ParallelPlateSlicing.hpp"

using Slic3r::GUI::ParallelPlateSlicing;

TEST_CASE("Plate admission is bounded and completion is attributed", "[ParallelPlate][Queue]")
{
    ParallelPlateSlicing batch;
    const auto generation = batch.begin({3, 7, 9, 12}, 2);
    REQUIRE(batch.take_available() == std::vector<int>{3, 7});
    REQUIRE(batch.take_available().empty());
    REQUIRE_FALSE(batch.finish(9, generation, true)); // Not admitted yet.
    REQUIRE(batch.progress(7, generation, 60));
    REQUIRE(batch.percent() == Approx(15.));
    REQUIRE(batch.finish(7, generation, true));
    REQUIRE_FALSE(batch.finish(7, generation, true));
    REQUIRE(batch.take_available() == std::vector<int>{9});
    REQUIRE(batch.finish(3, generation, false));
    REQUIRE(batch.failed());
    REQUIRE(batch.take_available() == std::vector<int>{12});
    REQUIRE(batch.finish(12, generation, true));
    REQUIRE_FALSE(batch.done());
    REQUIRE(batch.finish(9, generation, true));
    REQUIRE(batch.done());
    REQUIRE(batch.percent() == Approx(100.));
}

TEST_CASE("Cancel and replacement reject queued stale plate events", "[ParallelPlate][Queue]")
{
    ParallelPlateSlicing batch;
    auto old = batch.begin({0, 1, 2}, 2);
    REQUIRE(batch.take_available().size() == 2);
    batch.cancel();
    REQUIRE_FALSE(batch.active());
    REQUIRE(batch.take_available().empty());
    REQUIRE_FALSE(batch.finish(0, old, true));
    auto current = batch.begin({0, 2}, 1);
    REQUIRE(current != old);
    REQUIRE(batch.take_available() == std::vector<int>{0});
    REQUIRE_FALSE(batch.progress(0, old, 100));
    REQUIRE_FALSE(batch.finish(0, old, true));
    REQUIRE(batch.finish(0, current, true));
    REQUIRE(batch.take_available() == std::vector<int>{2});
}

TEST_CASE("Plate concurrency leaves CPU and memory headroom", "[ParallelPlate][Queue]")
{
    constexpr std::uint64_t gib = std::uint64_t(1024) * 1024 * 1024;
    REQUIRE(ParallelPlateSlicing::concurrency(10, 16, 128 * gib) == 4);
    REQUIRE(ParallelPlateSlicing::concurrency(10, 16, 8 * gib) == 2);
    REQUIRE(ParallelPlateSlicing::concurrency(10, 2, 128 * gib) == 1);
    REQUIRE(ParallelPlateSlicing::concurrency(10, 0, 0) == 1);
    REQUIRE(ParallelPlateSlicing::concurrency(1, 16, 128 * gib) == 1);
    ParallelPlateSlicing batch;
    batch.begin({}, 4);
    REQUIRE(batch.done());
    REQUIRE(batch.take_available().empty());
}
