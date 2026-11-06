#include <catch2/catch.hpp>
#include "libslic3r/Fill/BridgeAwareCubicStrands.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace {
constexpr double root3 = 1.7320508075688772935;
BAC::CellId id(int64_t j, int64_t k) { return {{{0, j, k}}, 0}; }
const BAC::StrandPlane row{0, 1, 1};
BAC::Plan actual_plan(std::function<void()> cancel = {})
{
    BAC::Parameters p;
    p.bounds = BoundingBoxf3(Vec3d(-4., -4., -2.), Vec3d(4., 4., 3.));
    p.flow_spacing = .125; p.density = .25; p.angle = .37; p.throw_if_canceled = std::move(cancel);
    const double e = std::sqrt(6.) * p.flow_spacing / p.density;
    const ExPolygon square(Polygon({Point::new_scale(-3., -3.), Point::new_scale(3., -3.),
                                    Point::new_scale(3., 3.), Point::new_scale(-3., 3.)}));
    return BAC::Plan(p, {{{square}, 0., .9 * e}, {{square}, e / root3, .9 * e}});
}
void only(BAC::StrandProgram &program, std::set<BAC::CellId> keep, BAC::StrandBudget &budget)
{
    std::set<BAC::CellId> seen;
    for (const auto &run : program.runs(row, budget))
        for (const auto &atom : run.atoms) seen.insert(atom.owner);
    for (const auto &wanted : keep) REQUIRE(seen.count(wanted) == 1);
    for (const auto &owner : seen) if (!keep.count(owner)) program.set_owner(owner, false, budget);
}
void equal_runs(const std::vector<BAC::StrandRun> &a, const std::vector<BAC::StrandRun> &b)
{
    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        REQUIRE(a[i].points == b[i].points);
        REQUIRE(a[i].atoms.size() == b[i].atoms.size());
        REQUIRE(a[i].points.size() == a[i].atoms.size() + 1);
        for (size_t j = 0; j < a[i].atoms.size(); ++j) {
            CHECK(a[i].atoms[j].first == b[i].atoms[j].first);
            CHECK(a[i].atoms[j].last == b[i].atoms[j].last);
            CHECK(a[i].atoms[j].owner == b[i].atoms[j].owner);
        }
    }
}
void equal_programs(const BAC::StrandProgram &a, const BAC::StrandProgram &b, BAC::StrandBudget &budget)
{
    const auto rows = a.planes(budget);
    REQUIRE(rows == b.planes(budget));
    for (const auto &key : rows) equal_runs(a.runs(key, budget), b.runs(key, budget));
}
}

TEST_CASE("Canonical cubic strands retain the finite producer at each original breakpoint", "[BridgeAwareCubic][CanonicalStrands]")
{
    auto plan = actual_plan(); BAC::StrandBudget budget;
    auto program = plan.strand_program(0., budget);
    REQUIRE_FALSE(program.planes(budget).empty());
    const auto left = id(-1, 0), middle = id(-1, -1), right = id(0, -1);
    only(program, {left, middle, right}, budget);
    const auto before = program.runs(row, budget);
    REQUIRE(before.size() == 1); REQUIRE(before[0].atoms.size() == 3); REQUIRE(before[0].points.size() == 4);
    CHECK(before[0].atoms[0].owner == left); CHECK(before[0].atoms[1].owner == middle); CHECK(before[0].atoms[2].owner == right);
    const BAC::StrandEvent starts[] = {{-1,false},{-1,true},{0,false}}, ends[] = {{-1,true},{0,false},{0,true}};
    for (unsigned i = 0; i != 3; ++i) {
        CHECK(before[0].atoms[i].first == starts[i]); CHECK(before[0].atoms[i].last == ends[i]);
    }
    const auto touched = program.set_owner(middle, false, budget);
    CHECK(std::find(touched.begin(), touched.end(), row) != touched.end()); CHECK(touched.size() <= 3);
    CHECK(program.set_owner(middle, false, budget).empty());
    const auto after = program.runs(row, budget);
    REQUIRE(after.size() == 2);
    CHECK(after[0].points == Points({before[0].points[0], before[0].points[1]}));
    CHECK(after[1].points == Points({before[0].points[2], before[0].points[3]}));
    program.set_owner(middle, true, budget);
    equal_runs(program.runs(row, budget), before);
}

TEST_CASE("Canonical cubic strands never join through a removed stationary rounded atom", "[BridgeAwareCubic][CanonicalStrands]")
{
    auto plan = actual_plan(); BAC::StrandBudget budget;
    const double e = plan.bulk_cell_edge(), tie = (.5 * e) / root3;
    REQUIRE(tie * root3 == .5 * e);
    const double above = std::nextafter(tie, std::numeric_limits<double>::infinity());
    REQUIRE(above * root3 > .5 * e);
    auto program = plan.strand_program(above, budget);
    const auto left = id(-1, 0), tiny = id(0, 0), right = id(0, -1);
    only(program, {left, tiny, right}, budget);
    const auto full = program.runs(row, budget);
    REQUIRE(full.size() == 1); REQUIRE(full[0].atoms.size() == 3);
    CHECK(full[0].atoms[1].owner == tiny); CHECK(full[0].points[1] == full[0].points[2]);
    CHECK_FALSE(full[0].atoms[1].first == full[0].atoms[1].last);
    program.set_owner(tiny, false, budget);
    const auto cut = program.runs(row, budget);
    REQUIRE(cut.size() == 2); CHECK(cut[0].points.back() == cut[1].points.front());
    auto tied = plan.strand_program(tie, budget);
    only(tied, {left, right}, budget);
    const auto exact = tied.runs(row, budget);
    REQUIRE(exact.size() == 1); REQUIRE(exact[0].atoms.size() == 2);
    CHECK(exact[0].atoms[0].last == exact[0].atoms[1].first);
    CHECK_FALSE(exact[0].atoms[0].last.shifted);
}

TEST_CASE("Canonical cubic incremental owner edits equal a freshly collapsed real Plan", "[BridgeAwareCubic][CanonicalStrands]")
{
    auto plan = actual_plan(); BAC::StrandBudget budget;
    const auto removed = id(-1,-1);
    auto cold = plan.collapsed(removed);
    std::set<BAC::CellId> retained;
    for (const auto &r : cold.refinements()) if (!r.solid) retained.insert(r.id);
    for (double z : {-0.17, 0., .13, .77}) {
        CAPTURE(z);
        auto live = plan.strand_program(z, budget), original = plan.strand_program(z, budget);
        REQUIRE_FALSE(original.planes(budget).empty());
        std::vector<BAC::CellId> removed_here;
        for (const auto &r : plan.refinements()) if (!retained.count(r.id) && live.contains_owner(r.id, budget)) {
            live.set_owner(r.id, false, budget); removed_here.push_back(r.id);
        }
        equal_programs(live, cold.strand_program(z, budget), budget);
        for (auto i = removed_here.rbegin(); i != removed_here.rend(); ++i) live.set_owner(*i, true, budget);
        equal_programs(live, original, budget);
    }
}

TEST_CASE("Canonical cubic strand limits and cancellation preserve the previous owner state", "[BridgeAwareCubic][CanonicalStrands]")
{
    struct Canceled {};
    bool cancel = false;
    auto plan = actual_plan([&]{ if (cancel) throw Canceled{}; });
    BAC::StrandBudget budget;
    auto program = plan.strand_program(0., budget);
    const auto before = program.runs(row, budget); REQUIRE_FALSE(before.empty());
    BAC::StrandBudget short_work{0,200000}, short_storage{5000000,1};
    CHECK_THROWS_AS(plan.strand_program(0., short_storage), std::length_error);
    CHECK_THROWS_AS(program.set_owner(id(-1,-1), false, short_work), std::length_error);
    CHECK_THROWS_AS(program.runs(row, short_storage), std::length_error);
    cancel = true;
    CHECK_THROWS_AS(program.set_owner(id(-1,-1), false, budget), Canceled);
    cancel = false; equal_runs(program.runs(row, budget), before);
    CHECK_THROWS_AS(program.set_owner(id(999,999), true, budget), std::out_of_range);
}
