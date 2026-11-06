#include <catch2/catch.hpp>
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"

#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicFailureReplay.hpp"
#include "libslic3r/Fill/BridgeAwareCubicInteriorShadow.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSupport.hpp"

#include <stdexcept>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Contact = Slic3r::BridgeAwareCubicContact;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
using Replay = BAC::ModelFailureReplay;

ExtrusionPath bead(Vec2d from, Vec2d to)
{
    ExtrusionPath result(erInternalInfill, .08, .4f, .2f);
    result.polyline = Polyline::new_scale({from, to});
    return result;
}

ExPolygon rectangle(double left, double bottom, double right, double top)
{
    return ExPolygon(Polygon({Point::new_scale(left, bottom), Point::new_scale(right, bottom),
                              Point::new_scale(right, top), Point::new_scale(left, top)}));
}

Validation::Result check_span(const ExtrusionPath &lower, const ExtrusionPath &upper, Validation::WorkBudget &budget)
{
    const auto contact = Contact::footprint({{&lower, .2}}, upper, .4, 0., Contact::FootprintMode::ContactsOnly, &budget);
    return Validation::validate_contacts({}, {}, {upper.polyline}, {rectangle(-2., -1., 12., 1.)}, 5., 0., {contact}, &budget);
}
}

TEST_CASE("Bridge aware failure replay resolves only the same frozen path and exact phase",
          "[BridgeAwareCubic][FailureReplay]")
{
    Replay::Hint hint{17, 1, {}, Replay::Kind::Run};
    hint.phase.shift = Vec2d(.00024, .00032);
    Validation::WorkBudget budget;
    const auto paths = [](size_t index) { return index == 1 ? size_t(2) : size_t(1); };
    const auto cancel = [] {};
    const auto probe = Replay::resolve(hint, {11, 17, 23}, paths, budget, cancel);
    REQUIRE(probe);
    CHECK(probe->target == 1);
    CHECK(probe->path == 1);
    CHECK(probe->includes_phase(hint.phase));
    CHECK(budget.remaining_work == 50000000 - 2);
    CHECK(budget.charged_work[size_t(Validation::WorkCategory::Validation)] == 2);
    BAC::SerializedXY compensated = hint.phase;
    compensated.shift.x() += 1.;
    compensated.plate_origin.x() += 1.;
    CHECK_FALSE(probe->includes_phase(compensated)); // Same net offset is not the same operation order.
    CHECK_FALSE(Replay::resolve(hint, {11, 23}, paths, budget, cancel));
    CHECK_FALSE(Replay::resolve(hint, {17}, paths, budget, cancel)); // Path 1 absent in this group.
    budget.remaining_work = 1;
    CHECK_FALSE(Replay::resolve(hint, {11, 17}, paths, budget, cancel));
    CHECK(budget.remaining_work == 0);
    CHECK_THROWS(Replay::resolve(hint, {17}, paths, budget, [] { throw std::runtime_error("canceled"); }));
}

TEST_CASE("Bridge aware failure replay storage is bounded and region scoped",
          "[BridgeAwareCubic][FailureReplay]")
{
    Replay replay(1);
    const BAC::CellId cell{{{1, 2, 3}}, 2};
    Replay::Hint hint{17, 0, {}, Replay::Kind::Cell};
    REQUIRE(replay.remember({0, cell}, hint));
    CHECK_FALSE(replay.find({1, cell}));
    CHECK_FALSE(replay.remember({1, cell}, hint));
    hint.path = 2;
    REQUIRE(replay.remember({0, cell}, hint));
    REQUIRE(replay.find({0, cell}));
    CHECK(replay.find({0, cell})->path == 2);
    CHECK(replay.size() == 1);
    replay.erase({0, cell});
    CHECK_FALSE(replay.find({0, cell}));
    REQUIRE(replay.remember({1, cell}, hint));
    Replay disabled(0);
    CHECK_FALSE(disabled.remember({0, cell}, hint));
}

TEST_CASE("Bridge aware failure replay passing probe cannot replace the full model check",
          "[BridgeAwareCubic][FailureReplay]")
{
    const ExtrusionPaths upper{bead({0., .1}, {10., .1}), bead({0., .25}, {10., .25})};
    ExtrusionPath lower = bead({-1., 0.}, {11., 0.});
    Validation::WorkBudget original_budget;
    REQUIRE(check_span(lower, upper[0], original_budget).passed());
    const auto original = check_span(lower, upper[1], original_budget);
    REQUIRE(original.input_valid);
    REQUIRE(original.completed);
    const auto kind = Replay::failure_kind(original);
    REQUIRE(kind);
    CHECK(*kind == Replay::Kind::Run);
    const Replay::Hint hint{17, 1, {}, *kind};

    // Actual changed material supports the old failing path while removing
    // contact from another immutable path. Replaying the saved outcome would
    // be wrong; accepting solely because the fresh probe passes is also wrong.
    lower.polyline.translate(Point::new_scale(0., .35));
    Validation::WorkBudget shared;
    const auto probe = Replay::resolve(hint, {17}, [&](size_t) { return upper.size(); }, shared, [] {});
    REQUIRE(probe);
    const size_t before_probe = shared.remaining_work;
    const auto current_probe = check_span(lower, upper[probe->path], shared);
    REQUIRE(current_probe.passed());
    CHECK_FALSE(Replay::failure_kind(current_probe));
    CHECK(shared.remaining_work < before_probe);
    const size_t after_probe = shared.remaining_work;
    const auto other = check_span(lower, upper[0], shared);
    REQUIRE(other.input_valid);
    REQUIRE(other.completed);
    CHECK_FALSE(other.passed());
    CHECK(shared.remaining_work < after_probe);

    // Both attempts consume one allowance. An incomplete second check cannot
    // become a geometric rejection hint, nor can the first check refill it.
    Validation::WorkBudget limited;
    limited.remaining_work = before_probe - after_probe + 1;
    REQUIRE(check_span(lower, upper[1], limited).passed());
    const auto incomplete = check_span(lower, upper[0], limited);
    CHECK_FALSE(incomplete.completed);
    CHECK(incomplete.resource_limited);
    CHECK_FALSE(Replay::failure_kind(incomplete));
    auto invalid = original;
    invalid.input_valid = false;
    CHECK_FALSE(Replay::failure_kind(invalid));
}

TEST_CASE("Bridge aware failure replay cell hint retains a whole physical opening",
          "[BridgeAwareCubic][FailureReplay]")
{
    ExtrusionPaths lower{bead({0., 0.}, {18., 0.}), bead({18., 0.}, {18., 2.}),
                         bead({18., 2.}, {0., 2.}), bead({0., 2.}, {0., 0.})};
    const ExtrusionPath upper = bead({9., 0.}, {9., 2.});
    std::vector<Contact::Bead> deposited;
    for (const auto &path : lower) deposited.push_back({&path, .2});
    Validation::WorkBudget budget;
    const auto contact = Contact::footprint(deposited, upper, .4, 0., Contact::FootprintMode::Full, &budget);
    REQUIRE(contact.input_valid);
    REQUIRE(contact.completed);
    const ExPolygons selection{rectangle(8.99, 0., 9.01, 2.)};
    ExPolygons all_layer_demands = selection;
    all_layer_demands.push_back(rectangle(20., 0., 20.1, 1.));
    const auto domain = BAC::support_topology_domain(contact.physical_footprint, all_layer_demands);
    const auto prepared = Validation::prepare_geometry(domain, contact.physical_footprint, &budget);
    const auto result = Validation::validate_prepared_contacts_for_trial(*prepared,
        {upper.polyline}, selection, 5., 0., {contact}, &budget);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    CHECK(result.max_unsupported_mm < 5.);
    const auto kind = Replay::failure_kind(result);
    REQUIRE(kind);
    CHECK(*kind == Replay::Kind::Cell);
    const Replay::Hint hint{23, 0, {}, *kind};
    const auto probe = Replay::resolve(hint, {11, 23}, [](size_t) { return size_t(1); }, budget, [] {});
    REQUIRE(probe);
    CHECK(probe->target == 1);
    CHECK(probe->path == 0);
    const auto replayed = Validation::validate_prepared_contacts_for_trial(*prepared,
        {upper.polyline}, selection, 5., 0., {contact}, &budget);
    REQUIRE(replayed.input_valid);
    REQUIRE(replayed.completed);
    CHECK(Replay::failure_kind(replayed) == kind);
    REQUIRE_FALSE(result.void_cells.empty());
    CHECK(result.max_void_diameter_mm > 17.);
    // The 20-micrometre selection selects the complete 18-mm opening. It
    // supplies neither a clipped computation window nor artificial walls.
    CHECK(std::any_of(result.void_cells.begin(), result.void_cells.end(), [](const auto &cell) {
        return cell.closed && cell.exceeds_gap && cell.diameter_mm > 17.;
    }));
}

TEST_CASE("Sparse rejection audit distinguishes different failures of the same kind",
          "[BridgeAwareCubic][FailureReplay]")
{
    const auto charge = [](size_t) { return true; };
    Validation::Result reported;
    // Completed physical counterexamples normally carry human-readable reasons.
    // The complete/valid/resource status and failing geometry are authoritative.
    reported.failure_reason = "Unsupported strand lacks two actual end anchors.";
    SECTION("air interval shifted on the same path with unchanged length and classification") {
        reported.unsupported_runs.push_back({3, 1., 3., 2., .001, Validation::UnsupportedRunKind::Floating, false});
        const auto original = reported;
        Replay::ExactWitness witness;
        REQUIRE(witness.take(reported, 32768, charge));
        REQUIRE(witness.matches(original, charge));
        reported.unsupported_runs.front().begin_mm += .25;
        reported.unsupported_runs.front().end_mm += .25;
        REQUIRE(Replay::failure_kind(original) == Replay::failure_kind(reported));
        CHECK_FALSE(witness.matches(reported, charge));
    }
    SECTION("a different opening has the same diameter and status") {
        reported.void_cells.push_back({rectangle(0., 0., 8., 1.), std::hypot(8., 1.), true, true});
        auto other = reported;
        Replay::ExactWitness witness;
        REQUIRE_FALSE(witness.take(reported, 1, charge));
        REQUIRE(witness.take(reported, 32768, charge));
        REQUIRE(witness.matches(other, charge));
        other.void_cells.front().geometry.translate(Point::new_scale(.1, 0.));
        REQUIRE(Replay::failure_kind(other) == Replay::Kind::Cell);
        CHECK_FALSE(witness.matches(other, charge));
    }
}

TEST_CASE("Sparse generation restores original owned fills and provenance on unwind",
          "[BridgeAwareCubic][FailureReplay]")
{
    ExtrusionEntityCollection fills;
    fills.no_sort = true;
    fills.loop_node_range = {4, 8};
    fills.entities.push_back(new ExtrusionPath(bead({1., 2.}, {3., 4.})));
    const auto *const base_fill = fills.entities.front();
    const auto *const base_owners = fills.entities.data();
    const auto base_capacity = fills.entities.capacity();
    std::vector<std::shared_ptr<const int>> snapshots{std::make_shared<const int>(42)};
    const auto base_token = snapshots.front();
    const auto *const base_slots = snapshots.data();
    struct ProbeInterrupted {};
    CHECK_THROWS_AS(([&] {
        Replay::ScopedLowerStage stage(fills, snapshots);
        REQUIRE(fills.entities.empty());
        REQUIRE(snapshots.empty());
        fills.entities.push_back(new ExtrusionPath(bead({0., 0.}, {9., 0.})));
        snapshots.push_back(std::make_shared<const int>(7));
        throw ProbeInterrupted{};
    }()), ProbeInterrupted);
    REQUIRE(fills.entities.size() == 1);
    CHECK(fills.entities.front() == base_fill);
    CHECK(fills.entities.data() == base_owners);
    CHECK(fills.entities.capacity() == base_capacity);
    CHECK(fills.no_sort);
    CHECK(fills.loop_node_range.first == 4);
    CHECK(fills.loop_node_range.second == 8);
    REQUIRE(snapshots.size() == 1);
    CHECK(snapshots.data() == base_slots);
    CHECK(snapshots.front() == base_token);
}

TEST_CASE("Parent-interior shadow demands a selected connected chord, not a large remote cell",
          "[BridgeAwareCubic][FirstSweepInterior]") {
    namespace S=BAC::InteriorShadow;
    const auto yes=[](size_t){return true;};
    const auto p=[](double x,double y){return Point::new_scale(x,y);};
    const S::Chord c{p(3,3),p(1,1),p(9,9)};
    auto window=rectangle(0,0,10,10);
    CHECK(S::contains(window,c,yes));
    CHECK(S::long_enough(c.a,c.b,11.));
    CHECK_FALSE(S::long_enough(p(1,1),p(4,5),5.));
    CHECK_FALSE(S::contains(window,S::Chord{p(3,3),p(0,1),p(9,9)},yes));
    auto hole=rectangle(5,5,6,6).contour;hole.make_clockwise();window.holes.push_back(hole);
    CHECK_FALSE(S::contains(window,c,yes)); // Connector crosses a hole despite all three endpoints being in material.
    CHECK_FALSE(S::inside_any(ExPolygons{window},S::Chord{p(5.5,5.5),p(5.5,5.5),p(5.5,5.5)},yes));
    CHECK_FALSE(S::contains(rectangle(20,0,30,10),c,yes)); // Same diameter is not the same opening.
    CHECK_FALSE(S::tube_disjoint(p(0,0),p(4,0),p(2,1),p(2,3),scale_(1.)));
    CHECK(S::tube_disjoint(p(0,0),p(4,0),p(2,1),p(2,3),scale_(1.)-1));
    CHECK_FALSE(S::tube_disjoint(p(3,3),p(3,3),p(3,3),p(3,3),0)); // Even an actual zero-width axis is retained.
    CHECK_FALSE(S::contains(rectangle(0,0,10,10),c,[](size_t){return false;}));
}
TEST_CASE("First-sweep geometric selection uses a finite positive path mask and a strict inset",
          "[BridgeAwareCubic][FirstSweepInterior]") {
    namespace S=BAC::InteriorShadow;
    const auto yes=[](size_t){return true;};
    const auto parent=rectangle(0,0,12,12);
    const auto upper=bead({-3.,6.},{15.,6.});
    S::Chord c;
    REQUIRE(S::propose(parent.contour,upper.polyline,{rectangle(.5,5.9,11.5,6.1)},8.,scale_(.4),c,yes));
    CHECK(S::contains(parent,c,yes));
    CHECK_FALSE(S::propose(parent.contour,upper.polyline,{rectangle(0,8,12,9)},8.,scale_(.4),c,yes));
    CHECK_FALSE(S::propose(parent.contour,upper.polyline,{parent},30.,scale_(.4),c,yes));
    CHECK_FALSE(S::propose(parent.contour,upper.polyline,{parent},8.,scale_(8.),c,yes));
}

TEST_CASE("First-sweep chord challenge identifies an actual selected failing cell and rejects a dividing stroke",
          "[BridgeAwareCubic][FirstSweepInterior]") {
    namespace S=BAC::InteriorShadow;
    const auto pos=[](double x,double y){return Point::new_scale(x,y);};
    const S::Chord chord{pos(6,6),pos(2,2),pos(10,10)};
    const auto yes=[](size_t){return true;};
    const auto upper=bead({1.,6.},{11.,6.});
    ExtrusionPaths lower{bead({0.,-1.},{0.,13.}),bead({12.,-1.},{12.,13.}),
                         bead({-1.,0.},{13.,0.}),bead({-1.,12.},{13.,12.})};
    const bool divider=GENERATE(false,true);
    if (divider) lower.push_back(bead({6.,-1.},{6.,13.}));
    std::vector<Contact::Bead> beads;
    for(const auto &path:lower) beads.push_back({&path,.2});
    auto prep=Contact::prepare_lower(beads,.4,upper.height);
    const auto raw=Contact::footprint(*prep,upper,.4,0.,Contact::FootprintMode::ContactsOnly);
    REQUIRE(raw.input_valid); REQUIRE(raw.completed);
    const auto physical=Contact::physical_sections(*prep);
    REQUIRE(physical.input_valid); REQUIRE(physical.completed);
    Polylines air;
    for (const auto &seg:raw.segments) {
        const auto i=seg.segment_index;
        append(air,diff_pl(Polylines{Polyline(upper.polyline.points[i],upper.polyline.points[i+1])},seg.centerline_support));
    }
    const auto selected=intersection_ex(ExPolygons{rectangle(0,5.9,12,6.1)},union_ex(offset(air,scale_(.5*upper.width))));
    const auto checked=Validation::validate_contacts({rectangle(-3,-3,15,15)},physical.physical_footprint,
        {upper.polyline},selected,8.,0.,{raw});
    REQUIRE(checked.input_valid); REQUIRE(checked.completed); REQUIRE_FALSE(checked.passed());
    bool any_bad=false, matching=false;
    for (const auto &cell:checked.void_cells) if (!cell.closed || cell.exceeds_gap) {
        any_bad=true;
        matching |= S::contains(cell.geometry,chord,yes);
    }
    REQUIRE(any_bad); // Both variants still have an oversized opening on this path.
    CHECK(matching == !divider); // Only the undivided opening contains the proposed connected chord.
    if (!divider) CHECK(S::inside_any(selected,S::Chord{chord.q,chord.q,chord.q},yes));
}
