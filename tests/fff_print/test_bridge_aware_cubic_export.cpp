#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"
#include "bridge_aware_cubic_observer.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace BACFixtures;
using namespace BACObserver;

namespace {

struct ExportFixture {
    std::string        name;
    Model            (*model)();
    DynamicPrintConfig settings;     // on top of fixture_config(gap, pattern) with the fixture's own config function
    double             gap = 5.;
    DynamicPrintConfig (*base)(const char *) = nullptr;   // nullptr: fixture_config(gap, pattern)
};

DynamicPrintConfig settings_of(const ExportFixture &fixture, const char *pattern)
{
    DynamicPrintConfig config = fixture.base != nullptr ? fixture.base(pattern) : fixture_config(fixture.gap, pattern);
    config.apply(fixture.settings, true);
    return config;
}

// The stepped cavity: a 40 x 30 x 14 mm block whose roof over the left 20 mm lies at 8 mm and over the right at 14 mm, both over
// sparse infill, with a 2 mm wall between them as a step.
Model stepped_cavity()
{
    TriangleMesh body(its_make_cube(40., 30., 8.));
    TriangleMesh right(its_make_cube(22., 30., 6.));
    right.translate(18., 0., 8.);
    body.merge(right);
    return single_part("Bridge aware cubic stepped cavity", std::move(body));
}

// The partial roof whose right half (x from 20 mm, the block and the body under it) has no top shell: a parameter modifier with
// top_shell_layers 0. Its only roof over sparse infill is the body's over the left half, so nothing above the right half plans
// lines there and the sparse lines away from that roof must equal Cubic's.
Model half_open_roof()
{
    using namespace Slic3r;
    Model model = partial_roof();
    TriangleMesh mesh(its_make_cube(20., 30., 14.));
    mesh.translate(20., 0., 0.);
    ModelVolume *right = model.objects.front()->add_volume(std::move(mesh));
    right->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    right->name = "Open top modifier";
    right->config.set_key_value("top_shell_layers", new ConfigOptionInt(0));
    return model;
}

// The small cube shifted along x so that the lattice's phase under the roof differs: k · a_0 / 6 for k = 0..5 at 5 %.
Model phased_cube(double shift_mm)
{
    Model model = small_cube();
    model.objects.front()->instances.front()->set_offset(Vec3d(80. + shift_mm, 80., 0.));
    return model;
}

} // namespace

// P2: siblings stay coarse. On the plane layer under the half-open partial roof, outside the roof's footprint grown by the gap,
// the sparse lines equal Cubic's but for the lattice's extreme lines (within one line width of the island's extent across each
// family).
TEST_CASE("Bridge aware cubic leaves the lattice coarse away from the roof", "[BridgeAwareCubic][P2]")
{
    Workspace workspace;
    const DynamicPrintConfig settings = fixture_config(5., "bridgeawarecubic");
    const std::vector<EmittedPath> bac = exported_extrusions(workspace, half_open_roof(), settings, "p2-bac.gcode");
    const std::vector<EmittedPath> cubic = exported_extrusions(workspace, half_open_roof(), fixture_config(5., "cubic"), "p2-cubic.gcode");
    // The plane under the roof, from the Cubic control: the first plane whose roof layer prints solid fill of some extent (a wall
    // seam makes the oracle report slivers of demand on lower layers too). The roof footprint is that solid fill, not the walls,
    // which run round the whole island; the region outside it grown by the gap is where the lattice must stay coarse.
    const std::vector<PlaneMeasure> planes = measure_planes(cubic, 0.4, 5.);
    REQUIRE_FALSE(planes.empty());
    const std::map<int, EmittedLayer> layers_bac = emitted_layers(bac), layers_cubic = emitted_layers(cubic);
    const auto solid_paths_of = [&layers_cubic](int key) {
        EmittedLayer out;
        for (const EmittedPath *path : layers_cubic.at(key))
            if (path->role == erSolidInfill || path->role == erTopSolidInfill || path->role == erBridgeInfill || path->role == erBottomSurface ||
                path->role == erFloatingVerticalShell)
                out.push_back(path);
        return out;
    };
    int plane = 0, roof = 0;
    EmittedLayer roof_paths;
    for (const PlaneMeasure &candidate : planes) {
        EmittedLayer solid = solid_paths_of(candidate.upper);
        if (area(footprint(solid)) * SCALING_FACTOR * SCALING_FACTOR < 50.) continue;
        plane = candidate.lower;
        roof = candidate.upper;
        roof_paths = std::move(solid);
        break;
    }
    REQUIRE(roof != 0);
    REQUIRE(layers_cubic.count(plane) == 1);
    REQUIRE(layers_bac.count(plane) == 1);
    const ExPolygons near_roof = offset_ex(footprint(roof_paths), float(scale_(5.)), ClipperLib::jtRound, scale_(.001));
    // The sparse island's extent, from the Cubic control's sparse footprint.
    EmittedLayer sparse_cubic;
    for (const EmittedPath *path : layers_cubic.at(plane))
        if (path->role == erInternalInfill) sparse_cubic.push_back(path);
    const ExPolygons island = footprint(sparse_cubic);
    const double line_width = 0.4 + 0.02;
    const auto far_from_extremes = [&](const SparseSegment &s) {
        // The segment's family direction and the island's extent across it.
        const Vec2d d = (s.b - s.a).cast<double>().normalized();
        const Vec2d n(-d.y(), d.x());
        double lo = std::numeric_limits<double>::max(), hi = std::numeric_limits<double>::lowest();
        for (const ExPolygon &part : island)
            for (const Point &p : part.contour.points) {
                const double x = n.dot(p.cast<double>());
                lo = std::min(lo, x);
                hi = std::max(hi, x);
            }
        const double x = n.dot(s.a.cast<double>());
        return x - lo > scale_(line_width) && hi - x > scale_(line_width);
    };
    const auto keep = [&](const std::vector<EmittedPath> &paths) {
        std::vector<SparseSegment> out;
        for (const SparseSegment &s : sparse_segments(paths)) {
            if (s.plane != plane) continue;
            if (!intersection_pl(Polylines{Polyline(s.a, s.b)}, near_roof).empty()) continue;
            if (!far_from_extremes(s)) continue;
            out.push_back(s);
        }
        return out;
    };
    const std::vector<SparseSegment> mine = keep(bac), theirs = keep(cubic);
    REQUIRE_FALSE(theirs.empty());
    // Every segment of Cubic's has a counterpart of the pattern's within 0.001 mm, and the pattern adds none away from the roof.
    std::vector<bool> used(mine.size(), false);
    size_t unmatched = 0;
    for (const SparseSegment &s : theirs) {
        bool matched = false;
        for (size_t j = 0; j < mine.size() && !matched; ++j)
            if (!used[j] && (s.a - mine[j].a).cast<double>().norm() <= scale_(.001) && (s.b - mine[j].b).cast<double>().norm() <= scale_(.001)) {
                used[j] = true;
                matched = true;
            }
        unmatched += !matched;
    }
    CHECK(unmatched == 0);
    std::ostringstream extra;
    size_t listed = 0;
    for (size_t j = 0; j < mine.size() && listed < 12; ++j)
        if (!used[j]) {
            extra << " [" << unscale<double>(mine[j].a.x()) << "," << unscale<double>(mine[j].a.y()) << " -> " << unscale<double>(mine[j].b.x()) << "," << unscale<double>(mine[j].b.y()) << "]";
            ++listed;
        }
    INFO("the pattern's segments without a Cubic counterpart:" << extra.str());
    CHECK(mine.size() == theirs.size());
}

// K1: the roof contract on the exported G-code. On every plane where non-sparse material lies over the pattern's void, every
// opening of the void under it and every unsupported run of its paths is within the gap. Cubic fails the same measure on the
// same part, and the measure breaks when the sparse lines of the plane below are taken away.
void check_k1(Workspace &workspace, const std::vector<ExportFixture> &fixtures)
{
    for (const ExportFixture &fixture : fixtures) {
        INFO(fixture.name);
        const DynamicPrintConfig settings = settings_of(fixture, "bridgeawarecubic");
        const std::vector<EmittedPath> paths = exported_extrusions(workspace, fixture.model(), settings, fixture.name + "-bac.gcode");
        const std::vector<PlaneMeasure> planes = measure_planes(paths, 0.4, fixture.gap);
        REQUIRE_FALSE(planes.empty());
        for (const PlaneMeasure &plane : planes) {
            INFO("plane below z = " << plane.lower / 1000. << " mm, roof z = " << plane.upper / 1000. << " mm, bead " << plane.bead);
            CHECK(plane.widest_opening <= fixture.gap);
            CHECK(plane.longest_run <= fixture.gap);
        }
        // Controls: Cubic exceeds the gap on the same part (the small cube's cell is 17.5 mm; at gap 8 the large cube's too), and
        // removing the sparse lines of each plane below breaks the measure.
        const std::vector<EmittedPath> control = exported_extrusions(workspace, fixture.model(), settings_of(fixture, "cubic"), fixture.name + "-cubic.gcode");
        const std::vector<PlaneMeasure> control_planes = measure_planes(control, 0.4, fixture.gap);
        REQUIRE_FALSE(control_planes.empty());
        CHECK(std::any_of(control_planes.begin(), control_planes.end(), [&](const PlaneMeasure &p) { return p.widest_opening > fixture.gap; }));
        std::vector<int> plane_keys;
        for (const PlaneMeasure &p : planes) plane_keys.push_back(p.lower);
        const std::vector<PlaneMeasure> broken = measure_planes(without_added_sparse(paths, control, plane_keys), 0.4, fixture.gap);
        CHECK(std::any_of(broken.begin(), broken.end(), [&](const PlaneMeasure &p) { return p.widest_opening > fixture.gap || p.longest_run > fixture.gap; }));
    }
}

// Roofs that end at walls: the cubes.
TEST_CASE("Bridge aware cubic bounds every opening and run under the roofs", "[BridgeAwareCubic][K1]")
{
    Workspace workspace;
    check_k1(workspace, {
        { "small cube", small_cube, {}, 5. },
        { "large cube", large_cube, {}, 5. },
        { "small cube at 3 %", small_cube, with({}, {{"sparse_infill_density", "3%"}}), 5. },
        { "small cube at 10 %", small_cube, with({}, {{"sparse_infill_density", "10%"}}), 5. },
        { "small cube, gap 3", small_cube, {}, 3. },
        { "large cube, gap 8", large_cube, {}, 8. },
    });

    SECTION("six lattice phases under the roof") {
        const double a0 = 0.357 * 3. / 0.05 * std::sqrt(2. / 3.);
        for (int k = 0; k < 6; ++k) {
            INFO("phase " << k);
            const std::vector<EmittedPath> paths = exported_extrusions(workspace, phased_cube(k * a0 / 6.), fixture_config(5., "bridgeawarecubic"), "phase-" + std::to_string(k) + ".gcode");
            const std::vector<PlaneMeasure> planes = measure_planes(paths, 0.4, 5.);
            REQUIRE_FALSE(planes.empty());
            for (const PlaneMeasure &plane : planes) {
                CHECK(plane.widest_opening <= 5.);
                CHECK(plane.longest_run <= 5.);
            }
        }
    }

    SECTION("a second instance at a fractional offset") {
        Model model = small_cube();
        model.objects.front()->add_instance()->set_offset(Vec3d(80. + 0.37 * 17.49 + 40., 80., 0.));
        const std::vector<EmittedPath> paths = exported_extrusions(workspace, std::move(model), fixture_config(5., "bridgeawarecubic"), "instances.gcode");
        const std::vector<PlaneMeasure> planes = measure_planes(paths, 0.4, 5.);
        REQUIRE_FALSE(planes.empty());
        for (const PlaneMeasure &plane : planes) {
            CHECK(plane.widest_opening <= 5.);
            CHECK(plane.longest_run <= 5.);
        }
    }
}

// Roofs with an edge over the pattern's void (the bridge grows past it: the rails of stage 6b bound the growth) and two lattices
// with a seam (the seam walls of stage 6a bound the gutter). These pass once Tasks 12 and 13 are in; until then this case fails
// and the suite runs with ~[K1edges].
TEST_CASE("Bridge aware cubic bounds every opening and run under roofs with edges over the void", "[BridgeAwareCubic][K1][K1edges]")
{
    Workspace workspace;
    check_k1(workspace, {
        { "rib", rib, {}, 5., rib_config },
        { "modifier", modifier, {}, 8., modifier_config },
        { "partial roof", partial_roof, {}, 5. },
        { "partial roof, two internal bridge layers", partial_roof, with({}, {{"internal_bridge_layers", "2"}}), 5. },
        { "partial roof, three internal bridge layers", partial_roof, with({}, {{"internal_bridge_layers", "3"}}), 5. },
        { "partial roof, thick bridges", partial_roof, with({}, {{"thick_bridges", "1"}}), 5. },
        { "stepped cavity", stepped_cavity, {}, 5. },
    });
}

// K2: the pattern's added lines are held. On the fixtures, every added sparse piece off the hold set of the plane below is at
// most G − 0.1 mm long between held points, and no added line ends free over the void. The Cubic control has no added line, and
// taking away the sparse lines of the layer below the lowest plane leaves that plane's added lines free.
void check_k2(Workspace &workspace, const std::vector<ExportFixture> &fixtures)
{
    for (const ExportFixture &fixture : fixtures) {
        INFO(fixture.name);
        const std::vector<EmittedPath> paths = exported_extrusions(workspace, fixture.model(), settings_of(fixture, "bridgeawarecubic"), fixture.name + "-k2-bac.gcode");
        const std::vector<EmittedPath> control = exported_extrusions(workspace, fixture.model(), settings_of(fixture, "cubic"), fixture.name + "-k2-cubic.gcode");
        // The modifier fixture has two gaps: 3 mm on its right half (x from 24 mm of the part, placed at x = 80), 8 mm on the left.
        const double left_gap = fixture.gap;
        const auto gap_b = [&fixture, left_gap](const Vec2d &at) { return (fixture.name == "modifier" && at.x() >= 80. + 24. ? 3. : left_gap) - 0.1; };
        const K2Measure measure = measure_k2(paths, control, gap_b);
        INFO("added pieces " << measure.added_pieces << ", longest " << measure.longest << " mm, first place " << (measure.where.empty() ? Vec2d(0, 0) : measure.where.front()).transpose());
        CHECK(measure.free_ends == 0);
        CHECK(measure.long_stretches == 0);
        CHECK(measure_k2(control, control, gap_b).added_pieces == 0);
        std::vector<EmittedPath> without;
        const std::vector<PlaneMeasure> planes = measure_planes(paths, 0.4, fixture.gap);
        REQUIRE_FALSE(planes.empty());
        int lowest = planes.front().lower;
        for (const PlaneMeasure &p : planes) lowest = std::min(lowest, p.lower);
        std::vector<int> keys;
        for (const auto &[key, layer] : emitted_layers(paths)) keys.push_back(key);
        const auto lowest_it = std::find(keys.begin(), keys.end(), lowest);
        REQUIRE(lowest_it != keys.begin());
        const int below = *std::prev(lowest_it);
        for (const EmittedPath &path : paths)
            if (!(path.role == erInternalInfill && plane_key(path.z) == below))
                without.push_back(path);
        // Without their holders the plane's lines hang: free ends, or, where an end is fused to another line, long stretches.
        const K2Measure broken = measure_k2(without, control, gap_b);
        CHECK(broken.free_ends + broken.long_stretches > 0);
    }
}

TEST_CASE("Bridge aware cubic holds every line it adds", "[BridgeAwareCubic][K2]")
{
    Workspace workspace;
    check_k2(workspace, {
        { "small cube", small_cube, {}, 5. },
        { "large cube", large_cube, {}, 5. },
        { "rib", rib, {}, 5., rib_config },
        { "box 40 at 3 %", [] { return single_part("Bridge aware cubic box 40", TriangleMesh(its_make_cube(40., 40., 40.))); },
          with({}, {{"sparse_infill_density", "3%"}}), 5. },
    });
}

// Two lattices with a seam: the connector joins the pattern's line ends along the island boundary at the seam, over the seam
// gutter (Cubic's boundary behaviour, accepted), and the sweep finds no holder beyond that boundary. The seam walls of Task 12
// make the seam material; until then this case fails and the suite runs with ~[K2seam].
TEST_CASE("Bridge aware cubic holds every line it adds across a seam", "[BridgeAwareCubic][K2][K2seam]")
{
    Workspace workspace;
    check_k2(workspace, { { "modifier", modifier, {}, 8., modifier_config } });
}

// K1n: the nozzle-floor gap. At a gap of 0.4 mm (the nozzle diameter) the refinement saturates into solid columns under the roof,
// the part slices, and K1 and K2 hold with the clamped gap.
TEST_CASE("Bridge aware cubic holds the contract at the nozzle-floor gap", "[BridgeAwareCubic][K1n]")
{
    Workspace workspace;
    const DynamicPrintConfig settings = with(fixture_config(0.4, "bridgeawarecubic"), {{"sparse_infill_density", "3%"}});
    std::vector<EmittedPath> paths;
    REQUIRE_NOTHROW(paths = exported_extrusions(workspace, small_cube(), settings, "k1n.gcode"));
    const std::vector<PlaneMeasure> planes = measure_planes(paths, 0.4, 0.4);
    REQUIRE_FALSE(planes.empty());
    for (const PlaneMeasure &plane : planes) {
        INFO("plane below z = " << plane.lower / 1000.);
        CHECK(plane.widest_opening <= 0.4);
        CHECK(plane.longest_run <= 0.4);
    }
    const std::vector<EmittedPath> control = exported_extrusions(workspace, small_cube(), with(fixture_config(0.4, "cubic"), {{"sparse_infill_density", "3%"}}), "k1n-cubic.gcode");
    const K2Measure measure = measure_k2(paths, control, 0.3);
    CHECK(measure.free_ends == 0);
    CHECK(measure.long_stretches == 0);
}
