#include <catch2/catch.hpp>

#ifdef __APPLE__
#include <libproc.h>
#include <unistd.h>
#endif

#include "bridge_aware_cubic_fixtures.hpp"
#include "bridge_aware_cubic_observer.hpp"

#include "libslic3r/Fill/BridgeAwareCubicPlan.hpp"
#include "libslic3r/Fill/FillBridgeAwareCubic.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <set>
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
    double             direction = 45.;                   // the infill direction of the base config
    std::vector<double> other_directions = {};            // the other regions' infill directions (a modifier's)
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
// P2's comparison: on the plane layer, outside the roof's footprint grown by the gap, the pattern's lattice segments equal Cubic's
// within 0.001 mm one for one, and the pattern adds none there.
void check_coarse_away_from_roof(Workspace &workspace, const DynamicPrintConfig &settings, const DynamicPrintConfig &control_settings, const std::string &tag)
{
    const std::vector<EmittedPath> bac = exported_extrusions(workspace, half_open_roof(), settings, tag + "-bac.gcode");
    const std::vector<EmittedPath> cubic = exported_extrusions(workspace, half_open_roof(), control_settings, tag + "-cubic.gcode");
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

TEST_CASE("Bridge aware cubic leaves the lattice coarse away from the roof", "[BridgeAwareCubic][P2]")
{
    Workspace workspace;
    check_coarse_away_from_roof(workspace, fixture_config(5., "bridgeawarecubic"), fixture_config(5., "cubic"), "p2");
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

// Roofs with an edge over the pattern's void (the bridge grows past it: the rails of 3.5b bound the growth) and two lattices with
// a seam (the seam walls of 3.5a bound the gutter). Not here: roofs under two or three internal bridge layers (this fork's thick
// bases, re-anchored by the stock generator after the plan) and the rib's first layer (the void band between the body's top fill
// and the one-bead solid ring under the rib's walls), both recorded limitations in docs/bridge-aware-cubic.md, measured by the
// local debug dump.
TEST_CASE("Bridge aware cubic bounds every opening and run under roofs with edges over the void", "[BridgeAwareCubic][K1][K1edges]")
{
    Workspace workspace;
    check_k1(workspace, {
        { "modifier", modifier, {}, 8., modifier_config, 45. },
        { "partial roof", partial_roof, {}, 5. },
        { "partial roof, thick bridges", partial_roof, with({}, {{"thick_bridges", "1"}}), 5. },
        { "stepped cavity", stepped_cavity, {}, 5. },
    });
}

// K2: the pattern's added lines are held. On the fixtures, every added sparse piece off the hold set of the plane below is at
// most G − 0.1 mm long between held points, and no added line ends free over the void. The Cubic control has no added line, and
// taking away the sparse lines of the layer below the lowest plane leaves that plane's added lines free. Every sparse line stays
// inside the area the stock pipeline gives the infill.
void check_k2(Workspace &workspace, const std::vector<ExportFixture> &fixtures)
{
    for (const ExportFixture &fixture : fixtures) {
        INFO(fixture.name);
        Print print;
        const std::vector<EmittedPath> paths = exported_extrusions(print, workspace, fixture.model(), settings_of(fixture, "bridgeawarecubic"), fixture.name + "-k2-bac.gcode");
        const std::vector<EmittedPath> control = exported_extrusions(workspace, fixture.model(), settings_of(fixture, "cubic"), fixture.name + "-k2-cubic.gcode");
        // Every line stays inside the area the stock pipeline gives the infill (K1n carries the measure's control).
        const std::optional<Vec2d> outside = sparse_outside_fill_area(print, paths);
        INFO("a sparse path leaves the fill area at " << (outside ? *outside : Vec2d::Zero()).transpose());
        CHECK_FALSE(outside.has_value());
        // The modifier fixture has two gaps: 3 mm on its right half (x from 24 mm of the part, placed at x = 80), 8 mm on the left.
        const double left_gap = fixture.gap;
        const auto gap_b = [&fixture, left_gap](const Vec2d &at, double) { return (fixture.name == "modifier" && at.x() >= 80. + 24. ? 3. : left_gap) - 0.1; };
        std::vector<double> directions = fixture.other_directions;
        directions.push_back(fixture.direction);
        const K2Measure measure = measure_k2(paths, control, gap_b, directions);
        INFO("added pieces " << measure.added_pieces << ", longest " << measure.longest << " mm, first place " << (measure.where.empty() ? Vec2d(0, 0) : measure.where.front()).transpose());
        CHECK(measure.added_pieces > 0);
        CHECK(measure.free_ends == 0);
        CHECK(measure.long_stretches == 0);
        CHECK(measure_k2(control, control, gap_b, fixture.direction).added_pieces == 0);
        // The measure's own control: a hanging stub on a sparse line of the lowest plane and a two-segment stretch in the void,
        // both added to the export, must come back as a free end and a long stretch.
        {
            const std::vector<PlaneMeasure> planes = measure_planes(paths, 0.4, fixture.gap);
            REQUIRE_FALSE(planes.empty());
            int lowest = planes.front().lower;
            for (const PlaneMeasure &p : planes) lowest = std::min(lowest, p.lower);
            const std::map<int, EmittedLayer> layers = emitted_layers(paths);
            const EmittedLayer &layer = layers.at(lowest);
            // The longest straight segment of a sparse path of that layer (every path may carry anchors, so no path is two points).
            const EmittedPath *longest = nullptr;
            Vec2d a = Vec2d::Zero(), b = Vec2d::Zero();
            double longest_length = 0.;
            for (const EmittedPath *path : layer)
                if (path->role == erInternalInfill)
                    for (size_t i = 1; i < path->path.points.size(); ++i) {
                        const double length = (path->path.points[i] - path->path.points[i - 1]).cast<double>().norm();
                        if (length > longest_length) {
                            longest_length = length;
                            longest = path;
                            a = path->path.points[i - 1].cast<double>();
                            b = path->path.points[i].cast<double>();
                        }
                    }
            REQUIRE(longest != nullptr);
            const Vec2d mid = 0.5 * (a + b), dir = (b - a).normalized(), perp(-dir.y(), dir.x());
            const auto clear_of_all = [&](const Vec2d &point, double clearance_mm) {
                for (const auto &[key, paths_of_layer] : layers) {
                    if (key != lowest && key != std::prev(layers.find(lowest))->first) continue;
                    for (const EmittedPath *path : paths_of_layer)
                        for (size_t i = 1; i < path->path.points.size(); ++i)
                            if (unscale<double>(Line(path->path.points[i - 1], path->path.points[i]).distance_to(Point(coord_t(point.x()), coord_t(point.y())))) < clearance_mm) return false;
                }
                return true;
            };
            std::vector<EmittedPath> injected = paths;
            bool stub_added = false, stretch_added = false;
            for (const double side : { 1., -1. }) {
                const Vec2d stub_end = mid + side * perp * scale_(0.45);
                if (!stub_added && clear_of_all(stub_end, 0.3)) {
                    EmittedPath stub = *longest;
                    stub.path = Polyline(Point(coord_t(mid.x()), coord_t(mid.y())), Point(coord_t(stub_end.x()), coord_t(stub_end.y())));
                    injected.push_back(stub);
                    stub_added = true;
                }
                for (double offset = 1.; offset <= 8. && !stretch_added; offset += 0.5) {
                    const Vec2d s0 = mid + side * perp * scale_(offset), s1 = s0 + side * perp * scale_(3.) + dir * scale_(0.3), s2 = s0 + side * perp * scale_(6.);
                    if (clear_of_all(s0, 0.6) && clear_of_all(s1, 0.6) && clear_of_all(s2, 0.6) && clear_of_all(0.5 * (s0 + s1), 0.6) && clear_of_all(0.5 * (s1 + s2), 0.6)) {
                        EmittedPath stretch = *longest;
                        stretch.path = Polyline(Points{ Point(coord_t(s0.x()), coord_t(s0.y())), Point(coord_t(s1.x()), coord_t(s1.y())), Point(coord_t(s2.x()), coord_t(s2.y())) });
                        injected.push_back(stretch);
                        stretch_added = true;
                    }
                }
            }
            if (stub_added || stretch_added) {
                const K2Measure broken = measure_k2(injected, control, gap_b, fixture.direction);
                if (stub_added) CHECK(broken.free_ends >= 1);
                if (stretch_added && fixture.gap <= 6.) CHECK(broken.long_stretches >= 1);
            }
        }
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
        const K2Measure broken = measure_k2(without, control, gap_b, fixture.direction);
        CHECK(broken.free_ends + broken.long_stretches > 0);
    }
}

const std::initializer_list<std::pair<const std::string, std::string>> k2_user_settings = {
    {"wall_loops", "1"}, {"wall_generator", "arachne"}, {"infill_wall_overlap", "75%"}, {"detect_narrow_internal_solid_infill", "1"},
    {"thick_bridges", "1"}, {"sparse_infill_anchor", "1"}, {"sparse_infill_anchor_max", "1"}, {"sparse_infill_density", "3%"} };

TEST_CASE("Bridge aware cubic holds every line it adds", "[BridgeAwareCubic][K2]")
{
    Workspace workspace;
    check_k2(workspace, {
        { "small cube", small_cube, {}, 5. },
        { "large cube", large_cube, {}, 5. },
        { "rib", rib, {}, 5., rib_config, 17. },
        { "box 40 at 3 %", [] { return single_part("Bridge aware cubic box 40", TriangleMesh(its_make_cube(40., 40., 40.))); },
          with({}, {{"sparse_infill_density", "3%"}}), 5. },
        // Without anchors (no nub, no join) every end holds from below.
        { "small cube, no anchor", small_cube, with({}, {{"sparse_infill_anchor", "0"}}), 5. },
        { "small cube, no connections", small_cube, with({}, {{"sparse_infill_anchor_max", "0"}}), 5. },
        // The user's settings: one Arachne wall at 75 % overlap, narrow solid infill, thick bridges, 1 mm anchors, 3 % density (on
        // the rib they leave one free end: docs/bridge-aware-cubic.md).
        { "large cube, the user's settings", large_cube, with({}, k2_user_settings), 5. },
        { "partial roof, the user's settings", partial_roof, with({}, k2_user_settings), 5. },
    });
}

// Two lattices with a seam: the seam wall (3.5a) makes the seam material, so the lines that end on it are fused to it and the
// gutter between the lattices holds nothing unheld.
TEST_CASE("Bridge aware cubic holds every line it adds across a seam", "[BridgeAwareCubic][K2][K2seam]")
{
    Workspace workspace;
    check_k2(workspace, { { "modifier", modifier, {}, 8., modifier_config, 45., { 17. } } });
}

// K1n: the nozzle-floor gap. At a gap of 0.4 mm (the nozzle diameter) the refinement saturates into solid columns under the roof
// and the part slices; the columns' lines, whose cells reach past the island at the walls, stay inside the area the stock
// pipeline gives the infill, on the fixtures' walls and on the user's (one Arachne wall the infill overlaps by 75 %). K1 and K2
// do not hold there yet (a known limitation, docs/bridge-aware-cubic.md).
TEST_CASE("Bridge aware cubic slices at the nozzle-floor gap", "[BridgeAwareCubic][K1n]")
{
    Workspace workspace;
    const DynamicPrintConfig fixture = with(fixture_config(0.4, "bridgeawarecubic"), {{"sparse_infill_density", "3%"}});
    for (const DynamicPrintConfig &settings : { fixture, with(fixture, {{"wall_loops", "1"}, {"wall_generator", "arachne"}, {"infill_wall_overlap", "75%"}}) }) {
        Print print;
        std::vector<EmittedPath> paths;
        REQUIRE_NOTHROW(paths = exported_extrusions(print, workspace, small_cube(), settings, "k1n.gcode"));
        CHECK_FALSE(measure_planes(paths, 0.4, 0.4).empty());
        const std::optional<Vec2d> outside = sparse_outside_fill_area(print, paths);
        INFO("a sparse path leaves the fill area at " << (outside ? *outside : Vec2d::Zero()).transpose());
        CHECK_FALSE(outside.has_value());
        // The measure's control: a bead from the rightmost sparse vertex one bead further out leaves the fill area.
        const EmittedPath *rightmost = nullptr;
        Point far = Point::Zero();
        for (const EmittedPath &path : paths)
            if (path.role == erInternalInfill)
                for (const Point &p : path.path.points)
                    if (rightmost == nullptr || p.x() > far.x()) {
                        rightmost = &path;
                        far = p;
                    }
        REQUIRE(rightmost != nullptr);
        std::vector<EmittedPath> injected = paths;
        injected.push_back(*rightmost);
        injected.back().path = Polyline(far, far + Point(coord_t(scale_(rightmost->width)), 0));
        CHECK(sparse_outside_fill_area(print, injected).has_value());
    }
}

// S3: infill combination (3.9a). The pattern's combined surfaces equal Cubic's (the void surfaces and the thick surfaces per
// layer). On the export, read without the plan: every line the pattern adds along the lattice prints at layer height; a thick line along a lattice
// direction lies on the lattice of its layer (Cubic's single-layer lines: outside the zones the pattern prints its own lattice at
// the combined height); a layer-height lattice line of a combined layer rests on a line of its own plane one layer below (a thick
// line counting on every layer it fills); K2 holds.
TEST_CASE("Bridge aware cubic honours infill combination through single-layer zones", "[BridgeAwareCubic][S3]")
{
    Workspace workspace;
    const std::vector<ExportFixture> fixtures = {
        { "small cube", small_cube, with({}, {{"infill_combination", "1"}}), 5. },
        { "large cube", large_cube, with({}, {{"infill_combination", "1"}}), 5. },
    };
    for (const ExportFixture &fixture : fixtures) {
        INFO(fixture.name);
        const DynamicPrintConfig settings = observed(settings_of(fixture, "bridgeawarecubic")), control_settings = observed(settings_of(fixture, "cubic"));
        Print print, control_print;
        Model model = fixture.model(), control_model = fixture.model();
        slice(print, model, settings);
        slice(control_print, control_model, control_settings);
        const PrintObject &object = *print.get_object(size_t(0)), &control_object = *control_print.get_object(size_t(0));
        REQUIRE(object.layers().size() == control_object.layers().size());
        // The surfaces: the void surfaces (count, and area within the outline's length times the resolution, what simplifying a
        // contour within the resolution can change) and the thick surfaces' thicknesses per layer equal Cubic's.
        const double resolution = settings.opt_float("resolution");
        const auto summarise = [](const Layer &layer) {
            size_t voids = 0; double void_area = 0., outline = 0.; std::multiset<int> thick;
            for (const LayerRegion *region : layer.regions())
                for (const Surface &surface : region->fill_surfaces.surfaces) {
                    if (surface.surface_type == stInternalVoid) {
                        ++voids;
                        void_area += surface.expolygon.area() * SCALING_FACTOR * SCALING_FACTOR;
                        for (const Polygon &polygon : to_polygons(surface.expolygon)) outline += unscale<double>(polygon.length());
                    } else if (surface.surface_type == stInternal && surface.thickness_layers > 1)
                        thick.insert(int(surface.thickness_layers));
                }
            return std::make_tuple(voids, void_area, thick, outline);
        };
        size_t combined_layers_count = 0;
        for (size_t l = 0; l < object.layers().size(); ++l) {
            INFO("layer " << l);
            const auto mine = summarise(*object.layers()[l]), theirs = summarise(*control_object.layers()[l]);
            CHECK(std::get<0>(mine) == std::get<0>(theirs));
            CHECK(std::abs(std::get<1>(mine) - std::get<1>(theirs)) <= std::max(std::get<3>(mine), std::get<3>(theirs)) * resolution);
            CHECK(std::get<2>(mine) == std::get<2>(theirs));
            combined_layers_count += !std::get<2>(mine).empty();
        }
        CHECK(combined_layers_count >= 5);
        const std::string filename = print.export_gcode((workspace.path / (fixture.name + "-s3.gcode")).string(), nullptr);
        const std::vector<EmittedPath> paths = read_extrusions(filename, settings);
        const std::string control_file = control_print.export_gcode((workspace.path / (fixture.name + "-s3-cubic.gcode")).string(), nullptr);
        const std::vector<EmittedPath> combined_control = read_extrusions(control_file, control_settings);
        const std::vector<EmittedPath> single_control = exported_extrusions(workspace, fixture.model(), with(settings_of(fixture, "cubic"), {{"infill_combination", "0"}}), fixture.name + "-s3-cubic-single.gcode");
        const std::map<int, EmittedLayer> layers = emitted_layers(paths), single_layers = emitted_layers(single_control), combined_layers = emitted_layers(combined_control);
        std::map<int, double> layer_height;
        for (const Layer *layer : object.layers())
            layer_height[plane_key(layer->print_z)] = layer->height;
        const std::vector<double> directions = { std::fmod(fixture.direction, 180.), std::fmod(fixture.direction + 60., 180.), std::fmod(fixture.direction + 120., 180.) };
        // A thick path fills every layer of its height: unfolded onto each (the layers' keys below its own).
        std::vector<EmittedPath> unfolded = paths;
        std::set<int> combined_keys;
        for (const EmittedPath &path : paths) {
            auto it = layers.find(plane_key(path.z));
            const double h = layer_height.count(plane_key(path.z)) ? layer_height.at(plane_key(path.z)) : path.height;
            if (path.role != erInternalInfill || path.height <= h + 1e-3 || it == layers.end())
                continue;
            combined_keys.insert(it->first);
            double filled = h;
            for (auto below = it; below != layers.begin() && filled < path.height - 1e-3;) {
                --below;
                EmittedPath copy = path;
                copy.z = below->first / 1000.;
                unfolded.push_back(copy);
                combined_keys.insert(below->first);
                filled += layer_height.count(below->first) ? layer_height.at(below->first) : h;
            }
        }
        REQUIRE_FALSE(combined_keys.empty());
        const std::map<int, EmittedLayer> unfolded_layers = emitted_layers(unfolded);
        double sparse_w = 0.;
        for (const EmittedPath &path : paths)
            if (path.role == erInternalInfill) { sparse_w = path.width; break; }
        size_t added = 0, thick_lattice = 0, supported = 0;
        for (const auto &[key, layer] : layers) {
            const std::vector<Line> single = control_sparse_segments(single_layers, key), combined = control_sparse_segments(combined_layers, key);
            const double h = layer_height.count(key) ? layer_height.at(key) : 0.;
            const auto below = unfolded_layers.find(key);
            const bool has_below = below != unfolded_layers.end() && below != unfolded_layers.begin();
            ExPolygons below_other;
            Polylines below_sparse;
            if (has_below) {
                EmittedLayer other;
                for (const EmittedPath *path : std::prev(below)->second) {
                    if (path->role == erInternalInfill)
                        below_sparse.push_back(path->path);
                    else
                        other.push_back(path);
                }
                below_other = footprint(other);
            }
            // The walls' band: a lattice segment with both ends in it is a piece of one of the lattice's extreme lines at a corner
            // of the island, which the pattern prints and Cubic drops (L1), not a line the pattern adds.
            EmittedLayer walls_here;
            for (const EmittedPath *path : layer)
                if (path->role == erExternalPerimeter || path->role == erPerimeter) walls_here.push_back(path);
            const ExPolygons wall_band = offset_ex(footprint(walls_here), float(scale_(sparse_w)), ClipperLib::jtRound, scale_(.001));
            const auto in_wall_band = [&wall_band](const Point &p) { return std::any_of(wall_band.begin(), wall_band.end(), [&p](const ExPolygon &e) { return e.contains(p); }); };
            for (const EmittedPath *path : layer) {
                if (path->role != erInternalInfill)
                    continue;
                const bool thick = path->height > h + 1e-3;
                for (size_t i = 1; i < path->path.points.size(); ++i) {
                    const Point &a = path->path.points[i - 1], &b = path->path.points[i];
                    const bool lattice = along_lattice(a, b, directions);
                    const bool on_single = on_control_line(a, b, single);
                    INFO("z = " << path->z << " segment " << unscale(a).transpose() << " -> " << unscale(b).transpose() << " height " << path->height);
                    if (lattice && !on_single && !on_control_line(a, b, combined) && !(in_wall_band(a) && in_wall_band(b))) {
                        // A line the pattern adds (a strand or a column piece; off the lattice directions lie the connector's hooks
                        // along the island's boundary, Cubic's behaviour): at layer height.
                        ++added;
                        CHECK(path->height == Approx(h).margin(1e-3));
                    } else if (thick && lattice && unscale<double>((b - a).cast<double>().norm()) >= path->width && !(in_wall_band(a) && in_wall_band(b))) {
                        ++thick_lattice;
                        CHECK(on_single);
                        // Outside the single-layer zones only: on no layer the thick bead fills does a layer-height bead of its
                        // direction run within half a bead of it (a bead in from its ends; the zone's edge cuts the two at
                        // places a drift apart).
                        const Vec2d pa = a.cast<double>(), d = (b - a).cast<double>();
                        const double length = d.norm(), step = scale_(0.5 * path->width), margin = scale_(path->width);
                        for (auto filled = unfolded_layers.find(key); filled != unfolded_layers.end(); --filled) {
                            for (double t = margin; t <= length - margin; t += step) {
                                const Point q = (pa + d * (t / length)).cast<coord_t>();
                                double nearest = std::numeric_limits<double>::max();
                                for (const EmittedPath *other : filled->second) {
                                    if (other->role != erInternalInfill || other->height > (layer_height.count(filled->first) ? layer_height.at(filled->first) : h) + 1e-3)
                                        continue;
                                    for (size_t j = 1; j < other->path.points.size(); ++j) {
                                        const Point &oa = other->path.points[j - 1], &ob = other->path.points[j];
                                        const Vec2d od = (ob - oa).cast<double>();
                                        if (od.norm() < 1. || std::abs(od.normalized().dot(d / length)) < 0.999)
                                            continue;
                                        nearest = std::min(nearest, unscale<double>(Line(oa, ob).distance_to(q)));
                                    }
                                }
                                INFO("thick bead point " << unscale(q).transpose() << " on z = " << filled->first / 1000. << ": a layer-height bead of its direction at " << nearest << " mm");
                                CHECK(nearest > 0.5 * path->width);
                            }
                            // On down while the next layer lies inside the bead's height.
                            if (filled == unfolded_layers.begin() || std::prev(filled)->first <= plane_key(path->z - path->height + 1e-3))
                                break;
                        }
                    } else if (!thick && lattice && on_single && has_below && combined_keys.count(key)) {
                        // A lattice line at layer height on a combined layer: each point half a bead in from its ends rests on a
                        // line of the layer below (its own plane drifts by less than half a bead a layer) or on other material.
                        const Vec2d pa = a.cast<double>(), d = (b - a).cast<double>();
                        const double length = d.norm(), step = scale_(0.5 * path->width), margin = scale_(0.5 * path->width);
                        for (double t = margin; t <= length - margin; t += step) {
                            const Point q = (pa + d * (t / length)).cast<coord_t>();
                            double nearest = std::numeric_limits<double>::max();
                            for (const Polyline &line : below_sparse)
                                for (size_t j = 1; j < line.points.size(); ++j)
                                    nearest = std::min(nearest, unscale<double>(Line(line.points[j - 1], line.points[j]).distance_to(q)));
                            const bool on_material = std::any_of(below_other.begin(), below_other.end(), [&q](const ExPolygon &e) { return e.contains(q); });
                            INFO("point " << unscale(q).transpose() << ", nearest line below " << nearest << " mm");
                            CHECK((on_material || nearest <= 0.5 * path->width + 0.01));
                            ++supported;
                        }
                    }
                }
            }
        }
        CHECK(added > 0);
        CHECK(thick_lattice > 0);
        CHECK(supported > 0);
        // K1 on the unfolded paths: a thick bead holds every layer it fills.
        const std::vector<PlaneMeasure> planes = measure_planes(unfolded, 0.4, fixture.gap);
        REQUIRE_FALSE(planes.empty());
        for (const PlaneMeasure &plane : planes) {
            INFO("plane below z = " << plane.lower / 1000. << " mm");
            CHECK(plane.widest_opening <= fixture.gap);
            CHECK(plane.longest_run <= fixture.gap);
        }
        // K2 judges the paths as emitted (an unfolded copy lies off its layer's lattice) against both Cubic slices: the combined
        // one has the hooks along the combined islands' boundaries, the single-layer one the lattice of every layer.
        std::vector<EmittedPath> control = combined_control;
        append(control, single_control);
        const K2Measure measure = measure_k2(paths, control, fixture.gap - 0.1, fixture.direction);
        INFO("added pieces " << measure.added_pieces << ", first place " << (measure.where.empty() ? Vec2d(0, 0) : measure.where.front()).transpose());
        CHECK(measure.added_pieces > 0);
        CHECK(measure.free_ends == 0);
        CHECK(measure.long_stretches == 0);
    }
}

// S4: fill multiline (3.9b). At N = 2 every lattice line is a band of two sub-lines: away from the roof the bands equal Cubic's at
// N = 2 (P2's comparison), and K1 and K2 hold with the bands (K2's hold set is the sub-lines' centre lines, the control Cubic's bands).
// The pitch grows with N (FillCubic: N s_f / density per family), so the comparison runs at 10 %, P2's pitch at N = 1 and 5 %: at
// 5 % no band crosses the region away from the half-open roof.
TEST_CASE("Bridge aware cubic honours fill multiline as bands", "[BridgeAwareCubic][S4]")
{
    Workspace workspace;
    check_coarse_away_from_roof(workspace, with(fixture_config(5., "bridgeawarecubic"), {{"fill_multiline", "2"}, {"sparse_infill_density", "10%"}}),
                                with(fixture_config(5., "cubic"), {{"fill_multiline", "2"}, {"sparse_infill_density", "10%"}}), "s4");
    // Three lines: a switched or cover piece is λ + 2 s_f / sin 60° long, or its outer sub-lines hang.
    const std::vector<ExportFixture> bands = {
        { "small cube, two lines", small_cube, with({}, {{"fill_multiline", "2"}}), 5. },
        { "small cube, three lines", small_cube, with({}, {{"fill_multiline", "3"}}), 5. },
    };
    check_k1(workspace, bands);
    check_k2(workspace, bands);
}

// S5: a shared object. Print slices one of two identical objects and the other prints its layers; an object sliced alone and then
// shared (an identical object added before it, which the print slices instead) reads its source's plan, not the one it made
// alone. Instances and shared copies print one object's layers, translated: K1 measures a second instance at a fractional offset.
TEST_CASE("Bridge aware cubic gives a shared object its source's plan", "[BridgeAwareCubic][S5]")
{
    Model model = small_cube();
    // Print shares objects whose meshes, configs and transformations agree but for x and y; Model::add_object sets the copy's
    // extruder, as the GUI sets every object's.
    model.objects.front()->config.set_key_value("extruder", new ConfigOptionInt(1));
    const DynamicPrintConfig settings = fixture_config(5., "bridgeawarecubic");
    Print print;
    slice(print, model, settings);
    REQUIRE(print.objects().size() == 1);
    const PrintObject *alone = print.get_object(0);
    REQUIRE(alone->bridge_aware_cubic_plan() != nullptr);
    ModelObject *copy = model.add_object(*model.objects.front());
    copy->instances.front()->set_offset(model.objects.front()->instances.front()->get_offset() + Vec3d(70., 0., 0.));
    std::rotate(model.objects.begin(), model.objects.end() - 1, model.objects.end());
    slice(print, model, settings);
    REQUIRE(print.objects().size() == 2);
    const PrintObject &first = *print.get_object(0), &second = *print.get_object(1);
    const PrintObject &sharing = second.get_shared_object() != nullptr ? second : first;
    REQUIRE(sharing.get_shared_object() != nullptr);
    CHECK(&sharing == alone);    // the object that shares holds a plan of its own from its slice alone
    REQUIRE(sharing.get_shared_object()->bridge_aware_cubic_plan() != nullptr);
    CHECK(sharing.bridge_aware_cubic_plan() == sharing.get_shared_object()->bridge_aware_cubic_plan());
}

// D1: determinism. Two slices of one project export the same G-code byte for byte, and a reslice after a setting changed exports
// what a fresh slice exports, at the changed value and after it changed back.
TEST_CASE("Bridge aware cubic exports the same G-code on every slice and reslices to the same fills", "[BridgeAwareCubic][D1]")
{
    Workspace workspace;
    struct Case { const char *name; Model (*model)(); DynamicPrintConfig settings; };
    const Case cases[] = {
        { "large cube", large_cube, fixture_config(5., "bridgeawarecubic") },
        { "partial roof, three internal bridge layers", partial_roof, with(fixture_config(5., "bridgeawarecubic"), {{"internal_bridge_layers", "3"}}) },
        { "modifier", modifier, modifier_config("bridgeawarecubic") },
    };
    for (const Case &c : cases) {
        INFO(c.name);
        const std::string first  = exported_gcode(workspace, c.model(), c.settings, std::string(c.name) + "-d1-first.gcode");
        const std::string second = exported_gcode(workspace, c.model(), c.settings, std::string(c.name) + "-d1-second.gcode");
        // The header's generation time is the one line allowed to differ.
        const auto without_time = [](const std::string &gcode) {
            std::istringstream in(gcode);
            std::string out, line;
            while (std::getline(in, line))
                if (line.rfind("; generated by", 0) != 0 && line.rfind("; generated", 0) != 0)
                    out += line + "\n";
            return out;
        };
        CHECK(without_time(first) == without_time(second));
    }
    // A reslice after a setting changed exports what a fresh slice at the changed value exports (the plan was rebuilt), and after it
    // changed back what a fresh slice exports: on the partial roof with three internal bridge layers; the anchor settings feed the
    // plan's fused ends.
    const auto without_time = [](const std::string &gcode) {
        std::istringstream in(gcode);
        std::string out, line;
        while (std::getline(in, line))
            if (line.rfind("; generated", 0) != 0)
                out += line + "\n";
        return out;
    };
    const auto exported_now = [&workspace](Print &print, const std::string &name) {
        std::ifstream file(print.export_gcode((workspace.path / name).string(), nullptr), std::ios::binary);
        std::string out, line;
        while (std::getline(file, line))
            if (line.rfind("; OBJECT_ID:", 0) != 0)
                out += line + "\n";
        return out;
    };
    const DynamicPrintConfig settings = observed(with(fixture_config(5., "bridgeawarecubic"), {{"internal_bridge_layers", "3"}}));
    const std::string fresh = without_time(exported_gcode(workspace, partial_roof(), settings, "d1-fresh.gcode"));
    const struct { const char *key, *changed; } changes[] = {
        { "bridge_aware_cubic_max_gap", "6" }, { "sparse_infill_density", "7%" }, { "infill_direction", "60" },
        { "sparse_infill_anchor", "0" }, { "sparse_infill_anchor_max", "0" },
    };
    for (const auto &change : changes) {
        INFO("reslice after " << change.key << " = " << change.changed << " and back");
        const DynamicPrintConfig changed = with(settings, {{change.key, change.changed}});
        Print print;
        Model model = partial_roof();
        slice(print, model, settings);
        print.apply(model, changed);
        print.process();
        CHECK(without_time(exported_now(print, "d1-changed.gcode")) == without_time(exported_gcode(workspace, partial_roof(), changed, "d1-changed-fresh.gcode")));
        print.apply(model, settings);
        print.process();
        CHECK(without_time(exported_now(print, "d1-back.gcode")) == fresh);
    }
}

// X1 (hidden, information only): what the pattern costs over Cubic in instructions retired by process() and the G-code export, on
// the four fixtures, each slice on a fresh print: a warm-up pair, then five pairs alternating the order, compared by their medians.
namespace {
uint64_t instructions_retired()
{
#ifdef __APPLE__
    rusage_info_v4 info;
    if (proc_pid_rusage(getpid(), RUSAGE_INFO_V4, reinterpret_cast<rusage_info_t*>(&info)) == 0)
        return info.ri_instructions;
#endif
    return 0;
}

double slice_cost(Model &model, const DynamicPrintConfig &config, const std::string &gcode)
{
    Print print;
    const uint64_t before = instructions_retired();
    slice(print, model, config);
    print.export_gcode(gcode, nullptr);
    return double(instructions_retired() - before);
}
} // namespace

TEST_CASE("Bridge aware cubic timing against Cubic", "[.][BACTiming]")
{
    Workspace workspace;
    struct Timed { const char *name; Model (*model)(); DynamicPrintConfig (*base)(const char *); double gap; };
    const Timed fixtures[] = { { "small cube", small_cube, nullptr, 5. }, { "large cube", large_cube, nullptr, 5. },
                               { "rib", rib, rib_config, 5. }, { "modifier", modifier, modifier_config, 8. } };
    const std::string gcode = (workspace.path / "timing.gcode").string();
    for (const Timed &fixture : fixtures) {
        const DynamicPrintConfig cubic = fixture.base ? fixture.base("cubic") : fixture_config(fixture.gap, "cubic");
        const DynamicPrintConfig bac = fixture.base ? fixture.base("bridgeawarecubic") : fixture_config(fixture.gap, "bridgeawarecubic");
        Model model = fixture.model();
        std::vector<double> cubic_cost, bac_cost;
        for (int pair = 0; pair <= 5; ++pair) {
            double c = 0., b = 0.;
            if (pair % 2 == 0) { c = slice_cost(model, cubic, gcode); b = slice_cost(model, bac, gcode); }
            else               { b = slice_cost(model, bac, gcode); c = slice_cost(model, cubic, gcode); }
            if (pair > 0) { cubic_cost.push_back(c); bac_cost.push_back(b); }
        }
        std::vector<double> ratios;
        for (size_t i = 0; i < cubic_cost.size(); ++i)
            ratios.push_back(cubic_cost[i] > 0. ? bac_cost[i] / cubic_cost[i] : 0.);
        std::sort(cubic_cost.begin(), cubic_cost.end());
        std::sort(bac_cost.begin(), bac_cost.end());
        std::sort(ratios.begin(), ratios.end());
        std::cout << "BAC_TIMING " << fixture.name << " cubic " << cubic_cost[cubic_cost.size() / 2] << " bac " << bac_cost[bac_cost.size() / 2]
                  << " pair ratio median " << ratios[ratios.size() / 2] << " worst " << ratios.back() << std::endl;
    }
}

// X2 (hidden): writes the native projects of the official benchmark (docs/bridge-aware-cubic-benchmark.md) into
// BAC_BENCHMARK_FIXTURE_DIR, refusing to overwrite; carried over from the earlier branch, its fixtures unchanged.
TEST_CASE("Bridge aware cubic writes native CLI benchmark fixtures", "[.][BACBenchmarkFixtures]")
{
    namespace fs = boost::filesystem;
    const char *requested_directory = std::getenv("BAC_BENCHMARK_FIXTURE_DIR");
    REQUIRE(requested_directory != nullptr);
    REQUIRE(std::string(requested_directory).size() > 0);
    const fs::path directory(requested_directory);
    REQUIRE(directory.is_absolute());
    fs::create_directories(directory);
    REQUIRE(fs::is_directory(directory));
    // Preflight every destination before writing any artifact. A repeated run
    // must use a fresh directory rather than overwrite a previous measurement.
    REQUIRE_FALSE(fs::exists(directory / "manifest.json"));
    for (const char *size : {"small", "large", "rib", "modifier"})
        for (const char *pattern : {"cubic", "bridgeawarecubic"})
            for (const char *suffix : {".3mf", ".3mf.tmp", ".config.ini", ".config.json"})
                REQUIRE_FALSE(fs::exists(directory / (std::string(size) + "-" + pattern + suffix)));

    Workspace workspace;
    DynamicPrintConfig defaults;
    // DynamicPrintConfig::full_print_config() currently supplies region
    // defaults only. Explicitly complete object, region and printer defaults.
    defaults.apply(FullPrintConfig::defaults());
    defaults.apply(observed(fixture_config(5., "bridgeawarecubic")));
    defaults.set_deserialize_strict({{"inherits", ""}, {"inherits_group", ""}});
    nlohmann::json manifest = {
        {"schema_version", 1},
        {"generator_version", SLIC3R_VERSION},
        {"configuration_basis", "FullPrintConfig defaults + named fixture config, normalized by Print::apply"},
        {"slicing_performed", false},
        {"allowed_pair_config_differences", {"sparse_infill_pattern", "bridge_aware_cubic_max_gap"}},
        {"fixtures", nlohmann::json::array()}
    };
    using SerializedSettings = std::map<std::string, std::string>;
    std::map<std::string, SerializedSettings> common_settings_by_family;
    for (const std::string size : {"small", "large", "rib", "modifier"}) {
        const bool rib = size == "rib", modifier = size == "modifier";
        const std::string family = rib ? "rib" : modifier ? "modifier" : "cube";
        const double width = modifier ? 48. : size == "large" ? 64. : 32.;
        const double depth = rib ? 26. : size == "large" ? 56. : 28.;
        const double height = rib ? 15. : 18.;
        const double density = rib ? 12. : 5., gap = modifier ? 8. : 5.;
        const size_t model_triangles = rib ? 36 : 12;
        DynamicPrintConfig fixture_defaults = defaults;
        if (rib || modifier) {
            // Keep these physical cases equal to acceptance_config() and
            // modifier_config(), including defaults absent from the observed fixture config.
            fixture_defaults.clear();
            fixture_defaults.apply(FullPrintConfig::defaults());
            fixture_defaults.set_deserialize_strict({
                {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
                {"sparse_infill_density", rib ? "12%" : "5%"},
                {"sparse_infill_pattern", "bridgeawarecubic"},
                {"bridge_aware_cubic_max_gap", rib ? "5" : "8"},
                {"infill_direction", rib ? "17" : "45"},
                {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
                {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
                {"wall_loops", "2"}, {"wall_generator", "classic"},
                {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
                {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
                {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"},
                {"inherits", ""}, {"inherits_group", ""}
            });
            if (rib) fixture_defaults.set_deserialize_strict({{"ensure_vertical_shell_thickness", "disabled"}});
        }
        // Keep every fixture on the cubes' existing empty machine scripts.
        // Native defaults can move Z before the slicer declares G90/G21.
        fixture_defaults.set_deserialize_strict({{"machine_start_gcode", ""},
                                                 {"machine_end_gcode", ""}});
        Model model;
        ModelObject *object = model.add_object();
        object->name = "BAC benchmark " + size;
        ModelVolume *volume = nullptr, *right = nullptr;
        if (rib) {
            // Same common-frame assembly as the InwardWalls acceptance case.
            TriangleMesh body(its_make_cube(32., 26., 10.));
            TriangleMesh first(its_make_cube(5., 22., 5.));
            first.translate(6., 2., 10.);
            TriangleMesh second(its_make_cube(4., 18., 3.4));
            second.translate(22., 4., 10.);
            body.merge(first);
            body.merge(second);
            volume = object->add_volume(std::move(body));
            volume->name = "Benchmark inward ribs";
        } else {
            volume = object->add_volume(TriangleMesh(its_make_cube(width, depth, height)));
            volume->name = "Benchmark cube";
        }
        if (modifier) {
            TriangleMesh mesh(its_make_cube(24., 28., 18.));
            mesh.translate(24., 0., 0.);
            right = object->add_volume(std::move(mesh));
            right->set_type(ModelVolumeType::PARAMETER_MODIFIER);
            right->name = "Benchmark right half modifier";
            right->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(12.));
            right->config.set("infill_direction", 17.);
        }
        object->add_instance()->set_offset(Vec3d(80., 80., 0.));
        REQUIRE(object->facets_count() == model_triangles);
        REQUIRE(object->volumes.size() == size_t(modifier ? 2 : 1));
        for (const ModelVolume *part : object->volumes) REQUIRE(part->material_id().empty());
        const auto bounds = object->bounding_box();
        REQUIRE(bounds.min.x() == Approx(80.));
        REQUIRE(bounds.min.y() == Approx(80.));
        REQUIRE(bounds.min.z() == Approx(0.));
        REQUIRE(bounds.max.x() == Approx(80. + width));
        REQUIRE(bounds.max.y() == Approx(80. + depth));
        REQUIRE(bounds.max.z() == Approx(height));
        std::set<std::pair<int, int>> assignments{{0, 0}};
        PlateData plate(0, assignments, false);
        plate.plate_name = "Benchmark plate";
        std::vector<SerializedSettings> common_volume_settings;
        for (const bool bac : {false, true}) {
            const std::string pattern = bac ? "bridgeawarecubic" : "cubic";
            const std::string stem = size + "-" + pattern;
            DynamicPrintConfig config = fixture_defaults;
            config.set_deserialize_strict({{"sparse_infill_pattern", pattern}});
            if (right) {
                if (bac) right->config.set("bridge_aware_cubic_max_gap", 3.);
                else right->config.erase("bridge_aware_cubic_max_gap");
            }
            Print configured;
            configured.set_status_silent();
            configured.auto_assign_extruders(object);
            configured.apply(model, config);
            REQUIRE(configured.objects().size() == 1);
            REQUIRE(configured.get_object(size_t(0))->layers().empty());
            REQUIRE_FALSE(configured.is_step_done(posSlice));
            config = configured.full_print_config();
            for (const char *key : {"machine_start_gcode", "machine_end_gcode"}) {
                CAPTURE(stem, key);
                REQUIRE(config.opt_string(key).empty());
            }
            // The native CLI indexes process, filament and printer inheritance
            // slots directly. Empty entries preserve no-inheritance semantics;
            // an empty vector is not a valid CLI project shape.
            const auto *filament_settings = config.option<ConfigOptionStrings>("filament_settings_id");
            REQUIRE(filament_settings != nullptr);
            const size_t filament_count = filament_settings->values.size();
            REQUIRE(filament_count == 1);
            config.set_key_value("inherits_group", new ConfigOptionStrings(filament_count + 2, ""));
            // Packaging also indexes one filament ID per slot. Empty IDs keep
            // the fixture independent of any named material/profile identity.
            config.set_key_value("filament_ids", new ConfigOptionStrings(filament_count, ""));
            // Stock Cubic must not depend on ignoring an unknown BAC option.
            // Remove it after applying defaults, which may reintroduce it.
            if (!bac) config.erase("bridge_aware_cubic_max_gap");
            REQUIRE(object->config.empty());
            REQUIRE(volume->config.empty());
            REQUIRE(plate.config.empty());
            REQUIRE(config.opt_serialize("inherits").empty());
            const auto &inheritance = config.option<ConfigOptionStrings>("inherits_group")->values;
            REQUIRE(inheritance.size() == filament_count + 2);
            REQUIRE(std::all_of(inheritance.begin(), inheritance.end(), [](const std::string &entry) { return entry.empty(); }));
            const auto &filament_ids = config.option<ConfigOptionStrings>("filament_ids")->values;
            REQUIRE(filament_ids.size() == filament_count);
            REQUIRE(std::all_of(filament_ids.begin(), filament_ids.end(), [](const std::string &entry) { return entry.empty(); }));
            std::map<std::string, std::string> comparable;
            std::map<std::string, std::string> serialized_settings;
            std::vector<std::string> nil_settings;
            for (const std::string &key : config.keys()) {
                serialized_settings.emplace(key, config.opt_serialize(key));
                if (config.option(key)->is_nil()) nil_settings.push_back(key);
                if (key != "sparse_infill_pattern" && key != "bridge_aware_cubic_max_gap")
                    comparable.emplace(key, config.opt_serialize(key));
            }
            auto &common_settings = common_settings_by_family[family];
            if (common_settings.empty()) common_settings = comparable;
            else REQUIRE(comparable == common_settings);

            nlohmann::json volumes = nlohmann::json::array();
            std::vector<SerializedSettings> comparable_volumes;
            size_t total_triangles = 0;
            for (size_t index = 0; index < object->volumes.size(); ++index) {
                const ModelVolume &part = *object->volumes[index];
                SerializedSettings local, local_comparable;
                std::vector<std::string> local_nil;
                for (const std::string &key : part.config.keys()) {
                    local.emplace(key, part.config.opt_serialize(key));
                    if (part.config.option(key)->is_nil()) local_nil.push_back(key);
                    if (key != "bridge_aware_cubic_max_gap") local_comparable.emplace(key, part.config.opt_serialize(key));
                }
                comparable_volumes.push_back(std::move(local_comparable));
                if (&part == right) {
                    REQUIRE(local.size() == size_t(bac ? 3 : 2));
                    REQUIRE(local.at("sparse_infill_density") == "12%");
                    REQUIRE(local.at("infill_direction") == "17");
                    if (bac) REQUIRE(local.at("bridge_aware_cubic_max_gap") == "3");
                } else REQUIRE(part.config.empty());
                const auto part_bounds = part.mesh().transformed_bounding_box(part.get_matrix());
                total_triangles += part.mesh().facets_count();
                volumes.push_back({
                    {"index", index}, {"name", part.name}, {"type", ModelVolume::type_to_string(part.type())},
                    {"triangle_count", part.mesh().facets_count()},
                    {"bounds_min_mm", {part_bounds.min.x(), part_bounds.min.y(), part_bounds.min.z()}},
                    {"bounds_max_mm", {part_bounds.max.x(), part_bounds.max.y(), part_bounds.max.z()}},
                    {"serialized_config", local}, {"nil_config_keys", local_nil}
                });
            }
            if (common_volume_settings.empty()) common_volume_settings = comparable_volumes;
            else REQUIRE(comparable_volumes == common_volume_settings);
            REQUIRE(total_triangles == model_triangles + size_t(modifier ? 12 : 0));

            const std::string archive = (directory / (stem + ".3mf")).string();
            StoreParams params;
            params.path = archive.c_str();
            params.model = &model;
            params.config = &config;
            params.plate_data_list = {&plate};
            REQUIRE(store_bbs_3mf(params));
            REQUIRE(fs::file_size(archive) > 0);
            const std::string snapshot = (directory / (stem + ".config.ini")).string();
            config.save(snapshot);
            REQUIRE(fs::file_size(snapshot) > 0);
            // Match the embedded project's nozzle-stat representation while
            // retaining the complete applied values separately in the INI.
            DynamicPrintConfig project_config = config;
            split_nozzle_stats_for_export(project_config);
            const std::string project_snapshot = (directory / (stem + ".config.json")).string();
            project_config.save_to_json(project_snapshot, "project_settings", "project", SLIC3R_VERSION);
            REQUIRE(fs::file_size(project_snapshot) > 0);
            manifest["fixtures"].push_back({
                {"name", stem}, {"size", size}, {"family", family}, {"pattern", pattern},
                {"source_fixture", rib ? "InwardWalls" : modifier ? "Modifiers local density angle and gap ownership" : "BACQualification"},
                {"archive", stem + ".3mf"}, {"config_ini", stem + ".config.ini"},
                {"project_config_json", stem + ".config.json"},
                {"dimensions_mm", {width, depth, height}}, {"instance_offset_mm", {80., 80., 0.}},
                {"bounds_min_mm", {bounds.min.x(), bounds.min.y(), bounds.min.z()}},
                {"bounds_max_mm", {bounds.max.x(), bounds.max.y(), bounds.max.z()}},
                {"expected_layer_count", rib ? 75 : 90}, {"triangle_count", total_triangles},
                {"model_triangle_count", model_triangles}, {"volume_count", object->volumes.size()}, {"volumes", volumes},
                {"object_count", 1}, {"instance_count", 1}, {"plate_count", 1},
                {"filament_count", filament_count},
                {"density_percent", density}, {"bac_gap_mm", bac ? nlohmann::json(gap) : nlohmann::json(nullptr)},
                {"config_key_count", config.keys().size()},
                {"serialized_config", serialized_settings}, {"nil_config_keys", nil_settings}
            });
        }
    }
    const std::string manifest_path = (directory / "manifest.json").string();
    std::ofstream output(manifest_path);
    REQUIRE(output.is_open());
    output << std::setw(2) << manifest << '\n';
    output.close();
    REQUIRE(output.good());
    std::cout << "BAC_BENCHMARK_FIXTURES manifest=" << manifest_path << " archives=8 slicing_performed=0" << std::endl;
}
