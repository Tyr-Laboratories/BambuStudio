#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"
#include "bridge_aware_cubic_observer.hpp"


#include <boost/filesystem.hpp>

#include <algorithm>
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
    double             right_gap = 0.;                    // the gap of the modifier fixtures' right half (x from 24 mm of the part)
    double             right_from_z = 0.;                 // from this height (mm) up
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
        // The modifier fixtures have two gaps: the right half's (x from 24 mm of the part, placed at x = 80) and the left's.
        const auto gap_b = [&fixture](const Vec2d &at, double z) {
            return (fixture.right_gap > 0. && at.x() >= 80. + 24. && z >= fixture.right_from_z - 1e-6 ? fixture.right_gap : fixture.gap) - 0.1; };
        std::vector<double> directions = fixture.other_directions;
        directions.push_back(fixture.direction);
        const K2Measure measure = measure_k2(paths, control, gap_b, directions);
        INFO("added pieces " << measure.added_pieces << ", longest " << measure.longest << " mm, first place " << (measure.where.empty() ? Vec2d(0, 0) : measure.where.front()).transpose());
        CHECK(measure.added_pieces > 0);
        CHECK(measure.free_ends == 0);
        CHECK(measure.long_stretches == 0);
        CHECK(measure_k2(control, control, gap_b, fixture.direction).added_pieces == 0);
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
// gutter between the lattices holds nothing unheld; a floating modifier's first course (at 6.2 mm, over the object's coarse
// lattice) is held from below.
TEST_CASE("Bridge aware cubic holds every line it adds across a seam", "[BridgeAwareCubic][K2][K2seam]")
{
    Workspace workspace;
    check_k2(workspace, { { "modifier", modifier, {}, 8., modifier_config, 45., { 17. }, 3. },
                          { "floating modifier", floating_modifier, {}, 8., modifier_config, 45., { 17. }, 3., 6.2 } });
}

// S3: infill combination (3.9a). The pattern's combined surfaces equal Cubic's (the void surfaces and the thick surfaces per
// layer); on the export K1 holds with every thick bead counted on each layer it fills, and K2 holds.
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
        const std::map<int, EmittedLayer> layers = emitted_layers(paths);
        std::map<int, double> layer_height;
        for (const Layer *layer : object.layers())
            layer_height[plane_key(layer->print_z)] = layer->height;
        // A thick path fills every layer of its height: unfolded onto each (the layers' keys below its own).
        std::vector<EmittedPath> unfolded = paths;
        for (const EmittedPath &path : paths) {
            auto it = layers.find(plane_key(path.z));
            const double h = layer_height.count(plane_key(path.z)) ? layer_height.at(plane_key(path.z)) : path.height;
            if (path.role != erInternalInfill || path.height <= h + 1e-3 || it == layers.end())
                continue;
            for (double filled = h; it != layers.begin() && filled < path.height - 1e-3;) {
                --it;
                unfolded.push_back(path);
                unfolded.back().z = it->first / 1000.;
                filled += layer_height.count(it->first) ? layer_height.at(it->first) : h;
            }
        }
        REQUIRE(unfolded.size() > paths.size());
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

// D1: determinism, on the partial roof with three internal bridge layers. Two slices of one project export the same G-code byte for
// byte, and a reslice after a setting changed exports what a fresh slice at the changed value exports (the plan was rebuilt), and
// after it changed back what a fresh slice exports; the anchor settings feed the plan's fused ends.
TEST_CASE("Bridge aware cubic exports the same G-code on every slice and reslices to the same fills", "[BridgeAwareCubic][D1]")
{
    Workspace workspace;
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
    CHECK(without_time(exported_gcode(workspace, partial_roof(), settings, "d1-again.gcode")) == fresh);
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
