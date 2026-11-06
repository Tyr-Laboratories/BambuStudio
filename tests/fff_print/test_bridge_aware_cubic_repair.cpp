#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSupport.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {

ExPolygon rectangle(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon{Points{Point::new_scale(x0, y0), Point::new_scale(x1, y0),
                                   Point::new_scale(x1, y1), Point::new_scale(x0, y1)}});
}

Polyline segment(double x0, double y0, double x1, double y1)
{
    return Polyline(Point::new_scale(x0, y0), Point::new_scale(x1, y1));
}

void visit_paths(const ExtrusionEntity &entity, const std::function<void(const ExtrusionPath &)> &visit)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) visit(*path);
    else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const ExtrusionEntity *child : collection->entities) visit_paths(*child, visit);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const ExtrusionPath &path : multi->paths) visit(path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const ExtrusionPath &path : loop->paths) visit(path);
}

} // namespace

TEST_CASE("Bridge aware cubic certifies final local repairs after attachment and coarsening",
          "[BridgeAwareCubic][Integration][RepairLocality]")
{
    Model model;
    auto *object = model.add_object();
    object->name = "Bridge aware cubic local repair fixture";
    object->add_volume(TriangleMesh(its_make_cube(32., 28., 18.)));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig&>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "5%"}, {"sparse_infill_pattern", "bridgeawarecubic"},
        {"bridge_aware_cubic_max_gap", "5"},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
        {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"}
    });
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(object);
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const auto *plan = print.get_object(size_t(0))->bridge_aware_cubic_plan(0);
    REQUIRE(plan != nullptr);
    REQUIRE_FALSE(plan->demands().empty());
    // Final plans contain surviving obligations, not intermediate repair
    // history. A temporary Z=10.4 demand may disappear after endpoint repair
    // and coarsening. process() must still complete its final certificate on
    // nonempty actual sparse and model-roof deposition.
    size_t sparse_paths = 0, model_paths = 0;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions())
            visit_paths(region->fills, [&](const ExtrusionPath &path) {
                if (path.is_force_no_extrusion() || path.mm3_per_mm <= 0. || path.polyline.length() == 0.) return;
                if (path.role() == erInternalInfill) ++sparse_paths;
                else if (is_solid_infill(path.role())) ++model_paths;
            });
    REQUIRE(sparse_paths > 0);
    REQUIRE(model_paths > 0);
}

TEST_CASE("Bridge aware repair selection excludes passing strands on the same support plane",
          "[BridgeAwareCubic][RepairLocality][RunSelection]")
{
    constexpr double width = .4, gap = 5., tolerance = .001;
    const bool swapped = GENERATE(false, true);
    Polylines paths{segment(0., 0., 2., 0.), segment(0., 3., 4., 3.)};
    if (swapped) std::swap(paths[0], paths[1]);
    const size_t failing_index = swapped ? 1 : 0, passing_index = swapped ? 0 : 1;
    std::vector<ExPolygons> demands;
    ExPolygons grouped;
    for (const Polyline &path : paths) {
        demands.push_back(union_ex(offset(Polylines{path}, scale_(.5 * width))));
        append(grouped, demands.back());
    }
    grouped = union_ex(grouped);
    const ExPolygons support{rectangle(-.5, -.5, 1.8, .5), rectangle(-.5, 2.5, 1., 3.5),
                             rectangle(3., 2.5, 4.5, 3.5)};
    const auto result = Validation::validate({}, support, paths, grouped, gap, tolerance);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    REQUIRE_FALSE(result.passed());
    REQUIRE(result.void_cells.empty());
    REQUIRE(result.unsupported_runs.size() == 2);
    REQUIRE(std::count_if(result.unsupported_runs.begin(), result.unsupported_runs.end(),
        [&](const auto &run) { return run.path_index == failing_index; }) == 1);
    REQUIRE(std::count_if(result.unsupported_runs.begin(), result.unsupported_runs.end(),
        [&](const auto &run) { return run.path_index == passing_index; }) == 1);
    for (const auto &run : result.unsupported_runs) {
        CHECK_FALSE(run.exceeds_gap);
        if (run.path_index == failing_index) {
            CHECK(run.kind == Validation::UnsupportedRunKind::OneSidedOverhang);
            CHECK(run.length_mm == Approx(.2).margin(2. * SCALING_FACTOR));
        } else {
            REQUIRE(run.path_index == passing_index);
            CHECK(run.kind == Validation::UnsupportedRunKind::TwoEndedBridge);
            CHECK(run.length_mm == Approx(2.).margin(2. * SCALING_FACTOR));
        }
    }
    const auto passing = Validation::validate({}, support, paths, demands[passing_index], gap, tolerance);
    REQUIRE(passing.input_valid);
    REQUIRE(passing.completed);
    REQUIRE(passing.unsupported_runs.size() == 1);
    CHECK(passing.unsupported_runs.front().path_index == passing_index);
    CHECK(passing.passed());

    const ExPolygons repair = BAC::select_unsupported_run_demands(paths, {}, {width, width}, demands,
                                                                 result.unsupported_runs, tolerance);
    REQUIRE_FALSE(repair.empty());
    CHECK(diff_ex(ExPolygons{rectangle(1.85, -.05, 1.95, .05)}, repair).empty());
    CHECK(intersection_ex(repair, demands[passing_index]).empty());
    CHECK(intersection_ex(repair, ExPolygons{rectangle(.2, -.1, .4, .1)}).empty());
    CHECK(diff_ex(repair, demands[failing_index]).empty());
    CHECK(diff_ex(repair, grouped).empty());
}

TEST_CASE("Bridge aware repair retains each path's selected scope at a crossing",
          "[BridgeAwareCubic][RepairLocality][RunSelection][DemandSelection]")
{
    constexpr double width = .4, gap = 2., tolerance = .001;
    const bool swapped = GENERATE(false, true);
    Polylines paths{segment(0., 0., 10., 0.), segment(5., -2., 5., 2.)};
    std::vector<ExPolygons> demands{
        intersection_ex(union_ex(offset(Polylines{paths[0]}, scale_(.5 * width))),
                        ExPolygons{rectangle(8.5, -.2, 10.1, .2)}),
        union_ex(offset(Polylines{paths[1]}, scale_(.5 * width)))};
    if (swapped) {
        std::swap(paths[0], paths[1]);
        std::swap(demands[0], demands[1]);
    }
    const size_t restricted_index = swapped ? 1 : 0;
    ExPolygons grouped = demands[0];
    append(grouped, demands[1]);
    grouped = union_ex(grouped);
    const ExPolygons support{rectangle(-1., -.5, 2., .5), rectangle(7., -.5, 11., .5),
        rectangle(4.5, -3., 5.5, -.5), rectangle(4.5, .5, 5.5, 3.)};
    for (size_t index = 0; index < paths.size(); ++index) {
        const auto scoped = Validation::validate({}, support, {paths[index]}, demands[index], gap, tolerance);
        REQUIRE(scoped.input_valid);
        REQUIRE(scoped.completed);
        CHECK(scoped.passed());
    }
    const auto leaked = Validation::validate({}, support, paths, grouped, gap, tolerance);
    REQUIRE(leaked.input_valid);
    REQUIRE(leaked.completed);
    REQUIRE_FALSE(leaked.passed());
    REQUIRE(leaked.unsupported_runs.size() == 2);
    const auto remote = std::find_if(leaked.unsupported_runs.begin(), leaked.unsupported_runs.end(),
        [&](const Validation::UnsupportedRun &run) { return run.path_index == restricted_index; });
    REQUIRE(remote != leaked.unsupported_runs.end());
    REQUIRE(remote->kind == Validation::UnsupportedRunKind::TwoEndedBridge);
    REQUIRE(remote->exceeds_gap);
    CHECK(remote->length_mm == Approx(5.).margin(2. * SCALING_FACTOR));

    // The same physical source bead crosses the other path's selected mask.
    // Clipping only to the source bead cannot repair this lost demand identity.
    const ExPolygons wrong = BAC::select_unsupported_run_demands(paths, {}, {width, width},
        {grouped, grouped}, leaked.unsupported_runs, tolerance);
    REQUIRE_FALSE(wrong.empty());
    CHECK_FALSE(intersection_ex(wrong, ExPolygons{rectangle(4.9, -.05, 5.1, .05)}).empty());
    const ExPolygons correct = BAC::select_unsupported_run_demands(paths, {}, {width, width}, demands,
                                                                 leaked.unsupported_runs, tolerance);
    CHECK(correct.empty());
}

TEST_CASE("Bridge aware selected repair propagates the complete failing run across a region seam",
          "[BridgeAwareCubic][RepairLocality][RunSelection][CompleteSelectedRuns]")
{
    constexpr double width = .4, gap = 3., tolerance = .001;
    const bool swapped = GENERATE(false, true);
    Polylines paths{
        Polyline(Points{Point::new_scale(-4., 0.), Point::new_scale(0., 0.), Point::new_scale(8., 0.)}),
        segment(-4., 3., 8., 3.)};
    std::vector<ExPolygons> beads;
    for (const Polyline &path : paths)
        beads.push_back(union_ex(offset(Polylines{path}, scale_(.5 * width))));
    std::vector<ExPolygons> demands{
        intersection_ex(beads[0], ExPolygons{rectangle(0., -1., 9., 1.)}), beads[1]};
    if (swapped) {
        std::swap(paths[0], paths[1]);
        std::swap(beads[0], beads[1]);
        std::swap(demands[0], demands[1]);
    }
    const size_t failing_index = swapped ? 1 : 0, passing_index = swapped ? 0 : 1;
    const Polylines originals = paths;
    const ExPolygons support{
        rectangle(-5., -.5, -3., .5), rectangle(2., -.5, 4., .5), rectangle(6., -.5, 9., .5),
        rectangle(-5., 2.5, -2., 3.5), rectangle(0., 2.5, 9., 3.5)};
    const ExPolygons grouped = union_ex(demands[0], demands[1]);
    const auto result = Validation::validate({}, support, paths, grouped, gap, tolerance);
    REQUIRE(result.input_valid);
    REQUIRE(result.completed);
    REQUIRE_FALSE(result.passed());
    REQUIRE(result.void_cells.empty());
    REQUIRE(result.unsupported_runs.size() == 3);
    size_t failing_runs = 0, same_path_passing_runs = 0, unrelated_passing_runs = 0;
    for (const auto &run : result.unsupported_runs) {
        REQUIRE(run.kind == Validation::UnsupportedRunKind::TwoEndedBridge);
        if (run.exceeds_gap) {
            REQUIRE(run.path_index == failing_index);
            CHECK(run.begin_mm == Approx(1.).margin(2. * SCALING_FACTOR));
            CHECK(run.end_mm == Approx(6.).margin(2. * SCALING_FACTOR));
            CHECK(run.length_mm == Approx(5.).margin(2. * SCALING_FACTOR));
            ++failing_runs;
        } else {
            CHECK(run.length_mm == Approx(2.).margin(2. * SCALING_FACTOR));
            if (run.path_index == failing_index) ++same_path_passing_runs;
            else {
                REQUIRE(run.path_index == passing_index);
                ++unrelated_passing_runs;
            }
        }
    }
    REQUIRE(failing_runs == 1);
    REQUIRE(same_path_passing_runs == 1);
    REQUIRE(unrelated_passing_runs == 1);

    const ExPolygons local = BAC::select_unsupported_run_demands(paths, {}, {width, width}, demands,
        result.unsupported_runs, tolerance);
    const ExPolygons complete = BAC::select_unsupported_run_demands(paths, {}, {width, width}, demands,
        result.unsupported_runs, tolerance, true);
    const ExPolygons left_region{rectangle(-4., -1., 0., 1.)};
    const ExPolygons right_region{rectangle(0., -1., 9., 1.)};
    REQUIRE_FALSE(local.empty());
    REQUIRE_FALSE(complete.empty());
    CHECK(diff_ex(local, demands[failing_index]).empty());
    CHECK(intersection_ex(local, left_region).empty());
    CHECK(diff_ex(local, complete).empty());

    const ExPolygons left_repair = intersection_ex(complete, left_region);
    const ExPolygons right_repair = intersection_ex(complete, right_region);
    REQUIRE_FALSE(left_repair.empty());
    REQUIRE_FALSE(right_repair.empty());
    CHECK(diff_ex(ExPolygons{rectangle(-2.5, -.15, -.5, .15)}, left_repair).empty());
    CHECK(diff_ex(ExPolygons{rectangle(.5, -.15, 1.5, .15)}, right_repair).empty());
    CHECK(diff_ex(complete, union_ex(left_repair, right_repair)).empty());
    CHECK(diff_ex(complete, beads[failing_index]).empty());
    // Neither the separate passing interval of this same source nor a
    // passing source with its own selected mask authorizes additional repair.
    for (const ExPolygons &repair : {local, complete}) {
        CHECK(intersection_ex(repair, ExPolygons{rectangle(4.1, -.2, 5.9, .2)}).empty());
        CHECK(intersection_ex(repair, beads[passing_index]).empty());
        CHECK(intersection_ex(repair, ExPolygons{rectangle(-3.9, -.2, -3.5, .2)}).empty());
    }
    for (size_t index = 0; index < paths.size(); ++index)
        CHECK(paths[index].points == originals[index].points);
}
