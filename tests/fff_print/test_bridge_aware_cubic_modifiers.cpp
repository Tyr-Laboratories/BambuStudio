#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <tuple>

using namespace Slic3r;

namespace {

DynamicPrintConfig modifier_config()
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "5%"}, {"sparse_infill_pattern", "bridgeawarecubic"},
        {"bridge_aware_cubic_max_gap", "8"}, {"infill_direction", "45"},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
        {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"}
    });
    return config;
}

Model fixture(double height = 18.)
{
    Model model;
    auto *object = model.add_object();
    object->name = "Bridge aware cubic modifier fixture";
    object->add_volume(TriangleMesh(its_make_cube(48., 28., height)));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

ModelVolume *right_modifier(Model &model, double height = 18.)
{
    TriangleMesh mesh(its_make_cube(24., 28., height));
    mesh.translate(24., 0., 0.);
    auto *modifier = model.objects.front()->add_volume(mesh);
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    return modifier;
}

void visit_paths(const ExtrusionEntity &entity, const std::function<void(const ExtrusionPath &)> &visit)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) visit(*path);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const auto &path : multi->paths) visit(path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const auto &path : loop->paths) visit(path);
    else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const auto *child : collection->entities) visit_paths(*child, visit);
}

void slice(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();
}

struct GapReport {
    std::map<double, size_t> targets_by_gap;
    std::set<size_t> target_layers;
};

// Measure entire unsupported emitted runs before selecting their governing
// region. Clipping a run at a modifier seam would manufacture an anchor and
// incorrectly turn one excessive gap into two passing gaps.
GapReport check_emitted_gaps(const Print &print)
{
    GapReport report;
    const auto &layers = print.get_object(size_t(0))->layers();
    for (size_t i = 1; i < layers.size(); ++i) {
        std::vector<std::pair<ExPolygons, double>> participating;
        Polygons support;
        for (const LayerRegion *region : layers[i - 1]->regions()) {
            const auto &config = region->region().config();
            if (config.sparse_infill_pattern.value == ipBridgeAwareCubic &&
                config.sparse_infill_density.value > 0. && config.sparse_infill_density.value < 100.) {
                ExPolygons sparse;
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
                        sparse.push_back(surface.expolygon);
                if (!sparse.empty()) participating.emplace_back(std::move(sparse), config.bridge_aware_cubic_max_gap.value);
            }
            const auto deposited_support = [&](const ExtrusionPath &path) {
                if (!path.is_force_no_extrusion()) path.polygons_covered_by_width(support, 0.f);
            };
            visit_paths(region->perimeters, deposited_support);
            visit_paths(region->fills, deposited_support);
        }
        support = union_(support);
        for (const LayerRegion *region : layers[i]->regions())
            visit_paths(region->fills, [&](const ExtrusionPath &path) {
                if (path.role() == erInternalInfill || path.is_force_no_extrusion()) return;
                for (const auto &[sparse, limit] : participating) {
                    if (!intersection_pl(Polylines{path.polyline}, sparse).empty()) {
                        ++report.targets_by_gap[limit];
                        report.target_layers.insert(i);
                    }
                }
                for (const Polyline &gap : diff_pl(Polylines{path.polyline}, support)) {
                    double limit = std::numeric_limits<double>::infinity();
                    for (const auto &[sparse, region_limit] : participating)
                        if (!intersection_pl(Polylines{gap}, sparse).empty()) limit = std::min(limit, region_limit);
                    if (!std::isfinite(limit)) continue;
                    const double measured = unscale<double>(gap.length());
                    CAPTURE(i, limit, measured);
                    CHECK(measured <= limit);
                }
            });
    }
    return report;
}

// Compare deposited geometry independently of path order and direction. The
// cross-section and role are retained so combined and ordinary paths differ.
using Segment = std::tuple<size_t, int, double, double, double, coord_t, coord_t, coord_t, coord_t>;
std::vector<Segment> deposited_segments(const Print &print)
{
    std::vector<Segment> result;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions()) {
            const auto record = [&](const ExtrusionPath &path) {
                if (path.is_force_no_extrusion()) return;
                for (const Line &line : path.polyline.lines()) {
                    Point a = line.a, b = line.b;
                    if (a == b) continue;
                    if (std::make_pair(a.x(), a.y()) > std::make_pair(b.x(), b.y())) std::swap(a, b);
                    result.emplace_back(layer->id(), int(path.role()), path.width, path.height,
                                        path.mm3_per_mm, a.x(), a.y(), b.x(), b.y());
                }
            };
            visit_paths(region->perimeters, record);
            visit_paths(region->fills, record);
        }
    std::sort(result.begin(), result.end());
    return result;
}

} // namespace

TEST_CASE("Bridge aware cubic modifier plans retain local density angle and gap ownership",
          "[BridgeAwareCubic][Integration][Modifiers]")
{
    auto model = fixture();
    auto *modifier = right_modifier(model);
    modifier->config.set("bridge_aware_cubic_max_gap", 3.);
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(12.));
    modifier->config.set("infill_direction", 17.);
    auto config = modifier_config();
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    const auto *object = print.get_object(size_t(0));
    std::set<const BridgeAwareCubic::Plan *> plans;
    std::set<double> gaps;
    for (size_t i = 0; i < object->num_printing_regions(); ++i) {
        const auto &region_config = object->printing_region(i).config();
        const auto *plan = object->bridge_aware_cubic_plan(i);
        REQUIRE(plan != nullptr);
        plans.insert(plan);
        gaps.insert(region_config.bridge_aware_cubic_max_gap.value);
        CHECK(plan->parameters().density == Approx(region_config.sparse_infill_density.value * 0.01));
        CHECK(plan->parameters().angle == Approx(region_config.infill_direction.value * PI / 180.));
        REQUIRE_FALSE(plan->demands().empty());
    }
    REQUIRE(plans.size() == 2);
    REQUIRE((gaps == std::set<double>{3., 8.}));
    const auto report = check_emitted_gaps(print);
    REQUIRE(report.targets_by_gap.count(3.) == 1);
    REQUIRE(report.targets_by_gap.count(8.) == 1);
    CHECK(report.targets_by_gap.at(3.) > 0);
    CHECK(report.targets_by_gap.at(8.) > 0);
}

TEST_CASE("Bridge aware cubic modifier edits rebuild to the same paths as a fresh slice",
          "[BridgeAwareCubic][Integration][Modifiers][Reslice]")
{
    auto model = fixture();
    auto *modifier = right_modifier(model);
    modifier->config.set("bridge_aware_cubic_max_gap", 6.);
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(10.));
    modifier->config.set("infill_direction", 45.);
    auto config = modifier_config();
    Print reused;
    slice(reused, model, config);
    const auto baseline = deposited_segments(reused);
    REQUIRE_FALSE(baseline.empty());

    SECTION("Gap change invalidates support planning") { modifier->config.set("bridge_aware_cubic_max_gap", 3.); }
    SECTION("Direction change invalidates support planning") { modifier->config.set("infill_direction", 17.); }
    SECTION("Density change invalidates support planning") {
        modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(15.));
    }
    reused.apply(model, config);
    reused.process();
    const auto changed = deposited_segments(reused);
    CHECK(changed != baseline);
    Print fresh;
    slice(fresh, model, config);
    CHECK(changed == deposited_segments(fresh));
    REQUIRE_FALSE(check_emitted_gaps(reused).target_layers.empty());

    modifier->config.set("bridge_aware_cubic_max_gap", 6.);
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(10.));
    modifier->config.set("infill_direction", 45.);
    reused.apply(model, config);
    reused.process();
    CHECK(deposited_segments(reused) == baseline);
}

TEST_CASE("Bridge aware cubic does not fill an empty neighboring region to certify a roof",
          "[BridgeAwareCubic][Integration][Modifiers][Limits]")
{
    auto model = fixture();
    // Stop this modifier at the plane below the roof so the upper extrusion
    // remains continuous across the supporting-region seam.
    right_modifier(model, 17.4)->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(0.));
    auto config = modifier_config();
    Print print;
    std::string failure;
    try { slice(print, model, config); }
    catch (const SlicingError &error) {
        failure = error.what();
        // An infeasible boundary must be diagnosed, not mistaken for a missing
        // enum, unrelated exception, or a successful certified slice.
        CHECK(failure.find("Bridge aware cubic") != std::string::npos);
        CHECK(failure.find("layer") != std::string::npos);
    }
    CAPTURE(failure);
    REQUIRE(print.objects().size() == 1);
    REQUIRE(print.get_object(size_t(0))->layer_count() > 50);
    size_t empty_regions = 0, participating_paths = 0;
    bool zero_at_support_plane = false, parent_roof_above = false;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions()) {
            const bool empty = region->region().config().sparse_infill_density.value == 0.;
            if (empty && !region->slices.empty()) ++empty_regions;
            if (empty && std::abs(layer->print_z - 17.4) < EPSILON && !region->slices.empty())
                zero_at_support_plane = true;
            if (!empty && std::abs(layer->print_z - 17.6) < EPSILON)
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    parent_roof_above |= surface.is_solid();
            visit_paths(region->fills, [&](const ExtrusionPath &path) {
                if (path.role() != erInternalInfill || path.is_force_no_extrusion()) return;
                CHECK_FALSE(empty);
                ++participating_paths;
            });
        }
    REQUIRE(empty_regions > 50);
    REQUIRE(zero_at_support_plane);
    REQUIRE(parent_roof_above);
    if (failure.empty()) {
        // A real active-side partition can close the cells at the seam. In
        // that valid outcome, verify complete unsupported runs without ever
        // treating the modifier edge itself as deposited material.
        REQUIRE(participating_paths > 0);
        REQUIRE_FALSE(check_emitted_gaps(print).target_layers.empty());
    }
}

TEST_CASE("Bridge aware cubic respects a through cavity and multiple roof elevations",
          "[BridgeAwareCubic][Integration][Geometry]")
{
    Model model;
    auto *object = model.add_object();
    object->name = "Bridge aware cubic stepped cavity fixture";
    TriangleMesh stepped(its_make_cube(48., 28., 12.));
    TriangleMesh tower(its_make_cube(18., 16., 8.));
    tower.translate(2., 6., 12.);
    stepped.merge(tower);
    object->add_volume(stepped);
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    TriangleMesh bore(its_make_cube(6., 6., 22.));
    bore.translate(7., 11., -1.);
    auto *negative = object->add_volume(bore);
    negative->set_type(ModelVolumeType::NEGATIVE_VOLUME);
    auto config = modifier_config();
    config.set_deserialize_strict({{"bridge_aware_cubic_max_gap", "5"}, {"sparse_infill_density", "15%"}});
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    size_t hole_layers = 0;
    for (const Layer *layer : print.get_object(size_t(0))->layers()) {
        Polygons holes;
        for (const LayerRegion *region : layer->regions())
            for (const Surface &surface : region->slices.surfaces)
                for (Polygon hole : surface.expolygon.holes) {
                    hole.reverse();
                    holes.push_back(std::move(hole));
                }
        if (holes.empty()) continue;
        ++hole_layers;
        const auto cavity_core = offset_ex(holes, -scale_(0.25));
        for (const LayerRegion *region : layer->regions()) {
            Polygons deposited;
            visit_paths(region->fills, [&](const ExtrusionPath &path) {
                if (!path.is_force_no_extrusion()) path.polygons_covered_by_width(deposited, 0.f);
            });
            CHECK(intersection_ex(deposited, cavity_core).empty());
        }
    }
    REQUIRE(hole_layers > 50);
    const auto report = check_emitted_gaps(print);
    REQUIRE(report.target_layers.size() >= 2);
    CHECK(*report.target_layers.rbegin() - *report.target_layers.begin() > 25);
}

TEST_CASE("Bridge aware cubic combines coarse bulk while retaining roof support deposition",
          "[BridgeAwareCubic][Integration][Combination]")
{
    auto model = fixture(60.);
    auto config = modifier_config();
    config.set_deserialize_strict({{"bridge_aware_cubic_max_gap", "4"}, {"sparse_infill_density", "15%"},
                                   {"infill_combination", "1"}});
    Print print;
    REQUIRE_NOTHROW(slice(print, model, config));
    size_t combined_bulk_paths = 0, refined_paths = 0;
    const auto *object = print.get_object(size_t(0));
    for (const Layer *layer : object->layers())
        for (size_t i = 0; i < layer->regions().size(); ++i) {
            const LayerRegion *region = layer->regions()[i];
            const auto *plan = object->bridge_aware_cubic_plan(i);
            REQUIRE(plan != nullptr);
            const auto refinement = plan->refinement_area(layer->print_z);
            visit_paths(region->fills, [&](const ExtrusionPath &path) {
                if (path.role() != erInternalInfill || path.is_force_no_extrusion()) return;
                if (path.height > layer->height + EPSILON && layer->print_z < 25.) ++combined_bulk_paths;
                if (!intersection_pl(Polylines{path.polyline}, refinement).empty()) {
                    ++refined_paths;
                    CHECK(path.height <= layer->height + EPSILON);
                }
            });
        }
    REQUIRE(combined_bulk_paths > 0);
    REQUIRE(refined_paths > 0);
    const auto report = check_emitted_gaps(print);
    REQUIRE(report.targets_by_gap.count(4.) == 1);
    CHECK(report.targets_by_gap.at(4.) > 20);
}
