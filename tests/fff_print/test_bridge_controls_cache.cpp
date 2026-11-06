#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <algorithm>
#include <boost/filesystem.hpp>
#include <fstream>
#include <functional>
#include <map>
#include <iomanip>
#include <limits>
#include <sstream>
#include <nlohmann/json.hpp>
#include <set>
#include <tuple>

using namespace Slic3r;

namespace {

struct BridgeCacheWorkspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("bridge-controls-cache-%%%%-%%%%");
    BridgeCacheWorkspace() { boost::filesystem::create_directories(path); }
    ~BridgeCacheWorkspace()
    {
        boost::system::error_code error;
        boost::filesystem::remove_all(path, error);
    }
};

DynamicPrintConfig cache_config(const std::string &generator, bool stacked)
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "15%"}, {"sparse_infill_pattern", "grid"},
        {"top_shell_layers", "4"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", "0"}, {"wall_generator", generator},
        {"thick_bridges", "1"}, {"bridge_flow", "1"},
        {"infill_wall_overlap", "15%"}, {"bridge_layer_density", "100%"},
        {"bridge_line_travel_extend", "100%"}, {"internal_bridge_layers", stacked ? "3" : "1"}
    });
    return config;
}

Model cache_model(bool internal)
{
    Model model;
    auto *object = model.add_object();
    object->name = "Bridge slicing cache fixture";
    TriangleMesh mesh(its_make_cube(internal ? 30. : 6., 20., 8.));
    if (!internal) {
        TriangleMesh pillar(its_make_cube(6., 20., 8.));
        pillar.translate(24., 0., 0.);
        mesh.merge(pillar);
        TriangleMesh roof(its_make_cube(30., 20., 1.2));
        roof.translate(0., 0., 8.);
        mesh.merge(roof);
    }
    object->add_volume(mesh);
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

void prepare_print(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
}

void visit_cache_paths(const ExtrusionEntity &entity, const std::function<void(const ExtrusionPath &)> &visit)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity))
        visit(*path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const auto &path : loop->paths) visit(path);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const auto &path : multi->paths) visit(path);
    else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const auto *child : collection->entities) visit_cache_paths(*child, visit);
    else
        FAIL("Unexpected extrusion entity in bridge cache fixture");
}

using CacheBridgeSegment = std::tuple<size_t, double, double, double, bool, coord_t, coord_t, coord_t, coord_t>;

std::vector<CacheBridgeSegment> cache_role_paths(const Print &print, ExtrusionRole role)
{
    std::vector<CacheBridgeSegment> result;
    for (const auto *layer : print.get_object(size_t(0))->layers())
        for (const auto *region : layer->regions())
            visit_cache_paths(region->fills, [&](const ExtrusionPath &path) {
                if (path.role() != role) return;
                for (const auto &line : path.polyline.lines()) {
                    Point a = line.a, b = line.b;
                    if (a == b) continue;
                    if (std::make_pair(a.x(), a.y()) > std::make_pair(b.x(), b.y())) std::swap(a, b);
                    result.emplace_back(layer->id(), path.width, path.height, path.mm3_per_mm,
                        path.is_force_no_extrusion(), a.x(), a.y(), b.x(), b.y());
                }
            });
    std::sort(result.begin(), result.end());
    return result;
}



void check_cached_bridge_paths(const std::vector<CacheBridgeSegment> &actual,
                               const std::vector<CacheBridgeSegment> &expected)
{
    REQUIRE(actual.size() == expected.size());
    std::vector<bool> used(expected.size(), false);
    const double maximum_distance_squared = double(SCALED_EPSILON) * double(SCALED_EPSILON);
    auto endpoint = [](const CacheBridgeSegment &segment, bool last) {
        return last ? Vec2d(double(std::get<7>(segment)), double(std::get<8>(segment))) :
                      Vec2d(double(std::get<5>(segment)), double(std::get<6>(segment)));
    };
    for (size_t index = 0; index < actual.size(); ++index) {
        const auto &segment = actual[index];
        size_t best = expected.size();
        double best_distance_squared = std::numeric_limits<double>::infinity();
        for (size_t candidate_index = 0; candidate_index < expected.size(); ++candidate_index) {
            if (used[candidate_index]) continue;
            const auto &candidate = expected[candidate_index];
            // Layer, width, height, flow, and no-extrusion connector flags
            // remain exact. Only coordinates may differ within native EPS.
            if (std::get<0>(segment) != std::get<0>(candidate) ||
                std::get<1>(segment) != std::get<1>(candidate) ||
                std::get<2>(segment) != std::get<2>(candidate) ||
                std::get<3>(segment) != std::get<3>(candidate) ||
                std::get<4>(segment) != std::get<4>(candidate)) continue;
            for (bool reversed : {false, true}) {
                const double distance_squared = std::max(
                    (endpoint(segment, false) - endpoint(candidate, reversed)).squaredNorm(),
                    (endpoint(segment, true) - endpoint(candidate, !reversed)).squaredNorm());
                if (distance_squared < best_distance_squared) {
                    best_distance_squared = distance_squared;
                    best = candidate_index;
                }
            }
        }
        CAPTURE(index, best_distance_squared, maximum_distance_squared);
        REQUIRE(best < expected.size());
        REQUIRE(best_distance_squared <= maximum_distance_squared);
        // A fresh segment cannot satisfy more than one cached segment, so
        // the tolerance cannot hide duplicate or missing extrusions.
        used[best] = true;
    }
}

std::vector<CacheBridgeSegment> cache_bridge_paths(const Print &print)
{
    return cache_role_paths(print, erBridgeInfill);
}

using CacheFlowProfile = std::set<std::tuple<double, double, double>>;

std::string cache_flow_diagnostics(const CacheFlowProfile &actual, const CacheFlowProfile &expected)
{
    std::ostringstream out;
    out << std::setprecision(17);
    for (const auto &[name, profile] : {std::make_pair("actual", &actual), std::make_pair("expected", &expected)}) {
        out << name << "={";
        for (const auto &[width, height, flow] : *profile)
            out << '(' << width << ',' << height << ',' << flow << "),";
        out << "} ";
    }
    return out.str();
}

std::string cache_surface_flow_diagnostics(const Print &print, size_t index)
{
    std::ostringstream out;
    out << std::setprecision(17);
    for (const LayerRegion *region : print.get_object(size_t(0))->get_layer(index)->regions()) {
        size_t bases = 0, followups = 0;
        double base_area = 0., followup_area = 0.;
        for (const Surface &surface : region->fill_surfaces.surfaces)
            if (surface.surface_type == stInternalBridge) {
                (surface.bridge_followup ? followups : bases)++;
                (surface.bridge_followup ? followup_area : base_area) += surface.expolygon.area() * SCALING_FACTOR * SCALING_FACTOR;
            }
        out << "base_surfaces=" << bases << " base_area=" << base_area
            << " followup_surfaces=" << followups << " followup_area=" << followup_area << "; ";
    }
    return out.str();
}

struct CacheFillLayer {
    Polygons coverage;
    CacheFlowProfile flows;
    double volume = 0.;
    double junction_trim = 0.;
    double junction_volume_budget = 0.;
};
using CacheFillLayers = std::map<size_t, CacheFillLayer>;

CacheFillLayers cache_fill_layers(const Print &print, ExtrusionRole role)
{
    CacheFillLayers result;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions())
            visit_cache_paths(region->fills, [&](const ExtrusionPath &path) {
                if (path.role() != role || path.is_force_no_extrusion() || path.polyline.length() == 0.) return;
                auto &output = result[layer->id()];
                output.volume += unscale<double>(path.polyline.length()) * path.mm3_per_mm;
                output.flows.emplace(path.width, path.height, path.mm3_per_mm);
                path.polygons_covered_by_width(output.coverage, 0.f);
                // With boundary connectors disabled, Lightning's random choice
                // of continuing branch can only relocate a terminal junction
                // trim of at most half a strand spacing per resulting path.
                const double trim = region->flow(frInfill).spacing() * 0.5;
                output.junction_trim = std::max(output.junction_trim, trim);
                output.junction_volume_budget += trim * path.mm3_per_mm;
            });
    for (auto &[index, layer] : result)
        layer.coverage = union_(layer.coverage);
    return result;
}

double cache_fill_volume(const CacheFillLayers &layers)
{
    double volume = 0.;
    for (const auto &[index, layer] : layers) volume += layer.volume;
    return volume;
}

void compare_lightning_sparse(const CacheFillLayers &actual, const CacheFillLayers &expected)
{
    REQUIRE_FALSE(expected.empty());
    REQUIRE(actual.size() == expected.size());
    double total_budget = 0.;
    for (const auto &[index, reference] : expected) {
        CAPTURE(index);
        REQUIRE(actual.count(index) == 1);
        const auto &current = actual.at(index);
        CHECK(bool(current.flows == reference.flows));
        // Test physical deposition, independently of random traversal order.
        // A junction trim may switch branches, but cannot move an entire
        // branch or erase infill elsewhere in this layer.
        const double distance = std::max(current.junction_trim, reference.junction_trim) + unscale<double>(SCALED_EPSILON);
        const auto reference_neighborhood = offset_ex(reference.coverage, float(scale_(distance)));
        const auto current_neighborhood = offset_ex(current.coverage, float(scale_(distance)));
        CHECK(area(diff_ex(current.coverage, reference_neighborhood)) * SCALING_FACTOR * SCALING_FACTOR < 0.0001);
        CHECK(area(diff_ex(reference.coverage, current_neighborhood)) * SCALING_FACTOR * SCALING_FACTOR < 0.0001);
        const double budget = current.junction_volume_budget + reference.junction_volume_budget;
        CHECK(current.volume == Approx(reference.volume).margin(budget + 1.e-6));
        total_budget += budget;
    }
    const double reference_volume = cache_fill_volume(expected);
    REQUIRE(reference_volume > 0.);
    // Ensure the geometrically justified allowance is informative: it must
    // not permit loss of even a quarter of this fixture's sparse material.
    REQUIRE(total_budget < reference_volume * 0.25);
    CHECK(cache_fill_volume(actual) == Approx(reference_volume).margin(total_budget + 1.e-6));
}

void compare_bridge_metadata(const Print &source, const Print &restored, bool stacked)
{
    const auto *before = source.get_object(size_t(0));
    const auto *after = restored.get_object(size_t(0));
    REQUIRE(after->layer_count() == before->layer_count());
    size_t nonzero_overlap_layers = 0, followup_surfaces = 0;
    std::set<size_t> bridge_planes;
    for (size_t index = 0; index < before->layer_count(); ++index) {
        const auto *expected_layer = before->get_layer(index);
        const auto *actual_layer = after->get_layer(index);
        REQUIRE(actual_layer->regions().size() == expected_layer->regions().size());
        for (size_t region_index = 0; region_index < expected_layer->regions().size(); ++region_index) {
            CAPTURE(index, region_index);
            const auto *expected = expected_layer->get_region(int(region_index));
            const auto *actual = actual_layer->get_region(int(region_index));
            nonzero_overlap_layers += expected->bridge_fill_overlap_delta != 0;
            CHECK(actual->bridge_fill_overlap_delta == expected->bridge_fill_overlap_delta);
            CHECK(bool(actual->bridge_fill_expolygons == expected->bridge_fill_expolygons));
            REQUIRE(actual->fill_surfaces.surfaces.size() == expected->fill_surfaces.surfaces.size());
            for (size_t surface_index = 0; surface_index < expected->fill_surfaces.surfaces.size(); ++surface_index) {
                const auto &expected_surface = expected->fill_surfaces.surfaces[surface_index];
                const auto &actual_surface = actual->fill_surfaces.surfaces[surface_index];
                followup_surfaces += expected_surface.bridge_followup;
                if (expected_surface.surface_type == stInternalBridge) bridge_planes.insert(index);
                CHECK(actual_surface.bridge_followup == expected_surface.bridge_followup);
            }
        }
    }
    REQUIRE(nonzero_overlap_layers > 0);
    if (stacked) {
        REQUIRE(bridge_planes.size() == 3);
        REQUIRE(followup_surfaces > 0);
    }
}

void compare_density_reslice(Print &restored, Model &model, DynamicPrintConfig config)
{
    restored.process(nullptr, true);
    REQUIRE(restored.get_object(size_t(0))->is_step_done(posPrepareInfill));
    const auto original_paths = cache_bridge_paths(restored);
    REQUIRE_FALSE(original_paths.empty());
    config.set_key_value("bridge_layer_density", new ConfigOptionPercent(50.));
    restored.apply(model, config);
    REQUIRE(restored.get_object(size_t(0))->is_step_done(posPerimeters));
    REQUIRE_FALSE(restored.get_object(size_t(0))->is_step_done(posPrepareInfill));
    REQUIRE_FALSE(restored.get_object(size_t(0))->is_step_done(posInfill));
    restored.process();
    const auto actual_paths = cache_bridge_paths(restored);
    REQUIRE(bool(actual_paths != original_paths));

    Print fresh;
    prepare_print(fresh, model, config);
    fresh.process();
    const auto expected_paths = cache_bridge_paths(fresh);
    REQUIRE_FALSE(expected_paths.empty());
    CAPTURE(actual_paths.size(), expected_paths.size());
    // Preparation starts from cached, already simplified perimeter paths.
    // Arachne's fresh paths can differ by a few scaled coordinate units.
    // Keep the native geometric tolerance and exact deposition attributes.
    check_cached_bridge_paths(actual_paths, expected_paths);
}

size_t remove_bridge_metadata(nlohmann::json &value)
{
    size_t removed = 0;
    if (value.is_object()) {
        for (const auto *key : {"bridge_fill_expolygons", "bridge_fill_overlap_delta", "bridge_followup"})
            removed += value.erase(key);
        for (auto &child : value.items()) removed += remove_bridge_metadata(child.value());
    } else if (value.is_array())
        for (auto &child : value) removed += remove_bridge_metadata(child);
    return removed;
}

} // namespace

TEST_CASE("Cached bridge overlap and stack flow survive a later density edit", "[BridgeControls][Cache][Reslice]")
{
    const std::string generator = GENERATE("classic", "arachne");
    const bool internal = GENERATE(false, true);
    CAPTURE(generator, internal);
    BridgeCacheWorkspace workspace;
    auto config = cache_config(generator, internal);
    auto model = cache_model(internal);
    Print source;
    prepare_print(source, model, config);
    source.process();
    int exported = 0;
    REQUIRE(source.export_cached_data(workspace.path.string(), exported) == 0);
    REQUIRE(exported == 1);

    Print restored;
    prepare_print(restored, model, config);
    REQUIRE(restored.load_cached_data(workspace.path.string()) == 0);
    compare_bridge_metadata(source, restored, internal);
    compare_density_reslice(restored, model, config);
}

TEST_CASE("Legacy slicing cache defaults missing bridge metadata safely", "[BridgeControls][Cache][Legacy]")
{
    BridgeCacheWorkspace workspace;
    auto config = cache_config("classic", false);
    config.set_key_value("bridge_line_travel_extend", new ConfigOptionPercent(15.));
    auto model = cache_model(true);
    Print source;
    prepare_print(source, model, config);
    source.process();
    int exported = 0;
    REQUIRE(source.export_cached_data(workspace.path.string(), exported) == 0);
    REQUIRE(exported == 1);
    size_t rewritten = 0, removed = 0;
    for (const auto &entry : boost::filesystem::directory_iterator(workspace.path)) {
        if (entry.path().extension() != ".json") continue;
        nlohmann::json cached;
        {
            std::ifstream input(entry.path().string());
            REQUIRE(input.good());
            input >> cached;
        }
        removed += remove_bridge_metadata(cached);
        std::ofstream output(entry.path().string(), std::ios::trunc);
        REQUIRE(output.good());
        output << cached.dump();
        ++rewritten;
    }
    REQUIRE(rewritten == 1);
    REQUIRE(removed > 0);

    // Retain the current filename/hash here to exercise the legacy reader
    // fallback itself, independently of the import hash guard tested below.
    Print restored;
    prepare_print(restored, model, config);
    REQUIRE(restored.load_cached_data(workspace.path.string()) == 0);
    REQUIRE(restored.get_object(size_t(0))->layer_count() == source.get_object(size_t(0))->layer_count());
    for (const auto *layer : restored.get_object(size_t(0))->layers())
        for (const auto *region : layer->regions()) {
            CHECK(region->bridge_fill_overlap_delta == 0);
            CHECK(bool(region->bridge_fill_expolygons == region->fill_expolygons));
            for (const auto &surface : region->fill_surfaces.surfaces)
                CHECK_FALSE(surface.bridge_followup);
        }
    compare_density_reslice(restored, model, config);
}

TEST_CASE("Bridge control changes cannot import another configuration's slicing cache", "[BridgeControls][Cache][Config]")
{
    const std::string key = GENERATE("bridge_layer_density", "bridge_line_travel_extend", "internal_bridge_layers");
    CAPTURE(key);
    BridgeCacheWorkspace workspace;
    auto config = cache_config("classic", false);
    config.set_key_value("bridge_line_travel_extend", new ConfigOptionPercent(15.));
    auto model = cache_model(true);
    Print source;
    prepare_print(source, model, config);
    source.process();
    REQUIRE(source.num_print_regions() == 1);
    const auto original_hash = source.get_print_region(0).config_hash();
    int exported = 0;
    REQUIRE(source.export_cached_data(workspace.path.string(), exported) == 0);
    REQUIRE(exported == 1);

    if (key == "internal_bridge_layers") config.set_key_value(key, new ConfigOptionInt(3));
    else config.set_key_value(key, new ConfigOptionPercent(key == "bridge_layer_density" ? 50. : 100.));
    Print restored;
    prepare_print(restored, model, config);
    REQUIRE(restored.num_print_regions() == 1);
    REQUIRE(restored.get_print_region(0).config_hash() != original_hash);
    REQUIRE(restored.load_cached_data(workspace.path.string()) == 0);
    CHECK(restored.get_object(size_t(0))->layer_count() == 0);
    restored.process(nullptr, true);
    REQUIRE(restored.get_object(size_t(0))->layer_count() > 0);

    Print fresh;
    prepare_print(fresh, model, config);
    fresh.process();
    CHECK(bool(cache_bridge_paths(restored) == cache_bridge_paths(fresh)));
}


TEST_CASE("Cached density edits rebuild Lightning and cubic infill dependencies",
          "[BridgeControls][Cache][Dependencies]")
{
    const std::string pattern = GENERATE("lightning", "adaptivecubic", "supportcubic");
    const int bridge_layers = GENERATE(1, 3);
    CAPTURE(pattern, bridge_layers);
    BridgeCacheWorkspace workspace;
    auto config = cache_config("classic", bridge_layers == 3);
    config.set_deserialize_strict({
        {"sparse_infill_pattern", pattern}, {"sparse_infill_anchor", "0"},
        {"sparse_infill_anchor_max", "0"}
    });
    auto model = cache_model(true);
    Print source;
    prepare_print(source, model, config);
    source.process();
    const auto original_sparse = cache_fill_layers(source, erInternalInfill);
    const auto original_bridges = cache_fill_layers(source, erBridgeInfill);
    REQUIRE(cache_fill_volume(original_sparse) > 0.);
    REQUIRE(original_bridges.size() == size_t(bridge_layers));
    int exported = 0;
    REQUIRE(source.export_cached_data(workspace.path.string(), exported) == 0);
    REQUIRE(exported == 1);

    Print restored;
    prepare_print(restored, model, config);
    REQUIRE(restored.load_cached_data(workspace.path.string()) == 0);
    REQUIRE(restored.get_object(size_t(0))->layer_count() == source.get_object(size_t(0))->layer_count());
    restored.process(nullptr, true);
    REQUIRE(bool(cache_role_paths(restored, erInternalInfill) == cache_role_paths(source, erInternalInfill)));
    REQUIRE(bool(cache_bridge_paths(restored) == cache_bridge_paths(source)));
    REQUIRE(restored.get_object(size_t(0))->is_step_done(posPrepareInfill));

    config.set_key_value("bridge_layer_density", new ConfigOptionPercent(50.));
    restored.apply(model, config);
    REQUIRE(restored.get_object(size_t(0))->is_step_done(posSlice));
    REQUIRE(restored.get_object(size_t(0))->is_step_done(posPerimeters));
    // Without rebuilding preparation, Lightning silently emits no sparse
    // lines and Adaptive/Support Cubic dereference a missing octree. Stop
    // safely here on an unpatched build, before exercising either failure.
    REQUIRE_FALSE(restored.get_object(size_t(0))->is_step_done(posPrepareInfill));
    REQUIRE_FALSE(restored.get_object(size_t(0))->is_step_done(posInfill));
    restored.process();

    Print fresh;
    prepare_print(fresh, model, config);
    fresh.process();
    const auto actual_sparse = cache_fill_layers(restored, erInternalInfill);
    const auto expected_sparse = cache_fill_layers(fresh, erInternalInfill);
    if (pattern == "lightning")
        compare_lightning_sparse(actual_sparse, expected_sparse);
    else {
        REQUIRE_FALSE(expected_sparse.empty());
        CHECK(bool(cache_role_paths(restored, erInternalInfill) == cache_role_paths(fresh, erInternalInfill)));
        CHECK(cache_fill_volume(actual_sparse) == Approx(cache_fill_volume(expected_sparse)).margin(1.e-6));
        CHECK(bool(cache_bridge_paths(restored) == cache_bridge_paths(fresh)));
    }

    const auto actual_bridges = cache_fill_layers(restored, erBridgeInfill);
    const auto expected_bridges = cache_fill_layers(fresh, erBridgeInfill);
    REQUIRE(actual_bridges.size() == size_t(bridge_layers));
    REQUIRE(actual_bridges.size() == expected_bridges.size());
    CHECK(cache_fill_volume(actual_bridges) < cache_fill_volume(original_bridges));
    for (const auto &[index, reference] : expected_bridges) {
        CAPTURE(index);
        REQUIRE(actual_bridges.count(index) == 1);
        REQUIRE(original_bridges.count(index) == 1);
        const auto &actual = actual_bridges.at(index);
        // Density is allowed to change the number of strands, never the
        // strand cross-section or the thick-base/thin-follow-up flow policy.
        INFO("fresh comparison: " << cache_flow_diagnostics(actual.flows, reference.flows));
        INFO("original comparison: " << cache_flow_diagnostics(actual.flows, original_bridges.at(index).flows));
        INFO("restored surfaces: " << cache_surface_flow_diagnostics(restored, index));
        INFO("fresh surfaces: " << cache_surface_flow_diagnostics(fresh, index));
        INFO("original surfaces: " << cache_surface_flow_diagnostics(source, index));
        if (pattern != "lightning" || bridge_layers != 1) {
            CHECK(bool(actual.flows == reference.flows));
            CHECK(bool(actual.flows == original_bridges.at(index).flows));
        }
        // A retained base fringe may share the original roof's layer with a
        // supported follow-up surface. Check the categories actually present,
        // rather than assuming every strand above the lowest plane is thin.
        std::set<double> allowed_heights;
        for (const LayerRegion *region : restored.get_object(size_t(0))->get_layer(index)->regions())
            for (const Surface &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == stInternalBridge)
                    allowed_heights.insert(region->bridging_flow(frSolidInfill, !surface.bridge_followup).height());
        REQUIRE_FALSE(allowed_heights.empty());
        for (const auto &[width, height, flow] : actual.flows) {
            CHECK(flow > 0.);
            CHECK(std::any_of(allowed_heights.begin(), allowed_heights.end(),
                [&](double expected_height) { return height == Approx(expected_height); }));
        }
        for (double height : allowed_heights)
            CHECK(std::any_of(actual.flows.begin(), actual.flows.end(),
                [&](const auto &profile) { return std::get<1>(profile) == Approx(height); }));
    }

    if (pattern == "lightning" && bridge_layers == 1) {
        // Stock Lightning randomly chooses continuing branches. Rebuilding
        // its anchors can change bridge angle and fitted strand spacing, so
        // independently prepared slices need not have identical flow values.
        // Change density on this same prepared geometry to prove that the
        // density control itself preserves every actual strand flow exactly.
        config.set_key_value("bridge_layer_density", new ConfigOptionPercent(100.));
        restored.apply(model, config);
        REQUIRE(restored.get_object(size_t(0))->is_step_done(posPerimeters));
        REQUIRE(restored.get_object(size_t(0))->is_step_done(posPrepareInfill));
        REQUIRE_FALSE(restored.get_object(size_t(0))->is_step_done(posInfill));
        restored.process();
        const auto full_density_bridges = cache_fill_layers(restored, erBridgeInfill);
        REQUIRE(full_density_bridges.size() == actual_bridges.size());
        CHECK(cache_fill_volume(full_density_bridges) > cache_fill_volume(actual_bridges));
        for (const auto &[index, previous] : actual_bridges) {
            REQUIRE(full_density_bridges.count(index) == 1);
            INFO(cache_flow_diagnostics(full_density_bridges.at(index).flows, previous.flows));
            CHECK(bool(full_density_bridges.at(index).flows == previous.flows));
        }
    }
}
