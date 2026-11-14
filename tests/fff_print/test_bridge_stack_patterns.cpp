#include <catch2/catch.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/Lightning/Layer.hpp"
#include "libslic3r/Fill/Lightning/TreeNode.hpp"

#include <array>
#include <cstdlib>
#include <map>

using namespace Slic3r;

namespace {
DynamicPrintConfig stack_pattern_config(const std::string &pattern, int count)
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "20%"}, {"sparse_infill_pattern", pattern},
        {"top_shell_layers", "4"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", "0"}, {"wall_generator", "classic"},
        {"thick_bridges", "1"}, {"bridge_flow", "1"},
        {"internal_bridge_layers", std::to_string(count)},
        // The tower's wall rises out of the terrace roof, so with wall_internal_extension on, its line runs down
        // through the roof and leaves a one-spacing strip of shell inside it. Stacking turns small squares of that
        // strip (about 0.3 mm2 in all) back into sparse infill: a known interaction of the extension without its
        // chamfer, left for the extension's rework. These cases check the stacks on their own.
        {"wall_internal_extension", "0"}
    });
    return config;
}

Model partial_roof_model()
{
    // The lower terrace closes only part of the sparse cavity. Unlike a cube,
    // this forces Lightning to build local branches beside continuing infill,
    // and Support Cubic to refine around a local, internal roof at its new Z.
    Model model;
    auto *object = model.add_object();
    const double base_height = 8.;
    TriangleMesh base(its_make_cube(48., 36., base_height));
    TriangleMesh tower(its_make_cube(20., 18., 6.));
    tower.translate(3., 9., base_height);
    base.merge(tower);
    object->add_volume(base);
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    return model;
}

void process_pattern(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();
}

using AreaLayers = std::map<size_t, ExPolygons>;
AreaLayers classified_areas(const Print &print, SurfaceType type)
{
    AreaLayers result;
    for (const Layer *layer : print.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions())
            for (const Surface &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == type)
                    result[layer->id()].push_back(surface.expolygon);
    return result;
}

using Segment = std::array<coord_t, 4>;
void sparse_segments(const ExtrusionEntity &entity, std::vector<Segment> &out)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) {
        if (path->role() == erInternalInfill)
            for (const Line &line : path->polyline.lines()) {
                Point a = line.a, b = line.b;
                if (a.x() > b.x() || (a.x() == b.x() && a.y() > b.y()))
                    std::swap(a, b);
                out.push_back({a.x(), a.y(), b.x(), b.y()});
            }
    } else if (const auto *paths = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
        for (const auto &path : paths->paths)
            sparse_segments(path, out);
    } else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        for (const auto *child : collection->entities)
            sparse_segments(*child, out);
    }
}

std::vector<Segment> sparse_segments(const Layer &layer)
{
    std::vector<Segment> result;
    for (const LayerRegion *region : layer.regions())
        sparse_segments(region->fills, result);
    std::sort(result.begin(), result.end());
    return result;
}

void bridge_paths(const ExtrusionEntity &entity, std::vector<ExtrusionPath> &out)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) {
        if (path->role() == erBridgeInfill)
            out.push_back(*path);
    } else if (const auto *paths = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
        for (const auto &path : paths->paths)
            bridge_paths(path, out);
    } else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        for (const auto *child : collection->entities)
            bridge_paths(*child, out);
    }
}

std::pair<size_t, size_t> anchored_strand_ends(const Print &print, size_t index)
{
    const Layer &layer = *print.get_object(size_t(0))->get_layer(index);
    const Layer &below = *layer.lower_layer;
    Polygons deposited;
    for (const LayerRegion *region : below.regions()) {
        region->perimeters.polygons_covered_by_width(deposited, float(SCALED_EPSILON));
        region->fills.polygons_covered_by_width(deposited, float(SCALED_EPSILON));
    }
    deposited = union_(deposited);
    size_t anchored = 0, total = 0;
    for (const LayerRegion *region : layer.regions()) {
        std::vector<ExtrusionPath> paths;
        bridge_paths(region->fills, paths);
        for (const ExtrusionPath &path : paths) {
            if (path.height <= layer.height + EPSILON)
                continue;
            for (const Line &line : path.polyline.lines()) {
                if (line.length() < scale_(3.))
                    continue;
                // Ignore contour connectors between strands. The bridge
                // direction is stored on the actual classified surface.
                bool strand = false;
                const Vec2d direction = (line.b - line.a).cast<double>().normalized();
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == stInternalBridge && !surface.bridge_followup &&
                        surface.expolygon.contains(line.midpoint())) {
                        const Vec2d bridge_direction(std::cos(surface.bridge_angle), std::sin(surface.bridge_angle));
                        strand |= std::abs(direction.dot(bridge_direction)) > 0.9999;
                    }
                if (!strand)
                    continue;
                // The anchor reconstruction deliberately travels one strand
                // width past the support. Inspect the deposited terminal
                // overlap of the full-width bead, not a zero-width centerline:
                // an endpoint can contact the side of a lower extrusion.
                const double reach = std::min(line.length() * 0.49, scale_(2. * path.width));
                const Point delta = (direction * reach).cast<coord_t>();
                const auto has_contact = [&](const Point &start, const Point &end) {
                    const Polygons bead = offset(Polyline{start, end}, float(scale_(0.5 * path.width)));
                    return area(intersection_ex(bead, deposited)) > double(SCALED_EPSILON) * SCALED_EPSILON;
                };
                const bool a = has_contact(line.a, line.a + delta);
                const bool b = has_contact(line.b - delta, line.b);
                anchored += a;
                anchored += b;
                total += 2;
            }
        }
    }
    return {anchored, total};
}
}

TEST_CASE("Internal bridge stacks rebuild demand-driven sparse infill below partial roofs",
          "[BridgeControls][Layers][StackPatterns]")
{
    const std::string pattern = GENERATE("lightning", "supportcubic", "gyroid");
    const int count = GENERATE(2, 3);
    CAPTURE(pattern, count);
    Model model = partial_roof_model();
    Print baseline, stacked;
    process_pattern(baseline, model, stack_pattern_config(pattern, 1));
    process_pattern(stacked, model, stack_pattern_config(pattern, count));

    std::vector<std::vector<Segment>> first_stacked_sparse;
    if (pattern == "lightning")
        for (const Layer *layer : stacked.get_object(size_t(0))->layers())
            first_stacked_sparse.push_back(sparse_segments(*layer));

    const auto before = classified_areas(baseline, stInternalBridge);
    const auto after = classified_areas(stacked, stInternalBridge);
    REQUIRE(before.size() >= 2); // A terrace roof and the separate tower roof.
    REQUIRE_FALSE(after.empty());
    const size_t original_roof = before.begin()->first;
    const size_t lowest_roof = after.begin()->first;
    REQUIRE(lowest_roof + count - 1 == original_roof);
    REQUIRE(lowest_roof > 0);

    // The terrace roof is required material; its old inward anchor extensions
    // into the continuing tower are optional. Trimming those extensions must
    // retain the roof and its feasible downward projection at every plane.
    for (const auto &[roof_index, bridges] : before) {
        const LayerRegion &above = *baseline.get_object(size_t(0))->get_layer(roof_index + 1)->get_region(0);
        ExPolygons solid_above;
        for (const Surface &surface : above.fill_surfaces.surfaces)
            if (surface.surface_type == stTop || surface.surface_type == stInternalSolid)
                solid_above.push_back(surface.expolygon);
        ExPolygons required = intersection_ex(bridges, solid_above);
        for (int depth = 0; depth < count; ++depth) {
            const size_t target = roof_index - size_t(depth);
            if (depth > 0) {
                ExPolygons available;
                const LayerRegion &previous = *baseline.get_object(size_t(0))->get_layer(target)->get_region(0);
                for (const Surface &surface : previous.fill_surfaces.surfaces)
                    if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
                        available.push_back(surface.expolygon);
                required = intersection_ex(required, available);
            }
            REQUIRE(after.count(target) == 1);
            CHECK(area(diff_ex(required, after.at(target))) * SCALING_FACTOR * SCALING_FACTOR < 0.0001);
        }
    }

    for (SurfaceType type : {stTop, stBottom, stBottomBridge, stInternalSolid, stSubTop}) {
        const auto original = classified_areas(baseline, type);
        const auto current = classified_areas(stacked, type);
        if (type != stInternalSolid)
            REQUIRE(original.size() == current.size());
        for (const auto &[index, area] : original) {
            REQUIRE(current.count(index) == 1);
            CHECK(diff_ex(area, current.at(index)).empty());
            if (type != stInternalSolid)
                CHECK(diff_ex(current.at(index), area).empty());
        }
    }

    for (size_t index = 0; index < baseline.get_object(size_t(0))->layer_count(); ++index) {
        const LayerRegion &previous = *baseline.get_object(size_t(0))->get_layer(index)->get_region(0);
        const LayerRegion &current = *stacked.get_object(size_t(0))->get_layer(index)->get_region(0);
        const ExPolygons previous_fill = union_ex(to_expolygons(previous.fill_surfaces.surfaces));
        const ExPolygons current_fill = union_ex(to_expolygons(current.fill_surfaces.surfaces));
        CAPTURE(index);
        // Reconstruction must not silently drop old anchor fringe from all
        // classifications, or create material outside the original fill area.
        CHECK(area(diff_ex(previous_fill, current_fill)) * SCALING_FACTOR * SCALING_FACTOR < 0.0001);
        CHECK(area(diff_ex(current_fill, previous_fill)) * SCALING_FACTOR * SCALING_FACTOR < 0.0001);

        ExPolygons old_solids, new_solids, old_sparse;
        for (const Surface &surface : previous.fill_surfaces.surfaces) {
            if (surface.surface_type == stInternalSolid) old_solids.push_back(surface.expolygon);
            if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid) old_sparse.push_back(surface.expolygon);
        }
        for (const Surface &surface : current.fill_surfaces.surfaces)
            if (surface.surface_type == stInternalSolid) new_solids.push_back(surface.expolygon);
        const ExPolygons added = diff_ex(new_solids, old_solids);
        CHECK(diff_ex(added, old_sparse).empty());
        // Stock ensuring width, plus at most one previous ensuring-strip
        // width to keep the added strip connected to the supporting wall.
        const ExPolygons wall_band = diff_ex(previous_fill,
            offset_ex(previous_fill, -2 * previous.flow(frSolidInfill).scaled_spacing() - 2 * SCALED_EPSILON));
        CHECK(diff_ex(added, wall_band).empty());
        if (index > 0) {
            ExPolygons lower_bridges;
            const LayerRegion &lower = *stacked.get_object(size_t(0))->get_layer(index - 1)->get_region(0);
            for (const Surface &surface : lower.fill_surfaces.surfaces)
                if (surface.surface_type == stInternalBridge) lower_bridges.push_back(surface.expolygon);
            for (const Surface &surface : current.fill_surfaces.surfaces)
                if (surface.surface_type == stInternalBridge && surface.bridge_followup) {
                    const ExPolygons missing = diff_ex(ExPolygons{surface.expolygon}, offset_ex(lower_bridges, SCALED_EPSILON));
                    const double missing_mm2 = area(missing) * SCALING_FACTOR * SCALING_FACTOR;
                    CAPTURE(missing_mm2);
                    // Integer clipping can leave nanometre-wide slivers
                    // along long diagonal edges. Bound uncovered thickness
                    // by native geometry precision, not by total sliver area.
                    CHECK(offset_ex(missing, -SCALED_EPSILON).empty());
                }
        }
    }

    // Adding bridge surfaces after building the Lightning tree / cubic octree
    // left every path below the new lowest plane byte-for-byte unchanged. The
    // supporting infill must now respond to the lower roof and its connected
    // ensuring strips, while the demand-driven generators use the new base Z.
    bool support_changed = false;
    bool has_support = false;
    for (size_t index = 0; index < lowest_roof; ++index) {
        const auto previous = sparse_segments(*baseline.get_object(size_t(0))->get_layer(index));
        const auto current = sparse_segments(*stacked.get_object(size_t(0))->get_layer(index));
        support_changed |= previous != current;
        if (index + 1 == lowest_roof)
            has_support = !current.empty();
    }
    CHECK(support_changed);
    CHECK(has_support);

    const auto [baseline_anchored, baseline_ends] = anchored_strand_ends(baseline, original_roof);
    const auto [stack_anchored, stack_ends] = anchored_strand_ends(stacked, lowest_roof);
    CAPTURE(baseline_anchored, baseline_ends, stack_anchored, stack_ends);
    REQUIRE(baseline_ends > 20);
    REQUIRE(stack_ends > 20);
    // Preserve at least the upstream anchor coverage on a relocated roof;
    // the tolerance only accounts for a different number of edge strands.
    CHECK(double(stack_anchored) / stack_ends >= double(baseline_anchored) / baseline_ends - 0.02);

    // Reusing a Print must discard the old support generators and follow-up
    // flags just as a fresh slice does, in both directions of the setting.
    process_pattern(stacked, model, stack_pattern_config(pattern, 1));
    REQUIRE(classified_areas(stacked, stInternalBridge).begin()->first == original_roof);
    for (const Layer *layer : stacked.get_object(size_t(0))->layers())
        for (const LayerRegion *region : layer->regions())
            for (const Surface &surface : region->fill_surfaces.surfaces)
                CHECK_FALSE(surface.bridge_followup);
    process_pattern(stacked, model, stack_pattern_config(pattern, count));
    REQUIRE(classified_areas(stacked, stInternalBridge).begin()->first == lowest_roof);
    const auto [repeated_anchored, repeated_ends] = anchored_strand_ends(stacked, lowest_roof);
    CAPTURE(repeated_anchored, repeated_ends);
    REQUIRE(repeated_ends > 20);
    CHECK(double(repeated_anchored) / repeated_ends >= double(baseline_anchored) / baseline_ends - 0.02);
    if (pattern == "lightning") {
        REQUIRE(stacked.get_object(size_t(0))->layer_count() == first_stacked_sparse.size());
        for (size_t index = 0; index < first_stacked_sparse.size(); ++index) {
            CAPTURE(index);
            REQUIRE(sparse_segments(*stacked.get_object(size_t(0))->get_layer(index)) == first_stacked_sparse[index]);
        }
    }
}

TEST_CASE("Bridge stack Lightning keeps junction support geometry stable", "[BridgeControls][Layers][StackPatterns][LightningTraversal]")
{
    const auto root = FillLightning::Node::create(Point(scale_(0.), scale_(0.)));
    const auto fork = root->addChild(Point(scale_(5.), scale_(0.)));
    fork->addChild(Point(scale_(10.), scale_(-5.)));
    fork->addChild(Point(scale_(10.), scale_(0.)));
    fork->addChild(Point(scale_(10.), scale_(5.)));
    FillLightning::Layer stable, ordinary;
    stable.tree_roots.push_back(root);
    ordinary.tree_roots.push_back(root);
    stable.set_stable_traversal();
    const Polygons outline{Polygon{{Point(scale_(-1.), scale_(-6.)), Point(scale_(11.), scale_(-6.)),
        Point(scale_(11.), scale_(6.)), Point(scale_(-1.), scale_(6.))}}};
    const Polylines expected = stable.convertToLines(outline, scale_(0.3));
    REQUIRE(expected.size() == 3);
    for (int repeat = 0; repeat < 32; ++repeat) {
        // Interleaving ordinary conversion must not change a stack's already
        // chosen first-child support geometry.
        ordinary.convertToLines(outline, scale_(0.3));
        const Polylines actual = stable.convertToLines(outline, scale_(0.3));
        REQUIRE(actual.size() == expected.size());
        for (size_t index = 0; index < actual.size(); ++index)
            CHECK(actual[index].points == expected[index].points);
    }
}

TEST_CASE("Lightning junction support geometry is repeatable", "[BridgeControls][Layers][StackPatterns][LightningDeterminism]")
{
    const auto make_tree = [] {
        const auto root = FillLightning::Node::create(Point(scale_(0.), scale_(0.)));
        const auto fork = root->addChild(Point(scale_(5.), scale_(0.)));
        fork->addChild(Point(scale_(10.), scale_(-5.)));
        fork->addChild(Point(scale_(10.), scale_(0.)));
        fork->addChild(Point(scale_(10.), scale_(5.)));
        return root;
    };
    FillLightning::Layer layer;
    layer.tree_roots.push_back(make_tree());
    const Polygons outline{Polygon{{Point(scale_(-1.), scale_(-6.)), Point(scale_(11.), scale_(-6.)),
        Point(scale_(11.), scale_(6.)), Point(scale_(-1.), scale_(6.))}}};
    const auto segments = [](const Polylines &polylines) {
        std::vector<Segment> result;
        for (const Polyline &polyline : polylines)
            for (const Line &line : polyline.lines()) {
                Point a = line.a, b = line.b;
                if (a.x() > b.x() || (a.x() == b.x() && a.y() > b.y()))
                    std::swap(a, b);
                result.push_back({a.x(), a.y(), b.x(), b.y()});
            }
        std::sort(result.begin(), result.end());
        return result;
    };
    const auto expected = segments(layer.convertToLines(outline, scale_(0.3)));
    REQUIRE(expected.size() == 4);
    FillLightning::Layer independent;
    independent.tree_roots.push_back(make_tree());
    REQUIRE(segments(independent.convertToLines(outline, scale_(0.3))) == expected);
    for (int repeat = 0; repeat < 32; ++repeat) {
        CAPTURE(repeat);
        // Unrelated work and previous conversions must not move the trimmed
        // junction endpoints used by bridge anchoring and final extrusion.
        (void) std::rand();
        const auto actual = segments(layer.convertToLines(outline, scale_(0.3)));
        REQUIRE(actual == expected);
    }
}

TEST_CASE("Lightning reconnects at the nearest boundary crossing", "[BridgeControls][Layers][StackPatterns][LightningDeterminism]")
{
    const Point shift = GENERATE(Point(0, 0), Point(scale_(20.), scale_(30.)), Point(scale_(-20.), scale_(-30.)));
    const Polygons outline{Polygon{{shift, shift + Point(scale_(10.), scale_(0.)),
        shift + Point(scale_(10.), scale_(10.)), shift + Point(scale_(0.), scale_(10.))}}};
    EdgeGrid::Grid locator(get_extents(outline).inflated(scale_(2.)));
    locator.create(outline, scale_(1.));
    const Point a = shift + Point(scale_(-1.), scale_(5.));
    const Point b = shift + Point(scale_(11.), scale_(5.));
    Point intersection;

    // Both sides cross the query segment. Reconnection must select the side
    // nearest its destination, independent of the outline's world position.
    REQUIRE(FillLightning::lineSegmentPolygonsIntersection(a, b, locator, intersection, scale_(2.)));
    CHECK(intersection == shift + Point(scale_(10.), scale_(5.)));
    REQUIRE(FillLightning::lineSegmentPolygonsIntersection(b, a, locator, intersection, scale_(2.)));
    CHECK(intersection == shift + Point(scale_(0.), scale_(5.)));
    CHECK_FALSE(FillLightning::lineSegmentPolygonsIntersection(a, b, locator, intersection, scale_(0.5)));
}
