// Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
// Original source of Thomas Rahm's tree supports:
// https://github.com/ThomasRahm/CuraEngine
//
// Original CuraEngine copyright:
// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "TreeModelVolumes.hpp"
#include "TreeSupportCommon.hpp"

#include "../BuildVolume.hpp"
#include "../ClipperUtils.hpp"
#include "../Flow.hpp"
#include "../Layer.hpp"
#include "../Point.hpp"
#include "../Print.hpp"
#include "../PrintConfig.hpp"
#include "../Utils.hpp"
#include "../format.hpp"

#include <optional>
#include <string_view>

#include <boost/log/trivial.hpp>

#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>

namespace Slic3r::TreeSupport3D
{

using namespace std::literals;

// or warning
// had to use a define beacuse the macro processing inside macro BOOST_LOG_TRIVIAL()
#define error_level_not_in_cache debug

//FIXME Machine border is currently ignored.
static Polygons calculateMachineBorderCollision(Polygon machine_border)
{
    // Put a border of 1m around the print volume so that we don't collide.
#if 1
    //FIXME just returning no border will let tree support legs collide with print bed boundary
    return {};
#else
    //FIXME offsetting by 1000mm easily overflows int32_tr coordinate.
    Polygons out = offset(machine_border, scaled<float>(1000.), jtMiter, 1.2);
    machine_border.reverse(); // Makes the polygon negative so that we subtract the actual volume from the collision area.
    out.emplace_back(std::move(machine_border));
    return out;
#endif
}

template<typename Done, typename Compute>
void TreeModelVolumes::compute_once(const LazyCache cache, const coord_t radius, Done &&done, Compute &&compute) const
{
    std::mutex *lock;
    {
        std::lock_guard<std::mutex> guard(m_lazy_locks->mutex);
        std::unique_ptr<std::mutex> &slot = m_lazy_locks->locks[{ int(cache), radius }];
        if (! slot)
            slot = std::make_unique<std::mutex>();
        lock = slot.get();
    }
    std::lock_guard<std::mutex> guard(*lock);
    // Another thread may have computed the entry while this one waited.
    if (! done())
        // Waiting for its nested loops, this thread must not pick up a task that
        // requests the same entries: that task would wait for this very thread.
        tbb::this_task_arena::isolate(std::forward<Compute>(compute));
}

TreeModelVolumes::TreeModelVolumes(
    const PrintObject &print_object,
    const BuildVolume &build_volume,
    const coord_t max_move, const coord_t max_move_slow, size_t current_mesh_idx, 
#ifdef SLIC3R_TREESUPPORTS_PROGRESS
    double progress_multiplier, double progress_offset, 
#endif // SLIC3R_TREESUPPORTS_PROGRESS
    const std::vector<Polygons>& additional_excluded_areas) :
    // -2 to avoid rounding errors
    m_max_move{ std::max<coord_t>(max_move - 2, 0) }, m_max_move_slow{ std::max<coord_t>(max_move_slow - 2, 0) },
#ifdef SLIC3R_TREESUPPORTS_PROGRESS
    m_progress_multiplier{ progress_multiplier }, m_progress_offset{ progress_offset },
#endif // SLIC3R_TREESUPPORTS_PROGRESS
    m_machine_border{ calculateMachineBorderCollision(build_volume.polygon()) }
{
    m_bed_area = build_volume.polygon();
    Polygons machine_borders;
    if (!m_bed_area.empty()) {
        Polygon hole(m_bed_area);
        hole.reverse();
        ExPolygon  machine_outline(offset(m_bed_area, scale_(1000))[0], hole);
        ExPolygons outlines = machine_outline.split_expoly_with_holes(scale_(1.), {});

        if (print_object.print()->config().enable_wrapping_detection.value) {
            Pointfs wrapping_detection_area = print_object.print()->config().wrapping_exclude_area.values;
            Polygon wrapping_poly;
            for (size_t i = 0; i < wrapping_detection_area.size(); ++i) {
                auto pt = wrapping_detection_area[i];
                wrapping_poly.points.emplace_back(Point(scale_(pt.x()), scale_(pt.y())) - print_object.instances().front().shift);
            }
            outlines = union_ex(outlines, {wrapping_poly});
        }

        for (const auto &outline : outlines) machine_borders.emplace_back(outline.contour);
    }
#if 0
    std::unordered_map<size_t, size_t> mesh_to_layeroutline_idx;
    for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); ++ mesh_idx) {
        SliceMeshStorage mesh = storage.meshes[mesh_idx];
        bool added = false;
        for (size_t idx = 0; idx < m_layer_outlines.size(); ++ idx)
            if (TreeSupport::TreeSupportSettings(m_layer_outlines[idx].first) == TreeSupport::TreeSupportSettings(mesh.settings)) {
                added = true;
                mesh_to_layeroutline_idx[mesh_idx] = idx;
            }
        if (! added) {
            mesh_to_layeroutline_idx[mesh_idx] = m_layer_outlines.size();
            m_layer_outlines.emplace_back(mesh.settings, std::vector<Polygons>(storage.support.supportLayers.size(), Polygons()));
        }
    }
    for (size_t idx = 0; idx < m_layer_outlines.size(); ++ idx) {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, m_layer_outlines[idx].second.size()),
            [&](const tbb::blocked_range<size_t> &range) {
            for (const size_t layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx)
                m_layer_outlines[idx].second[layer_idx] = union_(m_layer_outlines[idx].second[layer_idx]); 
        });
    }
    m_current_outline_idx = mesh_to_layeroutline_idx[current_mesh_idx];

#else
    {
        m_anti_overhang = print_object.slice_support_blockers();
        TreeSupportMeshGroupSettings mesh_settings(print_object);
        const TreeSupportSettings config{ mesh_settings, print_object.slicing_parameters() };
	    m_current_min_xy_dist = config.xy_min_distance;
	    m_current_min_xy_dist_delta = config.xy_distance - m_current_min_xy_dist;
	    assert(m_current_min_xy_dist_delta >= 0);
	    m_increase_until_radius = config.increase_radius_until_radius;
	    m_radius_0 = config.getRadius(0);
	    m_raft_layers = config.raft_layers;
        m_current_outline_idx = 0;
        m_layer_outlines.emplace_back(mesh_settings, std::vector<Polygons>{});
        std::vector<Polygons> &outlines = m_layer_outlines.front().second;
        size_t num_raft_layers = m_raft_layers.size();
        size_t num_layers = print_object.layer_count() + num_raft_layers;
        outlines.assign(num_layers, Polygons{});
        tbb::parallel_for(tbb::blocked_range<size_t>(num_raft_layers, num_layers, std::min<size_t>(1, std::max<size_t>(16, num_layers / (8 * tbb::this_task_arena::max_concurrency())))),
            [&](const tbb::blocked_range<size_t> &range) {
                for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++layer_idx) {
                    outlines[layer_idx] = polygons_simplify(to_polygons(print_object.get_layer(layer_idx - num_raft_layers)->lslices), mesh_settings.resolution);
                    outlines[layer_idx].insert(outlines[layer_idx].end(), machine_borders.begin(), machine_borders.end());
                }
        });
    }
#endif

    if (print_object.config().support_on_build_plate_only.value && print_object.has_support_base_painting()) {
        std::vector<Polygons> painted = print_object.project_support_base_facets();
        m_painted_bases_only = std::any_of(painted.begin(), painted.end(), [](const Polygons &p) { return !p.empty(); });
        if (m_painted_bases_only) {
            m_painted_base_areas.resize(m_raft_layers.size());
            for (Polygons &layer : painted)
                m_painted_base_areas.emplace_back(union_(layer));
            // Include the exterior collision ring as well as the bed. Starting
            // with just the bed would invent reachable strips at its edges as
            // the first avoidance polygon is inset during propagation.
            m_painted_route_domain = offset(m_bed_area, scaled<float>(1000.), jtMiter, 1.2);
            if (m_painted_route_domain.empty())
                m_painted_bases_only = false;
        }
    }

    {
        // The threshold of TreeSupport::detect_overhangs, which finds the
        // overhangs Organic tips stand on; mirrored, the bases feet stand on.
        const double threshold = print_object.config().support_threshold_angle.value;
        const double angle     = std::min(threshold > EPSILON ? threshold + 1. : 30., 89.);
        m_base_landing_step = coord_t(double(m_layer_outlines[m_current_outline_idx].first.layer_height) / std::tan(angle * M_PI / 180.));
    }

    m_support_rests_on_model = m_painted_bases_only;
    m_min_resolution  = std::numeric_limits<coord_t>::max();
    for (auto data_pair : m_layer_outlines) {
        m_support_rests_on_model |= ! data_pair.first.support_material_buildplate_only;
        m_min_resolution = std::min(m_min_resolution, data_pair.first.resolution);
    }

    if (m_painted_bases_only) {
        const TreeSupportMeshGroupSettings &settings = m_layer_outlines[m_current_outline_idx].first;
        // Same bottom gap as the placeable areas computed with the collisions.
        m_painted_base_below = LayerIndex(round(double(settings.support_bottom_distance) / double(settings.layer_height))) + 1;
        // Neighbouring bands of one sloped surface come from the exact mesh, but
        // the slices clipping them are simplified and offset by the XY
        // compensations. Close the hairline cracks this leaves between them.
        const PrintObjectConfig &object_config = print_object.config();
        const float crack = float(m_min_resolution) + scaled<float>(
            std::abs(object_config.xy_contour_compensation.value) + std::abs(object_config.xy_hole_compensation.value));
        m_painted_crack = crack;
        const std::vector<Polygons> &outlines = m_layer_outlines[m_current_outline_idx].second;
        m_painted_base_visible.assign(m_painted_base_areas.size(), Polygons{});
        Polygons visible;
        for (size_t layer = 0; layer < m_painted_base_areas.size(); ++ layer) {
            if (! visible.empty() && layer > 0) {
                // Paint stays visible until material appearing above it, an
                // overhang, covers it. Material continuing from the layer below
                // covers nothing new; testing only the overhang keeps adjacent
                // bands of a slope from being trimmed by their shared boundary.
                BoundingBox bbox = get_extents(visible);
                bbox.offset(SCALED_EPSILON);
                const Polygons overhang = diff(
                    ClipperUtils::clip_clipper_polygons_with_subject_bbox(outlines[layer], bbox),
                    ClipperUtils::clip_clipper_polygons_with_subject_bbox(outlines[layer - 1], bbox));
                if (! overhang.empty())
                    visible = diff(visible, overhang);
            }
            if (! m_painted_base_areas[layer].empty()) {
                visible = closing(union_(visible, m_painted_base_areas[layer]), crack);
                m_painted_base_visible[layer] = visible;
            }
        }
    }


#if 0
    for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++) {
        SliceMeshStorage mesh = storage.meshes[mesh_idx];
        tbb::parallel_for(tbb::blocked_range<size_t>(0, m_layer_outlines[mesh_to_layeroutline_idx[mesh_idx]].second.size()),
            [&](const tbb::blocked_range<size_t> &range) {
            for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx)
                if (layer_idx < mesh.layer_nr_max_filled_layer) {
                    Polygons outline = extractOutlineFromMesh(mesh, layer_idx);
                    append(m_layer_outlines[mesh_to_layeroutline_idx[mesh_idx]].second[layer_idx], outline);
                }
        });
    }
    if (! additional_excluded_areas.empty()) {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, m_anti_overhang.size()),
            [&](const tbb::blocked_range<size_t> &range) {
            for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                if (layer_idx < coord_t(additional_excluded_areas.size()))
                    append(m_anti_overhang[layer_idx], additional_excluded_areas[layer_idx]);
    //          if (SUPPORT_TREE_AVOID_SUPPORT_BLOCKER)
    //              append(m_anti_overhang[layer_idx], storage.support.supportLayers[layer_idx].anti_overhang);
    //FIXME block wipe tower
    //          if (storage.primeTower.enabled)
    //              append(m_anti_overhang[layer_idx], layer_idx == 0 ? storage.primeTower.outer_poly_first_layer : storage.primeTower.outer_poly);
                m_anti_overhang[layer_idx] = union_(m_anti_overhang[layer_idx]);
            }
        });
    }
#endif
}

TreeModelVolumes::TreeModelVolumes(const TreeModelVolumes &source, LayerIndex seam_layer, const Polygons &seam_centers,
                                 std::function<void()> throw_on_cancel, coord_t max_seam_radius) :
    m_bed_area(source.m_bed_area),
    m_max_move(source.m_max_move),
    m_max_move_slow(source.m_max_move_slow),
    m_min_resolution(source.m_min_resolution),
    m_current_outline_idx(0),
    m_current_min_xy_dist(source.m_current_min_xy_dist),
    m_current_min_xy_dist_delta(source.m_current_min_xy_dist_delta),
    m_support_rests_on_model(false),
#ifdef SLIC3R_TREESUPPORTS_PROGRESS
    m_progress_multiplier(0.), m_progress_offset(0.),
#endif
    m_increase_until_radius(source.m_increase_until_radius),
    m_ignorable_radii(source.m_ignorable_radii),
    m_radius_0(source.m_radius_0),
    m_reversed_source(&source),
    m_reversed_throw_on_cancel(std::move(throw_on_cancel)),
    m_seam_layer(seam_layer),
    m_max_seam_radius(max_seam_radius)
{
    assert(seam_layer >= 0);
    assert(max_seam_radius >= 0);
    assert(source.m_reversed_source == nullptr);
    // Include the exterior ring so shrinking avoidance cannot invent a route
    // at the bed boundary, as in the existing painted-base route domain.
    Polygons seam_domain = offset(m_bed_area, scaled<float>(1000.), jtMiter, 1.2);
    m_seam_excluded_centers = diff(seam_domain, seam_centers);
    if (max_seam_radius != std::numeric_limits<coord_t>::max())
        m_seam_blocked_domain = std::move(seam_domain);
}

void TreeModelVolumes::precalculate(const PrintObject& print_object, const coord_t max_layer, std::function<void()> throw_on_cancel)
{
    assert(m_reversed_source == nullptr);
    auto t_start = std::chrono::high_resolution_clock::now();
    m_precalculated = true;

    // Get the config corresponding to one mesh that is in the current group. Which one has to be irrelevant.
    // Not the prettiest way to do this, but it ensures some calculations that may be a bit more complex
    // like inital layer diameter are only done in once.
    TreeSupportSettings config(m_layer_outlines[m_current_outline_idx].first, print_object.slicing_parameters());

    {
        // calculate which radius each layer in the tip may have.
        std::vector<coord_t> possible_tip_radiis;
        for (size_t distance_to_top = 0; distance_to_top <= config.tip_layers; ++ distance_to_top) {
            possible_tip_radiis.emplace_back(ceilRadius(config.getRadius(distance_to_top)));
            possible_tip_radiis.emplace_back(ceilRadius(config.getRadius(distance_to_top) + m_current_min_xy_dist_delta));
        }
        sort_remove_duplicates(possible_tip_radiis);
        // It theoretically may happen in the tip, that the radius can change so much in-between 2 layers, 
        // that a ceil step is skipped (as in there is a radius r so that ceilRadius(radius(dtt))<ceilRadius(r)<ceilRadius(radius(dtt+1))). 
        // As such a radius will not reasonable happen in the tree and it will most likely not be requested,
        // there is no need to calculate them. So just skip these.
        for (coord_t radius_eval = m_radius_0; radius_eval <= config.branch_radius; radius_eval = ceilRadius(radius_eval + 1))
            if (! std::binary_search(possible_tip_radiis.begin(), possible_tip_radiis.end(), radius_eval))
                m_ignorable_radii.emplace_back(radius_eval);
    }

    if (throw_on_cancel)
        throw_on_cancel();

    // it may seem that the required avoidance can be of a smaller radius when going to model (no initial layer diameter for to model branches)
    // but as for every branch going towards the bp, the to model avoidance is required to check for possible merges with to model branches, this assumption is in-fact wrong.
    std::unordered_map<coord_t, LayerIndex> radius_until_layer;
    // while it is possible to calculate, up to which layer the avoidance should be calculated, this simulation is easier to understand, and does not need to be adjusted if something of the radius calculation is changed.
    // Overhead with an assumed worst case of 6600 layers was about 2ms
    for (LayerIndex distance_to_top = 0; distance_to_top <= max_layer; ++ distance_to_top) {
        const LayerIndex current_layer = max_layer - distance_to_top;
        auto update_radius_until_layer = [&radius_until_layer, current_layer](coord_t r) {
            auto it = radius_until_layer.find(r);
            if (it == radius_until_layer.end())
                radius_until_layer.emplace_hint(it, r, current_layer);
        };
        // regular radius
        update_radius_until_layer(ceilRadius(config.getRadius(distance_to_top, 0) + m_current_min_xy_dist_delta));
        // the maximum radius that the radius with the min_xy_dist can achieve
        update_radius_until_layer(ceilRadius(config.getRadius(distance_to_top, 0)));
        update_radius_until_layer(ceilRadius(config.recommendedMinRadius(current_layer) + m_current_min_xy_dist_delta));
    }

    if (throw_on_cancel)
        throw_on_cancel();

    // Copy to deque to use in parallel for later.
    std::vector<RadiusLayerPair> relevant_avoidance_radiis{ radius_until_layer.begin(), radius_until_layer.end() };

    // Append additional radiis needed for collision.
    // To calculate collision holefree for every radius, the collision of radius m_increase_until_radius will be required.
    radius_until_layer[ceilRadius(m_increase_until_radius + m_current_min_xy_dist_delta)] = max_layer;
    // Collision for radius 0 needs to be calculated everywhere, as it will be used to ensure valid xy_distance in drawAreas.
    radius_until_layer[0] = max_layer;
    if (m_current_min_xy_dist_delta != 0)
        radius_until_layer[m_current_min_xy_dist_delta] = max_layer;

    // Now that required_avoidance_limit contains the maximum of ild and regular required radius just copy.
    std::vector<RadiusLayerPair> relevant_collision_radiis{ radius_until_layer.begin(), radius_until_layer.end() };

    // Calculate the relevant collisions
    calculateCollision(relevant_collision_radiis, throw_on_cancel);

    // calculate a separate Collisions with all holes removed. These are relevant for some avoidances that try to avoid holes (called safe)
    std::vector<RadiusLayerPair> relevant_hole_collision_radiis;
    for (RadiusLayerPair key : relevant_avoidance_radiis)
        if (key.first < m_increase_until_radius + m_current_min_xy_dist_delta)
            relevant_hole_collision_radiis.emplace_back(key);

    // Calculate collisions without holes, built from regular collision
    calculateCollisionHolefree(relevant_hole_collision_radiis, throw_on_cancel);
    // Let placables be calculated from calculateAvoidance() for better parallelization.
    if (m_support_rests_on_model)
        calculatePlaceables(relevant_avoidance_radiis, throw_on_cancel);

    auto t_coll = std::chrono::high_resolution_clock::now();

    // Calculate the relevant avoidances in parallel as far as possible
    {
        tbb::task_group task_group;
        task_group.run([this, relevant_avoidance_radiis, throw_on_cancel]{ calculateAvoidance(relevant_avoidance_radiis, true, m_support_rests_on_model, throw_on_cancel); });
        task_group.run([this, relevant_avoidance_radiis, throw_on_cancel]{ calculateWallRestrictions(relevant_avoidance_radiis, throw_on_cancel); });
        task_group.wait();
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    auto dur_col = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_coll - t_start).count();
    auto dur_avo = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_coll).count();

//    m_precalculated = true;
    BOOST_LOG_TRIVIAL(info) << "Precalculating collision took" << dur_col << " ms. Precalculating avoidance took " << dur_avo << " ms.";

#if 0
    // Paint caches into SVGs:
    auto paint_cache_into_SVGs = [this](const RadiusLayerPolygonCache &cache, std::string_view name) {
        const std::vector<std::pair<RadiusLayerPair, std::reference_wrapper<const Polygons>>> sorted = cache.sorted();
        static constexpr const std::string_view colors[] = {
            "red", "green", "blue", "magenta", "orange"
        };
        static constexpr const size_t num_colors = sizeof(colors) / sizeof(colors[0]);
        for (size_t i = 0; i < sorted.size();) {
            // Find range of cache items with the same layer index.
            size_t j = i;
            for (++ j; j < sorted.size() && sorted[i].first.second == sorted[j].first.second; ++ j) ;
            // Collect expolygons in reverse order (largest to smallest).
            std::vector<std::pair<Slic3r::ExPolygons, SVG::ExPolygonAttributes>> expolygons_with_attributes;
            for (int k = int(j - 1); k >= int(i); -- k) {
                std::string legend = format("radius-%1%", unscaled<float>(sorted[k].first.first));
                expolygons_with_attributes.push_back({ union_ex(sorted[k].second), SVG::ExPolygonAttributes(legend, std::string(colors[(k - int(i)) % num_colors]), 1.) });
                SVG::export_expolygons(debug_out_path("treesupport_cache-%s-%d-%s.svg", name.data(), sorted[i].first.second, legend.c_str()), { expolygons_with_attributes.back() });
            }
            // Render the range of per radius collision polygons into a common SVG.
            SVG::export_expolygons(debug_out_path("treesupport_cache-%s-%d.svg", name.data(), sorted[i].first.second), expolygons_with_attributes);
            i = j;
        }
    };
    paint_cache_into_SVGs(m_collision_cache,                    "collision_cache");
    paint_cache_into_SVGs(m_collision_cache_holefree,           "collision_cache_holefree");
    paint_cache_into_SVGs(m_avoidance_cache,                    "avoidance_cache");
    paint_cache_into_SVGs(m_avoidance_cache_slow,               "avoidance_cache_slow");
    paint_cache_into_SVGs(m_avoidance_cache_to_model,           "avoidance_cache_to_model");
    paint_cache_into_SVGs(m_avoidance_cache_to_model_slow,      "avoidance_cache_to_model_slow");
    paint_cache_into_SVGs(m_placeable_areas_cache,              "placable_areas_cache");
    paint_cache_into_SVGs(m_avoidance_cache_holefree,           "avoidance_cache_holefree");
    paint_cache_into_SVGs(m_avoidance_cache_holefree_to_model,  "avoidance_cache_holefree_to_model");
    paint_cache_into_SVGs(m_wall_restrictions_cache,            "wall_restrictions_cache");
    paint_cache_into_SVGs(m_wall_restrictions_cache_min,        "wall_restrictions_cache_min");
#endif
}

const Polygons& TreeModelVolumes::getCollision(const coord_t orig_radius, LayerIndex layer_idx, bool min_xy_dist) const
{
    if (m_reversed_source) {
        assert(layer_idx >= 0 && layer_idx <= m_seam_layer);
        const LayerIndex physical_layer = m_seam_layer - layer_idx;
        if (m_reversed_throw_on_cancel) {
            const coord_t source_radius = m_reversed_source->ceilRadius(orig_radius, min_xy_dist);
            if (!m_reversed_source->m_collision_cache.getArea({source_radius, physical_layer})) {
                m_reversed_source->compute_once(LazyCache::Collision, source_radius,
                    [&] { return m_reversed_source->m_collision_cache.getArea({source_radius, physical_layer}).has_value(); },
                    [&] { const_cast<TreeModelVolumes*>(m_reversed_source)->calculateCollision(source_radius, physical_layer, m_reversed_throw_on_cancel); });
                m_reversed_throw_on_cancel();
            }
        }
        const Polygons &physical = m_reversed_source->getCollision(orig_radius, physical_layer, min_xy_dist);
        // Support blocker slices are object layers, while collision layers also
        // include raft layers. Keep their exact physical layer when reversing.
        const LayerIndex blocker_layer = physical_layer - LayerIndex(m_reversed_source->m_raft_layers.size());
        if (blocker_layer < 0 || blocker_layer >= LayerIndex(m_reversed_source->m_anti_overhang.size()) ||
            m_reversed_source->m_anti_overhang[blocker_layer].empty())
            return physical;
        const coord_t radius = ceilRadius(orig_radius, min_xy_dist);
        if (auto cached = m_collision_cache.getArea({radius, layer_idx}); cached)
            return cached->get();
        // The forward generator's existing last-mesh condition may omit
        // blockers. Correct this only in the new view to preserve bed trees.
        if (m_reversed_throw_on_cancel)
            m_reversed_throw_on_cancel();
        const Polygons blocked = offset(union_ex(m_reversed_source->m_anti_overhang[blocker_layer]), radius, jtMiter, 1.2);
        const_cast<TreeModelVolumes*>(this)->m_collision_cache.insert(
            std::vector<std::pair<RadiusLayerPair, Polygons>>{{{radius, layer_idx}, union_(physical, blocked)}});
        return m_collision_cache.getArea({radius, layer_idx})->get();
    }
    const coord_t radius = this->ceilRadius(orig_radius, min_xy_dist);
    if (std::optional<std::reference_wrapper<const Polygons>> result = m_collision_cache.getArea({ radius, layer_idx }); result)
        return (*result).get();
    if (m_precalculated) {
        BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate collision at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
        tree_supports_show_error("Not precalculated Collision requested."sv, false);
    }
    compute_once(LazyCache::Collision, radius,
        [&] { return m_collision_cache.getArea({ radius, layer_idx }).has_value(); },
        [&] { const_cast<TreeModelVolumes*>(this)->calculateCollision(radius, layer_idx, []{}); });
    return getCollision(orig_radius, layer_idx, min_xy_dist);
}

// Get a collision area at a given layer for a radius that is a lower or equial to the key radius.
// It is expected that the collision area is precalculated for a given layer at least for the radius zero.
// Used for pushing tree supports away from object during the final Organic optimization step.
std::optional<std::pair<coord_t, std::reference_wrapper<const Polygons>>> TreeModelVolumes::get_collision_lower_bound_area(LayerIndex layer_id, coord_t max_radius) const
{
    if (m_reversed_source) {
        assert(layer_id >= 0 && layer_id <= m_seam_layer);
        // Populate the zero-radius view as well so a smoothing query never
        // omits a blocker merely because no routing radius was requested yet.
        getCollision(0, layer_id, true);
        if (auto local = m_collision_cache.get_lower_bound_area({max_radius, layer_id}); local)
            return local;
        return m_reversed_source->get_collision_lower_bound_area(m_seam_layer - layer_id, max_radius);
    }
    return m_collision_cache.get_lower_bound_area({ max_radius, layer_id });
}

// Private. Only called internally by calculateAvoidance() and calculateAvoidanceToModel(), radius is already snapped to grid.
const Polygons& TreeModelVolumes::getCollisionHolefree(coord_t radius, LayerIndex layer_idx) const
{
    assert(radius == this->ceilRadius(radius));
    assert(radius < m_increase_until_radius + m_current_min_xy_dist_delta);
    if (std::optional<std::reference_wrapper<const Polygons>> result = m_collision_cache_holefree.getArea({ radius, layer_idx }); result)
        return (*result).get();
    if (m_precalculated) {
        BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate collision holefree at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
        tree_supports_show_error("Not precalculated Holefree Collision requested."sv, false);
    }
    compute_once(LazyCache::CollisionHolefree, radius,
        [&] { return m_collision_cache_holefree.getArea({ radius, layer_idx }).has_value(); },
        [&] {
            if (m_reversed_source)
                const_cast<TreeModelVolumes*>(this)->calculateCollisionHolefree(
                    std::vector<RadiusLayerPair>{{radius, layer_idx}}, m_reversed_throw_on_cancel);
            else
                const_cast<TreeModelVolumes*>(this)->calculateCollisionHolefree({ radius, layer_idx });
        });
    // A nested TBB loop may return without filling its cache when a sibling
    // throws. Observe cancellation before recursively requesting that entry.
    if (m_reversed_source && m_reversed_throw_on_cancel)
        m_reversed_throw_on_cancel();
    return getCollisionHolefree(radius, layer_idx);
}

const Polygons& TreeModelVolumes::getAvoidance(const coord_t orig_radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist) const
{
    // Routing radii do not decrease towards the seam. An oversized branch
    // cannot reach that endpoint from any layer; let the existing solver retry
    // without radius growth. Check the physical request before rounding or
    // adding XY clearance, so an allowed cache neighbor cannot bypass the cap.
    if (m_reversed_source && orig_radius > m_max_seam_radius) {
        assert(layer_idx >= 0 && layer_idx <= m_seam_layer);
        if (m_reversed_throw_on_cancel)
            m_reversed_throw_on_cancel();
        return m_seam_blocked_domain;
    }
    return getAvoidanceImpl(orig_radius, layer_idx, type, to_model, min_xy_dist);
}

const Polygons& TreeModelVolumes::getAvoidanceImpl(const coord_t orig_radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist) const
{
    if (m_reversed_source) {
        assert(layer_idx >= 0 && layer_idx <= m_seam_layer);
        // Both route types reach the seam. Intermediate physical surfaces must
        // never become alternate endpoints in the reversed traversal.
        to_model = false;
        if (layer_idx == 0) {
            const coord_t radius = ceilRadius(orig_radius, min_xy_dist);
            if (auto cached = m_seam_avoidance_cache.getArea({radius, 0}); cached)
                return cached->get();
            const_cast<TreeModelVolumes*>(this)->m_seam_avoidance_cache.insert(
                std::vector<std::pair<RadiusLayerPair, Polygons>>{{{radius, 0},
                    union_(getCollision(orig_radius, 0, min_xy_dist), m_seam_excluded_centers)}});
            return m_seam_avoidance_cache.getArea({radius, 0})->get();
        }
    }
    if (layer_idx == 0) {
        if (to_model && m_painted_bases_only)
            return m_painted_route_domain;
        return getCollision(orig_radius, layer_idx, min_xy_dist);
    }

    const coord_t radius = this->ceilRadius(orig_radius, min_xy_dist);
    if (type == AvoidanceType::FastSafe && radius >= m_increase_until_radius + m_current_min_xy_dist_delta)
        // no holes anymore by definition at this request
        type = AvoidanceType::Fast;

    if (std::optional<std::reference_wrapper<const Polygons>> result = 
            this->avoidance_cache(type, to_model).getArea({ radius, layer_idx }); 
        result)
        return (*result).get();

    if (m_precalculated) {
        if (to_model) {
            BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate Avoidance to model at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
            tree_supports_show_error("Not precalculated Avoidance(to model) requested."sv, false);
        } else {
            BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate Avoidance at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
            tree_supports_show_error("Not precalculated Avoidance(to buildplate) requested."sv, false);
        }
    }
    compute_once(to_model ? LazyCache::AvoidanceToModel : LazyCache::AvoidanceToBuildPlate, radius,
        [&] { return this->avoidance_cache(type, to_model).getArea({ radius, layer_idx }).has_value(); },
        [&] {
            if (m_reversed_source)
                const_cast<TreeModelVolumes*>(this)->calculateAvoidance(
                    std::vector<RadiusLayerPair>{{radius, layer_idx}}, !to_model, to_model, m_reversed_throw_on_cancel);
            else
                const_cast<TreeModelVolumes*>(this)->calculateAvoidance({ radius, layer_idx }, ! to_model, to_model);
        });
    if (m_reversed_source && m_reversed_throw_on_cancel)
        m_reversed_throw_on_cancel();
    // Retrive failed and correct result was calculated. Now it has to be retrived.
    return getAvoidanceImpl(orig_radius, layer_idx, type, to_model, min_xy_dist);
}

const Polygons& TreeModelVolumes::getPlaceableAreas(const coord_t orig_radius, LayerIndex layer_idx, std::function<void()> throw_on_cancel) const
{
    if (m_reversed_source) {
        static const Polygons no_intermediate_contacts;
        return no_intermediate_contacts;
    }
    const coord_t radius = ceilRadius(orig_radius);
    if (std::optional<std::reference_wrapper<const Polygons>> result = m_placeable_areas_cache.getArea({ radius, layer_idx }); result)
        return (*result).get();
    if (m_precalculated) {
        BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate Placeable Areas at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
        tree_supports_show_error("Not precalculated Placeable areas requested."sv, false);
    }	
    if (orig_radius == 0) {
        // Collision generation also populates radius-zero placeables. Return
        // that result, never the collision polygons used to compute it.
        this->getCollision(0, layer_idx, true);
        if (auto result = m_placeable_areas_cache.getArea({0, layer_idx}); result)
            return result->get();
        static const Polygons no_model_contacts;
        return no_model_contacts;
    }
    compute_once(LazyCache::Placeables, radius,
        [&] { return m_placeable_areas_cache.getArea({ radius, layer_idx }).has_value(); },
        [&] { const_cast<TreeModelVolumes*>(this)->calculatePlaceables(radius, layer_idx, throw_on_cancel); });
    return getPlaceableAreas(orig_radius, layer_idx, throw_on_cancel);
}

const Polygons& TreeModelVolumes::getWallRestriction(const coord_t orig_radius, LayerIndex layer_idx, bool min_xy_dist) const
{
    assert(layer_idx > 0);
    if (layer_idx == 0) 
        // Should never be requested as there will be no going below layer 0 ..., 
        // but just to be sure some semi-sane catch. Alternative would be empty Polygon.
        return getCollision(orig_radius, layer_idx, min_xy_dist);

    min_xy_dist &= m_current_min_xy_dist_delta > 0;

    const coord_t radius = ceilRadius(orig_radius);
    if (std::optional<std::reference_wrapper<const Polygons>> result = 
        (min_xy_dist ? m_wall_restrictions_cache_min : m_wall_restrictions_cache).getArea({ radius, layer_idx });
        result)
        return (*result).get();
    if (m_precalculated) {
        BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate Wall restricions at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
        tree_supports_show_error(
            min_xy_dist ? 
                "Not precalculated Wall restriction of minimum xy distance requested )." :
                "Not precalculated Wall restriction requested )."sv
            , false);
    }
    compute_once(LazyCache::WallRestrictions, radius,
        [&] { return (min_xy_dist ? m_wall_restrictions_cache_min : m_wall_restrictions_cache).getArea({ radius, layer_idx }).has_value(); },
        [&] {
            if (m_reversed_source)
                const_cast<TreeModelVolumes*>(this)->calculateWallRestrictions(
                    std::vector<RadiusLayerPair>{{radius, layer_idx}}, m_reversed_throw_on_cancel);
            else
                const_cast<TreeModelVolumes*>(this)->calculateWallRestrictions({ radius, layer_idx });
        });
    if (m_reversed_source && m_reversed_throw_on_cancel)
        m_reversed_throw_on_cancel();
    return getWallRestriction(orig_radius, layer_idx, min_xy_dist); // Retrieve failed and correct result was calculated. Now it has to be retrieved.
}

void TreeModelVolumes::calculateCollision(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)
{
    tbb::parallel_for(tbb::blocked_range<size_t>(0, keys.size()),
        [&](const tbb::blocked_range<size_t> &range) {
        for (size_t ikey = range.begin(); ikey != range.end(); ++ ikey) {
            const LayerIndex radius        = keys[ikey].first;
            const size_t     max_layer_idx = keys[ikey].second;
            // recursive call to parallel_for.
            calculateCollision(radius, max_layer_idx, throw_on_cancel);
        }
    });
}

void TreeModelVolumes::calculateCollision(const coord_t radius, const LayerIndex max_layer_idx, std::function<void()> throw_on_cancel)
{
//    assert(radius == this->ceilRadius(radius));

    // Process the outlines from least layers to most layers so that the final union will run over the longest vector.
    std::vector<size_t> layer_outline_indices(m_layer_outlines.size(), 0);
    std::iota(layer_outline_indices.begin(), layer_outline_indices.end(), 0);
    std::sort(layer_outline_indices.begin(), layer_outline_indices.end(),
        [this](size_t i, size_t j) { return m_layer_outlines[i].second.size() < m_layer_outlines[j].second.size(); });

    LayerPolygonCache           data;
    // Another lazy requester may have populated this radius since the miss.
    const LayerIndex first_missing_layer = m_collision_cache.getMaxCalculatedLayer(radius) + 1;
    if (first_missing_layer > max_layer_idx)
        return;
    data.allocate(first_missing_layer, max_layer_idx + 1);

    const bool                  calculate_placable = m_support_rests_on_model && radius == 0;
    LayerPolygonCache           data_placeable;
    if (calculate_placable)
        data_placeable.allocate(data.begin(), data.end());

    for (size_t outline_idx : layer_outline_indices)
        if (const std::vector<Polygons> &outlines = m_layer_outlines[outline_idx].second; ! outlines.empty()) {
            const TreeSupportMeshGroupSettings  &settings = m_layer_outlines[outline_idx].first;
            const coord_t       layer_height              = settings.layer_height;
            const int           z_distance_bottom_layers  = int(round(double(settings.support_bottom_distance) / double(layer_height)));
            const int           z_distance_top_layers     = int(round(double(settings.support_top_distance) / double(layer_height)));
            const coord_t       xy_distance               = outline_idx == m_current_outline_idx ? m_current_min_xy_dist : 
                // technically this causes collision for the normal xy_distance to be larger by m_current_min_xy_dist_delta for all 
                // not currently processing meshes as this delta will be added at request time.
                // avoiding this would require saving each collision for each outline_idx separately.
                // and later for each avoidance... But avoidance calculation has to be for the whole scene and can NOT be done for each outline_idx separately and combined later.
                // so avoiding this inaccuracy seems infeasible as it would require 2x the avoidance calculations => 0.5x the performance.
                //FIXME support_xy_distance is not corrected for "soluble" flag, see TreeSupportSettings constructor.
                settings.support_xy_distance;

            // 1) Calculate offsets of collision areas in parallel.
            LayerPolygonCache collision_areas_offsetted;
            collision_areas_offsetted.allocate(
                std::max<LayerIndex>(0, data.begin() - z_distance_bottom_layers),
                std::min<LayerIndex>(outlines.size(), data.end() + z_distance_top_layers));
            // union_ex(machine border + outline) of each layer depends on neither the radius nor the layer it is
            // summed into: 1) keeps it for the top range of 2), which unions the same polygons again.
            const LayerIndex        unions_begin = collision_areas_offsetted.begin();
            std::vector<ExPolygons> collision_unions(size_t(std::max<LayerIndex>(0, collision_areas_offsetted.end() - unions_begin)));
            tbb::parallel_for(tbb::blocked_range<LayerIndex>(collision_areas_offsetted.begin(), collision_areas_offsetted.end()),
                [&outlines, &machine_border = std::as_const(m_machine_border), offset_value = radius + xy_distance, &collision_areas_offsetted, &throw_on_cancel,
                 &collision_unions, unions_begin, first_top = data.begin() + 1, z_distance_top_layers]
                (const tbb::blocked_range<LayerIndex> &range) {
                for (LayerIndex layer_idx = range.begin(); layer_idx != range.end(); ++ layer_idx) {
                    Polygons collision_areas = machine_border;
                    append(collision_areas, outlines[layer_idx]);
                    ExPolygons &collision_union = collision_unions[size_t(layer_idx - unions_begin)];
                    if (offset_value != 0 || (z_distance_top_layers > 0 && layer_idx >= first_top))
                        collision_union = union_ex(collision_areas);
                    // jtRound is not needed here, as the overshoot can not cause errors in the algorithm, because no assumptions are made about the model.
                    // if a key does not exist when it is accessed it is added!
                    collision_areas_offsetted[layer_idx] = offset_value == 0 ?
                            union_(collision_areas) :
                            offset(collision_union, offset_value, ClipperLib::jtMiter, 1.2);
                    if(throw_on_cancel)
                        throw_on_cancel();
                }
            });

            // 2) Sum over top / bottom ranges.
            const bool processing_last_mesh = outline_idx == layer_outline_indices.size();
            tbb::parallel_for(tbb::blocked_range<LayerIndex>(data.begin(), data.end()),
                [&collision_areas_offsetted, &outlines, &collision_unions, unions_begin, &anti_overhang = m_anti_overhang, radius,
                    xy_distance, z_distance_bottom_layers, z_distance_top_layers, min_resolution = m_min_resolution, &data, processing_last_mesh, &throw_on_cancel]
            (const tbb::blocked_range<LayerIndex>& range) {
                    for (LayerIndex layer_idx = range.begin(); layer_idx != range.end(); ++layer_idx) {
                        Polygons collisions;
                        for (int i = - z_distance_bottom_layers; i <= 0; ++ i)
                            if (int j = layer_idx + i; collision_areas_offsetted.has(j))
                                append(collisions, collision_areas_offsetted[j]);
                        for (int i = 1; i <= z_distance_top_layers; ++ i)
                            if (int j = layer_idx + i; j < int(outlines.size())) {
                                // union_ex(machine border + outlines[j]), kept by 1).
                                const ExPolygons &collision_areas_original = collision_unions[size_t(j - unions_begin)];

                                // If just the collision (including the xy distance) of the layers above is accumulated, it leads to the
                                // following issue:
                                // Example: assuming the z distance is 2 layer
                                // + = xy_distance
                                // - = model
                                // o = overhang of the area two layers above that should result in tips on this layer
                                //
                                //  +-----+
                                //   +-----+
                                //    +-----+
                                //   o +-----+
                                // If just the collision above is accumulated the overhang will get overwritten by the xy_distance of the
                                // layer below the overhang...
                                //
                                // This only causes issues if the overhang area is thinner than xy_distance
                                // Just accumulating areas of the model above without the xy distance is also problematic, as then support
                                // may get closer to the model (on the diagonal downwards) than the user intended. Example (s = support):
                                //  +-----+
                                //   +-----+
                                //   +-----+
                                //   s+-----+

                                // technically the calculation below is off by one layer, as the actual distance between plastic one layer
                                // down is 0 not layer height, as this layer is filled with said plastic. But otherwise a part of the
                                // overhang that is expected to be supported is overwritten by the remaining part of the xy distance of the
                                // layer below the to be supported area.
                                coord_t required_range_x =
                                    (xy_distance - ((i - (z_distance_top_layers == 1 ? 0.5 : 0)) * xy_distance / z_distance_top_layers)); 
                                    // the conditional -0.5 ensures that plastic can never touch on the diagonal
                                    // downward when the z_distance_top_layers = 1. It is assumed to be better to
                                    // not support an overhang<90 degree than to risk fusing to it.
                                append(collisions, offset(collision_areas_original, radius + required_range_x, ClipperLib::jtMiter, 1.2));
                        }
                        collisions = processing_last_mesh && layer_idx < int(anti_overhang.size()) ? 
                                union_(collisions, offset(union_ex(anti_overhang[layer_idx]), radius, ClipperLib::jtMiter, 1.2)) : 
                                union_(collisions);
                        auto &dst = data[layer_idx];
                        if (processing_last_mesh) {
                            if (! dst.empty())
                                collisions = union_(collisions, dst);
                            dst = polygons_simplify(collisions, min_resolution);
                        } else
                            append(dst, std::move(collisions));
                        if (throw_on_cancel)
                            throw_on_cancel();
                    }
                });

            // 3) Optionally calculate placables.
            if (calculate_placable) {
                // Calculating both the collision areas and placable areas.
                tbb::parallel_for(tbb::blocked_range<LayerIndex>(std::max(z_distance_bottom_layers + 1, data.begin()), data.end()),
                    [&collision_areas_offsetted, &outlines, &anti_overhang = m_anti_overhang, processing_last_mesh,
                     min_resolution = m_min_resolution, z_distance_bottom_layers, xy_distance, &data_placeable, &throw_on_cancel,
                     painted_only = m_painted_bases_only, &painted_bases = m_painted_base_areas]
                (const tbb::blocked_range<LayerIndex>& range) {
                    for (LayerIndex layer_idx = range.begin(); layer_idx != range.end(); ++ layer_idx) {
                        LayerIndex layer_idx_below = layer_idx - z_distance_bottom_layers - 1;
                        assert(layer_idx_below >= 0);
                        const Polygons &current = collision_areas_offsetted[layer_idx];
                        const Polygons &below   = outlines[layer_idx_below];
                        Polygons placable = diff(
                            // Inflate the surface to sit on by the separation distance to increase chance of a support being placed on a sloped surface.
                            offset(below, xy_distance), 
                            layer_idx_below < int(anti_overhang.size()) ? union_(current, anti_overhang[layer_idx_below]) : current);
                        if (painted_only) {
                            // Keep the actual painted surface. Unlike native
                            // placeables this is not enlarged by XY separation;
                            // nonzero radii are inset by the branch footprint.
                            placable = layer_idx_below < LayerIndex(painted_bases.size()) ?
                                intersection(placable, painted_bases[layer_idx_below]) : Polygons{};
                        }
                        auto &dst     = data_placeable[layer_idx];
                        if (processing_last_mesh) {
                            if (! dst.empty())
                                placable = union_(placable, dst);
                            dst = polygons_simplify(placable, min_resolution);
                        } else
                            append(dst, placable);
                        if (throw_on_cancel)
                            throw_on_cancel();
                    }
                });
            } else {
                // Calculating just the collision areas.
            }
        }
#ifdef SLIC3R_TREESUPPORTS_PROGRESS
    {
        std::lock_guard<std::mutex> critical_section(*m_critical_progress);
        if (m_precalculated && m_precalculation_progress < TREE_PROGRESS_PRECALC_COLL) {
            m_precalculation_progress += TREE_PROGRESS_PRECALC_COLL / keys.size();
            Progress::messageProgress(Progress::Stage::SUPPORT, m_precalculation_progress * m_progress_multiplier + m_progress_offset, TREE_PROGRESS_TOTAL);
        }
    }
#endif
    if (throw_on_cancel)
        throw_on_cancel();
    // Publish derived radius-zero contacts first: a concurrent lazy reader
    // that observes collision data must also be able to read its placeables.
    if (calculate_placable)
        m_placeable_areas_cache.insert(std::move(data_placeable), radius);
    m_collision_cache.insert(std::move(data), radius);
}

void TreeModelVolumes::calculateCollisionHolefree(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)
{
    LayerIndex max_layer = 0;
    for (long long unsigned int i = 0; i < keys.size(); i++)
        max_layer = std::max(max_layer, keys[i].second);

    tbb::parallel_for(tbb::blocked_range<LayerIndex>(0, max_layer + 1, keys.size()),
        [&](const tbb::blocked_range<LayerIndex> &range) {
        std::vector<std::pair<RadiusLayerPair, Polygons>> data;
        data.reserve(range.size() * keys.size());
        for (LayerIndex layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
            // The collision unioned below is the same for every key: union it once per layer.
            std::optional<ExPolygons> collision_union;
            for (RadiusLayerPair key : keys)
                if (layer_idx <= key.second && !m_collision_cache_holefree.getArea({key.first, layer_idx})) {
                    // Forward trees precompute this table in one batch. A
                    // reversed view requests it lazily while growing away from
                    // its seam, so an uncached layer must not recompute every
                    // preceding layer. Keep the same polygons and allow holes
                    // left by cancellation to be filled on a later request.
                    // Logically increase the collision by m_increase_until_radius
                    coord_t radius = key.first;
                    assert(radius == this->ceilRadius(radius));
                    assert(radius < m_increase_until_radius + m_current_min_xy_dist_delta);
                    coord_t increase_radius_ceil = ceilRadius(m_increase_until_radius, false) - radius;
                    assert(increase_radius_ceil > 0);
                    // this union is important as otherwise holes(in form of lines that will increase to holes in a later step) can get unioned onto the area.
                    if (! collision_union)
                        collision_union = union_ex(this->getCollision(m_increase_until_radius, layer_idx, false));
                    data.emplace_back(RadiusLayerPair(radius, layer_idx), polygons_simplify(
                        offset(*collision_union, 5 - increase_radius_ceil, ClipperLib::jtRound, m_min_resolution),
                        m_min_resolution));
                    if (throw_on_cancel)
                        throw_on_cancel();
                }
        }
        m_collision_cache_holefree.insert(std::move(data));
    });
}

void TreeModelVolumes::calculateAvoidance(const std::vector<RadiusLayerPair> &keys, bool to_build_plate, bool to_model, std::function<void()> throw_on_cancel)
{
    // For every RadiusLayer pair there are 3 avoidances that have to be calculated.
    // Prepare tasks for parallelization.
    struct AvoidanceTask {
        AvoidanceType   type;
        coord_t         radius;
        LayerIndex      max_required_layer;
        bool            to_model;
        LayerIndex      start_layer;

        bool slow()     const { return this->type == AvoidanceType::Slow; }
        bool holefree() const { return this->type == AvoidanceType::FastSafe; }
    };

    std::vector<AvoidanceTask> avoidance_tasks;
    avoidance_tasks.reserve((int(to_build_plate) + int(to_model)) * keys.size() * size_t(AvoidanceType::Count));

    for (int iter_idx = 0; iter_idx < 2 * int(keys.size()) * int(AvoidanceType::Count); ++ iter_idx) {
        AvoidanceTask task{
            AvoidanceType(iter_idx % int(AvoidanceType::Count)),
            keys[iter_idx / 6].first,  // radius
            keys[iter_idx / 6].second, // max_layer
            ((iter_idx / 3) & 1) != 0  // to_model
        };
        // Ensure start_layer is at least 1 as if no avoidance was calculated yet getMaxCalculatedLayer() returns -1.
        task.start_layer = std::max<LayerIndex>(1, 1 + avoidance_cache(task.type, task.to_model).getMaxCalculatedLayer(task.radius));
        if (task.start_layer > task.max_required_layer) {
            BOOST_LOG_TRIVIAL(debug) << "Calculation requested for value already calculated?";
            continue;
        }
        if ((task.to_model ? to_model : to_build_plate) &&
            (! task.holefree() || task.radius < m_increase_until_radius + m_current_min_xy_dist_delta))
            avoidance_tasks.emplace_back(task);
    }

    if(throw_on_cancel)
        throw_on_cancel();

    if (m_reversed_source) {
        // Slow and FastSafe consume the same hole-free collision table. Fill
        // its missing rows once before those tasks run in parallel, instead of
        // letting both repeatedly launch nested per-layer calculations. Warm
        // the shared physical collision source before parallel cache readers.
        std::vector<RadiusLayerPair> holefree_keys;
        for (const RadiusLayerPair &key : keys)
            if (key.first < m_increase_until_radius + m_current_min_xy_dist_delta)
                holefree_keys.push_back(key);
        if (!holefree_keys.empty()) {
            getCollision(m_increase_until_radius, 0, false);
            for (const RadiusLayerPair &key : holefree_keys)
                compute_once(LazyCache::CollisionHolefree, key.first,
                    [&] { return m_collision_cache_holefree.getArea(key).has_value(); },
                    [&] { calculateCollisionHolefree(std::vector<RadiusLayerPair>{ key }, throw_on_cancel); });
        }
    }

    tbb::parallel_for(tbb::blocked_range<size_t>(0, avoidance_tasks.size(), 1),
        [this, &avoidance_tasks, &throw_on_cancel](const tbb::blocked_range<size_t> &range) {
        for (size_t task_idx = range.begin(); task_idx < range.end(); ++ task_idx) {
            const AvoidanceTask &task = avoidance_tasks[task_idx];
            assert(! task.holefree() || task.radius < m_increase_until_radius + m_current_min_xy_dist_delta);
            if (task.to_model)
                // ensuring Placeableareas are calculated
                //FIXME pass throw_on_cancel
                getPlaceableAreas(task.radius, task.max_required_layer, throw_on_cancel);
            // The following loop propagating avoidance regions bottom up is inherently serial.
            const bool  collision_holefree = (task.slow() || task.holefree()) && task.radius < m_increase_until_radius + m_current_min_xy_dist_delta;
            const float max_move           = task.slow() ? m_max_move_slow : m_max_move;
            // Limiting the offset step so that unioning the shrunk latest_avoidance with the current layer collisions
            // will not create gaps in the resulting avoidance region letting a tree support branch tunneling through an object wall.
            float move_step      = 1.9 * std::max(task.radius, m_current_min_xy_dist);
            if (move_step < EPSILON) return;
            int   move_steps     = round_up_divide<int>(max_move, move_step);
            assert(move_steps > 0);
            float last_move_step = max_move - (move_steps - 1) * move_step;
            if (last_move_step < scaled<float>(0.05)) {
                //assert(move_steps > 1);
                if (move_steps > 1) {
                    // Avoid taking a very short last step, stretch the other steps a bit instead.
                    move_step = max_move / (-- move_steps);
                    last_move_step = move_step;
                }
            }
            // minDist as the delta was already added, also avoidance for layer 0 will return the collision.
            Polygons    latest_avoidance   = getAvoidanceImpl(task.radius, task.start_layer - 1, task.type, task.to_model, true);
            std::vector<std::pair<RadiusLayerPair, Polygons>> data;
            data.reserve(task.max_required_layer + 1 - task.start_layer);
            for (LayerIndex layer_idx = task.start_layer; layer_idx <= task.max_required_layer; ++ layer_idx) {
                // Merge current layer collisions with shrunk last_avoidance.
                const Polygons &current_layer_collisions = collision_holefree ? getCollisionHolefree(task.radius, layer_idx) : getCollision(task.radius, layer_idx, true);
                // For mildly steep branch angles only one step will be taken.
                for (int istep = 0; istep < move_steps; ++ istep)
                    latest_avoidance = union_(current_layer_collisions,
                        offset(latest_avoidance,
                            istep + 1 == move_steps ? - last_move_step : - move_step,
                            ClipperLib::jtRound, m_min_resolution));
                if (task.to_model)
                    latest_avoidance = diff(latest_avoidance, getPlaceableAreas(task.radius, layer_idx, throw_on_cancel));
                latest_avoidance = polygons_simplify(latest_avoidance, m_min_resolution);
                data.emplace_back(RadiusLayerPair{task.radius, layer_idx}, latest_avoidance);
                if (throw_on_cancel)
                    throw_on_cancel();
            }
#ifdef SLIC3R_TREESUPPORTS_PROGRESS
            {
                std::lock_guard<std::mutex> critical_section(*m_critical_progress);
                if (m_precalculated && m_precalculation_progress < TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_PRECALC_AVO) {
                    m_precalculation_progress += to_model ? 
                        0.4 * TREE_PROGRESS_PRECALC_AVO / (keys.size() * 3) :
                        m_support_rests_on_model ? 0.4 : 1 * TREE_PROGRESS_PRECALC_AVO / (keys.size() * 3);
                    Progress::messageProgress(Progress::Stage::SUPPORT, m_precalculation_progress * m_progress_multiplier + m_progress_offset, TREE_PROGRESS_TOTAL);
                }
            }
#endif
            avoidance_cache(task.type, task.to_model).insert(std::move(data));
        }
    });
}


void TreeModelVolumes::calculatePlaceables(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)
{
    if (!keys.empty()) {
        LayerIndex max_layer = 0;
        for (const RadiusLayerPair &key : keys)
            max_layer = std::max(max_layer, key.second);
        getPlaceableAreas(0, max_layer, throw_on_cancel);
    }
    tbb::parallel_for(tbb::blocked_range<size_t>(0, keys.size()),
        [&, keys](const tbb::blocked_range<size_t>& range) {
            for (size_t key_idx = range.begin(); key_idx < range.end(); ++ key_idx)
                this->calculatePlaceables(keys[key_idx].first, keys[key_idx].second, throw_on_cancel);
        });
}

void TreeModelVolumes::calculatePlaceables(const coord_t radius, const LayerIndex max_required_layer, std::function<void()> throw_on_cancel)
{
    // Populate the shared zero-radius source once before parallel offsetting.
    // Otherwise lazy requests for different layers race to extend that cache.
    getPlaceableAreas(0, max_required_layer, throw_on_cancel);
    LayerIndex start_layer = 1 + m_placeable_areas_cache.getMaxCalculatedLayer(radius);
    if (start_layer > max_required_layer) {
        BOOST_LOG_TRIVIAL(debug) << "Requested calculation for value already calculated ?";
        return;
    }

    std::vector<Polygons> data(max_required_layer + 1 - start_layer, Polygons{});

    if (start_layer == 0)
        data[0] = diff(m_machine_border, getCollision(radius, 0, true));

    tbb::parallel_for(tbb::blocked_range<LayerIndex>(std::max(1, start_layer), max_required_layer + 1),
        [this, &data, radius, start_layer, &throw_on_cancel](const tbb::blocked_range<LayerIndex>& range) {
            for (LayerIndex layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                data[layer_idx - start_layer] = m_painted_bases_only ?
                    // Organic may draw a branch up to this minimum radius
                    // even while its collision radius is still smaller, and a
                    // branch keeping the regular XY distance lands with the
                    // difference to the minimum one added. Its landing is checked
                    // at that radius rounded up to the cache grid, so route with
                    // the widest such foot as well.
                    calculatePaintedPlaceables(std::max(radius, ceilRadius(m_increase_until_radius + m_current_min_xy_dist_delta)), layer_idx) :
                    offset(
                        union_ex(getPlaceableAreas(0, layer_idx, throw_on_cancel)),
                        // As a placeable area is calculated by (collision of the layer below) - (collision of the current layer) and the collision is offset by xy_distance,
                        // it can happen that a small line is considered a flat area to place something onto, even though it is mostly
                        // xy_distance that cant support it. Making the area smaller by xy_distance fixes this.
                        - (radius + m_current_min_xy_dist + m_current_min_xy_dist_delta),
                        jtMiter, 1.2);
                if(throw_on_cancel)
                    throw_on_cancel();
            }
        });
#ifdef SLIC3R_TREESUPPORTS_PROGRESS
    {
        std::lock_guard<std::mutex> critical_section(*m_critical_progress);
        if (m_precalculated && m_precalculation_progress < TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_PRECALC_AVO) {
            m_precalculation_progress += 0.2 * TREE_PROGRESS_PRECALC_AVO / (keys.size());
            Progress::messageProgress(Progress::Stage::SUPPORT, m_precalculation_progress * m_progress_multiplier + m_progress_offset, TREE_PROGRESS_TOTAL);
        }
    }
#endif
    m_placeable_areas_cache.insert(std::move(data), start_layer, radius);
}

Polygons TreeModelVolumes::calculatePaintedPlaceables(const coord_t footprint, const LayerIndex layer_idx) const
{
    // Like a tip under a curved overhang, which only has to touch that layer's
    // thin band of overhang, a branch ends where it touches paint at its base
    // layer. A sloped base has only a thin band per layer, so its footprint
    // rests on the bands of the lower layers as well. All of them must be paint.
    const LayerIndex base = layer_idx - m_painted_base_below;
    if (base < 0 || base >= LayerIndex(m_painted_base_areas.size()) || m_painted_base_areas[base].empty())
        return {};
    // Only where the paint is flatter than the support threshold angle can a
    // base stand on it, as only overhangs flatter than it carry tips.
    const Polygons band = intersection(m_painted_base_areas[base], calculateBaseBand(base, get_extents(m_painted_base_areas[base])));
    if (band.empty())
        return {};
    BoundingBox window = get_extents(band);
    window.offset(m_current_min_xy_dist + 2 * footprint + SCALED_EPSILON);
    // Where not even a branch of zero radius may stand at this layer: the routing
    // collision, which also keeps the bottom gap, and the blockers that native
    // placeable areas exclude.
    Polygons blocked = ClipperUtils::clip_clipper_polygons_with_subject_bbox(getCollision(0, layer_idx, true), window);
    if (base < LayerIndex(m_anti_overhang.size()))
        append(blocked, ClipperUtils::clip_clipper_polygons_with_subject_bbox(m_anti_overhang[base], window));
    // The branch cannot descend further where its collision one layer down,
    // which keeps the XY distance from the base layer's material, would overlap
    // that layer's paint.
    const Polygons contact = diff(offset(band, float(m_current_min_xy_dist), jtMiter, 1.2), blocked);
    if (contact.empty())
        return {};
    const Polygons paint = diff(ClipperUtils::clip_clipper_polygons_with_subject_bbox(m_painted_base_visible[base], window), blocked);
    return intersection(
        offset(union_ex(paint), - float(footprint), jtMiter, 1.2),
        offset(contact, float(footprint), jtMiter, 1.2));
}

const Polygons& TreeModelVolumes::getPaintedBaseSurface(const LayerIndex layer_idx) const
{
    static const Polygons none;
    const LayerIndex base = layer_idx - m_painted_base_below;
    return m_painted_bases_only && base >= 0 && base < LayerIndex(m_painted_base_visible.size()) ?
        m_painted_base_visible[base] : none;
}

Polygons TreeModelVolumes::getBaseLandingAreas(const LayerIndex layer_idx, std::function<void()> throw_on_cancel) const
{
    assert(m_reversed_source == nullptr);
    const Polygons &placeable = getPlaceableAreas(0, layer_idx, throw_on_cancel);
    if (placeable.empty())
        return {};
    const TreeSupportMeshGroupSettings &settings = m_layer_outlines[m_current_outline_idx].first;
    const std::vector<Polygons>        &outlines = m_layer_outlines[m_current_outline_idx].second;
    // The layer whose top the base is, under the bottom gap, as for the placeable areas.
    const LayerIndex below = layer_idx - LayerIndex(round(double(settings.support_bottom_distance) / double(settings.layer_height))) - 1;
    if (below < 0 || below + 1 >= LayerIndex(outlines.size()))
        return {};
    return intersection(placeable, calculateBaseBand(below, get_extents(placeable)));
}

Polygons TreeModelVolumes::getUnpaintedRestingAreas(const LayerIndex layer_idx, BoundingBox window) const
{
    assert(m_reversed_source == nullptr);
    if (! m_painted_bases_only)
        return {};
    const std::vector<Polygons> &outlines = m_layer_outlines[m_current_outline_idx].second;
    // The surface support at this layer rests on: the top of the layer under the
    // bottom gap, where the layer above it does not cover it.
    const LayerIndex below = layer_idx - m_painted_base_below;
    if (below < 0 || below + 1 >= LayerIndex(outlines.size()))
        return {};
    // The opening below reaches twice the crack in from the window's edge.
    window.offset(coord_t(2.f * m_painted_crack) + SCALED_EPSILON);
    const Polygons surface = diff(ClipperUtils::clip_clipper_polygons_with_subject_bbox(outlines[below], window),
                                  ClipperUtils::clip_clipper_polygons_with_subject_bbox(outlines[below + 1], window));
    if (surface.empty())
        return {};
    Polygons permitted = below < LayerIndex(m_painted_base_areas.size()) ?
        ClipperUtils::clip_clipper_polygons_with_subject_bbox(m_painted_base_areas[below], window) : Polygons{};
    if (below < LayerIndex(m_anti_overhang.size()))
        permitted = diff(permitted, m_anti_overhang[below]);
    // The paint comes from the exact mesh and the surface from simplified, XY
    // compensated slices: ignore unpainted slivers no wider than their cracks.
    return opening(diff(surface, permitted), m_painted_crack);
}

Polygons TreeModelVolumes::calculateBaseBand(const LayerIndex layer_idx, BoundingBox window) const
{
    // The mirror image of TreeSupport::detect_overhangs: the part of this layer
    // reaching beyond the layer above it by more than m_base_landing_step.
    const std::vector<Polygons> &outlines = m_layer_outlines[m_current_outline_idx].second;
    if (layer_idx < 0 || layer_idx + 1 >= LayerIndex(outlines.size()))
        return {};
    // Only the layer above within the step of the window can trim it.
    window.offset(m_base_landing_step + SCALED_EPSILON);
    return diff(
        ClipperUtils::clip_clipper_polygons_with_subject_bbox(outlines[layer_idx], window),
        offset(ClipperUtils::clip_clipper_polygons_with_subject_bbox(outlines[layer_idx + 1], window), float(m_base_landing_step), ClipperLib::jtSquare, 0.));
}

void TreeModelVolumes::calculateWallRestrictions(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)
{
    // Wall restrictions are mainly important when they represent actual walls that are printed, and not "just" the configured z_distance, because technically valid placement is no excuse for moving through a wall.
    // As they exist to prevent accidentially moving though a wall at high speed between layers like thie (x = wall,i = influence area,o= empty space,d = blocked area because of z distance) Assume maximum movement distance is two characters and maximum safe movement distance of one character

    /* Potential issue addressed by the wall restrictions: Influence area may lag through a wall
     *  layer z+1:iiiiiiiiiiioooo
     *  layer z+0:xxxxxiiiiiiiooo
     *  layer z-1:ooooixxxxxxxxxx
     */

    // The radius for the upper collission has to be 0 as otherwise one may not enter areas that may be forbidden on layer_idx but not one below (c = not an influence area even though it should ):
    /*
     *  layer z+1:xxxxxiiiiiioo
     *  layer z+0:dddddiiiiiiio
     *  layer z-1:dddocdddddddd
     */
    // Also there can not just the collision of the lower layer be used because if it were:
    /*
     *  layer z+1:dddddiiiiiiiiiio
     *  layer z+0:xxxxxddddddddddc
     *  layer z-1:dddddxxxxxxxxxxc
     */
    // Or of the upper layer be used because if it were:
    /*
     *  layer z+1:dddddiiiiiiiiiio
     *  layer z+0:xxxxcddddddddddc
     *  layer z-1:ddddcxxxxxxxxxxc
     */

    // And just offseting with maximum movement distance (and not in multiple steps) could cause:
    /*
     *  layer z:   oxiiiiiiiiioo
     *  layer z-1: ixiiiiiiiiiii
     */

    tbb::parallel_for(tbb::blocked_range<size_t>(0, keys.size()),
        [&, keys](const tbb::blocked_range<size_t> &range) {
        for (size_t key_idx = range.begin(); key_idx < range.end(); ++ key_idx) {
            const coord_t    radius             = keys[key_idx].first;
            const LayerIndex max_required_layer = keys[key_idx].second;
            const coord_t    min_layer_bottom   = std::max(1, m_wall_restrictions_cache.getMaxCalculatedLayer(radius));
            const size_t     buffer_size        = max_required_layer + 1 - min_layer_bottom;
            std::vector<Polygons> data(buffer_size, Polygons{});
            std::vector<Polygons> data_min;
            if (m_current_min_xy_dist_delta > 0)
                data_min.assign(buffer_size, Polygons{});
            tbb::parallel_for(tbb::blocked_range<LayerIndex>(min_layer_bottom, max_required_layer + 1),
                [this, &data, &data_min, radius, min_layer_bottom, &throw_on_cancel](const tbb::blocked_range<LayerIndex> &range) {
                for (LayerIndex layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                    data[layer_idx - min_layer_bottom] = polygons_simplify(
                        // radius contains m_current_min_xy_dist_delta already if required
                        intersection(getCollision(0, layer_idx, false), getCollision(radius, layer_idx - 1, true)),
                        m_min_resolution);
                    if (! data_min.empty())
                        data_min[layer_idx - min_layer_bottom] = 
                            polygons_simplify(
                                intersection(getCollision(0, layer_idx, true), getCollision(radius, layer_idx - 1, true)),
                                m_min_resolution);
                    if (throw_on_cancel)
                        throw_on_cancel();
                }
            });
            // An inherited cancellation may skip part of the nested loop.
            // Do not publish its unfilled slots as valid wall restrictions.
            if (m_reversed_source && throw_on_cancel)
                throw_on_cancel();
            m_wall_restrictions_cache.insert(std::move(data), min_layer_bottom, radius);
            if (! data_min.empty())
                m_wall_restrictions_cache_min.insert(std::move(data_min), min_layer_bottom, radius);
        }
    });
}

coord_t TreeModelVolumes::ceilRadius(const coord_t radius) const
{
    if (radius == 0)
        return 0;

    coord_t out = m_radius_0;
    if (radius > m_radius_0) {
        // generate SUPPORT_TREE_PRE_EXPONENTIAL_STEPS of radiis before starting to exponentially increase them.
        coord_t initial_radius_delta = SUPPORT_TREE_EXPONENTIAL_THRESHOLD - m_radius_0;
        auto ignore = [this](coord_t r) { return std::binary_search(m_ignorable_radii.begin(), m_ignorable_radii.end(), r); };
        if (initial_radius_delta > SUPPORT_TREE_COLLISION_RESOLUTION) {
            const int num_steps = round_up_divide(initial_radius_delta, SUPPORT_TREE_EXPONENTIAL_THRESHOLD);
            const int stepsize  = initial_radius_delta / num_steps;
            out += stepsize;
            for (auto step = 0; step < num_steps; ++ step) {
                if (out >= radius && ! ignore(out))
                    return out;
                out += stepsize;
            }
        } else
            out += SUPPORT_TREE_COLLISION_RESOLUTION;
        while (out < radius || ignore(out)) {
            assert(out * SUPPORT_TREE_EXPONENTIAL_FACTOR > out + SUPPORT_TREE_COLLISION_RESOLUTION);
            out = out * SUPPORT_TREE_EXPONENTIAL_FACTOR;
        }
    }
    return out;
}

void TreeModelVolumes::RadiusLayerPolygonCache::allocate_layers(size_t num_layers)
{
    if (num_layers > m_data.size()) {
        if (num_layers > m_data.capacity())
            m_data.reserve(next_highest_power_of_2(num_layers));
        m_data.resize(num_layers, {});
    }
}

// For debugging purposes, sorted by layer index, then by radius.
std::vector<std::pair<TreeModelVolumes::RadiusLayerPair, std::reference_wrapper<const Polygons>>> TreeModelVolumes::RadiusLayerPolygonCache::sorted() const
{
    std::vector<std::pair<RadiusLayerPair, std::reference_wrapper<const Polygons>>> out;
    for (auto &layer : m_data) {
        auto layer_idx = LayerIndex(&layer - m_data.data());
        for (auto &radius_polygons : layer)
            out.emplace_back(std::make_pair(radius_polygons.first, layer_idx), radius_polygons.second);
    }
    assert(std::is_sorted(out.begin(), out.end(), [](auto &l, auto &r){ return l.first.second < r.first.second || (l.first.second == r.first.second) && l.first.first < r.first.first; }));
    return out;
}

} // namespace Slic3r::TreeSupport3D
