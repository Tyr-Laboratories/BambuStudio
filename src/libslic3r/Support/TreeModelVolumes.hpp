// Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
// Original source of Thomas Rahm's tree supports:
// https://github.com/ThomasRahm/CuraEngine
//
// Original CuraEngine copyright:
// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef slic3r_TreeModelVolumes_hpp
#define slic3r_TreeModelVolumes_hpp

#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <boost/functional/hash.hpp>

#include "TreeSupportCommon.hpp"

#include "../Point.hpp"
#include "../Polygon.hpp"
#include "../PrintConfig.hpp"

namespace Slic3r
{

class BuildVolume;
class PrintObject;

namespace TreeSupport3D
{

static constexpr const double  SUPPORT_TREE_EXPONENTIAL_FACTOR = 1.5;
static constexpr const coord_t SUPPORT_TREE_EXPONENTIAL_THRESHOLD = scaled<coord_t>(1. * SUPPORT_TREE_EXPONENTIAL_FACTOR);
static constexpr const coord_t SUPPORT_TREE_COLLISION_RESOLUTION = scaled<coord_t>(0.5);
static constexpr const bool    SUPPORT_TREE_AVOID_SUPPORT_BLOCKER = true;

class TreeModelVolumes
{
public:
    TreeModelVolumes() = default;
    explicit TreeModelVolumes(const PrintObject &print_object, const BuildVolume &build_volume,
        coord_t max_move, coord_t max_move_slow, size_t current_mesh_idx, 
#ifdef SLIC3R_TREESUPPORTS_PROGRESS
        double progress_multiplier, 
        double progress_offset, 
#endif // SLIC3R_TREESUPPORTS_PROGRESS
        const std::vector<Polygons> &additional_excluded_areas = {});
    // Route from physical model foundations towards a seam using the ordinary
    // downward generator: physical_layer = seam_layer - virtual_layer.
    // seam_centers constrains branch centers, not their material footprint.
    // The source must outlive this view and remain at a stable address. Caches
    // are populated lazily; do not call the PrintObject precalculate() on a view.
    // The callback is polled by lazy calculations, including source cache misses.
    // max_seam_radius bounds requested routing radii before cache rounding;
    // physical collisions are unchanged. The default leaves radii unrestricted.
    TreeModelVolumes(const TreeModelVolumes &source, LayerIndex seam_layer, const Polygons &seam_centers,
                     std::function<void()> throw_on_cancel = {},
                     coord_t max_seam_radius = std::numeric_limits<coord_t>::max());
    TreeModelVolumes(TreeModelVolumes&&) = default;
    TreeModelVolumes& operator=(TreeModelVolumes&&) = default;

    TreeModelVolumes(const TreeModelVolumes&) = delete;
    TreeModelVolumes& operator=(const TreeModelVolumes&) = delete;

    void clear() { 
        this->clear_all_but_object_collision();
        m_collision_cache.clear();
    }
    void clear_all_but_object_collision() { 
        //m_collision_cache.clear_all_but_radius0();
        m_collision_cache_holefree.clear();
        m_avoidance_cache.clear();
        m_avoidance_cache_slow.clear();
        m_avoidance_cache_to_model.clear();
        m_avoidance_cache_to_model_slow.clear();
        m_placeable_areas_cache.clear();
        m_avoidance_cache_holefree.clear();
        m_avoidance_cache_holefree_to_model.clear();
        m_wall_restrictions_cache.clear();
        m_wall_restrictions_cache_min.clear();
        m_seam_avoidance_cache.clear();
    }

    enum class AvoidanceType : int8_t
    {
        Slow,
        FastSafe,
        Fast,
        Count
    };

    /*!
     * \brief Precalculate avoidances and collisions up to max_layer.
     *
     * Knowledge about branch angle is used to only calculate avoidances and collisions that may actually be needed.
     * Not calling precalculate() will cause the class to lazily calculate avoidances and collisions as needed, which will be a lot slower on systems with more then one or two cores!
     */
    void precalculate(const PrintObject& print_object, const coord_t max_layer, std::function<void()> throw_on_cancel);

    /*!
     * \brief Provides the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model.
     *
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param min_xy_dist Is the minimum xy distance used.
     * \return Polygons object
     */
    const Polygons& getCollision(const coord_t radius, LayerIndex layer_idx, bool min_xy_dist) const;

    // Get a collision area at a given layer for a radius that is a lower or equial to the key radius.
    // It is expected that the collision area is precalculated for a given layer at least for the radius zero.
    // Used for pushing tree supports away from object during the final Organic optimization step.
    std::optional<std::pair<coord_t, std::reference_wrapper<const Polygons>>> get_collision_lower_bound_area(LayerIndex layer_id, coord_t max_radius) const;

    /*!
     * \brief Provides the areas that have to be avoided by the tree's branches
     * in order to reach the build plate.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model or be unable to reach the build platform.
     *
     * The input collision areas are inset by the maximum move distance and
     * propagated upwards.
     *
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param type Is the propagation with the maximum move distance slow required.
     * \param to_model Does the avoidance allow good connections with the model.
     * \param min_xy_dist is the minimum xy distance used.
     * \return Polygons object
     */
    const Polygons& getAvoidance(coord_t radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist) const;
    /*!
     * \brief Provides the area represents all areas on the model where the branch does completely fit on the given layer.
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \return Polygons object
     */
    const Polygons& getPlaceableAreas(coord_t radius, LayerIndex layer_idx, std::function<void()> throw_on_cancel) const;
    /*!
     * \brief Provides the area that represents the walls, as in the printed area, of the model. This is an abstract representation not equal with the outline. See calculateWallRestrictions for better description.
     * \param radius The radius of the node of interest.
     * \param layer_idx The layer of interest.
     * \param min_xy_dist is the minimum xy distance used.
     * \return Polygons object
     */
    const Polygons& getWallRestriction(coord_t radius, LayerIndex layer_idx, bool min_xy_dist) const;
    /*!
     * \brief Round \p radius upwards to either a multiple of m_radius_sample_resolution or a exponentially increasing value
     *
     *	It also adds the difference between the minimum xy distance and the regular one.
     *
     * \param radius The radius of the node of interest
     * \param min_xy_dist is the minimum xy distance used.
     * \return The rounded radius
     */
    coord_t ceilRadius(const coord_t radius, const bool min_xy_dist) const {
        assert(radius >= 0);
        return min_xy_dist ? 
            this->ceilRadius(radius) :
            // special case as if a radius 0 is requested it could be to ensure correct xy distance. As such it is beneficial if the collision is as close to the configured values as possible.
            radius > 0 ? this->ceilRadius(radius + m_current_min_xy_dist_delta) : m_current_min_xy_dist_delta;
    }
    /*!
     * \brief Round \p radius upwards to the maximum that would still round up to the same value as the provided one.
     *
     * \param radius The radius of the node of interest
     * \param min_xy_dist is the minimum xy distance used.
     * \return The maximum radius, resulting in the same rounding.
     */
    coord_t getRadiusNextCeil(coord_t radius, bool min_xy_dist) const {
        assert(radius > 0);
        return min_xy_dist ?
            this->ceilRadius(radius) :
            this->ceilRadius(radius + m_current_min_xy_dist_delta) - m_current_min_xy_dist_delta;
    }

    // True only when build-plate-only supports have usable upward-facing base paint.
    // In this mode to_model avoidances describe routes to paint, not to the bed.
    bool painted_bases_only() const { return m_painted_bases_only; }

    // The paint visible from above under a branch ending at layer_idx, across
    // every lower layer it spans; empty unless paint lies at that branch's base.
    const Polygons& getPaintedBaseSurface(LayerIndex layer_idx) const;

    // Where a foundation foot may stand at layer_idx: the mirror image of the
    // overhangs tree tips support. TreeSupport::detect_overhangs counts the part
    // of a layer that reaches beyond the layer below it by more than a layer
    // height over the tangent of the support threshold angle. A base is the part
    // of the layer under the bottom gap that reaches as far beyond the layer
    // above it: a surface flatter than that angle. Like a tip on its overhang, a
    // foot only has to stand on this band, however thin a slope makes it.
    // Limited to the placeable areas of radius zero, so to paint where support
    // bases are painted, and clear of support blockers. Not for reversed views.
    Polygons getBaseLandingAreas(LayerIndex layer_idx, std::function<void()> throw_on_cancel) const;

    // Where support at layer_idx would rest on the model, within the bottom gap,
    // on surface that is not painted as a base: empty unless support bases are
    // painted. Clipped to window. Not for reversed views.
    Polygons getUnpaintedRestingAreas(LayerIndex layer_idx, BoundingBox window) const;

    Polygon m_bed_area;

private:
    // Caching polygons for a range of layers.
    class LayerPolygonCache {
    public:
        void allocate(LayerIndex aidx_begin, LayerIndex aidx_end) {
            m_idx_begin = aidx_begin;
            m_idx_end = aidx_end;
            m_polygons.assign(aidx_end - aidx_begin, {});
        }

        LayerIndex begin() const { return m_idx_begin; }
        LayerIndex end()   const { return m_idx_end; }
        size_t     size()  const { return m_polygons.size(); }

        bool      has(LayerIndex idx) const { return idx >= m_idx_begin && idx < m_idx_end; }
        Polygons& operator[](LayerIndex idx) { assert(idx >= m_idx_begin && idx < m_idx_end); return m_polygons[idx - m_idx_begin]; }
        std::vector<Polygons>& polygons_mutable() { return m_polygons; }

    private:
        std::vector<Polygons> m_polygons;
        LayerIndex            m_idx_begin;
        LayerIndex            m_idx_end;
    };

    /*!
     * \brief Convenience typedef for the keys to the caches
     */
    using RadiusLayerPair             = std::pair<coord_t, LayerIndex>;
    class RadiusLayerPolygonCache {
        // Map from radius to Polygons. Cache of one layer collision regions.
        using LayerData = std::map<coord_t, Polygons>;
        // Vector of layers, at each layer map of radius to Polygons.
        // Reference to Polygons returned shall be stable to insertion.
        using Layers = std::vector<LayerData>;
    public:
        RadiusLayerPolygonCache() = default;
        RadiusLayerPolygonCache(RadiusLayerPolygonCache &&rhs) : m_data(std::move(rhs.m_data)), m_max_layer(std::move(rhs.m_max_layer)) {}
        RadiusLayerPolygonCache& operator=(RadiusLayerPolygonCache &&rhs) { m_data = std::move(rhs.m_data); m_max_layer = std::move(rhs.m_max_layer); return *this; }

        RadiusLayerPolygonCache(const RadiusLayerPolygonCache&) = delete;
        RadiusLayerPolygonCache& operator=(const RadiusLayerPolygonCache&) = delete;

        void insert(std::vector<std::pair<RadiusLayerPair, Polygons>> &&in) {
            std::lock_guard<std::mutex> guard(m_mutex);
            for (auto &d : in) {
                this->get_allocate_layer_data(d.first.second).emplace(d.first.first, std::move(d.second));
                this->note_layer(d.first.first, d.first.second);
            }
        }
        // by layer
        void insert(std::vector<std::pair<coord_t, Polygons>> &&in, coord_t radius) {
            std::lock_guard<std::mutex> guard(m_mutex);
            for (auto &d : in) {
                this->get_allocate_layer_data(d.first).emplace(radius, std::move(d.second));
                this->note_layer(radius, d.first);
            }
        }
        void insert(std::vector<Polygons> &&in, coord_t first_layer_idx, coord_t radius) {
            std::lock_guard<std::mutex> guard(m_mutex);
            allocate_layers(first_layer_idx + in.size());
            for (auto &d : in) {
                m_data[first_layer_idx].emplace(radius, std::move(d));
                this->note_layer(radius, first_layer_idx ++);
            }
        }
        void insert(LayerPolygonCache &&in, coord_t radius) {
            std::lock_guard<std::mutex> guard(m_mutex);
            LayerIndex i = in.begin();
            allocate_layers(i + LayerIndex(in.size()));
            for (auto &d : in.polygons_mutable()) {
                m_data[i].emplace(radius, std::move(d));
                this->note_layer(radius, i ++);
            }
        }
        /*!
         * \brief Checks a cache for a given RadiusLayerPair and returns it if it is found
         * \param key RadiusLayerPair of the requested areas. The radius will be calculated up to the provided layer.
         * \return A wrapped optional reference of the requested area (if it was found, an empty optional if nothing was found)
         */
        std::optional<std::reference_wrapper<const Polygons>> getArea(const TreeModelVolumes::RadiusLayerPair &key) const {
            std::lock_guard<std::mutex> guard(m_mutex);
            if (key.second >= m_data.size())
                return std::optional<std::reference_wrapper<const Polygons>>{};
            const auto &layer = m_data[key.second];
            auto it = layer.find(key.first);
            return it == layer.end() ? 
                std::optional<std::reference_wrapper<const Polygons>>{} : std::optional<std::reference_wrapper<const Polygons>>{ it->second };
        }
        // Get a collision area at a given layer for a radius that is a lower or equial to the key radius.
        std::optional<std::pair<coord_t, std::reference_wrapper<const Polygons>>> get_lower_bound_area(const TreeModelVolumes::RadiusLayerPair &key) const {
            std::lock_guard<std::mutex> guard(m_mutex);
            if (key.second >= m_data.size())
                return {};
            const auto &layer = m_data[key.second];
            if (layer.empty())
                return {};
            auto it = layer.lower_bound(key.first);
            if (it == layer.end() || it->first != key.first) {
                if (it == layer.begin())
                    return {};
                -- it;
            }
            return std::make_pair(it->first, std::reference_wrapper<const Polygons>(it->second));
        }
        /*!
         * \brief Get the highest already calculated layer in the cache.
         * \param radius The radius for which the highest already calculated layer has to be found.
         * \param map The cache in which the lookup is performed.
         *
         * \return A wrapped optional reference of the requested area (if it was found, an empty optional if nothing was found)
         */
        LayerIndex getMaxCalculatedLayer(coord_t radius) const {
            std::lock_guard<std::mutex> guard(m_mutex);
            auto it = m_max_layer.find(radius);
            // The placeable on model areas do not exist on layer 0, as there can not be model below it. As such it may be possible that layer 1 is available, but layer 0 does not exist.
            return it == m_max_layer.end() || it->second == 0 ? -1 : it->second;
        }

        // For debugging purposes, sorted by layer index, then by radius.
        [[nodiscard]] std::vector<std::pair<RadiusLayerPair, std::reference_wrapper<const Polygons>>> sorted() const;

        void clear() { m_data.clear(); m_max_layer.clear(); }
        void clear_all_but_radius0() { 
            m_max_layer.clear();
            for (LayerData &l : m_data) {
                auto begin = l.begin();
                auto end = l.end();
                if (begin != end && ++ begin != end)
                    l.erase(begin, end);
                if (! l.empty())
                    this->note_layer(l.begin()->first, LayerIndex(&l - m_data.data()));
            }
        }

    private:
        LayerData&          get_allocate_layer_data(LayerIndex layer_idx) {
            allocate_layers(layer_idx + 1);
            return m_data[layer_idx];
        }
        void                allocate_layers(size_t num_layers);
        void                note_layer(coord_t radius, LayerIndex layer_idx) {
            auto [it, inserted] = m_max_layer.emplace(radius, layer_idx);
            if (! inserted && it->second < layer_idx)
                it->second = layer_idx;
        }

        Layers              m_data;
        // Highest layer holding each radius, so getMaxCalculatedLayer() needs no scan.
        std::map<coord_t, LayerIndex> m_max_layer;
        mutable std::mutex  m_mutex;
    };


    /*!
     * \brief Provides the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer. Holes are removed.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model or be inside a hole.
     * A Hole is defined as an area, in which a branch with m_increase_until_radius radius would collide with the wall.
     * minimum xy distance is always used.
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param min_xy_dist Is the minimum xy distance used.
     * \return Polygons object
     */
    const Polygons& getCollisionHolefree(coord_t radius, LayerIndex layer_idx) const;

    // Entries that are computed when first requested, possibly by many threads at
    // once. Each of them would compute the same entries and only the first result
    // would be kept: let one thread compute them while the others wait for it.
    // A computation only ever waits for a later family in this order, never for
    // its own family and radius: avoidance -> placeables -> hole-free collision
    // -> collision, and wall restrictions -> collision. A reversed view waits
    // only for its source's collision.
    enum class LazyCache { Collision, CollisionHolefree, AvoidanceToBuildPlate, AvoidanceToModel, Placeables, WallRestrictions };
    struct LazyLocks {
        std::mutex mutex;
        std::map<std::pair<int, coord_t>, std::unique_ptr<std::mutex>> locks;
    };
    std::unique_ptr<LazyLocks> m_lazy_locks { std::make_unique<LazyLocks>() };
    // Run compute() unless done() holds, at most one thread at a time per cache and radius.
    template<typename Done, typename Compute>
    void compute_once(LazyCache cache, coord_t radius, Done &&done, Compute &&compute) const;

    // Internal avoidance reads may use rounded collision-cache keys, including
    // XY clearance. Only public physical-radius requests apply the seam limit.
    const Polygons& getAvoidanceImpl(coord_t radius, LayerIndex layer_idx, AvoidanceType type,
                                    bool to_model, bool min_xy_dist) const;

    /*!
     * \brief Round \p radius upwards to either a multiple of m_radius_sample_resolution or a exponentially increasing value
     *
     * \param radius The radius of the node of interest
     */
    coord_t ceilRadius(const coord_t radius) const;

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model. Result is saved in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateCollision(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel);
    void calculateCollision(const coord_t radius, const LayerIndex max_layer_idx, std::function<void()> throw_on_cancel);
    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer. Holes are removed.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model or be inside a hole. Result is saved in the cache.
     * A Hole is defined as an area, in which a branch with m_increase_until_radius radius would collide with the wall.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateCollisionHolefree(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer. Holes are removed.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model or be inside a hole. Result is saved in the cache.
     * A Hole is defined as an area, in which a branch with m_increase_until_radius radius would collide with the wall.
     * \param key RadiusLayerPairs the requested areas. The radius will be calculated up to the provided layer.
     */
    void calculateCollisionHolefree(RadiusLayerPair key)
    {
        calculateCollisionHolefree(std::vector<RadiusLayerPair>{ RadiusLayerPair(key) }, []{});
    }

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model. Result is saved in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateAvoidance(const std::vector<RadiusLayerPair> &keys, bool to_build_plate, bool to_model, std::function<void()> throw_on_cancel);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model. Result is saved in the cache.
     * \param key RadiusLayerPair of the requested areas. It will be calculated up to the provided layer.
     */
    void calculateAvoidance(RadiusLayerPair key, bool to_build_plate, bool to_model)
    {
        calculateAvoidance(std::vector<RadiusLayerPair>{ RadiusLayerPair(key) }, to_build_plate, to_model, []{});
    }

    /*!
     * \brief Creates the areas where a branch of a given radius can be place on the model.
     * Result is saved in the cache.
     * \param key RadiusLayerPair of the requested areas. It will be calculated up to the provided layer.
     */
    void calculatePlaceables(const coord_t radius, const LayerIndex max_required_layer, std::function<void()> throw_on_cancel);


    /*!
     * \brief Creates the areas where a branch of a given radius can be placed on the model.
     * Result is saved in the cache.
     * \param keys RadiusLayerPair of the requested areas. The radius will be calculated up to the provided layer.
     */
    void calculatePlaceables(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel);

    // Centres where a branch with this footprint radius may end at layer_idx,
    // touching paint at its base layer and with all of its footprint over paint.
    Polygons calculatePaintedPlaceables(coord_t footprint, LayerIndex layer_idx) const;
    // The part of layer_idx's top that reaches beyond the layer above it by more than
    // m_base_landing_step, clipped to window: a surface flatter than the support
    // threshold angle. Not for reversed views.
    Polygons calculateBaseBand(LayerIndex layer_idx, BoundingBox window) const;

    /*!
     * \brief Creates the areas that can not be passed when expanding an area downwards. As such these areas are an somewhat abstract representation of a wall (as in a printed object).
     *
     * These areas are at least xy_min_dist wide. When calculating it is always assumed that every wall is printed on top of another (as in has an overlap with the wall a layer below). Result is saved in the corresponding cache.
     *
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateWallRestrictions(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel);

    /*!
     * \brief Creates the areas that can not be passed when expanding an area downwards. As such these areas are an somewhat abstract representation of a wall (as in a printed object).
     * These areas are at least xy_min_dist wide. When calculating it is always assumed that every wall is printed on top of another (as in has an overlap with the wall a layer below). Result is saved in the corresponding cache.
     * \param key RadiusLayerPair of the requested area. It well be will be calculated up to the provided layer.
     */
    void calculateWallRestrictions(RadiusLayerPair key)
    {
        calculateWallRestrictions(std::vector<RadiusLayerPair>{ RadiusLayerPair(key) }, []{});
    }

    /*!
     * \brief The maximum distance that the center point of a tree branch may move in consecutive layers if it has to avoid the model.
     */
    coord_t m_max_move;
    /*!
     * \brief The maximum distance that the centre-point of a tree branch may
     * move in consecutive layers if it does not have to avoid the model
     */
    coord_t m_max_move_slow;
    /*!
     * \brief The smallest maximum resolution for simplify
     */
    coord_t m_min_resolution;

    bool m_precalculated = false;
    /*!
     * \brief The index to access the outline corresponding with the currently processing mesh
     */
    size_t m_current_outline_idx;
    /*!
     * \brief The minimum required clearance between the model and the tree branches
     */
    coord_t m_current_min_xy_dist;
    /*!
     * \brief The difference between the minimum required clearance between the model and the tree branches and the regular one.
     */
    coord_t m_current_min_xy_dist_delta;
    /*!
     * \brief Does at least one mesh allow support to rest on a model.
     */
    bool m_support_rests_on_model;
#ifdef SLIC3R_TREESUPPORTS_PROGRESS
    /*!
     * \brief The progress of the precalculate function for communicating it to the progress bar.
     */
    coord_t m_precalculation_progress = 0;
    /*!
     * \brief The progress multiplier of all values added progress bar.
     * Required for the progress bar the behave as expected when areas have to be calculated multiple times
     */
    double m_progress_multiplier;
    /*!
     * \brief The progress offset added to all values communicated to the progress bar.
     * Required for the progress bar the behave as expected when areas have to be calculated multiple times
     */
    double m_progress_offset;
#endif // SLIC3R_TREESUPPORTS_PROGRESS
    /*!
     * \brief Increase radius in the resulting drawn branches, even if the avoidance does not allow it. Will be cut later to still fit.
     */
    coord_t m_increase_until_radius;

    /*!
     * \brief Polygons representing the limits of the printable area of the
     * machine
     */
    Polygons m_machine_border;
    /*!
     * \brief Storage for layer outlines and the corresponding settings of the meshes grouped by meshes with identical setting.
     */
    std::vector<std::pair<TreeSupportMeshGroupSettings, std::vector<Polygons>>> m_layer_outlines;
    /*!
     * \brief Storage for areas that should be avoided, like support blocker or previous generated trees.
     */
    std::vector<Polygons> m_anti_overhang;

    bool m_painted_bases_only { false };
    // Actual painted surface footprints, indexed with the organic raft offset.
    std::vector<Polygons> m_painted_base_areas;
    // At each layer holding painted surface: all paint visible from above there,
    // including the uncovered paint of lower layers. A sloped base gives each
    // layer only a thin band, so a branch foot on it spans several layers' bands.
    std::vector<Polygons> m_painted_base_visible;
    // A branch ending at layer L rests on the surface of layer L - m_painted_base_below.
    LayerIndex m_painted_base_below { 1 };
    // How far a base must reach beyond the layer above it, as an overhang must
    // reach beyond the layer below it in TreeSupport::detect_overhangs.
    coord_t m_base_landing_step { 0 };
    // The widest hairline crack between bands of painted mesh surface and the
    // simplified, XY compensated slices clipping them.
    float m_painted_crack { 0.f };
    // Filled work domain used to seed model-only reachability below any paint.
    Polygons m_painted_route_domain;
    /*!
     * \brief Radii that can be ignored by ceilRadius as they will never be requested, sorted.
     */
    std::vector<coord_t> m_ignorable_radii;

    /*!
     * \brief Smallest radius a branch can have. This is the radius of a SupportElement with DTT=0.
     */
    coord_t m_radius_0;

    // Z heights of the raft layers (additional layers below the object, last raft layer aligned with the bottom of the first object layer).
    std::vector<double>         m_raft_layers;

    /*!
     * \brief Caches for the collision, avoidance and areas on the model where support can be placed safely
     * at given radius and layer indices.
     */
    RadiusLayerPolygonCache     m_collision_cache;
    RadiusLayerPolygonCache     m_collision_cache_holefree;
    RadiusLayerPolygonCache     m_avoidance_cache;
    RadiusLayerPolygonCache     m_avoidance_cache_slow;
    RadiusLayerPolygonCache     m_avoidance_cache_to_model;
    RadiusLayerPolygonCache     m_avoidance_cache_to_model_slow;
    RadiusLayerPolygonCache     m_placeable_areas_cache;

    // A reversed view shares physical collision data but owns all directional
    // avoidance/wall caches. Blockers are added to its own collision cache.
    const TreeModelVolumes    *m_reversed_source { nullptr };
    std::function<void()>      m_reversed_throw_on_cancel;
    LayerIndex                 m_seam_layer { 0 };
    coord_t                    m_max_seam_radius { std::numeric_limits<coord_t>::max() };
    Polygons                   m_seam_blocked_domain;
    Polygons                   m_seam_excluded_centers;
    RadiusLayerPolygonCache    m_seam_avoidance_cache;

    /*!
     * \brief Caches to avoid holes smaller than the radius until which the radius is always increased, as they are free of holes. 
     * Also called safe avoidances, as they are safe regarding not running into holes.
     */
    RadiusLayerPolygonCache     m_avoidance_cache_holefree;
    RadiusLayerPolygonCache     m_avoidance_cache_holefree_to_model;

    RadiusLayerPolygonCache& avoidance_cache(const AvoidanceType type, const bool to_model) {
        if (to_model) {
            switch (type) {
            case AvoidanceType::Fast:       return m_avoidance_cache_to_model;
            case AvoidanceType::Slow:       return m_avoidance_cache_to_model_slow;
            case AvoidanceType::Count:      assert(false);
            case AvoidanceType::FastSafe:   return m_avoidance_cache_holefree_to_model;
            }
        } else {
            switch (type) {
            case AvoidanceType::Fast:       return m_avoidance_cache;
            case AvoidanceType::Slow:       return m_avoidance_cache_slow;
            case AvoidanceType::Count:      assert(false);
            case AvoidanceType::FastSafe:   return m_avoidance_cache_holefree;
            }
        }
        assert(false);
        return m_avoidance_cache;
    }
    const RadiusLayerPolygonCache& avoidance_cache(const AvoidanceType type, const bool to_model) const {
        return const_cast<TreeModelVolumes*>(this)->avoidance_cache(type, to_model);
    }

    /*!
     * \brief Caches to represent walls not allowed to be passed over.
     */
    RadiusLayerPolygonCache     m_wall_restrictions_cache;

    // A different cache for min_xy_dist as the maximal safe distance an influence area can be increased(guaranteed overlap of two walls in consecutive layer) 
    // is much smaller when min_xy_dist is used. This causes the area of the wall restriction to be thinner and as such just using the min_xy_dist wall 
    // restriction would be slower.    
    RadiusLayerPolygonCache     m_wall_restrictions_cache_min;

#ifdef SLIC3R_TREESUPPORTS_PROGRESS
    std::unique_ptr<std::mutex> m_critical_progress { std::make_unique<std::mutex>() };
#endif // SLIC3R_TREESUPPORTS_PROGRESS
};

} // namespace TreeSupport3D
} // namespace Slic3r

#endif //slic3r_TreeModelVolumes_hpp
