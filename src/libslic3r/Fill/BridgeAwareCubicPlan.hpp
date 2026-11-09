#ifndef slic3r_BridgeAwareCubicPlan_hpp_
#define slic3r_BridgeAwareCubicPlan_hpp_

// Bridge aware cubic: the object's plan (rewrite architecture, section 3). Built once per object, after clip_fill_surfaces and
// before bridge_over_infill, from the sliced surfaces; read-only afterwards by the filler, the anchoring preview, the pass of
// section 3.8 and the development checks. It holds the lattice groups, the material of every layer, the demand planes under
// the roofs, the courses (seam walls, rails, solid-column courses) and the strands (needed, extensions, columns, covers) to
// print, by layer.

#include "../libslic3r.h"
#include "../BridgeAwareCubicMaterial.hpp"
#include "../ExPolygon.hpp"
#include "../Polyline.hpp"
#include "BridgeAwareCubicLattice.hpp"

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace Slic3r {

class PrintObject;
class Layer;

namespace BridgeAwareCubic {

enum class StrandKind : uint8_t { Needed, Extension, Column, Cover, SolidFace };

// A straight piece along a plane's trace on a layer: parameters t0 ≤ t1 along trace(plane, z of the layer), mm.
struct Strand
{
    PlaneKey   plane;
    double     t0 = 0., t1 = 0.;
    StrandKind kind = StrandKind::Needed;
    double     gap  = 0.;      // G of the plane that asked for it: the sweep bridges up to G − 0.1 mm
};

// A demand plane (3.4): the roof footprint D⁺ of layer `roof_layer`, measured and held on layer `plane_layer`.
struct Plane
{
    size_t      roof_layer  = 0;
    size_t      plane_layer = 0;
    double      gap         = 0.;    // G of the region whose void the footprint lies over
    double      bead        = 0.;    // r: the roof's narrowest bead that crosses the void
    ExPolygons  footprint;           // D⁺, scaled, absolute
    BoundingBox bbox;
};

struct PlaneSummary
{
    size_t roof_layer, plane_layer;
    double gap;
    double footprint_area_mm2;
};

struct Place
{
    size_t      layer;
    Vec2d       where;      // mm, absolute (the object's frame)
    std::string reason;
};

// The statistics of section 5.2, gathered in every build (a few places per object); the development flag governs the warnings.
struct PlanStatistics
{
    size_t             visits = 0;                 // cells visited by the refinement, all groups
    bool               visit_warning = false;      // some group passed 1,000,000 visits
    std::vector<Place> unheld;                     // sweep stretches and pieces left unheld, with the reason
    std::vector<Place> courses_over_other_void;    // lowest courses over another pattern's void, a 0 % region or air
    std::vector<size_t> thick_layers;              // layers whose height gives s_m ≥ w/2
    size_t             rails = 0;                  // rails laid
    std::vector<std::pair<size_t, ExPolygons>> rail_roofs;   // (roof layer, the candidate area each rail was laid for): the verifier counts the roofs the generator did not bridge
};

// The fill boundary of a group on one layer (3.8): every island of the group's sparse fill inset as the filler insets it,
// less the courses' beads; with an index for the plan's queries.
struct LayerBoundary
{
    ExPolygons boundary;
    AreaIndex  index;
};

class ObjectPlan;

class GroupPlan
{
public:
    const LatticeParams&        lattice() const { return m_lattice; }
    const std::vector<size_t>&  regions() const { return m_regions; }
    int                         bridge_layers() const { return m_bridge_layers; }
    // Whether the regions' sparse infill anchor settings make the connector anchor line ends (and the filler add nubs).
    bool                        anchors() const { return m_anchors; }
    const std::vector<Plane>&   planes() const { return m_planes; }
    const LayerBoundary&        boundary(size_t layer) const { return m_boundaries[layer]; }
    const std::vector<Strand>&  strands(size_t layer) const { return m_strands[layer]; }
    size_t                      layer_count() const { return m_boundaries.size(); }

    // Any strand or course on any layer.
    bool  has_content() const;
    // The strands of a layer whose trace meets `bbox`, as scaled segments, every sub-line of a band.
    Lines strands_at(size_t layer, const BoundingBox &bbox) const;
    // The segment of a strand on its layer, scaled, for one sub-line offset (mm).
    Line  segment(const Strand &strand, size_t layer, double z, double subline_offset = 0.) const;

    // For the plan's own units (refinement, solid columns, the sweep): add a strand on a layer; note that courses exist.
    void  add_strand(size_t layer, const Strand &strand) { m_strands[layer].push_back(strand); }
    void  mark_courses() { m_has_courses = true; }

private:
    friend class ObjectPlan;
    friend void refine(GroupPlan &, ObjectPlan &, const std::function<void()> &);
    friend void sweep(GroupPlan &, ObjectPlan &, const std::function<void()> &);

    LatticeParams                    m_lattice;
    std::vector<size_t>              m_regions;
    int                              m_bridge_layers = 1;
    bool                             m_anchors = true;
    std::vector<Plane>               m_planes;          // ordered by (plane_layer, roof_layer)
    std::vector<LayerBoundary>       m_boundaries;      // by layer
    std::vector<std::vector<Strand>> m_strands;         // by layer
    std::vector<double>              m_layer_z;         // print_z by layer
    bool                             m_has_courses = false;
};

class ObjectPlan
{
public:
    static std::unique_ptr<ObjectPlan> build(const PrintObject &object, const std::function<void()> &cancel);
    // The same plan for an object whose sparse infill is any pattern (its planes depend on the surfaces only): the verifier's
    // oracle measures a Cubic object with the planes its surfaces define (test V1). Never used by the slice.
    static std::unique_ptr<ObjectPlan> build_for_verification(const PrintObject &object, const std::function<void()> &cancel);

    // nullptr when the region does not print the pattern.
    const GroupPlan*          group_of_region(size_t region_id) const;
    // Seam walls, rails and solid-column courses of the region on the layer; empty when none.
    const Polylines&          courses_at(size_t layer, size_t region_id) const;
    // The courses that cut the fill boundary (seam walls and rails, 3.5): a solid column's courses lie inside its faces and
    // must not cut the lattice there (3.6c).
    const Polylines&          boundary_courses_at(size_t layer, size_t region_id) const;
    std::vector<PlaneSummary> planes() const;
    // Z_l of 3.9a; empty without infill combination.
    ExPolygons                single_layer_zone(size_t layer) const;
    const PlanStatistics&     statistics() const { return m_statistics; }
    PlanStatistics&           statistics_mutable() { return m_statistics; }
    // For the plan's own units: add courses of a region on a layer; `cut_boundary` for seam walls and rails, not for the solid
    // columns' courses.
    void                      add_courses(size_t layer, size_t region_id, const Polylines &courses, bool cut_boundary)
    {
        append(m_courses[layer][region_id], courses);
        if (cut_boundary)
            append(m_boundary_courses[layer][region_id], courses);
    }
    // G after the clamp to the nozzle diameter; 0 when the region does not print the pattern.
    double                    gap_of_region(size_t region_id) const;

    const std::vector<GroupPlan>& groups() const { return m_groups; }
    const LayerMaterial&          material(size_t layer) const { return m_materials[layer]; }
    size_t                        layer_count() const { return m_layer_heights.size(); }
    double                        print_z(size_t layer) const { return m_layer_heights[layer].first; }
    double                        layer_height(size_t layer) const { return m_layer_heights[layer].second; }
    // Half the nozzle diameter of the extruder that prints the layer's walls: the overhang distance.
    double                        overhang_distance(size_t layer) const { return m_overhang[layer]; }
    // The margins (mm) the sweep takes off its reaches α, c and w/2, so that a point the plan holds stays held on the export:
    // for the plan's own lines (two-point paths, which the path simplifier never moves) the G-code's coordinate rounding and the
    // checker's arc tolerance; for paths of the stock pipeline (bulk lines with their connector hooks, walls, solid fill) also
    // the path simplification to the print's `resolution`, or to SPARSE_INFILL_RESOLUTION for sparse infill when arcs are fitted.
    double                        vertex_margin() const { return m_vertex_margin; }
    double                        path_margin() const { return m_path_margin; }
    const std::vector<std::vector<size_t>>& groups_regions() const { return m_groups_regions; }

private:
    friend void refine(GroupPlan &, ObjectPlan &, const std::function<void()> &);
    friend void sweep(GroupPlan &, ObjectPlan &, const std::function<void()> &);

    void collect_regions(const PrintObject &object, bool any_sparse_pattern = false);
    void collect_material(const PrintObject &object, const std::function<void()> &cancel);
    void collect_seam_walls(const PrintObject &object, const std::function<void()> &cancel);
    void collect_rails(const PrintObject &object, const std::function<void()> &cancel);
    void collect_planes(const PrintObject &object, const std::function<void()> &cancel);
    void collect_boundaries(const PrintObject &object, const std::function<void()> &cancel);
    // The fill boundary of a group on a layer (3.8) from the layer's sparse islands and the group's boundary courses so far.
    ExPolygons group_boundary(const PrintObject &object, size_t layer_index, size_t g, const std::vector<SparseIsland> &islands) const;

    std::vector<GroupPlan>                    m_groups;
    std::vector<std::vector<size_t>>          m_groups_regions;
    std::vector<int>                          m_group_of_region;    // −1 when the region does not print the pattern
    std::vector<double>                       m_gap_of_region;
    std::vector<double>                       m_bead_of_region;     // the narrowest wall or solid bead of the region
    std::vector<LayerMaterial>                m_materials;          // by layer
    std::vector<std::pair<double, double>>    m_layer_heights;      // (print_z, height) by layer
    std::vector<double>                       m_overhang;           // h_n by layer
    std::vector<std::vector<Polylines>>       m_courses;            // [layer][region]
    std::vector<std::vector<Polylines>>       m_boundary_courses;   // [layer][region]: those that cut the fill boundary
    std::vector<std::map<size_t, ExPolygons>> m_rail_bands;         // [group][roof layer]: the band between a roof's edge and its rail (3.5b)
    double                                    m_vertex_margin = 0., m_path_margin = 0.;
    PlanStatistics                            m_statistics;
    Polylines                                 m_no_courses;
};

// Section 3.6: refinement of a group under its planes (needed strands and solid columns).
void refine(GroupPlan &group, ObjectPlan &plan, const std::function<void()> &cancel);
// Section 3.7: the support sweep of a group.
void sweep(GroupPlan &group, ObjectPlan &plan, const std::function<void()> &cancel);

} // namespace BridgeAwareCubic
} // namespace Slic3r

#endif // slic3r_BridgeAwareCubicPlan_hpp_
