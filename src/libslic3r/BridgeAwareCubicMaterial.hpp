#ifndef slic3r_BridgeAwareCubicMaterial_hpp_
#define slic3r_BridgeAwareCubicMaterial_hpp_

// Bridge aware cubic: one definition of printed material and void (rewrite architecture, section 3.3), the fill boundary the
// filler clips to (3.8), and an index over areas for the plan's many point and segment queries.
//
// Two sides, both conservative: what MAY print (M⁺: the walls' beads, every non-sparse surface, the pattern's courses) is the
// roof side of the overhang rule, what SURELY prints (M⁻: the walls' beads, the parts of non-sparse surfaces at least 1.1
// spacings wide, the courses) is the support side. The void V is everything inside the outline that does not surely print.

#include "libslic3r.h"
#include "BoundingBox.hpp"
#include "EdgeGrid.hpp"
#include "ExPolygon.hpp"
#include "Polyline.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace Slic3r {

class Layer;
class LayerRegion;

namespace BridgeAwareCubic {

struct LayerMaterial
{
    ExPolygons              walls;           // the beads of the perimeters and the gap fill between them, by nominal width, inside lslices
    ExPolygons              may_print;       // M⁺
    ExPolygons              surely_prints;   // M⁻
    ExPolygons              void_area;       // V = lslices − M⁻
    std::vector<ExPolygons> void_of_group;   // S^G: the components of V that meet the sparse surfaces of group G's regions
};

// groups_regions[g] lists the region ids of lattice group g. Reads the layer's surfaces and configs only.
LayerMaterial printed_material(const Layer &layer, const std::vector<std::vector<size_t>> &groups_regions);

// Adds courses (seam walls, rails: beads of width `width_mm` the pattern prints whole) to both sides of the material and
// recomputes the void.
void add_courses(LayerMaterial &material, const Polylines &courses, double width_mm, const Layer &layer,
                 const std::vector<std::vector<size_t>> &groups_regions);

// The islands of the pattern's sparse fills on a layer, as Layer::make_fills will hand them to the filler, each with the
// regions whose sparse fill it holds (through Fill.cpp's grouping).
struct SparseIsland
{
    std::vector<size_t> region_ids;
    ExPolygon           island;
};
std::vector<SparseIsland> sparse_islands(const Layer &layer);

// The boundary the filler clips the lattice to: the island inset by half the sparse spacing (as FillCubic insets it), less the
// courses' beads kept at w/2 − 0.05 from a course's centre line, so that a line ends inside the course's bead and the filler's
// nub fuses it to the course. `infill_wall_overlap_percent` is kept for the callers and no longer enters.
ExPolygons fill_boundary(const ExPolygon &island, const Polylines &courses, double spacing_mm, double width_mm,
                         double infill_wall_overlap_percent);

// An index over ExPolygons (scaled, absolute) for point containment and for the parts of a segment that lie inside.
class AreaIndex
{
public:
    AreaIndex() = default;
    explicit AreaIndex(ExPolygons polygons, coord_t resolution = coord_t(scale_(1.)));
    // Copies rebuild the grid (a copy happens only when a plan's containers reallocate, with empty boundaries).
    AreaIndex(const AreaIndex &other) : AreaIndex(other.m_polygons, other.m_grid ? other.m_grid->resolution() : coord_t(scale_(1.))) {}
    AreaIndex &operator=(const AreaIndex &other) { *this = AreaIndex(other); return *this; }
    AreaIndex(AreaIndex &&other) noexcept
        : m_polygons(std::move(other.m_polygons)), m_bbox(other.m_bbox), m_grid(std::move(other.m_grid)), m_centre_inside(std::move(other.m_centre_inside)) {}
    AreaIndex &operator=(AreaIndex &&other) noexcept
    {
        m_polygons      = std::move(other.m_polygons);
        m_bbox          = other.m_bbox;
        m_grid          = std::move(other.m_grid);
        m_centre_inside = std::move(other.m_centre_inside);
        return *this;
    }

    bool               empty() const { return m_polygons.empty(); }
    const ExPolygons&  polygons() const { return m_polygons; }
    const BoundingBox& bbox() const { return m_bbox; }
    bool               contains(const Point &point) const;
    // The parameter intervals of the segment a + t (b − a), t in [0, 1], that lie inside the area, in increasing order.
    std::vector<std::pair<double, double>> segment_inside(const Point &a, const Point &b) const;
    // Whether segment_inside(a, b) is the one interval [0, 1] (within 1e-9), without building it.
    bool               segment_inside_whole(const Point &a, const Point &b) const;
    // Whether the segment crosses or touches any edge of the area.
    bool               segment_crosses_edge(const Point &a, const Point &b) const;

private:
    // The parameters along a→b where the segment crosses an edge, sorted; the second form appends them to `out`.
    std::vector<double> crossings(const Point &a, const Point &b) const;
    void                crossings(const Point &a, const Point &b, std::vector<double> &out) const;

    ExPolygons                       m_polygons;
    BoundingBox                      m_bbox;      // of the polygons
    std::unique_ptr<EdgeGrid::Grid>  m_grid;      // over a bbox inflated by two cells, so that clipped queries stay inside it
    std::vector<uint8_t>             m_centre_inside;    // per grid cell (row-major): whether its centre lies inside
};

} // namespace BridgeAwareCubic
} // namespace Slic3r

#endif // slic3r_BridgeAwareCubicMaterial_hpp_
