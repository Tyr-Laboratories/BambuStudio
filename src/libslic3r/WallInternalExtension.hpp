#ifndef slic3r_WallInternalExtension_hpp_
#define slic3r_WallInternalExtension_hpp_

// Wall internal extension: where an outer wall rises out of a top surface, or ends under a bottom surface, the wall
// line is continued inside the part, through the shell and `wall_internal_extension` mm beyond it, with a 45 degree
// chamfer on the surface side. See docs/superpowers/specs/2026-11-11-wall-internal-extension-design.md.

#include "ExPolygon.hpp"
#include "Polyline.hpp"

#include <vector>

namespace Slic3r {
namespace WallInternalExtension {

// What the joint search needs from one layer. Geometry is scaled, z values are in mm.
struct LayerGeometry
{
    double     bottom_z = 0.;
    double     print_z  = 0.;
    // The layer's islands (Layer::lslices).
    ExPolygons outline;
    // Area covered by every wall extrusion of the layer.
    ExPolygons walls;
};

// Settings of the region whose outer wall forms a joint.
struct JointSettings
{
    // wall_internal_extension, mm.
    double  extension              = 0.;
    int     top_shell_layers       = 0;
    double  top_shell_thickness    = 0.;
    int     bottom_shell_layers    = 0;
    double  bottom_shell_thickness = 0.;
    // Outer wall line width at the joint layer, scaled.
    coord_t line_width             = 0;
    // Slicing tolerance, max(resolution, slice_closing_radius), scaled.
    coord_t tolerance              = 0;
};

enum class JointSide {
    // The wall rises out of a top surface; the extension goes down.
    Top,
    // The wall ends under a bottom surface; the extension goes up.
    Bottom
};

// Pieces of one region's outer wall line at `layer` that stand on a flat surface, and how far the extension reaches.
struct Joint
{
    size_t    layer       = 0;
    JointSide side        = JointSide::Top;
    // Centrelines of the qualifying pieces of the outer wall line, scaled.
    Polylines lines;
    // Depth of the shell beside the joint, mm.
    double    shell_depth = 0.;
    // wall_internal_extension of the joint's region, mm.
    double    extension   = 0.;
    // Outer wall line width at the joint layer, scaled.
    coord_t   line_width  = 0;
};

// Depth in mm of the top shell under a top surface on `surface_layer`, built as discover_vertical_shells() builds it:
// layer i <= surface_layer belongs to it while (surface_layer - i < shell_layers) or
// (print_z[surface_layer] - print_z[i] < shell_thickness). 0 when no layer qualifies.
double top_shell_depth(const std::vector<LayerGeometry> &layers, size_t surface_layer, int shell_layers, double shell_thickness);
// The mirror image for the bottom shell above a bottom surface on `surface_layer`.
double bottom_shell_depth(const std::vector<LayerGeometry> &layers, size_t surface_layer, int shell_layers, double shell_thickness);

// Joints formed by `outer_wall_lines`, the centrelines of one region's outer wall extrusions at `layer`. A piece of the
// line is a top joint where its whole width stands over the inside of the layer below, away from that layer's walls,
// and the wall above it stays strictly steeper than 45 degrees, leaning either way, until it is `extension` tall; a
// bottom joint is the mirror image. At most one joint per side.
std::vector<Joint> find_joints(const std::vector<LayerGeometry> &layers, size_t layer, const Polylines &outer_wall_lines,
                               const JointSettings &settings);

} // namespace WallInternalExtension
} // namespace Slic3r

#endif // slic3r_WallInternalExtension_hpp_
