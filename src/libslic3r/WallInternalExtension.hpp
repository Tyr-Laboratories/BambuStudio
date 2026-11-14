#ifndef slic3r_WallInternalExtension_hpp_
#define slic3r_WallInternalExtension_hpp_

// Wall internal extension: where an outer wall rises out of a top surface, or ends under a bottom surface, the wall
// line is continued inside the part, through the shell and `wall_internal_extension` mm beyond it, with a 45 degree
// chamfer on the surface side when `wall_internal_extension_chamfer` is on (off by default).
// See docs/superpowers/specs/2026-11-11-wall-internal-extension-design.md.

#include "ExPolygon.hpp"
#include "ExtrusionEntityCollection.hpp"
#include "Polyline.hpp"

#include <vector>

namespace Slic3r {
class Flow;
class PrintObject;
namespace Arachne { struct WallToolPathsParams; }

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
    // `walls` grown by `carried_reach` (scaled): the area that carries a joint line, made once per layer when the joint
    // lines of both neighbours have that reach. find_joints() computes it itself for any other reach.
    ExPolygons carried;
    float      carried_reach = -1.f;
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
    // Outer wall line spacing at the joint layer, scaled. Neighbouring lines of one layer overlap by line_width - line_spacing.
    coord_t line_spacing           = 0;
    // Slicing tolerance (the G-code resolution), scaled.
    coord_t tolerance              = 0;
    // wall_internal_extension_chamfer: add the 45 degree chamfer on the surface side.
    bool    chamfer                = false;
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
    // Slicing tolerance, scaled (JointSettings::tolerance).
    coord_t   tolerance   = 0;
    // Whether the extension gets its chamfer (JointSettings::chamfer).
    bool      chamfer     = false;
};

// Depth in mm of the top shell under a top surface on `surface_layer`, built as discover_vertical_shells() builds it:
// layer i <= surface_layer belongs to it while (surface_layer - i < shell_layers) or
// (print_z[surface_layer] - print_z[i] < shell_thickness). 0 when no layer qualifies.
double top_shell_depth(const std::vector<LayerGeometry> &layers, size_t surface_layer, int shell_layers, double shell_thickness);
// The mirror image for the bottom shell above a bottom surface on `surface_layer`.
double bottom_shell_depth(const std::vector<LayerGeometry> &layers, size_t surface_layer, int shell_layers, double shell_thickness);

// How far, scaled, a neighbouring layer's walls are grown to find where they carry a joint line of `settings`: half the
// line width, less the overlap of neighbouring lines and the slicing tolerance.
float carried_reach(const JointSettings &settings);

// Joints formed by `outer_wall_lines`, the centrelines of one region's outer wall extrusions at `layer`. A piece of the
// line is a top joint where its whole width stands over the inside of the layer below, away from that layer's walls (a
// wall line overlapping it no more than neighbouring lines of one layer do only touches it), and the wall above it stays
// strictly steeper than 45 degrees, leaning either way, until it is `extension` tall; a bottom joint is the mirror image.
// At most one joint per side.
std::vector<Joint> find_joints(const std::vector<LayerGeometry> &layers, size_t layer, const Polylines &outer_wall_lines,
                               const JointSettings &settings);

struct ExtensionArea
{
    // Covered by extension lines and chamfers at the target layer, before clipping to the part.
    ExPolygons all;
    // The part of `all` on a top joint's deepest layer: nothing of the extension is below it.
    ExPolygons hanging;
};

// Area at `target_layer` covered by the extension lines and 45 degree chamfers of `joints`. A top joint's extension
// reaches down to shell_depth + extension below the surface, a bottom joint's up to the same distance above it. A
// joint with `chamfer` set adds the chamfer on the layers beyond the shell, on the surface side only: as wide as the
// extension next to the shell, narrowing by one layer height per layer to nothing at the extension's far end. Without
// it the extension is the line's own footprint on every layer.
ExtensionArea extension_area(const std::vector<LayerGeometry> &layers, const std::vector<Joint> &joints, size_t target_layer);

// Centrelines of the outer wall extrusions (erExternalPerimeter, erOverhangPerimeter) in `perimeters`, at any depth.
// The consecutive outer wall paths of a loop are joined back into one closed line.
Polylines outer_wall_lines(const ExtrusionEntityCollection &perimeters);

// Lines of role erWallExtension filling `area`: variable width Arachne walls of `flow`'s spacing. A line with more than
// half its length inside `bridge_area` is printed like a bridge: one constant width path of `bridge_flow`, flagged
// cfWallExtensionBridge.
ExtrusionEntityCollection extension_extrusions(const ExPolygon &area, const ExPolygons &bridge_area, const Flow &flow,
                                               const Flow &bridge_flow, double layer_height, size_t inset_count,
                                               const Arachne::WallToolPathsParams &params);

// Runs the wall internal extension on an object whose walls are generated: adds the extension lines to the regions'
// perimeters, cuts their area out of the fill sets and records it in LayerRegion::wall_extension_area. Does nothing in
// spiral vase mode or when no region has wall_internal_extension above 0.
void apply(PrintObject &object);

} // namespace WallInternalExtension
} // namespace Slic3r

#endif // slic3r_WallInternalExtension_hpp_
