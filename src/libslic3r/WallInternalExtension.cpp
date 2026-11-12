#include "WallInternalExtension.hpp"

#include "ClipperUtils.hpp"
#include "libslic3r.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace Slic3r {
namespace WallInternalExtension {

double top_shell_depth(const std::vector<LayerGeometry> &layers, size_t surface_layer, int shell_layers, double shell_thickness)
{
    if (surface_layer >= layers.size())
        return 0.;
    const double          top = layers[surface_layer].print_z;
    std::optional<size_t> deepest;
    for (size_t i = surface_layer + 1; i-- > 0;) {
        if (int(surface_layer - i) < shell_layers || top - layers[i].print_z < shell_thickness - EPSILON)
            deepest = i;
        else
            break;
    }
    return deepest ? top - layers[*deepest].bottom_z : 0.;
}

double bottom_shell_depth(const std::vector<LayerGeometry> &layers, size_t surface_layer, int shell_layers, double shell_thickness)
{
    if (surface_layer >= layers.size())
        return 0.;
    const double          bottom = layers[surface_layer].bottom_z;
    std::optional<size_t> highest;
    for (size_t i = surface_layer; i < layers.size(); ++i) {
        if (int(i - surface_layer) < shell_layers || layers[i].bottom_z - bottom < shell_thickness - EPSILON)
            highest = i;
        else
            break;
    }
    return highest ? layers[*highest].print_z - bottom : 0.;
}

namespace {

Polylines clip(const Polylines &lines, const ExPolygons &area)
{
    return lines.empty() || area.empty() ? Polylines() : intersection_pl(lines, area);
}

// Pieces of `lines` whose whole line width stands over the inside of `neighbour`, clear of its walls.
Polylines standing_on(const Polylines &lines, const LayerGeometry &neighbour, coord_t line_width)
{
    const float half_width = float(line_width) / 2.f;
    return clip(lines, diff_ex(offset_ex(neighbour.outline, -half_width), offset_ex(neighbour.walls, half_width)));
}

// Keeps the pieces of `lines`, an outer wall line at `layer`, where the wall continues strictly steeper than 45 degrees
// in `direction` (+1 up, -1 down) until it is `extension` tall. The line lies line_width / 2 inside its own outline. Where
// the wall has moved sideways by d over a height dz (d > 0 leaning out), the line lies line_width / 2 + d inside the
// outline of that layer, so "steeper than 45 degrees" keeps the depth strictly between line_width / 2 - dz and
// line_width / 2 + dz, less the slicing tolerance.
Polylines steep_for_extension(Polylines lines, const std::vector<LayerGeometry> &layers, size_t layer, int direction,
                              const JointSettings &settings)
{
    const LayerGeometry &joint      = layers[layer];
    const coord_t        half_width = settings.line_width / 2;
    for (size_t j = 1; !lines.empty(); ++j) {
        const double height = direction > 0 ? layers[layer + j - 1].print_z - joint.bottom_z :
                                              joint.print_z - layers[layer - j + 1].bottom_z;
        if (height >= settings.extension - EPSILON)
            return lines;
        if (direction > 0 ? layer + j >= layers.size() : j > layer)
            return {};
        const LayerGeometry &next       = layers[direction > 0 ? layer + j : layer - j];
        const coord_t        dz         = scaled<coord_t>(std::abs(next.print_z - joint.print_z));
        const coord_t        deepest    = half_width + dz - settings.tolerance;
        const coord_t        shallowest = half_width - dz + settings.tolerance;
        if (deepest <= shallowest)
            return {};
        lines = clip(lines, diff_ex(offset_ex(next.outline, -float(shallowest)), offset_ex(next.outline, -float(deepest))));
    }
    return {};
}

void drop_shorter_than(Polylines &lines, coord_t min_length)
{
    lines.erase(std::remove_if(lines.begin(), lines.end(), [min_length](const Polyline &line) { return line.length() < min_length; }),
                lines.end());
}

} // namespace

std::vector<Joint> find_joints(const std::vector<LayerGeometry> &layers, size_t layer, const Polylines &outer_wall_lines,
                               const JointSettings &settings)
{
    std::vector<Joint> joints;
    if (settings.extension <= 0. || settings.line_width <= 0 || layer >= layers.size() || outer_wall_lines.empty())
        return joints;
    // Only the outer wall: its centreline lies within one line width of the layer's outline.
    const LayerGeometry &here  = layers[layer];
    const Polylines      outer = clip(outer_wall_lines, diff_ex(here.outline, offset_ex(here.outline, -float(settings.line_width))));
    if (outer.empty())
        return joints;

    if (layer >= 1) {
        Polylines lines = steep_for_extension(standing_on(outer, layers[layer - 1], settings.line_width), layers, layer, +1, settings);
        drop_shorter_than(lines, settings.line_width);
        if (!lines.empty())
            joints.push_back({ layer, JointSide::Top, std::move(lines),
                               top_shell_depth(layers, layer - 1, settings.top_shell_layers, settings.top_shell_thickness),
                               settings.extension, settings.line_width });
    }
    if (layer + 1 < layers.size()) {
        Polylines lines = steep_for_extension(standing_on(outer, layers[layer + 1], settings.line_width), layers, layer, -1, settings);
        drop_shorter_than(lines, settings.line_width);
        if (!lines.empty())
            joints.push_back({ layer, JointSide::Bottom, std::move(lines),
                               bottom_shell_depth(layers, layer + 1, settings.bottom_shell_layers, settings.bottom_shell_thickness),
                               settings.extension, settings.line_width });
    }
    return joints;
}

} // namespace WallInternalExtension
} // namespace Slic3r
