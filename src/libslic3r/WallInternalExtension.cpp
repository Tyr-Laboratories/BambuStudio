#include "WallInternalExtension.hpp"

#include "Arachne/WallToolPaths.hpp"
#include "Arachne/utils/ExtrusionLine.hpp"
#include "ClipperUtils.hpp"
#include "Flow.hpp"
#include "Layer.hpp"
#include "PerimeterGenerator.hpp"
#include "Print.hpp"
#include "libslic3r.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <map>
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

// Joins pieces whose ends meet: clipping a closed line also cuts it at its first point, and a wall arrives as several
// paths sharing their ends, so a run of the wall across such a point would otherwise come back in parts.
void join_touching_pieces(Polylines &pieces)
{
    if (pieces.size() < 2)
        return;
    auto key = [](const Point &p) { return std::make_pair(p.x(), p.y()); };
    std::map<std::pair<coord_t, coord_t>, size_t> by_start;
    for (size_t i = 0; i < pieces.size(); ++i)
        by_start.emplace(key(pieces[i].first_point()), i);
    std::vector<bool> has_predecessor(pieces.size(), false), used(pieces.size(), false);
    for (size_t i = 0; i < pieces.size(); ++i)
        if (auto it = by_start.find(key(pieces[i].last_point())); it != by_start.end() && it->second != i)
            has_predecessor[it->second] = true;
    Polylines joined;
    auto chain_from = [&](size_t i) {
        Polyline chain = std::move(pieces[i]);
        used[i]        = true;
        for (;;) {
            auto it = by_start.find(key(chain.last_point()));
            if (it == by_start.end() || used[it->second])
                break;
            used[it->second] = true;
            const Polyline &next = pieces[it->second];
            chain.points.insert(chain.points.end(), next.points.begin() + 1, next.points.end());
        }
        joined.emplace_back(std::move(chain));
    };
    // Open chains first, from pieces nothing leads into; whatever is left forms closed chains.
    for (size_t i = 0; i < pieces.size(); ++i)
        if (!used[i] && !has_predecessor[i])
            chain_from(i);
    for (size_t i = 0; i < pieces.size(); ++i)
        if (!used[i])
            chain_from(i);
    pieces = std::move(joined);
}

Polylines clip(const Polylines &lines, const ExPolygons &area)
{
    if (lines.empty() || area.empty())
        return {};
    Polylines pieces = intersection_pl(lines, area);
    join_touching_pieces(pieces);
    return pieces;
}

// The walls of layers[n] that carry a wall line standing on them, so that it forms no joint: the outer walls, and the
// inner walls continuing a wall of layers[n + beyond], the next layer away from the joint. An inner wall lying on fill
// is part of the surface: "only one wall on top surfaces" puts one layer of such a wall around the area under a part
// rising from a top surface, right under that part's outer wall.
ExPolygons carrying_walls(const std::vector<LayerGeometry> &layers, size_t n, int beyond)
{
    const LayerGeometry &here = layers[n];
    if (beyond < 0 ? n == 0 : n + 1 >= layers.size())
        return here.outer_walls;
    ExPolygons out = intersection_ex(diff_ex(here.walls, here.outer_walls), layers[beyond < 0 ? n - 1 : n + 1].walls);
    append(out, here.outer_walls);
    return union_ex(out);
}

// Pieces of `lines` whose whole line width stands over the inside of layers[n], clear of the walls that carry them.
Polylines standing_on(const Polylines &lines, const std::vector<LayerGeometry> &layers, size_t n, int beyond, coord_t line_width)
{
    const float half_width = float(line_width) / 2.f;
    return clip(lines, diff_ex(offset_ex(layers[n].outline, -half_width), offset_ex(carrying_walls(layers, n, beyond), half_width)));
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
        // Even an extension of one layer needs the wall to go on steeply: a gently sloped skin steps in on every layer.
        if (j > 1 && height >= settings.extension - EPSILON)
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
        Polylines lines = steep_for_extension(standing_on(outer, layers, layer - 1, -1, settings.line_width), layers, layer, +1, settings);
        drop_shorter_than(lines, settings.line_width);
        if (!lines.empty())
            joints.push_back({ layer, JointSide::Top, std::move(lines),
                               top_shell_depth(layers, layer - 1, settings.top_shell_layers, settings.top_shell_thickness),
                               settings.extension, settings.line_width, settings.tolerance });
    }
    if (layer + 1 < layers.size()) {
        Polylines lines = steep_for_extension(standing_on(outer, layers, layer + 1, +1, settings.line_width), layers, layer, -1, settings);
        drop_shorter_than(lines, settings.line_width);
        if (!lines.empty())
            joints.push_back({ layer, JointSide::Bottom, std::move(lines),
                               bottom_shell_depth(layers, layer + 1, settings.bottom_shell_layers, settings.bottom_shell_thickness),
                               settings.extension, settings.line_width, settings.tolerance });
    }
    return joints;
}

namespace {

// Area within `delta` of `lines`. Closed loops (first point repeated) stay closed; open pieces end flat at their ends.
Polygons band(const Polylines &lines, float delta, ClipperLib::JoinType join, double limit)
{
    Polylines open, closed;
    for (const Polyline &line : lines) {
        if (line.points.size() > 2 && line.points.front() == line.points.back()) {
            Polyline loop = line;
            loop.points.pop_back();
            closed.emplace_back(std::move(loop));
        } else if (line.points.size() >= 2)
            open.emplace_back(line);
    }
    Polygons out;
    if (!open.empty())
        append(out, offset(open, delta, join, limit, ClipperLib::etOpenButt));
    if (!closed.empty())
        append(out, offset(closed, delta, join, limit, ClipperLib::etClosedLine));
    return out;
}

} // namespace

ExtensionArea extension_area(const std::vector<LayerGeometry> &layers, const std::vector<Joint> &joints, size_t target_layer)
{
    ExtensionArea area;
    if (target_layer >= layers.size())
        return area;
    const LayerGeometry &target = layers[target_layer];
    Polygons             all, hanging;
    for (const Joint &joint : joints) {
        if (joint.layer >= layers.size() || joint.lines.empty() || joint.line_width <= 0)
            continue;
        const LayerGeometry &surface = layers[joint.layer];
        const double         reach   = joint.shell_depth + joint.extension;
        double               chamfer = 0.;
        bool                 deepest = false;
        if (joint.side == JointSide::Top) {
            if (target_layer >= joint.layer)
                continue;
            const double depth = surface.bottom_z - target.bottom_z;
            if (depth > reach + EPSILON)
                continue;
            if (depth > joint.shell_depth + EPSILON)
                chamfer = std::clamp(target.print_z - (surface.bottom_z - reach), 0., joint.extension);
            deepest = target_layer == 0 || surface.bottom_z - layers[target_layer - 1].bottom_z > reach + EPSILON;
        } else {
            if (target_layer <= joint.layer)
                continue;
            const double depth = target.print_z - surface.print_z;
            if (depth > reach + EPSILON)
                continue;
            if (depth > joint.shell_depth + EPSILON)
                chamfer = std::clamp(surface.print_z + reach - target.bottom_z, 0., joint.extension);
        }
        const float half_width = float(joint.line_width) / 2.f;
        // The line's own footprint keeps the sharp corners of the outer wall it continues.
        Polygons piece = band(joint.lines, half_width, ClipperLib::jtMiter, 3.);
        if (chamfer > 0.) {
            // The chamfer keeps a constant distance outside the wall, so its face stays at 45 degrees around corners too.
            // Without a slicing tolerance to bound the arc segments, corners are mitred instead.
            const float    reach_out = half_width + float(scale_(chamfer));
            const Polygons outer     = joint.tolerance > 0 ? band(joint.lines, reach_out, ClipperLib::jtRound, double(joint.tolerance)) :
                                                             band(joint.lines, reach_out, ClipperLib::jtMiter, 3.);
            // Measured from the line itself rather than the outline, so the chamfer always joins the line's footprint,
            // also where a one-line fin's line sits deeper than half a line width.
            append(piece, diff(outer, offset(surface.outline, -half_width)));
        }
        if (deepest)
            append(hanging, piece);
        append(all, std::move(piece));
    }
    area.all     = union_ex(all);
    area.hanging = union_ex(hanging);
    return area;
}

namespace {

bool is_outer_wall(const ExtrusionPath &path) { return path.role() == erExternalPerimeter || path.role() == erOverhangPerimeter; }

// Joins consecutive outer wall paths into lines; a loop made only of outer wall paths becomes one closed line.
void append_outer_wall_runs(const ExtrusionPaths &paths, bool closed, Polylines &out)
{
    Polyline run;
    bool     all_outer = true;
    for (const ExtrusionPath &path : paths) {
        if (!is_outer_wall(path)) {
            all_outer = false;
            if (run.points.size() >= 2)
                out.emplace_back(std::move(run));
            run.points.clear();
            continue;
        }
        for (const Point &point : path.polyline.points)
            if (run.points.empty() || run.points.back() != point)
                run.points.push_back(point);
    }
    if (closed && all_outer && run.points.size() > 2 && run.points.front() != run.points.back())
        run.points.push_back(run.points.front());
    if (run.points.size() >= 2)
        out.emplace_back(std::move(run));
}

// Calls visit(paths, closed) for every loop (closed), multi-path and single path in `entity`, at any depth.
template<typename Visit> void visit_path_runs(const ExtrusionEntity &entity, const Visit &visit)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        for (const ExtrusionEntity *child : collection->entities)
            visit_path_runs(*child, visit);
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
        visit(loop->paths, true);
    } else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
        visit(multipath->paths, false);
    } else if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) {
        visit(ExtrusionPaths{ *path }, false);
    }
}

// Area covered by the outer wall paths in `perimeters`.
Polygons outer_wall_cover(const ExtrusionEntityCollection &perimeters)
{
    Polygons out;
    visit_path_runs(perimeters, [&out](const ExtrusionPaths &paths, bool) {
        for (const ExtrusionPath &path : paths)
            if (is_outer_wall(path))
                path.polygons_covered_by_width(out, 0.f);
    });
    return out;
}

} // namespace

Polylines outer_wall_lines(const ExtrusionEntityCollection &perimeters)
{
    Polylines out;
    visit_path_runs(perimeters, [&out](const ExtrusionPaths &paths, bool closed) { append_outer_wall_runs(paths, closed, out); });
    return out;
}

ExtrusionEntityCollection extension_extrusions(const ExPolygon &area, const ExPolygons &bridge_area, const Flow &flow,
                                               const Flow &bridge_flow, double layer_height, size_t inset_count,
                                               const Arachne::WallToolPathsParams &params)
{
    ExtrusionEntityCollection out;
    const coord_t             spacing = flow.scaled_spacing();
    // WallToolPaths keeps a reference to its outline: it must outlive tool_paths.
    const Polygons            outline = to_polygons(area);
    Arachne::WallToolPaths    tool_paths(outline, spacing, spacing, inset_count, 0, layer_height, params);
    std::vector<Arachne::VariableWidthLines> tool_lines = tool_paths.getToolPaths();
    for (Arachne::VariableWidthLines &inset : tool_lines)
        for (Arachne::ExtrusionLine &line : inset) {
            if (line.junctions.size() < 2)
                continue;
            // As in process_arachne(): a closed line occasionally comes without its closing junction.
            if (line.is_closed && line.junctions.front().p != line.junctions.back().p)
                line.junctions.emplace_back(line.junctions.front());
            Polyline centreline;
            for (const Arachne::ExtrusionJunction &junction : line.junctions)
                centreline.points.emplace_back(junction.p);
            const double length = centreline.length();
            if (length <= 0.)
                continue;
            if (!bridge_area.empty() && total_length(intersection_pl(Polylines{ centreline }, bridge_area)) > 0.5 * length) {
                ExtrusionPath path(erWallExtension, bridge_flow.mm3_per_mm(), bridge_flow.width(), bridge_flow.height());
                path.polyline = std::move(centreline);
                path.set_customize_flag(CustomizeFlag::cfWallExtensionBridge);
                if (line.is_closed) {
                    ExtrusionLoop loop;
                    loop.paths.emplace_back(std::move(path));
                    out.append(std::move(loop));
                } else
                    out.append(std::move(path));
                continue;
            }
            ExtrusionPaths paths;
            extrusion_paths_append(paths, line, erWallExtension, flow);
            if (paths.empty())
                continue;
            if (line.is_closed) {
                out.append(ExtrusionLoop(std::move(paths)));
                continue;
            }
            // One open line gives connected paths; split where rounding left a gap, as process_arachne() does.
            ExtrusionMultiPath multipath;
            for (ExtrusionPath &path : paths) {
                if (!multipath.paths.empty() && multipath.paths.back().last_point() != path.first_point()) {
                    out.append(std::move(multipath));
                    multipath = ExtrusionMultiPath();
                }
                multipath.paths.emplace_back(std::move(path));
            }
            out.append(std::move(multipath));
        }
    return out;
}

void apply(PrintObject &object)
{
    LayerPtrs &layers = object.layers();
    for (Layer *layer : layers)
        for (LayerRegion *region : layer->regions())
            region->wall_extension_area.clear();

    const Print &print = *object.print();
    const auto throw_if_canceled = [&print]() {
        if (print.canceled())
            throw CanceledException();
    };
    if (print.config().spiral_mode.value || layers.size() < 3)
        return;
    double extension_max = 0.;
    for (size_t region_id = 0; region_id < object.num_printing_regions(); ++region_id)
        extension_max = std::max(extension_max, object.printing_region(region_id).config().wall_internal_extension.value);
    if (extension_max <= 0.)
        return;
    // Straight walls move between layers only by contour simplification, so the steepness test allows for the G-code
    // resolution. slice_closing_radius closes gaps without moving straight walls.
    const coord_t tolerance = scaled<coord_t>(print.config().resolution.value);

    // Every layer's outline and walls, read before any layer is changed.
    std::vector<LayerGeometry> geometry(layers.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, layers.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++i) {
            const Layer   &layer = *layers[i];
            LayerGeometry &g     = geometry[i];
            g.bottom_z = layer.bottom_z();
            g.print_z  = layer.print_z;
            g.outline  = layer.lslices;
            Polygons walls, outer_walls;
            for (const LayerRegion *region : layer.regions()) {
                polygons_append(walls, region->perimeters.polygons_covered_by_width());
                polygons_append(outer_walls, outer_wall_cover(region->perimeters));
            }
            g.walls       = union_ex(walls);
            g.outer_walls = union_ex(outer_walls);
        }
    });
    throw_if_canceled();

    // Joints. Regions that differ only in wall_internal_extension share one perimeter group, whose walls all sit in its
    // first region, so every outer wall line of the layer is split among the regions by their slices.
    std::vector<std::vector<Joint>> joints(layers.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, layers.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++i) {
            const Layer &layer = *layers[i];
            Polylines    lines;
            for (const LayerRegion *region : layer.regions())
                append(lines, outer_wall_lines(region->perimeters));
            if (lines.empty())
                continue;
            for (const LayerRegion *region : layer.regions()) {
                const PrintRegionConfig &config = region->region().config();
                if (config.wall_internal_extension.value <= 0. || region->slices.empty())
                    continue;
                const Polylines own = intersection_pl(lines, to_expolygons(region->slices.surfaces));
                if (own.empty())
                    continue;
                JointSettings settings;
                settings.extension              = config.wall_internal_extension.value;
                settings.top_shell_layers       = config.top_shell_layers.value;
                settings.top_shell_thickness    = config.top_shell_thickness.value;
                settings.bottom_shell_layers    = config.bottom_shell_layers.value;
                settings.bottom_shell_thickness = config.bottom_shell_thickness.value;
                settings.line_width             = region->flow(frExternalPerimeter).scaled_width();
                settings.tolerance              = tolerance;
                append(joints[i], find_joints(geometry, i, own, settings));
            }
        }
    });
    throw_if_canceled();

    double reach_max = 0.;
    for (const std::vector<Joint> &layer_joints : joints)
        for (const Joint &joint : layer_joints)
            reach_max = std::max(reach_max, joint.shell_depth + joint.extension);
    if (reach_max <= 0.)
        return;

    // The area each region prints on each layer: inside the part with material directly above and below (never on a
    // surface or over air), inside the region's fill area, and only where the region itself has the setting above 0.
    std::vector<std::vector<ExPolygons>> areas(layers.size());
    std::vector<ExPolygons>              hanging(layers.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(1, layers.size() - 1), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t l = range.begin(); l < range.end(); ++l) {
            std::vector<Joint> nearby;
            for (size_t n = l + 1; n < layers.size() && geometry[n].bottom_z - geometry[l].bottom_z <= reach_max + EPSILON; ++n)
                append(nearby, joints[n]);
            for (size_t n = l; n-- > 0 && geometry[l].print_z - geometry[n].print_z <= reach_max + EPSILON;)
                append(nearby, joints[n]);
            if (nearby.empty())
                continue;
            ExtensionArea area = extension_area(geometry, nearby, l);
            if (area.all.empty())
                continue;
            area.all = intersection_ex(area.all, intersection_ex(layers[l - 1]->lslices, layers[l + 1]->lslices));
            const LayerRegionPtrs &regions = layers[l]->regions();
            areas[l].assign(regions.size(), ExPolygons());
            for (size_t r = 0; r < regions.size(); ++r)
                if (regions[r]->region().config().wall_internal_extension.value > 0.)
                    areas[l][r] = intersection_ex(area.all, regions[r]->fill_no_overlap_expolygons);
            hanging[l] = std::move(area.hanging);
        }
    });
    throw_if_canceled();

    // Lines and cut-outs. Each layer only changes its own regions.
    const Arachne::WallToolPathsParams params = make_wall_tool_paths_params(object.config(), print.config());
    tbb::parallel_for(tbb::blocked_range<size_t>(1, layers.size() - 1), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t l = range.begin(); l < range.end(); ++l) {
            if (areas[l].empty())
                continue;
            throw_if_canceled();
            // What lines on this layer stand on: the walls and the extension lines of the layer below.
            Polygons support_below = to_polygons(geometry[l - 1].walls);
            for (const ExPolygons &below : areas[l - 1])
                polygons_append(support_below, to_polygons(below));
            const LayerRegionPtrs &regions = layers[l]->regions();
            for (size_t r = 0; r < regions.size() && r < areas[l].size(); ++r) {
                const ExPolygons &area = areas[l][r];
                if (area.empty())
                    continue;
                LayerRegion &region      = *regions[r];
                const Flow   flow        = region.flow(frExternalPerimeter);
                const Flow   bridge_flow = region.bridging_flow(frPerimeter, object.config().thick_bridges);
                // Only a top joint's deepest layer can hang over the sparse infill.
                const ExPolygons bridge_area = diff_ex(intersection_ex(hanging[l], area),
                                                       offset(support_below, float(flow.scaled_spacing()) / 2.f));
                const size_t inset_count = size_t(std::ceil(double(flow.scaled_width() + scaled<coord_t>(extension_max)) /
                                                            double(flow.scaled_spacing()))) + 1;
                for (const ExPolygon &piece : area) {
                    ExtrusionEntityCollection lines = extension_extrusions(piece, bridge_area, flow, bridge_flow, layers[l]->height,
                                                                           inset_count, params);
                    if (!lines.entities.empty())
                        region.perimeters.append(std::move(lines));
                }
                // Fill meets the extension lines at normal line spacing, as neighbouring solid lines meet.
                region.fill_expolygons            = diff_ex(region.fill_expolygons, area);
                region.fill_no_overlap_expolygons = diff_ex(region.fill_no_overlap_expolygons, area);
                region.bridge_fill_expolygons     = diff_ex(region.bridge_fill_expolygons, area);
                region.fill_surfaces.set(region.fill_expolygons, stInternal);
                region.wall_extension_area        = area;
            }
        }
    });
    throw_if_canceled();
}

} // namespace WallInternalExtension
} // namespace Slic3r
