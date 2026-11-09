#ifndef slic3r_tests_bridge_aware_cubic_observer_hpp_
#define slic3r_tests_bridge_aware_cubic_observer_hpp_

// The export observer of the Bridge aware cubic tests: reads the exported G-code back as extrusions with their roles, widths
// and heights, and measures the contract on them (rewrite architecture 5.1 applied to the export, 7.2). It shares no code
// with the pattern. Carried over from the branch this pattern replaces (ef9c17704, test_bridge_aware_cubic_export.cpp) as
// 7.2 lists, with measure_first_roof generalised to every plane and several islands (measure_planes).

#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Print.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace BACObserver {

using namespace Slic3r;

// The fixture's settings with what the observer needs of the export: relative E, planar seams and no arc fitting, so that every
// extrusion is a G1 move on its layer's plane, no skirt and no machine G-code.
inline DynamicPrintConfig observed(DynamicPrintConfig config)
{
    config.set_deserialize_strict({
        {"skirt_loops", "0"}, {"brim_type", "no_brim"}, {"enable_arc_fitting", "0"}, {"seam_gap", "0"},
        {"override_filament_scarf_seam_setting", "1"}, {"seam_slope_type", "none"}, {"apply_scarf_seam_on_circles", "0"},
        {"spiral_mode", "0"}, {"use_relative_e_distances", "1"}, {"slow_down_for_layer_cooling", "0"},
        {"machine_start_gcode", ""}, {"machine_end_gcode", ""}, {"before_layer_change_gcode", ""}, {"layer_change_gcode", ""}
    });
    return config;
}

inline bool model_role(ExtrusionRole role)
{
    return role == erPerimeter || role == erExternalPerimeter || role == erOverhangPerimeter ||
        role == erInternalInfill || role == erSolidInfill || role == erTopSolidInfill ||
        role == erBottomSurface || role == erBridgeInfill || role == erGapFill || role == erFloatingVerticalShell;
}

// Recover the actual decimal coordinates, rather than retaining the reader's float conversion error.
inline double emitted_coordinate(double value) { return std::round(value * 1000.) / 1000.; }
inline Point emitted_point(double x, double y)
{
    return Point(coord_t(std::llround(scale_(emitted_coordinate(x)))), coord_t(std::llround(scale_(emitted_coordinate(y)))));
}
inline int plane_key(double z) { return int(std::lround(z * 1000.)); }

inline double exact_axis_value(const GCodeReader::GCodeLine &line, char axis)
{
    std::istringstream words(line.raw().substr(0, line.raw().find(';')));
    std::string word;
    while (words >> word)
        if (word.size() > 1 && word.front() == axis) return std::stod(word.substr(1));
    FAIL("A parsed axis must have a decimal token in its G-code line.");
    return 0.;
}

struct EmittedPath {
    Polyline path;
    ExtrusionRole role = erNone;
    double z = 0., width = 0., height = 0.;
    size_t first_line = 0, last_line = 0;
};

inline std::vector<EmittedPath> read_extrusions(const std::string &filename, const DynamicPrintConfig &config)
{
    std::vector<EmittedPath> paths;
    GCodeReader reader;
    reader.apply_config(config);
    const auto role_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role);
    const auto width_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width);
    const auto height_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height);
    ExtrusionRole role = erNone;
    double width = 0., height = 0.;
    size_t line_number = 0;
    bool continuous = false;
    bool relative_e = config.opt_bool("use_relative_e_distances");
    double exact_z = 0., exact_e = 0.;
    REQUIRE(reader.parse_file(filename, [&](GCodeReader &before, const GCodeReader::GCodeLine &line) {
        ++line_number;
        const bool motion = line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3");
        if (line.cmd_is("M82")) relative_e = false;
        if (line.cmd_is("M83")) relative_e = true;
        if (line.has_z() && (motion || line.cmd_is("G92")))
            exact_z = exact_axis_value(line, 'Z');
        double extrusion_delta = 0.;
        if (line.has_e() && (motion || line.cmd_is("G92"))) {
            const double next_e = exact_axis_value(line, 'E');
            if (motion) extrusion_delta = relative_e ? next_e : next_e - exact_e;
            exact_e = motion && relative_e ? exact_e + next_e : next_e;
        }
        const std::string_view comment = line.comment();
        if (comment.substr(0, role_tag.size()) == role_tag)
            role = ExtrusionEntity::string_to_role(comment.substr(role_tag.size()));
        if (comment.substr(0, width_tag.size()) == width_tag)
            width = std::stod(std::string(comment.substr(width_tag.size())));
        if (comment.substr(0, height_tag.size()) == height_tag)
            height = std::stod(std::string(comment.substr(height_tag.size())));

        const bool depositing = motion && extrusion_delta > 0. && line.dist_XY(before) > 0.;
        if (!depositing || !model_role(role)) {
            if (motion && (line.dist_XY(before) > 0. || line.dist_Z(before) != 0. || extrusion_delta != 0.))
                continuous = false;
            return;
        }
        REQUIRE(line.cmd_is("G1"));
        REQUIRE(width > 0.);
        REQUIRE(height > 0.);
        const Point from = emitted_point(before.x(), before.y());
        const Point to = emitted_point(line.new_X(before), line.new_Y(before));
        const double z = exact_z;
        if (from == to) return;
        if (continuous && !paths.empty() && paths.back().role == role && paths.back().z == z &&
            paths.back().width == width && paths.back().height == height && paths.back().path.last_point() == from) {
            paths.back().path.points.push_back(to);
            paths.back().last_line = line_number;
        } else {
            paths.push_back({Polyline(Points{from, to}), role, z, width, height, line_number, line_number});
        }
        continuous = true;
    }));
    return paths;
}

using EmittedLayer = std::vector<const EmittedPath *>;

inline ExPolygons footprint(const EmittedLayer &paths, ExtrusionRole role = erNone)
{
    Polygons strokes;
    for (const EmittedPath *path : paths)
        if (role == erNone || path->role == role)
            append(strokes, offset(path->path, float(scale_(path->width * .5)), ClipperLib::jtRound, scale_(.001)));
    return union_ex(strokes);
}

// The emitted paths by deposition plane.
inline std::map<int, EmittedLayer> emitted_layers(const std::vector<EmittedPath> &paths)
{
    std::map<int, EmittedLayer> layers;
    for (const EmittedPath &path : paths) layers[plane_key(path.z)].push_back(&path);
    return layers;
}

// The G-code that a fixture exports, as a string, without the lines that name the object by its id in the process
// ("; OBJECT_ID:"), which grows with every model that the process makes.
inline std::string exported_gcode(const BACFixtures::Workspace &workspace, Model model, const DynamicPrintConfig &settings,
                                  const std::string &name)
{
    Print print;
    BACFixtures::slice(print, model, observed(settings));
    const std::string filename = print.export_gcode((workspace.path / name).string(), nullptr);
    std::ifstream file(filename, std::ios::binary);
    std::string out, line;
    while (std::getline(file, line))
        if (line.rfind("; OBJECT_ID:", 0) != 0) {
            out += line;
            out += '\n';
        }
    return out;
}

// Slices the model with the settings into the print, exports it into the workspace and reads the extrusions back.
inline std::vector<EmittedPath> exported_extrusions(Print &print, const BACFixtures::Workspace &workspace, Model model,
                                                    const DynamicPrintConfig &settings, const std::string &name)
{
    const DynamicPrintConfig config = observed(settings);
    BACFixtures::slice(print, model, config);
    const std::string filename = print.export_gcode((workspace.path / name).string(), nullptr);
    return read_extrusions(filename, config);
}

inline std::vector<EmittedPath> exported_extrusions(const BACFixtures::Workspace &workspace, Model model, const DynamicPrintConfig &settings,
                                                    const std::string &name)
{
    Print print;
    return exported_extrusions(print, workspace, std::move(model), settings, name);
}

// A place (mm, on the bed) where a sparse infill path of the export leaves the area the stock pipeline gives its layer's infill:
// the regions' fill areas, which reach into the walls by the infill/wall overlap, grown by the print's resolution, by which the
// path simplification may move a vertex. Cubic's lines end half a spacing inside it.
inline std::optional<Vec2d> sparse_outside_fill_area(const Print &print, const std::vector<EmittedPath> &paths)
{
    std::map<int, ExPolygons> areas;
    for (const PrintObject *object : print.objects())
        for (const PrintInstance &instance : object->instances())
            for (const Layer *layer : object->layers())
                for (const LayerRegion *region : layer->regions())
                    for (ExPolygon area : region->fill_expolygons) {
                        area.translate(instance.shift);
                        areas[plane_key(layer->print_z)].push_back(std::move(area));
                    }
    for (auto &[key, area] : areas)
        area = offset_ex(union_ex(area), float(scale_(print.config().resolution.value)));
    std::map<int, Polylines> sparse;
    for (const EmittedPath &path : paths)
        if (path.role == erInternalInfill)
            sparse[plane_key(path.z)].push_back(path.path);
    for (const auto &[key, lines] : sparse) {
        const auto it = areas.find(key);
        for (const Polyline &piece : it == areas.end() ? lines : diff_pl(lines, it->second))
            if (piece.length() > SCALED_EPSILON)
                return unscale(piece.last_point());
    }
    return std::nullopt;
}

inline double diameter_mm(const Points &points)
{
    double best = 0.;
    for (size_t i = 0; i < points.size(); ++i)
        for (size_t j = i + 1; j < points.size(); ++j)
            best = std::max(best, (points[i] - points[j]).cast<double>().norm());
    return unscale<double>(best);
}

// A strip's length, L(X) = per(X)/2 − 4 area(X)/per(X) (the architecture's appendix A).
inline double strip_length_mm(const ExPolygon &shape)
{
    double perimeter = shape.contour.length();
    for (const Polygon &hole : shape.holes) perimeter += hole.length();
    if (perimeter <= 0.) return 0.;
    return unscale<double>(0.5 * perimeter - 4. * std::abs(shape.area()) / perimeter);
}

struct PlaneMeasure {
    int    lower = 0, upper = 0;             // plane keys (z in µm) of the plane below and of the roof
    double gap = 0.;                         // G the measure compares to
    double bead = 0.;                        // r: the narrowest roof bead over the void
    size_t openings = 0, runs = 0;
    double widest_opening = 0., longest_run = 0.;
    Vec2d  where_widest = Vec2d::Zero(), where_longest = Vec2d::Zero();   // mm, on the bed
    size_t bridge_free_ends = 0, other_free_ends = 0;
    std::vector<Vec2d> where_free;           // mm, on the bed: the free bridge ends
    ExPolygons demand, components;           // scaled, on the bed: the roof's demand and the opening components (for drawing)
};

// Every pair of consecutive emitted planes where non-sparse material of the upper lies over the sparse void of the lower beyond
// half the nozzle from what the lower prints, measured as the contract measures it: the openings of the void under the roof
// (components of opening(void, r/2) ∩ offset(roof, r/2)), the unsupported runs of the roof's paths over the void, and the free
// ends of open paths over the void by role. Reads only the G-code.
inline std::vector<PlaneMeasure> measure_planes(const std::vector<EmittedPath> &paths, double nozzle_mm, double gap_mm)
{
    std::vector<PlaneMeasure> out;
    const std::map<int, EmittedLayer> layers = emitted_layers(paths);
    const double h_n = 0.5 * nozzle_mm;
    for (auto it = layers.begin(); it != layers.end() && std::next(it) != layers.end(); ++it) {
        const EmittedLayer &lower = it->second, &upper = std::next(it)->second;
        EmittedLayer sparse;
        for (const EmittedPath *path : lower)
            if (path->role == erInternalInfill) sparse.push_back(path);
        if (sparse.empty()) continue;
        const ExPolygons printed = footprint(lower);
        // The domain: inside the outer wall of the plane below, or, without walls, the filled outline of what it prints.
        ExPolygons domain;
        for (const ExPolygon &wall : footprint(lower, erExternalPerimeter)) domain.emplace_back(wall.contour);
        if (domain.empty())
            for (const ExPolygon &part : printed) domain.emplace_back(part.contour);
        const ExPolygons near_sparse = offset_ex(footprint(sparse), float(scale_(.01)));
        ExPolygons sparse_void;
        for (ExPolygon &part : diff_ex(domain, printed))
            if (!intersection_ex(ExPolygons{part}, near_sparse).empty()) sparse_void.push_back(std::move(part));
        if (sparse_void.empty()) continue;
        // The roof: non-sparse paths of the upper plane over the void beyond the overhang distance from what surely prints.
        EmittedLayer roof_paths;
        for (const EmittedPath *path : upper)
            if (path->role != erInternalInfill) roof_paths.push_back(path);
        if (roof_paths.empty()) continue;
        const ExPolygons supported = offset_ex(printed, float(scale_(h_n)), ClipperLib::jtRound, scale_(.001));
        ExPolygons demand = intersection_ex(diff_ex(footprint(roof_paths), supported), sparse_void);
        demand = opening_ex(demand, float(scale_(.002)), ClipperLib::jtRound, scale_(.001));
        if (demand.empty()) continue;
        PlaneMeasure measure;
        measure.lower = it->first;
        measure.upper = std::next(it)->first;
        measure.gap = gap_mm;
        const ExPolygons near_demand = offset_ex(demand, float(scale_(.01)));
        double bead = std::numeric_limits<double>::max();
        EmittedLayer roof;
        for (const EmittedPath *path : roof_paths)
            if (!intersection_pl(Polylines{path->path}, offset_ex(near_demand, float(scale_(path->width * .5)))).empty()) {
                roof.push_back(path);
                bead = std::min(bead, path->width);
            }
        if (roof.empty()) continue;
        measure.bead = bead;
        // Openings.
        const ExPolygons opened = opening_ex(sparse_void, float(scale_(.5 * bead)), ClipperLib::jtRound, scale_(.001));
        measure.demand = demand;
        for (const ExPolygon &component : intersection_ex(opened, offset_ex(demand, float(scale_(.5 * bead)), ClipperLib::jtRound, scale_(.001)))) {
            measure.components.push_back(component);
            ++measure.openings;
            const double span = std::max(diameter_mm(Geometry::convex_hull(component.contour.points).points), strip_length_mm(component));
            if (span > measure.widest_opening) {
                measure.widest_opening = span;
                measure.where_widest = unscale(component.contour.centroid());
            }
        }
        // Runs: the roof paths clipped to the void beyond the overhang distance (and the touching margin) of what prints below.
        const ExPolygons off_support = diff_ex(domain, offset_ex(printed, float(scale_(h_n + .01)), ClipperLib::jtRound, scale_(.001)));
        const ExPolygons walls_above = footprint(upper, erExternalPerimeter);
        ExPolygons wall_overlap = walls_above;
        append(wall_overlap, footprint(upper, erPerimeter));
        wall_overlap = union_ex(wall_overlap);
        for (const EmittedPath *path : roof) {
            for (const Polyline &run : diff_pl(Polylines{path->path}, offset_ex(printed, float(scale_(h_n + .01)), ClipperLib::jtRound, scale_(.001)))) {
                if (intersection_pl(Polylines{run}, sparse_void).empty()) continue;
                ++measure.runs;
                if (unscale<double>(run.length()) > measure.longest_run) {
                    measure.longest_run = unscale<double>(run.length());
                    measure.where_longest = unscale(run.points[run.points.size() / 2]);
                }
            }
            // Free ends: an open path's end over the void, off the support and outside its own layer's wall overlap.
            if (path->path.first_point() == path->path.last_point()) continue;
            const ExPolygons support = offset_ex(printed, float(scale_(.5 * path->width + .01)), ClipperLib::jtRound, scale_(.001));
            const ExPolygons anchored_by_wall = offset_ex(wall_overlap, float(scale_(.5 * path->width + .01)), ClipperLib::jtRound, scale_(.001));
            for (const Point &end : { path->path.first_point(), path->path.last_point() }) {
                const auto inside = [&end](const ExPolygons &area) {
                    return std::any_of(area.begin(), area.end(), [&end](const ExPolygon &part) { return part.contains(end); });
                };
                if (inside(sparse_void) && !inside(support) && !inside(anchored_by_wall)) {
                    if (path->role == erBridgeInfill) {
                        ++measure.bridge_free_ends;
                        measure.where_free.push_back(unscale(end));
                    } else
                        ++measure.other_free_ends;
                }
            }
        }
        (void)off_support;
        out.push_back(std::move(measure));
    }
    return out;
}

struct BridgeEnds {
    size_t planes = 0;    // planes with bridge infill over a plane with sparse infill
    size_t paths = 0;     // the bridge paths of those planes
    size_t hanging = 0;   // the ends of those paths over the previous plane's sparse void, not anchored by a wall of their own plane
    std::vector<std::pair<double, Vec2d>> where;   // z and place (mm, on the bed) of each hanging end
};

// Counts, on the exported G-code alone, the ends of the bridge paths that hang over the sparse void of the plane below: inside
// the outer wall of that plane, farther than half the path's width from everything it prints, in a part of what it leaves
// unprinted that its sparse infill borders, and outside the wall overlap of the bridge's own plane.
inline BridgeEnds count_bridge_ends(const std::vector<EmittedPath> &paths)
{
    BridgeEnds result;
    const std::map<int, EmittedLayer> layers = emitted_layers(paths);
    for (auto it = layers.begin(); it != layers.end() && std::next(it) != layers.end(); ++it) {
        const EmittedLayer &lower = it->second, &upper = std::next(it)->second;
        EmittedLayer sparse, bridges;
        for (const EmittedPath *path : lower)
            if (path->role == erInternalInfill) sparse.push_back(path);
        for (const EmittedPath *path : upper)
            if (path->role == erBridgeInfill) bridges.push_back(path);
        if (sparse.empty() || bridges.empty()) continue;
        ++result.planes;
        ExPolygons domain;
        for (const ExPolygon &wall : footprint(lower, erExternalPerimeter)) domain.emplace_back(wall.contour);
        const ExPolygons printed = footprint(lower);
        const ExPolygons near_sparse = offset_ex(footprint(sparse), float(scale_(.01)));
        ExPolygons sparse_void;
        for (ExPolygon &part : diff_ex(domain, printed))
            if (!intersection_ex(ExPolygons{part}, near_sparse).empty()) sparse_void.push_back(std::move(part));
        ExPolygons walls = footprint(upper, erExternalPerimeter);
        append(walls, footprint(upper, erPerimeter));
        walls = union_ex(walls);
        for (const EmittedPath *bridge : bridges) {
            ++result.paths;
            const ExPolygons support = offset_ex(printed, float(scale_(.5 * bridge->width - .001)), ClipperLib::jtRound, scale_(.001));
            const ExPolygons anchored = offset_ex(walls, float(scale_(.5 * bridge->width + .01)), ClipperLib::jtRound, scale_(.001));
            for (const Point &end : { bridge->path.first_point(), bridge->path.last_point() }) {
                const auto inside = [&end](const ExPolygons &area) {
                    return std::any_of(area.begin(), area.end(), [&end](const ExPolygon &part) { return part.contains(end); });
                };
                if (inside(sparse_void) && !inside(support) && !inside(anchored)) {
                    ++result.hanging;
                    result.where.emplace_back(bridge->z, unscale(end));
                }
            }
        }
    }
    return result;
}

// The sparse infill segments of every plane, each path simplified so that collinear vertices do not split a line, as ordered
// endpoint pairs with their plane key: for the comparisons of one export's sparse lines with another's.
struct SparseSegment { int plane; Point a, b; };
inline std::vector<SparseSegment> sparse_segments(const std::vector<EmittedPath> &paths)
{
    std::vector<SparseSegment> out;
    for (const EmittedPath &path : paths) {
        if (path.role != erInternalInfill) continue;
        Polyline simplified = path.path;
        simplified.simplify(scale_(.001));
        for (size_t i = 1; i < simplified.points.size(); ++i) {
            Point a = simplified.points[i - 1], b = simplified.points[i];
            if (a == b) continue;
            if (std::make_pair(a.x(), a.y()) > std::make_pair(b.x(), b.y())) std::swap(a, b);
            out.push_back({plane_key(path.z), a, b});
        }
    }
    return out;
}


// The sparse segments of a control export on one plane.
inline std::vector<Line> control_sparse_segments(const std::map<int, EmittedLayer> &control_layers, int plane)
{
    std::vector<Line> out;
    auto it = control_layers.find(plane);
    if (it == control_layers.end()) return out;
    for (const EmittedPath *path : it->second)
        if (path->role == erInternalInfill)
            for (size_t i = 1; i < path->path.points.size(); ++i)
                out.emplace_back(path->path.points[i - 1], path->path.points[i]);
    return out;
}

// Whether a segment lies on one of the control's lines: collinear within 0.5° and 0.02 mm. The extents are not compared: the two
// slices' islands differ where the bridge generator took sparse area, and a piece on a bulk line's line lies on that printed line.
inline bool on_control_line(const Point &pa, const Point &pb, const std::vector<Line> &control)
{
    const Vec2d a = pa.cast<double>(), b = pb.cast<double>();
    const Vec2d d = b - a;
    const double length = d.norm();
    if (length < 1.) return true;
    for (const Line &seg : control) {
        const Vec2d e = (seg.b - seg.a).cast<double>();
        const double el = e.norm();
        if (el < 1.) continue;
        if (std::abs(d.x() * e.y() - d.y() * e.x()) / (length * el) > std::sin(0.5 * M_PI / 180.)) continue;
        const Vec2d mid = 0.5 * (a + b);
        const double off = std::abs((mid.x() - seg.a.x()) * e.y() - (mid.y() - seg.a.y()) * e.x()) / el;
        if (off > scale_(.02)) continue;
        return true;
    }
    return false;
}

// The paths without the pattern's added sparse lines on the given planes: of a sparse path, only the runs of segments that lie on
// lines of the control remain (the connector chains added lines with bulk lines, so a chain is split). The negative control of
// the roof measure, which must then exceed the gap.
inline std::vector<EmittedPath> without_added_sparse(const std::vector<EmittedPath> &paths, const std::vector<EmittedPath> &control, const std::vector<int> &planes)
{
    const std::map<int, EmittedLayer> control_layers = emitted_layers(control);
    std::map<int, std::vector<Line>> control_segments;
    for (const int plane : planes) control_segments[plane] = control_sparse_segments(control_layers, plane);
    std::vector<EmittedPath> out;
    for (const EmittedPath &path : paths) {
        const int plane = plane_key(path.z);
        if (path.role != erInternalInfill || !control_segments.count(plane)) {
            out.push_back(path);
            continue;
        }
        EmittedPath run = path;
        run.path.points.clear();
        const auto flush = [&]() {
            if (run.path.points.size() >= 2) out.push_back(run);
            run.path.points.clear();
        };
        for (size_t i = 1; i < path.path.points.size(); ++i) {
            if (on_control_line(path.path.points[i - 1], path.path.points[i], control_segments[plane])) {
                if (run.path.points.empty()) run.path.points.push_back(path.path.points[i - 1]);
                run.path.points.push_back(path.path.points[i]);
            } else
                flush();
        }
        flush();
    }
    return out;
}

// K2 on the export: the pattern's added lines (sparse pieces not on a bulk line of the Cubic control: every segment collinear
// within 0.5° and 0.02 mm with one of its sparse segments on the same plane) that lie off the hold set of the plane below. The
// hold set reads the hold rule R of the architecture's 3.7 on the G-code: the centre line of a sparse bead within w/2 of any point
// of a lower sparse bead's centre line (own plane or crossing; round ends), or within w/2 + 0.01 mm of a lower non-sparse footprint.
// An unheld piece at an end of its path is a free end unless that end is fused alongside another bead of its own layer (the
// anchors of 9 November 2026: a nub along a perimeter or along the line the strand meets). An added piece off the hold set
// must be a stretch of at most `gap_b` between held points and must not reach an end of its path.
struct K2Measure {
    size_t added_pieces = 0;   // unheld pieces of added lines
    size_t free_ends = 0;      // those that reach an end of their path
    size_t long_stretches = 0; // those longer than gap_b between held points
    double longest = 0.;
    std::vector<Vec2d> where;  // mm: free ends and long stretches
};

// Whether every segment of a polyline lies on a control line.
inline bool on_control_lines(const Polyline &piece, const std::vector<Line> &control)
{
    for (size_t i = 1; i < piece.points.size(); ++i)
        if (!on_control_line(piece.points[i - 1], piece.points[i], control)) return false;
    return true;
}

// Whether a segment runs in one of the lattice directions (within 1.5°).
inline bool along_lattice(const Point &a, const Point &b, const std::vector<double> &directions)
{
    const double angle = std::fmod(std::atan2(double(b.y() - a.y()), double(b.x() - a.x())) * 180. / M_PI + 360., 180.);
    for (const double d : directions) {
        const double diff = std::abs(angle - d);
        if (std::min(diff, 180. - diff) <= 1.5) return true;
    }
    return false;
}

// Whether every segment of a piece is a join of the connector along the island boundary: off the lattice directions and on the
// line of a control segment (the control's own joins run along the same boundary). Cubic's boundary behaviour, accepted by the
// user on 9 November 2026, not a line of the pattern.
inline bool boundary_join(const Polyline &piece, const std::vector<Line> &control, const std::vector<double> &directions)
{
    for (size_t i = 1; i < piece.points.size(); ++i) {
        const Point &pa = piece.points[i - 1], &pb = piece.points[i];
        if (along_lattice(pa, pb, directions)) return false;
        const Vec2d a = pa.cast<double>(), d = (pb - pa).cast<double>();
        const double length = d.norm();
        if (length < 1.) continue;
        bool collinear = false;
        for (const Line &seg : control) {
            const Vec2d e = (seg.b - seg.a).cast<double>();
            const double el = e.norm();
            if (el < 1.) continue;
            if (std::abs(d.x() * e.y() - d.y() * e.x()) / (length * el) > std::sin(0.5 * M_PI / 180.)) continue;
            const Vec2d mid = 0.5 * (a + pb.cast<double>());
            if (std::abs((mid.x() - seg.a.x()) * e.y() - (mid.y() - seg.a.y()) * e.x()) / el > scale_(.02)) continue;
            collinear = true;
            break;
        }
        if (!collinear) return false;
    }
    return true;
}

// `gap_b_at` gives G_b (mm) at a place on the bed (mm) and a height z (mm): the fixtures with two regions have two gaps.
// `infill_directions_deg` are the regions' infill directions: the lattice directions of each are it and it + 60° and + 120°.
inline K2Measure measure_k2(const std::vector<EmittedPath> &paths, const std::vector<EmittedPath> &control,
                            const std::function<double(const Vec2d&, double)> &gap_b_at, const std::vector<double> &infill_directions_deg)
{
    K2Measure result;
    std::vector<double> directions;
    for (const double direction : infill_directions_deg)
        for (int k = 0; k < 3; ++k)
            directions.push_back(std::fmod(direction + 60. * k + 360., 180.));
    const std::map<int, EmittedLayer> layers = emitted_layers(paths), control_layers = emitted_layers(control);
    for (auto it = layers.begin(); it != layers.end() && std::next(it) != layers.end(); ++it) {
        const EmittedLayer &lower = it->second, &upper = std::next(it)->second;
        EmittedLayer lower_sparse, lower_other, sparse;
        for (const EmittedPath *path : lower) (path->role == erInternalInfill ? lower_sparse : lower_other).push_back(path);
        for (const EmittedPath *path : upper) if (path->role == erInternalInfill) sparse.push_back(path);
        if (sparse.empty()) continue;
        const std::vector<Line> control_segments = control_sparse_segments(control_layers, std::next(it)->first);
        const std::vector<Line> lower_control = control_sparse_segments(control_layers, it->first);
        std::map<double, ExPolygons> holds;    // by the upper bead's width
        const auto hold_of = [&](double w) -> const ExPolygons& {
            auto found = holds.find(w);
            if (found != holds.end()) return found->second;
            Polygons hold;
            // Rule R holds a point whose centre lies within w/2 of any point of the lower centre line: the union over the centre
            // line's segments of each segment grown by the disc (round ends). Segment by segment, because Clipper's offset of a
            // whole polyline puts no cap where a path folds back on itself.
            // A lower sparse segment off the lattice directions that is at least a bead long, or lies on a control line, is a
            // bead the pattern prints whole (a seam wall, a rail) or a connector's hook: material of its layer, which holds as the
            // walls and solid fills do, by its footprint and the touching measure (rule iii), not by its centre line. A short
            // off-lattice hop (a nub's step to its line, a join link) holds by its centre line like any sparse line.
            for (const EmittedPath *path : lower_sparse)
                for (size_t i = 1; i < path->path.points.size(); ++i)
                    if (path->path.points[i - 1] != path->path.points[i]) {
                        const Point &pa = path->path.points[i - 1], &pb = path->path.points[i];
                        const bool lattice = along_lattice(pa, pb, directions);
                        const bool bead = !lattice && (unscale<double>((pb - pa).cast<double>().norm()) >= path->width - 1e-4 || on_control_line(pa, pb, lower_control));
                        const double reach = bead ? .5 * path->width + .5 * w + .01 : .5 * w;
                        append(hold, offset(Polyline(path->path.points[i - 1], path->path.points[i]), float(scale_(reach)), ClipperLib::jtRound, scale_(.001), ClipperLib::etOpenRound));
                    }
            append(hold, to_polygons(offset_ex(footprint(lower_other), float(scale_(.5 * w + .01)), ClipperLib::jtRound, scale_(.001))));
            return holds.emplace(w, union_ex(hold)).first->second;
        };
        // An end whose bead overlaps another bead of its own layer (an anchor along a perimeter or along the line it meets) is
        // fused to it, not hanging: the beads' centre lines are within half their widths and the touching measure.
        // The distance from the end's centre to the nearest other bead's edge on this layer (negative when the end lies inside
        // that bead), or nothing when no other path comes within 2 mm.
        // Rule (iv) on the export: a vertex of a path is fused to another bead of its layer when it lies on that bead's centre line
        // (within half the bead's width and the touching measure), in the interior of the bead's path (at least that far from its
        // ends: a bead ending beside another does not hold it), the path's own geometry within one bead of arc length not counting
        // (another part of the same chain does: the connector joins lines along the boundary into one path). Returns the distance
        // from the vertex to the nearest such centre line, or nothing.
        const auto arc_positions = [](const Polyline &path) {
            std::vector<double> arc(path.points.size(), 0.);
            for (size_t i = 1; i < path.points.size(); ++i) arc[i] = arc[i - 1] + unscale<double>((path.points[i] - path.points[i - 1]).cast<double>().norm());
            return arc;
        };
        // A closed path (a course laid as a loop) has no ends: arc distances along it go round, and nothing lies at its ends.
        const auto is_closed = [](const Polyline &path) { return path.points.size() > 2 && path.first_point() == path.last_point(); };
        const auto fused_at = [&upper, &arc_positions, &is_closed](const EmittedPath &own, size_t vertex) -> std::optional<double> {
            const Point &v = own.path.points[vertex];
            const std::vector<double> own_arc = arc_positions(own.path);
            const bool own_closed = is_closed(own.path);
            const auto own_distance = [&](double a, double b) { const double d = std::abs(a - b); return own_closed ? std::min(d, own_arc.back() - d) : d; };
            std::optional<double> best;
            for (const EmittedPath *other : upper) {
                if (!get_extents(other->path).inflated(coord_t(scale_(2.))).contains(v)) continue;
                const double reach = .5 * other->width + .01;
                const std::vector<double> arc = other == &own ? own_arc : arc_positions(other->path);
                const bool other_closed = other == &own ? own_closed : is_closed(other->path);
                for (size_t i = 1; i < other->path.points.size(); ++i) {
                    if (other == &own && std::min(own_distance(own_arc[i - 1], own_arc[vertex]), own_distance(own_arc[i], own_arc[vertex])) <= own.width + .01)
                        continue;    // the path's own geometry within a bead of the vertex
                    const Line seg(other->path.points[i - 1], other->path.points[i]);
                    const double d = unscale<double>(seg.distance_to(v));
                    if (d > reach) continue;
                    // The foot's position along the met path: in the interior, a bead's half width from the ends.
                    const Vec2d a = seg.a.cast<double>(), dvec = (seg.b - seg.a).cast<double>();
                    const double l2 = dvec.squaredNorm();
                    const double t = l2 > 0. ? std::min(1., std::max(0., (v.cast<double>() - a).dot(dvec) / l2)) : 0.;
                    const double foot = arc[i - 1] + t * (arc[i] - arc[i - 1]);
                    if (!other_closed && (foot < reach || arc.back() - foot < reach)) continue;
                    if (!best || d < *best) best = d;
                }
            }
            return best;
        };
        const auto fused = [&fused_at](const EmittedPath &own, size_t vertex) { return fused_at(own, vertex).has_value(); };
        for (const EmittedPath *path : sparse) {
            const ExPolygons &hold = hold_of(path->width);
            // A vertex of the path that lies on another bead of the layer is a junction where the pattern's line ended and its nub
            // along the met line begins (or where the connector joined two lines): the line landed on existing material there, so
            // the path is measured in the parts between such vertices, each part's end at a junction being fused, not free.
            // A vertex where a lattice line turns into a connector hook along the island boundary (off the lattice directions, on
            // the line of a control segment: Cubic's lines hook along the same boundary) is a junction too: the hook is measured
            // as a connection piece of its own and the line's end at the junction is not free (Cubic's boundary behaviour, accepted
            // by the user on 9 November 2026).
            const auto is_hook = [&](const Point &a, const Point &b) { return !along_lattice(a, b, directions) && on_control_line(a, b, control_segments); };
            const auto hook_junction = [&](const EmittedPath &own, size_t i) {
                const Points &pts = own.path.points;
                if (is_closed(own.path)) {
                    const size_t ring = pts.size() - 1, k = i % ring;
                    return is_hook(pts[(k + ring - 1) % ring], pts[k]) != is_hook(pts[k], pts[(k + 1) % ring]);
                }
                if (i == 0 || i + 1 >= pts.size()) return false;
                return is_hook(pts[i - 1], pts[i]) != is_hook(pts[i], pts[i + 1]);
            };
            // A closed path (a course laid as a loop) has no ends. It is measured from the first point where it is held (where it
            // enters the hold set, a vertex or a point inside a segment) or a fused vertex, round to itself, so that a stretch
            // through its start point is one stretch; that point counts as fused at both ends of the measure. A loop with no held
            // and no fused point is not held anywhere: it is one free piece.
            const bool closed = is_closed(path->path);
            constexpr size_t inserted = size_t(-1);
            Polyline measured = path->path;
            std::vector<size_t> orig(measured.points.size());
            std::iota(orig.begin(), orig.end(), 0);
            bool loop_held = !closed;
            if (closed) {
                const Points &pts = path->path.points;
                const size_t ring = pts.size() - 1;
                const auto held_point = [&hold](const Point &v) { return std::any_of(hold.begin(), hold.end(), [&v](const ExPolygon &e) { return e.contains(v); }); };
                for (size_t i = 0; i < ring && !loop_held; ++i) {
                    std::optional<Point> entry;
                    size_t from = i;
                    if (held_point(pts[i]) || fused(*path, i))
                        entry = pts[i];
                    else {
                        // The segment's first point inside the hold set.
                        Polylines inside = intersection_pl(Polylines{ Polyline(pts[i], pts[i + 1]) }, hold);
                        for (const Polyline &piece : inside)
                            for (const Point &q : { piece.first_point(), piece.last_point() })
                                if (!entry || (q - pts[i]).cast<double>().norm() < (*entry - pts[i]).cast<double>().norm())
                                    entry = q;
                    }
                    if (!entry)
                        continue;
                    loop_held = true;
                    measured.points.clear();
                    orig.clear();
                    measured.points.push_back(*entry);
                    orig.push_back(*entry == pts[i] ? i : inserted);
                    for (size_t j = 1; j <= ring; ++j) {
                        const size_t k = (from + j) % ring;
                        measured.points.push_back(pts[k]);
                        orig.push_back(k);
                    }
                    if (*entry != pts[i]) {
                        measured.points.push_back(*entry);
                        orig.push_back(inserted);
                    } else {
                        orig.back() = i;
                    }
                }
            }
            if (!loop_held) {
                if (on_control_lines(path->path, control_segments) || boundary_join(path->path, control_segments, directions))
                    continue;    // a loop of Cubic's own lines or joins
                ++result.added_pieces;
                ++result.free_ends;
                result.where.push_back(unscale(path->path.first_point()));
                result.longest = std::max(result.longest, unscale<double>(path->path.length()));
                continue;
            }
            const size_t n_pts = measured.points.size();
            std::vector<bool> fused_vertex(n_pts, false);
            for (size_t i = 0; i < n_pts; ++i)
                fused_vertex[i] = (closed && (i == 0 || i + 1 == n_pts)) ||
                                  (orig[i] != inserted && (fused(*path, orig[i]) || hook_junction(*path, orig[i])));
            struct Part { Polyline line; bool start_fused, end_fused; };
            std::vector<Part> parts;
            {
                Part part{ Polyline(), fused_vertex.front(), false };
                for (size_t i = 0; i < n_pts; ++i) {
                    const Point &v = measured.points[i];
                    part.line.points.push_back(v);
                    if (i > 0 && i + 1 < n_pts && fused_vertex[i]) {
                        part.end_fused = true;
                        parts.push_back(part);
                        part = Part{ Polyline(), true, false };
                        part.line.points.push_back(v);
                    }
                }
                part.end_fused = fused_vertex.back();
                if (part.line.points.size() >= 2) parts.push_back(part);
            }
            for (const Part &part : parts)
            for (const Polyline &piece : diff_pl(Polylines{part.line}, hold)) {
                if (piece.length() < scale_(1e-4) || on_control_lines(piece, control_segments) || boundary_join(piece, control_segments, directions)) continue;
                ++result.added_pieces;
                const bool touches_start = piece.first_point() == part.line.first_point() || piece.last_point() == part.line.first_point();
                const bool touches_end   = piece.first_point() == part.line.last_point() || piece.last_point() == part.line.last_point();
                std::optional<Point> free_end;
                if (touches_start && !part.start_fused) free_end = part.line.first_point();
                else if (touches_end && !part.end_fused) free_end = part.line.last_point();
                const bool at_end = free_end.has_value();
                const double length = unscale<double>(piece.length());
                const double gap_b = gap_b_at(unscale(piece.first_point()), path->z);
                if (at_end) {
                    ++result.free_ends;
                    result.where.push_back(unscale(*free_end));
                } else if (length > gap_b) {
                    ++result.long_stretches;
                    result.where.push_back(unscale(piece.points[piece.points.size() / 2]));
                }
                result.longest = std::max(result.longest, length);
            }
        }
    }
    return result;
}

inline K2Measure measure_k2(const std::vector<EmittedPath> &paths, const std::vector<EmittedPath> &control,
                            const std::function<double(const Vec2d&, double)> &gap_b_at, double infill_direction_deg)
{
    return measure_k2(paths, control, gap_b_at, std::vector<double>{ infill_direction_deg });
}

inline K2Measure measure_k2(const std::vector<EmittedPath> &paths, const std::vector<EmittedPath> &control, double gap_b, const std::vector<double> &infill_directions_deg)
{
    return measure_k2(paths, control, [gap_b](const Vec2d &, double) { return gap_b; }, infill_directions_deg);
}

inline K2Measure measure_k2(const std::vector<EmittedPath> &paths, const std::vector<EmittedPath> &control, double gap_b, double infill_direction_deg)
{
    return measure_k2(paths, control, [gap_b](const Vec2d &, double) { return gap_b; }, infill_direction_deg);
}

} // namespace BACObserver

#endif // slic3r_tests_bridge_aware_cubic_observer_hpp_
