#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubic.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSerialization.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils.hpp"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

using namespace Slic3r;

namespace {

struct ExportWorkspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("bridge-aware-export-%%%%-%%%%");
    std::string old_data = data_dir(), old_temp = temporary_dir();

    ExportWorkspace()
    {
        boost::filesystem::create_directories(path);
        set_data_dir(path.string());
        set_temporary_dir(path.string());
    }
    ~ExportWorkspace()
    {
        set_data_dir(old_data);
        set_temporary_dir(old_temp);
        boost::system::error_code error;
        boost::filesystem::remove_all(path, error);
    }
};

DynamicPrintConfig export_config()
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"sparse_infill_density", "5%"}, {"sparse_infill_pattern", "bridgeawarecubic"},
        {"bridge_aware_cubic_max_gap", "5"},
        {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
        {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
        {"wall_loops", "2"}, {"wall_generator", "classic"},
        {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
        {"skirt_loops", "0"}, {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
        {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"},
        {"override_filament_scarf_seam_setting", "1"}, {"seam_slope_type", "none"},
        {"apply_scarf_seam_on_circles", "0"}, {"spiral_mode", "0"},
        {"use_relative_e_distances", "1"}, {"slow_down_for_layer_cooling", "0"},
        {"machine_start_gcode", ""}, {"machine_end_gcode", ""},
        {"before_layer_change_gcode", ""}, {"layer_change_gcode", ""}
    });
    return config;
}

bool model_role(ExtrusionRole role)
{
    return role == erPerimeter || role == erExternalPerimeter || role == erOverhangPerimeter ||
        role == erInternalInfill || role == erSolidInfill || role == erTopSolidInfill ||
        role == erBottomSurface || role == erBridgeInfill || role == erGapFill;
}

// Recover the actual decimal coordinates, rather than retaining the reader's
// float conversion error. This does not move a G-code coordinate off its grid.
double emitted_coordinate(double value) { return std::round(value * 1000.) / 1000.; }
Point emitted_point(double x, double y)
{
    // Point::new_scale truncates its floating result. Even an exact decimal
    // such as 65.000 can scale just below an integer and lose a geometry unit.
    return Point(coord_t(std::llround(scale_(emitted_coordinate(x)))),
                 coord_t(std::llround(scale_(emitted_coordinate(y)))));
}
int plane_key(double z) { return int(std::lround(z * 1000.)); }

double exact_axis_value(const GCodeReader::GCodeLine &line, char axis)
{
    // Read written decimals directly: float coordinates can hide fractional
    // Z preservation and tiny positive absolute-E increments.
    std::istringstream words(line.raw().substr(0, line.raw().find(';')));
    std::string word;
    while (words >> word)
        if (word.size() > 1 && word.front() == axis) return std::stod(word.substr(1));
    FAIL("A parsed axis must have a decimal token in its G-code line.");
    return 0.;
}

double height_roundoff(double height, double z)
{
    const float stored_height = float(height);
    return .5 * double(std::nextafter(stored_height, std::numeric_limits<float>::infinity()) - stored_height) +
        8. * std::numeric_limits<double>::epsilon() * std::max(1., std::abs(z));
}

struct EmittedPath {
    Polyline path;
    ExtrusionRole role = erNone;
    double z = 0., width = 0., height = 0.;
    size_t first_line = 0, last_line = 0;
    std::vector<double> extrusion_deltas;
};

std::vector<EmittedPath> read_extrusions(const std::string &filename, const DynamicPrintConfig &config)
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
        const double from_z = exact_z;
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
        // The plain-seam fixture has no planar-to-arc or sloped transformation.
        REQUIRE(line.cmd_is("G1"));
        REQUIRE(from_z == exact_z);
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
            paths.back().extrusion_deltas.push_back(extrusion_delta);
        } else {
            paths.push_back({Polyline(Points{from, to}), role, z, width, height, line_number, line_number,
                             {extrusion_delta}});
        }
        continuous = true;
    }));
    return paths;
}

using EmittedLayer = std::vector<const EmittedPath *>;

void append_segment_strokes(Polygons &strokes, const Polyline &path, double width)
{
    for (size_t i = 1; i < path.points.size(); ++i)
        if (path.points[i - 1] != path.points[i])
            append(strokes, offset(Polyline(Points{path.points[i - 1], path.points[i]}), scale_(width * .5)));
}

ExPolygons footprint(const EmittedLayer &paths, ExtrusionRole role = erNone, bool individual_segments = false)
{
    Polygons strokes;
    for (const EmittedPath *path : paths)
        if (role == erNone || path->role == role) {
            if (individual_segments) append_segment_strokes(strokes, path->path, path->width);
            else append(strokes, offset(path->path, scale_(path->width * .5)));
        }
    return union_ex(strokes);
}

struct Measurement {
    size_t targets = 0, gaps = 0, cells = 0, unanchored = 0, open_cells = 0;
    double max_gap = 0., max_cell = 0.;
};

// This helper deliberately uses neither the BAC planner nor its validator.
// Full emitted extrusion runs are clipped against actual preceding material;
// the roof demand only selects entire voids, never closes them artificially.
// Width tags define the nominal XY footprint: this is not a material-contact
// or deposited-volume qualification.
Measurement measure(const EmittedLayer &support_paths, const EmittedLayer &roof_paths,
                    const ExPolygons &physical_domain)
{
    const ExPolygons support = offset_ex(footprint(support_paths), -scale_(.001));
    const ExPolygons demand = footprint(roof_paths);
    Measurement result;
    for (const EmittedPath *path : roof_paths) {
        ++result.targets;
        for (const Polyline &air : diff_pl(Polylines{path->path}, support)) {
            ++result.gaps;
            result.max_gap = std::max(result.max_gap, unscale<double>(air.length()));
            // A zero-length boundary intersection is not an end anchor.
            if (air.first_point() == path->path.first_point() || air.last_point() == path->path.first_point() ||
                air.first_point() == path->path.last_point() || air.last_point() == path->path.last_point())
                ++result.unanchored;
        }
    }
    const ExPolygons boundary_band = diff_ex(physical_domain, offset_ex(physical_domain, -scale_(.001)));
    for (const ExPolygon &cell : diff_ex(physical_domain, support)) {
        if (intersection_ex(cell, demand).empty()) continue;
        ++result.cells;
        if (!intersection_ex(cell, boundary_band).empty()) ++result.open_cells;
        // Maximum pairwise vertex distance is the polygon/hull diameter even
        // for an irregular cell. It does not reuse ideal cube dimensions.
        double squared_diameter = 0.;
        for (const Point &a : cell.contour.points)
            for (const Point &b : cell.contour.points)
                squared_diameter = std::max(squared_diameter, (a.cast<double>() - b.cast<double>()).squaredNorm());
        result.max_cell = std::max(result.max_cell, unscale<double>(std::sqrt(squared_diameter)));
    }
    return result;
}

ExtrusionPath emitted_bead(const EmittedPath &path)
{
    // The model consumes nominal dimensions and requires positive extrusion.
    // Actual positive E was established while parsing; the area below models
    // those dimensions, and does not independently qualify physical flow.
    const double area = path.height * (path.width - path.height * (1. - PI * .25));
    ExtrusionPath result(path.role, area, float(path.width), float(path.height));
    result.polyline = path.path;
    return result;
}

void require_exported_contact(const EmittedLayer &support_paths, const EmittedLayer &roof_paths,
                              const ExPolygons &physical_domain)
{
    std::vector<ExtrusionPath> beads;
    beads.reserve(support_paths.size());
    for (const EmittedPath *path : support_paths) beads.push_back(emitted_bead(*path));
    std::vector<BridgeAwareCubicContact::Bead> deposited;
    for (size_t i = 0; i < support_paths.size(); ++i)
        deposited.push_back({&beads[i], support_paths[i]->z});
    // Coordinates have already been serialized: only polygon numerical
    // uncertainty remains, not a second allowance for future export rounding.
    constexpr double tolerance = 16. * SCALING_FACTOR;
    for (const EmittedPath *path : roof_paths) {
        const ExtrusionPath target = emitted_bead(*path);
        const auto contact = BridgeAwareCubicContact::footprint(deposited, target, path->z, tolerance);
        REQUIRE(contact.input_valid);
        REQUIRE(contact.completed);
        const auto result = BridgeAwareCubicValidation::validate_contacts(physical_domain,
            contact.physical_footprint, {path->path}, footprint(EmittedLayer{path}), 5., tolerance, {contact});
        CAPTURE(path->z, path->width, path->height, result.max_unsupported_mm,
                result.max_void_diameter_mm, result.failure_reason);
        CHECK(result.passed());
    }
}

void visit_paths(const ExtrusionEntity &entity, const std::function<void(const ExtrusionPath &)> &visit)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) visit(*path);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const auto &path : multi->paths) visit(path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const auto &path : loop->paths) visit(path);
    else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const auto *child : collection->entities) visit_paths(*child, visit);
}

size_t require_exported_volume(const Print &print, const Layer &source, const EmittedLayer &emitted,
                               bool relative_e)
{
    // The selected roof and its predecessor are above the first layer. These
    // roles have neither first-layer nor top-surface flow adjustments.
    REQUIRE(source.id() > 0);
    const auto selected_role = [](ExtrusionRole role) {
        return role == erInternalInfill || role == erBridgeInfill;
    };
    const double diameter = print.config().filament_diameter.get_at(0);
    const double filament_e_per_mm3 = print.config().filament_flow_ratio.get_at(0) /
        (diameter * diameter * .25 * PI);
    using Dimensions = std::tuple<int, float, float>;
    std::map<Dimensions, std::set<double>> source_rates;
    const PrintObject *object = print.get_object(size_t(0));
    REQUIRE(object->instances().size() == 1);
    BridgeAwareCubic::SerializedXY transform;
    transform.shift = unscaled(object->instances().front().shift);
    transform.nozzle_offset = print.config().extruder_offset.get_at(0);
    transform.plate_origin = print.get_plate_origin().head<2>();
    size_t rounding_witnesses = 0;
    for (const LayerRegion *region : source.regions()) {
        visit_paths(region->fills, [&](const ExtrusionPath &path) {
            if (!selected_role(path.role()) || path.is_force_no_extrusion() || path.mm3_per_mm <= 0.) return;
            // Keep calibration multipliers explicit. Nominal dimensions alone
            // do not recover the source flow or the requested filament amount.
            const double rate = filament_e_per_mm3 * path.mm3_per_mm * region->region().config().print_flow_ratio.value;
            source_rates[{int(path.role()), path.width, path.height}].insert(rate);
            for (const Line &segment : path.polyline.lines()) {
                const double serialized_length = (transform(segment.b) - transform(segment.a)).cast<double>().norm() * SCALING_FACTOR;
                const double original_length = segment.length() * SCALING_FACTOR;
                if (serialized_length > 0. && std::abs(serialized_length - original_length) * rate > 1e-8)
                    ++rounding_witnesses;
            }
        });
    }
    size_t checked_segments = 0;
    // Relative E rounds one token; an absolute increment subtracts two rounded
    // tokens. The extra 1e-12 covers arithmetic at this fixture's coordinates.
    const double tolerance = (relative_e ? .5e-9 : 1e-9) + 1e-12;
    for (const EmittedPath *path : emitted) {
        if (!selected_role(path->role)) continue;
        const auto found = source_rates.find({int(path->role), float(path->width), float(path->height)});
        REQUIRE(found != source_rates.end());
        const auto &rates = found->second;
        REQUIRE_FALSE(rates.empty());
        // Require unambiguous source flow for this geometry/dimension group;
        // do not silently choose a rate that happens to fit the emitted E.
        REQUIRE(*rates.rbegin() - *rates.begin() <= 64. * std::numeric_limits<double>::epsilon());
        REQUIRE(path->extrusion_deltas.size() + 1 == path->path.points.size());
        for (size_t i = 1; i < path->path.points.size(); ++i) {
            const double length = (path->path.points[i] - path->path.points[i - 1]).cast<double>().norm() * SCALING_FACTOR;
            const double expected_e = length * *rates.begin();
            const double actual_e = path->extrusion_deltas[i - 1];
            CAPTURE(path->z, path->role, i, length, expected_e, actual_e);
            CHECK(std::abs(actual_e - expected_e) <= tolerance);
            ++checked_segments;
        }
    }
    REQUIRE(checked_segments > 0);
    return rounding_witnesses;
}

ExPolygons preview_footprint(const Print &print, const Layer &layer, ExtrusionRole role,
                            float width = std::numeric_limits<float>::quiet_NaN(),
                            float height = std::numeric_limits<float>::quiet_NaN())
{
    const auto *object = print.get_object(size_t(0));
    const Vec2d translation = unscaled(object->instances().front().shift) -
        print.config().extruder_offset.get_at(0) - print.get_plate_origin().head<2>();
    Polygons strokes;
    const auto visit = [&](const ExtrusionPath &path) {
        if (path.role() != role || path.is_force_no_extrusion() || path.mm3_per_mm <= 0.) return;
        if ((std::isfinite(width) && path.width != width) || (std::isfinite(height) && path.height != height)) return;
        Polyline rounded = path.polyline;
        for (Point &point : rounded.points) {
            const Vec2d xy = unscaled(point) + translation;
            point = emitted_point(xy.x(), xy.y());
        }
        append_segment_strokes(strokes, rounded, path.width);
    };
    for (const LayerRegion *region : layer.regions()) {
        visit_paths(region->perimeters, visit);
        visit_paths(region->fills, visit);
    }
    return union_ex(strokes);
}

} // namespace

TEST_CASE("Bridge aware cubic export observer preserves the exact decimal XY grid",
          "[BridgeAwareCubic][Export][ExportCoordinates]")
{
    CHECK(emitted_point(65., 65.003) == Point(6500000, 6500300));
    CHECK(emitted_point(-65., -65.003) == Point(-6500000, -6500300));
    CHECK(emitted_point(double(float(65.003)), double(float(80.023))) == Point(6500300, 8002300));
    const Point delta = emitted_point(80.023, 80.023) - emitted_point(65., 65.);
    REQUIRE(delta == Point(1502300, 1502300));
    // The first E/mm failure had one truncated axis: hypot(15.023,15.02299).
    // Both actual decimal displacements are 15.023 mm.
    CHECK(delta.cast<double>().norm() * SCALING_FACTOR == Approx(std::hypot(15.023, 15.023)).margin(1e-12));
}

TEST_CASE("Bridge aware cubic exported coordinates retain roof gaps and preview support geometry",
          "[BridgeAwareCubic][Integration][Export]")
{
    // The fractional placement exercises serialization after instance and
    // plate translation, including rounding phases absent from local paths.
    // Retain the four placement/height cases and add one full absolute-E
    // certificate check, rather than duplicating the geometry matrix.
    const auto [translated, layer_height, relative_e] = GENERATE(
        std::make_tuple(false, .2, true), std::make_tuple(true, .2, true),
        std::make_tuple(false, .2004, true), std::make_tuple(true, .2004, true),
        std::make_tuple(true, .2004, false));
    CAPTURE(translated, layer_height, relative_e);
    ExportWorkspace workspace;
    auto config = export_config();
    config.set_key_value("layer_height", new ConfigOptionFloat(layer_height));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(layer_height));
    config.set_key_value("use_relative_e_distances", new ConfigOptionBool(relative_e));
    if (!relative_e) {
        config.set_deserialize_strict({{"filament_flow_ratio", "0.97"}, {"print_flow_ratio", "1.03"}});
    }
    Model model;
    auto *model_object = model.add_object();
    model_object->name = "Bridge aware cubic exported roof";
    model_object->add_volume(TriangleMesh(its_make_cube(32., 28., 18.)));
    model_object->add_instance()->set_offset(translated ? Vec3d(91.00037, 87.00061, 0.) : Vec3d(80., 80., 0.));
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model_object);
    print.apply(model, config);
    if (translated) print.set_plate_origin(Vec3d(11.00013, 7.00029, 0.));
    REQUIRE_NOTHROW(print.process());
    GCodeProcessorResult preview;
    std::string filename;
    REQUIRE_NOTHROW(filename = print.export_gcode((workspace.path / "roof.gcode").string(), &preview));
    REQUIRE(boost::filesystem::is_regular_file(filename));
    REQUIRE_FALSE(preview.moves.empty());
    const auto paths = read_extrusions(filename, config);
    REQUIRE_FALSE(paths.empty());
    std::map<int, EmittedLayer> layers;
    for (const EmittedPath &path : paths) layers[plane_key(path.z)].push_back(&path);
    REQUIRE(layers.size() > 50);
    // This closed cube has model walls on every source layer. A truncated
    // export must not pass merely because its first roof was already written.
    REQUIRE(layers.size() == print.get_object(size_t(0))->layers().size());

    // Fractional-height tangency must survive actual serialization. For
    // example, .2004/.4008 rounded independently to .200/.401 create .0006 mm
    // of air. Inspect every deposition plane so the selected roof's rounding
    // phase cannot conceal this failure.
    size_t preserved_planes = 0;
    for (auto it = layers.begin(); it != layers.end(); ++it) {
        const Layer *source = nullptr;
        for (const Layer *layer : print.get_object(size_t(0))->layers())
            if (plane_key(layer->print_z) == it->first) source = layer;
        REQUIRE(source != nullptr);
        const double actual_z = it->second.front()->z;
        CAPTURE(actual_z, source->print_z);
        CHECK(std::abs(actual_z - source->print_z) < 1e-8);
        ++preserved_planes;
        if (it == layers.begin()) continue;
        const double preceding_z = std::prev(it)->second.front()->z;
        for (const EmittedPath *path : it->second) {
            REQUIRE(path->z == actual_z);
            CHECK(path->z - path->height <= preceding_z + height_roundoff(path->height, path->z));
        }
    }
    REQUIRE(preserved_planes > 50);

    // Find the actual first roof using emitted roles and planes, independently
    // of the plan, fill-surface classification, or an assumed roof layer index.
    auto upper = layers.end();
    for (auto it = std::next(layers.begin()); it != layers.end(); ++it) {
        const auto &preceding = std::prev(it)->second;
        const bool sparse_below = std::any_of(preceding.begin(), preceding.end(),
            [](const EmittedPath *path) { return path->role == erInternalInfill; });
        const bool bridge_here = std::any_of(it->second.begin(), it->second.end(),
            [](const EmittedPath *path) { return path->role == erBridgeInfill; });
        if (sparse_below && bridge_here) { upper = it; break; }
    }
    REQUIRE(upper != layers.end());
    const auto lower = std::prev(upper);
    REQUIRE(upper->second.front()->z > lower->second.front()->z);
    REQUIRE(upper->second.front()->z - lower->second.front()->z < .202);
    EmittedLayer roof;
    size_t first_roof_line = std::numeric_limits<size_t>::max();
    for (const EmittedPath *path : upper->second)
        if (path->role == erBridgeInfill) {
            roof.push_back(path);
            first_roof_line = std::min(first_roof_line, path->first_line);
            REQUIRE(path->z - path->height <= lower->second.front()->z + height_roundoff(path->height, path->z));
        }
    REQUIRE_FALSE(roof.empty());
    for (const EmittedPath *path : lower->second) {
        REQUIRE(path->last_line < first_roof_line);
        REQUIRE(path->z < roof.front()->z);
    }

    // Recover the whole material domain from its real closed external wall,
    // not from roof bounds or a modifier edge. An open wall cannot supply it.
    const auto outer_wall = footprint(lower->second, erExternalPerimeter);
    REQUIRE(outer_wall.size() == 1);
    REQUIRE_FALSE(outer_wall.front().holes.empty());
    const ExPolygons domain{ExPolygon(outer_wall.front().contour)};
    const Measurement actual = measure(lower->second, roof, domain);
    CAPTURE(actual.targets, actual.gaps, actual.cells, actual.max_gap, actual.max_cell);
    REQUIRE(actual.targets > 0);
    REQUIRE(actual.gaps > 5);
    REQUIRE(actual.cells > 5);
    CHECK(actual.unanchored == 0);
    CHECK(actual.open_cells == 0);
    CHECK(actual.max_gap <= 5.);
    CHECK(actual.max_cell <= 5.);

    // This is an output-input regression of the shared contact model, in
    // addition to the independent nominal-width observer above. Reconstruct
    // its beads only from emitted positive-E paths, dimensions and exact Z.
    require_exported_contact(lower->second, roof, domain);

    // Prove the observer detects absent preceding partitions; upper-layer or
    // future material must not make the same roof pass after their removal.
    EmittedLayer missing_sparse;
    for (const EmittedPath *path : lower->second)
        if (path->role != erInternalInfill) missing_sparse.push_back(path);
    const Measurement broken = measure(missing_sparse, roof, domain);
    CHECK((broken.max_gap > 5. || broken.max_cell > 5. || broken.unanchored > 0));

    // Path reversal and seam placement can subdivide edges. For coherence,
    // sweep each segment separately on both sides so path chunking cannot
    // change corner joins, then compare geometry by role.
    size_t volume_rounding_witnesses = 0;
    for (const auto layer_it : {lower, upper}) {
        const Layer *source = nullptr;
        for (const Layer *layer : print.get_object(size_t(0))->layers())
            if (plane_key(layer->print_z) == layer_it->first) source = layer;
        REQUIRE(source != nullptr);
        volume_rounding_witnesses += require_exported_volume(print, *source, layer_it->second, relative_e);
        for (const ExtrusionRole role : {erExternalPerimeter, erPerimeter, erInternalInfill, erBridgeInfill}) {
            CAPTURE(layer_it->first, role);
            const auto before_export = preview_footprint(print, *source, role);
            const auto after_export = footprint(layer_it->second, role, true);
            CHECK(before_export.empty() == after_export.empty());
            CHECK(diff_ex(before_export, offset_ex(after_export, scale_(.002))).empty());
            CHECK(diff_ex(after_export, offset_ex(before_export, scale_(.002))).empty());
            // Dimensions must round-trip the source floats, and the same
            // dimensions must cover the same geometry. A rounded comment may
            // otherwise silently change the contact plane or bead section.
            std::map<std::pair<float, float>, EmittedLayer> dimensions;
            for (const EmittedPath *path : layer_it->second)
                if (path->role == role) dimensions[{float(path->width), float(path->height)}].push_back(path);
            for (const auto &entry : dimensions) {
                CAPTURE(entry.first.first, entry.first.second);
                const auto source_group = preview_footprint(print, *source, role, entry.first.first, entry.first.second);
                const auto output_group = footprint(entry.second, role, true);
                REQUIRE_FALSE(source_group.empty());
                CHECK(diff_ex(output_group, offset_ex(source_group, scale_(.002))).empty());
                CHECK(diff_ex(source_group, offset_ex(output_group, scale_(.002))).empty());
            }
        }
    }
    // At least one tested source edge changes length enough on the output grid
    // to reject the former source-length E calculation by a wide margin.
    REQUIRE(volume_rounding_witnesses > 0);
}

TEST_CASE("Bridge aware cubic export retains ordinary fitted arcs in a mixed print",
          "[BridgeAwareCubic][Integration][Export][MixedObjects]")
{
    ExportWorkspace workspace;
    auto config = export_config();
    config.set_key_value("enable_arc_fitting", new ConfigOptionBool(true));
    Model model;
    auto *bac = model.add_object();
    bac->name = "BAC object beside ordinary arcs";
    // A short solid cube exercises the BAC object's export policy without
    // adding another expensive support-refinement fixture to this scope test.
    bac->add_volume(TriangleMesh(its_make_cube(8., 8., 1.2)));
    bac->add_instance()->set_offset(Vec3d(80., 80., 0.));
    auto *ordinary = model.add_object();
    ordinary->name = "Ordinary arc-fitted cylinder";
    ordinary->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipCubic));
    ordinary->add_volume(TriangleMesh(its_make_cylinder(3., 1.2, PI / 128.)));
    ordinary->add_instance()->set_offset(Vec3d(120., 80., 0.));

    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(bac);
    print.auto_assign_extruders(ordinary);
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    REQUIRE(print.objects().size() == 2);
    size_t bac_objects = 0, ordinary_objects = 0, ordinary_source_arcs = 0;
    for (const PrintObject *object : print.objects()) {
        const bool certified = object->has_bridge_aware_cubic();
        if (certified) ++bac_objects;
        else ++ordinary_objects;
        for (const Layer *layer : object->layers())
            for (const LayerRegion *region : layer->regions()) {
                const auto check_path = [&](const ExtrusionPath &path) {
                    if (path.is_force_no_extrusion() || path.mm3_per_mm <= 0.) return;
                    for (const PathFittingData &fit : path.polyline.fitting_result)
                        if (fit.path_type == EMovePathType::Arc_move_cw || fit.path_type == EMovePathType::Arc_move_ccw) {
                            REQUIRE_FALSE(certified);
                            ++ordinary_source_arcs;
                        }
                };
                visit_paths(region->perimeters, check_path);
                visit_paths(region->fills, check_path);
            }
    }
    REQUIRE(bac_objects == 1);
    REQUIRE(ordinary_objects == 1);
    REQUIRE(ordinary_source_arcs > 0);

    GCodeProcessorResult preview;
    std::string filename;
    REQUIRE_NOTHROW(filename = print.export_gcode((workspace.path / "mixed-arcs.gcode").string(), &preview));
    REQUIRE(boost::filesystem::is_regular_file(filename));
    REQUIRE_FALSE(preview.moves.empty());
    GCodeReader reader;
    reader.apply_config(config);
    const auto role_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role);
    ExtrusionRole role = erNone;
    size_t ordinary_output_arcs = 0, bac_output_lines = 0;
    REQUIRE(reader.parse_file(filename, [&](GCodeReader &before, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.substr(0, role_tag.size()) == role_tag)
            role = ExtrusionEntity::string_to_role(comment.substr(role_tag.size()));
        if (!model_role(role) || !line.extruding(before)) return;
        if (line.cmd_is("G2") || line.cmd_is("G3")) {
            // The objects are separated in X, so this witnesses an ordinary
            // object's actual curved deposition, not a travel spiral.
            CHECK(before.x() > 100.);
            CHECK(line.new_X(before) > 100.);
            ++ordinary_output_arcs;
        } else if (line.cmd_is("G1") && line.dist_XY(before) > 0. && line.new_X(before) < 100.) {
            ++bac_output_lines;
        }
    }));
    CHECK(ordinary_output_arcs > 0);
    CHECK(bac_output_lines > 0);
}

namespace {
struct QualificationMeasurement {
    double process_s = 0., export_s = 0., process_peak_rss_mib = 0.;
    double source_model_mm3 = 0., moving_model_mm3 = 0., moving_sparse_mm3 = 0.;
    size_t layers = 0, nodes = 0, splits = 0, solid_cells = 0, refined_layers = 0;
    size_t closure_courses = 0, attachment_courses = 0;
    unsigned max_depth = 0;
    double refined_z_min = 0., refined_z_max = 0., max_refined_area_mm2 = 0., refined_prism_mm3 = 0.;
};

QualificationMeasurement qualify_cube(const char *size_name, double width, double depth, bool bac,
                                       const ExportWorkspace &workspace)
{
    auto config = export_config();
    const std::string pattern = bac ? "bridgeawarecubic" : "cubic";
    config.set_deserialize_strict({{"sparse_infill_pattern", pattern}});
    Model model;
    auto *model_object = model.add_object();
    model_object->name = std::string("BAC qualification ") + size_name + " " + pattern;
    model_object->add_volume(TriangleMesh(its_make_cube(width, depth, 18.)));
    model_object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model_object);
    print.apply(model, config);
    QualificationMeasurement result;
    const auto started = std::chrono::steady_clock::now();
    REQUIRE_NOTHROW(print.process());
    const auto processed = std::chrono::steady_clock::now();
    result.process_s = std::chrono::duration<double>(processed - started).count();
    std::string filename;
    REQUIRE_NOTHROW(filename = print.export_gcode((workspace.path / (pattern + ".gcode")).string(), nullptr));
    const auto exported = std::chrono::steady_clock::now();
    result.export_s = std::chrono::duration<double>(exported - processed).count();
    const auto emitted = read_extrusions(filename, config);
    REQUIRE_FALSE(emitted.empty());
    const double diameter = print.config().filament_diameter.get_at(0);
    const double filament_area = diameter * diameter * .25 * PI;
    for (const EmittedPath &path : emitted)
        for (double delta_e : path.extrusion_deltas) {
            result.moving_model_mm3 += delta_e * filament_area;
            if (path.role == erInternalInfill) result.moving_sparse_mm3 += delta_e * filament_area;
        }
    const PrintObject *object = print.get_object(size_t(0));
    result.layers = object->layers().size();
    std::set<const BridgeAwareCubic::Plan *> plans;
    for (size_t region = 0; region < object->num_printing_regions(); ++region)
        if (const auto *plan = object->bridge_aware_cubic_plan(region)) plans.insert(plan);
    for (const auto *plan : plans) {
        result.nodes += plan->statistics().nodes;
        result.splits += plan->statistics().splits;
        result.solid_cells += plan->statistics().solid_cells;
        result.max_depth = std::max(result.max_depth, plan->statistics().max_depth);
    }
    for (const Layer *layer : object->layers()) {
        for (const LayerRegion *region : layer->regions()) {
            const auto volume = [&](const ExtrusionPath &path) {
                if (!path.is_force_no_extrusion() && path.mm3_per_mm > 0.)
                    result.source_model_mm3 += unscale<double>(path.length()) * path.mm3_per_mm;
            };
            visit_paths(region->perimeters, volume);
            visit_paths(region->fills, volume);
        }
        ExPolygons selected;
        for (const auto *plan : plans) {
            append(selected, plan->refinement_area(layer->print_z));
            result.closure_courses += plan->closures(layer->print_z).size();
            result.attachment_courses += plan->attachments(layer->print_z).size();
        }
        if (!selected.empty()) {
            selected = intersection_ex(union_ex(selected), layer->lslices);
            const double selected_area = area(selected) * SCALING_FACTOR * SCALING_FACTOR;
            if (selected_area > 0.) {
                if (result.refined_layers++ == 0) result.refined_z_min = layer->print_z;
                result.refined_z_max = layer->print_z;
                result.max_refined_area_mm2 = std::max(result.max_refined_area_mm2, selected_area);
                result.refined_prism_mm3 += selected_area * layer->height;
            }
        }
    }
    result.process_peak_rss_mib = get_peak_rss_mb();
    REQUIRE(result.moving_model_mm3 > 0.);
    // Refinement area is the active selected-parent envelope, not a claim
    // about added bead volume. Closures/attachments are counted separately.
    // Moving model volume comes from actual E, excluding retraction/priming
    // and stationary E-only moves; source nominal volume records all paths.
    std::cout << std::setprecision(12)
        << "BAC_QUALIFICATION size=" << size_name << " dimensions_mm=" << width << 'x' << depth << "x18"
        << " pattern=" << pattern << " density_percent=5 gap_mm=5"
        << " process_s=" << result.process_s << " export_s=" << result.export_s
        << " process_peak_rss_mib=" << result.process_peak_rss_mib
        << " source_model_mm3=" << result.source_model_mm3
        << " moving_model_mm3=" << result.moving_model_mm3
        << " moving_sparse_mm3=" << result.moving_sparse_mm3
        << " layers=" << result.layers << " plan_nodes=" << result.nodes << " plan_splits=" << result.splits
        << " max_depth=" << result.max_depth << " solid_cells=" << result.solid_cells
        << " selected_parent_layers=" << result.refined_layers
        << " selected_parent_z_min=" << result.refined_z_min << " selected_parent_z_max=" << result.refined_z_max
        << " selected_parent_max_area_mm2=" << result.max_refined_area_mm2
        << " selected_parent_prism_mm3=" << result.refined_prism_mm3
        << " closure_courses=" << result.closure_courses << " attachment_courses=" << result.attachment_courses
        << std::endl;
    return result;
}

void qualify_size(const char *size_name, double width, double depth)
{
    ExportWorkspace workspace;
    const auto ordinary = qualify_cube(size_name, width, depth, false, workspace);
    const auto bac = qualify_cube(size_name, width, depth, true, workspace);
    std::cout << std::setprecision(12) << "BAC_QUALIFICATION_DELTA size=" << size_name
        << " added_moving_model_mm3=" << bac.moving_model_mm3 - ordinary.moving_model_mm3
        << " added_moving_model_percent=" << 100. * (bac.moving_model_mm3 / ordinary.moving_model_mm3 - 1.)
        << " added_moving_sparse_mm3=" << bac.moving_sparse_mm3 - ordinary.moving_sparse_mm3
        << " process_time_ratio=" << bac.process_s / ordinary.process_s
        << " pair_process_peak_rss_mib=" << get_peak_rss_mb() << std::endl;
}
} // namespace

// Opt in one case per process so peak RSS is attributable to that fixture pair.
// These measurements deliberately have no performance or material threshold.
TEST_CASE("Bridge aware cubic qualification measurements small", "[.][BACQualification][BACQualificationSmall]")
{
    qualify_size("small", 32., 28.);
}

TEST_CASE("Bridge aware cubic qualification measurements large", "[.][BACQualification][BACQualificationLarge]")
{
    qualify_size("large", 64., 56.);
}

// Persistent inputs for the separately orchestrated native CLI comparison.
// Generating these archives performs configuration application only, never
// slicing or G-code export, and does not change either qualification fixture.
TEST_CASE("Bridge aware cubic writes native CLI benchmark fixtures", "[.][BACBenchmarkFixtures]")
{
    namespace fs = boost::filesystem;
    const char *requested_directory = std::getenv("BAC_BENCHMARK_FIXTURE_DIR");
    REQUIRE(requested_directory != nullptr);
    REQUIRE(std::string(requested_directory).size() > 0);
    const fs::path directory(requested_directory);
    REQUIRE(directory.is_absolute());
    fs::create_directories(directory);
    REQUIRE(fs::is_directory(directory));
    // Preflight every destination before writing any artifact. A repeated run
    // must use a fresh directory rather than overwrite a previous measurement.
    REQUIRE_FALSE(fs::exists(directory / "manifest.json"));
    for (const char *size : {"small", "large", "rib", "modifier"})
        for (const char *pattern : {"cubic", "bridgeawarecubic"})
            for (const char *suffix : {".3mf", ".3mf.tmp", ".config.ini", ".config.json"})
                REQUIRE_FALSE(fs::exists(directory / (std::string(size) + "-" + pattern + suffix)));

    ExportWorkspace workspace;
    DynamicPrintConfig defaults;
    // DynamicPrintConfig::full_print_config() currently supplies region
    // defaults only. Explicitly complete object, region and printer defaults.
    defaults.apply(FullPrintConfig::defaults());
    defaults.apply(export_config());
    defaults.set_deserialize_strict({{"inherits", ""}, {"inherits_group", ""}});
    nlohmann::json manifest = {
        {"schema_version", 1},
        {"generator_version", SLIC3R_VERSION},
        {"configuration_basis", "FullPrintConfig defaults + named fixture config, normalized by Print::apply"},
        {"slicing_performed", false},
        {"allowed_pair_config_differences", {"sparse_infill_pattern", "bridge_aware_cubic_max_gap"}},
        {"fixtures", nlohmann::json::array()}
    };
    using SerializedSettings = std::map<std::string, std::string>;
    std::map<std::string, SerializedSettings> common_settings_by_family;
    for (const std::string size : {"small", "large", "rib", "modifier"}) {
        const bool rib = size == "rib", modifier = size == "modifier";
        const std::string family = rib ? "rib" : modifier ? "modifier" : "cube";
        const double width = modifier ? 48. : size == "large" ? 64. : 32.;
        const double depth = rib ? 26. : size == "large" ? 56. : 28.;
        const double height = rib ? 15. : 18.;
        const double density = rib ? 12. : 5., gap = modifier ? 8. : 5.;
        const size_t model_triangles = rib ? 36 : 12;
        DynamicPrintConfig fixture_defaults = defaults;
        if (rib || modifier) {
            // Keep these physical cases equal to acceptance_config() and
            // modifier_config(), including defaults absent from export_config().
            fixture_defaults.clear();
            fixture_defaults.apply(FullPrintConfig::defaults());
            fixture_defaults.set_deserialize_strict({
                {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
                {"sparse_infill_density", rib ? "12%" : "5%"},
                {"sparse_infill_pattern", "bridgeawarecubic"},
                {"bridge_aware_cubic_max_gap", rib ? "5" : "8"},
                {"infill_direction", rib ? "17" : "45"},
                {"top_shell_layers", "3"}, {"top_shell_thickness", "0"},
                {"bottom_shell_layers", "3"}, {"bottom_shell_thickness", "0"},
                {"wall_loops", "2"}, {"wall_generator", "classic"},
                {"detect_narrow_internal_solid_infill", "0"}, {"brim_type", "no_brim"},
                {"enable_arc_fitting", "0"}, {"thick_bridges", "0"},
                {"bridge_flow", "1"}, {"fill_multiline", "1"}, {"seam_gap", "0"},
                {"inherits", ""}, {"inherits_group", ""}
            });
            if (rib) fixture_defaults.set_deserialize_strict({{"ensure_vertical_shell_thickness", "disabled"}});
        }
        Model model;
        ModelObject *object = model.add_object();
        object->name = "BAC benchmark " + size;
        ModelVolume *volume = nullptr, *right = nullptr;
        if (rib) {
            // Same common-frame assembly as the InwardWalls acceptance case.
            TriangleMesh body(its_make_cube(32., 26., 10.));
            TriangleMesh first(its_make_cube(5., 22., 5.));
            first.translate(6., 2., 10.);
            TriangleMesh second(its_make_cube(4., 18., 3.4));
            second.translate(22., 4., 10.);
            body.merge(first);
            body.merge(second);
            volume = object->add_volume(std::move(body));
            volume->name = "Benchmark inward ribs";
        } else {
            volume = object->add_volume(TriangleMesh(its_make_cube(width, depth, height)));
            volume->name = "Benchmark cube";
        }
        if (modifier) {
            TriangleMesh mesh(its_make_cube(24., 28., 18.));
            mesh.translate(24., 0., 0.);
            right = object->add_volume(std::move(mesh));
            right->set_type(ModelVolumeType::PARAMETER_MODIFIER);
            right->name = "Benchmark right half modifier";
            right->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(12.));
            right->config.set("infill_direction", 17.);
        }
        object->add_instance()->set_offset(Vec3d(80., 80., 0.));
        REQUIRE(object->facets_count() == model_triangles);
        REQUIRE(object->volumes.size() == size_t(modifier ? 2 : 1));
        for (const ModelVolume *part : object->volumes) REQUIRE(part->material_id().empty());
        const auto bounds = object->bounding_box();
        REQUIRE(bounds.min.x() == Approx(80.));
        REQUIRE(bounds.min.y() == Approx(80.));
        REQUIRE(bounds.min.z() == Approx(0.));
        REQUIRE(bounds.max.x() == Approx(80. + width));
        REQUIRE(bounds.max.y() == Approx(80. + depth));
        REQUIRE(bounds.max.z() == Approx(height));
        std::set<std::pair<int, int>> assignments{{0, 0}};
        PlateData plate(0, assignments, false);
        plate.plate_name = "Benchmark plate";
        std::vector<SerializedSettings> common_volume_settings;
        for (const bool bac : {false, true}) {
            const std::string pattern = bac ? "bridgeawarecubic" : "cubic";
            const std::string stem = size + "-" + pattern;
            DynamicPrintConfig config = fixture_defaults;
            config.set_deserialize_strict({{"sparse_infill_pattern", pattern}});
            if (right) {
                if (bac) right->config.set("bridge_aware_cubic_max_gap", 3.);
                else right->config.erase("bridge_aware_cubic_max_gap");
            }
            Print configured;
            configured.set_status_silent();
            configured.auto_assign_extruders(object);
            configured.apply(model, config);
            REQUIRE(configured.objects().size() == 1);
            REQUIRE(configured.get_object(size_t(0))->layers().empty());
            REQUIRE_FALSE(configured.is_step_done(posSlice));
            config = configured.full_print_config();
            // The native CLI indexes process, filament and printer inheritance
            // slots directly. Empty entries preserve no-inheritance semantics;
            // an empty vector is not a valid CLI project shape.
            const auto *filament_settings = config.option<ConfigOptionStrings>("filament_settings_id");
            REQUIRE(filament_settings != nullptr);
            const size_t filament_count = filament_settings->values.size();
            REQUIRE(filament_count == 1);
            config.set_key_value("inherits_group", new ConfigOptionStrings(filament_count + 2, ""));
            // Packaging also indexes one filament ID per slot. Empty IDs keep
            // the fixture independent of any named material/profile identity.
            config.set_key_value("filament_ids", new ConfigOptionStrings(filament_count, ""));
            // Stock Cubic must not depend on ignoring an unknown BAC option.
            // Remove it after applying defaults, which may reintroduce it.
            if (!bac) config.erase("bridge_aware_cubic_max_gap");
            REQUIRE(object->config.empty());
            REQUIRE(volume->config.empty());
            REQUIRE(plate.config.empty());
            REQUIRE(config.opt_serialize("inherits").empty());
            const auto &inheritance = config.option<ConfigOptionStrings>("inherits_group")->values;
            REQUIRE(inheritance.size() == filament_count + 2);
            REQUIRE(std::all_of(inheritance.begin(), inheritance.end(), [](const std::string &entry) { return entry.empty(); }));
            const auto &filament_ids = config.option<ConfigOptionStrings>("filament_ids")->values;
            REQUIRE(filament_ids.size() == filament_count);
            REQUIRE(std::all_of(filament_ids.begin(), filament_ids.end(), [](const std::string &entry) { return entry.empty(); }));
            std::map<std::string, std::string> comparable;
            std::map<std::string, std::string> serialized_settings;
            std::vector<std::string> nil_settings;
            for (const std::string &key : config.keys()) {
                serialized_settings.emplace(key, config.opt_serialize(key));
                if (config.option(key)->is_nil()) nil_settings.push_back(key);
                if (key != "sparse_infill_pattern" && key != "bridge_aware_cubic_max_gap")
                    comparable.emplace(key, config.opt_serialize(key));
            }
            auto &common_settings = common_settings_by_family[family];
            if (common_settings.empty()) common_settings = comparable;
            else REQUIRE(comparable == common_settings);

            nlohmann::json volumes = nlohmann::json::array();
            std::vector<SerializedSettings> comparable_volumes;
            size_t total_triangles = 0;
            for (size_t index = 0; index < object->volumes.size(); ++index) {
                const ModelVolume &part = *object->volumes[index];
                SerializedSettings local, local_comparable;
                std::vector<std::string> local_nil;
                for (const std::string &key : part.config.keys()) {
                    local.emplace(key, part.config.opt_serialize(key));
                    if (part.config.option(key)->is_nil()) local_nil.push_back(key);
                    if (key != "bridge_aware_cubic_max_gap") local_comparable.emplace(key, part.config.opt_serialize(key));
                }
                comparable_volumes.push_back(std::move(local_comparable));
                if (&part == right) {
                    REQUIRE(local.size() == size_t(bac ? 3 : 2));
                    REQUIRE(local.at("sparse_infill_density") == "12%");
                    REQUIRE(local.at("infill_direction") == "17");
                    if (bac) REQUIRE(local.at("bridge_aware_cubic_max_gap") == "3");
                } else REQUIRE(part.config.empty());
                const auto part_bounds = part.mesh().transformed_bounding_box(part.get_matrix());
                total_triangles += part.mesh().facets_count();
                volumes.push_back({
                    {"index", index}, {"name", part.name}, {"type", ModelVolume::type_to_string(part.type())},
                    {"triangle_count", part.mesh().facets_count()},
                    {"bounds_min_mm", {part_bounds.min.x(), part_bounds.min.y(), part_bounds.min.z()}},
                    {"bounds_max_mm", {part_bounds.max.x(), part_bounds.max.y(), part_bounds.max.z()}},
                    {"serialized_config", local}, {"nil_config_keys", local_nil}
                });
            }
            if (common_volume_settings.empty()) common_volume_settings = comparable_volumes;
            else REQUIRE(comparable_volumes == common_volume_settings);
            REQUIRE(total_triangles == model_triangles + size_t(modifier ? 12 : 0));

            const std::string archive = (directory / (stem + ".3mf")).string();
            StoreParams params;
            params.path = archive.c_str();
            params.model = &model;
            params.config = &config;
            params.plate_data_list = {&plate};
            REQUIRE(store_bbs_3mf(params));
            REQUIRE(fs::file_size(archive) > 0);
            const std::string snapshot = (directory / (stem + ".config.ini")).string();
            config.save(snapshot);
            REQUIRE(fs::file_size(snapshot) > 0);
            // Match the embedded project's nozzle-stat representation while
            // retaining the complete applied values separately in the INI.
            DynamicPrintConfig project_config = config;
            split_nozzle_stats_for_export(project_config);
            const std::string project_snapshot = (directory / (stem + ".config.json")).string();
            project_config.save_to_json(project_snapshot, "project_settings", "project", SLIC3R_VERSION);
            REQUIRE(fs::file_size(project_snapshot) > 0);
            manifest["fixtures"].push_back({
                {"name", stem}, {"size", size}, {"family", family}, {"pattern", pattern},
                {"source_fixture", rib ? "InwardWalls" : modifier ? "Modifiers local density angle and gap ownership" : "BACQualification"},
                {"archive", stem + ".3mf"}, {"config_ini", stem + ".config.ini"},
                {"project_config_json", stem + ".config.json"},
                {"dimensions_mm", {width, depth, height}}, {"instance_offset_mm", {80., 80., 0.}},
                {"bounds_min_mm", {bounds.min.x(), bounds.min.y(), bounds.min.z()}},
                {"bounds_max_mm", {bounds.max.x(), bounds.max.y(), bounds.max.z()}},
                {"expected_layer_count", rib ? 75 : 90}, {"triangle_count", total_triangles},
                {"model_triangle_count", model_triangles}, {"volume_count", object->volumes.size()}, {"volumes", volumes},
                {"object_count", 1}, {"instance_count", 1}, {"plate_count", 1},
                {"filament_count", filament_count},
                {"density_percent", density}, {"bac_gap_mm", bac ? nlohmann::json(gap) : nlohmann::json(nullptr)},
                {"config_key_count", config.keys().size()},
                {"serialized_config", serialized_settings}, {"nil_config_keys", nil_settings}
            });
        }
    }
    const std::string manifest_path = (directory / "manifest.json").string();
    std::ofstream output(manifest_path);
    REQUIRE(output.is_open());
    output << std::setw(2) << manifest << '\n';
    output.close();
    REQUIRE(output.good());
    std::cout << "BAC_BENCHMARK_FIXTURES manifest=" << manifest_path << " archives=8 slicing_performed=0" << std::endl;
}
