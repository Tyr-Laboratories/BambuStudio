#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubic.hpp"
#include "libslic3r/Fill/BridgeAwareCubicStrands.hpp"
#include "libslic3r/Fill/BridgeAwareCubicDomainProbe.hpp"
#include <mutex>
#include <numeric>
#include <optional>
#include "libslic3r/Fill/BridgeAwareCubicSerialization.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils.hpp"

#include <boost/filesystem.hpp>
#include <boost/multiprecision/cpp_int.hpp>
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
        // Keep every fixture on the cubes' existing empty machine scripts.
        // Native defaults can move Z before the slicer declares G90/G21.
        fixture_defaults.set_deserialize_strict({{"machine_start_gcode", ""},
                                                 {"machine_end_gcode", ""}});
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
            for (const char *key : {"machine_start_gcode", "machine_end_gcode"}) {
                CAPTURE(stem, key);
                REQUIRE(config.opt_string(key).empty());
            }
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

// A diagnostic on actual certified qualification Plan states. No canonical
// point is installed in a PrintObject, passed to Contact, or sent to export.
namespace canonical_plan_diagnostic {
namespace BAC = Slic3r::BridgeAwareCubic;
using Plane = BAC::StrandPlane;
using Budget = BAC::StrandBudget;
using Atom = std::tuple<int64_t, bool, int64_t, bool, BAC::CellId, coord_t, coord_t, coord_t, coord_t>;
using Run = std::vector<Atom>;
using Rows = std::vector<Run>;
using Image = std::map<Plane, Rows>;
using OwnerSet = std::set<BAC::CellId>;
using XY = std::pair<coord_t, coord_t>;
using Axis = std::pair<XY, XY>;
using Json = nlohmann::json;
struct Counts {
    size_t base_faces = 0, trial_faces = 0, removed_faces = 0, added_faces = 0;
    size_t base_runs = 0, trial_runs = 0, removed_runs = 0, added_runs = 0;
    size_t base_planes = 0, trial_planes = 0, touched_planes = 0, changed_planes = 0;
    size_t evaluated_layers = 0, changed_layers = 0, rollback_checks = 0;
    size_t toggle_kernel_units = 0, fresh_kernel_units = 0, restore_kernel_units = 0;
    void add(const Counts &x) {
        base_faces+=x.base_faces; trial_faces+=x.trial_faces; removed_faces+=x.removed_faces; added_faces+=x.added_faces;
        base_runs+=x.base_runs; trial_runs+=x.trial_runs; removed_runs+=x.removed_runs; added_runs+=x.added_runs;
        base_planes+=x.base_planes; trial_planes+=x.trial_planes; touched_planes+=x.touched_planes; changed_planes+=x.changed_planes;
        evaluated_layers+=x.evaluated_layers; changed_layers+=x.changed_layers; rollback_checks+=x.rollback_checks;
        toggle_kernel_units+=x.toggle_kernel_units; fresh_kernel_units+=x.fresh_kernel_units; restore_kernel_units+=x.restore_kernel_units;
    }
    void publish(Json &j) const {
        j["faces"] = {{"base",base_faces},{"trial",trial_faces},{"removed",removed_faces},{"added",added_faces}};
        j["runs"] = {{"base",base_runs},{"trial",trial_runs},{"removed",removed_runs},{"added",added_runs}};
        j["planes"] = {{"base",base_planes},{"trial",trial_planes},{"touched",touched_planes},{"changed",changed_planes}};
        j["logical_kernel_work_not_elapsed_time"] = {{"toggle_plus_changed_rows",toggle_kernel_units},{"fresh_full_program_plus_rows",fresh_kernel_units},{"restore_plus_changed_rows",restore_kernel_units}};
        j["layers"] = {{"evaluated",evaluated_layers},{"changed",changed_layers},{"rollback_checks",rollback_checks}};
    }
};

OwnerSet owners(const BAC::Plan &plan)
{
    OwnerSet out;
    for (const auto &r : plan.refinements()) if (!r.solid) out.insert(r.id);
    return out;
}
Rows rows(const BAC::StrandProgram &program, const Plane &plane, Budget &budget)
{
    Rows out;
    for (const auto &r : program.runs(plane, budget)) {
        REQUIRE_FALSE(r.atoms.empty());
        REQUIRE(r.points.size() == r.atoms.size() + 1);
        auto &row = out.emplace_back();
        row.reserve(r.atoms.size());
        for (size_t i = 0; i < r.atoms.size(); ++i) {
            const auto &a = r.atoms[i];
            const Point &p = r.points[i], &q = r.points[i + 1];
            row.emplace_back(a.first.slot, a.first.shifted, a.last.slot, a.last.shifted, a.owner,
                             p.x(), p.y(), q.x(), q.y());
        }
    }
    return out;
}
Image image(const BAC::StrandProgram &program, Budget &budget)
{
    Image result;
    for (const auto &plane : program.planes(budget)) {
        auto values = rows(program, plane, budget);
        REQUIRE_FALSE(values.empty());
        REQUIRE(result.emplace(plane, std::move(values)).second);
    }
    return result;
}
// Reassemble ONLY rows listed by the incremental program. This independently
// checks both its touched-plane list and exact ordered run/owner materialization.
void update(Image &result, const BAC::StrandProgram &program, const std::set<Plane> &changed, Budget &budget)
{
    for (const auto &p : changed) {
        auto value = rows(program, p, budget);
        if (value.empty()) result.erase(p); else result[p] = std::move(value);
    }
}
template<class T> std::pair<size_t, size_t> diff_counts(std::vector<T> a, std::vector<T> b)
{
    std::sort(a.begin(), a.end()); std::sort(b.begin(), b.end());
    size_t x = 0, y = 0, common = 0;
    while (x < a.size() && y < b.size()) {
        if (a[x] < b[y]) ++x; else if (b[y] < a[x]) ++y; else { ++x; ++y; ++common; }
    }
    return {a.size() - common, b.size() - common};
}
void count_delta(const Image &base, const Image &fresh, size_t touched, Counts &c)
{
    ++c.evaluated_layers;
    c.base_planes += base.size(); c.trial_planes += fresh.size(); c.touched_planes += touched;
    std::set<Plane> planes;
    for (const auto &e : base) planes.insert(e.first);
    for (const auto &e : fresh) planes.insert(e.first);
    const Rows empty;
    bool changed = false;
    for (const auto &p : planes) {
        auto x = base.find(p), y = fresh.find(p);
        const auto &a = x == base.end() ? empty : x->second;
        const auto &b = y == fresh.end() ? empty : y->second;
        c.base_runs += a.size(); c.trial_runs += b.size();
        for (const auto &r : a) c.base_faces += r.size();
        for (const auto &r : b) c.trial_faces += r.size();
        if (a == b) continue;
        changed = true; ++c.changed_planes;
        const auto runs = diff_counts(a, b);
        c.removed_runs += runs.first; c.added_runs += runs.second;
        std::vector<Atom> af, bf;
        for (const auto &r : a) af.insert(af.end(), r.begin(), r.end());
        for (const auto &r : b) bf.insert(bf.end(), r.begin(), r.end());
        const auto faces = diff_counts(std::move(af), std::move(bf));
        c.removed_faces += faces.first; c.added_faces += faces.second;
    }
    c.changed_layers += changed;
}
// The existing broad reference retains the ORIGINAL raw partition sources,
// even ones clipped outside the bounding rectangle. Compare oriented outer
// endpoints; internal bends/barriers are separate, never silently suppressed.
struct Legacy {
    size_t compared = 0, unavailable = 0, canonical = 0, old = 0, only_new = 0, only_old = 0;
    size_t order_different = 0, atoms = 0, stationary = 0, bends = 0, internal_vertices = 0;
    void inspect(const BAC::Plan &plan, const Image &im, double z, const BoundingBox &box,
                 Slic3r::BridgeAwareCubicValidation::WorkBudget &budget) {
        std::vector<Axis> now, before;
        for (const auto &plane : im) for (const auto &r : plane.second) {
            const XY first{std::get<5>(r.front()), std::get<6>(r.front())};
            const XY last{std::get<7>(r.back()), std::get<8>(r.back())};
            if (first != last) now.emplace_back(first, last);
            using Wide = boost::multiprecision::int256_t;
            for (size_t i = 0; i < r.size(); ++i) {
                const auto &a = r[i]; ++atoms;
                stationary += std::get<5>(a) == std::get<7>(a) && std::get<6>(a) == std::get<8>(a);
                if (i + 1 == r.size()) continue;
                ++internal_vertices;
                const Wide dx = Wide(last.first) - first.first, dy = Wide(last.second) - first.second;
                bends += dx * (Wide(std::get<8>(a)) - first.second) != dy * (Wide(std::get<7>(a)) - first.first);
            }
        }
        const auto actual = plan.reference_lines(z, box, 200000, &budget);
        if (!actual.complete) { ++unavailable; return; }
        ++compared;
        for (const auto &source : actual.sources) {
            REQUIRE(source.kind == BAC::LineSource::Kind::Partition);
            before.push_back({{source.canonical_a.x(), source.canonical_a.y()}, {source.canonical_b.x(), source.canonical_b.y()}});
        }
        canonical += now.size(); old += before.size(); order_different += now != before;
        const auto delta = diff_counts(std::move(before), std::move(now));
        only_old += delta.first; only_new += delta.second;
    }
    Json json() const { return {{"compared_layers",compared},{"unavailable_layers",unavailable},{"canonical_nonstationary_outer_axes",canonical},
        {"legacy_outer_axes",old},{"canonical_only_axes",only_new},{"legacy_only_axes",only_old},{"ordered_axis_sequence_changed_layers",order_different},
        {"all_raw_atoms",atoms},{"stationary_atoms",stationary},{"internal_vertices",internal_vertices},{"off_outer_chord_vertices",bends}}; }
};

void qualify(double width, double depth, const char *name)
{
    ExportWorkspace workspace;
    auto config = export_config(); // Same settings as BACQualification, including 5% base density and G=5.
    Model model;
    auto *m = model.add_object();
    m->name = std::string("BAC qualification ") + name + " bridgeawarecubic";
    m->add_volume(TriangleMesh(its_make_cube(width, depth, 18.)));
    m->add_instance()->set_offset(Vec3d(80., 80., 0.));
    Print print;
    print.set_status_silent(); print.auto_assign_extruders(m); print.apply(model, config);
    print.process(); // All production decisions and actual deposition precede this diagnostic.
    const PrintObject &object = *print.get_object(size_t(0));
    REQUIRE(object.layers().size() <= 128);
    REQUIRE(object.num_printing_regions() == 1);
    const BAC::Plan *plan = object.bridge_aware_cubic_plan(0);
    REQUIRE(plan != nullptr);
    REQUIRE(plan->statistics().nodes <= 100000);
    const auto active = owners(*plan);
    REQUIRE_FALSE(active.empty()); REQUIRE(active.size() <= 2048);
    const std::vector<BAC::CellId> ids(active.begin(), active.end());
    // Deliberately bounded, stratified over actual depth/address-order. The
    // report states the denominator. Trials are independent counterfactuals,
    // never presumed valid/irreducible or installed into the certified object.
    const size_t count = std::min<size_t>(12, ids.size());
    Budget budget; budget.work_left = 50000000; budget.max_records = 200000;
    Slic3r::BridgeAwareCubicValidation::WorkBudget reference_budget;
    Legacy legacy;
    Counts total;
    size_t solid_layers = 0, closure_layers = 0, attachment_layers = 0, plain_layers = 0;
    for (size_t t = 0; t < count; ++t) {
        const auto id = ids[count == 1 ? 0 : t * (ids.size() - 1) / (count - 1)];
        const BAC::Plan trial = plan->collapsed(id);
        const auto survivors = owners(trial);
        REQUIRE(std::includes(active.begin(),active.end(),survivors.begin(),survivors.end()));
        std::vector<BAC::CellId> removed;
        std::set_difference(active.begin(),active.end(),survivors.begin(),survivors.end(),std::back_inserter(removed));
        REQUIRE(std::find(removed.begin(),removed.end(),id) != removed.end());
        Counts changed;
        for (const auto *layer : object.layers()) {
            const double z = layer->print_z;
            CAPTURE(t, id.depth, id.address[0], id.address[1], id.address[2], z);
            if (!t) {
                const bool has_solid = !plan->solid_sections(z).empty(), has_closure = plan->has_closures(z);
                bool has_attachment = false;
                plan->visit_attachments(z, [&](const auto &) { has_attachment = true; return false; });
                solid_layers += has_solid; closure_layers += has_closure; attachment_layers += has_attachment;
                plain_layers += !has_solid && !has_closure && !has_attachment;
            }
            auto program = plan->strand_program(z, budget);
            const auto before = image(program, budget);
            if (!t) legacy.inspect(*plan, before, z, object.bounding_box(), reference_budget);
            const auto before_toggle = budget.work_left;
            std::set<Plane> touched;
            std::vector<BAC::CellId> live;
            for (const auto &owner : removed) if (program.contains_owner(owner, budget)) {
                live.push_back(owner);
                const auto changes = program.set_owner(owner, false, budget);
                REQUIRE(changes.size() <= 3);
                touched.insert(changes.begin(), changes.end());
            }
            Image incremental = before;
            update(incremental, program, touched, budget);
            changed.toggle_kernel_units += before_toggle - budget.work_left;
            const auto before_fresh = budget.work_left;
            const auto rebuilt = trial.strand_program(z, budget);
            const auto fresh = image(rebuilt, budget);
            changed.fresh_kernel_units += before_fresh - budget.work_left;
            // Compare full ordered plane/run/atom records and all integer
            // breakpoints; sizes, hashes or unchanged outer endpoints alone
            // cannot establish toggle equality.
            REQUIRE(bool(incremental == fresh));
            REQUIRE(bool(image(program,budget) == fresh));
            count_delta(before, fresh, touched.size(), changed);
            const auto before_restore = budget.work_left;
            std::set<Plane> restored;
            for (auto it = live.rbegin(); it != live.rend(); ++it) {
                const auto changes = program.set_owner(*it, true, budget);
                restored.insert(changes.begin(),changes.end());
            }
            update(incremental, program, restored, budget);
            changed.restore_kernel_units += before_restore - budget.work_left;
            REQUIRE(bool(incremental == before));
            REQUIRE(bool(image(program,budget) == before));
            ++changed.rollback_checks;
        }
        total.add(changed);
        Json result = {{"fixture",name},{"trial",t},{"cell_depth",id.depth},{"cell_address",id.address},
            {"removed_split_owners",removed.size()},{"remaining_split_owners",survivors.size()}};
        changed.publish(result);
        std::cout << "BAC_CANONICAL_RAW_TRIAL " << result.dump() << '\n';
    }
    REQUIRE(total.changed_layers > 0);
    REQUIRE(total.removed_faces > 0);
    REQUIRE(total.changed_planes <= total.touched_planes);
    REQUIRE(total.rollback_checks == count * object.layers().size());
    REQUIRE(legacy.compared > 0);
    REQUIRE(legacy.old > 0);
    Json summary = {{"fixture",name},{"qualification", "raw geometry only; not a deposited-material certificate"},
        {"actual_layers",object.layers().size()},{"available_split_ids",ids.size()},{"sampled_split_ids",count},
        {"baseline_guard_layers",{{"solid",solid_layers},{"closure",closure_layers},{"attachment",attachment_layers},
            {"no_active_payload",plain_layers}}}, {"legacy_broad_reference",legacy.json()},
        {"strand_total_work_used",50000000-budget.work_left},{"reference_work_used",50000000-reference_budget.remaining_work},
        {"actual_deposition_changed",false}};
    total.publish(summary);
    std::cout << "BAC_CANONICAL_RAW_SUMMARY " << summary.dump() << std::endl;
}
} // namespace canonical_plan_diagnostic

TEST_CASE("Canonical raw plane program matches actual qualification Plan collapse updates - small",
          "[.][BACCanonicalRealPlan][BACCanonicalRealPlanSmall]")
{ canonical_plan_diagnostic::qualify(32.,28.,"small"); }

TEST_CASE("Canonical raw plane program matches actual qualification Plan collapse updates - large",
          "[.][BACCanonicalRealPlan][BACCanonicalRealPlanLarge]")
{ canonical_plan_diagnostic::qualify(64.,56.,"large"); }

namespace canonical_fill_diagnostic {
namespace BAC = Slic3r::BridgeAwareCubic;
using namespace canonical_plan_diagnostic;
struct Call {
    std::shared_ptr<const void> plan_identity;
    size_t emitter = 0, mixed_paths = 0, mixed_segments = 0;
    double z = 0., width = 0.;
    bool missing_inset = false;
    ExPolygons overlap, allowed;
};
struct LayerCalls { size_t units = 0; bool unavailable = false; std::vector<Call> calls; };
class Capture final : public BAC::DomainProbe {
public:
    // Own only numeric runtime identities; a delayed unrelated invocation
    // must never dereference the old target Print or PrintObject.
    const ObjectID target_print, target_object;
    std::mutex mutex;
    std::map<const Layer *, LayerCalls> layers;
    size_t units = 0, work_left = 50000000, begins = 0;
    bool overflow = false;
    explicit Capture(const PrintObject &o):target_print(o.print()->id()),target_object(o.id()) {}
    bool begin_layer(const Layer &layer) override {
        const auto *candidate = layer.object(); // belongs to the live caller
        if (!candidate || candidate->id() != target_object || !candidate->print() ||
            candidate->print()->id() != target_print) return false;
        std::lock_guard<std::mutex> lock(mutex);
        if (overflow) return false;
        try {
            if (!layers.count(&layer) && layers.size() == 128) { overflow = true; return false; }
            auto &r = layers[&layer]; units -= r.units; r = {}; ++begins;
        } catch (const std::bad_alloc &) { overflow = true; return false; }
        return true;
    }
    void surface(const Layer &layer,size_t region,const BAC::Plan &plan,double z,double width,
        const ExPolygons &overlap,const ExPolygons *allowed,const Polylines &output) override {
        std::lock_guard<std::mutex> lock(mutex);
        auto found = layers.find(&layer);
        if (overflow || found == layers.end() || found->second.unavailable) return;
        auto &row = found->second;
        const auto decline = [&] { units -= row.units; row = {}; row.unavailable = true; };
        // Logical records/vertices only; the observer does not claim a total
        // native allocator/RSS bound. Old generations are destroyed at begin.
        size_t need = 16;
        const auto admit = [&](size_t n) {
            if (n > 250000 - need || n > work_left) return false;
            need += n; work_left -= n; return true;
        };
        const auto shape_units = [&](const ExPolygons &ps) {
            if (!admit(ps.size())) return false;
            for (const auto &p : ps) {
                if (!admit(p.contour.size()) || !admit(p.holes.size())) return false;
                for (const auto &h : p.holes) if (!admit(h.size())) return false;
            }
            return true;
        };
        if (row.calls.size() == 128 || !admit(output.size()) || !shape_units(overlap) ||
            (allowed && !shape_units(*allowed)) || need > 250000-units) { decline(); return; }
        try {
            Call call; call.plan_identity = plan.immutable_identity(); call.emitter=region; call.z=z; call.width=width;
            call.mixed_paths = output.size();
            for (const auto &p : output) if (p.size() > 0) call.mixed_segments += p.size()-1;
            call.overlap=overlap; call.missing_inset=allowed==nullptr; if (allowed) call.allowed=*allowed;
            row.calls.push_back(std::move(call)); row.units += need; units += need;
        } catch (const std::bad_alloc &) { decline(); }
    }
};
struct Install {
    std::shared_ptr<BAC::DomainProbe> installed;
    explicit Install(std::shared_ptr<BAC::DomainProbe> probe) : installed(std::move(probe)) {
        if (!installed) throw std::invalid_argument("Cannot install a null fill-domain observer");
        std::shared_ptr<BAC::DomainProbe> empty;
        if (!std::atomic_compare_exchange_strong_explicit(&BAC::diagnostic_domain_probe,&empty,installed,
                std::memory_order_acq_rel,std::memory_order_acquire))
            throw std::logic_error("A fill-domain observer is already installed");
    }
    Install(const Install &) = delete;
    Install &operator=(const Install &) = delete;
    ~Install() {
        auto expected=installed;
        // Remove only this scope's registration; never clear another owner.
        std::atomic_compare_exchange_strong_explicit(&BAC::diagnostic_domain_probe,&expected,
            std::shared_ptr<BAC::DomainProbe>{},std::memory_order_acq_rel,std::memory_order_acquire);
    }
};
using Key = std::tuple<int64_t,bool,int64_t,bool,BAC::CellId>;
Key key(const BAC::StrandAtom &a) { return {a.first.slot,a.first.shifted,a.last.slot,a.last.shifted,a.owner}; }
using Fraction = std::pair<int64_t,int64_t>;
using Edge = std::tuple<Key,Fraction,Fraction,size_t,XY,XY>;
using CRun = std::vector<Edge>;
using CRows = std::vector<CRun>;
using CImage = std::map<Plane,CRows>;
using Wide = boost::multiprecision::int256_t;
int compare(const Fraction &a,const Fraction &b) {
    const Wide diff=Wide(a.first)*b.second-Wide(b.first)*a.second;
    return diff < 0 ? -1 : diff > 0 ? 1 : 0;
}
struct Inventory {
    size_t fragments=0,stationary=0,material_components_used=0,raw_stationary=0;
    size_t positive_gaps=0,coincident_gap_ends=0,different_material_splits=0;
};
struct Incidence { size_t run, ordinal; XY a,b; };
using Incidences = std::map<Key,Incidence>;
Incidences incidence(const BAC::StrandProgram &raw,const Plane &p,Budget &budget,Inventory *inv) {
    Incidences out; size_t id=0;
    for (const auto &r:raw.runs(p,budget)) {
        REQUIRE(r.points.size()==r.atoms.size()+1);
        for (size_t i=0;i<r.atoms.size();++i) {
            REQUIRE(out.emplace(key(r.atoms[i]),Incidence{id,i,{r.points[i].x(),r.points[i].y()},
                {r.points[i+1].x(),r.points[i+1].y()}}).second);
            if (inv) inv->raw_stationary += r.points[i]==r.points[i+1];
        }
        ++id;
    }
    return out;
}
template<class View> CRows clip_rows(const View &view,const BAC::StrandProgram &raw,
    const Plane &p,size_t component_count,Budget &budget,Inventory *inv=nullptr) {
    CRows out; const auto sources=incidence(raw,p,budget,inv);
    std::optional<Edge> previous; size_t previous_run=0;
    std::set<size_t> used_components;
    for (const auto &r:view.runs(p,budget)) {
        REQUIRE_FALSE(r.atoms.empty()); REQUIRE(r.points.size()==r.atoms.size()+1);
        auto &dst=out.emplace_back();
        for (size_t i=0;i<r.atoms.size();++i) {
            const auto &a=r.atoms[i]; const Key k=key(a.source);
            const Fraction f{a.first.numerator,a.first.denominator},l{a.last.numerator,a.last.denominator};
            REQUIRE(f.second>0); REQUIRE(l.second>0); REQUIRE(f.first>=0); REQUIRE(l.first<=l.second);
            REQUIRE(std::gcd(f.first,f.second)==1); REQUIRE(std::gcd(l.first,l.second)==1); REQUIRE(compare(f,l)<0);
            REQUIRE(a.material_component < component_count);
            const auto src=sources.find(k); REQUIRE(src!=sources.end());
            XY begin{r.points[i].x(),r.points[i].y()},end{r.points[i+1].x(),r.points[i+1].y()};
            Edge e{k,f,l,a.material_component,begin,end};
            if (previous) {
                const auto &v=*previous; const auto &old=sources.at(std::get<0>(v));
                const bool same_source=std::get<0>(v)==k, same_raw_run=old.run==src->second.run;
                const bool same_part=a.material_component==std::get<3>(v);
                const bool consecutive=same_raw_run && old.ordinal+1==src->second.ordinal;
                const bool prev_at_end=std::get<2>(v)==Fraction(1,1), now_at_start=f==Fraction(0,1);
                const bool connected=same_source ? compare(std::get<2>(v),f)==0 : consecutive && prev_at_end && now_at_start;
                if (i) {
                    REQUIRE(same_part); REQUIRE(connected); REQUIRE(std::get<5>(v)==begin);
                } else if (inv && same_raw_run) {
                    const bool forward_gap=same_source ? compare(std::get<2>(v),f)<0 :
                        old.ordinal<src->second.ordinal && (!consecutive||!prev_at_end||!now_at_start);
                    if (forward_gap) { ++inv->positive_gaps; inv->coincident_gap_ends+=std::get<5>(v)==begin; }
                    if (!same_part) ++inv->different_material_splits;
                    REQUIRE(previous_run!=out.size()-1);
                }
            }
            if (inv) { ++inv->fragments; inv->stationary+=begin==end; used_components.insert(a.material_component); }
            dst.push_back(e); previous=std::move(e); previous_run=out.size()-1;
        }
    }
    if (inv) inv->material_components_used+=used_components.size();
    return out;
}
template<class View> CImage clip_image(const View &view,const BAC::StrandProgram &raw,
                                        size_t component_count,Budget &budget,Inventory *inv=nullptr) {
    CImage out;
    for (const auto &p:view.planes(budget)) {
        auto rows=clip_rows(view,raw,p,component_count,budget,inv);
        REQUIRE_FALSE(rows.empty()); REQUIRE(out.emplace(p,std::move(rows)).second);
    }
    return out;
}
template<class View> void update_clip(CImage &img,const View &view,const BAC::StrandProgram &raw,
    const std::set<Plane> &touched,size_t component_count,Budget &budget) {
    for (const auto &p:touched) {
        auto rows=clip_rows(view,raw,p,component_count,budget);
        if (rows.empty()) img.erase(p); else img[p]=std::move(rows);
    }
}
struct Locality {
    size_t base_edges=0,trial_edges=0, removed_edges=0,added_edges=0;
    size_t base_runs=0,trial_runs=0,removed_runs=0,added_runs=0;
    size_t base_rows=0,trial_rows=0,touched_rows=0,changed_rows=0;
    size_t compared=0,changed_domains=0,unavailable_geometry=0;
    void count(const CImage &a,const CImage &b,size_t touched) {
        ++compared; changed_domains+=a!=b; touched_rows+=touched;
        base_rows+=a.size(); trial_rows+=b.size();
        std::set<Plane> all; for (const auto &x:a) all.insert(x.first); for (const auto &x:b) all.insert(x.first);
        const CRows empty;
        for (const auto &p:all) {
            auto x=a.find(p),y=b.find(p); const auto &old=x==a.end()?empty:x->second, &now=y==b.end()?empty:y->second;
            base_runs+=old.size(); trial_runs+=now.size();
            for(const auto &r:old) base_edges+=r.size(); for(const auto &r:now) trial_edges+=r.size();
            if(old==now) continue;
            ++changed_rows; const auto runs=diff_counts(old,now); removed_runs+=runs.first; added_runs+=runs.second;
            std::vector<Edge> first,last; for(const auto &r:old) first.insert(first.end(),r.begin(),r.end());
            for(const auto &r:now) last.insert(last.end(),r.begin(),r.end());
            const auto edges=diff_counts(std::move(first),std::move(last)); removed_edges+=edges.first; added_edges+=edges.second;
        }
    }
    Json json() const { return {{"compared_domain_trials",compared},{"changed_domain_trials",changed_domains},
        {"unavailable_domain_trials",unavailable_geometry},{"fragment_counts",{base_edges,trial_edges,removed_edges,added_edges}},
        {"run_counts",{base_runs,trial_runs,removed_runs,added_runs}},{"row_counts",{base_rows,trial_rows,touched_rows,changed_rows}}}; }
};
void qualify_clip(const std::string &kind)
{
    ExportWorkspace workspace;
    const bool cavity=kind=="cavity",large=kind=="large";
    auto cfg=export_config();
    Model model; auto *m=model.add_object(); m->name="BAC actual fill domains " + kind;
    m->add_volume(TriangleMesh(its_make_cube(cavity?28.:large?64.:32.,cavity?24.:large?56.:28.,cavity?14.:18.)));
    m->add_instance()->set_offset(Vec3d(80.,80.,0.));
    if(cavity) {
        // Same physical input as the HoleClosure acceptance test, including
        // the real negative-volume bore and its 12%/17-degree sparse policy.
        TriangleMesh bore(its_make_cube(3.,3.,9.)); bore.translate(12.5,10.5,-.5);
        m->add_volume(bore)->set_type(ModelVolumeType::NEGATIVE_VOLUME);
        cfg.clear(); cfg.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
        cfg.set_deserialize_strict({{"layer_height","0.2"},{"initial_layer_print_height","0.2"},
            {"sparse_infill_density","12%"},{"sparse_infill_pattern","bridgeawarecubic"},{"bridge_aware_cubic_max_gap","5"},
            {"infill_direction","17"},{"top_shell_layers","3"},{"top_shell_thickness","0"},{"bottom_shell_layers","3"},
            {"bottom_shell_thickness","0"},{"wall_loops","2"},{"wall_generator","classic"},{"detect_narrow_internal_solid_infill","0"},
            {"brim_type","no_brim"},{"enable_arc_fitting","0"},{"thick_bridges","0"},{"bridge_flow","1"},{"fill_multiline","1"},{"seam_gap","0"}});
    }
    Print print; print.set_status_silent(); print.auto_assign_extruders(m); print.apply(model,cfg);
    const auto &object=*print.get_object(size_t(0)); auto owned_capture=std::make_shared<Capture>(object);
    auto &capture=*owned_capture;
    { Install observer(owned_capture); print.process(); } // Target fill jobs have joined; foreign leases cannot match its unique IDs.
    REQUIRE_FALSE(capture.overflow); REQUIRE(object.layers().size()<=128); REQUIRE(object.num_printing_regions()==1);
    const auto *base=object.bridge_aware_cubic_plan(0); REQUIRE(base!=nullptr);
    const auto identity=base->immutable_identity(); const auto active=owners(*base); REQUIRE_FALSE(active.empty());
    REQUIRE(base->statistics().nodes<=100000); REQUIRE(active.size()<=2048);
    const std::vector<BAC::CellId> ids(active.begin(),active.end());
    const size_t trials=std::min<size_t>(6,ids.size());
    Budget budget; budget.work_left=50000000; budget.max_records=200000;
    Locality total; Inventory inventory;
    size_t missing_layers=0,declined_layers=0,surface_calls=0,stale_calls=0,missing_inset=0,empty_inset=0,
        empty_overlaps=0,surfaces=0,overlap_holes=0,allowed_components=0,allowed_holes=0,actual_paths=0,actual_segments=0,
        more_than_one_surface=0,more_than_one_allowed=0,solid_calls=0,closure_calls=0,attachment_calls=0;
    std::map<std::string,size_t> reasons;
    for(size_t ti=0;ti<trials;++ti) {
        const auto id=ids[trials==1?0:ti*(ids.size()-1)/(trials-1)];
        const auto trial=base->collapsed(id); const auto survive=owners(trial);
        REQUIRE(std::includes(active.begin(),active.end(),survive.begin(),survive.end()));
        std::vector<BAC::CellId> remove;
        std::set_difference(active.begin(),active.end(),survive.begin(),survive.end(),std::back_inserter(remove));
        REQUIRE(std::find(remove.begin(),remove.end(),id)!=remove.end());
        const size_t start=total.compared,start_base=total.base_edges,start_remove=total.removed_edges,start_add=total.added_edges;
        for(const Layer *layer:object.layers()) {
            auto layer_capture=capture.layers.find(layer);
            if(layer_capture==capture.layers.end()) { if(!ti) ++missing_layers; continue; }
            if(layer_capture->second.unavailable) { if(!ti) ++declined_layers; continue; }
            size_t ordinal=0;
            for(const Call &call:layer_capture->second.calls) {
                ++ordinal; if(!ti) ++surface_calls;
                if(call.emitter!=0 || call.z!=layer->print_z || !(call.width>0.) || !std::isfinite(call.width) || call.plan_identity!=identity) {
                    if(!ti) ++stale_calls; continue;
                }
                if(call.missing_inset) { if(!ti) ++missing_inset; continue; }
                if(!ti) {
                    actual_paths+=call.mixed_paths; actual_segments+=call.mixed_segments;
                    surfaces+=call.overlap.size(); allowed_components+=call.allowed.size();
                    empty_overlaps+=call.overlap.empty(); empty_inset+=call.allowed.empty();
                    more_than_one_surface+=call.overlap.size()>1; more_than_one_allowed+=call.allowed.size()>1;
                    for(const auto &x:call.overlap) overlap_holes+=x.holes.size();
                    for(const auto &x:call.allowed) allowed_holes+=x.holes.size();
                    solid_calls+=!base->solid_sections(call.z).empty(); closure_calls+=base->has_closures(call.z);
                    bool attached=false; base->visit_attachments(call.z,[&](const auto &){attached=true;return false;});
                    attachment_calls+=attached;
                }
                for(size_t component=0;component<call.overlap.size();++component) {
                    CAPTURE(ti,id.depth,id.address[0],id.address[1],id.address[2],call.z,ordinal,component);
                    const auto &surface=call.overlap[component];
                    auto raw=base->strand_program(call.z,budget);
                    std::optional<BAC::ClippedStrands> view;
                    try { view=raw.clipped(surface,call.allowed,budget); }
                    catch(const std::domain_error &e) {
                        ++total.unavailable_geometry;
                        if(reasons.size()<16 || reasons.count(e.what())) ++reasons[e.what()];
                        continue; // No partial view, no representation switch, no physical certificate.
                    }
                    auto before=clip_image(*view,raw,call.allowed.size(),budget,ti==0?&inventory:nullptr);
                    std::set<Plane> touched; std::vector<BAC::CellId> live;
                    for(const auto &who:remove) if(raw.contains_owner(who,budget)) {
                        live.push_back(who); auto v=raw.set_owner(who,false,budget); touched.insert(v.begin(),v.end());
                    }
                    auto updated=before; update_clip(updated,*view,raw,touched,call.allowed.size(),budget);
                    const auto fresh_raw=trial.strand_program(call.z,budget);
                    // A positive baseline catalog contains the surviving atoms;
                    // never hide an unexpected failure after an owner removal.
                    const auto fresh_view=fresh_raw.clipped(surface,call.allowed,budget);
                    const auto fresh=clip_image(fresh_view,fresh_raw,call.allowed.size(),budget);
                    REQUIRE(bool(updated==fresh)); REQUIRE(bool(clip_image(*view,raw,call.allowed.size(),budget)==fresh));
                    total.count(before,fresh,touched.size());
                    std::set<Plane> restore;
                    for(auto it=live.rbegin();it!=live.rend();++it) {
                        auto v=raw.set_owner(*it,true,budget); restore.insert(v.begin(),v.end());
                    }
                    update_clip(updated,*view,raw,restore,call.allowed.size(),budget);
                    REQUIRE(bool(updated==before)); REQUIRE(bool(clip_image(*view,raw,call.allowed.size(),budget)==before));
                }
            }
        }
        std::cout<<"BAC_CANONICAL_CLIP_TRIAL "<<Json({{"fixture",kind},{"trial",ti},{"cell_depth",id.depth},{"cell_address",id.address},
            {"removed_split_ids",remove.size()},{"compared_domains",total.compared-start},{"base_fragments",total.base_edges-start_base},
            {"removed_fragments",total.removed_edges-start_remove},{"added_fragments",total.added_edges-start_add}}).dump()<<'\n';
    }
    Json report={{"fixture",kind},{"scope","actual filler masks, canonical partition atoms only; no emitted support verdict"},
        {"layer_generation_begins",capture.begins},{"last_generation_surface_calls",surface_calls},{"missing_layer_begin",missing_layers},
        {"capture_quota_declined_layers",declined_layers},{"stale_or_incompatible_calls",stale_calls},
        {"missing_inset_calls",missing_inset},{"empty_inset_calls",empty_inset},{"empty_overlap_calls",empty_overlaps},
        {"overlap_components",surfaces},{"overlap_holes",overlap_holes},{"inset_components",allowed_components},{"inset_holes",allowed_holes},
        {"multi_overlap_calls",more_than_one_surface},{"multi_inset_calls",more_than_one_allowed},
        {"baseline_solid_calls",solid_calls},{"baseline_closure_calls",closure_calls},{"baseline_attachment_calls",attachment_calls},
        {"actual_legacy_mixed_paths",actual_paths},{"actual_legacy_mixed_segments",actual_segments},
        {"baseline_clipped_atoms",inventory.fragments},{"baseline_clipped_stationary",inventory.stationary},
        {"raw_stationary_on_nonempty_rows",inventory.raw_stationary},{"observed_exact_gaps",inventory.positive_gaps},
        {"coincident_gap_ends",inventory.coincident_gap_ends},{"different_material_splits",inventory.different_material_splits},
        {"checked_clipped_locality",total.json()},{"active_cells",ids.size()},{"sampled_trials",trials},
        {"unavailable_geometry_reasons",reasons},{"work_used",50000000-budget.work_left}};
    std::cout<<"BAC_CANONICAL_CLIP_SUMMARY "<<report.dump()<<std::endl;
    REQUIRE(surface_calls>0); REQUIRE(surfaces>0); REQUIRE(total.compared>0); REQUIRE(inventory.fragments>0);
    REQUIRE(total.changed_domains>0); REQUIRE(total.removed_edges>0); REQUIRE(total.changed_rows<=total.touched_rows);
    if(cavity) { REQUIRE(allowed_holes>0); REQUIRE(overlap_holes>0); }
}
}
TEST_CASE("Canonical clipped planes match actual small qualification fill domain edits", "[.][BACCanonicalRealClip][BACCanonicalRealClipSmall]")
{ canonical_fill_diagnostic::qualify_clip("small"); }
TEST_CASE("Canonical clipped planes match actual large qualification fill domain edits", "[.][BACCanonicalRealClip][BACCanonicalRealClipLarge]")
{ canonical_fill_diagnostic::qualify_clip("large"); }
TEST_CASE("Canonical clipped planes match real blind cavity fill domain edits", "[.][BACCanonicalRealClip][BACCanonicalRealClipCavity]")
{ canonical_fill_diagnostic::qualify_clip("cavity"); }

TEST_CASE("Actual fill domain observer leases survive uninstall and reject nested installations",
          "[.][BACCanonicalDomainProbeLease]")
{
    using canonical_fill_diagnostic::Capture;
    using canonical_fill_diagnostic::Install;
    using Probe=Slic3r::BridgeAwareCubic::DomainProbe;
    const auto make_object=[](Print &print,Model &model) {
        auto *m=model.add_object();
        m->add_volume(TriangleMesh(its_make_cube(3.,3.,3.)));
        m->add_instance()->set_offset(Vec3d(80.,80.,0.));
        print.set_status_silent(); print.auto_assign_extruders(m); print.apply(model,export_config());
        return print.get_object(size_t(0));
    };
    Model foreign_model; Print foreign_print;
    auto *foreign=make_object(foreign_print,foreign_model);
    auto *foreign_layer=foreign->add_layer(0,.2,.2,.1);
    auto replacement=std::make_shared<Capture>(*foreign);
    std::weak_ptr<Probe> original;
    std::shared_ptr<Probe> delayed;
    {
        Model target_model; Print target_print;
        auto *target=make_object(target_print,target_model);
        auto capture=std::make_shared<Capture>(*target); original=capture;
        {
            Install active(capture);
            // Equivalent to a caller pausing immediately after make_fills' load.
            delayed=std::atomic_load_explicit(&Slic3r::BridgeAwareCubic::diagnostic_domain_probe,std::memory_order_acquire);
            REQUIRE(delayed==capture);
            REQUIRE_THROWS_AS(Install{replacement},std::logic_error);
            REQUIRE(std::atomic_load(&Slic3r::BridgeAwareCubic::diagnostic_domain_probe)==capture);
        }
        REQUIRE_FALSE(std::atomic_load(&Slic3r::BridgeAwareCubic::diagnostic_domain_probe));
    } // Original target Print, target PrintObject, and caller-owned capture are gone.
    REQUIRE_FALSE(original.expired());
    {
        Install active(replacement);
        REQUIRE_FALSE(delayed->begin_layer(*foreign_layer));
        REQUIRE(replacement->begin_layer(*foreign_layer));
        REQUIRE(replacement->begins==1);
        delayed.reset();
        REQUIRE(original.expired());
        REQUIRE(std::atomic_load(&Slic3r::BridgeAwareCubic::diagnostic_domain_probe)==replacement);
    }
    REQUIRE_FALSE(std::atomic_load(&Slic3r::BridgeAwareCubic::diagnostic_domain_probe));
}
