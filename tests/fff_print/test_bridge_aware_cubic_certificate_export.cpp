#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Fill/BridgeAwareCubic.hpp"
#include "libslic3r/Fill/BridgeAwareCubicCertificate.hpp"
#include "libslic3r/Fill/BridgeAwareCubicContact.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSerialization.hpp"
#include "libslic3r/Fill/BridgeAwareCubicSupport.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils.hpp"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Contact = Slic3r::BridgeAwareCubicContact;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
using Json = nlohmann::json;

struct CertificateWorkspace {
    boost::filesystem::path directory = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("bridge-aware-certificate-%%%%-%%%%");
    std::string old_data = data_dir(), old_temp = temporary_dir();
    std::map<std::string, std::optional<std::string>> environment;
    size_t sequence = 0;

    static int set(const char *name, const char *value)
    {
#ifdef _WIN32
        return _putenv_s(name, value == nullptr ? "" : value);
#else
        return value == nullptr ? unsetenv(name) : setenv(name, value, 1);
#endif
    }

    CertificateWorkspace()
    {
        for (const char *name : {"BAC_PROFILE_FILE", "BAC_PROFILE_STAGE", "BAC_PROFILE_FULL_TRIAL_CHECKS"}) {
            const char *value = std::getenv(name);
            environment.emplace(name, value == nullptr ? std::nullopt : std::optional<std::string>(value));
        }
        boost::filesystem::create_directories(directory);
        set_data_dir(directory.string());
        set_temporary_dir(directory.string());
    }

    ~CertificateWorkspace()
    {
        for (const auto &entry : environment)
            set(entry.first.c_str(), entry.second ? entry.second->c_str() : nullptr);
        set_data_dir(old_data);
        set_temporary_dir(old_temp);
        boost::system::error_code error;
        boost::filesystem::remove_all(directory, error);
    }

    void clear_profile_controls() const
    {
        // Change process state only after the restoring guard is fully alive.
        for (const auto &entry : environment)
            REQUIRE(set(entry.first.c_str(), nullptr) == 0);
    }

    std::string select_profile()
    {
        const std::string path = (directory / (std::to_string(++sequence) + ".jsonl")).string();
        REQUIRE_FALSE(boost::filesystem::exists(path));
        REQUIRE(set("BAC_PROFILE_FILE", path.c_str()) == 0);
        REQUIRE(set("BAC_PROFILE_STAGE", "export") == 0);
        return path;
    }
};

DynamicPrintConfig certificate_config()
{
    // The existing small qualification fixture, with no extra slicer mode.
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

void visit_paths(ExtrusionEntity &entity, const std::function<void(ExtrusionPath &)> &visit)
{
    if (auto *path = dynamic_cast<ExtrusionPath *>(&entity)) {
        if (!path->is_force_no_extrusion() && path->mm3_per_mm > 0. && path->polyline.size() >= 2)
            visit(*path);
    } else if (auto *multi = dynamic_cast<ExtrusionMultiPath *>(&entity)) {
        for (auto &path : multi->paths) visit_paths(path, visit);
    } else if (auto *loop = dynamic_cast<ExtrusionLoop *>(&entity)) {
        for (auto &path : loop->paths) visit_paths(path, visit);
    } else if (auto *collection = dynamic_cast<ExtrusionEntityCollection *>(&entity)) {
        for (auto *child : collection->entities) visit_paths(*child, visit);
    }
}

void visit_layer(Layer &layer, const std::function<void(ExtrusionPath &)> &visit)
{
    for (LayerRegion *region : layer.regions()) {
        visit_paths(region->perimeters, visit);
        visit_paths(region->fills, visit);
        visit_paths(region->thin_fills, visit);
    }
}

BAC::SerializedXY phase_for(const PrintObject &object)
{
    REQUIRE(object.instances().size() == 1);
    BAC::SerializedXY phase;
    phase.shift = unscale(Vec2crd(object.instances().front().shift));
    phase.nozzle_offset = object.print()->config().extruder_offset.get_at(0);
    phase.plate_origin = object.print()->get_plate_origin().head<2>();
    return phase;
}

ExtrusionPath emitted(const ExtrusionPath &source, const BAC::SerializedXY &phase)
{
    ExtrusionPath path = source;
    path.polyline = phase.emitted_polyline(source.polyline);
    path.polyline.translate(-phase(Point(0, 0)));
    return path;
}

struct PhysicalTarget {
    Layer *lower = nullptr, *upper = nullptr;
    ExtrusionPath *path = nullptr;
    ExPolygons demand;
};

struct PhysicalResult {
    Contact::PathContact contact;
    Validation::Result checked;
};

PhysicalResult prove(const PhysicalTarget &target, const BAC::SerializedXY &phase)
{
    ExtrusionPaths lower;
    visit_layer(*target.lower, [&](ExtrusionPath &path) { lower.push_back(emitted(path, phase)); });
    std::vector<Contact::Bead> beads;
    for (const ExtrusionPath &path : lower)
        beads.push_back({&path, GCodeWriter::serialized_z(target.lower->print_z, true)});
    const ExtrusionPath upper = emitted(*target.path, phase);
    PhysicalResult result;
    result.contact = Contact::footprint(beads, upper,
        GCodeWriter::serialized_z(target.upper->print_z, true), 0.);
    REQUIRE(result.contact.input_valid);
    REQUIRE(result.contact.completed);
    const ExPolygons demand = BAC::serialized_demand(target.demand, {target.path->polyline}, phase);
    REQUIRE_FALSE(demand.empty());
    result.checked = Validation::validate_contacts(
        BAC::support_topology_domain(result.contact.physical_footprint, demand),
        result.contact.physical_footprint, {upper.polyline}, demand, 5., 0., {result.contact});
    return result;
}

PhysicalTarget direct_roof_target(PrintObject &object)
{
    const auto phase = phase_for(object);
    size_t eligible_layers = 0, roof_paths = 0, long_crossings = 0, passing_candidates = 0;
    std::ostringstream evidence;
    evidence << std::setprecision(17);
    for (size_t i = 1; i < object.layers().size(); ++i) {
        Layer &lower = *object.get_layer(i - 1), &upper = *object.get_layer(i);
        ExPolygons sparse, roof;
        for (const Surface &surface : lower.get_region(0)->fill_surfaces.surfaces)
            if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
                sparse.push_back(surface.expolygon);
        for (const Surface &surface : upper.get_region(0)->fill_surfaces.surfaces)
            if (surface.surface_type == stInternalBridge && !surface.bridge_followup)
                roof.push_back(surface.expolygon);
        const ExPolygons eligible = intersection_ex(sparse, roof);
        if (eligible.empty()) continue;
        ++eligible_layers;
        PhysicalTarget selected;
        visit_paths(upper.get_region(0)->fills, [&](ExtrusionPath &path) {
            if (path.role() == erBridgeInfill) ++roof_paths;
            if (selected.path != nullptr || path.role() != erBridgeInfill ||
                path.polyline.length() * SCALING_FACTOR <= 5.) return;
            const auto crossing = intersection_pl(Polylines{path.polyline}, eligible);
            double selected_length = 0.;
            for (const Polyline &part : crossing) selected_length += part.length() * SCALING_FACTOR;
            if (selected_length <= 5.) return;
            ++long_crossings;
            PhysicalTarget candidate{&lower, &upper, &path,
                intersection_ex(union_ex(offset(path.polyline, scale_(.5 * path.width))), eligible)};
            const auto actual = prove(candidate, phase);
            passing_candidates += actual.checked.passed();
            if (long_crossings <= 8)
                evidence << "layer=" << i << " lower_z=" << lower.print_z << " upper_z=" << upper.print_z
                    << " width=" << path.width << " height=" << path.height
                    << " contact_z=" << actual.contact.contact_z << " passed=" << actual.checked.passed()
                    << " input_valid=" << actual.checked.input_valid << " completed=" << actual.checked.completed
                    << " max_run=" << actual.checked.max_unsupported_mm
                    << " max_cell=" << actual.checked.max_void_diameter_mm << '\n';
            // Base internal bridges use thick flow even when external thick
            // bridges are disabled. Their valid contact plane can be inside
            // the lower bead; the complete physical proof is authoritative.
            if (actual.checked.passed()) selected = std::move(candidate);
        });
        if (selected.path != nullptr) return selected;
    }
    CAPTURE(eligible_layers, roof_paths, long_crossings, passing_candidates);
    INFO(evidence.str());
    FAIL("The qualified cube must contain a passing direct roof over more than 5 mm of actual sparse material.");
    return {};
}

struct RestoreLayer {
    Layer &layer;
    const double z;
    std::vector<std::pair<ExtrusionPath *, ExtrusionPath>> paths;
    explicit RestoreLayer(Layer &source) : layer(source), z(source.print_z)
    {
        visit_layer(layer, [&](ExtrusionPath &path) { paths.emplace_back(&path, path); });
        REQUIRE_FALSE(paths.empty());
    }
    ~RestoreLayer()
    {
        for (auto &entry : paths) *entry.first = entry.second;
        layer.print_z = z;
    }
};

std::string contents(const std::string &filename)
{
    std::ifstream input(filename, std::ios::binary);
    REQUIRE(input.is_open());
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::map<uint64_t, Json> validate_records(const std::string &filename)
{
    std::ifstream input(filename);
    REQUIRE(input.is_open());
    std::map<uint64_t, Json> result;
    std::string line;
    while (std::getline(input, line)) {
        const Json record = Json::parse(line);
        if (record.at("record") == "total" && record.at("session") == "validate") {
            REQUIRE(record.at("stage") == "export");
            REQUIRE_FALSE(record.at("incomplete").get<bool>());
            REQUIRE(result.emplace(record.at("object").get<uint64_t>(), record).second);
        }
    }
    return result;
}

Json validate_record(const std::string &filename)
{
    const auto records = validate_records(filename);
    REQUIRE(records.size() == 1);
    return records.begin()->second;
}

size_t calls(const Json &record, const char *name)
{
    size_t count = 0;
    for (const auto &metric : record.at("metrics"))
        if (metric.at("name") == name) count += metric.at("calls").get<size_t>();
    return count;
}

void require_hit(const Json &record)
{
    REQUIRE(record.at("final_proof_hits").get<size_t>() == 1);
    REQUIRE(record.at("final_proof_misses").get<size_t>() == 0);
    REQUIRE(calls(record, "check_model") + calls(record, "check_construction") == 0);
}

void require_renewal(const Json &record)
{
    REQUIRE(record.at("final_proof_hits").get<size_t>() == 0);
    REQUIRE(record.at("final_proof_misses").get<size_t>() == 1);
    REQUIRE(record.at("final_proof_publications").get<size_t>() == 1);
    REQUIRE(record.at("final_proof_bytes").get<size_t>() > 0);
    REQUIRE(calls(record, "check_model") > 0);
    REQUIRE(calls(record, "collect_construction") > 0);
    REQUIRE(calls(record, "check_model") + calls(record, "check_construction") >= 2);
}
} // namespace

TEST_CASE("Bridge aware cubic public export rejects stale deposition proofs and renews benign state",
          "[BridgeAwareCubic][Integration][Export][BACCertificateExport]")
{
    CertificateWorkspace workspace;
    workspace.clear_profile_controls();
    Model model;
    auto *part = model.add_object();
    part->name = "Bridge aware cubic certificate mutation cube";
    part->add_volume(TriangleMesh(its_make_cube(32., 28., 18.)));
    part->add_instance()->set_offset(Vec3d(80., 80., 0.));
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(part);
    print.apply(model, certificate_config());
    // Deliberately one slice: every later operation must inspect current
    // mutable paths through export, without rebuilding away the corruption.
    REQUIRE_NOTHROW(print.process());
    REQUIRE(print.objects().size() == 1);
    PrintObject &object = *print.get_object(size_t(0));
    REQUIRE(object.layers().size() == 90);
    REQUIRE(object.num_printing_regions() == 1);
    const std::string filename = (workspace.directory / "same-file.gcode").string();
    const auto export_current = [&] {
        const std::string profile = workspace.select_profile();
        REQUIRE_NOTHROW(print.export_gcode(filename, nullptr));
        REQUIRE(boost::filesystem::is_regular_file(filename));
        return validate_record(profile);
    };
    export_current();
    // Actual emission may reorder mutable source collections. Warm the exact
    // post-export state before asserting reuse on the next untouched call.
    export_current();
    require_hit(export_current());
    // Locate the witness in the final post-emission tree, so no borrowed path
    // pointer survives the first exporter pass that can rearrange that tree.
    const PhysicalTarget target = direct_roof_target(object);
    REQUIRE(prove(target, phase_for(object)).checked.passed());

    for (const std::string mutation : {"points", "width", "Z"}) {
        CAPTURE(mutation, target.lower->id(), target.lower->print_z, target.upper->print_z);
        require_hit(export_current());
        const std::string original_file = contents(filename);
        {
            RestoreLayer restore(*target.lower);
            if (mutation == "points") {
                for (auto &entry : restore.paths) entry.first->polyline.translate(Point::new_scale(100., 100.));
            } else if (mutation == "width") {
                // Reduce real lower cross sections while preserving valid
                // circular dimensions. Overlapping thick roofs need not
                // meet these beads at a zero-width tangent plane.
                for (auto &entry : restore.paths) entry.first->width = entry.first->height;
            } else target.lower->print_z =
                GCodeWriter::serialized_z(target.upper->print_z, true) - target.path->height - .01;
            auto broken = prove(target, phase_for(object));
            bool reduced_height = false;
            if (mutation == "width" && broken.checked.passed()) {
                // A stronger, still positive circular bead is needed when
                // the original lattice has enough margin after width alone
                // changes. At two geometry units its finite wall is below
                // the contact model's conservative material resolution.
                for (auto &entry : restore.paths) {
                    entry.first->width = entry.first->height = float(2. * SCALING_FACTOR);
                    entry.first->mm3_per_mm = PI * double(entry.first->width) * entry.first->height * .25;
                }
                reduced_height = true;
                broken = prove(target, phase_for(object));
            }
            CAPTURE(reduced_height, broken.contact.contact_z, broken.checked.max_unsupported_mm,
                    broken.checked.max_void_diameter_mm);
            REQUIRE(broken.checked.input_valid);
            REQUIRE(broken.checked.completed);
            REQUIRE_FALSE(broken.checked.passed());
            if (reduced_height) {
                REQUIRE(broken.contact.physical_footprint.empty());
                REQUIRE_FALSE(broken.checked.void_cells.empty());
            } else if (mutation != "width") REQUIRE_FALSE(broken.checked.unsupported_runs.empty());
            workspace.select_profile();
            REQUIRE_THROWS_AS(print.export_gcode(filename, nullptr), SlicingError);
            CHECK(contents(filename) == original_file);
        }
        REQUIRE(prove(target, phase_for(object)).checked.passed());
        export_current();
        require_hit(export_current());
    }

    // The first layer has no preceding-layer demand. A fitted positive path
    // there must still invalidate a warm record and fail complete preflight.
    {
        auto &fills = object.get_layer(0)->get_region(0)->fills;
        auto *extra = new ExtrusionPath(erInternalInfill, .08, .4f, .2f);
        extra->polyline = Polyline::new_scale({Vec2d(0., 0.), Vec2d(1., 0.)});
        extra->polyline.reset_to_linear_move();
        REQUIRE_FALSE(extra->polyline.fitting_result.empty());
        fills.entities.push_back(extra);
        struct RemoveExtra {
            ExtrusionEntityCollection &fills;
            ~RemoveExtra() { delete fills.entities.back(); fills.entities.pop_back(); }
        } remove{fills};
        workspace.select_profile();
        REQUIRE_THROWS_AS(print.export_gcode(filename, nullptr), SlicingError);
    }
    export_current();
    require_hit(export_current());

    print.cancel();
    workspace.select_profile();
    CHECK_THROWS_AS(print.export_gcode(filename, nullptr), CanceledException);
    print.restart();
    require_hit(export_current());

    const auto old_phase = phase_for(object);
    object.instances().front().shift.x() += scale_(1.);
    const auto new_phase = phase_for(object);
    REQUIRE(new_phase.shift != old_phase.shift);
    size_t unchanged_paths = 0;
    for (Layer *layer : object.layers())
        visit_layer(*layer, [&](ExtrusionPath &path) {
            REQUIRE(emitted(path, old_phase).polyline.points == emitted(path, new_phase).polyline.points);
            ++unchanged_paths;
        });
    REQUIRE(unchanged_paths > 100);
    REQUIRE(prove(target, new_phase).checked.passed());
    require_renewal(export_current());
    require_hit(export_current());

    target.path->overhang_degree += .125;
    REQUIRE(prove(target, phase_for(object)).checked.passed());
    require_renewal(export_current());
    require_hit(export_current());

    // Rebuilding equivalent demand into the same mutable Plan wrapper must
    // retire the old immutable identity. Geometry alone cannot authorize
    // reuse of a proof tied to an earlier construction state.
    auto *plan = const_cast<BAC::Plan *>(object.bridge_aware_cubic_plan(0));
    REQUIRE(plan != nullptr);
    const auto old_identity = plan->immutable_identity();
    REQUIRE(old_identity);
    *plan = plan->with_demands(plan->demands());
    REQUIRE(object.bridge_aware_cubic_plan(0) == plan);
    REQUIRE(plan->immutable_identity() != old_identity);
    REQUIRE(prove(target, phase_for(object)).checked.passed());
    require_renewal(export_current());
    require_hit(export_current());
}

TEST_CASE("Bridge aware cubic shared caller proofs track phases and representative mutations",
          "[BridgeAwareCubic][Integration][Export][BACCertificateShared]")
{
    CertificateWorkspace workspace;
    workspace.clear_profile_controls();
    Model model;
    auto *part = model.add_object();
    part->name = "Bridge aware cubic shared certificate geometry";
    part->config.set_key_value("extruder", new ConfigOptionInt(1));
    part->add_volume(TriangleMesh(its_make_cube(32., 28., 18.)));
    part->add_instance()->set_offset(Vec3d(80., 80., 0.));
    auto *copy = model.add_object(*part);
    copy->name = "Bridge aware cubic shared certificate caller";
    REQUIRE(copy->volumes.front()->mesh_ptr() == part->volumes.front()->mesh_ptr());
    REQUIRE(copy->instances.size() == 1);
    copy->instances.front()->set_offset(Vec3d(160., 80., 0.));
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(part);
    print.auto_assign_extruders(copy);
    print.apply(model, certificate_config());
    REQUIRE_NOTHROW(print.process());
    REQUIRE(print.objects().size() == 2);
    PrintObject *caller = nullptr;
    for (PrintObject *object : print.objects()) {
        if (object->get_shared_object() == nullptr) continue;
        REQUIRE(caller == nullptr);
        caller = object;
    }
    REQUIRE(caller != nullptr);
    PrintObject &geometry = *caller->get_shared_object();
    REQUIRE(geometry.get_shared_object() == nullptr);
    REQUIRE(geometry.layers().size() == 90);
    REQUIRE(caller->layers() == geometry.layers());
    const uint64_t caller_id = caller->model_object()->id().id;
    const uint64_t geometry_id = geometry.model_object()->id().id;
    REQUIRE(caller_id != geometry_id);
    const std::string filename = (workspace.directory / "shared-same-file.gcode").string();
    const auto export_current = [&] {
        const std::string profile = workspace.select_profile();
        REQUIRE_NOTHROW(print.export_gcode(filename, nullptr));
        REQUIRE(boost::filesystem::is_regular_file(filename));
        auto records = validate_records(profile);
        REQUIRE(records.size() == 2);
        REQUIRE(records.count(caller_id) == 1);
        REQUIRE(records.count(geometry_id) == 1);
        return records;
    };
    export_current();
    // Capture only after public export proved the actual shared caller. This
    // helper record does not install or fabricate an internal proof result.
    const auto old_phase = phase_for(*caller);
    const auto representative_phase = phase_for(geometry);
    const auto before = BAC::DepositionStateSnapshot::capture(geometry, *caller, {old_phase});
    REQUIRE(before);
    REQUIRE(before->matches(geometry, *caller, {old_phase}));
    REQUIRE_FALSE(BAC::DepositionStateSnapshot::capture(*caller, geometry, {old_phase}));

    caller->instances().front().shift.x() += scale_(1.);
    const auto changed_phase = phase_for(*caller);
    REQUIRE(changed_phase.shift != old_phase.shift);
    REQUIRE(phase_for(geometry).shift == representative_phase.shift);
    REQUIRE_FALSE(before->matches(geometry, *caller, {changed_phase}));
    const auto changed = export_current();
    // Caller-specific retained coverage is conservative: the representative
    // may also miss. Only the new caller phase is required to renew here.
    require_renewal(changed.at(caller_id));
    const auto renewed = BAC::DepositionStateSnapshot::capture(geometry, *caller, {changed_phase});
    REQUIRE(renewed);
    REQUIRE(renewed->matches(geometry, *caller, {changed_phase}));
    REQUIRE_FALSE(renewed->matches(geometry, *caller, {old_phase}));

    const PhysicalTarget target = direct_roof_target(geometry);
    REQUIRE(prove(target, representative_phase).checked.passed());
    REQUIRE(prove(target, changed_phase).checked.passed());
    const std::string valid_file = contents(filename);
    {
        RestoreLayer restore(*target.lower);
        for (auto &entry : restore.paths) entry.first->polyline.translate(Point::new_scale(100., 100.));
        for (const auto &phase : {representative_phase, changed_phase}) {
            const auto broken = prove(target, phase);
            REQUIRE(broken.checked.input_valid);
            REQUIRE(broken.checked.completed);
            REQUIRE_FALSE(broken.checked.passed());
            REQUIRE_FALSE(broken.checked.unsupported_runs.empty());
        }
        REQUIRE_FALSE(renewed->matches(geometry, *caller, {changed_phase}));
        workspace.select_profile();
        REQUIRE_THROWS_AS(print.export_gcode(filename, nullptr), SlicingError);
        CHECK(contents(filename) == valid_file);
    }
    REQUIRE(prove(target, changed_phase).checked.passed());
    export_current();
}
