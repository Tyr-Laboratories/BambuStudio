#include <catch2/catch.hpp>

#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Fill/BridgeAwareCubic.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <typeinfo>
#include <vector>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;

namespace {
using Json = nlohmann::json;

// Profiling is opt-in, and these process-wide controls must not leak into
// another Catch case, including when a REQUIRE unwinds this scope.
struct TrialEnvironment {
    std::map<std::string, std::optional<std::string>> previous;
    boost::filesystem::path directory = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("bridge-aware-trial-parity-%%%%-%%%%");

    static int set(const char *name, const char *value)
    {
#ifdef _WIN32
        return _putenv_s(name, value == nullptr ? "" : value);
#else
        return value == nullptr ? unsetenv(name) : setenv(name, value, 1);
#endif
    }

    TrialEnvironment()
    {
        for (const char *name : {"BAC_PROFILE_FILE", "BAC_PROFILE_STAGE", "BAC_PROFILE_FULL_TRIAL_CHECKS",
                                 "BAC_PROFILE_AUDIT_EMISSION_COLLECTION", "BAC_PROFILE_AUDIT_CONTACT_EPOCHS",
                                 "BAC_PROFILE_AUDIT_COMPLETE_SPANS", "BAC_PROFILE_AUDIT_SPARSE_REJECTIONS"}) {
            const char *value = std::getenv(name);
            previous.emplace(name, value == nullptr ? std::nullopt : std::optional<std::string>(value));
        }
        boost::filesystem::create_directories(directory);
    }

    ~TrialEnvironment()
    {
        for (const auto &entry : previous)
            set(entry.first.c_str(), entry.second ? entry.second->c_str() : nullptr);
        boost::system::error_code error;
        boost::filesystem::remove_all(directory, error);
    }

    std::string select(bool exhaustive, bool sparse_audit) const
    {
        const std::string file = (directory / (exhaustive ? "exhaustive.jsonl" : sparse_audit ? "early.jsonl" : "sparse-production.jsonl")).string();
        REQUIRE_FALSE(boost::filesystem::exists(file));
        REQUIRE(set("BAC_PROFILE_FILE", file.c_str()) == 0);
        REQUIRE(set("BAC_PROFILE_STAGE", "process") == 0);
        REQUIRE(set("BAC_PROFILE_FULL_TRIAL_CHECKS", exhaustive ? "1" : nullptr) == 0);
        REQUIRE(set("BAC_PROFILE_AUDIT_EMISSION_COLLECTION", exhaustive ? nullptr : "1") == 0);
        REQUIRE(set("BAC_PROFILE_AUDIT_CONTACT_EPOCHS", exhaustive ? nullptr : "1") == 0);
        REQUIRE(set("BAC_PROFILE_AUDIT_COMPLETE_SPANS", exhaustive ? nullptr : "1") == 0);
        REQUIRE(set("BAC_PROFILE_AUDIT_SPARSE_REJECTIONS", !exhaustive && sparse_audit ? "1" : nullptr) == 0);
        return file;
    }
};

DynamicPrintConfig trial_config()
{
    // Exact small BACQualification geometry/config; both modes receive this
    // same immutable input. No time threshold is asserted by this case.
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

Json points(const Points &input)
{
    Json result = Json::array();
    for (const Point &point : input) result.push_back({point.x(), point.y()});
    return result;
}

Json polygons(const ExPolygons &input)
{
    Json result = Json::array();
    for (const ExPolygon &polygon : input) {
        Json holes = Json::array();
        for (const Polygon &hole : polygon.holes) holes.push_back(points(hole.points));
        result.push_back({{"contour", points(polygon.contour.points)}, {"holes", holes}});
    }
    return result;
}

Json owners(const std::vector<BAC::CellId> &input)
{
    Json result = Json::array();
    for (const BAC::CellId &owner : input) result.push_back({owner.depth, owner.address});
    return result;
}

Json entity_snapshot(const ExtrusionEntity &entity)
{
    Json result = {
        {"type", typeid(entity).name()}, {"role", int(entity.role())},
        {"can_reverse", entity.can_reverse()}, {"can_sort", entity.can_sort()},
        {"customize", int(entity.get_customize_flag())}, {"cooling_node", entity.get_cooling_node()}
    };
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        result["no_sort"] = collection->no_sort;
        result["loop_node_range"] = {collection->loop_node_range.first, collection->loop_node_range.second};
        result["children"] = Json::array();
        for (const ExtrusionEntity *child : collection->entities)
            result["children"].push_back(entity_snapshot(*child));
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
        REQUIRE(dynamic_cast<const ExtrusionLoopSloped *>(loop) == nullptr);
        result["loop_role"] = int(loop->loop_role());
        result["paths"] = Json::array();
        for (const ExtrusionPath &path : loop->paths) result["paths"].push_back(entity_snapshot(path));
    } else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
        result["paths"] = Json::array();
        for (const ExtrusionPath &path : multi->paths) result["paths"].push_back(entity_snapshot(path));
    } else if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) {
        REQUIRE(dynamic_cast<const ExtrusionPathSloped *>(path) == nullptr);
        REQUIRE(path->polyline.fitting_result.empty());
        result["points"] = points(path->polyline.points);
        result["width"] = path->width;
        result["height"] = path->height;
        result["mm3_per_mm"] = path->mm3_per_mm;
        result["no_extrusion"] = path->is_force_no_extrusion();
        result["overhang_degree"] = path->overhang_degree;
        result["curve_degree"] = path->curve_degree;
        result["smooth_speed"] = path->smooth_speed;
    } else FAIL("The complete final extrusion tree must have a known snapshot representation.");
    return result;
}

Json plan_snapshot(const BAC::Plan &plan, const PrintObject &object)
{
    Json result = {{"demands", Json::array()}, {"refinements", Json::array()},
                   {"layers", Json::array()}, {"active_demands", plan.active_demands()}};
    const auto &stats = plan.statistics();
    result["statistics"] = {stats.root_cells, stats.nodes, stats.splits, stats.leaves, stats.max_depth, stats.solid_cells};
    for (const auto &demand : plan.demands())
        result["demands"].push_back({{"z", demand.z}, {"gap", demand.gap}, {"dependency", demand.dependency},
            {"owners", owners(demand.owners)}, {"footprint", polygons(demand.footprint)}});
    for (const auto &refinement : plan.refinements())
        result["refinements"].push_back({{"id", owners({refinement.id})}, {"solid", refinement.solid},
            {"bottom_z", refinement.bottom_z}, {"top_z", refinement.top_z}, {"demands", refinement.demand_indices}});
    for (const Layer *layer : object.layers()) {
        Json closures = Json::array(), attachments = Json::array();
        for (const auto &closure : plan.closures(layer->print_z)) {
            Json paths = Json::array();
            for (const Polyline &path : closure.paths) paths.push_back(points(path.points));
            closures.push_back({{"z", closure.z}, {"emitter", closure.emitting_region}, {"paths", paths},
                {"owners", owners(closure.owners)}, {"reservation", polygons(closure.reservation)}});
        }
        for (const auto &attachment : plan.attachments(layer->print_z))
            attachments.push_back({{"z", attachment.z}, {"emitter", attachment.emitting_region},
                {"original", points(attachment.original.points)}, {"replacement", points(attachment.replacement.points)},
                {"width", attachment.width}, {"height", attachment.height}, {"owners", owners(attachment.owners)},
                {"solid_cut", attachment.solid_cut}, {"selection", polygons(attachment.demand_selection)}});
        result["layers"].push_back({{"z", layer->print_z}, {"closures", closures}, {"attachments", attachments},
            {"refinement_area", polygons(plan.refinement_area(layer->print_z))},
            {"solid_area", polygons(plan.solid_area(layer->print_z))}});
    }
    return result;
}

struct TrialResult {
    std::vector<Json> layers, plans;
    Json counters;
};

TrialResult run_trial_mode(Model &model, const DynamicPrintConfig &config, const TrialEnvironment &environment, bool exhaustive, bool sparse_audit = true)
{
    const std::string profile = environment.select(exhaustive, sparse_audit);
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.get_object(size_t(0));
    REQUIRE(object.layers().size() == 90);
    TrialResult result;
    for (const Layer *layer : object.layers()) {
        Json regions = Json::array();
        for (const LayerRegion *region : layer->regions())
            regions.push_back({{"perimeters", entity_snapshot(region->perimeters)},
                {"fills", entity_snapshot(region->fills)}, {"thin_fills", entity_snapshot(region->thin_fills)}});
        result.layers.push_back({{"z", layer->print_z}, {"height", layer->height}, {"regions", regions}});
    }
    for (size_t region = 0; region < object.num_printing_regions(); ++region) {
        const BAC::Plan *plan = object.bridge_aware_cubic_plan(region);
        REQUIRE(plan != nullptr);
        result.plans.push_back(plan_snapshot(*plan, object));
    }
    std::ifstream input(profile);
    REQUIRE(input.is_open());
    size_t prepare_totals = 0;
    std::string line;
    while (std::getline(input, line)) {
        const Json record = Json::parse(line);
        if (record.at("record") == "total" && record.at("session") == "prepare") {
            REQUIRE_FALSE(record.at("incomplete").get<bool>());
            REQUIRE_FALSE(record.at("identity_counts_truncated").get<bool>());
            REQUIRE(record.at("stage") == "process");
            result.counters = record;
            ++prepare_totals;
        }
    }
    REQUIRE(prepare_totals == 1);
    REQUIRE(result.counters.at("full_trial_checks").get<bool>() == exhaustive);
    REQUIRE(result.counters.at("rejected").get<size_t>() > 0);
    REQUIRE(result.counters.at("accepted").get<size_t>() > 0);
    size_t final_proofs = 0;
    for (const Json &metric : result.counters.at("metrics"))
        if (metric.at("name") == "final_proof") final_proofs += metric.at("calls").get<size_t>();
    REQUIRE(final_proofs > 0);
    if (exhaustive) {
        REQUIRE(result.counters.at("early_trial_rejections").get<size_t>() == 0);
        for (const char *name : {"sparse_probe_attempts", "sparse_probe_rejections",
                                "sparse_probe_saved_generation_layers", "sparse_probe_saved_rollback_layers",
                                "sparse_probe_would_skip_generation_layers", "sparse_probe_would_skip_rollback_layers",
                                "sparse_probe_audits", "sparse_probe_audit_mismatches"}) {
            CAPTURE(name);
            REQUIRE(result.counters.value(name, size_t(0)) == 0);
        }
        for (const char *counter : {"model_cache_hits", "model_cache_fallbacks", "model_cache_vertices", "model_cache_items"}) {
            CAPTURE(counter);
            REQUIRE(result.counters.at(counter).get<size_t>() == 0);
        }
        for (const char *counter : {"model_input_hits", "model_input_audits", "model_input_captures", "model_input_bytes"}) {
            CAPTURE(counter);
            REQUIRE(result.counters.at(counter).get<size_t>() == 0);
        }
        for (const char *counter : {"model_replay_probes", "model_replay_rejections", "model_replay_passes",
                                   "model_replay_unavailable", "model_replay_stores", "model_replay_entries"}) {
            CAPTURE(counter);
            REQUIRE(result.counters.at(counter).get<size_t>() == 0);
        }
        for (const char *counter : {"construction_containment_indices", "construction_containment_queries",
                                   "construction_containment_proofs"}) {
            CAPTURE(counter);
            REQUIRE(result.counters.at(counter).get<size_t>() == 0);
        }
        for (const char *counter : {"span_proof_hits", "span_proof_misses", "span_proof_publications",
                                   "span_proof_evictions", "span_proof_entries", "span_proof_units"}) {
            CAPTURE(counter);
            REQUIRE(result.counters.at(counter).get<size_t>() == 0);
        }
        for (const char *counter : {"contact_epoch_captures", "contact_epoch_transitions", "contact_epoch_peak_bytes",
                                   "span_epoch_hits", "span_epoch_advances", "span_epoch_audits",
                                   "complete_span_queries", "complete_span_proofs", "complete_span_audits"}) {
            CAPTURE(counter);
            REQUIRE(result.counters.at(counter).get<size_t>() == 0);
        }
    } else {
        REQUIRE(result.counters.at("early_trial_rejections").get<size_t>() > 0);
        REQUIRE(result.counters.at("model_cache_hits").get<size_t>() > 0);
        // These are aggregate retained-cache admission limits, across every
        // actual instance phase, rather than limits on verification coverage.
        REQUIRE(result.counters.at("model_cache_vertices").get<size_t>() > 0);
        REQUIRE(result.counters.at("model_cache_vertices").get<size_t>() <= 200000);
        REQUIRE(result.counters.at("model_cache_items").get<size_t>() > 0);
        REQUIRE(result.counters.at("model_cache_items").get<size_t>() <= 20000);
        REQUIRE(result.counters.at("model_input_captures").get<size_t>() == 1);
        REQUIRE(result.counters.at("model_input_hits").get<size_t>() > 0);
        REQUIRE(result.counters.at("model_input_audits") == result.counters.at("model_input_hits"));
        REQUIRE(result.counters.at("model_input_bytes").get<size_t>() > 0);
        REQUIRE(result.counters.at("model_input_combined_bytes").get<size_t>() <= 512 * 1024);
        REQUIRE(result.counters.at("model_input_capture_peak_bytes").get<size_t>() <= 512 * 1024);
        REQUIRE(result.counters.at("model_input_combined_vertices").get<size_t>() <= 200000);
        REQUIRE(result.counters.at("model_input_combined_items").get<size_t>() <= 20000);
        // A remembered failure only proposes a new physical check. Exercise
        // both that check and a newly verified rejection on this real fixture.
        if (sparse_audit) {
            REQUIRE(result.counters.at("model_replay_probes").get<size_t>() > 0);
            REQUIRE(result.counters.at("model_replay_rejections").get<size_t>() > 0);
        }
        REQUIRE(result.counters.at("model_replay_stores").get<size_t>() > 0);
        REQUIRE(result.counters.at("model_replay_entries").get<size_t>() <= 4096);
        // Repeated candidates must actually skip multi-layer generation and
        // rollback. In this diagnostic mode every proposed rejection must
        // independently fail the original full candidate route from the base.
        // The full route compares the candidate's own complete ordered model
        // Targets to the frozen model, and then uses the actual lower emission.
        const size_t sparse_rejected = result.counters.value("sparse_probe_rejections", size_t(0));
        REQUIRE(sparse_rejected > 0);
        REQUIRE(result.counters.at("sparse_probe_attempts").get<size_t>() >= sparse_rejected);
        REQUIRE(sparse_rejected <= result.counters.at("repeated_cell_trials").get<size_t>());
        REQUIRE(result.counters.at("sparse_probe_would_skip_generation_layers").get<size_t>() > 0);
        REQUIRE(result.counters.at("sparse_probe_would_skip_rollback_layers").get<size_t>() > 0);
        if (sparse_audit) {
            REQUIRE(result.counters.at("sparse_probe_saved_generation_layers").get<size_t>() == 0);
            REQUIRE(result.counters.at("sparse_probe_saved_rollback_layers").get<size_t>() == 0);
            REQUIRE(result.counters.at("sparse_probe_audits").get<size_t>() == sparse_rejected);
        } else {
            REQUIRE(result.counters.at("sparse_probe_saved_generation_layers").get<size_t>() ==
                    result.counters.at("sparse_probe_would_skip_generation_layers").get<size_t>());
            REQUIRE(result.counters.at("sparse_probe_saved_rollback_layers").get<size_t>() ==
                    result.counters.at("sparse_probe_would_skip_rollback_layers").get<size_t>());
            REQUIRE(result.counters.at("sparse_probe_audits").get<size_t>() == 0);
        }
        REQUIRE(result.counters.at("sparse_probe_audit_mismatches").get<size_t>() == 0);
        for (const char *counter : {"construction_containment_indices", "construction_containment_queries",
                                   "construction_containment_proofs"}) {
            CAPTURE(counter);
            REQUIRE(result.counters.at(counter).get<size_t>() > 0);
        }
        REQUIRE(result.counters.at("span_proof_hits").get<size_t>() > 0);
        REQUIRE(result.counters.at("span_proof_misses").get<size_t>() > 0);
        REQUIRE(result.counters.at("span_proof_publications").get<size_t>() > 0);
        REQUIRE(result.counters.at("span_proof_entries").get<size_t>() > 0);
        REQUIRE(result.counters.at("span_proof_entries").get<size_t>() <= 20000);
        REQUIRE(result.counters.at("span_proof_units").get<size_t>() > 0);
        REQUIRE(result.counters.at("span_proof_units").get<size_t>() <= 200000);
        // Exercise deferred classification and exact rollback reuse on real
        // regenerated fills. Complete output/Plan parity is checked below;
        // these counters ensure the optimized lifecycle was actually used.
        REQUIRE(result.counters.at("emission_projection_builds").get<size_t>() > 0);
        REQUIRE(result.counters.at("emission_projection_reuses").get<size_t>() > 0);
        REQUIRE(result.counters.at("emission_rollback_restored").get<size_t>() > 0);
        REQUIRE(result.counters.at("emission_rollback_mismatches").get<size_t>() == 0);
        REQUIRE(result.counters.at("emission_rejected_without_projection").get<size_t>() > 0);
        REQUIRE(result.counters.at("emission_collection_audits").get<size_t>() > 0);
        // The retained material identity must be exercised on real trial
        // generations, with each fast hit compared against fresh discovery.
        for (const char *counter : {"contact_epoch_captures", "contact_epoch_transitions",
                                   "span_epoch_hits", "span_epoch_advances", "span_epoch_audits"}) {
            CAPTURE(counter);
            REQUIRE(result.counters.at(counter).get<size_t>() > 0);
        }
        REQUIRE(result.counters.at("span_epoch_audit_mismatches").get<size_t>() == 0);
        for (const char *counter : {"complete_span_queries", "complete_span_proofs", "complete_span_audits"}) {
            CAPTURE(counter);
            REQUIRE(result.counters.at(counter).get<size_t>() > 0);
        }
        REQUIRE(result.counters.at("complete_span_audit_mismatches").get<size_t>() == 0);
        REQUIRE(result.counters.at("contact_epoch_peak_bytes").get<size_t>() > 0);
        REQUIRE(result.counters.at("contact_epoch_peak_bytes").get<size_t>() <= 2 * 1024 * 1024);
    }
    return result;
}
} // namespace

TEST_CASE("Bridge aware cubic early trial rejection preserves exhaustive final output",
          "[.][BridgeAwareCubic][BACTrialRejectionParity]")
{
    TrialEnvironment environment;
    Model model;
    auto *object = model.add_object();
    object->name = "BAC small qualification trial parity";
    object->add_volume(TriangleMesh(its_make_cube(32., 28., 18.)));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    const auto config = trial_config();
    const TrialResult exhaustive = run_trial_mode(model, config, environment, true);
    const TrialResult audited = run_trial_mode(model, config, environment, false, true);
    const TrialResult production = run_trial_mode(model, config, environment, false, false);
    for (const auto *mode : {&audited, &production}) {
        const auto &early = *mode;
        for (const char *counter : {"trials", "accepted", "rejected", "repeated_cell_trials", "closure_trials"}) {
            CAPTURE(counter);
            CHECK(exhaustive.counters.at(counter) == early.counters.at(counter));
        }
        REQUIRE(exhaustive.layers.size() == early.layers.size());
        for (size_t layer = 0; layer < exhaustive.layers.size(); ++layer) {
            CAPTURE(layer);
            CHECK(bool(exhaustive.layers[layer] == early.layers[layer]));
        }
        REQUIRE(exhaustive.plans.size() == early.plans.size());
        for (size_t region = 0; region < exhaustive.plans.size(); ++region) {
            CAPTURE(region);
            CHECK(bool(exhaustive.plans[region] == early.plans[region]));
        }
    }
}
