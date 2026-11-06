#include <catch_main.hpp>

#include "fg_test_serialization.hpp"
#include "fg_test_evaluator.hpp"
#include "fg_test_utils.hpp"
#include <libslic3r/FlushVolPredictor.hpp>

#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

namespace fs = std::filesystem;
using namespace Slic3r;
using namespace Slic3r::FGTest;

// ============ Helpers ============

static std::vector<std::string> collect_test_files(const std::string& dir) {
    std::vector<std::string> files;
    if (!fs::exists(dir)) return files;
    for (auto& entry : fs::recursive_directory_iterator(dir)) {
        if (entry.path().extension() == ".json" &&
            entry.path().string().find(".result.") == std::string::npos) {
            files.push_back(entry.path().string());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

static std::vector<std::string> get_golden_files() {
    static std::vector<std::string> files = collect_test_files(FG_TEST_GOLDEN_DIR);
    return files;
}

static bool is_constraint_feasible(const FilamentGroupContext& ctx,
                                    const std::vector<unsigned int>& used_filaments) {
    int total_capacity = 0;
    for (auto sz : ctx.machine_info.max_group_size)
        total_capacity += sz;
    if (total_capacity < (int)used_filaments.size())
        return false;

    // Check that every filament has at least one valid nozzle
    for (auto fil : used_filaments) {
        bool has_valid_nozzle = false;
        for (size_t nid = 0; nid < ctx.nozzle_info.nozzle_list.size(); ++nid) {
            auto& nozzle = ctx.nozzle_info.nozzle_list[nid];
            // Check unprintable_filaments
            if (nozzle.extruder_id >= 0 && nozzle.extruder_id < (int)ctx.model_info.unprintable_filaments.size()) {
                if (ctx.model_info.unprintable_filaments[nozzle.extruder_id].count(fil))
                    continue;
            }
            // Check unprintable_volumes
            if (ctx.model_info.unprintable_volumes.count(fil)) {
                if (ctx.model_info.unprintable_volumes.at(fil).count(nozzle.volume_type))
                    continue;
            }
            has_valid_nozzle = true;
            break;
        }
        if (!has_valid_nozzle)
            return false;
    }
    return true;
}

// ============ Property Check Specs ============

struct PropertySpec {
    std::string id;
    std::string config;
    int seed;
    int num_filaments;
    int num_layers;
    bool chaotic;
    bool with_constraints;
    FGMode mode;
    FGStrategy strategy;
    bool group_with_time;
};

static std::vector<PropertySpec> build_property_specs() {
    std::vector<PropertySpec> specs;

    // Config A: 20 cases
    for (int i = 0; i < 6; ++i) {
        int seed = 90000 + i;
        TestRng rng(seed);
        specs.push_back({"prop_a_basic_" + std::to_string(i), "A", seed,
                         rng.rand_int(2, 6), rng.rand_int(100, 400),
                         false, false, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    for (int i = 0; i < 4; ++i) {
        int seed = 90100 + i;
        TestRng rng(seed);
        specs.push_back({"prop_a_stress_" + std::to_string(i), "A", seed,
                         rng.rand_int(7, 10), rng.rand_int(500, 1000),
                         false, false, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    for (int i = 0; i < 4; ++i) {
        int seed = 90200 + i;
        TestRng rng(seed);
        specs.push_back({"prop_a_constraint_" + std::to_string(i), "A", seed,
                         rng.rand_int(3, 8), rng.rand_int(100, 400),
                         false, true, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    for (int i = 0; i < 3; ++i) {
        int seed = 90300 + i;
        TestRng rng(seed);
        specs.push_back({"prop_a_edge_" + std::to_string(i), "A", seed,
                         rng.rand_int(2, 3), rng.rand_int(10, 50),
                         true, false, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    specs.push_back({"prop_a_mode_match", "A", 90400,
                     5, 200, false, false, FGMode::MatchMode, FGStrategy::BestCost, false});
    specs.push_back({"prop_a_mode_bestfit", "A", 90401,
                     5, 200, false, false, FGMode::FlushMode, FGStrategy::BestFit, false});
    specs.push_back({"prop_a_mode_time", "A", 90402,
                     5, 200, false, false, FGMode::FlushMode, FGStrategy::BestCost, true});

    // Config B: 25 cases
    for (int i = 0; i < 6; ++i) {
        int seed = 91000 + i;
        TestRng rng(seed);
        specs.push_back({"prop_b_basic_" + std::to_string(i), "B", seed,
                         rng.rand_int(3, 8), rng.rand_int(100, 400),
                         false, false, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    for (int i = 0; i < 6; ++i) {
        int seed = 91100 + i;
        TestRng rng(seed);
        specs.push_back({"prop_b_stress_" + std::to_string(i), "B", seed,
                         rng.rand_int(9, 12), rng.rand_int(500, 1000),
                         false, false, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    for (int i = 0; i < 7; ++i) {
        int seed = 91200 + i;
        TestRng rng(seed);
        specs.push_back({"prop_b_constraint_" + std::to_string(i), "B", seed,
                         rng.rand_int(4, 10), rng.rand_int(100, 400),
                         false, true, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    for (int i = 0; i < 3; ++i) {
        int seed = 91300 + i;
        TestRng rng(seed);
        specs.push_back({"prop_b_edge_" + std::to_string(i), "B", seed,
                         rng.rand_int(2, 4), rng.rand_int(10, 50),
                         true, false, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    specs.push_back({"prop_b_mode_match", "B", 91400,
                     6, 200, false, false, FGMode::MatchMode, FGStrategy::BestCost, false});
    specs.push_back({"prop_b_mode_bestfit", "B", 91401,
                     6, 200, false, false, FGMode::FlushMode, FGStrategy::BestFit, false});
    specs.push_back({"prop_b_mode_time", "B", 91402,
                     6, 200, false, false, FGMode::FlushMode, FGStrategy::BestCost, true});

    // Config C: 15 cases
    for (int i = 0; i < 5; ++i) {
        int seed = 92000 + i;
        TestRng rng(seed);
        specs.push_back({"prop_c_basic_" + std::to_string(i), "C", seed,
                         rng.rand_int(3, 9), rng.rand_int(100, 400),
                         false, false, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    for (int i = 0; i < 3; ++i) {
        int seed = 92100 + i;
        TestRng rng(seed);
        specs.push_back({"prop_c_stress_" + std::to_string(i), "C", seed,
                         rng.rand_int(10, 15), rng.rand_int(500, 1000),
                         false, false, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    for (int i = 0; i < 3; ++i) {
        int seed = 92200 + i;
        TestRng rng(seed);
        specs.push_back({"prop_c_constraint_" + std::to_string(i), "C", seed,
                         rng.rand_int(4, 9), rng.rand_int(100, 400),
                         false, true, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    for (int i = 0; i < 2; ++i) {
        int seed = 92300 + i;
        TestRng rng(seed);
        specs.push_back({"prop_c_edge_" + std::to_string(i), "C", seed,
                         rng.rand_int(2, 4), rng.rand_int(10, 50),
                         true, false, FGMode::FlushMode, FGStrategy::BestCost, false});
    }
    specs.push_back({"prop_c_mode_match", "C", 92400,
                     6, 200, false, false, FGMode::MatchMode, FGStrategy::BestCost, false});
    specs.push_back({"prop_c_mode_bestfit", "C", 92401,
                     6, 200, false, false, FGMode::FlushMode, FGStrategy::BestFit, false});

    return specs;
}

static std::vector<PropertySpec>& get_property_specs() {
    static std::vector<PropertySpec> specs = build_property_specs();
    return specs;
}

// ============ Layer 1: Golden Regression ============

// The goldens (3eb211493) predate the intentional AMS policy changes
// (61f9ed42f, a56460272): a better loaded-spool match may spend up to five
// typical filament changes above the optimizer's best cost. Keep the original
// numeric optimizer limits, and check that final AMS selection satisfies its
// separate, bounded objective instead of raising any golden score or tolerance.
static double ams_selection_budget(const FilamentGroupContext& ctx, double best_score)
{
    const auto used = collect_sorted_used_filaments(ctx.model_info.layer_filaments);
    std::vector<double> changes;
    for (const auto& matrix : ctx.model_info.flush_matrix)
        for (size_t i = 0; i < used.size(); ++i)
            for (size_t j = i + 1; j < used.size(); ++j)
                changes.push_back(std::max(matrix[used[i]][used[j]], matrix[used[j]][used[i]]));
    std::sort(changes.begin(), changes.end());
    const double median_change = changes.empty() ? 0.0 : changes[changes.size() / 2];
    // 0.4536 converts purge volume into score; 1.2 is the one-change margin.
    return std::max({60.0, 5.0 * 1.2 * 0.4536 * median_change,
                     best_score * ctx.group_info.max_gap_threshold});
}

static int ams_match_cost(const FilamentGroupContext& ctx, const std::vector<int>& map)
{
    int total = 0;
    for (unsigned int filament : collect_sorted_used_filaments(ctx.model_info.layer_filaments)) {
        const auto& model = ctx.model_info.filament_info.at(filament);
        const int extruder = ctx.nozzle_info.nozzle_list.at(map.at(filament)).extruder_id;
        int best = 9999;
        if (extruder >= 0 && extruder < int(ctx.machine_info.machine_filament_info.size())) {
            // Flush-mode matching permits a spool to serve every model filament.
            // Thus its flow problem independently reduces to a nearest compatible
            // spool for each filament; no production matching solver is used here.
            for (const auto& spool : ctx.machine_info.machine_filament_info[extruder]) {
                if (spool.type != model.type || spool.is_support != model.is_support)
                    continue;
                const float distance = FlushPredict::calc_color_distance(
                    FlushPredict::RGBColor(model.color.r, model.color.g, model.color.b),
                    FlushPredict::RGBColor(spool.color.r, spool.color.g, spool.color.b));
                if (distance <= 20.0f)
                    best = std::min(best, int(distance));
            }
        }
        total += best;
    }
    return total;
}

TEST_CASE("FilamentGroup golden regression", "[filament_group][golden]") {
    auto files = get_golden_files();
    if (files.empty()) {
        WARN("No golden files found in " FG_TEST_GOLDEN_DIR);
        REQUIRE(!files.empty());
        return;
    }

    auto file_path = GENERATE_REF(from_range(files));

    DYNAMIC_SECTION("Golden: " << fs::path(file_path).stem().string()) {
        auto tc = load_test_case(file_path);
        REQUIRE(tc.base_result.has_value());

        std::vector<std::vector<int>> retained_groups;
        auto result = run_and_evaluate(tc.context, &retained_groups);
        auto eval = full_evaluate_map(tc.context, result.filament_map);

        auto& base = *tc.base_result;

        INFO("Case: " << tc.metadata.id);
        INFO("Base score: " << base.full_score);
        INFO("Actual score: " << eval.full_score);
        INFO("Flush cost: " << eval.flush_cost);
        INFO("Elapsed: " << result.elapsed_ms << " ms");

        int tolerance = std::max(50, (int)(base.full_score * 0.03));

        REQUIRE(result.constraints_ok);
        if (tc.context.group_info.mode == FGMode::FlushMode) {
            REQUIRE(!retained_groups.empty());
            // The min-heap is exported in ascending objective order, so the
            // first retained group is the optimizer's best feasible candidate.
            const auto best = full_evaluate_map(tc.context, retained_groups.front());
            INFO("Optimizer score: " << best.full_score);
            REQUIRE(best.constraints_ok);
            REQUIRE(best.full_score <= base.full_score + tolerance);
            REQUIRE(std::find(retained_groups.begin(), retained_groups.end(), result.filament_map)
                    != retained_groups.end());

            // Enumeration can add a 10-point master-extruder balance penalty;
            // clustering does not. Its maximum difference is accounted for
            // explicitly while keeping the five-change AMS budget unchanged.
            constexpr double max_balance_penalty = 10.0;
            const double budget = ams_selection_budget(tc.context, best.full_score + max_balance_penalty);
            REQUIRE(eval.full_score <= best.full_score + budget + max_balance_penalty + 1e-6);
            if (!tc.context.group_info.has_filament_switcher) {
                const int selected_match_cost = ams_match_cost(tc.context, result.filament_map);
                for (const auto& candidate : retained_groups)
                    REQUIRE(selected_match_cost <= ams_match_cost(tc.context, candidate));
            }
        } else {
            REQUIRE(eval.full_score <= base.full_score + tolerance);
        }
        REQUIRE(result.elapsed_ms < 20000.0);
    }
}

TEST_CASE("FilamentGroup AMS candidates have a bounded five-change budget",
          "[filament_group][ams]")
{
    using namespace FilamentGroupUtils;
    MemoryedGroupHeap groups;
    update_memoryed_groups(MemoryedGroup({0, 1}, 100.0, 1), 20.0, 0.01, groups);
    update_memoryed_groups(MemoryedGroup({0, 0}, 200.0, 1), 20.0, 0.01, groups);
    update_memoryed_groups(MemoryedGroup({1, 1}, 200.001, 1), 20.0, 0.01, groups);
    REQUIRE(groups.size() == 2);
    REQUIRE(groups.top().cost == 100.0);
    groups.pop();
    REQUIRE(groups.top().cost == 200.0);

    // A newly discovered optimum must evict formerly acceptable candidates.
    update_memoryed_groups(MemoryedGroup({1, 0}, 50.0, 1), 20.0, 0.01, groups);
    REQUIRE(groups.size() == 1);
    REQUIRE(groups.top().cost == 50.0);
}

TEST_CASE("FilamentGroup AMS selection prefers loaded compatible spools",
          "[filament_group][ams]")
{
    using namespace FilamentGroupUtils;
    const Color black(0, 0, 0), white(255, 255, 255);
    std::vector<FilamentGroupUtils::FilamentInfo> model = {{black, "PLA", false, ModelOnly},
                                                         {white, "PLA", false, ModelOnly}};
    std::vector<std::vector<MachineFilamentInfo>> spools(2);
    for (const auto& filament : model) {
        MachineFilamentInfo spool{};
        static_cast<FilamentGroupUtils::FilamentInfo&>(spool) = filament;
        spool.extruder_id = 0;
        spools[0].push_back(spool);
    }
    const std::vector<MultiNozzleUtils::NozzleInfo> nozzles = {
        {"0.4", NozzleVolumeType::nvtStandard, 0, 0},
        {"0.4", NozzleVolumeType::nvtStandard, 1, 1}};
    const std::vector<std::vector<int>> candidates = {{0, 1}, {0, 0}};
    REQUIRE(select_best_group_for_ams(candidates, nozzles, {0, 1}, model, spools) == candidates[1]);
    REQUIRE(select_best_group_for_ams(candidates, nozzles, {0, 1}, model, spools, true) == candidates[0]);
    REQUIRE(select_best_group_for_ams(candidates, nozzles, {0, 1}, model, {}) == candidates[0]);
}

// ============ Layer 2: Property Checks ============

TEST_CASE("FilamentGroup property checks", "[filament_group][property]") {
    auto& specs = get_property_specs();
    auto spec = GENERATE_REF(from_range(specs));

    DYNAMIC_SECTION("Property: " << spec.id) {
        auto tc = build_test_case(spec.id, spec.config, spec.seed,
                                   spec.num_filaments, spec.num_layers,
                                   spec.chaotic, spec.with_constraints,
                                   spec.mode, spec.strategy, spec.group_with_time);

        auto result = run_and_evaluate(tc.context);

        INFO("Case: " << spec.id);
        INFO("Config: " << spec.config);
        INFO("Flush cost: " << result.flush_cost);
        INFO("Elapsed: " << result.elapsed_ms << " ms");

        REQUIRE(result.elapsed_ms < 10000.0);
        REQUIRE(result.flush_cost >= 0);

        auto used_filaments = collect_sorted_used_filaments(tc.context.model_info.layer_filaments);
        if (is_constraint_feasible(tc.context, used_filaments)) {
            if (!result.constraints_ok) {
                for (auto& v : result.violations)
                    WARN("Violation: " << v);
            }
            REQUIRE(result.constraints_ok);
        } else {
            if (!result.constraints_ok) {
                WARN("Constraint violation (infeasible case, soft): " << spec.id);
            }
        }
    }
}

// ============ Golden Update Utility ============

TEST_CASE("FilamentGroup update golden", "[filament_group][update-golden][.]") {
    auto files = get_golden_files();
    REQUIRE(!files.empty());

    int updated = 0;
    for (auto& file_path : files) {
        auto tc = load_test_case(file_path);
        auto result = run_and_evaluate(tc.context);
        auto eval = full_evaluate_map(tc.context, result.filament_map);

        BaseResult base;
        base.full_score = eval.full_score;
        base.flush_cost = eval.flush_cost;
        base.constraints_ok = result.constraints_ok;

        tc.base_result = base;
        save_test_case(file_path, tc);
        updated++;

        std::cout << "Updated: " << fs::path(file_path).stem().string()
                  << " score=" << base.full_score
                  << " flush=" << base.flush_cost << std::endl;
    }

    std::cout << "\nUpdated " << updated << " golden files." << std::endl;
    REQUIRE(updated > 0);
}

TEST_CASE("FilamentGroup MatchMode shares capacity across loaded slots", "[filament_group][capacity]") {
    auto tc = build_test_case("match_capacity", "A", 93201, 3, 1, false, false,
                             FGMode::MatchMode, FGStrategy::BestCost, false);
    auto& ctx = tc.context;
    ctx.model_info.layer_filaments = {{0, 1, 2}};
    ctx.machine_info.max_group_size = {2, 1};
    ctx.machine_info.prefer_non_model_filament = {false, false};
    for (auto& filament : ctx.model_info.filament_info) {
        filament.type = "PLA";
        filament.is_support = false;
        filament.usage_type = FilamentUsageType::ModelOnly;
        filament.color = FilamentGroupUtils::Color(255, 0, 0);
    }
    for (size_t ext = 0; ext < ctx.machine_info.machine_filament_info.size(); ++ext)
        for (auto& filament : ctx.machine_info.machine_filament_info[ext])
            filament.color = ext == 0 ? FilamentGroupUtils::Color(255, 0, 0)
                                      : FilamentGroupUtils::Color(0, 0, 255);

    auto result = run_and_evaluate(ctx);
    CAPTURE(result.violations);
    REQUIRE(result.constraints_ok);
    REQUIRE(std::count(result.filament_map.begin(), result.filament_map.end(), 0) == 2);
    REQUIRE(std::count(result.filament_map.begin(), result.filament_map.end(), 1) == 1);
}

TEST_CASE("FilamentGroup MatchMode reassigns type matches to satisfy physical limits", "[filament_group][capacity]") {
    auto tc = build_test_case("match_reassignment", "A", 93202, 2, 1, false, false,
                             FGMode::MatchMode, FGStrategy::BestCost, false);
    auto& ctx = tc.context;
    ctx.model_info.layer_filaments = {{0, 1}};
    ctx.machine_info.max_group_size = {1, 1};
    ctx.machine_info.prefer_non_model_filament = {false, false};
    ctx.model_info.unprintable_filaments = {{1}, {}};
    for (size_t i = 0; i < ctx.model_info.filament_info.size(); ++i) {
        auto& filament = ctx.model_info.filament_info[i];
        filament.type = i == 0 ? "PLA" : "PETG";
        filament.is_support = false;
        filament.usage_type = FilamentUsageType::ModelOnly;
    }
    for (size_t ext = 0; ext < ctx.machine_info.machine_filament_info.size(); ++ext)
        for (auto& filament : ctx.machine_info.machine_filament_info[ext])
            filament.type = ext == 0 ? "PETG" : "PLA";

    auto result = run_and_evaluate(ctx);
    CAPTURE(result.violations);
    REQUIRE(result.constraints_ok);
    REQUIRE(result.filament_map == (std::vector<int>{0, 1}));
}

TEST_CASE("FilamentGroup FlushMode shares capacity across selector nozzles", "[filament_group][capacity]") {
    auto tc = build_test_case("selector_capacity", "B", 93203, 4, 1, false, false,
                             FGMode::FlushMode, FGStrategy::BestCost, false);
    auto& ctx = tc.context;
    ctx.model_info.layer_filaments = {{0, 1, 2, 3}};
    ctx.machine_info.max_group_size = {2, 2};
    ctx.machine_info.prefer_non_model_filament = {false, false};
    ctx.machine_info.machine_filament_info = {{}, {}};
    ctx.nozzle_info.nozzle_list = {{"0.4", NozzleVolumeType::nvtStandard, 0, 0},
                                  {"0.4", NozzleVolumeType::nvtStandard, 1, 1},
                                  {"0.4", NozzleVolumeType::nvtStandard, 1, 2},
                                  {"0.4", NozzleVolumeType::nvtStandard, 1, 3}};
    ctx.nozzle_info.extruder_nozzle_list = {{0, {0}}, {1, {1, 2, 3}}};
    for (auto& matrix : ctx.model_info.flush_matrix)
        for (size_t i = 0; i < matrix.size(); ++i)
            for (size_t j = 0; j < matrix[i].size(); ++j)
                matrix[i][j] = i == j ? 0.f : 1000.f;

    auto result = run_and_evaluate(ctx);
    CAPTURE(result.violations);
    REQUIRE(result.constraints_ok);
    REQUIRE(std::count(result.filament_map.begin(), result.filament_map.end(), 0) == 2);
}

TEST_CASE("FilamentGroup MatchMode can use capacity on an unloaded extruder", "[filament_group][capacity]") {
    auto tc = build_test_case("match_unloaded_capacity", "A", 93204, 2, 1, false, false,
                             FGMode::MatchMode, FGStrategy::BestCost, false);
    auto& ctx = tc.context;
    ctx.model_info.layer_filaments = {{0, 1}};
    ctx.machine_info.max_group_size = {1, 1};
    ctx.machine_info.prefer_non_model_filament = {false, false};
    ctx.machine_info.machine_filament_info[0].resize(1);
    ctx.machine_info.machine_filament_info[1].clear();
    for (auto& filament : ctx.model_info.filament_info) {
        filament.type = "PLA";
        filament.is_support = false;
        filament.usage_type = FilamentUsageType::ModelOnly;
    }

    auto result = run_and_evaluate(ctx);
    CAPTURE(result.violations);
    REQUIRE(result.constraints_ok);
    REQUIRE(std::count(result.filament_map.begin(), result.filament_map.end(), 0) == 1);
    REQUIRE(std::count(result.filament_map.begin(), result.filament_map.end(), 1) == 1);
    FilamentGroup direct_match(ctx);
    REQUIRE(direct_match.calc_filament_group_for_match() == result.filament_map);
}

TEST_CASE("FilamentGroup supports a nozzle only on the right extruder", "[filament_group][capacity]") {
    auto tc = build_test_case("right_only_nozzle", "A", 93205, 2, 1, false, false,
                             FGMode::FlushMode, FGStrategy::BestCost, false);
    auto& ctx = tc.context;
    ctx.model_info.layer_filaments = {{0, 1}};
    ctx.machine_info.max_group_size = {0, 2};
    ctx.machine_info.machine_filament_info = {{}, {}};
    ctx.nozzle_info.nozzle_list = {{"0.4", NozzleVolumeType::nvtStandard, 1, 0}};
    ctx.nozzle_info.extruder_nozzle_list = {{1, {0}}};

    auto result = run_and_evaluate(ctx);
    CAPTURE(result.violations);
    REQUIRE(result.constraints_ok);
    REQUIRE(result.filament_map == (std::vector<int>{0, 0}));
}
