#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string_view>

// Run in a fresh process for each sample. Input is a single-object project;
// loading and model slicing are deliberately outside the support measurement.
int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr << "usage: benchmark_organic_support project.3mf resources-dir [output.json]\n";
        return 2;
    }
    using namespace Slic3r;
    using Clock = std::chrono::steady_clock;
    const auto temporary = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("organic-benchmark-%%%%-%%%%");
    boost::filesystem::create_directories(temporary);
    struct Cleanup {
        boost::filesystem::path path;
        ~Cleanup() { boost::system::error_code error; boost::filesystem::remove_all(path, error); }
    } cleanup{temporary};
    set_data_dir(temporary.string());
    set_temporary_dir(temporary.string());
    set_resources_dir(argv[2]);
    set_logging_level(3);
    DynamicPrintConfig config;
    config.apply(FullPrintConfig::defaults());
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
    En3mfType type = En3mfType::From_Other;
    PlateDataPtrs plates;
    std::vector<Preset *> presets;
    Semver version;
    Model model = Model::read_from_archive(argv[1], &config, &substitutions, type,
        LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances,
        &plates, &presets, &version);
    for (auto *plate : plates) delete plate;
    if (model.objects.size() != 1) {
        std::cerr << "Expected exactly one object; got " << model.objects.size() << '\n';
        return 2;
    }
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    PrintObject &object = *print.get_object(0);
    const auto start = Clock::now();
    object.slice();
    const auto sliced = Clock::now();
    std::cout << "BENCHMARK_SUPPORT_START" << std::endl;
    const auto support_start = Clock::now();
    object.generate_support_preview();
    const auto supported = Clock::now();
    // The entire ordered path serialization includes coordinates, extrusion
    // roles, dimensions and cooling metadata, without relying on polygon area.
    uint64_t fingerprint = 1469598103934665603ULL;
    auto hash = [&](const auto &value) {
        for (unsigned char c : std::string_view(reinterpret_cast<const char *>(&value), sizeof(value))) {
            fingerprint ^= c;
            fingerprint *= 1099511628211ULL;
        }
    };
    size_t paths = 0, interface_paths = 0;
    auto visit = [&](auto &&self, const ExtrusionEntity &entity) -> void {
        hash(entity.get_customize_flag()); hash(entity.get_cooling_node());
        if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
            for (const auto *child : collection->entities) self(self, *child);
        } else if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) {
            ++paths;
            interface_paths += path->role() == erSupportMaterialInterface;
            hash(path->role()); hash(path->width); hash(path->height); hash(path->mm3_per_mm);
            hash(path->overhang_degree); hash(path->curve_degree);
            for (const Point &p : path->polyline.points) { hash(p.x()); hash(p.y()); }
        } else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
            for (const auto &path : multi->paths) self(self, path);
        } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
            for (const auto &path : loop->paths) self(self, path);
        } else throw std::runtime_error("Unknown extrusion entity in fingerprint");
    };
    for (const auto *layer : object.support_layers()) {
        hash(layer->print_z); hash(layer->height);
        visit(visit, layer->support_fills);
    }
    if (paths == 0) throw std::runtime_error("Benchmark did not generate support paths");
    std::ostringstream result;
    result << std::setprecision(12) << "{\"slice_seconds\":"
           << std::chrono::duration<double>(sliced-start).count()
           << ",\"support_seconds\":" << std::chrono::duration<double>(supported-support_start).count()
           << ",\"support_layers\":" << object.support_layers().size()
           << ",\"paths\":" << paths << ",\"interface_paths\":" << interface_paths
           << ",\"path_fingerprint\":\"" << fingerprint << "\"}\n";
    std::cout << "BENCHMARK_RESULT " << result.str();
    if (argc > 3) std::ofstream(argv[3]) << result.str();
}
