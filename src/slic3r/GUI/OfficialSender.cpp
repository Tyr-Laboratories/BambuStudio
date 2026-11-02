#include "OfficialSender.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>

namespace Slic3r::GUI::OfficialSender {
namespace {
constexpr const char *ready_file = ".tyr-sender-ready";

bool within(const fs::path &path, const fs::path &root)
{
    const auto relative = fs::weakly_canonical(path).lexically_relative(fs::weakly_canonical(root));
    return !relative.empty() && *relative.begin() != "..";
}

void private_directory(const fs::path &path)
{
    fs::create_directories(path);
    if (fs::is_symlink(path) || !fs::is_directory(path))
        throw std::runtime_error("The sender settings directory is not a regular directory.");
    fs::permissions(path, fs::perms::owner_all);
}

fs::path unique_directory(const fs::path &parent, const std::string &prefix)
{
    std::random_device random;
    for (unsigned attempt = 0; attempt < 20; ++attempt) {
        std::ostringstream name;
        name << prefix << std::hex << random() << random();
        auto candidate = parent / name.str();
        if (fs::create_directory(candidate)) {
            fs::permissions(candidate, fs::perms::owner_all);
            return candidate;
        }
    }
    throw std::runtime_error("Could not create a private sender job directory.");
}

// Preserve framework links without following them into unrelated user files.
void copy_state(const fs::path &source, const fs::path &target, const fs::path &root)
{
    if (fs::is_symlink(source)) {
        const auto link = fs::read_symlink(source);
        if (source == root || link.is_absolute() || !within(source.parent_path() / link, root))
            throw std::runtime_error("Sender setup found a link outside the selected settings folder.");
        fs::copy_symlink(source, target);
    } else if (fs::is_directory(source)) {
        private_directory(target);
        for (const auto &entry : fs::directory_iterator(source))
            copy_state(entry.path(), target / entry.path().filename(), root);
    } else if (fs::is_regular_file(source)) {
        fs::copy_file(source, target);
        // Network executables retain their executable bit; account files stay private.
        const auto executable = fs::status(source).permissions() & fs::perms::owner_exec;
        fs::permissions(target, fs::perms::owner_read | fs::perms::owner_write | executable);
    } else {
        throw std::runtime_error("Sender setup found an unsupported settings file.");
    }
}

void write_private(const fs::path &path, const std::string &contents)
{
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream || !(stream << contents) || !(stream.flush()))
        throw std::runtime_error("Could not write sender settings.");
    fs::permissions(path, fs::perms::owner_read | fs::perms::owner_write);
}
} // namespace

fs::path data_directory(const fs::path &source_data)
{
    const auto source = fs::absolute(source_data).lexically_normal();
    return source.parent_path() / (source.filename().string() + "-Tyr-Sender");
}

void prepare_data(const fs::path &source_data, const fs::path &sender_data)
{
    if (!fs::is_directory(source_data) || within(sender_data, source_data) || within(source_data, sender_data))
        throw std::runtime_error("The sender requires a separate settings directory.");
    if (fs::is_symlink(sender_data))
        throw std::runtime_error("The sender settings directory must not be a symbolic link.");
    if (fs::is_regular_file(sender_data / ready_file) && fs::is_regular_file(sender_data / "BambuStudio.conf"))
        return; // Never overwrite an existing sender's refreshed session or preferences.
    if (fs::exists(sender_data))
        throw std::runtime_error("The sender settings directory already exists but setup is incomplete.");

    nlohmann::json config;
    try {
        if (fs::is_symlink(source_data / "BambuStudio.conf"))
            throw std::runtime_error("linked config");
        std::ifstream stream(source_data / "BambuStudio.conf");
        stream >> config;
        if (!config.is_object() || !config.at("app").is_object())
            throw std::runtime_error("invalid config");
    } catch (...) {
        // Parser messages can contain snippets of credentials. Do not expose them.
        throw std::runtime_error("Could not read the current application settings for sender setup.");
    }
    auto &app = config["app"];
    app["single_instance"] = true;
    app["sync_user_preset"] = false;
    app["sync_system_preset"] = false;
    app["enable_beta_version_update"] = false;
    app["update_network_plugin"] = false;
    app["ignore_module_cert"] = false;
    app["last_backup_path"] = "";
    app["backup_switch"] = false;
    app["show_home_page"] = false;
    config.erase("recent");
    config.erase("recent_projects");

    const auto temporary = unique_directory(sender_data.parent_path(), ".tyr-sender-setup-");
    try {
        for (const char *name : {"BambuNetworkEngine.conf", "user", "system", "printers", "plugins", "certs"}) {
            const auto source = source_data / name;
            if (fs::exists(source) || fs::is_symlink(source))
                copy_state(source, temporary / name, source);
        }
        write_private(temporary / "BambuStudio.conf", config.dump(2) + "\n");
        write_private(temporary / ready_file, "1\n");
        // A complete directory becomes visible at once. Never repair/replace another instance's data.
        if (fs::exists(sender_data))
            throw std::runtime_error("Another sender setup is already in progress.");
        fs::rename(temporary, sender_data);
    } catch (...) {
        std::error_code ignored;
        fs::remove_all(temporary, ignored);
        throw;
    }
}

fs::path stage_job(const fs::path &source_job, const fs::path &sender_data, const std::string &name)
{
    if (!fs::is_regular_file(source_job) || fs::is_symlink(source_job) || fs::file_size(source_job) == 0)
        throw std::runtime_error("The sliced file is missing or empty. Slice the plate before sending.");
    if (!fs::is_regular_file(sender_data / ready_file))
        throw std::runtime_error("The official sender has not been set up.");
    const auto jobs = sender_data / "tyr_handoff_jobs";
    private_directory(jobs);
    const auto directory = unique_directory(jobs, "job-");
    std::string filename = fs::path(name).filename().string();
    for (char &c : filename)
        if (static_cast<unsigned char>(c) < 32 || c == ':' || c == '\\') c = '_';
    if (filename.empty() || filename == "." || filename == "..") filename = "Plate";
    const auto destination = directory / (filename + ".gcode.3mf");
    try {
        fs::copy_file(source_job, destination);
        fs::permissions(destination, fs::perms::owner_read | fs::perms::owner_write);
    } catch (...) {
        std::error_code ignored;
        fs::remove_all(directory, ignored);
        throw;
    }
    // Retain the copy independently of source project edits and fork shutdown.
    return destination;
}

std::vector<std::string> launch_arguments(const fs::path &sender_data, const fs::path &job)
{
    // Existing-instance IPC treats standalone paths as files to open: keep datadir in one token.
    return {"--datadir=" + fs::absolute(sender_data).string(), fs::absolute(job).string()};
}
}
