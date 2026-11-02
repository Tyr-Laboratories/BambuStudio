#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace Slic3r::GUI::OfficialSender {

namespace fs = std::filesystem;

// Local state only. No networking, application launches, or printer commands.
fs::path data_directory(const fs::path &source_data);
void prepare_data(const fs::path &source_data, const fs::path &sender_data);
fs::path stage_job(const fs::path &source_job, const fs::path &sender_data, const std::string &name);
std::vector<std::string> launch_arguments(const fs::path &sender_data, const fs::path &job);

#ifdef __APPLE__
bool is_configured(const fs::path &resources);
fs::path application_path(const fs::path &resources);
void verify_application(const fs::path &app);
void launch(const fs::path &app, const fs::path &sender_data, const fs::path &job);
#endif

} // namespace Slic3r::GUI::OfficialSender
