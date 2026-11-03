#pragma once

#include "nlohmann/json.hpp"
#include <string>

namespace Slic3r::GUI {
// SDK completion replies may represent the job ID as a string or an integer.
// Missing or malformed IDs cannot establish a match with printer telemetry.
inline std::string print_job_id_from_reply(const std::string &reply)
{
    const auto parsed = nlohmann::json::parse(reply, nullptr, false);
    if (!parsed.is_object() || !parsed.contains("job_id")) return {};
    const auto &id = parsed["job_id"];
    if (id.is_string()) return id.get<std::string>();
    if (id.is_number_integer()) return id.dump();
    return {};
}
} // namespace Slic3r::GUI
