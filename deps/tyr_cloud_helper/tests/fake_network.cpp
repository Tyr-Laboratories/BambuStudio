// Offline substitute for the proprietary SDK. No sockets or cloud operations.
#include "src/slic3r/Utils/bambu_networking.hpp"
#include <chrono>
#include <thread>

using namespace Slic3r;
extern "C" {
std::string bambu_network_get_version() { return "02.05.02.58"; }
void* bambu_network_create_agent(std::string) { return reinterpret_cast<void*>(1); }
int bambu_network_destroy_agent(void*) { return 0; }
bool bambu_network_is_user_login(void*) { return true; }
int bambu_network_change_user(void*, std::string) { return 0; }
std::string bambu_network_get_user_id(void*) { return "offline-user"; }
std::string bambu_network_build_login_info(void*) { return "PRIVATE-SESSION-SENTINEL"; }
int bambu_network_set_extra_http_header(void*, std::map<std::string, std::string>) { return 0; }
int bambu_network_start_print(void*, PrintParams params, OnUpdateStatusFn update,
                             WasCancelledFn cancel, OnWaitFn wait)
{
    if (params.connection_type != "cloud" || params.plate_index != 7 ||
        params.filename != "/artifacts/plate.gcode.3mf" ||
        params.ams_mapping != "[2]" || params.task_bed_type != "textured_plate")
        return -23;
    update(1, 25, "progress fixture");
    if (params.task_name == "cancel") {
        for (int i = 0; i < 500 && !cancel(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return cancel() ? -18 : -17;
    }
    const bool accepted = wait(5, "{\"job_id\":\"offline-job\"}");
    if (params.task_name == "two_waits" && accepted &&
        !wait(5, "{\"job_id\":\"offline-job\"}"))
        return -18;
    if (!accepted) return -18;
    update(6, 100, "finished fixture");
    return 0;
}
}
