#include "slic3r/Utils/bambu_networking.hpp"
#include <utility>

// Only the vendor ABI is replaced. The test runs the production NetworkAgent.
// Subscription transitions return a sentinel instead of actually deadlocking,
// so an unsafe call fails the test without hanging its runner.
namespace {
struct Agent {
    BBL::OnMessageFn message;
    std::vector<std::string> printers;
    unsigned starts = 0;
    unsigned stops = 0;
};
}
extern "C" {
void* bambu_network_create_agent(std::string) { return new Agent; }
int bambu_network_destroy_agent(void* p) { delete static_cast<Agent*>(p); return 0; }
int bambu_network_set_on_message_fn(void* p, BBL::OnMessageFn fn)
{
    static_cast<Agent*>(p)->message = std::move(fn);
    return 0;
}
int bambu_network_start_subscribe(void* p, std::string)
{
    ++static_cast<Agent*>(p)->starts;
    return -71;
}
int bambu_network_stop_subscribe(void* p, std::string)
{
    ++static_cast<Agent*>(p)->stops;
    return -72;
}
int bambu_network_add_subscribe(void* p, std::vector<std::string> printers)
{
    static_cast<Agent*>(p)->printers = std::move(printers);
    return 0;
}
int bambu_network_del_subscribe(void* p, std::vector<std::string>)
{
    static_cast<Agent*>(p)->printers.clear();
    return 0;
}
void test_emit_status(void* p)
{
    auto& agent = *static_cast<Agent*>(p);
    if (agent.message)
        for (const auto& printer : agent.printers) agent.message(printer, "status");
}
unsigned test_start_count(void* p) { return static_cast<Agent*>(p)->starts; }
unsigned test_stop_count(void* p) { return static_cast<Agent*>(p)->stops; }
}
