#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include "slic3r/Utils/NetworkAgent.hpp"
#include "libslic3r/Utils.hpp"
#include <filesystem>
#include <memory>
#include <unistd.h>

namespace {
struct Fixture {
    std::filesystem::path root;
    std::string previous_data_dir = Slic3r::data_dir();
    std::unique_ptr<Slic3r::NetworkAgent> agent;

    Fixture()
    {
        char path[] = "/tmp/tyr-network-subscription-XXXXXX";
        const char* made = mkdtemp(path);
        if (!made) throw std::runtime_error("Cannot create isolated network test directory");
        root = made;
        std::filesystem::create_directory(root / "plugins");
        std::filesystem::copy_file(FAKE_IDLE_NETWORK_PATH, root / "plugins/libbambu_networking.dylib");
        Slic3r::set_data_dir(root.string());
        if (Slic3r::NetworkAgent::initialize_network_module(false, false) != 0)
            throw std::runtime_error("Cannot load test plugin");
        agent = std::make_unique<Slic3r::NetworkAgent>(root.string());
    }
    ~Fixture()
    {
        agent.reset();
        Slic3r::NetworkAgent::unload_network_module();
        Slic3r::set_data_dir(previous_data_dir);
        std::filesystem::remove_all(root);
    }
    template<class Fn> Fn function(const char* name)
    {
        auto p = Slic3r::NetworkAgent::get_network_function(name);
        if (!p) throw std::runtime_error("Missing test plugin function");
        return reinterpret_cast<Fn>(p);
    }
};
}

TEST_CASE("Mac idle and wake never enter native pause or resume", "[network-subscription]")
{
    Fixture fixture;
    auto& agent = *fixture.agent;
    for (unsigned cycle = 0; cycle < 1000; ++cycle) {
        REQUIRE(agent.stop_subscribe("app") == 0);
        REQUIRE(agent.start_subscribe("app") == 0);
    }
    CHECK(fixture.function<unsigned(*)(void*)>("test_start_count")(agent.get_network_agent()) == 0);
    CHECK(fixture.function<unsigned(*)(void*)>("test_stop_count")(agent.get_network_agent()) == 0);
}

TEST_CASE("Mac idle keeps printer subscriptions and status callbacks intact", "[network-subscription]")
{
    Fixture fixture;
    auto& agent = *fixture.agent;
    auto emit = fixture.function<void(*)(void*)>("test_emit_status");
    std::vector<std::string> received;
    REQUIRE(agent.set_on_message_fn([&](std::string printer, std::string) { received.push_back(printer); }) == 0);
    REQUIRE(agent.add_subscribe({"first"}) == 0);
    emit(agent.get_network_agent());
    REQUIRE(agent.stop_subscribe("app") == 0);
    emit(agent.get_network_agent());
    REQUIRE(agent.start_subscribe("app") == 0);
    emit(agent.get_network_agent());
    CHECK(received == std::vector<std::string>{"first", "first", "first"});
    REQUIRE(agent.del_subscribe({"first"}) == 0);
    emit(agent.get_network_agent());
    CHECK(received.size() == 3);
    REQUIRE(agent.add_subscribe({"second"}) == 0);
    emit(agent.get_network_agent());
    CHECK(received.back() == "second");
    CHECK(received.size() == 4);
}

TEST_CASE("Mac explicit non-app modules still reach the vendor and propagate errors", "[network-subscription]")
{
    Fixture fixture;
    auto& agent = *fixture.agent;
    CHECK(agent.start_subscribe("tunnel") == -71);
    CHECK(agent.stop_subscribe("tunnel") == -72);
    CHECK(fixture.function<unsigned(*)(void*)>("test_start_count")(agent.get_network_agent()) == 1);
    CHECK(fixture.function<unsigned(*)(void*)>("test_stop_count")(agent.get_network_agent()) == 1);
}
