#include "catch2/catch.hpp"
#include "slic3r/Utils/TyrCloudCameraRequests.hpp"
#include <atomic>
#include <future>
#include <thread>

using Requests=Slic3r::TyrCloudCameraRequests;
using namespace std::chrono_literals;

TEST_CASE("Camera setup runs away from the caller and returns its own result", "[camera-requests]")
{
    Requests requests;
    const auto caller=std::this_thread::get_id();
    std::promise<std::thread::id> worker;
    std::promise<std::string> reply;
    auto result=reply.get_future();auto started=worker.get_future();
    std::atomic<bool> release{false};
    REQUIRE(requests.submit([&](Requests::Cancelled cancelled){
        worker.set_value(std::this_thread::get_id());
        while(!release&&!cancelled())std::this_thread::sleep_for(1ms);
        return "private-camera-session";
    },[&](std::string value, Slic3r::TyrCloudCameraGuard){reply.set_value(std::move(value));}));
    REQUIRE(started.wait_for(2s)==std::future_status::ready);
    CHECK(started.get()!=caller);
    CHECK(result.wait_for(0ms)==std::future_status::timeout);
    release=true;
    REQUIRE(result.wait_for(2s)==std::future_status::ready);
    CHECK(result.get()=="private-camera-session");
}
TEST_CASE("Account changes cancel old camera work and discard pending and late results", "[camera-requests]")
{
    Requests requests;
    std::promise<void> started;
    auto beginning=started.get_future();
    std::atomic<int> old_callbacks{0},pending_runs{0};
    REQUIRE(requests.submit([&](Requests::Cancelled cancelled){
        started.set_value();while(!cancelled())std::this_thread::sleep_for(1ms);return "old account";
    },[&](std::string, Slic3r::TyrCloudCameraGuard){++old_callbacks;}));
    REQUIRE(beginning.wait_for(2s)==std::future_status::ready);
    REQUIRE(requests.submit([&](Requests::Cancelled){++pending_runs;return "pending old account";},[&](std::string, Slic3r::TyrCloudCameraGuard){++old_callbacks;}));
    requests.cancel_all();
    std::promise<std::string> fresh;
    auto result=fresh.get_future();
    REQUIRE(requests.submit([](Requests::Cancelled){return "new account";},[&](std::string value, Slic3r::TyrCloudCameraGuard){fresh.set_value(std::move(value));}));
    REQUIRE(result.wait_for(2s)==std::future_status::ready);
    CHECK(result.get()=="new account");
    CHECK(old_callbacks==0);
    CHECK(pending_runs==0);
}
TEST_CASE("Camera failures cannot expose exception payloads or stop following requests", "[camera-requests]")
{
    Requests requests;
    std::promise<std::string> failed,next;
    auto result=failed.get_future();auto later=next.get_future();
    REQUIRE(requests.submit([](Requests::Cancelled)->std::string{throw std::runtime_error("SECRET camera token");},[&](std::string value, Slic3r::TyrCloudCameraGuard){failed.set_value(std::move(value));}));
    REQUIRE(requests.submit([](Requests::Cancelled){return "next";},[&](std::string value, Slic3r::TyrCloudCameraGuard){next.set_value(std::move(value));}));
    REQUIRE(result.wait_for(2s)==std::future_status::ready);
    const auto message=result.get();
    CHECK_FALSE(message.empty());
    CHECK(message.find("SECRET")==std::string::npos);
    REQUIRE(later.wait_for(2s)==std::future_status::ready);
    CHECK(later.get()=="next");
}
TEST_CASE("Closing camera requests cancels and joins the active request", "[camera-requests]")
{
    std::promise<void> started;auto beginning=started.get_future();
    std::atomic<bool> stopped{false},delivered{false};
    const auto before=std::chrono::steady_clock::now();
    {
        Requests requests;
        REQUIRE(requests.submit([&](Requests::Cancelled cancelled){
            started.set_value();while(!cancelled())std::this_thread::sleep_for(1ms);stopped=true;return "obsolete";
        },[&](std::string, Slic3r::TyrCloudCameraGuard){delivered=true;}));
        REQUIRE(beginning.wait_for(2s)==std::future_status::ready);
    }
    CHECK(stopped);
    CHECK_FALSE(delivered);
    CHECK(std::chrono::steady_clock::now()-before<2s);
}

TEST_CASE("A queued GUI camera result is invalid after an account change or shutdown", "[camera-requests]")
{
    std::promise<std::function<bool()>> queued;
    auto delivery = queued.get_future();
    std::atomic<bool> applied{false};
    {
        Requests requests;
        REQUIRE(requests.submit([](Requests::Cancelled) { return "private old-account session"; },
            [&](std::string, Slic3r::TyrCloudCameraGuard guard) {
                queued.set_value([guard, &applied] { return guard.deliver([&] { applied = true; }); });
            }));
        REQUIRE(delivery.wait_for(2s) == std::future_status::ready);
        requests.cancel_all();
    }
    CHECK_FALSE(delivery.get()());
    CHECK_FALSE(applied);
}

TEST_CASE("Account transitions serialize identity snapshots and request admission", "[camera-requests]")
{
    Requests requests;
    std::atomic<bool> release{false};
    std::atomic<int> account{1}, snapshot{0};
    std::promise<void> entered;
    auto beginning = entered.get_future();
    auto change = std::async(std::launch::async, [&] {
        return requests.account_change([&] {
            entered.set_value();
            while (!release) std::this_thread::sleep_for(1ms);
            account = 2;
            return 0;
        });
    });
    const bool changing = beginning.wait_for(2s) == std::future_status::ready;
    std::promise<std::string> answer;
    auto result = answer.get_future();
    auto admission = std::async(std::launch::async, [&] {
        return requests.submit_factory([&] {
            snapshot = account.load();
            return Requests::Work([value = snapshot.load()](Requests::Cancelled) { return std::to_string(value); });
        }, [&](std::string value, Slic3r::TyrCloudCameraGuard) { answer.set_value(std::move(value)); });
    });
    const auto during_change = admission.wait_for(20ms);
    release = true;
    REQUIRE(changing);
    CHECK(during_change == std::future_status::timeout);
    REQUIRE(change.get() == 0);
    REQUIRE(admission.get());
    REQUIRE(result.wait_for(2s) == std::future_status::ready);
    CHECK(snapshot == 2);
    CHECK(result.get() == "2");
}

TEST_CASE("Account mutation refuses reentrant requests before taking an identity snapshot", "[camera-requests]")
{
    Requests requests;
    bool captured = false, admitted = false;
    REQUIRE(requests.account_change([&] {
        admitted = requests.submit_factory([&] { captured = true; return Requests::Work{}; },
            [](std::string, Slic3r::TyrCloudCameraGuard) {});
        return 0;
    }) == 0);
    CHECK_FALSE(captured);
    CHECK_FALSE(admitted);
}

TEST_CASE("Account cancellation waits for an already entered camera delivery", "[camera-requests]")
{
    Requests requests;
    std::atomic<bool> release{false};
    std::promise<void> entered;
    auto beginning = entered.get_future();
    REQUIRE(requests.submit([](Requests::Cancelled) { return "ready"; },
        [&](std::string, Slic3r::TyrCloudCameraGuard) {
            entered.set_value();
            while (!release) std::this_thread::sleep_for(1ms);
        }));
    const bool delivering = beginning.wait_for(2s) == std::future_status::ready;
    auto cancel = std::async(std::launch::async, [&] { requests.cancel_all(); });
    const auto before_release = cancel.wait_for(20ms);
    release = true;
    REQUIRE(delivering);
    CHECK(before_release == std::future_status::timeout);
    REQUIRE(cancel.wait_for(2s) == std::future_status::ready);
    cancel.get();
}
