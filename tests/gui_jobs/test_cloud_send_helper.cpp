#define CATCH_CONFIG_RUNNER
#include "catch2/catch.hpp"
#include "slic3r/Utils/TyrCloudSendHelper.hpp"
#include "slic3r/Utils/TyrCloudSendRecovery.hpp"
#include "nlohmann/json.hpp"
#include <openssl/sha.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <atomic>
#include <thread>
#include <chrono>
#include <future>
#include <unistd.h>
#include <sys/stat.h>
using json       = nlohmann::json;
namespace fs     = std::filesystem;
namespace helper = Slic3r::TyrCloudSendHelper;
namespace {
std::string exe;
bool        jobs_empty(const fs::path &parent)
{
    for (const auto &entry : fs::directory_iterator(parent))
        if (entry.path().filename() != ".recovery.lock") return false;
    return true;
}
std::string read_file(const fs::path &p)
{
    std::ifstream f(p);
    return {std::istreambuf_iterator<char>(f), {}};
}
void write_file(const fs::path &p, const std::string &s)
{
    fs::create_directories(p.parent_path());
    std::ofstream(p) << s;
}
std::string digest(const std::string &s)
{
    unsigned char d[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char *>(s.data()), s.size(), d);
    std::ostringstream o;
    for (auto b : d) o << std::hex << std::setw(2) << std::setfill('0') << int(b);
    return o.str();
}
const std::string image_id = "sha256:" + std::string(64, 'a');
struct Fixture
{
    fs::path         root;
    std::string      oldpath;
    BBL::PrintParams params{};
    Fixture(const std::string &mode = "success")
    {
        char p[] = "/tmp/tyr-helper-test-XXXXXX";
        root     = mkdtemp(p);
        oldpath  = getenv("PATH") ? getenv("PATH") : "";
        fs::create_directories(root / "bin");
        fs::create_symlink(exe, root / "bin/docker");
        setenv("PATH", (root / "bin").c_str(), 1);
        setenv("TYR_TEST_ROOT", root.c_str(), 1);
        setenv("TYR_TEST_MODE", mode.c_str(), 1);
        json hashes = json::object();
        for (const auto &file : {"image.tar", "pin.json", "plugin/libbambu_networking.so", "plugin/libBambuSource.so", "plugin/liblive555.so",
                                 "plugin/linux_payload_manifest.json", "certs/slicer_base64.cer"}) {
            write_file(root / "resources/cloud-helper" / file, "fixture");
            hashes[file] = digest("fixture");
        }
        json manifest = {{"schema_version", 1},
                         {"protocol_version", 1},
                         {"host_patch_version", 1},
                         {"image_id", image_id},
                         {"image_file", "image.tar"},
                         {"platform", "linux/arm64"},
                         {"executable", "/runtime/tyr_bambu_cloud_host"},
                         {"network_abi", "02.05.02.58"},
                         {"hashes", hashes}};
        write_file(root / "resources/cloud-helper/manifest.json", manifest.dump());
        write_file(root / "source.3mf", "exact sliced job bytes");
        write_file(root / "BambuNetworkEngine.conf", "opaque SDK session SECRET_TOKEN");
        write_file(root / "config.3mf", "exact config bytes");
        params.filename        = (root / "source.3mf").string();
        params.config_filename = (root / "config.3mf").string();
        params.dev_id          = "printer-test";
        params.plate_index     = 1;
        params.ams_mapping     = "[3]";
        params.task_use_ams    = true;
    }
    ~Fixture()
    {
        setenv("PATH", oldpath.c_str(), 1);
        unsetenv("TYR_TEST_ROOT");
        unsetenv("TYR_TEST_MODE");
        for (const auto &entry : fs::recursive_directory_iterator(root))
            if (entry.is_directory()) fs::permissions(entry.path(), fs::perms::owner_all, fs::perm_options::replace);
        fs::remove_all(root);
    }
    int send(BBL::OnUpdateStatusFn update = {}, BBL::WasCancelledFn cancel = {}, BBL::OnWaitFn wait = {})
    {
        return helper::send(params, (root / "BambuNetworkEngine.conf").string(), "user-test", "US", (root / "resources").string(), (root / "data").string(), update, cancel, wait);
    }
    bool marked(const char *s) { return fs::exists(root / s); }
};
void respond(uint32_t id, const json &body)
{
    std::string s   = body.dump();
    uint32_t    h[] = {0x52424a50, 2, id, uint32_t(s.size())};
    std::cout.write(reinterpret_cast<char *>(h), 16);
    std::cout.write(s.data(), s.size());
    std::cout.flush();
}
int fake_docker(int argc, char **argv)
{
    fs::path                 root = getenv("TYR_TEST_ROOT");
    std::string              mode = getenv("TYR_TEST_MODE");
    std::vector<std::string> args(argv + 1, argv + argc);
    if (args.size() < 3 || args[0] != "--context" || args[1] != "desktop-linux") return 90;
    args.erase(args.begin(), args.begin() + 2);
    auto mark = [&](const char *key) { write_file(root / key, "yes"); };
    mark("docker-called");
    for (auto &a : args)
        if (a.find("SECRET_TOKEN") != std::string::npos) return 91;
    if (args[0] == "context") {
        std::cout << json({{"Host", mode == "remote-context" ? "tcp://remote:2376" : "unix:///tmp/fake-docker.sock"}}).dump();
        return 0;
    }
    if (args.size() > 1 && args[0] == "image" && args[1] == "inspect") {
        if ((mode == "import" || mode == "cancel-import") && !fs::exists(root / "loaded")) return 1;
        if (mode == "creation-race") {
            mark("image-inspected");
            const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
            while (!fs::exists(root / "hold-ready") && std::chrono::steady_clock::now() < until) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << json({{"Id", image_id}, {"Os", "linux"}, {"Architecture", mode == "wrong-architecture" ? "amd64" : "arm64"}}).dump();
        return 0;
    }
    if (args[0] == "load") {
        mark("loaded");
        if (mode == "cancel-import") std::this_thread::sleep_for(std::chrono::seconds(5));
        return 0;
    }
    if (args[0] == "rm" || (args.size() > 1 && args[0] == "container" && args[1] == "rm")) {
        if (mode == "cleanup-failure") return 1;
        mark("cleaned");
        if (mode == "cleanup-lock") {
            mark("cleanup-requested");
            const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
            while (!fs::exists(root / "cleanup-lock-held") && std::chrono::steady_clock::now() < until) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return 0;
    }
    if (args[0] == "container" && args[1] == "ls") {
        if (mode == "cleanup-failure")
            for (auto &a : args)
                if (a.find("name=^/") == 0) std::cout << a.substr(7, a.size() - 8);
        return 0;
    }
    if (args[0] != "run") return 92;
    mark("run");
    for (size_t i = 0; i + 1 < args.size(); ++i)
        if (args[i] == "--mount") {
            const auto &a     = args[i + 1];
            auto        begin = a.find("src=") + 4, end = a.find(",dst=");
            fs::path    source = a.substr(begin, end - begin);
            if (a.find("dst=/job") != std::string::npos && mode != "verify" && mode != "verify-rejected") {
                if (read_file(source / "plate.gcode.3mf") != "exact sliced job bytes" || read_file(source / "config.3mf") != "exact config bytes") return 97;
                struct stat st{};
                stat((source / "plate.gcode.3mf").c_str(), &st);
                if ((st.st_mode & 0777) != 0600) return 98;
                if (a.find(",readonly") == std::string::npos) return 99;
                mark("staged-bytes-private");
                if (mode == "cleanup-files") fs::permissions(source, fs::perms::owner_read | fs::perms::owner_exec, fs::perm_options::replace);
            }
            if (a.find("dst=/session") != std::string::npos) {
                struct stat st{};
                stat(source.parent_path().c_str(), &st);
                if ((st.st_mode & 0777) != 0700) return 100;
                if (read_file(source / "BambuNetworkEngine.conf") != "opaque SDK session SECRET_TOKEN") return 101;
                stat((source / "BambuNetworkEngine.conf").c_str(), &st);
                if ((st.st_mode & 0777) != 0600) return 102;
                mark("private-auth");
            }
        }
    uint32_t pending = 0;
    bool     sent    = false;
    int64_t  job     = 0;
    for (;;) {
        uint32_t h[4];
        if (!std::cin.read(reinterpret_cast<char *>(h), 16)) return 0;
        if (h[0] != 0x52424a50 || h[1] != 1 || h[3] > 16 * 1024 * 1024) return 93;
        std::string s(h[3], '\0');
        if (!std::cin.read(&s[0], s.size())) return 94;
        auto        req = json::parse(s);
        auto        p   = req.at("payload");
        std::string m   = req.at("method");
        json        r   = {{"ok", true}, {"value", 0}};
        if (m == "bridge.handshake")
            r = {{"ok", true},
                 {"protocol_version", 1},
                 {"network_abi_version", "02.05.02.58"},
                 {"auth_capabilities", {{"tyr_cloud_helper_version", 1}, {"tyr_redaction_version", 1}, {"tyr_wait_reply_version", 1}, {"bambu_network_start_print", true}}}};
        if (m == "net.create_agent") r["value"] = 1;
        if (m == "net.start" && mode == "cancel-init") {
            mark("initializing");
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        if (m == "net.get_user_print_info") {
            r["http_code"] = mode == "verify-rejected" ? 401 : 200;
            r["http_body"] = "private inventory";
            mark("account-checked");
        }
        if (m == "net.change_user") return 95; // Native auth import uses the opaque session, never a fabricated login payload.
        if (m == "net.is_user_login") r["value"] = true;
        if (m == "net.get_user_id") r["value"] = mode == "wrong-user" ? "wrong-user" : "user-test";
        if (m == "net.start_print") {
            mark("submitted");
            pending = h[2];
            job     = p.at("client_job_id");
            write_file(root / "print-params", p.at("params").dump());
            if (mode == "crash") return 44;
            if (mode == "malformed") {
                uint32_t bad[] = {0, 2, h[2], 0};
                std::cout.write(reinterpret_cast<char *>(bad), 16);
                std::cout.flush();
                continue;
            }
            continue;
        }
        if (m == "bridge.poll_events") {
            r["events"] = json::array();
            if (pending && !sent) {
                sent = true;
                r["events"].push_back({{"name", "job.update_status"}, {"payload", {{"job_id", job}, {"status", 2}, {"code", 35}, {"msg", "Uploading"}}}});
                if (mode != "cancel") r["events"].push_back({{"name", "job.wait"}, {"payload", {{"job_id", job}, {"request_id", 8}, {"status", 7}, {"job_info", "{}"}}}});
            }
        }
        if (m == "bridge.job_wait_reply") {
            const bool reply = p.at("reply").get<bool>();
            if (reply != (mode != "wait-false")) return 96;
            mark("wait-replied");
            respond(pending, {{"ok", true}, {"value", mode == "sdk-error" || !reply ? -42 : 0}});
            pending = 0;
        }
        if (m == "bridge.job_cancel") {
            mark("cancelled");
            respond(pending, {{"ok", true}, {"value", BAMBU_NETWORK_ERR_CANCELED}});
            pending = 0;
        }
        respond(h[2], r);
        if (mode == "crash-wait" && m == "bridge.poll_events" && sent) return 44;
    }
}
} // namespace
TEST_CASE("Cloud helper stages the selected plate, authenticates privately and bridges callbacks", "[cloud-helper]")
{
    Fixture f;
    int     status = 0;
    bool    waited = false;
    REQUIRE(helper::available((f.root / "resources").string()));
    REQUIRE(f.send(
                [&](int s, int code, std::string msg) {
                    status = s;
                    REQUIRE(code == 35);
                    REQUIRE(msg == "Uploading");
                },
                {},
                [&](int s, std::string info) {
                    waited = true;
                    return s == 7 && info == "{}";
                }) == 0);
    REQUIRE(status == 2);
    REQUIRE(waited);
    REQUIRE(f.marked("staged-bytes-private"));
    REQUIRE(f.marked("private-auth"));
    REQUIRE(f.marked("wait-replied"));
    REQUIRE(f.marked("cleaned"));
    auto p = json::parse(read_file(f.root / "print-params"));
    REQUIRE(p["filename"] == "/job/plate.gcode.3mf");
    REQUIRE(p["plate_index"] == 1);
    REQUIRE(p["ams_mapping"] == "[3]");
    REQUIRE(p["task_use_ams"] == true);
    REQUIRE(read_file(f.root / "source.3mf") == "exact sliced job bytes");
    REQUIRE(read_file(f.root / "BambuNetworkEngine.conf") == "opaque SDK session SECRET_TOKEN");
    REQUIRE(jobs_empty(f.root / "data/cloud-send-jobs"));
}
TEST_CASE("Cloud helper refuses changed resources before invoking any runtime", "[cloud-helper]")
{
    Fixture f;
    write_file(f.root / "resources/cloud-helper/image.tar", "tampered");
    REQUIRE_THROWS_AS(f.send(), std::runtime_error);
    REQUIRE_FALSE(f.marked("docker-called"));
}
TEST_CASE("Cloud helper verifies account identity before submitting", "[cloud-helper]")
{
    Fixture f("wrong-user");
    REQUIRE_THROWS_AS(f.send(), std::runtime_error);
    REQUIRE_FALSE(f.marked("submitted"));
    REQUIRE(f.marked("cleaned"));
}
TEST_CASE("Cloud helper imports its verified runtime only when absent", "[cloud-helper]")
{
    Fixture f("import");
    REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == 0);
    REQUIRE(f.marked("loaded"));
}
TEST_CASE("Cloud helper preserves SDK failures and routes cancellation", "[cloud-helper]")
{
    SECTION("sdk error")
    {
        Fixture f("sdk-error");
        REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == -42);
        REQUIRE(f.marked("cleaned"));
    }
    SECTION("cancel")
    {
        Fixture f("cancel");
        bool    cancel = false;
        REQUIRE(f.send([&](int, int, std::string) { cancel = true; }, [&] { return cancel; }) == BAMBU_NETWORK_ERR_CANCELED);
        REQUIRE(f.marked("cancelled"));
        REQUIRE(f.marked("cleaned"));
    }
}
TEST_CASE("Cloud helper cleans up after transport failures without exposing account data", "[cloud-helper]")
{
    for (const auto &mode : {"crash", "malformed"}) {
        Fixture f(mode);
        try {
            f.send();
            FAIL("expected transport failure");
        } catch (const std::runtime_error &e) {
            REQUIRE(std::string(e.what()).find("SECRET_TOKEN") == std::string::npos);
        }
        REQUIRE(f.marked("submitted"));
        REQUIRE(f.marked("cleaned"));
    }
}
TEST_CASE("Cloud helper preserves a false completion reply", "[cloud-helper]")
{
    Fixture f("wait-false");
    REQUIRE(f.send({}, {}, [](int, std::string) { return false; }) == -42);
    REQUIRE(f.marked("wait-replied"));
}
TEST_CASE("Cloud helper cancellation remains responsive during completion callback", "[cloud-helper]")
{
    Fixture           f("wait-cancel");
    std::atomic<bool> cancel{false};
    REQUIRE(f.send(
                {}, [&] { return cancel.load(); },
                [&](int, std::string) {
                    cancel           = true;
                    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(4);
                    while (!f.marked("cancelled") && std::chrono::steady_clock::now() < until) std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    return false;
                }) == BAMBU_NETWORK_ERR_CANCELED);
    REQUIRE(f.marked("cancelled"));
    REQUIRE(f.marked("cleaned"));
}
TEST_CASE("Cloud helper rejects incompatible images and unverified SDK files", "[cloud-helper]")
{
    SECTION("architecture")
    {
        Fixture f("wrong-architecture");
        REQUIRE_THROWS_AS(f.send(), std::runtime_error);
        REQUIRE_FALSE(f.marked("run"));
    }
    SECTION("unverified file")
    {
        Fixture f;
        write_file(f.root / "resources/cloud-helper/plugin/extra.so", "unverified");
        REQUIRE_THROWS_AS(f.send(), std::runtime_error);
        REQUIRE_FALSE(f.marked("docker-called"));
    }
}
TEST_CASE("Cloud helper retains private staging when container removal is unconfirmed", "[cloud-helper]")
{
    Fixture f("cleanup-failure");
    REQUIRE_THROWS_WITH(f.send({}, {}, [](int, std::string) { return true; }), Catch::Matchers::Contains("could not be stopped"));
    REQUIRE_FALSE(jobs_empty(f.root / "data/cloud-send-jobs"));
}
TEST_CASE("Cloud helper account preflight is read-only and checks server acceptance", "[cloud-helper]")
{
    for (const char *mode : {"verify", "verify-rejected"}) {
        Fixture f(mode);
        auto    verify = [&] {
            helper::verify_account((f.root / "BambuNetworkEngine.conf").string(), "user-test", "US", (f.root / "resources").string(), (f.root / "data").string());
        };
        if (std::string(mode) == "verify")
            REQUIRE_NOTHROW(verify());
        else
            REQUIRE_THROWS_AS(verify(), std::runtime_error);
        REQUIRE(f.marked("account-checked"));
        REQUIRE_FALSE(f.marked("submitted"));
        REQUIRE(f.marked("cleaned"));
    }
}
TEST_CASE("Cloud helper refuses remote Docker contexts before mounting private account data", "[cloud-helper]")
{
    Fixture f("remote-context");
    REQUIRE_THROWS_WITH(f.send(), Catch::Matchers::Contains("Remote Docker contexts"));
    REQUIRE_FALSE(f.marked("run"));
}
TEST_CASE("Cloud helper never forwards a LAN route or printer access credentials", "[cloud-helper]")
{
    Fixture f;
    f.params.dev_ip          = "192.0.2.15";
    f.params.username        = "bblp";
    f.params.password        = "private-access-code";
    f.params.connection_type = "lan";
    f.params.comments        = "native acceleration";
    REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == 0);
    const auto p = json::parse(read_file(f.root / "print-params"));
    REQUIRE(p["dev_ip"] == "");
    REQUIRE(p["username"] == "");
    REQUIRE(p["password"] == "");
    REQUIRE(p["connection_type"] == "cloud");
    REQUIRE(p["comments"] == "no_ip");
    REQUIRE(f.params.dev_ip == "192.0.2.15");
    REQUIRE(f.params.password == "private-access-code");
}
TEST_CASE("Cloud helper cancels preparation without waiting for runtime operations", "[cloud-helper]")
{
    for (const char *mode : {"cancel-import", "cancel-init"}) {
        Fixture    f(mode);
        const auto started = std::chrono::steady_clock::now();
        REQUIRE(f.send({}, [&] { return f.marked("loaded") || f.marked("initializing"); }) == BAMBU_NETWORK_ERR_CANCELED);
        REQUIRE(std::chrono::steady_clock::now() - started < std::chrono::seconds(2));
        REQUIRE_FALSE(f.marked("submitted"));
        if (std::string(mode) == "cancel-init") REQUIRE(f.marked("cleaned"));
    }
}
TEST_CASE("Cloud helper stops a failed transport before joining its completion callback", "[cloud-helper]")
{
    Fixture f("crash-wait");
    bool    cleanup_before_callback_return = false;
    REQUIRE_THROWS_AS(f.send({}, {},
                             [&](int, std::string) {
                                 const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
                                 while (!f.marked("cleaned") && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                 cleanup_before_callback_return = f.marked("cleaned");
                                 return false;
                             }),
                      std::runtime_error);
    REQUIRE(cleanup_before_callback_return);
}
TEST_CASE("Cloud helper reconciles an interrupted submission before admitting another print", "[cloud-helper]")
{
    Fixture    f;
    const auto parent = f.root / "data/cloud-send-jobs";
    const auto stale  = parent / "job-ABC123";
    fs::create_directories(stale);
    fs::permissions(parent, fs::perms::owner_all, fs::perm_options::replace);
    fs::permissions(stale, fs::perms::owner_all, fs::perm_options::replace);
    {
        Slic3r::TyrCloudSendRecovery::Record record(stale, "tyr-cloud-send-" + std::to_string(getpid()) + "-ABC123", f.params.dev_id);
        record.set_phase("submitting");
    }
    REQUIRE_THROWS_WITH(f.send(), Catch::Matchers::Contains("previous interrupted send"));
    REQUIRE(f.marked("cleaned"));
    REQUIRE_FALSE(f.marked("submitted"));
    REQUIRE_FALSE(fs::exists(stale));
    REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == 0);
}
TEST_CASE("Cloud helper retains failed cleanup and consumes its uncertainty only after stopping", "[cloud-helper]")
{
    Fixture    f("cleanup-failure");
    const auto parent = f.root / "data/cloud-send-jobs";
    const auto stale  = parent / "job-DEF456";
    fs::create_directories(stale);
    fs::permissions(parent, fs::perms::owner_all, fs::perm_options::replace);
    fs::permissions(stale, fs::perms::owner_all, fs::perm_options::replace);
    {
        Slic3r::TyrCloudSendRecovery::Record record(stale, "tyr-cloud-send-" + std::to_string(getpid()) + "-DEF456", f.params.dev_id);
        record.set_phase("submitting");
    }
    REQUIRE_THROWS_WITH(f.send(), Catch::Matchers::Contains("previous print helper could not be stopped"));
    REQUIRE(fs::exists(stale / "owner.json"));
    REQUIRE_FALSE(f.marked("run"));
    setenv("TYR_TEST_MODE", "success", 1);
    REQUIRE_THROWS_WITH(f.send(), Catch::Matchers::Contains("previous interrupted send"));
    REQUIRE_FALSE(f.marked("submitted"));
    REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == 0);
}
TEST_CASE("Cloud helper protects job creation after preflight reconciliation", "[cloud-helper]")
{
    Fixture           f("creation-race");
    std::atomic<bool> release{false};
    auto              competing = std::async(std::launch::async, [&] {
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!f.marked("image-inspected") && std::chrono::steady_clock::now() < until) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        Slic3r::TyrCloudSendRecovery::Record::DirectoryGuard guard(f.root / "data/cloud-send-jobs");
        write_file(f.root / "hold-ready", "ready");
        while (!release.load() && std::chrono::steady_clock::now() < until) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    });
    std::string       failure;
    try {
        f.send();
    } catch (const std::runtime_error &e) {
        failure = e.what();
    }
    release = true;
    competing.get();
    REQUIRE(failure.find("Another print attempt") != std::string::npos);
    REQUIRE_FALSE(f.marked("run"));
    REQUIRE(jobs_empty(f.root / "data/cloud-send-jobs"));
}
TEST_CASE("Cloud helper preserves recovery ownership after a partial file cleanup failure", "[cloud-helper]")
{
    Fixture f("cleanup-files");
    REQUIRE_THROWS_WITH(f.send({}, {}, [](int, std::string) { return true; }), Catch::Matchers::Contains("private files could not be removed"));
    bool retained = false;
    for (const auto &entry : fs::directory_iterator(f.root / "data/cloud-send-jobs")) {
        if (!entry.is_directory()) continue;
        retained = fs::exists(entry.path() / "owner.json") && fs::exists(entry.path() / "owner.lock");
        fs::permissions(entry.path() / "payload", fs::perms::owner_all, fs::perm_options::replace);
    }
    REQUIRE(retained);
}
TEST_CASE("Cloud helper never calls a submitted job unsent when cleanup is busy", "[cloud-helper]")
{
    Fixture           f("cleanup-lock");
    std::atomic<bool> release{false};
    auto              competing = std::async(std::launch::async, [&] {
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!f.marked("cleanup-requested") && std::chrono::steady_clock::now() < until) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        Slic3r::TyrCloudSendRecovery::Record::DirectoryGuard guard(f.root / "data/cloud-send-jobs");
        write_file(f.root / "cleanup-lock-held", "ready");
        while (!release.load() && std::chrono::steady_clock::now() < until) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    });
    std::string       failure;
    try {
        f.send({}, {}, [](int, std::string) { return true; });
    } catch (const std::runtime_error &e) {
        failure = e.what();
    }
    release = true;
    competing.get();
    REQUIRE(f.marked("submitted"));
    REQUIRE(failure.find("Check the printer") != std::string::npos);
    REQUIRE(failure.find("No new print was submitted") == std::string::npos);
    REQUIRE_FALSE(jobs_empty(f.root / "data/cloud-send-jobs"));
}
int main(int argc, char **argv)
{
    if (fs::path(argv[0]).filename() == "docker") return fake_docker(argc, argv);
    exe = fs::canonical(argv[0]).string();
    return Catch::Session().run(argc, argv);
}
