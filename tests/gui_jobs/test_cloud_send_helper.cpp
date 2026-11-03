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
        if (mode == "tamper-copy") write_file(root / "resources/cloud-helper/plugin/libbambu_networking.so", "changed after verification");
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
    fs::path workspace;
    int      runtime_mounts = 0;
    for (size_t i = 0; i + 1 < args.size(); ++i)
        if (args[i] == "--mount") {
            const auto &a     = args[i + 1];
            auto        begin = a.find("src=") + 4, end = a.find(",dst=");
            fs::path    source = a.substr(begin, end - begin);
            if (a.find("dst=/job") != std::string::npos) workspace = source.parent_path();
            // Model Docker Desktop's restricted host filesystem: all mounts
            // must come from the private user-data workspace, never the app.
            const auto relative = source.lexically_relative(workspace);
            if (workspace.empty() || relative.empty() || *relative.begin() == "..") return 125;
            if (a.find("dst=/plugin") != std::string::npos || a.find("dst=/certs") != std::string::npos) {
                if (a.find(",readonly") == std::string::npos) return 103;
                struct stat st{};
                stat(source.c_str(), &st);
                if ((st.st_mode & 0777) != 0700) return 104;
                const bool                     plugin   = a.find("dst=/plugin") != std::string::npos;
                const std::vector<std::string> expected = plugin ? std::vector<std::string>{"libbambu_networking.so", "libBambuSource.so", "liblive555.so",
                                                                                            "linux_payload_manifest.json"} :
                                                                   std::vector<std::string>{"slicer_base64.cer"};
                size_t                         count    = 0;
                for (const auto &entry : fs::directory_iterator(source)) ++count;
                if (count != expected.size()) return 105;
                for (const auto &filename : expected) {
                    if (read_file(source / filename) != "fixture") return 106;
                    stat((source / filename).c_str(), &st);
                    if ((st.st_mode & 0777) != 0600) return 107;
                }
                if (++runtime_mounts == 2) mark("staged-runtime");
            }
            if (a.find("dst=/job") != std::string::npos && !fs::is_empty(source)) {
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
    uint32_t pending   = 0;
    bool     sent      = false;
    int64_t  job       = 0;
    bool     connected = false, server_reported = false, selected = false, printer_reported = false;
    bool     message_callback = false, printer_callback = false, subscribe_callback = false;
    bool     selection_cleared = false, inventory_checked = false, tunnel_started = false, subscribed = false, multi_machine = false;
    unsigned preparation_polls = 0, certificate_calls = 0, submissions = 0, status_requests = 0;
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
        if (m == "net.set_on_message_fn") message_callback = true;
        if (m == "net.set_on_printer_connected_fn") printer_callback = true;
        if (m == "net.set_on_subscribe_failure_fn") subscribe_callback = true;
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
            r["http_body"] =
                json({{"devices", json::array({{{"dev_id", mode == "missing-printer" ? "another-printer" : "printer-test"}, {"dev_online", mode != "offline-printer"}}})}}).dump();
            inventory_checked = true;
            mark("account-checked");
        }
        if (m == "net.set_user_selected_machine") {
            if (p.at("dev_id") == "")
                selection_cleared = !connected;
            else {
                if (p.at("dev_id") != "printer-test" || !inventory_checked || !subscribed || !multi_machine) return 108;
                selected = true;
                mark("printer-selected");
            }
        }
        if (m == "net.connect_server") connected = true;
        if (m == "net.is_server_connected") r["value"] = server_reported;
        if (m == "net.enable_multi_machine") multi_machine = p.at("enable") == true;
        if (m == "net.start_subscribe") {
            if (p.at("module") != "tunnel") return 109;
            tunnel_started = true;
        }
        if (m == "net.add_subscribe") {
            if (p.at("devs") != json::array({"printer-test"}) || !tunnel_started) return 110;
            subscribed = true;
        }
        if (m == "net.install_device_cert") {
            if (p.at("dev_id") != "printer-test" || p.at("lan_only") != false) return 111;
            write_file(root / "certificate-calls", std::to_string(++certificate_calls));
        }
        if (m == "net.send_message") {
            const auto query    = json::parse(p.at("msg").get<std::string>());
            const auto sequence = std::stoll(query.at("pushing").at("sequence_id").get<std::string>());
            if (sequence < 20000 || sequence > 29999) return 115;
            if (p.at("dev_id") != "printer-test" || p.at("qos") != 0 || p.at("flag") != 0 || !selected || !certificate_calls || query.size() != 1 ||
                query.at("pushing").at("command") != "pushall" || query.at("pushing").at("version") != 1 || query.at("pushing").at("push_target") != 1)
                return 112;
            if (json::parse(read_file(workspace / "owner.json")).at("phase") != "running") return 113;
            write_file(root / "status-request-count", std::to_string(++status_requests));
            if (mode == "query-delayed" && status_requests == 1) r["value"] = -2;
        }
        if (m == "net.change_user") return 95; // Native auth import uses the opaque session, never a fabricated login payload.
        if (m == "net.is_user_login") r["value"] = true;
        if (m == "net.get_user_id") r["value"] = mode == "wrong-user" ? "wrong-user" : "user-test";
        if (m == "net.get_camera_url") {
            if (!printer_reported || p.at("dev_id") != "printer-test|01.02|\"tutk\",\"agora\"") return 116;
            write_file(root / "camera-requested", "yes");
            if(mode=="camera-wait") continue;
            r["result"] = mode == "camera-denied" ? "[1:403]" :
                          mode == "camera-local" ? "bambu:///local/192.0.2.1?passwd=SECRET_TOKEN" :
                          mode == "camera-wrong-protocol" ? "bambu:///tutk-invalid?token=SECRET_TOKEN" :
                          mode == "camera-agora" ? "bambu:///camera/agora?token=SECRET_TOKEN" :
                          "bambu:///tutk?uid=private-camera-uid&authkey=SECRET_TOKEN";
        }
        if (m == "net.start_print") {
            if (!fs::exists(root/"staged-bytes-private")) return 97;
            mark("submitted");
            write_file(root / "submission-count", std::to_string(++submissions));
            if (json::parse(read_file(workspace / "owner.json")).at("phase") != "submitting") return 114;
            if (!printer_reported || !selection_cleared || !message_callback || !printer_callback || !subscribe_callback) {
                respond(h[2], {{"ok", true}, {"value", BAMBU_NETOWRK_ERR_PRINT_SP_ENC_FLAG_NOT_READY}});
                continue;
            }
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
            if (!pending && connected && !server_reported) {
                if (mode != "connection-timeout") {
                    server_reported = true;
                    r["events"].push_back({{"agent", 1}, {"name", "on_server_connected"}, {"payload", {{"return_code", 0}, {"reason_code", 0}}}});
                }
                mark("cloud-polled");
            }
            if (!pending && selected && certificate_calls && status_requests && !printer_reported) {
                ++preparation_polls;
                mark("printer-polled");
                if (mode == "expired-login")
                    r["events"].push_back({{"agent", 1}, {"name", "on_user_login"}, {"payload", {{"login", false}, {"online_login", 0}}}});
                else if (mode == "auth-rejected")
                    r["events"].push_back({{"agent", 1}, {"name", "on_http_error"}, {"payload", {{"http_code", 401}}}});
                else if (mode == "subscription-failed")
                    r["events"].push_back({{"agent", 1}, {"name", "on_subscribe_failure"}, {"payload", {{"topic", "device/printer-test/report"}}}});
                else if (mode == "certificate-refresh" && preparation_polls < 5) {
                    for (unsigned i = 0; i < 20; ++i) {
                        r["events"].push_back({{"agent", 1}, {"name", "on_message"}, {"payload", {{"dev_id", ""}, {"msg", "wait_info"}}}});
                        r["events"].push_back({{"agent", 1}, {"name", "on_printer_connected"}, {"payload", {{"topic_str", "tunnel/printer-test"}}}});
                    }
                } else if (mode != "no-report" && mode != "cancel-printer" && (mode != "late-report" || preparation_polls >= 4) &&
                           (mode != "query-delayed" || status_requests >= 2)) {
                    const std::string target = mode == "wrong-printer" ? "another-printer" : "printer-test";
                    const std::string report = mode == "invalid-report"     ? "not JSON" :
                                               mode == "heartbeat"          ? "{}" :
                                               mode == "empty-print-report" ? R"({"print":{}})" :
                                               mode == "non-printer-report" ? R"({"info":{"command":"get_version"}})" :
                                                                              R"({"print":{"gcode_state":"IDLE","command":"push_status"}})";
                    r["events"].push_back({{"agent", mode == "wrong-agent" ? 99 : 1}, {"name", "on_message"}, {"payload", {{"dev_id", target}, {"msg", report}}}});
                    if (mode == "wrong-printer" || mode == "wrong-agent" || mode == "invalid-report" || mode == "non-printer-report" || mode == "heartbeat" ||
                        mode == "empty-print-report") {
                        // Let the client consume the first invalid report. If
                        // it accepts that report it will submit before this
                        // second poll gives the test permission to cancel.
                        if (preparation_polls >= 2) mark("invalid-report-delivered");
                    } else {
                        printer_reported = true;
                        mark("printer-report-delivered");
                    }
                }
            }
            if (pending && !sent) {
                sent = true;
                r["events"].push_back({{"name", "job.update_status"}, {"payload", {{"job_id", job}, {"status", 2}, {"code", 35}, {"msg", "Uploading"}}}});
                if (mode == "timing" || mode == "timing-storm") {
                    const unsigned count = mode == "timing-storm" ? 100 : 8;
                    for (unsigned i = 0; i < count; ++i)
                        r["events"].push_back({{"name", "job.update_status"},
                                               {"payload",
                                                {{"job_id", job},
                                                 {"status", mode == "timing-storm" ? int(i % 2 + 2) : 2},
                                                 {"code", int(i)},
                                                 {"msg", "PRIVATE SDK MESSAGE SECRET_TOKEN printer-test"}}}});
                }
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
            // Allow the waiting completion callback to return before the
            // cancellation RPC finishes. A terminal job needs no wait reply.
            if (mode == "wait-cancel") std::this_thread::sleep_for(std::chrono::milliseconds(100));
            respond(pending, {{"ok", true}, {"value", BAMBU_NETWORK_ERR_CANCELED}});
            pending = 0;
        }
        respond(h[2], r);
        if (mode == "early-result" && pending && sent) {
            respond(pending, {{"ok", true}, {"value", 0}});
            pending = 0;
        }
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
TEST_CASE("Camera authorization prepares the target without submitting or replacing send timing", "[cloud-helper][camera]")
{
    Fixture f;
    const auto timing=f.root/"data/cloud-send-latest-timing.json";
    write_file(timing,"last real print attempt");
    const auto url=helper::camera_url((f.root/"BambuNetworkEngine.conf").string(),"user-test","US",
        (f.root/"resources").string(),(f.root/"data").string(),"printer-test|01.02|\"tutk\",\"agora\"");
    REQUIRE(url=="bambu:///tutk?uid=private-camera-uid&authkey=SECRET_TOKEN");
    REQUIRE(f.marked("printer-report-delivered"));
    REQUIRE(f.marked("camera-requested"));
    REQUIRE_FALSE(f.marked("submitted"));
    REQUIRE(jobs_empty(f.root/"data/cloud-send-jobs"));
    REQUIRE(read_file(timing)=="last real print attempt");
}
TEST_CASE("Camera authorization rejects refused or non-cloud sessions without exposing their values", "[cloud-helper][camera]")
{
    for(const char* mode:{"camera-denied","camera-local","camera-wrong-protocol"}) {
        Fixture f(mode);
        std::string error;
        try { (void)helper::camera_url((f.root/"BambuNetworkEngine.conf").string(),"user-test","US",
            (f.root/"resources").string(),(f.root/"data").string(),"printer-test|01.02|\"tutk\",\"agora\""); }
        catch(const std::exception& e){error=e.what();}
        REQUIRE(f.marked("camera-requested"));
        REQUIRE(error.find("supported camera connection")!=std::string::npos);
        REQUIRE(error.find("SECRET_TOKEN")==std::string::npos);
        REQUIRE(error.find("192.0.2.1")==std::string::npos);
        REQUIRE_FALSE(f.marked("submitted"));
        REQUIRE(jobs_empty(f.root/"data/cloud-send-jobs"));
    }
}
TEST_CASE("Camera requests cancel and stop before requesting a session", "[cloud-helper][camera]")
{
    Fixture f("cancel-printer");
    REQUIRE_THROWS(helper::camera_url((f.root/"BambuNetworkEngine.conf").string(),"user-test","US",
        (f.root/"resources").string(),(f.root/"data").string(),"printer-test|01.02|\"tutk\",\"agora\"",
        [&]{return f.marked("printer-polled");}));
    REQUIRE(f.marked("printer-polled"));
    REQUIRE_FALSE(f.marked("camera-requested"));
    REQUIRE_FALSE(f.marked("submitted"));
    REQUIRE(jobs_empty(f.root/"data/cloud-send-jobs"));
}
TEST_CASE("Camera requests can cancel during the SDK authorization callback", "[cloud-helper][camera]")
{
    Fixture f("camera-wait");
    const auto started=std::chrono::steady_clock::now();
    REQUIRE_THROWS_WITH(helper::camera_url((f.root/"BambuNetworkEngine.conf").string(),"user-test","US",
        (f.root/"resources").string(),(f.root/"data").string(),"printer-test|01.02|\"tutk\",\"agora\"",
        [&]{return f.marked("camera-requested");}),Catch::Matchers::Contains("canceled"));
    REQUIRE(std::chrono::steady_clock::now()-started<std::chrono::seconds(2));
    REQUIRE_FALSE(f.marked("submitted"));
    REQUIRE(jobs_empty(f.root/"data/cloud-send-jobs"));
}
TEST_CASE("Camera authorization accepts the cloud Agora path and refuses invalid requests early", "[cloud-helper][camera]")
{
    Fixture f("camera-agora");
    REQUIRE(helper::camera_url((f.root/"BambuNetworkEngine.conf").string(),"user-test","US",
        (f.root/"resources").string(),(f.root/"data").string(),"printer-test|01.02|\"tutk\",\"agora\"") == "bambu:///camera/agora?token=SECRET_TOKEN");
    REQUIRE_FALSE(f.marked("submitted"));
    for(const char* request:{"","|firmware","printer-test","printer-test|bad\ninput"}) {
        REQUIRE_THROWS_WITH(helper::camera_url((f.root/"BambuNetworkEngine.conf").string(),"user-test","US",
            (f.root/"resources").string(),(f.root/"data").string(),request),Catch::Matchers::Contains("invalid"));
    }
}
TEST_CASE("Cloud helper prepares the selected printer before its single submission", "[cloud-helper][printer-preparation]")
{
    Fixture f("late-report");
    REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == 0);
    REQUIRE(f.marked("account-checked"));
    REQUIRE(f.marked("printer-report-delivered"));
    REQUIRE(read_file(f.root / "submission-count") == "1");
    REQUIRE(jobs_empty(f.root / "data/cloud-send-jobs"));
}
TEST_CASE("Cloud helper never submits to a missing or offline account printer", "[cloud-helper][printer-preparation]")
{
    for (const char *mode : {"missing-printer", "offline-printer"}) {
        Fixture f(mode);
        REQUIRE_THROWS_AS(f.send(), std::runtime_error);
        REQUIRE_FALSE(f.marked("submitted"));
        REQUIRE(f.marked("cleaned"));
    }
}
TEST_CASE("Cloud helper never submits after preparation authentication or subscription failure", "[cloud-helper][printer-preparation]")
{
    for (const char *mode : {"expired-login", "auth-rejected", "subscription-failed"}) {
        Fixture f(mode);
        REQUIRE_THROWS_AS(f.send(), std::runtime_error);
        REQUIRE_FALSE(f.marked("submitted"));
        REQUIRE(f.marked("cleaned"));
    }
}
TEST_CASE("Cloud helper cannot treat unrelated messages as printer readiness", "[cloud-helper][printer-preparation]")
{
    for (const char *mode : {"wrong-printer", "wrong-agent", "invalid-report", "non-printer-report", "heartbeat", "empty-print-report"}) {
        Fixture f(mode);
        REQUIRE(f.send({}, [&] { return f.marked("invalid-report-delivered"); }) == BAMBU_NETWORK_ERR_CANCELED);
        REQUIRE_FALSE(f.marked("submitted"));
        REQUIRE(f.marked("cleaned"));
    }
}
TEST_CASE("Cloud helper cancellation stops cloud and printer preparation", "[cloud-helper][printer-preparation]")
{
    for (const char *mode : {"connection-timeout", "cancel-printer"}) {
        Fixture    f(mode);
        const auto started = std::chrono::steady_clock::now();
        REQUIRE(f.send({}, [&] { return f.marked("cloud-polled") && (std::string(mode) == "connection-timeout" || f.marked("printer-polled")); }) == BAMBU_NETWORK_ERR_CANCELED);
        REQUIRE(std::chrono::steady_clock::now() - started < std::chrono::seconds(2));
        REQUIRE_FALSE(f.marked("submitted"));
        REQUIRE(f.marked("cleaned"));
    }
}
TEST_CASE("Cloud helper bounds missing printer reports without submitting", "[cloud-helper][printer-preparation]")
{
    Fixture    f("no-report");
    const auto started = std::chrono::steady_clock::now();
    REQUIRE_THROWS_WITH(f.send(), Catch::Matchers::Contains("printer information"));
    REQUIRE(std::chrono::steady_clock::now() - started < std::chrono::seconds(40));
    REQUIRE_FALSE(f.marked("submitted"));
    REQUIRE(f.marked("cleaned"));
    REQUIRE(read_file(f.root / "status-request-count") == "3");
}
TEST_CASE("Cloud helper printer preflight uses status preparation without submitting", "[cloud-helper][printer-preparation]")
{
    Fixture f("verify-printer");
    REQUIRE_NOTHROW(
        helper::verify_printer((f.root / "BambuNetworkEngine.conf").string(), "user-test", "US", (f.root / "resources").string(), (f.root / "data").string(), "printer-test"));
    REQUIRE(f.marked("account-checked"));
    REQUIRE(f.marked("printer-report-delivered"));
    REQUIRE_FALSE(f.marked("submitted"));
    REQUIRE(read_file(f.root / "status-request-count") == "1");
    REQUIRE(jobs_empty(f.root / "data/cloud-send-jobs"));
}
TEST_CASE("Cloud helper bounds certificate refresh without treating it as readiness", "[cloud-helper][printer-preparation]")
{
    Fixture f("certificate-refresh");
    REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == 0);
    REQUIRE(f.marked("printer-report-delivered"));
    REQUIRE(read_file(f.root / "certificate-calls") == "3");
    REQUIRE(read_file(f.root / "submission-count") == "1");
}
TEST_CASE("Cloud helper retries a status request while its printer route becomes ready", "[cloud-helper][printer-preparation]")
{
    Fixture f("query-delayed");
    REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == 0);
    REQUIRE(f.marked("printer-report-delivered"));
    REQUIRE(read_file(f.root / "status-request-count") == "2");
    REQUIRE(read_file(f.root / "submission-count") == "1");
}
TEST_CASE("Cloud helper stages verified runtime assets inside the private job workspace", "[cloud-helper]")
{
    Fixture f;
    REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == 0);
    REQUIRE(f.marked("staged-runtime"));
    const auto hashes = json::parse(read_file(f.root / "resources/cloud-helper/manifest.json")).at("hashes");
    for (auto it = hashes.begin(); it != hashes.end(); ++it) REQUIRE(digest(read_file(f.root / "resources/cloud-helper" / it.key())) == it.value().get<std::string>());
    REQUIRE(jobs_empty(f.root / "data/cloud-send-jobs"));
}
TEST_CASE("Cloud helper rechecks copied runtime assets before launching", "[cloud-helper]")
{
    Fixture f("tamper-copy");
    REQUIRE_THROWS_WITH(f.send(), Catch::Matchers::Contains("integrity check"));
    REQUIRE_FALSE(f.marked("run"));
    REQUIRE(jobs_empty(f.root / "data/cloud-send-jobs"));
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
TEST_CASE("Cloud helper records private bounded phase timing without SDK messages", "[cloud-helper][timing]")
{
    for (const char *mode : {"timing", "timing-storm"}) {
        Fixture f(mode);
        REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == 0);
        const auto path = f.root / "data/cloud-send-latest-timing.json";
        REQUIRE(fs::is_regular_file(path));
        struct stat st{};
        REQUIRE(stat(path.c_str(), &st) == 0);
        REQUIRE((st.st_mode & 0777) == 0600);
        const auto raw = read_file(path);
        for (const auto &private_value : {"SECRET_TOKEN", "printer-test", "user-test", "PRIVATE SDK MESSAGE", "source.3mf", f.root.c_str()})
            REQUIRE(raw.find(private_value) == std::string::npos);
        const auto report = json::parse(raw);
        REQUIRE(report.at("schema_version") == 1);
        REQUIRE(report.at("outcome") == "succeeded");
        REQUIRE(report.at("sdk_result") == 0);
        REQUIRE(report.at("markers").size() <= 64);
        int64_t                  previous = -1;
        std::vector<std::string> phases;
        unsigned                 stages = 0;
        for (const auto &marker : report.at("markers")) {
            const auto elapsed = marker.at("elapsed_ms").get<int64_t>();
            REQUIRE(elapsed >= previous);
            REQUIRE(elapsed <= report.at("elapsed_ms").get<int64_t>());
            previous = elapsed;
            phases.push_back(marker.at("phase").get<std::string>());
            if (phases.back() == "sdk_stage") ++stages;
        }
        if (std::string(mode) == "timing") {
            REQUIRE(stages == 1);
            REQUIRE(phases == std::vector<std::string>{"started", "runtime_verified", "runtime_ready", "files_staged", "account_ready", "printer_ready", "submission_started",
                                                       "sdk_stage", "completion_wait_started", "completion_wait_finished", "sdk_returned", "cleanup_finished"});
        } else {
            REQUIRE(report.at("omitted_markers").get<unsigned>() > 0);
        }
    }
}
TEST_CASE("Cloud helper records failed interrupted and cancelled attempts without changing their result", "[cloud-helper][timing]")
{
    for (const char *mode : {"sdk-error", "wrong-user", "cancel"}) {
        Fixture f(mode);
        if (std::string(mode) == "wrong-user")
            REQUIRE_THROWS(f.send());
        else
            REQUIRE(f.send(
                        {}, [&] { return std::string(mode) == "cancel" && f.marked("submitted"); }, [](int, std::string) { return true; }) ==
                    (std::string(mode) == "cancel" ? BAMBU_NETWORK_ERR_CANCELED : -42));
        const auto path = f.root / "data/cloud-send-latest-timing.json";
        REQUIRE(fs::is_regular_file(path));
        const auto report = json::parse(read_file(path));
        REQUIRE(report.at("outcome") == (std::string(mode) == "cancel" ? "cancelled" : std::string(mode) == "wrong-user" ? "interrupted" : "failed"));
        if (std::string(mode) == "wrong-user")
            REQUIRE(report.at("sdk_result").is_null());
        else
            REQUIRE(report.at("sdk_result") == (std::string(mode) == "cancel" ? BAMBU_NETWORK_ERR_CANCELED : -42));
    }
}
TEST_CASE("Cloud helper timing separates SDK return from a late completion callback", "[cloud-helper][timing]")
{
    Fixture f("early-result");
    REQUIRE(f.send({}, {}, [](int, std::string) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        return true;
    }) == 0);
    const auto               report = json::parse(read_file(f.root / "data/cloud-send-latest-timing.json"));
    std::vector<std::string> phases;
    for (const auto &marker : report.at("markers")) phases.push_back(marker.at("phase").get<std::string>());
    const auto sdk  = std::find(phases.begin(), phases.end(), "sdk_returned");
    const auto wait = std::find(phases.begin(), phases.end(), "completion_wait_finished");
    REQUIRE(sdk != phases.end());
    REQUIRE(wait != phases.end());
    REQUIRE(sdk < wait);
}
TEST_CASE("Cloud helper verification probes preserve the last actual send timing", "[cloud-helper][timing]")
{
    for (const char *mode : {"verify", "verify-printer"}) {
        Fixture    f(mode);
        const auto path = f.root / "data/cloud-send-latest-timing.json";
        write_file(path, "previous actual send");
        if (std::string(mode) == "verify")
            helper::verify_account((f.root / "BambuNetworkEngine.conf").string(), "user-test", "US", (f.root / "resources").string(), (f.root / "data").string());
        else
            helper::verify_printer((f.root / "BambuNetworkEngine.conf").string(), "user-test", "US", (f.root / "resources").string(), (f.root / "data").string(), "printer-test");
        REQUIRE(read_file(path) == "previous actual send");
    }
}
TEST_CASE("Cloud helper timing write failure cannot fail a successful send", "[cloud-helper][timing]")
{
    Fixture    f;
    const auto path = f.root / "data/cloud-send-latest-timing.json";
    fs::create_directories(path);
    REQUIRE(f.send({}, {}, [](int, std::string) { return true; }) == 0);
    REQUIRE(fs::is_directory(path));
    for (const auto &entry : fs::directory_iterator(f.root / "data")) REQUIRE(entry.path().filename().string().find(".cloud-send-timing-") != 0);
}
int main(int argc, char **argv)
{
    if (fs::path(argv[0]).filename() == "docker") return fake_docker(argc, argv);
    exe = fs::canonical(argv[0]).string();
    return Catch::Session().run(argc, argv);
}
