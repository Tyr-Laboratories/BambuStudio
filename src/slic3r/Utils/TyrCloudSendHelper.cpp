#include "TyrCloudSendHelper.hpp"
#include "TyrCloudSendRecovery.hpp"
#include "nlohmann/json.hpp"
#include <openssl/evp.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

namespace Slic3r::TyrCloudSendHelper {
namespace {
namespace fs                   = std::filesystem;
using Json                     = nlohmann::json;
using Clock                    = std::chrono::steady_clock;
using Seconds                  = std::chrono::seconds;
constexpr uint32_t frame_magic = 0x52424a50;
constexpr size_t   max_frame   = 16 * 1024 * 1024;
struct PreparationCancelled
{};
constexpr const char *transport_error = "The print helper stopped responding. Print sending could not be confirmed; check the printer before retrying.";

[[noreturn]] void fail(const char *message) { throw std::runtime_error(message); }
bool              was_cancelled(const BBL::WasCancelledFn &fn) { return fn && fn(); }
Json              parse_json(const std::string &text, const char *error)
{
    auto result = Json::parse(text, nullptr, false);
    if (result.is_discarded()) fail(error);
    return result;
}
std::string file_text(const fs::path &path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) fail("The bundled print helper is incomplete. Reinstall this build.");
    return {std::istreambuf_iterator<char>(file), {}};
}
bool hex_digest(const std::string &s)
{
    return s.size() == 64 && std::all_of(s.begin(), s.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}
std::string sha256(const fs::path &path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) fail("A bundled print helper file could not be read. Reinstall this build.");
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
    if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) fail("Print helper integrity checking failed.");
    std::array<char, 1024 * 1024> buffer;
    while (file) {
        file.read(buffer.data(), buffer.size());
        if (EVP_DigestUpdate(context.get(), buffer.data(), size_t(file.gcount())) != 1) fail("Print helper integrity checking failed.");
    }
    if (!file.eof()) fail("A bundled print helper file could not be read.");
    unsigned char bytes[EVP_MAX_MD_SIZE];
    unsigned int  size = 0;
    if (EVP_DigestFinal_ex(context.get(), bytes, &size) != 1) fail("Print helper integrity checking failed.");
    std::ostringstream out;
    for (unsigned int i = 0; i < size; ++i) out << std::hex << std::setfill('0') << std::setw(2) << int(bytes[i]);
    return out.str();
}
struct Runtime
{
    fs::path                        root;
    std::string                     image;
    fs::path                        archive;
    std::map<fs::path, std::string> mount_hashes;
};
Runtime verified_runtime(const std::string &resources)
{
    Runtime r;
    r.root       = fs::absolute(fs::path(resources) / "cloud-helper");
    const auto m = parse_json(file_text(r.root / "manifest.json"), "The bundled print helper manifest is invalid.");
    if (m.value("schema_version", 0) != 1 || m.value("protocol_version", 0) != 1 || m.value("host_patch_version", 0) != 1 || m.value("platform", "") != "linux/arm64" ||
        m.value("network_abi", "") != "02.05.02.58" || m.value("executable", "") != "/runtime/tyr_bambu_cloud_host" || m.value("image_file", "") != "image.tar")
        fail("The bundled print helper version is incompatible. Reinstall this build.");
    r.image = m.value("image_id", "");
    if (r.image.substr(0, 7) != "sha256:" || !hex_digest(r.image.substr(7))) fail("The bundled print helper image identity is invalid.");
    const auto &hashes = m.at("hashes");
    for (const char *required : {"image.tar", "pin.json", "plugin/libbambu_networking.so", "plugin/libBambuSource.so", "plugin/liblive555.so",
                                 "plugin/linux_payload_manifest.json", "certs/slicer_base64.cer"})
        if (!hashes.contains(required)) fail("The bundled print helper integrity manifest is incomplete.");
    if (!hashes.is_object()) fail("The bundled print helper integrity manifest is invalid.");
    for (auto it = hashes.begin(); it != hashes.end(); ++it) {
        const fs::path relative(it.key());
        if (relative.empty() || relative.is_absolute()) fail("The bundled print helper contains an invalid path.");
        auto full = r.root;
        for (const auto &component : relative) {
            if (component == "." || component == "..") fail("The bundled print helper contains an invalid path.");
            full /= component;
            if (fs::is_symlink(fs::symlink_status(full))) fail("The bundled print helper contains an unexpected symbolic link.");
        }
        const auto digest = it.value().get<std::string>();
        if (!hex_digest(digest) || !fs::is_regular_file(full) || sha256(full) != digest) fail("The bundled print helper failed its integrity check. Reinstall this build.");
        if (*relative.begin() == "plugin" || *relative.begin() == "certs") r.mount_hashes.emplace(relative, digest);
    }
    // The SDK loads files by name from these directories; reject unhashed extras.
    for (const char *directory : {"plugin", "certs"})
        for (const auto &file : fs::recursive_directory_iterator(r.root / directory))
            if (file.is_symlink() || (file.is_regular_file() && !hashes.contains(fs::relative(file.path(), r.root).generic_string())))
                fail("The bundled print helper contains an unverified file.");
    r.archive = r.root / "image.tar";
    return r;
}
std::string docker_executable()
{
    std::vector<fs::path> candidates;
    std::istringstream    paths(std::getenv("PATH") ? std::getenv("PATH") : "");
    for (std::string path; std::getline(paths, path, ':');)
        if (!path.empty() && fs::path(path).is_absolute()) candidates.emplace_back(fs::path(path) / "docker");
    for (const char *path : {"/usr/local/bin/docker", "/opt/homebrew/bin/docker", "/Applications/Docker.app/Contents/Resources/bin/docker"}) candidates.emplace_back(path);
    for (const auto &path : candidates)
        if (::access(path.c_str(), X_OK) == 0 && fs::is_regular_file(path)) return path.string();
    fail("Docker Desktop is required for cloud printing. Install and start Docker Desktop, then try again.");
}
void close_fd(int &fd)
{
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}
class Process
{
public:
    int   input = -1, output = -1;
    pid_t pid    = -1;
    int   status = 0;
    Process(const std::string &executable, const std::vector<std::string> &args)
    {
        int in[2] = {-1, -1}, out[2] = {-1, -1};
        if (::pipe(in) || ::pipe(out)) {
            for (auto fd : {in[0], in[1], out[0], out[1]})
                if (fd >= 0) ::close(fd);
            fail("Could not open the print helper communication channel.");
        }
        for (auto fd : {in[0], in[1], out[0], out[1]}) ::fcntl(fd, F_SETFD, FD_CLOEXEC);
        posix_spawn_file_actions_t actions;
        posix_spawn_file_actions_init(&actions);
        posix_spawn_file_actions_adddup2(&actions, in[0], STDIN_FILENO);
        posix_spawn_file_actions_adddup2(&actions, out[1], STDOUT_FILENO);
        posix_spawn_file_actions_addopen(&actions, STDERR_FILENO, "/dev/null", O_WRONLY, 0);
        for (auto fd : {in[0], in[1], out[0], out[1]}) posix_spawn_file_actions_addclose(&actions, fd);
        std::vector<std::string> words{executable, "--context", "desktop-linux"};
        words.insert(words.end(), args.begin(), args.end());
        std::vector<char *> argv;
        for (auto &word : words) argv.push_back(word.data());
        argv.push_back(nullptr);
        const int result = posix_spawn(&pid, executable.c_str(), &actions, nullptr, argv.data(), environ);
        posix_spawn_file_actions_destroy(&actions);
        ::close(in[0]);
        ::close(out[1]);
        input  = in[1];
        output = out[0];
        if (result != 0) {
            pid = -1;
            close_fd(input);
            close_fd(output);
            fail("Could not start Docker Desktop's print helper.");
        }
        ::fcntl(input, F_SETFL, ::fcntl(input, F_GETFL) | O_NONBLOCK);
        ::fcntl(output, F_SETFL, ::fcntl(output, F_GETFL) | O_NONBLOCK);
#ifdef F_SETNOSIGPIPE
        ::fcntl(input, F_SETNOSIGPIPE, 1);
#endif
    }
    ~Process() { stop(); }
    Process(const Process &) = delete;
    bool exited()
    {
        if (pid < 0) return true;
        const auto result = ::waitpid(pid, &status, WNOHANG);
        if (result == pid || (result < 0 && errno == ECHILD)) {
            pid = -1;
            return true;
        }
        return false;
    }
    void stop()
    {
        close_fd(input);
        close_fd(output);
        if (pid > 0) {
            ::kill(pid, SIGKILL);
            while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
            pid = -1;
        }
    }
    void transfer(void *bytes, size_t size, bool writing, Clock::time_point deadline, const BBL::WasCancelledFn &cancelled = {})
    {
        auto *p  = static_cast<char *>(bytes);
        int   fd = writing ? input : output;
        while (size) {
            if (was_cancelled(cancelled)) throw PreparationCancelled{};
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
            if (remaining <= 0) fail(transport_error);
            pollfd    poll_fd{fd, short(writing ? POLLOUT : POLLIN), 0};
            const int ready = ::poll(&poll_fd, 1, int(std::min<int64_t>(remaining, 100)));
            if (ready < 0 && errno == EINTR) continue;
            if (ready < 0) fail(transport_error);
            if (!ready) continue;
            const auto n = writing ? ::write(fd, p, size) : ::read(fd, p, size);
            if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            if (n <= 0) fail(transport_error);
            p += n;
            size -= size_t(n);
        }
    }
};
struct CommandResult
{
    int         code;
    std::string output;
};
CommandResult command(const std::string &docker, const std::vector<std::string> &args, Seconds timeout, const BBL::WasCancelledFn &cancelled = {})
{
    Process process(docker, args);
    close_fd(process.input);
    const auto  deadline = Clock::now() + timeout;
    std::string output;
    bool        eof = false;
    while (!eof || !process.exited()) {
        if (was_cancelled(cancelled)) throw PreparationCancelled{};
        if (Clock::now() > deadline) fail("Docker Desktop did not respond in time. Check that it is running.");
        pollfd p{process.output, POLLIN, 0};
        if (!eof && ::poll(&p, 1, 100) > 0) {
            char       buffer[8192];
            const auto n = ::read(process.output, buffer, sizeof(buffer));
            if (n == 0)
                eof = true;
            else if (n > 0) {
                if (output.size() + size_t(n) > max_frame) fail("Docker returned an unexpected response.");
                output.append(buffer, size_t(n));
            } else if (errno != EINTR && errno != EAGAIN)
                fail("Docker's output could not be read.");
        } else if (eof)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return {WIFEXITED(process.status) ? WEXITSTATUS(process.status) : -1, std::move(output)};
}
void verify_local_context(const std::string &docker, const BBL::WasCancelledFn &cancelled)
{
    const auto context = command(docker, {"context", "inspect", "desktop-linux", "--format", "{{json .Endpoints.docker}}"}, Seconds(10), cancelled);
    if (context.code != 0) fail("Docker Desktop's local context is unavailable. Start Docker Desktop and try again.");
    const auto endpoint = parse_json(context.output, "Docker Desktop returned an invalid local context.");
    if (endpoint.value("Host", "").find("unix:///") != 0) fail("Cloud printing requires Docker Desktop on this Mac. Remote Docker contexts are not allowed.");
}
void ensure_image(const std::string &docker, const Runtime &r, const BBL::WasCancelledFn &cancelled)
{
    auto inspect = command(docker, {"image", "inspect", "--format", "{{json .}}", r.image}, Seconds(15), cancelled);
    if (inspect.code != 0) {
        if (command(docker, {"load", "-i", r.archive.string()}, Seconds(180), cancelled).code != 0)
            fail("The bundled print runtime could not be imported. Start Docker Desktop and try again.");
        inspect = command(docker, {"image", "inspect", "--format", "{{json .}}", r.image}, Seconds(15), cancelled);
    }
    if (inspect.code != 0) fail("Docker could not find the bundled print runtime after importing it.");
    const auto image = parse_json(inspect.output, "Docker returned an invalid image identity.");
    if (image.value("Id", "") != r.image || image.value("Os", "") != "linux" || image.value("Architecture", "") != "arm64")
        fail("The print runtime image does not match this application.");
}
class Rpc
{
    BBL::WasCancelledFn      preparation_cancel;
    Process                 &process;
    uint32_t                 next = 1;
    std::map<uint32_t, Json> replies;
    static void              put(std::array<unsigned char, 16> &h, size_t offset, uint32_t v)
    {
        for (unsigned i = 0; i < 4; ++i) h[offset + i] = static_cast<unsigned char>(v >> (i * 8));
    }
    static uint32_t get(const std::array<unsigned char, 16> &h, size_t offset)
    {
        uint32_t v = 0;
        for (unsigned i = 0; i < 4; ++i) v |= uint32_t(h[offset + i]) << (i * 8);
        return v;
    }

public:
    explicit Rpc(Process &child, BBL::WasCancelledFn cancelled) : preparation_cancel(std::move(cancelled)), process(child) {}
    void     preparation_finished() { preparation_cancel = {}; }
    uint32_t request(const std::string &method, const Json &payload)
    {
        std::string body = Json{{"method", method}, {"payload", payload}}.dump();
        if (body.size() > max_frame) fail("The print helper request is too large.");
        const uint32_t                id = next++;
        std::array<unsigned char, 16> header{};
        put(header, 0, frame_magic);
        put(header, 4, 1);
        put(header, 8, id);
        put(header, 12, uint32_t(body.size()));
        const auto deadline = Clock::now() + Seconds(15);
        process.transfer(header.data(), header.size(), true, deadline, preparation_cancel);
        process.transfer(body.data(), body.size(), true, deadline, preparation_cancel);
        return id;
    }
    bool has(uint32_t id) const { return replies.count(id) != 0; }
    Json take(uint32_t id)
    {
        auto value = std::move(replies.at(id));
        replies.erase(id);
        if (!value.is_object() || !value.value("ok", false)) fail("The print helper rejected a request. Check the account and helper installation.");
        return value;
    }
    Json response(uint32_t id, Seconds timeout = Seconds(15))
    {
        const auto deadline = Clock::now() + timeout;
        while (!has(id)) {
            std::array<unsigned char, 16> h{};
            process.transfer(h.data(), h.size(), false, deadline, preparation_cancel);
            const uint32_t length = get(h, 12), reply_id = get(h, 8);
            if (get(h, 0) != frame_magic || get(h, 4) != 2 || length > max_frame || reply_id == 0 || reply_id >= next || has(reply_id)) fail(transport_error);
            std::string body(length, '\0');
            if (length) process.transfer(body.data(), body.size(), false, deadline, preparation_cancel);
            replies.emplace(reply_id, parse_json(body, transport_error));
            if (replies.size() > 16) fail(transport_error);
        }
        return take(id);
    }
    Json call(const std::string &method, const Json &payload = Json::object(), Seconds timeout = Seconds(15)) { return response(request(method, payload), timeout); }
};
void private_directory(const fs::path &path)
{
    fs::create_directories(path);
    if (fs::is_symlink(fs::symlink_status(path))) fail("The private print staging directory is invalid.");
    fs::permissions(path, fs::perms::owner_all, fs::perm_options::replace);
}
std::string csv_field(std::string value)
{
    if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
    std::string result = "\"";
    for (char c : value) {
        if (c == '\"') result += '\"';
        result += c;
    }
    return result + '\"';
}
std::string mount(const fs::path &from, const char *to, bool readonly) { return "type=bind," + csv_field("src=" + from.string()) + ",dst=" + to + (readonly ? ",readonly" : ""); }
bool        stop_container(const std::string &docker, const std::string &name)
{
    try {
        if (command(docker, {"rm", "--force", name}, Seconds(10)).code == 0) return true;
        const auto found = command(docker, {"container", "ls", "-a", "--filter", "name=^/" + name + "$", "--format", "{{.Names}}"}, Seconds(10));
        return found.code == 0 && found.output.find(name) == std::string::npos;
    } catch (...) {
        return false;
    }
}
class Job
{
    std::string                                   docker;
    std::unique_ptr<TyrCloudSendRecovery::Record> recovery;
    bool                                          container_attempted = false, finished = false;

public:
    fs::path    root;
    std::string name;
    Job(const std::string &data, const std::string &executable, const std::string &target) : docker(executable)
    {
        const fs::path parent = fs::absolute(fs::path(data) / "cloud-send-jobs");
        private_directory(parent);
        TyrCloudSendRecovery::Record::DirectoryGuard guard(parent);
        std::string                                  pattern = (parent / "job-XXXXXX").string();
        std::vector<char>                            bytes(pattern.begin(), pattern.end());
        bytes.push_back(0);
        char *path = ::mkdtemp(bytes.data());
        if (!path) fail("Could not create a private directory for this print.");
        root     = path;
        name     = "tyr-cloud-send-" + std::to_string(::getpid()) + "-" + root.filename().string().substr(4);
        recovery = std::make_unique<TyrCloudSendRecovery::Record>(root, name, target);
        private_directory(root / "payload");
        private_directory(root / "session/logs");
    }
    ~Job()
    {
        if (!finished) {
            try {
                finish();
            } catch (...) { /* Retain staging if container cleanup cannot be confirmed. */
            }
        }
    }
    void stage(const std::string &original, const char *filename)
    {
        if (!fs::is_regular_file(original)) fail("The sliced print file is missing. Slice the plate again before sending.");
        fs::copy_file(original, root / "payload" / filename, fs::copy_options::none);
        fs::permissions(root / "payload" / filename, fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::replace);
    }
    void stage_runtime(const Runtime &runtime)
    {
        // Docker Desktop may not share /Applications with its VM. Stage only
        // verified SDK assets beside the job, where the account and payload
        // mounts already reside, without changing the user's shared folders.
        private_directory(root / "runtime");
        for (const auto &[relative, digest] : runtime.mount_hashes) {
            auto directory = root / "runtime";
            for (const auto &component : relative.parent_path()) {
                directory /= component;
                private_directory(directory);
            }
            const auto destination = root / "runtime" / relative;
            fs::copy_file(runtime.root / relative, destination, fs::copy_options::none);
            fs::permissions(destination, fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::replace);
            if (sha256(destination) != digest) fail("The staged print helper failed its integrity check. Reinstall this build.");
        }
    }
    void                     submitting() { recovery->set_phase("submitting"); }
    std::vector<std::string> arguments(const Runtime &r)
    {
        recovery->set_phase("running");
        container_attempted = true;
        return {"run",
                "--rm",
                "-i",
                "--platform",
                "linux/arm64",
                "--read-only",
                "--cap-drop",
                "ALL",
                "--security-opt",
                "no-new-privileges",
                "--pids-limit",
                "256",
                "--ulimit",
                "core=0",
                "--tmpfs",
                "/tmp:rw,noexec,nosuid,size=128m",
                "--name",
                name,
                "--mount",
                mount(root / "payload", "/job", true),
                "--mount",
                mount(root / "session", "/session", false),
                "--mount",
                mount(root / "runtime/plugin", "/plugin", true),
                "--mount",
                mount(root / "runtime/certs", "/certs", true),
                r.image};
    }
    void finish()
    {
        if (finished) return;
        if (container_attempted && !stop_container(docker, name)) {
            finished = true;
            fail("The print helper could not be stopped. Check the printer and restart Docker Desktop before retrying. Private staging was retained until the helper can be "
                 "stopped.");
        }
        bool removed = false;
        try {
            TyrCloudSendRecovery::Record::DirectoryGuard guard(root.parent_path());
            removed = TyrCloudSendRecovery::detail::remove_stopped_job(root);
        } catch (...) {
            // The directory guard's preparation diagnostic does not describe
            // an upload that may already have been accepted by the printer.
        }
        if (!removed) {
            finished = true;
            fail("The print helper stopped, but its private files could not be removed. Check the printer before retrying. Its recovery record was retained.");
        }
        finished = true;
    }
};
Json print_parameters(const BBL::PrintParams &p)
{
    Json j = Json::object();
#define COPY(name) j[#name] = p.name
    COPY(dev_id);
    COPY(task_name);
    COPY(project_name);
    COPY(preset_name);
    COPY(plate_index);
    COPY(ftp_folder);
    COPY(ftp_file);
    COPY(ftp_file_md5);
    COPY(nozzle_mapping);
    COPY(ams_mapping);
    COPY(ams_mapping2);
    COPY(ams_mapping_info);
    COPY(nozzles_info);
    COPY(connection_type);
    COPY(comments);
    COPY(origin_profile_id);
    COPY(stl_design_id);
    COPY(origin_model_id);
    COPY(print_type);
    COPY(dst_file);
    COPY(dev_name);
    COPY(dev_ip);
    COPY(use_ssl_for_ftp);
    COPY(use_ssl_for_mqtt);
    COPY(username);
    COPY(password);
    COPY(task_bed_leveling);
    COPY(task_flow_cali);
    COPY(task_vibration_cali);
    COPY(task_layer_inspect);
    COPY(task_record_timelapse);
    COPY(task_use_ams);
    COPY(task_bed_type);
    COPY(extra_options);
    COPY(auto_bed_leveling);
    COPY(auto_flow_cali);
    COPY(auto_offset_cali);
    COPY(extruder_cali_manual_mode);
    COPY(task_ext_change_assist);
    COPY(try_emmc_print);
#undef COPY
    // This helper is exclusively the cloud transport. The native dialog may
    // provide LAN acceleration fields; do not copy that route or its access code.
    j["connection_type"] = "cloud";
    j["comments"]        = "no_ip";
    j["dev_ip"]          = "";
    j["username"]        = "";
    j["password"]        = "";
    j["filename"]        = "/job/plate.gcode.3mf";
    j["config_filename"] = p.config_filename.empty() ? "" : "/job/config.3mf";
    return j;
}
void prepare_printer(Rpc &rpc, const Json &agent, const std::string &target, const BBL::WasCancelledFn &cancelled)
{
    if (target.empty()) fail("Select a printer before sending.");
    auto net = [&](const char *method, Json payload = Json::object()) {
        payload["agent"] = agent;
        return rpc.call(std::string("net.") + method, payload);
    };
    auto check = [&](const char *method, Json payload = Json::object()) {
        if (net(method, payload).value("value", -1) != 0)
            fail("The print helper could not prepare the selected printer. No print was submitted. Check the connection and try again.");
    };
    auto is_target_topic = [&](const std::string &topic) {
        // SDK callbacks use either the device ID or an MQTT/tunnel topic.
        // Match a complete component, never another printer's ID prefix.
        std::istringstream parts(topic);
        for (std::string part; std::getline(parts, part, '/');)
            if (part == target) return true;
        return false;
    };
    bool     cloud_connected = false, target_started = false, report_received = false;
    unsigned certificate_calls   = 0;
    auto     install_certificate = [&] {
        if (certificate_calls < 3) {
            check("install_device_cert", {{"dev_id", target}, {"lan_only", false}});
            ++certificate_calls;
        }
    };
    auto poll = [&] {
        if (was_cancelled(cancelled)) throw PreparationCancelled{};
        bool       refresh_certificate = false;
        const auto events              = rpc.call("bridge.poll_events", {{"limit", 256}}).value("events", Json::array());
        for (const auto &event : events) {
            if (event.value("agent", Json()) != agent) continue;
            const auto name    = event.value("name", "");
            const auto payload = event.value("payload", Json::object());
            if (name == "on_user_login" && !payload.value("login", false))
                fail("The Bambu account session expired while preparing the printer. Sign in again; no print was submitted.");
            if (name == "on_http_error" && (payload.value("http_code", 0) == 401 || payload.value("http_code", 0) == 403))
                fail("The cloud service rejected the account session. Sign in again; no print was submitted.");
            if (name == "on_server_connected") {
                if (payload.value("return_code", -1) != 0) fail("The cloud connection was rejected while preparing the printer. No print was submitted.");
                cloud_connected = true;
            }
            if (target_started && name == "on_subscribe_failure" && is_target_topic(payload.value("topic", "")))
                fail("The selected printer's status subscription failed. No print was submitted. Check the printer connection and try again.");
            if (target_started && name == "on_printer_connected" && is_target_topic(payload.value("topic_str", ""))) refresh_certificate = true;
            if (target_started && name == "on_message") {
                const auto device  = payload.value("dev_id", "");
                const auto message = payload.value("msg", "");
                if ((device.empty() || device == target) && message == "wait_info") refresh_certificate = true;
                if (device != target) continue;
                const auto parsed = Json::parse(message, nullptr, false);
                if (!parsed.is_object() || !parsed.contains("print") || !parsed["print"].is_object()) continue;
                const auto &print = parsed["print"];
                if (print.value("command", Json()) == "push_status" && print.contains("gcode_state") && print["gcode_state"].is_string() &&
                    !print["gcode_state"].get_ref<const std::string &>().empty())
                    report_received = true;
            }
        }
        if (refresh_certificate) install_certificate();
    };

    check("set_user_selected_machine", {{"dev_id", ""}});
    check("connect_server");
    const auto cloud_deadline = Clock::now() + Seconds(15);
    while (!cloud_connected) {
        poll();
        if (!cloud_connected) cloud_connected = net("is_server_connected").value("value", false);
        if (cloud_connected) break;
        if (Clock::now() >= cloud_deadline) fail("The cloud connection did not become ready. No print was submitted. Check the connection and try again.");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    const auto account = net("get_user_print_info");
    const auto http    = account.value("http_code", 0);
    if (account.value("value", -1) != 0 || http < 200 || http >= 300)
        fail("The cloud service did not accept the current account session. Sign in again; no print was submitted.");
    const auto inventory = Json::parse(account.value("http_body", ""), nullptr, false);
    if (!inventory.is_object() || !inventory.contains("devices") || !inventory["devices"].is_array())
        fail("The cloud service returned invalid printer information. No print was submitted.");
    bool found = false;
    for (const auto &device : inventory["devices"]) {
        if (!device.is_object() || device.value("dev_id", Json()) != target) continue;
        found = true;
        if (device.value("dev_online", Json()) != true) fail("The selected printer is offline. No print was submitted. Reconnect the printer and try again.");
    }
    if (!found) fail("The selected printer is not available in this account. No print was submitted.");
    check("enable_multi_machine", {{"enable", true}});
    check("start_subscribe", {{"module", "tunnel"}});
    target_started = true;
    check("add_subscribe", {{"devs", Json::array({target})}});
    check("set_user_selected_machine", {{"dev_id", target}});
    install_certificate();

    unsigned   status_requests     = 0;
    bool       status_requested    = false;
    unsigned   sequence            = 20000;
    auto       next_status_request = Clock::now();
    const auto printer_deadline    = Clock::now() + Seconds(30);
    while (!report_received) {
        if (was_cancelled(cancelled)) throw PreparationCancelled{};
        if (status_requests < 3 && Clock::now() >= next_status_request) {
            // Fixed read-only status request. Never send print/control commands
            // during preparation or a verify_printer probe.
            const Json query = {{"pushing", {{"sequence_id", std::to_string(++sequence)}, {"command", "pushall"}, {"version", 1}, {"push_target", 1}}}};
            // Selection/subscription is asynchronous. A read-only query may
            // be rejected until the route is ready; keep the bounded refreshes
            // while authentication and subscription errors still fail in poll.
            status_requested = net("send_message", {{"dev_id", target}, {"qos", 0}, {"flag", 0}, {"msg", query.dump()}}).value("value", -1) == 0 || status_requested;
            ++status_requests;
            next_status_request = Clock::now() + Seconds(5);
        }
        poll();
        if (report_received) break;
        if (Clock::now() >= printer_deadline) {
            if (!status_requested) fail("The selected printer status request could not be sent. No print was submitted. Check the printer connection and try again.");
            fail("The selected printer information did not become ready. No print was submitted. Check the printer connection and try again.");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (was_cancelled(cancelled)) throw PreparationCancelled{};
}
enum class RunMode { Send, VerifyAccount, VerifyPrinter };
int run_send(const BBL::PrintParams &params,
             const std::string      &session_file,
             const std::string      &expected_user_id,
             const std::string      &country,
             const std::string      &resources,
             const std::string      &data,
             BBL::OnUpdateStatusFn   update,
             BBL::WasCancelledFn     cancelled,
             BBL::OnWaitFn           wait,
             RunMode                 mode = RunMode::Send)
{
    const bool verify_only = mode != RunMode::Send;
    if (was_cancelled(cancelled)) return BAMBU_NETWORK_ERR_CANCELED;
    if (expected_user_id.empty()) fail("Sign in before sending a cloud print.");
    if ((params.task_record_timelapse && params.task_timelapse_use_internal) || !params.queue_plate_id.empty())
        fail("This cloud helper does not support internal-storage timelapse or queued print jobs. Disable that option for this print.");
    if (!fs::is_regular_file(session_file)) fail("The saved Bambu account session is missing. Sign in again before sending.");
    const Runtime     runtime = verified_runtime(resources);
    const std::string docker  = docker_executable();
    verify_local_context(docker, cancelled);
    if (!verify_only) {
        const auto parent = fs::absolute(fs::path(data) / "cloud-send-jobs");
        private_directory(parent);
        if (TyrCloudSendRecovery::Record::reconcile(parent, params.dev_id, [&](const std::string &name) { return stop_container(docker, name); }))
            fail("A previous interrupted send may already have reached its printer. Check the previous printer before retrying this print.");
    }
    ensure_image(docker, runtime, cancelled);
    if (was_cancelled(cancelled)) return BAMBU_NETWORK_ERR_CANCELED;
    Job job(data, docker, params.dev_id);
    job.stage_runtime(runtime);
    fs::copy_file(session_file, job.root / "session/BambuNetworkEngine.conf", fs::copy_options::none);
    fs::permissions(job.root / "session/BambuNetworkEngine.conf", fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::replace);
    if (!verify_only) {
        job.stage(params.filename, "plate.gcode.3mf");
        if (!params.config_filename.empty()) job.stage(params.config_filename, "config.3mf");
    }
    // Keep the callback future outside the try: an exception must stop the
    // container before joining a callback that still observes native telemetry.
    std::future<bool> waiting;
    try {
        Process    process(docker, job.arguments(runtime));
        Rpc        rpc(process, cancelled);
        const auto h    = rpc.call("bridge.handshake");
        const auto caps = h.value("auth_capabilities", Json::object());
        if (h.value("protocol_version", 0) != 1 || h.value("network_abi_version", "") != "02.05.02.58" || caps.value("tyr_cloud_helper_version", 0) != 1 ||
            caps.value("tyr_redaction_version", 0) != 1 || caps.value("tyr_wait_reply_version", 0) != 1 || !caps.value("bambu_network_start_print", false))
            fail("The print helper does not have the required compatibility and privacy support.");
        auto print = print_parameters(params);
        if (h.contains("supported_print_params"))
            for (auto it = print.begin(); it != print.end(); ++it)
                if (std::find(h["supported_print_params"].begin(), h["supported_print_params"].end(), Json(it.key())) == h["supported_print_params"].end())
                    fail("The print helper does not support this application's print parameters.");
        auto agent = rpc.call("net.create_agent", {{"log_dir", "/session/logs"}, {"country_code", country}}).at("value");
        auto net   = [&](const char *method, Json payload = Json::object()) {
            payload["agent"] = agent;
            return rpc.call(std::string("net.") + method, payload);
        };
        auto check = [&](const char *method, Json payload = Json::object()) {
            if (net(method, payload).value("value", -1) != 0) fail("The print helper could not initialize the current account. Check your sign-in and try again.");
        };
        check("set_config_dir", {{"config_dir", "/session"}});
        check("init_log");
        check("set_cert_file", {{"folder", "/certs"}, {"filename", "slicer_base64.cer"}});
        check("set_extra_http_header", {{"headers",
                                         {{"X-BBL-Client-Type", "slicer"},
                                          {"X-BBL-Client-Name", "BambuStudio"},
                                          {"X-BBL-Client-Version", "02.05.02.51"},
                                          {"X-BBL-OS-Type", "linux"},
                                          {"X-BBL-Language", "en"},
                                          {"X-BBL-Device-ID", job.name}}}});
        for (const char *method : {"set_queue_on_main_fn", "set_on_user_login_fn", "set_on_server_connected_fn", "set_on_http_error_fn", "set_on_message_fn",
                                   "set_on_printer_connected_fn", "set_on_subscribe_failure_fn"})
            check(method);
        check("set_country_code", {{"country_code", country}});
        check("start");
        if (!net("is_user_login").value("value", false) || net("get_user_id").value("value", "") != expected_user_id)
            fail("The print helper account does not match Bambu Studio. Sign out and sign in again before sending.");
        if (mode == RunMode::VerifyAccount) {
            check("connect_server");
            const auto account = net("get_user_print_info");
            const int  http    = account.value("http_code", 0);
            if (account.value("value", -1) != 0 || http < 200 || http >= 300) fail("The cloud service did not accept the current account session. Sign in again before sending.");
            job.finish();
            return 0;
        }
        prepare_printer(rpc, agent, params.dev_id, cancelled);
        if (mode == RunMode::VerifyPrinter) {
            job.finish();
            return 0;
        }
        if (was_cancelled(cancelled)) {
            job.finish();
            return BAMBU_NETWORK_ERR_CANCELED;
        }
        rpc.preparation_finished();
        job.submitting();
        constexpr int64_t job_id          = 1;
        const auto        request         = rpc.request("net.start_print", {{"agent", agent}, {"client_job_id", job_id}, {"params", print}});
        bool              cancel_sent     = false;
        auto              cancel_deadline = Clock::time_point::max();
        const auto        job_deadline    = Clock::now() + Seconds(600);
        int64_t           wait_request    = 0;
        auto              poll_events     = [&] {
            const auto events = rpc.call("bridge.poll_events", {{"limit", 256}}).value("events", Json::array());
            for (const auto &e : events) {
                const auto p = e.value("payload", Json::object());
                if (p.value("job_id", int64_t(0)) != job_id) continue;
                const std::string name = e.value("name", "");
                if (name == "job.update_status" && update)
                    update(p.at("status").get<int>(), p.at("code").get<int>(), p.value("msg", ""));
                else if (name == "job.wait") {
                    if (waiting.valid()) fail("The print helper requested overlapping completion checks.");
                    wait_request             = p.at("request_id").get<int64_t>();
                    const int         status = p.at("status").get<int>();
                    const std::string info   = p.value("job_info", "");
                    waiting = std::async(std::launch::async, [wait, status, info, cancel_sent] { return !cancel_sent && wait && wait(status, info); });
                }
            }
            return events.size();
        };
        while (!rpc.has(request)) {
            if (was_cancelled(cancelled) && !cancel_sent) {
                rpc.call("bridge.job_cancel", {{"job_id", job_id}, {"cancel", true}});
                cancel_sent     = true;
                cancel_deadline = Clock::now() + Seconds(15);
            }
            if (Clock::now() > job_deadline || Clock::now() > cancel_deadline) fail(transport_error);
            poll_events();
            if (waiting.valid() && waiting.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                const bool reply = waiting.get();
                rpc.call("bridge.job_wait_reply", {{"job_id", job_id}, {"request_id", wait_request}, {"reply", !cancel_sent && reply}});
            }
            if (!rpc.has(request)) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        const int result = rpc.take(request).at("value").get<int>();
        // SDK callbacks precede its return, but may still be queued in the host.
        for (unsigned i = 0; i < 32 && poll_events() == 256; ++i) {}
        if (waiting.valid()) (void) waiting.get();
        job.finish();
        return result;
    } catch (...) {
        job.finish();
        throw;
    }
}
} // namespace
bool available(const std::string &resources_dir)
{
    std::error_code error;
    return fs::is_regular_file(fs::path(resources_dir) / "cloud-helper/manifest.json", error);
}
int send(const BBL::PrintParams &params,
         const std::string      &session_file,
         const std::string      &expected_user_id,
         const std::string      &country,
         const std::string      &resources_dir,
         const std::string      &data_dir,
         BBL::OnUpdateStatusFn   update,
         BBL::WasCancelledFn     cancelled,
         BBL::OnWaitFn           wait)
{
    try {
        return run_send(params, session_file, expected_user_id, country, resources_dir, data_dir, std::move(update), std::move(cancelled), std::move(wait));
    } catch (const PreparationCancelled &) {
        return BAMBU_NETWORK_ERR_CANCELED;
    } catch (const Json::exception &) {
        fail("The print helper returned an incompatible response. Check the printer before retrying.");
    } catch (const fs::filesystem_error &) {
        fail("The print helper could not access its private files. Check disk space and permissions, and check the printer before retrying.");
    }
}
void verify_account(const std::string &session_file, const std::string &expected_user_id, const std::string &country, const std::string &resources_dir, const std::string &data_dir)
{
    try {
        (void) run_send(BBL::PrintParams{}, session_file, expected_user_id, country, resources_dir, data_dir, {}, {}, {}, RunMode::VerifyAccount);
    } catch (const Json::exception &) {
        fail("The print helper returned an incompatible account response.");
    } catch (const fs::filesystem_error &) {
        fail("The print helper could not access its private account files.");
    }
}
void verify_printer(const std::string &session_file,
                    const std::string &expected_user_id,
                    const std::string &country,
                    const std::string &resources_dir,
                    const std::string &data_dir,
                    const std::string &printer_id)
{
    BBL::PrintParams params{};
    params.dev_id = printer_id;
    try {
        (void) run_send(params, session_file, expected_user_id, country, resources_dir, data_dir, {}, {}, {}, RunMode::VerifyPrinter);
    } catch (const Json::exception &) {
        fail("The print helper returned incompatible printer information. No print was submitted.");
    } catch (const fs::filesystem_error &) {
        fail("The print helper could not access its private account files. No print was submitted.");
    }
}
} // namespace Slic3r::TyrCloudSendHelper
