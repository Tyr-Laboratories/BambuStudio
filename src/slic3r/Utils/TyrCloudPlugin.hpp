#pragma once

// The public app ships the open helper only. Bambu's separately licensed Linux
// plugin is obtained from its pinned vendor URL into this user's private cache.
#include "nlohmann/json.hpp"
#include <openssl/evp.h>
#include <curl/curl.h>
#include <miniz.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace Slic3r::TyrCloudPlugin {
namespace fs = std::filesystem;
using Json = nlohmann::json;
using Cancelled = std::function<bool()>;
struct CancelledDownload {};
constexpr size_t max_archive_size = 50 * 1024 * 1024;
constexpr size_t max_payload_size = 100 * 1024 * 1024;
struct Spec {
    std::string url, archive_sha256, abi;
    std::map<std::string, std::string> payload_sha256, entries;
};
inline Spec official_spec()
{
    return {
        "https://public-cdn.bblmw.com/upgrade/studio/plugins/02.05.02.58/9fb586c207/linux_02.05.02.58.zip",
        "e7158dca80c0853520877229a18739e1b6f21b70a9c0f01db19af78a374287d3", "02.05.02.58",
        {{"libbambu_networking.so", "c9bdb8e6868c461d5c456dc7ba913744e449e2cc4ce2f29e175b381a1ff95d05"},
         {"libBambuSource.so", "506a202d225e9efb7e082b7920983bcadb93e85f0927d7b5b43c4dd7bf7d2f21"},
         {"liblive555.so", "1dd3be062635b56db4fe9a3e446935f7223de1d02c60c7958bd5fda4c92b5df1"}},
        {{"libbambu_networking.so", "libbambu_networking.so"}, {"libBambuSource.so", "libBambuSource.so"}, {"liblive555.so", "liblive555.so"}}};
}
[[noreturn]] inline void fail(const char *message) { throw std::runtime_error(message); }
inline void check_cancelled(const Cancelled &cancelled) { if (cancelled && cancelled()) throw CancelledDownload{}; }
inline Spec verified_pin(const Json &pin)
{
    const auto spec = official_spec();
    if (pin.value("plugin_url", "") != spec.url || pin.value("plugin_sha256", "") != spec.archive_sha256 ||
        pin.value("network_abi", "") != spec.abi || pin.value("network_sha256", "") != spec.payload_sha256.at("libbambu_networking.so") ||
        !pin.contains("payload_sha256") || pin["payload_sha256"] != Json(spec.payload_sha256) ||
        !pin.contains("plugin_entries") || pin["plugin_entries"] != Json(spec.entries))
        fail("The cloud plugin dependency pin is invalid. Reinstall this build.");
    return spec;
}
inline std::string sha256(const fs::path &path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) fail("A cloud helper file could not be read.");
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
    if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) fail("Cloud plugin integrity checking failed.");
    std::vector<char> buffer(64 * 1024);
    while (file) {
        file.read(buffer.data(), buffer.size());
        if (EVP_DigestUpdate(context.get(), buffer.data(), size_t(file.gcount())) != 1) fail("Cloud plugin integrity checking failed.");
    }
    if (!file.eof()) fail("A cloud helper file could not be read.");
    unsigned char bytes[EVP_MAX_MD_SIZE];
    unsigned size = 0;
    if (EVP_DigestFinal_ex(context.get(), bytes, &size) != 1) fail("Cloud plugin integrity checking failed.");
    std::ostringstream text;
    for (unsigned i = 0; i < size; ++i) text << std::hex << std::setfill('0') << std::setw(2) << int(bytes[i]);
    return text.str();
}
inline Json payload_manifest(const Spec &spec)
{
    Json files = Json::array();
    // Keep the SDK's established order independent of std::map key ordering.
    for (const auto *name : {"libbambu_networking.so", "libBambuSource.so", "liblive555.so"})
        files.push_back({{"name", name}, {"sha256", spec.payload_sha256.at(name)}, {"abi_version", spec.abi}});
    return {{"files", files}};
}
inline void private_directory(const fs::path &path)
{
    fs::create_directory(path);
    struct stat st{};
    if (::lstat(path.c_str(), &st) != 0 || !S_ISDIR(st.st_mode) || st.st_uid != ::geteuid())
        fail("The cloud plugin cache directory is invalid.");
    if (::chmod(path.c_str(), 0700) != 0) fail("The cloud plugin cache permissions could not be set.");
}
inline void write_private(const fs::path &path, const char *bytes, size_t size)
{
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (fd < 0) fail("A private cloud plugin file could not be created.");
    bool ok = true;
    while (size) {
        const auto count = ::write(fd, bytes, size);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) { ok = false; break; }
        bytes += count;
        size -= size_t(count);
    }
    if (::close(fd) != 0) ok = false;
    if (!ok) fail("The cloud plugin could not be saved. Check available disk space.");
}
inline bool cache_valid(const fs::path &path, const Spec &spec)
{
    struct stat root{};
    if (::lstat(path.c_str(), &root) != 0 || !S_ISDIR(root.st_mode) || root.st_uid != ::geteuid() || (root.st_mode & 0777) != 0700) return false;
    size_t count = 0;
    for (const auto &file : fs::directory_iterator(path)) {
        struct stat st{};
        if (::lstat(file.path().c_str(), &st) != 0 || !S_ISREG(st.st_mode) || st.st_uid != ::geteuid() || (st.st_mode & 0777) != 0600) return false;
        const auto name = file.path().filename().string();
        if (name == "linux_payload_manifest.json") {
            std::ifstream stream(file.path());
            const auto manifest = Json::parse(stream, nullptr, false);
            if (manifest.is_discarded() || manifest != payload_manifest(spec)) return false;
        } else if (const auto it = spec.payload_sha256.find(name); it == spec.payload_sha256.end() || sha256(file.path()) != it->second) return false;
        ++count;
    }
    return count == spec.payload_sha256.size() + 1;
}
inline void extract_archive(const fs::path &archive, const fs::path &target, const Spec &spec, const Cancelled &cancelled)
{
    check_cancelled(cancelled);
    if (fs::file_size(archive) > max_archive_size || sha256(archive) != spec.archive_sha256)
        fail("The downloaded Bambu cloud plugin failed its checksum check. Try again later.");
    struct Zip {
        mz_zip_archive value{};
        ~Zip() { if (value.m_pState) mz_zip_reader_end(&value); }
    } zip;
    if (!mz_zip_reader_init_file(&zip.value, archive.c_str(), 0)) fail("The Bambu cloud plugin archive is invalid.");
    std::set<std::string> names;
    std::map<std::string, mz_uint> selected;
    const auto count = mz_zip_reader_get_num_files(&zip.value);
    if (count > 1000) fail("The Bambu cloud plugin archive has an unexpected layout.");
    for (mz_uint i = 0; i < count; ++i) {
        check_cancelled(cancelled);
        mz_zip_archive_file_stat st{};
        const auto length = mz_zip_reader_get_filename(&zip.value, i, nullptr, 0);
        if (!length || length > 1024 || !mz_zip_reader_file_stat(&zip.value, i, &st)) fail("The Bambu cloud plugin archive is invalid.");
        std::vector<char> bytes(length);
        if (mz_zip_reader_get_filename(&zip.value, i, bytes.data(), length) != length) fail("The Bambu cloud plugin archive is invalid.");
        const std::string name(bytes.data(), length - 1);
        const fs::path relative(name);
        const auto mode = mode_t(st.m_external_attr >> 16);
        if (name.empty() || name.find('\0') != std::string::npos || name.find('\\') != std::string::npos || relative.is_absolute() || !names.insert(name).second ||
            std::any_of(relative.begin(), relative.end(), [](const fs::path &part) { return part == ".." || part == "."; }) ||
            ((mode & S_IFMT) != 0 && !S_ISREG(mode) && !S_ISDIR(mode)))
            fail("The Bambu cloud plugin archive contains an unsafe entry.");
        for (const auto &[basename, entry] : spec.entries) {
            if (relative.filename() == basename && name != entry) fail("The Bambu cloud plugin archive contains an unexpected duplicate payload.");
            if (name == entry) {
                if (st.m_is_directory || ((mode & S_IFMT) != 0 && !S_ISREG(mode)) || st.m_uncomp_size == 0 || st.m_uncomp_size > max_payload_size)
                    fail("The Bambu cloud plugin payload is invalid.");
                selected.emplace(basename, i);
            }
        }
    }
    if (selected.size() != spec.entries.size()) fail("The Bambu cloud plugin archive is missing a required library.");
    private_directory(target);
    for (const auto &[name, index] : selected) {
        check_cancelled(cancelled);
        mz_zip_archive_file_stat st{};
        if (!mz_zip_reader_file_stat(&zip.value, index, &st)) fail("The Bambu cloud plugin archive is invalid.");
        std::vector<char> payload(size_t(st.m_uncomp_size));
        if (!mz_zip_reader_extract_to_mem(&zip.value, index, payload.data(), payload.size(), 0)) fail("The Bambu cloud plugin archive could not be extracted.");
        write_private(target / name, payload.data(), payload.size());
        if (sha256(target / name) != spec.payload_sha256.at(name)) fail("A Bambu cloud plugin library failed its checksum check.");
    }
    const auto manifest = payload_manifest(spec).dump(2) + "\n";
    write_private(target / "linux_payload_manifest.json", manifest.data(), manifest.size());
    check_cancelled(cancelled);
}
// The fetch seam keeps archive/cache tests fully offline. Production always
// calls this through verified_pin() and the fixed HTTPS transport below.
using Fetch = std::function<void(const fs::path &, const Cancelled &)>;
inline fs::path acquire(const fs::path &data, const Spec &spec, const Fetch &fetch, const Cancelled &cancelled)
{
    check_cancelled(cancelled);
    fs::create_directories(data);
    const auto parent = data / "cloud-plugin-cache";
    private_directory(parent);
    struct Lock {
        int fd = -1;
        ~Lock() { if (fd >= 0) ::close(fd); }
    } lock;
    lock.fd = ::open((parent / ".lock").c_str(), O_CREAT | O_RDWR | O_NOFOLLOW | O_CLOEXEC, 0600);
    struct stat st{};
    if (lock.fd < 0 || ::fstat(lock.fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_uid != ::geteuid() || ::fchmod(lock.fd, 0600) != 0)
        fail("The cloud plugin cache could not be locked.");
    while (::flock(lock.fd, LOCK_EX | LOCK_NB) != 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN && errno != EINTR) fail("The cloud plugin cache could not be locked.");
        check_cancelled(cancelled);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    const auto destination = parent / spec.archive_sha256;
    if (cache_valid(destination, spec)) return destination;
    if (fs::exists(destination) || fs::is_symlink(fs::symlink_status(destination))) fs::remove_all(destination);
    std::string pattern = (parent / ".download-XXXXXX").string();
    if (!::mkdtemp(pattern.data())) fail("The cloud plugin download directory could not be created.");
    struct Temporary {
        fs::path path;
        ~Temporary() { std::error_code ignored; fs::remove_all(path, ignored); }
    } temporary{pattern};
    const auto archive = temporary.path / "plugin.zip", payload = temporary.path / "payload";
    fetch(archive, cancelled);
    check_cancelled(cancelled);
    extract_archive(archive, payload, spec, cancelled);
    if (!cache_valid(payload, spec)) fail("The downloaded cloud plugin failed validation.");
    fs::rename(payload, destination);
    return destination;
}
inline void download_official(const fs::path &destination, const fs::path &ca_file, const Cancelled &cancelled,
                              const std::function<void(int)> &progress = {})
{
    check_cancelled(cancelled);
    struct Download {
        int fd = -1;
        size_t size = 0;
        bool cancelled = false, failed = false;
        Cancelled cancel;
        std::function<void(int)> progress;
        int reported = -1;
        ~Download() { if (fd >= 0) ::close(fd); }
    } state;
    state.cancel = cancelled;
    state.progress = progress;
    state.fd = ::open(destination.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (state.fd < 0) fail("The Bambu cloud plugin download could not be saved.");
    static const int initialized = curl_global_init(CURL_GLOBAL_DEFAULT);
    if (initialized != CURLE_OK) fail("The Bambu cloud plugin downloader could not start.");
    std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> curl(curl_easy_init(), curl_easy_cleanup);
    if (!curl) fail("The Bambu cloud plugin downloader could not start.");
    const auto spec = official_spec();
    curl_easy_setopt(curl.get(), CURLOPT_URL, spec.url.c_str());
    curl_easy_setopt(curl.get(), CURLOPT_PROTOCOLS, long(CURLPROTO_HTTPS));
    curl_easy_setopt(curl.get(), CURLOPT_FOLLOWLOCATION, 0L);
    curl_easy_setopt(curl.get(), CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl.get(), CURLOPT_SSL_VERIFYHOST, 2L);
    curl_easy_setopt(curl.get(), CURLOPT_CAINFO, ca_file.c_str());
    curl_easy_setopt(curl.get(), CURLOPT_CONNECTTIMEOUT, 15L);
    curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT, 180L);
    curl_easy_setopt(curl.get(), CURLOPT_LOW_SPEED_LIMIT, 1024L);
    curl_easy_setopt(curl.get(), CURLOPT_LOW_SPEED_TIME, 30L);
    curl_easy_setopt(curl.get(), CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl.get(), CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(curl.get(), CURLOPT_MAXFILESIZE_LARGE, curl_off_t(max_archive_size));
    curl_easy_setopt(curl.get(), CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(curl.get(), CURLOPT_XFERINFODATA, &state);
    curl_easy_setopt(curl.get(), CURLOPT_XFERINFOFUNCTION, +[](void *p, curl_off_t total, curl_off_t now, curl_off_t, curl_off_t) -> int {
        auto &s = *static_cast<Download *>(p);
        try {
            if (s.cancel && s.cancel()) { s.cancelled = true; return 1; }
            if (total > curl_off_t(max_archive_size) || now > curl_off_t(max_archive_size)) { s.failed = true; return 1; }
            const int percent = total > 0 ? int(std::min<curl_off_t>(100, now * 100 / total)) : 0;
            if (s.progress && percent != s.reported) { s.progress(percent); s.reported = percent; }
        } catch (...) { s.failed = true; return 1; }
        return 0;
    });
    curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &state);
    curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, +[](char *bytes, size_t size, size_t count, void *p) -> size_t {
        auto &s = *static_cast<Download *>(p);
        if (size && count > (max_archive_size - s.size) / size) { s.failed = true; return 0; }
        const size_t length = size * count;
        size_t offset = 0;
        while (offset < length) {
            const auto wrote = ::write(s.fd, bytes + offset, length - offset);
            if (wrote < 0 && errno == EINTR) continue;
            if (wrote <= 0) { s.failed = true; return 0; }
            offset += size_t(wrote);
        }
        s.size += length;
        return length;
    });
    const auto result = curl_easy_perform(curl.get());
    long status = 0;
    curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &status);
    if (state.cancelled || (cancelled && cancelled())) throw CancelledDownload{};
    if (result != CURLE_OK || status != 200 || state.failed)
        fail("The Bambu cloud plugin could not be downloaded securely. Check your internet connection and try again. No print was sent.");
    const int fd = state.fd;
    state.fd = -1;
    if (::close(fd) != 0) fail("The Bambu cloud plugin download could not be saved. Check disk space.");
}
} // namespace Slic3r::TyrCloudPlugin
