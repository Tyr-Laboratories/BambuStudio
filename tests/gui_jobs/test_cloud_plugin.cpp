#include "catch2/catch.hpp"
#include "slic3r/Utils/TyrCloudPlugin.hpp"
#include <atomic>
#include <future>
#include <thread>
namespace plugin = Slic3r::TyrCloudPlugin;
namespace fs = std::filesystem;
namespace {
struct PluginFixture {
    fs::path root, archive, data;
    plugin::Spec spec;
    PluginFixture() {
        char path[] = "/tmp/tyr-cloud-plugin-test-XXXXXX";
        root = ::mkdtemp(path);
        archive = root / "plugin.zip";
        data = root / "data";
        spec = plugin::official_spec();
        for (auto &[name, hash] : spec.payload_sha256) {
            std::ofstream(root / name) << name;
            hash = plugin::sha256(root / name);
        }
        make_archive();
    }
    ~PluginFixture() { std::error_code ignored; fs::remove_all(root, ignored); }
    void make_archive(const std::string &extra = "", bool omit_video = false) {
        mz_zip_archive zip{};
        REQUIRE(mz_zip_writer_init_file(&zip, archive.c_str(), 0));
        for (const auto &[name, hash] : spec.payload_sha256) {
            if (omit_video && name == "liblive555.so") continue;
            REQUIRE(mz_zip_writer_add_mem(&zip, name.c_str(), name.data(), name.size(), MZ_DEFAULT_COMPRESSION));
        }
        if (!extra.empty()) REQUIRE(mz_zip_writer_add_mem(&zip, extra.c_str(), "extra", 5, MZ_DEFAULT_COMPRESSION));
        REQUIRE(mz_zip_writer_finalize_archive(&zip));
        REQUIRE(mz_zip_writer_end(&zip));
        spec.archive_sha256 = plugin::sha256(archive);
    }
    void rename_entry(const std::string &from, const std::string &to) {
        REQUIRE(from.size() == to.size());
        std::ifstream in(archive, std::ios::binary);
        std::string body{std::istreambuf_iterator<char>(in), {}};
        for (size_t pos = 0; (pos = body.find(from, pos)) != std::string::npos; pos += to.size()) body.replace(pos, from.size(), to);
        std::ofstream(archive, std::ios::binary).write(body.data(), body.size());
        spec.archive_sha256 = plugin::sha256(archive);
    }
    void make_first_entry_symlink() {
        std::ifstream in(archive, std::ios::binary);
        std::string body{std::istreambuf_iterator<char>(in), {}};
        const auto central = body.find(std::string("PK\1\2", 4));
        REQUIRE(central != std::string::npos);
        const uint32_t mode = (S_IFLNK | 0777) << 16;
        for (unsigned i = 0; i < 4; ++i) body[central + 38 + i] = char(mode >> (8 * i));
        std::ofstream(archive, std::ios::binary).write(body.data(), body.size());
        spec.archive_sha256 = plugin::sha256(archive);
    }
    plugin::Fetch fetch(int &count) {
        return [this, &count](const fs::path &to, const plugin::Cancelled &) {
            ++count;
            fs::copy_file(archive, to);
        };
    }
    bool clean_after_failure() {
        if (!fs::exists(data / "cloud-plugin-cache")) return true;
        for (const auto &entry : fs::directory_iterator(data / "cloud-plugin-cache"))
            if (entry.path().filename() != ".lock") return false;
        return true;
    }
};
}
TEST_CASE("Public runtime pins every vendor payload and only its fixed HTTPS origin", "[cloud-helper][public-runtime]")
{
    const auto spec = plugin::official_spec();
    plugin::Json pin = {{"plugin_url", spec.url}, {"plugin_sha256", spec.archive_sha256}, {"network_abi", spec.abi},
                        {"network_sha256", spec.payload_sha256.at("libbambu_networking.so")}, {"payload_sha256", spec.payload_sha256}, {"plugin_entries", spec.entries}};
    REQUIRE_NOTHROW(plugin::verified_pin(pin));
    // The public runtime builder and native downloader must advance together.
    const auto pin_path = fs::path(__FILE__).parent_path().parent_path().parent_path() / "scripts/tyr_cloud_helper/pin.json";
    std::ifstream checked_in_pin(pin_path);
    REQUIRE(checked_in_pin.good());
    const auto published_pin = plugin::Json::parse(checked_in_pin);
    REQUIRE_NOTHROW(plugin::verified_pin(published_pin));
    for (const auto *field : {"plugin_url", "plugin_sha256", "network_abi", "network_sha256", "payload_sha256", "plugin_entries"}) {
        auto altered = pin;
        altered[field] = "changed";
        REQUIRE_THROWS(plugin::verified_pin(altered));
    }
}
TEST_CASE("Public plugin cold cache extracts exact verified payloads and warm cache rechecks them", "[cloud-helper][public-runtime]")
{
    PluginFixture f;
    f.make_archive("unused-vendor-library.so");
    int downloads = 0;
    const auto cache = plugin::acquire(f.data, f.spec, f.fetch(downloads), {});
    REQUIRE(downloads == 1);
    REQUIRE(plugin::cache_valid(cache, f.spec));
    REQUIRE_FALSE(fs::exists(cache / "unused-vendor-library.so"));
    REQUIRE(plugin::acquire(f.data, f.spec, f.fetch(downloads), {}) == cache);
    REQUIRE(downloads == 1);
    std::ofstream(cache / "libBambuSource.so") << "tampered";
    REQUIRE(plugin::acquire(f.data, f.spec, f.fetch(downloads), {}) == cache);
    REQUIRE(downloads == 2);
    REQUIRE(plugin::cache_valid(cache, f.spec));
    std::ofstream(cache / "unverified-library.so") << "extra";
    REQUIRE(plugin::acquire(f.data, f.spec, f.fetch(downloads), {}) == cache);
    REQUIRE(downloads == 3);
    REQUIRE_FALSE(fs::exists(cache / "unverified-library.so"));
}
TEST_CASE("Public plugin verifies archive and all individual payload hashes", "[cloud-helper][public-runtime]")
{
    PluginFixture f;
    SECTION("archive") { f.spec.archive_sha256 = std::string(64, '0'); }
    SECTION("network") { f.spec.payload_sha256["libbambu_networking.so"] = std::string(64, '0'); }
    SECTION("source") { f.spec.payload_sha256["libBambuSource.so"] = std::string(64, '0'); }
    SECTION("video") { f.spec.payload_sha256["liblive555.so"] = std::string(64, '0'); }
    SECTION("missing video") { f.make_archive("", true); }
    int downloads = 0;
    REQUIRE_THROWS_AS(plugin::acquire(f.data, f.spec, f.fetch(downloads), {}), std::runtime_error);
    REQUIRE(f.clean_after_failure());
}
TEST_CASE("Public plugin refuses unsafe ZIP entries without writing outside private staging", "[cloud-helper][public-runtime]")
{
    PluginFixture f;
    SECTION("traversal") { f.make_archive("../escaped"); }
    SECTION("absolute") { f.make_archive("x/tmp/escaped"); f.rename_entry("x/tmp/escaped", "//tmp/escaped"); }
    SECTION("backslash") { f.make_archive("..\\escaped"); }
    SECTION("duplicate") { f.make_archive("libbambu_networking.so"); }
    SECTION("alternate payload path") { f.make_archive("another/libbambu_networking.so"); }
    SECTION("symlink") { f.make_first_entry_symlink(); }
    int downloads = 0;
    REQUIRE_THROWS_AS(plugin::acquire(f.data, f.spec, f.fetch(downloads), {}), std::runtime_error);
    REQUIRE(f.clean_after_failure());
    REQUIRE_FALSE(fs::exists(f.data / "escaped"));
}
TEST_CASE("Public plugin refuses cache directory and lock symlinks", "[cloud-helper][public-runtime]")
{
    PluginFixture f;
    fs::create_directories(f.data);
    SECTION("directory") { fs::create_directory_symlink(f.root, f.data / "cloud-plugin-cache"); }
    SECTION("lock") {
        fs::create_directories(f.data / "cloud-plugin-cache");
        fs::create_symlink(f.archive, f.data / "cloud-plugin-cache/.lock");
    }
    int downloads = 0;
    REQUIRE_THROWS_AS(plugin::acquire(f.data, f.spec, f.fetch(downloads), {}), std::runtime_error);
    REQUIRE(downloads == 0);
    REQUIRE(plugin::sha256(f.archive) == f.spec.archive_sha256);
}
TEST_CASE("Public plugin cancellation leaves no partial cache or download", "[cloud-helper][public-runtime]")
{
    PluginFixture f;
    bool cancel = false;
    int downloads = 0;
    auto fetch = f.fetch(downloads);
    SECTION("already cancelled") { cancel = true; }
    SECTION("during download") {
        fetch = [&](const fs::path &path, const plugin::Cancelled &) {
            std::ofstream(path) << "partial";
            cancel = true;
        };
    }
    SECTION("download failed") {
        fetch = [&](const fs::path &path, const plugin::Cancelled &) {
            std::ofstream(path) << "partial";
            throw std::runtime_error("network unavailable");
        };
        REQUIRE_THROWS_WITH(plugin::acquire(f.data, f.spec, fetch, [&] { return cancel; }), "network unavailable");
        REQUIRE(f.clean_after_failure());
        return;
    }
    REQUIRE_THROWS_AS(plugin::acquire(f.data, f.spec, fetch, [&] { return cancel; }), plugin::CancelledDownload);
    REQUIRE(f.clean_after_failure());
}
TEST_CASE("Concurrent public plugin users share one verified download and cancellation breaks lock wait", "[cloud-helper][public-runtime]")
{
    PluginFixture f;
    std::atomic<int> downloads{0};
    std::promise<void> started, release;
    auto released = release.get_future().share();
    plugin::Fetch fetch = [&](const fs::path &to, const plugin::Cancelled &) {
        ++downloads;
        started.set_value();
        released.wait();
        fs::copy_file(f.archive, to);
    };
    auto first = std::async(std::launch::async, [&] { return plugin::acquire(f.data, f.spec, fetch, {}); });
    started.get_future().wait();
    auto second = std::async(std::launch::async, [&] {
        const auto stop = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        try { (void)plugin::acquire(f.data, f.spec, fetch, [&] { return std::chrono::steady_clock::now() >= stop; }); }
        catch (const plugin::CancelledDownload &) { return true; }
        return false;
    });
    const bool ready = second.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    release.set_value();
    REQUIRE(ready);
    REQUIRE(second.get());
    const auto cache = first.get();
    REQUIRE(plugin::acquire(f.data, f.spec, fetch, {}) == cache);
    REQUIRE(downloads == 1);
    REQUIRE(plugin::cache_valid(cache, f.spec));
}
