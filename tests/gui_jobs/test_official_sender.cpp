#include "catch_main.hpp"
#include "slic3r/GUI/OfficialSender.hpp"
#include <fstream>
#include <chrono>
#include <nlohmann/json.hpp>

namespace Sender = Slic3r::GUI::OfficialSender;
namespace fs = std::filesystem;
using nlohmann::json;

namespace {
struct Files {
    fs::path root = fs::temp_directory_path() / ("tyr-sender-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::path source = root / "Source settings";
    fs::path target = root / "Sender settings";
    Files() { fs::create_directories(source); }
    ~Files() { std::error_code ec; fs::remove_all(root, ec); }
};
void write(const fs::path &path, const std::string &bytes) {
    fs::create_directories(path.parent_path());
    std::ofstream f(path, std::ios::binary); f << bytes;
}
std::string read(const fs::path &path) {
    std::ifstream f(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}
void config(const fs::path &root) {
    write(root / "BambuStudio.conf", R"({"app":{"region":"North America","single_instance":false,"last_backup_path":"old-recovery","sync_user_preset":true,"sync_system_preset":true,"installed_networking":true},"recent_projects":["private-model.3mf"],"presets":{"print":"My process"},"access_code":{"fake":"fixture-only"}})");
}
}

TEST_CASE("Sender bootstrap isolates selected state and disables duplicate profile writers", "[sender]") {
    Files f; config(f.source);
    const auto original = read(f.source / "BambuStudio.conf");
    write(f.source / "BambuNetworkEngine.conf", std::string("opaque\0state", 12));
    write(f.source / "user" / "123" / "print" / "custom.json", "profile");
    write(f.source / "system" / "BBL.json", "system");
    write(f.source / "printers" / "settings.json", "printer");
    write(f.source / "plugins" / "version.txt", "02.08.04.60");
    write(f.source / "log" / "private.log", "do not copy");
    write(f.source / "cache" / "old-job.3mf", "do not copy");
    Sender::prepare_data(f.source, f.target);
    REQUIRE(fs::exists(f.target / "BambuStudio.conf"));
    auto copy = json::parse(read(f.target / "BambuStudio.conf"));
    CHECK(copy["app"]["single_instance"] == true);
    CHECK(copy["app"]["sync_user_preset"] == false);
    CHECK(copy["app"]["sync_system_preset"] == false);
    CHECK(copy["app"]["last_backup_path"] == "");
    CHECK(copy["presets"]["print"] == "My process");
    CHECK(copy["access_code"]["fake"] == "fixture-only");
    CHECK_FALSE(copy.contains("recent_projects"));
    CHECK(read(f.target / "BambuNetworkEngine.conf") == std::string("opaque\0state", 12));
    CHECK(read(f.target / "user" / "123" / "print" / "custom.json") == "profile");
    CHECK(read(f.target / "plugins" / "version.txt") == "02.08.04.60");
    CHECK_FALSE(fs::exists(f.target / "log"));
    CHECK_FALSE(fs::exists(f.target / "cache"));
    CHECK(read(f.source / "BambuStudio.conf") == original);
    CHECK((fs::status(f.target).permissions() & fs::perms::others_all) == fs::perms::none);
}

TEST_CASE("Later handoffs preserve sender login and settings", "[sender]") {
    Files f; config(f.source); Sender::prepare_data(f.source, f.target);
    write(f.target / "BambuNetworkEngine.conf", "refreshed sender session");
    write(f.target / "BambuStudio.conf", "new sender preferences");
    write(f.source / "BambuNetworkEngine.conf", "older source session");
    Sender::prepare_data(f.source, f.target);
    CHECK(read(f.target / "BambuNetworkEngine.conf") == "refreshed sender session");
    CHECK(read(f.target / "BambuStudio.conf") == "new sender preferences");
}

TEST_CASE("Sender refuses invalid or overlapping bootstrap without leaving partial settings", "[sender]") {
    Files f;
    SECTION("invalid config") { write(f.source / "BambuStudio.conf", "not json"); CHECK_THROWS(Sender::prepare_data(f.source, f.target)); CHECK_FALSE(fs::exists(f.target)); }
    SECTION("same directory") { config(f.source); CHECK_THROWS(Sender::prepare_data(f.source, f.source)); CHECK(fs::exists(f.source / "BambuStudio.conf")); }
    SECTION("partial target") { config(f.source); write(f.target / "existing", "keep"); CHECK_THROWS(Sender::prepare_data(f.source, f.target)); CHECK(read(f.target / "existing") == "keep"); }
}

TEST_CASE("Sender preserves internal plugin links and refuses links outside copied state", "[sender]") {
    Files f; config(f.source);
    write(f.source / "plugins" / "Versions" / "A" / "library", "signed bytes");
    fs::create_directory_symlink("A", f.source / "plugins" / "Versions" / "Current");
    SECTION("internal framework link") {
        Sender::prepare_data(f.source, f.target);
        CHECK(fs::is_symlink(f.target / "plugins" / "Versions" / "Current"));
        CHECK(read(f.target / "plugins" / "Versions" / "Current" / "library") == "signed bytes");
    }
    SECTION("external link") {
        write(f.root / "outside", "private");
        fs::create_symlink(f.root / "outside", f.source / "user");
        CHECK_THROWS(Sender::prepare_data(f.source, f.target));
        CHECK_FALSE(fs::exists(f.target));
    }
    SECTION("relative top-level link") {
        write(f.root / "outside", "private");
        fs::create_symlink("../outside", f.source / "user");
        CHECK_THROWS(Sender::prepare_data(f.source, f.target));
        CHECK_FALSE(fs::exists(f.target));
    }
}

TEST_CASE("Handoffs retain distinct immutable job bytes with safe launch arguments", "[sender]") {
    Files f; config(f.source); Sender::prepare_data(f.source, f.target);
    auto original = f.root / "input.gcode.3mf";
    write(original, std::string("PK\0sliced-job", 13));
    auto first = Sender::stage_job(original, f.target, "Plate 1 – bridge 'test' $(echo nope)");
    REQUIRE(fs::is_regular_file(first));
    CHECK(first.extension() == ".3mf");
    CHECK(read(first) == std::string("PK\0sliced-job", 13));
    write(original, "second-job");
    auto second = Sender::stage_job(original, f.target, "Plate 1 – bridge 'test' $(echo nope)");
    CHECK(first != second);
    CHECK(read(first) == std::string("PK\0sliced-job", 13));
    CHECK(read(second) == "second-job");
    const auto args = Sender::launch_arguments(f.target, first);
    REQUIRE(args.size() == 2);
    CHECK(args[0] == "--datadir=" + f.target.string());
    CHECK(args[1] == first.string());
    CHECK_THROWS(Sender::stage_job(f.root / "missing.3mf", f.target, "missing"));
}
