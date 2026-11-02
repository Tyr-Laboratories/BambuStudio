#include <sys/types.h>
#include <signal.h>

#include "catch2/catch.hpp"
#include "slic3r/Utils/TyrCloudSendRecovery.hpp"

#include <fstream>
#include <memory>
#include <sys/wait.h>

namespace {
namespace fs = std::filesystem;
using Slic3r::TyrCloudSendRecovery::Record;

struct RecoveryFixture {
    fs::path parent;
    RecoveryFixture()
    {
        char path[] = "/tmp/tyr-recovery-test-XXXXXX";
        const char* created = ::mkdtemp(path);
        if (!created) throw std::runtime_error("Could not create recovery test directory");
        parent = created;
    }
    ~RecoveryFixture() { fs::remove_all(parent); }
    fs::path directory(const std::string& suffix = "Ab1234")
    {
        const auto path = parent / ("job-" + suffix);
        fs::create_directory(path);
        fs::permissions(path, fs::perms::owner_all, fs::perm_options::replace);
        return path;
    }
    std::string name(const fs::path& root, pid_t pid = ::getpid()) const
    {
        return "tyr-cloud-send-" + std::to_string(pid) + "-" + root.filename().string().substr(4);
    }
};
}

TEST_CASE("Recovery preserves active sends and removes only stopped abandoned attempts", "[cloud-helper][recovery]")
{
    RecoveryFixture fixture;
    const auto root = fixture.directory();
    auto record = std::make_unique<Record>(root, fixture.name(root), "printer-a");
    record->set_phase("submitting");
    int stopped = 0;
    auto stop = [&](const std::string& name) {
        CHECK(name == fixture.name(root));
        CHECK(fs::exists(root / "owner.json"));
        ++stopped;
        return true;
    };
    CHECK_FALSE(Record::reconcile(fixture.parent, "printer-a", stop));
    CHECK(stopped == 0);
    CHECK(fs::exists(root));
    record.reset();
    CHECK(Record::reconcile(fixture.parent, "printer-a", stop));
    CHECK(stopped == 1);
    CHECK_FALSE(fs::exists(root));
    CHECK_FALSE(Record::reconcile(fixture.parent, "printer-a", stop));
    CHECK(stopped == 1);
}

TEST_CASE("Failed stale-container cleanup durably blocks further submissions", "[cloud-helper][recovery]")
{
    RecoveryFixture fixture;
    const auto root = fixture.directory();
    { Record record(root, fixture.name(root), "printer-a"); record.set_phase("submitting"); }
    std::ofstream(root / "account-copy") << "private fixture session";
    auto unavailable = [](const std::string&) { return false; };
    for (int retry = 0; retry != 2; ++retry) {
        CHECK_THROWS_WITH(Record::reconcile(fixture.parent, "printer-a", unavailable), Catch::Matchers::Contains("could not be stopped"));
        CHECK(fs::exists(root / "owner.json"));
        CHECK(fs::exists(root / "account-copy"));
    }
    CHECK_THROWS_WITH(Record::reconcile(fixture.parent, "printer-b", unavailable), Catch::Matchers::Contains("could not be stopped"));
    CHECK(fs::exists(root));
    CHECK(Record::reconcile(fixture.parent, "printer-a", [](const std::string&) { return true; }));
    CHECK_FALSE(fs::exists(root));
}

TEST_CASE("Recovery distinguishes preparation and known completion from uncertain submission", "[cloud-helper][recovery]")
{
    for (const char* phase : {"prepared", "running", "finished"}) {
        RecoveryFixture fixture;
        const auto root = fixture.directory();
        { Record record(root, fixture.name(root), "printer-a"); record.set_phase(phase); }
        CHECK_FALSE(Record::reconcile(fixture.parent, "printer-a", [](const std::string&) { return true; }));
        CHECK_FALSE(fs::exists(root));
    }
}

TEST_CASE("Unknown and malformed recovery directories are never adopted or deleted", "[cloud-helper][recovery]")
{
    RecoveryFixture fixture;
    const auto unknown = fixture.directory("Un1234");
    std::ofstream(unknown / "unrelated") << "leave this alone";
    const auto malformed = fixture.directory("Ba1234");
    { Record record(malformed, fixture.name(malformed), "printer-a"); }
    std::ofstream(malformed / "owner.json") << "not json";
    const auto wrong_container = fixture.directory("Wr1234");
    { Record record(wrong_container, fixture.name(wrong_container), "printer-a"); }
    nlohmann::json record;
    { std::ifstream in(wrong_container / "owner.json"); in >> record; }
    record["container"] = "unrelated-container";
    std::ofstream(wrong_container / "owner.json") << record.dump();
    int stopped = 0;
    CHECK_THROWS_WITH(Record::reconcile(fixture.parent, "printer-a", [&](const std::string&) { ++stopped; return true; }),
                      Catch::Matchers::Contains("ownership record"));
    CHECK(stopped == 0);
    CHECK(fs::exists(unknown / "unrelated"));
    CHECK(fs::exists(malformed));
    CHECK(fs::exists(wrong_container));
}

TEST_CASE("Recovery cleans other stale printers but warns before any new submission", "[cloud-helper][recovery]")
{
    RecoveryFixture fixture;
    const auto root = fixture.directory();
    { Record record(root, fixture.name(root), "printer-a"); record.set_phase("submitting"); }
    std::ofstream(root / "account-copy") << "private fixture session";
    int stopped = 0;
    CHECK(Record::reconcile(fixture.parent, "printer-b", [&](const std::string&) { ++stopped; return true; }));
    CHECK(stopped == 1);
    CHECK_FALSE(fs::exists(root));
    CHECK_FALSE(Record::reconcile(fixture.parent, "printer-a", [&](const std::string&) { ++stopped; return true; }));
    CHECK(stopped == 1);
}

TEST_CASE("Recovery does not follow directory or metadata symbolic links", "[cloud-helper][recovery]")
{
    RecoveryFixture fixture;
    const auto outside = fixture.parent / "unrelated-directory";
    fs::create_directory(outside);
    std::ofstream(outside / "keep") << "unrelated";
    fs::create_directory_symlink(outside, fixture.parent / "job-Sy1234");
    const auto root = fixture.directory();
    { Record record(root, fixture.name(root), "printer-a"); }
    fs::remove(root / "owner.json");
    fs::create_symlink(outside / "keep", root / "owner.json");
    int stopped = 0;
    CHECK_THROWS_WITH(Record::reconcile(fixture.parent, "printer-a", [&](const std::string&) { ++stopped; return true; }),
                      Catch::Matchers::Contains("ownership record"));
    CHECK(stopped == 0);
    CHECK(fs::exists(outside / "keep"));
    CHECK(fs::is_symlink(root / "owner.json"));
}

TEST_CASE("Recovery records are private and a live ownership lock survives another process", "[cloud-helper][recovery]")
{
    RecoveryFixture fixture;
    const auto root = fixture.directory();
    Record record(root, fixture.name(root), "printer-a");
    record.set_phase("submitting");
    struct stat metadata{}, lock{};
    REQUIRE(::stat((root / "owner.json").c_str(), &metadata) == 0);
    REQUIRE(::stat((root / "owner.lock").c_str(), &lock) == 0);
    CHECK((metadata.st_mode & 0777) == 0600);
    CHECK((lock.st_mode & 0777) == 0600);
    const pid_t child = ::fork();
    REQUIRE(child >= 0);
    if (child == 0) {
        try {
            bool stopped = false;
            const bool uncertain = Record::reconcile(fixture.parent, "printer-a", [&](const std::string&) { stopped = true; return true; });
            ::_exit(!stopped && !uncertain && fs::exists(root) ? 0 : 1);
        } catch (...) { ::_exit(2); }
    }
    int status = 0;
    REQUIRE(::waitpid(child, &status, 0) == child);
    REQUIRE(WIFEXITED(status));
    CHECK(WEXITSTATUS(status) == 0);
    CHECK(fs::exists(root));
}

TEST_CASE("Recovery finds an attempt after its owner process is killed", "[cloud-helper][recovery]")
{
    RecoveryFixture fixture;
    const auto root = fixture.directory();
    int ready[2];
    REQUIRE(::pipe(ready) == 0);
    const pid_t child = ::fork();
    REQUIRE(child >= 0);
    if (child == 0) {
        ::close(ready[0]);
        try {
            Record record(root, fixture.name(root), "printer-a");
            record.set_phase("submitting");
            const char marker = '1';
            (void) ::write(ready[1], &marker, 1);
            for (;;) ::pause();
        } catch (...) { ::_exit(2); }
    }
    ::close(ready[1]);
    char marker = 0;
    const auto received = ::read(ready[0], &marker, 1);
    ::close(ready[0]);
    ::kill(child, SIGKILL);
    int status = 0;
    REQUIRE(::waitpid(child, &status, 0) == child);
    REQUIRE(received == 1);
    REQUIRE(marker == '1');
    std::string stopped;
    CHECK(Record::reconcile(fixture.parent, "printer-a", [&](const std::string& name) { stopped = name; return true; }));
    CHECK(stopped == fixture.name(root, child));
    CHECK_FALSE(fs::exists(root));
}

TEST_CASE("Recovery does not inspect a job while its ownership record is being created", "[cloud-helper][recovery]")
{
    RecoveryFixture fixture;
    fs::path root;
    int stopped = 0;
    {
        Record::DirectoryGuard creation(fixture.parent);
        root = fixture.directory();
        CHECK_THROWS_WITH(Record::reconcile(fixture.parent, "printer-a", [&](const std::string&) { ++stopped; return true; }),
                          Catch::Matchers::Contains("preparing or recovering"));
        CHECK(stopped == 0);
        CHECK(fs::exists(root));
        Record record(root, fixture.name(root), "printer-a");
    }
    CHECK_FALSE(Record::reconcile(fixture.parent, "printer-a", [&](const std::string& name) {
        CHECK(name == fixture.name(root));
        ++stopped;
        return true;
    }));
    CHECK(stopped == 1);
    CHECK_FALSE(fs::exists(root));
}
