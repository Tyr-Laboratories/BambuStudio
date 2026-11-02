#pragma once

#include "nlohmann/json.hpp"
#include <cerrno>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace Slic3r::TyrCloudSendRecovery {
namespace detail {
namespace fs = std::filesystem;
using Json = nlohmann::json;

[[noreturn]] inline void fail()
{
    throw std::runtime_error("The private print recovery record could not be saved. No new print was submitted.");
}

struct Descriptor {
    int value = -1;
    explicit Descriptor(int fd = -1) : value(fd) {}
    ~Descriptor() { if (value >= 0) ::close(value); }
    Descriptor(const Descriptor&) = delete;
    Descriptor& operator=(const Descriptor&) = delete;
};

inline bool private_file(int fd)
{
    struct stat s{};
    return fd >= 0 && ::fstat(fd, &s) == 0 && S_ISREG(s.st_mode) && s.st_uid == ::getuid() &&
           (s.st_mode & 0777) == 0600 && s.st_nlink == 1;
}

inline bool private_directory(int fd)
{
    struct stat s{};
    return fd >= 0 && ::fstat(fd, &s) == 0 && S_ISDIR(s.st_mode) && s.st_uid == ::getuid() &&
           (s.st_mode & 0777) == 0700;
}

inline bool valid_phase(const std::string& phase)
{
    return phase == "prepared" || phase == "running" || phase == "submitting" || phase == "finished";
}

inline bool valid_root(const fs::path& root)
{
    const std::string name = root.filename().string();
    if (name.size() != 10 || name.substr(0, 4) != "job-") return false;
    for (char c : name.substr(4))
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return false;
    return true;
}

inline std::string container_name(const fs::path& root, std::int64_t pid)
{
    return "tyr-cloud-send-" + std::to_string(pid) + "-" + root.filename().string().substr(4);
}

inline Json read_record(int directory)
{
    Descriptor file(::openat(directory, "owner.json", O_RDONLY | O_CLOEXEC | O_NOFOLLOW));
    struct stat s{};
    if (!private_file(file.value) || ::fstat(file.value, &s) != 0 || s.st_size < 1 || s.st_size > 16384) return {};
    std::string bytes(static_cast<size_t>(s.st_size), '\0');
    size_t offset = 0;
    while (offset < bytes.size()) {
        const auto n = ::read(file.value, bytes.data() + offset, bytes.size() - offset);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return {};
        offset += static_cast<size_t>(n);
    }
    auto value = Json::parse(bytes, nullptr, false);
    return value.is_discarded() ? Json{} : value;
}

inline bool valid_record(const Json& value, const fs::path& root)
{
    try {
        if (!value.is_object() || !valid_root(root) || value.value("schema", 0) != 1 ||
            !value.at("owner_pid").is_number_integer()) return false;
        const auto pid = value.at("owner_pid").get<std::int64_t>();
        return pid > 0 && pid <= std::numeric_limits<pid_t>::max() &&
               value.at("container").get<std::string>() == container_name(root, pid) &&
               value.at("target").is_string() && valid_phase(value.at("phase").get<std::string>());
    } catch (const Json::exception&) {
        return false;
    }
}

inline void write_record(int directory, const Json& value)
{
    // Only the lock holder writes this directory. Remove a partial replacement
    // left by a previous failed write, without following any symbolic link.
    if (::unlinkat(directory, "owner.json.tmp", 0) != 0 && errno != ENOENT) fail();
    Descriptor file(::openat(directory, "owner.json.tmp", O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600));
    if (file.value < 0 || ::fchmod(file.value, 0600) != 0) fail();
    const std::string bytes = value.dump();
    size_t offset = 0;
    while (offset < bytes.size()) {
        const auto n = ::write(file.value, bytes.data() + offset, bytes.size() - offset);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) fail();
        offset += static_cast<size_t>(n);
    }
    if (::fsync(file.value) != 0 || ::renameat(directory, "owner.json.tmp", directory, "owner.json") != 0 || ::fsync(directory) != 0) fail();
}

inline bool remove_stopped_job(const fs::path& root)
{
    // Keep ownership metadata until all payload/session cleanup succeeds. A
    // partial recursive-delete failure must not turn an owned job into an
    // unrecognizable directory containing the remaining account data.
    std::error_code error;
    for (fs::directory_iterator it(root, error), end; !error && it != end; it.increment(error)) {
        const auto name = it->path().filename();
        if (name != "owner.json" && name != "owner.lock") {
            fs::remove_all(it->path(), error);
            if (error) return false;
        }
    }
    if (error) return false;
    fs::remove_all(root, error);
    return !error;
}
} // namespace detail

// The lock is the ownership authority: a PID may be recycled, while a held flock
// prevents another live send or application instance from adopting the attempt.
// Destruction releases ownership; it deliberately preserves recovery evidence.
class Record {
    detail::Descriptor m_directory;
    detail::Descriptor m_lock;
    detail::Json m_record;

public:
    // Hold this before creating a job directory and until its Record is saved.
    // Reconciliation takes the same lock so an in-progress creation cannot be
    // mistaken for an abandoned attempt with missing ownership metadata.
    class DirectoryGuard {
        detail::Descriptor m_directory;
        detail::Descriptor m_lock;
    public:
        explicit DirectoryGuard(const std::filesystem::path& parent)
            : m_directory(::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW)),
              m_lock(m_directory.value < 0 ? -1 : ::openat(m_directory.value, ".recovery.lock", O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600))
        {
            if (!detail::private_directory(m_directory.value) || !detail::private_file(m_lock.value)) detail::fail();
            if (::flock(m_lock.value, LOCK_EX | LOCK_NB) != 0)
                throw std::runtime_error("Another print attempt is preparing or recovering. No new print was submitted. Please retry after it finishes.");
        }
        DirectoryGuard(const DirectoryGuard&) = delete;
        DirectoryGuard& operator=(const DirectoryGuard&) = delete;
    };

    Record(const std::filesystem::path& root, const std::string& container, const std::string& target)
        : m_directory(::open(root.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW)),
          m_lock(m_directory.value < 0 ? -1 : ::openat(m_directory.value, "owner.lock", O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600))
    {
        if (!detail::valid_root(root) || !detail::private_directory(m_directory.value) ||
            !detail::private_file(m_lock.value) || ::flock(m_lock.value, LOCK_EX | LOCK_NB) != 0 ||
            container != detail::container_name(root, ::getpid())) detail::fail();
        m_record = {{"schema", 1}, {"owner_pid", ::getpid()}, {"container", container}, {"target", target}, {"phase", "prepared"}};
        detail::write_record(m_directory.value, m_record);
    }
    Record(const Record&) = delete;
    Record& operator=(const Record&) = delete;

    void set_phase(const std::string& phase)
    {
        if (!detail::valid_phase(phase)) detail::fail();
        auto next = m_record;
        next["phase"] = phase;
        detail::write_record(m_directory.value, next);
        m_record = std::move(next);
    }

    // A true result consumes recovered uncertain attempts for any printer.
    // The caller must stop this send and ask the user to check the previous
    // printer(s), even when the newly selected target is different.
    // Failed cleanup retains its record, so the next call remains blocked.
    static bool reconcile(const std::filesystem::path& parent, const std::string& /*target*/,
                          const std::function<bool(const std::string&)>& stop)
    {
        namespace fs = std::filesystem;
        std::error_code error;
        if (!fs::exists(parent, error)) {
            if (error) detail::fail();
            return false;
        }
        DirectoryGuard guard(parent);
        detail::Descriptor parent_fd(::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
        if (!detail::private_directory(parent_fd.value)) detail::fail();
        bool uncertain = false, blocked = false, unknown = false;
        for (const auto& entry : fs::directory_iterator(parent)) {
            const auto root = entry.path();
            if (!detail::valid_root(root)) continue;
            detail::Descriptor directory(::openat(parent_fd.value, root.filename().c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
            if (!detail::private_directory(directory.value)) { unknown = true; continue; }
            detail::Descriptor lock(::openat(directory.value, "owner.lock", O_RDWR | O_CLOEXEC | O_NOFOLLOW));
            if (!detail::private_file(lock.value)) { unknown = true; continue; }
            if (::flock(lock.value, LOCK_EX | LOCK_NB) != 0) {
                if (errno != EWOULDBLOCK && errno != EAGAIN) unknown = true;
                continue;
            }
            const auto record = detail::read_record(directory.value);
            if (!detail::valid_record(record, root)) { unknown = true; continue; }
            bool stopped = false;
            try { stopped = stop(record.at("container").get<std::string>()); } catch (...) {}
            if (!stopped) {
                blocked = true;
                continue;
            }
            // Do not remove a replacement directory if the path changed while
            // the bounded container stop was in progress.
            struct stat opened{}, current{};
            if (::fstat(directory.value, &opened) != 0 || ::lstat(root.c_str(), &current) != 0 ||
                opened.st_dev != current.st_dev || opened.st_ino != current.st_ino) {
                blocked = true;
                continue;
            }
            if (!detail::remove_stopped_job(root)) {
                blocked = true;
                continue;
            }
            uncertain = uncertain || record.at("phase") == "submitting";
        }
        if (blocked)
            throw std::runtime_error("A previous print helper could not be stopped. Check the printer and restart Docker Desktop before retrying. Its private recovery data was retained.");
        if (unknown)
            throw std::runtime_error("A previous print attempt has an unreadable ownership record. No new print was submitted. Check the previous printer and have its private recovery data inspected before retrying.");
        return uncertain;
    }
};
} // namespace Slic3r::TyrCloudSendRecovery
