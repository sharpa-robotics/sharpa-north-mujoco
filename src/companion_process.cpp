// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "sharpa_mujoco/companion_process.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <spawn.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

extern char** environ;

namespace sharpa::mujoco
{
namespace
{
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

const char* native_multiarch_library_directory()
{
#if defined(__x86_64__) || defined(_M_X64)
    return "x86_64-linux-gnu";
#elif defined(__aarch64__) || defined(_M_ARM64)
    return "aarch64-linux-gnu";
#else
    return "";
#endif
}

bool identifier(const std::string& value)
{
    return !value.empty() && value.size() <= 128 &&
           value.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") == std::string::npos;
}

fs::path contained(const fs::path& root, const std::string& relative, bool allow_missing = false)
{
    const fs::path input(relative);
    if (relative.empty() || input.is_absolute()) throw std::runtime_error("Expected a relative companion path");
    const auto resolved = allow_missing ? fs::weakly_canonical(root / input) : fs::canonical(root / input);
    const auto suffix = resolved.lexically_relative(root);
    if (suffix.empty() || *suffix.begin() == "..") throw std::runtime_error("Companion path escapes its runtime directory");
    // Preserve the logical basename for Loader's variant matching, even when
    // an existing base config is a symlink to another bundled file.
    return allow_missing ? (root / input).lexically_normal() : resolved;
}

struct Launch
{
    fs::path root;
    std::string name;
    std::string profile;
    std::vector<std::string> arguments;
    std::map<std::string, std::string> environment;
};

Launch read_launch(const std::string& directory, const std::string& requested_profile)
{
    Launch launch;
    launch.root = fs::canonical(directory);
    std::ifstream stream(contained(launch.root, "companion.conf"));
    if (!stream) throw std::runtime_error("Companion launch descriptor is missing");
    std::map<std::string, std::string> fields;
    std::vector<std::string> arguments;
    std::string line;
    while (std::getline(stream, line))
    {
        if (line.empty() || line.front() == '#') continue;
        const auto equals = line.find('=');
        if (equals == std::string::npos || line.find('\r') != std::string::npos || line.find('\0') != std::string::npos)
            throw std::runtime_error("Invalid companion launch descriptor");
        const auto key = line.substr(0, equals);
        const auto value = line.substr(equals + 1);
        if (key == "argument") arguments.push_back(value);
        else if (!fields.emplace(key, value).second) throw std::runtime_error("Duplicate companion descriptor field: " + key);
    }
    if (fields["schema_version"] != "2" || !identifier(fields["name"]))
        throw std::runtime_error("Unsupported companion launch descriptor");
    launch.name = fields["name"];
    for (const auto& field : fields)
    {
        if (field.first.rfind("environment.", 0) != 0) continue;
        const auto name = field.first.substr(12);
        if (name.empty() || name.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos)
            throw std::runtime_error("Invalid companion environment variable");
        launch.environment.emplace(name, field.second);
    }
    launch.profile = requested_profile.empty() ? fields["default_profile"] : requested_profile;
    if (!identifier(launch.profile) || !fields.count("profile." + launch.profile))
        throw std::runtime_error("Unknown companion profile: " + launch.profile);
    const auto executable = contained(launch.root, fields["executable"]);
    if (!fs::is_regular_file(executable) || access(executable.c_str(), X_OK) != 0)
        throw std::runtime_error("Companion is not executable: " + executable.string());
    const auto config = contained(launch.root, fields.at("profile." + launch.profile), true);
    // The producer lists actual files, while Loader resolves the logical
    // config argument using device.conf. Do not select a device variant here.
    if (!fs::is_directory(config.parent_path()) || (fs::exists(config) && !fs::is_regular_file(config)))
        throw std::runtime_error("Invalid companion profile path");
    const auto prefix = "profile_file." + launch.profile + ".";
    bool has_files = false;
    for (const auto& field : fields)
    {
        if (field.first.rfind(prefix, 0) != 0) continue;
        const auto file = contained(launch.root, field.second);
        if (!fs::is_regular_file(file)) throw std::runtime_error("Companion profile file is not a regular file");
        has_files = true;
    }
    if (!has_files) throw std::runtime_error("Companion profile has no files");
    launch.arguments.push_back(executable.string());
    launch.arguments.insert(launch.arguments.end(), arguments.begin(), arguments.end());
    launch.arguments.push_back(config.string());
    return launch;
}

fs::path state_directory()
{
    const auto directory = fs::temp_directory_path() / ("sharpa-mujoco-" + std::to_string(getuid()));
    if (mkdir(directory.c_str(), 0700) != 0 && errno != EEXIST)
        throw std::runtime_error("Cannot create companion state directory: " + std::string(strerror(errno)));
    struct stat info{};
    if (lstat(directory.c_str(), &info) != 0 || !S_ISDIR(info.st_mode) ||
        info.st_uid != getuid() || (info.st_mode & 0077) != 0)
        throw std::runtime_error("Companion state directory must be owned by this user with mode 0700");
    return directory;
}

int open_file(const fs::path& path, int flags)
{
    const int fd = open(path.c_str(), flags | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd < 0) throw std::runtime_error("Cannot open " + path.string() + ": " + strerror(errno));
    return fd;
}

bool exited(pid_t pid)
{
    siginfo_t info{};
    if (waitid(P_PID, pid, &info, WEXITED | WNOHANG | WNOWAIT) != 0) return errno == ECHILD;
    return info.si_pid == pid;
}

struct ProcessIdentity
{
    pid_t process_group = 0;
    unsigned long long start_time = 0;
    char state = '\0';
    std::string executable;
    unsigned long long executable_device = 0;
    unsigned long long executable_inode = 0;
};

bool read_process_identity(pid_t pid, ProcessIdentity& identity)
{
    if (pid <= 1) return false;
    const fs::path process_root = fs::path("/proc") / std::to_string(pid);
    struct stat info{};
    if (stat(process_root.c_str(), &info) != 0 || info.st_uid != getuid()) return false;

    std::ifstream stream(process_root / "stat");
    std::string line;
    if (!std::getline(stream, line)) return false;
    const auto close = line.rfind(')');
    if (close == std::string::npos || close + 2 >= line.size()) return false;
    std::istringstream fields(line.substr(close + 2));
    std::vector<std::string> values;
    for (std::string value; fields >> value;) values.push_back(std::move(value));
    // /proc/<pid>/stat fields 3, 5 and 22 are state, process group and start time.
    if (values.size() <= 19 || values.front().size() != 1) return false;
    try
    {
        identity.state = values[0].front();
        identity.process_group = static_cast<pid_t>(std::stol(values[2]));
        identity.start_time = std::stoull(values[19]);
    }
    catch (...) { return false; }

    const auto executable_link = process_root / "exe";
    struct stat executable_info{};
    // The procfs executable handle remains stat-able after its pathname is unlinked.
    if (stat(executable_link.c_str(), &executable_info) == 0)
    {
        identity.executable_device = static_cast<unsigned long long>(executable_info.st_dev);
        identity.executable_inode = static_cast<unsigned long long>(executable_info.st_ino);
    }

    std::vector<char> target(4096);
    const auto count = readlink(executable_link.c_str(), target.data(), target.size() - 1);
    if (count > 0)
    {
        target[static_cast<size_t>(count)] = '\0';
        identity.executable.assign(target.data(), static_cast<size_t>(count));
    }
    return true;
}

bool process_group_running(pid_t process_group)
{
    std::error_code error;
    for (const auto& entry : fs::directory_iterator("/proc", error))
    {
        if (error) break;
        const auto name = entry.path().filename().string();
        if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) continue;
        ProcessIdentity identity;
        if (read_process_identity(static_cast<pid_t>(std::stol(name)), identity) &&
            identity.process_group == process_group && identity.state != 'Z')
            return true;
    }
    return false;
}

struct OwnershipRecord
{
    int version = 0;
    pid_t pid = 0;
    unsigned long long start_time = 0;
    std::string executable;
    unsigned long long executable_device = 0;
    unsigned long long executable_inode = 0;
};

OwnershipRecord read_ownership_record(int descriptor)
{
    OwnershipRecord record;
    if (descriptor < 0 || lseek(descriptor, 0, SEEK_SET) < 0) return record;
    std::string content;
    std::vector<char> buffer(4096);
    const auto count = read(descriptor, buffer.data(), buffer.size());
    if (count <= 0) return record;
    content.assign(buffer.data(), static_cast<size_t>(count));
    std::istringstream lines(content);
    std::string line;
    try
    {
        while (std::getline(lines, line))
        {
            if (line.rfind("version=", 0) == 0) record.version = std::stoi(line.substr(8));
            else if (line.rfind("pid=", 0) == 0) record.pid = static_cast<pid_t>(std::stol(line.substr(4)));
            else if (line.rfind("start_time=", 0) == 0) record.start_time = std::stoull(line.substr(11));
            else if (line.rfind("executable=", 0) == 0) record.executable = line.substr(11);
            else if (line.rfind("executable_device=", 0) == 0) record.executable_device = std::stoull(line.substr(18));
            else if (line.rfind("executable_inode=", 0) == 0) record.executable_inode = std::stoull(line.substr(17));
        }
    }
    catch (...) { return {}; }
    return record;
}

void clear_ownership_record(int descriptor)
{
    if (descriptor < 0) return;
    if (ftruncate(descriptor, 0) == 0) lseek(descriptor, 0, SEEK_SET);
}

void write_ownership_record(int descriptor, pid_t pid)
{
    ProcessIdentity identity;
    if (!read_process_identity(pid, identity))
        throw std::runtime_error("Cannot read the new companion process identity");
    if (identity.process_group != pid)
        throw std::runtime_error("New companion does not own its process group");
    if (identity.executable.empty() || !identity.executable_device || !identity.executable_inode)
        throw std::runtime_error("Cannot identify the new companion executable");
    const std::string content = "version=2\npid=" + std::to_string(pid) +
        "\nstart_time=" + std::to_string(identity.start_time) +
        "\nexecutable=" + identity.executable +
        "\nexecutable_device=" + std::to_string(identity.executable_device) +
        "\nexecutable_inode=" + std::to_string(identity.executable_inode) + "\n";
    if (ftruncate(descriptor, 0) != 0 || lseek(descriptor, 0, SEEK_SET) < 0 ||
        write(descriptor, content.data(), content.size()) != static_cast<ssize_t>(content.size()) ||
        fsync(descriptor) != 0)
        throw std::runtime_error("Cannot persist companion ownership: " + std::string(strerror(errno)));
}

void stop_orphaned_process_group(int descriptor)
{
    const auto record = read_ownership_record(descriptor);
    const bool supported_version = record.version == 1 || record.version == 2;
    const bool complete_v2_identity = record.version != 2 ||
        (record.executable_device && record.executable_inode);
    if (!supported_version || !complete_v2_identity || record.pid <= 1 ||
        !record.start_time || record.executable.empty())
    {
        clear_ownership_record(descriptor);
        return;
    }
    ProcessIdentity identity;
    if (!read_process_identity(record.pid, identity))
    {
        clear_ownership_record(descriptor);
        return;
    }
    std::string normalized_executable = identity.executable;
    // Version 1 records predate inode identity; retain safe recovery for unlinked executables.
    static constexpr char DeletedSuffix[] = " (deleted)";
    if (normalized_executable.size() >= sizeof(DeletedSuffix) - 1 &&
        normalized_executable.compare(
            normalized_executable.size() - (sizeof(DeletedSuffix) - 1),
            sizeof(DeletedSuffix) - 1,
            DeletedSuffix) == 0)
        normalized_executable.resize(normalized_executable.size() - (sizeof(DeletedSuffix) - 1));
    const bool executable_matches = record.version == 2
        ? identity.executable_device == record.executable_device &&
          identity.executable_inode == record.executable_inode
        : normalized_executable == record.executable;
    if (identity.process_group != record.pid || identity.start_time != record.start_time ||
        (identity.state != 'Z' && !executable_matches))
        throw std::runtime_error("Refusing to stop an unverified process from stale companion ownership state");

    kill(-record.pid, SIGTERM);
    const auto graceful_deadline = Clock::now() + std::chrono::seconds(5);
    while (process_group_running(record.pid) && Clock::now() < graceful_deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    if (process_group_running(record.pid)) kill(-record.pid, SIGKILL);
    const auto forced_deadline = Clock::now() + std::chrono::seconds(2);
    while (process_group_running(record.pid) && Clock::now() < forced_deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    if (process_group_running(record.pid))
        throw std::runtime_error("Cannot stop the orphaned companion process group");
    clear_ownership_record(descriptor);
}

void location_anchor() {}
}  // namespace

class CompanionProcess::Impl
{
public:
    mutable std::mutex mutex;
    std::mutex operation_mutex;
    CompanionProcessStatus current;
    std::thread watcher;
    std::atomic<bool> watching{false};
    int lock_fd = -1;
    pid_t child = 0;
    std::string root;
    std::string profile;

    void release_lock()
    {
        if (lock_fd >= 0) close(lock_fd);
        lock_fd = -1;
    }

    void reap()
    {
        if (child > 0)
        {
            // Keep the leader unreaped until its process group has been stopped;
            // this prevents PID reuse while signaling descendants.
            kill(-child, SIGKILL);
            int status = 0;
            pid_t result;
            do { result = waitpid(child, &status, 0); } while (result < 0 && errno == EINTR);
            if (result > 0)
                current.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
            child = 0;
            current.pid = 0;
        }
        clear_ownership_record(lock_fd);
        release_lock();
    }

    void stop_owned()
    {
        watching.store(false);
        if (watcher.joinable()) watcher.join();
        std::lock_guard<std::mutex> guard(mutex);
        if (child > 0)
        {
            kill(-child, SIGTERM);
            const auto deadline = Clock::now() + std::chrono::seconds(5);
            while (!exited(child) && Clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            reap();
        }
        release_lock();
        current.state = CompanionProcessState::Stopped;
    }
};

CompanionProcess::CompanionProcess() : impl_(std::make_unique<Impl>()) {}
CompanionProcess::~CompanionProcess() { stop(); }

bool CompanionProcess::prepare(const std::string& directory, const std::string& requested_profile,
                               std::string* error)
{
    std::lock_guard<std::mutex> operation(impl_->operation_mutex);
    try
    {
        const auto launch = read_launch(directory, requested_profile);
        {
            std::lock_guard<std::mutex> guard(impl_->mutex);
            if (impl_->child > 0)
            {
                if (impl_->root == launch.root && impl_->profile == launch.profile)
                {
                    if (error) error->clear();
                    return true;
                }
                if (error) *error = "Stop the current companion before changing its profile";
                return false;
            }
            if (impl_->lock_fd >= 0 && impl_->root == launch.root && impl_->profile == launch.profile)
            {
                if (error) error->clear();
                return true;
            }
        }

        impl_->stop_owned();
        std::lock_guard<std::mutex> guard(impl_->mutex);
        impl_->current = {};
        impl_->current.state = CompanionProcessState::Starting;
        impl_->root = launch.root;
        impl_->profile = launch.profile;
        const auto state_root = state_directory();
        impl_->lock_fd = open_file(state_root / (launch.name + ".lock"), O_CREAT | O_RDWR);
        if (flock(impl_->lock_fd, LOCK_EX | LOCK_NB) != 0)
            throw std::runtime_error("Companion is already owned by another simulator: " + launch.name);
        stop_orphaned_process_group(impl_->lock_fd);
        impl_->current.log_path = (state_root / (launch.name + ".log")).string();
        if (error) error->clear();
        return true;
    }
    catch (const std::exception& exception)
    {
        std::lock_guard<std::mutex> guard(impl_->mutex);
        impl_->release_lock();
        impl_->current.state = CompanionProcessState::Failed;
        impl_->current.error = exception.what();
        if (error) *error = impl_->current.error;
        return false;
    }
}

bool CompanionProcess::start(const std::string& directory, const std::string& requested_profile, std::string* error)
{
    std::lock_guard<std::mutex> operation(impl_->operation_mutex);
    bool replacing = false;
    try
    {
        auto launch = read_launch(directory, requested_profile);
        {
            std::lock_guard<std::mutex> guard(impl_->mutex);
            if (impl_->child > 0)
            {
                if (impl_->root == launch.root && impl_->profile == launch.profile)
                {
                    if (error) error->clear();
                    return true;
                }
                if (error) *error = "Stop the current companion before changing its profile";
                return false;
            }
        }
        bool prepared = false;
        {
            std::lock_guard<std::mutex> guard(impl_->mutex);
            prepared = impl_->lock_fd >= 0 && impl_->root == launch.root && impl_->profile == launch.profile;
        }
        if (!prepared)
        {
            impl_->stop_owned();
        }
        replacing = true;
        std::lock_guard<std::mutex> guard(impl_->mutex);
        impl_->current.error.clear();
        impl_->current.exit_code = 0;
        impl_->current.pid = 0;
        impl_->current.state = CompanionProcessState::Starting;
        const auto state_root = state_directory();
        for (auto& field : launch.environment)
        {
            for (const auto& token : std::map<std::string, std::string>{
                     {"{root}", launch.root.string()}, {"{state}", state_root.string()}})
            {
                size_t position = 0;
                while ((position = field.second.find(token.first, position)) != std::string::npos)
                {
                    field.second.replace(position, token.first.size(), token.second);
                    position += token.second.size();
                }
            }
        }
        if (!prepared)
        {
            impl_->lock_fd = open_file(state_root / (launch.name + ".lock"), O_CREAT | O_RDWR);
            if (flock(impl_->lock_fd, LOCK_EX | LOCK_NB) != 0)
                throw std::runtime_error("Companion is already owned by another simulator: " + launch.name);
            stop_orphaned_process_group(impl_->lock_fd);
        }
        impl_->current.log_path = (state_root / (launch.name + ".log")).string();
        const int log_fd = open_file(impl_->current.log_path, O_CREAT | O_WRONLY | O_TRUNC);
        std::vector<std::string> environment;
        for (char** value = environ; *value; ++value)
        {
            const std::string entry(*value);
            if (entry.rfind("LD_LIBRARY_PATH=", 0) != 0 && entry.rfind("LD_PRELOAD=", 0) != 0 &&
                entry.rfind("ORIN_NVCOMP_SHIM_ACTIVE=", 0) != 0 &&
                launch.environment.count(entry.substr(0, entry.find('='))) == 0)
                environment.push_back(entry);
        }
        for (const auto& field : launch.environment)
            environment.push_back(field.first + "=" + field.second);
        // Isolate the child's native libraries from its host application's ABI.
        std::string library_path = (launch.root / "lib").string();
        if (const char* multiarch = native_multiarch_library_directory(); *multiarch)
            library_path += ":" + (launch.root / "lib" / multiarch).string();
        library_path += ":" + (launch.root / "avatar_sdk/lib").string();
        environment.push_back("LD_LIBRARY_PATH=" + library_path);
        std::vector<char*> envp;
        for (auto& entry : environment) envp.push_back(entry.data());
        envp.push_back(nullptr);
        auto arguments = launch.arguments;
        std::vector<char*> argv;
        for (auto& argument : arguments) argv.push_back(argument.data());
        argv.push_back(nullptr);

        posix_spawn_file_actions_t actions;
        posix_spawnattr_t attributes;
        int code = posix_spawn_file_actions_init(&actions);
        if (code != 0) { close(log_fd); throw std::runtime_error(strerror(code)); }
        code = posix_spawnattr_init(&attributes);
        if (code != 0) { close(log_fd); posix_spawn_file_actions_destroy(&actions); throw std::runtime_error(strerror(code)); }
        auto check = [&code](int result) { if (code == 0) code = result; };
        check(posix_spawn_file_actions_addchdir_np(&actions, launch.root.c_str()));
        check(posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0));
        check(posix_spawn_file_actions_adddup2(&actions, log_fd, STDOUT_FILENO));
        check(posix_spawn_file_actions_adddup2(&actions, log_fd, STDERR_FILENO));
        check(posix_spawn_file_actions_addclose(&actions, log_fd));
        sigset_t mask, defaults;
        sigemptyset(&mask);
        sigemptyset(&defaults);
        for (int signal : {SIGINT, SIGTERM, SIGHUP, SIGPIPE}) sigaddset(&defaults, signal);
        check(posix_spawnattr_setsigmask(&attributes, &mask));
        check(posix_spawnattr_setsigdefault(&attributes, &defaults));
        check(posix_spawnattr_setpgroup(&attributes, 0));
        check(posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF));
        if (code == 0)
            code = posix_spawn(&impl_->child, argv.front(), &actions, &attributes, argv.data(), envp.data());
        posix_spawn_file_actions_destroy(&actions);
        posix_spawnattr_destroy(&attributes);
        close(log_fd);
        if (code != 0) { impl_->child = 0; throw std::runtime_error("Cannot spawn companion: " + std::string(strerror(code))); }
        write_ownership_record(impl_->lock_fd, impl_->child);
        impl_->current.pid = impl_->child;
        impl_->root = launch.root;
        impl_->profile = launch.profile;
        const auto deadline = Clock::now() + std::chrono::milliseconds(300);
        do
        {
            if (exited(impl_->child))
            {
                impl_->reap();
                throw std::runtime_error("Companion exited during startup; see " + impl_->current.log_path);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } while (Clock::now() < deadline);
        impl_->current.state = CompanionProcessState::Running;
        impl_->watching.store(true);
        impl_->watcher = std::thread([this] {
            while (impl_->watching.load())
            {
                {
                    std::lock_guard<std::mutex> guard(impl_->mutex);
                    if (impl_->child > 0 && exited(impl_->child))
                    {
                        impl_->reap();
                        impl_->current.state = CompanionProcessState::Failed;
                        impl_->current.error = "Companion exited (" + std::to_string(impl_->current.exit_code) +
                                               "); see " + impl_->current.log_path;
                        std::fprintf(stderr, "%s\n", impl_->current.error.c_str());
                        impl_->watching.store(false);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
        if (error) error->clear();
        return true;
    }
    catch (const std::exception& exception)
    {
        std::lock_guard<std::mutex> guard(impl_->mutex);
        if (!replacing && impl_->child > 0)
        {
            if (error) *error = exception.what();
            return false;
        }
        if (impl_->child > 0) impl_->reap();
        impl_->release_lock();
        impl_->current.state = CompanionProcessState::Failed;
        impl_->current.error = exception.what();
        if (error) *error = impl_->current.error;
        return false;
    }
}

void CompanionProcess::stop() noexcept
{
    std::lock_guard<std::mutex> operation(impl_->operation_mutex);
    impl_->stop_owned();
}

CompanionProcessStatus CompanionProcess::status() const
{
    std::lock_guard<std::mutex> guard(impl_->mutex);
    return impl_->current;
}

std::string CompanionProcess::installed_root(const std::string& name)
{
    if (!identifier(name)) return {};
    std::vector<fs::path> candidates;
    Dl_info info{};
    if (dladdr(reinterpret_cast<const void*>(&location_anchor), &info) && info.dli_fname)
        candidates.push_back(fs::absolute(info.dli_fname).parent_path());
    std::error_code ec;
    const auto executable = fs::read_symlink("/proc/self/exe", ec);
    if (!ec)
    {
        candidates.push_back(executable.parent_path());
        candidates.push_back(executable.parent_path().parent_path() / "lib");
        // Standalone CMake target lives at <build>/apps/simulator/.
        candidates.push_back(executable.parent_path().parent_path().parent_path());
    }
    for (const auto& base : candidates)
    {
        const auto root = base / "sharpa_mujoco/companions" / name;
        if (fs::is_regular_file(root / "companion.conf", ec)) return root.string();
    }
    return {};
}

}  // namespace sharpa::mujoco
