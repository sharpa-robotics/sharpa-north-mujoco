// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/c_api.h"

#include <sys/stat.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

namespace fs = std::filesystem;
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

bool process_running(pid_t pid)
{
    std::ifstream stream(fs::path("/proc") / std::to_string(pid) / "stat");
    std::string line;
    if (!std::getline(stream, line)) return false;
    const auto close = line.rfind(')');
    return close != std::string::npos && close + 2 < line.size() && line[close + 2] != 'Z';
}

int main(int argc, char** argv)
{
    if (argc != 2) return EXIT_FAILURE;
    char directory[] = "/tmp/sharpa-companion-test.XXXXXX";
    const char* created = mkdtemp(directory);
    if (!created) return EXIT_FAILURE;
    const fs::path root(created);
    const fs::path worker_source = fs::canonical(argv[1]);
    const fs::path worker = root / "worker";
    const auto install_worker = [&] {
        fs::remove(worker);
        fs::copy_file(worker_source, worker);
        chmod(worker.c_str(), 0755);
    };
    using Owner = std::unique_ptr<sharpa_mujoco_companion, decltype(&sharpa_mujoco_companion_destroy)>;
    try
    {
        const std::string name = "test_" + std::to_string(getpid());
        std::ofstream(root / "companion.conf")
            << "schema_version=2\nname=" << name
            << "\nexecutable=worker\ndefault_profile=normal\nprofile.normal=normal\nprofile.fail=fail\nprofile.slow=slow\n"
            << "profile_file.normal.0=normal\nprofile_file.fail.0=fail\nprofile_file.slow.0=slow\n"
            << "environment.TEST_COMPANION_ROOT={root}\n";
        std::ofstream(root / "normal") << "normal";
        std::ofstream(root / "fail") << "fail";
        std::ofstream(root / "slow") << "slow";
        install_worker();
        Owner first(sharpa_mujoco_companion_create(), sharpa_mujoco_companion_destroy);
        Owner second(sharpa_mujoco_companion_create(), sharpa_mujoco_companion_destroy);
        require(first && second, "create failed");
        char error[2048]{};
        require(!sharpa_mujoco_companion_start(first.get(), "missing_runtime", nullptr, nullptr, error, sizeof(error)), "missing runtime accepted");
        int state, pid, code;
        require(sharpa_mujoco_companion_prepare(first.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)), error);
        require(sharpa_mujoco_companion_status(first.get(), &state, &pid, &code, error, sizeof(error)), "prepared status failed");
        require(state == SHARPA_MUJOCO_COMPANION_STARTING && pid == 0, "prepare launched a child");
        require(!sharpa_mujoco_companion_prepare(second.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)), "duplicate prepare accepted");
        require(sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)), error);
        require(sharpa_mujoco_companion_status(first.get(), &state, &pid, &code, error, sizeof(error)), "status failed");
        require(state == SHARPA_MUJOCO_COMPANION_RUNNING && pid > 0, "not running");
        const int original_pid = pid;
        require(sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)), "start not idempotent");
        require(!sharpa_mujoco_companion_start(second.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)), "duplicate owner accepted");
        require(!sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), "../bad", error, sizeof(error)), "invalid profile accepted");
        require(kill(original_pid, 0) == 0, "invalid profile stopped existing process");
        char log[1024]{};
        require(sharpa_mujoco_companion_log_path(first.get(), log, sizeof(log)), "log path failed");
        std::ifstream output(log);
        std::string cwd, libraries;
        std::getline(output, cwd);
        std::getline(output, libraries);
        require(cwd == root && libraries.find((root / "lib").string()) == 0, "child cwd or environment incorrect");
#if defined(__x86_64__) || defined(_M_X64)
        require(libraries.find((root / "lib/x86_64-linux-gnu").string()) != std::string::npos,
                "x86_64 companion library directory missing");
#elif defined(__aarch64__) || defined(_M_ARM64)
        require(libraries.find((root / "lib/aarch64-linux-gnu").string()) != std::string::npos,
                "aarch64 companion library directory missing");
#endif
        std::string configured_root;
        std::getline(output, configured_root);
        require(configured_root == root, "descriptor environment was not expanded");
        sharpa_mujoco_companion_stop(first.get());
        require(kill(original_pid, 0) != 0, "child was not reaped");
        sharpa_mujoco_companion_stop(first.get());
        // A v2 descriptor supplies actual files but passes the logical name
        // unchanged. The worker intentionally does not implement device matching.
        const fs::path descriptor_path = root / "companion.conf";
        std::ifstream original_descriptor_input(descriptor_path);
        const std::string original_descriptor(
            (std::istreambuf_iterator<char>(original_descriptor_input)), std::istreambuf_iterator<char>());
        fs::create_directories(root / "config/sim");
        const fs::path variant = root / "config/sim/fixture.M1.B1.json";
        std::ofstream(variant) << "{}";
        const auto write_variant_descriptor = [&](const std::string& config, const std::string& files)
        {
            std::ofstream(descriptor_path)
                << "schema_version=2\nname=" << name
                << "\nexecutable=worker\nargument=-c\ndefault_profile=fixture\nprofile.fixture=" << config
                << "\n" << files;
        };
        const std::string variant_field = "profile_file.fixture.0=config/sim/fixture.M1.B1.json\n";
        write_variant_descriptor("config/sim/fixture.json", variant_field);
        require(sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)), error);
        std::ifstream variant_log(log);
        std::string argument;
        for (int line = 0; line < 4; ++line) std::getline(variant_log, argument);
        require(argument == (root / "config/sim/fixture.json").string(), "logical config argument changed");
        require(!fs::exists(root / "config/sim/fixture.json"), "fixture accidentally contains base config");
        sharpa_mujoco_companion_stop(first.get());
        fs::create_symlink(variant.filename(), root / "config/sim/fixture.json");
        require(sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)), error);
        std::ifstream symlink_log(log);
        for (int line = 0; line < 4; ++line) std::getline(symlink_log, argument);
        require(argument == (root / "config/sim/fixture.json").string(), "logical symlink basename changed");
        sharpa_mujoco_companion_stop(first.get());
        fs::remove(root / "config/sim/fixture.json");
        fs::remove(variant);
        require(!sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)),
                "missing profile file accepted");
        std::ofstream(variant) << "{}";
        write_variant_descriptor("config/sim/fixture.json", "");
        require(!sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)),
                "empty profile catalog accepted");
        fs::create_directory_symlink(root.parent_path(), root / "outside");
        write_variant_descriptor("outside/missing.json", variant_field);
        require(!sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)),
                "escaping logical path accepted");
        write_variant_descriptor("config/sim/fixture.json", "profile_file.fixture.0=outside/missing.json\n");
        require(!sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)),
                "escaping profile file accepted");
        std::ofstream(descriptor_path) << original_descriptor;
        require(!sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), "fail", error, sizeof(error)), "immediate failure accepted");
        require(sharpa_mujoco_companion_start(second.get(), name.c_str(), root.c_str(), "slow", error, sizeof(error)), error);
        for (int index = 0; index < 60; ++index)
        {
            sharpa_mujoco_companion_status(second.get(), &state, &pid, &code, error, sizeof(error));
            if (state == SHARPA_MUJOCO_COMPANION_FAILED) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        require(state == SHARPA_MUJOCO_COMPANION_FAILED && code == 9 && pid == 0, "late failure was not detected/reaped");
        require(sharpa_mujoco_companion_start(first.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)), error);
        sharpa_mujoco_companion_status(first.get(), &state, &pid, &code, error, sizeof(error));
        first.reset();
        require(kill(pid, 0) != 0, "destructor left child running");

        const auto abandon_companion = [&]() -> int
        {
            int ownership_pipe[2];
            require(pipe(ownership_pipe) == 0, "cannot create ownership pipe");
            const pid_t crashed_owner = fork();
            require(crashed_owner >= 0, "cannot fork crashed owner");
            if (crashed_owner == 0)
            {
                close(ownership_pipe[0]);
                Owner abandoned(sharpa_mujoco_companion_create(), sharpa_mujoco_companion_destroy);
                int abandoned_pid = 0;
                if (!abandoned || !sharpa_mujoco_companion_start(
                        abandoned.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)) ||
                    !sharpa_mujoco_companion_status(
                        abandoned.get(), nullptr, &abandoned_pid, nullptr, error, sizeof(error)) ||
                    write(ownership_pipe[1], &abandoned_pid, sizeof(abandoned_pid)) != sizeof(abandoned_pid))
                    _exit(2);
                // Model an owner killed before C++ destructors or UE shutdown hooks run.
                _exit(0);
            }
            close(ownership_pipe[1]);
            int abandoned_pid = 0;
            require(read(ownership_pipe[0], &abandoned_pid, sizeof(abandoned_pid)) == sizeof(abandoned_pid),
                    "crashed owner did not report its companion");
            close(ownership_pipe[0]);
            int crashed_status = 0;
            require(waitpid(crashed_owner, &crashed_status, 0) == crashed_owner &&
                        WIFEXITED(crashed_status) && WEXITSTATUS(crashed_status) == 0,
                    "crashed owner fixture failed");
            return abandoned_pid;
        };

        const fs::path ownership_path = fs::temp_directory_path() /
            ("sharpa-mujoco-" + std::to_string(getuid())) / (name + ".lock");
        int abandoned_pid = abandon_companion();
        require(kill(abandoned_pid, 0) == 0, "crashed owner did not leave a companion to recover");
        std::ifstream valid_ownership_input(ownership_path);
        const std::string valid_ownership(
            (std::istreambuf_iterator<char>(valid_ownership_input)), std::istreambuf_iterator<char>());
        std::string mismatched_ownership = valid_ownership;
        const auto inode_value = mismatched_ownership.find("executable_inode=");
        require(inode_value != std::string::npos, "version 2 ownership record has no executable inode");
        const auto inode_begin = inode_value + sizeof("executable_inode=") - 1;
        const auto inode_end = mismatched_ownership.find('\n', inode_begin);
        const auto wrong_inode = std::stoull(mismatched_ownership.substr(inode_begin, inode_end - inode_begin)) + 1;
        mismatched_ownership.replace(inode_begin, inode_end - inode_begin, std::to_string(wrong_inode));
        std::ofstream(ownership_path, std::ios::trunc) << mismatched_ownership;
        Owner rejected(sharpa_mujoco_companion_create(), sharpa_mujoco_companion_destroy);
        require(rejected && !sharpa_mujoco_companion_prepare(
                    rejected.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)),
                "mismatched executable identity was accepted");
        require(process_running(abandoned_pid), "identity mismatch killed the unverified process");
        std::ofstream(ownership_path, std::ios::trunc) << valid_ownership;
        install_worker();
        const auto deleted_executable = fs::read_symlink(
            fs::path("/proc") / std::to_string(abandoned_pid) / "exe");
        require(deleted_executable.string().find(" (deleted)") != std::string::npos,
                "replaced worker executable was not marked deleted");

        Owner recovered(sharpa_mujoco_companion_create(), sharpa_mujoco_companion_destroy);
        require(recovered && sharpa_mujoco_companion_prepare(
                    recovered.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)),
                error);
        require(!process_running(abandoned_pid), "prepare left the orphaned companion running");
        require(sharpa_mujoco_companion_status(
                    recovered.get(), &state, &pid, &code, error, sizeof(error)) &&
                    state == SHARPA_MUJOCO_COMPANION_STARTING && pid == 0,
                "recovery prepare launched a replacement prematurely");
        require(sharpa_mujoco_companion_start(
                    recovered.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)),
                error);
        int recovered_pid = 0;
        require(sharpa_mujoco_companion_status(
                    recovered.get(), nullptr, &recovered_pid, nullptr, error, sizeof(error)),
                "recovered status failed");
        require(recovered_pid > 0 && recovered_pid != abandoned_pid,
                "recovery did not replace the orphaned companion");
        sharpa_mujoco_companion_stop(recovered.get());

        const int legacy_pid = abandon_companion();
        require(kill(legacy_pid, 0) == 0, "legacy owner did not leave a companion to recover");
        std::ifstream ownership_input(ownership_path);
        std::string ownership(
            (std::istreambuf_iterator<char>(ownership_input)), std::istreambuf_iterator<char>());
        require(ownership.rfind("version=2\n", 0) == 0, "new ownership record is not version 2");
        ownership.replace(0, sizeof("version=2") - 1, "version=1");
        std::ofstream(ownership_path, std::ios::trunc) << ownership;
        install_worker();
        Owner legacy_recovered(sharpa_mujoco_companion_create(), sharpa_mujoco_companion_destroy);
        require(legacy_recovered && sharpa_mujoco_companion_prepare(
                    legacy_recovered.get(), name.c_str(), root.c_str(), nullptr, error, sizeof(error)),
                "version 1 deleted executable was not recovered");
        require(!process_running(legacy_pid), "version 1 recovery left the deleted executable running");
        fs::remove_all(root);
        return EXIT_SUCCESS;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAILED: " << error.what() << '\n';
        fs::remove_all(root);
        return EXIT_FAILURE;
    }
}
