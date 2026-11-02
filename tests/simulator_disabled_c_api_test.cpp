// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/c_api.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>

namespace
{
bool require(bool condition, const char* message, const char* detail = nullptr)
{
    if (condition) return true;
    std::cerr << "FAILED: " << message;
    if (detail && detail[0] != '\0') std::cerr << ": " << detail;
    std::cerr << '\n';
    return false;
}

bool reports_disabled(int result, const char* error)
{
    return result == 0 && std::strstr(error, "disabled") &&
        std::strstr(error, "SHARPA_MUJOCO_BUILD_EMBEDDED_SIMULATOR=ON");
}
}

int main()
{
    std::unique_ptr<sharpa_mujoco_runtime, decltype(&sharpa_mujoco_runtime_destroy)> runtime(
        sharpa_mujoco_runtime_create(), sharpa_mujoco_runtime_destroy);
    char error[512] = {};
    if (!require(runtime != nullptr, "runtime allocation") ||
        !require(sharpa_mujoco_runtime_load(
            runtime.get(), SHARPA_MUJOCO_TEST_MODEL, error, sizeof(error)) != 0,
            "core model load", error)) return EXIT_FAILURE;

    // Reject embedding before trying to resolve a default or user config file.
    for (const char* config : {static_cast<const char*>(nullptr), "", "/missing/simulator.conf"})
    {
        if (!require(reports_disabled(sharpa_mujoco_runtime_configure_simulator(
                runtime.get(), config, error, sizeof(error)), error),
                "disabled simulator configuration", error)) return EXIT_FAILURE;
    }
    if (!require(reports_disabled(sharpa_mujoco_runtime_start_simulator(
            runtime.get(), error, sizeof(error)), error),
            "disabled simulator start", error)) return EXIT_FAILURE;

    int state = SHARPA_MUJOCO_SIMULATOR_RUNNING;
    size_t device_count = 10;
    size_t running_count = 10;
    if (!require(reports_disabled(sharpa_mujoco_runtime_simulator_status(
            runtime.get(), &state, &device_count, &running_count, error, sizeof(error)), error) &&
            state == SHARPA_MUJOCO_SIMULATOR_UNCONFIGURED &&
            device_count == 0 && running_count == 0,
            "disabled status must not report success or stale devices", error)) return EXIT_FAILURE;

    char name[32] = "stale device";
    int running = 1;
    if (!require(sharpa_mujoco_runtime_simulator_device_info(
            runtime.get(), 0, name, sizeof(name), &running) == 0 &&
            name[0] == '\0' && running == 0,
            "disabled device query")) return EXIT_FAILURE;

    size_t issue_count = 10;
    if (!require(reports_disabled(sharpa_mujoco_runtime_simulator_environment_status(
            runtime.get(), &issue_count, error, sizeof(error)), error) && issue_count == 0,
            "disabled environment status", error)) return EXIT_FAILURE;

    int code = SHARPA_MUJOCO_ENVIRONMENT_SOCKETCAN_MISSING;
    int can_prepare = 1;
    char resource[32] = "stale interface";
    if (!require(reports_disabled(sharpa_mujoco_runtime_simulator_environment_issue(
            runtime.get(), 0, &code, &can_prepare, resource, sizeof(resource),
            error, sizeof(error)), error) &&
            code == 0 && can_prepare == 0 && resource[0] == '\0',
            "disabled environment issue", error)) return EXIT_FAILURE;

    // Disabled entry points remain safe for callers omitting output buffers.
    char short_error[2] = {'?', '?'};
    if (!require(
            sharpa_mujoco_runtime_configure_simulator(nullptr, nullptr, nullptr, 0) == 0 &&
            sharpa_mujoco_runtime_start_simulator(nullptr, short_error, sizeof(short_error)) == 0 &&
            short_error[0] != '\0' && short_error[1] == '\0' &&
            sharpa_mujoco_runtime_simulator_status(
                nullptr, nullptr, nullptr, nullptr, nullptr, 0) == 0 &&
            sharpa_mujoco_runtime_simulator_device_info(nullptr, 0, nullptr, 0, nullptr) == 0 &&
            sharpa_mujoco_runtime_simulator_environment_status(nullptr, nullptr, nullptr, 0) == 0 &&
            sharpa_mujoco_runtime_simulator_environment_issue(
                nullptr, 0, nullptr, nullptr, nullptr, 0, nullptr, 0) == 0,
            "disabled APIs tolerate null and short buffers")) return EXIT_FAILURE;

    sharpa_mujoco_runtime_stop_simulator(nullptr);
    sharpa_mujoco_runtime_stop_simulator(runtime.get());
    sharpa_mujoco_runtime_stop_simulator(runtime.get());
    if (!require(sharpa_mujoco_runtime_step(runtime.get(), 2, error, sizeof(error)) != 0,
            "core simulation still works after rejected device calls", error)) return EXIT_FAILURE;
    sharpa_mujoco_runtime_unload(runtime.get());
    if (!require(sharpa_mujoco_runtime_load(
            runtime.get(), SHARPA_MUJOCO_TEST_MODEL, error, sizeof(error)) != 0,
            "core model reload", error)) return EXIT_FAILURE;

    std::cout << "Disabled embedded simulator C ABI test passed.\n";
    return EXIT_SUCCESS;
}
