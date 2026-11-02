// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/c_api.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

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

bool snapshot_rate_test()
{
    char error[4096] = {};
    std::unique_ptr<sharpa_mujoco_runtime, decltype(&sharpa_mujoco_runtime_destroy)> runtime(
        sharpa_mujoco_runtime_create(), sharpa_mujoco_runtime_destroy);
    if (!require(runtime != nullptr, "snapshot runtime allocation") ||
        !require(sharpa_mujoco_runtime_load(runtime.get(), SHARPA_MUJOCO_TEST_MODEL,
                     error, sizeof(error)) != 0, "snapshot model load", error)) return false;

    const int bodies = sharpa_mujoco_runtime_body_count(runtime.get());
    std::vector<double> positions(bodies * 3), quaternions(bodies * 4);
    // The minimal model steps at 500 Hz. A 200 Hz snapshot request is rounded
    // to three physics steps (6 ms). Reconfigure the same paused Runtime to
    // verify both configuration propagation and that the rate is not hardcoded.
    for (const auto& test : {std::pair<const char*, uint64_t>{SHARPA_MUJOCO_SNAPSHOT_CONFIG, 1},
                            std::pair<const char*, uint64_t>{SHARPA_MUJOCO_TORSO_CONFIG, 3}})
    {
        if (!require(sharpa_mujoco_runtime_configure_simulator(runtime.get(), test.first,
                         error, sizeof(error)) != 0, "snapshot configuration", error) ||
            !require(sharpa_mujoco_runtime_start(runtime.get(), error, sizeof(error)) != 0,
                         "snapshot runtime start", error)) return false;

        // Do not start devices: this checks the same embedded C ABI used by UE
        // without binding ports or contacting Platform. Compare step/version
        // deltas, not wall-clock timing, so scheduler delays do not affect it.
        uint64_t initial_version = 0, first_version = 0, first_step = 0, version = 0, step = 0;
        double simulation_time = 0.0;
        bool copied = true;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline)
        {
            copied = sharpa_mujoco_runtime_copy_body_pose(runtime.get(), &version, &step,
                &simulation_time, positions.data(), positions.size(),
                quaternions.data(), quaternions.size()) != 0;
            if (!copied) break;
            if (initial_version == 0) initial_version = version;
            if (first_version == 0 && version > initial_version)
            {
                first_version = version;
                first_step = step;
            }
            if (first_version != 0 && version - first_version >= 20) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        sharpa_mujoco_runtime_pause(runtime.get());
        if (!require(copied && first_version != 0 && version - first_version >= 20,
                     "snapshot publication progressed", error)) return false;
        if (!require(step - first_step == test.second * (version - first_version),
                     "embedded snapshot rate matches configured rate"))
        {
            std::cerr << "steps=" << step - first_step << " snapshots=" << version - first_version
                      << " expected steps/snapshot=" << test.second << '\n';
            return false;
        }
    }
    return true;
}
}

int main()
{
    if (!snapshot_rate_test()) return EXIT_FAILURE;
    char error[4096] = {};
    sharpa_mujoco_runtime* runtime = sharpa_mujoco_runtime_create();
    if (!require(runtime != nullptr, "runtime allocation")) return EXIT_FAILURE;

    bool valid = require(
        sharpa_mujoco_runtime_load(
            runtime, SHARPA_MUJOCO_NTH_HA1_MODEL, error, sizeof(error)) != 0,
        "whole-body model load", error);
    int state = -1;
    size_t device_count = 0;
    size_t running_count = 0;
    valid = valid && require(
        sharpa_mujoco_runtime_configure_simulator(
            runtime, SHARPA_MUJOCO_TORSO_CONFIG, error, sizeof(error)) != 0,
        "loopback simulator configuration", error);
    valid = valid && require(
        sharpa_mujoco_runtime_simulator_status(
            runtime, &state, &device_count, &running_count, error, sizeof(error)) != 0 &&
            state == SHARPA_MUJOCO_SIMULATOR_CONFIGURED &&
            device_count == 1 && running_count == 0,
        "loopback simulator status", error);

    for (int cycle = 0; valid && cycle < 2; ++cycle)
    {
        valid = require(
            sharpa_mujoco_runtime_start_simulator(runtime, error, sizeof(error)) != 0,
            "simulator start", error);
        valid = valid && require(
            sharpa_mujoco_runtime_simulator_status(
                runtime, &state, &device_count, &running_count, error, sizeof(error)) != 0 &&
                state == SHARPA_MUJOCO_SIMULATOR_RUNNING &&
                device_count == 1 && running_count == device_count,
            "running simulator status", error);
        if (cycle == 0)
        {
            valid = valid && require(
                sharpa_mujoco_runtime_start(runtime, error, sizeof(error)) != 0,
                "runtime start", error);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            sharpa_mujoco_runtime_pause(runtime);
        }
        sharpa_mujoco_runtime_stop_simulator(runtime);
    }

    // Direct consumers bypassing a launcher still get the device's actionable
    // error unchanged through the service and C API, with no partial startup.
    valid = valid && require(
        sharpa_mujoco_runtime_configure_simulator(
            runtime, SHARPA_MUJOCO_MISSING_CAN_CONFIG, error, sizeof(error)) != 0,
        "missing CAN configuration", error);
    size_t environment_issue_count = 0;
    valid = valid && require(
        sharpa_mujoco_runtime_simulator_environment_status(
            runtime, &environment_issue_count, error, sizeof(error)) != 0 &&
            environment_issue_count == 1,
        "structured environment preflight", error);
    int issue_code = 0;
    int can_prepare = 0;
    char issue_resource[64] = {};
    char issue_detail[1024] = {};
    valid = valid && require(
        sharpa_mujoco_runtime_simulator_environment_issue(
            runtime, 0, &issue_code, &can_prepare,
            issue_resource, sizeof(issue_resource),
            issue_detail, sizeof(issue_detail)) != 0 &&
            issue_code == SHARPA_MUJOCO_ENVIRONMENT_SOCKETCAN_MISSING &&
            can_prepare != 0 &&
            std::strcmp(issue_resource, "vcan_missing") == 0 &&
            std::strstr(issue_detail, "vcan_missing") != nullptr,
        "structured CAN remediation", issue_detail);
    valid = valid && require(
        sharpa_mujoco_runtime_start_simulator(runtime, error, sizeof(error)) == 0 &&
            std::strstr(error, "vcan_missing") != nullptr &&
            std::strstr(error, "setup_vcan.sh") != nullptr,
        "CAN recovery instructions propagate to direct consumers", error);
    valid = valid && require(
        sharpa_mujoco_runtime_simulator_status(
            runtime, &state, &device_count, &running_count, error, sizeof(error)) != 0 &&
            state == SHARPA_MUJOCO_SIMULATOR_CONFIGURED && running_count == 0,
        "failed CAN startup leaves no running devices", error);

    sharpa_mujoco_runtime_destroy(runtime);
    if (!valid) return EXIT_FAILURE;
    std::cout << "Embedded simulator C ABI lifecycle test passed.\n";
    return EXIT_SUCCESS;
}
