// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/export.h"

#include <memory>
#include <string>
#include <vector>

namespace sharpa::mujoco
{

class Runtime;

namespace simulator
{

struct SimulatorConfig;

enum class SimulatorServiceState
{
    Unconfigured,
    Configured,
    Running,
    Faulted,
};

struct SimulatorDeviceStatus
{
    std::string name;
    bool running = false;
};

enum class SimulatorEnvironmentIssueCode
{
    SocketCanMissing,
    SocketCanDown,
    SocketCanUnusable,
};

struct SimulatorEnvironmentIssue
{
    SimulatorEnvironmentIssueCode code =
        SimulatorEnvironmentIssueCode::SocketCanUnusable;
    std::string resource;
    std::string detail;
    bool can_prepare_automatically = false;
};

// Embeds the production device simulators around a caller-owned Runtime.
// The Runtime must be loaded and paused while configuring the service, and
// must outlive the service. stop() is idempotent.
// Configuration applies simulation.snapshot_rate_hz to the Runtime while
// preserving the caller's other Runtime options.
class SHARPA_MUJOCO_API SimulatorService
{
public:
    SimulatorService();
    ~SimulatorService();

    SimulatorService(const SimulatorService&) = delete;
    SimulatorService& operator=(const SimulatorService&) = delete;

    bool configure(
        Runtime& runtime,
        const std::string& config_path,
        bool force_loopback_torso,
        std::string* error = nullptr);
    // Internal resolved-config entry point used by the standalone CLI after it
    // applies command-line overrides.  Normal embedders should use the path
    // overload so configuration ownership stays inside sharpa_mujoco.
    bool configure(
        Runtime& runtime,
        const SimulatorConfig& config,
        bool force_loopback_torso,
        std::string* error = nullptr);
    bool start(Runtime& runtime, std::string* error = nullptr);
    void stop() noexcept;

    SimulatorServiceState state() const noexcept;
    std::vector<SimulatorEnvironmentIssue> environment_issues() const;
    std::vector<SimulatorDeviceStatus> device_status() const;
    std::string last_error() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace simulator
}  // namespace sharpa::mujoco
