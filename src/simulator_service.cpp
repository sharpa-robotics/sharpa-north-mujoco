// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/simulator_service.h"

#include "sharpa_mujoco/device_plugin.h"
#include "sharpa_mujoco/runtime.h"
#include "simulator_config.h"
#include "simulator_environment.h"
#include "simulator_factory.h"

#include <exception>
#include <memory>
#include <optional>
#include <utility>

namespace sharpa::mujoco::simulator
{

class SimulatorService::Impl
{
public:
    bool configure(
        Runtime& runtime,
        const std::string& config_path,
        bool force_loopback_torso,
        std::string* error)
    {
        stop();
        manager_ = std::make_unique<DevicePluginManager>();
        device_names_.clear();
        config_.reset();
        last_error_.clear();
        state_ = SimulatorServiceState::Unconfigured;

        if (!runtime.is_loaded())
        {
            return fail("Simulator service requires a loaded Runtime.", error);
        }
        if (runtime.is_running())
        {
            return fail("Simulator service must be configured while the Runtime is paused.", error);
        }
        if (config_path.empty())
        {
            return fail("Simulator configuration path must not be empty.", error);
        }

        SimulatorConfig config = default_config();
        std::string detail;
        if (!load_config_file(config_path, config, &detail))
        {
            return fail(detail, error);
        }
        return configure(runtime, config, force_loopback_torso, error);
    }

    bool configure(
        Runtime& runtime,
        SimulatorConfig config,
        bool force_loopback_torso,
        std::string* error)
    {
        stop();
        manager_ = std::make_unique<DevicePluginManager>();
        device_names_.clear();
        config_.reset();
        last_error_.clear();
        state_ = SimulatorServiceState::Unconfigured;

        if (!runtime.is_loaded())
        {
            return fail("Simulator service requires a loaded Runtime.", error);
        }
        if (runtime.is_running())
        {
            return fail("Simulator service must be configured while the Runtime is paused.", error);
        }
        std::string detail;
        if (force_loopback_torso)
        {
            auto torso = config.devices.find("torso");
            if (torso != config.devices.end())
            {
                torso->second.driver = "udp";
            }
        }

        if (!configure_device_plugins(config, runtime, *manager_, &detail))
        {
            manager_.reset();
            return fail(detail, error);
        }

        // Embedders create their Runtime before loading the simulator config.
        // Apply the same snapshot cadence as the standalone CLI, preserving
        // the caller's scheduling and other Runtime options.
        RuntimeConfig runtime_config = runtime.config();
        runtime_config.snapshot_rate_hz = config.snapshot_rate_hz;
        if (!runtime.set_config(runtime_config, &detail))
        {
            manager_.reset();
            return fail(detail, error);
        }

        for (const auto& [name, device] : config.devices)
        {
            if (device.enabled)
            {
                device_names_.push_back(name);
            }
        }
        config_ = std::move(config);
        state_ = SimulatorServiceState::Configured;
        return true;
    }

    bool start(Runtime& runtime, std::string* error)
    {
        if (state_ == SimulatorServiceState::Running)
        {
            return true;
        }
        if (state_ != SimulatorServiceState::Configured || !manager_)
        {
            return fail("Simulator service is not configured.", error);
        }
        if (!runtime.is_loaded() || runtime.is_running())
        {
            return fail(
                "Simulator devices must start after model load and before Runtime start.",
                error);
        }
        const std::vector<SimulatorEnvironmentIssue> issues = environment_issues();
        if (!issues.empty())
        {
            std::string detail = issues.front().detail;
            if (issues.front().can_prepare_automatically)
            {
                detail += " Run: sudo bash scripts/setup_vcan.sh " +
                    issues.front().resource + "; then restart simulation.";
            }
            else
            {
                detail +=
                    " Prepare the configured CAN interface before starting simulation.";
            }
            last_error_ = std::move(detail);
            if (error) *error = last_error_;
            return false;
        }
        std::string detail;
        if (!manager_->start_all(runtime, &detail))
        {
            return fail(detail, error);
        }
        last_error_.clear();
        state_ = SimulatorServiceState::Running;
        return true;
    }

    void stop() noexcept
    {
        if (manager_)
        {
            manager_->stop_all();
        }
        if (state_ != SimulatorServiceState::Unconfigured)
        {
            state_ = SimulatorServiceState::Configured;
        }
    }

    SimulatorServiceState state() const noexcept { return state_; }

    std::vector<SimulatorEnvironmentIssue> environment_issues() const
    {
        return config_
            ? inspect_simulator_environment(*config_)
            : std::vector<SimulatorEnvironmentIssue>{};
    }

    std::vector<SimulatorDeviceStatus> device_status() const
    {
        std::vector<SimulatorDeviceStatus> result;
        result.reserve(device_names_.size());
        for (const std::string& name : device_names_)
        {
            result.push_back({name, state_ == SimulatorServiceState::Running});
        }
        return result;
    }

    const std::string& last_error() const noexcept { return last_error_; }

private:
    bool fail(const std::string& detail, std::string* error)
    {
        last_error_ = detail.empty() ? "Unknown simulator service error." : detail;
        state_ = SimulatorServiceState::Faulted;
        if (error)
        {
            *error = last_error_;
        }
        return false;
    }

    std::unique_ptr<DevicePluginManager> manager_;
    std::optional<SimulatorConfig> config_;
    std::vector<std::string> device_names_;
    SimulatorServiceState state_ = SimulatorServiceState::Unconfigured;
    std::string last_error_;
};

SimulatorService::SimulatorService() : impl_(std::make_unique<Impl>()) {}
SimulatorService::~SimulatorService() = default;

bool SimulatorService::configure(
    Runtime& runtime,
    const std::string& config_path,
    bool force_loopback_torso,
    std::string* error)
{
    try
    {
        return impl_->configure(
            runtime,
            config_path,
            force_loopback_torso,
            error);
    }
    catch (const std::exception& exception)
    {
        if (error) *error = exception.what();
        return false;
    }
    catch (...)
    {
        if (error) *error = "Unknown simulator service exception.";
        return false;
    }
}

bool SimulatorService::configure(
    Runtime& runtime,
    const SimulatorConfig& config,
    bool force_loopback_torso,
    std::string* error)
{
    try
    {
        return impl_->configure(runtime, config, force_loopback_torso, error);
    }
    catch (const std::exception& exception)
    {
        if (error) *error = exception.what();
        return false;
    }
    catch (...)
    {
        if (error) *error = "Unknown simulator service exception.";
        return false;
    }
}

bool SimulatorService::start(Runtime& runtime, std::string* error)
{
    try
    {
        return impl_->start(runtime, error);
    }
    catch (const std::exception& exception)
    {
        if (error) *error = exception.what();
        return false;
    }
    catch (...)
    {
        if (error) *error = "Unknown simulator service exception.";
        return false;
    }
}

void SimulatorService::stop() noexcept { impl_->stop(); }
SimulatorServiceState SimulatorService::state() const noexcept { return impl_->state(); }
std::vector<SimulatorEnvironmentIssue> SimulatorService::environment_issues() const
{
    try
    {
        return impl_->environment_issues();
    }
    catch (...)
    {
        return {};
    }
}
std::vector<SimulatorDeviceStatus> SimulatorService::device_status() const
{
    return impl_->device_status();
}
std::string SimulatorService::last_error() const { return impl_->last_error(); }

}  // namespace sharpa::mujoco::simulator
