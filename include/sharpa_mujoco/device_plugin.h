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

class SHARPA_MUJOCO_API IDevicePlugin
{
public:
    virtual ~IDevicePlugin() = default;

    virtual std::string name() const = 0;
    virtual bool start(Runtime& runtime, std::string* error = nullptr) = 0;
    // Must be safe after a partial/failed start and when called repeatedly.
    virtual void stop() noexcept = 0;
    virtual bool is_running() const noexcept = 0;
};

class SHARPA_MUJOCO_API DevicePluginManager
{
public:
    DevicePluginManager() = default;
    ~DevicePluginManager();

    DevicePluginManager(const DevicePluginManager&) = delete;
    DevicePluginManager& operator=(const DevicePluginManager&) = delete;

    bool add(std::unique_ptr<IDevicePlugin> plugin, std::string* error = nullptr);
    bool start_all(Runtime& runtime, std::string* error = nullptr);
    void stop_all() noexcept;
    IDevicePlugin* find(const std::string& name) const noexcept;
    std::size_t size() const noexcept;

private:
    std::vector<std::unique_ptr<IDevicePlugin>> plugins_;
    std::size_t started_count_ = 0;
};

}  // namespace sharpa::mujoco
