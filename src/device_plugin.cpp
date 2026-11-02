// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/device_plugin.h"

#include "sharpa_mujoco/runtime.h"

#include <exception>
#include <utility>

namespace sharpa::mujoco
{

DevicePluginManager::~DevicePluginManager()
{
    stop_all();
}

bool DevicePluginManager::add(std::unique_ptr<IDevicePlugin> plugin, std::string* error)
{
    if (started_count_ != 0)
    {
        if (error)
        {
            *error = "Cannot add a device plugin while the manager is running.";
        }
        return false;
    }
    if (!plugin)
    {
        if (error)
        {
            *error = "Device plugin must not be null.";
        }
        return false;
    }
    if (plugin->name().empty())
    {
        if (error)
        {
            *error = "Device plugin name must not be empty.";
        }
        return false;
    }
    if (find(plugin->name()))
    {
        if (error)
        {
            *error = "Duplicate device plugin name: " + plugin->name();
        }
        return false;
    }
    plugins_.push_back(std::move(plugin));
    return true;
}

bool DevicePluginManager::start_all(Runtime& runtime, std::string* error)
{
    if (started_count_ != 0)
    {
        return true;
    }

    try
    {
        for (auto& plugin : plugins_)
        {
            std::string plugin_error;
            // Include this attempt in rollback even if start() partially
            // acquires resources and then returns false or throws.
            ++started_count_;
            if (!plugin->start(runtime, &plugin_error))
            {
                stop_all();
                if (error)
                {
                    *error = "Failed to start device plugin '" + plugin->name() +
                             "': " + plugin_error;
                }
                return false;
            }
        }
    }
    catch (const std::exception& exception)
    {
        stop_all();
        if (error)
        {
            *error = std::string("Device plugin start failed: ") + exception.what();
        }
        return false;
    }
    catch (...)
    {
        stop_all();
        if (error)
        {
            *error = "Device plugin start failed.";
        }
        return false;
    }
    return true;
}

void DevicePluginManager::stop_all() noexcept
{
    while (started_count_ > 0)
    {
        try
        {
            plugins_[--started_count_]->stop();
        }
        catch (...)
        {
        }
    }
}

IDevicePlugin* DevicePluginManager::find(const std::string& name) const noexcept
{
    for (const auto& plugin : plugins_)
    {
        if (plugin->name() == name)
        {
            return plugin.get();
        }
    }
    return nullptr;
}

std::size_t DevicePluginManager::size() const noexcept
{
    return plugins_.size();
}

}  // namespace sharpa::mujoco
