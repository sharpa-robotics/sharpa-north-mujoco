// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/device_plugin.h"
#include "sharpa_mujoco/runtime.h"

#include <iostream>
#include <stdexcept>
#include <utility>

namespace
{
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

class PartialPlugin : public sharpa::mujoco::IDevicePlugin
{
public:
    PartialPlugin(std::string label, std::vector<std::string>& events)
        : label_(std::move(label)), events_(events) {}
    std::string name() const override { return label_; }
    bool start(sharpa::mujoco::Runtime&, std::string* error) override
    {
        events_.push_back("start:" + label_);
        running_ = true;  // Simulate resources acquired before a later failure.
        if (failure == 1) { if (error) *error = "partial start"; return false; }
        if (failure == 2) throw std::runtime_error("partial start");
        if (failure == 3) throw 42;
        return true;
    }
    void stop() noexcept override
    {
        events_.push_back("stop:" + label_);
        running_ = false;
    }
    bool is_running() const noexcept override { return running_; }
    int failure = 0;
private:
    std::string label_;
    std::vector<std::string>& events_;
    bool running_ = false;
};
}

int main()
{
    try
    {
        for (int failure : {1, 2, 3})
        {
            for (int failing_index : {0, 1})
            {
                sharpa::mujoco::Runtime runtime;
                std::vector<std::string> events;
                sharpa::mujoco::DevicePluginManager manager;
                std::vector<PartialPlugin*> plugins;
                std::string error;
                for (const char* name : {"first", "second", "last"})
                {
                    auto plugin = std::make_unique<PartialPlugin>(name, events);
                    plugins.push_back(plugin.get());
                    require(manager.add(std::move(plugin), &error), "Could not add plugin");
                }
                plugins[failing_index]->failure = failure;
                require(!manager.start_all(runtime, &error) && !error.empty(), "Failure was not reported");
                const std::vector<std::string> expected = failing_index == 0
                    ? std::vector<std::string>{"start:first", "stop:first"}
                    : std::vector<std::string>{"start:first", "start:second", "stop:second", "stop:first"};
                require(events == expected, "Rollback missed current plugin or stopped in wrong order");
                for (auto* plugin : plugins) require(!plugin->is_running(), "Leaked running plugin");
                manager.stop_all();
                require(events == expected, "Rollback left stale manager bookkeeping");
                plugins[failing_index]->failure = 0;
                events.clear();
                require(manager.start_all(runtime, &error), "Could not restart after failed start");
                manager.stop_all();
                require(events == std::vector<std::string>{"start:first", "start:second", "start:last",
                            "stop:last", "stop:second", "stop:first"}, "Restart/stop order is wrong");
            }
        }
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
