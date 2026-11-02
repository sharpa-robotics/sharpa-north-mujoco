// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <map>
#include <string>

namespace sharpa::mujoco::simulator
{

struct DeviceConfig
{
    bool enabled = false;
    std::string driver;
    std::map<std::string, std::string> options;
};

struct ViewerSettings
{
    bool enabled = true;
    double sync_rate_hz = 60.0;
    bool vsync = true;
};

struct SimulatorConfig
{
    std::string model_path;
    double snapshot_rate_hz = 200.0;
    ViewerSettings viewer;
    std::map<std::string, DeviceConfig> devices;
};

SimulatorConfig default_config();

bool load_config_file(
    const std::string& path,
    SimulatorConfig& config,
    std::string* error = nullptr);

}  // namespace sharpa::mujoco::simulator
