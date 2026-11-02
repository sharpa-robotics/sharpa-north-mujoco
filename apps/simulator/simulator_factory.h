// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

namespace sharpa::mujoco
{
class DevicePluginManager;
class Runtime;
}

namespace sharpa::mujoco::simulator
{
struct SimulatorConfig;

bool configure_device_plugins(
    const SimulatorConfig& config,
    Runtime& runtime,
    DevicePluginManager& plugins,
    std::string* error = nullptr);
}
