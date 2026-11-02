// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "model_assembler.h"
#include "simulator_config.h"

#include <map>
#include <string>
#include <vector>

namespace sharpa::mujoco::simulator
{

bool parse_lidar_mounts(
    const std::map<std::string, DeviceConfig>& devices,
    std::vector<LidarMount>& mounts,
    std::string* error = nullptr);

}  // namespace sharpa::mujoco::simulator
