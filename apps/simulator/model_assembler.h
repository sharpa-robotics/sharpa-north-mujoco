// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/plugins/lidar/lidar_plugin.h"

#include <array>
#include <string>
#include <vector>

namespace sharpa::mujoco
{
class Runtime;
}

namespace sharpa::mujoco::simulator
{

struct LidarMount
{
    plugins::lidar::LidarInstanceConfig instance;
    std::string parent_body;
    std::string instance_prefix;
    std::string local_site_name = "ray_origin";
    std::string sensor_model;
    std::array<double, 3> position{0.0, 0.0, 0.0};
    std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};
};

struct TactileMount
{
    std::string parent_body;
    std::string instance_prefix;
    std::string local_site_name = "frame";
    std::string sensor_model;
    std::array<double, 3> position{0.0, 0.0, 0.0};
    std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};
};

bool attach_lidar_mounts(
    Runtime& runtime,
    const std::vector<LidarMount>& mounts,
    std::string* error = nullptr);

bool attach_tactile_mounts(
    Runtime& runtime,
    const std::vector<TactileMount>& mounts,
    std::string* error = nullptr);

}  // namespace sharpa::mujoco::simulator
