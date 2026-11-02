// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "lidar_config.h"

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace
{

bool require(bool value, const std::string& message)
{
    if (!value)
    {
        std::cerr << message << '\n';
    }
    return value;
}

sharpa::mujoco::simulator::DeviceConfig valid_ftx()
{
    sharpa::mujoco::simulator::DeviceConfig device;
    device.enabled = true;
    device.driver = "hesai_ftx140";
    device.options = {
        {"parent_body", "lower_body_link_5"},
        {"site_name", "ray_origin"},
        {"sensor_model", SHARPA_MUJOCO_FTX_SENSOR_MODEL},
        {"calibration_file", SHARPA_MUJOCO_FTX_CALIBRATION},
        {"raycast_exclude_meshes", "mesh_front, mesh_shared"},
        {"raycast_threads", "1"},
        {"mount_pos", "0,0,0"},
        {"mount_quat", "1,0,0,0"},
        {"bind_port", "45980"},
        {"peer_port", "9180"},
        {"scan_rate_hz", "10"},
        {"min_range_m", "0.1"},
        {"max_range_m", "60"},
    };
    return device;
}

}  // namespace

int main()
{
    std::map<std::string, sharpa::mujoco::simulator::DeviceConfig> devices;
    devices.emplace("lidar", valid_ftx());
    std::vector<sharpa::mujoco::simulator::LidarMount> mounts;
    std::string error;
    if (!require(
            sharpa::mujoco::simulator::parse_lidar_mounts(devices, mounts, &error),
            error) ||
        !require(
            mounts.size() == 1 &&
                mounts.front().sensor_model == SHARPA_MUJOCO_FTX_SENSOR_MODEL &&
                mounts.front().instance_prefix == "lidar_" &&
                mounts.front().instance.site_name == "lidar_ray_origin" &&
                mounts.front().instance.raycast_threads == 1 &&
                mounts.front().instance.raycast_exclude_meshes ==
                    std::vector<std::string>{"mesh_front", "mesh_shared"},
            "Valid LiDAR configuration was not preserved."))
    {
        return EXIT_FAILURE;
    }
    devices["lidar"] = valid_ftx();
    devices["lidar"].options["raycast_threads"] = "-1";
    if (!require(
            !sharpa::mujoco::simulator::parse_lidar_mounts(
                devices, mounts, &error) &&
                error.find("raycast_threads") != std::string::npos,
            "Negative LiDAR raycast thread count was accepted."))
    {
        return EXIT_FAILURE;
    }
    devices["lidar"] = valid_ftx();
    devices["lidar"].options["bind_port"] = "70000";
    if (!require(
               !sharpa::mujoco::simulator::parse_lidar_mounts(
                   devices, mounts, &error) &&
                   error.find("bind_port") != std::string::npos,
               "Out-of-range LiDAR port was accepted."))
    {
        return EXIT_FAILURE;
    }
    devices["lidar"] = valid_ftx();
    devices["lidar"].options["instance_prefix"] = "legacy_";
    return require(
               !sharpa::mujoco::simulator::parse_lidar_mounts(
                   devices, mounts, &error) &&
                   error.find("deprecated") != std::string::npos,
               "Deprecated instance_prefix was accepted.")
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
