// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "model_assembler.h"
#include "lidar_config.h"
#include "simulator_config.h"

#include "sharpa_mujoco/runtime.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
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

}  // namespace

int main(int argc, char** argv)
{
    std::string config_path;
    std::string model_override;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if ((argument == "--config" || argument == "--model") &&
            index + 1 < argc)
        {
            const std::string value = argv[++index];
            if (argument == "--config")
            {
                config_path = value;
            }
            else
            {
                model_override = value;
            }
            continue;
        }
        std::cerr << "Usage: " << argv[0]
                  << " --config <path> [--model <path>]\n";
        return EXIT_FAILURE;
    }
    if (config_path.empty())
    {
        std::cerr << "Usage: " << argv[0]
                  << " --config <path> [--model <path>]\n";
        return EXIT_FAILURE;
    }

    sharpa::mujoco::simulator::SimulatorConfig config =
        sharpa::mujoco::simulator::default_config();
    std::string error;
    if (!require(
            sharpa::mujoco::simulator::load_config_file(
                config_path, config, &error),
            error))
    {
        return EXIT_FAILURE;
    }
    if (!model_override.empty())
    {
        config.model_path = model_override;
    }
    std::vector<sharpa::mujoco::simulator::LidarMount> mounts;
    if (!require(
            sharpa::mujoco::simulator::parse_lidar_mounts(
                config.devices, mounts, &error),
            error) ||
        !require(!mounts.empty(), "Configuration contains no LiDAR devices."))
    {
        return EXIT_FAILURE;
    }

    sharpa::mujoco::Runtime runtime;
    if (!require(
            runtime.load(config.model_path, &error),
            error))
    {
        return EXIT_FAILURE;
    }

    sharpa::mujoco::simulator::LidarMount rejected = mounts.front();
    rejected.instance_prefix = "rollback_";
    rejected.instance.site_name = "rollback_ray_origin";
    rejected.sensor_model = "does-not-exist.xml";
    if (!require(
            !sharpa::mujoco::simulator::attach_lidar_mounts(
                runtime, {rejected}, &error),
            "Invalid LiDAR model attachment unexpectedly succeeded."))
    {
        return EXIT_FAILURE;
    }
    if (!require(
            error.find("Could not parse LiDAR sensor_model") != std::string::npos &&
                runtime.last_error() == error,
            "Transactional assembly replaced the detailed callback error."))
    {
        return EXIT_FAILURE;
    }
    bool found_rollback = false;
    runtime.read_native([&](const mjModel* model, const mjData*) {
        found_rollback =
            mj_name2id(model, mjOBJ_SITE, "rollback_ray_origin") >= 0;
    });
    if (!require(
            !found_rollback,
            "Failed LiDAR attachment left a site in the active model."))
    {
        return EXIT_FAILURE;
    }

    if (!require(
            sharpa::mujoco::simulator::attach_lidar_mounts(
                runtime, mounts, &error),
            error))
    {
        return EXIT_FAILURE;
    }

    bool found_all = true;
    runtime.read_native([&](const mjModel* model, const mjData*) {
        found_all = std::all_of(
            mounts.begin(),
            mounts.end(),
            [&](const auto& mount) {
                return mj_name2id(
                           model,
                           mjOBJ_SITE,
                           mount.instance.site_name.c_str()) >= 0;
            });
    });
    return require(found_all, "LiDAR ray origin sites were not attached.")
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
