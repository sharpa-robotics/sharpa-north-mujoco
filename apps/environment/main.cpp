// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "simulator_config.h"
#include "simulator_environment.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace
{

void usage()
{
    std::cout << "Usage: sharpa_mujoco_environment --config PATH [--list-setup-vcan]\n"
                 "Performs read-only host checks required by the simulator configuration.\n";
}

}  // namespace

int main(int argc, char** argv)
{
    std::string config_path;
    bool list_setup_vcan = false;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--config" && index + 1 < argc)
        {
            config_path = argv[++index];
        }
        else if (argument == "--list-setup-vcan")
        {
            list_setup_vcan = true;
        }
        else if (argument == "--help" || argument == "-h")
        {
            usage();
            return EXIT_SUCCESS;
        }
        else
        {
            usage();
            return 2;
        }
    }
    if (config_path.empty())
    {
        usage();
        return 2;
    }

    sharpa::mujoco::simulator::SimulatorConfig config =
        sharpa::mujoco::simulator::default_config();
    std::string error;
    if (!sharpa::mujoco::simulator::load_config_file(config_path, config, &error))
    {
        std::cerr << error << '\n';
        return 2;
    }

    const auto issues =
        sharpa::mujoco::simulator::inspect_simulator_environment(config);
    if (issues.empty())
    {
        return EXIT_SUCCESS;
    }

    bool all_preparable = true;
    for (const auto& issue : issues)
    {
        if (list_setup_vcan && issue.can_prepare_automatically)
        {
            std::cout << issue.resource << '\n';
        }
        else
        {
            std::cerr << issue.detail << '\n';
        }
        all_preparable &= issue.can_prepare_automatically;
    }
    return list_setup_vcan && all_preparable ? 10 : EXIT_FAILURE;
}
