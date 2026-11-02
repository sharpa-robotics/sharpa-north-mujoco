// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "simulator_environment.h"

#include "sharpa_mujoco/plugins/common/socketcan_transport.h"

#include <set>
#include <string>
#include <utility>

namespace sharpa::mujoco::simulator
{

std::vector<SimulatorEnvironmentIssue> inspect_simulator_environment(
    const SimulatorConfig& config)
{
    std::vector<SimulatorEnvironmentIssue> result;
    std::set<std::string> checked;
    for (const auto& [device_name, device] : config.devices)
    {
        (void)device_name;
        if (!device.enabled || device.driver != "can")
        {
            continue;
        }
        for (const auto& [option, resource] : device.options)
        {
            constexpr const char* suffix = "_can_interface";
            const bool is_interface = option == "can_interface" ||
                (option.size() >= std::char_traits<char>::length(suffix) &&
                 option.compare(
                     option.size() - std::char_traits<char>::length(suffix),
                     std::char_traits<char>::length(suffix), suffix) == 0);
            if (!is_interface || !checked.insert(resource).second)
            {
                continue;
            }

            std::string detail;
            using plugins::common::SocketCanInterfaceState;
            const SocketCanInterfaceState state =
                plugins::common::inspect_socketcan_interface(resource, &detail);
            if (state == SocketCanInterfaceState::Ready)
            {
                continue;
            }

            SimulatorEnvironmentIssue issue;
            issue.resource = resource;
            issue.can_prepare_automatically =
                issue.resource.rfind("vcan", 0) == 0 &&
                issue.resource.find_first_not_of(
                    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-") ==
                    std::string::npos;
            switch (state)
            {
            case SocketCanInterfaceState::Missing:
                issue.code = SimulatorEnvironmentIssueCode::SocketCanMissing;
                break;
            case SocketCanInterfaceState::Down:
                issue.code = SimulatorEnvironmentIssueCode::SocketCanDown;
                break;
            case SocketCanInterfaceState::Unusable:
            case SocketCanInterfaceState::Ready:
                issue.code = SimulatorEnvironmentIssueCode::SocketCanUnusable;
                issue.can_prepare_automatically = false;
                break;
            }
            issue.detail = "SocketCAN interface '" + issue.resource + "' " +
                    (detail.empty() ? "is not ready" : detail) + ".";
            result.push_back(std::move(issue));
        }
    }
    return result;
}

}  // namespace sharpa::mujoco::simulator
