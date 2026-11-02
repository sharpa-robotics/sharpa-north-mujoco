// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/simulator_service.h"
#include "simulator_config.h"

#include <vector>

namespace sharpa::mujoco::simulator
{

// Read-only host checks implied by a resolved simulator configuration.
std::vector<SimulatorEnvironmentIssue> inspect_simulator_environment(
    const SimulatorConfig& config);

}  // namespace sharpa::mujoco::simulator
