// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <mujoco/mujoco.h>

#include <cstddef>
#include <cstdint>
#include <functional>

namespace sharpa::mujoco
{

enum class StepDecision
{
    Proceed,
    Wait,
};

struct NativeStepContext
{
    std::uint64_t step_index = 0;
    std::size_t frame_substep_index = 0;
    std::size_t frame_substep_count = 1;
    double timestep = 0.0;
};

using NativeReadCallback = std::function<void(const mjModel*, const mjData*)>;
using NativeWriteCallback = std::function<void(const mjModel*, mjData*)>;
using NativeSpecReadCallback = std::function<void(const mjSpec*)>;
using NativeSpecEditCallback = std::function<bool(mjSpec*)>;
using NativeBeforeStepCallback =
    std::function<StepDecision(const mjModel*, mjData*, const NativeStepContext&)>;
using NativeAfterStepCallback =
    std::function<void(const mjModel*, mjData*, const NativeStepContext&)>;

}  // namespace sharpa::mujoco
