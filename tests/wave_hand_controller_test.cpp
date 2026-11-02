// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "wave_hand_controller.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

namespace
{

using sharpa::mujoco::plugins::wave_hand::WaveControlMode;
using sharpa::mujoco::plugins::wave_hand::WaveHandController;
using sharpa::mujoco::plugins::wave_hand::WaveJointState;
using sharpa::mujoco::plugins::wave_hand::WaveJointTargets;
using sharpa::mujoco::plugins::wave_hand::kWaveJointCount;

constexpr float kPi = 3.14159265358979323846F;

bool require(bool condition, const char* message)
{
    if (!condition) std::cerr << "FAILED: " << message << '\n';
    return condition;
}

std::array<float, kWaveJointCount> filled(float value)
{
    std::array<float, kWaveJointCount> result{};
    result.fill(value);
    return result;
}

bool parameter_test()
{
    WaveHandController controller;
    auto kp = filled(7.5F);
    if (!require(controller.set_float_parameter(62, kp),
                 "MIT kp array was rejected."))
        return false;
    std::array<float, kWaveJointCount> readback{};
    if (!require(controller.get_float_parameter(62, readback) && readback == kp,
                 "MIT kp array did not round trip."))
        return false;

    auto invalid = filled(8.0F);
    invalid[12] = std::numeric_limits<float>::quiet_NaN();
    if (!require(!controller.set_float_parameter(62, invalid),
                 "NaN parameter was accepted.") ||
        !require(controller.get_float_parameter(62, readback) && readback == kp,
                 "Rejected parameter write was not atomic."))
        return false;

    auto zero_mass = filled(0.0F);
    if (!require(controller.set_float_parameter(46, zero_mass),
                 "Firmware-valid zero mass array was rejected."))
        return false;

    if (!require(controller.set_scalar_parameter(0, 2.0F) &&
                     controller.set_scalar_parameter(64, 0.5F),
                 "Controller scalar parameters were rejected."))
        return false;
    float scalar = 0.0F;
    if (!require(controller.get_scalar_parameter(64, scalar) &&
                     std::fabs(scalar - 0.5F) < 1e-6F,
                 "Velocity filter alpha did not round trip."))
        return false;
    std::array<float, kWaveJointCount> soft_minimum{};
    if (!require(controller.get_readonly_float_parameter(36, soft_minimum) &&
                     std::fabs(soft_minimum[0] + 5.0F * kPi / 180.0F) < 1e-6F,
                 "Read-only soft limit array is wrong."))
        return false;
    WaveJointState state;
    state.effort.fill(1.0F);
    const auto output =
        controller.update(WaveControlMode::Admittance, 0.001F, 1.0F, state);
    if (!require(std::isfinite(output.value[0]),
                 "Zero admittance mass produced a non-finite target."))
        return false;

    std::array<std::uint8_t, kWaveJointCount> enabled{};
    enabled[5] = 1;
    enabled[9] = 1;
    if (!require(controller.set_u8_parameter(51, enabled),
                 "Friction enable array was rejected."))
        return false;
    enabled[3] = 2;
    return require(!controller.set_u8_parameter(51, enabled),
                   "Out-of-range friction enable was accepted.");
}

bool differential_limit_test()
{
    WaveHandController controller;
    std::array<float, kWaveJointCount> positions{};
    positions[0] = -49.0F * kPi / 180.0F;
    const auto first = controller.position_limits(1, positions);
    positions[0] = -48.0F * kPi / 180.0F;
    const auto second = controller.position_limits(1, positions);
    positions[0] = -48.5F * kPi / 180.0F;
    const auto half = controller.position_limits(1, positions);
    if (!require(std::fabs(half[0] - (first[0] + second[0]) * 0.5F) < 1e-6F &&
                     std::fabs(half[1] - (first[1] + second[1]) * 0.5F) < 1e-6F,
                 "Differential limit interpolation is wrong."))
        return false;
    positions[0] = 130.0F * kPi / 180.0F;
    const auto last = controller.position_limits(1, positions);
    positions[0] = 200.0F * kPi / 180.0F;
    const auto above = controller.position_limits(1, positions);
    if (!require(last == above,
                 "Differential limit did not clamp above the last table row."))
        return false;

    positions[0] = -48.5F * kPi / 180.0F;
    WaveJointTargets targets;
    targets.position[0] = positions[0];
    targets.position[1] = 1.0F;
    controller.reset(positions);
    controller.set_targets(targets);
    WaveJointState state;
    state.position = positions;
    const auto output =
        controller.update(WaveControlMode::Position, 0.01F, 1.0F, state);
    return require(output.value[1] <= half[1] + 1e-6F,
                   "Difference target exceeded its partner-dependent limit.");
}

bool mit_friction_test()
{
    WaveHandController controller;
    std::array<float, kWaveJointCount> positions{};
    controller.reset(positions);
    WaveJointTargets targets;
    targets.position[5] = 0.02F;
    controller.set_targets(targets);
    WaveJointState state;
    const auto enabled =
        controller.update(WaveControlMode::Mit, 0.001F, 1.0F, state);

    std::array<std::uint8_t, kWaveJointCount> friction{};
    if (!controller.set_u8_parameter(51, friction)) return false;
    const auto disabled =
        controller.update(WaveControlMode::Mit, 0.001F, 1.0F, state);
    if (!require(enabled.value[5] > disabled.value[5],
                 "Enabled MIT friction did not add positive breakaway torque."))
        return false;

    state.velocity[5] = -0.2F;
    friction[5] = 1;
    if (!controller.set_u8_parameter(51, friction)) return false;
    const auto moving =
        controller.update(WaveControlMode::Mit, 0.001F, 1.0F, state);
    friction[5] = 0;
    if (!controller.set_u8_parameter(51, friction)) return false;
    const auto moving_without =
        controller.update(WaveControlMode::Mit, 0.001F, 1.0F, state);
    if (!require(moving.value[5] < moving_without.value[5],
                 "Moving MIT friction did not follow velocity direction."))
        return false;

    WaveHandController filtered;
    filtered.reset({});
    WaveJointTargets filtered_targets;
    filtered_targets.velocity[5] = -0.2F;
    filtered.set_targets(filtered_targets);
    WaveJointState filtered_state;
    filtered_state.velocity[5] = -0.2F;
    if (!filtered.set_scalar_parameter(64, 0.001F)) return false;
    const auto filtered_output =
        filtered.update(WaveControlMode::Mit, 0.001F, 1.0F, filtered_state);
    WaveHandController unfiltered;
    unfiltered.reset({});
    unfiltered.set_targets(filtered_targets);
    const auto unfiltered_output =
        unfiltered.update(WaveControlMode::Mit, 0.001F, 1.0F, filtered_state);
    return require(std::fabs(filtered_output.value[5]) <
                       std::fabs(unfiltered_output.value[5]),
                   "Velocity filter alpha did not affect MIT friction input.");
}

bool dip_torque_damping_test()
{
    WaveHandController controller;
    controller.reset({});
    WaveJointTargets targets;
    targets.velocity[8] = 1.0F;
    targets.velocity[7] = 1.0F;
    controller.set_targets(targets);
    WaveJointState state;
    state.velocity[8] = 1.0F;
    state.velocity[7] = 1.0F;
    const auto output =
        controller.update(WaveControlMode::Mit, 0.001F, 1.0F, state);
    return require(output.value[8] < output.value[7] - 0.009F,
                   "DIP open-loop torque damping was not applied.");
}

bool admittance_limit_test()
{
    WaveHandController controller;
    std::array<float, kWaveJointCount> positions{};
    positions[0] = 104.0F * kPi / 180.0F;
    controller.reset(positions);
    WaveJointTargets targets;
    targets.position = positions;
    controller.set_targets(targets);
    WaveJointState state;
    state.position = positions;
    // Negative effort increases reference-delta toward the upper limit.
    state.effort[0] = -1.0F;
    float previous = positions[0];
    for (int i = 0; i < 500; ++i)
    {
        const auto output =
            controller.update(WaveControlMode::Admittance, 0.001F, 1.0F, state);
        previous = output.value[0];
        state.position[0] = previous;
    }
    return require(std::isfinite(previous) &&
                       previous > 105.0F * kPi / 180.0F &&
                       previous <= 113.0F * kPi / 180.0F + 1e-4F,
                   "Admittance soft-to-hard limit behavior is wrong.");
}

bool admittance_stiffness_test()
{
    const auto displacement = [](float coefficient) {
        WaveHandController controller;
        std::array<float, kWaveJointCount> positions{};
        controller.reset(positions);
        controller.set_targets({});
        controller.set_scalar_parameter(0, coefficient);
        WaveJointState state;
        state.effort[0] = 0.5F;
        float target = 0.0F;
        for (int i = 0; i < 200; ++i)
        {
            const auto output = controller.update(
                WaveControlMode::Admittance, 0.001F, 1.0F, state);
            target = output.value[0];
            state.position[0] = target;
        }
        return std::fabs(target);
    };
    return require(displacement(10.0F) < displacement(0.0F),
                   "Stiffness coefficient did not reduce admittance displacement.");
}

}  // namespace

int main()
{
    return parameter_test() && differential_limit_test() &&
                   mit_friction_test() && dip_torque_damping_test() &&
                   admittance_limit_test() &&
                   admittance_stiffness_test()
               ? 0
               : 1;
}
