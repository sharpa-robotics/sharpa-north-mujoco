// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sharpa_mujoco/plugins/wave_hand/wave_protocol.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace sharpa::mujoco::plugins::wave_hand
{

enum class WaveControlMode : std::uint8_t
{
    Floating = 10,
    Position = 13,
    Admittance = 14,
    Mit = 15,
};

struct WaveJointTargets
{
    std::array<float, kWaveJointCount> position{};
    std::array<float, kWaveJointCount> velocity{};
    std::array<float, kWaveJointCount> torque{};
};

struct WaveJointState
{
    std::array<float, kWaveJointCount> position{};
    std::array<float, kWaveJointCount> velocity{};
    std::array<float, kWaveJointCount> effort{};
};

struct WaveControlOutput
{
    std::array<float, kWaveJointCount> value{};
    bool torque_mode = false;
};

class WaveHandController
{
public:
    WaveHandController();

    void reset(const std::array<float, kWaveJointCount>& positions);
    void set_targets(const WaveJointTargets& targets);

    WaveControlOutput update(WaveControlMode mode,
                             float timestep,
                             float speed_coefficient,
                             const WaveJointState& state);

    bool set_float_parameter(std::uint8_t id,
                             const std::array<float, kWaveJointCount>& values);
    bool get_float_parameter(
        std::uint8_t id,
        std::array<float, kWaveJointCount>& values) const;
    bool set_u8_parameter(
        std::uint8_t id,
        const std::array<std::uint8_t, kWaveJointCount>& values);
    bool get_u8_parameter(
        std::uint8_t id,
        std::array<std::uint8_t, kWaveJointCount>& values) const;
    bool set_scalar_parameter(std::uint8_t id, float value);
    bool get_scalar_parameter(std::uint8_t id, float& value) const;
    bool get_readonly_float_parameter(
        std::uint8_t id,
        std::array<float, kWaveJointCount>& values) const;

    // Exposed for deterministic controller tests and diagnostics.
    std::array<float, 2> position_limits(
        std::size_t joint,
        const std::array<float, kWaveJointCount>& positions) const;

private:
    struct Parameters
    {
        float mass = 0.001F;
        float damping = 0.25F;
        float stiffness = 3.6F;
        float torque_threshold = 0.12F;
        float speed_limit = 0.0F;
        std::uint8_t friction_enable = 0;
        float coulomb_friction = 0.1F;
        float viscous_friction = 0.001F;
        float static_friction = 0.25F;
        float stribeck_velocity = 0.02F;
        float mit_kp = 1.0F;
        float mit_kd = 0.01F;
        float torque_damping = 0.0F;
    };

    struct State
    {
        float applied_position = 0.0F;
        float applied_velocity = 0.0F;
        float admittance_position = 0.0F;
        float admittance_velocity = 0.0F;
        float filtered_velocity = 0.0F;
    };

    std::array<float, 2> differential_limits(std::size_t joint,
                                              float common_position) const;
    float limit_target(std::size_t joint,
                       float target,
                       const std::array<float, kWaveJointCount>& positions) const;
    float friction_compensation(std::size_t joint,
                                float velocity,
                                float position_error) const;
    float update_admittance(std::size_t joint,
                            float reference,
                            float effort,
                            float timestep);

    WaveJointTargets targets_{};
    std::array<Parameters, kWaveJointCount> parameters_{};
    std::array<State, kWaveJointCount> state_{};
    float stiffness_coefficient_ = 1.0F;
    float velocity_filter_alpha_ = 1.0F;
};

}  // namespace sharpa::mujoco::plugins::wave_hand
