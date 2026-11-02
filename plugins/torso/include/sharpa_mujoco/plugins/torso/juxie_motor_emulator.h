// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"

#include <cstdint>
#include <array>
#include <optional>

namespace sharpa::mujoco::plugins::torso
{

class JuxieMotorEmulator
{
public:
    struct MitCommand
    {
        double position_rad = 0.0;
        double velocity_rad_s = 0.0;
        double kp_nm_per_rad = 0.0;
        double kd_nm_s_per_rad = 0.0;
        double torque_ff_nm = 0.0;
    };

    explicit JuxieMotorEmulator(std::uint8_t motor_id = 1);

    std::uint8_t motor_id() const noexcept;
    bool enabled() const noexcept;
    bool brake_released() const noexcept;
    std::uint8_t control_mode() const noexcept;
    std::uint16_t heartbeat_period_ms() const noexcept;
    std::int16_t target_position() const noexcept;
    const MitCommand& mit_command() const noexcept;
    // Reload the session-local saved parameter bank, clearing volatile state.
    // No host filesystem or real motor flash is written.
    void power_cycle() noexcept;

    std::optional<common::CanFdFrame> process(
        const common::CanFdFrame& request);
    bool apply_csp_slot(const std::uint8_t* slot, std::size_t size);
    bool apply_mit_slot(const std::uint8_t* slot, std::size_t size);

    // Velocity is rotor-side rad/s. PDO units depend on physical CAN motor ID;
    // SDO 0x606C continues to report rotor RPM.
    void set_feedback(
        std::int16_t position,
        double motor_velocity_rad_s,
        std::int16_t torque_raw,
        std::int32_t encoder_raw);
    common::CanFdFrame feedback_frame(std::uint8_t can_interface) const;
    common::CanFdFrame heartbeat_frame(std::uint8_t can_interface) const;

private:
    std::uint32_t read_object(std::uint16_t index) const noexcept;

    std::uint8_t motor_id_ = 1;
    bool enabled_ = false;
    bool brake_released_ = false;
    std::uint8_t control_mode_ = 0;
    std::uint16_t heartbeat_period_ms_ = 100;
    std::int16_t target_position_ = 0;
    MitCommand mit_command_;
    std::int16_t feedback_position_ = 0;
    std::int16_t feedback_velocity_raw_ = 0;
    std::int32_t velocity_rpm_ = 0;
    std::int16_t torque_raw_ = 0;
    std::int32_t encoder_raw_ = 0;
    std::array<std::uint32_t, 6> torque_params_{1200, 200, 35000, 0, 8, 0};
    std::array<std::uint32_t, 4> notch_params_{1, 1800, 400, 70};
    std::array<std::uint32_t, 6> saved_torque_params_ = torque_params_;
    std::array<std::uint32_t, 4> saved_notch_params_ = notch_params_;
};

}  // namespace sharpa::mujoco::plugins::torso
