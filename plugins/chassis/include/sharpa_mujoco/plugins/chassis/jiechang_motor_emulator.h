// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace sharpa::mujoco::plugins::chassis
{

class JiechangMotorEmulator
{
public:
    explicit JiechangMotorEmulator(std::uint8_t motor_id = 1);

    std::uint8_t motor_id() const noexcept;
    bool enabled() const noexcept;
    std::int8_t mode() const noexcept;
    bool brake_released() const noexcept;
    std::uint16_t control_word() const noexcept;
    std::int16_t target_torque_permille() const noexcept;
    std::int32_t target_position_count() const noexcept;
    std::int32_t target_velocity_count() const noexcept;
    std::uint32_t mit_kp() const noexcept;
    std::uint32_t mit_kd() const noexcept;

    std::optional<common::CanFdFrame> process_sdo(
        const common::CanFdFrame& request);
    bool apply_pdo1_slot(const std::uint8_t* slot, std::size_t size);
    bool apply_pdo2_slot(const std::uint8_t* slot, std::size_t size);
    bool apply_pdo3_slot(const std::uint8_t* slot, std::size_t size);

    void set_feedback(
        std::int32_t position_count,
        std::int32_t velocity_count,
        std::int16_t torque_permille = 0,
        std::int16_t current_permille = 0);
    common::CanFdFrame feedback_frame(std::uint8_t can_interface) const;

    static constexpr double position_counts_per_revolution() noexcept
    {
        return 524288.0;
    }
    static constexpr double velocity_counts_per_rpm() noexcept
    {
        return 300.0 / 60.0;
    }

private:
    std::uint32_t read_object(std::uint16_t index) const noexcept;
    bool write_object(
        std::uint16_t index,
        std::uint8_t subindex,
        std::uint32_t value,
        std::size_t width) noexcept;

    std::uint8_t motor_id_ = 1;
    std::int8_t mode_ = 8;
    std::uint8_t release_brake_ = 1;
    std::uint16_t control_word_ = 0;
    std::uint16_t error_code_ = 0;
    std::int16_t target_torque_permille_ = 0;
    std::int32_t target_position_count_ = 0;
    std::int32_t target_velocity_count_ = 0;
    std::uint32_t mit_kp_ = 0;
    std::uint32_t mit_kd_ = 0;
    std::int32_t feedback_position_count_ = 0;
    std::int32_t feedback_velocity_count_ = 0;
    std::int16_t feedback_torque_permille_ = 0;
    std::int16_t feedback_current_permille_ = 0;
};

}  // namespace sharpa::mujoco::plugins::chassis
