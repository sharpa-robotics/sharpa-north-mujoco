// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace sharpa::mujoco::plugins::arm
{

struct AxisControl
{
    std::uint8_t node_id = 0;
    std::int8_t mode = 0;
    std::uint16_t control_word = 0;
    float position_rad = 0.0F;
    float velocity_rad_s = 0.0F;
    float torque_nm = 0.0F;
    float kp = 0.0F;
    float kd = 0.0F;
};

struct AxisFeedback
{
    std::uint8_t node_id = 0;
    std::int8_t mode = 0;
    std::uint16_t status_word = 0;
    float position_rad = 0.0F;
    float velocity_rad_s = 0.0F;
    float torque_nm = 0.0F;
    std::uint16_t fault_code = 0;
    float low_speed_position_rad = 0.0F;
    float low_speed_velocity_rad_s = 0.0F;
    float bus_voltage_v = 48.0F;
    float driver_temperature_c = 25.0F;
    float motor_temperature_c = 25.0F;
    float torque_sensor_nm = 0.0F;
};

struct DecodedControl
{
    std::uint32_t sequence = 0;
    std::size_t axis_count = 0;
    std::array<AxisControl, 2> axes{};
};

class ArmUdpCodec
{
public:
    static constexpr std::uint8_t kControlCommand = 0x20;
    static constexpr std::uint8_t kFeedbackCommand = 0x18;
    static constexpr std::size_t kControlAxisSize = 17;
    static constexpr std::size_t kFeedbackAxisSize = 21;
    static constexpr std::size_t kExtendedFeedbackAxisSize = 30;

    static std::uint16_t crc16_ccitt_false(
        const std::uint8_t* data, std::size_t size);
    static bool decode_control(
        const std::uint8_t* data, std::size_t size, DecodedControl& out);
    static bool encode_feedback(
        std::uint32_t sequence,
        const AxisFeedback* axes,
        std::size_t axis_count,
        std::uint64_t timestamp_ns,
        std::vector<std::uint8_t>& out,
        bool extended = false);
    // Read-only encoder diagnostics, 10-byte platform SDO service.
    static bool respond_sdo(const std::uint8_t* data, std::size_t size,
                            std::size_t axis_count, std::vector<std::uint8_t>& out);
};

}  // namespace sharpa::mujoco::plugins::arm
