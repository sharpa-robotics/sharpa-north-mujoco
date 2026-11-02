// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/arm/arm_udp_codec.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sharpa::mujoco::plugins::arm
{
namespace
{
constexpr std::size_t kHeaderSize = 4;
constexpr std::size_t kCrcSize = 2;

std::uint16_t read_u16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>(
        p[0] | (static_cast<std::uint16_t>(p[1]) << 8U));
}

std::uint32_t read_u32(const std::uint8_t* p)
{
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8U) |
           (static_cast<std::uint32_t>(p[2]) << 16U) |
           (static_cast<std::uint32_t>(p[3]) << 24U);
}

std::int16_t read_i16(const std::uint8_t* p)
{
    return static_cast<std::int16_t>(read_u16(p));
}

std::int32_t read_i32(const std::uint8_t* p)
{
    return static_cast<std::int32_t>(read_u32(p));
}

void append_u16(std::vector<std::uint8_t>& out, std::uint16_t value)
{
    out.push_back(static_cast<std::uint8_t>(value));
    out.push_back(static_cast<std::uint8_t>(value >> 8U));
}

void append_u32(std::vector<std::uint8_t>& out, std::uint32_t value)
{
    out.push_back(static_cast<std::uint8_t>(value));
    out.push_back(static_cast<std::uint8_t>(value >> 8U));
    out.push_back(static_cast<std::uint8_t>(value >> 16U));
    out.push_back(static_cast<std::uint8_t>(value >> 24U));
}

template <typename T>
T scaled(float value, float scale)
{
    const double finite = std::isfinite(value) ? value : 0.0;
    return static_cast<T>(std::llround(std::clamp(
        finite * scale,
        static_cast<double>(std::numeric_limits<T>::min()),
        static_cast<double>(std::numeric_limits<T>::max()))));
}

void append_axis(std::vector<std::uint8_t>& out, const AxisFeedback& axis)
{
    out.push_back(ArmUdpCodec::kFeedbackCommand);
    out.push_back(axis.node_id);
    out.push_back(static_cast<std::uint8_t>(axis.mode));
    append_u16(out, axis.status_word);
    append_u32(out, static_cast<std::uint32_t>(
                        scaled<std::int32_t>(axis.position_rad, 1000000.0F)));
    append_u16(out, static_cast<std::uint16_t>(
                        scaled<std::int16_t>(axis.velocity_rad_s, 1000.0F)));
    append_u16(out, static_cast<std::uint16_t>(
                        scaled<std::int16_t>(axis.torque_nm, 100.0F)));
    append_u16(out, axis.fault_code);
    append_u32(out, static_cast<std::uint32_t>(scaled<std::int32_t>(
                        axis.low_speed_position_rad, 1000000.0F)));
    append_u16(out, static_cast<std::uint16_t>(scaled<std::int16_t>(
                        axis.low_speed_velocity_rad_s, 1000.0F)));
}
}  // namespace

std::uint16_t ArmUdpCodec::crc16_ccitt_false(
    const std::uint8_t* data, std::size_t size)
{
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= static_cast<std::uint16_t>(data[i]) << 8U;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x8000U) != 0U
                      ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                      : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

bool ArmUdpCodec::decode_control(
    const std::uint8_t* data, std::size_t size, DecodedControl& out)
{
    const bool single = size == kHeaderSize + kControlAxisSize + kCrcSize;
    const bool dual = size == kHeaderSize + 2 * kControlAxisSize + kCrcSize;
    if (!data || (!single && !dual) ||
        read_u16(data + size - kCrcSize) !=
            crc16_ccitt_false(data, size - kCrcSize))
    {
        return false;
    }
    DecodedControl decoded;
    decoded.sequence = read_u32(data);
    decoded.axis_count = single ? 1 : 2;
    for (std::size_t i = 0; i < decoded.axis_count; ++i)
    {
        const std::uint8_t* p = data + kHeaderSize + i * kControlAxisSize;
        if (p[0] != kControlCommand)
        {
            return false;
        }
        AxisControl& axis = decoded.axes[i];
        axis.node_id = p[1];
        axis.mode = static_cast<std::int8_t>(p[2]);
        axis.control_word = read_u16(p + 3);
        axis.position_rad = static_cast<float>(read_i32(p + 5)) / 1000000.0F;
        axis.velocity_rad_s = static_cast<float>(read_i16(p + 9)) / 1000.0F;
        axis.torque_nm = static_cast<float>(read_i16(p + 11)) / 100.0F;
        axis.kp = static_cast<float>(read_u16(p + 13)) / 100.0F;
        axis.kd = static_cast<float>(read_u16(p + 15)) / 1000.0F;
    }
    out = decoded;
    return true;
}

bool ArmUdpCodec::encode_feedback(
    std::uint32_t sequence,
    const AxisFeedback* axes,
    std::size_t axis_count,
    std::uint64_t timestamp_ns,
    std::vector<std::uint8_t>& out,
    bool extended)
{
    if (!axes || (axis_count != 1 && axis_count != 2))
    {
        out.clear();
        return false;
    }
    out.clear();
    out.reserve(kHeaderSize + axis_count *
        (extended ? kExtendedFeedbackAxisSize : kFeedbackAxisSize) + kCrcSize + 8);
    append_u32(out, sequence);
    for (std::size_t i = 0; i < axis_count; ++i)
    {
        append_axis(out, axes[i]);
        if (extended)
        {
            out.push_back(static_cast<std::uint8_t>(scaled<std::int8_t>(axes[i].bus_voltage_v, 1)));
            out.push_back(static_cast<std::uint8_t>(scaled<std::int8_t>(axes[i].driver_temperature_c, 1)));
            out.push_back(static_cast<std::uint8_t>(scaled<std::int8_t>(axes[i].motor_temperature_c, 1)));
            append_u16(out, static_cast<std::uint16_t>(scaled<std::int16_t>(axes[i].torque_sensor_nm, 100)));
            out.insert(out.end(), 4, 0);
        }
    }
    append_u16(out, crc16_ccitt_false(out.data(), out.size()));
    append_u32(out, static_cast<std::uint32_t>(timestamp_ns));
    append_u32(out, static_cast<std::uint32_t>(timestamp_ns >> 32U));
    return true;
}

bool ArmUdpCodec::respond_sdo(const std::uint8_t* data, std::size_t size,
                             std::size_t axis_count, std::vector<std::uint8_t>& out)
{
    out.clear();
    if (!data || size != 10 || data[0] != 0x03 ||
        read_u16(data + 8) != crc16_ccitt_false(data, 8)) return false;
    out.assign(data, data + 4);
    const auto index = read_u16(data + 2);
    // Noise-free virtual high/low-speed encoders have no CRC errors. These
    // counters describe encoder links, NOT malformed host UDP packets.
    const bool supported = data[1] < axis_count && index >= 0x30AE && index <= 0x30B1;
    if (!supported) out[0] |= 0x80;
    append_u32(out, supported ? 0 : 2);  // Illegal address exception.
    append_u16(out, crc16_ccitt_false(out.data(), out.size()));
    return true;
}

}  // namespace sharpa::mujoco::plugins::arm
