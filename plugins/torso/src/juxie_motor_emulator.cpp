// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/torso/juxie_motor_emulator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace sharpa::mujoco::plugins::torso
{
namespace
{

constexpr double kMitPositionRangeRad = 3.14;
constexpr double kMitVelocityRangeRadS = 3.14;
constexpr double kMitKpMax = 2500.0;
constexpr double kMitKdMax = 500.0;
constexpr double kMitTorqueMaxNm = 300.0;
constexpr double kRadSToRpm = 30.0 / 3.14159265358979323846;
constexpr double kFeedbackRadSPerLsb = 0.01;

template <typename T>
T round_clamped(double value)
{
    return static_cast<T>(std::lround(std::clamp(
        value,
        static_cast<double>(std::numeric_limits<T>::min()),
        static_cast<double>(std::numeric_limits<T>::max()))));
}

void write_be_i16(std::uint8_t* data, std::int16_t value)
{
    common::write_be16(data, static_cast<std::uint16_t>(value));
}

double decode_symmetric(
    std::uint16_t raw,
    std::uint16_t raw_max,
    double range)
{
    return static_cast<double>(raw) / static_cast<double>(raw_max) *
               (2.0 * range) -
           range;
}

}  // namespace

JuxieMotorEmulator::JuxieMotorEmulator(std::uint8_t motor_id)
    : motor_id_(motor_id)
{
}

std::uint8_t JuxieMotorEmulator::motor_id() const noexcept
{
    return motor_id_;
}

bool JuxieMotorEmulator::enabled() const noexcept
{
    return enabled_;
}

bool JuxieMotorEmulator::brake_released() const noexcept
{
    return brake_released_;
}

std::uint8_t JuxieMotorEmulator::control_mode() const noexcept
{
    return control_mode_;
}

std::uint16_t JuxieMotorEmulator::heartbeat_period_ms() const noexcept
{
    return heartbeat_period_ms_;
}

std::int16_t JuxieMotorEmulator::target_position() const noexcept
{
    return target_position_;
}

const JuxieMotorEmulator::MitCommand&
JuxieMotorEmulator::mit_command() const noexcept
{
    return mit_command_;
}

void JuxieMotorEmulator::power_cycle() noexcept
{
    const auto torque = saved_torque_params_;
    const auto notch = saved_notch_params_;
    *this = JuxieMotorEmulator(motor_id_);
    torque_params_ = saved_torque_params_ = torque;
    notch_params_ = saved_notch_params_ = notch;
}

std::uint32_t JuxieMotorEmulator::read_object(std::uint16_t index) const noexcept
{
    switch (index)
    {
        case 0x6041:
            return enabled_ ? 0x0027U : 0x0040U;
        case 0x603F:
            return 0;
        case 0x6064:
            return static_cast<std::uint32_t>(
                static_cast<std::int32_t>(feedback_position_));
        case 0x606C:
            return static_cast<std::uint32_t>(velocity_rpm_);
        case 0x6078:
            return 0;  // No electrical current model.
        case 0x1017:
            return heartbeat_period_ms_;
        case 0x2542:
            return static_cast<std::uint32_t>(
                std::lround(kMitPositionRangeRad / 0.01));
        case 0x2543:
            return static_cast<std::uint32_t>(
                std::lround(kMitVelocityRangeRadS / 0.01));
        case 0x2544:
            return static_cast<std::uint32_t>(kMitKpMax);
        case 0x2545:
            return static_cast<std::uint32_t>(kMitKdMax);
        case 0x2546:
            return static_cast<std::uint32_t>(kMitTorqueMaxNm);
        default:
            return 0;
    }
}

std::optional<common::CanFdFrame> JuxieMotorEmulator::process(
    const common::CanFdFrame& request)
{
    if (request.can_id != 0x600U + motor_id_ || request.data.size() < 8)
    {
        return std::nullopt;
    }
    const std::uint8_t command = request.data[0];
    const std::uint16_t index = static_cast<std::uint16_t>(
        request.data[1] | (static_cast<std::uint16_t>(request.data[2]) << 8U));
    const std::uint8_t subindex = request.data[3];

    common::CanFdFrame response;
    response.can_interface = request.can_interface;
    response.can_id = 0x580U + motor_id_;
    response.can_flags = request.can_flags;
    response.data.assign(8, 0);
    response.data[1] = request.data[1];
    response.data[2] = request.data[2];
    response.data[3] = subindex;

    const auto abort = [&](std::uint32_t code) {
        response.data[0] = 0x80;
        common::write_le32(response.data.data() + 4, code);
        return std::optional<common::CanFdFrame>(response);
    };
    std::size_t object_width = 4;
    bool writable = false;
    bool disabled_only = false;
    std::uint32_t* parameter = nullptr;
    std::uint32_t minimum = 0, maximum = 0xFFFFFFFFU;
    if (index == 0x2548 || index == 0x2547)
    {
        const bool torque = index == 0x2548;
        if (subindex == 0 || subindex > (torque ? 6 : 4))
            return abort(0x06090011U);
        const std::size_t slot = subindex - 1;
        parameter = torque ? &torque_params_[slot] : &notch_params_[slot];
        writable = true;
        disabled_only = !torque || (subindex != 4 && subindex != 6);
        if (torque)
        {
            constexpr std::array<std::size_t, 6> widths{2, 2, 4, 4, 1, 4};
            constexpr std::array<std::uint32_t, 6> mins{1000, 0, 1000, 0, 0, 0};
            constexpr std::array<std::uint32_t, 6> maxs{20000, 2000, 1000000, 100000, 15, 10000000};
            object_width = widths[slot]; minimum = mins[slot]; maximum = maxs[slot];
        }
        else
        {
            constexpr std::array<std::uint32_t, 4> mins{0, 100, 50, 0};
            constexpr std::array<std::uint32_t, 4> maxs{1, 45000, 5000, 100};
            minimum = mins[slot]; maximum = maxs[slot];
        }
    }
    else
    {
        switch (index)
        {
            case 0x6040: case 0x1017:
                object_width = 2; writable = true; break;
            case 0x6041: case 0x603F: case 0x6078:
                object_width = 2; break;
            case 0x6064: case 0x606C:
            case 0x2542: case 0x2543: case 0x2544: case 0x2545: case 0x2546:
                break;
            case 0x1010: case 0x1011:
                writable = true; disabled_only = index == 0x1011; break;
            default: return abort(0x06020000U);
        }
        if (subindex != (index == 0x1010 ? 1 : index == 0x1011 ? 3 : 0))
            return abort(0x06090011U);
    }

    if (command == 0x40)
    {
        if (index == 0x6040) return abort(0x06010001U);
        const std::uint32_t value = parameter ? *parameter :
            (index == 0x1010 || index == 0x1011 ? 1 : read_object(index));
        response.data[0] = object_width == 1 ? 0x4F : object_width == 2 ? 0x4B : 0x43;
        common::write_le32(response.data.data() + 4, value);
        return response;
    }

    std::size_t width = 0;
    switch (command)
    {
        case 0x2F:
            width = 1;
            break;
        case 0x2B:
            width = 2;
            break;
        case 0x27:
            width = 3;
            break;
        case 0x23:
            width = 4;
            break;
        default:
            return abort(0x05040001U);
    }

    if (!writable) return abort(0x06010002U);
    if (width != object_width) return abort(0x06070010U);
    if (disabled_only && enabled_) return abort(0x08000022U);
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < width; ++i)
        value |= static_cast<std::uint32_t>(request.data[4 + i]) << (8U * i);
    if (value < minimum || value > maximum) return abort(0x06090030U);
    if (parameter)
    {
        *parameter = value;
        // These are two views of the same firmware notch-enable bit.
        if (index == 0x2547 && subindex == 1)
            torque_params_[4] = (torque_params_[4] & ~8U) | (value ? 8U : 0U);
        if (index == 0x2548 && subindex == 5)
            notch_params_[0] = (value >> 3U) & 1U;
    }
    if (index == 0x1010)
    {
        if (value != 0x65766173U) return abort(0x06090030U);
        saved_torque_params_ = torque_params_;
        saved_torque_params_[3] = 0;  // Jl is RAM-only.
        saved_notch_params_ = notch_params_;
    }
    if (index == 0x1011)
    {
        if (value != 0x64616F6CU) return abort(0x06090030U);
        const JuxieMotorEmulator defaults(motor_id_);
        torque_params_ = saved_torque_params_ = defaults.torque_params_;
        notch_params_ = saved_notch_params_ = defaults.notch_params_;
    }

    if (index == 0x1017 && width >= 2)
    {
        heartbeat_period_ms_ = static_cast<std::uint16_t>(
            request.data[4] | (static_cast<std::uint16_t>(request.data[5]) << 8U));
    }
    else if (index == 0x6040 && width >= 1)
    {
        if (value != 0x80 && value != 0x0F && value != 0x4F &&
            value != 0x07 && value != 0x06)
            return abort(0x06090030U);
        const std::uint8_t word = request.data[4];
        if (word == 0x80)
        {
            enabled_ = false;
        }
        else if (word == 0x0F || word == 0x4F)
        {
            enabled_ = true;
            brake_released_ = true;
        }
        else if (word == 0x07)
        {
            enabled_ = false;
            brake_released_ = true;
        }
        else if (word == 0x06)
        {
            enabled_ = false;
            brake_released_ = false;
        }
    }
    response.data[0] = 0x60;
    return response;
}

bool JuxieMotorEmulator::apply_csp_slot(
    const std::uint8_t* slot,
    std::size_t size)
{
    if (!slot || size < 7)
    {
        return false;
    }
    const std::uint8_t control = slot[0];
    enabled_ = (control & 0x80U) != 0;
    brake_released_ = (control & 0x40U) != 0;
    control_mode_ = static_cast<std::uint8_t>((control >> 1U) & 0x0FU);
    if ((control & 0x20U) != 0)
    {
        enabled_ = false;
    }
    target_position_ = static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(slot[1]) << 8U) | slot[2]);
    return control_mode_ == 0x03U;
}

bool JuxieMotorEmulator::apply_mit_slot(
    const std::uint8_t* slot,
    std::size_t size)
{
    if (!slot || size < 9)
    {
        return false;
    }
    const std::uint8_t control = slot[0];
    enabled_ = (control & 0x80U) != 0;
    brake_released_ = (control & 0x40U) != 0;
    control_mode_ = static_cast<std::uint8_t>((control >> 1U) & 0x0FU);
    if ((control & 0x20U) != 0)
    {
        enabled_ = false;
    }
    const std::uint16_t position_raw =
        (static_cast<std::uint16_t>(slot[1]) << 8U) | slot[2];
    const std::uint16_t velocity_raw =
        (static_cast<std::uint16_t>(slot[3]) << 4U) | (slot[4] >> 4U);
    const std::uint16_t kp_raw =
        (static_cast<std::uint16_t>(slot[4] & 0x0FU) << 8U) | slot[5];
    const std::uint16_t kd_raw =
        (static_cast<std::uint16_t>(slot[6]) << 4U) | (slot[7] >> 4U);
    const std::uint16_t torque_raw =
        (static_cast<std::uint16_t>(slot[7] & 0x0FU) << 8U) | slot[8];
    mit_command_ = {
        decode_symmetric(position_raw, 65535U, kMitPositionRangeRad),
        decode_symmetric(velocity_raw, 4095U, kMitVelocityRangeRadS),
        static_cast<double>(kp_raw) / 4095.0 * kMitKpMax,
        static_cast<double>(kd_raw) / 4095.0 * kMitKdMax,
        decode_symmetric(torque_raw, 4095U, kMitTorqueMaxNm),
    };
    return control_mode_ == 0x06U;
}

void JuxieMotorEmulator::set_feedback(
    std::int16_t position,
    double motor_velocity_rad_s,
    std::int16_t torque_raw,
    std::int32_t encoder_raw)
{
    feedback_position_ = position;
    // Inverse of Platform JuxieBodyProtocol::FeedbackVelocityRawToRpm:
    // IDs 1-5 use 0.01 rotor rad/s per LSB; IDs 6-8 use rotor RPM.
    const double velocity_raw = motor_id_ >= 1 && motor_id_ <= 5
        ? motor_velocity_rad_s / kFeedbackRadSPerLsb
        : motor_velocity_rad_s * kRadSToRpm;
    feedback_velocity_raw_ = round_clamped<std::int16_t>(velocity_raw);
    // SDO 0x606C is a separate int32 RPM field, even if the PDO saturates.
    velocity_rpm_ = round_clamped<std::int32_t>(motor_velocity_rad_s * kRadSToRpm);
    torque_raw_ = torque_raw;
    encoder_raw_ = std::max(-0x800000, std::min(0x7FFFFF, encoder_raw));
}

common::CanFdFrame JuxieMotorEmulator::feedback_frame(
    std::uint8_t can_interface) const
{
    common::CanFdFrame frame;
    frame.can_interface = can_interface;
    frame.can_id = 0x300U + motor_id_;
    frame.can_flags = common::kCanFdFlagBrs;
    frame.data.assign(16, 0);
    write_be_i16(frame.data.data(), feedback_position_);
    write_be_i16(frame.data.data() + 2, feedback_velocity_raw_);
    write_be_i16(frame.data.data() + 4, 0);
    write_be_i16(frame.data.data() + 6, 0);
    write_be_i16(frame.data.data() + 8, 250);
    write_be_i16(frame.data.data() + 10, torque_raw_);
    const std::uint32_t encoder =
        static_cast<std::uint32_t>(encoder_raw_) & 0x00FFFFFFU;
    frame.data[12] = static_cast<std::uint8_t>(encoder >> 16U);
    frame.data[13] = static_cast<std::uint8_t>(encoder >> 8U);
    frame.data[14] = static_cast<std::uint8_t>(encoder);
    frame.data[15] =
        static_cast<std::uint8_t>((enabled_ ? 0x80U : 0U) |
                                  (brake_released_ ? 0x40U : 0U) |
                                  control_mode_);
    return frame;
}

common::CanFdFrame JuxieMotorEmulator::heartbeat_frame(
    std::uint8_t can_interface) const
{
    common::CanFdFrame frame;
    frame.can_interface = can_interface;
    frame.can_id = 0x700U + motor_id_;
    frame.data = {static_cast<std::uint8_t>(enabled_ ? 0x05U : 0x7FU)};
    return frame;
}

}  // namespace sharpa::mujoco::plugins::torso
