// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/chassis/jiechang_motor_emulator.h"

namespace sharpa::mujoco::plugins::chassis
{
namespace
{

constexpr std::uint16_t kControlWord = 0x6040;
constexpr std::uint16_t kStatusWord = 0x6041;
constexpr std::uint16_t kErrorCode = 0x603F;
constexpr std::uint16_t kMode = 0x6060;
constexpr std::uint16_t kPositionActual = 0x6064;
constexpr std::uint16_t kVelocityActual = 0x606C;
constexpr std::uint16_t kCurrentActual = 0x6078;
constexpr std::uint16_t kTargetTorque = 0x6071;
constexpr std::uint16_t kTargetPosition = 0x607A;
constexpr std::uint16_t kTargetVelocity = 0x60FF;
constexpr std::uint16_t kHardwareVersion = 0x2005;
constexpr std::uint16_t kFirmwareVersion = 0x2006;
constexpr std::uint16_t kReleaseBrake = 0x4602;

std::uint16_t status_word(std::uint16_t control_word)
{
    return (control_word & 0x000FU) == 0x000FU ? 0x0027U : 0x0040U;
}

}  // namespace

JiechangMotorEmulator::JiechangMotorEmulator(std::uint8_t motor_id)
    : motor_id_(motor_id)
{
}

std::uint8_t JiechangMotorEmulator::motor_id() const noexcept { return motor_id_; }
bool JiechangMotorEmulator::enabled() const noexcept
{
    return (control_word_ & 0x000FU) == 0x000FU;
}
std::int8_t JiechangMotorEmulator::mode() const noexcept { return mode_; }
bool JiechangMotorEmulator::brake_released() const noexcept
{
    return release_brake_ != 0;
}
std::uint16_t JiechangMotorEmulator::control_word() const noexcept
{
    return control_word_;
}
std::int16_t JiechangMotorEmulator::target_torque_permille() const noexcept
{
    return target_torque_permille_;
}
std::int32_t JiechangMotorEmulator::target_position_count() const noexcept
{
    return target_position_count_;
}
std::int32_t JiechangMotorEmulator::target_velocity_count() const noexcept
{
    return target_velocity_count_;
}
std::uint32_t JiechangMotorEmulator::mit_kp() const noexcept { return mit_kp_; }
std::uint32_t JiechangMotorEmulator::mit_kd() const noexcept { return mit_kd_; }

std::uint32_t JiechangMotorEmulator::read_object(
    std::uint16_t index) const noexcept
{
    switch (index)
    {
        case kHardwareVersion:
            return 0x00010000U;
        case kFirmwareVersion:
            return 0x00010000U;
        case kErrorCode:
            return error_code_;
        case kControlWord:
            return control_word_;
        case kStatusWord:
            return status_word(control_word_);
        case kMode:
            return static_cast<std::uint8_t>(mode_);
        case kPositionActual:
            return static_cast<std::uint32_t>(feedback_position_count_);
        case kVelocityActual:
            return static_cast<std::uint32_t>(feedback_velocity_count_);
        case kCurrentActual:
            return static_cast<std::uint16_t>(feedback_current_permille_);
        case kTargetTorque:
            return static_cast<std::uint16_t>(target_torque_permille_);
        case kTargetPosition:
            return static_cast<std::uint32_t>(target_position_count_);
        case kTargetVelocity:
            return static_cast<std::uint32_t>(target_velocity_count_);
        case kReleaseBrake:
            return release_brake_;
        default:
            return 0;
    }
}

bool JiechangMotorEmulator::write_object(
    std::uint16_t index,
    std::uint8_t subindex,
    std::uint32_t value,
    std::size_t width) noexcept
{
    if (subindex != 0)
    {
        return false;
    }
    switch (index)
    {
        case kControlWord:
            control_word_ = static_cast<std::uint16_t>(value);
            if ((control_word_ & 0x0080U) != 0)
            {
                error_code_ = 0;
            }
            return width >= 2;
        case kMode:
            mode_ = static_cast<std::int8_t>(value);
            return width >= 1;
        case kReleaseBrake:
            release_brake_ = static_cast<std::uint8_t>(value);
            return width >= 1;
        case kTargetTorque:
            target_torque_permille_ = static_cast<std::int16_t>(value);
            return width >= 2;
        case kTargetPosition:
            target_position_count_ = static_cast<std::int32_t>(value);
            return width == 4;
        case kTargetVelocity:
            target_velocity_count_ = static_cast<std::int32_t>(value);
            return width == 4;
        default:
            return false;
    }
}

std::optional<common::CanFdFrame> JiechangMotorEmulator::process_sdo(
    const common::CanFdFrame& request)
{
    if (request.can_id != 0x600U + motor_id_ || request.data.size() != 8)
    {
        return std::nullopt;
    }
    const std::uint8_t command = request.data[0];
    const std::uint16_t index = common::read_le16(request.data.data() + 1);
    const std::uint8_t subindex = request.data[3];

    common::CanFdFrame response;
    response.can_interface = request.can_interface;
    response.can_id = 0x580U + motor_id_;
    response.can_flags = request.can_flags;
    response.data.assign(8, 0);
    response.data[1] = request.data[1];
    response.data[2] = request.data[2];
    response.data[3] = subindex;

    if (command == 0x40)
    {
        std::size_t width = 4;
        if (index == kMode || index == kReleaseBrake)
        {
            width = 1;
        }
        else if (
            index == kErrorCode || index == kControlWord ||
            index == kStatusWord || index == kCurrentActual ||
            index == kTargetTorque)
        {
            width = 2;
        }
        response.data[0] =
            width == 1 ? 0x4FU : (width == 2 ? 0x4BU : 0x43U);
        common::write_le32(response.data.data() + 4, read_object(index));
        return response;
    }

    const std::size_t width =
        command == 0x2FU ? 1U : (command == 0x2BU ? 2U :
        (command == 0x27U ? 3U : (command == 0x23U ? 4U : 0U)));
    if (width == 0 ||
        !write_object(
            index, subindex, common::read_le32(request.data.data() + 4), width))
    {
        response.data[0] = 0x80;
        common::write_le32(response.data.data() + 4, 0x06020000U);
        return response;
    }
    response.data[0] = 0x60;
    return response;
}

bool JiechangMotorEmulator::apply_pdo1_slot(
    const std::uint8_t* slot,
    std::size_t size)
{
    if (!slot || size < 6)
    {
        return false;
    }
    mode_ = static_cast<std::int8_t>(slot[0]);
    release_brake_ = slot[1];
    control_word_ = common::read_le16(slot + 2);
    target_torque_permille_ =
        static_cast<std::int16_t>(common::read_le16(slot + 4));
    return true;
}

bool JiechangMotorEmulator::apply_pdo2_slot(
    const std::uint8_t* slot,
    std::size_t size)
{
    if (!slot || size < 8)
    {
        return false;
    }
    target_position_count_ =
        static_cast<std::int32_t>(common::read_le32(slot));
    target_velocity_count_ =
        static_cast<std::int32_t>(common::read_le32(slot + 4));
    return true;
}

bool JiechangMotorEmulator::apply_pdo3_slot(
    const std::uint8_t* slot,
    std::size_t size)
{
    if (!slot || size < 8)
    {
        return false;
    }
    mit_kp_ = common::read_le32(slot);
    mit_kd_ = common::read_le32(slot + 4);
    return true;
}

void JiechangMotorEmulator::set_feedback(
    std::int32_t position_count,
    std::int32_t velocity_count,
    std::int16_t torque_permille,
    std::int16_t current_permille)
{
    feedback_position_count_ = position_count;
    feedback_velocity_count_ = velocity_count;
    feedback_torque_permille_ = torque_permille;
    feedback_current_permille_ = current_permille;
}

common::CanFdFrame JiechangMotorEmulator::feedback_frame(
    std::uint8_t can_interface) const
{
    common::CanFdFrame frame;
    frame.can_interface = can_interface;
    frame.can_id = 0x180U + motor_id_;
    frame.can_flags = common::kCanFdFlagBrs;
    frame.data.assign(16, 0);
    common::write_le16(frame.data.data(), error_code_);
    common::write_le16(frame.data.data() + 2, status_word(control_word_));
    common::write_le32(
        frame.data.data() + 4,
        static_cast<std::uint32_t>(feedback_position_count_));
    common::write_le32(
        frame.data.data() + 8,
        static_cast<std::uint32_t>(feedback_velocity_count_));
    common::write_le16(
        frame.data.data() + 12,
        static_cast<std::uint16_t>(feedback_torque_permille_));
    common::write_le16(
        frame.data.data() + 14,
        static_cast<std::uint16_t>(feedback_current_permille_));
    return frame;
}

}  // namespace sharpa::mujoco::plugins::chassis
