// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/chassis/kinco_motor_emulator.h"

#include <algorithm>
#include <stdexcept>

namespace sharpa::mujoco::plugins::chassis
{
namespace
{

constexpr std::uint32_t kAbortToggle = 0x05030000U;
constexpr std::uint32_t kAbortCommand = 0x05040001U;
constexpr std::uint32_t kAbortNoObject = 0x06020000U;
constexpr std::uint32_t kAbortReadOnly = 0x06010002U;
constexpr std::uint32_t kAbortWriteOnly = 0x06010001U;
constexpr std::uint32_t kAbortNoSubindex = 0x06090011U;
constexpr std::uint32_t kAbortTypeMismatch = 0x06070010U;
constexpr std::uint32_t kAbortValueRange = 0x06090030U;
constexpr std::uint32_t kAbortMapping = 0x06040041U;
constexpr std::uint32_t kAbortGeneral = 0x08000000U;
constexpr std::uint8_t kCanRtrFlag = 0x02U;

std::size_t expedited_width(std::uint8_t command) noexcept
{
    switch (command)
    {
        case 0x2FU:
            return 1;
        case 0x2BU:
            return 2;
        case 0x27U:
            return 3;
        case 0x23U:
            return 4;
        default:
            return 0;
    }
}

std::uint8_t upload_command(std::uint8_t width) noexcept
{
    return width == 1 ? 0x4FU : (width == 2 ? 0x4BU :
           (width == 3 ? 0x47U : 0x43U));
}

std::uint32_t width_mask(std::size_t width) noexcept
{
    return width >= 4 ? 0xFFFFFFFFU : ((1U << (width * 8U)) - 1U);
}

std::uint32_t read_little_endian(
    const std::uint8_t* data,
    std::size_t width) noexcept
{
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < width; ++i)
    {
        value |= static_cast<std::uint32_t>(data[i]) << (8U * i);
    }
    return value;
}

void append_little_endian(
    std::vector<std::uint8_t>& data,
    std::uint32_t value,
    std::size_t width)
{
    for (std::size_t i = 0; i < width; ++i)
    {
        data.push_back(static_cast<std::uint8_t>(value >> (8U * i)));
    }
}

}  // namespace

KincoMotorEmulator::KincoMotorEmulator(std::uint8_t node_id)
    : node_id_(node_id)
{
    if (node_id_ < 1 || node_id_ > 8)
    {
        throw std::invalid_argument("Kinco CANopen node id must be 1..8");
    }
    max_current_ = node_id_ <= 4 ? 202U : 354U;
    reset_communication();
}

std::uint8_t KincoMotorEmulator::node_id() const noexcept { return node_id_; }
KincoMotorEmulator::NmtState KincoMotorEmulator::nmt_state() const noexcept
{
    return nmt_state_;
}
std::int8_t KincoMotorEmulator::mode() const noexcept { return mode_; }
bool KincoMotorEmulator::enabled() const noexcept
{
    return drive_state_ == DriveState::operation_enabled;
}
std::uint16_t KincoMotorEmulator::control_word() const noexcept
{
    return control_word_;
}
std::uint16_t KincoMotorEmulator::status_word() const noexcept
{
    switch (drive_state_)
    {
        case DriveState::switch_on_disabled:
            return 0x0040U;
        case DriveState::ready_to_switch_on:
            return 0x0021U;
        case DriveState::switched_on:
            return 0x0023U;
        case DriveState::operation_enabled:
            return 0x0027U | (mode_ == 6 && homing_attained_ ? 0x1400U : 0U) |
                   (mode_ == 6 && homing_error_ ? 0x2000U : 0U);
        case DriveState::quick_stop_active:
            return 0x0007U;
        case DriveState::fault:
            return 0x0008U;
    }
    return 0;
}
std::int32_t KincoMotorEmulator::target_position() const noexcept
{
    return target_position_;
}
std::int32_t KincoMotorEmulator::target_velocity() const noexcept
{
    return target_velocity_;
}
std::int16_t KincoMotorEmulator::target_current() const noexcept
{
    return target_current_;
}
std::uint32_t KincoMotorEmulator::profile_velocity() const noexcept
{
    return profile_velocity_;
}
std::uint32_t KincoMotorEmulator::profile_acceleration() const noexcept
{
    return profile_acceleration_;
}
std::uint32_t KincoMotorEmulator::profile_deceleration() const noexcept
{
    return profile_deceleration_;
}
std::uint32_t KincoMotorEmulator::quick_stop_deceleration() const noexcept
{
    return quick_stop_deceleration_;
}
std::uint32_t
KincoMotorEmulator::encoder_counts_per_revolution() const noexcept
{
    return 65536U;
}
std::uint16_t KincoMotorEmulator::peak_current_raw() const noexcept
{
    return node_id_ <= 4 ? 202U : 354U;
}

void KincoMotorEmulator::set_actual_position(std::int32_t value) noexcept
{
    actual_position_ = value;
}
void KincoMotorEmulator::set_actual_velocity(std::int32_t value) noexcept
{
    actual_velocity_ = value;
}
void KincoMotorEmulator::set_actual_current(std::int16_t value) noexcept
{
    actual_current_ = value;
}
void KincoMotorEmulator::set_fault(std::uint16_t error_code) noexcept
{
    error_code_ = error_code;
    if (error_code_ != 0)
    {
        drive_state_ = DriveState::fault;
        homing_active_ = homing_attained_ = false;
    }
}

void KincoMotorEmulator::update_homing() noexcept
{
    // MJCF steering zero is already the calibrated mechanism reference. The
    // real drive searches its negative end stop and applies 0x607C; simulation
    // returns to that known reference without driving into a mechanical stop.
    // 128 encoder counts ~= 0.08 output degrees at the default 9:1 reduction;
    // 5000 velocity units ~= 0.28 motor rpm. Require position AND standstill.
    if (homing_active_ && actual_position_ >= -128 && actual_position_ <= 128 &&
        actual_velocity_ >= -5000 && actual_velocity_ <= 5000)
    {
        homing_attained_ = true;
    }
}

void KincoMotorEmulator::reset_communication() noexcept
{
    nmt_state_ = NmtState::pre_operational;
    guard_toggle_ = false;
    guard_time_ms_ = 0;
    heartbeat_period_ms_ = 0;
    life_time_factor_ = 0;
    guard_cob_id_ = 0x700U + node_id_;
    abort_connection_option_ = 0;

    rpdo_communication_[0] = {0x200U + node_id_, 255U, 0U};
    rpdo_communication_[1] = {0x300U + node_id_, 255U, 0U};
    tpdo_communication_[0] = {0x180U + node_id_, 255U, 0U};
    tpdo_communication_[1] = {0x280U + node_id_, 255U, 0U};

    if (node_id_ <= 4)
    {
        rpdo_mapping_[0] = {2U, {0x60400010U, 0x607A0020U}};
    }
    else
    {
        rpdo_mapping_[0] = {2U, {0x60400010U, 0x60FF0020U}};
    }
    rpdo_mapping_[1] = {2U, {0x60F60810U, 0x60400010U}};
    tpdo_mapping_[0] = {2U, {0x60640020U, 0x606C0020U}};
    tpdo_mapping_[1] = {2U, {0x60780010U, 0x60410010U}};
}

void KincoMotorEmulator::reset_node() noexcept
{
    control_word_ = 0;
    error_code_ = 0;
    mode_ = 0;
    target_position_ = 0;
    target_velocity_ = 0;
    target_current_ = 0;
    max_current_ = peak_current_raw();
    max_profile_velocity_ = 0;
    profile_velocity_ = 0;
    profile_acceleration_ = 0;
    profile_deceleration_ = 0;
    quick_stop_deceleration_ = 0;
    home_offset_ = 0;
    homing_method_ = 0;
    homing_speeds_ = {};
    homing_acceleration_ = 0;
    homing_power_on_ = homing_offset_mode_ = 0;
    homing_active_ = homing_attained_ = homing_error_ = false;
    drive_state_ = DriveState::switch_on_disabled;
    reset_communication();
}

void KincoMotorEmulator::apply_control_word(std::uint16_t value) noexcept
{
    const bool start_homing = (value & 0x0010U) != 0 &&
                              (control_word_ & 0x0010U) == 0;
    control_word_ = value;
    if ((value & 0x0080U) != 0 && drive_state_ == DriveState::fault)
    {
        error_code_ = 0;
        drive_state_ = DriveState::switch_on_disabled;
        return;
    }
    if (drive_state_ == DriveState::fault)
    {
        return;
    }
    if ((value & 0x000FU) == 0x000FU)
    {
        drive_state_ = DriveState::operation_enabled;
    }
    else if ((value & 0x000FU) == 0x0007U)
    {
        drive_state_ = DriveState::switched_on;
    }
    else if ((value & 0x008FU) == 0x0002U)
    {
        drive_state_ = DriveState::quick_stop_active;
    }
    else if ((value & 0x008FU) == 0x0006U)
    {
        drive_state_ = DriveState::ready_to_switch_on;
    }
    else if ((value & 0x0002U) == 0)
    {
        drive_state_ = DriveState::switch_on_disabled;
    }
    if (!enabled() || (value & 0x0100U) != 0 || (mode_ == 6 && (value & 0x0010U) == 0))
    {
        homing_active_ = homing_attained_ = false;
    }
    else if (mode_ == 6 && start_homing && nmt_state_ == NmtState::operational)
    {
        homing_attained_ = false;
        homing_error_ = node_id_ > 4 || homing_method_ != -17 ||
                        homing_speeds_[0] == 0 || homing_speeds_[1] == 0 ||
                        homing_acceleration_ == 0;
        homing_active_ = !homing_error_;
    }
}

std::optional<KincoMotorEmulator::ObjectValue>
KincoMotorEmulator::read_object(
    std::uint16_t index,
    std::uint8_t subindex) const noexcept
{
    const auto value = [](std::uint32_t data, std::uint8_t width, bool writable)
        -> std::optional<ObjectValue>
    {
        return ObjectValue{data, width, writable};
    };
    if (index >= 0x1400U && index <= 0x1401U)
    {
        const auto& pdo = rpdo_communication_[index - 0x1400U];
        if (subindex == 0) return value(2, 1, false);
        if (subindex == 1) return value(pdo.cob_id, 4, true);
        if (subindex == 2) return value(pdo.transmission_type, 1, true);
        if (subindex == 3) return value(pdo.inhibit_time, 2, true);
        return std::nullopt;
    }
    if (index >= 0x1800U && index <= 0x1801U)
    {
        const auto& pdo = tpdo_communication_[index - 0x1800U];
        if (subindex == 0) return value(5, 1, false);
        if (subindex == 1) return value(pdo.cob_id, 4, true);
        if (subindex == 2) return value(pdo.transmission_type, 1, true);
        if (subindex == 3) return value(pdo.inhibit_time, 2, true);
        if (subindex == 5) return value(pdo.event_timer_ms, 2, true);
        return std::nullopt;
    }
    if ((index >= 0x1600U && index <= 0x1601U) ||
        (index >= 0x1A00U && index <= 0x1A01U))
    {
        const bool receive = index < 0x1800U;
        const std::size_t pdo_number =
            receive ? index - 0x1600U : index - 0x1A00U;
        const auto& mapping = receive ? rpdo_mapping_[pdo_number] :
                                        tpdo_mapping_[pdo_number];
        if (subindex == 0) return value(mapping.count, 1, true);
        if (subindex <= mapping.entries.size())
        {
            return value(mapping.entries[subindex - 1U], 4, true);
        }
        return std::nullopt;
    }

    if (index == 0x6099U)
    {
        if (subindex == 0) return value(5, 1, false);
        if (subindex == 1 || subindex == 2)
            return value(homing_speeds_[subindex - 1], 4, true);
        if (subindex == 3) return value(homing_power_on_, 1, true);
        if (subindex == 5) return value(homing_offset_mode_, 1, true);
        return std::nullopt;
    }
    if (subindex != 0 && !(index == 0x60F6U && subindex == 8) &&
        !(index == 0x6510U && subindex == 3))
    {
        return std::nullopt;
    }
    switch (index)
    {
        case 0x100CU: return value(guard_time_ms_, 2, true);
        case 0x100DU: return value(life_time_factor_, 1, true);
        case 0x100EU: return value(guard_cob_id_, 4, true);
        case 0x1017U: return value(heartbeat_period_ms_, 2, true);
        case 0x2690U: return value(0, 1, true);
        case 0x6007U: return value(abort_connection_option_, 1, true);
        case 0x603FU: return value(error_code_, 2, false);
        case 0x6040U: return value(control_word_, 2, true);
        case 0x6041U: return value(status_word(), 2, false);
        case 0x6060U: return value(static_cast<std::uint8_t>(mode_), 1, true);
        case 0x6061U: return value(static_cast<std::uint8_t>(mode_), 1, false);
        case 0x6064U: return value(static_cast<std::uint32_t>(actual_position_), 4, false);
        case 0x606CU: return value(static_cast<std::uint32_t>(actual_velocity_), 4, false);
        case 0x6073U: return value(max_current_, 2, true);
        case 0x6078U: return value(static_cast<std::uint16_t>(actual_current_), 2, false);
        case 0x607AU: return value(static_cast<std::uint32_t>(target_position_), 4, true);
        case 0x607CU: return value(static_cast<std::uint32_t>(home_offset_), 4, true);
        case 0x607FU: return value(max_profile_velocity_, 4, true);
        case 0x6081U: return value(profile_velocity_, 4, true);
        case 0x6083U: return value(profile_acceleration_, 4, true);
        case 0x6084U: return value(profile_deceleration_, 4, true);
        case 0x6085U: return value(quick_stop_deceleration_, 4, true);
        case 0x6098U: return value(static_cast<std::uint8_t>(homing_method_), 1, true);
        case 0x609AU: return value(homing_acceleration_, 4, true);
        case 0x60F6U: return value(static_cast<std::uint16_t>(target_current_), 2, true);
        case 0x60FFU: return value(static_cast<std::uint32_t>(target_velocity_), 4, true);
        case 0x6510U: return value(peak_current_raw(), 2, false);
        default: return std::nullopt;
    }
}

std::uint32_t KincoMotorEmulator::write_object(
    std::uint16_t index,
    std::uint8_t subindex,
    std::uint32_t value,
    std::size_t width) noexcept
{
    if (index == 0x2FF0U)
    {
        if (subindex != 1U) return kAbortNoSubindex;
        if (width != 1U) return kAbortTypeMismatch;
        if (value != 1U) return kAbortValueRange;
        // Platform saves Kinco control parameters before enabling. The
        // emulator retains them in memory for this session; acknowledge the
        // trigger without resetting configuration, enabling or moving motors.
        // Power-cycle persistence is not simulated.
        return 0;
    }

    const auto object = read_object(index, subindex);
    if (!object)
    {
        return subindex == 0 ? kAbortNoObject : kAbortNoSubindex;
    }
    if (!object->writable)
    {
        return kAbortReadOnly;
    }
    // Older Platform bundles wrote this vendor object as U16; current drivers
    // use the documented U8. Accept both while reporting its canonical width.
    if (width != object->width && !(index == 0x6007U && width == 2))
    {
        return kAbortTypeMismatch;
    }

    if (index >= 0x1400U && index <= 0x1401U)
    {
        auto& pdo = rpdo_communication_[index - 0x1400U];
        if (subindex == 1) pdo.cob_id = value;
        else if (subindex == 2) pdo.transmission_type = static_cast<std::uint8_t>(value);
        else if (subindex == 3) pdo.inhibit_time = static_cast<std::uint16_t>(value);
        return 0;
    }
    if (index >= 0x1800U && index <= 0x1801U)
    {
        auto& pdo = tpdo_communication_[index - 0x1800U];
        if (subindex == 1) pdo.cob_id = value;
        else if (subindex == 2) pdo.transmission_type = static_cast<std::uint8_t>(value);
        else if (subindex == 3) pdo.inhibit_time = static_cast<std::uint16_t>(value);
        else if (subindex == 5) pdo.event_timer_ms = static_cast<std::uint16_t>(value);
        return 0;
    }
    if ((index >= 0x1600U && index <= 0x1601U) ||
        (index >= 0x1A00U && index <= 0x1A01U))
    {
        const bool receive = index < 0x1800U;
        const std::size_t pdo_number =
            receive ? index - 0x1600U : index - 0x1A00U;
        auto& mapping = receive ? rpdo_mapping_[pdo_number] :
                                  tpdo_mapping_[pdo_number];
        if (subindex == 0)
        {
            if (value > mapping.entries.size()) return kAbortMapping;
            mapping.count = static_cast<std::uint8_t>(value);
        }
        else
        {
            const std::uint8_t bits = static_cast<std::uint8_t>(value);
            if (bits == 0 || bits > 32) return kAbortMapping;
            mapping.entries[subindex - 1U] = value;
        }
        return 0;
    }

    switch (index)
    {
        case 0x100CU: guard_time_ms_ = static_cast<std::uint16_t>(value); break;
        case 0x100DU: life_time_factor_ = static_cast<std::uint8_t>(value); break;
        case 0x100EU: guard_cob_id_ = value; break;
        case 0x1017U: heartbeat_period_ms_ = static_cast<std::uint16_t>(value); break;
        case 0x2690U:
            if (value == 10U)
            {
                actual_position_ = 0;
                error_code_ = 0;
            }
            break;
        case 0x6007U: abort_connection_option_ = static_cast<std::uint16_t>(value); break;
        case 0x6040U: apply_control_word(static_cast<std::uint16_t>(value)); break;
        case 0x6060U:
            if (mode_ != static_cast<std::int8_t>(value))
                homing_active_ = homing_attained_ = homing_error_ = false;
            mode_ = static_cast<std::int8_t>(value);
            break;
        case 0x6073U: max_current_ = static_cast<std::uint16_t>(value); break;
        case 0x607AU: target_position_ = static_cast<std::int32_t>(value); break;
        case 0x607CU: home_offset_ = static_cast<std::int32_t>(value); break;
        case 0x607FU: max_profile_velocity_ = value; break;
        case 0x6081U: profile_velocity_ = value; break;
        case 0x6083U: profile_acceleration_ = value; break;
        case 0x6084U: profile_deceleration_ = value; break;
        case 0x6085U: quick_stop_deceleration_ = value; break;
        case 0x6098U: homing_method_ = static_cast<std::int8_t>(value); break;
        case 0x6099U:
            if (subindex == 1 || subindex == 2) homing_speeds_[subindex - 1] = value;
            else if (subindex == 3) homing_power_on_ = static_cast<std::uint8_t>(value);
            else if (subindex == 5) homing_offset_mode_ = static_cast<std::uint8_t>(value);
            break;
        case 0x609AU: homing_acceleration_ = value; break;
        case 0x60F6U: target_current_ = static_cast<std::int16_t>(value); break;
        case 0x60FFU: target_velocity_ = static_cast<std::int32_t>(value); break;
        default: return kAbortGeneral;
    }
    return 0;
}

std::optional<common::CanFdFrame> KincoMotorEmulator::process_sdo(
    const common::CanFdFrame& request)
{
    if (request.data.size() != 8)
    {
        return std::nullopt;
    }
    common::CanFdFrame response;
    response.can_interface = request.can_interface;
    response.can_id = 0x580U + node_id_;
    response.data.assign(8, 0);
    std::copy_n(request.data.begin() + 1, 3, response.data.begin() + 1);
    const std::uint16_t index = common::read_le16(request.data.data() + 1);
    const std::uint8_t subindex = request.data[3];

    const auto abort = [&response](std::uint32_t code)
    {
        response.data[0] = 0x80U;
        common::write_le32(response.data.data() + 4, code);
    };
    if (request.data[0] == 0x40U)
    {
        const auto object = read_object(index, subindex);
        if (!object)
        {
            abort(subindex == 0 ? kAbortNoObject : kAbortNoSubindex);
        }
        else
        {
            response.data[0] = upload_command(object->width);
            common::write_le32(response.data.data() + 4, object->value);
        }
        return response;
    }
    if (request.data[0] == 0x80U)
    {
        return std::nullopt;
    }
    const std::size_t width = expedited_width(request.data[0]);
    if (width == 0)
    {
        abort((request.data[0] & 0xE0U) == 0x20U ? kAbortCommand :
                                                       kAbortToggle);
        return response;
    }
    const std::uint32_t code = write_object(
        index,
        subindex,
        common::read_le32(request.data.data() + 4) & width_mask(width),
        width);
    if (code != 0)
    {
        abort(code);
    }
    else
    {
        response.data[0] = 0x60U;
    }
    return response;
}

std::optional<common::CanFdFrame> KincoMotorEmulator::process_rpdo(
    const common::CanFdFrame& request,
    std::size_t pdo_number)
{
    const auto& mapping = rpdo_mapping_[pdo_number];
    std::size_t offset = 0;
    for (std::size_t i = 0; i < mapping.count; ++i)
    {
        const std::uint32_t entry = mapping.entries[i];
        const std::uint8_t bits = static_cast<std::uint8_t>(entry);
        if (bits == 0 || bits > 32 || (bits % 8U) != 0)
        {
            return std::nullopt;
        }
        const std::size_t width = bits / 8U;
        if (offset + width > request.data.size())
        {
            return std::nullopt;
        }
        const std::uint16_t index = static_cast<std::uint16_t>(entry >> 16U);
        const std::uint8_t subindex = static_cast<std::uint8_t>(entry >> 8U);
        if (write_object(
                index,
                subindex,
                read_little_endian(request.data.data() + offset, width),
                width) != 0)
        {
            return std::nullopt;
        }
        offset += width;
    }
    return std::nullopt;
}

common::CanFdFrame KincoMotorEmulator::make_tpdo(
    std::uint8_t can_interface,
    std::size_t pdo_number) const
{
    common::CanFdFrame frame;
    frame.can_interface = can_interface;
    frame.can_id = tpdo_communication_[pdo_number].cob_id & 0x1FFFFFFFU;
    const auto& mapping = tpdo_mapping_[pdo_number];
    for (std::size_t i = 0; i < mapping.count; ++i)
    {
        const std::uint32_t entry = mapping.entries[i];
        const std::uint8_t bits = static_cast<std::uint8_t>(entry);
        if (bits == 0 || bits > 32 || (bits % 8U) != 0)
        {
            break;
        }
        const auto object = read_object(
            static_cast<std::uint16_t>(entry >> 16U),
            static_cast<std::uint8_t>(entry >> 8U));
        if (!object || object->width * 8U != bits || frame.data.size() + object->width > 8)
        {
            break;
        }
        append_little_endian(frame.data, object->value, object->width);
    }
    return frame;
}

common::CanFdFrame KincoMotorEmulator::make_guard_response(
    std::uint8_t can_interface)
{
    common::CanFdFrame frame = heartbeat_frame(can_interface);
    frame.data[0] |= guard_toggle_ ? 0x80U : 0U;
    guard_toggle_ = !guard_toggle_;
    return frame;
}

std::optional<common::CanFdFrame> KincoMotorEmulator::process(
    const common::CanFdFrame& request)
{
    if (request.can_id == 0 && request.data.size() >= 2)
    {
        const std::uint8_t command = request.data[0];
        const std::uint8_t target = request.data[1];
        if (target != 0 && target != node_id_) return std::nullopt;
        if (command == 0x01U) nmt_state_ = NmtState::operational;
        else if (command == 0x02U) nmt_state_ = NmtState::stopped;
        else if (command == 0x80U) nmt_state_ = NmtState::pre_operational;
        else if (command == 0x81U) reset_node();
        else if (command == 0x82U) reset_communication();
        if (nmt_state_ != NmtState::operational)
            homing_active_ = homing_attained_ = false;
        return std::nullopt;
    }
    if (request.can_id == 0x600U + node_id_)
    {
        return process_sdo(request);
    }
    if ((request.can_id & 0x1FFFFFFFU) == (guard_cob_id_ & 0x1FFFFFFFU) &&
        (request.data.empty() || (request.can_id & 0x40000000U) != 0 ||
         (request.can_flags & kCanRtrFlag) != 0))
    {
        return make_guard_response(request.can_interface);
    }
    if (nmt_state_ != NmtState::operational)
    {
        return std::nullopt;
    }
    for (std::size_t i = 0; i < rpdo_communication_.size(); ++i)
    {
        const std::uint32_t cob_id = rpdo_communication_[i].cob_id;
        if ((cob_id & 0x80000000U) == 0 &&
            request.can_id == (cob_id & 0x1FFFFFFFU))
        {
            return process_rpdo(request, i);
        }
    }
    return std::nullopt;
}

common::CanFdFrame KincoMotorEmulator::tpdo1_frame(
    std::uint8_t can_interface) const
{
    return make_tpdo(can_interface, 0);
}
common::CanFdFrame KincoMotorEmulator::tpdo2_frame(
    std::uint8_t can_interface) const
{
    return make_tpdo(can_interface, 1);
}
common::CanFdFrame KincoMotorEmulator::heartbeat_frame(
    std::uint8_t can_interface) const
{
    common::CanFdFrame frame;
    frame.can_interface = can_interface;
    frame.can_id = guard_cob_id_ & 0x1FFFFFFFU;
    frame.data = {static_cast<std::uint8_t>(nmt_state_)};
    return frame;
}

}  // namespace sharpa::mujoco::plugins::chassis
