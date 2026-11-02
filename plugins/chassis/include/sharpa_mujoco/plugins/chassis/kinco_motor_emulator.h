// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace sharpa::mujoco::plugins::chassis
{

class KincoMotorEmulator
{
public:
    enum class NmtState : std::uint8_t
    {
        stopped = 0x04,
        operational = 0x05,
        pre_operational = 0x7F,
    };

    explicit KincoMotorEmulator(std::uint8_t node_id = 1);

    std::uint8_t node_id() const noexcept;
    NmtState nmt_state() const noexcept;
    std::int8_t mode() const noexcept;
    bool enabled() const noexcept;
    bool homing_active() const noexcept { return homing_active_; }
    std::uint32_t homing_velocity() const noexcept { return homing_speeds_[1]; }
    std::uint32_t homing_acceleration() const noexcept { return homing_acceleration_; }
    std::uint16_t control_word() const noexcept;
    std::uint16_t status_word() const noexcept;
    std::int32_t target_position() const noexcept;
    std::int32_t target_velocity() const noexcept;
    std::int16_t target_current() const noexcept;
    std::uint32_t profile_velocity() const noexcept;
    std::uint32_t profile_acceleration() const noexcept;
    std::uint32_t profile_deceleration() const noexcept;
    std::uint32_t quick_stop_deceleration() const noexcept;
    std::uint32_t encoder_counts_per_revolution() const noexcept;
    std::uint16_t peak_current_raw() const noexcept;
    std::uint16_t heartbeat_period_ms() const noexcept { return heartbeat_period_ms_; }

    void set_actual_position(std::int32_t value) noexcept;
    void set_actual_velocity(std::int32_t value) noexcept;
    void set_actual_current(std::int16_t value) noexcept;
    void set_fault(std::uint16_t error_code) noexcept;
    // Call after updating physical position/velocity; never synthesize encoder
    // feedback to acknowledge homing before the simulated joint has settled.
    void update_homing() noexcept;

    std::optional<common::CanFdFrame> process(
        const common::CanFdFrame& request);
    common::CanFdFrame tpdo1_frame(std::uint8_t can_interface) const;
    common::CanFdFrame tpdo2_frame(std::uint8_t can_interface) const;
    common::CanFdFrame heartbeat_frame(std::uint8_t can_interface) const;

private:
    enum class DriveState
    {
        switch_on_disabled,
        ready_to_switch_on,
        switched_on,
        operation_enabled,
        quick_stop_active,
        fault,
    };

    struct PdoCommunication
    {
        std::uint32_t cob_id = 0;
        std::uint8_t transmission_type = 255;
        std::uint16_t inhibit_time = 0;
        std::uint16_t event_timer_ms = 0;
    };

    struct PdoMapping
    {
        std::uint8_t count = 0;
        std::array<std::uint32_t, 8> entries{};
    };

    struct ObjectValue
    {
        std::uint32_t value = 0;
        std::uint8_t width = 0;
        bool writable = false;
    };

    std::optional<common::CanFdFrame> process_sdo(
        const common::CanFdFrame& request);
    std::optional<common::CanFdFrame> process_rpdo(
        const common::CanFdFrame& request,
        std::size_t pdo_number);
    common::CanFdFrame make_tpdo(
        std::uint8_t can_interface,
        std::size_t pdo_number) const;
    common::CanFdFrame make_guard_response(std::uint8_t can_interface);

    std::optional<ObjectValue> read_object(
        std::uint16_t index,
        std::uint8_t subindex) const noexcept;
    std::uint32_t write_object(
        std::uint16_t index,
        std::uint8_t subindex,
        std::uint32_t value,
        std::size_t width) noexcept;
    void apply_control_word(std::uint16_t value) noexcept;
    void reset_communication() noexcept;
    void reset_node() noexcept;

    std::uint8_t node_id_ = 1;
    NmtState nmt_state_ = NmtState::pre_operational;
    DriveState drive_state_ = DriveState::switch_on_disabled;
    bool guard_toggle_ = false;

    std::uint16_t control_word_ = 0;
    std::uint16_t error_code_ = 0;
    std::int8_t mode_ = 0;
    std::int32_t actual_position_ = 0;
    std::int32_t actual_velocity_ = 0;
    std::int16_t actual_current_ = 0;
    std::int32_t target_position_ = 0;
    std::int32_t target_velocity_ = 0;
    std::int16_t target_current_ = 0;
    std::uint16_t max_current_ = 0;
    std::uint32_t max_profile_velocity_ = 0;
    std::uint32_t profile_velocity_ = 0;
    std::uint32_t profile_acceleration_ = 0;
    std::uint32_t profile_deceleration_ = 0;
    std::uint32_t quick_stop_deceleration_ = 0;
    std::int32_t home_offset_ = 0;
    std::int8_t homing_method_ = 0;
    std::array<std::uint32_t, 2> homing_speeds_{};
    std::uint32_t homing_acceleration_ = 0;
    std::uint8_t homing_power_on_ = 0;
    std::uint8_t homing_offset_mode_ = 0;
    bool homing_active_ = false;
    bool homing_attained_ = false;
    bool homing_error_ = false;
    std::uint16_t guard_time_ms_ = 0;
    std::uint16_t heartbeat_period_ms_ = 0;
    std::uint8_t life_time_factor_ = 0;
    std::uint32_t guard_cob_id_ = 0;
    std::uint16_t abort_connection_option_ = 0;

    std::array<PdoCommunication, 2> rpdo_communication_{};
    std::array<PdoCommunication, 2> tpdo_communication_{};
    std::array<PdoMapping, 2> rpdo_mapping_{};
    std::array<PdoMapping, 2> tpdo_mapping_{};
};

}  // namespace sharpa::mujoco::plugins::chassis
