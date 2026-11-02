// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"

#include <array>
#include <map>
#include <optional>
#include <utility>

namespace sharpa::mujoco::plugins::chassis
{

// Platform's Jiechang classic-CAN CSP/CSV contract. Motion and feedback counts
// are linear: 262144 counts = 1 mm (not the steer/wheel encoder conversion).
class JiechangStretchEmulator
{
  public:
    explicit JiechangStretchEmulator(std::uint8_t node_id);
    std::optional<common::CanFdFrame> process(const common::CanFdFrame& frame);
    std::array<common::CanFdFrame, 2> feedback(std::uint8_t bus) const;
    void set_feedback(std::int32_t position, std::int32_t velocity, std::int16_t torque,
                      std::int16_t current);
    bool enabled() const noexcept;
    std::int8_t mode() const noexcept { return mode_; }
    std::int32_t target_position() const noexcept { return target_position_; }
    std::int32_t target_velocity() const noexcept { return target_velocity_; }
    double maximum_velocity() const noexcept;
    double command_timeout_s() const noexcept;

  private:
    std::uint16_t status() const noexcept;
    std::uint8_t node_id_;
    bool operational_ = false;
    std::int8_t mode_ = 8;
    std::uint16_t control_word_ = 0;
    std::uint16_t pending_control_word_ = 0;
    std::int32_t target_position_ = 0, pending_position_ = 0;
    std::int32_t target_velocity_ = 0, pending_velocity_ = 0;
    std::int32_t position_ = 0, velocity_ = 0;
    std::int16_t torque_ = 0, current_ = 0;
    // Retain PDO configuration writes so SDO reads report the configured values.
    std::map<std::pair<std::uint16_t, std::uint8_t>,
             std::pair<std::uint32_t, std::size_t>>
        objects_;
};

} // namespace sharpa::mujoco::plugins::chassis
