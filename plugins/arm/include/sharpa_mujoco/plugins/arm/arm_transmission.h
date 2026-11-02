// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>

namespace sharpa::mujoco::plugins::arm
{

enum class ArmSide
{
    Left,
    Right,
};

class ArmTransmission
{
public:
    static constexpr std::size_t kAxisCount = 7;
    using AxisArray = std::array<double, kAxisCount>;

    explicit ArmTransmission(ArmSide side);

    AxisArray joint_to_motor(const AxisArray& joints) const;
    AxisArray motor_to_joint(const AxisArray& motors) const;
    AxisArray motor_torque_to_joint(const AxisArray& torques) const;
    AxisArray joint_torque_to_motor(const AxisArray& torques) const;

private:
    ArmSide side_;
};

}  // namespace sharpa::mujoco::plugins::arm
