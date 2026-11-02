// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/arm/arm_transmission.h"

namespace sharpa::mujoco::plugins::arm
{
namespace
{
using AxisArray = ArmTransmission::AxisArray;

AxisArray directions(ArmSide side)
{
    return side == ArmSide::Left
               ? AxisArray{-1, -1, 1, -1, 1, 1, -1}
               : AxisArray{1, -1, -1, -1, 1, 1, -1};
}

AxisArray joint_ratios(ArmSide side)
{
    return side == ArmSide::Left
               ? AxisArray{1, 1, 1, 1, 0.5, 1, -1}
               : AxisArray{1, 1, 1, 1, -0.5, 1, 1};
}

constexpr AxisArray cable_ratios()
{
    return {1, 1, 1, 2.34375, 2.34375, 1, 1};
}
}  // namespace

ArmTransmission::ArmTransmission(ArmSide side) : side_(side) {}

ArmTransmission::AxisArray ArmTransmission::joint_to_motor(
    const AxisArray& joints) const
{
    const AxisArray direction = directions(side_);
    const AxisArray cable = cable_ratios();
    const AxisArray ratio = joint_ratios(side_);
    AxisArray motors{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        motors[i] = joints[i] * ratio[i] * direction[i] * cable[i];
    }
    motors[3] = (joints[3] * ratio[3] + joints[4] * ratio[4]) *
                direction[3] * cable[3];
    motors[4] = (joints[3] * ratio[3] - joints[4] * ratio[4]) *
                direction[4] * cable[4];
    motors[5] = (joints[5] * ratio[5] + joints[6] * ratio[6]) *
                direction[5] * cable[5];
    motors[6] = (joints[5] * ratio[5] - joints[6] * ratio[6]) *
                direction[6] * cable[6];
    return motors;
}

ArmTransmission::AxisArray ArmTransmission::motor_to_joint(
    const AxisArray& motors) const
{
    const AxisArray direction = directions(side_);
    const AxisArray cable = cable_ratios();
    const AxisArray ratio = joint_ratios(side_);
    AxisArray joints{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        joints[i] = motors[i] / (direction[i] * cable[i]) / ratio[i];
    }
    const double motor3 = motors[3] / (direction[3] * cable[3]);
    const double motor4 = motors[4] / (direction[4] * cable[4]);
    joints[3] = 0.5 * (motor3 + motor4) / ratio[3];
    joints[4] = 0.5 * (motor3 - motor4) / ratio[4];
    const double motor5 = motors[5] / (direction[5] * cable[5]);
    const double motor6 = motors[6] / (direction[6] * cable[6]);
    joints[5] = 0.5 * (motor5 + motor6) / ratio[5];
    joints[6] = 0.5 * (motor5 - motor6) / ratio[6];
    return joints;
}

ArmTransmission::AxisArray ArmTransmission::joint_torque_to_motor(
    const AxisArray& torques) const
{
    const AxisArray direction = directions(side_);
    const AxisArray cable = cable_ratios();
    const AxisArray ratio = joint_ratios(side_);
    AxisArray motors{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        motors[i] = torques[i] / ratio[i] / (direction[i] * cable[i]);
    }
    motors[3] = (torques[3] / ratio[3] + torques[4] / ratio[4]) /
                (2.0 * direction[3] * cable[3]);
    motors[4] = (torques[3] / ratio[3] - torques[4] / ratio[4]) /
                (2.0 * direction[4] * cable[4]);
    motors[5] = (torques[5] / ratio[5] + torques[6] / ratio[6]) /
                (2.0 * direction[5] * cable[5]);
    motors[6] = (torques[5] / ratio[5] - torques[6] / ratio[6]) /
                (2.0 * direction[6] * cable[6]);
    return motors;
}

ArmTransmission::AxisArray ArmTransmission::motor_torque_to_joint(
    const AxisArray& torques) const
{
    const AxisArray direction = directions(side_);
    const AxisArray cable = cable_ratios();
    const AxisArray ratio = joint_ratios(side_);
    AxisArray joints{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        joints[i] = torques[i] * ratio[i] * direction[i] * cable[i];
    }
    joints[3] =
        direction[3] * cable[3] * ratio[3] * torques[3] +
        direction[4] * cable[4] * ratio[3] * torques[4];
    joints[4] =
        direction[3] * cable[3] * ratio[4] * torques[3] -
        direction[4] * cable[4] * ratio[4] * torques[4];
    joints[5] =
        direction[5] * cable[5] * ratio[5] * torques[5] +
        direction[6] * cable[6] * ratio[5] * torques[6];
    joints[6] =
        direction[5] * cable[5] * ratio[6] * torques[5] -
        direction[6] * cable[6] * ratio[6] * torques[6];
    return joints;
}

}  // namespace sharpa::mujoco::plugins::arm
