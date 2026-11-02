// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>

namespace sharpa::mujoco::plugins::torso
{

// Portable production-geometry conversion for POC2 motor slots 3/4.
class ParallelWaistKinematics
{
public:
    struct Config
    {
        double lax = 36.5;
        double lay = 0.0;
        double laz = 38.0;
        double l4 = 102.5;
        double lc = 46.5;
        double ll = 104.073;
        double lb = 19.0;
        int elbow = -1;
        double theta1_sign = -1.0;
        double theta2_sign = 1.0;
    };

    ParallelWaistKinematics();
    explicit ParallelWaistKinematics(Config config);

    // Joint order is {pitch(qp), roll(qr)}; motor order is slots {3, 4}.
    bool joint_to_motor(
        const std::array<double, 2>& joint_rad,
        std::array<double, 2>& motor_rad) const;
    bool motor_to_joint(
        const std::array<double, 2>& motor_rad,
        std::array<double, 2>& joint_rad) const;
    // J = d(motor angle) / d({pitch, roll}); rows are motor slots {3, 4}.
    bool motor_jacobian(
        const std::array<double, 2>& joint_rad,
        std::array<std::array<double, 2>, 2>& jacobian) const;
    bool motor_torque_to_joint_torque(
        const std::array<double, 2>& joint_rad,
        const std::array<double, 2>& motor_torque,
        std::array<double, 2>& joint_torque) const;
    bool joint_torque_to_motor_torque(
        const std::array<double, 2>& joint_rad,
        const std::array<double, 2>& joint_torque,
        std::array<double, 2>& motor_torque) const;
    // Returns the diagonal of J^T I J for joint order {pitch, roll}.
    // The off-diagonal coupling is intentionally not represented by MuJoCo armature.
    bool reflected_inertia_diagonal(
        const std::array<double, 2>& joint_rad,
        const std::array<double, 2>& motor_inertia,
        std::array<double, 2>& joint_inertia) const;

private:
    struct Geometry;
    struct GeometryDerivatives;
    bool inverse(
        double roll,
        double pitch,
        double& theta1,
        double& theta2,
        int elbow) const;
    bool solve_from_seed(
        double theta1,
        double theta2,
        double seed_roll,
        double seed_pitch,
        double& roll,
        double& pitch) const;
    Geometry geometry(double roll, double pitch) const;
    GeometryDerivatives geometry_derivatives(double roll, double pitch) const;

    Config config_;
};

}  // namespace sharpa::mujoco::plugins::torso
