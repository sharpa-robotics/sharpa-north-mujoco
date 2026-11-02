// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/torso/parallel_waist_kinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sharpa::mujoco::plugins::torso
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double wrap(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

struct ParallelWaistKinematics::Geometry
{
    double p1;
    double q1;
    double z1;
    double p2;
    double q2;
    double z2;
};

struct ParallelWaistKinematics::GeometryDerivatives
{
    double dp1_droll;
    double dq1_droll;
    double dz1_droll;
    double dp2_droll;
    double dq2_droll;
    double dz2_droll;
    double dp1_dpitch;
    double dq1_dpitch;
    double dp2_dpitch;
    double dq2_dpitch;
};

ParallelWaistKinematics::ParallelWaistKinematics(Config config)
    : config_(config)
{
}

ParallelWaistKinematics::ParallelWaistKinematics()
    : ParallelWaistKinematics(Config{})
{
}

ParallelWaistKinematics::Geometry ParallelWaistKinematics::geometry(
    double roll,
    double pitch) const
{
    const double sr = std::sin(roll);
    const double cr = std::cos(roll);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);
    return {
        config_.l4 * cp - config_.lax * sr * cp - config_.lay * cr * cp +
            config_.laz * sp,
        config_.l4 * sp - config_.lax * sr * sp - config_.lay * cr * sp -
            config_.laz * cp,
        -config_.lax * cr + config_.lay * sr + config_.lb,
        config_.l4 * cp + config_.lax * sr * cp - config_.lay * cr * cp +
            config_.laz * sp,
        config_.l4 * sp + config_.lax * sr * sp - config_.lay * cr * sp -
            config_.laz * cp,
        config_.lax * cr + config_.lay * sr - config_.lb,
    };
}

ParallelWaistKinematics::GeometryDerivatives
ParallelWaistKinematics::geometry_derivatives(
    double roll,
    double pitch) const
{
    const double sr = std::sin(roll);
    const double cr = std::cos(roll);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);
    return {
        -config_.lax * cr * cp + config_.lay * sr * cp,
        -config_.lax * cr * sp + config_.lay * sr * sp,
        config_.lax * sr + config_.lay * cr,
        config_.lax * cr * cp + config_.lay * sr * cp,
        config_.lax * cr * sp + config_.lay * sr * sp,
        -config_.lax * sr + config_.lay * cr,
        -config_.l4 * sp + config_.lax * sr * sp +
            config_.lay * cr * sp + config_.laz * cp,
        config_.l4 * cp - config_.lax * sr * cp -
            config_.lay * cr * cp + config_.laz * sp,
        -config_.l4 * sp - config_.lax * sr * sp +
            config_.lay * cr * sp + config_.laz * cp,
        config_.l4 * cp + config_.lax * sr * cp -
            config_.lay * cr * cp + config_.laz * sp,
    };
}

bool ParallelWaistKinematics::inverse(
    double roll,
    double pitch,
    double& theta1,
    double& theta2,
    int elbow) const
{
    const Geometry g = geometry(roll, pitch);
    const double a1 = -2.0 * config_.lc * g.p1;
    const double b1 = 2.0 * config_.lc * g.q1;
    const double c1 = config_.ll * config_.ll - g.p1 * g.p1 - g.q1 * g.q1 -
                      g.z1 * g.z1 - config_.lc * config_.lc;
    const double a2 = -2.0 * config_.lc * g.p2;
    const double b2 = 2.0 * config_.lc * g.q2;
    const double c2 = config_.ll * config_.ll - g.p2 * g.p2 - g.q2 * g.q2 -
                      g.z2 * g.z2 - config_.lc * config_.lc;
    const double r1 = std::hypot(a1, b1);
    const double r2 = std::hypot(a2, b2);
    if (std::abs(c1) > r1 || std::abs(c2) > r2)
    {
        return false;
    }
    const double sign = elbow >= 0 ? 1.0 : -1.0;
    theta1 = wrap(
        std::atan2(c1, sign * std::sqrt(std::max(0.0, r1 * r1 - c1 * c1))) -
        std::atan2(b1, a1));
    theta2 = wrap(
        std::atan2(c2, sign * std::sqrt(std::max(0.0, r2 * r2 - c2 * c2))) -
        std::atan2(b2, a2));
    return true;
}

bool ParallelWaistKinematics::joint_to_motor(
    const std::array<double, 2>& joint_rad,
    std::array<double, 2>& motor_rad) const
{
    double theta1 = 0;
    double theta2 = 0;
    if (!inverse(joint_rad[1], joint_rad[0], theta1, theta2, config_.elbow))
    {
        return false;
    }
    motor_rad = {
        config_.theta1_sign * theta1,
        config_.theta2_sign * theta2,
    };
    return true;
}

bool ParallelWaistKinematics::motor_jacobian(
    const std::array<double, 2>& joint_rad,
    std::array<std::array<double, 2>, 2>& jacobian) const
{
    const double pitch = joint_rad[0];
    const double roll = joint_rad[1];
    double theta1 = 0.0;
    double theta2 = 0.0;
    if (!inverse(roll, pitch, theta1, theta2, config_.elbow))
    {
        return false;
    }
    const Geometry g = geometry(roll, pitch);
    const GeometryDerivatives d = geometry_derivatives(roll, pitch);
    const double u1 = g.p1 - config_.lc * std::sin(theta1);
    const double v1 = g.q1 + config_.lc * std::cos(theta1);
    const double u2 = g.p2 - config_.lc * std::sin(theta2);
    const double v2 = g.q2 + config_.lc * std::cos(theta2);
    const double jt1 =
        -2.0 * config_.lc *
        (g.p1 * std::cos(theta1) + g.q1 * std::sin(theta1));
    const double jt2 =
        -2.0 * config_.lc *
        (g.p2 * std::cos(theta2) + g.q2 * std::sin(theta2));
    if (!std::isfinite(jt1) || !std::isfinite(jt2) ||
        std::abs(jt1) < 1e-10 || std::abs(jt2) < 1e-10)
    {
        return false;
    }
    jacobian = {{
        {{
            config_.theta1_sign *
                -(2.0 * u1 * d.dp1_dpitch + 2.0 * v1 * d.dq1_dpitch) /
                jt1,
            config_.theta1_sign *
                -(2.0 * u1 * d.dp1_droll + 2.0 * v1 * d.dq1_droll +
                  2.0 * g.z1 * d.dz1_droll) /
                jt1,
        }},
        {{
            config_.theta2_sign *
                -(2.0 * u2 * d.dp2_dpitch + 2.0 * v2 * d.dq2_dpitch) /
                jt2,
            config_.theta2_sign *
                -(2.0 * u2 * d.dp2_droll + 2.0 * v2 * d.dq2_droll +
                  2.0 * g.z2 * d.dz2_droll) /
                jt2,
        }},
    }};
    return std::isfinite(jacobian[0][0]) &&
           std::isfinite(jacobian[0][1]) &&
           std::isfinite(jacobian[1][0]) &&
           std::isfinite(jacobian[1][1]);
}

bool ParallelWaistKinematics::motor_torque_to_joint_torque(
    const std::array<double, 2>& joint_rad,
    const std::array<double, 2>& motor_torque,
    std::array<double, 2>& joint_torque) const
{
    std::array<std::array<double, 2>, 2> jacobian{};
    if (!motor_jacobian(joint_rad, jacobian) ||
        !std::isfinite(motor_torque[0]) || !std::isfinite(motor_torque[1]))
    {
        return false;
    }
    joint_torque = {
        jacobian[0][0] * motor_torque[0] +
            jacobian[1][0] * motor_torque[1],
        jacobian[0][1] * motor_torque[0] +
            jacobian[1][1] * motor_torque[1],
    };
    return std::isfinite(joint_torque[0]) && std::isfinite(joint_torque[1]);
}

bool ParallelWaistKinematics::joint_torque_to_motor_torque(
    const std::array<double, 2>& joint_rad,
    const std::array<double, 2>& joint_torque,
    std::array<double, 2>& motor_torque) const
{
    std::array<std::array<double, 2>, 2> jacobian{};
    if (!motor_jacobian(joint_rad, jacobian) ||
        !std::isfinite(joint_torque[0]) || !std::isfinite(joint_torque[1]))
    {
        return false;
    }
    // tau_joint = J^T tau_motor.
    const double determinant =
        jacobian[0][0] * jacobian[1][1] -
        jacobian[0][1] * jacobian[1][0];
    if (!std::isfinite(determinant) || std::abs(determinant) < 1e-10)
    {
        return false;
    }
    motor_torque = {
        (jacobian[1][1] * joint_torque[0] -
         jacobian[1][0] * joint_torque[1]) /
            determinant,
        (-jacobian[0][1] * joint_torque[0] +
         jacobian[0][0] * joint_torque[1]) /
            determinant,
    };
    return std::isfinite(motor_torque[0]) && std::isfinite(motor_torque[1]);
}

bool ParallelWaistKinematics::reflected_inertia_diagonal(
    const std::array<double, 2>& joint_rad,
    const std::array<double, 2>& motor_inertia,
    std::array<double, 2>& joint_inertia) const
{
    std::array<std::array<double, 2>, 2> jacobian{};
    if (!motor_jacobian(joint_rad, jacobian) ||
        !std::isfinite(motor_inertia[0]) || !std::isfinite(motor_inertia[1]) ||
        motor_inertia[0] < 0.0 || motor_inertia[1] < 0.0)
    {
        return false;
    }
    joint_inertia = {
        motor_inertia[0] * jacobian[0][0] * jacobian[0][0] +
            motor_inertia[1] * jacobian[1][0] * jacobian[1][0],
        motor_inertia[0] * jacobian[0][1] * jacobian[0][1] +
            motor_inertia[1] * jacobian[1][1] * jacobian[1][1],
    };
    return std::isfinite(joint_inertia[0]) &&
           std::isfinite(joint_inertia[1]) && joint_inertia[0] >= 0.0 &&
           joint_inertia[1] >= 0.0;
}

bool ParallelWaistKinematics::solve_from_seed(
    double theta1,
    double theta2,
    double seed_roll,
    double seed_pitch,
    double& roll,
    double& pitch) const
{
    roll = seed_roll;
    pitch = seed_pitch;
    for (int iteration = 0; iteration < 100; ++iteration)
    {
        const Geometry g = geometry(roll, pitch);
        const double u1 = g.p1 - config_.lc * std::sin(theta1);
        const double v1 = g.q1 + config_.lc * std::cos(theta1);
        const double u2 = g.p2 - config_.lc * std::sin(theta2);
        const double v2 = g.q2 + config_.lc * std::cos(theta2);
        const double f1 =
            u1 * u1 + v1 * v1 + g.z1 * g.z1 - config_.ll * config_.ll;
        const double f2 =
            u2 * u2 + v2 * v2 + g.z2 * g.z2 - config_.ll * config_.ll;
        if (std::hypot(f1, f2) < 1e-9)
        {
            return true;
        }

        const double epsilon = 1e-6;
        const auto residual = [&](double r, double p, int row) {
            const Geometry h = geometry(r, p);
            const double u = (row == 0 ? h.p1 : h.p2) -
                             config_.lc * std::sin(row == 0 ? theta1 : theta2);
            const double v = (row == 0 ? h.q1 : h.q2) +
                             config_.lc * std::cos(row == 0 ? theta1 : theta2);
            const double z = row == 0 ? h.z1 : h.z2;
            return u * u + v * v + z * z - config_.ll * config_.ll;
        };
        const double j11 = (residual(roll + epsilon, pitch, 0) - f1) / epsilon;
        const double j12 = (residual(roll, pitch + epsilon, 0) - f1) / epsilon;
        const double j21 = (residual(roll + epsilon, pitch, 1) - f2) / epsilon;
        const double j22 = (residual(roll, pitch + epsilon, 1) - f2) / epsilon;
        const double determinant = j11 * j22 - j12 * j21;
        if (!std::isfinite(determinant) || std::abs(determinant) < 1e-10)
        {
            return false;
        }
        roll += (-f1 * j22 + j12 * f2) / determinant;
        pitch += (j21 * f1 - j11 * f2) / determinant;
        if (!std::isfinite(roll) || !std::isfinite(pitch))
        {
            return false;
        }
    }
    return false;
}

bool ParallelWaistKinematics::motor_to_joint(
    const std::array<double, 2>& motor_rad,
    std::array<double, 2>& joint_rad) const
{
    const double theta1 = config_.theta1_sign * motor_rad[0];
    const double theta2 = config_.theta2_sign * motor_rad[1];
    constexpr std::array<std::array<double, 2>, 6> seeds{{
        {{0.0, 0.0}},
        {{0.0, 40.0 * kPi / 180.0}},
        {{0.0, 80.0 * kPi / 180.0}},
        {{20.0 * kPi / 180.0, 80.0 * kPi / 180.0}},
        {{-20.0 * kPi / 180.0, 80.0 * kPi / 180.0}},
        {{0.0, 120.0 * kPi / 180.0}},
    }};
    double best_score = std::numeric_limits<double>::infinity();
    std::array<double, 2> best{};
    for (const auto& seed : seeds)
    {
        double roll = 0;
        double pitch = 0;
        if (!solve_from_seed(theta1, theta2, seed[0], seed[1], roll, pitch) ||
            pitch < -35.0 * kPi / 180.0 || pitch > 125.0 * kPi / 180.0 ||
            roll < -35.0 * kPi / 180.0 || roll > 35.0 * kPi / 180.0)
        {
            continue;
        }
        double check1 = 0;
        double check2 = 0;
        if (!inverse(roll, pitch, check1, check2, config_.elbow))
        {
            continue;
        }
        const double score =
            std::hypot(wrap(check1 - theta1), wrap(check2 - theta2));
        if (score < best_score)
        {
            best_score = score;
            best = {pitch, roll};
        }
    }
    if (best_score > 3.0 * kPi / 180.0)
    {
        return false;
    }
    joint_rad = best;
    return true;
}

}  // namespace sharpa::mujoco::plugins::torso
