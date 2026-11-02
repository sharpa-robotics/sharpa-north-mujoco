// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/chassis/wheel_velocity_pi.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace
{
using sharpa::mujoco::plugins::chassis::WheelVelocityPi;
using sharpa::mujoco::plugins::chassis::WheelVelocityPiParams;

bool require(bool condition, const char* message)
{
    if (!condition) std::cerr << "FAILED: " << message << '\n';
    return condition;
}

bool test_zero_speed_disturbance_rejection()
{
    // A freely moving output shaft, with no friction or position constraint.
    // PI must balance a sustained external torque at zero speed and return
    // toward its original angle after that external torque is removed.
    auto run = [](double ki) {
        WheelVelocityPi controller;
        WheelVelocityPiParams p;
        p.ki = ki;
        double velocity = 0.0, angle = 0.0, control = 0.0;
        double held_speed = 0.0, held_torque = 0.0, held_angle = 0.0;
        for (int step = 0; step <= 7000; ++step)
        {
            const double t = step * 0.001;
            if (step % 5 == 0) control = controller.update(0.0, velocity, t, p);
            const double disturbance = t >= 1.0 && t < 4.0 ? 1.0 : 0.0;
            velocity += (control + disturbance) / 0.1 * 0.001;
            angle += velocity * 0.001;
            if (step == 3999)
            {
                held_speed = velocity;
                held_torque = control;
                held_angle = angle;
            }
        }
        return std::array<double, 4>{held_speed, held_torque, held_angle, angle};
    };
    const auto pi = run(100.0), proportional = run(0.0);
    std::cout << "PI held speed=" << pi[0] << " torque=" << pi[1]
              << " displaced=" << pi[2] << " released angle=" << pi[3]
              << "; P held speed=" << proportional[0] << '\n';
    return require(std::abs(pi[0]) < 1e-4, "PI did not reject the constant disturbance.") &&
           require(std::abs(pi[1] + 1.0) < 1e-3, "Zero-speed PI lost its balancing torque.") &&
           require(std::abs(pi[2] - 0.01) < 1e-3, "PI holding stiffness has the wrong scale.") &&
           require(std::abs(pi[3]) < 1e-4, "PI did not restore the displaced shaft after release.") &&
           require(proportional[0] > 0.05, "Ki=0 did not recover proportional-only drift.");
}

bool test_saturation_and_reset()
{
    WheelVelocityPi controller;
    WheelVelocityPiParams p;
    p.kp = 1.0; p.ki = 10.0; p.integral_limit_nm = 0.5; p.torque_limit_nm = 1.0;
    for (int n = 0; n < 1000; ++n)
        if (!require(controller.update(100.0, 0.0, n * 0.01, p) == 1.0, "Torque limit was exceeded.")) return false;
    if (!require(controller.update(0.0, 0.0, 10.0, p) == 0.0, "Saturation wound up the integral.")) return false;
    controller.reset();
    controller.update(0.1, 0.0, 0.0, p);
    for (int n = 1; n < 100; ++n) controller.update(0.1, 0.0, n * 0.01, p);
    if (!require(std::abs(controller.update(0.0, 0.0, 1.0, p) - 0.5) < 1e-9, "Integral torque clamp failed.")) return false;
    const double held = controller.update(0.0, 0.0, 1.0, p);
    for (int n = 0; n < 100; ++n)
        if (!require(controller.update(0.0, 0.0, 1.0, p) == held, "Duplicate snapshot integrated twice.")) return false;
    controller.reset();
    if (!require(controller.update(0.0, 0.0, 1.01, p) == 0.0, "Reset retained stale torque.")) return false;
    controller.update(0.1, 0.0, 1.02, p);
    controller.update(0.1, 0.0, 1.03, p);
    if (!require(controller.update(0.0, 0.0, 0.0, p) == 0.0, "Simulation rewind retained torque.")) return false;
    controller.update(0.1, 0.0, 0.01, p);
    if (!require(controller.update(0.0, 0.0, 2.0, p) == 0.0, "Control gap integrated stale error.")) return false;
    controller.update(0.1, 0.0, 2.01, p);
    p.ki = 0.0;
    return require(controller.update(0.0, 0.0, 2.02, p) == 0.0, "Disabling Ki retained torque.") &&
           require(controller.update(0.0, std::numeric_limits<double>::quiet_NaN(), 2.03, p) == 0.0,
                   "Nonfinite feedback produced a nonzero command.");
}
}  // namespace

int main()
{
    return test_zero_speed_disturbance_rejection() && test_saturation_and_reset()
               ? EXIT_SUCCESS : EXIT_FAILURE;
}
