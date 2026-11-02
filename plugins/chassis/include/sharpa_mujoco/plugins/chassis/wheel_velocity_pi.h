// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <cmath>

namespace sharpa::mujoco::plugins::chassis
{

struct WheelVelocityPiParams
{
    double kp = 15.0;                   // Nm / (rad/s), output shaft.
    double ki = 100.0;                  // Nm / rad, output shaft.
    double integral_limit_nm = 9.0;     // Absolute integral contribution.
    double torque_limit_nm = 9.0;       // Available total output torque.
    double max_step_s = 0.1;            // Do not integrate over control gaps.
};

/** Per-wheel PI. Call with simulation time, not packet-arrival wall time.
 * Repeated submissions of the same snapshot do not advance the integral.
 * The owner resets on disable, mode changes, timeout and runtime reset.
 */
class WheelVelocityPi
{
public:
    void reset() noexcept
    {
        integral_nm_ = 0.0;
        last_time_s_ = -1.0;
    }

    double update(double target_rad_s, double measured_rad_s,
                  double simulation_time_s, const WheelVelocityPiParams& p)
    {
        if (!std::isfinite(target_rad_s) || !std::isfinite(measured_rad_s) ||
            !std::isfinite(simulation_time_s))
        {
            reset();
            return 0.0;
        }
        double dt = last_time_s_ < 0.0 ? 0.0 : simulation_time_s - last_time_s_;
        if (dt < 0.0 || dt > p.max_step_s)
        {
            reset();
            dt = 0.0;
        }
        last_time_s_ = simulation_time_s;
        const double limit = std::max(0.0, p.torque_limit_nm);
        const double i_limit = std::min(limit, p.integral_limit_nm);
        integral_nm_ = p.ki > 0.0 ? std::clamp(integral_nm_, -i_limit, i_limit) : 0.0;
        const double error = target_rad_s - measured_rad_s;
        const double proportional = p.kp * error;
        const double candidate = std::clamp(
            integral_nm_ + p.ki * error * dt, -i_limit, i_limit);
        const double change = candidate - integral_nm_;
        const double unsaturated = proportional + candidate;
        // Conditional integration: do not wind up further into saturation;
        // allow opposite-sign error to unwind an existing integral.
        if (!((unsaturated > limit && change > 0.0) ||
              (unsaturated < -limit && change < 0.0)))
        {
            integral_nm_ = candidate;
        }
        return std::clamp(proportional + integral_nm_, -limit, limit);
    }

private:
    double integral_nm_ = 0.0;
    double last_time_s_ = -1.0;
};

}  // namespace sharpa::mujoco::plugins::chassis
