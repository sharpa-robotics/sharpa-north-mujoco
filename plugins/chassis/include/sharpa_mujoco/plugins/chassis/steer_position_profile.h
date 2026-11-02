// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <utility>

namespace sharpa::mujoco::plugins::chassis
{

inline double kinco_profile_ramp_rad_s2(std::uint32_t raw,
                                        std::uint32_t encoder_resolution,
                                        double gear_ratio)
{
    // 0x6083/0x6084: DEC = RPS/s * 65536 * encoder_resolution / 4e6.
    // This differs from the 0x6081 velocity encoding.
    constexpr double two_pi = 6.28318530717958647692;
    return static_cast<double>(raw) * 4000000.0 * two_pi /
        (65536.0 * encoder_resolution * gear_ratio);
}

struct SteerReference
{
    double position = 0.0;
    double velocity = 0.0;
};

// Output-shaft profile reference. The plant still moves only through actuator
// torque: this never writes measured joint position or velocity.
class SteerPositionProfile
{
public:
    void reset() noexcept
    {
        last_time_ = -1.0;
        history_.clear();
        reference_ = {};
    }

    SteerReference update(double target, double measured, double time,
                          double velocity_limit, double acceleration,
                          double deceleration, double delay, double max_step)
    {
        if (!std::isfinite(target) || !std::isfinite(measured) ||
            !std::isfinite(time) || !std::isfinite(velocity_limit) ||
            !std::isfinite(acceleration) || !std::isfinite(deceleration) ||
            !std::isfinite(delay) || velocity_limit <= 0.0 ||
            acceleration <= 0.0 || deceleration <= 0.0 || delay < 0.0)
        {
            reset();
            return {std::isfinite(measured) ? measured : 0.0, 0.0};
        }
        if (last_time_ < 0.0 || time < last_time_ || time - last_time_ > max_step)
        {
            reset();
            reference_ = {measured, 0.0};
            delayed_target_ = measured;
            last_time_ = time;
        }
        if (!history_.empty() && history_.back().first == time)
            history_.back().second = target;
        else
            history_.emplace_back(time, target);
        while (!history_.empty() && history_.front().first <= time - delay)
        {
            delayed_target_ = history_.front().second;
            history_.pop_front();
        }
        const double dt = time - last_time_;
        last_time_ = time;
        if (dt <= 0.0) return reference_;
        const double error = delayed_target_ - reference_.position;
        const double braking_speed = std::max(
            0.0, std::sqrt(2.0 * deceleration * std::abs(error)) - deceleration * dt);
        const double desired = std::copysign(
            std::min(velocity_limit, braking_speed), error);
        const bool slowing = reference_.velocity * desired < 0.0 ||
            std::abs(desired) < std::abs(reference_.velocity);
        const double change = (slowing ? deceleration : acceleration) * dt;
        const double velocity = reference_.velocity +
            std::clamp(desired - reference_.velocity, -change, change);
        reference_.position += 0.5 * (reference_.velocity + velocity) * dt;
        reference_.velocity = velocity;
        return reference_;
    }

private:
    double last_time_ = -1.0;
    double delayed_target_ = 0.0;
    SteerReference reference_;
    std::deque<std::pair<double, double>> history_;
};
}  // namespace sharpa::mujoco::plugins::chassis
