// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "wave_hand_controller.h"

#include "wave_hand_limit_tables.generated.h"

#include <algorithm>
#include <cmath>

namespace sharpa::mujoco::plugins::wave_hand
{
namespace
{

constexpr float kPi = 3.14159265358979323846F;
constexpr float kDegreesToRadians = kPi / 180.0F;
constexpr float kSimulationAccelerationLimit = 500.0F;
constexpr float kAdmittanceTorqueLimit = 1.0F;
constexpr float kAdmittanceAccelerationLimit = 1000.0F;
constexpr float kHardLimitTolerance = 3.0F * kDegreesToRadians;

constexpr std::uint8_t kStiffnessCoefficientId = 0;
constexpr std::uint8_t kMassId = 46;
constexpr std::uint8_t kDampingId = 47;
constexpr std::uint8_t kStiffnessId = 48;
constexpr std::uint8_t kTorqueThresholdId = 49;
constexpr std::uint8_t kSpeedLimitId = 50;
constexpr std::uint8_t kFrictionEnableId = 51;
constexpr std::uint8_t kCoulombFrictionId = 52;
constexpr std::uint8_t kViscousFrictionId = 53;
constexpr std::uint8_t kStaticFrictionId = 54;
constexpr std::uint8_t kStribeckVelocityId = 55;
constexpr std::uint8_t kMitKpId = 62;
constexpr std::uint8_t kMitKdId = 63;
constexpr std::uint8_t kVelocityFilterAlphaId = 64;
constexpr std::uint8_t kSoftMinimumId = 36;
constexpr std::uint8_t kSoftMaximumId = 37;

constexpr std::array<float, kWaveJointCount> kSoftMinimum{{
    -5, -18, -25, -18, 0, -10, -18, 0, 0, -10, -18,
    0, 0, -10, -18, 0, 0, 0, -10, -18, 0, 0,
}};
constexpr std::array<float, kWaveJointCount> kSoftMaximum{{
    105, 18, 75, 18, 95, 84, 18, 95, 75, 84, 18,
    95, 75, 84, 18, 95, 75, 25, 84, 18, 95, 75,
}};
constexpr std::array<float, kWaveJointCount> kHardMinimum{{
    -10, -20, -30, -20, 0, -50, -20, 0, 0, -50, -20,
    0, 0, -50, -20, 0, 0, 0, -50, -20, 0, 0,
}};
constexpr std::array<float, kWaveJointCount> kHardMaximum{{
    110, 20, 80, 20, 100, 90, 20, 100, 80, 90, 20,
    100, 80, 90, 20, 100, 80, 25, 90, 20, 100, 80,
}};

constexpr std::array<float, kWaveJointCount> kMitKp{{
    20, 20, 8, 25, 4, 5, 20, 4, 1, 5, 20,
    4, 1, 5, 20, 4, 1, 2, 5, 20, 4, 1,
}};
constexpr std::array<float, kWaveJointCount> kMitKd{{
    .5F, .5F, .2F, .25F, .1F, .2F, .25F, .1F, .01F, .2F, .25F,
    .1F, .01F, .2F, .25F, .1F, .01F, .01F, .2F, .25F, .1F, .01F,
}};
constexpr std::array<float, kWaveJointCount> kAdmittanceStiffness{{
    3.6F, 3.6F, 2.4F, 2.4F, 2.4F, 3.6F, 3.6F, 2.4F, .8F, 3.6F, 3.6F,
    2.4F, .8F, 3.6F, 3.6F, 2.4F, .8F, 3.6F, 3.6F, 3.6F, 2.4F, .8F,
}};
constexpr std::array<float, kWaveJointCount> kAdmittanceDamping{{
    .25F, .25F, .25F, .25F, .15F, .25F, .25F, .15F, .05F, .25F, .25F,
    .15F, .05F, .25F, .25F, .15F, .05F, .25F, .25F, .25F, .15F, .05F,
}};
constexpr std::array<float, kWaveJointCount> kAdmittanceThreshold{{
    .24F, .24F, .12F, .12F, .08F, .12F, .12F, .08F, .08F, .12F, .12F,
    .08F, .08F, .12F, .12F, .08F, .08F, .24F, .12F, .12F, .08F, .08F,
}};

constexpr std::array<std::size_t, 6> kDifferenceJoints{{1, 3, 6, 10, 14, 19}};
constexpr std::array<std::size_t, 6> kCommonJoints{{0, 2, 5, 9, 13, 18}};
constexpr std::array<const detail::DifferentialLimitTable*, 6> kLimitTables{{
    &detail::thumb_cmc_side_lookup_table,
    &detail::thumb_mcp_side_lookup_table,
    &detail::index_mcp_side_lookup_table,
    &detail::middle_mcp_side_lookup_table,
    &detail::ring_mcp_side_lookup_table,
    &detail::pinky_mcp_side_lookup_table,
}};

float radians(float degrees)
{
    return degrees * kDegreesToRadians;
}

}  // namespace

WaveHandController::WaveHandController()
{
    for (std::size_t i = 0; i < kWaveJointCount; ++i)
    {
        parameters_[i].mass = 0.001F;
        parameters_[i].damping = kAdmittanceDamping[i];
        parameters_[i].stiffness = kAdmittanceStiffness[i];
        parameters_[i].torque_threshold = kAdmittanceThreshold[i];
        parameters_[i].speed_limit = 4.0F * kPi;
        parameters_[i].friction_enable =
            i == 5 || i == 9 || i == 13 || i == 18 ? 1U : 0U;
        parameters_[i].mit_kp = kMitKp[i];
        parameters_[i].mit_kd = kMitKd[i];
        parameters_[i].torque_damping =
            i == 8 || i == 12 || i == 16 || i == 21 ? 0.01F : 0.0F;
    }
}

void WaveHandController::reset(
    const std::array<float, kWaveJointCount>& positions)
{
    targets_ = {};
    targets_.position = positions;
    for (std::size_t i = 0; i < kWaveJointCount; ++i)
    {
        state_[i] = {};
        state_[i].applied_position = positions[i];
    }
}

void WaveHandController::set_targets(const WaveJointTargets& targets)
{
    targets_ = targets;
}

WaveControlOutput WaveHandController::update(
    WaveControlMode mode,
    float timestep,
    float speed_coefficient,
    const WaveJointState& state)
{
    WaveControlOutput output;
    output.torque_mode =
        mode == WaveControlMode::Floating || mode == WaveControlMode::Mit;
    if (!(timestep > 0.0F) || !std::isfinite(timestep)) return output;

    for (std::size_t i = 0; i < kWaveJointCount; ++i)
    {
        state_[i].filtered_velocity =
            velocity_filter_alpha_ * state.velocity[i] +
            (1.0F - velocity_filter_alpha_) * state_[i].filtered_velocity;
        if (mode == WaveControlMode::Floating)
        {
            output.value[i] = 0.0F;
            continue;
        }

        const float target = limit_target(i, targets_.position[i], state.position);
        if (mode == WaveControlMode::Mit)
        {
            const float error = target - state.position[i];
            output.value[i] =
                targets_.torque[i] + parameters_[i].mit_kp * error +
                parameters_[i].mit_kd *
                    (targets_.velocity[i] - state.velocity[i]) +
                friction_compensation(i, state_[i].filtered_velocity, error) -
                parameters_[i].torque_damping * state.velocity[i];
            continue;
        }

        const float requested = mode == WaveControlMode::Admittance
                                    ? update_admittance(
                                          i, target, state.effort[i], timestep)
                                    : target;
        const float maximum_velocity =
            parameters_[i].speed_limit * speed_coefficient;
        const float desired_velocity = std::clamp(
            (requested - state_[i].applied_position) / timestep,
            -maximum_velocity, maximum_velocity);
        const float maximum_change = kSimulationAccelerationLimit * timestep;
        state_[i].applied_velocity += std::clamp(
            desired_velocity - state_[i].applied_velocity,
            -maximum_change, maximum_change);
        float step = state_[i].applied_velocity * timestep;
        const float remaining = requested - state_[i].applied_position;
        if ((step > 0.0F && step > remaining) ||
            (step < 0.0F && step < remaining))
        {
            step = remaining;
            state_[i].applied_velocity = 0.0F;
        }
        state_[i].applied_position += step;
        output.value[i] = state_[i].applied_position;
    }
    return output;
}

std::array<float, 2> WaveHandController::differential_limits(
    std::size_t joint,
    float common_position) const
{
    const auto found = std::find(kDifferenceJoints.begin(),
                                 kDifferenceJoints.end(), joint);
    if (found == kDifferenceJoints.end())
        return {radians(kSoftMinimum[joint]), radians(kSoftMaximum[joint])};
    const std::size_t table_index =
        static_cast<std::size_t>(found - kDifferenceJoints.begin());
    const float coordinate = common_position / kDegreesToRadians + 49.0F;
    const auto& table = *kLimitTables[table_index];
    if (coordinate <= 0.0F)
        return {table.front().minimum, table.front().maximum};
    if (coordinate >= static_cast<float>(table.size() - 1))
        return {table.back().minimum, table.back().maximum};
    const std::size_t lower = static_cast<std::size_t>(coordinate);
    const float fraction = coordinate - static_cast<float>(lower);
    return {
        table[lower].minimum +
            fraction * (table[lower + 1].minimum - table[lower].minimum),
        table[lower].maximum +
            fraction * (table[lower + 1].maximum - table[lower].maximum),
    };
}

std::array<float, 2> WaveHandController::position_limits(
    std::size_t joint,
    const std::array<float, kWaveJointCount>& positions) const
{
    if (joint >= kWaveJointCount) return {0.0F, 0.0F};
    const auto found = std::find(kDifferenceJoints.begin(),
                                 kDifferenceJoints.end(), joint);
    if (found == kDifferenceJoints.end())
        return {radians(kSoftMinimum[joint]), radians(kSoftMaximum[joint])};
    const std::size_t index =
        static_cast<std::size_t>(found - kDifferenceJoints.begin());
    auto limits = differential_limits(joint, positions[kCommonJoints[index]]);
    limits[0] = std::max(limits[0], radians(kSoftMinimum[joint]));
    limits[1] = std::min(limits[1], radians(kSoftMaximum[joint]));
    return limits;
}

float WaveHandController::limit_target(
    std::size_t joint,
    float target,
    const std::array<float, kWaveJointCount>& positions) const
{
    const auto limits = position_limits(joint, positions);
    return std::clamp(target, limits[0], limits[1]);
}

float WaveHandController::friction_compensation(
    std::size_t joint,
    float velocity,
    float position_error) const
{
    const Parameters& p = parameters_[joint];
    if (p.friction_enable == 0) return 0.0F;
    const float absolute_velocity = std::fabs(velocity);
    const float ratio = absolute_velocity / p.stribeck_velocity;
    const float magnitude = p.coulomb_friction +
                            (p.static_friction - p.coulomb_friction) *
                                std::exp(-(ratio * ratio)) +
                            p.viscous_friction * absolute_velocity;
    constexpr float kVelocitySmooth = 0.05F;
    constexpr float kErrorDeadzone = 0.005F;
    const float velocity_weight =
        std::pow(std::tanh(absolute_velocity / kVelocitySmooth), 2.0F);
    const float velocity_direction = std::tanh(velocity / kVelocitySmooth);
    const float error_direction = std::fabs(position_error) > kErrorDeadzone
                                      ? std::tanh(position_error /
                                                  (kErrorDeadzone * 3.0F))
                                      : 0.0F;
    return magnitude *
           (velocity_weight * velocity_direction +
            (1.0F - velocity_weight) * error_direction);
}

float WaveHandController::update_admittance(
    std::size_t joint,
    float reference,
    float effort,
    float timestep)
{
    Parameters& p = parameters_[joint];
    State& s = state_[joint];
    if (effort > p.torque_threshold)
        effort -= p.torque_threshold;
    else if (effort < -p.torque_threshold)
        effort += p.torque_threshold;
    else
        effort = 0.0F;
    effort = std::clamp(effort, -kAdmittanceTorqueLimit,
                        kAdmittanceTorqueLimit);
    const float acceleration = p.mass > 0.0F
                                   ? std::clamp(
                                         (effort - p.damping *
                                                       s.admittance_velocity -
                                          stiffness_coefficient_ * p.stiffness *
                                              s.admittance_position) /
                                             p.mass,
                                         -kAdmittanceAccelerationLimit,
                                         kAdmittanceAccelerationLimit)
                                   : 0.0F;
    s.admittance_velocity += acceleration * timestep;

    const float predicted_delta =
        s.admittance_position + s.admittance_velocity * timestep;
    const float predicted_position = reference - predicted_delta;
    // The reference already uses the differential dynamic limit. Firmware
    // applies progressive resistance against the joint's fixed soft/hard
    // envelope, not against the moving differential boundary.
    const std::array<float, 2> soft{
        radians(kSoftMinimum[joint]), radians(kSoftMaximum[joint])};
    const float hard_minimum = radians(kHardMinimum[joint]);
    const float hard_maximum = radians(kHardMaximum[joint]);

    if (predicted_position < soft[0])
    {
        const float penetration = soft[0] - predicted_position;
        const float gap = soft[0] - hard_minimum;
        const float ratio = gap > 0.001F
                                ? std::clamp(penetration / gap, 0.0F, 1.0F)
                                : 1.0F;
        const float extra_force =
            ratio * stiffness_coefficient_ * p.stiffness * penetration;
        const float extra_damping = ratio * p.damping * 2.0F;
        if (s.admittance_velocity > 0.0F && p.mass > 0.0F)
        {
            s.admittance_velocity -=
                (extra_force + extra_damping * s.admittance_velocity) *
                timestep / p.mass;
            if (s.admittance_velocity <= 0.0F ||
                std::fabs(s.admittance_velocity) < 0.001F)
                s.admittance_velocity = 0.0F;
        }
        else if (s.admittance_velocity < 0.0F && p.mass > 0.0F)
        {
            s.admittance_velocity +=
                extra_damping * std::fabs(s.admittance_velocity) * timestep /
                p.mass;
        }
        if (predicted_position < hard_minimum - kHardLimitTolerance)
            s.admittance_velocity = 0.0F;
    }
    else if (predicted_position > soft[1])
    {
        const float penetration = predicted_position - soft[1];
        const float gap = hard_maximum - soft[1];
        const float ratio = gap > 0.001F
                                ? std::clamp(penetration / gap, 0.0F, 1.0F)
                                : 1.0F;
        const float extra_force =
            ratio * stiffness_coefficient_ * p.stiffness * penetration;
        const float extra_damping = ratio * p.damping * 2.0F;
        if (s.admittance_velocity < 0.0F && p.mass > 0.0F)
        {
            s.admittance_velocity +=
                (extra_force + extra_damping *
                                   std::fabs(s.admittance_velocity)) *
                timestep / p.mass;
            if (s.admittance_velocity >= 0.0F ||
                std::fabs(s.admittance_velocity) < 0.001F)
                s.admittance_velocity = 0.0F;
        }
        else if (s.admittance_velocity > 0.0F && p.mass > 0.0F)
        {
            s.admittance_velocity -=
                extra_damping * s.admittance_velocity * timestep / p.mass;
        }
        if (predicted_position > hard_maximum + kHardLimitTolerance)
            s.admittance_velocity = 0.0F;
    }

    s.admittance_position += s.admittance_velocity * timestep;
    return reference - s.admittance_position;
}

bool WaveHandController::set_float_parameter(
    std::uint8_t id,
    const std::array<float, kWaveJointCount>& values)
{
    float Parameters::* member = nullptr;
    float minimum = 0.0F;
    float maximum = 10.0F;
    switch (id)
    {
        case kMassId: member = &Parameters::mass; break;
        case kDampingId: member = &Parameters::damping; break;
        case kStiffnessId: member = &Parameters::stiffness; break;
        case kTorqueThresholdId: member = &Parameters::torque_threshold; break;
        case kSpeedLimitId:
            member = &Parameters::speed_limit;
            minimum = kPi;
            maximum = 5.0F * kPi;
            break;
        case kCoulombFrictionId:
            member = &Parameters::coulomb_friction;
            maximum = 1.0F;
            break;
        case kViscousFrictionId:
            member = &Parameters::viscous_friction;
            maximum = 0.1F;
            break;
        case kStaticFrictionId:
            member = &Parameters::static_friction;
            maximum = 1.0F;
            break;
        case kStribeckVelocityId:
            member = &Parameters::stribeck_velocity;
            minimum = 0.01F;
            maximum = 1.0F;
            break;
        case kMitKpId:
            member = &Parameters::mit_kp;
            maximum = 100.0F;
            break;
        case kMitKdId: member = &Parameters::mit_kd; break;
        default: return false;
    }
    for (float value : values)
        if (!std::isfinite(value) || value < minimum || value > maximum)
            return false;
    for (std::size_t i = 0; i < kWaveJointCount; ++i)
        parameters_[i].*member = values[i];
    return true;
}

bool WaveHandController::get_float_parameter(
    std::uint8_t id,
    std::array<float, kWaveJointCount>& values) const
{
    float Parameters::* member = nullptr;
    switch (id)
    {
        case kMassId: member = &Parameters::mass; break;
        case kDampingId: member = &Parameters::damping; break;
        case kStiffnessId: member = &Parameters::stiffness; break;
        case kTorqueThresholdId: member = &Parameters::torque_threshold; break;
        case kSpeedLimitId: member = &Parameters::speed_limit; break;
        case kCoulombFrictionId: member = &Parameters::coulomb_friction; break;
        case kViscousFrictionId: member = &Parameters::viscous_friction; break;
        case kStaticFrictionId: member = &Parameters::static_friction; break;
        case kStribeckVelocityId: member = &Parameters::stribeck_velocity; break;
        case kMitKpId: member = &Parameters::mit_kp; break;
        case kMitKdId: member = &Parameters::mit_kd; break;
        default: return false;
    }
    for (std::size_t i = 0; i < kWaveJointCount; ++i)
        values[i] = parameters_[i].*member;
    return true;
}

bool WaveHandController::set_u8_parameter(
    std::uint8_t id,
    const std::array<std::uint8_t, kWaveJointCount>& values)
{
    if (id != kFrictionEnableId) return false;
    for (std::uint8_t value : values)
        if (value > 1U) return false;
    for (std::size_t i = 0; i < kWaveJointCount; ++i)
        parameters_[i].friction_enable = values[i];
    return true;
}

bool WaveHandController::get_u8_parameter(
    std::uint8_t id,
    std::array<std::uint8_t, kWaveJointCount>& values) const
{
    if (id != kFrictionEnableId) return false;
    for (std::size_t i = 0; i < kWaveJointCount; ++i)
        values[i] = parameters_[i].friction_enable;
    return true;
}

bool WaveHandController::set_scalar_parameter(std::uint8_t id, float value)
{
    if (!std::isfinite(value)) return false;
    if (id == kStiffnessCoefficientId && value >= 0.0F && value <= 10.0F)
    {
        stiffness_coefficient_ = value;
        return true;
    }
    if (id == kVelocityFilterAlphaId && value >= 0.001F && value <= 1.0F)
    {
        velocity_filter_alpha_ = value;
        return true;
    }
    return false;
}

bool WaveHandController::get_scalar_parameter(std::uint8_t id,
                                               float& value) const
{
    if (id == kStiffnessCoefficientId)
        value = stiffness_coefficient_;
    else if (id == kVelocityFilterAlphaId)
        value = velocity_filter_alpha_;
    else
        return false;
    return true;
}

bool WaveHandController::get_readonly_float_parameter(
    std::uint8_t id,
    std::array<float, kWaveJointCount>& values) const
{
    const std::array<float, kWaveJointCount>* source = nullptr;
    if (id == kSoftMinimumId)
        source = &kSoftMinimum;
    else if (id == kSoftMaximumId)
        source = &kSoftMaximum;
    else
        return false;
    for (std::size_t i = 0; i < values.size(); ++i)
        values[i] = radians((*source)[i]);
    return true;
}

}  // namespace sharpa::mujoco::plugins::wave_hand
