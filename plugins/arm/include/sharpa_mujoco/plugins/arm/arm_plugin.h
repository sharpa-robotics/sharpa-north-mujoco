// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sharpa_mujoco/device_plugin.h"
#include "sharpa_mujoco/plugins/arm/arm_transmission.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sharpa::mujoco::plugins::arm
{

enum class ArmCspControlSpace
{
    Joint,
    Motor,
};

struct ArmUdpEndpointConfig
{
    std::string bind_ip = "127.0.0.1";
    std::uint16_t bind_port = 21000;
    std::string peer_ip = "127.0.0.1";
    std::uint16_t peer_port = 8080;
    std::vector<std::size_t> motor_indices;
};

struct ArmPluginConfig
{
    std::string name = "arm_left";
    ArmSide side = ArmSide::Left;
    double feedback_rate_hz = 1000.0;
    double command_timeout_s = 0.1;
    // Fixed wall-clock transport delays in motor order, seconds; zero bypasses.
    // Motors on the same board must use equal delays. Range [0, 0.1];
    // command delay must be smaller than command_timeout_s.
    // Enable/stop/mode/homing transitions bypass and flush delayed commands.
    std::array<double, 7> command_delay_s{};
    std::array<double, 7> feedback_delay_s{};
    double quick_stop_deceleration_rad_s2 = 10.0;
    double quick_stop_velocity_threshold_rad_s = 0.02;
    double quick_stop_timeout_s = 1.0;
    double quick_stop_kp = 20.0;
    double speed_kp = 5.0;
    double speed_ki = 20.0;
    double speed_integral_limit_nm = 20.0;
    bool diagnostic_logging = false;
    bool extended_feedback = true;
    // Requires a distinct bind IP per board. Set 0 for legacy PDO-only setups.
    std::uint16_t sdo_port = 65530;
    double bus_voltage_v = 48.0;
    double driver_temperature_c = 25.0;
    double motor_temperature_c = 25.0;
    ArmCspControlSpace csp_control_space = ArmCspControlSpace::Joint;
    std::array<double, 7> sim_joint_kp{
        500.0, 500.0, 300.0, 200.0, 50.0, 50.0, 50.0};
    std::array<double, 7> sim_joint_kd{
        7.5, 7.5, 2.0, 1.0, 1.0, 1.0, 1.0};
    // Motor-space CSP gains; also used for the physical-pose hold in Homing.
    std::array<double, 7> csp_kp{
        100.0, 100.0, 100.0, 40.0, 40.0, 30.0, 30.0};
    std::array<double, 7> csp_kd{
        10.0, 10.0, 10.0, 5.0, 5.0, 5.0, 5.0};
    std::array<double, 7> torque_limits_nm{
        30.0, 30.0, 30.0, 5.0, 5.0, 5.0, 5.0};
    std::array<ArmUdpEndpointConfig, 5> endpoints;
};

class ArmPlugin final : public IDevicePlugin
{
public:
    explicit ArmPlugin(ArmPluginConfig config);
    ~ArmPlugin() override;

    std::string name() const override;
    bool start(Runtime& runtime, std::string* error) override;
    void stop() noexcept override;
    bool is_running() const noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sharpa::mujoco::plugins::arm
