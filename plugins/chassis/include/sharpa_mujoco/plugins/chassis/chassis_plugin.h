// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/device_plugin.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace sharpa::mujoco::plugins::chassis
{

struct ChassisUdpEndpointConfig
{
    std::string bind_ip = "127.0.0.1";
    std::uint16_t bind_port = 0;
    std::string peer_ip = "127.0.0.1";
    std::uint16_t peer_port = 0;
    std::uint32_t uplink_source_code = 0x02050000U;
};

struct ChassisAxisConfig
{
    std::string joint;
    std::string actuator;
    double sign = 1.0;
};

enum class ChassisMotorFamily
{
    Jiechang,
    Kinco,
};

enum class ChassisStretchFamily
{
    M42p,
    Jiechang,
};

struct ChassisPluginConfig
{
    ChassisMotorFamily motor_family = ChassisMotorFamily::Jiechang;
    ChassisStretchFamily stretch_family = ChassisStretchFamily::M42p;
    std::array<std::uint8_t, 4> stretch_can_interfaces{0, 0, 1, 1};
    // Jiechang: 262144 counts/revolution, 1 mm screw lead, gear ratio 1.
    double jiechang_stretch_counts_per_meter = 262144000.0;
    ChassisUdpEndpointConfig jiechang_endpoint{
        "127.0.0.1", 45620, "127.0.0.1", 45220, 0x02050000U};
    ChassisUdpEndpointConfig kinco_endpoint{
        "127.0.0.1", 45620, "127.0.0.1", 45220, 0x02050000U};
    ChassisUdpEndpointConfig m42p_endpoint{
        "127.0.0.1", 45621, "127.0.0.1", 45221, 0x02050000U};
    std::array<std::uint8_t, 8> motor_can_interfaces{1, 0, 0, 1, 1, 0, 0, 1};
    std::array<std::uint8_t, 4> m42p_rs485_interfaces{0, 0, 1, 1};
    std::array<ChassisAxisConfig, 12> axes{{
        {"chassis_steer_joint_1", "chassis_steer_joint_1_position", -1.0},
        {"chassis_steer_joint_2", "chassis_steer_joint_2_position", -1.0},
        {"chassis_steer_joint_3", "chassis_steer_joint_3_position", -1.0},
        {"chassis_steer_joint_4", "chassis_steer_joint_4_position", -1.0},
        {"chassis_wheel_joint_1", "chassis_wheel_joint_1_velocity", -1.0},
        {"chassis_wheel_joint_2", "chassis_wheel_joint_2_velocity", 1.0},
        {"chassis_wheel_joint_3", "chassis_wheel_joint_3_velocity", 1.0},
        {"chassis_wheel_joint_4", "chassis_wheel_joint_4_velocity", -1.0},
        {"chassis_slider_joint_1", "chassis_slider_joint_1_position", 1.0},
        {"chassis_slider_joint_2", "chassis_slider_joint_2_position", 1.0},
        {"chassis_slider_joint_3", "chassis_slider_joint_3_position", 1.0},
        {"chassis_slider_joint_4", "chassis_slider_joint_4_position", 1.0},
    }};
    double feedback_rate_hz = 100.0;
    double command_timeout_s = 0.1;
    double slider_steps_per_meter = 2133333.333333;
    double slider_minimum_m = 0.0;
    double slider_maximum_m = 0.08;
    double steer_gear_ratio = 47.25;
    double wheel_gear_ratio = 6.28;
    double steer_rated_current_a = 1.2;
    double wheel_rated_current_a = 3.2;
    double steer_rated_motor_torque_nm = 0.26;
    double wheel_rated_motor_torque_nm = 0.50;
    double steer_position_kp = 120.0;
    double steer_position_kd = 12.0;
    double wheel_velocity_kv = 15.0;
    double wheel_velocity_ki = 100.0;
    double wheel_velocity_integral_limit_nm = 3.14;
    double jiechang_steer_torque_limit_nm = 12.285;
    double jiechang_wheel_torque_limit_nm = 3.14;
    double jiechang_mit_kp_scale = 1.0;
    double jiechang_mit_kd_scale = 1.0;

    std::uint32_t kinco_encoder_resolution = 65536;
    double kinco_steer_gear_ratio = 9.0;
    double kinco_wheel_gear_ratio = 10.0;
    double kinco_steer_rated_current_a = 6.5;
    double kinco_wheel_rated_current_a = 8.5;
    double kinco_steer_max_current_arms = 14.3;
    double kinco_wheel_max_current_arms = 25.0;
    double kinco_steer_output_kt_nm_per_a = 6.0 / 6.5;
    double kinco_wheel_output_kt_nm_per_a = 9.0 / 8.5;
    double kinco_steer_position_kp = 120.0;
    double kinco_steer_position_kd = 12.0;
    // Zero retains direct position PD. Positive values apply the programmed
    // Kinco profile ramps (0x6083/0x6084) in output-shaft units.
    double kinco_steer_profile_acceleration_scale = 0.0;
    double kinco_steer_command_delay_s = 0.0;
    double kinco_steer_velocity_feedforward = 0.0;
    bool kinco_control_in_step = false;
    double kinco_wheel_velocity_kp = 15.0;
    double kinco_wheel_velocity_ki = 100.0;
    double kinco_wheel_velocity_integral_limit_nm = 9.0;
    double kinco_steer_continuous_torque_nm = 6.0;
    double kinco_steer_peak_torque_nm = 13.5;
    double kinco_steer_peak_speed_rpm = 245.0;
    double kinco_steer_rated_speed_rpm = 277.0;
    double kinco_wheel_continuous_torque_nm = 9.0;
    double kinco_wheel_peak_torque_nm = 27.0;
    double kinco_wheel_peak_speed_rpm = 200.0;
    double kinco_wheel_rated_speed_rpm = 280.0;
};

class ChassisPlugin final : public IDevicePlugin
{
public:
    explicit ChassisPlugin(ChassisPluginConfig config = {});
    ~ChassisPlugin() override;

    ChassisPlugin(const ChassisPlugin&) = delete;
    ChassisPlugin& operator=(const ChassisPlugin&) = delete;

    std::string name() const override;
    bool start(Runtime& runtime, std::string* error = nullptr) override;
    void stop() noexcept override;
    bool is_running() const noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sharpa::mujoco::plugins::chassis
