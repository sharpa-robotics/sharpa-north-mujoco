// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/device_plugin.h"
#include "sharpa_mujoco/companion_process.h"
#include "sharpa_mujoco/plugins/arm/arm_plugin.h"
#include "sharpa_mujoco/plugins/chassis/chassis_plugin.h"
#include "sharpa_mujoco/plugins/lidar/lidar_plugin.h"
#include "sharpa_mujoco/plugins/torso/torso_plugin.h"
#include "sharpa_mujoco/plugins/wave_hand/wave_hand_plugin.h"
#include "sharpa_mujoco/plugins/tactile/tactile_plugin.h"
#include "sharpa_mujoco/runtime.h"
#include "sharpa_mujoco/simulator_service.h"
#include "lidar_config.h"
#include "model_assembler.h"
#include "simulator_config.h"
#include "simulator_factory.h"

#if SHARPA_MUJOCO_HAS_VIEWER
#include "sharpa_mujoco/viewer.h"
#endif

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <set>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

#ifndef SHARPA_MUJOCO_SERVICE_LIBRARY
std::atomic<bool> keep_running{true};

void handle_signal(int)
{
    keep_running.store(false);
}

void print_usage()
{
    std::cout
        << "Usage: sharpa_mujoco_simulator [options]\n"
        << "Options:\n"
        << "  --config <path>         Configuration file (default: sharpa_mujoco.conf)\n"
        << "  --model <path>          Override simulation.model\n"
        << "  --enable-device <name>  Enable a configured device\n"
        << "  --disable-device <name> Disable a configured device\n"
        << "  --lower-bind-port <p>   Override lower MCU simulator port\n"
        << "  --lower-peer-port <p>   Override lower host port\n"
        << "  --upper-bind-port <p>   Override upper MCU simulator port\n"
        << "  --upper-peer-port <p>   Override upper host port\n"
        << "  --feedback-rate <hz>    Override motor feedback rate\n"
        << "  --viewer                Enable the MuJoCo Viewer\n"
        << "  --no-viewer             Disable the MuJoCo Viewer\n"
        << "  --with-platform         Require the installed Platform companion\n"
        << "  --without-platform      Do not launch Platform (bundled default: launch)\n"
        << "  --platform-config <id>  Select a bundled Platform configuration\n"
        << "  --help                  Show this message\n";
}
#endif

std::uint16_t parse_port(const std::string& value)
{
    const unsigned long parsed = std::stoul(value);
    if (parsed == 0 || parsed > std::numeric_limits<std::uint16_t>::max())
    {
        throw std::out_of_range("port must be in range 1..65535");
    }
    return static_cast<std::uint16_t>(parsed);
}

double parse_positive_rate(const std::string& value)
{
    const double parsed = std::stod(value);
    if (parsed <= 0.0)
    {
        throw std::out_of_range("rate must be positive");
    }
    return parsed;
}

bool parse_bool(const std::string& value)
{
    if (value == "true" || value == "1")
    {
        return true;
    }
    if (value == "false" || value == "0")
    {
        return false;
    }
    throw std::invalid_argument("boolean value must be true, false, 1, or 0");
}

double parse_non_negative(const std::string& value, const char* name)
{
    const double parsed = std::stod(value);
    if (!std::isfinite(parsed) || parsed < 0.0)
    {
        throw std::out_of_range(std::string(name) + " must be finite and non-negative");
    }
    return parsed;
}

std::vector<std::uint8_t> parse_motor_ids(const std::string& value)
{
    std::vector<std::uint8_t> result;
    std::istringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ','))
    {
        const unsigned long id = std::stoul(item);
        if (id == 0 || id > 8)
        {
            throw std::out_of_range("motor ID must be in range 1..8");
        }
        result.push_back(static_cast<std::uint8_t>(id));
    }
    if (result.empty())
    {
        throw std::invalid_argument("motor ID list must not be empty");
    }
    return result;
}

std::array<std::uint8_t, 8> parse_motor_id_map(const std::string& value)
{
    const std::vector<std::uint8_t> parsed = parse_motor_ids(value);
    if (parsed.size() != 8)
    {
        throw std::invalid_argument("motor_ids must contain exactly 8 values");
    }
    std::array<std::uint8_t, 8> result{};
    std::copy(parsed.begin(), parsed.end(), result.begin());
    std::array<bool, 9> seen{};
    for (std::uint8_t id : result)
    {
        if (seen[id])
        {
            throw std::invalid_argument("motor_ids must not contain duplicates");
        }
        seen[id] = true;
    }
    return result;
}

std::array<double, 8> parse_motor_signs(const std::string& value)
{
    std::array<double, 8> result{};
    std::istringstream stream(value);
    std::string item;
    std::size_t index = 0;
    while (std::getline(stream, item, ','))
    {
        if (index >= result.size())
        {
            throw std::invalid_argument("motor_signs must contain exactly 8 values");
        }
        result[index++] = std::stod(item);
    }
    if (index != result.size())
    {
        throw std::invalid_argument("motor_signs must contain exactly 8 values");
    }
    return result;
}

std::array<double, 8> parse_non_negative_array(
    const std::string& value,
    const char* name)
{
    std::array<double, 8> result{};
    std::istringstream stream(value);
    std::string item;
    std::size_t index = 0;
    while (std::getline(stream, item, ','))
    {
        if (index >= result.size())
        {
            throw std::invalid_argument(
                std::string(name) + " must contain exactly 8 values");
        }
        result[index++] = parse_non_negative(item, name);
    }
    if (index != result.size())
    {
        throw std::invalid_argument(
            std::string(name) + " must contain exactly 8 values");
    }
    return result;
}

std::vector<std::size_t> parse_arm_motor_indices(const std::string& value)
{
    std::vector<std::size_t> result;
    std::istringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ','))
    {
        const unsigned long index = std::stoul(item);
        if (index > 6)
        {
            throw std::out_of_range("arm motor index must be in range 0..6");
        }
        result.push_back(static_cast<std::size_t>(index));
    }
    if (result.empty() || result.size() > 2)
    {
        throw std::invalid_argument(
            "arm endpoint motor_indices must contain one or two values");
    }
    return result;
}

std::array<double, 7> parse_arm_positive_array(
    const std::string& value,
    const char* name)
{
    std::array<double, 7> result{};
    std::istringstream stream(value);
    std::string item;
    std::size_t index = 0;
    while (std::getline(stream, item, ','))
    {
        if (index >= result.size())
        {
            throw std::invalid_argument(
                std::string(name) + " must contain exactly 7 values");
        }
        result[index++] = parse_positive_rate(item);
    }
    if (index != result.size())
    {
        throw std::invalid_argument(
            std::string(name) + " must contain exactly 7 values");
    }
    return result;
}

std::string option(
    const sharpa::mujoco::simulator::DeviceConfig& device,
    const std::string& key,
    const std::string& fallback)
{
    const auto found = device.options.find(key);
    return found == device.options.end() ? fallback : found->second;
}

sharpa::mujoco::plugins::torso::TorsoPluginConfig torso_config_from(
    const sharpa::mujoco::simulator::DeviceConfig& device)
{
    if (device.driver != "udp" && device.driver != "can")
    {
        throw std::invalid_argument(
            "devices.torso.driver must be 'udp' or 'can', got '" +
            device.driver + "'");
    }

    sharpa::mujoco::plugins::torso::TorsoPluginConfig config;
    config.transport = device.driver == "can"
        ? sharpa::mujoco::plugins::torso::TorsoTransport::SocketCan
        : sharpa::mujoco::plugins::torso::TorsoTransport::UdpMcu;
    const std::array<std::string, 2> prefixes{"lower_", "upper_"};
    for (std::size_t index = 0; index < config.endpoints.size(); ++index)
    {
        auto& endpoint = config.endpoints[index];
        const std::string& prefix = prefixes[index];
        endpoint.bind_ip = option(device, prefix + "bind_ip", endpoint.bind_ip);
        endpoint.bind_port = parse_port(option(
            device, prefix + "bind_port", std::to_string(endpoint.bind_port)));
        endpoint.peer_ip = option(device, prefix + "peer_ip", endpoint.peer_ip);
        endpoint.peer_port = parse_port(option(
            device, prefix + "peer_port", std::to_string(endpoint.peer_port)));
        const unsigned long can_interface = std::stoul(option(
            device, prefix + "can_if", std::to_string(endpoint.can_interface)));
        if (can_interface > 255)
        {
            throw std::out_of_range("CAN interface must be in range 0..255");
        }
        endpoint.can_interface = static_cast<std::uint8_t>(can_interface);
        endpoint.motor_ids = parse_motor_ids(option(
            device,
            prefix + "motor_ids",
            index == 0 ? "1,2,3,4,5" : "6,7,8"));
        config.can_interfaces[index] = option(
            device,
            prefix + "can_interface",
            config.can_interfaces[index]);
    }
    config.feedback_rate_hz = parse_positive_rate(option(
        device, "feedback_rate_hz", std::to_string(config.feedback_rate_hz)));
    config.heartbeat_rate_hz = parse_positive_rate(option(
        device, "heartbeat_rate_hz", std::to_string(config.heartbeat_rate_hz)));
    config.command_delay_s = parse_non_negative_array(
        option(device, "command_delay_s", "0,0,0,0,0,0,0,0"), "command_delay_s");
    config.feedback_delay_s = parse_non_negative_array(
        option(device, "feedback_delay_s", "0,0,0,0,0,0,0,0"), "feedback_delay_s");
    config.motor_ids = parse_motor_id_map(
        option(device, "motor_ids", "1,2,4,3,5,6,7,8"));
    config.motor_signs = parse_motor_signs(
        option(device, "motor_signs", "-1,-1,1,1,-1,1,1,1"));
    config.csp_kp = parse_non_negative_array(
        option(device, "csp_kp", "1248,1248,1248,1482.5,1482.5,50,50,50"),
        "csp_kp");
    config.csp_kd = parse_non_negative_array(
        option(device, "csp_kd", "5,5,5,5,5,0,0,0"),
        "csp_kd");
    config.command_velocity_filter_s = parse_non_negative_array(
        option(device, "command_velocity_filter_s", "0,0,0,0,0,0,0,0"),
        "command_velocity_filter_s");
    config.command_velocity_scale = parse_non_negative_array(
        option(device, "command_velocity_scale", "1,1,1,1,1,1,1,1"),
        "command_velocity_scale");
    config.mit_kp_scale = parse_non_negative_array(
        option(device, "mit_kp_scale", "1,1,1,1,1,1,1,1"),
        "mit_kp_scale");
    config.mit_kd_scale = parse_non_negative_array(
        option(device, "mit_kd_scale", "1,1,1,1,1,1,1,1"),
        "mit_kd_scale");
    config.mit_torque_ff_scale = parse_non_negative_array(
        option(device, "mit_torque_ff_scale", "1,1,1,1,1,1,1,1"),
        "mit_torque_ff_scale");
    config.mit_torque_limit_nm = parse_non_negative_array(
        option(device, "mit_torque_limit_nm", "300,300,300,300,300,300,300,300"),
        "mit_torque_limit_nm");
    config.mit_torque_time_constant_s = parse_non_negative_array(
        option(device, "mit_torque_time_constant_s", "0,0,0,0,0,0,0,0"),
        "mit_torque_time_constant_s");
    config.mit_reference_velocity_limit_rad_s = parse_non_negative_array(
        option(
            device,
            "mit_reference_velocity_limit_rad_s",
            "0,0,0,0,0,0,0,0"),
        "mit_reference_velocity_limit_rad_s");
    config.mit_reference_acceleration_limit_rad_s2 = parse_non_negative_array(
        option(
            device,
            "mit_reference_acceleration_limit_rad_s2",
            "0,0,0,0,0,0,0,0"),
        "mit_reference_acceleration_limit_rad_s2");
    config.motor_reduction_ratios = parse_non_negative_array(
        option(device, "motor_reduction_ratios", "161,161,161,161,161,101,101,101"),
        "motor_reduction_ratios");
    config.feedback_velocity_scale = parse_non_negative_array(
        option(device, "feedback_velocity_scale", "1,1,1,1,1,1,1,1"),
        "feedback_velocity_scale");
    if (!option(device, "feedback_velocity_ratio", "").empty())
        throw std::invalid_argument(
            "feedback_velocity_ratio is obsolete: use motor_reduction_ratios and "
            "feedback_velocity_scale (old ratio / physical reduction).");
    config.dynamic_waist_armature = parse_bool(option(
        device,
        "dynamic_waist_armature",
        config.dynamic_waist_armature ? "true" : "false"));
    config.waist_motor_inertia[0] = parse_non_negative(
        option(
            device,
            "waist_motor_1_inertia",
            std::to_string(config.waist_motor_inertia[0])),
        "waist_motor_1_inertia");
    config.waist_motor_inertia[1] = parse_non_negative(
        option(
            device,
            "waist_motor_2_inertia",
            std::to_string(config.waist_motor_inertia[1])),
        "waist_motor_2_inertia");
    return config;
}

sharpa::mujoco::plugins::chassis::ChassisPluginConfig chassis_config_from(
    const sharpa::mujoco::simulator::DeviceConfig& device)
{
    if (device.driver != "udp_mcu_jiechang" &&
        device.driver != "udp_mcu_kinco")
    {
        throw std::invalid_argument(
            "devices.chassis.driver must be 'udp_mcu_jiechang' or "
            "'udp_mcu_kinco', got '" +
            device.driver + "'");
    }
    sharpa::mujoco::plugins::chassis::ChassisPluginConfig config;
    config.motor_family =
        device.driver == "udp_mcu_kinco"
            ? sharpa::mujoco::plugins::chassis::ChassisMotorFamily::Kinco
            : sharpa::mujoco::plugins::chassis::ChassisMotorFamily::Jiechang;
    const std::string stretch_driver = option(device, "stretch_driver", "m42p");
    if (stretch_driver != "m42p" && stretch_driver != "jiechang")
    {
        throw std::invalid_argument("devices.chassis.stretch_driver must be 'm42p' or 'jiechang'");
    }
    config.stretch_family = stretch_driver == "jiechang"
        ? sharpa::mujoco::plugins::chassis::ChassisStretchFamily::Jiechang
        : sharpa::mujoco::plugins::chassis::ChassisStretchFamily::M42p;
    // MuJoCo steer axes are -Z and wheel axes are +Y. These signs convert
    // physical motor coordinates using Platform's selected wheel_mappings.
    const auto signs = parse_motor_signs(option(device, "motor_signs",
        device.driver == "udp_mcu_kinco" ? "1,1,1,1,-1,-1,-1,-1" : "-1,-1,-1,-1,-1,1,1,-1"));
    for (std::size_t axis = 0; axis < signs.size(); ++axis)
    {
        if (signs[axis] != -1.0 && signs[axis] != 1.0)
        {
            throw std::invalid_argument("devices.chassis.motor_signs must contain only -1 or 1");
        }
        config.axes[axis].sign = signs[axis];
    }
    auto parse_endpoint = [&](const std::string& prefix, auto& endpoint) {
        endpoint.bind_ip = option(device, prefix + "_bind_ip", endpoint.bind_ip);
        endpoint.bind_port = parse_port(option(
            device, prefix + "_bind_port", std::to_string(endpoint.bind_port)));
        endpoint.peer_ip = option(device, prefix + "_peer_ip", endpoint.peer_ip);
        endpoint.peer_port = parse_port(option(
            device, prefix + "_peer_port", std::to_string(endpoint.peer_port)));
    };
    parse_endpoint("jiechang", config.jiechang_endpoint);
    parse_endpoint("kinco", config.kinco_endpoint);
    parse_endpoint("m42p", config.m42p_endpoint);
    config.feedback_rate_hz = parse_positive_rate(option(
        device, "feedback_rate_hz", std::to_string(config.feedback_rate_hz)));
    config.command_timeout_s = parse_positive_rate(option(
        device, "command_timeout_s", std::to_string(config.command_timeout_s)));
    config.slider_steps_per_meter = std::stod(option(
        device, "slider_steps_per_meter",
        std::to_string(config.slider_steps_per_meter)));
    config.slider_minimum_m = std::stod(option(
        device, "slider_minimum_m", std::to_string(config.slider_minimum_m)));
    config.slider_maximum_m = std::stod(option(
        device, "slider_maximum_m", std::to_string(config.slider_maximum_m)));
    config.steer_gear_ratio = std::stod(option(
        device, "steer_gear_ratio", std::to_string(config.steer_gear_ratio)));
    config.wheel_gear_ratio = std::stod(option(
        device, "wheel_gear_ratio", std::to_string(config.wheel_gear_ratio)));
    config.steer_rated_current_a = std::stod(option(
        device, "steer_rated_current_a",
        std::to_string(config.steer_rated_current_a)));
    config.wheel_rated_current_a = std::stod(option(
        device, "wheel_rated_current_a",
        std::to_string(config.wheel_rated_current_a)));
    config.steer_rated_motor_torque_nm = std::stod(option(
        device, "steer_rated_motor_torque_nm",
        std::to_string(config.steer_rated_motor_torque_nm)));
    config.wheel_rated_motor_torque_nm = std::stod(option(
        device, "wheel_rated_motor_torque_nm",
        std::to_string(config.wheel_rated_motor_torque_nm)));
    config.steer_position_kp = std::stod(option(
        device, "steer_position_kp",
        std::to_string(config.steer_position_kp)));
    config.steer_position_kd = std::stod(option(
        device, "steer_position_kd",
        std::to_string(config.steer_position_kd)));
    config.wheel_velocity_kv = std::stod(option(
        device, "wheel_velocity_kv",
        std::to_string(config.wheel_velocity_kv)));
    config.jiechang_steer_torque_limit_nm = std::stod(option(
        device, "jiechang_steer_torque_limit_nm",
        std::to_string(config.jiechang_steer_torque_limit_nm)));
    config.jiechang_wheel_torque_limit_nm = std::stod(option(
        device, "jiechang_wheel_torque_limit_nm",
        std::to_string(config.jiechang_wheel_torque_limit_nm)));
    config.jiechang_mit_kp_scale = std::stod(option(
        device, "jiechang_mit_kp_scale",
        std::to_string(config.jiechang_mit_kp_scale)));
    config.jiechang_mit_kd_scale = std::stod(option(
        device, "jiechang_mit_kd_scale",
        std::to_string(config.jiechang_mit_kd_scale)));
    config.kinco_encoder_resolution = static_cast<std::uint32_t>(std::stoul(
        option(device, "kinco_encoder_resolution",
               std::to_string(config.kinco_encoder_resolution))));
#define PARSE_CHASSIS_DOUBLE(field)                                            \
    config.field = std::stod(option(                                           \
        device, #field, std::to_string(config.field)))
    PARSE_CHASSIS_DOUBLE(jiechang_stretch_counts_per_meter);
    PARSE_CHASSIS_DOUBLE(wheel_velocity_ki);
    PARSE_CHASSIS_DOUBLE(wheel_velocity_integral_limit_nm);
    PARSE_CHASSIS_DOUBLE(kinco_steer_gear_ratio);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_gear_ratio);
    PARSE_CHASSIS_DOUBLE(kinco_steer_rated_current_a);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_rated_current_a);
    PARSE_CHASSIS_DOUBLE(kinco_steer_max_current_arms);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_max_current_arms);
    PARSE_CHASSIS_DOUBLE(kinco_steer_output_kt_nm_per_a);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_output_kt_nm_per_a);
    PARSE_CHASSIS_DOUBLE(kinco_steer_position_kp);
    PARSE_CHASSIS_DOUBLE(kinco_steer_position_kd);
    PARSE_CHASSIS_DOUBLE(kinco_steer_profile_acceleration_scale);
    PARSE_CHASSIS_DOUBLE(kinco_steer_command_delay_s);
    PARSE_CHASSIS_DOUBLE(kinco_steer_velocity_feedforward);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_velocity_kp);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_velocity_ki);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_velocity_integral_limit_nm);
    PARSE_CHASSIS_DOUBLE(kinco_steer_continuous_torque_nm);
    PARSE_CHASSIS_DOUBLE(kinco_steer_peak_torque_nm);
    PARSE_CHASSIS_DOUBLE(kinco_steer_peak_speed_rpm);
    PARSE_CHASSIS_DOUBLE(kinco_steer_rated_speed_rpm);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_continuous_torque_nm);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_peak_torque_nm);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_peak_speed_rpm);
    PARSE_CHASSIS_DOUBLE(kinco_wheel_rated_speed_rpm);
#undef PARSE_CHASSIS_DOUBLE
    config.kinco_control_in_step = parse_bool(option(
        device, "kinco_control_in_step", "false"));
    return config;
}

sharpa::mujoco::plugins::arm::ArmPluginConfig arm_config_from(
    const std::string& name,
    const sharpa::mujoco::simulator::DeviceConfig& device)
{
    using namespace sharpa::mujoco::plugins::arm;
    if (device.driver != "udp_arm")
    {
        throw std::invalid_argument(
            "devices." + name + ".driver must be 'udp_arm', got '" +
            device.driver + "'");
    }
    ArmPluginConfig config;
    config.name = name;
    config.side = name == "arm_left" ? ArmSide::Left : ArmSide::Right;
    config.feedback_rate_hz = parse_positive_rate(option(
        device, "feedback_rate_hz", std::to_string(config.feedback_rate_hz)));
    config.command_timeout_s = parse_positive_rate(option(
        device, "command_timeout_s", std::to_string(config.command_timeout_s)));
    const auto parse_delay = [&](const char* name) {
        std::array<double, 7> values{};
        std::istringstream stream(option(device, name, "0,0,0,0,0,0,0"));
        std::string item;
        std::size_t i = 0;
        while (std::getline(stream, item, ','))
        {
            if (i == values.size()) throw std::invalid_argument(std::string(name) + " requires 7 values");
            values[i++] = parse_non_negative(item, name);
        }
        if (i != values.size()) throw std::invalid_argument(std::string(name) + " requires 7 values");
        return values;
    };
    config.command_delay_s = parse_delay("command_delay_s");
    config.feedback_delay_s = parse_delay("feedback_delay_s");
    config.quick_stop_deceleration_rad_s2 = parse_positive_rate(option(
        device, "quick_stop_deceleration_rad_s2",
        std::to_string(config.quick_stop_deceleration_rad_s2)));
    config.quick_stop_velocity_threshold_rad_s = parse_positive_rate(option(
        device, "quick_stop_velocity_threshold_rad_s",
        std::to_string(config.quick_stop_velocity_threshold_rad_s)));
    config.quick_stop_timeout_s = parse_positive_rate(option(
        device, "quick_stop_timeout_s",
        std::to_string(config.quick_stop_timeout_s)));
    config.quick_stop_kp = parse_positive_rate(option(
        device, "quick_stop_kp", std::to_string(config.quick_stop_kp)));
    config.speed_kp = parse_non_negative(option(
        device, "speed_kp", std::to_string(config.speed_kp)), "speed_kp");
    config.speed_ki = parse_non_negative(option(
        device, "speed_ki", std::to_string(config.speed_ki)), "speed_ki");
    config.speed_integral_limit_nm = parse_non_negative(option(
        device, "speed_integral_limit_nm",
        std::to_string(config.speed_integral_limit_nm)),
        "speed_integral_limit_nm");
    config.diagnostic_logging = parse_bool(option(
        device, "diagnostic_logging",
        config.diagnostic_logging ? "true" : "false"));
    config.extended_feedback = parse_bool(option(device, "extended_feedback", "true"));
    const auto sdo_port = option(device, "sdo_port", "65530");
    config.sdo_port = sdo_port == "0" ? 0 : parse_port(sdo_port);
    config.bus_voltage_v = std::stod(option(device, "bus_voltage_v", "48"));
    config.driver_temperature_c = std::stod(option(device, "driver_temperature_c", "25"));
    config.motor_temperature_c = std::stod(option(device, "motor_temperature_c", "25"));
    const std::string csp_control_space = option(
        device, "csp_control_space", "joint");
    if (csp_control_space == "joint")
    {
        config.csp_control_space = ArmCspControlSpace::Joint;
    }
    else if (csp_control_space == "motor")
    {
        config.csp_control_space = ArmCspControlSpace::Motor;
    }
    else
    {
        throw std::invalid_argument(
            "csp_control_space must be 'joint' or 'motor', got '" +
            csp_control_space + "'");
    }
    config.sim_joint_kp = parse_arm_positive_array(option(
        device, "sim_joint_kp", "500,500,300,200,50,50,50"),
        "sim_joint_kp");
    config.sim_joint_kd = parse_arm_positive_array(option(
        device, "sim_joint_kd", "7.5,7.5,2,1,1,1,1"),
        "sim_joint_kd");
    config.csp_kp = parse_arm_positive_array(option(
        device, "csp_kp", "100,100,100,40,40,30,30"), "csp_kp");
    config.csp_kd = parse_arm_positive_array(option(
        device, "csp_kd", "10,10,10,5,5,5,5"), "csp_kd");
    config.torque_limits_nm = parse_arm_positive_array(option(
        device, "torque_limits_nm",
        "30,30,30,5,5,5,5"),
        "torque_limits_nm");
    const int first_host = config.side == ArmSide::Left ? 101 : 106;
    const std::uint16_t peer_port =
        config.side == ArmSide::Left ? 8080 : 8081;
    for (std::size_t index = 0; index < config.endpoints.size(); ++index)
    {
        auto& endpoint = config.endpoints[index];
        const std::string prefix = "endpoint" + std::to_string(index) + "_";
        endpoint.bind_ip = option(
            device, prefix + "bind_ip",
            "127.0.40." + std::to_string(first_host + static_cast<int>(index)));
        endpoint.bind_port = parse_port(option(
            device, prefix + "bind_port", "21000"));
        endpoint.peer_ip = option(device, prefix + "peer_ip", "127.0.0.1");
        endpoint.peer_port = parse_port(option(
            device, prefix + "peer_port", std::to_string(peer_port)));
        const std::string mapping =
            index < 3 ? std::to_string(index)
                      : (index == 3 ? "3,4" : "5,6");
        endpoint.motor_indices = parse_arm_motor_indices(
            option(device, prefix + "motor_indices", mapping));
    }
    return config;
}

sharpa::mujoco::plugins::wave_hand::WaveHandPluginConfig wave_hand_config_from(
    const std::string& name,
    const sharpa::mujoco::simulator::DeviceConfig& device,
    bool fingertip_tactile_supported)
{
    using namespace sharpa::mujoco::plugins::wave_hand;
    if (device.driver != "wave_sdk_loopback")
    {
        throw std::invalid_argument(
            "devices." + name + ".driver must be 'wave_sdk_loopback', got '" +
            device.driver + "'");
    }
    const bool left = name == "hand_left";
    WaveHandPluginConfig config;
    config.name = name;
    config.side = left ? WaveHandSide::kLeft : WaveHandSide::kRight;
    config.device_ip = option(
        device, "device_ip", left ? "127.0.50.10" : "127.0.50.20");
    config.sdk_ip = option(device, "sdk_ip", "127.0.0.1");
    config.serial = option(
        device, "serial_number", left ? "SIM-WAVE-L-0001" : "SIM-WAVE-R-0001");
    config.PN = option(device, "product_number", "Wave01-MOCK");
    config.tcp_port = parse_port(option(device, "tcp_port", "59999"));
    config.command_port = parse_port(option(
        device, "command_udp_port", left ? "50020" : "50030"));
    config.feedback_destination = parse_port(option(
        device, "sdk_joint_port", left ? "50000" : "50010"));
    config.heartbeat_destination = parse_port(option(
        device, "sdk_heartbeat_port", "54321"));
    config.feedback_source_port = parse_port(option(
        device, "feedback_source_port", "10000"));
    config.feedback_rate_hz = parse_positive_rate(option(
        device, "feedback_rate_hz", "500"));
    config.heartbeat_rate_hz = parse_positive_rate(option(
        device, "heartbeat_rate_hz", "1"));
    config.fingertip_tactile_supported = fingertip_tactile_supported;
    config.strict_command_state = parse_bool(option(
        device, "strict_command_state", "false"));
    config.MAC = left
        ? std::array<std::uint8_t, 6>{0x02, 0, 0, 0, 0x50, 0x10}
        : std::array<std::uint8_t, 6>{0x02, 0, 0, 0, 0x50, 0x20};
    return config;
}

sharpa::mujoco::plugins::tactile::TactilePluginConfig tactile_config_from(
    const sharpa::mujoco::simulator::DeviceConfig& device)
{
    using namespace sharpa::mujoco::plugins::tactile;
    if (device.driver != "wave_tactile")
    {
        throw std::invalid_argument(
            "devices.tactile.driver must be 'wave_tactile', got '" +
            device.driver + "'");
    }
    TactilePluginConfig config;
    config.sdk_ip = option(device, "sdk_ip", "127.0.0.1");
    config.right_device_ip = option(
        device, "right_device_ip", "127.0.50.20");
    config.left_device_ip = option(
        device, "left_device_ip", "127.0.50.10");
    config.config_port = parse_port(option(device, "config_port", "59998"));
    config.right_destination_port = parse_port(option(
        device, "right_sdk_port", "50001"));
    config.left_destination_port = parse_port(option(
        device, "left_sdk_port", "50011"));
    config.rate_hz = parse_positive_rate(option(device, "rate_hz", "30"));
    return config;
}

}  // namespace

namespace sharpa::mujoco::simulator
{

bool configure_device_plugins(
    const SimulatorConfig& config,
    Runtime& runtime,
    DevicePluginManager& plugins,
    std::string* error)
{
    try
    {
        std::set<std::string> wave_endpoints;
        std::set<std::string> wave_serials;
        std::vector<LidarMount> lidar_mounts;
        if (!parse_lidar_mounts(config.devices, lidar_mounts, error) ||
            !attach_lidar_mounts(runtime, lidar_mounts, error))
        {
            return false;
        }
        if (!lidar_mounts.empty())
        {
            plugins::lidar::LidarPluginConfig lidar_config;
            lidar_config.instances.reserve(lidar_mounts.size());
            for (const auto& mount : lidar_mounts)
            {
                lidar_config.instances.push_back(mount.instance);
            }
            if (!plugins.add(
                    std::make_unique<plugins::lidar::LidarPlugin>(
                        std::move(lidar_config)),
                    error))
            {
                return false;
            }
        }

        const auto tactile_device = config.devices.find("tactile");
        if (tactile_device != config.devices.end() && tactile_device->second.enabled)
        {
            const std::string sensor_model = option(
                tactile_device->second,
                "sensor_model",
                "mjcf/sensors/tactile/wave_fingertip_frame.xml");
            if (!std::filesystem::is_regular_file(sensor_model))
            {
                throw std::invalid_argument(
                    "tactile sensor_model does not exist: " + sensor_model);
            }
            const std::array<std::pair<const char*, const char*>, 10> fingers{{
                {"right_pinky_DP", "right_pinky_tactile_"},
                {"right_ring_DP", "right_ring_tactile_"},
                {"right_middle_DP", "right_middle_tactile_"},
                {"right_index_DP", "right_index_tactile_"},
                {"right_thumb_DP", "right_thumb_tactile_"},
                {"left_pinky_DP", "left_pinky_tactile_"},
                {"left_ring_DP", "left_ring_tactile_"},
                {"left_middle_DP", "left_middle_tactile_"},
                {"left_index_DP", "left_index_tactile_"},
                {"left_thumb_DP", "left_thumb_tactile_"},
            }};
            std::vector<TactileMount> tactile_mounts;
            for (const auto& [parent, prefix] : fingers)
            {
                TactileMount mount;
                mount.parent_body = parent;
                mount.instance_prefix = prefix;
                mount.sensor_model = sensor_model;
                mount.quaternion = {{0.5, 0.5, -0.5, 0.5}};
                tactile_mounts.push_back(std::move(mount));
            }
            if (!attach_tactile_mounts(runtime, tactile_mounts, error))
            {
                return false;
            }
            if (!plugins.add(
                    std::make_unique<plugins::tactile::TactilePlugin>(
                        tactile_config_from(tactile_device->second)),
                    error))
            {
                return false;
            }
        }

        for (const auto& [name, device] : config.devices)
        {
            if (!device.enabled)
            {
                continue;
            }
            if (name == "torso")
            {
                if (!plugins.add(
                        std::make_unique<plugins::torso::TorsoPlugin>(
                            torso_config_from(device)),
                        error))
                {
                    return false;
                }
                continue;
            }
            if (name == "chassis")
            {
                if (!plugins.add(
                        std::make_unique<plugins::chassis::ChassisPlugin>(
                            chassis_config_from(device)),
                        error))
                {
                    return false;
                }
                continue;
            }
            if (name == "arm_left" || name == "arm_right")
            {
                if (!plugins.add(
                        std::make_unique<plugins::arm::ArmPlugin>(
                            arm_config_from(name, device)),
                        error))
                {
                    return false;
                }
                continue;
            }
            if (name == "hand_left" || name == "hand_right")
            {
                auto hand_config = wave_hand_config_from(
                    name,
                    device,
                    tactile_device != config.devices.end() &&
                        tactile_device->second.enabled);
                const std::string tcp_endpoint =
                    hand_config.device_ip + ":" +
                    std::to_string(hand_config.tcp_port);
                const std::string udp_endpoint =
                    hand_config.device_ip + ":" +
                    std::to_string(hand_config.command_port);
                if (!wave_endpoints.insert(tcp_endpoint).second ||
                    !wave_endpoints.insert(udp_endpoint).second)
                {
                    throw std::invalid_argument(
                        "duplicate Wave hand endpoint: " + hand_config.device_ip);
                }
                if (!wave_serials.insert(hand_config.serial).second)
                {
                    throw std::invalid_argument(
                        "duplicate Wave hand serial number: " +
                        hand_config.serial);
                }
                if (!plugins.add(
                        std::make_unique<plugins::wave_hand::WaveHandPlugin>(
                            std::move(hand_config)),
                        error))
                {
                    return false;
                }
                continue;
            }
            if (name == "tactile" ||
                device.driver == "hesai_ftx140" ||
                device.driver == "hesai_jt16")
            {
                continue;
            }
            throw std::invalid_argument(
                "enabled device '" + name + "' is not implemented");
        }
    }
    catch (const std::exception& exception)
    {
        if (error)
        {
            *error = std::string("Device configuration error: ") + exception.what();
        }
        return false;
    }
    return true;
}

}  // namespace sharpa::mujoco::simulator

#ifndef SHARPA_MUJOCO_SERVICE_LIBRARY
int main(int argc, char** argv)
{
    using sharpa::mujoco::simulator::SimulatorConfig;

    SimulatorConfig config = sharpa::mujoco::simulator::default_config();
    std::string config_path;
    bool explicit_config = false;
    enum class PlatformLaunchMode { Automatic, Required, Disabled };
    PlatformLaunchMode platform_mode = PlatformLaunchMode::Automatic;
    std::string platform_config;

    try
    {
        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            if (argument == "--help" || argument == "-h")
            {
                print_usage();
                return EXIT_SUCCESS;
            }
            if (argument == "--config")
            {
                if (index + 1 >= argc)
                {
                    throw std::invalid_argument("--config requires a value");
                }
                config_path = argv[++index];
                explicit_config = true;
            }
        }

        if (config_path.empty())
        {
            if (const char* environment = std::getenv("SHARPA_MUJOCO_CONFIG"))
            {
                config_path = environment;
                explicit_config = true;
            }
            else if (std::filesystem::is_regular_file("sharpa_mujoco.conf"))
            {
                config_path = "sharpa_mujoco.conf";
            }
        }

        std::string config_error;
        if (!config_path.empty() &&
            !sharpa::mujoco::simulator::load_config_file(
                config_path,
                config,
                &config_error))
        {
            throw std::runtime_error(config_error);
        }
        if (explicit_config && config_path.empty())
        {
            throw std::invalid_argument("configuration path is empty");
        }

        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            auto require_value = [&](const char* option) -> std::string {
                if (index + 1 >= argc)
                {
                    throw std::invalid_argument(std::string(option) + " requires a value");
                }
                return argv[++index];
            };

            if (argument == "--model")
            {
                config.model_path = require_value("--model");
            }
            else if (argument == "--config")
            {
                ++index;
            }
            else if (argument == "--enable-device")
            {
                config.devices[require_value("--enable-device")].enabled = true;
            }
            else if (argument == "--disable-device")
            {
                config.devices[require_value("--disable-device")].enabled = false;
            }
            else if (argument == "--lower-bind-port")
            {
                config.devices["torso"].options["lower_bind_port"] =
                    require_value("--lower-bind-port");
            }
            else if (argument == "--lower-peer-port")
            {
                config.devices["torso"].options["lower_peer_port"] =
                    require_value("--lower-peer-port");
            }
            else if (argument == "--upper-bind-port")
            {
                config.devices["torso"].options["upper_bind_port"] =
                    require_value("--upper-bind-port");
            }
            else if (argument == "--upper-peer-port")
            {
                config.devices["torso"].options["upper_peer_port"] =
                    require_value("--upper-peer-port");
            }
            else if (argument == "--feedback-rate")
            {
                config.devices["torso"].options["feedback_rate_hz"] =
                    require_value("--feedback-rate");
            }
            else if (argument == "--viewer")
            {
                config.viewer.enabled = true;
            }
            else if (argument == "--with-platform")
            {
                if (platform_mode == PlatformLaunchMode::Disabled)
                {
                    throw std::invalid_argument(
                        "--with-platform conflicts with --without-platform");
                }
                platform_mode = PlatformLaunchMode::Required;
            }
            else if (argument == "--without-platform")
            {
                if (platform_mode == PlatformLaunchMode::Required)
                {
                    throw std::invalid_argument(
                        "--without-platform conflicts with --with-platform");
                }
                platform_mode = PlatformLaunchMode::Disabled;
            }
            else if (argument == "--platform-config")
            {
                platform_config = require_value("--platform-config");
            }
            else if (argument == "--no-viewer")
            {
                config.viewer.enabled = false;
            }
            else
            {
                throw std::invalid_argument("unknown option: " + argument);
            }
        }
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Configuration error: " << exception.what() << '\n';
        print_usage();
        return EXIT_FAILURE;
    }

    if (config.model_path.empty())
    {
        std::cerr << "Configuration error: simulation.model is required.\n";
        print_usage();
        return EXIT_FAILURE;
    }

    const std::string platform_root =
        sharpa::mujoco::CompanionProcess::installed_root("sharpa_platform");
    const bool with_platform = platform_mode == PlatformLaunchMode::Required ||
        (platform_mode == PlatformLaunchMode::Automatic && !platform_root.empty());

    if (!platform_config.empty() && !with_platform)
    {
        std::cerr << (platform_mode == PlatformLaunchMode::Disabled
                ? "--platform-config conflicts with --without-platform\n"
                : "--platform-config requires a staged Platform runtime\n");
        return EXIT_FAILURE;
    }

#if !SHARPA_MUJOCO_HAS_VIEWER
    if (config.viewer.enabled)
    {
        std::cerr
            << "Configuration error: viewer.enabled=true, but this build does "
               "not include the Linux Viewer. Rebuild with "
               "SHARPA_MUJOCO_BUILD_VIEWER=ON or use --no-viewer.\n";
        return EXIT_FAILURE;
    }
#endif

    sharpa::mujoco::RuntimeConfig runtime_config;
    runtime_config.snapshot_rate_hz = config.snapshot_rate_hz;
    sharpa::mujoco::Runtime runtime(runtime_config);
    sharpa::mujoco::simulator::SimulatorService simulator;
    std::string error;
    if (!runtime.load(config.model_path, &error))
    {
        std::cerr << error << '\n';
        return EXIT_FAILURE;
    }

    if (!simulator.configure(runtime, config, false, &error))
    {
        std::cerr << error << '\n';
        runtime.unload();
        return EXIT_FAILURE;
    }

    if (!simulator.start(runtime, &error))
    {
        std::cerr << error << '\n';
        runtime.unload();
        return EXIT_FAILURE;
    }
    if (!runtime.start(&error))
    {
        std::cerr << error << '\n';
        simulator.stop();
        runtime.unload();
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    sharpa::mujoco::CompanionProcess companion;
    if (with_platform)
    {
        if (platform_root.empty() || !companion.start(platform_root, platform_config, &error))
        {
            std::cerr << "Platform startup failed: " << (platform_root.empty() ? "runtime is not staged; rebuild with the Platform bundle" : error) << '\n';
            runtime.pause();
            simulator.stop();
            return EXIT_FAILURE;
        }
        std::cout << "Platform process started, PID " << companion.status().pid
                  << "; log: " << companion.status().log_path << '\n';
    }
    const sharpa::mujoco::ModelInfo model = runtime.model_info();
    std::cout << "MuJoCo simulator running: " << model.source_path << '\n'
              << "Enabled devices: " << simulator.device_status().size() << '\n';
    if (config.viewer.enabled)
    {
        std::cout << "Viewer: enabled at " << config.viewer.sync_rate_hz
                  << " Hz\n";
    }
    else
    {
        std::cout << "Viewer: disabled\n";
    }
    std::cout << "Press Ctrl+C to stop.\n";

    bool viewer_failed = false;
    std::string viewer_error;
    if (config.viewer.enabled)
    {
#if SHARPA_MUJOCO_HAS_VIEWER
        sharpa::mujoco::ViewerConfig viewer_config;
        viewer_config.sync_rate_hz = config.viewer.sync_rate_hz;
        viewer_config.vsync = config.viewer.vsync;
        sharpa::mujoco::Viewer viewer(viewer_config);
        if (!viewer.run(runtime, keep_running, &viewer_error))
        {
            viewer_failed = true;
        }
        keep_running.store(false);
#endif
    }
    else
    {
        while (keep_running.load() && runtime.is_running())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    const bool runtime_faulted = runtime.state() == sharpa::mujoco::RuntimeState::Faulted;
    const std::string runtime_error = runtime.last_error();
    const auto companion_status = companion.status();
    companion.stop();
    runtime.pause();
    simulator.stop();
    runtime.unload();
    if (with_platform && companion_status.state == sharpa::mujoco::CompanionProcessState::Failed)
    {
        std::cerr << companion_status.error << '\n';
        return EXIT_FAILURE;
    }
    if (runtime_faulted)
    {
        std::cerr << "Runtime stopped with an error: " << runtime_error << '\n';
        return EXIT_FAILURE;
    }
    if (viewer_failed)
    {
        std::cerr << viewer_error << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
#endif
