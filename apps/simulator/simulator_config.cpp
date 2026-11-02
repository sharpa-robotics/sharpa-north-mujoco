// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "simulator_config.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace sharpa::mujoco::simulator
{
namespace
{

std::string trim(std::string value)
{
    const auto not_space = [](unsigned char character) {
        return !std::isspace(character);
    };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(
        std::find_if(value.rbegin(), value.rend(), not_space).base(),
        value.end());
    return value;
}

std::string unquote(std::string value)
{
    if (value.size() >= 2 &&
        ((value.front() == '"' && value.back() == '"') ||
         (value.front() == '\'' && value.back() == '\'')))
    {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

bool parse_bool(const std::string& value)
{
    if (value == "true" || value == "1" || value == "yes" || value == "on")
    {
        return true;
    }
    if (value == "false" || value == "0" || value == "no" || value == "off")
    {
        return false;
    }
    throw std::invalid_argument("expected a boolean, got '" + value + "'");
}

bool apply_entry(
    const std::string& section,
    const std::string& key,
    const std::string& value,
    const std::filesystem::path& config_directory,
    SimulatorConfig& config,
    std::string* error)
{
    try
    {
        // Root entries belong to SDK/build integrations. The simulator shares
        // this file but only consumes the sections below.
        if (section.empty())
        {
            return true;
        }
        if (section == "simulation")
        {
            if (key == "model")
            {
                std::filesystem::path model(value);
                if (model.is_relative())
                {
                    model = config_directory / model;
                }
                config.model_path = model.lexically_normal().string();
                return true;
            }
            if (key == "snapshot_rate_hz")
            {
                config.snapshot_rate_hz = std::stod(value);
                if (config.snapshot_rate_hz <= 0.0)
                {
                    throw std::out_of_range("snapshot_rate_hz must be positive");
                }
                return true;
            }
        }
        if (section == "viewer")
        {
            if (key == "enabled")
            {
                config.viewer.enabled = parse_bool(value);
                return true;
            }
            if (key == "sync_rate_hz")
            {
                config.viewer.sync_rate_hz = std::stod(value);
                if (config.viewer.sync_rate_hz <= 0.0)
                {
                    throw std::out_of_range(
                        "viewer.sync_rate_hz must be positive");
                }
                return true;
            }
            if (key == "vsync")
            {
                config.viewer.vsync = parse_bool(value);
                return true;
            }
        }

        constexpr const char* device_prefix = "devices.";
        if (section.rfind(device_prefix, 0) == 0)
        {
            const std::string device_name = section.substr(std::char_traits<char>::length(device_prefix));
            if (device_name.empty())
            {
                throw std::invalid_argument("device section requires a name");
            }
            DeviceConfig& device = config.devices[device_name];
            if (key == "enabled")
            {
                device.enabled = parse_bool(value);
            }
            else if (key == "driver")
            {
                device.driver = value;
            }
            else
            {
                if (key == "calibration_file" || key == "sensor_model")
                {
                    std::filesystem::path resource(value);
                    if (resource.is_relative())
                    {
                        resource = config_directory / resource;
                    }
                    device.options[key] = resource.lexically_normal().string();
                }
                else
                {
                    device.options[key] = value;
                }
            }
            return true;
        }
    }
    catch (const std::exception& exception)
    {
        if (error)
        {
            *error = exception.what();
        }
        return false;
    }

    if (error)
    {
        *error = "unknown configuration key '" +
                 (section.empty() ? key : section + "." + key) + "'";
    }
    return false;
}

}  // namespace

SimulatorConfig default_config()
{
    SimulatorConfig config;
    DeviceConfig torso;
    torso.enabled = true;
    torso.driver = "udp";
    torso.options = {
        {"lower_bind_ip", "127.0.0.1"},
        {"lower_bind_port", "45520"},
        {"lower_peer_ip", "127.0.0.1"},
        {"lower_peer_port", "45120"},
        {"lower_can_if", "0"},
        {"lower_motor_ids", "1,2,3,4,5"},
        {"lower_can_interface", "vcan0"},
        {"upper_bind_ip", "127.0.0.1"},
        {"upper_bind_port", "45521"},
        {"upper_peer_ip", "127.0.0.1"},
        {"upper_peer_port", "45121"},
        {"upper_can_if", "0"},
        {"upper_motor_ids", "6,7,8"},
        {"upper_can_interface", "vcan1"},
        {"feedback_rate_hz", "200"},
        {"heartbeat_rate_hz", "10"},
        {"motor_signs", "-1,-1,1,1,-1,1,1,1"},
        {"dynamic_waist_armature", "true"},
        {"waist_motor_1_inertia", "1.647253629"},
        {"waist_motor_2_inertia", "1.647253629"},
    };
    config.devices.emplace("torso", std::move(torso));
    return config;
}

bool load_config_file(
    const std::string& path,
    SimulatorConfig& config,
    std::string* error)
{
    std::ifstream input(path);
    if (!input)
    {
        if (error)
        {
            *error = "cannot open simulator configuration: " + path;
        }
        return false;
    }

    const std::filesystem::path config_directory =
        std::filesystem::absolute(path).parent_path();
    std::string section;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos)
        {
            line.erase(comment);
        }
        line = trim(std::move(line));
        if (line.empty())
        {
            continue;
        }
        if (line.front() == '[' && line.back() == ']')
        {
            section = trim(line.substr(1, line.size() - 2));
            continue;
        }

        const std::size_t equals = line.find('=');
        if (equals == std::string::npos)
        {
            if (error)
            {
                *error = path + ':' + std::to_string(line_number) +
                         ": expected key=value";
            }
            return false;
        }
        const std::string key = trim(line.substr(0, equals));
        const std::string value = unquote(trim(line.substr(equals + 1)));
        std::string entry_error;
        if (key.empty() ||
            !apply_entry(
                section,
                key,
                value,
                config_directory,
                config,
                &entry_error))
        {
            if (error)
            {
                *error = path + ':' + std::to_string(line_number) + ": " +
                         (key.empty() ? "configuration key is empty" : entry_error);
            }
            return false;
        }
    }
    return true;
}

}  // namespace sharpa::mujoco::simulator
