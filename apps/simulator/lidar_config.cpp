// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "lidar_config.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace sharpa::mujoco::simulator
{
namespace
{

std::string option(
    const DeviceConfig& device,
    const std::string& key,
    const std::string& fallback)
{
    const auto found = device.options.find(key);
    return found == device.options.end() ? fallback : found->second;
}

std::vector<double> parse_numbers(const std::string& text)
{
    std::string normalized = text;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');
    std::istringstream input(normalized);
    std::vector<double> values;
    double value = 0.0;
    while (input >> value)
    {
        values.push_back(value);
    }
    return values;
}

std::vector<std::string> parse_string_list(
    const std::string& text,
    const char* key)
{
    std::vector<std::string> values;
    std::istringstream input(text);
    std::string value;
    while (std::getline(input, value, ','))
    {
        const std::size_t begin = value.find_first_not_of(" \t");
        const std::size_t end = value.find_last_not_of(" \t");
        if (begin == std::string::npos)
        {
            throw std::invalid_argument(
                std::string(key) + " must not contain empty names");
        }
        values.push_back(value.substr(begin, end - begin + 1));
    }
    return values;
}

template <std::size_t Count>
std::array<double, Count> parse_array(const std::string& value, const char* key)
{
    const std::vector<double> parsed = parse_numbers(value);
    if (parsed.size() != Count)
    {
        throw std::invalid_argument(
            std::string(key) + " must contain exactly " + std::to_string(Count) + " numbers");
    }
    std::array<double, Count> output{};
    std::copy(parsed.begin(), parsed.end(), output.begin());
    return output;
}

std::uint16_t parse_port(const std::string& value, const char* key)
{
    const unsigned long parsed = std::stoul(value);
    if (parsed == 0 || parsed > 65535)
    {
        throw std::out_of_range(std::string(key) + " must be in range 1..65535");
    }
    return static_cast<std::uint16_t>(parsed);
}

std::size_t parse_thread_count(const std::string& value, const char* key)
{
    if (value.empty() || value.front() == '-')
    {
        throw std::out_of_range(
            std::string(key) + " must be in range 0..256");
    }
    std::size_t parsed_characters = 0;
    const unsigned long parsed = std::stoul(value, &parsed_characters);
    if (parsed_characters != value.size() || parsed > 256)
    {
        throw std::out_of_range(
            std::string(key) + " must be in range 0..256");
    }
    return static_cast<std::size_t>(parsed);
}

void validate_finite_positive(double value, const char* key)
{
    if (!std::isfinite(value) || value <= 0.0)
    {
        throw std::out_of_range(std::string(key) + " must be finite and positive");
    }
}

}  // namespace

bool parse_lidar_mounts(
    const std::map<std::string, DeviceConfig>& devices,
    std::vector<LidarMount>& mounts,
    std::string* error)
{
    mounts.clear();
    std::string current_name;
    try
    {
        for (const auto& [name, device] : devices)
        {
            if (!device.enabled ||
                (device.driver != "hesai_ftx140" && device.driver != "hesai_jt16"))
            {
                continue;
            }
            current_name = name;

            LidarMount mount;
            mount.instance.name = name;
            mount.instance.model =
                device.driver == "hesai_ftx140"
                    ? plugins::lidar::LidarModel::Ftx140
                    : plugins::lidar::LidarModel::Jt16;
            mount.parent_body = option(device, "parent_body", "");
            if (device.options.find("instance_prefix") != device.options.end())
            {
                throw std::invalid_argument(
                    "instance_prefix is deprecated; it is derived from the device name");
            }
            mount.instance_prefix = name + "_";
            mount.local_site_name = option(device, "site_name", "ray_origin");
            if (mount.local_site_name.empty())
            {
                throw std::invalid_argument("site_name must not be empty");
            }
            mount.instance.site_name =
                mount.instance_prefix + mount.local_site_name;
            mount.position = parse_array<3>(option(device, "mount_pos", ""), "mount_pos");
            mount.quaternion = parse_array<4>(
                option(device, "mount_quat", ""), "mount_quat");
            const double quaternion_norm = std::sqrt(
                mount.quaternion[0] * mount.quaternion[0] +
                mount.quaternion[1] * mount.quaternion[1] +
                mount.quaternion[2] * mount.quaternion[2] +
                mount.quaternion[3] * mount.quaternion[3]);
            if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-8)
            {
                throw std::invalid_argument("mount_quat must be a non-zero finite quaternion");
            }
            for (double& component : mount.quaternion)
            {
                component /= quaternion_norm;
            }
            mount.instance.bind_ip = option(device, "bind_ip", "127.0.0.1");
            mount.instance.bind_port = parse_port(option(device, "bind_port", "0"), "bind_port");
            mount.instance.peer_ip = option(device, "peer_ip", "127.0.0.1");
            mount.instance.peer_port = parse_port(option(device, "peer_port", "0"), "peer_port");
            mount.instance.calibration_file = option(device, "calibration_file", "");
            if (mount.instance.calibration_file.empty() ||
                !std::filesystem::is_regular_file(mount.instance.calibration_file))
            {
                throw std::invalid_argument(
                    "calibration_file does not exist: " + mount.instance.calibration_file);
            }
            mount.instance.raycast_exclude_meshes = parse_string_list(
                option(device, "raycast_exclude_meshes", ""),
                "raycast_exclude_meshes");
            mount.instance.raycast_threads = parse_thread_count(
                option(device, "raycast_threads", "0"),
                "raycast_threads");
            mount.sensor_model = option(device, "sensor_model", "");
            if (mount.sensor_model.empty() ||
                !std::filesystem::is_regular_file(mount.sensor_model))
            {
                throw std::invalid_argument(
                    "sensor_model does not exist: " + mount.sensor_model);
            }
            mount.instance.scan_rate_hz = std::stod(option(device, "scan_rate_hz", "10"));
            mount.instance.min_range_m = std::stod(option(device, "min_range_m", "0.1"));
            mount.instance.max_range_m = std::stod(option(device, "max_range_m", "60"));
            validate_finite_positive(mount.instance.scan_rate_hz, "scan_rate_hz");
            validate_finite_positive(mount.instance.min_range_m, "min_range_m");
            validate_finite_positive(mount.instance.max_range_m, "max_range_m");
            if (mount.instance.max_range_m <= mount.instance.min_range_m)
            {
                throw std::out_of_range("max_range_m must exceed min_range_m");
            }
            mounts.push_back(std::move(mount));
        }
    }
    catch (const std::exception& exception)
    {
        if (error)
        {
            *error = std::string("Invalid LiDAR configuration: ") + exception.what();
            if (!current_name.empty())
            {
                *error = "Invalid LiDAR configuration for '" + current_name +
                         "': " + exception.what();
            }
        }
        mounts.clear();
        return false;
    }
    return true;
}

}  // namespace sharpa::mujoco::simulator
