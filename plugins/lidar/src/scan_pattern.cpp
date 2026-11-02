// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/lidar/scan_pattern.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace sharpa::mujoco::plugins::lidar
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

bool fail(const std::string& message, std::string* error)
{
    if (error)
    {
        *error = message;
    }
    return false;
}

std::vector<std::string> split_csv(const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ','))
    {
        fields.push_back(field);
    }
    return fields;
}

bool normalize(std::array<double, 3>& direction, std::string* error)
{
    const double norm = std::sqrt(
        direction[0] * direction[0] + direction[1] * direction[1] +
        direction[2] * direction[2]);
    if (!std::isfinite(norm) || norm < 1e-8)
    {
        return fail("LiDAR calibration contains a zero or non-finite direction.", error);
    }
    for (double& value : direction)
    {
        value /= norm;
    }
    return true;
}

}  // namespace

std::array<double, 3> jt16_direction_platform(
    double elevation_degrees,
    double azimuth_radians)
{
    const double elevation = elevation_degrees * kPi / 180.0;
    return {
        std::sin(azimuth_radians) * std::cos(elevation),
        std::cos(azimuth_radians) * std::cos(elevation),
        std::sin(elevation)};
}

bool load_ftx140_scan_pattern(
    const std::string& path,
    ScanPattern& pattern,
    std::string* error)
{
    std::ifstream input(path);
    if (!input)
    {
        return fail("Could not open FTX140 calibration: " + path, error);
    }
    std::string line;
    std::getline(input, line);
    pattern.ftx140_directions.assign(kFtx140Rows * kFtx140Columns, {});
    std::vector<bool> seen(pattern.ftx140_directions.size(), false);
    std::size_t count = 0;
    try
    {
        while (std::getline(input, line))
        {
            const auto fields = split_csv(line);
            if (fields.size() != 5)
            {
                return fail("Invalid FTX140 calibration CSV row.", error);
            }
            const std::size_t row = std::stoul(fields[0]);
            const std::size_t column = std::stoul(fields[1]);
            if (row >= kFtx140Rows || column >= kFtx140Columns)
            {
                return fail("FTX140 calibration index is out of range.", error);
            }
            const std::size_t index = row * kFtx140Columns + column;
            if (seen[index])
            {
                return fail("FTX140 calibration contains duplicate directions.", error);
            }
            std::array<double, 3> direction{
                std::stod(fields[2]), std::stod(fields[3]), std::stod(fields[4])};
            if (!normalize(direction, error))
            {
                return false;
            }
            seen[index] = true;
            pattern.ftx140_directions[index] = direction;
            ++count;
        }
    }
    catch (const std::exception& exception)
    {
        return fail("Invalid FTX140 calibration: " + std::string(exception.what()), error);
    }
    return count == pattern.ftx140_directions.size()
               ? true
               : fail("FTX140 calibration must contain 192x256 directions.", error);
}

bool load_jt16_scan_pattern(
    const std::string& path,
    ScanPattern& pattern,
    std::string* error)
{
    std::ifstream input(path);
    if (!input)
    {
        return fail("Could not open JT16 calibration: " + path, error);
    }
    std::string line;
    std::getline(input, line);
    std::array<bool, 16> seen{};
    std::size_t count = 0;
    try
    {
        while (std::getline(input, line))
        {
            const auto fields = split_csv(line);
            if (fields.size() != 3)
            {
                return fail("Invalid JT16 calibration CSV row.", error);
            }
            const std::size_t laser_id = std::stoul(fields[0]);
            if (laser_id == 0 || laser_id > seen.size() || seen[laser_id - 1])
            {
                return fail("JT16 calibration LaserID is invalid or duplicated.", error);
            }
            const double elevation = std::stod(fields[1]);
            const double azimuth = std::stod(fields[2]);
            if (!std::isfinite(elevation) || !std::isfinite(azimuth))
            {
                return fail("JT16 calibration contains non-finite angles.", error);
            }
            seen[laser_id - 1] = true;
            pattern.jt16_elevations[laser_id - 1] = elevation;
            pattern.jt16_azimuth_offsets[laser_id - 1] = azimuth * kPi / 180.0;
            ++count;
        }
    }
    catch (const std::exception& exception)
    {
        return fail("Invalid JT16 calibration: " + std::string(exception.what()), error);
    }
    return count == seen.size()
               ? true
               : fail("JT16 calibration must contain exactly 16 LaserID rows.", error);
}

}  // namespace sharpa::mujoco::plugins::lidar
