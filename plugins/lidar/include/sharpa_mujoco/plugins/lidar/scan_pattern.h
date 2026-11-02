// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/plugins/lidar/lidar_protocol.h"

#include <array>
#include <string>
#include <vector>

namespace sharpa::mujoco::plugins::lidar
{

struct ScanPattern
{
    std::vector<std::array<double, 3>> ftx140_directions;
    std::array<double, 16> jt16_elevations{};
    std::array<double, 16> jt16_azimuth_offsets{};
};

// Reconstruct JT16 directions with the same axis convention used by the
// sharpa_platform JT16 decoder.
std::array<double, 3> jt16_direction_platform(
    double elevation_degrees,
    double azimuth_radians);

bool load_ftx140_scan_pattern(
    const std::string& path,
    ScanPattern& pattern,
    std::string* error = nullptr);

bool load_jt16_scan_pattern(
    const std::string& path,
    ScanPattern& pattern,
    std::string* error = nullptr);

}  // namespace sharpa::mujoco::plugins::lidar
