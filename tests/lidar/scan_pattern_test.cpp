// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/lidar/scan_pattern.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace
{

bool require(bool value, const std::string& message)
{
    if (!value)
    {
        std::cerr << message << '\n';
    }
    return value;
}

bool near(double actual, double expected, double tolerance = 1e-9)
{
    return std::abs(actual - expected) <= tolerance;
}

}  // namespace

int main()
{
    using namespace sharpa::mujoco::plugins::lidar;
    ScanPattern ftx;
    ScanPattern jt16;
    std::string error;
    if (!require(
            load_ftx140_scan_pattern(SHARPA_MUJOCO_FTX_CALIBRATION, ftx, &error),
            error) ||
        !require(
            load_jt16_scan_pattern(SHARPA_MUJOCO_JT16_CALIBRATION, jt16, &error),
            error))
    {
        return EXIT_FAILURE;
    }
    const auto& first_direction = ftx.ftx140_directions.front();
    const auto& last_direction = ftx.ftx140_directions.back();
    const double norm = std::sqrt(
        first_direction[0] * first_direction[0] +
        first_direction[1] * first_direction[1] +
        first_direction[2] * first_direction[2]);
    const auto jt16_zero = jt16_direction_platform(0.0, 0.0);
    const auto jt16_quarter_turn =
        jt16_direction_platform(0.0, 0.5 * std::acos(-1.0));
    const auto jt16_up = jt16_direction_platform(30.0, 0.0);
    return require(
               ftx.ftx140_directions.size() ==
                       kFtx140Rows * kFtx140Columns &&
                   std::abs(norm - 1.0) < 1e-9 &&
                   near(first_direction[0], 0.18360593533384292) &&
                   near(first_direction[1], 0.7863999432391372) &&
                   near(first_direction[2], -0.5897999574293529) &&
                   near(last_direction[0], 0.1908123384329388) &&
                   near(last_direction[1], -0.7860433205971804) &&
                   near(last_direction[2], 0.5879851610766311) &&
                   near(jt16.jt16_elevations.front(), 0.01) &&
                   near(jt16.jt16_elevations.back(), 40.213685) &&
                   near(
                       jt16.jt16_azimuth_offsets.front(),
                       193.49 * std::acos(-1.0) / 180.0) &&
                   near(
                       jt16.jt16_azimuth_offsets.back(),
                       188.31 * std::acos(-1.0) / 180.0) &&
                   near(jt16_zero[0], 0.0) &&
                   near(jt16_zero[1], 1.0) &&
                   near(jt16_zero[2], 0.0) &&
                   near(jt16_quarter_turn[0], 1.0) &&
                   near(jt16_quarter_turn[1], 0.0) &&
                   near(jt16_quarter_turn[2], 0.0) &&
                   jt16_up[2] > 0.0,
               "LiDAR calibration scan patterns are invalid.")
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
