// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "../modules/viewer/src/benchmark_metrics.h"
#include <iostream>
#include <limits>

int main()
{
    using sharpa::mujoco::detail::frame_statistics;
    const auto require = [](bool ok) { if (!ok) throw std::runtime_error("Incorrect frame statistics"); };
    require(frame_statistics({}).average_fps == 0);
    std::vector<double> frames(100, 1.0 / 60);
    require(std::abs(frame_statistics(frames).average_fps - 60) < 1e-9);
    frames[0] = frames[1] = 0.1;
    auto stats = frame_statistics(frames);
    require(stats.average_fps < 55); // Mean instantaneous FPS would hide these stalls.
    require(std::abs(stats.p95_seconds - 1.0 / 60) < 1e-9);
    frames.assign(20, 1.0 / 60);
    frames[0] = 0.1;
    require(frame_statistics(frames).p95_seconds < 0.02);
    frames[1] = 0.1;
    require(frame_statistics(frames).p95_seconds == 0.1);
    for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()})
    {
        bool rejected = false;
        try { frame_statistics({value}); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected);
    }
    std::cout << "Frame statistics passed\n";
}
