// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace sharpa::mujoco::detail
{
struct FrameStatistics
{
    double average_fps = 0;
    double p95_seconds = 0;
};

inline FrameStatistics frame_statistics(const std::vector<double>& intervals)
{
    if (intervals.empty()) return {};
    for (const auto value : intervals)
        if (!std::isfinite(value) || value <= 0)
            throw std::invalid_argument("Frame intervals must be finite and positive");
    auto ordered = intervals;
    std::sort(ordered.begin(), ordered.end());
    return {intervals.size() / std::accumulate(intervals.begin(), intervals.end(), 0.0),
            ordered[static_cast<std::size_t>(std::ceil(0.95 * ordered.size())) - 1]};
}
}  // namespace sharpa::mujoco::detail
