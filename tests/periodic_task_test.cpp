// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/common/periodic_task.h"

#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>

int main()
{
    using namespace std::chrono_literals;
    using Clock = std::chrono::steady_clock;
    const auto require = [](bool value, const char* message) {
        if (!value) throw std::runtime_error(message);
    };
    try
    {
        sharpa::mujoco::plugins::common::PeriodicTask task;
        for (double rate : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::denorm_min(),
                            std::numeric_limits<double>::max()})
            require(!task.start(rate, [] {}), "Accepted invalid/unrepresentable interval");
        require(!task.start(1.0, {}), "Accepted missing callback");
        std::promise<void> first_tick;
        auto tick = first_tick.get_future();
        require(task.start(0.01, [&] { first_tick.set_value(); }), "Could not start low-frequency task");
        require(tick.wait_for(1s) == std::future_status::ready, "Task did not execute its first tick");
        require(!task.start(1.0, [] {}), "Accepted duplicate start");
        // Allow the worker to enter its 100-second wait, then interrupt it.
        std::this_thread::sleep_for(20ms);
        const auto started = Clock::now();
        task.stop();
        require(Clock::now() - started < 1s, "stop() waited for the timer period");
        require(!task.is_running(), "Stopped task still reports running");
        task.stop();
        for (int attempt = 0; attempt < 50; ++attempt)
        {
            std::atomic<int> calls{0};
            require(task.start(0.01, [&] { ++calls; }), "Restart failed");
            // Exercise cancellation before or during the transition to wait.
            task.stop();
            const int stopped_calls = calls.load();
            require(!task.is_running() && calls.load() == stopped_calls, "Worker survived stop");
        }
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
