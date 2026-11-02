// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace sharpa::mujoco::plugins::common
{

class PeriodicTask
{
public:
    PeriodicTask() = default;
    ~PeriodicTask()
    {
        stop();
    }

    PeriodicTask(const PeriodicTask&) = delete;
    PeriodicTask& operator=(const PeriodicTask&) = delete;

    bool start(double rate_hz, std::function<void()> task)
    {
        if (running_.load() || worker_.joinable() || !std::isfinite(rate_hz) || rate_hz <= 0.0 || !task)
        {
            return false;
        }
        using Clock = std::chrono::steady_clock;
        const auto seconds = std::chrono::duration<double>(1.0 / rate_hz);
        // Require a positive, representable timer interval before converting.
        if (!std::isfinite(seconds.count()) ||
            seconds < Clock::duration(1) || seconds > Clock::duration::max() / 2)
        {
            return false;
        }
        const auto period = std::chrono::duration_cast<Clock::duration>(seconds);
        running_.store(true);
        try
        {
            worker_ = std::thread([this, period, task = std::move(task)]() {
                auto deadline = Clock::now();
                while (running_.load())
                {
                    task();
                    deadline += period;
                    std::unique_lock<std::mutex> lock(wait_mutex_);
                    if (wakeup_.wait_until(lock, deadline, [this] { return !running_.load(); }))
                    {
                        break;
                    }
                    if (Clock::now() - deadline > period)
                    {
                        deadline = Clock::now();
                    }
                }
            });
        }
        catch (...)
        {
            running_.store(false);
            throw;
        }
        return true;
    }

    void stop() noexcept
    {
        {
            // Pair the predicate change with wait_until to avoid lost wakeups.
            std::lock_guard<std::mutex> lock(wait_mutex_);
            running_.store(false);
        }
        wakeup_.notify_all();
        if (worker_.joinable())
        {
            worker_.join();
        }
    }

    bool is_running() const noexcept
    {
        return running_.load();
    }

private:
    std::atomic<bool> running_{false};
    std::mutex wait_mutex_;
    std::condition_variable wakeup_;
    std::thread worker_;
};

}  // namespace sharpa::mujoco::plugins::common
