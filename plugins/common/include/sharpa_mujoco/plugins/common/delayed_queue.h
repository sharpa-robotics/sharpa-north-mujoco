// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <chrono>
#include <cstddef>

namespace sharpa::mujoco::plugins::common
{

// Fixed-capacity FIFO for constant transport delays. Callers provide a monotonic
// clock and external synchronization; no sleeping or clock adjustment occurs.
// Deadlines must be nondecreasing. On overflow retain the newest samples.
template <typename T, std::size_t Capacity = 1024>
class DelayedQueue
{
    static_assert(Capacity > 0, "A delayed queue needs storage");

public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    void push(const T& value, TimePoint due)
    {
        if (size_ == Capacity)
        {
            head_ = (head_ + 1) % Capacity;
            --size_;
            ++dropped_;
        }
        entries_[(head_ + size_) % Capacity] = {due, value};
        ++size_;
    }

    bool pop_ready(TimePoint now, T& value)
    {
        if (size_ == 0 || entries_[head_].due > now) return false;
        value = entries_[head_].value;
        head_ = (head_ + 1) % Capacity;
        --size_;
        return true;
    }

    void clear() noexcept { head_ = size_ = dropped_ = 0; }
    std::size_t size() const noexcept { return size_; }
    std::size_t dropped() const noexcept { return dropped_; }

private:
    struct Entry
    {
        TimePoint due{};
        T value{};
    };
    std::array<Entry, Capacity> entries_{};
    std::size_t head_ = 0;
    std::size_t size_ = 0;
    std::size_t dropped_ = 0;
};

inline std::chrono::steady_clock::duration transport_delay(double seconds)
{
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(seconds));
}

}  // namespace sharpa::mujoco::plugins::common
