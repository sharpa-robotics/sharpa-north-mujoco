// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/common/delayed_queue.h"
#include <cstdlib>
#include <iostream>
using sharpa::mujoco::plugins::common::DelayedQueue;
int main()
{
    using namespace std::chrono_literals;
    DelayedQueue<int, 2> queue;
    const auto start = DelayedQueue<int>::TimePoint{};
    int value = -1;
    const auto check = [](bool ok) { if (!ok) std::cerr << "Delay queue contract failed\n"; return ok; };
    queue.push(1, start + 20ms);
    queue.push(2, start + 30ms);
    if (!check(!queue.pop_ready(start + 19ms, value) && value == -1) ||
        !check(queue.pop_ready(start + 20ms, value) && value == 1) ||
        !check(!queue.pop_ready(start + 29ms, value)) ||
        !check(queue.pop_ready(start + 30ms, value) && value == 2)) return EXIT_FAILURE;
    queue.push(3, start); queue.push(4, start); queue.push(5, start);
    if (!check(queue.size() == 2 && queue.dropped() == 1) ||
        !check(queue.pop_ready(start, value) && value == 4) ||
        !check(queue.pop_ready(start, value) && value == 5)) return EXIT_FAILURE;
    queue.push(6, start + 1s); queue.clear();
    if (!check(!queue.pop_ready(start + 2s, value) && queue.dropped() == 0)) return EXIT_FAILURE;
    queue.push(7, start);
    if (!check(queue.pop_ready(start, value) && value == 7)) return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
