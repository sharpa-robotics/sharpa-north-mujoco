// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <memory>
#include <string>

namespace sharpa::mujoco
{

class Runtime;

struct ViewerConfig
{
    // Also caps rendering when vsync is disabled.
    double sync_rate_hz = 60.0;
    bool vsync = true;
};

class Viewer
{
public:
    explicit Viewer(ViewerConfig config = {});
    ~Viewer();

    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    // Runs the GLFW render loop on the calling thread until the window closes,
    // shutdown is requested, or Runtime stops.
    bool run(
        Runtime& runtime,
        const std::atomic<bool>& keep_running,
        std::string* error = nullptr);

    void request_exit() noexcept;
    bool is_running() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sharpa::mujoco
