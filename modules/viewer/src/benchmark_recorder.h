// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "benchmark_metrics.h"
#include "sharpa_mujoco/runtime.h"
#include <GLFW/glfw3.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

namespace sharpa::mujoco::detail
{
inline std::string json_string(const std::string& text)
{
    std::ostringstream out;
    out << '"';
    for (const unsigned char value : text)
    {
        if (value == '"' || value == '\\') out << '\\' << value;
        else if (value < 32) out << "\\u" << std::hex << std::setw(4)
                               << std::setfill('0') << static_cast<unsigned int>(value);
        else out << value;
    }
    out << '"';
    return out.str();
}

class BenchmarkRecorder
{
    using Clock = std::chrono::steady_clock;
public:
    BenchmarkRecorder(std::string path, Runtime& runtime)
        : path_(std::move(path)), runtime_(runtime)
    {
        const auto parent = std::filesystem::path(path_).parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent);
        if (std::filesystem::exists(path_))
            throw std::runtime_error("Benchmark report already exists: " + path_);
        std::ofstream probe(path_ + ".tmp", std::ios::trunc);
        if (!probe) throw std::runtime_error("Cannot write benchmark report: " + path_);
    }

    void prepare()
    {
        window_ = glfwGetCurrentContext();
        if (!window_) throw std::runtime_error("Benchmark needs a current graphics context");
        glfwSetWindowAttrib(window_, GLFW_DECORATED, GLFW_FALSE);
        int x = 0, y = 0;
        glfwGetMonitorPos(glfwGetPrimaryMonitor(), &x, &y);
        glfwSetWindowPos(window_, x, y);
        glfwSetWindowSize(window_, 1920, 1080);
        glfwFocusWindow(window_);
        using GetString = const unsigned char* (*)(unsigned int);
        const auto get_string = reinterpret_cast<GetString>(glfwGetProcAddress("glGetString"));
        if (!get_string) throw std::runtime_error("glGetString unavailable");
        const auto read = [get_string](unsigned int name) {
            const auto* value = get_string(name);
            return value ? std::string(reinterpret_cast<const char*>(value)) : std::string{};
        };
        vendor_ = read(0x1F00); renderer_ = read(0x1F01); version_ = read(0x1F02);
        started_ = Clock::now();
        std::cout << "Benchmark: warmup 10s, sample 60s, 1920x1080, 60Hz, VSync off\n";
    }

    void frame()
    {
        if (written_) return;
        const auto now = Clock::now();
        if (std::chrono::duration<double>(now - started_).count() < 10) return;
        int width = 0, height = 0;
        glfwGetFramebufferSize(window_, &width, &height);
        width_ = width; height_ = height;
        dimensions_ok_ = dimensions_ok_ && width == 1920 && height == 1080;
        if (previous_ == Clock::time_point{})
        {
            previous_ = sample_started_ = now;
            initial_sim_ = runtime_.latest_snapshot().simulation_time;
            return;
        }
        intervals_.push_back(std::chrono::duration<double>(now - previous_).count());
        previous_ = now;
        elapsed_ = std::chrono::duration<double>(now - sample_started_).count();
        if (elapsed_ >= 60)
        {
            final_sim_ = runtime_.latest_snapshot().simulation_time;
            complete_ = true;
            write();
        }
    }

    void finish()
    {
        if (!written_)
        {
            if (previous_ != Clock::time_point{})
                final_sim_ = runtime_.latest_snapshot().simulation_time;
            write();
        }
    }

private:
    void write()
    {
        const auto stats = frame_statistics(intervals_);
        const double ratio = elapsed_ > 0 ? (final_sim_ - initial_sim_) / elapsed_ : 0;
        const bool hardware = vendor_.find("NVIDIA") != std::string::npos &&
            renderer_.find("llvmpipe") == std::string::npos;
        const bool passed = complete_ && dimensions_ok_ && hardware &&
            stats.average_fps >= 55 && stats.p95_seconds <= 0.0333 && ratio >= 0.98;
        std::ofstream out(path_ + ".tmp", std::ios::trunc);
        out << std::setprecision(12)
            << "{\n\"complete\":" << (complete_ ? "true" : "false")
            << ",\n\"passed\":" << (passed ? "true" : "false")
            << ",\n\"gl_vendor\":" << json_string(vendor_)
            << ",\n\"gl_renderer\":" << json_string(renderer_)
            << ",\n\"gl_version\":" << json_string(version_)
            << ",\n\"framebuffer\":[" << width_ << ',' << height_ << ']'
            << ",\n\"dimensions_stable\":" << (dimensions_ok_ ? "true" : "false")
            << ",\n\"warmup_seconds\":10,\n\"target_hz\":60,\n\"vsync\":false"
            << ",\n\"sample_seconds\":" << elapsed_
            << ",\n\"average_fps\":" << stats.average_fps
            << ",\n\"frame_time_p95_ms\":" << stats.p95_seconds * 1000
            << ",\n\"simulation_start\":" << initial_sim_
            << ",\n\"simulation_end\":" << final_sim_
            << ",\n\"realtime_ratio\":" << ratio
            << ",\n\"frame_intervals_seconds\":[";
        for (std::size_t i = 0; i < intervals_.size(); ++i)
            out << (i ? "," : "") << intervals_[i];
        out << "]\n}\n";
        out.close();
        if (!out) throw std::runtime_error("Cannot write benchmark report: " + path_);
        std::filesystem::rename(path_ + ".tmp", path_);
        written_ = true;
        std::cout << "Benchmark report: " << path_ << "; passed=" << passed << '\n';
    }

    std::string path_;
    Runtime& runtime_;
    GLFWwindow* window_ = nullptr;
    Clock::time_point started_{}, sample_started_{}, previous_{};
    std::vector<double> intervals_;
    std::string vendor_, renderer_, version_;
    int width_ = 0, height_ = 0;
    double initial_sim_ = 0, final_sim_ = 0, elapsed_ = 0;
    bool dimensions_ok_ = true, written_ = false, complete_ = false;
};
}  // namespace sharpa::mujoco::detail
