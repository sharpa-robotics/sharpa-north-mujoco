// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/viewer.h"

#include "sharpa_mujoco/runtime.h"

#include "glfw_adapter.h"
#include "simulate.h"
#include "benchmark_recorder.h"

#include <mujoco/mujoco.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace sharpa::mujoco
{
namespace
{

// Keep the event loop responsive without relying on compositor/driver VSync.
// In particular, background GLX windows can spend about a second in SwapBuffers.
// Software pacing bounds GPU usage when the caller disables VSync.
class PacedGlfwAdapter final : public ::mujoco::GlfwAdapter
{
public:
    explicit PacedGlfwAdapter(double rate_hz,
        std::shared_ptr<detail::BenchmarkRecorder> benchmark = {})
        : interval_(std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(1.0 / rate_hz))), benchmark_(std::move(benchmark))
    {
        if (benchmark_) benchmark_->prepare();
    }

    void SetVSync(bool enabled) override
    {
        ::mujoco::GlfwAdapter::SetVSync(enabled);
        vsync_enabled_ = enabled;
        next_frame_ = {};
    }

    void SwapBuffers() override
    {
        ::mujoco::GlfwAdapter::SwapBuffers();
        if (!vsync_enabled_)
        {
            const auto now = Clock::now();
            if (next_frame_ == Clock::time_point{} || next_frame_ < now - interval_)
                next_frame_ = now;
            next_frame_ += interval_;
            std::this_thread::sleep_until(next_frame_);
        }
        if (benchmark_) benchmark_->frame();
    }

private:
    using Clock = std::chrono::steady_clock;
    Clock::duration interval_;
    Clock::time_point next_frame_{};
    bool vsync_enabled_ = true;
    std::shared_ptr<detail::BenchmarkRecorder> benchmark_;
};

void disable_section(mjUI& ui, const char* name)
{
    for (int section_index = 0; section_index < ui.nsect; ++section_index)
    {
        mjuiSection& section = ui.sect[section_index];
        if (std::strcmp(section.name, name) != 0)
        {
            continue;
        }
        for (int item_index = 0; item_index < section.nitem; ++item_index)
        {
            section.item[item_index].state = 0;
        }
        return;
    }
}

void make_visual_only(::mujoco::Simulate& simulate)
{
    const std::lock_guard<::mujoco::SimulateMutex> lock(simulate.mtx);
    disable_section(simulate.ui0, "Simulation");
    disable_section(simulate.ui0, "Physics");
    disable_section(simulate.ui1, "Joint");
    disable_section(simulate.ui1, "Control");
    disable_section(simulate.ui1, "Equality");
}

}  // namespace

bool Viewer::check_environment(std::string* error)
{
    const auto fail = [error](const std::string& reason, bool glfw_failure) {
        const char* detail = nullptr;
        if (glfw_failure) glfwGetError(&detail);
        if (error)
        {
            *error = "Viewer unavailable: " + reason;
            if (detail) *error += std::string(" (GLFW: ") + detail + ")";
            if (glfwGetCurrentContext())
            {
                using GetString = const unsigned char* (*)(unsigned int);
                auto get_string = reinterpret_cast<GetString>(glfwGetProcAddress("glGetString"));
                if (get_string)
                {
                    for (const auto& entry : {std::pair{0x1F00u, "vendor"},
                                             std::pair{0x1F01u, "renderer"},
                                             std::pair{0x1F02u, "version"}})
                    {
                        const auto* value = get_string(entry.first);
                        if (value) *error += std::string("\nOpenGL ") + entry.second + ": " +
                            reinterpret_cast<const char*>(value);
                    }
                }
            }
            *error += "\nRequired: a Linux graphical desktop and desktop OpenGL 1.5+ "
                "with compatibility functions, GL_ARB_framebuffer_object and "
                "GL_ARB_vertex_buffer_object."
                "\nSee CONFIGURATION.md (Graphics requirements) or "
                "CONFIGURATION.zh-CN.md (图形环境要求)."
                "\nCheck the display session and GPU driver. For NVIDIA, run nvidia-smi; "
                "if it reports Driver/library version mismatch after an update, "
                "save your work and reboot.";
        }
        return false;
    };
    // Do not let an unrelated earlier GLFW error replace a capability diagnosis.
    glfwGetError(nullptr);
    if (!glfwInit()) return fail("could not initialize GLFW", true);
    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 1);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    GLFWwindow* window = glfwCreateWindow(64, 64, "Sharpa MuJoCo graphics check", nullptr, nullptr);
    if (!window)
    {
        fail("could not create an OpenGL window", true);
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window);
    const int major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    const int minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    const char* capability_error = nullptr;
    if (glfwGetWindowAttrib(window, GLFW_OPENGL_PROFILE) == GLFW_OPENGL_CORE_PROFILE ||
        glfwGetWindowAttrib(window, GLFW_OPENGL_FORWARD_COMPAT) ||
        (major == 3 && minor == 1 && !glfwExtensionSupported("GL_ARB_compatibility")))
        capability_error = "MuJoCo's classic Viewer requires OpenGL compatibility functions";
    else if (!glfwExtensionSupported("GL_ARB_framebuffer_object"))
        capability_error = "GL_ARB_framebuffer_object is unavailable";
    else if (!glfwExtensionSupported("GL_ARB_vertex_buffer_object"))
        capability_error = "GL_ARB_vertex_buffer_object is unavailable";
    if (capability_error) fail(capability_error, false);
    glfwDestroyWindow(window);
    glfwTerminate();
    if (capability_error) return false;
    if (error) error->clear();
    return true;
}

class Viewer::Impl
{
public:
    explicit Impl(ViewerConfig config)
        : config_(config)
    {
    }

    ~Impl()
    {
        request_exit();
        join_sync_thread();
        release_visual_state();
    }

    bool run(
        Runtime& runtime,
        const std::atomic<bool>& keep_running,
        std::string* error)
    {
        if (running_.load())
        {
            return fail("Viewer is already running.", error);
        }
        if (!std::isfinite(config_.sync_rate_hz) || config_.sync_rate_hz <= 0.0)
        {
            return fail("Viewer sync rate must be finite and positive.", error);
        }
        if (!clone_visual_state(runtime, error))
        {
            return false;
        }

        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&options_);
        mjv_defaultPerturb(&perturb_);

        std::shared_ptr<detail::BenchmarkRecorder> benchmark;
        try
        {
            if (!config_.benchmark_report.empty())
                benchmark = std::make_shared<detail::BenchmarkRecorder>(config_.benchmark_report, runtime);
            simulate_ = std::make_unique<::mujoco::Simulate>(
                std::make_unique<PacedGlfwAdapter>(config_.sync_rate_hz, benchmark),
                &camera_,
                &options_,
                &perturb_,
                true);
            simulate_->vsync = config_.vsync ? 1 : 0;
            running_.store(true);
            sync_thread_ = std::thread(
                [this, &runtime, &keep_running]() {
                    sync_loop(runtime, keep_running);
                });
            simulate_->RenderLoop();
            if (benchmark) benchmark->finish();
        }
        catch (const std::exception& exception)
        {
            set_thread_error(std::string("Viewer failed: ") + exception.what());
        }
        catch (...)
        {
            set_thread_error("Viewer failed with an unknown error.");
        }

        request_exit();
        cancel_pending_load();
        join_sync_thread();
        running_.store(false);

        const std::string thread_error = take_thread_error();
        simulate_.reset();
        release_visual_state();
        if (!thread_error.empty())
        {
            return fail(thread_error, error);
        }
        return true;
    }

    void request_exit() noexcept
    {
        if (simulate_)
        {
            simulate_->exitrequest.store(1);
        }
    }

    bool is_running() const noexcept
    {
        return running_.load();
    }

private:
    bool clone_visual_state(Runtime& runtime, std::string* error)
    {
        std::string native_error;
        const bool copied = runtime.read_native(
            [this](const mjModel* model, const mjData* data) {
                model_ = mj_copyModel(nullptr, model);
                if (!model_)
                {
                    throw std::runtime_error("could not copy the MuJoCo model");
                }
                data_ = mj_makeData(model_);
                if (!data_)
                {
                    throw std::runtime_error(
                        "could not allocate MuJoCo visualization data");
                }
                if (!mjv_copyData(data_, model_, data))
                {
                    throw std::runtime_error(
                        "could not copy MuJoCo visualization data");
                }
            },
            &native_error);
        if (!copied)
        {
            release_visual_state();
            return fail(native_error, error);
        }
        model_path_ = runtime.model_info().source_path;
        return true;
    }

    void sync_loop(
        Runtime& runtime,
        const std::atomic<bool>& keep_running)
    {
        try
        {
            simulate_->Load(model_, data_, model_path_.c_str());
            if (simulate_->exitrequest.load())
            {
                return;
            }
            make_visual_only(*simulate_);

            using Clock = std::chrono::steady_clock;
            const std::chrono::duration<double> interval(
                1.0 / config_.sync_rate_hz);
            auto next_sync = Clock::now();

            while (keep_running.load() &&
                   runtime.is_running() &&
                   !simulate_->exitrequest.load())
            {
                std::string native_error;
                if (!runtime.read_native(
                        [this](const mjModel*, const mjData* data) {
                            if (!mjv_copyData(data_, model_, data))
                            {
                                throw std::runtime_error(
                                    "could not synchronize visualization data");
                            }
                        },
                        &native_error))
                {
                    throw std::runtime_error(native_error);
                }

                {
                    const std::lock_guard<::mujoco::SimulateMutex> lock(
                        simulate_->mtx);
                    if (simulate_->exitrequest.load())
                    {
                        break;
                    }
                    simulate_->Sync(false);
                }
                next_sync +=
                    std::chrono::duration_cast<Clock::duration>(interval);
                std::this_thread::sleep_until(next_sync);
                if (next_sync < Clock::now() - interval)
                {
                    next_sync = Clock::now();
                }
            }
        }
        catch (const std::exception& exception)
        {
            set_thread_error(
                std::string("Viewer synchronization failed: ") +
                exception.what());
        }
        catch (...)
        {
            set_thread_error(
                "Viewer synchronization failed with an unknown error.");
        }
        request_exit();
    }

    void cancel_pending_load() noexcept
    {
        if (!simulate_)
        {
            return;
        }
        {
            const std::lock_guard<::mujoco::SimulateMutex> lock(simulate_->mtx);
            simulate_->loadrequest = 0;
        }
        simulate_->cond_loadrequest.notify_all();
    }

    void join_sync_thread() noexcept
    {
        if (sync_thread_.joinable())
        {
            sync_thread_.join();
        }
    }

    void release_visual_state() noexcept
    {
        if (data_)
        {
            mj_deleteData(data_);
            data_ = nullptr;
        }
        if (model_)
        {
            mj_deleteModel(model_);
            model_ = nullptr;
        }
    }

    void set_thread_error(std::string message)
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if (thread_error_.empty())
        {
            thread_error_ = std::move(message);
        }
    }

    std::string take_thread_error()
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        return std::move(thread_error_);
    }

    static bool fail(const std::string& message, std::string* error)
    {
        if (error)
        {
            *error = message;
        }
        return false;
    }

    ViewerConfig config_;
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    std::string model_path_;
    mjvCamera camera_{};
    mjvOption options_{};
    mjvPerturb perturb_{};
    std::unique_ptr<::mujoco::Simulate> simulate_;
    std::thread sync_thread_;
    std::atomic<bool> running_{false};
    std::mutex error_mutex_;
    std::string thread_error_;
};

Viewer::Viewer(ViewerConfig config)
    : impl_(std::make_unique<Impl>(config))
{
}

Viewer::~Viewer() = default;

bool Viewer::run(
    Runtime& runtime,
    const std::atomic<bool>& keep_running,
    std::string* error)
{
    return impl_->run(runtime, keep_running, error);
}

void Viewer::request_exit() noexcept
{
    impl_->request_exit();
}

bool Viewer::is_running() const noexcept
{
    return impl_->is_running();
}

}  // namespace sharpa::mujoco
