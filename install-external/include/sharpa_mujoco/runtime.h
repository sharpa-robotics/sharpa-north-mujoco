// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/export.h"
#include "sharpa_mujoco/native_access.h"
#include "sharpa_mujoco/types.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace sharpa::mujoco
{

class SHARPA_MUJOCO_API Runtime
{
public:
    using SnapshotCallback = std::function<void(const PoseSnapshot&)>;

    explicit Runtime(RuntimeConfig config = {});
    ~Runtime();

    Runtime(const Runtime&) = delete;
    Runtime& operator=(const Runtime&) = delete;
    Runtime(Runtime&&) noexcept;
    Runtime& operator=(Runtime&&) noexcept;

    bool load(const std::string& model_path, std::string* error = nullptr);
    void unload();

    bool start(std::string* error = nullptr);
    void pause();
    bool reset(std::string* error = nullptr);
    bool step(std::size_t count = 1, std::string* error = nullptr);

    bool submit_control(const ControlCommand& command, std::string* error = nullptr);
    bool set_mocap_pose(const std::string& body_name, const Pose& pose, std::string* error = nullptr);
    bool set_equality_active(const std::string& equality_name, bool active, std::string* error = nullptr);
    bool read_native(const NativeReadCallback& callback, std::string* error = nullptr) const;
    bool write_native(
        const NativeWriteCallback& callback,
        bool forward_after = true,
        std::string* error = nullptr);
    bool has_editable_spec() const;
    bool read_spec(const NativeSpecReadCallback& callback, std::string* error = nullptr) const;
    bool edit_spec(const NativeSpecEditCallback& callback, std::string* error = nullptr);
    bool recompile_spec(std::string* error = nullptr);

    RuntimeState state() const noexcept;
    bool is_loaded() const noexcept;
    bool is_running() const noexcept;

    RuntimeConfig config() const;
    bool set_config(const RuntimeConfig& config, std::string* error = nullptr);
    ModelInfo model_info() const;
    PoseSnapshot latest_snapshot() const;
    std::string last_error() const;

    void set_snapshot_callback(SnapshotCallback callback);
    void clear_snapshot_callback();
    void set_step_callbacks(
        NativeBeforeStepCallback before_step,
        NativeAfterStepCallback after_step);
    void clear_step_callbacks();
    void notify_step_ready();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sharpa::mujoco
