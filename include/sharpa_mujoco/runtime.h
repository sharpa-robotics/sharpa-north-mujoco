// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/export.h"
#include "sharpa_mujoco/native_access.h"
#include "sharpa_mujoco/scene_builder.h"
#include "sharpa_mujoco/types.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <cstdint>
#include <vector>

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
    // Assembles and compiles the scene directly into this Runtime. No XML or
    // MJB is written; use SceneBuilder::build only when an artifact is wanted.
    bool load_scene(const SceneDescription& scene, std::string* error = nullptr);
    void unload();

    bool start(std::string* error = nullptr);
    void pause();
    bool reset(std::string* error = nullptr);
    bool step(std::size_t count = 1, std::string* error = nullptr);

    // Reject non-finite values without changing the pending command.
    bool submit_control(const ControlCommand& command, std::string* error = nullptr);
    // Require finite components and a nonzero quaternion; normalize on success.
    // Invalid poses leave the current mocap pose unchanged.
    bool set_mocap_pose(const std::string& body_name, const Pose& pose, std::string* error = nullptr);
    // Activating a single-joint equality first retargets it to the joint's
    // current position, so re-engaging a brake does not snap back to startup.
    bool set_equality_active(const std::string& equality_name, bool active, std::string* error = nullptr);
    // Applies a group of equality changes atomically and performs at most one
    // forward pass. No equality is changed if validation fails.
    bool set_equalities_active(
        const std::vector<std::string>& equality_names,
        bool active,
        std::string* error = nullptr);
    bool read_native(const NativeReadCallback& callback, std::string* error = nullptr) const;
    bool write_native(
        const NativeWriteCallback& callback,
        bool forward_after = true,
        std::string* error = nullptr);
    // Model assembly is only valid after load and before the first start.
    // A successful assembly replaces the model and resets simulation state.
    bool assemble_spec_transactional(
        const NativeSpecAssemblyCallback& callback,
        std::string* error = nullptr);
    bool has_editable_spec() const;
    bool read_spec(const NativeSpecReadCallback& callback, std::string* error = nullptr) const;
    bool edit_spec(const NativeSpecEditCallback& callback, std::string* error = nullptr);
    bool recompile_spec(std::string* error = nullptr);
    bool save_xml(const std::string& path, std::string* error = nullptr) const;
    bool save_mjb(const std::string& path, std::string* error = nullptr) const;

    RuntimeState state() const noexcept;
    bool is_loaded() const noexcept;
    bool is_running() const noexcept;
    std::uint64_t reset_generation() const noexcept;

    RuntimeConfig config() const;
    bool set_config(const RuntimeConfig& config, std::string* error = nullptr);
    ModelInfo model_info() const;
    PoseSnapshot latest_snapshot() const;
    PoseSnapshot latest_snapshot(SnapshotTiming* timing) const;
    void set_timing_enabled(bool enabled);
    std::string last_error() const;

    void set_snapshot_callback(SnapshotCallback callback);
    void clear_snapshot_callback();
    void set_step_callbacks(
        NativeBeforeStepCallback before_step,
        NativeAfterStepCallback after_step);
    void clear_step_callbacks();
    std::uint64_t add_model_updater(NativeModelUpdater updater);
    void remove_model_updater(std::uint64_t updater_id);
    // Reset observers run synchronously while Runtime owns the simulation
    // lock, establishing a strict ordering between a reset and device input.
    // Observers must not call Runtime methods that acquire the simulation lock.
    std::uint64_t add_reset_observer(NativeResetObserver observer);
    void remove_reset_observer(std::uint64_t observer_id);
    void notify_step_ready();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sharpa::mujoco
