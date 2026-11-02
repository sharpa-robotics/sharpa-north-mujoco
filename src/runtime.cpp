// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/runtime.h"

#include "mjcf_path_utils.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cctype>
#include <cstring>
#include <exception>
#include <filesystem>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <thread>
#include <unordered_map>
#include <utility>

namespace sharpa::mujoco
{
namespace
{

using TimingClock = std::chrono::steady_clock;

double elapsed_seconds(
    const TimingClock::time_point start,
    const TimingClock::time_point end)
{
    return std::chrono::duration<double>(end - start).count();
}

std::string object_name(const mjModel* model, mjtObj type, int id, const char* fallback_prefix)
{
    if (const char* name = mj_id2name(model, type, id))
    {
        return name;
    }
    return std::string(fallback_prefix) + std::to_string(id);
}

int joint_qpos_width(int joint_type)
{
    switch (joint_type)
    {
        case mjJNT_FREE:
            return 7;
        case mjJNT_BALL:
            return 4;
        case mjJNT_SLIDE:
        case mjJNT_HINGE:
            return 1;
        default:
            return 0;
    }
}

int joint_dof_width(int joint_type)
{
    switch (joint_type)
    {
        case mjJNT_FREE:
            return 6;
        case mjJNT_BALL:
            return 3;
        case mjJNT_SLIDE:
        case mjJNT_HINGE:
            return 1;
        default:
            return 0;
    }
}

std::string lowercase_extension(const std::string& path)
{
    const std::size_t dot = path.find_last_of('.');
    if (dot == std::string::npos)
    {
        return {};
    }

    std::string extension = path.substr(dot);
    std::transform(
        extension.begin(),
        extension.end(),
        extension.begin(),
        [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    return extension;
}

}  // namespace

class Runtime::Impl
{
public:
    explicit Impl(RuntimeConfig initial_config)
        : config_(initial_config)
    {
        if (config_.max_catch_up_steps == 0)
        {
            config_.max_catch_up_steps = 1;
        }
        if (config_.snapshot_rate_hz < 0.0)
        {
            config_.snapshot_rate_hz = 0.0;
        }
        if (config_.action_hold_substeps == 0)
        {
            config_.action_hold_substeps = 1;
        }
        config_.minimum_step_interval_scale =
            std::clamp(config_.minimum_step_interval_scale, 0.5, 1.0);
        config_.maximum_pacing_catch_up_seconds =
            std::max(0.0, config_.maximum_pacing_catch_up_seconds);
        config_.action_wait_timeout_seconds =
            std::max(0.0, config_.action_wait_timeout_seconds);
    }

    ~Impl()
    {
        unload();
    }

    bool load(const std::string& model_path, std::string* error)
    {
        pause();
        if (model_path.empty())
        {
            return fail("Model path must not be empty.", error);
        }

        mjSpec* loaded_spec = nullptr;
        mjModel* loaded_model = nullptr;
        if (lowercase_extension(model_path) == ".mjb")
        {
            loaded_model = mj_loadModel(model_path.c_str(), nullptr);
            if (!loaded_model)
            {
                return fail("Failed to load MJB model: " + model_path, error);
            }
        }
        else
        {
            char mujoco_error[1024] = {};
            loaded_spec = mj_parseXML(
                model_path.c_str(),
                nullptr,
                mujoco_error,
                static_cast<int>(sizeof(mujoco_error)));
            if (!loaded_spec)
            {
                const std::string detail = mujoco_error[0] != '\0' ? mujoco_error : "unknown MuJoCo error";
                return fail("Failed to parse MJCF model '" + model_path + "': " + detail, error);
            }
            loaded_model = mj_compile(loaded_spec, nullptr);
            if (!loaded_model)
            {
                const char* spec_error = mjs_getError(loaded_spec);
                const std::string detail =
                    spec_error && spec_error[0] != '\0' ? spec_error : "unknown MuJoCo compile error";
                mj_deleteSpec(loaded_spec);
                return fail("Failed to compile MJCF model '" + model_path + "': " + detail, error);
            }
        }

        mjData* loaded_data = mj_makeData(loaded_model);
        if (!loaded_data)
        {
            mj_deleteModel(loaded_model);
            if (loaded_spec)
            {
                mj_deleteSpec(loaded_spec);
            }
            return fail("MuJoCo could not allocate mjData for: " + model_path, error);
        }

        std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
        destroy_model_locked();
        spec_ = loaded_spec;
        model_ = loaded_model;
        data_ = loaded_data;
        model_path_ = model_path;
        record_data_reset_locked(true);
        mj_forward(model_, data_);
        rebuild_model_info_locked();
        publish_snapshot_locked();
        state_.store(RuntimeState::Loaded);
        clear_last_error();
        return true;
    }

    bool load_scene(const SceneDescription& scene, std::string* error)
    {
        pause();
        std::string assembly_error;
        std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> loaded_spec(
            SceneBuilder::assemble(scene, &assembly_error), &mj_deleteSpec);
        if (!loaded_spec)
        {
            return fail(
                assembly_error.empty()
                    ? "Failed to assemble the runtime scene."
                    : assembly_error,
                error);
        }

        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> loaded_model(
            mj_compile(loaded_spec.get(), nullptr), &mj_deleteModel);
        if (!loaded_model)
        {
            const char* spec_error = mjs_getError(loaded_spec.get());
            return fail(
                "Failed to compile assembled scene: " +
                    std::string(
                        spec_error && spec_error[0] != '\0'
                            ? spec_error
                            : "unknown MuJoCo compile error"),
                error);
        }
        std::unique_ptr<mjData, decltype(&mj_deleteData)> loaded_data(
            mj_makeData(loaded_model.get()), &mj_deleteData);
        if (!loaded_data)
        {
            return fail("MuJoCo could not allocate mjData for the assembled scene.", error);
        }

        std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
        destroy_model_locked();
        spec_ = loaded_spec.release();
        model_ = loaded_model.release();
        data_ = loaded_data.release();
        model_path_ = scene.robot_xml_path;
        record_data_reset_locked(true);
        mj_forward(model_, data_);
        rebuild_model_info_locked();
        publish_snapshot_locked();
        state_.store(RuntimeState::Loaded);
        clear_last_error();
        return true;
    }

    void unload()
    {
        pause();
        {
            std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
            destroy_model_locked();
        }
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            latest_snapshot_ = {};
        }
        state_.store(RuntimeState::Empty);
    }

    bool start(std::string* error)
    {
        if (state_.load() == RuntimeState::Running)
        {
            return true;
        }

        {
            std::shared_lock<std::shared_mutex> lock(simulation_mutex_);
            if (!model_ || !data_)
            {
                return fail("Cannot start before a model is loaded.", error);
            }
        }

        if (worker_.joinable())
        {
            if (worker_.get_id() == std::this_thread::get_id())
            {
                return fail("Cannot restart Runtime from its simulation callback thread.", error);
            }
            worker_.join();
        }

        stop_requested_.store(false);
        state_.store(RuntimeState::Running);
        worker_ = std::thread([this]() { run_loop(); });
        clear_last_error();
        return true;
    }

    void pause()
    {
        stop_requested_.store(true);
        wake_condition_.notify_all();

        if (worker_.joinable())
        {
            if (worker_.get_id() == std::this_thread::get_id())
            {
                return;
            }
            worker_.join();
        }

        if (model_ && state_.load() != RuntimeState::Faulted)
        {
            state_.store(RuntimeState::Paused);
        }
    }

    bool reset(std::string* error)
    {
        const bool restart = state_.load() == RuntimeState::Running;
        pause();

        {
            std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
            if (!model_ || !data_)
            {
                return fail("Cannot reset before a model is loaded.", error);
            }

            mj_resetData(model_, data_);
            record_data_reset_locked(true);
            mj_forward(model_, data_);
            publish_snapshot_locked();
        }

        state_.store(RuntimeState::Paused);
        clear_last_error();
        return !restart || start(error);
    }

    bool step(std::size_t count, std::string* error)
    {
        if (count == 0)
        {
            return true;
        }
        if (state_.load() == RuntimeState::Running)
        {
            return fail("Manual stepping is not allowed while the simulation thread is running.", error);
        }

        PoseSnapshot callback_snapshot;
        bool published = false;
        {
            std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
            if (!model_ || !data_)
            {
                return fail("Cannot step before a model is loaded.", error);
            }

            for (std::size_t index = 0; index < count; ++index)
            {
                const bool measure_timing = timing_enabled_;
                const auto simulation_step_started_at = measure_timing
                    ? TimingClock::now()
                    : TimingClock::time_point{};
                const double simulation_time_before_step = data_->time;
                const auto warnings_before_step = dynamics_reset_warning_counts_locked();
                apply_pending_command_locked();
                NativeStepContext context;
                context.step_index = step_index_ + 1;
                context.frame_substep_index = 0;
                context.frame_substep_count = 1;
                context.timestep = std::max(1e-6, static_cast<double>(model_->opt.timestep));
                if (invoke_before_step_locked(context) == StepDecision::Wait)
                {
                    return fail("Manual step is waiting for an action.", error);
                }
                invoke_model_updaters_locked(context);
                apply_direct_targets_locked();
                const auto physics_step_started_at = measure_timing
                    ? TimingClock::now()
                    : TimingClock::time_point{};
                mj_step(model_, data_);
                const auto physics_step_finished_at = measure_timing
                    ? TimingClock::now()
                    : TimingClock::time_point{};
                step_index_ = context.step_index;
                record_automatic_data_reset_locked(
                    simulation_time_before_step, warnings_before_step);
                if (!direct_targets_.empty())
                {
                    apply_direct_targets_locked();
                    mj_forward(model_, data_);
                }
                invoke_after_step_locked(context);
                if (measure_timing)
                {
                    latest_physics_step_seconds_ = elapsed_seconds(
                        physics_step_started_at, physics_step_finished_at);
                    latest_simulation_step_seconds_ = elapsed_seconds(
                        simulation_step_started_at, TimingClock::now());
                }
            }
            callback_snapshot = publish_snapshot_locked();
            published = true;
        }

        state_.store(RuntimeState::Paused);
        clear_last_error();
        if (published)
        {
            invoke_snapshot_callback(callback_snapshot);
        }
        return true;
    }

    bool submit_control(const ControlCommand& command, std::string* error)
    {
        std::shared_lock<std::shared_mutex> lock(simulation_mutex_);
        if (!model_ || !data_)
        {
            return fail("Cannot submit control before a model is loaded.", error);
        }

        if (command.ordered_values.size() > static_cast<std::size_t>(model_->nu))
        {
            return fail(
                "Ordered control count exceeds model actuator count (" +
                    std::to_string(command.ordered_values.size()) + " > " + std::to_string(model_->nu) + ").",
                error);
        }

        for (std::size_t index = 0; index < command.ordered_values.size(); ++index)
        {
            if (!std::isfinite(command.ordered_values[index]))
            {
                return fail("Control value must be finite at index " + std::to_string(index), error);
            }
        }
        for (const NamedControl& named : command.named_values)
        {
            if (!std::isfinite(named.value))
            {
                return fail("Control value must be finite for actuator: " + named.actuator, error);
            }
            const auto found = actuator_ids_.find(named.actuator);
            if (found == actuator_ids_.end())
            {
                return fail("Unknown actuator: " + named.actuator, error);
            }
        }

        if (command.mode == ControlMode::DirectJointPosition)
        {
            for (std::size_t index = 0; index < command.ordered_values.size(); ++index)
            {
                if (!supports_direct_position_locked(static_cast<int>(index)))
                {
                    return fail(
                        "Actuator does not target a scalar hinge/slide joint: " + std::to_string(index),
                        error);
                }
            }
            for (const NamedControl& named : command.named_values)
            {
                const int actuator_id = actuator_ids_.at(named.actuator);
                if (!supports_direct_position_locked(actuator_id))
                {
                    return fail(
                        "Actuator does not target a scalar hinge/slide joint: " + named.actuator,
                        error);
                }
            }
        }

        {
            std::lock_guard<std::mutex> command_lock(command_mutex_);
            if (!pending_command_ ||
                pending_command_->mode != command.mode ||
                command.replace_all)
            {
                pending_command_ = command;
            }
            else
            {
                ControlCommand& pending = *pending_command_;
                pending.replace_all = pending.replace_all && command.replace_all;
                if (!command.ordered_values.empty())
                {
                    if (pending.ordered_values.size() <
                        command.ordered_values.size())
                    {
                        pending.ordered_values.resize(
                            command.ordered_values.size(), 0.0);
                    }
                    std::copy(
                        command.ordered_values.begin(),
                        command.ordered_values.end(),
                        pending.ordered_values.begin());
                }
                for (const NamedControl& incoming : command.named_values)
                {
                    const auto existing = std::find_if(
                        pending.named_values.begin(),
                        pending.named_values.end(),
                        [&](const NamedControl& current) {
                            return current.actuator == incoming.actuator;
                        });
                    if (existing == pending.named_values.end())
                    {
                        pending.named_values.push_back(incoming);
                    }
                    else
                    {
                        existing->value = incoming.value;
                    }
                }
            }
            pending_action_frame_ = pending_action_frame_ || command.opens_action_frame;
        }
        notify_step_ready();
        clear_last_error();
        return true;
    }

    bool set_mocap_pose(const std::string& body_name, const Pose& pose, std::string* error)
    {
        std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
        if (!model_ || !data_)
        {
            return fail("Cannot set a mocap pose before a model is loaded.", error);
        }

        const int body_id = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
        if (body_id < 0)
        {
            return fail("Unknown body: " + body_name, error);
        }

        const int mocap_id = model_->body_mocapid[body_id];
        if (mocap_id < 0 || mocap_id >= model_->nmocap)
        {
            return fail("Body is not a mocap body: " + body_name, error);
        }

        for (double value : pose.position)
        {
            if (!std::isfinite(value))
            {
                return fail("Mocap position must contain only finite values.", error);
            }
        }
        double quaternion_scale = 0.0;
        for (double value : pose.quaternion)
        {
            if (!std::isfinite(value))
            {
                return fail("Mocap quaternion must contain only finite values.", error);
            }
            quaternion_scale = std::max(quaternion_scale, std::abs(value));
        }
        if (quaternion_scale == 0.0)
        {
            return fail("Mocap quaternion must not be zero.", error);
        }
        // Scale first so normalization cannot overflow or underflow for finite
        // nonzero inputs. Validate the whole pose before changing mjData.
        auto quaternion = pose.quaternion;
        for (double& value : quaternion) value /= quaternion_scale;
        mju_normalize4(quaternion.data());
        std::copy(pose.position.begin(), pose.position.end(), data_->mocap_pos + 3 * mocap_id);
        std::copy(quaternion.begin(), quaternion.end(), data_->mocap_quat + 4 * mocap_id);
        mj_forward(model_, data_);
        publish_snapshot_locked();
        clear_last_error();
        return true;
    }

    bool set_equality_active(const std::string& equality_name, bool active, std::string* error)
    {
        return set_equalities_active({equality_name}, active, error);
    }

    bool set_equalities_active(
        const std::vector<std::string>& equality_names,
        bool active,
        std::string* error)
    {
        std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
        if (!model_ || !data_)
        {
            return fail("Cannot change an equality constraint before a model is loaded.", error);
        }

        std::vector<int> equality_ids;
        equality_ids.reserve(equality_names.size());
        for (const std::string& equality_name : equality_names)
        {
            const int equality_id =
                mj_name2id(model_, mjOBJ_EQUALITY, equality_name.c_str());
            if (equality_id < 0)
            {
                return fail("Unknown equality constraint: " + equality_name, error);
            }
            if (std::find(equality_ids.begin(), equality_ids.end(), equality_id) !=
                equality_ids.end())
            {
                continue;
            }
            if (active && model_->eq_type[equality_id] == mjEQ_JOINT &&
                model_->eq_obj2id[equality_id] < 0)
            {
                const int joint_id = model_->eq_obj1id[equality_id];
                if (joint_id < 0 || joint_id >= model_->njnt ||
                    model_->jnt_type[joint_id] == mjJNT_FREE ||
                    model_->jnt_type[joint_id] == mjJNT_BALL)
                {
                    return fail(
                        "Cannot retarget startup brake equality: " + equality_name,
                        error);
                }
            }
            equality_ids.push_back(equality_id);
        }

        for (const int equality_id : equality_ids)
        {
            if (active && model_->eq_type[equality_id] == mjEQ_JOINT &&
                model_->eq_obj2id[equality_id] < 0)
            {
                const int joint_id = model_->eq_obj1id[equality_id];
                const int qpos_address = model_->jnt_qposadr[joint_id];
                model_->eq_data[mjNEQDATA * equality_id] =
                    data_->qpos[qpos_address];
            }
            data_->eq_active[equality_id] = active ? 1 : 0;
        }
        if (equality_ids.empty())
        {
            clear_last_error();
            return true;
        }
        mj_forward(model_, data_);
        publish_snapshot_locked();
        clear_last_error();
        return true;
    }

    bool read_native(const NativeReadCallback& callback, std::string* error) const
    {
        if (!callback)
        {
            return const_cast<Impl*>(this)->fail("Native read callback must not be empty.", error);
        }

        std::shared_lock<std::shared_mutex> lock(simulation_mutex_);
        if (!model_ || !data_)
        {
            return const_cast<Impl*>(this)->fail(
                "Cannot read native data before a model is loaded.",
                error);
        }

        try
        {
            callback(model_, data_);
        }
        catch (const std::exception& exception)
        {
            return const_cast<Impl*>(this)->fail(
                std::string("Native read callback failed: ") + exception.what(),
                error);
        }
        catch (...)
        {
            return const_cast<Impl*>(this)->fail("Native read callback failed.", error);
        }
        return true;
    }

    bool write_native(
        const NativeWriteCallback& callback,
        bool forward_after,
        std::string* error)
    {
        if (!callback)
        {
            return fail("Native write callback must not be empty.", error);
        }

        PoseSnapshot snapshot;
        {
            std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
            if (!model_ || !data_)
            {
                return fail("Cannot write native data before a model is loaded.", error);
            }

            try
            {
                callback(model_, data_);
            }
            catch (const std::exception& exception)
            {
                return fail(
                    std::string("Native write callback failed: ") + exception.what(),
                    error);
            }
            catch (...)
            {
                return fail("Native write callback failed.", error);
            }

            if (forward_after)
            {
                mj_forward(model_, data_);
            }
            snapshot = publish_snapshot_locked();
        }

        invoke_snapshot_callback(snapshot);
        clear_last_error();
        return true;
    }

    bool has_editable_spec() const
    {
        std::shared_lock<std::shared_mutex> lock(simulation_mutex_);
        return spec_ != nullptr;
    }

    bool read_spec(const NativeSpecReadCallback& callback, std::string* error) const
    {
        if (!callback)
        {
            return const_cast<Impl*>(this)->fail("Native spec read callback must not be empty.", error);
        }

        std::shared_lock<std::shared_mutex> lock(simulation_mutex_);
        if (!spec_)
        {
            return const_cast<Impl*>(this)->fail(
                "The loaded model does not have an editable specification.",
                error);
        }

        try
        {
            callback(spec_);
        }
        catch (const std::exception& exception)
        {
            return const_cast<Impl*>(this)->fail(
                std::string("Native spec read callback failed: ") + exception.what(),
                error);
        }
        catch (...)
        {
            return const_cast<Impl*>(this)->fail("Native spec read callback failed.", error);
        }

        const_cast<Impl*>(this)->clear_last_error();
        return true;
    }

    bool edit_spec(const NativeSpecEditCallback& callback, std::string* error)
    {
        if (!callback)
        {
            return fail("Native spec edit callback must not be empty.", error);
        }
        if (state_.load() == RuntimeState::Running)
        {
            return fail("Cannot edit the model specification while the runtime is running.", error);
        }

        std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
        if (!spec_ || !model_ || !data_)
        {
            return fail("The loaded model does not have an editable specification.", error);
        }

        try
        {
            if (!callback(spec_))
            {
                return fail("Native spec edit callback rejected the edit.", error);
            }
        }
        catch (const std::exception& exception)
        {
            return fail(
                std::string("Native spec edit callback failed: ") + exception.what(),
                error);
        }
        catch (...)
        {
            return fail("Native spec edit callback failed.", error);
        }

        clear_last_error();
        return true;
    }

    bool assemble_spec_transactional(
        const NativeSpecAssemblyCallback& callback,
        std::string* error)
    {
        if (!callback)
        {
            return fail("Native spec edit callback must not be empty.", error);
        }
        if (state_.load() != RuntimeState::Loaded)
        {
            return fail(
                "Model assembly is only allowed after load and before runtime start.",
                error);
        }

        PoseSnapshot snapshot;
        {
            std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
            if (!spec_ || !model_ || !data_)
            {
                return fail("The loaded model does not have an editable specification.", error);
            }
            mjSpec* candidate_spec = mj_copySpec(spec_);
            if (!candidate_spec)
            {
                return fail("Could not copy the model specification.", error);
            }
            std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> candidate(
                candidate_spec, &mj_deleteSpec);
            try
            {
                std::string callback_error;
                if (!callback(candidate.get(), &callback_error))
                {
                    return fail(
                        callback_error.empty()
                            ? "Native spec assembly callback rejected the edit."
                            : callback_error,
                        error);
                }
            }
            catch (const std::exception& exception)
            {
                return fail(
                    std::string("Native transactional spec edit callback failed: ") +
                        exception.what(),
                    error);
            }
            catch (...)
            {
                return fail("Native transactional spec edit callback failed.", error);
            }

            mjModel* candidate_model = mj_compile(candidate.get(), nullptr);
            if (!candidate_model)
            {
                const char* spec_error = mjs_getError(candidate.get());
                return fail(
                    "Failed to compile transactional model specification: " +
                        std::string(
                            spec_error && spec_error[0] != '\0'
                                ? spec_error
                                : "unknown MuJoCo compile error"),
                    error);
            }
            std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_candidate(
                candidate_model, &mj_deleteModel);
            mjData* candidate_data = mj_makeData(candidate_model);
            if (!candidate_data)
            {
                return fail(
                    "MuJoCo could not allocate transactional model data.", error);
            }
            std::unique_ptr<mjData, decltype(&mj_deleteData)> data_candidate(
                candidate_data, &mj_deleteData);

            mj_deleteData(data_);
            mj_deleteModel(model_);
            mj_deleteSpec(spec_);
            spec_ = candidate.release();
            model_ = model_candidate.release();
            data_ = data_candidate.release();
            record_data_reset_locked(true);
            actuator_ids_.clear();
            rebuild_model_info_locked();
            mj_forward(model_, data_);
            snapshot = publish_snapshot_locked();
        }
        invoke_snapshot_callback(snapshot);
        clear_last_error();
        return true;
    }

    bool recompile_spec(std::string* error)
    {
        if (state_.load() == RuntimeState::Running)
        {
            return fail("Cannot recompile the model specification while the runtime is running.", error);
        }

        PoseSnapshot snapshot;
        {
            std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
            if (!spec_ || !model_ || !data_)
            {
                return fail("The loaded model does not have an editable specification.", error);
            }
            const int result = mj_recompile(spec_, nullptr, model_, data_);
            if (result != 0)
            {
                const char* spec_error = mjs_getError(spec_);
                const std::string detail =
                    spec_error && spec_error[0] != '\0' ? spec_error : "unknown MuJoCo recompile error";
                return fail("Failed to recompile model specification: " + detail, error);
            }

            {
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                pending_command_.reset();
                pending_action_frame_ = false;
            }
            direct_targets_.clear();
            action_frame_ready_ = false;
            actuator_ids_.clear();
            rebuild_model_info_locked();
            mj_forward(model_, data_);
            snapshot = publish_snapshot_locked();
        }

        invoke_snapshot_callback(snapshot);
        clear_last_error();
        return true;
    }

    bool save_xml(const std::string& path, std::string* error) const
    {
        if (path.empty())
        {
            return fail("XML output path must not be empty.", error);
        }
        std::shared_lock<std::shared_mutex> lock(simulation_mutex_);
        if (!spec_)
        {
            return fail("The loaded model does not have an editable XML specification.", error);
        }
        std::error_code filesystem_error;
        const std::filesystem::path output(path);
        const std::filesystem::path parent = output.parent_path();
        if (!parent.empty())
        {
            std::filesystem::create_directories(parent, filesystem_error);
            if (filesystem_error)
            {
                return fail("Could not create XML output directory: " + filesystem_error.message(), error);
            }
        }
        std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> export_spec(
            mj_copySpec(spec_), &mj_deleteSpec);
        if (!export_spec)
        {
            return fail("Could not copy the model specification for XML export.", error);
        }
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> export_model(
            mj_compile(export_spec.get(), nullptr), &mj_deleteModel);
        if (!export_model)
        {
            const char* detail = mjs_getError(export_spec.get());
            return fail(
                "Could not compile the model specification for XML export: " +
                    std::string(detail ? detail : "unknown error"),
                error);
        }
        const std::filesystem::path source_directory = detail::model_directory(
            *spec_, model_path_, filesystem_error);
        if (filesystem_error)
        {
            return fail(
                "Could not resolve the source model directory: " +
                    filesystem_error.message(),
                error);
        }
        std::filesystem::path output_directory = parent;
        if (output_directory.empty())
        {
            output_directory = std::filesystem::current_path(filesystem_error);
        }
        else
        {
            output_directory = std::filesystem::absolute(
                output_directory, filesystem_error);
        }
        if (filesystem_error)
        {
            return fail(
                "Could not resolve the XML output directory: " +
                    filesystem_error.message(),
                error);
        }
        detail::rebase_asset_directories(
            *export_spec, source_directory, output_directory);

        std::array<char, 2048> save_error{};
        if (mj_saveXML(export_spec.get(), path.c_str(), save_error.data(), static_cast<int>(save_error.size())) != 0)
        {
            return fail("Could not save XML: " + std::string(save_error.data()), error);
        }
        if (error)
        {
            error->clear();
        }
        return true;
    }

    bool save_mjb(const std::string& path, std::string* error) const
    {
        if (path.empty())
        {
            return fail("MJB output path must not be empty.", error);
        }
        std::shared_lock<std::shared_mutex> lock(simulation_mutex_);
        if (!model_)
        {
            return fail("Cannot save MJB before a model is loaded.", error);
        }
        std::error_code filesystem_error;
        const std::filesystem::path output(path);
        const std::filesystem::path parent = output.parent_path();
        if (!parent.empty())
        {
            std::filesystem::create_directories(parent, filesystem_error);
            if (filesystem_error)
            {
                return fail("Could not create MJB output directory: " + filesystem_error.message(), error);
            }
        }
        mj_saveModel(model_, path.c_str(), nullptr, 0);
        if (!std::filesystem::is_regular_file(output, filesystem_error) ||
            std::filesystem::file_size(output, filesystem_error) == 0)
        {
            return fail("MuJoCo did not create a valid MJB: " + path, error);
        }
        if (error)
        {
            error->clear();
        }
        return true;
    }

    RuntimeState state() const noexcept
    {
        return state_.load();
    }

    std::uint64_t reset_generation() const noexcept
    {
        return reset_generation_.load(std::memory_order_acquire);
    }

    RuntimeConfig config() const
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        return config_;
    }

    bool set_config(const RuntimeConfig& config, std::string* error)
    {
        if (config.max_catch_up_steps == 0)
        {
            return fail("max_catch_up_steps must be at least 1.", error);
        }
        if (config.snapshot_rate_hz < 0.0)
        {
            return fail("snapshot_rate_hz must not be negative.", error);
        }
        if (config.action_hold_substeps == 0)
        {
            return fail("action_hold_substeps must be at least 1.", error);
        }
        if (config.minimum_step_interval_scale < 0.5 ||
            config.minimum_step_interval_scale > 1.0)
        {
            return fail("minimum_step_interval_scale must be between 0.5 and 1.0.", error);
        }
        if (config.maximum_pacing_catch_up_seconds < 0.0)
        {
            return fail("maximum_pacing_catch_up_seconds must not be negative.", error);
        }
        if (config.action_wait_timeout_seconds < 0.0)
        {
            return fail("action_wait_timeout_seconds must not be negative.", error);
        }

        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            config_ = config;
        }
        wake_condition_.notify_all();
        clear_last_error();
        return true;
    }

    ModelInfo model_info() const
    {
        std::shared_lock<std::shared_mutex> lock(simulation_mutex_);
        return model_info_;
    }

    PoseSnapshot latest_snapshot(SnapshotTiming* timing) const
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        PoseSnapshot snapshot = latest_snapshot_;
        if (timing)
        {
            *timing = latest_snapshot_timing_;
            timing->version = snapshot.version;
            if (latest_snapshot_published_at_ != TimingClock::time_point{})
            {
                timing->snapshot_age_seconds = std::max(
                    0.0,
                    elapsed_seconds(latest_snapshot_published_at_, TimingClock::now()));
            }
        }
        return snapshot;
    }

    void set_timing_enabled(bool enabled)
    {
        std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
        if (timing_enabled_ == enabled)
        {
            return;
        }
        timing_enabled_ = enabled;
        reset_timing_state_locked();
    }

    std::string last_error() const
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        return last_error_;
    }

    void set_snapshot_callback(SnapshotCallback callback)
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        snapshot_callback_ = std::move(callback);
    }

    void clear_snapshot_callback()
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        snapshot_callback_ = nullptr;
    }

    void set_step_callbacks(
        NativeBeforeStepCallback before_step,
        NativeAfterStepCallback after_step)
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        before_step_callback_ = std::move(before_step);
        after_step_callback_ = std::move(after_step);
        notify_step_ready();
    }

    void clear_step_callbacks()
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        before_step_callback_ = nullptr;
        after_step_callback_ = nullptr;
    }

    std::uint64_t add_model_updater(NativeModelUpdater updater)
    {
        if (!updater)
        {
            return 0;
        }
        std::lock_guard<std::mutex> lock(callback_mutex_);
        const std::uint64_t id = next_model_updater_id_++;
        model_updaters_.emplace(id, std::move(updater));
        return id;
    }

    void remove_model_updater(std::uint64_t updater_id)
    {
        if (updater_id == 0)
        {
            return;
        }
        // Wait for any in-flight simulation step before releasing callback state.
        std::unique_lock<std::shared_mutex> simulation_lock(simulation_mutex_);
        std::lock_guard<std::mutex> lock(callback_mutex_);
        model_updaters_.erase(updater_id);
    }

    std::uint64_t add_reset_observer(NativeResetObserver observer)
    {
        if (!observer)
        {
            return 0;
        }
        std::lock_guard<std::mutex> lock(callback_mutex_);
        const std::uint64_t id = next_reset_observer_id_++;
        reset_observers_.emplace(id, std::move(observer));
        return id;
    }

    void remove_reset_observer(std::uint64_t observer_id)
    {
        if (observer_id == 0)
        {
            return;
        }
        // Match model-updater removal: no reset observer may still be running
        // after this call returns.
        std::unique_lock<std::shared_mutex> simulation_lock(simulation_mutex_);
        std::lock_guard<std::mutex> lock(callback_mutex_);
        reset_observers_.erase(observer_id);
    }

    void notify_step_ready()
    {
        wake_generation_.fetch_add(1, std::memory_order_relaxed);
        wake_condition_.notify_all();
    }

private:
    bool fail(const std::string& message, std::string* error) const
    {
        {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_ = message;
        }
        if (error)
        {
            *error = message;
        }
        return false;
    }

    void clear_last_error()
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }

    void destroy_model_locked()
    {
        {
            std::lock_guard<std::mutex> command_lock(command_mutex_);
            pending_command_.reset();
            pending_action_frame_ = false;
        }
        direct_targets_.clear();
        action_frame_ready_ = false;
        step_index_ = 0;
        reset_timing_state_locked();
        actuator_ids_.clear();
        model_info_ = {};
        model_path_.clear();

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
        if (spec_)
        {
            mj_deleteSpec(spec_);
            spec_ = nullptr;
        }
    }

    void reset_timing_state_locked()
    {
        latest_simulation_step_seconds_ = 0.0;
        latest_physics_step_seconds_ = 0.0;
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        latest_snapshot_timing_ = {};
        latest_snapshot_published_at_ = {};
    }

    // Every Runtime-owned replacement or reset of mjData passes through here.
    // Device plugins use reset_generation_ as the single lifecycle boundary
    // for invalidating drive state, cached commands, and startup-brake state.
    void record_data_reset_locked(bool reset_step_sequence)
    {
        {
            std::lock_guard<std::mutex> command_lock(command_mutex_);
            pending_command_.reset();
            pending_action_frame_ = false;
        }
        direct_targets_.clear();
        action_frame_ready_ = false;
        if (reset_step_sequence)
        {
            step_index_ = 0;
        }
        reset_timing_state_locked();
        next_snapshot_sim_time_ = data_ ? data_->time : 0.0;
        const std::uint64_t generation =
            reset_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
        invoke_reset_observers_locked(generation);
    }

    std::array<int, 3> dynamics_reset_warning_counts_locked() const noexcept
    {
        if (!data_)
        {
            return {};
        }
        return {
            data_->warning[mjWARN_BADQPOS].number,
            data_->warning[mjWARN_BADQVEL].number,
            data_->warning[mjWARN_BADQACC].number,
        };
    }

    void record_automatic_data_reset_locked(
        double simulation_time_before_step,
        const std::array<int, 3>& warnings_before_step)
    {
        if (!data_)
        {
            return;
        }
        const auto warnings_after_step = dynamics_reset_warning_counts_locked();
        const bool dynamics_warning_advanced =
            warnings_after_step[0] > warnings_before_step[0] ||
            warnings_after_step[1] > warnings_before_step[1] ||
            warnings_after_step[2] > warnings_before_step[2];
        const bool simulation_time_rewound =
            data_->time + 1e-12 < simulation_time_before_step;
        const bool automatic_reset_enabled =
            (model_->opt.disableflags & mjDSBL_AUTORESET) == 0;
        if (!simulation_time_rewound &&
            !(automatic_reset_enabled && dynamics_warning_advanced))
        {
            return;
        }
        // mj_step resets mjData when the dynamics become invalid. Keep the
        // monotonic runtime step sequence, but invalidate all control state in
        // exactly the same way as an explicit reset.
        record_data_reset_locked(false);
    }

    void rebuild_model_info_locked()
    {
        model_info_ = {};
        model_info_.source_path = model_path_;
        model_info_.timestep = model_->opt.timestep;
        model_info_.body_count = static_cast<int>(model_->nbody);
        model_info_.geom_count = static_cast<int>(model_->ngeom);
        model_info_.joint_count = static_cast<int>(model_->njnt);
        model_info_.actuator_count = static_cast<int>(model_->nu);
        model_info_.qpos_count = static_cast<int>(model_->nq);
        model_info_.dof_count = static_cast<int>(model_->nv);
        model_info_.sensor_count = static_cast<int>(model_->nsensor);

        model_info_.bodies.reserve(static_cast<std::size_t>(model_->nbody));
        for (int id = 0; id < model_->nbody; ++id)
        {
            BodyInfo body;
            body.id = id;
            body.name = object_name(model_, mjOBJ_BODY, id, "body_");
            body.mocap_id = model_->body_mocapid[id];
            body.parent_id = model_->body_parentid[id];
            body.joint_address = model_->body_jntadr[id];
            body.joint_count = model_->body_jntnum[id];
            model_info_.bodies.push_back(std::move(body));
        }

        model_info_.geoms.reserve(static_cast<std::size_t>(model_->ngeom));
        for (int id = 0; id < model_->ngeom; ++id)
        {
            GeomInfo geom;
            geom.id = id;
            geom.name = object_name(model_, mjOBJ_GEOM, id, "geom_");
            geom.body_id = model_->geom_bodyid[id];
            geom.type = model_->geom_type[id];
            model_info_.geoms.push_back(std::move(geom));
        }

        model_info_.joints.reserve(static_cast<std::size_t>(model_->njnt));
        for (int id = 0; id < model_->njnt; ++id)
        {
            JointInfo joint;
            joint.id = id;
            joint.name = object_name(model_, mjOBJ_JOINT, id, "joint_");
            joint.type = model_->jnt_type[id];
            joint.qpos_address = model_->jnt_qposadr[id];
            joint.qpos_width = joint_qpos_width(joint.type);
            joint.dof_address = model_->jnt_dofadr[id];
            joint.dof_width = joint_dof_width(joint.type);
            joint.limited = model_->jnt_limited[id] != 0;
            joint.range_min = model_->jnt_range[2 * id];
            joint.range_max = model_->jnt_range[2 * id + 1];
            model_info_.joints.push_back(std::move(joint));
        }

        model_info_.actuators.reserve(static_cast<std::size_t>(model_->nu));
        for (int id = 0; id < model_->nu; ++id)
        {
            ActuatorInfo actuator;
            actuator.id = id;
            actuator.name = object_name(model_, mjOBJ_ACTUATOR, id, "actuator_");
            actuator.transmission_type = model_->actuator_trntype[id];
            actuator.transmission_id = model_->actuator_trnid[2 * id];
            actuator_ids_[actuator.name] = id;
            model_info_.actuators.push_back(std::move(actuator));
        }

        model_info_.sensors.reserve(static_cast<std::size_t>(model_->nsensor));
        for (int id = 0; id < model_->nsensor; ++id)
        {
            SensorInfo sensor;
            sensor.id = id;
            sensor.name = object_name(model_, mjOBJ_SENSOR, id, "sensor_");
            sensor.type = model_->sensor_type[id];
            sensor.data_address = model_->sensor_adr[id];
            sensor.dimension = model_->sensor_dim[id];
            model_info_.sensors.push_back(std::move(sensor));
        }
    }

    bool supports_direct_position_locked(int actuator_id) const
    {
        if (actuator_id < 0 || actuator_id >= model_->nu)
        {
            return false;
        }
        if (model_->actuator_trntype[actuator_id] != mjTRN_JOINT)
        {
            return false;
        }

        const int joint_id = model_->actuator_trnid[2 * actuator_id];
        if (joint_id < 0 || joint_id >= model_->njnt)
        {
            return false;
        }

        const int joint_type = model_->jnt_type[joint_id];
        return joint_type == mjJNT_HINGE || joint_type == mjJNT_SLIDE;
    }

    void apply_pending_command_locked()
    {
        std::optional<ControlCommand> command;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!pending_command_)
            {
                return;
            }
            command = std::move(pending_command_);
            pending_command_.reset();
            action_frame_ready_ = action_frame_ready_ || pending_action_frame_;
            pending_action_frame_ = false;
        }

        if (command->mode == ControlMode::Actuator)
        {
            if (command->replace_all)
            {
                std::fill(data_->ctrl, data_->ctrl + model_->nu, 0.0);
            }
            for (std::size_t index = 0; index < command->ordered_values.size(); ++index)
            {
                data_->ctrl[index] = command->ordered_values[index];
            }
            for (const NamedControl& named : command->named_values)
            {
                data_->ctrl[actuator_ids_.at(named.actuator)] = named.value;
            }
            return;
        }

        if (command->replace_all)
        {
            direct_targets_.clear();
        }
        for (std::size_t index = 0; index < command->ordered_values.size(); ++index)
        {
            direct_targets_[static_cast<int>(index)] = command->ordered_values[index];
        }
        for (const NamedControl& named : command->named_values)
        {
            direct_targets_[actuator_ids_.at(named.actuator)] = named.value;
        }
    }

    void apply_direct_targets_locked()
    {
        for (const auto& [actuator_id, target] : direct_targets_)
        {
            if (!supports_direct_position_locked(actuator_id))
            {
                continue;
            }

            const int joint_id = model_->actuator_trnid[2 * actuator_id];
            const int qpos_address = model_->jnt_qposadr[joint_id];
            const int dof_address = model_->jnt_dofadr[joint_id];
            data_->qpos[qpos_address] = target;
            data_->qvel[dof_address] = 0.0;
            data_->ctrl[actuator_id] = target;
        }
    }

    PoseSnapshot publish_snapshot_locked()
    {
        const bool measure_timing = timing_enabled_;
        const auto build_started_at = measure_timing
            ? TimingClock::now()
            : TimingClock::time_point{};
        PoseSnapshot snapshot;
        snapshot.version = ++snapshot_version_;
        snapshot.step_index = step_index_;
        snapshot.simulation_time = data_->time;
        snapshot.body_count = static_cast<int>(model_->nbody);
        snapshot.body_positions.assign(data_->xpos, data_->xpos + model_->nbody * 3);
        snapshot.body_quaternions.assign(data_->xquat, data_->xquat + model_->nbody * 4);
        snapshot.body_velocities.resize(static_cast<std::size_t>(model_->nbody * 6));
        for (int body_id = 0; body_id < model_->nbody; ++body_id)
        {
            mj_objectVelocity(
                model_,
                data_,
                mjOBJ_BODY,
                body_id,
                snapshot.body_velocities.data() + body_id * 6,
                0);
        }
        snapshot.geom_count = static_cast<int>(model_->ngeom);
        snapshot.geom_positions.assign(
            data_->geom_xpos,
            data_->geom_xpos + model_->ngeom * 3);
        snapshot.geom_quaternions.resize(static_cast<std::size_t>(model_->ngeom * 4));
        for (int geom_id = 0; geom_id < model_->ngeom; ++geom_id)
        {
            mju_mat2Quat(
                snapshot.geom_quaternions.data() + geom_id * 4,
                data_->geom_xmat + geom_id * 9);
        }
        snapshot.qpos.assign(data_->qpos, data_->qpos + model_->nq);
        snapshot.qvel.assign(data_->qvel, data_->qvel + model_->nv);
        snapshot.controls.assign(data_->ctrl, data_->ctrl + model_->nu);
        snapshot.actuator_forces.assign(
            data_->actuator_force,
            data_->actuator_force + model_->nu);
        snapshot.sensor_values.assign(
            data_->sensordata,
            data_->sensordata + model_->nsensordata);

        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            latest_snapshot_ = snapshot;
            if (measure_timing)
            {
                const auto published_at = TimingClock::now();
                latest_snapshot_timing_.available = true;
                latest_snapshot_timing_.version = snapshot.version;
                latest_snapshot_timing_.snapshot_age_seconds = 0.0;
                latest_snapshot_timing_.publish_interval_seconds =
                    latest_snapshot_published_at_ == TimingClock::time_point{}
                        ? 0.0
                        : elapsed_seconds(latest_snapshot_published_at_, published_at);
                latest_snapshot_timing_.simulation_step_seconds =
                    latest_simulation_step_seconds_;
                latest_snapshot_timing_.physics_step_seconds =
                    latest_physics_step_seconds_;
                latest_snapshot_timing_.snapshot_build_seconds =
                    elapsed_seconds(build_started_at, published_at);
                latest_snapshot_published_at_ = published_at;
            }
            else
            {
                latest_snapshot_timing_ = {};
                latest_snapshot_published_at_ = {};
            }
        }
        return snapshot;
    }

    bool should_publish_snapshot_locked(const RuntimeConfig& config)
    {
        if (config.snapshot_rate_hz <= 0.0)
        {
            return true;
        }
        if (data_->time + 1e-12 < next_snapshot_sim_time_)
        {
            return false;
        }

        next_snapshot_sim_time_ = data_->time + 1.0 / config.snapshot_rate_hz;
        return true;
    }

    void invoke_snapshot_callback(const PoseSnapshot& snapshot)
    {
        SnapshotCallback callback;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            callback = snapshot_callback_;
        }
        if (callback)
        {
            callback(snapshot);
        }
    }

    StepDecision invoke_before_step_locked(const NativeStepContext& context)
    {
        NativeBeforeStepCallback callback;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            callback = before_step_callback_;
        }
        return callback ? callback(model_, data_, context) : StepDecision::Proceed;
    }

    void invoke_model_updaters_locked(const NativeStepContext& context)
    {
        std::vector<NativeModelUpdater> updaters;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            updaters.reserve(model_updaters_.size());
            for (const auto& [id, updater] : model_updaters_)
            {
                (void)id;
                updaters.push_back(updater);
            }
        }
        for (const auto& updater : updaters)
        {
            updater(model_, data_, context);
        }
    }

    void invoke_reset_observers_locked(std::uint64_t generation)
    {
        std::vector<NativeResetObserver> observers;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            observers.reserve(reset_observers_.size());
            for (const auto& [id, observer] : reset_observers_)
            {
                (void)id;
                observers.push_back(observer);
            }
        }
        for (const auto& observer : observers)
        {
            observer(generation);
        }
    }

    void invoke_after_step_locked(const NativeStepContext& context)
    {
        NativeAfterStepCallback callback;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            callback = after_step_callback_;
        }
        if (callback)
        {
            callback(model_, data_, context);
        }
    }

    double timestep() const
    {
        std::shared_lock<std::shared_mutex> lock(simulation_mutex_);
        return model_ ? std::max(1e-6, static_cast<double>(model_->opt.timestep)) : 0.002;
    }

    enum class WorkerStepResult
    {
        Stepped,
        SteppedAndPublished,
        Waiting,
        Unavailable,
    };

    WorkerStepResult step_from_worker(
        PoseSnapshot& published_snapshot,
        std::size_t frame_substep_index,
        std::size_t frame_substep_count)
    {
        std::unique_lock<std::shared_mutex> lock(simulation_mutex_);
        if (!model_ || !data_)
        {
            return WorkerStepResult::Unavailable;
        }

        const bool measure_timing = timing_enabled_;
        const auto simulation_step_started_at = measure_timing
            ? TimingClock::now()
            : TimingClock::time_point{};
        const double simulation_time_before_step = data_->time;
        const auto warnings_before_step = dynamics_reset_warning_counts_locked();
        apply_pending_command_locked();
        const RuntimeConfig current_config = config();
        if (current_config.scheduler_mode == SchedulerMode::ActionGated &&
            current_config.gate_on_control_command &&
            frame_substep_index == 0 &&
            !action_frame_ready_)
        {
            return WorkerStepResult::Waiting;
        }

        NativeStepContext context;
        context.step_index = step_index_ + 1;
        context.frame_substep_index = frame_substep_index;
        context.frame_substep_count = std::max<std::size_t>(1, frame_substep_count);
        context.timestep = std::max(1e-6, static_cast<double>(model_->opt.timestep));
        if (invoke_before_step_locked(context) == StepDecision::Wait)
        {
            return WorkerStepResult::Waiting;
        }
        invoke_model_updaters_locked(context);
        if (current_config.scheduler_mode == SchedulerMode::ActionGated &&
            frame_substep_index == 0)
        {
            action_frame_ready_ = false;
        }

        apply_direct_targets_locked();
        const auto physics_step_started_at = measure_timing
            ? TimingClock::now()
            : TimingClock::time_point{};
        mj_step(model_, data_);
        const auto physics_step_finished_at = measure_timing
            ? TimingClock::now()
            : TimingClock::time_point{};
        step_index_ = context.step_index;
        record_automatic_data_reset_locked(
            simulation_time_before_step, warnings_before_step);
        if (!direct_targets_.empty())
        {
            apply_direct_targets_locked();
            mj_forward(model_, data_);
        }
        invoke_after_step_locked(context);
        if (measure_timing)
        {
            latest_physics_step_seconds_ = elapsed_seconds(
                physics_step_started_at, physics_step_finished_at);
            latest_simulation_step_seconds_ = elapsed_seconds(
                simulation_step_started_at, TimingClock::now());
        }

        if (should_publish_snapshot_locked(current_config))
        {
            published_snapshot = publish_snapshot_locked();
            return WorkerStepResult::SteppedAndPublished;
        }
        return WorkerStepResult::Stepped;
    }

    void run_loop()
    {
        using Clock = std::chrono::steady_clock;
        auto next_deadline = Clock::now();
        auto policy_next_frame = Clock::now();
        Clock::time_point last_policy_step{};
        bool policy_scheduler_active = false;

        const auto wait_for_action = [this](std::uint64_t observed_generation, double timeout_seconds)
        {
            std::unique_lock<std::mutex> wait_lock(wake_mutex_);
            const auto predicate = [this, observed_generation]()
            {
                return stop_requested_.load() ||
                       wake_generation_.load(std::memory_order_relaxed) != observed_generation;
            };
            if (timeout_seconds <= 0.0)
            {
                wake_condition_.wait(wait_lock, predicate);
            }
            else
            {
                wake_condition_.wait_for(
                    wait_lock,
                    std::chrono::duration<double>(timeout_seconds),
                    predicate);
            }
        };

        try
        {
            while (!stop_requested_.load())
            {
                const RuntimeConfig current_config = config();
                const double step_seconds = timestep();

                if (current_config.scheduler_mode == SchedulerMode::ActionGated)
                {
                    const std::size_t required_substeps =
                        std::max<std::size_t>(1, current_config.action_hold_substeps);
                    const auto step_duration = std::chrono::duration<double>(step_seconds);
                    const auto frame_duration =
                        std::chrono::duration<double>(step_seconds * required_substeps);

                    if (!policy_scheduler_active)
                    {
                        policy_next_frame = Clock::now();
                        last_policy_step = {};
                        policy_scheduler_active = true;
                    }

                    {
                        std::unique_lock<std::mutex> wait_lock(wake_mutex_);
                        wake_condition_.wait_until(
                            wait_lock,
                            policy_next_frame,
                            [this]() { return stop_requested_.load(); });
                    }
                    if (stop_requested_.load())
                    {
                        break;
                    }

                    const auto scheduled_frame_start = policy_next_frame;
                    const auto frame_start = Clock::now();
                    const double frame_start_delay = std::max(
                        0.0,
                        std::chrono::duration<double>(
                            frame_start - scheduled_frame_start)
                            .count());
                    auto effective_frame_start = current_config.adaptive_policy_pacing
                        ? scheduled_frame_start
                        : frame_start;
                    if (current_config.adaptive_policy_pacing &&
                        frame_start_delay > current_config.maximum_pacing_catch_up_seconds)
                    {
                        effective_frame_start = frame_start;
                    }

                    const double minimum_scale =
                        std::clamp(current_config.minimum_step_interval_scale, 0.5, 1.0);
                    const auto minimum_step_interval =
                        std::chrono::duration<double>(step_seconds * minimum_scale);
                    bool frame_completed = true;
                    bool frame_waiting = false;

                    for (std::size_t substep = 0;
                         substep < required_substeps && !stop_requested_.load();
                         ++substep)
                    {
                        auto target_start =
                            effective_frame_start +
                            std::chrono::duration_cast<Clock::duration>(step_duration * substep);
                        if (current_config.adaptive_policy_pacing &&
                            last_policy_step != Clock::time_point{})
                        {
                            target_start = std::max(
                                target_start,
                                last_policy_step +
                                    std::chrono::duration_cast<Clock::duration>(
                                        minimum_step_interval));
                        }

                        {
                            std::unique_lock<std::mutex> wait_lock(wake_mutex_);
                            wake_condition_.wait_until(
                                wait_lock,
                                target_start,
                                [this]() { return stop_requested_.load(); });
                        }
                        if (stop_requested_.load())
                        {
                            frame_completed = false;
                            break;
                        }

                        const std::uint64_t observed_generation =
                            wake_generation_.load(std::memory_order_relaxed);
                        PoseSnapshot snapshot;
                        const WorkerStepResult result = step_from_worker(
                            snapshot,
                            substep,
                            required_substeps);
                        if (result == WorkerStepResult::SteppedAndPublished)
                        {
                            invoke_snapshot_callback(snapshot);
                        }
                        else if (result == WorkerStepResult::Waiting)
                        {
                            frame_completed = false;
                            frame_waiting = true;
                            wait_for_action(
                                observed_generation,
                                current_config.action_wait_timeout_seconds);
                            break;
                        }
                        else if (result == WorkerStepResult::Unavailable)
                        {
                            frame_completed = false;
                            wait_for_action(observed_generation, 0.001);
                            break;
                        }
                        last_policy_step = Clock::now();
                    }

                    if (frame_completed)
                    {
                        policy_next_frame =
                            effective_frame_start +
                            std::chrono::duration_cast<Clock::duration>(frame_duration);
                        const double behind_seconds = std::chrono::duration<double>(
                                                          Clock::now() - policy_next_frame)
                                                          .count();
                        if (current_config.adaptive_policy_pacing &&
                            behind_seconds >
                                current_config.maximum_pacing_catch_up_seconds)
                        {
                            policy_next_frame = Clock::now();
                        }
                    }
                    else if (frame_waiting)
                    {
                        policy_next_frame = Clock::now();
                    }
                    continue;
                }

                if (policy_scheduler_active)
                {
                    policy_scheduler_active = false;
                    next_deadline = Clock::now();
                }

                if (!current_config.realtime)
                {
                    const std::uint64_t observed_generation =
                        wake_generation_.load(std::memory_order_relaxed);
                    PoseSnapshot snapshot;
                    const WorkerStepResult result =
                        step_from_worker(snapshot, 0, 1);
                    if (result == WorkerStepResult::SteppedAndPublished)
                    {
                        invoke_snapshot_callback(snapshot);
                    }
                    else if (result == WorkerStepResult::Waiting)
                    {
                        wait_for_action(
                            observed_generation,
                            current_config.action_wait_timeout_seconds);
                    }
                    else if (result == WorkerStepResult::Unavailable)
                    {
                        wait_for_action(observed_generation, 0.001);
                    }
                    else
                    {
                        std::this_thread::yield();
                    }
                    continue;
                }

                {
                    std::unique_lock<std::mutex> wait_lock(wake_mutex_);
                    wake_condition_.wait_until(
                        wait_lock,
                        next_deadline,
                        [this]() { return stop_requested_.load(); });
                }
                if (stop_requested_.load())
                {
                    break;
                }

                const auto step_duration = std::chrono::duration<double>(step_seconds);
                std::size_t steps = 0;
                auto now = Clock::now();

                do
                {
                    const std::uint64_t observed_generation =
                        wake_generation_.load(std::memory_order_relaxed);
                    PoseSnapshot snapshot;
                    const WorkerStepResult result =
                        step_from_worker(snapshot, 0, 1);
                    if (result == WorkerStepResult::SteppedAndPublished)
                    {
                        invoke_snapshot_callback(snapshot);
                    }
                    else if (result == WorkerStepResult::Waiting)
                    {
                        wait_for_action(
                            observed_generation,
                            current_config.action_wait_timeout_seconds);
                        next_deadline = Clock::now();
                        break;
                    }
                    else if (result == WorkerStepResult::Unavailable)
                    {
                        wait_for_action(observed_generation, 0.001);
                        next_deadline = Clock::now();
                        break;
                    }

                    ++steps;
                    next_deadline +=
                        std::chrono::duration_cast<Clock::duration>(step_duration);
                    now = Clock::now();
                } while (
                    !stop_requested_.load() &&
                    now >= next_deadline &&
                    steps < current_config.max_catch_up_steps);

                if (now >= next_deadline)
                {
                    next_deadline =
                        now + std::chrono::duration_cast<Clock::duration>(step_duration);
                }
            }
        }
        catch (const std::exception& exception)
        {
            fail(std::string("Simulation callback failed: ") + exception.what(), nullptr);
            state_.store(RuntimeState::Faulted);
        }
        catch (...)
        {
            fail("Simulation callback failed.", nullptr);
            state_.store(RuntimeState::Faulted);
        }

        if (state_.load() != RuntimeState::Faulted)
        {
            state_.store(model_ ? RuntimeState::Paused : RuntimeState::Empty);
        }
    }

    mjSpec* spec_ = nullptr;
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    std::string model_path_;
    ModelInfo model_info_;
    std::unordered_map<std::string, int> actuator_ids_;
    std::unordered_map<int, double> direct_targets_;

    mutable std::shared_mutex simulation_mutex_;
    mutable std::mutex config_mutex_;
    RuntimeConfig config_;

    std::mutex command_mutex_;
    std::optional<ControlCommand> pending_command_;
    bool pending_action_frame_ = false;
    bool action_frame_ready_ = false;

    mutable std::mutex snapshot_mutex_;
    PoseSnapshot latest_snapshot_;
    SnapshotTiming latest_snapshot_timing_;
    TimingClock::time_point latest_snapshot_published_at_{};
    std::uint64_t snapshot_version_ = 0;
    std::uint64_t step_index_ = 0;
    double next_snapshot_sim_time_ = 0.0;
    double latest_simulation_step_seconds_ = 0.0;
    double latest_physics_step_seconds_ = 0.0;
    bool timing_enabled_ = true;

    mutable std::mutex callback_mutex_;
    SnapshotCallback snapshot_callback_;
    NativeBeforeStepCallback before_step_callback_;
    NativeAfterStepCallback after_step_callback_;
    std::unordered_map<std::uint64_t, NativeModelUpdater> model_updaters_;
    std::uint64_t next_model_updater_id_ = 1;
    std::unordered_map<std::uint64_t, NativeResetObserver> reset_observers_;
    std::uint64_t next_reset_observer_id_ = 1;

    mutable std::mutex error_mutex_;
    mutable std::string last_error_;

    std::atomic<RuntimeState> state_{RuntimeState::Empty};
    std::atomic<std::uint64_t> reset_generation_{0};
    std::atomic<bool> stop_requested_{false};
    std::thread worker_;
    std::mutex wake_mutex_;
    std::condition_variable wake_condition_;
    std::atomic<std::uint64_t> wake_generation_{0};
};

Runtime::Runtime(RuntimeConfig config)
    : impl_(std::make_unique<Impl>(config))
{
}

Runtime::~Runtime() = default;

Runtime::Runtime(Runtime&&) noexcept = default;

Runtime& Runtime::operator=(Runtime&&) noexcept = default;

bool Runtime::load(const std::string& model_path, std::string* error)
{
    return impl_->load(model_path, error);
}

bool Runtime::load_scene(const SceneDescription& scene, std::string* error)
{
    return impl_->load_scene(scene, error);
}

void Runtime::unload()
{
    impl_->unload();
}

bool Runtime::start(std::string* error)
{
    return impl_->start(error);
}

void Runtime::pause()
{
    impl_->pause();
}

bool Runtime::reset(std::string* error)
{
    return impl_->reset(error);
}

bool Runtime::step(std::size_t count, std::string* error)
{
    return impl_->step(count, error);
}

bool Runtime::submit_control(const ControlCommand& command, std::string* error)
{
    return impl_->submit_control(command, error);
}

bool Runtime::set_mocap_pose(const std::string& body_name, const Pose& pose, std::string* error)
{
    return impl_->set_mocap_pose(body_name, pose, error);
}

bool Runtime::set_equality_active(const std::string& equality_name, bool active, std::string* error)
{
    return impl_->set_equality_active(equality_name, active, error);
}

bool Runtime::set_equalities_active(
    const std::vector<std::string>& equality_names,
    bool active,
    std::string* error)
{
    return impl_->set_equalities_active(equality_names, active, error);
}

bool Runtime::read_native(const NativeReadCallback& callback, std::string* error) const
{
    return impl_->read_native(callback, error);
}

bool Runtime::write_native(
    const NativeWriteCallback& callback,
    bool forward_after,
    std::string* error)
{
    return impl_->write_native(callback, forward_after, error);
}

bool Runtime::has_editable_spec() const
{
    return impl_->has_editable_spec();
}

bool Runtime::read_spec(const NativeSpecReadCallback& callback, std::string* error) const
{
    return impl_->read_spec(callback, error);
}

bool Runtime::edit_spec(const NativeSpecEditCallback& callback, std::string* error)
{
    return impl_->edit_spec(callback, error);
}

bool Runtime::assemble_spec_transactional(
    const NativeSpecAssemblyCallback& callback,
    std::string* error)
{
    return impl_->assemble_spec_transactional(callback, error);
}

bool Runtime::recompile_spec(std::string* error)
{
    return impl_->recompile_spec(error);
}

bool Runtime::save_xml(const std::string& path, std::string* error) const
{
    return impl_->save_xml(path, error);
}

bool Runtime::save_mjb(const std::string& path, std::string* error) const
{
    return impl_->save_mjb(path, error);
}

RuntimeState Runtime::state() const noexcept
{
    return impl_->state();
}

bool Runtime::is_loaded() const noexcept
{
    const RuntimeState current = state();
    return current == RuntimeState::Loaded ||
           current == RuntimeState::Paused ||
           current == RuntimeState::Running;
}

bool Runtime::is_running() const noexcept
{
    return state() == RuntimeState::Running;
}

std::uint64_t Runtime::reset_generation() const noexcept
{
    return impl_->reset_generation();
}

RuntimeConfig Runtime::config() const
{
    return impl_->config();
}

bool Runtime::set_config(const RuntimeConfig& config, std::string* error)
{
    return impl_->set_config(config, error);
}

ModelInfo Runtime::model_info() const
{
    return impl_->model_info();
}

PoseSnapshot Runtime::latest_snapshot() const
{
    return impl_->latest_snapshot(nullptr);
}

PoseSnapshot Runtime::latest_snapshot(SnapshotTiming* timing) const
{
    return impl_->latest_snapshot(timing);
}

void Runtime::set_timing_enabled(bool enabled)
{
    impl_->set_timing_enabled(enabled);
}

std::string Runtime::last_error() const
{
    return impl_->last_error();
}

void Runtime::set_snapshot_callback(SnapshotCallback callback)
{
    impl_->set_snapshot_callback(std::move(callback));
}

void Runtime::clear_snapshot_callback()
{
    impl_->clear_snapshot_callback();
}

void Runtime::set_step_callbacks(
    NativeBeforeStepCallback before_step,
    NativeAfterStepCallback after_step)
{
    impl_->set_step_callbacks(std::move(before_step), std::move(after_step));
}

void Runtime::clear_step_callbacks()
{
    impl_->clear_step_callbacks();
}

std::uint64_t Runtime::add_model_updater(NativeModelUpdater updater)
{
    return impl_->add_model_updater(std::move(updater));
}

void Runtime::remove_model_updater(std::uint64_t updater_id)
{
    impl_->remove_model_updater(updater_id);
}

std::uint64_t Runtime::add_reset_observer(NativeResetObserver observer)
{
    return impl_->add_reset_observer(std::move(observer));
}

void Runtime::remove_reset_observer(std::uint64_t observer_id)
{
    impl_->remove_reset_observer(observer_id);
}

void Runtime::notify_step_ready()
{
    impl_->notify_step_ready();
}

}  // namespace sharpa::mujoco
