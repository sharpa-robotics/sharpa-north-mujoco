// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/arm/arm_plugin.h"

#include "sharpa_mujoco/plugins/arm/arm_udp_codec.h"
#include "sharpa_mujoco/plugins/arm/cia402.h"
#include "sharpa_mujoco/plugins/common/periodic_task.h"
#include "sharpa_mujoco/plugins/common/delayed_queue.h"
#include "sharpa_mujoco/plugins/common/startup_brake.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/runtime.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <mutex>
#include <thread>

namespace sharpa::mujoco::plugins::arm
{
namespace
{
constexpr std::size_t kAxisCount = 7;
constexpr std::int8_t kHomingMode = 6;
constexpr std::int8_t kCspMode = 8;
constexpr std::int8_t kSpeedMode = 9;
constexpr std::int8_t kTorqueMode = 10;
constexpr std::int8_t kMitMode = 11;

bool supported_mode(std::int8_t mode)
{
    return mode == kHomingMode || mode == kCspMode || mode == kSpeedMode ||
           mode == kTorqueMode || mode == kMitMode;
}

bool valid_command(const AxisControl& command)
{
    return supported_mode(command.mode) &&
           std::isfinite(command.position_rad) &&
           std::isfinite(command.velocity_rad_s) &&
           std::isfinite(command.torque_nm) &&
           std::isfinite(command.kp) &&
           std::isfinite(command.kd);
}

std::uint64_t timestamp_ns()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool fail(const std::string& message, std::string* error)
{
    if (error) *error = message;
    return false;
}
}  // namespace

class ArmPlugin::Impl
{
public:
    explicit Impl(ArmPluginConfig config)
        : config_(std::move(config)), transmission_(config_.side)
    {
    }
    ~Impl() { stop(); }

    bool start(Runtime& runtime, std::string* error)
    {
        if (running_) return true;
        if (!runtime.is_loaded()) return fail("Arm plugin requires a loaded model.", error);
        if (config_.feedback_rate_hz <= 0) return fail("Arm feedback rate must be positive.", error);
        if (!std::isfinite(config_.command_timeout_s) ||
            config_.command_timeout_s <= 0.0 ||
            !std::isfinite(config_.quick_stop_deceleration_rad_s2) ||
            config_.quick_stop_deceleration_rad_s2 <= 0.0 ||
            !std::isfinite(config_.quick_stop_velocity_threshold_rad_s) ||
            config_.quick_stop_velocity_threshold_rad_s <= 0.0 ||
            !std::isfinite(config_.quick_stop_timeout_s) ||
            config_.quick_stop_timeout_s <= 0.0 ||
            !std::isfinite(config_.quick_stop_kp) ||
            config_.quick_stop_kp <= 0.0 ||
            !std::isfinite(config_.speed_kp) || config_.speed_kp < 0.0 ||
            !std::isfinite(config_.speed_ki) || config_.speed_ki < 0.0 ||
            !std::isfinite(config_.speed_integral_limit_nm) ||
            config_.speed_integral_limit_nm < 0.0)
        {
            return fail("Arm controller configuration is invalid.", error);
        }
        for (double limit : config_.torque_limits_nm)
        {
            if (!std::isfinite(limit) || limit <= 0.0)
                return fail("Arm torque limits must be finite and positive.", error);
        }
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            for (double delay : {config_.command_delay_s[axis], config_.feedback_delay_s[axis]})
                if (!std::isfinite(delay) || delay < 0.0 || delay > 0.1)
                    return fail("Arm transport delay must be finite and in [0, 0.1] seconds.", error);
            if (config_.command_delay_s[axis] >= config_.command_timeout_s)
                return fail("Arm command delay must be below command_timeout_s.", error);
            if (!std::isfinite(config_.sim_joint_kp[axis]) ||
                config_.sim_joint_kp[axis] < 0.0 ||
                !std::isfinite(config_.sim_joint_kd[axis]) ||
                config_.sim_joint_kd[axis] < 0.0)
            {
                return fail(
                    "Arm simulated joint gains must be finite and non-negative.",
                    error);
            }
            if (!std::isfinite(config_.csp_kp[axis]) ||
                config_.csp_kp[axis] < 0.0 ||
                !std::isfinite(config_.csp_kd[axis]) ||
                config_.csp_kd[axis] < 0.0)
            {
                return fail("Arm CSP gains must be finite and non-negative.", error);
            }
        }
        if (!resolve_model(runtime.model_info(), error)) return false;
        for (double value : {config_.bus_voltage_v, config_.driver_temperature_c,
                             config_.motor_temperature_c})
            if (!std::isfinite(value) || value < -128 || value > 127)
                return fail("Arm telemetry must fit signed 8-bit fields.", error);
        if (!startup_brake_.initialize(
                runtime,
                std::vector<std::string>(
                    joint_names_.begin(), joint_names_.end()),
                error))
        {
            return false;
        }
        const PoseSnapshot snapshot = runtime.latest_snapshot();
        if (!snapshot_valid(snapshot)) return fail("Initial arm snapshot is incomplete.", error);

        std::array<bool, kAxisCount> assigned{};
        for (std::size_t i = 0; i < endpoints_.size(); ++i)
        {
            auto& endpoint = endpoints_[i];
            endpoint.config = &config_.endpoints[i];
            if (config_.sdo_port != 0)
            {
                for (std::size_t j = 0; j < i; ++j)
                    if (config_.endpoints[j].bind_ip == endpoint.config->bind_ip)
                    {
                        stop();
                        return fail("Arm SDO requires distinct board bind IPs; use sdo_port=0 for PDO-only setups.", error);
                    }
                if (!endpoint.sdo_socket.open_and_bind(
                        {endpoint.config->bind_ip, config_.sdo_port}, 50, error,
                        common::UdpBindMode::Exclusive))
                {
                    stop();
                    return false;
                }
            }
            if (endpoint.config->motor_indices.empty() ||
                endpoint.config->motor_indices.size() > 2 ||
                endpoint.config->bind_port == 0 || endpoint.config->peer_port == 0)
            {
                stop();
                return fail("Arm endpoint configuration is invalid.", error);
            }
            for (std::size_t motor : endpoint.config->motor_indices)
            {
                if (motor >= kAxisCount || assigned[motor])
                {
                    stop();
                    return fail("Arm motor indices must cover 0..6 exactly once.", error);
                }
                const auto first = endpoint.config->motor_indices.front();
                if (config_.command_delay_s[motor] != config_.command_delay_s[first] ||
                    config_.feedback_delay_s[motor] != config_.feedback_delay_s[first])
                {
                    stop();
                    return fail("Motors sharing an arm board require equal transport delays.", error);
                }
                assigned[motor] = true;
            }
            if (!endpoint.socket.open_and_bind(
                    {endpoint.config->bind_ip, endpoint.config->bind_port}, 50, error))
            {
                stop();
                return false;
            }
        }
        if (std::find(assigned.begin(), assigned.end(), false) != assigned.end())
        {
            stop();
            return fail("Arm endpoints must assign all seven motors.", error);
        }
        for (auto& queue : delayed_controls_) queue.clear();
        for (auto& queue : delayed_feedback_) queue.clear();
        feedback_min_version_ = 0;
        runtime_ = &runtime;
        reset_observer_id_ = runtime.add_reset_observer(
            [this](std::uint64_t) {
                // Runtime owns the simulation lock. This is the only reset
                // entry point; feedback must never clear newly queued input.
                std::lock_guard<std::mutex> lock(mutex_);
                reset_control_state_locked();
                if (std::any_of(config_.feedback_delay_s.begin(), config_.feedback_delay_s.end(),
                                [](double delay) { return delay > 0.0; }))
                    feedback_min_version_ = runtime_->latest_snapshot().version + 1;
            });
        if (reset_observer_id_ == 0)
        {
            runtime_ = nullptr;
            stop();
            return fail("Failed to register arm reset observer.", error);
        }
        model_updater_id_ = runtime.add_model_updater(
            [this](mjModel* model, mjData* data, const NativeStepContext& context) {
                update_control(model, data, context);
            });
        if (model_updater_id_ == 0)
        {
            stop();
            return fail("Failed to register arm control updater.", error);
        }
        running_ = true;
        for (std::size_t i = 0; i < endpoints_.size(); ++i)
        {
            endpoints_[i].receiver = std::thread([this, i] { receive_loop(i); });
            if (config_.sdo_port != 0)
                endpoints_[i].sdo_receiver = std::thread([this, i] { receive_sdo_loop(i); });
        }
        if (!feedback_.start(config_.feedback_rate_hz, [this] { send_feedback(); }))
        {
            stop();
            return fail("Failed to start arm feedback.", error);
        }
        return true;
    }

    void stop() noexcept
    {
        running_ = false;
        if (runtime_ && model_updater_id_ != 0)
        {
            runtime_->remove_model_updater(model_updater_id_);
            model_updater_id_ = 0;
        }
        if (runtime_ && reset_observer_id_ != 0)
        {
            runtime_->remove_reset_observer(reset_observer_id_);
            reset_observer_id_ = 0;
        }
        feedback_.stop();
        for (auto& endpoint : endpoints_)
        {
            endpoint.socket.close();
            endpoint.sdo_socket.close();
        }
        for (auto& endpoint : endpoints_)
        {
            if (endpoint.receiver.joinable()) endpoint.receiver.join();
            if (endpoint.sdo_receiver.joinable()) endpoint.sdo_receiver.join();
        }
        for (auto& queue : delayed_controls_) queue.clear();
        for (auto& queue : delayed_feedback_) queue.clear();
        runtime_ = nullptr;
    }

    bool running() const noexcept { return running_; }
    std::string name() const { return config_.name; }

private:
    struct Endpoint
    {
        const ArmUdpEndpointConfig* config = nullptr;
        common::UdpSocket socket;
        std::thread receiver;
        common::UdpSocket sdo_socket;
        std::thread sdo_receiver;
        std::atomic<std::uint32_t> sequence{1};
    };

    void receive_sdo_loop(std::size_t index)
    {
        auto& endpoint = endpoints_[index];
        std::array<std::uint8_t, 64> buffer{};
        while (running_)
        {
            std::size_t size = 0;
            common::UdpEndpoint source;
            if (!endpoint.sdo_socket.receive(buffer.data(), buffer.size(), size, nullptr, &source) ||
                source.ip != endpoint.config->peer_ip || source.port != config_.sdo_port)
                continue;
            std::vector<std::uint8_t> response;
            if (ArmUdpCodec::respond_sdo(buffer.data(), size,
                    endpoint.config->motor_indices.size(), response))
                endpoint.sdo_socket.send_to(source, response.data(), response.size());
        }
    }

    bool resolve_model(const ModelInfo& model, std::string* error)
    {
        const std::string prefix =
            config_.side == ArmSide::Left ? "left_arm_joint_" : "right_arm_joint_";
        for (std::size_t i = 0; i < kAxisCount; ++i)
        {
            const std::string joint_name = prefix + std::to_string(i + 1);
            const std::string actuator_name = joint_name + "_ctrl";
            const auto joint = std::find_if(model.joints.begin(), model.joints.end(),
                [&](const JointInfo& value) { return value.name == joint_name; });
            const auto actuator = std::find_if(model.actuators.begin(), model.actuators.end(),
                [&](const ActuatorInfo& value) { return value.name == actuator_name; });
            if (joint == model.joints.end() || joint->qpos_width != 1 ||
                joint->dof_width != 1 || actuator == model.actuators.end() ||
                actuator->transmission_id != joint->id)
            {
                return fail("Missing or invalid arm joint/actuator: " + joint_name, error);
            }
            qpos_[i] = joint->qpos_address;
            dof_[i] = joint->dof_address;
            actuator_ids_[i] = actuator->id;
            actuator_names_[i] = actuator_name;
            joint_names_[i] = joint_name;
        }
        return true;
    }

    bool snapshot_valid(const PoseSnapshot& snapshot) const
    {
        for (std::size_t i = 0; i < kAxisCount; ++i)
        {
            if (static_cast<std::size_t>(qpos_[i]) >= snapshot.qpos.size() ||
                static_cast<std::size_t>(dof_[i]) >= snapshot.qvel.size() ||
                static_cast<std::size_t>(actuator_ids_[i]) >= snapshot.actuator_forces.size())
                return false;
        }
        return true;
    }

    void receive_loop(std::size_t endpoint_index)
    {
        auto& endpoint = endpoints_[endpoint_index];
        std::array<std::uint8_t, 128> buffer{};
        while (running_)
        {
            std::size_t received = 0;
            common::UdpEndpoint source;
            if (!endpoint.socket.receive(buffer.data(), buffer.size(), received, nullptr, &source))
                continue;
            if (source.ip != endpoint.config->peer_ip || source.port != endpoint.config->peer_port)
            {
                continue;
            }
            DecodedControl decoded{};
            if (!ArmUdpCodec::decode_control(buffer.data(), received, decoded) ||
                decoded.axis_count != endpoint.config->motor_indices.size())
            {
                continue;
            }
            queue_control(endpoint_index, decoded);
        }
    }

    void queue_control(std::size_t endpoint_index, const DecodedControl& decoded)
    {
        if (!runtime_) return;
        const auto& motor_indices =
            config_.endpoints[endpoint_index].motor_indices;
        if (decoded.axis_count != motor_indices.size()) return;
        // Validate frame structure before enqueueing. Control words are
        // independent: one motor's enable must not suppress its peer's stop.
        for (std::size_t slot = 0; slot < decoded.axis_count; ++slot)
        {
            if (decoded.axes[slot].node_id != slot) return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        pending_controls_[endpoint_index].push(
            PendingControl{decoded, std::chrono::steady_clock::now()});
    }

    void update_control(
        const mjModel* model,
        mjData* data,
        const NativeStepContext& context)
    {
        if (!running_ || !model || !data) return;
        ArmTransmission::AxisArray joint_positions{}, joint_velocities{};
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            joint_positions[axis] = data->qpos[qpos_[axis]];
            joint_velocities[axis] = data->qvel[dof_[axis]];
        }
        const auto motor_positions = transmission_.joint_to_motor(joint_positions);
        const auto motor_velocities = transmission_.joint_to_motor(joint_velocities);
        ArmTransmission::AxisArray motor_torques{};
        std::array<bool, kAxisCount> active{};
        std::array<bool, kAxisCount> quick_stopping{};
        std::array<bool, kAxisCount> quick_stop_complete{};
        std::array<std::int8_t, kAxisCount> modes{};
        std::array<AxisControl, kAxisCount> commands{};
        ArmTransmission::AxisArray zero_offsets{};
        const auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            apply_pending_controls_locked(model, data);
            zero_offsets = zero_offsets_;
            for (std::size_t axis = 0; axis < kAxisCount; ++axis)
            {
                const bool timed_out = drives_[axis].operation_enabled() &&
                    (!command_valid_[axis] ||
                     std::chrono::duration<double>(
                         now - command_times_[axis]).count() >
                         config_.command_timeout_s);
                if (timed_out)
                {
                    drives_[axis].apply(0x0002U);
                    clear_command(axis);
                    quick_stop_initialized_[axis] = false;
                    quick_stop_elapsed_s_[axis] = 0.0;
                    speed_integrals_[axis] = 0.0;
                }
            }
            for (const auto pair :
                 {std::array<std::size_t, 2>{3, 4},
                  std::array<std::size_t, 2>{5, 6}})
            {
                const bool pair_quick_stop =
                    drives_[pair[0]].state() == Cia402State::QuickStopActive ||
                    drives_[pair[1]].state() == Cia402State::QuickStopActive;
                if (!pair_quick_stop) continue;
                for (const std::size_t axis : pair)
                {
                    if (!drives_[axis].operation_enabled()) continue;
                    drives_[axis].apply(0x0002U);
                    clear_command(axis);
                    quick_stop_initialized_[axis] = false;
                    quick_stop_elapsed_s_[axis] = 0.0;
                    speed_integrals_[axis] = 0.0;
                }
            }
            for (std::size_t axis = 0; axis < kAxisCount; ++axis)
            {
                modes[axis] = modes_[axis];
                commands[axis] = commands_[axis];
                if (drives_[axis].state() == Cia402State::QuickStopActive)
                {
                    quick_stopping[axis] = true;
                    speed_integrals_[axis] = 0.0;
                    if (!quick_stop_initialized_[axis])
                    {
                        quick_stop_reference_velocity_[axis] =
                            motor_velocities[axis];
                        quick_stop_elapsed_s_[axis] = 0.0;
                        quick_stop_initialized_[axis] = true;
                    }
                    const double velocity_step =
                        config_.quick_stop_deceleration_rad_s2 * context.timestep;
                    double& reference = quick_stop_reference_velocity_[axis];
                    if (std::abs(reference) <= velocity_step)
                        reference = 0.0;
                    else
                        reference -= std::copysign(velocity_step, reference);
                    quick_stop_elapsed_s_[axis] += context.timestep;
                    motor_torques[axis] = config_.quick_stop_kp *
                        (reference - motor_velocities[axis]);
                    quick_stop_complete[axis] =
                        (reference == 0.0 &&
                         std::abs(motor_velocities[axis]) <=
                             config_.quick_stop_velocity_threshold_rad_s) ||
                        quick_stop_elapsed_s_[axis] >= config_.quick_stop_timeout_s;
                    continue;
                }
                const bool fresh = command_valid_[axis] &&
                    std::chrono::duration<double>(now - command_times_[axis]).count() <=
                        config_.command_timeout_s;
                if (!drives_[axis].operation_enabled() ||
                    !supported_mode(modes_[axis]) || !fresh)
                {
                    speed_integrals_[axis] = 0.0;
                    continue;
                }
                active[axis] = true;
                const AxisControl& command = commands_[axis];
                switch (modes_[axis])
                {
                    case kHomingMode:
                        // Homing method: current position becomes encoder zero;
                        // hold the physical pose, never teleport MuJoCo qpos.
                        motor_torques[axis] = config_.csp_kp[axis] *
                            (homing_hold_positions_[axis] - motor_positions[axis]) -
                            config_.csp_kd[axis] * motor_velocities[axis];
                        break;
                    case kCspMode:
                        if (config_.csp_control_space ==
                            ArmCspControlSpace::Motor)
                        {
                            motor_torques[axis] =
                                config_.csp_kp[axis] *
                                    (command.position_rad + zero_offsets[axis] - motor_positions[axis]) +
                                config_.csp_kd[axis] *
                                    (command.velocity_rad_s - motor_velocities[axis]) +
                                command.torque_nm;
                        }
                        break;
                    case kMitMode:
                        motor_torques[axis] =
                            command.kp * (command.position_rad + zero_offsets[axis] - motor_positions[axis]) +
                            command.kd * (command.velocity_rad_s - motor_velocities[axis]) +
                            command.torque_nm;
                        break;
                    case kSpeedMode:
                    {
                        const double error =
                            command.velocity_rad_s - motor_velocities[axis];
                        speed_integrals_[axis] = std::clamp(
                            speed_integrals_[axis] + config_.speed_ki * error *
                                context.timestep,
                            -config_.speed_integral_limit_nm,
                            config_.speed_integral_limit_nm);
                        motor_torques[axis] =
                            config_.speed_kp * error + speed_integrals_[axis];
                        break;
                    }
                    case kTorqueMode:
                        motor_torques[axis] = command.torque_nm;
                        break;
                    default:
                        break;
                }
            }
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                if (!quick_stopping[axis] || !quick_stop_complete[axis]) continue;
                drives_[axis].complete_quick_stop();
                quick_stop_initialized_[axis] = false;
                quick_stopping[axis] = false;
                motor_torques[axis] = 0.0;
            }
            for (const auto pair :
                 {std::array<std::size_t, 2>{3, 4},
                  std::array<std::size_t, 2>{5, 6}})
            {
                const bool pair_stopping =
                    quick_stopping[pair[0]] || quick_stopping[pair[1]];
                const bool pair_complete =
                    (!quick_stopping[pair[0]] || quick_stop_complete[pair[0]]) &&
                    (!quick_stopping[pair[1]] || quick_stop_complete[pair[1]]);
                if (!pair_stopping || !pair_complete) continue;
                for (const std::size_t axis : pair)
                {
                    drives_[axis].complete_quick_stop();
                    quick_stop_initialized_[axis] = false;
                    quick_stopping[axis] = false;
                    motor_torques[axis] = 0.0;
                }
            }
        }
        if (config_.csp_control_space == ArmCspControlSpace::Joint)
        {
            ArmTransmission::AxisArray target_motor_positions{};
            ArmTransmission::AxisArray target_motor_velocities{};
            ArmTransmission::AxisArray feedforward_motor_torques{};
            std::array<bool, kAxisCount> controlled_joints{};
            for (std::size_t axis = 0; axis < kAxisCount; ++axis)
            {
                if (active[axis] && modes[axis] == kCspMode)
                {
                    target_motor_positions[axis] = commands[axis].position_rad + zero_offsets[axis];
                    target_motor_velocities[axis] = commands[axis].velocity_rad_s;
                    feedforward_motor_torques[axis] = commands[axis].torque_nm;
                }
            }
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                controlled_joints[axis] =
                    active[axis] && modes[axis] == kCspMode;
            }
            for (const auto pair :
                 {std::array<std::size_t, 2>{3, 4},
                  std::array<std::size_t, 2>{5, 6}})
            {
                const bool pair_csp =
                    active[pair[0]] && active[pair[1]] &&
                    modes[pair[0]] == kCspMode &&
                    modes[pair[1]] == kCspMode;
                controlled_joints[pair[0]] = pair_csp;
                controlled_joints[pair[1]] = pair_csp;
            }

            const auto target_joint_positions =
                transmission_.motor_to_joint(target_motor_positions);
            const auto target_joint_velocities =
                transmission_.motor_to_joint(target_motor_velocities);
            const auto feedforward_joint_torques =
                transmission_.motor_torque_to_joint(feedforward_motor_torques);
            ArmTransmission::AxisArray desired_joint_torques{};
            for (std::size_t axis = 0; axis < kAxisCount; ++axis)
            {
                if (!controlled_joints[axis]) continue;
                desired_joint_torques[axis] =
                    config_.sim_joint_kp[axis] *
                        (target_joint_positions[axis] - joint_positions[axis]) +
                    config_.sim_joint_kd[axis] *
                        (target_joint_velocities[axis] - joint_velocities[axis]) +
                    feedforward_joint_torques[axis];
            }
            const auto csp_motor_torques =
                transmission_.joint_torque_to_motor(desired_joint_torques);
            for (std::size_t axis = 0; axis < kAxisCount; ++axis)
            {
                if (active[axis] && modes[axis] == kCspMode)
                    motor_torques[axis] = csp_motor_torques[axis];
            }
        }
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            if (!std::isfinite(motor_torques[axis]))
                motor_torques[axis] = 0.0;
            motor_torques[axis] = std::clamp(
                motor_torques[axis],
                -config_.torque_limits_nm[axis],
                config_.torque_limits_nm[axis]);
        }
        for (const auto pair :
             {std::array<std::size_t, 2>{3, 4},
              std::array<std::size_t, 2>{5, 6}})
        {
            const bool first_driven =
                active[pair[0]] || quick_stopping[pair[0]];
            const bool second_driven =
                active[pair[1]] || quick_stopping[pair[1]];
            if (!first_driven || !second_driven)
            {
                motor_torques[pair[0]] = 0.0;
                motor_torques[pair[1]] = 0.0;
            }
        }
        const auto joint_torques =
            transmission_.motor_torque_to_joint(motor_torques);
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            if (actuator_ids_[axis] >= 0 && actuator_ids_[axis] < model->nu)
                data->ctrl[actuator_ids_[axis]] = joint_torques[axis];
        }
    }

    void send_feedback()
    {
        Runtime* runtime = runtime_;
        if (!running_ || !runtime) return;
        const PoseSnapshot snapshot = runtime->latest_snapshot();
        if (!snapshot_valid(snapshot)) return;
        ArmTransmission::AxisArray joints{}, velocities{}, torques{};
        for (std::size_t i = 0; i < kAxisCount; ++i)
        {
            joints[i] = snapshot.qpos[static_cast<std::size_t>(qpos_[i])];
            velocities[i] = snapshot.qvel[static_cast<std::size_t>(dof_[i])];
            torques[i] = snapshot.actuator_forces[static_cast<std::size_t>(actuator_ids_[i])];
        }
        const auto motor_positions = transmission_.joint_to_motor(joints);
        const auto motor_velocities = transmission_.joint_to_motor(velocities);
        const auto motor_torques = transmission_.joint_torque_to_motor(torques);
        std::lock_guard<std::mutex> lock(mutex_);
        if (snapshot.version < feedback_min_version_) return;
        const auto joint_zero_offsets = transmission_.motor_to_joint(zero_offsets_);
        for (std::size_t index = 0; index < endpoints_.size(); ++index)
        {
            auto& endpoint = endpoints_[index];
            std::array<AxisFeedback, 2> axes{};
            for (std::size_t slot = 0; slot < endpoint.config->motor_indices.size(); ++slot)
            {
                const std::size_t motor = endpoint.config->motor_indices[slot];
                axes[slot].node_id = static_cast<std::uint8_t>(slot);
                axes[slot].mode = modes_[motor];
                axes[slot].status_word = drives_[motor].status_word() |
                    (modes_[motor] == kHomingMode && homing_attained_[motor] ? 0x1000U : 0U);
                axes[slot].position_rad = static_cast<float>(motor_positions[motor] - zero_offsets_[motor]);
                axes[slot].velocity_rad_s = static_cast<float>(motor_velocities[motor]);
                axes[slot].torque_nm = static_cast<float>(motor_torques[motor]);
                // Direct axes are direction-corrected by platform; differential
                // axes' low-speed sensors already report joint coordinates.
                axes[slot].low_speed_position_rad = motor < 3 ? axes[slot].position_rad :
                    static_cast<float>(joints[motor] - joint_zero_offsets[motor]);
                axes[slot].low_speed_velocity_rad_s = motor < 3 ? axes[slot].velocity_rad_s :
                    static_cast<float>(velocities[motor]);
                axes[slot].bus_voltage_v = static_cast<float>(config_.bus_voltage_v);
                axes[slot].driver_temperature_c = static_cast<float>(config_.driver_temperature_c);
                axes[slot].motor_temperature_c = static_cast<float>(config_.motor_temperature_c);
                axes[slot].torque_sensor_nm = axes[slot].torque_nm;
            }
            std::uint64_t sample_timestamp = timestamp_ns();
            const double delay = config_.feedback_delay_s[endpoint.config->motor_indices.front()];
            if (delay > 0.0)
            {
                const auto now = std::chrono::steady_clock::now();
                delayed_feedback_[index].push({axes, sample_timestamp}, now + common::transport_delay(delay));
                DelayedFeedback released;
                bool ready = false;
                while (delayed_feedback_[index].pop_ready(now, released))
                {
                    axes = released.axes;
                    sample_timestamp = released.sample_timestamp;
                    ready = true;
                }
                if (!ready) continue;
                // Position/velocity/torque belong to one delayed sample. Drive
                // safety/status remains current rather than replaying old faults.
                for (std::size_t slot = 0; slot < endpoint.config->motor_indices.size(); ++slot)
                {
                    const auto motor = endpoint.config->motor_indices[slot];
                    axes[slot].mode = modes_[motor];
                    axes[slot].status_word = drives_[motor].status_word() |
                        (modes_[motor] == kHomingMode && homing_attained_[motor] ? 0x1000U : 0U);
                }
            }
            std::vector<std::uint8_t> frame;
            if (ArmUdpCodec::encode_feedback(endpoint.sequence++, axes.data(),
                    endpoint.config->motor_indices.size(), sample_timestamp, frame,
                    config_.extended_feedback))
            {
                endpoint.socket.send_to(
                    {endpoint.config->peer_ip, endpoint.config->peer_port},
                    frame.data(), frame.size());
            }
        }
    }

    void clear_command(std::size_t axis) noexcept
    {
        for (std::size_t i = 0; i < config_.endpoints.size(); ++i)
        {
            const auto& motors = config_.endpoints[i].motor_indices;
            if (std::find(motors.begin(), motors.end(), axis) != motors.end())
                delayed_controls_[i].clear();
        }
        commands_[axis] = {};
        command_times_[axis] = {};
        command_valid_[axis] = false;
    }

    struct PendingControl
    {
        DecodedControl decoded;
        std::chrono::steady_clock::time_point received_at;
    };

    struct DelayedFeedback
    {
        std::array<AxisFeedback, 2> axes{};
        std::uint64_t sample_timestamp = 0;
    };

    struct PendingControlQueue
    {
        static constexpr std::size_t kCapacity = 32;

        void push(const PendingControl& pending) noexcept
        {
            if (size < kCapacity)
            {
                entries[(head + size) % kCapacity] = pending;
                ++size;
                return;
            }
            // A bounded queue prevents a stalled simulation from allocating
            // without limit. Preserve the newest control state on overflow.
            entries[head] = pending;
            head = (head + 1) % kCapacity;
        }

        bool pop(PendingControl& pending) noexcept
        {
            if (size == 0) return false;
            pending = entries[head];
            head = (head + 1) % kCapacity;
            --size;
            return true;
        }

        void clear() noexcept
        {
            head = 0;
            size = 0;
        }

        std::array<PendingControl, kCapacity> entries{};
        std::size_t head = 0;
        std::size_t size = 0;
    };

    void apply_pending_controls_locked(const mjModel* model, mjData* data)
    {
        std::array<bool, kAxisCount> updated{};
        ArmTransmission::AxisArray joints{};
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
            joints[axis] = data->qpos[qpos_[axis]];
        const auto physical_positions = transmission_.joint_to_motor(joints);
        for (std::size_t endpoint_index = 0;
             endpoint_index < pending_controls_.size();
             ++endpoint_index)
        {
            const auto& motor_indices =
                config_.endpoints[endpoint_index].motor_indices;
            const auto apply_frame = [&](const PendingControl& pending) {

                for (std::size_t slot = 0;
                     slot < pending.decoded.axis_count;
                     ++slot)
                {
                    const std::size_t motor = motor_indices[slot];
                    const AxisControl& command = pending.decoded.axes[slot];
                    const bool command_is_valid = valid_command(command);
                    const bool requests_enable =
                        (command.control_word & 0x008FU) == 0x000FU;
                    // Reject an invalid enable only for this motor. A stop
                    // on its peer must still reach the paired-stop logic.
                    if (requests_enable && !command_is_valid) continue;
                    const bool was_speed = modes_[motor] == kSpeedMode;
                    const bool was_homing = modes_[motor] == kHomingMode;
                    const Cia402State previous_state = drives_[motor].state();
                    if (command_is_valid)
                    {
                        commands_[motor] = command;
                        command_times_[motor] = pending.received_at;
                        command_valid_[motor] = true;
                    }
                    else
                    {
                        clear_command(motor);
                    }
                    modes_[motor] = command.mode;
                    drives_[motor].apply(command.control_word);
                    if (command.mode != kHomingMode || !was_homing)
                    {
                        homing_attained_[motor] = false;
                        homing_start_[motor] = false;
                        homing_hold_positions_[motor] = physical_positions[motor];
                    }
                    const bool start_homing = command.mode == kHomingMode &&
                        drives_[motor].operation_enabled() && (command.control_word & 0x10U);
                    if (start_homing && !homing_start_[motor])
                    {
                        zero_offsets_[motor] = physical_positions[motor];
                        homing_hold_positions_[motor] = physical_positions[motor];
                        homing_attained_[motor] = true;
                    }
                    homing_start_[motor] = start_homing;
                    updated[motor] = true;
                    if (drives_[motor].state() == Cia402State::QuickStopActive &&
                        previous_state != Cia402State::QuickStopActive)
                    {
                        quick_stop_initialized_[motor] = false;
                        quick_stop_elapsed_s_[motor] = 0.0;
                    }
                    else if (drives_[motor].state() != Cia402State::QuickStopActive)
                    {
                        quick_stop_initialized_[motor] = false;
                        quick_stop_elapsed_s_[motor] = 0.0;
                    }
                    if (drives_[motor].operation_enabled() && command_is_valid)
                    {
                        if (command.mode != kSpeedMode || !was_speed)
                            speed_integrals_[motor] = 0.0;
                    }
                    else
                    {
                        clear_command(motor);
                        speed_integrals_[motor] = 0.0;
                    }
                }
            };
            PendingControl pending;
            const double delay = config_.command_delay_s[motor_indices.front()];
            while (pending_controls_[endpoint_index].pop(pending))
            {
                bool bypass_delay = false;
                bool feedback_discontinuity = false;
                for (std::size_t slot = 0; slot < pending.decoded.axis_count; ++slot)
                {
                    const auto motor = motor_indices[slot];
                    const auto& cmd = pending.decoded.axes[slot];
                    bypass_delay = bypass_delay || !valid_command(cmd) ||
                        (cmd.control_word & 0x008FU) != 0x000FU ||
                        !drives_[motor].operation_enabled() || cmd.mode != modes_[motor] ||
                        cmd.mode == kHomingMode;
                    auto next_drive = drives_[motor];
                    next_drive.apply(cmd.control_word);
                    feedback_discontinuity = feedback_discontinuity ||
                        cmd.mode != modes_[motor] || next_drive.state() != drives_[motor].state() ||
                        (cmd.mode == kHomingMode && (cmd.control_word & 0x10U) &&
                         !homing_start_[motor]);
                }
                if (delay == 0.0 || bypass_delay)
                {
                    delayed_controls_[endpoint_index].clear();
                    if (feedback_discontinuity) delayed_feedback_[endpoint_index].clear();
                    apply_frame(pending);
                }
                else delayed_controls_[endpoint_index].push(
                    pending, pending.received_at + common::transport_delay(delay));
            }
            const auto now = std::chrono::steady_clock::now();
            while (delayed_controls_[endpoint_index].pop_ready(now, pending)) apply_frame(pending);
        }

        const auto release_or_fault =
            [&](std::initializer_list<std::size_t> axes) {
                bool should_release = false;
                for (const std::size_t axis : axes)
                {
                    should_release = should_release || updated[axis];
                    if (!drives_[axis].operation_enabled() ||
                        !supported_mode(modes_[axis]) ||
                        !command_valid_[axis])
                    {
                        return;
                    }
                }
                if (!should_release) return;
                std::string brake_error;
                if (startup_brake_.release_in_step(
                        model, data, axes, &brake_error))
                {
                    return;
                }
                if (config_.diagnostic_logging)
                {
                    std::fprintf(stderr, "[%s] Startup brake release failed: %s\n",
                                 config_.name.c_str(), brake_error.c_str());
                }
                for (const std::size_t axis : axes)
                {
                    drives_[axis].inject_fault();
                    clear_command(axis);
                    speed_integrals_[axis] = 0.0;
                }
            };
        release_or_fault({0});
        release_or_fault({1});
        release_or_fault({2});
        release_or_fault({3, 4});
        release_or_fault({5, 6});
    }

    void reset_control_state_locked() noexcept
    {
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            drives_[axis] = Cia402Drive{};
            modes_[axis] = 0;
            clear_command(axis);
        }
        for (auto& pending : pending_controls_) pending.clear();
        for (auto& queue : delayed_controls_) queue.clear();
        for (auto& queue : delayed_feedback_) queue.clear();
        speed_integrals_.fill(0.0);
        quick_stop_reference_velocity_.fill(0.0);
        quick_stop_elapsed_s_.fill(0.0);
        quick_stop_initialized_.fill(false);
        zero_offsets_.fill(0.0);
        homing_hold_positions_.fill(0.0);
        homing_attained_.fill(false);
        homing_start_.fill(false);
        feedback_min_version_ = 0;
    }

    ArmPluginConfig config_;
    ArmTransmission transmission_;
    Runtime* runtime_ = nullptr;
    std::uint64_t model_updater_id_ = 0;
    std::uint64_t reset_observer_id_ = 0;
    std::atomic<bool> running_{false};
    std::array<Endpoint, 5> endpoints_;
    std::array<Cia402Drive, kAxisCount> drives_;
    std::array<std::int8_t, kAxisCount> modes_{};
    std::array<AxisControl, kAxisCount> commands_{};
    std::array<std::chrono::steady_clock::time_point, kAxisCount> command_times_{};
    std::array<bool, kAxisCount> command_valid_{};
    std::array<PendingControlQueue, 5> pending_controls_{};
    std::array<common::DelayedQueue<PendingControl>, 5> delayed_controls_;
    std::array<common::DelayedQueue<DelayedFeedback>, 5> delayed_feedback_;
    std::uint64_t feedback_min_version_ = 0;
    ArmTransmission::AxisArray speed_integrals_{};
    ArmTransmission::AxisArray quick_stop_reference_velocity_{};
    ArmTransmission::AxisArray quick_stop_elapsed_s_{};
    std::array<bool, kAxisCount> quick_stop_initialized_{};
    ArmTransmission::AxisArray zero_offsets_{};
    ArmTransmission::AxisArray homing_hold_positions_{};
    std::array<bool, kAxisCount> homing_attained_{};
    std::array<bool, kAxisCount> homing_start_{};
    std::array<int, kAxisCount> qpos_{};
    std::array<int, kAxisCount> dof_{};
    std::array<int, kAxisCount> actuator_ids_{};
    std::array<std::string, kAxisCount> actuator_names_;
    std::array<std::string, kAxisCount> joint_names_;
    std::mutex mutex_;
    common::PeriodicTask feedback_;
    common::StartupBrake startup_brake_;
};

ArmPlugin::ArmPlugin(ArmPluginConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}
ArmPlugin::~ArmPlugin() = default;
std::string ArmPlugin::name() const { return impl_->name(); }
bool ArmPlugin::start(Runtime& runtime, std::string* error) { return impl_->start(runtime, error); }
void ArmPlugin::stop() noexcept { impl_->stop(); }
bool ArmPlugin::is_running() const noexcept { return impl_->running(); }

}  // namespace sharpa::mujoco::plugins::arm
