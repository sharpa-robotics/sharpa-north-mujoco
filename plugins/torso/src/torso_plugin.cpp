// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/torso/torso_plugin.h"

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"
#include "sharpa_mujoco/plugins/common/delayed_queue.h"
#include "sharpa_mujoco/plugins/common/periodic_task.h"
#include "sharpa_mujoco/plugins/common/socketcan_transport.h"
#include "sharpa_mujoco/plugins/common/startup_brake.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/plugins/torso/juxie_motor_emulator.h"
#include "sharpa_mujoco/plugins/torso/parallel_waist_kinematics.h"
#include "sharpa_mujoco/runtime.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

namespace sharpa::mujoco::plugins::torso
{
namespace
{

constexpr std::size_t kAxisCount = 8;
constexpr double kPi = 3.14159265358979323846;
constexpr double kCommandPositionChangeEpsilonRad = 1.0e-12;
constexpr double kMinimumCommandVelocityHoldSeconds = 0.06;
constexpr std::array<const char*, kAxisCount> kJointNames{
    "lower_body_joint_1",
    "lower_body_joint_2",
    "lower_body_joint_3",
    "lower_body_joint_4",
    "lower_body_joint_5",
    "neck_joint_1",
    "neck_joint_2",
    "neck_joint_3",
};

std::uint64_t timestamp_ns()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

bool fail(const std::string& message, std::string* error)
{
    if (error)
    {
        *error = message;
    }
    return false;
}

std::int16_t clamp_i16(long value)
{
    return static_cast<std::int16_t>(std::max<long>(
        std::numeric_limits<std::int16_t>::min(),
        std::min<long>(std::numeric_limits<std::int16_t>::max(), value)));
}

std::int16_t radians_to_count(double radians)
{
    return clamp_i16(std::lround(radians * 32768.0 / kPi));
}

double count_to_radians(std::int16_t count)
{
    return static_cast<double>(count) * kPi / 32768.0;
}

}  // namespace

class TorsoPlugin::Impl
{
public:
    explicit Impl(TorsoPluginConfig config)
        : config_(std::move(config)),
          motors_{
              JuxieMotorEmulator(config_.motor_ids[0]),
              JuxieMotorEmulator(config_.motor_ids[1]),
              JuxieMotorEmulator(config_.motor_ids[2]),
              JuxieMotorEmulator(config_.motor_ids[3]),
              JuxieMotorEmulator(config_.motor_ids[4]),
              JuxieMotorEmulator(config_.motor_ids[5]),
              JuxieMotorEmulator(config_.motor_ids[6]),
              JuxieMotorEmulator(config_.motor_ids[7])}
    {
    }

    ~Impl()
    {
        stop();
    }

    bool start(Runtime& runtime, std::string* error)
    {
        if (running_.load())
        {
            return true;
        }
        if (!runtime.is_loaded())
        {
            return fail("Torso plugin requires a loaded MuJoCo model.", error);
        }
        if (config_.feedback_rate_hz <= 0.0 || config_.heartbeat_rate_hz <= 0.0)
        {
            return fail("Torso feedback and heartbeat rates must be positive.", error);
        }
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            for (double delay : {config_.command_delay_s[axis], config_.feedback_delay_s[axis]})
                if (!std::isfinite(delay) || delay < 0.0 || delay > 0.1)
                    return fail("Torso transport delay must be finite and in [0, 0.1] seconds.", error);
        }
        if (config_.command_delay_s[2] != config_.command_delay_s[3] ||
            config_.feedback_delay_s[2] != config_.feedback_delay_s[3])
            return fail("Coupled waist motors require equal transport delays.", error);
        for (double ratio : config_.motor_reduction_ratios)
        {
            if (!std::isfinite(ratio) || ratio <= 0.0)
            {
                return fail(
                    "Torso motor reduction ratios must be finite and positive.",
                    error);
            }
        }
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            if (!std::isfinite(config_.feedback_velocity_scale[axis]) ||
                config_.feedback_velocity_scale[axis] <= 0.0)
                return fail("Torso velocity scales must be finite and positive.", error);
            if (!std::isfinite(config_.command_velocity_filter_s[axis]) ||
                config_.command_velocity_filter_s[axis] < 0.0 ||
                !std::isfinite(config_.command_velocity_scale[axis]) ||
                config_.command_velocity_scale[axis] < 0.0 ||
                !std::isfinite(config_.mit_kp_scale[axis]) ||
                config_.mit_kp_scale[axis] < 0.0 ||
                !std::isfinite(config_.mit_kd_scale[axis]) ||
                config_.mit_kd_scale[axis] < 0.0 ||
                !std::isfinite(config_.mit_torque_ff_scale[axis]) ||
                config_.mit_torque_ff_scale[axis] < 0.0 ||
                !std::isfinite(config_.mit_torque_limit_nm[axis]) ||
                config_.mit_torque_limit_nm[axis] <= 0.0 ||
                !std::isfinite(config_.mit_torque_time_constant_s[axis]) ||
                config_.mit_torque_time_constant_s[axis] < 0.0 ||
                !std::isfinite(
                    config_.mit_reference_velocity_limit_rad_s[axis]) ||
                config_.mit_reference_velocity_limit_rad_s[axis] < 0.0 ||
                !std::isfinite(
                    config_.mit_reference_acceleration_limit_rad_s2[axis]) ||
                config_.mit_reference_acceleration_limit_rad_s2[axis] < 0.0)
            {
                return fail(
                    "Torso control calibration parameters must be finite and "
                    "non-negative, and torque limits must be positive.",
                    error);
            }
            const bool has_reference_velocity_limit =
                config_.mit_reference_velocity_limit_rad_s[axis] > 0.0;
            const bool has_reference_acceleration_limit =
                config_.mit_reference_acceleration_limit_rad_s2[axis] > 0.0;
            if (has_reference_velocity_limit !=
                has_reference_acceleration_limit)
            {
                return fail(
                    "Torso MIT reference velocity and acceleration limits must "
                    "both be zero or both be positive.",
                    error);
            }
        }
        if (config_.dynamic_waist_armature &&
            (!std::isfinite(config_.waist_motor_inertia[0]) ||
             !std::isfinite(config_.waist_motor_inertia[1]) ||
             config_.waist_motor_inertia[0] < 0.0 ||
             config_.waist_motor_inertia[1] < 0.0))
        {
            return fail("Torso waist motor inertias must be finite and non-negative.", error);
        }
        if (!resolve_model(runtime.model_info(), error))
        {
            return false;
        }
        if (!startup_brake_.initialize(
                runtime,
                std::vector<std::string>(
                    kJointNames.begin(), kJointNames.end()),
                error))
        {
            return false;
        }
        const PoseSnapshot snapshot = runtime.latest_snapshot();
        if (!snapshot_has_mapping(snapshot))
        {
            return fail("Initial runtime snapshot does not contain torso state.", error);
        }
        std::array<bool, kAxisCount> mapped{};
        for (std::uint8_t motor_id : config_.motor_ids)
        {
            if (motor_id == 0 || motor_id > kAxisCount ||
                mapped[motor_id - 1])
            {
                return fail(
                    "Torso motor ID map must be a permutation of 1..8.",
                    error);
            }
            mapped[motor_id - 1] = true;
        }
        std::array<bool, kAxisCount> assigned{};
        for (std::size_t index = 0; index < endpoints_.size(); ++index)
        {
            EndpointRuntime& state = endpoints_[index];
            state.config = &config_.endpoints[index];
            const TorsoMcuEndpointConfig& endpoint = *state.config;
            if (endpoint.motor_ids.empty() ||
                (config_.transport == TorsoTransport::UdpMcu &&
                 (endpoint.bind_port == 0 || endpoint.peer_port == 0)) ||
                (config_.transport == TorsoTransport::SocketCan &&
                 config_.can_interfaces[index].empty()))
            {
                stop();
                return fail("Torso MCU endpoint configuration is invalid.", error);
            }
            for (std::uint8_t motor_id : endpoint.motor_ids)
            {
                if (motor_id == 0 || motor_id > kAxisCount ||
                    assigned[motor_id - 1])
                {
                    stop();
                    return fail(
                        "Torso motor IDs must be unique and in range 1..8.",
                        error);
                }
                assigned[motor_id - 1] = true;
            }
            const bool opened = config_.transport == TorsoTransport::UdpMcu
                ? state.udp_socket.open_and_bind(
                      {endpoint.bind_ip, endpoint.bind_port}, 50, error)
                : state.can_socket.open_and_bind(
                      config_.can_interfaces[index], 50, error);
            if (!opened)
            {
                stop();
                return false;
            }
        }
        if (std::find(assigned.begin(), assigned.end(), false) != assigned.end())
        {
            stop();
            return fail("Torso endpoints must assign all eight motors.", error);
        }

        runtime_ = &runtime;
        clear_delays_locked();
        if (std::any_of(config_.command_delay_s.begin(), config_.command_delay_s.end(), [](double d) { return d > 0; }) ||
            std::any_of(config_.feedback_delay_s.begin(), config_.feedback_delay_s.end(), [](double d) { return d > 0; }))
        {
            reset_observer_id_ = runtime.add_reset_observer([this](std::uint64_t) {
                std::lock_guard<std::mutex> lock(motor_mutex_);
                clear_delays_locked();
                feedback_min_version_ = runtime_->latest_snapshot().version + 1;
                for (auto& motor : motors_) motor.power_cycle();
                command_sample_initialized_.fill(false);
                estimated_command_velocity_.fill(0.0);
                filtered_mit_torque_.fill(0.0);
                mit_reference_initialized_.fill(false);
            });
            if (reset_observer_id_ == 0) { stop(); return fail("Failed to register torso delay reset observer.", error); }
        }
        model_updater_id_ = runtime.add_model_updater(
            [this](
                mjModel* model,
                mjData* data,
                const NativeStepContext& context) {
                update_control(model, data, context.timestep);
                update_waist_armature(model, data);
            });
        if (model_updater_id_ == 0)
        {
            runtime_ = nullptr;
            stop();
            return fail("Failed to register torso control updater.", error);
        }
        running_.store(true);
        for (std::size_t index = 0; index < endpoints_.size(); ++index)
        {
            endpoints_[index].receiver =
                std::thread([this, index]() { receive_loop(index); });
        }
        if (!feedback_task_.start(
                config_.feedback_rate_hz, [this]() { send_feedback(); }) ||
            !heartbeat_task_.start(
                config_.heartbeat_rate_hz, [this]() { send_heartbeats(); }))
        {
            stop();
            return fail("Failed to start Torso MCU publishers.", error);
        }
        return true;
    }

    void stop() noexcept
    {
        running_.store(false);
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
        feedback_task_.stop();
        heartbeat_task_.stop();
        for (auto& endpoint : endpoints_)
        {
            endpoint.udp_socket.close();
            endpoint.can_socket.close();
        }
        for (auto& endpoint : endpoints_)
        {
            if (endpoint.receiver.joinable())
            {
                endpoint.receiver.join();
            }
        }
        clear_delays_locked();
        runtime_ = nullptr;
    }

    bool is_running() const noexcept
    {
        return running_.load();
    }

private:
    struct EndpointRuntime
    {
        const TorsoMcuEndpointConfig* config = nullptr;
        common::UdpSocket udp_socket;
        common::SocketCanFdSocket can_socket;
        std::thread receiver;
        std::atomic<std::uint32_t> sequence{1};
    };

    bool resolve_model(const ModelInfo& model, std::string* error)
    {
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            const std::string joint_name = kJointNames[axis];
            const JointInfo* joint = nullptr;
            for (const JointInfo& candidate : model.joints)
            {
                if (candidate.name == joint_name)
                {
                    joint = &candidate;
                    break;
                }
            }
            if (!joint || joint->qpos_width != 1 || joint->dof_width != 1)
            {
                return fail("Missing scalar torso joint: " + joint_name, error);
            }
            const std::string actuator_name = joint_name + "_position";
            const ActuatorInfo* actuator = nullptr;
            for (const ActuatorInfo& candidate : model.actuators)
            {
                if (candidate.name == actuator_name)
                {
                    actuator = &candidate;
                    break;
                }
            }
            if (!actuator || actuator->transmission_id != joint->id)
            {
                return fail(
                    "Missing or incorrectly mapped torso actuator: " + actuator_name,
                    error);
            }
            qpos_addresses_[axis] = joint->qpos_address;
            dof_addresses_[axis] = joint->dof_address;
            actuator_ids_[axis] = actuator->id;
            actuator_names_[axis] = actuator_name;
        }
        return true;
    }

    bool snapshot_has_mapping(const PoseSnapshot& snapshot) const
    {
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            if (qpos_addresses_[axis] < 0 || dof_addresses_[axis] < 0 ||
                actuator_ids_[axis] < 0 ||
                static_cast<std::size_t>(qpos_addresses_[axis]) >= snapshot.qpos.size() ||
                static_cast<std::size_t>(dof_addresses_[axis]) >= snapshot.qvel.size() ||
                static_cast<std::size_t>(actuator_ids_[axis]) >=
                    snapshot.actuator_forces.size())
            {
                return false;
            }
        }
        return true;
    }

    void update_waist_armature(mjModel* model, const mjData* data)
    {
        if (!model || !data || !config_.dynamic_waist_armature)
        {
            return;
        }
        constexpr std::size_t kPitchAxis = 2;
        constexpr std::size_t kRollAxis = 3;
        const int pitch_qpos = qpos_addresses_[kPitchAxis];
        const int roll_qpos = qpos_addresses_[kRollAxis];
        const int pitch_dof = dof_addresses_[kPitchAxis];
        const int roll_dof = dof_addresses_[kRollAxis];
        if (pitch_qpos < 0 || roll_qpos < 0 || pitch_dof < 0 || roll_dof < 0 ||
            pitch_qpos >= model->nq || roll_qpos >= model->nq ||
            pitch_dof >= model->nv || roll_dof >= model->nv)
        {
            return;
        }
        std::array<double, 2> inertia{};
        if (waist_.reflected_inertia_diagonal(
                {data->qpos[pitch_qpos], data->qpos[roll_qpos]},
                config_.waist_motor_inertia,
                inertia))
        {
            model->dof_armature[pitch_dof] = inertia[0];
            model->dof_armature[roll_dof] = inertia[1];
        }
    }

    void update_control(
        const mjModel* model,
        mjData* data,
        double timestep_s)
    {
        if (!model || !data)
        {
            return;
        }
        std::array<double, kAxisCount> motor_position{};
        std::array<double, kAxisCount> motor_velocity{};
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            motor_position[axis] =
                config_.motor_signs[axis] *
                data->qpos[qpos_addresses_[axis]];
            motor_velocity[axis] =
                config_.motor_signs[axis] *
                data->qvel[dof_addresses_[axis]];
        }
        const std::array<double, 2> waist_joint{
            data->qpos[qpos_addresses_[2]],
            data->qpos[qpos_addresses_[3]],
        };
        std::array<double, 2> waist_motor{};
        std::array<std::array<double, 2>, 2> waist_jacobian{};
        const bool waist_mapping_valid =
            waist_.joint_to_motor(waist_joint, waist_motor) &&
            waist_.motor_jacobian(waist_joint, waist_jacobian);
        if (waist_mapping_valid)
        {
            motor_position[2] = waist_motor[0];
            motor_position[3] = waist_motor[1];
            const double pitch_velocity = data->qvel[dof_addresses_[2]];
            const double roll_velocity = data->qvel[dof_addresses_[3]];
            motor_velocity[2] =
                waist_jacobian[0][0] * pitch_velocity +
                waist_jacobian[0][1] * roll_velocity;
            motor_velocity[3] =
                waist_jacobian[1][0] * pitch_velocity +
                waist_jacobian[1][1] * roll_velocity;
        }

        std::array<double, kAxisCount> motor_torque{};
        const auto command_now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(motor_mutex_);
        drain_delayed_commands_locked(command_now);
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            decay_stale_command_velocity(axis, command_now, timestep_s);
            const JuxieMotorEmulator& motor = motors_[axis];
            if (!motor.enabled() || !motor.brake_released())
            {
                filtered_mit_torque_[axis] = 0.0;
                mit_reference_velocity_[axis] = 0.0;
                mit_reference_initialized_[axis] = false;
                estimated_command_velocity_[axis] = 0.0;
                continue;
            }
            if (motor.control_mode() == 0x03U)
            {
                filtered_mit_torque_[axis] = 0.0;
                mit_reference_velocity_[axis] = 0.0;
                mit_reference_initialized_[axis] = false;
                motor_torque[axis] =
                    config_.csp_kp[axis] *
                        (count_to_radians(motor.target_position()) -
                         motor_position[axis]) +
                    config_.csp_kd[axis] *
                        (config_.command_velocity_scale[axis] *
                             estimated_command_velocity_[axis] -
                         motor_velocity[axis]);
            }
            else if (motor.control_mode() == 0x06U)
            {
                const auto& command = motor.mit_command();
                const double reference_velocity_limit =
                    config_.mit_reference_velocity_limit_rad_s[axis];
                const double reference_acceleration_limit =
                    config_.mit_reference_acceleration_limit_rad_s2[axis];
                const bool reference_profile_enabled =
                    reference_velocity_limit > 0.0 &&
                    reference_acceleration_limit > 0.0;

                if (!mit_reference_initialized_[axis])
                {
                    mit_reference_position_[axis] = command.position_rad;
                    mit_reference_velocity_[axis] = 0.0;
                    mit_reference_initialized_[axis] = true;
                }
                else if (reference_profile_enabled && timestep_s > 0.0 &&
                         std::isfinite(timestep_s))
                {
                    const double error =
                        command.position_rad - mit_reference_position_[axis];
                    double position_correction_velocity = 0.0;
                    if (std::abs(error) > 1.0e-12)
                    {
                        position_correction_velocity = std::copysign(
                            std::min(
                                reference_velocity_limit,
                                std::sqrt(
                                    2.0 * reference_acceleration_limit *
                                    std::abs(error))),
                            error);
                    }
                    const double desired_velocity = std::clamp(
                        command.velocity_rad_s +
                            config_.command_velocity_scale[axis] *
                                estimated_command_velocity_[axis] +
                            position_correction_velocity,
                        -reference_velocity_limit,
                        reference_velocity_limit);
                    const double maximum_velocity_change =
                        reference_acceleration_limit * timestep_s;
                    mit_reference_velocity_[axis] += std::clamp(
                        desired_velocity - mit_reference_velocity_[axis],
                        -maximum_velocity_change,
                        maximum_velocity_change);
                    const double next_reference_position =
                        mit_reference_position_[axis] +
                        mit_reference_velocity_[axis] * timestep_s;
                    if (error == 0.0 ||
                        error *
                                (command.position_rad -
                                 next_reference_position) <=
                            0.0)
                    {
                        mit_reference_position_[axis] = command.position_rad;
                        mit_reference_velocity_[axis] = 0.0;
                    }
                    else
                    {
                        mit_reference_position_[axis] =
                            next_reference_position;
                    }
                }
                else if (!reference_profile_enabled)
                {
                    mit_reference_position_[axis] = command.position_rad;
                    mit_reference_velocity_[axis] = command.velocity_rad_s;
                }
                const double target_velocity =
                    reference_profile_enabled
                    ? mit_reference_velocity_[axis]
                    : command.velocity_rad_s +
                          config_.command_velocity_scale[axis] *
                              estimated_command_velocity_[axis];
                const double target_position =
                    reference_profile_enabled
                    ? mit_reference_position_[axis]
                    : command.position_rad;
                const double raw_torque =
                    config_.mit_kp_scale[axis] * command.kp_nm_per_rad *
                        (target_position - motor_position[axis]) +
                    config_.mit_kd_scale[axis] * command.kd_nm_s_per_rad *
                        (target_velocity - motor_velocity[axis]) +
                    config_.mit_torque_ff_scale[axis] * command.torque_ff_nm;
                if (std::isfinite(raw_torque))
                {
                    const double torque_limit =
                        config_.mit_torque_limit_nm[axis];
                    const double target_torque =
                        torque_limit * std::tanh(raw_torque / torque_limit);
                    const double time_constant =
                        config_.mit_torque_time_constant_s[axis];
                    if (time_constant > 0.0 && timestep_s > 0.0 &&
                        std::isfinite(timestep_s))
                    {
                        const double alpha =
                            -std::expm1(-timestep_s / time_constant);
                        filtered_mit_torque_[axis] +=
                            alpha *
                            (target_torque - filtered_mit_torque_[axis]);
                    }
                    else
                    {
                        filtered_mit_torque_[axis] = target_torque;
                    }
                    motor_torque[axis] = filtered_mit_torque_[axis];
                }
                else
                {
                    filtered_mit_torque_[axis] = 0.0;
                }
            }
            else
            {
                filtered_mit_torque_[axis] = 0.0;
                mit_reference_velocity_[axis] = 0.0;
                mit_reference_initialized_[axis] = false;
            }
            if (!std::isfinite(motor_torque[axis]))
            {
                filtered_mit_torque_[axis] = 0.0;
                mit_reference_velocity_[axis] = 0.0;
                mit_reference_initialized_[axis] = false;
                motor_torque[axis] = 0.0;
            }
        }

        std::array<double, kAxisCount> joint_torque{};
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            joint_torque[axis] = config_.motor_signs[axis] * motor_torque[axis];
        }
        if (!waist_mapping_valid ||
            !motors_[2].enabled() || !motors_[2].brake_released() ||
            !motors_[3].enabled() || !motors_[3].brake_released())
        {
            joint_torque[2] = 0.0;
            joint_torque[3] = 0.0;
        }
        else
        {
            std::array<double, 2> waist_torque{};
            if (waist_.motor_torque_to_joint_torque(
                    waist_joint,
                    {motor_torque[2], motor_torque[3]},
                    waist_torque))
            {
                joint_torque[2] = waist_torque[0];
                joint_torque[3] = waist_torque[1];
            }
            else
            {
                joint_torque[2] = 0.0;
                joint_torque[3] = 0.0;
            }
        }
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            if (actuator_ids_[axis] >= 0 && actuator_ids_[axis] < model->nu)
            {
                data->ctrl[actuator_ids_[axis]] = joint_torque[axis];
            }
        }
    }

    bool endpoint_has_motor(
        std::size_t endpoint_index,
        std::uint8_t motor_id) const
    {
        const auto& ids = config_.endpoints[endpoint_index].motor_ids;
        return std::find(ids.begin(), ids.end(), motor_id) != ids.end();
    }

    std::size_t logical_motor_index(std::uint8_t motor_id) const
    {
        const auto found = std::find(
            config_.motor_ids.begin(), config_.motor_ids.end(), motor_id);
        return found == config_.motor_ids.end()
            ? kAxisCount
            : static_cast<std::size_t>(
                  found - config_.motor_ids.begin());
    }

    void process_can_frame(
        std::size_t endpoint_index,
        const common::CanFdFrame& frame)
    {
        if (frame.can_id >= 0x601U && frame.can_id <= 0x608U)
        {
            const std::uint8_t motor_id =
                static_cast<std::uint8_t>(frame.can_id - 0x600U);
            const std::size_t motor_index = logical_motor_index(motor_id);
            if (!endpoint_has_motor(endpoint_index, motor_id) ||
                motor_index >= kAxisCount)
            {
                return;
            }
            std::optional<common::CanFdFrame> response;
            {
                std::lock_guard<std::mutex> lock(motor_mutex_);
                const auto before = motors_[motor_index];
                response = motors_[motor_index].process(frame);
                if (before.enabled() != motors_[motor_index].enabled() ||
                    before.brake_released() != motors_[motor_index].brake_released() ||
                    before.control_mode() != motors_[motor_index].control_mode())
                {
                    delayed_commands_[motor_index].clear();
                    delayed_feedback_[motor_index].clear();
                    command_ready_[motor_index] = false;
                    command_sample_initialized_[motor_index] = false;
                }
            }
            if (response) send_can(endpoint_index, *response);
        }
        else if (frame.can_id == 0x200U && frame.data.size() == 64)
        {
            apply_csp(endpoint_index, frame);
        }
        else if (frame.can_id == 0x210U && frame.data.size() == 64)
        {
            apply_mit(endpoint_index, frame);
        }
    }

    void receive_loop(std::size_t endpoint_index)
    {
        EndpointRuntime& endpoint = endpoints_[endpoint_index];
        std::array<std::uint8_t, 2048> buffer{};
        while (running_.load())
        {
            if (config_.transport == TorsoTransport::SocketCan)
            {
                common::CanFdFrame frame;
                if (endpoint.can_socket.receive(frame))
                {
                    std::optional<common::CanFdFrame> latest_control;
                    const auto consume = [&](common::CanFdFrame received) {
                        const bool is_control =
                            (received.can_id == 0x200U ||
                             received.can_id == 0x210U) &&
                            received.data.size() == 64;
                        if (is_control)
                        {
                            latest_control = std::move(received);
                            return;
                        }
                        if (latest_control)
                        {
                            process_can_frame(endpoint_index, *latest_control);
                            latest_control.reset();
                        }
                        process_can_frame(endpoint_index, received);
                    };
                    consume(std::move(frame));
                    while (endpoint.can_socket.try_receive(frame))
                    {
                        consume(std::move(frame));
                    }
                    if (latest_control)
                    {
                        process_can_frame(endpoint_index, *latest_control);
                    }
                }
                continue;
            }
            std::size_t received = 0;
            common::UdpEndpoint source;
            if (!endpoint.udp_socket.receive(
                    buffer.data(), buffer.size(), received, nullptr, &source))
            {
                continue;
            }
            if (source.ip != endpoint.config->peer_ip ||
                source.port != endpoint.config->peer_port)
            {
                continue;
            }
            common::McuEnvelope envelope;
            common::CanFdFrame frame;
            if (!common::decode_mcu_downlink(buffer.data(), received, envelope) ||
                envelope.fragment_count != 1 ||
                envelope.source_code != common::make_source_code(
                    common::kMcuDomainCommand, common::kMcuClassMotor) ||
                !common::decode_can_fd_payload(
                    envelope.payload.data(), envelope.payload.size(), frame) ||
                frame.can_interface != endpoint.config->can_interface)
            {
                continue;
            }
            process_can_frame(endpoint_index, frame);
        }
    }

    bool joint_to_motor_positions(
        const PoseSnapshot& snapshot,
        std::array<double, kAxisCount>& motor_positions) const
    {
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            motor_positions[axis] =
                config_.motor_signs[axis] *
                snapshot.qpos[static_cast<std::size_t>(qpos_addresses_[axis])];
        }
        std::array<double, 2> waist_motors{};
        if (!waist_.joint_to_motor(
                {
                    snapshot.qpos[static_cast<std::size_t>(qpos_addresses_[2])],
                    snapshot.qpos[static_cast<std::size_t>(qpos_addresses_[3])],
                },
                waist_motors))
        {
            return false;
        }
        motor_positions[2] = waist_motors[0];
        motor_positions[3] = waist_motors[1];
        return true;
    }

    void update_command_velocity(
        std::size_t axis,
        double position_rad,
        std::chrono::steady_clock::time_point received_at)
    {
        const double filter_time = config_.command_velocity_filter_s[axis];
        if (!command_sample_initialized_[axis])
        {
            previous_command_position_[axis] = position_rad;
            previous_command_time_[axis] = received_at;
            estimated_command_velocity_[axis] = 0.0;
            command_sample_initialized_[axis] = true;
            return;
        }

        const double position_change =
            position_rad - previous_command_position_[axis];
        if (filter_time > 0.0 &&
            std::abs(position_change) <= kCommandPositionChangeEpsilonRad)
        {
            // Platform republishes the current motor target at the CAN servo
            // rate.  Those identical frames are holds, not new trajectory
            // samples.  Including them would turn each lower-rate position
            // update into a velocity spike followed by repeated decay.
            return;
        }

        const double elapsed_s = std::chrono::duration<double>(
            received_at - previous_command_time_[axis]).count();
        previous_command_time_[axis] = received_at;
        if (filter_time <= 0.0 || elapsed_s <= 1.0e-6 || elapsed_s > 0.25)
        {
            previous_command_position_[axis] = position_rad;
            estimated_command_velocity_[axis] = 0.0;
            return;
        }
        const double measured_velocity = std::clamp(
            position_change / elapsed_s,
            -3.14,
            3.14);
        previous_command_position_[axis] = position_rad;
        const double alpha = -std::expm1(-elapsed_s / filter_time);
        estimated_command_velocity_[axis] +=
            alpha *
            (measured_velocity - estimated_command_velocity_[axis]);
    }

    void decay_stale_command_velocity(
        std::size_t axis,
        std::chrono::steady_clock::time_point now,
        double timestep_s)
    {
        const double filter_time = config_.command_velocity_filter_s[axis];
        if (filter_time <= 0.0 || !command_sample_initialized_[axis] ||
            timestep_s <= 0.0 || !std::isfinite(timestep_s))
        {
            return;
        }
        const double age_s = std::chrono::duration<double>(
            now - previous_command_time_[axis]).count();
        const double hold_seconds =
            std::max(kMinimumCommandVelocityHoldSeconds, 2.0 * filter_time);
        if (age_s <= hold_seconds)
        {
            return;
        }
        const double alpha = -std::expm1(-timestep_s / filter_time);
        estimated_command_velocity_[axis] +=
            alpha * (0.0 - estimated_command_velocity_[axis]);
        if (std::abs(estimated_command_velocity_[axis]) < 1.0e-6)
        {
            estimated_command_velocity_[axis] = 0.0;
        }
    }

    void apply_csp(
        std::size_t endpoint_index,
        const common::CanFdFrame& frame)
    {
        const auto received_at = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            for (std::size_t slot = 0; slot < 8; ++slot)
            {
                const std::uint8_t motor_id = frame.data[56 + slot];
                if (motor_id == 0 || motor_id > 8 ||
                    !endpoint_has_motor(endpoint_index, motor_id))
                {
                    continue;
                }
                const std::size_t motor_index = logical_motor_index(motor_id);
                if (motor_index >= kAxisCount)
                {
                    continue;
                }
                if (config_.command_delay_s[motor_index] > 0.0)
                {
                    queue_delayed_slot(motor_index, frame.data.data() + slot * 7, 7, false, received_at);
                    continue;
                }
                command_ready_[motor_index] = motors_[motor_index].apply_csp_slot(
                    frame.data.data() + slot * 7, 7);
                if (command_ready_[motor_index])
                {
                    update_command_velocity(
                        motor_index,
                        count_to_radians(
                            motors_[motor_index].target_position()),
                        received_at);
                }
            }
        }
        release_ready_brakes();
    }

    void apply_mit(
        std::size_t endpoint_index,
        const common::CanFdFrame& frame)
    {
        const auto received_at = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            for (std::size_t slot = 0; slot < 6; ++slot)
            {
                const std::uint8_t motor_id = frame.data[56 + slot];
                if (motor_id == 0 || motor_id > 8 ||
                    !endpoint_has_motor(endpoint_index, motor_id))
                {
                    continue;
                }
                const std::size_t motor_index = logical_motor_index(motor_id);
                if (motor_index >= kAxisCount)
                {
                    continue;
                }
                if (config_.command_delay_s[motor_index] > 0.0)
                {
                    queue_delayed_slot(motor_index, frame.data.data() + slot * 9, 9, true, received_at);
                    continue;
                }
                command_ready_[motor_index] = motors_[motor_index].apply_mit_slot(
                    frame.data.data() + slot * 9, 9);
                if (command_ready_[motor_index])
                {
                    update_command_velocity(
                        motor_index,
                        motors_[motor_index].mit_command().position_rad,
                        received_at);
                }
            }
        }
        release_ready_brakes();
    }

    struct DelayedSlot
    {
        std::array<std::uint8_t, 9> bytes{};
        bool mit = false;
        std::chrono::steady_clock::time_point delivered_at{};
    };

    struct FeedbackSample
    {
        std::int16_t position = 0;
        double velocity = 0;
        std::int16_t torque = 0;
        std::int32_t encoder = 0;
    };

    void clear_delays_locked()
    {
        for (auto& queue : delayed_commands_) queue.clear();
        for (auto& queue : delayed_feedback_) queue.clear();
        command_ready_.fill(false);
        feedback_min_version_ = 0;
    }

    void queue_delayed_slot(std::size_t axis, const std::uint8_t* bytes,
                            std::size_t size, bool mit,
                            std::chrono::steady_clock::time_point received_at)
    {
        auto validator = motors_[axis];
        const bool valid = mit ? validator.apply_mit_slot(bytes, size)
                               : validator.apply_csp_slot(bytes, size);
        const bool transition = validator.enabled() != motors_[axis].enabled() ||
            validator.brake_released() != motors_[axis].brake_released() ||
            validator.control_mode() != motors_[axis].control_mode();
        // Disable, brake and mode transitions must not wait behind motion.
        // Invalid mode slots still carry stop bits in the existing protocol.
        if (transition || !validator.enabled() || !validator.brake_released() ||
            !valid || !command_ready_[axis])
        {
            delayed_commands_[axis].clear();
            if (transition) delayed_feedback_[axis].clear();
            motors_[axis] = validator;
            command_ready_[axis] = valid;
            command_sample_initialized_[axis] = false;
            if (valid)
                update_command_velocity(axis, mit ? validator.mit_command().position_rad
                    : count_to_radians(validator.target_position()), received_at);
            return;
        }
        DelayedSlot slot;
        std::copy_n(bytes, size, slot.bytes.begin());
        slot.mit = mit;
        slot.delivered_at = received_at + common::transport_delay(config_.command_delay_s[axis]);
        delayed_commands_[axis].push(slot, slot.delivered_at);
    }

    void drain_delayed_commands_locked(std::chrono::steady_clock::time_point now)
    {
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            if (config_.command_delay_s[axis] == 0.0) continue;
            DelayedSlot slot;
            while (delayed_commands_[axis].pop_ready(now, slot))
            {
                auto& motor = motors_[axis];
                const bool applied = slot.mit ? motor.apply_mit_slot(slot.bytes.data(), 9)
                                             : motor.apply_csp_slot(slot.bytes.data(), 7);
                if (applied)
                {
                    command_ready_[axis] = true;
                    update_command_velocity(axis, slot.mit ? motor.mit_command().position_rad
                        : count_to_radians(motor.target_position()), slot.delivered_at);
                }
            }
        }
    }

    void release_ready_brakes()
    {
        std::array<bool, kAxisCount> ready{};
        {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            for (std::size_t axis = 0; axis < kAxisCount; ++axis)
            {
                // CSP and MIT can be active on different motors at once.
                // An incoming frame must not re-lock motors in the other mode.
                const auto mode = motors_[axis].control_mode();
                ready[axis] =
                    motors_[axis].enabled() &&
                    motors_[axis].brake_released() &&
                    (mode == 0x03U || mode == 0x06U) &&
                    command_ready_[axis];
            }
        }
        if (ready[0]) startup_brake_.release(0);
        else startup_brake_.engage(0);
        if (ready[1]) startup_brake_.release(1);
        else startup_brake_.engage(1);
        if (ready[2] && ready[3]) startup_brake_.release({2, 3});
        else startup_brake_.engage({2, 3});
        for (std::size_t axis = 4; axis < kAxisCount; ++axis)
        {
            if (ready[axis]) startup_brake_.release(axis);
            else startup_brake_.engage(axis);
        }
    }

    void send_can(std::size_t endpoint_index, const common::CanFdFrame& frame)
    {
        EndpointRuntime& endpoint = endpoints_[endpoint_index];
        if (config_.transport == TorsoTransport::SocketCan)
        {
            endpoint.can_socket.send(frame);
            return;
        }
        common::McuEnvelope envelope;
        envelope.magic = common::kMcuMagicPoc2;
        envelope.source_code = common::make_source_code(
            common::kMcuDomainSensor, common::kMcuClassMotor);
        envelope.sequence = endpoint.sequence.fetch_add(1);
        envelope.timestamp_ns = timestamp_ns();
        envelope.gptp_ns = envelope.timestamp_ns;
        envelope.payload = common::encode_can_fd_payload(frame);
        const std::vector<std::uint8_t> datagram =
            common::encode_mcu_uplink(envelope);
        endpoint.udp_socket.send_to(
            {endpoint.config->peer_ip, endpoint.config->peer_port},
            datagram.data(),
            datagram.size());
    }

    void send_feedback()
    {
        Runtime* const runtime = runtime_;
        if (!running_.load() || !runtime)
        {
            return;
        }
        const PoseSnapshot snapshot = runtime->latest_snapshot();
        if (!snapshot_has_mapping(snapshot))
        {
            return;
        }
        std::array<double, kAxisCount> motor_positions{};
        if (!joint_to_motor_positions(snapshot, motor_positions))
        {
            return;
        }
        std::array<double, kAxisCount> motor_velocity{};
        std::array<double, kAxisCount> motor_torque{};
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            motor_velocity[axis] =
                config_.motor_signs[axis] *
                snapshot.qvel[static_cast<std::size_t>(dof_addresses_[axis])];
            motor_torque[axis] =
                config_.motor_signs[axis] *
                snapshot.actuator_forces[
                    static_cast<std::size_t>(actuator_ids_[axis])];
        }
        const std::array<double, 2> waist_joint{
            snapshot.qpos[static_cast<std::size_t>(qpos_addresses_[2])],
            snapshot.qpos[static_cast<std::size_t>(qpos_addresses_[3])],
        };
        std::array<std::array<double, 2>, 2> waist_jacobian{};
        if (waist_.motor_jacobian(waist_joint, waist_jacobian))
        {
            const double pitch_velocity =
                snapshot.qvel[static_cast<std::size_t>(dof_addresses_[2])];
            const double roll_velocity =
                snapshot.qvel[static_cast<std::size_t>(dof_addresses_[3])];
            motor_velocity[2] =
                waist_jacobian[0][0] * pitch_velocity +
                waist_jacobian[0][1] * roll_velocity;
            motor_velocity[3] =
                waist_jacobian[1][0] * pitch_velocity +
                waist_jacobian[1][1] * roll_velocity;
        }
        std::array<double, 2> waist_motor_torque{};
        if (waist_.joint_torque_to_motor_torque(
                waist_joint,
                {
                    snapshot.actuator_forces[
                        static_cast<std::size_t>(actuator_ids_[2])],
                    snapshot.actuator_forces[
                        static_cast<std::size_t>(actuator_ids_[3])],
                },
                waist_motor_torque))
        {
            motor_torque[2] = waist_motor_torque[0];
            motor_torque[3] = waist_motor_torque[1];
        }

        const auto sampled_at = std::chrono::steady_clock::now();
        std::array<bool, kAxisCount> ready{};
        std::lock_guard<std::mutex> lock(motor_mutex_);
        if (snapshot.version < feedback_min_version_) return;
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            FeedbackSample sample{
                radians_to_count(motor_positions[axis]),
                config_.motor_reduction_ratios[axis] *
                    config_.feedback_velocity_scale[axis] *
                    motor_velocity[axis],
                clamp_i16(std::lround(motor_torque[axis] / 0.05)),
                static_cast<std::int32_t>(std::lround(
                    std::clamp(motor_positions[axis] *
                        config_.motor_reduction_ratios[axis] *
                        65536.0 / (2.0 * kPi), -8388608.0, 8388607.0)))};
            if (config_.feedback_delay_s[axis] > 0.0)
            {
                delayed_feedback_[axis].push(sample, sampled_at + common::transport_delay(config_.feedback_delay_s[axis]));
                FeedbackSample released;
                while (delayed_feedback_[axis].pop_ready(sampled_at, released))
                {
                    sample = released;
                    ready[axis] = true;
                }
                if (!ready[axis]) continue;
            }
            else ready[axis] = true;
            motors_[axis].set_feedback(sample.position, sample.velocity, sample.torque, sample.encoder);
        }
        for (std::size_t endpoint = 0; endpoint < endpoints_.size(); ++endpoint)
        {
            for (std::uint8_t motor_id : config_.endpoints[endpoint].motor_ids)
            {
                const std::size_t motor_index = logical_motor_index(motor_id);
                if (motor_index >= kAxisCount)
                {
                    continue;
                }
                if (!ready[motor_index]) continue;
                send_can(
                    endpoint,
                    motors_[motor_index].feedback_frame(
                        config_.endpoints[endpoint].can_interface));
            }
        }
    }

    void send_heartbeats()
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        for (std::size_t endpoint = 0; endpoint < endpoints_.size(); ++endpoint)
        {
            for (std::uint8_t motor_id : config_.endpoints[endpoint].motor_ids)
            {
                const std::size_t motor_index = logical_motor_index(motor_id);
                if (motor_index >= kAxisCount)
                {
                    continue;
                }
                send_can(
                    endpoint,
                    motors_[motor_index].heartbeat_frame(
                        config_.endpoints[endpoint].can_interface));
            }
        }
    }

    TorsoPluginConfig config_;
    Runtime* runtime_ = nullptr;
    std::uint64_t model_updater_id_ = 0;
    std::uint64_t reset_observer_id_ = 0;
    std::array<common::DelayedQueue<DelayedSlot>, kAxisCount> delayed_commands_;
    std::array<common::DelayedQueue<FeedbackSample>, kAxisCount> delayed_feedback_;
    // A command must match its frame format before its motor can release a brake.
    // Keep this per-axis for both immediate and delayed CSP/MIT commands.
    std::array<bool, kAxisCount> command_ready_{};
    std::uint64_t feedback_min_version_ = 0;
    std::array<int, kAxisCount> qpos_addresses_{};
    std::array<int, kAxisCount> dof_addresses_{};
    std::array<int, kAxisCount> actuator_ids_{};
    std::array<std::string, kAxisCount> actuator_names_{};
    std::array<JuxieMotorEmulator, kAxisCount> motors_;
    std::array<double, kAxisCount> filtered_mit_torque_{};
    std::array<double, kAxisCount> mit_reference_position_{};
    std::array<double, kAxisCount> mit_reference_velocity_{};
    std::array<bool, kAxisCount> mit_reference_initialized_{};
    std::array<double, kAxisCount> previous_command_position_{};
    std::array<double, kAxisCount> estimated_command_velocity_{};
    std::array<std::chrono::steady_clock::time_point, kAxisCount>
        previous_command_time_{};
    std::array<bool, kAxisCount> command_sample_initialized_{};
    std::array<EndpointRuntime, 2> endpoints_;
    ParallelWaistKinematics waist_;
    common::PeriodicTask feedback_task_;
    common::PeriodicTask heartbeat_task_;
    common::StartupBrake startup_brake_;
    std::mutex motor_mutex_;
    std::atomic<bool> running_{false};
};

TorsoPlugin::TorsoPlugin(TorsoPluginConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

TorsoPlugin::~TorsoPlugin() = default;

std::string TorsoPlugin::name() const
{
    return "torso";
}

bool TorsoPlugin::start(Runtime& runtime, std::string* error)
{
    return impl_->start(runtime, error);
}

void TorsoPlugin::stop() noexcept
{
    impl_->stop();
}

bool TorsoPlugin::is_running() const noexcept
{
    return impl_->is_running();
}

}  // namespace sharpa::mujoco::plugins::torso
