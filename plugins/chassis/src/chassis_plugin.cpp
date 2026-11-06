// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/chassis/chassis_plugin.h"

#include "sharpa_mujoco/plugins/chassis/jiechang_motor_emulator.h"
#include "sharpa_mujoco/plugins/chassis/jiechang_stretch_emulator.h"
#include "sharpa_mujoco/plugins/chassis/kinco_motor_emulator.h"
#include "sharpa_mujoco/plugins/chassis/m42p_motor_emulator.h"
#include "sharpa_mujoco/plugins/chassis/wheel_velocity_pi.h"
#include "sharpa_mujoco/plugins/chassis/steer_position_profile.h"
#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"
#include "sharpa_mujoco/plugins/common/periodic_task.h"
#include "sharpa_mujoco/plugins/common/startup_brake.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/runtime.h"

#include <mujoco/mujoco.h>

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

namespace sharpa::mujoco::plugins::chassis
{
namespace
{

constexpr std::size_t kJiechangAxisCount = 8;
constexpr std::size_t kSliderCount = 4;
constexpr std::size_t kAxisCount = 12;
constexpr double kTwoPi = 6.28318530717958647692;

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

std::int32_t clamp_i32(double value)
{
    const double minimum =
        static_cast<double>(std::numeric_limits<std::int32_t>::min());
    const double maximum =
        static_cast<double>(std::numeric_limits<std::int32_t>::max());
    return static_cast<std::int32_t>(
        std::llround(std::max(minimum, std::min(maximum, value))));
}

std::int16_t clamp_i16(double value)
{
    const double minimum =
        static_cast<double>(std::numeric_limits<std::int16_t>::min());
    const double maximum =
        static_cast<double>(std::numeric_limits<std::int16_t>::max());
    return static_cast<std::int16_t>(
        std::llround(std::max(minimum, std::min(maximum, value))));
}

enum class AxisControlMode
{
    Unsupported,
    Position,
    Velocity,
    Torque,
    Mit,
};

struct AxisCommand
{
    std::size_t axis = 0;
    AxisControlMode mode = AxisControlMode::Unsupported;
    bool enabled = false;
    bool brake_released = false;
    double target_position = 0.0;
    double target_velocity = 0.0;
    double target_torque = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    std::chrono::steady_clock::time_point received_at{};
};

struct AxisFeedback
{
    std::size_t axis = 0;
    double position = 0.0;
    double velocity = 0.0;
    double joint_torque = 0.0;
};

}  // namespace

class ChassisPlugin::Impl
{
public:
    explicit Impl(ChassisPluginConfig config)
        : config_(std::move(config)),
          jiechang_{
              JiechangMotorEmulator(1), JiechangMotorEmulator(2),
              JiechangMotorEmulator(3), JiechangMotorEmulator(4),
              JiechangMotorEmulator(5), JiechangMotorEmulator(6),
              JiechangMotorEmulator(7), JiechangMotorEmulator(8)},
          kinco_{
              KincoMotorEmulator(1), KincoMotorEmulator(2),
              KincoMotorEmulator(3), KincoMotorEmulator(4),
              KincoMotorEmulator(5), KincoMotorEmulator(6),
              KincoMotorEmulator(7), KincoMotorEmulator(8)},
          m42p_{
              M42pMotorEmulator(1), M42pMotorEmulator(2),
              M42pMotorEmulator(3), M42pMotorEmulator(4)},
          stretch_{
              JiechangStretchEmulator(9), JiechangStretchEmulator(10),
              JiechangStretchEmulator(11), JiechangStretchEmulator(12)}
    {
    }

    ~Impl() { stop(); }

    bool start(Runtime& runtime, std::string* error)
    {
        if (running_.load())
        {
            return true;
        }
        if (!runtime.is_loaded())
        {
            return fail("Chassis plugin requires a loaded MuJoCo model.", error);
        }
        const std::array<double, 14> positive_values{
            config_.jiechang_stretch_counts_per_meter,
            config_.feedback_rate_hz,
            config_.command_timeout_s,
            config_.slider_steps_per_meter,
            config_.steer_gear_ratio,
            config_.wheel_gear_ratio,
            config_.steer_rated_current_a,
            config_.wheel_rated_current_a,
            config_.steer_rated_motor_torque_nm,
            config_.wheel_rated_motor_torque_nm,
            config_.steer_position_kp,
            config_.wheel_velocity_kv,
            config_.jiechang_steer_torque_limit_nm,
            config_.jiechang_wheel_torque_limit_nm};
        if (std::any_of(
                positive_values.begin(), positive_values.end(),
                [](double value) {
                    return !std::isfinite(value) || value <= 0.0;
                }) ||
            !std::isfinite(config_.steer_position_kd) ||
            config_.steer_position_kd < 0.0 ||
            !std::isfinite(config_.slider_minimum_m) ||
            !std::isfinite(config_.slider_maximum_m) ||
            config_.slider_minimum_m > config_.slider_maximum_m)
        {
            return fail(
                "Chassis conversion, rating, limit, and control parameters are invalid.",
                error);
        }
        if (!resolve_model(runtime.model_info(), error))
        {
            return false;
        }
        const std::array<double, 10> nonnegative_pi_values{
            config_.kinco_steer_position_kp,
            config_.kinco_steer_position_kd,
            config_.kinco_steer_profile_acceleration_scale,
            config_.kinco_steer_command_delay_s,
            config_.kinco_steer_velocity_feedforward,
            config_.kinco_wheel_velocity_kp,
            config_.wheel_velocity_ki,
            config_.wheel_velocity_integral_limit_nm,
            config_.kinco_wheel_velocity_ki,
            config_.kinco_wheel_velocity_integral_limit_nm};
        if (std::any_of(nonnegative_pi_values.begin(), nonnegative_pi_values.end(),
                        [](double v) { return !std::isfinite(v) || v < 0.0; }) ||
            config_.kinco_steer_command_delay_s > 1.0 ||
            config_.kinco_encoder_resolution == 0 ||
            !std::isfinite(config_.kinco_steer_gear_ratio) ||
            config_.kinco_steer_gear_ratio <= 0.0 ||
            (config_.wheel_velocity_ki > 0.0 && config_.wheel_velocity_integral_limit_nm <= 0.0) ||
            (config_.kinco_wheel_velocity_ki > 0.0 && config_.kinco_wheel_velocity_integral_limit_nm <= 0.0))
        {
            return fail("Chassis controller gains/limits must be finite and nonnegative; Kinco conversion scales and active PI integral limits must be positive; steer delay must be <= 1 s.", error);
        }
        // Anti-windup must respect the model's physical torque limits too,
        // not just the emulated drive's (possibly higher) peak rating.
        wheel_torque_limits_.fill(std::numeric_limits<double>::infinity());
        if (!runtime.read_native([&](const mjModel* model, const mjData*) {
                for (std::size_t wheel = 0; wheel < 4; ++wheel)
                {
                    const int actuator = actuator_ids_[4 + wheel];
                    const int joint = mj_name2id(model, mjOBJ_JOINT, config_.axes[4 + wheel].joint.c_str());
                    auto clamp_limit = [&](const mjtNum* range) {
                        wheel_torque_limits_[wheel] = std::min(
                            wheel_torque_limits_[wheel], std::min(std::abs(range[0]), std::abs(range[1])));
                    };
                    if (model->actuator_forcelimited[actuator])
                        clamp_limit(model->actuator_forcerange + 2 * actuator);
                    if (model->jnt_actfrclimited[joint])
                        clamp_limit(model->jnt_actfrcrange + 2 * joint);
                }
            }, error)) return false;
        for (auto& controller : wheel_pi_) controller.reset();
        for (auto& profile : steer_profiles_) profile.reset();
        wheel_pi_reset_generation_ = runtime.reset_generation();
        std::vector<std::string> brake_joints;
        brake_joints.reserve(12);
        for (std::size_t axis = 0; axis < 4; ++axis)
        {
            brake_joints.push_back(config_.axes[axis].joint);
        }
        for (std::size_t axis = kJiechangAxisCount; axis < kAxisCount; ++axis)
        {
            brake_joints.push_back(config_.axes[axis].joint);
        }
        // Keep existing steer/slider indices, append the four precise-model wheel brakes.
        for (std::size_t wheel = 0; wheel < 4; ++wheel)
            brake_joints.push_back(config_.axes[4 + wheel].joint);
        if (!startup_brake_.initialize(runtime, brake_joints, error))
        {
            return false;
        }
        if (!snapshot_has_mapping(runtime.latest_snapshot()))
        {
            return fail("Initial runtime snapshot lacks chassis state.", error);
        }
        if (!validate_coordinate_contract(runtime, error))
        {
            return false;
        }
        const ChassisUdpEndpointConfig& motor_endpoint =
            config_.motor_family == ChassisMotorFamily::Kinco
                ? config_.kinco_endpoint
                : config_.jiechang_endpoint;
        if (!open_endpoint(
                jiechang_endpoint_, motor_endpoint, error) ||
            (config_.stretch_family == ChassisStretchFamily::M42p &&
             !open_endpoint(m42p_endpoint_, config_.m42p_endpoint, error)))
        {
            stop();
            return false;
        }
        const std::int32_t minimum =
            clamp_i32(config_.slider_minimum_m * config_.slider_steps_per_meter);
        const std::int32_t maximum =
            clamp_i32(config_.slider_maximum_m * config_.slider_steps_per_meter);
        for (auto& motor : m42p_)
        {
            motor.set_limits(minimum, maximum);
        }

        // Seed each emulated motor from the loaded model once. During runtime,
        // the emulator position is the commanded mechanism state and must not
        // be overwritten by qpos: doing so would turn any externally induced
        // slider displacement into the next position target, allowing drift.
        const PoseSnapshot initial_snapshot = runtime.latest_snapshot();
        for (std::size_t slider = 0; slider < kSliderCount; ++slider)
        {
            const std::size_t axis = kJiechangAxisCount + slider;
            const double physical =
                config_.axes[axis].sign *
                initial_snapshot.qpos[
                    static_cast<std::size_t>(qpos_addresses_[axis])];
            stretch_reference_[slider] = physical;
            m42p_[slider].set_position_steps(
                clamp_i32(
                    (config_.slider_maximum_m - physical) *
                    config_.slider_steps_per_meter));
        }

        runtime_ = &runtime;
        if (config_.motor_family == ChassisMotorFamily::Kinco)
        {
            kinco_updater_id_ = runtime.add_model_updater(
                [this](mjModel* model, mjData* data, const NativeStepContext&) {
                    submit_kinco_command(model, data);
                });
            if (kinco_updater_id_ == 0)
            {
                stop();
                return fail("Failed to register Kinco control updater.", error);
            }
        }
        running_.store(true);
        jiechang_endpoint_.receiver = std::thread([this]() {
            if (config_.motor_family == ChassisMotorFamily::Kinco)
            {
                kinco_receive_loop();
            }
            else
            {
                jiechang_receive_loop();
            }
        });
        if (config_.stretch_family == ChassisStretchFamily::M42p)
        {
            m42p_endpoint_.receiver =
                std::thread([this]() { m42p_receive_loop(); });
        }
        if (!feedback_task_.start(
                config_.feedback_rate_hz, [this]() { feedback_tick(); }))
        {
            stop();
            return fail("Failed to start chassis feedback publisher.", error);
        }
        return true;
    }

    void stop() noexcept
    {
        running_.store(false);
        if (runtime_ && kinco_updater_id_ != 0)
        {
            runtime_->remove_model_updater(kinco_updater_id_);
            kinco_updater_id_ = 0;
        }
        feedback_task_.stop();
        jiechang_endpoint_.socket.close();
        m42p_endpoint_.socket.close();
        if (jiechang_endpoint_.receiver.joinable())
        {
            jiechang_endpoint_.receiver.join();
        }
        if (m42p_endpoint_.receiver.joinable())
        {
            m42p_endpoint_.receiver.join();
        }
        runtime_ = nullptr;
    }

    bool is_running() const noexcept { return running_.load(); }

private:
    struct EndpointRuntime
    {
        const ChassisUdpEndpointConfig* config = nullptr;
        common::UdpSocket socket;
        std::thread receiver;
        std::atomic<std::uint32_t> sequence{1};
    };

    bool open_endpoint(
        EndpointRuntime& runtime,
        const ChassisUdpEndpointConfig& config,
        std::string* error)
    {
        if (config.bind_port == 0 || config.peer_port == 0)
        {
            return fail("Chassis UDP endpoint ports must be nonzero.", error);
        }
        runtime.config = &config;
        return runtime.socket.open_and_bind(
            {config.bind_ip, config.bind_port}, 50, error);
    }

    bool resolve_model(const ModelInfo& model, std::string* error)
    {
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            const ChassisAxisConfig& mapping = config_.axes[axis];
            const auto joint = std::find_if(
                model.joints.begin(), model.joints.end(),
                [&](const JointInfo& candidate) {
                    return candidate.name == mapping.joint;
                });
            if (joint == model.joints.end() ||
                joint->qpos_width != 1 || joint->dof_width != 1)
            {
                return fail(
                    "Missing scalar chassis joint: " + mapping.joint, error);
            }
            const auto actuator = std::find_if(
                model.actuators.begin(), model.actuators.end(),
                [&](const ActuatorInfo& candidate) {
                    return candidate.name == mapping.actuator;
                });
            if (actuator == model.actuators.end() ||
                actuator->transmission_id != joint->id)
            {
                return fail(
                    "Missing or incorrectly mapped chassis actuator: " +
                        mapping.actuator,
                    error);
            }
            qpos_addresses_[axis] = joint->qpos_address;
            dof_addresses_[axis] = joint->dof_address;
            actuator_ids_[axis] = actuator->id;
        }
        return true;
    }

    bool snapshot_has_mapping(const PoseSnapshot& snapshot) const
    {
        for (std::size_t axis = 0; axis < kAxisCount; ++axis)
        {
            if (qpos_addresses_[axis] < 0 || dof_addresses_[axis] < 0 ||
                actuator_ids_[axis] < 0 ||
                static_cast<std::size_t>(qpos_addresses_[axis]) >=
                    snapshot.qpos.size() ||
                static_cast<std::size_t>(dof_addresses_[axis]) >=
                    snapshot.qvel.size() ||
                static_cast<std::size_t>(actuator_ids_[axis]) >=
                    snapshot.actuator_forces.size())
            {
                return false;
            }
        }
        return true;
    }

    bool validate_coordinate_contract(Runtime& runtime, std::string* error)
    {
        bool valid = true;
        std::string validation_error;
        std::string native_error;
        if (!runtime.read_native(
                [&](const mjModel* model, const mjData*) {
                    std::unique_ptr<mjData, decltype(&mj_deleteData)> neutral_data(
                        mj_makeData(model), &mj_deleteData);
                    if (!neutral_data)
                    {
                        valid = false;
                        validation_error =
                            "Could not allocate neutral MuJoCo data.";
                        return;
                    }
                    mj_forward(model, neutral_data.get());

                    std::array<int, kAxisCount> body_ids{};
                    for (std::size_t axis = 0; axis < kAxisCount; ++axis)
                    {
                        const int joint_id = model->actuator_trnid[
                            2 * actuator_ids_[axis]];
                        body_ids[axis] = model->jnt_bodyid[joint_id];
                    }

                    // The configured FLU contract belongs to the chassis, not
                    // to the MuJoCo world. SceneBuilder may place the robot at
                    // an arbitrary world pose, so use the lowest common
                    // non-world ancestor of all configured chassis joints as
                    // the reference frame.
                    int reference_body_id = body_ids.front();
                    while (reference_body_id > 0)
                    {
                        bool common_to_all = true;
                        for (const int body_id : body_ids)
                        {
                            int ancestor = body_id;
                            int hops = 0;
                            while (ancestor != reference_body_id &&
                                   ancestor > 0 && hops++ < model->nbody)
                            {
                                ancestor = model->body_parentid[ancestor];
                            }
                            if (ancestor != reference_body_id)
                            {
                                common_to_all = false;
                                break;
                            }
                        }
                        if (common_to_all)
                        {
                            break;
                        }
                        reference_body_id =
                            model->body_parentid[reference_body_id];
                    }
                    if (reference_body_id <= 0)
                    {
                        valid = false;
                        validation_error =
                            "Configured chassis joints do not share a "
                            "non-world reference body.";
                        return;
                    }

                    const mjtNum* reference_rotation =
                        neutral_data->xmat + 9 * reference_body_id;
                    constexpr double tolerance = 2e-5;
                    const auto check_axis = [&](std::size_t axis,
                                                const std::array<double, 3>& expected) {
                        const int joint_id = model->actuator_trnid[
                            2 * actuator_ids_[axis]];
                        const int body_id = body_ids[axis];
                        const mjtNum* matrix = neutral_data->xmat + 9 * body_id;
                        const mjtNum* local = model->jnt_axis + 3 * joint_id;
                        std::array<double, 3> world{};
                        mju_mulMatVec(world.data(), matrix, local, 3, 3);
                        std::array<double, 3> chassis{};
                        for (std::size_t component = 0; component < 3; ++component)
                        {
                            chassis[component] =
                                reference_rotation[component] * world[0] +
                                reference_rotation[3 + component] * world[1] +
                                reference_rotation[6 + component] * world[2];
                            if (!std::isfinite(chassis[component]) ||
                                std::abs(chassis[component] - expected[component]) >
                                    tolerance)
                            {
                                valid = false;
                            }
                        }
                        if (!valid && validation_error.empty())
                        {
                            validation_error =
                                "Joint '" + config_.axes[axis].joint +
                                "' axis in chassis frame is [" +
                                std::to_string(chassis[0]) + ", " +
                                std::to_string(chassis[1]) + ", " +
                                std::to_string(chassis[2]) + "]; expected [" +
                                std::to_string(expected[0]) + ", " +
                                std::to_string(expected[1]) + ", " +
                                std::to_string(expected[2]) + "].";
                        }
                    };
                    for (std::size_t axis = 0; axis < 4; ++axis)
                    {
                        check_axis(axis, {0.0, 0.0, -1.0});
                        check_axis(axis + 4, {0.0, 1.0, 0.0});
                    }
                    for (std::size_t axis = 8; axis < 12; ++axis)
                    {
                        constexpr std::array<double, 4> slider_world_y{
                            1.0, -1.0, -1.0, 1.0};
                        check_axis(
                            axis,
                            {0.0, slider_world_y[axis - 8], 0.0});
                    }
                },
                &native_error))
        {
            return fail(
                "Failed to validate chassis coordinate contract: " + native_error,
                error);
        }
        return valid ||
               fail(
                   "Chassis joint axes violate the configured FLU coordinate contract. " +
                       validation_error,
                   error);
    }

    bool receive_datagram(
        EndpointRuntime& endpoint,
        std::array<std::uint8_t, 2048>& buffer,
        std::size_t& received,
        common::McuEnvelope& envelope)
    {
        common::UdpEndpoint source;
        if (!endpoint.socket.receive(
                buffer.data(), buffer.size(), received, nullptr, &source) ||
            source.ip != endpoint.config->peer_ip ||
            source.port != endpoint.config->peer_port ||
            !common::decode_mcu_downlink(
                buffer.data(), received, envelope) ||
            envelope.fragment_count != 1 ||
            envelope.magic != common::kMcuMagicPoc2)
        {
            return false;
        }
        return true;
    }

    void jiechang_receive_loop()
    {
        std::array<std::uint8_t, 2048> buffer{};
        while (running_.load())
        {
            std::size_t received = 0;
            common::McuEnvelope envelope;
            common::CanFdFrame frame;
            if (!receive_datagram(
                    jiechang_endpoint_, buffer, received, envelope) ||
                !common::decode_can_fd_payload(
                    envelope.payload.data(), envelope.payload.size(), frame))
            {
                continue;
            }
            bool known_interface = false;
            for (const auto can_interface : config_.motor_can_interfaces)
            {
                known_interface =
                    known_interface || frame.can_interface == can_interface;
            }
            if (!known_interface)
            {
                continue;
            }
            process_jiechang_frame(frame);
            process_stretch_frame(frame);
        }
    }

    void kinco_receive_loop()
    {
        std::array<std::uint8_t, 2048> buffer{};
        while (running_.load())
        {
            std::size_t received = 0;
            common::McuEnvelope envelope;
            common::CanFdFrame frame;
            if (!receive_datagram(
                    jiechang_endpoint_, buffer, received, envelope) ||
                !common::decode_can_fd_payload(
                    envelope.payload.data(), envelope.payload.size(), frame))
            {
                continue;
            }
            process_kinco_frame(frame);
            process_stretch_frame(frame);
        }
    }

    void process_kinco_frame(const common::CanFdFrame& frame)
    {
        std::vector<common::CanFdFrame> responses;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (std::size_t axis = 0; axis < kinco_.size(); ++axis)
            {
                if (config_.motor_can_interfaces[axis] != frame.can_interface)
                {
                    continue;
                }
                if (auto response = kinco_[axis].process(frame))
                {
                    responses.push_back(std::move(*response));
                }
                if ((frame.can_id == 0x201U + axis || frame.can_id == 0x301U + axis) &&
                    !frame.data.empty())
                {
                    last_kinco_commands_[axis] = std::chrono::steady_clock::now();
                }
            }
        }
        for (const auto& response : responses)
        {
            send_can(response);
        }
        submit_kinco_command();
    }

    static double velocity_dec_to_rpm(
        std::int32_t value,
        std::uint32_t resolution)
    {
        return static_cast<double>(value) * 1875.0 /
               (512.0 * static_cast<double>(resolution));
    }

    static std::int32_t rpm_to_velocity_dec(
        double rpm,
        std::uint32_t resolution)
    {
        return clamp_i32(
            rpm * 512.0 * static_cast<double>(resolution) / 1875.0);
    }

    static double torque_speed_limit(
        double output_rpm,
        double peak_torque,
        double continuous_torque,
        double peak_speed,
        double rated_speed)
    {
        const double speed = std::abs(output_rpm);
        if (speed <= peak_speed)
        {
            return peak_torque;
        }
        if (speed >= rated_speed)
        {
            return continuous_torque;
        }
        const double fraction =
            (speed - peak_speed) / (rated_speed - peak_speed);
        return peak_torque +
               fraction * (continuous_torque - peak_torque);
    }

    void submit_kinco_command(const mjModel* model = nullptr, mjData* data = nullptr)
    {
        if (data == nullptr)
        {
            // Brake changes can call mj_forward. Keep them outside the physics
            // updater and outside the controller lock.
            std::array<bool, 8> release{};
            {
                std::lock_guard<std::mutex> control_lock(wheel_control_mutex_);
                std::lock_guard<std::mutex> lock(mutex_);
                for (std::size_t axis = 0; axis < 8; ++axis)
                {
                    const bool active = kinco_[axis].enabled() &&
                        (kinco_[axis].homing_active() ||
                         std::chrono::duration<double>(std::chrono::steady_clock::now() -
                            last_kinco_commands_[axis]).count() <= config_.command_timeout_s);
                    release[axis] = active;
                    if (axis < 4)
                    {
                        if (!active || (kinco_[axis].mode() != 1 && !kinco_[axis].homing_active()))
                            steer_profiles_[axis].reset();
                    }
                    else if (!active || kinco_[axis].mode() != -3)
                        wheel_pi_[axis - 4].reset();
                }
            }
            std::vector<std::size_t> released_axes;
            for (std::size_t axis = 0; axis < 8; ++axis)
                if (release[axis]) released_axes.push_back(axis < 4 ? axis : axis + 4);
            if (!released_axes.empty()) startup_brake_.release(released_axes);
            if (config_.kinco_control_in_step) return;
        }
        std::unique_lock<std::mutex> control_lock(wheel_control_mutex_);
        Runtime* const runtime = runtime_;
        if (!runtime)
        {
            return;
        }
        const PoseSnapshot snapshot = data ? PoseSnapshot{} : runtime->latest_snapshot();
        if (!data && !snapshot_has_mapping(snapshot))
        {
            return;
        }
        synchronize_wheel_pi_reset(*runtime);
        ControlCommand command;
        command.mode = ControlMode::Actuator;
        command.replace_all = false;
        command.opens_action_frame = false;
        command.named_values.reserve(kJiechangAxisCount);
        std::unique_lock<std::mutex> lock(mutex_);
        for (std::size_t axis = 0; axis < kJiechangAxisCount; ++axis)
        {
            const bool homing = kinco_[axis].homing_active();
            // Homing is an autonomous drive operation started by SDO, without
            // periodic RPDO targets. Ordinary motion keeps its RPDO watchdog.
            const bool fresh = homing || std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_kinco_commands_[axis]).count()
                <= config_.command_timeout_s;
            const bool steer = axis < 4;
            const double sign = config_.axes[axis].sign;
            const double gear_ratio =
                steer ? config_.kinco_steer_gear_ratio :
                        config_.kinco_wheel_gear_ratio;
            const double position = sign * (data ? data->qpos[qpos_addresses_[axis]] :
                snapshot.qpos[static_cast<std::size_t>(qpos_addresses_[axis])]);
            const double velocity = sign * (data ? data->qvel[dof_addresses_[axis]] :
                snapshot.qvel[static_cast<std::size_t>(dof_addresses_[axis])]);
            const double simulation_time = data ? data->time : snapshot.simulation_time;
            const KincoMotorEmulator& motor = kinco_[axis];
            // Homing must settle at standstill even with slow CAN feedback.
            // Evaluate its damping at every physics step, not from delayed
            // snapshots; retain the configured path for ordinary drive modes.
            if (data && !config_.kinco_control_in_step && !homing) continue;
            if (!data && homing)
            {
                continue;
            }
            if (!steer && !(fresh && motor.enabled() && motor.mode() == -3))
                wheel_pi_[axis - 4].reset();
            if (steer && !(fresh && motor.enabled() && (motor.mode() == 1 || homing)))
                steer_profiles_[axis].reset();
            double torque = 0.0;
            if (fresh && motor.enabled())
            {
                if (steer && (motor.mode() == 1 || homing))
                {
                    const double target = homing ? 0.0 :
                        static_cast<double>(motor.target_position()) *
                        kTwoPi /
                        (static_cast<double>(config_.kinco_encoder_resolution) *
                         gear_ratio);
                    SteerReference reference{target, 0.0};
                    const auto profile_velocity = homing ? motor.homing_velocity() : motor.profile_velocity();
                    const auto profile_acceleration = homing ? motor.homing_acceleration() : motor.profile_acceleration();
                    const auto profile_deceleration = homing ? motor.homing_acceleration() : motor.profile_deceleration();
                    const double acceleration_scale = homing ? 1.0 : config_.kinco_steer_profile_acceleration_scale;
                    if (acceleration_scale > 0.0 && profile_velocity > 0 &&
                        profile_acceleration > 0 && profile_deceleration > 0)
                    {
                        const double unit = 1875.0 * kTwoPi /
                            (512.0 * config_.kinco_encoder_resolution * 60.0 * gear_ratio);
                        const double acceleration = kinco_profile_ramp_rad_s2(
                            profile_acceleration, config_.kinco_encoder_resolution,
                            gear_ratio) * acceleration_scale;
                        const double deceleration = kinco_profile_ramp_rad_s2(
                            profile_deceleration, config_.kinco_encoder_resolution,
                            gear_ratio) * acceleration_scale;
                        reference = steer_profiles_[axis].update(
                            target, position, simulation_time,
                            unit * profile_velocity,
                            acceleration, deceleration,
                            homing ? 0.0 : config_.kinco_steer_command_delay_s, config_.command_timeout_s);
                    }
                    else
                    {
                        steer_profiles_[axis].reset();
                    }
                    torque =
                        config_.kinco_steer_position_kp *
                            (reference.position - position) +
                        config_.kinco_steer_position_kd *
                            (config_.kinco_steer_velocity_feedforward * reference.velocity - velocity);
                    if (homing && model)
                    {
                        // Account for one step of velocity/position response
                        // in the PD command. Explicit high damping can chatter
                        // on the low-inertia steering joint even at 500 Hz.
                        const double inertia = data->qM[model->dof_Madr[dof_addresses_[axis]]];
                        const double dt = model->opt.timestep;
                        torque /= 1.0 + (config_.kinco_steer_position_kd * dt +
                            config_.kinco_steer_position_kp * dt * dt) /
                            std::max(inertia, 1e-9);
                    }
                }
                else if (!steer && motor.mode() == -3)
                {
                    const double motor_rpm = velocity_dec_to_rpm(
                        motor.target_velocity(),
                        config_.kinco_encoder_resolution);
                    const double target =
                        motor_rpm * kTwoPi / (60.0 * gear_ratio);
                    const double limit = torque_speed_limit(
                        velocity * 60.0 / kTwoPi,
                        config_.kinco_wheel_peak_torque_nm,
                        config_.kinco_wheel_continuous_torque_nm,
                        config_.kinco_wheel_peak_speed_rpm,
                        config_.kinco_wheel_rated_speed_rpm);
                    const WheelVelocityPiParams params{
                        config_.kinco_wheel_velocity_kp,
                        config_.kinco_wheel_velocity_ki,
                        config_.kinco_wheel_velocity_integral_limit_nm,
                        std::min(limit, wheel_torque_limits_[axis - 4]),
                        config_.command_timeout_s};
                    torque = wheel_pi_[axis - 4].update(
                        target, velocity, simulation_time, params);
                }
                else if (motor.mode() == 4)
                {
                    // Platform steer uses 0x6510:03 peak Ap / sqrt(2);
                    // wheel uses configured max_current_ma (25 Ap) / sqrt(2).
                    const double current_arms = steer
                        ? static_cast<double>(motor.target_current()) *
                              config_.kinco_steer_max_current_arms /
                              2048.0
                        : static_cast<double>(motor.target_current()) *
                              (config_.kinco_wheel_max_current_arms / std::sqrt(2.0)) /
                              2048.0;
                    const double kt =
                        steer ? config_.kinco_steer_output_kt_nm_per_a :
                                config_.kinco_wheel_output_kt_nm_per_a;
                    torque = current_arms * kt;
                }
                const double output_rpm =
                    velocity * 60.0 / kTwoPi;
                const double limit = steer
                    ? torque_speed_limit(
                          output_rpm,
                          config_.kinco_steer_peak_torque_nm,
                          config_.kinco_steer_continuous_torque_nm,
                          config_.kinco_steer_peak_speed_rpm,
                          config_.kinco_steer_rated_speed_rpm)
                    : torque_speed_limit(
                          output_rpm,
                          config_.kinco_wheel_peak_torque_nm,
                          config_.kinco_wheel_continuous_torque_nm,
                          config_.kinco_wheel_peak_speed_rpm,
                          config_.kinco_wheel_rated_speed_rpm);
                torque = std::clamp(torque, -limit, limit);
            }
            if (data)
                data->ctrl[actuator_ids_[axis]] = sign * torque;
            else
            {
                command.named_values.push_back(
                    {config_.axes[axis].actuator, sign * torque});
            }
        }
        lock.unlock();
        control_lock.unlock();
        if (!data) runtime->submit_control(command);
    }

    void process_stretch_frame(const common::CanFdFrame& frame)
    {
        if (config_.stretch_family != ChassisStretchFamily::Jiechang) return;
        std::vector<common::CanFdFrame> replies;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (std::size_t slider = 0; slider < kSliderCount; ++slider)
            {
                if (frame.can_interface != config_.stretch_can_interfaces[slider]) continue;
                if (auto reply = stretch_[slider].process(frame)) replies.push_back(*reply);
                // Only that motor's RPDO traffic refreshes its watchdog. SYNC,
                // SDO polling and other nodes must not keep stale motion alive.
                if ((frame.can_id == 0x209U + slider && frame.data.size() == 6) ||
                    (frame.can_id == 0x309U + slider && frame.data.size() == 4))
                {
                    last_stretch_commands_[slider] = std::chrono::steady_clock::now();
                }
            }
        }
        for (const auto& reply : replies) send_can(reply);
    }

    void update_jiechang_stretch(const PoseSnapshot& snapshot, double period)
    {
        ControlCommand command;
        command.mode = ControlMode::Actuator;
        command.replace_all = false;
        command.opens_action_frame = false;
        std::vector<common::CanFdFrame> frames;
        std::array<bool, kSliderCount> release{};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (std::size_t slider = 0; slider < kSliderCount; ++slider)
            {
                const std::size_t axis = kJiechangAxisCount + slider;
                auto& motor = stretch_[slider];
                const double sign = config_.axes[axis].sign;
                const double position = sign * snapshot.qpos[qpos_addresses_[axis]];
                const double velocity = sign * snapshot.qvel[dof_addresses_[axis]];
                const double counts = config_.jiechang_stretch_counts_per_meter;
                // F = 2*pi*tau/lead; gear ratio 1, lead 1 mm, rated tau 0.060 Nm.
                const double torque = sign * snapshot.actuator_forces[actuator_ids_[axis]] * 0.001 / kTwoPi;
                const auto permille = clamp_i16(torque / 0.060 * 1000.0);
                motor.set_feedback(clamp_i32(position * counts), clamp_i32(velocity * counts), permille, permille);
                const auto feedback = motor.feedback(config_.stretch_can_interfaces[slider]);
                frames.insert(frames.end(), feedback.begin(), feedback.end());
                const bool fresh = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - last_stretch_commands_[slider]).count()
                    <= motor.command_timeout_s();
                if (motor.enabled() && fresh)
                {
                    release[slider] = true;
                    const double max_step = motor.maximum_velocity() / counts * period;
                    const double step = motor.mode() == 9
                        ? motor.target_velocity() / counts * period
                        : motor.target_position() / counts - stretch_reference_[slider];
                    stretch_reference_[slider] += std::clamp(step, -max_step, max_step);
                }
                // Disabled/stale commands hold the last reference. Reseeding
                // it from qpos every tick would allow external loads to drift.
                stretch_reference_[slider] = std::clamp(stretch_reference_[slider],
                    config_.slider_minimum_m, config_.slider_maximum_m);
                command.named_values.push_back({config_.axes[axis].actuator,
                    sign * stretch_reference_[slider]});
            }
        }
        runtime_->submit_control(command);
        for (std::size_t slider = 0; slider < kSliderCount; ++slider)
        {
            if (release[slider]) startup_brake_.release(4 + slider);
        }
        for (const auto& frame : frames) send_can(frame);
    }

    void process_jiechang_frame(const common::CanFdFrame& frame)
    {
        if (frame.can_id >= 0x601U && frame.can_id <= 0x608U)
        {
            const std::size_t axis = frame.can_id - 0x601U;
            if (config_.motor_can_interfaces[axis] != frame.can_interface)
            {
                return;
            }
            std::optional<common::CanFdFrame> response;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                response = jiechang_[axis].process_sdo(frame);
            }
            if (response)
            {
                send_can(*response);
            }
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (frame.can_id == 0x200U && frame.data.size() == 48)
            {
                for (std::size_t slot = 0; slot < kJiechangAxisCount; ++slot)
                {
                    if (config_.motor_can_interfaces[slot] ==
                        frame.can_interface)
                    {
                        jiechang_[slot].apply_pdo1_slot(
                            frame.data.data() + slot * 6, 6);
                    }
                }
            }
            else if (frame.can_id == 0x300U && frame.data.size() == 64)
            {
                for (std::size_t slot = 0; slot < kJiechangAxisCount; ++slot)
                {
                    if (config_.motor_can_interfaces[slot] ==
                        frame.can_interface)
                    {
                        jiechang_[slot].apply_pdo2_slot(
                            frame.data.data() + slot * 8, 8);
                        // Platform flushes the left/right CAN groups separately.
                        // Traffic on one bus must not keep the other bus alive.
                        last_jiechang_commands_[slot] =
                            std::chrono::steady_clock::now();
                    }
                }
            }
            else if (frame.can_id == 0x400U && frame.data.size() == 64)
            {
                for (std::size_t slot = 0; slot < kJiechangAxisCount; ++slot)
                {
                    if (config_.motor_can_interfaces[slot] ==
                        frame.can_interface)
                    {
                        jiechang_[slot].apply_pdo3_slot(
                            frame.data.data() + slot * 8, 8);
                    }
                }
            }
        }
        if (frame.can_id == 0x300U && frame.data.size() == 64)
        {
            submit_jiechang_command();
        }
        else if (frame.can_id == 0x080U && frame.data.empty())
        {
            send_jiechang_feedback(frame.can_interface);
        }
    }

    void m42p_receive_loop()
    {
        std::array<std::uint8_t, 2048> buffer{};
        while (running_.load())
        {
            std::size_t received = 0;
            common::McuEnvelope envelope;
            common::ModbusRtuFramePayload payload;
            if (!receive_datagram(
                    m42p_endpoint_, buffer, received, envelope) ||
                !common::decode_modbus_rtu_payload(
                    envelope.payload.data(), envelope.payload.size(), payload) ||
                payload.data.empty())
            {
                continue;
            }
            std::optional<std::vector<std::uint8_t>> response;
            std::optional<std::size_t> released_slider;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                for (std::size_t axis = 0; axis < kSliderCount; ++axis)
                {
                    if (config_.m42p_rs485_interfaces[axis] ==
                            payload.rs485_interface &&
                        m42p_[axis].slave() == payload.data[0])
                    {
                        response = m42p_[axis].process(
                            payload.data.data(), payload.data.size());
                        if (response && m42p_[axis].moving())
                        {
                            released_slider = axis;
                        }
                        break;
                    }
                }
            }
            if (released_slider)
            {
                startup_brake_.release(4 + *released_slider);
            }
            if (response &&
                (payload.flags & common::kModbusRtuFlagExpectReply) != 0)
            {
                common::ModbusRtuFramePayload reply;
                reply.rs485_interface = payload.rs485_interface;
                reply.flags = payload.flags;
                reply.data = std::move(*response);
                send_modbus(reply);
            }
        }
    }

    void submit_jiechang_command()
    {
        std::lock_guard<std::mutex> control_lock(wheel_control_mutex_);
        Runtime* const runtime = runtime_;
        if (!runtime)
        {
            return;
        }
        ControlCommand command;
        command.mode = ControlMode::Actuator;
        command.replace_all = false;
        command.opens_action_frame = false;
        command.named_values.reserve(kJiechangAxisCount);
        const PoseSnapshot snapshot = runtime->latest_snapshot();
        if (!snapshot_has_mapping(snapshot))
        {
            return;
        }
        synchronize_wheel_pi_reset(*runtime);
        std::array<AxisCommand, kJiechangAxisCount> axis_commands;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (std::size_t axis = 0; axis < kJiechangAxisCount; ++axis)
            {
                const bool steer = axis < 4;
                AxisCommand& axis_command = axis_commands[axis];
                axis_command.axis = axis;
                axis_command.enabled = jiechang_[axis].enabled();
                axis_command.brake_released =
                    jiechang_[axis].brake_released();
                axis_command.received_at = last_jiechang_commands_[axis];
                switch (jiechang_[axis].mode())
                {
                    case 8:
                        axis_command.mode =
                            steer ? AxisControlMode::Position :
                                    AxisControlMode::Unsupported;
                        break;
                    case 9:
                        axis_command.mode =
                            steer ? AxisControlMode::Unsupported :
                                    AxisControlMode::Velocity;
                        break;
                    case 10:
                        axis_command.mode = AxisControlMode::Torque;
                        break;
                    case -1:
                        axis_command.mode = AxisControlMode::Mit;
                        break;
                    default:
                        axis_command.mode = AxisControlMode::Unsupported;
                        break;
                }
                const double gear_ratio =
                    steer ? config_.steer_gear_ratio :
                            config_.wheel_gear_ratio;
                const double motor_radians =
                    static_cast<double>(
                        jiechang_[axis].target_position_count()) *
                    kTwoPi /
                    JiechangMotorEmulator::position_counts_per_revolution();
                const double motor_rpm =
                    static_cast<double>(
                        jiechang_[axis].target_velocity_count()) /
                    JiechangMotorEmulator::velocity_counts_per_rpm();
                const double rated_torque =
                    steer ? config_.steer_rated_motor_torque_nm :
                            config_.wheel_rated_motor_torque_nm;
                axis_command.target_position =
                    config_.axes[axis].sign * motor_radians / gear_ratio;
                axis_command.target_velocity =
                    config_.axes[axis].sign * motor_rpm * kTwoPi /
                    (60.0 * gear_ratio);
                axis_command.target_torque =
                    config_.axes[axis].sign * rated_torque * gear_ratio *
                    static_cast<double>(
                        jiechang_[axis].target_torque_permille()) /
                    1000.0;
                axis_command.kp =
                    config_.jiechang_mit_kp_scale *
                    static_cast<double>(jiechang_[axis].mit_kp());
                axis_command.kd =
                    config_.jiechang_mit_kd_scale *
                    static_cast<double>(jiechang_[axis].mit_kd());
            }
        }
        for (std::size_t axis = 0; axis < kJiechangAxisCount; ++axis)
        {
            double value = 0.0;
            const AxisCommand& axis_command = axis_commands[axis];
            const bool fresh =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() -
                    axis_command.received_at)
                    .count() <= config_.command_timeout_s;
            if (axis >= 4 && !(axis_command.enabled && axis_command.brake_released && fresh &&
                               axis_command.mode == AxisControlMode::Velocity))
                wheel_pi_[axis - 4].reset();
            if (axis_command.enabled && axis_command.brake_released && fresh)
            {
                const double position = snapshot.qpos[
                    static_cast<std::size_t>(qpos_addresses_[axis])];
                const double velocity = snapshot.qvel[
                    static_cast<std::size_t>(dof_addresses_[axis])];
                switch (axis_command.mode)
                {
                    case AxisControlMode::Position:
                        value =
                            config_.steer_position_kp *
                                (axis_command.target_position - position) -
                            config_.steer_position_kd * velocity;
                        break;
                    case AxisControlMode::Velocity:
                    {
                        const WheelVelocityPiParams params{
                            config_.wheel_velocity_kv,
                            config_.wheel_velocity_ki,
                            config_.wheel_velocity_integral_limit_nm,
                            std::min(config_.jiechang_wheel_torque_limit_nm, wheel_torque_limits_[axis - 4]),
                            config_.command_timeout_s};
                        value = wheel_pi_[axis - 4].update(
                            axis_command.target_velocity, velocity, snapshot.simulation_time, params);
                        break;
                    }
                    case AxisControlMode::Torque:
                        value = axis_command.target_torque;
                        break;
                    case AxisControlMode::Mit:
                        value =
                            axis_command.kp *
                                (axis_command.target_position - position) +
                            axis_command.kd *
                                (axis_command.target_velocity - velocity) +
                            axis_command.target_torque;
                        break;
                    case AxisControlMode::Unsupported:
                        break;
                }
                const double limit =
                    axis < 4 ? config_.jiechang_steer_torque_limit_nm :
                               config_.jiechang_wheel_torque_limit_nm;
                value = std::clamp(value, -limit, limit);
            }
            command.named_values.push_back(
                {config_.axes[axis].actuator, value});
        }
        if (!runtime->submit_control(command))
        {
            return;
        }
        std::vector<std::size_t> released_axes;
        for (std::size_t axis = 0; axis < kJiechangAxisCount; ++axis)
        {
            const AxisCommand& axis_command = axis_commands[axis];
            const bool fresh =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - axis_command.received_at)
                    .count() <= config_.command_timeout_s;
            if (fresh && axis_command.enabled && axis_command.brake_released &&
                (axis_command.mode == AxisControlMode::Position ||
                 axis_command.mode == AxisControlMode::Torque ||
                 axis_command.mode == AxisControlMode::Mit ||
                 axis_command.mode == AxisControlMode::Velocity))
            {
                released_axes.push_back(axis < 4 ? axis : axis + 4);
            }
        }
        if (!released_axes.empty()) startup_brake_.release(released_axes);
    }

    void submit_slider_command()
    {
        Runtime* const runtime = runtime_;
        if (!runtime)
        {
            return;
        }
        ControlCommand command;
        command.mode = ControlMode::Actuator;
        command.replace_all = false;
        command.opens_action_frame = false;
        command.named_values.reserve(kSliderCount);
        std::unique_lock<std::mutex> lock(mutex_);
        for (std::size_t slider = 0; slider < kSliderCount; ++slider)
        {
            const std::size_t axis = kJiechangAxisCount + slider;
            // M42P installation is inverted: negative motor motion extends the
            // mechanism, therefore decreasing steps maps to increasing q.
            const double physical =
                config_.slider_maximum_m -
                static_cast<double>(m42p_[slider].position_steps()) /
                    config_.slider_steps_per_meter;
            command.named_values.push_back(
                {config_.axes[axis].actuator,
                 config_.axes[axis].sign * physical});
        }
        lock.unlock();
        runtime->submit_control(command);
    }

    void feedback_tick()
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
        const double period = 1.0 / config_.feedback_rate_hz;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (std::size_t slider = 0; slider < kSliderCount; ++slider)
            {
                m42p_[slider].advance(period);
            }
            for (std::size_t axis = 0; axis < kJiechangAxisCount; ++axis)
            {
                const AxisFeedback feedback{
                    axis,
                    config_.axes[axis].sign *
                        snapshot.qpos[
                            static_cast<std::size_t>(qpos_addresses_[axis])],
                    config_.axes[axis].sign *
                        snapshot.qvel[
                            static_cast<std::size_t>(dof_addresses_[axis])],
                    snapshot.actuator_forces[
                        static_cast<std::size_t>(actuator_ids_[axis])]};
                const bool steer = feedback.axis < 4;
                const double gear_ratio =
                    steer ? config_.steer_gear_ratio :
                            config_.wheel_gear_ratio;
                const double rated_torque =
                    steer ? config_.steer_rated_motor_torque_nm :
                            config_.wheel_rated_motor_torque_nm;
                const double rated_current =
                    steer ? config_.steer_rated_current_a :
                            config_.wheel_rated_current_a;
                const double motor_torque =
                    config_.axes[axis].sign * feedback.joint_torque /
                    gear_ratio;
                const double torque_permille =
                    1000.0 * motor_torque / rated_torque;
                const double torque_constant =
                    rated_torque / rated_current;
                const double motor_current = motor_torque / torque_constant;
                const double current_permille =
                    1000.0 * motor_current / rated_current;
                const double physical_position =
                    feedback.position;
                const double physical_velocity =
                    feedback.velocity;
                const double motor_position =
                    physical_position * gear_ratio;
                const double motor_rpm =
                    physical_velocity * gear_ratio * 60.0 / kTwoPi;
                jiechang_[axis].set_feedback(
                    clamp_i32(
                        motor_position *
                        JiechangMotorEmulator::position_counts_per_revolution() /
                        kTwoPi),
                    clamp_i32(
                        motor_rpm *
                        JiechangMotorEmulator::velocity_counts_per_rpm()),
                    clamp_i16(torque_permille),
                    clamp_i16(current_permille));
            }
        }
        if (config_.stretch_family == ChassisStretchFamily::Jiechang)
        {
            update_jiechang_stretch(snapshot, period);
        }
        else
        {
            submit_slider_command();
        }
        if (config_.motor_family == ChassisMotorFamily::Jiechang)
        {
            submit_jiechang_command();
            send_jiechang_feedback();
        }
        else
        {
            submit_kinco_command();
            send_kinco_feedback(snapshot);
        }
    }

    void send_kinco_feedback(const PoseSnapshot& snapshot)
    {
        std::array<common::CanFdFrame, kJiechangAxisCount * 2> frames;
        std::vector<common::CanFdFrame> heartbeats;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (std::size_t axis = 0; axis < kJiechangAxisCount; ++axis)
            {
                const bool steer = axis < 4;
                const double sign = config_.axes[axis].sign;
                const double gear_ratio =
                    steer ? config_.kinco_steer_gear_ratio :
                            config_.kinco_wheel_gear_ratio;
                const double position = sign * snapshot.qpos[
                    static_cast<std::size_t>(qpos_addresses_[axis])];
                const double velocity = sign * snapshot.qvel[
                    static_cast<std::size_t>(dof_addresses_[axis])];
                const double torque = sign * snapshot.actuator_forces[
                    static_cast<std::size_t>(actuator_ids_[axis])];
                const double kt =
                    steer ? config_.kinco_steer_output_kt_nm_per_a :
                            config_.kinco_wheel_output_kt_nm_per_a;
                const double peak_ap =
                    steer ? config_.kinco_steer_max_current_arms *
                                std::sqrt(2.0) :
                            config_.kinco_wheel_max_current_arms *
                                std::sqrt(2.0);
                kinco_[axis].set_actual_position(clamp_i32(
                    position * gear_ratio *
                    static_cast<double>(config_.kinco_encoder_resolution) /
                    kTwoPi));
                kinco_[axis].set_actual_velocity(rpm_to_velocity_dec(
                    velocity * gear_ratio * 60.0 / kTwoPi,
                    config_.kinco_encoder_resolution));
                kinco_[axis].set_actual_current(clamp_i16(
                    (torque / kt) * std::sqrt(2.0) * 2048.0 / peak_ap));
                kinco_[axis].update_homing();
                frames[axis * 2] = kinco_[axis].tpdo1_frame(
                    config_.motor_can_interfaces[axis]);
                frames[axis * 2 + 1] = kinco_[axis].tpdo2_frame(
                    config_.motor_can_interfaces[axis]);
                const auto now = std::chrono::steady_clock::now();
                const auto period_ms = kinco_[axis].heartbeat_period_ms();
                if (period_ms > 0 && now - last_kinco_heartbeats_[axis] >=
                    std::chrono::milliseconds(period_ms))
                {
                    heartbeats.push_back(kinco_[axis].heartbeat_frame(config_.motor_can_interfaces[axis]));
                    last_kinco_heartbeats_[axis] = now;
                }
            }
        }
        for (const auto& frame : frames)
        {
            send_can(frame);
        }
        for (const auto& frame : heartbeats) send_can(frame);
    }

    void send_jiechang_feedback(
        std::optional<std::uint8_t> requested_interface = std::nullopt)
    {
        std::array<common::CanFdFrame, kJiechangAxisCount> frames;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (std::size_t axis = 0; axis < kJiechangAxisCount; ++axis)
            {
                frames[axis] = jiechang_[axis].feedback_frame(
                    config_.motor_can_interfaces[axis]);
            }
        }
        for (std::size_t axis = 0; axis < frames.size(); ++axis)
        {
            if (!requested_interface ||
                config_.motor_can_interfaces[axis] == *requested_interface)
            {
                send_can(frames[axis]);
            }
        }
    }

    void send_can(const common::CanFdFrame& frame)
    {
        common::McuEnvelope envelope;
        envelope.magic = common::kMcuMagicPoc2;
        envelope.source_code =
            jiechang_endpoint_.config->uplink_source_code;
        envelope.sequence = jiechang_endpoint_.sequence.fetch_add(1);
        envelope.timestamp_ns = timestamp_ns();
        envelope.gptp_ns = envelope.timestamp_ns;
        envelope.payload = common::encode_can_fd_payload(frame);
        const auto datagram = common::encode_mcu_uplink(envelope);
        jiechang_endpoint_.socket.send_to(
            {jiechang_endpoint_.config->peer_ip,
             jiechang_endpoint_.config->peer_port},
            datagram.data(), datagram.size());
    }

    void send_modbus(const common::ModbusRtuFramePayload& payload)
    {
        common::McuEnvelope envelope;
        envelope.magic = common::kMcuMagicPoc2;
        envelope.source_code = config_.m42p_endpoint.uplink_source_code;
        envelope.sequence = m42p_endpoint_.sequence.fetch_add(1);
        envelope.timestamp_ns = timestamp_ns();
        envelope.gptp_ns = envelope.timestamp_ns;
        envelope.payload = common::encode_modbus_rtu_payload(payload);
        const auto datagram = common::encode_mcu_uplink(envelope);
        m42p_endpoint_.socket.send_to(
            {m42p_endpoint_.config->peer_ip,
             m42p_endpoint_.config->peer_port},
            datagram.data(), datagram.size());
    }

    void synchronize_wheel_pi_reset(Runtime& runtime)
    {
        const auto generation = runtime.reset_generation();
        if (generation != wheel_pi_reset_generation_)
        {
            for (auto& controller : wheel_pi_) controller.reset();
            for (auto& profile : steer_profiles_) profile.reset();
            wheel_pi_reset_generation_ = generation;
        }
    }

    ChassisPluginConfig config_;
    std::mutex wheel_control_mutex_;
    std::array<WheelVelocityPi, 4> wheel_pi_;
    std::array<SteerPositionProfile, 4> steer_profiles_;
    std::array<double, 4> wheel_torque_limits_{};
    std::uint64_t wheel_pi_reset_generation_ = 0;
    std::uint64_t kinco_updater_id_ = 0;
    Runtime* runtime_ = nullptr;
    std::array<int, kAxisCount> qpos_addresses_{};
    std::array<int, kAxisCount> dof_addresses_{};
    std::array<int, kAxisCount> actuator_ids_{};
    std::array<JiechangMotorEmulator, kJiechangAxisCount> jiechang_;
    std::array<KincoMotorEmulator, kJiechangAxisCount> kinco_;
    std::array<M42pMotorEmulator, kSliderCount> m42p_;
    std::array<JiechangStretchEmulator, kSliderCount> stretch_;
    std::array<double, kSliderCount> stretch_reference_{};
    std::array<std::chrono::steady_clock::time_point, kSliderCount> last_stretch_commands_{};
    EndpointRuntime jiechang_endpoint_;
    EndpointRuntime m42p_endpoint_;
    common::PeriodicTask feedback_task_;
    common::StartupBrake startup_brake_;
    std::array<std::chrono::steady_clock::time_point, kJiechangAxisCount>
        last_jiechang_commands_{};
    std::array<std::chrono::steady_clock::time_point, kJiechangAxisCount> last_kinco_commands_{};
    std::array<std::chrono::steady_clock::time_point, kJiechangAxisCount> last_kinco_heartbeats_{};
    std::mutex mutex_;
    std::atomic<bool> running_{false};
};

ChassisPlugin::ChassisPlugin(ChassisPluginConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

ChassisPlugin::~ChassisPlugin() = default;

std::string ChassisPlugin::name() const { return "chassis"; }

bool ChassisPlugin::start(Runtime& runtime, std::string* error)
{
    return impl_->start(runtime, error);
}

void ChassisPlugin::stop() noexcept { impl_->stop(); }

bool ChassisPlugin::is_running() const noexcept
{
    return impl_->is_running();
}

}  // namespace sharpa::mujoco::plugins::chassis
