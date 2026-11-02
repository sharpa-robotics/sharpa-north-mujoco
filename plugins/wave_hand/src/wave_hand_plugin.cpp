// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/wave_hand/wave_hand_plugin.h"

#include "sharpa_mujoco/plugins/common/startup_brake.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/plugins/wave_hand/ltc_protocol.h"
#include "wave_hand_controller.h"
#include "sharpa_mujoco/runtime.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <poll.h>
#include <system_error>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <mujoco/mujoco.h>

namespace sharpa::mujoco::plugins::wave_hand
{
namespace
{

constexpr std::uint8_t kSuccess = 0;
constexpr std::uint8_t kInvalidInput = 1;
constexpr std::uint8_t kUnsupported = 5;
constexpr std::uint8_t kInnerError = 7;
constexpr std::uint8_t kOperationNotAllowed = 0x0D;
constexpr std::uint8_t kReady = 1;
constexpr std::uint8_t kFloatingMode = 10;
constexpr std::uint8_t kPositionMode = 13;
constexpr std::uint8_t kAdmittanceMode = 14;
constexpr std::uint8_t kMitMode = 15;
constexpr std::uint8_t kDefaultSource = 7;
constexpr std::uint16_t kHttpsPort = 8443;
constexpr float kCurrentCoefficientInterpolation = 0.1F;

constexpr std::uint16_t kGetReady = 0x0003;
constexpr std::uint16_t kGetStatus = 0x8002;
constexpr std::uint16_t kSetStatus = 0x8003;
constexpr std::uint16_t kGetMode = 0x8004;
constexpr std::uint16_t kSetMode = 0x8005;
constexpr std::uint16_t kGetCurrent = 0x800c;
constexpr std::uint16_t kSetCurrent = 0x800d;
constexpr std::uint16_t kGetJointPacketEnable = 0x8014;
constexpr std::uint16_t kSetJointPacketEnable = 0x8015;
constexpr std::uint16_t kGetError = 0x8026;
constexpr std::uint16_t kClearError = 0x804f;
constexpr std::uint16_t kGetSerial = 0x8028;
constexpr std::uint16_t kGetSource = 0x802a;
constexpr std::uint16_t kSetSource = 0x802b;
constexpr std::uint16_t kGetAllParameterInfo = 0x8052;
constexpr std::uint16_t kGetParameterInfo = 0x8053;
constexpr std::uint16_t kSetParameter = 0x8054;
constexpr std::uint16_t kGetParameter = 0x8055;

constexpr std::uint8_t kFloatParameterType = 9;
constexpr std::uint8_t kUint8ParameterType = 2;
constexpr std::uint8_t kWritablePersistentRanged = 0x0EU;
constexpr std::uint8_t kReadonly = 0x01U;
constexpr std::size_t kParameterInfoNameSize = 24;

struct SimulatedParameterInfo
{
    std::uint8_t group;
    std::uint8_t id;
    const char* name;
    std::uint8_t type;
    std::uint16_t size;
    std::uint8_t permissions;
    float minimum;
    float maximum;
};

struct SimulatedFault
{
    std::uint32_t joint_mask = 0;
    std::uint64_t sequence = 0;
};

constexpr std::array<SimulatedParameterInfo, 18> kSimulatedParameters{{
    {1, 0, "stiffness_coeff", kFloatParameterType, 1,
     kWritablePersistentRanged, 0.0F, 10.0F},
    {1, 1, "joint_speed_coeff", kFloatParameterType, 1,
     kWritablePersistentRanged, 0.01F, 1.0F},
    {1, 36, "min_position", kFloatParameterType, 22, kReadonly, 0.0F, 0.0F},
    {1, 37, "max_position", kFloatParameterType, 22, kReadonly, 0.0F, 0.0F},
    {1, 46, "mass", kFloatParameterType, 22,
     kWritablePersistentRanged, 0.0F, 10.0F},
    {1, 47, "damping", kFloatParameterType, 22,
     kWritablePersistentRanged, 0.0F, 10.0F},
    {1, 48, "stiffness", kFloatParameterType, 22,
     kWritablePersistentRanged, 0.0F, 10.0F},
    {1, 49, "torque_threshold", kFloatParameterType, 22,
     kWritablePersistentRanged, 0.0F, 10.0F},
    {1, 50, "joint_speed_limit", kFloatParameterType, 22,
     kWritablePersistentRanged, static_cast<float>(mjPI),
     5.0F * static_cast<float>(mjPI)},
    {1, 51, "friction_enable", kUint8ParameterType, 22,
     kWritablePersistentRanged, 0.0F, 1.0F},
    {1, 52, "coulomb_friction", kFloatParameterType, 22,
     kWritablePersistentRanged, 0.0F, 1.0F},
    {1, 53, "viscous_friction", kFloatParameterType, 22,
     kWritablePersistentRanged, 0.0F, 0.1F},
    {1, 54, "static_friction", kFloatParameterType, 22,
     kWritablePersistentRanged, 0.0F, 1.0F},
    {1, 55, "stribeck_velocity", kFloatParameterType, 22,
     kWritablePersistentRanged, 0.01F, 1.0F},
    {1, 62, "mit_kp", kFloatParameterType, 22,
     kWritablePersistentRanged, 0.0F, 100.0F},
    {1, 63, "mit_kd", kFloatParameterType, 22,
     kWritablePersistentRanged, 0.0F, 10.0F},
    {1, 64, "velocity_filter_alpha", kFloatParameterType, 1,
     kWritablePersistentRanged, 0.001F, 1.0F},
    {2, 0, "current_coeff", kFloatParameterType, 1,
     kWritablePersistentRanged, 0.2F, 1.0F},
}};

std::uint8_t fault_severity(std::uint16_t code)
{
    const std::uint8_t fault_id = static_cast<std::uint8_t>(code);
    if (fault_id == 0) return 0;
    const std::uint8_t level = (fault_id >> 6U) & 0x03U;
    return level == 0U ? 1U : level == 2U ? 3U : 2U;
}

constexpr std::array<const char*, kWaveJointCount> kJointSuffixes{{
    "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA", "thumb_IP",
    "index_MCP_FE", "index_MCP_AA", "index_PIP", "index_DIP",
    "middle_MCP_FE", "middle_MCP_AA", "middle_PIP", "middle_DIP",
    "ring_MCP_FE", "ring_MCP_AA", "ring_PIP", "ring_DIP",
    "pinky_CMC", "pinky_MCP_FE", "pinky_MCP_AA", "pinky_PIP", "pinky_DIP",
}};

bool supported_mode(std::uint8_t mode)
{
    return mode == kFloatingMode || mode == kPositionMode ||
           mode == kAdmittanceMode || mode == kMitMode;
}

bool fail(const std::string& message, std::string* error)
{
    if (error != nullptr)
    {
        *error = message;
    }
    return false;
}

std::uint64_t timestamp_us()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

bool parse_ipv4(const std::string& text, std::array<std::uint8_t, 4>& output)
{
    in_addr address{};
    if (::inet_pton(AF_INET, text.c_str(), &address) != 1)
    {
        return false;
    }
    std::memcpy(output.data(), &address.s_addr, output.size());
    return true;
}

template <std::size_t N>
void copy_text(const std::string& text, std::array<std::uint8_t, N>& output)
{
    output.fill(0);
    std::memcpy(output.data(), text.data(), std::min(text.size(), output.size()));
}

std::vector<std::uint8_t> byte_payload(std::uint8_t value)
{
    return {value};
}

std::vector<std::uint8_t> u16_payload(std::uint16_t value)
{
    return {static_cast<std::uint8_t>(value >> 8U),
            static_cast<std::uint8_t>(value)};
}

std::vector<std::uint8_t> u32_payload(std::uint32_t value)
{
    return {static_cast<std::uint8_t>(value >> 24U),
            static_cast<std::uint8_t>(value >> 16U),
            static_cast<std::uint8_t>(value >> 8U),
            static_cast<std::uint8_t>(value)};
}

std::vector<std::uint8_t> float_payload(float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return u32_payload(bits);
}

const SimulatedParameterInfo* find_parameter(std::uint8_t group,
                                             std::uint8_t id)
{
    const auto parameter = std::find_if(
        kSimulatedParameters.begin(), kSimulatedParameters.end(),
        [=](const SimulatedParameterInfo& value) {
            return value.group == group && value.id == id;
        });
    return parameter == kSimulatedParameters.end() ? nullptr : &*parameter;
}

std::vector<std::uint8_t> parameter_info_payload(
    const SimulatedParameterInfo& parameter)
{
    std::vector<std::uint8_t> payload(
        2 + kParameterInfoNameSize + 1 + 2 + 1 + 4 + 4, 0);
    payload[0] = parameter.group;
    payload[1] = parameter.id;
    const std::size_t name_size =
        std::min(std::strlen(parameter.name), kParameterInfoNameSize - 1);
    std::memcpy(payload.data() + 2, parameter.name, name_size);
    payload[2 + kParameterInfoNameSize] = parameter.type;
    payload[2 + kParameterInfoNameSize + 1] =
        static_cast<std::uint8_t>(parameter.size >> 8U);
    payload[2 + kParameterInfoNameSize + 2] =
        static_cast<std::uint8_t>(parameter.size);
    payload[2 + kParameterInfoNameSize + 3] = parameter.permissions;
    const auto minimum = float_payload(parameter.minimum);
    const auto maximum = float_payload(parameter.maximum);
    std::copy(minimum.begin(), minimum.end(), payload.end() - 8);
    std::copy(maximum.begin(), maximum.end(), payload.end() - 4);
    return payload;
}

bool read_float_be(const std::vector<std::uint8_t>& payload,
                   std::size_t offset,
                   float& value)
{
    if (payload.size() < offset + 4)
    {
        return false;
    }
    const std::uint32_t bits =
        (static_cast<std::uint32_t>(payload[offset]) << 24U) |
        (static_cast<std::uint32_t>(payload[offset + 1]) << 16U) |
        (static_cast<std::uint32_t>(payload[offset + 2]) << 8U) |
        static_cast<std::uint32_t>(payload[offset + 3]);
    std::memcpy(&value, &bits, sizeof(value));
    return std::isfinite(value);
}

bool send_all(int socket, const std::uint8_t* data, std::size_t size)
{
    std::size_t sent = 0;
    while (sent < size)
    {
        const ssize_t result =
            ::send(socket, data + sent, size - sent, MSG_NOSIGNAL);
        if (result > 0)
        {
            sent += static_cast<std::size_t>(result);
            continue;
        }
        if (result < 0 && errno == EINTR)
        {
            continue;
        }
        return false;
    }
    return true;
}

}  // namespace

class WaveHandPlugin::Impl
{
public:
    explicit Impl(WaveHandPluginConfig config) : config_(std::move(config)) {}
    ~Impl() { stop(); }

    bool start(Runtime& runtime, std::string* error)
    {
        if (alive_.load())
        {
            return true;
        }
        if (!runtime.is_loaded())
        {
            return fail("Wave hand plugin requires a loaded model.", error);
        }
        if (config_.name.empty() || config_.tcp_port == 0 ||
            config_.command_port == 0 || config_.feedback_source_port != 10000 ||
            config_.feedback_rate_hz <= 0.0 || config_.heartbeat_rate_hz <= 0.0)
        {
            return fail("Wave hand plugin configuration is invalid.", error);
        }
        for (std::size_t i = 0; i < kWaveJointCount; ++i)
        {
            if (!std::isfinite(config_.joint_signs[i]) ||
                !std::isfinite(config_.joint_offsets[i]) ||
                config_.joint_signs[i] == 0.0F)
            {
                return fail("Wave hand joint transform is invalid.", error);
            }
        }
        if (config_.feedback_destination == 0)
        {
            config_.feedback_destination =
                config_.side == WaveHandSide::kLeft ? 50000 : 50010;
        }
        if (config_.heartbeat_destination == 0)
        {
            config_.heartbeat_destination = 54321;
        }
        if (!parse_ipv4(config_.device_ip, device_ip_bytes_) ||
            !parse_ipv4(config_.sdk_ip, sdk_ip_bytes_))
        {
            return fail("Wave hand plugin requires numeric IPv4 addresses.", error);
        }
        if (!resolve_model(runtime.model_info(), error))
        {
            return false;
        }
        if (!startup_brake_.initialize(
                runtime,
                std::vector<std::string>(
                    joint_names_.begin(), joint_names_.end()),
                error))
        {
            return false;
        }
        if (!snapshot_valid(runtime.latest_snapshot()))
        {
            return fail("Initial wave hand snapshot is incomplete.", error);
        }
        if (!capture_actuator_model(runtime, error))
        {
            return false;
        }
        if (!command_socket_.open_and_bind(
                {config_.device_ip, config_.command_port}, 50, error,
                common::UdpBindMode::Exclusive))
        {
            return false;
        }
        if (!tx_socket_.open_and_bind(
                {config_.device_ip, config_.feedback_source_port}, 0, error,
                common::UdpBindMode::Exclusive))
        {
            command_socket_.close();
            return false;
        }
        if (!open_tcp_server(error))
        {
            command_socket_.close();
            tx_socket_.close();
            return false;
        }
        if (!open_https_fallback_server(error))
        {
            command_socket_.close();
            tx_socket_.close();
            close_tcp_sockets();
            return false;
        }

        runtime_ = &runtime;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            device_running_ = false;
            source_ = kDefaultSource;
            mode_ = kPositionMode;
            joint_packet_enable_ = true;
            speed_coefficient_ = 1.0F;
            current_coefficient_ = 1.0F;
            applied_current_coefficient_ = 1.0F;
            faults_.clear();
            fault_sequence_ = 0;
            faulted_ = false;
            reset_requested_.store(false);
            reset_recovery_ = ResetRecovery::None;
            reset_control_state_locked(runtime.latest_snapshot());
        }
        reset_observer_id_ = runtime.add_reset_observer(
            [this](std::uint64_t) {
                // Runtime owns the simulation lock. LTC can own state_mutex_
                // while waiting for Runtime, so never take that mutex here.
                reset_requested_.store(true, std::memory_order_release);
            });
        if (reset_observer_id_ == 0)
        {
            stop();
            return fail("Failed to register Wave hand reset observer.", error);
        }
        model_updater_id_ = runtime.add_model_updater(
            [this](mjModel* model, mjData* data,
                   const NativeStepContext& context) {
                update_control(model, data, context);
            });
        if (model_updater_id_ == 0)
        {
            stop();
            return fail("Failed to register Wave hand control updater.", error);
        }
        alive_.store(true);
        try
        {
            tcp_thread_ = std::thread([this] { tcp_loop(); });
            https_thread_ = std::thread([this] { https_fallback_loop(); });
            command_thread_ = std::thread([this] { command_loop(); });
            feedback_thread_ = std::thread([this] { feedback_loop(); });
            heartbeat_thread_ = std::thread([this] { heartbeat_loop(); });
        }
        catch (const std::system_error& exception)
        {
            stop();
            return fail("Wave hand worker creation failed: " +
                            std::string(exception.what()),
                        error);
        }
        return true;
    }

    void stop() noexcept
    {
        alive_.store(false);
        wait_cv_.notify_all();
        if (runtime_ != nullptr && reset_observer_id_ != 0)
        {
            runtime_->remove_reset_observer(reset_observer_id_);
            reset_observer_id_ = 0;
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (runtime_ != nullptr)
            {
                startup_brake_.engage(all_axes());
                device_running_ = false;
                restore_requested_ = true;
            }
        }
        if (runtime_ != nullptr && model_updater_id_ != 0)
        {
            if (!runtime_->is_running()) runtime_->step(1);
            {
                std::unique_lock<std::mutex> lock(restore_mutex_);
                restore_cv_.wait_for(
                    lock, std::chrono::milliseconds(50),
                    [this] { return !restore_requested_.load(); });
            }
            runtime_->remove_model_updater(model_updater_id_);
            model_updater_id_ = 0;
        }
        command_socket_.close();
        tx_socket_.close();
        close_tcp_sockets();
        if (tcp_thread_.joinable()) tcp_thread_.join();
        if (https_thread_.joinable()) https_thread_.join();
        if (command_thread_.joinable()) command_thread_.join();
        if (feedback_thread_.joinable()) feedback_thread_.join();
        if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
        runtime_ = nullptr;
    }

    std::string name() const { return config_.name; }
    bool running() const noexcept { return alive_.load(); }

    bool inject_fault(std::uint16_t code,
                      std::uint32_t joint_mask,
                      std::string* error)
    {
        constexpr std::uint32_t kJointMask =
            (1U << static_cast<unsigned>(kWaveJointCount)) - 1U;
        if (code == 0)
            return fail("Wave hand fault code must be non-zero.", error);
        if ((joint_mask & ~kJointMask) != 0)
            return fail("Wave hand fault mask exceeds 22 joints.", error);

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!alive_.load() || runtime_ == nullptr)
            return fail("Wave hand plugin must be running to inject a fault.",
                        error);
        if (!startup_brake_.engage(all_axes(), error)) return false;
        SimulatedFault& fault = faults_[code];
        fault.joint_mask |= joint_mask;
        fault.sequence = ++fault_sequence_;
        device_running_ = false;
        faulted_ = true;
        return true;
    }

private:
    static const std::vector<std::size_t>& all_axes()
    {
        static const std::vector<std::size_t> axes = [] {
            std::vector<std::size_t> result(kWaveJointCount);
            for (std::size_t i = 0; i < result.size(); ++i) result[i] = i;
            return result;
        }();
        return axes;
    }

    bool pause_locked()
    {
        if (!startup_brake_.engage(all_axes())) return false;
        device_running_ = false;
        reset_recovery_ = ResetRecovery::None;
        return true;
    }

    bool resume_locked()
    {
        if (runtime_ == nullptr) return false;
        const PoseSnapshot snapshot = runtime_->latest_snapshot();
        if (!snapshot_valid(snapshot)) return false;
        reset_control_state_locked(snapshot);
        if (!runtime_->write_native(
                [this, &snapshot](const mjModel*, mjData* data) {
                    for (std::size_t i = 0; i < kWaveJointCount; ++i)
                    {
                        data->ctrl[actuator_ids_[i]] =
                            mode_ == kFloatingMode || mode_ == kMitMode
                                ? 0.0
                                : snapshot.qpos[static_cast<std::size_t>(qpos_[i])];
                    }
                }))
            return false;
        if (!startup_brake_.release(all_axes())) return false;
        device_running_ = true;
        reset_recovery_ = ResetRecovery::None;
        return true;
    }

    void reset_control_state_locked(const PoseSnapshot& snapshot)
    {
        std::array<float, kWaveJointCount> positions{};
        for (std::size_t i = 0; i < kWaveJointCount; ++i)
        {
            positions[i] = static_cast<float>(
                snapshot.qpos[static_cast<std::size_t>(qpos_[i])]);
        }
        reset_control_state_locked(positions);
    }

    void reset_control_state_locked(
        const std::array<float, kWaveJointCount>& positions)
    {
        controller_.reset(positions);
        targets_ = {};
        targets_.position = positions;
        controller_.set_targets(targets_);
    }

    bool capture_actuator_model(Runtime& runtime, std::string* error)
    {
        return runtime.read_native(
            [this](const mjModel* model, const mjData*) {
                for (std::size_t i = 0; i < kWaveJointCount; ++i)
                {
                    const int id = actuator_ids_[i];
                    original_gain_types_[i] = model->actuator_gaintype[id];
                    original_bias_types_[i] = model->actuator_biastype[id];
                    original_ctrl_limited_[i] = model->actuator_ctrllimited[id];
                    original_force_limited_[i] = model->actuator_forcelimited[id];
                    for (int p = 0; p < mjNGAIN; ++p)
                        original_gain_parameters_[i][p] =
                            model->actuator_gainprm[id * mjNGAIN + p];
                    for (int p = 0; p < mjNBIAS; ++p)
                        original_bias_parameters_[i][p] =
                            model->actuator_biasprm[id * mjNBIAS + p];
                    for (int p = 0; p < 2; ++p)
                    {
                        original_ctrl_ranges_[i][p] =
                            model->actuator_ctrlrange[id * 2 + p];
                        original_force_ranges_[i][p] =
                            model->actuator_forcerange[id * 2 + p];
                    }
                }
            },
            error);
    }

    void restore_position_actuator(mjModel* model,
                                   std::size_t index,
                                   float current_coefficient) const
    {
        const int id = actuator_ids_[index];
        model->actuator_gaintype[id] = original_gain_types_[index];
        model->actuator_biastype[id] = original_bias_types_[index];
        model->actuator_ctrllimited[id] = original_ctrl_limited_[index];
        model->actuator_forcelimited[id] = original_force_limited_[index];
        for (int p = 0; p < mjNGAIN; ++p)
            model->actuator_gainprm[id * mjNGAIN + p] =
                original_gain_parameters_[index][p];
        for (int p = 0; p < mjNBIAS; ++p)
            model->actuator_biasprm[id * mjNBIAS + p] =
                original_bias_parameters_[index][p];
        for (int p = 0; p < 2; ++p)
        {
            model->actuator_ctrlrange[id * 2 + p] =
                original_ctrl_ranges_[index][p];
            model->actuator_forcerange[id * 2 + p] =
                original_force_ranges_[index][p] * current_coefficient;
        }
    }

    void configure_torque_actuator(mjModel* model,
                                   std::size_t index,
                                   float current_coefficient) const
    {
        const int id = actuator_ids_[index];
        model->actuator_gaintype[id] = mjGAIN_FIXED;
        model->actuator_biastype[id] = mjBIAS_NONE;
        model->actuator_ctrllimited[id] = 1;
        model->actuator_ctrlrange[id * 2] =
            original_force_ranges_[index][0] * current_coefficient;
        model->actuator_ctrlrange[id * 2 + 1] =
            original_force_ranges_[index][1] * current_coefficient;
        model->actuator_forcelimited[id] = original_force_limited_[index];
        for (int p = 0; p < mjNGAIN; ++p)
            model->actuator_gainprm[id * mjNGAIN + p] = p == 0 ? 1.0 : 0.0;
        for (int p = 0; p < mjNBIAS; ++p)
            model->actuator_biasprm[id * mjNBIAS + p] = 0.0;
        for (int p = 0; p < 2; ++p)
            model->actuator_forcerange[id * 2 + p] =
                original_force_ranges_[index][p] * current_coefficient;
    }

    bool resolve_model(const ModelInfo& model, std::string* error)
    {
        const std::string prefix =
            config_.side == WaveHandSide::kLeft ? "left_" : "right_";
        for (std::size_t i = 0; i < kWaveJointCount; ++i)
        {
            const std::string joint_name = prefix + kJointSuffixes[i];
            const std::string actuator_name = joint_name + "_ctrl";
            const auto joint = std::find_if(
                model.joints.begin(), model.joints.end(),
                [&](const JointInfo& item) { return item.name == joint_name; });
            const auto actuator = std::find_if(
                model.actuators.begin(), model.actuators.end(),
                [&](const ActuatorInfo& item) { return item.name == actuator_name; });
            if (joint == model.joints.end() || joint->qpos_width != 1 ||
                joint->dof_width != 1 || actuator == model.actuators.end() ||
                actuator->transmission_id != joint->id)
            {
                return fail("Missing or invalid wave hand joint/actuator: " +
                                joint_name,
                            error);
            }
            qpos_[i] = joint->qpos_address;
            dof_[i] = joint->dof_address;
            actuator_ids_[i] = actuator->id;
            joint_names_[i] = joint_name;
            joint_limited_[i] = joint->limited;
            joint_min_[i] = static_cast<float>(joint->range_min);
            joint_max_[i] = static_cast<float>(joint->range_max);
        }
        return true;
    }

    bool snapshot_valid(const PoseSnapshot& snapshot) const
    {
        for (std::size_t i = 0; i < kWaveJointCount; ++i)
        {
            if (qpos_[i] < 0 || dof_[i] < 0 || actuator_ids_[i] < 0 ||
                static_cast<std::size_t>(qpos_[i]) >= snapshot.qpos.size() ||
                static_cast<std::size_t>(dof_[i]) >= snapshot.qvel.size() ||
                static_cast<std::size_t>(actuator_ids_[i]) >=
                    snapshot.actuator_forces.size())
            {
                return false;
            }
        }
        return true;
    }

    bool open_tcp_server(std::string* error)
    {
        const int socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (socket < 0)
        {
            return fail("Wave hand TCP socket() failed: " +
                            std::to_string(errno),
                        error);
        }
        const int reuse = 1;
        ::setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        timeval timeout{};
        timeout.tv_usec = 100000;
        ::setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(config_.tcp_port);
        if (::inet_pton(AF_INET, config_.device_ip.c_str(), &address.sin_addr) !=
                1 ||
            ::bind(socket, reinterpret_cast<const sockaddr*>(&address),
                   sizeof(address)) != 0 ||
            ::listen(socket, 1) != 0)
        {
            const int code = errno;
            ::close(socket);
            return fail("Wave hand TCP bind/listen failed: " +
                            std::to_string(code),
                        error);
        }
        listen_fd_.store(socket);
        return true;
    }

    bool open_https_fallback_server(std::string* error)
    {
        const int socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (socket < 0)
        {
            return fail("Wave hand HTTPS fallback socket() failed: " +
                            std::to_string(errno),
                        error);
        }
        const int reuse = 1;
        ::setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(kHttpsPort);
        if (::inet_pton(AF_INET, config_.device_ip.c_str(), &address.sin_addr) !=
                1 ||
            ::bind(socket, reinterpret_cast<const sockaddr*>(&address),
                   sizeof(address)) != 0 ||
            ::listen(socket, 4) != 0)
        {
            const int code = errno;
            ::close(socket);
            return fail("Wave hand HTTPS fallback bind/listen failed: " +
                            std::to_string(code),
                        error);
        }
        https_listen_fd_.store(socket);
        return true;
    }

    void close_tcp_sockets() noexcept
    {
        const int client = client_fd_.exchange(-1);
        if (client >= 0)
        {
            ::shutdown(client, SHUT_RDWR);
            ::close(client);
        }
        const int listener = listen_fd_.exchange(-1);
        if (listener >= 0)
        {
            ::shutdown(listener, SHUT_RDWR);
            ::close(listener);
        }
        const int https_listener = https_listen_fd_.exchange(-1);
        if (https_listener >= 0)
        {
            ::shutdown(https_listener, SHUT_RDWR);
            ::close(https_listener);
        }
    }

    void https_fallback_loop()
    {
        while (alive_.load())
        {
            const int listener = https_listen_fd_.load();
            if (listener < 0) break;
            pollfd descriptor{listener, POLLIN, 0};
            const int ready = ::poll(&descriptor, 1, 100);
            if (ready < 0)
            {
                if (errno == EINTR) continue;
                return;
            }
            if (ready == 0 || (descriptor.revents & POLLIN) == 0) continue;
            const int accepted = ::accept(listener, nullptr, nullptr);
            if (accepted < 0) continue;
            linger reset{1, 0};
            ::setsockopt(accepted, SOL_SOCKET, SO_LINGER, &reset,
                         sizeof(reset));
            ::close(accepted);
        }
    }

    void tcp_loop()
    {
        LtcStreamParser parser;
        while (alive_.load())
        {
            const int listener = listen_fd_.load();
            if (listener < 0) break;
            const int client = client_fd_.load();
            pollfd descriptors[2]{
                {listener, POLLIN, 0},
                {client, POLLIN, 0},
            };
            const nfds_t count = client >= 0 ? 2U : 1U;
            const int ready = ::poll(descriptors, count, 100);
            if (ready < 0)
            {
                if (errno == EINTR) continue;
                return;
            }
            if (ready == 0) continue;

            if ((descriptors[0].revents & POLLIN) != 0)
            {
                sockaddr_in peer{};
                socklen_t peer_size = sizeof(peer);
                const int accepted = ::accept(
                    listener, reinterpret_cast<sockaddr*>(&peer), &peer_size);
                if (accepted >= 0)
                {
                    char peer_ip[INET_ADDRSTRLEN]{};
                    if (::inet_ntop(AF_INET, &peer.sin_addr, peer_ip,
                                    sizeof(peer_ip)) != nullptr &&
                        config_.sdk_ip == peer_ip)
                    {
                        timeval timeout{};
                        timeout.tv_usec = 100000;
                        ::setsockopt(accepted, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                                     sizeof(timeout));
                        const int old = client_fd_.exchange(accepted);
                        if (old >= 0)
                        {
                            ::shutdown(old, SHUT_RDWR);
                            ::close(old);
                        }
                        parser.reset();
                    }
                    else
                    {
                        ::close(accepted);
                    }
                }
            }

            if (client >= 0 &&
                (descriptors[1].revents & (POLLIN | POLLERR | POLLHUP)) != 0)
            {
                std::array<std::uint8_t, 4096> buffer{};
                const ssize_t received =
                    ::recv(client, buffer.data(), buffer.size(), 0);
                if (received <= 0)
                {
                    close_client(client);
                    parser.reset();
                    continue;
                }
                for (const LtcFrame& request : parser.push(
                         buffer.data(), static_cast<std::size_t>(received)))
                {
                    const LtcFrame response = handle_ltc(request);
                    std::vector<std::uint8_t> encoded;
                    if (!encode_ltc_frame(response, encoded) ||
                        !send_all(client, encoded.data(), encoded.size()))
                    {
                        close_client(client);
                        parser.reset();
                        break;
                    }
                }
            }
        }
    }

    void close_client(int client) noexcept
    {
        int expected = client;
        if (client_fd_.compare_exchange_strong(expected, -1))
        {
            ::shutdown(client, SHUT_RDWR);
            ::close(client);
        }
    }

    LtcFrame handle_ltc(const LtcFrame& request)
    {
        LtcFrame response;
        response.command = request.command;
        response.response_code = kSuccess;
        std::lock_guard<std::mutex> lock(state_mutex_);
        switch (request.command)
        {
            case kGetReady:
                response.payload = byte_payload(kReady);
                break;
            case kGetStatus:
                response.payload = byte_payload(device_running_ ? 1U : 0U);
                break;
            case kSetStatus:
                if (request.payload.size() != 1 || request.payload[0] > 1)
                    response.response_code = kInvalidInput;
                else if (request.payload[0] == 0)
                {
                    if (!pause_locked()) response.response_code = kInnerError;
                }
                else if (faulted_)
                    response.response_code = kOperationNotAllowed;
                else if (!device_running_ && !resume_locked())
                    response.response_code = kInnerError;
                break;
            case kGetMode:
                response.payload = byte_payload(mode_);
                break;
            case kSetMode:
                if (request.payload.size() != 1)
                    response.response_code = kInvalidInput;
                else if (device_running_ || faulted_)
                    response.response_code = kOperationNotAllowed;
                else if (!supported_mode(request.payload[0]))
                    response.response_code = kUnsupported;
                else
                {
                    mode_ = request.payload[0];
                    if (runtime_ != nullptr)
                    {
                        const PoseSnapshot snapshot = runtime_->latest_snapshot();
                        if (snapshot_valid(snapshot))
                            reset_control_state_locked(snapshot);
                    }
                }
                break;
            case kGetCurrent:
                response.payload = float_payload(current_coefficient_);
                break;
            case kSetCurrent:
            {
                float value = 0.0F;
                if (request.payload.empty())
                    response.response_code = kInvalidInput;
                else if (device_running_ || faulted_)
                    response.response_code = kOperationNotAllowed;
                else if (request.payload.size() != 4 ||
                         !read_float_be(request.payload, 0, value) ||
                         value < 0.2F || value > 1.0F)
                    response.response_code = kInvalidInput;
                else
                    current_coefficient_ = value;
                break;
            }
            case kGetJointPacketEnable:
                response.payload = byte_payload(joint_packet_enable_ ? 1U : 0U);
                break;
            case kSetJointPacketEnable:
                if (request.payload.empty())
                    response.response_code = kInvalidInput;
                else if (device_running_)
                    response.response_code = kOperationNotAllowed;
                else if (request.payload.size() != 1 || request.payload[0] > 1)
                    response.response_code = kInvalidInput;
                else
                    joint_packet_enable_ = request.payload[0] != 0;
                break;
            case kGetError:
                for (const auto& [code, fault] : faults_)
                {
                    const auto code_bytes = u16_payload(code);
                    const auto mask_bytes = u32_payload(fault.joint_mask);
                    response.payload.insert(response.payload.end(),
                                            code_bytes.begin(), code_bytes.end());
                    response.payload.insert(response.payload.end(),
                                            mask_bytes.begin(), mask_bytes.end());
                }
                break;
            case kClearError:
                if (request.payload.size() != 4)
                    response.response_code = kInvalidInput;
                else if (!pause_locked())
                    response.response_code = kInnerError;
                else
                {
                    faults_.clear();
                    faulted_ = false;
                }
                break;
            case kGetSerial:
                response.payload.assign(config_.serial.begin(),
                                        config_.serial.end());
                break;
            case kGetSource:
                response.payload = byte_payload(source_);
                break;
            case kSetSource:
                if (request.payload.size() != 1 ||
                    (request.payload[0] != 0 && request.payload[0] != 1 &&
                     request.payload[0] != 4 && request.payload[0] != 7))
                    response.response_code = kInvalidInput;
                else if (device_running_ || faulted_)
                    response.response_code = kOperationNotAllowed;
                else
                    source_ = request.payload[0];
                break;
            case kGetAllParameterInfo:
                for (const auto& parameter : kSimulatedParameters)
                {
                    response.payload.push_back(parameter.group);
                    response.payload.push_back(parameter.id);
                }
                break;
            case kGetParameterInfo:
                if (request.payload.size() < 2)
                    response.response_code = kInvalidInput;
                else if (const auto* parameter =
                             find_parameter(request.payload[0],
                                            request.payload[1]))
                    response.payload = parameter_info_payload(*parameter);
                else
                    response.response_code = kInvalidInput;
                break;
            case kSetParameter:
                set_parameter(request.payload, response);
                break;
            case kGetParameter:
                get_parameter(request.payload, response);
                break;
            default:
                response.response_code = kUnsupported;
                break;
        }
        return response;
    }

    void set_parameter(const std::vector<std::uint8_t>& payload,
                       LtcFrame& response)
    {
        if (payload.size() < 3)
        {
            response.response_code = kInvalidInput;
            return;
        }
        if (device_running_ || faulted_)
        {
            response.response_code = kOperationNotAllowed;
            return;
        }
        if (payload[0] == 1 && payload[1] == 1)
        {
            float value = 0.0F;
            if (payload.size() != 6 || !read_float_be(payload, 2, value))
            {
                response.response_code = kInvalidInput;
                return;
            }
            if (value < 0.01F || value > 1.0F)
                response.response_code = kInvalidInput;
            else
                speed_coefficient_ = value;
        }
        else if (payload[0] == 1 &&
                 (payload[1] == 0 || payload[1] == 64))
        {
            float value = 0.0F;
            if (payload.size() != 6 || !read_float_be(payload, 2, value) ||
                !controller_.set_scalar_parameter(payload[1], value))
                response.response_code = kInvalidInput;
        }
        else if (payload[0] == 1 &&
                 (payload[1] == 36 || payload[1] == 37))
        {
            response.response_code = kOperationNotAllowed;
        }
        else if (payload[0] == 2 && payload[1] == 0)
        {
            float value = 0.0F;
            if (payload.size() != 6 || !read_float_be(payload, 2, value))
            {
                response.response_code = kInvalidInput;
                return;
            }
            if (value < 0.2F || value > 1.0F)
                response.response_code = kInvalidInput;
            else
                current_coefficient_ = value;
        }
        else if (payload[0] == 1 && payload[1] == 51)
        {
            if (payload.size() != 2 + kWaveJointCount)
            {
                response.response_code = kInvalidInput;
                return;
            }
            std::array<std::uint8_t, kWaveJointCount> values{};
            std::copy(payload.begin() + 2, payload.end(), values.begin());
            if (!controller_.set_u8_parameter(payload[1], values))
                response.response_code = kInvalidInput;
        }
        else if (payload[0] == 1)
        {
            if (payload.size() != 2 + kWaveJointCount * sizeof(float))
            {
                response.response_code = kInvalidInput;
                return;
            }
            std::array<float, kWaveJointCount> values{};
            for (std::size_t i = 0; i < values.size(); ++i)
            {
                if (!read_float_be(payload, 2 + i * sizeof(float), values[i]))
                {
                    response.response_code = kInvalidInput;
                    return;
                }
            }
            if (!controller_.set_float_parameter(payload[1], values))
                response.response_code = kInvalidInput;
        }
        else
            response.response_code = kInvalidInput;
    }

    void get_parameter(const std::vector<std::uint8_t>& payload,
                       LtcFrame& response) const
    {
        if (payload.size() < 2)
        {
            response.response_code = kInvalidInput;
            return;
        }
        if (payload[0] == 1 && payload[1] == 1)
            response.payload = float_payload(speed_coefficient_);
        else if (payload[0] == 1 &&
                 (payload[1] == 0 || payload[1] == 64))
        {
            float value = 0.0F;
            if (!controller_.get_scalar_parameter(payload[1], value))
                response.response_code = kInvalidInput;
            else
                response.payload = float_payload(value);
        }
        else if (payload[0] == 2 && payload[1] == 0)
            response.payload = float_payload(current_coefficient_);
        else if (payload[0] == 1 && payload[1] == 51)
        {
            std::array<std::uint8_t, kWaveJointCount> values{};
            if (!controller_.get_u8_parameter(payload[1], values))
                response.response_code = kInvalidInput;
            else
                response.payload.assign(values.begin(), values.end());
        }
        else if (payload[0] == 1)
        {
            std::array<float, kWaveJointCount> values{};
            if (!controller_.get_float_parameter(payload[1], values) &&
                !controller_.get_readonly_float_parameter(payload[1], values))
                response.response_code = kInvalidInput;
            else
            {
                for (float value : values)
                {
                    const auto encoded = float_payload(value);
                    response.payload.insert(response.payload.end(),
                                            encoded.begin(), encoded.end());
                }
            }
        }
        else
            response.response_code = kInvalidInput;
    }

    void command_loop()
    {
        std::array<std::uint8_t, kWaveJointPacketV0Size> buffer{};
        while (alive_.load())
        {
            std::size_t received = 0;
            common::UdpEndpoint peer;
            if (!command_socket_.receive(buffer.data(), buffer.size(), received,
                                         nullptr, &peer))
                continue;
            if (!alive_.load()) break;
            const std::uint64_t received_generation = runtime_->reset_generation();
            if (peer.ip != config_.sdk_ip)
            {
                log_command_summary(peer, received, false, "peer");
                continue;
            }
            WaveJointPacketV0 packet;
            if (!decode_joint_packet_v0(buffer.data(), received, packet))
            {
                log_command_summary(peer, received, false, "decode");
                continue;
            }
            if (packet.header.side != config_.side)
            {
                log_command_summary(peer, received, false, "side");
                continue;
            }

            Runtime* runtime = runtime_;
            if (runtime == nullptr)
            {
                log_command_summary(peer, received, false, "lifecycle");
                continue;
            }
            std::lock_guard<std::mutex> lock(state_mutex_);
            // Drop commands until the simulation thread has rebased the
            // controller on the new mjData. Never replay a cached target.
            if (reset_requested_.load(std::memory_order_acquire) ||
                received_generation != runtime->reset_generation())
            {
                log_command_summary(peer, received, false, "reset");
                continue;
            }
            if (!device_running_ || faulted_ || source_ == kDefaultSource)
            {
                log_command_summary(peer, received, false, "lifecycle");
                continue;
            }
            if (config_.strict_command_state &&
                (packet.header.control_source != source_ ||
                 packet.header.control_mode != mode_))
            {
                log_command_summary(peer, received, false, "control");
                continue;
            }
            for (std::size_t i = 0; i < kWaveJointCount; ++i)
            {
                float target = packet.joints[i].angle * config_.joint_signs[i] +
                               config_.joint_offsets[i];
                if (joint_limited_[i])
                    target = std::clamp(target, joint_min_[i], joint_max_[i]);
                targets_.position[i] = target;
                targets_.velocity[i] =
                    packet.joints[i].velocity * config_.joint_signs[i];
                targets_.torque[i] =
                    packet.joints[i].torque * config_.joint_signs[i];
            }
            controller_.set_targets(targets_);
            if (reset_recovery_ == ResetRecovery::AwaitCommand)
                reset_recovery_ = ResetRecovery::CommandReady;
            runtime->notify_step_ready();
            log_command_summary(peer, received, true, "applied",
                                packet.joints[0].angle);
        }
    }

    void update_control(mjModel* model,
                        mjData* data,
                        const NativeStepContext& context)
    {
        // The updater runs while Runtime owns the simulation lock. LTC state
        // transitions may hold state_mutex_ while engaging a brake through
        // Runtime, so skip one control tick instead of inverting those locks.
        std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return;
        if (reset_requested_.load(std::memory_order_acquire) &&
            reset_requested_.exchange(false, std::memory_order_acq_rel))
        {
            std::array<float, kWaveJointCount> positions{};
            const bool torque_mode = mode_ == kFloatingMode || mode_ == kMitMode;
            for (std::size_t i = 0; i < kWaveJointCount; ++i)
            {
                positions[i] = static_cast<float>(data->qpos[qpos_[i]]);
                data->ctrl[actuator_ids_[i]] = torque_mode ? 0.0 : positions[i];
            }
            reset_control_state_locked(positions);
            // A task stop/start does not reconnect the SDK device. Preserve
            // the session, but require a newly accepted command to unlock it.
            reset_recovery_ = device_running_ && !faulted_
                ? ResetRecovery::AwaitCommand : ResetRecovery::None;
        }
        if (restore_requested_.load())
        {
            for (std::size_t i = 0; i < kWaveJointCount; ++i)
            {
                restore_position_actuator(model, i, 1.0F);
                data->ctrl[actuator_ids_[i]] = data->qpos[qpos_[i]];
            }
            restore_requested_.store(false);
            restore_cv_.notify_all();
            return;
        }
        applied_current_coefficient_ +=
            kCurrentCoefficientInterpolation *
            (current_coefficient_ - applied_current_coefficient_);
        applied_current_coefficient_ =
            std::clamp(applied_current_coefficient_, 0.2F, 1.0F);

        const bool torque_mode = mode_ == kFloatingMode || mode_ == kMitMode;
        for (std::size_t i = 0; i < kWaveJointCount; ++i)
        {
            if (torque_mode)
                configure_torque_actuator(
                    model, i, applied_current_coefficient_);
            else
                restore_position_actuator(
                    model, i, applied_current_coefficient_);
        }
        if (!device_running_ || faulted_)
        {
            if (torque_mode)
            {
                for (std::size_t i = 0; i < kWaveJointCount; ++i)
                    data->ctrl[actuator_ids_[i]] = 0.0;
            }
            return;
        }
        if (reset_recovery_ == ResetRecovery::AwaitCommand) return;
        if (reset_recovery_ == ResetRecovery::CommandReady)
        {
            if (!startup_brake_.release_in_step(model, data, all_axes())) return;
            reset_recovery_ = ResetRecovery::None;
        }

        WaveJointState state;
        for (std::size_t i = 0; i < kWaveJointCount; ++i)
        {
            state.position[i] = static_cast<float>(data->qpos[qpos_[i]]);
            state.velocity[i] = static_cast<float>(data->qvel[dof_[i]]);
            state.effort[i] =
                static_cast<float>(data->actuator_force[actuator_ids_[i]]);
        }
        const WaveControlOutput output = controller_.update(
            static_cast<WaveControlMode>(mode_),
            static_cast<float>(context.timestep), speed_coefficient_, state);
        for (std::size_t i = 0; i < kWaveJointCount; ++i)
        {
            const int actuator = actuator_ids_[i];
            if (output.torque_mode)
                data->ctrl[actuator] = std::clamp(
                    output.value[i],
                    static_cast<float>(model->actuator_ctrlrange[actuator * 2]),
                    static_cast<float>(model->actuator_ctrlrange[actuator * 2 + 1]));
            else
                data->ctrl[actuator] = output.value[i];
        }
    }

    void log_command_summary(const common::UdpEndpoint& peer,
                             std::size_t size,
                             bool accepted,
                             const char* reason,
                             float first_angle = 0.0F)
    {
        const std::uint64_t now = timestamp_us();
        std::uint64_t previous = last_command_log_us_.load();
        if (previous != 0 && now - previous < 1000000U) return;
        if (!last_command_log_us_.compare_exchange_strong(previous, now)) return;
        std::clog << "[WaveHand][" << config_.name << "] command src="
                  << peer.ip << ':' << peer.port << " bytes=" << size
                  << " accepted=" << (accepted ? 1 : 0)
                  << " reason=" << reason << " joint0=" << first_angle << '\n';
    }

    WaveDataHeader make_header(WavePacketType type,
                               std::uint8_t version,
                               std::uint16_t sequence) const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return make_header_locked(type, version, sequence);
    }

    WaveDataHeader make_header_locked(WavePacketType type,
                                      std::uint8_t version,
                                      std::uint16_t sequence) const
    {
        WaveDataHeader header;
        header.packet_type = type;
        header.payload_version = version;
        copy_text(config_.serial, header.serial_number);
        header.side = config_.side;
        header.sequence = sequence;
        header.timestamp = timestamp_us();
        header.control_source = source_;
        header.control_mode = mode_;
        header.running = device_running_;
        return header;
    }

    void feedback_loop()
    {
        periodic_loop(config_.feedback_rate_hz, [this] { send_feedback(); });
    }

    void heartbeat_loop()
    {
        for (int i = 0; i < 3 && alive_.load(); ++i)
        {
            send_heartbeat();
            if (i != 2)
            {
                std::unique_lock<std::mutex> lock(wait_mutex_);
                wait_cv_.wait_for(lock, std::chrono::milliseconds(100),
                                  [this] { return !alive_.load(); });
            }
        }
        periodic_loop(config_.heartbeat_rate_hz, [this] { send_heartbeat(); });
    }

    template <typename Task>
    void periodic_loop(double rate_hz, Task task)
    {
        using Clock = std::chrono::steady_clock;
        const auto period =
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(1.0 / rate_hz));
        auto deadline = Clock::now();
        while (alive_.load())
        {
            task();
            deadline += period;
            std::unique_lock<std::mutex> lock(wait_mutex_);
            wait_cv_.wait_until(lock, deadline,
                                [this] { return !alive_.load(); });
            if (Clock::now() - deadline > period) deadline = Clock::now();
        }
    }

    void send_feedback()
    {
        Runtime* runtime = runtime_;
        if (!alive_.load() || runtime == nullptr) return;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!joint_packet_enable_) return;
        }
        const PoseSnapshot snapshot = runtime->latest_snapshot();
        if (!snapshot_valid(snapshot)) return;
        WaveJointPacketV0 packet;
        packet.header =
            make_header(WavePacketType::kJointData, 0, feedback_sequence_++);
        for (std::size_t i = 0; i < kWaveJointCount; ++i)
        {
            packet.joints[i].angle =
                static_cast<float>(snapshot.qpos[static_cast<std::size_t>(qpos_[i])]);
            packet.joints[i].velocity =
                static_cast<float>(snapshot.qvel[static_cast<std::size_t>(dof_[i])]);
            packet.joints[i].torque = static_cast<float>(
                snapshot.actuator_forces[static_cast<std::size_t>(actuator_ids_[i])]);
        }
        std::array<std::uint8_t, kWaveJointPacketV0Size> encoded{};
        if (encode_joint_packet_v0(packet, encoded))
        {
            tx_socket_.send_to(
                {config_.sdk_ip, config_.feedback_destination}, encoded.data(),
                encoded.size());
        }
    }

    void send_heartbeat()
    {
        WaveHeartbeatV2 heartbeat;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            heartbeat.header = make_header_locked(
                WavePacketType::kHeartbeat, 2, heartbeat_sequence_++);
            if (!faults_.empty())
            {
                const auto selected = std::max_element(
                    faults_.begin(), faults_.end(),
                    [](const auto& left, const auto& right) {
                        const std::uint8_t left_severity =
                            fault_severity(left.first);
                        const std::uint8_t right_severity =
                            fault_severity(right.first);
                        return left_severity != right_severity
                                   ? left_severity < right_severity
                                   : left.second.sequence < right.second.sequence;
                    });
                heartbeat.status.error_code = selected->first;
                heartbeat.status.error_joint = selected->second.joint_mask;
            }
        }
        copy_text(config_.PN, heartbeat.part_number);
        copy_text(std::string("SHARPA"), heartbeat.manufacturer);
        // SDK 5.0.5 rejects the old simulated 1.0.0 version before consulting
        // its firmware recommendation map. Advertise the minimum compatible
        // Wave firmware version.
        heartbeat.firmware_version = {{3, 0, 0, 0}};
        heartbeat.mac = config_.MAC;
        heartbeat.device_ip = device_ip_bytes_;
        heartbeat.joint_destination_ip = sdk_ip_bytes_;
        heartbeat.tactile_destination_ip = sdk_ip_bytes_;
        heartbeat.debug_destination_ip = sdk_ip_bytes_;
        heartbeat.heartbeat_destination_ip = sdk_ip_bytes_;
        heartbeat.joint_destination_port = config_.feedback_destination;
        // SDK 5.0.5 treats a zero debug endpoint as a request to choose a
        // dynamic port, whose retry path leaks sockets. Match hardware defaults
        // even though these streams are not otherwise simulated.
        heartbeat.tactile_destination_port =
            config_.side == WaveHandSide::kLeft ? 50011 : 50001;
        heartbeat.debug_destination_port =
            config_.side == WaveHandSide::kLeft ? 50005 : 50015;
        heartbeat.heartbeat_destination_port =
            config_.heartbeat_destination;
        heartbeat.joint_listen_port = config_.command_port;
        heartbeat.tcp_listen_port = config_.tcp_port;
        if (config_.fingertip_tactile_supported)
            heartbeat.feature_config = kWaveFeatureFingertipTactile;
        std::array<std::uint8_t, kWaveHeartbeatV2Size> encoded{};
        if (encode_heartbeat_v2(heartbeat, encoded))
            tx_socket_.send_to(
                {config_.sdk_ip, config_.heartbeat_destination}, encoded.data(),
                encoded.size());
    }

    WaveHandPluginConfig config_;
    Runtime* runtime_ = nullptr;
    std::uint64_t model_updater_id_ = 0;
    std::uint64_t reset_observer_id_ = 0;
    std::atomic<bool> reset_requested_{false};
    enum class ResetRecovery { None, AwaitCommand, CommandReady };
    ResetRecovery reset_recovery_ = ResetRecovery::None;
    std::atomic<bool> alive_{false};
    common::UdpSocket command_socket_;
    common::UdpSocket tx_socket_;
    std::atomic<int> listen_fd_{-1};
    std::atomic<int> client_fd_{-1};
    std::atomic<int> https_listen_fd_{-1};
    std::thread tcp_thread_;
    std::thread https_thread_;
    std::thread command_thread_;
    std::thread feedback_thread_;
    std::thread heartbeat_thread_;
    std::mutex wait_mutex_;
    std::condition_variable wait_cv_;
    std::mutex restore_mutex_;
    std::condition_variable restore_cv_;
    std::atomic<bool> restore_requested_{false};

    std::array<int, kWaveJointCount> qpos_{};
    std::array<int, kWaveJointCount> dof_{};
    std::array<int, kWaveJointCount> actuator_ids_{};
    std::array<std::string, kWaveJointCount> joint_names_{};
    std::array<bool, kWaveJointCount> joint_limited_{};
    std::array<float, kWaveJointCount> joint_min_{};
    std::array<float, kWaveJointCount> joint_max_{};
    WaveHandController controller_;
    WaveJointTargets targets_{};
    std::array<int, kWaveJointCount> original_gain_types_{};
    std::array<int, kWaveJointCount> original_bias_types_{};
    std::array<int, kWaveJointCount> original_ctrl_limited_{};
    std::array<int, kWaveJointCount> original_force_limited_{};
    std::array<std::array<double, mjNGAIN>, kWaveJointCount>
        original_gain_parameters_{};
    std::array<std::array<double, mjNBIAS>, kWaveJointCount>
        original_bias_parameters_{};
    std::array<std::array<double, 2>, kWaveJointCount> original_ctrl_ranges_{};
    std::array<std::array<double, 2>, kWaveJointCount> original_force_ranges_{};
    std::array<std::uint8_t, 4> device_ip_bytes_{};
    std::array<std::uint8_t, 4> sdk_ip_bytes_{};

    mutable std::mutex state_mutex_;
    bool device_running_ = false;
    bool faulted_ = false;
    std::uint8_t source_ = kDefaultSource;
    std::uint8_t mode_ = kPositionMode;
    bool joint_packet_enable_ = true;
    float speed_coefficient_ = 1.0F;
    float current_coefficient_ = 1.0F;
    float applied_current_coefficient_ = 1.0F;
    std::map<std::uint16_t, SimulatedFault> faults_;
    std::uint64_t fault_sequence_ = 0;
    std::atomic<std::uint16_t> feedback_sequence_{0};
    std::atomic<std::uint16_t> heartbeat_sequence_{0};
    std::atomic<std::uint64_t> last_command_log_us_{0};
    common::StartupBrake startup_brake_;
};

WaveHandPlugin::WaveHandPlugin(WaveHandPluginConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

WaveHandPlugin::~WaveHandPlugin() = default;
std::string WaveHandPlugin::name() const { return impl_->name(); }
bool WaveHandPlugin::start(Runtime& runtime, std::string* error)
{
    return impl_->start(runtime, error);
}
void WaveHandPlugin::stop() noexcept { impl_->stop(); }
bool WaveHandPlugin::is_running() const noexcept { return impl_->running(); }
bool WaveHandPlugin::inject_fault(std::uint16_t code,
                                  std::uint32_t joint_mask,
                                  std::string* error)
{
    return impl_->inject_fault(code, joint_mask, error);
}

}  // namespace sharpa::mujoco::plugins::wave_hand
