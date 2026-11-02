// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "mcu_udp_test_helpers.h"

#include "sharpa_mujoco/device_plugin.h"
#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"
#include "sharpa_mujoco/plugins/common/socketcan_transport.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/plugins/torso/torso_plugin.h"
#include "sharpa_mujoco/plugins/torso/parallel_waist_kinematics.h"
#include "sharpa_mujoco/runtime.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

bool require(bool condition, const std::string& message)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << message << '\n';
    }
    return condition;
}

class RecordingPlugin final : public sharpa::mujoco::IDevicePlugin
{
public:
    RecordingPlugin(
        std::string name,
        bool start_result,
        std::vector<std::string>& events)
        : name_(std::move(name)),
          start_result_(start_result),
          events_(events)
    {
    }

    std::string name() const override { return name_; }
    bool start(sharpa::mujoco::Runtime&, std::string* error) override
    {
        events_.push_back("start:" + name_);
        if (!start_result_)
        {
            if (error) *error = "requested failure";
            return false;
        }
        running_ = true;
        return true;
    }
    void stop() noexcept override
    {
        if (running_)
        {
            events_.push_back("stop:" + name_);
            running_ = false;
        }
    }
    bool is_running() const noexcept override { return running_; }

private:
    std::string name_;
    bool start_result_;
    std::vector<std::string>& events_;
    bool running_ = false;
};

bool test_manager_rollback()
{
    sharpa::mujoco::Runtime runtime;
    sharpa::mujoco::DevicePluginManager manager;
    std::vector<std::string> events;
    std::string error;
    manager.add(std::make_unique<RecordingPlugin>("first", true, events), &error);
    manager.add(std::make_unique<RecordingPlugin>("failing", false, events), &error);
    const std::vector<std::string> expected{
        "start:first", "start:failing", "stop:first"};
    return require(
               !manager.start_all(runtime, &error),
               "Manager accepted a failed plugin.") &&
           require(events == expected, "Manager rollback order is incorrect.");
}

bool test_socketcan_validation()
{
    using sharpa::mujoco::plugins::common::SocketCanFdSocket;
    SocketCanFdSocket socket;
    std::string error;
    return require(
               !socket.open_and_bind("", 10, &error),
               "SocketCAN accepted an empty interface name.") &&
           require(
               !socket.open_and_bind("vcan_missing", 10, &error),
               "SocketCAN accepted a missing interface.") &&
           require(error.find("vcan_missing") != std::string::npos &&
                   error.find("setup_vcan.sh") != std::string::npos,
                   "Missing SocketCAN interface did not report its name and recovery command.") &&
           require(!socket.is_open(), "Failed SocketCAN open left a live socket.");
}

const sharpa::mujoco::ActuatorInfo* find_actuator(
    const sharpa::mujoco::ModelInfo& model,
    const std::string& name)
{
    for (const auto& actuator : model.actuators)
    {
        if (actuator.name == name) return &actuator;
    }
    return nullptr;
}

const sharpa::mujoco::JointInfo* find_joint(
    const sharpa::mujoco::ModelInfo& model,
    const std::string& name)
{
    for (const auto& joint : model.joints)
    {
        if (joint.name == name) return &joint;
    }
    return nullptr;
}

std::vector<std::uint8_t> make_downlink(
    std::uint8_t can_interface,
    std::uint32_t can_id,
    const std::vector<std::uint8_t>& can_data,
    std::uint32_t sequence)
{
    using namespace sharpa::mujoco::plugins::common;
    CanFdFrame frame;
    frame.can_interface = can_interface;
    frame.can_id = can_id;
    frame.can_flags = kCanFdFlagBrs;
    frame.data = can_data;
    const std::vector<std::uint8_t> payload = encode_can_fd_payload(frame);
    std::vector<std::uint8_t> output(kMcuEnvelopeSize + payload.size() + kMcuCrcSize, 0);
    write_be32(output.data(), kMcuMagicPoc2);
    output[4] = kMcuProtocolVersion;
    write_be32(
        output.data() + 6,
        make_source_code(kMcuDomainCommand, kMcuClassMotor));
    write_be32(output.data() + 10, sequence);
    write_be16(output.data() + 22, static_cast<std::uint16_t>(payload.size()));
    write_be16(output.data() + 26, 1);
    std::memcpy(output.data() + kMcuEnvelopeSize, payload.data(), payload.size());
    write_be32(
        output.data() + kMcuEnvelopeSize + payload.size(),
        mcu_crc32(output.data() + 4, (kMcuEnvelopeSize - 4) + payload.size()));
    return output;
}

std::int16_t position_count(double radians)
{
    return static_cast<std::int16_t>(std::lround(radians * 32768.0 / kPi));
}

void set_csp_slot(
    std::vector<std::uint8_t>& data,
    std::size_t slot,
    std::uint8_t motor_id,
    double motor_radians,
    bool enabled = true)
{
    const std::int16_t target = position_count(motor_radians);
    data[slot * 7] = enabled ? 0xC6 : 0x00;
    data[slot * 7 + 1] = static_cast<std::uint8_t>(
        static_cast<std::uint16_t>(target) >> 8U);
    data[slot * 7 + 2] = static_cast<std::uint8_t>(target);
    data[56 + slot] = motor_id;
}

void set_mit_slot(
    std::vector<std::uint8_t>& data,
    std::size_t slot,
    std::uint8_t motor_id,
    double position_rad,
    double velocity_rad_s,
    double kp_nm_per_rad,
    double kd_nm_s_per_rad,
    double torque_nm)
{
    const auto symmetric_raw = [](
                                   double value,
                                   double range,
                                   std::uint16_t maximum) {
        const double normalized =
            std::round(
                (value + range) / (2.0 * range) *
                static_cast<double>(maximum));
        return static_cast<std::uint16_t>(
            std::max(
                0.0,
                std::min(static_cast<double>(maximum), normalized)));
    };
    const auto unsigned_raw = [](double value, double maximum_value) {
        return static_cast<std::uint16_t>(std::lround(
            std::clamp(value, 0.0, maximum_value) / maximum_value * 4095.0));
    };
    const std::uint16_t position =
        symmetric_raw(position_rad, 3.14, 65535);
    const std::uint16_t velocity =
        symmetric_raw(velocity_rad_s, 3.14, 4095);
    const std::uint16_t kp = unsigned_raw(kp_nm_per_rad, 2500.0);
    const std::uint16_t kd = unsigned_raw(kd_nm_s_per_rad, 500.0);
    const std::uint16_t torque = symmetric_raw(torque_nm, 300.0, 4095);
    data[slot * 9] = 0xCC;
    data[slot * 9 + 1] = static_cast<std::uint8_t>(position >> 8U);
    data[slot * 9 + 2] = static_cast<std::uint8_t>(position);
    data[slot * 9 + 3] = static_cast<std::uint8_t>(velocity >> 4U);
    data[slot * 9 + 4] =
        static_cast<std::uint8_t>(
            ((velocity & 0x0FU) << 4U) | (kp >> 8U));
    data[slot * 9 + 5] = static_cast<std::uint8_t>(kp);
    data[slot * 9 + 6] = static_cast<std::uint8_t>(kd >> 4U);
    data[slot * 9 + 7] = static_cast<std::uint8_t>(
        ((kd & 0x0FU) << 4U) | (torque >> 8U));
    data[slot * 9 + 8] = static_cast<std::uint8_t>(torque);
    data[56 + slot] = motor_id;
}

void set_mit_torque_slot(
    std::vector<std::uint8_t>& data,
    std::size_t slot,
    std::uint8_t motor_id,
    double torque_nm)
{
    set_mit_slot(data, slot, motor_id, 0.0, 0.0, 0.0, 0.0, torque_nm);
}

bool test_mixed_mode_brakes(bool lower_first, double command_delay = 0.0)
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;

    Runtime runtime;
    std::string error;
    if (!runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error)) return require(false, error);
    torso::TorsoPluginConfig config;
    config.command_delay_s.fill(command_delay);
    std::array<common::UdpSocket, 2> hosts;
    for (std::size_t i = 0; i < hosts.size(); ++i)
    {
        common::UdpSocket probe;
        if (!hosts[i].open_and_bind({"127.0.0.1", 0}, 10, &error) ||
            !probe.open_and_bind({"127.0.0.1", 0}, 10, &error))
            return require(false, error);
        config.endpoints[i].bind_port = probe.local_port();
        config.endpoints[i].peer_port = hosts[i].local_port();
    }
    torso::TorsoPlugin plugin(config);
    if (!plugin.start(runtime, &error)) return require(false, error);
    for (std::size_t i = 0; i < hosts.size(); ++i)
    {
        if (!test::wait_for_mcu_feedback(
                hosts[i], {"127.0.0.1", config.endpoints[i].bind_port}, error, 0))
            return require(false, error);
    }

    const std::array<const char*, 8> brakes{
        "brake_lower_body_joint_1", "brake_lower_body_joint_2",
        "brake_lower_body_joint_3", "brake_lower_body_joint_4",
        "brake_lower_body_joint_5", "brake_neck_joint_1",
        "brake_neck_joint_2", "brake_neck_joint_3"};
    std::array<bool, 8> expected{};
    expected.fill(true);
    const auto expect_brakes = [&](const std::string& stage) {
        std::array<bool, 8> actual{};
        bool found = true;
        if (!runtime.read_native(
                [&](const mjModel* model, const mjData* data) {
                    for (std::size_t i = 0; i < brakes.size(); ++i)
                    {
                        const int id = mj_name2id(model, mjOBJ_EQUALITY, brakes[i]);
                        if (id < 0) found = false;
                        else actual[i] = data->eq_active[id] != 0;
                    }
                }, &error))
            return require(false, error);
        if (!require(found, "Missing torso brake equality.")) return false;
        for (std::size_t i = 0; i < brakes.size(); ++i)
        {
            if (!require(actual[i] == expected[i],
                         stage + ": wrong physical brake state for " + brakes[i]))
                return false;
        }
        return true;
    };

    std::uint32_t sequence = 1;
    const auto send = [&](std::size_t endpoint, std::uint32_t can_id,
                          const std::vector<std::uint8_t>& data) {
        const common::UdpEndpoint peer{"127.0.0.1", config.endpoints[endpoint].bind_port};
        const auto packet = make_downlink(0, can_id, data, sequence++);
        if (!hosts[endpoint].send_to(peer, packet.data(), packet.size(), &error))
            return require(false, error);
        // A read on the same receive queue acknowledges completion of the
        // preceding control handler, including its physical brake updates.
        // This avoids mistaking a not-yet-processed command for a passing test.
        const auto motor_id = config.endpoints[endpoint].motor_ids.front();
        const auto barrier = make_downlink(
            0, 0x600U + motor_id, {0x40, 0x41, 0x60, 0, 0, 0, 0, 0}, sequence++);
        if (!hosts[endpoint].send_to(peer, barrier.data(), barrier.size(), &error))
            return require(false, error);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline)
        {
            std::array<std::uint8_t, common::kMcuMaxDatagramSize + 1> bytes{};
            std::size_t size = 0;
            if (!hosts[endpoint].receive(bytes.data(), bytes.size(), size) ||
                size < common::kMcuGptpSize)
                continue;
            common::McuEnvelope envelope;
            common::CanFdFrame reply;
            if (common::decode_mcu_downlink(
                    bytes.data(), size - common::kMcuGptpSize, envelope) &&
                common::decode_can_fd_payload(
                    envelope.payload.data(), envelope.payload.size(), reply) &&
                reply.can_id == 0x580U + motor_id && reply.data.size() == 8 &&
                reply.data[0] == 0x4B && reply.data[1] == 0x41 && reply.data[2] == 0x60)
                return true;
        }
        return require(false, "Timed out acknowledging torso brake test command.");
    };

    // Match the recorded robot: lower five axes in MIT, three neck axes in CSP.
    std::array<std::vector<std::uint8_t>, 2> commands{
        std::vector<std::uint8_t>(64, 0), std::vector<std::uint8_t>(64, 0)};
    for (std::size_t i = 0; i < 5; ++i)
        set_mit_torque_slot(commands[0], i, config.motor_ids[i], 1.0);
    for (std::size_t i = 5; i < 8; ++i)
        set_csp_slot(commands[1], i - 5, config.motor_ids[i], 0.05);
    const auto send_group = [&](std::size_t endpoint) {
        return send(endpoint, endpoint == 0 ? 0x210U : 0x200U, commands[endpoint]);
    };
    if (!expect_brakes("Power on")) return false;
    const std::array<std::size_t, 2> order{
        lower_first ? 0U : 1U, lower_first ? 1U : 0U};
    for (std::size_t endpoint : order)
    {
        for (std::size_t i = 0; i < expected.size(); ++i)
            if ((i < 5) == (endpoint == 0)) expected[i] = false;
        if (!send_group(endpoint) || !expect_brakes("Mixed-mode enable")) return false;
    }
    for (int cycle = 0; cycle < 3; ++cycle)
    {
        for (std::size_t endpoint : order)
            if (!send_group(endpoint) || !expect_brakes("Alternating MIT/CSP")) return false;
    }

    std::vector<std::uint8_t> single(64, 0);
    // Disable remains effective even if the release bit stays set.
    set_mit_torque_slot(single, 0, 1, 1.0);
    single[0] = 0x4C;
    expected[0] = true;
    if (!send(0, 0x210, single) || !expect_brakes("Disable MIT axis") ||
        !send_group(1) || !expect_brakes("CSP must not release disabled MIT axis"))
        return false;
    expected[0] = false;
    if (!send_group(0) || !expect_brakes("Re-enable MIT axis")) return false;

    single.assign(64, 0);
    set_csp_slot(single, 0, 6, 0.05);
    single[0] = 0x86;  // Enabled, CSP, explicit brake lock.
    expected[5] = true;
    if (!send(1, 0x200, single) || !expect_brakes("Explicit CSP brake") ||
        !send_group(0) || !expect_brakes("MIT must preserve CSP brake command"))
        return false;
    expected[5] = false;
    if (!send_group(1) || !expect_brakes("Release CSP brake")) return false;

    // Mode changes affect only the addressed axis, even within one endpoint.
    single.assign(64, 0);
    set_csp_slot(single, 0, 5, 0.0);
    if (!send(0, 0x200, single) || !expect_brakes("MIT to CSP on one axis"))
        return false;
    set_mit_torque_slot(single, 0, 5, 1.0);
    if (!send(0, 0x210, single) || !expect_brakes("CSP to MIT on one axis"))
        return false;
    for (std::uint8_t control : {0xEC, 0xC8}) // Fault reset, unsupported mode.
    {
        single[0] = control;
        expected[4] = true;
        if (!send(0, 0x210, single) || !expect_brakes("Non-ready axis must stay locked") ||
            !send_group(1) || !expect_brakes("Other endpoint must preserve non-ready axis"))
            return false;
        expected[4] = false;
        if (!send_group(0) || !expect_brakes("Restore ready axis")) return false;
    }

    // A supported mode in the wrong frame format is still an invalid command.
    // Another motor's valid frame must not make that command ready.
    for (bool mit_frame : {true, false})
    {
        single.assign(64, 0);
        if (mit_frame) set_mit_torque_slot(single, 0, 5, 1.0);
        else set_csp_slot(single, 0, 5, 0.0);
        single[0] = mit_frame ? 0xC6 : 0xCC;
        expected[4] = true;
        if (!send(0, mit_frame ? 0x210 : 0x200, single) ||
            !expect_brakes("Frame/mode mismatch must stay locked") ||
            !send_group(1) || !expect_brakes("CSP must preserve rejected command lock"))
            return false;
        expected[4] = false;
        if (!send_group(0) || !expect_brakes("Restore valid command")) return false;
    }

    // The parallel waist must still lock both joints if either motor is disabled.
    single.assign(64, 0);
    set_mit_torque_slot(single, 0, config.motor_ids[2], 1.0);
    single[0] = 0x4C;
    expected[2] = expected[3] = true;
    if (!send(0, 0x210, single) || !expect_brakes("Coupled waist disable") ||
        !send_group(1) || !expect_brakes("CSP must preserve coupled waist lock"))
        return false;
    expected[2] = expected[3] = false;
    if (!send_group(0) || !expect_brakes("Coupled waist re-enable") ||
        !runtime.step(1, &error) || !expect_brakes("Mixed-mode physics step"))
        return false;
    return true;
}

bool test_rotor_feedback_units(bool remap_ids = false, double feedback_delay = 0.0)
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error)) return require(false, error);
    std::array<common::UdpSocket, 2> hosts;
    torso::TorsoPluginConfig config;
    config.feedback_delay_s.fill(feedback_delay);
    if (remap_ids)
    {
        // Units follow physical CAN IDs, not logical slots or endpoints.
        std::swap(config.motor_ids[0], config.motor_ids[5]);
        config.endpoints[0].motor_ids = {6, 2, 4, 3, 5};
        config.endpoints[1].motor_ids = {1, 7, 8};
        config.feedback_velocity_scale[0] = 0.5;
    }
    for (std::size_t i = 0; i < 2; ++i)
    {
        common::UdpSocket probe;
        if (!hosts[i].open_and_bind({"127.0.0.1", 0}, 100, &error) ||
            !probe.open_and_bind({"127.0.0.1", 0}, 100, &error)) return require(false, error);
        config.endpoints[i].bind_port = probe.local_port();
        config.endpoints[i].peer_port = hosts[i].local_port();
    }
    torso::TorsoPlugin plugin(config);
    if (!plugin.start(runtime, &error)) return require(false, error);
    const auto model = runtime.model_info();
    const std::array<const char*, 8> names{
        "lower_body_joint_1", "lower_body_joint_2", "lower_body_joint_3",
        "lower_body_joint_4", "lower_body_joint_5", "neck_joint_1",
        "neck_joint_2", "neck_joint_3"};
    std::array<const JointInfo*, 8> joints{};
    for (std::size_t axis = 0; axis < joints.size(); ++axis)
    {
        joints[axis] = find_joint(model, names[axis]);
        if (!require(joints[axis], "Missing feedback test joint.")) return false;
    }
    torso::ParallelWaistKinematics waist;
    const std::array<double, 8> base_velocity{0.3, -0.4, 0.2, -0.15, -0.25, -0.3, 0.4, -0.2};
    for (double direction : {1.0, -1.0, 0.0})
    {
        const std::array<double, 8> position{
            0.2, -0.2, direction < 0 ? 0.6 : 0.3,
            direction < 0 ? -0.2 : 0.1, 0.2, -0.2, 0.1, 0.1};
        std::array<double, 8> velocity{};
        for (std::size_t axis = 0; axis < velocity.size(); ++axis)
            velocity[axis] = direction * base_velocity[axis];
        if (!runtime.write_native([&](const mjModel*, mjData* data) {
                for (std::size_t axis = 0; axis < joints.size(); ++axis)
                {
                    data->qpos[joints[axis]->qpos_address] = position[axis];
                    data->qvel[joints[axis]->dof_address] = velocity[axis];
                }
            }, true, &error)) return require(false, error);

        std::array<double, 8> motor_position{};
        std::array<double, 8> motor_velocity{};
        for (std::size_t axis = 0; axis < joints.size(); ++axis)
        {
            motor_position[axis] = config.motor_signs[axis] * position[axis];
            motor_velocity[axis] = config.motor_signs[axis] * velocity[axis];
        }
        // Obtain waist motor speed from position differences, independently of
        // the analytic Jacobian used by the simulator's feedback path.
        constexpr double dt = 1e-5;
        std::array<double, 2> waist_position{}, before{}, after{};
        if (!waist.joint_to_motor({position[2], position[3]}, waist_position) ||
            !waist.joint_to_motor({position[2] - velocity[2] * dt,
                                  position[3] - velocity[3] * dt}, before) ||
            !waist.joint_to_motor({position[2] + velocity[2] * dt,
                                  position[3] + velocity[3] * dt}, after))
            return require(false, "Feedback test waist pose is unreachable.");
        for (std::size_t row = 0; row < 2; ++row)
        {
            motor_position[2 + row] = waist_position[row];
            motor_velocity[2 + row] = (after[row] - before[row]) / (2 * dt);
        }

        std::array<double, 8> decoded_motor_velocity{};
        for (std::size_t endpoint = 0; endpoint < 2; ++endpoint)
        {
            std::array<bool, 8> matched{};
            const auto complete = [&] {
                return std::all_of(config.endpoints[endpoint].motor_ids.begin(),
                                   config.endpoints[endpoint].motor_ids.end(),
                                   [&](std::uint8_t id) { return matched[id - 1]; });
            };
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            while (!complete() && std::chrono::steady_clock::now() < deadline)
            {
                std::array<std::uint8_t, 2048> bytes{};
                std::size_t size = 0;
                if (!hosts[endpoint].receive(bytes.data(), bytes.size(), size) || size < 44) continue;
                const auto payload_size = common::read_be16(bytes.data() + 22);
                if (size != common::kMcuEnvelopeSize + payload_size + common::kMcuCrcSize + common::kMcuGptpSize ||
                    common::read_be32(bytes.data() + common::kMcuEnvelopeSize + payload_size) !=
                        common::mcu_crc32(bytes.data() + 4, common::kMcuEnvelopeSize - 4 + payload_size)) continue;
                common::CanFdFrame frame;
                if (!common::decode_can_fd_payload(bytes.data() + common::kMcuEnvelopeSize, payload_size, frame) ||
                    frame.can_id < 0x301U || frame.can_id > 0x308U || frame.data.size() != 16) continue;
                const auto id = static_cast<std::uint8_t>(frame.can_id - 0x300U);
                const auto slot = std::find(config.motor_ids.begin(), config.motor_ids.end(), id);
                const auto axis = static_cast<std::size_t>(slot - config.motor_ids.begin());
                const auto& p = frame.data;
                const auto raw = static_cast<std::int16_t>(common::read_be16(p.data() + 2));
                std::int32_t encoder = (std::uint32_t(p[12]) << 16) | (std::uint32_t(p[13]) << 8) | p[14];
                if (encoder & 0x800000) encoder -= 0x1000000;
                // Platform normalizes lower IDs from 0.01 rad/s to RPM, then
                // converts RPM to output rad/s using the physical reduction.
                const double rotor_rad_s_per_lsb = id <= 5 ? 0.01 : kPi / 30;
                const double decoded_rpm = id <= 5 ? raw * 0.01 * 30 / kPi : raw;
                const double decoded_rotor_velocity = decoded_rpm * kPi / 30;
                const double ratio = config.motor_reduction_ratios[axis];
                const double scale = config.feedback_velocity_scale[axis];
                if (std::abs(decoded_rotor_velocity - motor_velocity[axis] * ratio * scale) >
                        rotor_rad_s_per_lsb * 0.5001 ||
                    encoder != std::lround(motor_position[axis] * ratio * 65536 / (2 * kPi))) continue;
                decoded_motor_velocity[axis] = decoded_rotor_velocity / (ratio * scale);
                matched[id - 1] = true;
            }
            if (!require(complete(), "Eight-axis rotor velocity/encoder feedback disagrees with physical units.")) return false;
        }
        std::array<std::array<double, 2>, 2> jacobian{};
        if (!waist.motor_jacobian({position[2], position[3]}, jacobian)) return false;
        const double determinant = jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0];
        const double decoded_pitch_velocity =
            (jacobian[1][1] * decoded_motor_velocity[2] - jacobian[0][1] * decoded_motor_velocity[3]) / determinant;
        const double decoded_roll_velocity =
            (-jacobian[1][0] * decoded_motor_velocity[2] + jacobian[0][0] * decoded_motor_velocity[3]) / determinant;
        if (!require(std::abs(decoded_pitch_velocity - velocity[2]) < 0.002 &&
                     std::abs(decoded_roll_velocity - velocity[3]) < 0.002,
                     "Decoded parallel waist joint velocity disagrees with position differences.")) return false;
    }
    return true;
}

bool test_torso_command_delay(bool csp = false)
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    using namespace std::chrono_literals;
    Runtime runtime;
    std::string error;
    if (!runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error)) return require(false,error);
    torso::TorsoPluginConfig config;
    config.command_delay_s[5]=0.08;
    std::array<common::UdpSocket,2> hosts;
    for(std::size_t i=0;i<2;++i)
    {
        common::UdpSocket probe;
        if(!hosts[i].open_and_bind({"127.0.0.1",0},10,&error) ||
           !probe.open_and_bind({"127.0.0.1",0},10,&error)) return require(false,error);
        config.endpoints[i].bind_port=probe.local_port();
        config.endpoints[i].peer_port=hosts[i].local_port();
    }
    {
        auto invalid=config;invalid.command_delay_s[2]=0.02;
        torso::TorsoPlugin rejected(invalid);
        if(!require(!rejected.start(runtime,&error),"Unequal waist delays accepted.")) return false;
    }
    {
        auto invalid=config;invalid.feedback_delay_s[0]=0.101;
        torso::TorsoPlugin rejected(invalid);
        if(!require(!rejected.start(runtime,&error),"Out-of-range torso delay accepted.")) return false;
    }
    torso::TorsoPlugin plugin(config);
    if(!plugin.start(runtime,&error)) return require(false,error);
    const auto model=runtime.model_info();
    const auto* actuator=find_actuator(model,"neck_joint_1_position");
    if(!require(actuator!=nullptr,"Missing neck actuator.")) return false;
    const auto* joint=find_joint(model,"neck_joint_1");
    if(!joint) return false;
    double position_before=0, velocity_before=0;
    const auto step = [&] {
        const auto previous=runtime.latest_snapshot();
        position_before=previous.qpos[joint->qpos_address];
        velocity_before=previous.qvel[joint->dof_address];
        return runtime.step(1,&error);
    };
    std::uint32_t sequence=1;
    const auto send = [&](double torque, bool enable=true) {
        std::vector<std::uint8_t> data(64,0);
        if(csp) set_csp_slot(data,0,6,torque*0.01);
        else set_mit_torque_slot(data,0,6,torque);
        if(!enable) data[0]=0; // An invalid-mode slot still carries disable bits.
        const auto packet=make_downlink(0,csp ? 0x200 : 0x210,data,sequence++);
        if(!hosts[1].send_to({"127.0.0.1",config.endpoints[1].bind_port},packet.data(),packet.size(),&error)) return false;
        std::this_thread::sleep_for(5ms);
        return step();
    };
    const auto expect = [&](double torque,bool enabled=true) {
        const double quantized=std::round((torque+300)/600*4095)/4095*600-300;
        const double target=position_count(torque*0.01)*kPi/32768;
        const double expected=!enabled ? 0 : csp ?
            config.csp_kp[5]*(target-position_before)-config.csp_kd[5]*velocity_before :
            300*std::tanh(quantized/300);
        return require(std::abs(runtime.latest_snapshot().controls[actuator->id]-expected)<1e-6,
                       "Torso delayed command / immediate disable mismatch.");
    };
    if(!test::wait_for_mcu_feedback(hosts[1],{"127.0.0.1",config.endpoints[1].bind_port},error,0)) return require(false,error);
    if(!send(1) || !expect(1) || !send(2) || !expect(1)) return false;
    std::this_thread::sleep_for(100ms);
    if(!step() || !expect(2)) return false;
    if(!send(3) || !expect(2) || !send(0,false) || !expect(0,false)) return false;
    std::this_thread::sleep_for(100ms);
    if(!step() || !expect(0,false)) return false;
    if(!send(1) || !send(4) || !runtime.reset(&error)) return false;
    std::this_thread::sleep_for(100ms);
    if(!step() || !expect(0,false)) return false;
    if (!send(1) || !expect(1) || !runtime.start(&error)) return false;
    const auto before = runtime.latest_snapshot().step_index;
    // Exercise receive-side brake updates concurrently with delayed delivery.
    for (int i=0; i<30; ++i)
    {
        std::vector<std::uint8_t> data(64,0);
        if(csp) set_csp_slot(data,0,6,0.01);
        else set_mit_torque_slot(data,0,6,1.0);
        const auto packet=make_downlink(0,csp ? 0x200 : 0x210,data,sequence++);
        if(!hosts[1].send_to({"127.0.0.1",config.endpoints[1].bind_port},packet.data(),packet.size(),&error)) return false;
        std::this_thread::sleep_for(5ms);
    }
    runtime.pause();
    return require(runtime.latest_snapshot().step_index > before + 20,
                   "Concurrent command delay stalled the physics thread.");
}

bool test_dual_endpoint_poc2_mapping()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;

    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error), error))
    {
        return false;
    }
    const ModelInfo model = runtime.model_info();
    const ActuatorInfo* axis1 =
        find_actuator(model, "lower_body_joint_1_position");
    const ActuatorInfo* axis6 =
        find_actuator(model, "neck_joint_1_position");
    const ActuatorInfo* axis7 =
        find_actuator(model, "neck_joint_2_position");
    const ActuatorInfo* axis8 =
        find_actuator(model, "neck_joint_3_position");
    const JointInfo* neck_axis1 = find_joint(model, "neck_joint_1");
    const JointInfo* neck_axis2 = find_joint(model, "neck_joint_2");
    const JointInfo* neck_axis3 = find_joint(model, "neck_joint_3");
    const JointInfo* waist_pitch = find_joint(model, "lower_body_joint_3");
    const JointInfo* waist_roll = find_joint(model, "lower_body_joint_4");
    if (!require(
            axis1 && axis6 && axis7 && axis8 && neck_axis1 && neck_axis2 &&
                neck_axis3 &&
                waist_pitch && waist_roll,
            "POC2 torso joints or actuators are missing."))
    {
        return false;
    }

    common::UdpSocket lower_host;
    common::UdpSocket upper_host;
    common::UdpSocket lower_probe;
    common::UdpSocket upper_probe;
    if (!lower_host.open_and_bind({"127.0.0.1", 0}, 100, &error) ||
        !upper_host.open_and_bind({"127.0.0.1", 0}, 100, &error) ||
        !lower_probe.open_and_bind({"127.0.0.1", 0}, 100, &error) ||
        !upper_probe.open_and_bind({"127.0.0.1", 0}, 100, &error))
    {
        return require(false, error);
    }

    torso::TorsoPluginConfig config;
    config.mit_torque_ff_scale[5] = 0.5;
    config.mit_torque_limit_nm[5] = 0.5;
    config.mit_torque_time_constant_s[5] = 0.02;
    config.mit_reference_velocity_limit_rad_s[6] = 0.1;
    config.mit_reference_acceleration_limit_rad_s2[6] = 1.0;
    config.command_velocity_filter_s[7] = 0.03;
    config.mit_kp_scale[7] = 0.0;
    config.endpoints[0].bind_port = lower_probe.local_port(&error);
    config.endpoints[0].peer_port = lower_host.local_port(&error);
    config.endpoints[1].bind_port = upper_probe.local_port(&error);
    config.endpoints[1].peer_port = upper_host.local_port(&error);
    lower_probe.close();
    upper_probe.close();

    DevicePluginManager manager;
    if (!manager.add(std::make_unique<torso::TorsoPlugin>(config), &error) ||
        !manager.start_all(runtime, &error))
    {
        return require(false, error);
    }
    if (!require(test::wait_for_mcu_feedback(
                     lower_host, {"127.0.0.1", config.endpoints[0].bind_port}, error, 0),
                 error) ||
        !require(test::wait_for_mcu_feedback(
                     upper_host, {"127.0.0.1", config.endpoints[1].bind_port}, error, 0),
                 error))
    {
        return false;
    }
    const PoseSnapshot initial_snapshot = runtime.latest_snapshot();

    std::vector<std::uint8_t> lower_csp(64, 0);
    std::vector<std::uint8_t> upper_csp(64, 0);
    set_csp_slot(lower_csp, 0, 1, -0.25);
    set_csp_slot(upper_csp, 0, 6, 0.10);
    const auto lower_packet = make_downlink(0, 0x200, lower_csp, 1);
    const auto upper_packet = make_downlink(0, 0x200, upper_csp, 1);
    if (!lower_host.send_to(
            {"127.0.0.1", config.endpoints[0].bind_port},
            lower_packet.data(), lower_packet.size(), &error) ||
        !upper_host.send_to(
            {"127.0.0.1", config.endpoints[1].bind_port},
            upper_packet.data(), upper_packet.size(), &error))
    {
        return require(false, error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!runtime.step(1, &error))
    {
        return require(false, error);
    }
    std::array<double, 2> waist_armature{};
    if (!runtime.read_native(
            [&](const mjModel* native_model, const mjData*) {
                waist_armature = {
                    native_model->dof_armature[waist_pitch->dof_address],
                    native_model->dof_armature[waist_roll->dof_address],
                };
            },
            &error))
    {
        return require(false, error);
    }
    const PoseSnapshot snapshot = runtime.latest_snapshot();
    const double axis1_target =
        static_cast<double>(position_count(-0.25)) * kPi / -32768.0;
    const double axis6_target =
        static_cast<double>(position_count(0.10)) * kPi / 32768.0;
    const double expected_axis1_torque =
        config.csp_kp[0] *
            (axis1_target - initial_snapshot.qpos[
                        static_cast<std::size_t>(
                            find_joint(model, "lower_body_joint_1")->qpos_address)]) -
        config.csp_kd[0] *
            initial_snapshot.qvel[static_cast<std::size_t>(
                find_joint(model, "lower_body_joint_1")->dof_address)];
    const double expected_axis6_torque =
        config.csp_kp[5] *
            (axis6_target - initial_snapshot.qpos[
                        static_cast<std::size_t>(neck_axis1->qpos_address)]) -
        config.csp_kd[5] *
            initial_snapshot.qvel[
                static_cast<std::size_t>(neck_axis1->dof_address)];
    const bool mapping_ok =
        require(
            std::abs(
                snapshot.controls[static_cast<std::size_t>(axis1->id)] -
                expected_axis1_torque) <
                1e-3,
            "POC2 CSP controller torque for lower endpoint is wrong.") &&
        require(
            std::abs(
                snapshot.controls[static_cast<std::size_t>(axis6->id)] -
                expected_axis6_torque) <
                1e-3,
            "POC2 CSP controller torque for upper endpoint is wrong.") &&
        require(
            std::abs(waist_armature[0] - 3.294507262) < 1e-6 &&
                std::abs(waist_armature[1] - 2.028046842) < 1e-6,
            "Dynamic waist armature was not applied before stepping.");
    const bool endpoints_ok =
        require(
            test::wait_for_mcu_feedback(
                lower_host, {"127.0.0.1", config.endpoints[0].bind_port}, error, 0),
            error) &&
        require(
            test::wait_for_mcu_feedback(
                upper_host, {"127.0.0.1", config.endpoints[1].bind_port}, error, 0),
            error);

    std::vector<std::uint8_t> upper_mit(64, 0);
    set_mit_torque_slot(upper_mit, 0, 6, 2.0);
    const auto mit_packet = make_downlink(0, 0x210, upper_mit, 2);
    if (!upper_host.send_to(
            {"127.0.0.1", config.endpoints[1].bind_port},
            mit_packet.data(),
            mit_packet.size(),
            &error))
    {
        return require(false, error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!runtime.step(1, &error))
    {
        return require(false, error);
    }
    const PoseSnapshot mit_snapshot = runtime.latest_snapshot();
    const double quantized_mit_torque =
        static_cast<double>(
            std::lround((2.0 + 300.0) / 600.0 * 4095.0)) /
            4095.0 * 600.0 -
        300.0;
    const double scaled_mit_torque =
        config.mit_torque_ff_scale[5] * quantized_mit_torque;
    const double expected_mit_torque =
        config.mit_torque_limit_nm[5] * std::tanh(
            scaled_mit_torque / config.mit_torque_limit_nm[5]);
    const double torque_filter_alpha =
        -std::expm1(-model.timestep / config.mit_torque_time_constant_s[5]);
    const bool mit_ok = require(
        std::abs(
            mit_snapshot.controls[static_cast<std::size_t>(axis6->id)] -
            torque_filter_alpha * expected_mit_torque) <
            1e-6,
        "POC2 MIT torque saturation or finite-bandwidth response was not applied.");

    std::vector<std::uint8_t> profile_mit(64, 0);
    set_mit_slot(profile_mit, 0, 7, 0.0, 0.0, 100.0, 10.0, 0.0);
    auto profile_packet = make_downlink(0, 0x210, profile_mit, 3);
    if (!upper_host.send_to(
            {"127.0.0.1", config.endpoints[1].bind_port},
            profile_packet.data(),
            profile_packet.size(),
            &error))
    {
        return require(false, error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!runtime.step(1, &error))
    {
        return require(false, error);
    }
    set_mit_slot(profile_mit, 0, 7, 1.0, 0.0, 100.0, 10.0, 0.0);
    profile_packet = make_downlink(0, 0x210, profile_mit, 4);
    if (!upper_host.send_to(
            {"127.0.0.1", config.endpoints[1].bind_port},
            profile_packet.data(),
            profile_packet.size(),
            &error))
    {
        return require(false, error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!runtime.step(1, &error))
    {
        return require(false, error);
    }
    const PoseSnapshot profile_snapshot = runtime.latest_snapshot();
    const double profile_torque =
        profile_snapshot.controls[static_cast<std::size_t>(axis7->id)];
    const bool profile_ok = require(
        profile_torque > 0.0 && profile_torque < 1.0,
        "POC2 MIT model-time reference did not smooth a command burst.");

    std::vector<std::uint8_t> velocity_mit(64, 0);
    set_mit_slot(velocity_mit, 0, 8, 0.0, 0.0, 0.0, 10.0, 0.0);
    auto velocity_packet = make_downlink(0, 0x210, velocity_mit, 5);
    if (!upper_host.send_to(
            {"127.0.0.1", config.endpoints[1].bind_port},
            velocity_packet.data(), velocity_packet.size(), &error))
    {
        return require(false, error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    set_mit_slot(velocity_mit, 0, 8, 0.1, 0.0, 0.0, 10.0, 0.0);
    velocity_packet = make_downlink(0, 0x210, velocity_mit, 6);
    if (!upper_host.send_to(
            {"127.0.0.1", config.endpoints[1].bind_port},
            velocity_packet.data(), velocity_packet.size(), &error))
    {
        return require(false, error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const auto reset_velocity_joint = [&]() {
        return runtime.write_native(
            [&](const mjModel*, mjData* data) {
                data->qpos[neck_axis3->qpos_address] = 0.0;
                data->qvel[neck_axis3->dof_address] = 0.0;
            },
            true,
            &error);
    };
    if (!reset_velocity_joint() || !runtime.step(1, &error))
    {
        return require(false, error);
    }
    const double first_velocity_torque =
        runtime.latest_snapshot().controls[static_cast<std::size_t>(axis8->id)];
    for (std::uint32_t sequence = 7; sequence < 17; ++sequence)
    {
        velocity_packet = make_downlink(0, 0x210, velocity_mit, sequence);
        if (!upper_host.send_to(
                {"127.0.0.1", config.endpoints[1].bind_port},
                velocity_packet.data(), velocity_packet.size(), &error))
        {
            return require(false, error);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!reset_velocity_joint() || !runtime.step(1, &error))
    {
        return require(false, error);
    }
    const double repeated_velocity_torque =
        runtime.latest_snapshot().controls[static_cast<std::size_t>(axis8->id)];
    const bool repeated_velocity_ok = require(
        first_velocity_torque > 1.0 &&
            std::abs(repeated_velocity_torque - first_velocity_torque) < 0.05,
        "Repeated MIT position holds changed the estimated command velocity.");

    std::vector<std::uint8_t> upper_disabled(64, 0);
    set_csp_slot(upper_disabled, 0, 6, 0.0, false);
    auto disabled_packet = make_downlink(0, 0x200, upper_disabled, 2);
    if (!upper_host.send_to(
            {"127.0.0.1", config.endpoints[1].bind_port},
            disabled_packet.data(),
            disabled_packet.size(),
            &error))
    {
        return require(false, error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!runtime.step(1, &error) ||
        !runtime.write_native(
            [&](const mjModel*, mjData* data) {
                data->qpos[neck_axis1->qpos_address] = 0.30;
            },
            true,
            &error))
    {
        return require(false, error);
    }
    disabled_packet = make_downlink(0, 0x200, upper_disabled, 3);
    if (!upper_host.send_to(
            {"127.0.0.1", config.endpoints[1].bind_port},
            disabled_packet.data(),
            disabled_packet.size(),
            &error))
    {
        return require(false, error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!runtime.step(1, &error))
    {
        return require(false, error);
    }
    const PoseSnapshot disabled_snapshot = runtime.latest_snapshot();
    const bool disabled_hold_ok = require(
        std::abs(disabled_snapshot.controls[static_cast<std::size_t>(axis6->id)]) <
            1e-9,
        "Disabled torso motor produced non-zero torque.");

    manager.stop_all();
    runtime.unload();
    return mapping_ok && endpoints_ok && mit_ok && profile_ok &&
           repeated_velocity_ok &&
           disabled_hold_ok;
}

}  // namespace

int main()
{
    if (!test_manager_rollback() || !test_socketcan_validation() ||
        !test_mixed_mode_brakes(true) || !test_mixed_mode_brakes(false) ||
        !test_mixed_mode_brakes(true, 0.04) || !test_mixed_mode_brakes(false, 0.04) ||
        !test_rotor_feedback_units() || !test_rotor_feedback_units(true) ||
        !test_rotor_feedback_units(false, 0.04) || !test_torso_command_delay() ||
        !test_torso_command_delay(true) ||
        !test_dual_endpoint_poc2_mapping())
    {
        return EXIT_FAILURE;
    }
    std::cout << "Device lifecycle, dual endpoint, and POC2 mapping tests passed.\n";
    return EXIT_SUCCESS;
}
