// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "mcu_udp_test_helpers.h"

#include "sharpa_mujoco/plugins/chassis/chassis_plugin.h"
#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/runtime.h"
#include "sharpa_mujoco/scene_builder.h"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

namespace
{

bool require(bool condition, const std::string& message)
{
    if (!condition) std::cerr << "FAILED: " << message << '\n';
    return condition;
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

bool configure_loopback_endpoints(
    sharpa::mujoco::plugins::chassis::ChassisPluginConfig& config,
    sharpa::mujoco::plugins::common::UdpSocket& jiechang_host,
    sharpa::mujoco::plugins::common::UdpSocket& m42p_host,
    std::string* error)
{
    using sharpa::mujoco::plugins::common::UdpSocket;
    UdpSocket jiechang_probe;
    UdpSocket m42p_probe;
    if (!jiechang_host.open_and_bind({"127.0.0.1", 0}, 20, error) ||
        !m42p_host.open_and_bind({"127.0.0.1", 0}, 20, error) ||
        !jiechang_probe.open_and_bind({"127.0.0.1", 0}, 20, error) ||
        !m42p_probe.open_and_bind({"127.0.0.1", 0}, 20, error))
    {
        return false;
    }
    config.jiechang_endpoint.bind_port =
        jiechang_probe.local_port(error);
    config.jiechang_endpoint.peer_port =
        jiechang_host.local_port(error);
    config.m42p_endpoint.bind_port = m42p_probe.local_port(error);
    config.m42p_endpoint.peer_port = m42p_host.local_port(error);
    jiechang_probe.close();
    m42p_probe.close();
    return config.jiechang_endpoint.bind_port != 0 &&
           config.jiechang_endpoint.peer_port != 0 &&
           config.m42p_endpoint.bind_port != 0 &&
           config.m42p_endpoint.peer_port != 0;
}

bool plugin_start_matches(
    const std::string& model_path,
    bool expected_to_start,
    const std::string& expected_error)
{
    namespace sm = sharpa::mujoco;
    namespace plugins = sm::plugins;
    sm::Runtime runtime;
    std::string error;
    if (!require(runtime.load(model_path, &error), error))
    {
        return false;
    }

    plugins::common::UdpSocket jiechang_host;
    plugins::common::UdpSocket m42p_host;
    plugins::chassis::ChassisPluginConfig config;
    if (!require(
            configure_loopback_endpoints(
                config, jiechang_host, m42p_host, &error),
            error))
    {
        return false;
    }

    plugins::chassis::ChassisPlugin plugin(config);
    const bool started = plugin.start(runtime, &error);
    bool ok = require(
        started == expected_to_start,
        expected_to_start
            ? ("Rotated-root chassis plugin failed to start: " + error).c_str()
            : "Invalid chassis axis unexpectedly passed validation.");
    if (!expected_to_start)
    {
        ok = require(
                 error.find(expected_error) != std::string::npos,
                 ("Chassis validation error lacked joint detail: " + error).c_str()) &&
             ok;
    }
    plugin.stop();
    runtime.unload();
    return ok;
}

bool test_chassis_coordinate_contract_uses_robot_frame()
{
    namespace sm = sharpa::mujoco;
    const std::filesystem::path output_root =
        std::filesystem::temp_directory_path() /
        "sharpa_mujoco_chassis_coordinate_test";
    std::error_code filesystem_error;
    std::filesystem::remove_all(output_root, filesystem_error);
    filesystem_error.clear();
    std::filesystem::create_directories(output_root, filesystem_error);
    if (!require(
            !filesystem_error,
            ("Could not create chassis coordinate test directory: " +
             filesystem_error.message()).c_str()))
    {
        return false;
    }

    constexpr double yaw_radians = 0.604201879;
    sm::SceneDescription scene;
    scene.robot_xml_path = SHARPA_MUJOCO_NTH_HA1_MODEL;
    scene.robot_world_pose.position = {-7.6, 18.0, 0.03};
    scene.robot_world_pose.quaternion = {
        std::cos(yaw_radians * 0.5), 0.0, 0.0,
        std::sin(yaw_radians * 0.5)};
    sm::SceneBuildOptions options;
    options.output_xml_path = (output_root / "rotated-root.xml").string();
    std::string error;
    if (!require(sm::SceneBuilder::build(scene, options, &error), error) ||
        !plugin_start_matches(options.output_xml_path, true, {}))
    {
        return false;
    }

    std::ifstream rotated_input(options.output_xml_path);
    std::string invalid_xml(
        (std::istreambuf_iterator<char>(rotated_input)),
        std::istreambuf_iterator<char>());
    const std::string joint_marker =
        "name=\"chassis_steer_joint_1\"";
    const std::string valid_axis = "axis=\"0 0 1\"";
    const std::size_t joint_position = invalid_xml.find(joint_marker);
    const std::size_t axis_position = invalid_xml.find(valid_axis, joint_position);
    if (!require(
            joint_position != std::string::npos &&
                axis_position != std::string::npos,
            "Could not locate the chassis axis in the generated test scene."))
    {
        return false;
    }
    invalid_xml.replace(axis_position, valid_axis.size(), "axis=\"1 0 0\"");
    const std::filesystem::path invalid_path =
        output_root / "invalid-relative-axis.xml";
    std::ofstream invalid_output(invalid_path);
    invalid_output << invalid_xml;
    invalid_output.close();
    return plugin_start_matches(
        invalid_path.string(), false, "chassis_steer_joint_1");
}

std::vector<std::uint8_t> make_downlink(
    std::uint8_t can_interface,
    std::uint32_t can_id,
    const std::vector<std::uint8_t>& can_data,
    std::uint32_t sequence,
    std::uint8_t can_flags = sharpa::mujoco::plugins::common::kCanFdFlagBrs)
{
    using namespace sharpa::mujoco::plugins::common;
    CanFdFrame frame;
    frame.can_interface = can_interface;
    frame.can_id = can_id;
    frame.can_flags = can_flags;
    frame.data = can_data;
    const auto payload = encode_can_fd_payload(frame);
    std::vector<std::uint8_t> output(
        kMcuEnvelopeSize + payload.size() + kMcuCrcSize, 0);
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
        mcu_crc32(
            output.data() + 4, (kMcuEnvelopeSize - 4) + payload.size()));
    return output;
}

void set_pdo1(
    std::vector<std::uint8_t>& data,
    std::size_t axis,
    std::int8_t mode,
    bool brake_released,
    bool enabled,
    std::int16_t torque_permille)
{
    using namespace sharpa::mujoco::plugins::common;
    std::uint8_t* slot = data.data() + axis * 6;
    slot[0] = static_cast<std::uint8_t>(mode);
    slot[1] = brake_released ? 1 : 0;
    write_le16(slot + 2, enabled ? 0x000FU : 0U);
    write_le16(slot + 4, static_cast<std::uint16_t>(torque_permille));
}

void set_pdo2(
    std::vector<std::uint8_t>& data,
    std::size_t axis,
    std::int32_t position,
    std::int32_t velocity)
{
    using namespace sharpa::mujoco::plugins::common;
    std::uint8_t* slot = data.data() + axis * 8;
    write_le32(slot, static_cast<std::uint32_t>(position));
    write_le32(slot + 4, static_cast<std::uint32_t>(velocity));
}

bool send_datagram(
    sharpa::mujoco::plugins::common::UdpSocket& host,
    std::uint16_t plugin_port,
    const std::vector<std::uint8_t>& packet,
    std::string* error)
{
    std::string send_error;
    if (host.send_to(
            {"127.0.0.1", plugin_port}, packet.data(), packet.size(), &send_error))
    {
        return true;
    }
    const std::string message = "Chassis UDP send 127.0.0.1:" +
        std::to_string(host.local_port()) + " -> 127.0.0.1:" +
        std::to_string(plugin_port) + ": " + send_error;
    if (error) *error = message;
    return require(false, message);
}

bool send_cycle(
    sharpa::mujoco::plugins::common::UdpSocket& host,
    std::uint16_t plugin_port,
    std::uint8_t can_interface,
    const std::vector<std::uint8_t>& pdo1,
    const std::vector<std::uint8_t>& pdo2,
    std::uint32_t& sequence,
    std::string* error)
{
    const auto first = make_downlink(can_interface, 0x200, pdo1, sequence++);
    const auto second = make_downlink(can_interface, 0x300, pdo2, sequence++);
    return send_datagram(host, plugin_port, first, error) &&
           send_datagram(host, plugin_port, second, error);
}

bool receive_tpdo(
    sharpa::mujoco::plugins::common::UdpSocket& socket,
    std::uint32_t can_id,
    std::int16_t& torque,
    std::int16_t& current)
{
    using namespace sharpa::mujoco::plugins::common;
    std::array<std::uint8_t, 2048> buffer{};
    for (int attempt = 0; attempt < 500; ++attempt)
    {
        std::size_t size = 0;
        if (!socket.receive(buffer.data(), buffer.size(), size)) continue;
        if (size < kMcuEnvelopeSize + kMcuCrcSize + kMcuGptpSize ||
            read_be32(buffer.data()) != kMcuMagicPoc2 ||
            buffer[4] != kMcuProtocolVersion)
        {
            continue;
        }
        const std::uint16_t payload_size = read_be16(buffer.data() + 22);
        if (size !=
                kMcuEnvelopeSize + payload_size + kMcuCrcSize + kMcuGptpSize ||
            read_be32(buffer.data() + kMcuEnvelopeSize + payload_size) !=
                mcu_crc32(
                    buffer.data() + 4,
                    (kMcuEnvelopeSize - 4) + payload_size))
        {
            continue;
        }
        CanFdFrame frame;
        if (!decode_can_fd_payload(
                buffer.data() + kMcuEnvelopeSize, payload_size, frame) ||
            frame.can_id != can_id || frame.data.size() != 16)
        {
            continue;
        }
        torque = static_cast<std::int16_t>(
            read_le16(frame.data.data() + 12));
        current = static_cast<std::int16_t>(
            read_le16(frame.data.data() + 14));
        if (std::abs(static_cast<int>(torque)) >= 18000) return true;
    }
    return torque != 0 || current != 0;
}

bool test_chassis_modes_and_feedback()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;

    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_NTH_HA1_MODEL, &error), error))
    {
        return false;
    }
    const ModelInfo model = runtime.model_info();
    const ActuatorInfo* steer =
        find_actuator(model, "chassis_steer_joint_1_position");
    const ActuatorInfo* wheel =
        find_actuator(model, "chassis_wheel_joint_1_velocity");
    if (!require(steer && wheel, "NTH-HA1 chassis motor actuators are missing."))
    {
        return false;
    }

    common::UdpSocket jiechang_host;
    common::UdpSocket m42p_host;
    common::UdpSocket jiechang_probe;
    common::UdpSocket m42p_probe;
    if (!jiechang_host.open_and_bind({"127.0.0.1", 0}, 20, &error) ||
        !m42p_host.open_and_bind({"127.0.0.1", 0}, 20, &error) ||
        !jiechang_probe.open_and_bind({"127.0.0.1", 0}, 20, &error) ||
        !m42p_probe.open_and_bind({"127.0.0.1", 0}, 20, &error))
    {
        return require(false, error);
    }

    chassis::ChassisPluginConfig config;
    config.feedback_rate_hz = 500.0;
    config.jiechang_endpoint.bind_port =
        jiechang_probe.local_port(&error);
    config.jiechang_endpoint.peer_port =
        jiechang_host.local_port(&error);
    config.m42p_endpoint.bind_port = m42p_probe.local_port(&error);
    config.m42p_endpoint.peer_port = m42p_host.local_port(&error);
    jiechang_probe.close();
    m42p_probe.close();

    chassis::ChassisPlugin plugin(config);
    if (!plugin.start(runtime, &error))
    {
        return require(false, error);
    }

    if (!require(test::wait_for_mcu_feedback(
                     jiechang_host, {"127.0.0.1", config.jiechang_endpoint.bind_port}, error),
                 error)) return false;

    std::uint32_t sequence = 1;
    std::vector<std::uint8_t> pdo1(48, 0);
    std::vector<std::uint8_t> pdo2(64, 0);
    set_pdo1(pdo1, 0, 10, true, true, 1000);
    if (!send_cycle(
            jiechang_host, config.jiechang_endpoint.bind_port, 1,
            pdo1, pdo2, sequence, &error))
    {
        plugin.stop();
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!runtime.step(1, &error))
    {
        plugin.stop();
        return require(false, error);
    }
    PoseSnapshot snapshot = runtime.latest_snapshot();
    bool ok = require(
        std::abs(
            snapshot.controls[static_cast<std::size_t>(steer->id)] + 12.285) <
            1e-3,
        "Positive steer CST did not apply signed geared torque.");

    pdo1.assign(48, 0);
    pdo2.assign(64, 0);
    set_pdo1(pdo1, 4, 9, true, true, 0);
    set_pdo2(pdo2, 4, 0, -300);
    if (!send_cycle(
            jiechang_host, config.jiechang_endpoint.bind_port, 1,
            pdo1, pdo2, sequence, &error))
    {
        plugin.stop();
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    runtime.step(1, &error);
    snapshot = runtime.latest_snapshot();
    ok = require(
             snapshot.controls[static_cast<std::size_t>(wheel->id)] > 3.0 &&
                 snapshot.controls[static_cast<std::size_t>(wheel->id)] <= 3.14,
             "Wheel CSV did not apply velocity error torque.") &&
         ok;

    pdo1.assign(48, 0);
    set_pdo1(pdo1, 4, 10, true, true, 20000);
    if (!send_cycle(
            jiechang_host, config.jiechang_endpoint.bind_port, 1,
            pdo1, pdo2, sequence, &error))
    {
        plugin.stop();
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    runtime.step(10, &error);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::int16_t torque = 0;
    std::int16_t current = 0;
    const bool got_feedback = receive_tpdo(
        jiechang_host, 0x185U, torque, current);
    ok = require(got_feedback, "Wheel TPDO feedback was not received.") && ok;
    if (got_feedback)
    {
        ok = require(
                 std::abs(static_cast<int>(torque)) >= 900 &&
                     std::abs(static_cast<int>(torque)) <= 1100 &&
                     torque == current,
                 "TPDO torque/current did not reflect limited actuator force.") &&
             ok;
    }

    pdo1.assign(48, 0);
    set_pdo1(pdo1, 4, 10, true, false, 20000);
    if (!send_cycle(
            jiechang_host, config.jiechang_endpoint.bind_port, 1,
            pdo1, pdo2, sequence, &error))
    {
        plugin.stop();
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    runtime.step(1, &error);
    snapshot = runtime.latest_snapshot();
    ok = require(
             std::abs(
                 snapshot.controls[static_cast<std::size_t>(wheel->id)]) <
                 1e-12,
             "Disabled mode retained a previous wheel command.") &&
         ok;

    std::vector<std::uint8_t> pdo3(64, 0);
    set_pdo1(pdo1, 4, -1, true, true, 0);
    set_pdo2(pdo2, 4, 524288, 0);
    using plugins::common::write_le32;
    write_le32(pdo3.data() + 4 * 8, 10);
    write_le32(pdo3.data() + 4 * 8 + 4, 1);
    const auto mit_gains =
        make_downlink(1, 0x400, pdo3, sequence++);
    if (!send_datagram(jiechang_host, config.jiechang_endpoint.bind_port, mit_gains, &error) ||
        !send_cycle(jiechang_host, config.jiechang_endpoint.bind_port, 1,
                    pdo1, pdo2, sequence, &error)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    runtime.step(1, &error);
    snapshot = runtime.latest_snapshot();
    ok = require(
             std::abs(
                 snapshot.controls[static_cast<std::size_t>(wheel->id)]) >
                 0.1,
             "MIT mode did not produce feedback torque.") &&
         ok;

    // Platform uses two independent multiplex groups. Right-bus PDOs and
    // left-bus SYNC/malformed frames must not renew left motor commands.
    pdo1.assign(48, 0);
    pdo2.assign(64, 0);
    set_pdo1(pdo1, 4, 10, true, true, 500);
    set_pdo1(pdo1, 5, 10, true, true, 500);
    for (std::uint8_t bus : {0, 1})
        if (!send_cycle(jiechang_host, config.jiechang_endpoint.bind_port,
                        bus, pdo1, pdo2, sequence, &error)) return false;
    for (int cycle = 0; cycle < 10; ++cycle)
    {
        if (!send_cycle(jiechang_host, config.jiechang_endpoint.bind_port,
                        0, pdo1, pdo2, sequence, &error)) return false;
        for (std::uint32_t id : {0x080U, 0x300U})
        {
            const auto packet = make_downlink(1, id,
                id == 0x300U ? std::vector<std::uint8_t>(8, 0) :
                               std::vector<std::uint8_t>{}, sequence++);
            if (!send_datagram(jiechang_host, config.jiechang_endpoint.bind_port,
                               packet, &error)) return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    runtime.step(1, &error);
    snapshot = runtime.latest_snapshot();
    const auto* right_wheel = find_actuator(model, "chassis_wheel_joint_2_velocity");
    ok = require(right_wheel &&
                 std::abs(snapshot.controls[wheel->id]) < 1e-12 &&
                 snapshot.controls[right_wheel->id] > 1.0,
                 "CAN groups did not expire independently after left-bus timeout.") && ok;

    plugin.stop();
    runtime.unload();
    return ok;
}

bool test_jiechang_stretch_udp()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_NTH_HA1_MODEL, &error), error)) return false;
    common::UdpSocket host, unused_slider_host;
    chassis::ChassisPluginConfig config;
    config.motor_family = chassis::ChassisMotorFamily::Kinco;
    config.stretch_family = chassis::ChassisStretchFamily::Jiechang;
    config.slider_maximum_m = 0.06;
    config.feedback_rate_hz = 200;
    if (!require(configure_loopback_endpoints(config, host, unused_slider_host, &error), error)) return false;
    config.kinco_endpoint = config.jiechang_endpoint;
    chassis::ChassisPlugin plugin(config);
    if (!require(plugin.start(runtime, &error) && runtime.start(&error), error)) return false;
    if (!require(test::wait_for_mcu_feedback(
                     host, {"127.0.0.1", config.kinco_endpoint.bind_port}, error),
                 error)) return false;
    std::uint32_t sequence = 1;
    const auto send = [&](std::uint8_t bus, std::uint32_t id, const std::vector<std::uint8_t>& data) {
        const auto packet = make_downlink(bus, id, data, sequence++, 0);
        return send_datagram(host, config.kinco_endpoint.bind_port, packet, &error);
    };
    std::array<int, 4> actuators{};
    const ModelInfo model_info = runtime.model_info();
    for (std::size_t i = 0; i < 4; ++i)
    {
        const auto* actuator = find_actuator(model_info, config.axes[8 + i].actuator);
        if (!require(actuator != nullptr, "Missing stretch actuator.")) return false;
        actuators[i] = actuator->id;
        const auto node = static_cast<std::uint8_t>(9 + i);
        const auto bus = config.stretch_can_interfaces[i];
        if (!send(bus, 0, {1, node}) ||
            !send(bus, 0x600 + node, {0x23, 0x80, 0x60, 0, 0, 0, 0xc8, 0}) ||
            !send(bus, 0x200 + node, {0x0f, 0, 0, 0, 0x10, 0}) ||
            !send(bus, 0x80, {})) return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    auto snapshot = runtime.latest_snapshot();
    for (const int id : actuators)
        if (!require(std::abs(snapshot.controls[id] - 0.004) < 1e-5,
                     "Stretch CSP must convert 1048576 counts to 4 mm on all four motors.")) return false;

    // A command on the other CAN bus must not change StretchA's target.
    if (!send(1, 0x209, {0x0f, 0, 0, 0, 0x20, 0}) ||
        !send(1, 0x80, {})) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!require(std::abs(runtime.latest_snapshot().controls[actuators[0]] - 0.004) < 1e-5,
                 "Stretch command crossed CAN buses.")) return false;
    if (!send(0, 0x609, {0x2f, 0x60, 0x60, 0, 9, 0, 0, 0}) ||
        !send(0, 0x309, {0, 0, 0xc8, 0}) || // 50 mm/s.
        !send(0, 0x80, {})) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!require(runtime.latest_snapshot().controls[actuators[0]] > 0.007,
                 "Stretch CSV velocity conversion or brake release failed.")) return false;
    // SYNC and SDO health queries cannot refresh an expired motion command.
    for (int i = 0; i < 40; ++i)
    {
        if (!send(0, 0x80, {}) ||
            !send(0, 0x609, {0x40, 0x41, 0x60, 0, 0, 0, 0})) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    snapshot = runtime.latest_snapshot();
    int position_address = -1;
    runtime.read_native([&](const mjModel* model, const mjData*) {
        position_address = model->jnt_qposadr[mj_name2id(model, mjOBJ_JOINT, "chassis_slider_joint_1")];
    });
    const bool stopped = require(std::abs(snapshot.controls[actuators[0]] - snapshot.qpos[position_address]) < 0.001,
                                 "Stale stretch velocity remained active while health queries continued.");
    plugin.stop();
    runtime.pause();
    return stopped;
}

bool test_wheel_pi_lifecycle(bool kinco, bool in_step = false)
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_NTH_HA1_MODEL, &error), error)) return false;
    common::UdpSocket host, slider_host;
    chassis::ChassisPluginConfig config;
    config.feedback_rate_hz = 500;
    config.kinco_control_in_step = in_step;
    if (!require(configure_loopback_endpoints(config, host, slider_host, &error), error)) return false;
    if (kinco)
    {
        config.motor_family = chassis::ChassisMotorFamily::Kinco;
        config.kinco_endpoint = config.jiechang_endpoint;
    }
    chassis::ChassisPlugin plugin(config);
    if (!require(plugin.start(runtime, &error), error)) return false;
    if (!require(test::wait_for_mcu_feedback(
                     host, {"127.0.0.1", config.jiechang_endpoint.bind_port}, error),
                 error)) return false;
    std::uint32_t sequence = 1;
    auto send_frame = [&](std::uint8_t bus, std::uint32_t id, const std::vector<std::uint8_t>& data) {
        const auto wire = make_downlink(bus, id, data, sequence++, 0);
        return send_datagram(host, config.jiechang_endpoint.bind_port, wire, &error);
    };
    auto set_mode = [&](int mode) {
        if (!kinco) return true;
        for (std::size_t axis = 4; axis < 8; ++axis)
        {
            const auto node = static_cast<std::uint8_t>(axis + 1);
            if (!send_frame(config.motor_can_interfaces[axis], 0, {1, node}) ||
                !send_frame(config.motor_can_interfaces[axis], 0x600 + node,
                            {0x2f, 0x60, 0x60, 0, static_cast<std::uint8_t>(mode == 9 ? -3 : 4), 0, 0, 0})) return false;
        }
        return true;
    };
    auto send = [&](bool enabled, int mode = 9) {
        if (kinco)
        {
            for (std::size_t axis = 4; axis < 8; ++axis)
            {
                std::vector<std::uint8_t> pdo(6, 0);
                common::write_le16(pdo.data(), enabled ? 0x0f : 0x06);
                if (!send_frame(config.motor_can_interfaces[axis], 0x201 + axis, pdo)) return false;
            }
            return true;
        }
        std::vector<std::uint8_t> pdo1(48, 0), pdo2(64, 0);
        for (int wheel = 0; wheel < 4; ++wheel) set_pdo1(pdo1, wheel + 4, mode, enabled, true, 0);
        for (const auto bus : {0, 1})
            if (!send_cycle(host, config.jiechang_endpoint.bind_port, bus, pdo1, pdo2, sequence, &error)) return false;
        return true;
    };
    const auto model_info = runtime.model_info();
    const auto* wheel = find_actuator(model_info, config.axes[4].actuator);
    if (!require(wheel != nullptr, "Missing wheel for PI test.")) return false;
    const int actuator = wheel->id;
    double simulated_time = 0.0;
    auto sample = [&](double speed) {
        simulated_time += 0.01;
        if (!runtime.write_native([&](const mjModel* model, mjData* data) {
                data->time = simulated_time;
                for (int i = 0; i < model->nv; ++i) data->qvel[i] = 0.0;
                const int joint = mj_name2id(model, mjOBJ_JOINT, config.axes[4].joint.c_str());
                data->qvel[model->jnt_dofadr[joint]] = speed;
            }, true, &error)) return false;
        if (!send(true)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        return runtime.step(1, &error);
    };
    auto accumulate = [&]() {
        for (int n = 0; n < 12; ++n) if (!sample(0.02)) return false;
        return sample(0.0) && require(runtime.latest_snapshot().controls[actuator] < -0.1,
                                     "UDP velocity mode did not retain opposing PI torque at zero speed.");
    };
    if (!set_mode(9) || !accumulate()) return false;
    if (!send(false)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(12));
    if (!sample(0.0) || !require(std::abs(runtime.latest_snapshot().controls[actuator]) < 1e-8,
                               "Disable/re-enable retained PI torque.")) return false;
    if (!accumulate() || !set_mode(10) || !send(true, 10)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(12));
    if (!set_mode(9) || !sample(0.0) || !require(std::abs(runtime.latest_snapshot().controls[actuator]) < 1e-8,
                                              "Mode switch retained PI torque.")) return false;
    if (!accumulate()) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    if (!sample(0.0) || !require(std::abs(runtime.latest_snapshot().controls[actuator]) < 1e-8,
                               "Command timeout retained PI torque.")) return false;
    if (!accumulate() || !runtime.reset(&error) || !sample(0.0) ||
        !require(std::abs(runtime.latest_snapshot().controls[actuator]) < 1e-8,
                 "Runtime reset retained PI torque.")) return false;
    plugin.stop();
    std::cout << (kinco ? "Kinco" : "Jiechang") << " PI lifecycle UDP checks passed.\n";
    return true;
}

bool test_kinco_homing_udp(bool in_step)
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_NTH_HA1_MODEL, &error), error)) return false;
    common::UdpSocket host, slider_host;
    chassis::ChassisPluginConfig config;
    config.motor_family = chassis::ChassisMotorFamily::Kinco;
    config.kinco_control_in_step = in_step;
    if (!require(configure_loopback_endpoints(config, host, slider_host, &error), error)) return false;
    config.kinco_endpoint = config.jiechang_endpoint;
    std::array<int, 4> qpos{};
    std::vector<std::string> brakes;
    if (!runtime.write_native([&](const mjModel* model, mjData* data) {
            for (std::size_t axis = 0; axis < 4; ++axis)
            {
                const int joint = mj_name2id(model, mjOBJ_JOINT, config.axes[axis].joint.c_str());
                qpos[axis] = model->jnt_qposadr[joint];
                data->qpos[qpos[axis]] = axis % 2 == 0 ? 0.25 : -0.25;
                brakes.push_back("brake_" + config.axes[axis].joint);
            }
        }, true, &error)) return false;
    // Hold the initial pose until SDO homing releases the startup brakes.
    if (!require(runtime.set_equalities_active(brakes, true, &error), error)) return false;
    chassis::ChassisPlugin plugin(config);
    if (!require(plugin.start(runtime, &error) && runtime.start(&error), error) ||
        !require(test::wait_for_mcu_feedback(
                     host, {"127.0.0.1", config.kinco_endpoint.bind_port}, error), error)) return false;
    std::uint32_t sequence = 1;
    auto send = [&](std::size_t axis, std::uint32_t id, const std::vector<std::uint8_t>& data) {
        return send_datagram(host, config.kinco_endpoint.bind_port,
                            make_downlink(config.motor_can_interfaces[axis], id, data, sequence++, 0), &error);
    };
    for (std::size_t axis = 0; axis < 4; ++axis)
    {
        const auto node = static_cast<std::uint8_t>(axis + 1);
        auto write = [&](std::uint16_t index, std::uint8_t sub,
                         std::uint32_t value, std::uint8_t command) {
            std::vector<std::uint8_t> data{command, static_cast<std::uint8_t>(index),
                static_cast<std::uint8_t>(index >> 8), sub, 0, 0, 0, 0};
            common::write_le32(data.data() + 4, value);
            return send(axis, 0x600 + node, data);
        };
        if (!send(axis, 0, {1, node}) || !write(0x6098, 0, 0xef, 0x2f) ||
            !write(0x607c, 0, 275130, 0x23) || !write(0x6099, 5, 0, 0x2f) ||
            !write(0x6099, 1, 322123, 0x23) || !write(0x6099, 2, 322123, 0x23) ||
            !write(0x609a, 0, 8053, 0x23) || !write(0x6099, 3, 2, 0x2f) ||
            !write(0x6060, 0, 6, 0x2f) || !write(0x6040, 0, 0x0f, 0x2b) ||
            !write(0x6040, 0, 0x1f, 0x2b)) return false;
    }
    // No RPDO keepalives: SDO-started homing must outlive the normal 100 ms
    // command watchdog and move actual joints before setting attained bits.
    std::array<bool, 4> attained{};
    bool observed_motion = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    std::array<std::uint8_t, common::kMcuMaxDatagramSize + 1> buffer{};
    while (std::chrono::steady_clock::now() < deadline &&
           !std::all_of(attained.begin(), attained.end(), [](bool done) { return done; }))
    {
        const auto snapshot = runtime.latest_snapshot();
        for (std::size_t axis = 0; axis < 4; ++axis)
        {
            const double position = std::abs(snapshot.qpos[qpos[axis]]);
            if (position > 0.03 && position < 0.2) observed_motion = true;
        }
        std::size_t size = 0;
        if (!host.receive(buffer.data(), buffer.size(), size) || size < common::kMcuGptpSize) continue;
        common::McuEnvelope envelope;
        common::CanFdFrame frame;
        if (!common::decode_mcu_downlink(buffer.data(), size - common::kMcuGptpSize, envelope) ||
            !common::decode_can_fd_payload(envelope.payload.data(), envelope.payload.size(), frame)) continue;
        if (!require(frame.can_id < 0x581 || frame.can_id > 0x584 || frame.data[0] != 0x80,
                     "Platform homing sequence received an SDO abort.")) return false;
        if (frame.can_id >= 0x281 && frame.can_id <= 0x284 && frame.data.size() == 4)
        {
            const auto axis = frame.can_id - 0x281;
            attained[axis] = (common::read_le16(frame.data.data() + 2) & 0x1000) != 0;
            if (attained[axis] && !require(std::abs(snapshot.qpos[qpos[axis]]) < 0.005,
                                         "Homing completed before physical arrival.")) return false;
        }
    }
    const auto snapshot = runtime.latest_snapshot();
    runtime.pause();
    plugin.stop();
    if (!require(observed_motion, "Homing did not traverse intermediate physical positions.") ||
        !require(std::all_of(attained.begin(), attained.end(), [](bool done) { return done; }),
                 "Homing did not complete for all four steer motors.")) return false;
    for (const auto address : qpos)
        if (!require(std::abs(snapshot.qpos[address]) < 0.005, "Homing did not reach model zero.")) return false;
    std::cout << "Kinco physical homing passed (in_step=" << in_step << ").\n";
    return true;
}

bool test_ground_motion(const char* label, double vx, double vy, double wz,
                        bool mit_steer = false, bool kinco = false)
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    RuntimeConfig runtime_config;
    runtime_config.snapshot_rate_hz = 500;
    Runtime runtime(runtime_config);
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_NTH_HA1_SCENE, &error), error)) return false;
    common::UdpSocket host, slider_host;
    chassis::ChassisPluginConfig config;
    config.feedback_rate_hz = 500;
    if (!require(configure_loopback_endpoints(config, host, slider_host, &error), error)) return false;
    if (kinco)
    {
        config.motor_family = chassis::ChassisMotorFamily::Kinco;
        config.kinco_endpoint = config.jiechang_endpoint;
        for (std::size_t i = 0; i < 8; ++i) config.axes[i].sign = i < 4 ? 1 : -1;
    }
    chassis::ChassisPlugin plugin(config);
    if (!require(plugin.start(runtime, &error), error)) return false;
    if (!require(test::wait_for_mcu_feedback(
                     host, {"127.0.0.1", config.jiechang_endpoint.bind_port}, error),
                 error)) return false;

    constexpr double pi = 3.14159265358979323846;
    // Platform chassis_kinematics.h and WheelFrameMapping: A/B/C/D=LF/RF/RR/LR,
    // steer_invert=true, wheel_sign={-1,+1,+1,-1}, 524288 counts/motor turn.
    constexpr std::array<double, 4> rx{0.1885, 0.1885, -0.2015, -0.2015};
    constexpr std::array<double, 4> ry{0.1785, -0.1785, -0.1785, 0.1785};
    constexpr std::array<double, 4> wheel_sign{-1, 1, 1, -1};
    std::array<double, 4> angles{}, speeds{};
    std::array<int, 4> qpos{};
    int root_qpos = -1;
    runtime.read_native([&](const mjModel* model, const mjData*) {
        const int root = mj_name2id(model, mjOBJ_JOINT, "north_freejoint");
        if (root >= 0) root_qpos = model->jnt_qposadr[root];
        for (int i = 0; i < 4; ++i)
        {
            const auto name = "chassis_steer_joint_" + std::to_string(i + 1);
            qpos[i] = model->jnt_qposadr[mj_name2id(model, mjOBJ_JOINT, name.c_str())];
        }
    });
    if (!require(root_qpos >= 0, "Ground motion test requires north_freejoint.")) return false;
    for (int i = 0; i < 4; ++i)
    {
        const double x = vx - wz * ry[i];
        const double y = vy + wz * rx[i];
        angles[i] = std::atan2(y, x);
        speeds[i] = std::hypot(x, y) / 0.0653886625;
        // Equivalent nearest steering representation used by Platform IK.
        if (angles[i] > pi / 2) { angles[i] -= pi; speeds[i] = -speeds[i]; }
        if (angles[i] < -pi / 2) { angles[i] += pi; speeds[i] = -speeds[i]; }
    }
    std::uint32_t sequence = 1;
    const auto send_kinco = [&](std::uint8_t bus, std::uint32_t id, const std::vector<std::uint8_t>& data) {
        const auto packet = make_downlink(bus, id, data, sequence++, 0);
        return send_datagram(host, config.kinco_endpoint.bind_port, packet, &error);
    };
    if (kinco)
    {
        for (std::size_t i = 0; i < 8; ++i)
        {
            const auto node = static_cast<std::uint8_t>(i + 1);
            const auto bus = config.motor_can_interfaces[i];
            if (!send_kinco(bus, 0, {1, node}) ||
                !send_kinco(bus, 0x600 + node, {0x2f, 0x60, 0x60, 0,
                    static_cast<std::uint8_t>(i < 4 ? 1 : -3), 0, 0, 0})) return false;
        }
    }
    const auto send = [&](bool drive) {
        if (kinco)
        {
            for (std::size_t i = 0; i < 8; ++i)
            {
                std::vector<std::uint8_t> pdo(6, 0);
                common::write_le16(pdo.data(), 0x0f);
                const double target = i < 4
                    ? -angles[i] * config.kinco_steer_gear_ratio * 65536 / (2 * pi)
                    : (drive ? -speeds[i - 4] * config.kinco_wheel_gear_ratio * 60 / (2 * pi) * 512 * 65536 / 1875 : 0);
                common::write_le32(pdo.data() + 2, static_cast<std::uint32_t>(static_cast<std::int32_t>(std::llround(target))));
                if (!send_kinco(config.motor_can_interfaces[i], 0x201 + i, pdo)) return false;
            }
            return true;
        }
        std::vector<std::uint8_t> pdo1(48, 0), pdo2(64, 0), pdo3(64, 0);
        for (int i = 0; i < 4; ++i)
        {
            set_pdo1(pdo1, i, mit_steer ? -1 : 8, true, true, 0);
            set_pdo2(pdo2, i, static_cast<std::int32_t>(std::llround(
                angles[i] * config.steer_gear_ratio * 524288 / (2 * pi))), 0);
            common::write_le32(pdo3.data() + i * 8, 120);
            common::write_le32(pdo3.data() + i * 8 + 4, 12);
            set_pdo1(pdo1, i + 4, 9, true, true, 0);
            set_pdo2(pdo2, i + 4, 0, drive ? static_cast<std::int32_t>(std::llround(
                wheel_sign[i] * speeds[i] * config.wheel_gear_ratio * 300 / (2 * pi))) : 0);
        }
        for (std::uint8_t bus : {0, 1})
        {
            // Platform sends PDO1, optional PDO3, PDO2, then SYNC on each bus.
            for (const auto id : {0x200, 0x400, 0x300, 0x080})
            {
                if (id == 0x400 && !mit_steer) continue;
                const auto packet = make_downlink(bus, id,
                    id == 0x200 ? pdo1 : id == 0x300 ? pdo2 :
                    id == 0x400 ? pdo3 : std::vector<std::uint8_t>{}, sequence++);
                if (!send_datagram(host, config.jiechang_endpoint.bind_port,
                                   packet, &error)) return false;
            }
        }
        return true;
    };
    if (!require(runtime.start(&error), error)) return false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    bool aligned = false;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (!send(false)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const auto snapshot = runtime.latest_snapshot();
        aligned = true;
        for (int i = 0; i < 4; ++i)
            aligned = aligned && std::abs(snapshot.qpos[qpos[i]] + angles[i]) < 0.10;
        // Let the chassis settle onto the floor before testing translation.
        if (aligned && snapshot.simulation_time > 0.3) break;
    }
    if (!require(aligned, std::string(label) + ": steering could not align on the ground.")) return false;
    const auto before = runtime.latest_snapshot();
    const auto drive_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
    while (std::chrono::steady_clock::now() < drive_deadline)
    {
        if (!send(true)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto after = runtime.latest_snapshot();
    if (!send(false)) return false;
    plugin.stop();
    runtime.pause();
    const auto yaw = [&](const PoseSnapshot& snapshot) {
        const double* q = snapshot.qpos.data() + root_qpos + 3;
        return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                          1 - 2 * (q[2] * q[2] + q[3] * q[3]));
    };
    const double heading = yaw(before);
    const double dx = after.qpos[root_qpos] - before.qpos[root_qpos];
    const double dy = after.qpos[root_qpos + 1] - before.qpos[root_qpos + 1];
    const double body_x = std::cos(heading) * dx + std::sin(heading) * dy;
    const double body_y = -std::sin(heading) * dx + std::cos(heading) * dy;
    const double turn = std::remainder(yaw(after) - heading, 2 * pi);
    std::cout << label << ": dx=" << body_x << " dy=" << body_y << " yaw=" << turn << '\n';
    if (wz != 0)
        return require(turn * std::copysign(1.0, wz) > 0.12 && std::hypot(dx, dy) < 0.08,
                       std::string(label) + ": wrong yaw direction or excessive drift.");
    const double progress = (body_x * vx + body_y * vy) / std::hypot(vx, vy);
    const double cross = (body_y * vx - body_x * vy) / std::hypot(vx, vy);
    return require(progress > 0.06 && std::abs(cross) < 0.06 && std::abs(turn) < 0.15,
                   std::string(label) + ": wrong translation direction or excessive drift.");
}

}  // namespace

int main()
{
    if (!test_chassis_coordinate_contract_uses_robot_frame())
    {
        return EXIT_FAILURE;
    }
    if (!test_chassis_modes_and_feedback()) return EXIT_FAILURE;
    if (!test_jiechang_stretch_udp()) return EXIT_FAILURE;
    if (!test_kinco_homing_udp(false) || !test_kinco_homing_udp(true)) return EXIT_FAILURE;
    if (!test_wheel_pi_lifecycle(false) || !test_wheel_pi_lifecycle(true) ||
        !test_wheel_pi_lifecycle(true, true)) return EXIT_FAILURE;
    if (!test_ground_motion("left", 0, 0.15, 0) ||
        !test_ground_motion("right", 0, -0.15, 0) ||
        !test_ground_motion("rotate-left", 0, 0, 0.3) ||
        !test_ground_motion("rotate-right", 0, 0, -0.3) ||
        !test_ground_motion("forward", 0.15, 0, 0) ||
        !test_ground_motion("backward", -0.15, 0, 0) ||
        !test_ground_motion("MIT left", 0, 0.15, 0, true) ||
        !test_ground_motion("Kinco forward", 0.15, 0, 0, false, true) ||
        !test_ground_motion("Kinco backward", -0.15, 0, 0, false, true) ||
        !test_ground_motion("Kinco left", 0, 0.15, 0, false, true) ||
        !test_ground_motion("Kinco right", 0, -0.15, 0, false, true) ||
        !test_ground_motion("Kinco rotate-left", 0, 0, 0.3, false, true) ||
        !test_ground_motion("Kinco rotate-right", 0, 0, -0.3, false, true)) return EXIT_FAILURE;
    std::cout << "Chassis UDP mode and feedback tests passed.\n";
    return EXIT_SUCCESS;
}
