// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/arm/arm_plugin.h"
#include "sharpa_mujoco/plugins/arm/arm_transmission.h"
#include "sharpa_mujoco/plugins/arm/arm_udp_codec.h"
#include "sharpa_mujoco/plugins/arm/cia402.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/runtime.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
bool require(bool condition, const std::string& message)
{
    if (!condition) std::cerr << "FAILED: " << message << '\n';
    return condition;
}

void append_u16(std::vector<std::uint8_t>& out, std::uint16_t value)
{
    out.push_back(static_cast<std::uint8_t>(value));
    out.push_back(static_cast<std::uint8_t>(value >> 8U));
}

void append_u32(std::vector<std::uint8_t>& out, std::uint32_t value)
{
    for (int shift = 0; shift < 32; shift += 8)
        out.push_back(static_cast<std::uint8_t>(value >> shift));
}

std::vector<std::uint8_t> control_frame(
    std::uint32_t sequence,
    std::uint16_t control_word,
    std::int8_t mode,
    float position,
    float velocity = 0.0F,
    float torque = 0.0F,
    float kp = 0.0F,
    float kd = 0.0F)
{
    using sharpa::mujoco::plugins::arm::ArmUdpCodec;
    std::vector<std::uint8_t> frame;
    append_u32(frame, sequence);
    frame.push_back(ArmUdpCodec::kControlCommand);
    frame.push_back(0);
    frame.push_back(static_cast<std::uint8_t>(mode));
    append_u16(frame, control_word);
    append_u32(frame, static_cast<std::uint32_t>(
        static_cast<std::int32_t>(std::lround(position * 1000000.0F))));
    append_u16(frame, static_cast<std::uint16_t>(
        static_cast<std::int16_t>(std::lround(velocity * 1000.0F))));
    append_u16(frame, static_cast<std::uint16_t>(
        static_cast<std::int16_t>(std::lround(torque * 100.0F))));
    append_u16(frame, static_cast<std::uint16_t>(std::lround(kp * 100.0F)));
    append_u16(frame, static_cast<std::uint16_t>(std::lround(kd * 1000.0F)));
    append_u16(frame, ArmUdpCodec::crc16_ccitt_false(frame.data(), frame.size()));
    return frame;
}

std::vector<std::uint8_t> dual_csp_control_frame(
    std::uint32_t sequence,
    const std::array<std::uint16_t, 2>& control_words,
    float position0,
    float position1,
    const std::array<std::int8_t, 2>& modes = {8, 8})
{
    using sharpa::mujoco::plugins::arm::ArmUdpCodec;
    std::vector<std::uint8_t> frame;
    append_u32(frame, sequence);
    for (std::uint8_t node_id = 0; node_id < 2; ++node_id)
    {
        const float position = node_id == 0 ? position0 : position1;
        frame.push_back(ArmUdpCodec::kControlCommand);
        frame.push_back(node_id);
        frame.push_back(static_cast<std::uint8_t>(modes[node_id]));
        append_u16(frame, control_words[node_id]);
        append_u32(frame, static_cast<std::uint32_t>(
            static_cast<std::int32_t>(std::lround(position * 1000000.0F))));
        append_u16(frame, 0);
        append_u16(frame, 0);
        append_u16(frame, 0);
        append_u16(frame, 0);
    }
    append_u16(frame, ArmUdpCodec::crc16_ccitt_false(frame.data(), frame.size()));
    return frame;
}

std::vector<std::uint8_t> dual_csp_control_frame(
    std::uint32_t sequence,
    std::uint16_t control_word,
    float position0,
    float position1)
{
    return dual_csp_control_frame(
        sequence, {control_word, control_word}, position0, position1);
}

bool protocol_and_transmission()
{
    using namespace sharpa::mujoco::plugins::arm;
    const std::array<std::uint8_t, 9> known{'1','2','3','4','5','6','7','8','9'};
    if (!require(
            ArmUdpCodec::crc16_ccitt_false(known.data(), known.size()) == 0x29B1,
            "CRC-16 known vector failed."))
        return false;
    const auto frame =
        control_frame(0x01020304, 0x000F, 11, 1.25F, -0.5F, 2.5F, 12.0F, 0.8F);
    DecodedControl decoded;
    if (!require(frame.size() == 23, "Single-axis control size is wrong.") ||
        !require(ArmUdpCodec::decode_control(frame.data(), frame.size(), decoded),
                 "Control frame decode failed.") ||
        !require(decoded.sequence == 0x01020304 && decoded.axis_count == 1,
                 "Control header decode failed.") ||
        !require(std::abs(decoded.axes[0].position_rad - 1.25F) < 1e-6F,
                 "Control position decode failed.") ||
        !require(decoded.axes[0].mode == 11 &&
                     std::abs(decoded.axes[0].velocity_rad_s + 0.5F) < 1e-6F &&
                     std::abs(decoded.axes[0].torque_nm - 2.5F) < 1e-6F &&
                     std::abs(decoded.axes[0].kp - 12.0F) < 1e-6F &&
                     std::abs(decoded.axes[0].kd - 0.8F) < 1e-6F,
                 "Control mode fields decode failed."))
        return false;
    auto corrupt = frame;
    corrupt[9] ^= 1;
    if (!require(!ArmUdpCodec::decode_control(corrupt.data(), corrupt.size(), decoded),
                 "Bad control CRC was accepted."))
        return false;

    std::array<AxisFeedback, 2> feedback{};
    feedback[0].bus_voltage_v = 48;
    feedback[0].driver_temperature_c = -5;
    feedback[0].motor_temperature_c = 37;
    feedback[0].torque_sensor_nm = -12.34F;
    feedback[1].node_id = 1;
    std::vector<std::uint8_t> encoded;
    if (!require(ArmUdpCodec::encode_feedback(7, feedback.data(), 2,
                     0x0102030405060708ULL, encoded, true) && encoded.size() == 74 &&
                 encoded[25] == 48 && encoded[26] == 0xFB && encoded[27] == 37 &&
                 encoded[28] == 0x2E && encoded[29] == 0xFB && encoded[35] == 1 &&
                 encoded[66] == 8 && encoded[73] == 1,
                 "Extended feedback layout/scaling/timestamp mismatch.")) return false;
    if (!require(ArmUdpCodec::encode_feedback(7, feedback.data(), 2, 0, encoded) &&
                 encoded.size() == 56, "Legacy feedback compatibility lost.")) return false;
    const auto sdo_request = [](std::uint8_t node, std::uint16_t index) {
        std::vector<std::uint8_t> data{3, node};
        append_u16(data, index); append_u32(data, 0);
        append_u16(data, ArmUdpCodec::crc16_ccitt_false(data.data(), data.size()));
        return data;
    };
    for (const auto request : {sdo_request(2, 0x30AE), sdo_request(0, 0x9999)})
    {
        if (!require(ArmUdpCodec::respond_sdo(request.data(), request.size(), 2, encoded) &&
                     encoded.size() == 10 && encoded[0] == 0x83 && encoded[4] == 2,
                     "Unsupported SDO address was falsely acknowledged.")) return false;
    }
    auto bad_sdo = sdo_request(0, 0x30AE);
    bad_sdo[8] ^= 1;
    if (!require(!ArmUdpCodec::respond_sdo(bad_sdo.data(), 10, 1, encoded) &&
                 !ArmUdpCodec::respond_sdo(bad_sdo.data(), 9, 1, encoded),
                 "Malformed SDO request was accepted.")) return false;

    Cia402Drive drive;
    drive.apply(0x0006);
    if (!require(drive.status_word() == 0x0021, "Shutdown transition failed.")) return false;
    drive.apply(0x0007);
    drive.apply(0x000F);
    if (!require(drive.operation_enabled(), "Enable operation transition failed.")) return false;
    drive.apply(0x0002);
    if (!require(drive.status_word() == 0x0007, "Quick stop transition failed.")) return false;
    drive.complete_quick_stop();
    if (!require(drive.status_word() == 0x0040,
                 "Quick stop completion did not disable the drive."))
        return false;
    drive.apply(0x0002);
    if (!require(drive.status_word() == 0x0040,
                 "A disabled drive re-entered quick stop."))
        return false;
    drive.inject_fault();
    drive.apply(0x0080);
    if (!require(drive.status_word() == 0x0040, "Fault reset transition failed.")) return false;

    for (ArmSide side : {ArmSide::Left, ArmSide::Right})
    {
        ArmTransmission transmission(side);
        ArmTransmission::AxisArray joints{0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7};
        const auto round_trip =
            transmission.motor_to_joint(transmission.joint_to_motor(joints));
        const auto torque_round_trip = transmission.motor_torque_to_joint(
            transmission.joint_torque_to_motor(joints));
        for (std::size_t i = 0; i < joints.size(); ++i)
            if (!require(
                    std::abs(round_trip[i] - joints[i]) < 1e-9 &&
                        std::abs(torque_round_trip[i] - joints[i]) < 1e-9,
                    "Arm position/torque transmission round trip failed."))
                return false;
    }
    return true;
}

bool udp_integration_for_side(
    sharpa::mujoco::plugins::arm::ArmSide side,
    const char* actuator_name,
    double expected_scale)
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error), error))
        return false;

    std::array<common::UdpSocket, 5> hosts;
    std::array<common::UdpSocket, 5> probes;
    arm::ArmPluginConfig config;
    config.sdo_port = 0;
    config.extended_feedback = false;  // Exercise legacy PDO compatibility here.
    config.name = side == arm::ArmSide::Left ? "arm_left" : "arm_right";
    config.side = side;
    config.feedback_rate_hz = 500;
    config.command_timeout_s = 0.05;
    config.quick_stop_deceleration_rad_s2 = 20.0;
    config.quick_stop_velocity_threshold_rad_s = 0.01;
    config.quick_stop_timeout_s = 0.02;
    config.quick_stop_kp = 20.0;
    config.speed_ki = 0.0;
    config.sim_joint_kp.fill(0.0);
    config.sim_joint_kd.fill(0.0);
    config.sim_joint_kp[3] = 20.0;
    config.sim_joint_kp[4] = 10.0;
    config.sim_joint_kp[5] = 5.0;
    config.sim_joint_kp[6] = 5.0;
    config.csp_kp.fill(0.0);
    config.csp_kd.fill(0.0);
    const std::array<std::vector<std::size_t>, 5> mappings{
        std::vector<std::size_t>{0}, {1}, {2}, {3, 4}, {5, 6}};
    for (std::size_t i = 0; i < hosts.size(); ++i)
    {
        if (!hosts[i].open_and_bind({"127.0.0.1", 0}, 100, &error) ||
            !probes[i].open_and_bind({"127.0.0.1", 0}, 100, &error))
            return require(false, error);
        config.endpoints[i].bind_port = probes[i].local_port(&error);
        config.endpoints[i].peer_port = hosts[i].local_port(&error);
        config.endpoints[i].motor_indices = mappings[i];
        probes[i].close();
    }
    arm::ArmPlugin plugin(config);
    if (!require(plugin.start(runtime, &error), error)) return false;

    std::array<std::uint8_t, 128> buffer{};
    std::size_t received = 0;
    const bool proactive =
        require(hosts[0].receive(buffer.data(), buffer.size(), received),
                "Arm did not send proactive feedback.") &&
        require(received == 35, "Single-axis feedback size is wrong.");

    const auto wait_for_wrist_status =
        [&](std::uint16_t first, std::uint16_t second) {
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(200);
            while (std::chrono::steady_clock::now() < deadline)
            {
                if (!hosts[4].receive(buffer.data(), buffer.size(), received) ||
                    received != 56)
                    continue;
                const auto status = [&](std::size_t slot) {
                    const std::size_t offset =
                        4 + slot * arm::ArmUdpCodec::kFeedbackAxisSize + 3;
                    return static_cast<std::uint16_t>(
                        buffer[offset] | (buffer[offset + 1] << 8U));
                };
                if (status(0) == first && status(1) == second) return true;
            }
            return false;
        };

    std::uint32_t sequence = 1;
    const auto send_control =
        [&](std::uint16_t word,
            std::int8_t mode,
            float position,
            float velocity,
            float torque,
            float kp,
            float kd) {
            const auto packet = control_frame(
                sequence++, word, mode, position, velocity, torque, kp, kd);
            if (!hosts[0].send_to(
                    {"127.0.0.1", config.endpoints[0].bind_port},
                    packet.data(), packet.size(), &error))
            {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            return true;
        };
    for (const std::uint16_t word : {0x0006, 0x0007, 0x000F})
    {
        if (!send_control(word, 10, 0.0F, 0.0F, 2.0F, 0.0F, 0.0F))
            return require(false, error);
    }
    const ModelInfo model = runtime.model_info();
    const std::string joint_prefix =
        side == arm::ArmSide::Left ? "left_arm_joint_" : "right_arm_joint_";
    const auto actuator = std::find_if(
        model.actuators.begin(), model.actuators.end(),
        [actuator_name](const ActuatorInfo& value) {
            return value.name == actuator_name;
        });
    if (!require(actuator != model.actuators.end(), "Arm actuator was not found."))
        return false;
    const auto expect_control =
        [&](double expected, const char* message, double tolerance = 1e-5) {
        if (!runtime.step(1, &error)) return require(false, error);
        const PoseSnapshot snapshot = runtime.latest_snapshot();
        return require(
            std::abs(snapshot.controls[static_cast<std::size_t>(actuator->id)] -
                     expected_scale * expected) < tolerance,
            message);
    };
    const bool control_ok =
        expect_control(2.0, "Torque mode output is wrong.") &&
        send_control(0x000F, 11, 0.0F, 0.0F, 1.5F, 0.0F, 0.0F) &&
        expect_control(1.5, "MIT mode feedforward output is wrong.") &&
        send_control(0x000F, 8, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F) &&
        expect_control(1.0, "CSP torque feedforward output is wrong.");
    if (!control_ok)
    {
        plugin.stop();
        runtime.unload();
        return false;
    }

    const auto axis_brake_released = [&] {
        bool released = false;
        runtime.read_native(
            [&](const mjModel* native_model, const mjData* data) {
                const int equality = mj_name2id(
                    native_model, mjOBJ_EQUALITY,
                    ("brake_" + joint_prefix + "1").c_str());
                released = equality >= 0 && data->eq_active[equality] == 0;
            },
            &error);
        return released;
    };
    const std::uint64_t reset_generation = runtime.reset_generation();
    bool post_reset_wrist_command_sent = true;
    runtime.set_step_callbacks(
        [](const mjModel*, mjData* data, const NativeStepContext&) {
            data->qvel[0] = std::numeric_limits<double>::quiet_NaN();
            return StepDecision::Proceed;
        },
        [&](const mjModel*, mjData*, const NativeStepContext&) {
            // This callback runs after Runtime has published the automatic
            // reset boundary, but before the next model-updater pass. The
            // complete enable sequence must survive for the wrist pair.
            for (const std::uint16_t word : {0x0006, 0x0007, 0x000F})
            {
                const auto packet = dual_csp_control_frame(
                    sequence++, word, 0.0F, 0.0F);
                post_reset_wrist_command_sent =
                    hosts[4].send_to(
                        {"127.0.0.1", config.endpoints[4].bind_port},
                        packet.data(), packet.size(), &error) &&
                    post_reset_wrist_command_sent;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        });
    const bool automatic_reset_triggered = runtime.step(1, &error);
    runtime.clear_step_callbacks();
    bool runtime_reset_recovery_ok = automatic_reset_triggered &&
        post_reset_wrist_command_sent &&
        require(runtime.reset_generation() == reset_generation + 1,
                "MuJoCo automatic reset generation did not advance.") &&
        runtime.step(1, &error);
    if (runtime_reset_recovery_ok)
    {
        const PoseSnapshot after_reset = runtime.latest_snapshot();
        bool wrist_pair_released = false;
        runtime.read_native(
            [&](const mjModel* native_model, const mjData* data) {
                const int motor6 = mj_name2id(
                    native_model,
                    mjOBJ_EQUALITY,
                    ("brake_" + joint_prefix + "6").c_str());
                const int motor7 = mj_name2id(
                    native_model,
                    mjOBJ_EQUALITY,
                    ("brake_" + joint_prefix + "7").c_str());
                wrist_pair_released =
                    motor6 >= 0 && motor7 >= 0 &&
                    data->eq_active[motor6] == 0 &&
                    data->eq_active[motor7] == 0;
            },
            &error);
        runtime_reset_recovery_ok = require(
            std::abs(after_reset.controls[
                static_cast<std::size_t>(actuator->id)]) < 1e-9,
            "Automatic reset did not invalidate the active arm command.") &&
            require(
                wrist_pair_released,
                "First post-reset wrist command did not atomically release motor 6/7 brakes.") &&
            require(!axis_brake_released(),
                    "Automatic reset did not restore the arm startup brake.") &&
            send_control(0x000F, 10, 0.0F, 0.0F, 2.0F, 0.0F, 0.0F) &&
            expect_control(
                2.0,
                "The first fresh enable after automatic reset was not applied.") &&
            require(axis_brake_released(),
                    "Fresh enable did not release the brake after automatic reset.");
    }
    if (!runtime_reset_recovery_ok)
    {
        plugin.stop();
        runtime.unload();
        return false;
    }

    // Reset clears input already queued before the boundary. Feedback may
    // run between a fresh enable and the next step, but must not clear it.
    const auto wrist_enable = dual_csp_control_frame(
        sequence++, 0x000F, 0.0F, 0.0F);
    if (!hosts[4].send_to(
            {"127.0.0.1", config.endpoints[4].bind_port},
            wrist_enable.data(), wrist_enable.size(), &error))
        return require(false, error);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!runtime.reset(&error) || !runtime.step(1, &error) ||
        !require(wait_for_wrist_status(0x0040, 0x0040),
                 "Explicit reset did not discard queued wrist enable.") ||
        !hosts[4].send_to(
            {"127.0.0.1", config.endpoints[4].bind_port},
            wrist_enable.data(), wrist_enable.size(), &error))
        return require(false, error);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!require(wait_for_wrist_status(0x0040, 0x0040),
                 "Feedback enabled the wrist before a simulation step.") ||
        !runtime.step(1, &error) ||
        !require(wait_for_wrist_status(0x0027, 0x0027),
                 "Feedback discarded the first post-reset wrist enable."))
        return false;

    const auto send_csp_pair =
        [&](std::size_t endpoint_index,
            std::uint16_t word,
            float position0,
            float position1) {
            const auto packet = dual_csp_control_frame(
                sequence++, word, position0, position1);
            if (!hosts[endpoint_index].send_to(
                    {"127.0.0.1", config.endpoints[endpoint_index].bind_port},
                    packet.data(), packet.size(), &error))
            {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            return true;
        };
    const auto check_csp_pair =
        [&](std::size_t endpoint_index,
            std::size_t first_axis,
            double first_delta,
            double second_delta) {
            for (const std::uint16_t word : {0x0006, 0x0007, 0x000F})
            {
                if (!send_csp_pair(endpoint_index, word, 0.0F, 0.0F))
                    return require(false, error);
            }

            const PoseSnapshot before = runtime.latest_snapshot();
            arm::ArmTransmission::AxisArray current_joints{};
            for (std::size_t axis = 0; axis < current_joints.size(); ++axis)
            {
                const std::string name =
                    joint_prefix + std::to_string(axis + 1);
                const auto item = std::find_if(
                    model.joints.begin(), model.joints.end(),
                    [&](const JointInfo& value) { return value.name == name; });
                if (item == model.joints.end())
                    return require(false, "Arm joint was not found.");
                current_joints[axis] = before.qpos[
                    static_cast<std::size_t>(item->qpos_address)];
            }
            auto target_joints = current_joints;
            target_joints[first_axis] += first_delta;
            target_joints[first_axis + 1] += second_delta;
            const auto target_motors =
                arm::ArmTransmission(side).joint_to_motor(target_joints);
            if (!send_csp_pair(
                    endpoint_index, 0x000F,
                    static_cast<float>(target_motors[first_axis]),
                    static_cast<float>(target_motors[first_axis + 1])))
            {
                return require(false, error);
            }
            if (!runtime.step(1, &error))
                return require(false, error);

            const PoseSnapshot after = runtime.latest_snapshot();
            arm::ArmTransmission::AxisArray requested_joint_torques{};
            requested_joint_torques[first_axis] =
                config.sim_joint_kp[first_axis] * first_delta;
            requested_joint_torques[first_axis + 1] =
                config.sim_joint_kp[first_axis + 1] * second_delta;
            auto limited_motor_torques = arm::ArmTransmission(side)
                                              .joint_torque_to_motor(
                                                  requested_joint_torques);
            for (std::size_t axis = first_axis;
                 axis <= first_axis + 1;
                 ++axis)
            {
                limited_motor_torques[axis] = std::clamp(
                    limited_motor_torques[axis],
                    -config.torque_limits_nm[axis],
                    config.torque_limits_nm[axis]);
            }
            const auto expected_joint_torques = arm::ArmTransmission(side)
                                                    .motor_torque_to_joint(
                                                        limited_motor_torques);
            for (std::size_t offset = 0; offset < 2; ++offset)
            {
                const std::size_t axis = first_axis + offset;
                const std::string actuator_name_for_axis =
                    joint_prefix + std::to_string(axis + 1) + "_ctrl";
                const auto arm_actuator = std::find_if(
                    model.actuators.begin(), model.actuators.end(),
                    [&](const ActuatorInfo& value) {
                        return value.name == actuator_name_for_axis;
                    });
                if (arm_actuator == model.actuators.end())
                    return require(false, "Arm actuator was not found.");
                if (!require(
                        std::abs(after.controls[static_cast<std::size_t>(
                                     arm_actuator->id)] -
                                 expected_joint_torques[axis]) < 5e-4,
                        "Joint-space CSP torque for a differential pair is wrong."))
                {
                    return false;
                }
            }
            return true;
        };
    const bool paired_csp_ok =
        check_csp_pair(3, 3, 0.01, 0.02) &&
        check_csp_pair(3, 3, 0.5, 1.0) &&
        check_csp_pair(4, 5, 0.02, 0.03);
    if (!paired_csp_ok)
    {
        plugin.stop();
        runtime.unload();
        return false;
    }
    const auto joint6 = std::find_if(
        model.joints.begin(), model.joints.end(),
        [&](const JointInfo& value) { return value.name == joint_prefix + "6"; });
    const auto actuator6 = std::find_if(
        model.actuators.begin(), model.actuators.end(),
        [&](const ActuatorInfo& value) {
            return value.name == joint_prefix + "6_ctrl";
        });
    const auto actuator7 = std::find_if(
        model.actuators.begin(), model.actuators.end(),
        [&](const ActuatorInfo& value) {
            return value.name == joint_prefix + "7_ctrl";
        });
    if (!require(
            joint6 != model.joints.end() &&
                actuator6 != model.actuators.end() &&
                actuator7 != model.actuators.end(),
            "Arm wrist joints or actuators were not found."))
    {
        plugin.stop();
        runtime.unload();
        return false;
    }
    const auto wrist_brakes_released = [&] {
        bool released = false;
        runtime.read_native(
            [&](const mjModel* native_model, const mjData* data) {
                const int brake6 = mj_name2id(
                    native_model, mjOBJ_EQUALITY,
                    ("brake_" + joint_prefix + "6").c_str());
                const int brake7 = mj_name2id(
                    native_model, mjOBJ_EQUALITY,
                    ("brake_" + joint_prefix + "7").c_str());
                released = brake6 >= 0 && brake7 >= 0 &&
                    data->eq_active[brake6] == 0 &&
                    data->eq_active[brake7] == 0;
            },
            &error);
        return released;
    };
    bool wrist_quick_stop_ok = runtime.write_native(
        [&](const mjModel*, mjData* data) {
            data->qvel[static_cast<std::size_t>(joint6->dof_address)] = 1.0;
        },
        true,
        &error);
    wrist_quick_stop_ok = wrist_quick_stop_ok &&
        send_csp_pair(4, 0x0002, 0.0F, 0.0F) &&
        runtime.step(1, &error);
    if (wrist_quick_stop_ok)
    {
        PoseSnapshot snapshot = runtime.latest_snapshot();
        wrist_quick_stop_ok = require(
            snapshot.controls[static_cast<std::size_t>(actuator6->id)] < -1e-6,
            "Wrist quick stop did not apply decelerating torque.") &&
            require(wrist_brakes_released(),
                    "Wrist quick stop engaged its startup brakes.");
    }
    wrist_quick_stop_ok = wrist_quick_stop_ok && runtime.step(20, &error);
    if (wrist_quick_stop_ok)
    {
        const PoseSnapshot snapshot = runtime.latest_snapshot();
        wrist_quick_stop_ok = require(
            std::abs(snapshot.controls[
                static_cast<std::size_t>(actuator6->id)]) < 1e-9 &&
                std::abs(snapshot.controls[
                    static_cast<std::size_t>(actuator7->id)]) < 1e-9,
            "Wrist quick stop did not down-enable both differential motors.") &&
            require(wrist_brakes_released(),
                    "Wrist down-enable re-engaged its startup brakes.");
    }
    if (!wrist_quick_stop_ok)
    {
        plugin.stop();
        runtime.unload();
        return false;
    }

    // Either motor may request a quick stop while its peer still sends
    // enable. The stop must reach the existing differential-pair logic.
    for (const bool invalid_peer_mode : {false, true})
    {
        for (const auto words : {std::array<std::uint16_t, 2>{0x0002, 0x000F},
                                 std::array<std::uint16_t, 2>{0x000F, 0x0002},
                                 std::array<std::uint16_t, 2>{0x0000, 0x000F},
                                 std::array<std::uint16_t, 2>{0x000F, 0x0000}})
        {
            if (!send_csp_pair(4, 0x000F, 0.0F, 0.0F) ||
                !runtime.step(1, &error) ||
                !runtime.write_native(
                    [&](const mjModel* native_model, mjData* data) {
                        std::fill(data->qvel, data->qvel + native_model->nv, 0.0);
                        data->qvel[static_cast<std::size_t>(joint6->dof_address)] = 1.0;
                    }, true, &error))
            {
                return require(false, error);
            }
            const bool quick_stop = words[0] == 0x0002 || words[1] == 0x0002;
            // An unsupported enable mode on the peer must not prevent a
            // valid stop. Its previously enabled state is left unchanged.
            const std::array<std::int8_t, 2> modes{
                static_cast<std::int8_t>(invalid_peer_mode && words[0] == 0x000F ? 0 : 8),
                static_cast<std::int8_t>(invalid_peer_mode && words[1] == 0x000F ? 0 : 8)};
            const auto packet = dual_csp_control_frame(
                sequence++, words, 0.0F, 0.0F, modes);
            if (!hosts[4].send_to(
                    {"127.0.0.1", config.endpoints[4].bind_port},
                    packet.data(), packet.size(), &error))
            {
                return require(false, error);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            if (!runtime.step(1, &error)) return require(false, error);
            if (quick_stop &&
                !require(runtime.latest_snapshot().controls[
                             static_cast<std::size_t>(actuator6->id)] < -1e-6,
                         "Mixed enable/quick-stop frame did not brake the wrist."))
                return false;
            if (quick_stop && !runtime.step(20, &error))
                return require(false, error);
            const PoseSnapshot stopped = runtime.latest_snapshot();
            if (!require(
                    std::abs(stopped.controls[static_cast<std::size_t>(actuator6->id)]) < 1e-9 &&
                        std::abs(stopped.controls[static_cast<std::size_t>(actuator7->id)]) < 1e-9,
                    "Mixed stop frame did not remove wrist drive torque.") ||
                !require(wait_for_wrist_status(
                             quick_stop || words[0] == 0 ? 0x0040 : 0x0027,
                             quick_stop || words[1] == 0 ? 0x0040 : 0x0027),
                         "Mixed stop frame did not update wrist drive status."))
            {
                return false;
            }
        }
    }
    const PoseSnapshot before_speed = runtime.latest_snapshot();
    const auto joint = std::find_if(
        model.joints.begin(), model.joints.end(),
        [actuator](const JointInfo& value) {
            return value.id == actuator->transmission_id;
        });
    arm::ArmTransmission::AxisArray joint_velocities{};
    for (std::size_t axis = 0; axis < joint_velocities.size(); ++axis)
    {
        const std::string joint_name = joint_prefix + std::to_string(axis + 1);
        const auto arm_joint = std::find_if(
            model.joints.begin(), model.joints.end(),
            [&](const JointInfo& value) { return value.name == joint_name; });
        if (arm_joint != model.joints.end())
            joint_velocities[axis] =
                before_speed.qvel[static_cast<std::size_t>(arm_joint->dof_address)];
    }
    const double motor_velocity =
        arm::ArmTransmission(side).joint_to_motor(joint_velocities)[0];
    bool direct_axis_enabled = true;
    for (const std::uint16_t word : {0x0006, 0x0007, 0x000F})
        direct_axis_enabled = direct_axis_enabled &&
            send_control(word, 9, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    const bool speed_ok = direct_axis_enabled &&
        send_control(
            0x000F, 9, 0.0F,
            static_cast<float>(motor_velocity + 0.4),
            0.0F, 0.0F, 0.0F) &&
        expect_control(2.0, "Speed mode proportional output is wrong.", 0.01);
    const auto set_joint_velocity = [&](double velocity) {
        return runtime.write_native(
            [&](const mjModel*, mjData* data) {
                data->qvel[static_cast<std::size_t>(joint->dof_address)] = velocity;
            },
            true,
            &error);
    };
    const auto brake_is_released = [&] {
        bool released = false;
        runtime.read_native(
            [&](const mjModel* native_model, const mjData* data) {
                const std::string equality_name =
                    "brake_" + joint_prefix + "1";
                const int equality = mj_name2id(
                    native_model, mjOBJ_EQUALITY, equality_name.c_str());
                released = equality >= 0 && data->eq_active[equality] == 0;
            },
            &error);
        return released;
    };
    const auto expect_braking_then_disabled = [&](const char* braking_message) {
        if (!runtime.step(1, &error)) return require(false, error);
        PoseSnapshot snapshot = runtime.latest_snapshot();
        if (!require(
                snapshot.controls[static_cast<std::size_t>(actuator->id)] < -1e-6,
                braking_message) ||
            !require(brake_is_released(),
                     "Quick stop unexpectedly engaged the startup brake."))
        {
            return false;
        }
        if (!runtime.step(20, &error)) return require(false, error);
        snapshot = runtime.latest_snapshot();
        return require(
                   std::abs(snapshot.controls[
                       static_cast<std::size_t>(actuator->id)]) < 1e-9,
                   "Quick stop did not end with zero drive torque.") &&
               require(brake_is_released(),
                       "Down-enable re-engaged the startup brake.");
    };
    const bool explicit_quick_stop_ok =
        speed_ok && set_joint_velocity(1.0) &&
        send_control(0x0002, 9, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F) &&
        expect_braking_then_disabled(
            "Explicit quick stop did not apply decelerating torque.");

    bool timeout_quick_stop_ok = false;
    bool stale_reenable_command_ok = false;
    if (explicit_quick_stop_ok)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        bool enabled_again = true;
        for (const std::uint16_t word : {0x0006, 0x0007, 0x000F})
            enabled_again = enabled_again &&
                send_control(word, 10, 0.0F, 0.0F, 2.0F, 0.0F, 0.0F);
        stale_reenable_command_ok = enabled_again &&
            expect_control(
                2.0,
                "The first command of a stale re-enable was not applied.");
        if (stale_reenable_command_ok && set_joint_velocity(1.0))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            timeout_quick_stop_ok = expect_braking_then_disabled(
                "Command timeout did not apply quick-stop decelerating torque.");
        }
    }
    plugin.stop();
    runtime.unload();
    return proactive && control_ok && speed_ok &&
           runtime_reset_recovery_ok && explicit_quick_stop_ok &&
           stale_reenable_command_ok &&
           timeout_quick_stop_ok;
}

bool homing_and_extended_services(sharpa::mujoco::plugins::arm::ArmSide side, double feedback_delay = 0.0)
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error), error)) return false;
    arm::ArmPluginConfig config;
    config.side = side;
    config.command_timeout_s = 1.0;
    config.feedback_delay_s.fill(feedback_delay);
    config.driver_temperature_c = -5;
    config.motor_temperature_c = 37;
    common::UdpSocket sdo_host;
    if (!sdo_host.open_and_bind({"127.0.0.1",0},100,&error)) return require(false,error);
    config.sdo_port = sdo_host.local_port();
    std::array<common::UdpSocket,5> hosts;
    const std::array<std::vector<std::size_t>,5> mappings{{{0},{1},{2},{3,4},{5,6}}};
    for (std::size_t i=0; i<5; ++i)
    {
        common::UdpSocket probe;
        config.endpoints[i].bind_ip = "127.77.0." + std::to_string(i+1);
        if (!hosts[i].open_and_bind({"127.0.0.1",0},100,&error) ||
            !probe.open_and_bind({config.endpoints[i].bind_ip,0},100,&error))
            return require(false,error);
        config.endpoints[i].bind_port = probe.local_port();
        config.endpoints[i].peer_port = hosts[i].local_port();
        config.endpoints[i].motor_indices = mappings[i];
    }
    arm::ArmPlugin plugin(config);
    if (!require(plugin.start(runtime,&error),error)) return false;
    std::array<std::uint8_t,128> buffer{};
    std::size_t received=0;
    const auto u16 = [&](std::size_t p) { return std::uint16_t(buffer[p] | (buffer[p+1]<<8)); };
    const auto i32 = [&](std::size_t p) {
        std::uint32_t value=0;
        for(int j=0;j<4;++j) value |= std::uint32_t(buffer[p+j]) << (8*j);
        return static_cast<std::int32_t>(value);
    };
    // Actual UDP SDO round-trip, including node 1 on both differential boards.
    for(std::size_t endpoint=0;endpoint<5;++endpoint)
        for(std::size_t node=0;node<mappings[endpoint].size();++node)
            for(std::uint16_t index=0x30AE;index<=0x30B1;++index)
            {
                std::vector<std::uint8_t> request{3,static_cast<std::uint8_t>(node)};
                append_u16(request,index); append_u32(request,0);
                append_u16(request,arm::ArmUdpCodec::crc16_ccitt_false(request.data(),request.size()));
                common::UdpEndpoint source;
                if (!sdo_host.send_to({config.endpoints[endpoint].bind_ip,config.sdo_port},request.data(),request.size()) ||
                    !sdo_host.receive(buffer.data(),buffer.size(),received,nullptr,&source) ||
                    !require(received==10 && source.port==config.sdo_port &&
                             source.ip==config.endpoints[endpoint].bind_ip && buffer[0]==3 &&
                             buffer[1]==node && u16(2)==index && i32(4)==0 &&
                             u16(8)==arm::ArmUdpCodec::crc16_ccitt_false(buffer.data(),8),
                             "Arm SDO diagnostics round-trip failed.")) return false;
            }
    const std::string prefix=side==arm::ArmSide::Left ? "left_arm_joint_" : "right_arm_joint_";
    std::array<int,7> qpos{},dof{},actuator{};
    const auto model=runtime.model_info();
    for(std::size_t i=0;i<7;++i)
    {
        const auto name=prefix+std::to_string(i+1);
        for(const auto& joint:model.joints) if(joint.name==name) {qpos[i]=joint.qpos_address;dof[i]=joint.dof_address;}
        for(const auto& motor:model.actuators) if(motor.name==name+"_ctrl") actuator[i]=motor.id;
    }
    const arm::ArmTransmission::AxisArray pose{0.1,-0.2,0.15,0.1,-0.1,0.12,-0.08};
    const auto set_pose = [&](const arm::ArmTransmission::AxisArray& p) {
        return runtime.write_native([&](const mjModel*,mjData* data) {
            for(std::size_t i=0;i<7;++i) { data->qpos[qpos[i]]=p[i]; data->qvel[dof[i]]=0; }
        },true,&error);
    };
    std::uint32_t seq=1;
    const auto send_all = [&](std::uint16_t word,std::int8_t mode) {
        for(std::size_t i=0;i<5;++i)
        {
            auto packet=i<3 ? control_frame(seq++,word,mode,0) :
                dual_csp_control_frame(seq++,{word,word},0,0,{mode,mode});
            if(!hosts[i].send_to({config.endpoints[i].bind_ip,config.endpoints[i].bind_port},packet.data(),packet.size())) return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        return runtime.step(1,&error);
    };
    const auto check_feedback = [&](std::int8_t mode,std::uint16_t status,
                                    const arm::ArmTransmission::AxisArray& expected) {
        arm::ArmTransmission transmission(side);
        const auto motor_positions=transmission.joint_to_motor(expected);
        for(std::size_t endpoint=0;endpoint<5;++endpoint)
        {
            bool matched=false;
            const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
            while(std::chrono::steady_clock::now()<deadline && !matched)
            {
                if(!hosts[endpoint].receive(buffer.data(),buffer.size(),received)) continue;
                const auto count=mappings[endpoint].size();
                if(received!=14+30*count) continue;
                matched=u16(4+30*count)==arm::ArmUdpCodec::crc16_ccitt_false(buffer.data(),4+30*count);
                for(std::size_t slot=0;slot<count;++slot)
                {
                    const auto motor=mappings[endpoint][slot];
                    const auto p=4+30*slot;
                    const double low=motor<3 ? motor_positions[motor] : expected[motor];
                    matched=matched && buffer[p+2]==mode && u16(p+3)==status &&
                        std::abs(i32(p+5)/1e6-motor_positions[motor])<1e-5 &&
                        std::abs(i32(p+15)/1e6-low)<1e-5 &&
                        buffer[p+21]==48 && static_cast<std::int8_t>(buffer[p+22])==-5 && buffer[p+23]==37;
                }
            }
            if(!require(matched,"Extended feedback / homing state or zero offset mismatch.")) return false;
        }
        return true;
    };
    // Start Homing at a nonzero pose, re-publish that exact pose after the
    // dynamics step to distinguish encoder offset from physics integration.
    if(!set_pose(pose) || !send_all(0x000F,6) || !set_pose(pose) ||
       !send_all(0x001F,6)) return false;
    const auto homed_snapshot = runtime.latest_snapshot();
    for (std::size_t i = 0; i < 7; ++i)
        if (!require(std::abs(homed_snapshot.qpos[qpos[i]] - pose[i]) < 0.005,
                     "Homing teleported the physical joint pose.")) return false;
    if(!set_pose(pose) ||
       !check_feedback(6,0x1027,{})) return false;
    auto moved=pose; for(auto& v:moved) v+=0.01;
    arm::ArmTransmission::AxisArray delta{}; delta.fill(0.01);
    // Holding start high must not repeatedly redefine zero.
    if(!set_pose(moved) || !send_all(0x001F,6) || !set_pose(moved) ||
       !check_feedback(6,0x1027,delta)) return false;
    if(!send_all(0x000F,6) || !send_all(0x0000,6) ||
       !set_pose(moved) || !check_feedback(6,0x1040,delta)) return false;
    // CSP target zero must refer to the homed pose, not model zero.
    if(!set_pose(pose) || !send_all(0x000F,8)) return false;
    const auto snapshot=runtime.latest_snapshot();
    for(auto id:actuator)
        if(!require(std::abs(snapshot.controls[id])<1e-7,"CSP ignored homing offset.")) return false;
    if(!runtime.reset(&error) || !set_pose(pose) || !check_feedback(0,0x40,pose)) return false;
    return true;
}

// Use a paused runtime: wall-clock delivery must not advance physics, and
// each explicit step exposes precisely which command reached the actuator.
bool transport_delay_integration()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    using namespace std::chrono_literals;
    Runtime runtime;
    std::string error;
    if (!runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error)) return require(false, error);
    arm::ArmPluginConfig config;
    config.sdo_port = 0;
    config.side = arm::ArmSide::Left;
    config.command_timeout_s = 0.5;
    config.command_delay_s.fill(0.08);
    config.feedback_delay_s.fill(0.08);
    const std::array<std::vector<std::size_t>, 5> mappings{{{0},{1},{2},{3,4},{5,6}}};
    std::array<common::UdpSocket, 5> hosts;
    for (std::size_t i=0; i<5; ++i)
    {
        common::UdpSocket probe;
        if (!hosts[i].open_and_bind({"127.0.0.1",0},10,&error) ||
            !probe.open_and_bind({"127.0.0.1",0},10,&error)) return require(false,error);
        config.endpoints[i].bind_port=probe.local_port();
        config.endpoints[i].peer_port=hosts[i].local_port();
        config.endpoints[i].motor_indices=mappings[i];
    }
    for (double bad : {-0.001, 0.101, std::numeric_limits<double>::quiet_NaN()})
    {
        auto invalid=config; invalid.command_delay_s[0]=bad;
        arm::ArmPlugin rejected(invalid);
        if (!require(!rejected.start(runtime,&error),"Invalid arm delay accepted.")) return false;
    }
    {
        auto invalid=config; invalid.feedback_delay_s[4]=0.04;
        arm::ArmPlugin rejected(invalid);
        if (!require(!rejected.start(runtime,&error),"Unequal board delay accepted.")) return false;
    }
    arm::ArmPlugin plugin(config);
    if (!plugin.start(runtime,&error)) return require(false,error);
    int actuator=-1;
    for(const auto& a:runtime.model_info().actuators)
        if(a.name=="left_arm_joint_1_ctrl") actuator=a.id;
    if (!require(actuator>=0,"Missing arm actuator.")) return false;
    std::uint32_t sequence=1;
    const auto send = [&](std::uint16_t word, float torque, std::int8_t mode = 10) {
        const auto packet=control_frame(sequence++,word,mode,0,0,torque);
        if(!hosts[0].send_to({"127.0.0.1",config.endpoints[0].bind_port},packet.data(),packet.size(),&error)) return false;
        std::this_thread::sleep_for(5ms);
        return runtime.step(1,&error);
    };
    const auto expect = [&](double torque) {
        return require(std::abs(runtime.latest_snapshot().controls[actuator]+torque)<1e-6,
                       "Arm delayed torque / immediate stop mismatch.");
    };
    if(!send(0x000F,1) || !expect(1) || !send(0x000F,2) || !expect(1)) return false;
    std::this_thread::sleep_for(100ms);
    if(!runtime.step(1,&error) || !expect(2)) return false;
    if(!send(0x000F,3) || !expect(2) || !send(0,0) || !expect(0)) return false;
    std::this_thread::sleep_for(100ms);
    if(!runtime.step(1,&error) || !expect(0)) return false;
    // Sample age is observable on the actual wire; no timestamp rewriting.
    std::array<std::uint8_t,128> bytes{};std::size_t size=0;
    bool feedback=false;
    const auto end=std::chrono::steady_clock::now()+500ms;
    while(std::chrono::steady_clock::now()<end)
    {
        if(!hosts[0].receive(bytes.data(),bytes.size(),size) || size!=44) continue;
        std::uint64_t sampled=0;
        for(int i=0;i<8;++i) sampled|=std::uint64_t(bytes[size-8+i])<<(8*i);
        const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const auto status=std::uint16_t(bytes[7] | (bytes[8]<<8));
        if(status==0x40 && now>=static_cast<std::int64_t>(sampled) && now-sampled>=70000000ULL)
        { feedback=true;break; }
    }
    if(!require(feedback,"Arm feedback delay/sample timestamp or immediate stop status failed.")) return false;
    if(!send(0x000F,1) || !send(0x000F,4) || !runtime.reset(&error)) return false;
    std::this_thread::sleep_for(100ms);
    if(!runtime.step(1,&error) || !expect(0)) return false;
    // A reset must also permit fresh traffic.
    if(!send(0x000F,1) || !expect(1) || !send(0x000F,2,11) || !expect(2) ||
       !send(0x000F,3,11) || !expect(2)) return false;
    std::this_thread::sleep_for(100ms);
    return runtime.step(1,&error) && expect(3);
}

bool udp_integration()
{
    using sharpa::mujoco::plugins::arm::ArmSide;
    return udp_integration_for_side(
               ArmSide::Left, "left_arm_joint_1_ctrl", -1.0) &&
           udp_integration_for_side(
               ArmSide::Right, "right_arm_joint_1_ctrl", 1.0);
}
}  // namespace

int main()
{
    using sharpa::mujoco::plugins::arm::ArmSide;
    if (!protocol_and_transmission() || !udp_integration() ||
        !transport_delay_integration() ||
        !homing_and_extended_services(ArmSide::Left) ||
        !homing_and_extended_services(ArmSide::Right) ||
        !homing_and_extended_services(ArmSide::Left, 0.04)) return EXIT_FAILURE;
    std::cout << "Arm UDP protocol, transmission, CiA402, and integration tests passed.\n";
    return EXIT_SUCCESS;
}
