// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/plugins/wave_hand/ltc_protocol.h"
#include "sharpa_mujoco/plugins/wave_hand/wave_hand_plugin.h"
#include "sharpa_mujoco/plugins/wave_hand/wave_protocol.h"
#include "sharpa_mujoco/runtime.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <string>
#include <filesystem>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <mujoco/mujoco.h>

namespace
{
bool require(bool condition, const std::string& message)
{
    if (!condition) std::cerr << "FAILED: " << message << '\n';
    return condition;
}

bool codec_test()
{
    using namespace sharpa::mujoco::plugins::wave_hand;
    const std::array<std::uint8_t, 9> known{'1','2','3','4','5','6','7','8','9'};
    if (!require(ieee_crc32(known.data(), known.size()) == 0xCBF43926U,
                 "CRC32 known vector failed."))
        return false;

    WaveJointPacketV0 packet;
    packet.header.packet_type = WavePacketType::kJointData;
    packet.header.payload_version = 0;
    packet.header.side = WaveHandSide::kLeft;
    packet.header.control_source = 4;
    packet.header.control_mode = 13;
    packet.joints[0].angle = 0.25F;
    std::array<std::uint8_t, kWaveJointPacketV0Size> bytes{};
    WaveJointPacketV0 decoded;
    if (!require(encode_joint_packet_v0(packet, bytes), "Joint encode failed.") ||
        !require(!decode_joint_packet_v0(bytes.data(), bytes.size(), decoded),
                 "Uplink protocol version accepted as downlink."))
        return false;
    bytes[2] = 0;
    const std::uint32_t downlink_crc =
        ieee_crc32(bytes.data(), bytes.size() - 4);
    for (std::size_t i = 0; i < 4; ++i)
        bytes[bytes.size() - 4 + i] =
            static_cast<std::uint8_t>(downlink_crc >> (8U * i));
    if (!require(decode_joint_packet_v0(bytes.data(), bytes.size(), decoded),
                 "Joint downlink decode failed.") ||
        !require(std::abs(decoded.joints[0].angle - 0.25F) < 1e-6F,
                 "Joint round trip failed."))
        return false;
    bytes[100] ^= 1;
    if (!require(!decode_joint_packet_v0(bytes.data(), bytes.size(), decoded),
                 "Bad joint CRC accepted."))
        return false;

    LtcFrame frame;
    frame.command = 0x8004;
    std::vector<std::uint8_t> encoded;
    const std::vector<std::uint8_t> get_mode_golden{
        0xBB, 0xEE, 0x00, 0x00, 0x80, 0x04, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x04, 0xAB, 0xF3, 0xD6, 0xF4};
    if (!require(encode_ltc_frame(frame, encoded), "LTC encode failed.") ||
        !require(encoded == get_mode_golden,
                 "LTC GET_CONTROL_MODE golden vector changed."))
        return false;
    LtcStreamParser parser;
    const std::array<std::uint8_t, 3> garbage{1, 2, 3};
    if (!require(parser.push(garbage.data(), garbage.size()).empty(),
                 "Garbage yielded LTC frame.") ||
        !require(parser.push(encoded.data(), 5).empty(), "Half LTC frame decoded."))
        return false;
    auto frames = parser.push(encoded.data() + 5, encoded.size() - 5);
    if (!require(frames.size() == 1 && frames[0].command == frame.command,
                 "LTC stream reassembly failed."))
        return false;

    std::vector<std::uint8_t> two_frames = encoded;
    two_frames.insert(two_frames.end(), encoded.begin(), encoded.end());
    frames = parser.push(two_frames);
    if (!require(frames.size() == 2, "LTC sticky packets were not split."))
        return false;

    std::vector<std::uint8_t> corrupt = encoded;
    corrupt.back() ^= 1U;
    corrupt.insert(corrupt.end(), encoded.begin(), encoded.end());
    frames = parser.push(corrupt);
    if (!require(frames.size() == 1 && frames[0].command == frame.command,
                 "LTC parser did not recover after a corrupt frame."))
        return false;

    LtcFrame oversized;
    oversized.payload.assign(kDefaultLtcMaxPayloadSize + 1U, 0);
    return require(!encode_ltc_frame(oversized, encoded),
                   "Oversized LTC payload was accepted.");
}

bool send_all(int socket, const std::vector<std::uint8_t>& bytes)
{
    std::size_t sent = 0;
    while (sent < bytes.size())
    {
        const ssize_t result =
            ::send(socket, bytes.data() + sent, bytes.size() - sent, 0);
        if (result <= 0) return false;
        sent += static_cast<std::size_t>(result);
    }
    return true;
}

bool ltc_exchange(int socket,
                  std::uint16_t command,
                  std::vector<std::uint8_t> payload,
                  sharpa::mujoco::plugins::wave_hand::LtcFrame& response)
{
    using namespace sharpa::mujoco::plugins::wave_hand;
    LtcFrame request;
    request.command = command;
    request.payload = std::move(payload);
    std::vector<std::uint8_t> encoded;
    if (!encode_ltc_frame(request, encoded) || !send_all(socket, encoded))
        return false;
    std::array<std::uint8_t, 256> buffer{};
    LtcStreamParser parser;
    while (true)
    {
        const ssize_t received = ::recv(socket, buffer.data(), buffer.size(), 0);
        if (received <= 0) return false;
        const auto frames =
            parser.push(buffer.data(), static_cast<std::size_t>(received));
        if (!frames.empty())
        {
            response = frames.front();
            return response.command == command;
        }
    }
}

bool ltc_request(int socket,
                 std::uint16_t command,
                 std::vector<std::uint8_t> payload = {})
{
    sharpa::mujoco::plugins::wave_hand::LtcFrame response;
    return ltc_exchange(socket, command, std::move(payload), response) &&
           response.response_code == 0;
}

bool ltc_response_code(int socket,
                       std::uint16_t command,
                       std::vector<std::uint8_t> payload,
                       std::uint8_t expected_code)
{
    sharpa::mujoco::plugins::wave_hand::LtcFrame response;
    return ltc_exchange(socket, command, std::move(payload), response) &&
           response.response_code == expected_code;
}

bool ltc_request_payload(int socket,
                         std::uint16_t command,
                         std::vector<std::uint8_t> request_payload,
                         std::vector<std::uint8_t>& response_payload)
{
    using namespace sharpa::mujoco::plugins::wave_hand;
    LtcFrame response;
    if (!ltc_exchange(socket, command, std::move(request_payload), response))
        return false;
    response_payload = std::move(response.payload);
    return response.response_code == 0;
}

std::vector<std::uint8_t> float_be(float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return {static_cast<std::uint8_t>(bits >> 24U),
            static_cast<std::uint8_t>(bits >> 16U),
            static_cast<std::uint8_t>(bits >> 8U),
            static_cast<std::uint8_t>(bits)};
}

std::vector<std::uint8_t> float_array_parameter(
    std::uint8_t id,
    const std::array<float,
                     sharpa::mujoco::plugins::wave_hand::kWaveJointCount>& values)
{
    std::vector<std::uint8_t> payload{1, id};
    for (float value : values)
    {
        const auto bytes = float_be(value);
        payload.insert(payload.end(), bytes.begin(), bytes.end());
    }
    return payload;
}

std::array<std::uint8_t,
           sharpa::mujoco::plugins::wave_hand::kWaveJointPacketV0Size>
encode_downlink(
    const sharpa::mujoco::plugins::wave_hand::WaveJointPacketV0& command)
{
    using namespace sharpa::mujoco::plugins::wave_hand;
    std::array<std::uint8_t, kWaveJointPacketV0Size> bytes{};
    if (!encode_joint_packet_v0(command, bytes)) return {};
    bytes[2] = 0;
    const std::uint32_t crc = ieee_crc32(bytes.data(), bytes.size() - 4);
    for (std::size_t i = 0; i < 4; ++i)
        bytes[bytes.size() - 4 + i] =
            static_cast<std::uint8_t>(crc >> (8U * i));
    return bytes;
}

bool receive_heartbeat(
    sharpa::mujoco::plugins::common::UdpSocket& receiver,
    sharpa::mujoco::plugins::wave_hand::WaveHeartbeatV2& heartbeat)
{
    using namespace sharpa::mujoco::plugins::wave_hand;
    std::array<std::uint8_t, 512> buffer{};
    std::size_t received = 0;
    return receiver.receive(buffer.data(), buffer.size(), received) &&
           received == kWaveHeartbeatV2Size &&
           decode_heartbeat_v2(buffer.data(), received, heartbeat);
}

bool equality_active(sharpa::mujoco::Runtime& runtime, const char* name)
{
    bool active = false;
    const bool read = runtime.read_native(
        [&](const mjModel* model, const mjData* data) {
            const int id = mj_name2id(model, mjOBJ_EQUALITY, name);
            active = id >= 0 && data->eq_active[id] != 0;
        });
    return read && active;
}

bool step_runtime(sharpa::mujoco::Runtime& runtime,
                  std::size_t count,
                  std::string& error)
{
    return runtime.step(count, &error);
}

bool integration_test()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error), error))
        return false;

    common::UdpSocket heartbeat_receiver;
    common::UdpSocket feedback_receiver;
    common::UdpSocket command_sender;
    if (!heartbeat_receiver.open_and_bind({"127.0.0.1", 0}, 500, &error) ||
        !feedback_receiver.open_and_bind({"127.0.0.1", 0}, 500, &error) ||
        !command_sender.open_and_bind({"127.0.0.1", 0}, 100, &error))
        return require(false, error);

    wave_hand::WaveHandPluginConfig config;
    config.name = "hand_left_test";
    config.side = wave_hand::WaveHandSide::kLeft;
    config.device_ip = "127.0.51.10";
    config.sdk_ip = "127.0.0.1";
    config.serial = "SIM-WAVE-L-TEST";
    config.PN = "Wave01-MOCK";
    config.tcp_port = 59991;
    config.command_port = 50120;
    config.feedback_destination = feedback_receiver.local_port(&error);
    config.heartbeat_destination = heartbeat_receiver.local_port(&error);
    config.feedback_source_port = 10000;
    config.feedback_rate_hz = 200;
    config.heartbeat_rate_hz = 10;
    config.fingertip_tactile_supported = true;
    config.strict_command_state = false;
    wave_hand::WaveHandPlugin plugin(config);
    if (!require(plugin.start(runtime, &error), error)) return false;

    std::array<std::uint8_t, 512> receive_buffer{};
    std::size_t received = 0;
    const bool heartbeat_ok =
        heartbeat_receiver.receive(receive_buffer.data(), receive_buffer.size(), received) &&
        require(received == wave_hand::kWaveHeartbeatV2Size,
                "Heartbeat size is wrong.");
    wave_hand::WaveHeartbeatV2 heartbeat;
    const bool heartbeat_endpoints_ok =
        heartbeat_ok &&
        require(wave_hand::decode_heartbeat_v2(
                    receive_buffer.data(), received, heartbeat),
                "Heartbeat decode failed.") &&
        require(heartbeat.tactile_destination_ip ==
                    std::array<std::uint8_t, 4>{127, 0, 0, 1},
                "Tactile destination IP is wrong.") &&
        require(heartbeat.debug_destination_ip ==
                    std::array<std::uint8_t, 4>{127, 0, 0, 1},
                "Debug destination IP is wrong.") &&
        require(heartbeat.tactile_destination_port == 50011,
                "Left tactile destination port is wrong.") &&
        require(heartbeat.feature_config ==
                    wave_hand::kWaveFeatureFingertipTactile,
                "Fingertip tactile capability is missing.") &&
        require(heartbeat.debug_destination_port == 50005,
                "Left debug destination port is wrong.");
    wave_hand::WaveHeartbeatV2 invalid_heartbeat = heartbeat;
    invalid_heartbeat.feature_config = 0x80;
    std::array<std::uint8_t, wave_hand::kWaveHeartbeatV2Size>
        invalid_heartbeat_bytes{};
    const bool heartbeat_feature_validation_ok = require(
        !wave_hand::encode_heartbeat_v2(
            invalid_heartbeat, invalid_heartbeat_bytes),
        "Unknown heartbeat feature bit was accepted.");
    const bool feedback_ok =
        feedback_receiver.receive(receive_buffer.data(), receive_buffer.size(), received) &&
        require(received == wave_hand::kWaveJointPacketV0Size,
                "Feedback size is wrong.");

    const int tcp = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(config.tcp_port);
    ::inet_pton(AF_INET, config.device_ip.c_str(), &address.sin_addr);
    const bool lifecycle_ok =
        require(tcp >= 0 &&
                    ::connect(tcp, reinterpret_cast<const sockaddr*>(&address),
                              sizeof(address)) == 0,
                "TCP connect failed.") &&
        [&] {
            std::vector<std::uint8_t> payload;
            const std::vector<std::uint8_t> expected_parameters{
                1, 0, 1, 1, 1, 36, 1, 37, 1, 46, 1, 47, 1, 48,
                1, 49, 1, 50, 1, 51, 1, 52, 1, 53, 1, 54, 1, 55,
                1, 62, 1, 63, 1, 64, 2, 0,
            };
            return require(
                       ltc_request_payload(tcp, 0x8052, {}, payload) &&
                           payload == expected_parameters,
                       "GET_ALL_PARAM_INFO failed.") &&
                   require(
                       ltc_request_payload(tcp, 0x8053, {1, 1}, payload) &&
                           payload.size() == 38 && payload[0] == 1 &&
                           payload[1] == 1 && payload[26] == 9 &&
                           payload[27] == 0 && payload[28] == 1 &&
                           payload[29] == 0x0E,
                       "GET_PARAM_INFO failed.");
        }() &&
        require(ltc_request(tcp, 0x8005, {13}), "POSITION mode failed.") &&
        require(ltc_request(tcp, 0x802b, {4}), "SDK source failed.") &&
        require(ltc_request(tcp, 0x8003, {1}), "Start failed.");

    wave_hand::WaveJointPacketV0 command;
    command.header.packet_type = wave_hand::WavePacketType::kJointData;
    command.header.payload_version = 0;
    command.header.side = wave_hand::WaveHandSide::kLeft;
    command.header.control_source = 4;
    command.header.control_mode = 0;
    command.joints[0].angle = 100.0F;
    for (std::size_t i = 1; i < wave_hand::kWaveJointCount; ++i)
        command.joints[i].angle = 0.01F * static_cast<float>(i);
    ControlCommand right_sentinel;
    right_sentinel.replace_all = false;
    right_sentinel.named_values.push_back({"right_thumb_CMC_FE_ctrl", 0.33});
    if (!runtime.submit_control(right_sentinel, &error))
        return require(false, error);
    const auto command_bytes = encode_downlink(command);
    bool command_ok = command_bytes[0] != 0;
    command_ok =
        command_ok &&
        command_sender.send_to(
            {config.device_ip, config.command_port},
            command_bytes.data(), command_bytes.size(), &error);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    if (!step_runtime(runtime, 250, error))
        return require(false, error);
    const ModelInfo model = runtime.model_info();
    const PoseSnapshot snapshot = runtime.latest_snapshot();
    bool moved = true;
    const std::array<const char*, wave_hand::kWaveJointCount> suffixes{{
        "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA",
        "thumb_IP", "index_MCP_FE", "index_MCP_AA", "index_PIP",
        "index_DIP", "middle_MCP_FE", "middle_MCP_AA", "middle_PIP",
        "middle_DIP", "ring_MCP_FE", "ring_MCP_AA", "ring_PIP",
        "ring_DIP", "pinky_CMC", "pinky_MCP_FE", "pinky_MCP_AA",
        "pinky_PIP", "pinky_DIP",
    }};
    for (std::size_t i = 0; i < suffixes.size(); ++i)
    {
        const std::string joint_name = "left_" + std::string(suffixes[i]);
        const std::string actuator_name = joint_name + "_ctrl";
        const auto joint = std::find_if(
            model.joints.begin(), model.joints.end(),
            [&](const JointInfo& value) { return value.name == joint_name; });
        const auto actuator = std::find_if(
            model.actuators.begin(), model.actuators.end(),
            [&](const ActuatorInfo& value) {
                return value.name == actuator_name;
            });
        double expected = command.joints[i].angle;
        if (joint != model.joints.end() && joint->limited)
            expected = std::clamp(expected, joint->range_min, joint->range_max);
        // The firmware soft limit is 105 degrees while the MuJoCo model keeps
        // the 110-degree hard range for collision-safe physical motion.
        if (i == 0) expected = std::min(expected, 105.0 * mjPI / 180.0);
        moved = moved && joint != model.joints.end() &&
                actuator != model.actuators.end() &&
                std::abs(snapshot.controls[
                             static_cast<std::size_t>(actuator->id)] -
                         expected) <
                    1e-5;
    }
    const auto right_actuator = std::find_if(
        model.actuators.begin(), model.actuators.end(),
        [](const ActuatorInfo& value) {
            return value.name == "right_thumb_CMC_FE_ctrl";
        });
    const bool isolated =
        right_actuator != model.actuators.end() &&
        std::abs(snapshot.controls[
                     static_cast<std::size_t>(right_actuator->id)] -
                 0.33) <
            1e-5;
    if (tcp >= 0) ::close(tcp);

    plugin.stop();
    runtime.unload();
    return heartbeat_endpoints_ok && heartbeat_feature_validation_ok &&
           feedback_ok && lifecycle_ok &&
           require(command_ok, "Command send failed.") &&
           require(moved, "Not all 22 POSITION targets reached MuJoCo.") &&
           require(isolated, "Left hand command changed right hand controls.");
}

bool p0_state_and_fault_test()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error), error))
        return false;

    common::UdpSocket heartbeat_receiver;
    common::UdpSocket feedback_receiver;
    common::UdpSocket command_sender;
    if (!heartbeat_receiver.open_and_bind({"127.0.0.1", 0}, 500, &error) ||
        !feedback_receiver.open_and_bind({"127.0.0.1", 0}, 120, &error) ||
        !command_sender.open_and_bind({"127.0.0.1", 0}, 100, &error))
        return require(false, error);

    wave_hand::WaveHandPluginConfig config;
    config.name = "hand_p0_test";
    config.side = wave_hand::WaveHandSide::kLeft;
    config.device_ip = "127.0.53.10";
    config.sdk_ip = "127.0.0.1";
    config.serial = "SIM-WAVE-L-P0";
    config.tcp_port = 59993;
    config.command_port = 50320;
    config.feedback_destination = feedback_receiver.local_port(&error);
    config.heartbeat_destination = heartbeat_receiver.local_port(&error);
    config.feedback_source_port = 10000;
    config.feedback_rate_hz = 20;
    config.heartbeat_rate_hz = 20;
    config.strict_command_state = true;
    wave_hand::WaveHandPlugin plugin(config);
    if (!require(plugin.start(runtime, &error), error)) return false;

    const int tcp = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(config.tcp_port);
    ::inet_pton(AF_INET, config.device_ip.c_str(), &address.sin_addr);
    if (!require(tcp >= 0 &&
                     ::connect(tcp, reinterpret_cast<const sockaddr*>(&address),
                               sizeof(address)) == 0,
                 "P0 TCP connect failed."))
        return false;

    std::vector<std::uint8_t> payload;
    bool ok =
        require(ltc_request_payload(tcp, 0x8002, {}, payload) &&
                    payload == std::vector<std::uint8_t>{0},
                "Wave hand did not start PAUSED.") &&
        require(ltc_response_code(tcp, 0x800D, {}, 0x01) &&
                    ltc_response_code(tcp, 0x800D, {0, 0, 0, 0}, 0x01),
                "Invalid current parameter did not return INVALID_INPUT.") &&
        require(ltc_response_code(tcp, 0x8053, {}, 0x01) &&
                    ltc_response_code(tcp, 0x8053, {9, 9}, 0x01),
                "Invalid parameter-info request did not return INVALID_INPUT.") &&
        require(ltc_response_code(tcp, 0x8054, {1, 1}, 0x01) &&
                    ltc_response_code(
                        tcp, 0x8054, {9, 9, 0x3F, 0x80, 0, 0}, 0x01) &&
                    ltc_response_code(
                        tcp, 0x8054, {1, 1, 0, 0, 0, 0}, 0x01),
                "Invalid parameter write did not return INVALID_INPUT.") &&
        require(ltc_response_code(tcp, 0x8055, {1}, 0x01) &&
                    ltc_response_code(tcp, 0x8055, {9, 9}, 0x01),
                "Invalid parameter read did not return INVALID_INPUT.") &&
        require(equality_active(runtime, "brake_left_thumb_CMC_FE"),
                "Initial PAUSED state did not engage the brake.") &&
        require(ltc_request_payload(tcp, 0x8014, {}, payload) &&
                    payload == std::vector<std::uint8_t>{1},
                "UDP joint feedback did not default enabled.") &&
        require(ltc_request(tcp, 0x8015, {0}),
                "UDP joint feedback disable failed.");

    std::array<std::uint8_t, 512> buffer{};
    std::size_t received = 0;
    while (feedback_receiver.receive(buffer.data(), buffer.size(), received))
    {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    ok = ok && require(
                   !feedback_receiver.receive(buffer.data(), buffer.size(), received),
                   "Joint feedback continued while disabled.") &&
         require(ltc_request(tcp, 0x8015, {1}),
                 "UDP joint feedback enable failed.") &&
         require(feedback_receiver.receive(buffer.data(), buffer.size(), received) &&
                     received == wave_hand::kWaveJointPacketV0Size,
                 "Joint feedback did not resume.");

    wave_hand::WaveHeartbeatV2 heartbeat;
    ok = ok && require(receive_heartbeat(heartbeat_receiver, heartbeat),
                       "Heartbeat stopped with joint feedback.") &&
         require(ltc_request(tcp, 0x802b, {4}), "SDK source failed.") &&
         require(ltc_request(tcp, 0x8005, {13}), "POSITION mode failed.") &&
         require(ltc_request(tcp, 0x8003, {1}), "P0 start failed.") &&
         require(!equality_active(runtime, "brake_left_thumb_CMC_FE"),
                 "RUNNING state did not release the brake.") &&
         require(ltc_response_code(tcp, 0x8005, {13}, 0x0D),
                 "Running mode change did not return OPERATION_NOT_ALLOWED.") &&
         require(ltc_response_code(tcp, 0x800D, {0x3F, 0, 0, 0}, 0x0D),
                 "Running current change did not return OPERATION_NOT_ALLOWED.") &&
         require(ltc_response_code(
                     tcp, 0x8054, {1, 1, 0x3F, 0, 0, 0}, 0x0D),
                 "Running parameter change did not return OPERATION_NOT_ALLOWED.") &&
         require(ltc_request_payload(tcp, 0x800C, {}, payload) &&
                     payload == std::vector<std::uint8_t>({0x3F, 0x80, 0, 0}),
                 "Rejected current change modified the parameter.") &&
         require(ltc_request_payload(tcp, 0x8055, {1, 1}, payload) &&
                     payload == std::vector<std::uint8_t>({0x3F, 0x80, 0, 0}),
                 "Rejected speed change modified the parameter.") &&
         require(ltc_request(tcp, 0x8003, {0}), "P0 stop failed.") &&
         require(equality_active(runtime, "brake_left_thumb_CMC_FE"),
                 "PAUSED state did not re-engage the brake.") &&
         require(ltc_request(tcp, 0x8005, {10}) &&
                     ltc_request(tcp, 0x8005, {14}) &&
                     ltc_request(tcp, 0x8005, {15}) &&
                     ltc_response_code(tcp, 0x8005, {12}, 0x05) &&
                     ltc_request(tcp, 0x8005, {13}),
                 "Supported or unsupported mode handling is wrong.");

    wave_hand::WaveJointPacketV0 command;
    command.header.packet_type = wave_hand::WavePacketType::kJointData;
    command.header.payload_version = 0;
    command.header.side = wave_hand::WaveHandSide::kLeft;
    command.header.control_source = 4;
    command.header.control_mode = 13;
    command.joints[0].angle = 0.4F;
    auto command_bytes = encode_downlink(command);
    ok = ok && require(ltc_request(tcp, 0x8003, {1}),
                       "Restart before command validation failed.");

    const ModelInfo model = runtime.model_info();
    const auto actuator = std::find_if(
        model.actuators.begin(), model.actuators.end(),
        [](const ActuatorInfo& value) {
            return value.name == "left_thumb_CMC_FE_ctrl";
        });
    if (!require(actuator != model.actuators.end(), "Wave actuator missing."))
        return false;
    const std::size_t actuator_id = static_cast<std::size_t>(actuator->id);

    ok = ok && require(command_sender.send_to(
                           {config.device_ip, config.command_port},
                           command_bytes.data(), command_bytes.size(), &error),
                       "Valid P0 command send failed.");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ok = ok && require(step_runtime(runtime, 80, error), error);
    const double accepted_target = runtime.latest_snapshot().controls[actuator_id];
    ok = ok && require(std::abs(accepted_target - 0.4) < 1e-5,
                       "Valid P0 command was not applied.");

    const auto joint = std::find_if(
        model.joints.begin(), model.joints.end(),
        [](const JointInfo& value) {
            return value.name == "left_thumb_CMC_FE";
        });
    if (!require(joint != model.joints.end(), "Wave joint missing.")) return false;
    // Task stop/start leaves the SDK session alive. Reset must discard its old
    // targets, then recover from ordinary joint packets without a TCP restart.
    for (int cycle = 0; ok && cycle < 2; ++cycle)
    {
        ok = require(runtime.reset(&error), error) &&
             require(equality_active(runtime, "brake_left_thumb_CMC_FE"),
                     "Reset did not restore the startup brake.") &&
             require(step_runtime(runtime, 20, error), error) &&
             require(std::abs(runtime.latest_snapshot().controls[actuator_id]) < 1e-5,
                     "Reset replayed a pre-reset target.") &&
             require(ltc_request_payload(tcp, 0x8002, {}, payload) &&
                         payload == std::vector<std::uint8_t>{1},
                     "Reset lost the active SDK session.") &&
             require(ltc_request(tcp, 0x8003, {1}),
                     "Idempotent start after reset failed.") &&
             require(equality_active(runtime, "brake_left_thumb_CMC_FE"),
                     "Reset unlocked without a new joint command.");
        command.header.control_source = 1;
        command_bytes = encode_downlink(command);
        ok = ok && require(command_sender.send_to(
                    {config.device_ip, config.command_port},
                    command_bytes.data(), command_bytes.size(), &error), error);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ok = ok && require(step_runtime(runtime, 2, error), error) &&
             require(equality_active(runtime, "brake_left_thumb_CMC_FE"),
                     "Invalid command unlocked the reset hand.");
        command.header.control_source = 4;
        command_bytes = encode_downlink(command);
        ok = ok && require(command_sender.send_to(
                    {config.device_ip, config.command_port},
                    command_bytes.data(), command_bytes.size(), &error), error);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ok = ok && require(step_runtime(runtime, 80, error), error) &&
             require(!equality_active(runtime, "brake_left_thumb_CMC_FE"),
                     "Fresh command did not release the reset hand.") &&
             require(std::abs(runtime.latest_snapshot().controls[actuator_id] - 0.4) < 1e-5,
                     "Fresh target was not applied after reset.") &&
             require(runtime.latest_snapshot().qpos[joint->qpos_address] > 0.05,
                     "Hand remained physically locked after reset.");
    }
    // Also exercise observer/updater ordering with the physics worker active.
    ok = ok && require(runtime.start(&error), error);
    for (int cycle = 0; ok && cycle < 3; ++cycle)
    {
        ok = require(runtime.reset(&error), error);
        bool recovered = false;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(2);
        while (ok && std::chrono::steady_clock::now() < deadline)
        {
            ok = require(command_sender.send_to(
                {config.device_ip, config.command_port},
                command_bytes.data(), command_bytes.size(), &error), error);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (!equality_active(runtime, "brake_left_thumb_CMC_FE") &&
                runtime.latest_snapshot().qpos[joint->qpos_address] > 0.05)
            {
                recovered = true;
                break;
            }
        }
        ok = require(recovered, "Running simulation did not recover after reset.") && ok;
    }
    runtime.pause();
    ok = ok && require(ltc_request(tcp, 0x8003, {0}),
                       "Pause before reset failed.") &&
         require(runtime.reset(&error), error) &&
         require(step_runtime(runtime, 2, error), error) &&
         require(command_sender.send_to({config.device_ip, config.command_port},
                     command_bytes.data(), command_bytes.size(), &error), error);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ok = ok && require(step_runtime(runtime, 20, error), error) &&
         require(equality_active(runtime, "brake_left_thumb_CMC_FE"),
                 "Reset resumed an explicitly paused hand.") &&
         require(ltc_request(tcp, 0x8003, {1}),
                 "Explicit resume after paused reset failed.") &&
         require(!equality_active(runtime, "brake_left_thumb_CMC_FE"),
                 "Explicit resume did not release the reset brake.");
    ok = ok && require(ltc_request(tcp, 0x8003, {0}),
                       "Stop before safe resume failed.");
    const double held_position =
        runtime.latest_snapshot().qpos[static_cast<std::size_t>(
            joint->qpos_address)];
    ok = ok && require(ltc_request(tcp, 0x8003, {1}),
                       "Safe resume failed.") &&
         require(std::abs(runtime.latest_snapshot().controls[actuator_id] -
                          held_position) < 1e-5,
                 "Resume reused the stale pre-stop target.");
    const double resumed_target = runtime.latest_snapshot().controls[actuator_id];

    command.header.control_source = 1;
    command.joints[0].angle = 0.1F;
    command_bytes = encode_downlink(command);
    ok = ok && require(command_sender.send_to(
                           {config.device_ip, config.command_port},
                           command_bytes.data(), command_bytes.size(), &error),
                       "Mismatched source command send failed.");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ok = ok && require(step_runtime(runtime, 80, error), error) &&
         require(std::abs(runtime.latest_snapshot().controls[actuator_id] -
                          resumed_target) < 1e-5,
                 "Mismatched command source was applied.");

    ok = ok && require(ltc_response_code(tcp, 0x802b, {0}, 0x0D),
                       "Running source change was not rejected.") &&
         require(ltc_request_payload(tcp, 0x802a, {}, payload) &&
                     payload == std::vector<std::uint8_t>{4},
                 "Rejected running source change modified the source.") &&
         require(ltc_request(tcp, 0x8003, {0}),
                 "Pause before APP source failed.") &&
         require(ltc_request(tcp, 0x802b, {0}), "APP source failed.") &&
         require(ltc_request(tcp, 0x8003, {1}),
                 "Resume after APP source failed.");
    command.header.control_source = 0;
    command.joints[0].angle = 0.3F;
    command_bytes = encode_downlink(command);
    ok = ok && require(command_sender.send_to(
                           {config.device_ip, config.command_port},
                           command_bytes.data(), command_bytes.size(), &error),
                       "APP source command send failed.");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ok = ok && require(step_runtime(runtime, 80, error), error) &&
         require(std::abs(runtime.latest_snapshot().controls[actuator_id] - 0.3) <
                     1e-5,
                 "Matching APP source command was not applied.");

    ok = ok && require(ltc_request(tcp, 0x8003, {0}),
                       "Pause before IDLE source failed.") &&
         require(ltc_request(tcp, 0x802b, {7}), "IDLE source failed.") &&
         require(ltc_request(tcp, 0x8003, {1}),
                 "Resume after IDLE source failed.");
    const double idle_target =
        runtime.latest_snapshot().controls[actuator_id];
    command.header.control_source = 7;
    command.joints[0].angle = 0.2F;
    command_bytes = encode_downlink(command);
    ok = ok && require(command_sender.send_to(
                           {config.device_ip, config.command_port},
                           command_bytes.data(), command_bytes.size(), &error),
                       "IDLE source command send failed.");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ok = ok && require(runtime.step(1, &error), error) &&
         require(std::abs(runtime.latest_snapshot().controls[actuator_id] -
                          idle_target) < 1e-5,
                 "IDLE source command was applied.");

    ok = ok && require(ltc_request(tcp, 0x8003, {0}),
                       "Pause before SDK source restore failed.") &&
         require(ltc_request(tcp, 0x802b, {4}),
                 "SDK source restore failed.") &&
         require(ltc_request(tcp, 0x8003, {1}),
                 "Resume after SDK source restore failed.");

    ok = ok && require(!plugin.inject_fault(0, 1U, &error),
                       "Zero fault code was accepted.") &&
         require(!plugin.inject_fault(0x0343, 1U << 22U, &error),
                 "Out-of-range fault mask was accepted.") &&
         require(plugin.inject_fault(0x0343, 0x00000003U, &error),
                       error) &&
         require(plugin.inject_fault(0x0343, 0x00000004U, &error),
                 "Duplicate fault injection failed.") &&
         require(plugin.inject_fault(0x0501, 0, &error),
                 "Warning fault injection failed.") &&
         require(plugin.inject_fault(0x0344, 0x00000008U, &error),
                 "Second error fault injection failed.") &&
         require(runtime.reset(&error), error) &&
         require(step_runtime(runtime, 2, error), error) &&
         require(equality_active(runtime, "brake_left_thumb_CMC_FE"),
                 "Reset unlocked a faulted hand.") &&
         require(ltc_request_payload(tcp, 0x8026, {}, payload) &&
                     payload == std::vector<std::uint8_t>(
                                    {0x03, 0x43, 0, 0, 0, 0x07,
                                     0x03, 0x44, 0, 0, 0, 0x08,
                                     0x05, 0x01, 0, 0, 0, 0}),
                 "LTC fault records are wrong or unstable.") &&
         require(ltc_response_code(tcp, 0x8003, {1}, 0x0D),
                 "Faulted hand resumed.") &&
         require(ltc_response_code(tcp, 0x802b, {0}, 0x0D),
                 "Faulted hand changed control source.") &&
         require(ltc_request_payload(tcp, 0x802a, {}, payload) &&
                     payload == std::vector<std::uint8_t>{4},
                 "Rejected fault-state source change modified the source.") &&
         require(ltc_response_code(tcp, 0x800D, {0x3F, 0, 0, 0}, 0x0D),
                 "Faulted hand changed current coefficient.") &&
         require(ltc_response_code(
                     tcp, 0x8054, {1, 1, 0x3F, 0, 0, 0}, 0x0D),
                 "Faulted hand changed speed coefficient.") &&
         require(ltc_request_payload(tcp, 0x800C, {}, payload) &&
                     payload == std::vector<std::uint8_t>({0x3F, 0x80, 0, 0}),
                 "Rejected fault-state current change modified the parameter.") &&
         require(ltc_request_payload(tcp, 0x8055, {1, 1}, payload) &&
                     payload == std::vector<std::uint8_t>({0x3F, 0x80, 0, 0}),
                 "Rejected fault-state speed change modified the parameter.") &&
         require(equality_active(runtime, "brake_left_thumb_CMC_FE"),
                 "FAULT state did not engage the brake.");

    const double faulted_target =
        runtime.latest_snapshot().controls[actuator_id];
    command.header.control_source = 4;
    command.joints[0].angle = 0.2F;
    command_bytes = encode_downlink(command);
    ok = ok && require(command_sender.send_to(
                           {config.device_ip, config.command_port},
                           command_bytes.data(), command_bytes.size(), &error),
                       "Fault-state command send failed.");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ok = ok && require(runtime.step(1, &error), error) &&
         require(std::abs(runtime.latest_snapshot().controls[actuator_id] -
                          faulted_target) < 1e-5,
                 "Faulted hand accepted a position command.");

    for (int attempt = 0; attempt < 100; ++attempt)
    {
        if (receive_heartbeat(heartbeat_receiver, heartbeat) &&
            heartbeat.status.error_code == 0x0344)
            break;
    }
    ok = ok && require(heartbeat.status.error_code == 0x0344 &&
                           heartbeat.status.error_joint == 0x00000008U &&
                           !heartbeat.header.running,
                       "Heartbeat did not select the latest equal-level fault.");

    ok = ok && require(plugin.inject_fault(0x0581, 0x00000010U, &error),
                       "Critical fault injection failed.");
    for (int attempt = 0; attempt < 100; ++attempt)
    {
        if (receive_heartbeat(heartbeat_receiver, heartbeat) &&
            heartbeat.status.error_code == 0x0581)
            break;
    }
    ok = ok && require(heartbeat.status.error_code == 0x0581 &&
                           heartbeat.status.error_joint == 0x00000010U &&
                           !heartbeat.header.running,
                       "Heartbeat did not select the most severe fault.") &&
         require(ltc_response_code(tcp, 0x804F, {}, 0x01),
                 "Malformed CLEAR_ERROR_CODE was accepted.") &&
         require(ltc_request(tcp, 0x804F, {0, 0, 0, 0}),
                 "CLEAR_ERROR_CODE failed.") &&
         require(ltc_request_payload(tcp, 0x8026, {}, payload) &&
                     payload.empty(),
                 "Fault records remained after clear.") &&
         require(ltc_request_payload(tcp, 0x8002, {}, payload) &&
                     payload == std::vector<std::uint8_t>{0},
                 "Cleared hand did not remain PAUSED.") &&
         require(ltc_request(tcp, 0x8003, {1}),
                 "Cleared hand could not restart.");

    const bool restart_fault_injected =
        plugin.inject_fault(0x0343, 1U, &error);
    ok = require(restart_fault_injected,
                 "Restart regression fault injection failed.") &&
         ok;
    ::close(tcp);
    plugin.stop();

    ok = require(runtime.reset(&error), "Reset after plugin stop failed.") && ok;
    const bool restarted = plugin.start(runtime, &error);
    ok = require(restarted, error) && ok;
    if (restarted)
    {
        const int restarted_tcp = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (restarted_tcp >= 0)
        {
            ok = require(
                     ::connect(restarted_tcp,
                               reinterpret_cast<const sockaddr*>(&address),
                               sizeof(address)) == 0,
                     "Restarted P0 TCP connect failed.") &&
                 require(ltc_request(restarted_tcp, 0x802B, {4}) &&
                             ltc_request(restarted_tcp, 0x8005, {13}) &&
                             ltc_request(restarted_tcp, 0x8003, {1}),
                         "Fault flag survived plugin restart.") &&
                 ok;
            ::close(restarted_tcp);
        }
        else
        {
            ok = require(false, "Restarted P0 TCP socket creation failed.") &&
                 ok;
        }
        plugin.stop();
    }
    runtime.unload();
    return ok;
}

bool advanced_control_modes_test()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error), error))
        return false;

    common::UdpSocket command_sender;
    if (!command_sender.open_and_bind({"127.0.0.1", 0}, 100, &error))
        return require(false, error);

    wave_hand::WaveHandPluginConfig config;
    config.name = "hand_advanced_control_test";
    config.side = wave_hand::WaveHandSide::kLeft;
    config.device_ip = "127.0.55.10";
    config.sdk_ip = "127.0.0.1";
    config.serial = "SIM-WAVE-L-CONTROL";
    config.tcp_port = 59995;
    config.command_port = 50520;
    config.heartbeat_destination = 1;
    config.feedback_rate_hz = 20;
    config.heartbeat_rate_hz = 1;
    config.strict_command_state = true;
    wave_hand::WaveHandPlugin plugin(config);
    if (!require(plugin.start(runtime, &error), error)) return false;

    const int tcp = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(config.tcp_port);
    ::inet_pton(AF_INET, config.device_ip.c_str(), &address.sin_addr);
    if (!require(tcp >= 0 &&
                     ::connect(tcp, reinterpret_cast<const sockaddr*>(&address),
                               sizeof(address)) == 0,
                 "Advanced control TCP connect failed."))
        return false;

    const ModelInfo model = runtime.model_info();
    const auto actuator = std::find_if(
        model.actuators.begin(), model.actuators.end(),
        [](const ActuatorInfo& value) {
            return value.name == "left_thumb_CMC_FE_ctrl";
        });
    const auto joint = std::find_if(
        model.joints.begin(), model.joints.end(),
        [](const JointInfo& value) {
            return value.name == "left_thumb_CMC_FE";
        });
    if (!require(actuator != model.actuators.end() && joint != model.joints.end(),
                 "Advanced control joint or actuator missing."))
        return false;
    const std::size_t actuator_id = static_cast<std::size_t>(actuator->id);
    const std::size_t qpos = static_cast<std::size_t>(joint->qpos_address);

    wave_hand::WaveJointPacketV0 command;
    command.header.packet_type = wave_hand::WavePacketType::kJointData;
    command.header.payload_version = 0;
    command.header.side = wave_hand::WaveHandSide::kLeft;
    command.header.control_source = 4;
    command.header.control_mode = 13;

    bool ok = require(ltc_request(tcp, 0x802B, {4}),
                      "Advanced control source failed.") &&
              require(ltc_request(tcp, 0x8054,
                                  {1, 1, 0x3D, 0x4C, 0xCC, 0xCD}),
                      "POSITION speed coefficient failed.") &&
              require(ltc_request(tcp, 0x8005, {13}),
                      "POSITION mode selection failed.") &&
              require(ltc_request(tcp, 0x8003, {1}),
                      "POSITION start failed.");
    std::array<float, wave_hand::kWaveJointCount> mit_kp{};
    mit_kp.fill(7.0F);
    ok = ok && require(
                   ltc_response_code(
                       tcp, 0x8054, float_array_parameter(62, mit_kp), 0x0D),
                   "RUNNING advanced parameter write was not rejected.") &&
         require(ltc_request(tcp, 0x8003, {0}),
                 "Pause before advanced parameter write failed.") &&
         require(ltc_request(tcp, 0x8054, float_array_parameter(62, mit_kp)),
                 "MIT kp array write failed.");
    std::vector<std::uint8_t> parameter_payload;
    const auto expected_kp = float_be(7.0F);
    ok = ok && require(ltc_request_payload(tcp, 0x8055, {1, 62},
                                           parameter_payload) &&
                           parameter_payload.size() ==
                               wave_hand::kWaveJointCount * sizeof(float) &&
                           std::equal(expected_kp.begin(), expected_kp.end(),
                                      parameter_payload.begin()),
                       "MIT kp array readback failed.") &&
         require(ltc_request(tcp, 0x8054,
                             {1, 64, 0x3F, 0x00, 0x00, 0x00}),
                 "Velocity filter alpha write failed.") &&
         require(ltc_request_payload(tcp, 0x8055, {1, 36},
                                     parameter_payload) &&
                     parameter_payload.size() ==
                         wave_hand::kWaveJointCount * sizeof(float),
                 "Soft minimum array read failed.") &&
         require(ltc_response_code(
                     tcp, 0x8054, float_array_parameter(36, mit_kp), 0x0D),
                 "Read-only soft minimum write was accepted.") &&
         require(ltc_request(tcp, 0x8003, {1}),
                 "Resume after advanced parameter write failed.");
    command.joints[0].angle = 1.0F;
    const double position_start = runtime.latest_snapshot().qpos[qpos];
    auto bytes = encode_downlink(command);
    ok = ok && require(command_sender.send_to(
                           {config.device_ip, config.command_port},
                           bytes.data(), bytes.size(), &error),
                       "POSITION command send failed.") &&
         require(step_runtime(runtime, 1, error), error);
    const double first_target = runtime.latest_snapshot().controls[actuator_id];
    ok = ok && require(std::abs(first_target - 1.0) > 0.1 &&
                           std::abs(first_target - position_start) < 0.01,
                       "POSITION target did not use acceleration/speed ramp.") &&
         require(step_runtime(runtime, 1000, error), error) &&
         require(std::abs(runtime.latest_snapshot().controls[actuator_id] - 1.0) <
                     1e-4,
                 "POSITION target did not converge.");

    double original_force_max = 0.0;
    runtime.read_native([&](const mjModel* native_model, const mjData*) {
        original_force_max =
            native_model->actuator_forcerange[actuator_id * 2 + 1];
    });
    ok = ok && require(ltc_request(tcp, 0x8003, {0}),
                       "Pause before current limit failed.") &&
         require(ltc_request(tcp, 0x800D, float_be(0.2F)),
                 "Current coefficient write failed.") &&
         require(ltc_request(tcp, 0x8003, {1}),
                 "Resume for current limit failed.") &&
         require(step_runtime(runtime, 80, error), error);
    double limited_force_max = 0.0;
    runtime.read_native([&](const mjModel* native_model, const mjData*) {
        limited_force_max =
            native_model->actuator_forcerange[actuator_id * 2 + 1];
    });
    ok = ok && require(std::abs(limited_force_max - original_force_max * 0.2) <
                           1e-2,
                       "Current coefficient did not scale actuator force range.");

    ok = ok && require(ltc_request(tcp, 0x8003, {0}),
                       "Pause before ZERO_FORCE failed.") &&
         require(ltc_request(tcp, 0x8005, {10}),
                 "ZERO_FORCE mode selection failed.") &&
         require(ltc_request(tcp, 0x8003, {1}),
                 "ZERO_FORCE start failed.") &&
         require(step_runtime(runtime, 2, error), error) &&
         require(std::abs(runtime.latest_snapshot().controls[actuator_id]) <
                     1e-8,
                 "ZERO_FORCE produced active torque.");

    ok = ok && require(ltc_request(tcp, 0x8003, {0}),
                       "Pause before MIT failed.") &&
         require(ltc_request(tcp, 0x800D, float_be(1.0F)),
                 "MIT current coefficient restore failed.") &&
         require(ltc_request(tcp, 0x8005, {15}),
                 "MIT mode selection failed.") &&
         require(ltc_request(tcp, 0x8003, {1}), "MIT start failed.");
    const float mit_position =
        static_cast<float>(runtime.latest_snapshot().qpos[qpos]);
    command.header.control_mode = 15;
    command.joints[0].angle = mit_position + 0.01F;
    command.joints[0].velocity = static_cast<float>(
        runtime.latest_snapshot().qvel[static_cast<std::size_t>(
            joint->dof_address)]);
    command.joints[0].torque = 0.1F;
    const float expected_mit_torque =
        command.joints[0].torque +
        7.0F * (command.joints[0].angle - mit_position);
    bytes = encode_downlink(command);
    ok = ok && require(command_sender.send_to(
                           {config.device_ip, config.command_port},
                           bytes.data(), bytes.size(), &error),
                       "MIT command send failed.");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ok = ok && require(step_runtime(runtime, 1, error), error);
    const double mit_torque = runtime.latest_snapshot().controls[actuator_id];
    ok = ok && require(std::abs(mit_torque - expected_mit_torque) < 0.03,
                       "MIT PD plus feed-forward formula is wrong.");
    command.joints[0].angle = 100.0F;
    command.joints[0].torque = 100.0F;
    bytes = encode_downlink(command);
    ok = ok && require(command_sender.send_to(
                           {config.device_ip, config.command_port},
                           bytes.data(), bytes.size(), &error),
                       "MIT clamp command send failed.");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ok = ok && require(step_runtime(runtime, 1, error), error) &&
         require(runtime.latest_snapshot().controls[actuator_id] <=
                     original_force_max + 1e-6,
                 "MIT torque exceeded actuator force limit.");

    ok = ok && require(ltc_request(tcp, 0x8003, {0}),
                       "Pause after MIT failed.") &&
         require(equality_active(runtime, "brake_left_thumb_CMC_FE"),
                 "MIT pause did not engage the brake.") &&
         require(step_runtime(runtime, 1, error), error) &&
         require(std::abs(runtime.latest_snapshot().controls[actuator_id]) <
                     1e-8,
                 "MIT pause did not clear active torque.") &&
         require(ltc_request(tcp, 0x8005, {14}),
                 "ADMITTANCE mode selection failed.") &&
         require(ltc_request(tcp, 0x8003, {1}),
                 "ADMITTANCE start failed.") &&
         require(step_runtime(runtime, 2, error), error);
    bool position_actuator_restored = false;
    runtime.read_native([&](const mjModel* native_model, const mjData*) {
        position_actuator_restored =
            native_model->actuator_biastype[actuator_id] == mjBIAS_AFFINE;
    });
    ok = ok && require(position_actuator_restored,
                       "ADMITTANCE did not restore the position actuator.");
    const double admittance_start =
        runtime.latest_snapshot().controls[actuator_id];
    runtime.write_native([&](const mjModel*, mjData* data) {
        data->qfrc_applied[joint->dof_address] = 5.0;
    });
    ok = ok && require(step_runtime(runtime, 50, error), error) &&
         require(std::abs(runtime.latest_snapshot().controls[actuator_id] -
                          admittance_start) > 1e-4,
                 "ADMITTANCE did not react to joint torque feedback.");
    runtime.write_native([&](const mjModel*, mjData* data) {
        data->qfrc_applied[joint->dof_address] = 0.0;
    });

    ::close(tcp);
    plugin.stop();
    runtime.unload();
    return ok;
}

bool dual_hand_fault_isolation_test()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error), error))
        return false;

    wave_hand::WaveHandPluginConfig left_config;
    left_config.name = "hand_left_fault_isolation";
    left_config.side = wave_hand::WaveHandSide::kLeft;
    left_config.device_ip = "127.0.54.10";
    left_config.sdk_ip = "127.0.0.1";
    left_config.serial = "SIM-WAVE-L-ISOLATION";
    left_config.tcp_port = 59994;
    left_config.command_port = 50420;
    left_config.heartbeat_destination = 1;
    left_config.feedback_rate_hz = 20;
    left_config.heartbeat_rate_hz = 1;

    wave_hand::WaveHandPluginConfig right_config = left_config;
    right_config.name = "hand_right_fault_isolation";
    right_config.side = wave_hand::WaveHandSide::kRight;
    right_config.device_ip = "127.0.54.20";
    right_config.serial = "SIM-WAVE-R-ISOLATION";
    right_config.command_port = 50430;

    wave_hand::WaveHandPlugin left(left_config);
    wave_hand::WaveHandPlugin right(right_config);
    if (!require(left.start(runtime, &error), error) ||
        !require(right.start(runtime, &error), error))
        return false;
    if (!require(left.inject_fault(0x0343, 1U, &error), error))
        return false;

    const auto connect_client = [](const wave_hand::WaveHandPluginConfig& config) {
        const int socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(config.tcp_port);
        ::inet_pton(AF_INET, config.device_ip.c_str(), &address.sin_addr);
        return socket >= 0 &&
                       ::connect(socket,
                                 reinterpret_cast<const sockaddr*>(&address),
                                 sizeof(address)) == 0
                   ? socket
                   : -1;
    };
    const int left_tcp = connect_client(left_config);
    const int right_tcp = connect_client(right_config);
    std::vector<std::uint8_t> left_faults;
    std::vector<std::uint8_t> right_faults;
    const bool isolated =
        require(left_tcp >= 0 && right_tcp >= 0,
                "Dual-hand isolation TCP connect failed.") &&
        require(ltc_request_payload(left_tcp, 0x8026, {}, left_faults) &&
                    left_faults == std::vector<std::uint8_t>(
                                       {0x03, 0x43, 0, 0, 0, 1}),
                "Left fault was not reported.") &&
        require(ltc_request_payload(right_tcp, 0x8026, {}, right_faults) &&
                    right_faults.empty(),
                "Left fault leaked into right hand.") &&
        require(ltc_request(right_tcp, 0x802b, {4}) &&
                    ltc_request(right_tcp, 0x8005, {13}) &&
                    ltc_request(right_tcp, 0x8003, {1}),
                "Left fault blocked right hand lifecycle.");

    if (left_tcp >= 0) ::close(left_tcp);
    if (right_tcp >= 0) ::close(right_tcp);
    right.stop();
    left.stop();
    runtime.unload();
    return isolated;
}

std::size_t directory_entry_count(const char* path)
{
    std::error_code error;
    std::size_t count = 0;
    for (std::filesystem::directory_iterator iterator(path, error), end;
         !error && iterator != end; iterator.increment(error))
        ++count;
    return error ? 0 : count;
}

bool lifecycle_test()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_POC2_MODEL, &error), error))
        return false;

    const std::size_t fd_baseline = directory_entry_count("/proc/self/fd");
    const std::size_t task_baseline = directory_entry_count("/proc/self/task");
    for (int iteration = 0; iteration < 20; ++iteration)
    {
        wave_hand::WaveHandPluginConfig config;
        config.name = "hand_lifecycle_test";
        config.side = wave_hand::WaveHandSide::kLeft;
        config.device_ip = "127.0.52.10";
        config.sdk_ip = "127.0.0.1";
        config.serial = "SIM-WAVE-L-LIFECYCLE";
        config.tcp_port = 59992;
        config.command_port = 50220;
        config.feedback_destination = 0;
        config.heartbeat_destination = 1;
        config.feedback_source_port = 10000;
        config.feedback_rate_hz = 200;
        config.heartbeat_rate_hz = 1;
        wave_hand::WaveHandPlugin plugin(config);
        if (!require(plugin.start(runtime, &error), error))
            return false;

        wave_hand::WaveHandPlugin duplicate(config);
        if (!require(!duplicate.start(runtime, &error),
                     "Duplicate Wave endpoints were accepted."))
            return false;

        if (iteration == 0)
        {
            const auto connect_client = [&config](std::uint16_t port) {
                const int socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
                sockaddr_in address{};
                address.sin_family = AF_INET;
                address.sin_port = htons(port);
                ::inet_pton(AF_INET, config.device_ip.c_str(),
                            &address.sin_addr);
                return socket >= 0 &&
                               ::connect(socket,
                                         reinterpret_cast<const sockaddr*>(
                                             &address),
                                         sizeof(address)) == 0
                           ? socket
                           : -1;
            };
            const int first = connect_client(config.tcp_port);
            const int replacement = connect_client(config.tcp_port);
            if (!require(first >= 0 && replacement >= 0 &&
                             ltc_request(replacement, 0x8004),
                         "TCP replacement session did not become active."))
                return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            std::uint8_t byte = 0;
            const ssize_t first_result =
                ::recv(first, &byte, sizeof(byte), MSG_DONTWAIT);
            if (!require(first_result == 0 ||
                             (first_result < 0 && errno != EAGAIN &&
                              errno != EWOULDBLOCK),
                         "Old TCP session was not replaced."))
                return false;
            ::close(first);
            ::close(replacement);

            const int https = connect_client(8443);
            bool https_closed = false;
            const auto https_deadline = std::chrono::steady_clock::now() +
                                        std::chrono::milliseconds(200);
            while (https >= 0 &&
                   std::chrono::steady_clock::now() < https_deadline)
            {
                std::uint8_t https_byte = 0;
                const ssize_t result = ::recv(
                    https, &https_byte, sizeof(https_byte), MSG_DONTWAIT);
                if (result == 0 ||
                    (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    https_closed = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            if (!require(https >= 0 && https_closed,
                         "HTTPS fallback connection did not close promptly."))
                return false;
            ::close(https);
        }

        const auto started = std::chrono::steady_clock::now();
        plugin.stop();
        const auto stop_time = std::chrono::steady_clock::now() - started;
        if (!require(stop_time < std::chrono::milliseconds(200),
                     "Wave plugin stop exceeded 200 ms."))
            return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const bool stable =
        directory_entry_count("/proc/self/fd") <= fd_baseline + 1 &&
        directory_entry_count("/proc/self/task") == task_baseline;
    runtime.unload();
    return require(stable, "Wave plugin leaked file descriptors or threads.");
}
}  // namespace

int main()
{
    if (!codec_test() || !integration_test() || !p0_state_and_fault_test() ||
        !advanced_control_modes_test() || !dual_hand_fault_isolation_test() ||
        !lifecycle_test())
        return EXIT_FAILURE;
    std::cout << "Wave hand protocol and plugin tests passed.\n";
    return EXIT_SUCCESS;
}
