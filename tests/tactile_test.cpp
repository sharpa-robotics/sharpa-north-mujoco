// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/tactile/tactile_plugin.h"
#include "sharpa_mujoco/plugins/tactile/tactile_protocol.h"
#include "sharpa_mujoco/runtime.h"
#include "model_assembler.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <msgpack11.hpp>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace
{

bool require(bool condition, const std::string& message)
{
    if (!condition) std::cerr << message << '\n';
    return condition;
}

bool config_request(
    const char* ip,
    std::uint16_t port,
    const msgpack11::MsgPack& request,
    msgpack11::MsgPack& response,
    bool split_request = false)
{
    const int socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket < 0) return false;
    timeval timeout{};
    timeout.tv_sec = 1;
    ::setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    ::inet_pton(AF_INET, ip, &address.sin_addr);
    if (::connect(socket, reinterpret_cast<const sockaddr*>(&address),
                  sizeof(address)) != 0)
    {
        ::close(socket);
        return false;
    }

    const std::string body = request.dump();
    std::vector<std::uint8_t> packet(12 + body.size() + 4, 0);
    packet[0] = 0xBB;
    packet[1] = 0xEE;
    const std::uint32_t size = static_cast<std::uint32_t>(body.size());
    std::memcpy(packet.data() + 8, &size, sizeof(size));
    std::memcpy(packet.data() + 12, body.data(), body.size());
    std::fill(packet.end() - 4, packet.end(), 0xFF);
    const std::size_t first_size = split_request ? 12 : packet.size();
    bool ok = ::send(socket, packet.data(), first_size, MSG_NOSIGNAL) ==
              static_cast<ssize_t>(first_size);
    if (ok && split_request)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ok = ::send(socket, packet.data() + first_size,
                    packet.size() - first_size, MSG_NOSIGNAL) ==
             static_cast<ssize_t>(packet.size() - first_size);
    }

    std::array<std::uint8_t, 4096> bytes{};
    const ssize_t received = ok
        ? ::recv(socket, bytes.data(), bytes.size(), 0)
        : -1;
    ::close(socket);
    if (received <= 16) return false;
    std::uint32_t response_size = 0;
    std::memcpy(&response_size, bytes.data() + 8, sizeof(response_size));
    if (12 + response_size + 4 > static_cast<std::size_t>(received))
        return false;
    std::string parse_error;
    response = msgpack11::MsgPack::parse(
        reinterpret_cast<const char*>(bytes.data() + 12),
        response_size, parse_error);
    return parse_error.empty() && response["ret_code"].is_int32() &&
           response["ret_code"].int32_value() == 0;
}

int open_udp_receiver(std::uint16_t port)
{
    const int socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket < 0) return -1;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    timeval timeout{};
    timeout.tv_sec = 1;
    ::setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    if (::bind(socket, reinterpret_cast<const sockaddr*>(&address),
               sizeof(address)) != 0)
    {
        ::close(socket);
        return -1;
    }
    return socket;
}

bool receive_channel_mask(int socket, int first_channel)
{
    using namespace sharpa::mujoco::plugins::tactile;
    std::uint32_t mask = 0;
    for (int attempt = 0; attempt < 30 && mask != 0x1FU; ++attempt)
    {
        std::array<std::uint8_t, kF6PacketSize> bytes{};
        if (::recv(socket, bytes.data(), bytes.size(), 0) !=
            static_cast<ssize_t>(bytes.size()))
            continue;
        F6Frame frame;
        if (!decode_f6_frame(bytes.data(), bytes.size(), frame) ||
            frame.channel < first_channel ||
            frame.channel >= first_channel + 5)
            return false;
        mask |= 1U << static_cast<unsigned>(frame.channel - first_channel);
    }
    return mask == 0x1FU;
}

void drain_udp(int socket)
{
    std::array<std::uint8_t, 256> bytes{};
    while (::recv(socket, bytes.data(), bytes.size(), MSG_DONTWAIT) > 0) {}
}

bool protocol_test()
{
    using namespace sharpa::mujoco::plugins::tactile;
    F6Frame frame;
    frame.channel = 7;
    frame.frame_id = 0x1234;
    frame.timestamp = 1234.5;
    frame.values = {{1.0F, -2.0F, 3.0F, -0.1F, 0.2F, -0.3F}};
    std::array<std::uint8_t, kF6PacketSize> bytes{};
    F6Frame decoded;
    if (!require(encode_f6_frame(frame, bytes), "F6 encode failed.") ||
        !require(decode_f6_frame(bytes.data(), bytes.size(), decoded),
                 "F6 decode failed."))
        return false;
    bool ok = require(decoded.channel == frame.channel &&
                          decoded.frame_id == frame.frame_id &&
                          decoded.timestamp == frame.timestamp &&
                          decoded.values == frame.values,
                      "F6 round trip mismatch.");
    bytes[40] ^= 1U;
    ok = require(!decode_f6_frame(bytes.data(), bytes.size(), decoded),
                 "F6 block corruption was accepted.") && ok;
    if (!encode_f6_frame(frame, bytes)) return false;
    std::fill(bytes.end() - 4, bytes.end(), 0);
    ok = require(!decode_f6_frame(bytes.data(), bytes.size(), decoded),
                 "F6 invalid stream CRC was accepted.") && ok;
    return ok;
}

bool model_test()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins::tactile;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_TACTILE_MODEL, &error),
                 error))
        return false;
    const std::array<std::pair<const char*, const char*>, 10> fingers{{
        {"right_pinky_DP", "right_pinky_tactile_"},
        {"right_ring_DP", "right_ring_tactile_"},
        {"right_middle_DP", "right_middle_tactile_"},
        {"right_index_DP", "right_index_tactile_"},
        {"right_thumb_DP", "right_thumb_tactile_"},
        {"left_pinky_DP", "left_pinky_tactile_"},
        {"left_ring_DP", "left_ring_tactile_"},
        {"left_middle_DP", "left_middle_tactile_"},
        {"left_index_DP", "left_index_tactile_"},
        {"left_thumb_DP", "left_thumb_tactile_"},
    }};
    std::vector<simulator::TactileMount> mounts;
    for (const auto& [parent, prefix] : fingers)
    {
        simulator::TactileMount mount;
        mount.parent_body = parent;
        mount.instance_prefix = prefix;
        mount.sensor_model = SHARPA_MUJOCO_TACTILE_FRAME;
        mount.quaternion = {{0.5, 0.5, -0.5, 0.5}};
        mounts.push_back(std::move(mount));
    }
    if (!require(simulator::attach_tactile_mounts(runtime, mounts, &error),
                 error))
        return false;
    bool sensor_mounts_valid = true;
    runtime.read_native([&](const mjModel* model, const mjData*) {
        constexpr std::array<double, 4> expected_quaternion{{
            0.5, 0.5, -0.5, 0.5}};
        for (const auto& [parent, prefix_value] : fingers)
        {
            const std::string prefix = prefix_value;
            const std::string surface = std::string(parent).substr(0, std::string(parent).size() - 3) +
                "_elastomer_collision_01";
            const int geom_id = mj_name2id(model, mjOBJ_GEOM, surface.c_str());
            const int site_id = mj_name2id(
                model, mjOBJ_SITE, (prefix + "frame").c_str());
            if (geom_id < 0 || site_id < 0)
            {
                sensor_mounts_valid = false;
                continue;
            }
            const int site_body = model->site_bodyid[site_id];
            const int parent_body = mj_name2id(model, mjOBJ_BODY, parent);
            if (model->body_parentid[site_body] != parent_body ||
                model->geom_bodyid[geom_id] != parent_body)
                sensor_mounts_valid = false;
            double quaternion_dot = 0.0;
            for (int element = 0; element < 4; ++element)
            {
                quaternion_dot += model->body_quat[4 * site_body + element] *
                                  expected_quaternion[element];
            }
            if (std::abs(std::abs(quaternion_dot) - 1.0) > 1e-5)
                sensor_mounts_valid = false;
        }
    });
    if (!require(sensor_mounts_valid,
                 "Tactile sensor mount definitions are invalid."))
        return false;
    const int right_udp = open_udp_receiver(50001);
    const int left_udp = open_udp_receiver(50011);
    if (!require(right_udp >= 0 && left_udp >= 0,
                 "Could not bind tactile test UDP receivers."))
        return false;

    TactilePluginConfig config;
    config.rate_hz = 10;
    TactilePlugin plugin(config);
    const bool started = plugin.start(runtime, &error);
    bool tcp_ok = false;
    bool udp_ok = false;
    if (started)
    {
        msgpack11::MsgPack response;
        const auto get_elastomer = msgpack11::MsgPack::object{
            {"type", "get"}, {"channel", -1},
            {"key", "elastomer_version"}};
        tcp_ok = config_request(
            "127.0.50.20", 59998, get_elastomer, response, true) &&
            response["content"].array_items().size() == 5;
        for (std::size_t index = 0; tcp_ok && index < 5; ++index)
        {
            const std::string expected =
                index == 4 ? "M8900001800" : "M8900001400";
            tcp_ok = response["content"][index]["value"].string_value() ==
                     expected;
        }
        tcp_ok = config_request(
                     "127.0.50.10", 59998, get_elastomer, response) && tcp_ok &&
                 response["content"].array_items().size() == 5 &&
                 response["content"][0]["channel"].int32_value() == 5 &&
                 response["content"][4]["channel"].int32_value() == 9;

        const auto set_fps = msgpack11::MsgPack::object{
            {"type", "set"}, {"channel", -1}, {"key", "fps"},
            {"value", 100}};
        tcp_ok = config_request(
                     "127.0.50.20", 59998, set_fps, response) && tcp_ok;

        udp_ok = receive_channel_mask(right_udp, 0) &&
                 receive_channel_mask(left_udp, 5);

        std::array<std::uint8_t, kF6PacketSize> bytes{};
        F6Frame frame;
        int channel_zero_frames = 0;
        const auto rate_start = std::chrono::steady_clock::now();
        while (channel_zero_frames < 6)
        {
            const ssize_t rate_received =
                ::recv(right_udp, bytes.data(), bytes.size(), 0);
            if (rate_received != static_cast<ssize_t>(bytes.size()) ||
                !decode_f6_frame(bytes.data(), bytes.size(), frame))
                break;
            if (frame.channel == 0) ++channel_zero_frames;
        }
        const double rate_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - rate_start).count();
        udp_ok = udp_ok && channel_zero_frames == 6 && rate_elapsed < 0.3;
    }
    plugin.stop();
    drain_udp(right_udp);
    drain_udp(left_udp);
    bool restart_ok = plugin.start(runtime, &error) &&
                      receive_channel_mask(right_udp, 0) &&
                      receive_channel_mask(left_udp, 5);
    if (restart_ok)
    {
        msgpack11::MsgPack response;
        const auto get_fps = msgpack11::MsgPack::object{
            {"type", "get"}, {"channel", -1}, {"key", "fps"}};
        restart_ok = config_request(
            "127.0.50.20", 59998, get_fps, response);
    }
    plugin.stop();
    ::close(right_udp);
    ::close(left_udp);
    runtime.unload();
    return require(started, error) &&
           require(tcp_ok, "Tactile SDK-compatible TCP exchange failed.") &&
           require(udp_ok, "Tactile SDK-compatible UDP exchange failed.") &&
           require(restart_ok, "Tactile stop/start recovery failed.");
}

bool contact_direction_test()
{
    using namespace sharpa::mujoco;
    using namespace sharpa::mujoco::plugins::tactile;
    Runtime runtime;
    std::string error;
    if (!require(runtime.load(SHARPA_MUJOCO_TACTILE_CONTACT_MODEL, &error),
                 error))
        return false;

    const int udp = open_udp_receiver(50191);
    if (!require(udp >= 0, "Could not bind tactile contact receiver."))
        return false;

    TactilePluginConfig config;
    config.right_device_ip = "127.0.51.20";
    config.left_device_ip = "127.0.51.10";
    config.config_port = 59997;
    config.right_destination_port = 50191;
    config.left_destination_port = 50192;
    TactilePlugin plugin(config);
    const bool torque_applied = runtime.write_native(
        [](const mjModel* model, mjData* data) {
            const int body_id =
                mj_name2id(model, mjOBJ_BODY, "contact_finger");
            if (body_id >= 0)
            {
                data->xfrc_applied[6 * body_id] = 2.0;
                data->xfrc_applied[6 * body_id + 5] = 0.2;
            }
        },
        true, &error);
    bool ok = require(torque_applied, error) &&
               require(plugin.start(runtime, &error), error) &&
               require(runtime.step(1000, &error), error);
    bool channel_received = false;
    float minimum_normal_force = 0.0F;
    float maximum_normal_force = 0.0F;
    bool normal_force_observed = false;
    bool positive_normal_force = false;
    bool tangential_force = false;
    bool eccentric_contact_torque = false;
    bool torsional_contact_torque = false;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (ok && std::chrono::steady_clock::now() < deadline &&
           (!positive_normal_force || !tangential_force ||
            !eccentric_contact_torque || !torsional_contact_torque))
    {
        std::array<std::uint8_t, kF6PacketSize> bytes{};
        const ssize_t received = ::recv(udp, bytes.data(), bytes.size(), 0);
        F6Frame frame;
        if (received == static_cast<ssize_t>(bytes.size()) &&
            decode_f6_frame(bytes.data(), bytes.size(), frame) &&
            frame.channel == 0)
        {
            if (!channel_received)
            {
                minimum_normal_force = frame.values[2];
                maximum_normal_force = frame.values[2];
            }
            else
            {
                minimum_normal_force =
                    std::min(minimum_normal_force, frame.values[2]);
                maximum_normal_force =
                    std::max(maximum_normal_force, frame.values[2]);
            }
            channel_received = true;
            normal_force_observed |= std::abs(frame.values[2]) > 1.0F;
            positive_normal_force |= frame.values[2] > 1.0F;
            tangential_force |= std::abs(frame.values[0]) > 0.1F;
            eccentric_contact_torque |= std::abs(frame.values[4]) > 1e-3F;
            torsional_contact_torque |= std::abs(frame.values[5]) > 1e-4F;
        }
    }
    plugin.stop();
    ::close(udp);
    runtime.unload();
    const std::string normal_range =
        " Observed Fz range: [" + std::to_string(minimum_normal_force) +
        ", " + std::to_string(maximum_normal_force) + "].";
    return ok &&
           require(channel_received,
                   "No valid tactile frame was received for channel 0 within 3 seconds.") &&
           require(normal_force_observed,
                   ("Channel 0 did not report sufficient contact force." +
                    normal_range).c_str()) &&
           require(positive_normal_force,
                   ("Fingertip contact normal force has the wrong sign." +
                    normal_range).c_str()) &&
           require(tangential_force,
                   "Tangential contact force was not included in F6.") &&
           require(eccentric_contact_torque,
                   "Eccentric contact moment was not included in F6.") &&
           require(torsional_contact_torque,
                   "Contact-frame torque was not included in F6.");
}

}  // namespace

int main()
{
    if (!protocol_test() || !model_test() || !contact_direction_test())
        return EXIT_FAILURE;
    std::cout << "Tactile protocol and model tests passed.\n";
    return EXIT_SUCCESS;
}
