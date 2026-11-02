// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "lidar_config.h"
#include "model_assembler.h"
#include "simulator_config.h"

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/plugins/lidar/lidar_plugin.h"
#include "sharpa_mujoco/plugins/lidar/lidar_protocol.h"
#include "sharpa_mujoco/runtime.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace
{

using sharpa::mujoco::plugins::common::UdpBindMode;
using sharpa::mujoco::plugins::common::UdpSocket;

constexpr std::size_t kFtxChannelOffset = 24;
constexpr std::size_t kFtxTailOffset = 504;
constexpr std::size_t kFtxColumnBlocks = 16;
constexpr std::size_t kFtxRowBlocks = 32;
constexpr std::size_t kJtAzimuthSamples = 360;
constexpr std::chrono::seconds kFrameTimeout{20};

struct ScanResult
{
    bool complete = false;
    std::size_t valid_points = 0;
    std::string error;
};

bool require(bool value, const std::string& message)
{
    if (!value)
    {
        std::cerr << message << '\n';
    }
    return value;
}

bool open_receiver(UdpSocket& socket, std::uint16_t& port, std::string& error)
{
    if (!socket.open_and_bind(
            {"127.0.0.1", 0}, 100, &error, UdpBindMode::Exclusive))
    {
        return false;
    }
    port = socket.local_port(&error);
    return port != 0;
}

bool reserve_source_ports(
    std::array<std::uint16_t, 3>& ports,
    std::string& error)
{
    std::array<UdpSocket, 3> reservations;
    for (std::size_t index = 0; index < reservations.size(); ++index)
    {
        if (!reservations[index].open_and_bind(
                {"127.0.0.1", 0}, 0, &error, UdpBindMode::Exclusive))
        {
            return false;
        }
        ports[index] = reservations[index].local_port(&error);
        if (ports[index] == 0)
        {
            return false;
        }
    }
    return true;
}

ScanResult receive_ftx_frame(
    UdpSocket& socket,
    std::atomic<bool>& cancel)
{
    using sharpa::mujoco::plugins::common::read_le16;
    using sharpa::mujoco::plugins::lidar::kFtx140ChannelsPerPacket;
    using sharpa::mujoco::plugins::lidar::kFtx140PacketSize;
    using sharpa::mujoco::plugins::lidar::kFtx140PacketsPerFrame;

    const auto deadline = std::chrono::steady_clock::now() + kFrameTimeout;
    std::array<std::uint8_t, 2048> buffer{};
    std::array<bool, kFtx140PacketsPerFrame> seen{};
    std::array<std::size_t, kFtx140PacketsPerFrame> packet_valid{};
    std::uint8_t frame_id = 0;
    bool has_frame = false;
    std::size_t seen_count = 0;
    std::size_t valid_points = 0;

    while (!cancel.load() && std::chrono::steady_clock::now() < deadline)
    {
        std::size_t received = 0;
        if (!socket.receive(buffer.data(), buffer.size(), received) ||
            received != kFtx140PacketSize || buffer[0] != 0xEE ||
            buffer[1] != 0xFF)
        {
            continue;
        }

        const std::uint8_t column_block = buffer[kFtxTailOffset + 7];
        const std::uint8_t row_block = buffer[kFtxTailOffset + 8];
        const std::uint8_t packet_frame_id = buffer[kFtxTailOffset + 9];
        if (column_block >= kFtxColumnBlocks || row_block >= kFtxRowBlocks)
        {
            continue;
        }
        if (!has_frame || frame_id != packet_frame_id)
        {
            seen.fill(false);
            packet_valid.fill(0);
            frame_id = packet_frame_id;
            has_frame = true;
            seen_count = 0;
            valid_points = 0;
        }

        const std::size_t packet_index =
            static_cast<std::size_t>(row_block) * kFtxColumnBlocks +
            column_block;
        std::size_t current_valid = 0;
        for (std::size_t channel = 0;
             channel < kFtx140ChannelsPerPacket;
             ++channel)
        {
            if (read_le16(
                    buffer.data() + kFtxChannelOffset + channel * 5) != 0)
            {
                ++current_valid;
            }
        }
        if (!seen[packet_index])
        {
            seen[packet_index] = true;
            ++seen_count;
        }
        else
        {
            valid_points -= packet_valid[packet_index];
        }
        packet_valid[packet_index] = current_valid;
        valid_points += current_valid;

        if (seen_count == kFtx140PacketsPerFrame)
        {
            return {true, valid_points, {}};
        }
    }
    return {false, 0, "timed out waiting for a complete FTX140 frame"};
}

ScanResult receive_jt16_frame(
    UdpSocket& socket,
    std::atomic<bool>& cancel)
{
    using namespace sharpa::mujoco::plugins;

    const auto deadline = std::chrono::steady_clock::now() + kFrameTimeout;
    std::array<std::uint8_t, 2048> buffer{};
    std::array<bool, kJtAzimuthSamples> seen{};
    std::array<std::size_t, kJtAzimuthSamples> packet_valid{};
    std::size_t seen_count = 0;
    std::size_t valid_points = 0;

    while (!cancel.load() && std::chrono::steady_clock::now() < deadline)
    {
        std::size_t received = 0;
        if (!socket.receive(buffer.data(), buffer.size(), received) ||
            received < common::kMcuEnvelopeSize + lidar::kJt16PacketSize)
        {
            continue;
        }
        const std::uint8_t* packet =
            buffer.data() + common::kMcuEnvelopeSize;
        if (packet[0] != 0xEE || packet[1] != 0xFF)
        {
            continue;
        }

        const std::uint16_t azimuth_centideg =
            common::read_le16(packet + 16);
        const std::size_t azimuth_index = azimuth_centideg / 100;
        if (azimuth_index >= kJtAzimuthSamples)
        {
            continue;
        }

        std::size_t current_valid = 0;
        for (std::size_t channel = 0;
             channel < lidar::kJt16ChannelCount;
             ++channel)
        {
            if (common::read_le16(packet + 18 + channel * 3) != 0)
            {
                ++current_valid;
            }
        }
        if (!seen[azimuth_index])
        {
            seen[azimuth_index] = true;
            ++seen_count;
        }
        else
        {
            valid_points -= packet_valid[azimuth_index];
        }
        packet_valid[azimuth_index] = current_valid;
        valid_points += current_valid;

        if (seen_count == kJtAzimuthSamples)
        {
            return {true, valid_points, {}};
        }
    }
    return {false, 0, "timed out waiting for complete JT16 angular coverage"};
}

}  // namespace

int main(int argc, char** argv)
{
    std::string config_path;
    std::string model_path;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if ((argument == "--config" || argument == "--model") &&
            index + 1 < argc)
        {
            const std::string value = argv[++index];
            if (argument == "--config")
            {
                config_path = value;
            }
            else
            {
                model_path = value;
            }
            continue;
        }
        std::cerr << "Usage: " << argv[0]
                  << " --config <path> --model <lidar_test_scene.xml>\n";
        return EXIT_FAILURE;
    }
    if (config_path.empty() || model_path.empty())
    {
        std::cerr << "Usage: " << argv[0]
                  << " --config <path> --model <lidar_test_scene.xml>\n";
        return EXIT_FAILURE;
    }

    sharpa::mujoco::simulator::SimulatorConfig config =
        sharpa::mujoco::simulator::default_config();
    std::string error;
    if (!require(
            sharpa::mujoco::simulator::load_config_file(
                config_path, config, &error),
            error))
    {
        return EXIT_FAILURE;
    }
    config.model_path = model_path;

    std::vector<sharpa::mujoco::simulator::LidarMount> mounts;
    if (!require(
            sharpa::mujoco::simulator::parse_lidar_mounts(
                config.devices, mounts, &error),
            error))
    {
        return EXIT_FAILURE;
    }
    const auto find_mount = [&](const std::string& name) {
        return std::find_if(
            mounts.begin(), mounts.end(),
            [&](const auto& mount) { return mount.instance.name == name; });
    };
    const auto front = find_mount("chest_lidar_front");
    const auto rear = find_mount("chest_lidar_rear");
    const auto chassis = find_mount("chassis_lidar_front");
    if (!require(
            front != mounts.end() && rear != mounts.end() &&
                chassis != mounts.end(),
            "The regression test requires front, rear, and chassis LiDARs."))
    {
        return EXIT_FAILURE;
    }

    std::array<UdpSocket, 3> receivers;
    std::array<std::uint16_t, 3> receiver_ports{};
    for (std::size_t index = 0; index < receivers.size(); ++index)
    {
        if (!require(
                open_receiver(receivers[index], receiver_ports[index], error),
                error))
        {
            return EXIT_FAILURE;
        }
    }
    std::array<std::uint16_t, 3> source_ports{};
    if (!require(reserve_source_ports(source_ports, error), error))
    {
        return EXIT_FAILURE;
    }

    const std::array<std::string, 3> names{
        "chest_lidar_front", "chest_lidar_rear", "chassis_lidar_front"};
    for (std::size_t index = 0; index < names.size(); ++index)
    {
        const auto mount = find_mount(names[index]);
        mount->instance.bind_ip = "127.0.0.1";
        mount->instance.bind_port = source_ports[index];
        mount->instance.peer_ip = "127.0.0.1";
        mount->instance.peer_port = receiver_ports[index];
    }

    sharpa::mujoco::Runtime runtime;
    if (!require(runtime.load(config.model_path, &error), error) ||
        !require(
            sharpa::mujoco::simulator::attach_lidar_mounts(
                runtime, mounts, &error),
            error))
    {
        return EXIT_FAILURE;
    }

    sharpa::mujoco::plugins::lidar::LidarPluginConfig plugin_config;
    for (const auto& mount : mounts)
    {
        plugin_config.instances.push_back(mount.instance);
    }
    sharpa::mujoco::plugins::lidar::LidarPlugin plugin(
        std::move(plugin_config));
    if (!require(plugin.start(runtime, &error), error) ||
        !require(runtime.start(&error), error))
    {
        plugin.stop();
        runtime.unload();
        return EXIT_FAILURE;
    }

    std::atomic<bool> cancel{false};
    std::array<ScanResult, 3> results;
    std::thread front_receiver([&]() {
        results[0] = receive_ftx_frame(receivers[0], cancel);
    });
    std::thread rear_receiver([&]() {
        results[1] = receive_ftx_frame(receivers[1], cancel);
    });
    std::thread chassis_receiver([&]() {
        results[2] = receive_jt16_frame(receivers[2], cancel);
    });
    front_receiver.join();
    rear_receiver.join();
    chassis_receiver.join();
    cancel.store(true);

    runtime.pause();
    plugin.stop();
    runtime.unload();

    constexpr std::array<std::size_t, 3> minimum_valid{1000, 1000, 100};
    bool passed = true;
    for (std::size_t index = 0; index < results.size(); ++index)
    {
        std::cout << names[index] << ": " << results[index].valid_points
                  << " valid points\n";
        if (!results[index].complete)
        {
            std::cerr << names[index] << ": " << results[index].error << '\n';
            passed = false;
        }
        else if (results[index].valid_points < minimum_valid[index])
        {
            std::cerr << names[index] << ": expected at least "
                      << minimum_valid[index] << " valid points, received "
                      << results[index].valid_points
                      << "; check raycast exclusions and model geometry\n";
            passed = false;
        }
    }
    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
