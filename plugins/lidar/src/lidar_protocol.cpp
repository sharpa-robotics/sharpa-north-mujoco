// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/lidar/lidar_protocol.h"

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sharpa::mujoco::plugins::lidar
{
namespace
{

constexpr double kDistanceUnitM = 0.004;

std::uint16_t quantize_distance(double distance_m)
{
    if (!(distance_m > 0.0))
    {
        return 0;
    }
    const double raw = std::round(distance_m / kDistanceUnitM);
    return static_cast<std::uint16_t>(std::clamp(
        raw,
        1.0,
        static_cast<double>(std::numeric_limits<std::uint16_t>::max())));
}

void write_utc_seconds(std::uint8_t* output, std::uint64_t timestamp_ns)
{
    const std::uint64_t seconds = timestamp_ns / 1'000'000'000ULL;
    output[0] = 0;
    for (int index = 0; index < 5; ++index)
    {
        output[5 - index] = static_cast<std::uint8_t>(seconds >> (index * 8));
    }
}

}  // namespace

std::uint32_t crc32_mpeg2(const std::uint8_t* data, std::size_t size) noexcept
{
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t index = 0; index < size; ++index)
    {
        crc ^= static_cast<std::uint32_t>(data[index]) << 24U;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc << 1U) ^ ((crc & 0x80000000U) ? 0x04C11DB7U : 0U);
        }
    }
    return crc;
}

std::vector<std::uint8_t> encode_ftx140_packet(
    const std::array<LidarReturn, kFtx140ChannelsPerPacket>& returns,
    const Ftx140PacketMetadata& metadata)
{
    // FTX140 v7.3: 6B pre-header, 18B header, 96x5B channels,
    // 30B tail, then the 20B reserved security block.
    std::vector<std::uint8_t> packet(kFtx140PacketSize, 0);
    packet[0] = 0xEE;
    packet[1] = 0xFF;
    packet[2] = 7;
    packet[3] = 3;

    common::write_le16(packet.data() + 6, 256);
    common::write_le16(packet.data() + 8, 192);
    packet[10] = 16;
    packet[11] = 32;
    packet[12] = 0;  // first return
    packet[13] = 4;  // millimetres
    packet[14] = 0;  // configuration number
    common::write_le16(packet.data() + 15, kFtx140ChannelsPerPacket);

    constexpr std::size_t channel_offset = 24;
    for (std::size_t index = 0; index < returns.size(); ++index)
    {
        const LidarReturn& sample = returns[index];
        const std::size_t offset = channel_offset + index * 5;
        common::write_le16(
            packet.data() + offset,
            sample.valid ? quantize_distance(sample.distance_m) : 0);
        packet[offset + 2] = sample.valid ? sample.reflectivity : 0;
        // background and confidence are reserved in the F04 manual.
    }

    constexpr std::size_t tail_offset = channel_offset + kFtx140ChannelsPerPacket * 5;
    packet[tail_offset + 7] = metadata.column_id;
    packet[tail_offset + 8] = metadata.row_id;
    packet[tail_offset + 9] = metadata.frame_id;
    packet[tail_offset + 11] = 1;  // first return
    common::write_le16(packet.data() + tail_offset + 12, 100);
    write_utc_seconds(packet.data() + tail_offset + 14, metadata.timestamp_ns);
    common::write_le32(
        packet.data() + tail_offset + 20,
        static_cast<std::uint32_t>((metadata.timestamp_ns / 1'000ULL) % 1'000'000ULL));
    packet[tail_offset + 24] = 0x42;
    common::write_le32(packet.data() + tail_offset + 25, metadata.sequence);
    return packet;
}

std::vector<std::uint8_t> encode_jt16_packet(
    const std::array<LidarReturn, kJt16ChannelCount>& returns,
    const Jt16PacketMetadata& metadata)
{
    std::vector<std::uint8_t> packet(kJt16PacketSize, 0);
    packet[0] = 0xEE;
    packet[1] = 0xFF;
    packet[2] = 1;
    packet[3] = 8;
    packet[5] = 0;  // cloud packet

    const std::uint32_t microseconds =
        static_cast<std::uint32_t>((metadata.timestamp_ns / 1'000ULL) % 1'000'000ULL);
    common::write_le32(packet.data() + 12, microseconds);
    common::write_le16(packet.data() + 16, metadata.azimuth_centideg);
    for (std::size_t channel = 0; channel < returns.size(); ++channel)
    {
        const std::size_t offset = 18 + channel * 3;
        const LidarReturn& sample = returns[channel];
        common::write_le16(
            packet.data() + offset,
            sample.valid ? quantize_distance(sample.distance_m) : 0);
        packet[offset + 2] = sample.valid ? sample.reflectivity : 0;
    }
    common::write_le16(packet.data() + 72, metadata.sequence);
    common::write_le32(packet.data() + 76, crc32_mpeg2(packet.data(), 76));
    return packet;
}

}  // namespace sharpa::mujoco::plugins::lidar
