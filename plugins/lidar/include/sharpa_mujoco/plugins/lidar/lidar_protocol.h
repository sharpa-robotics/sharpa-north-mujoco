// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace sharpa::mujoco::plugins::lidar
{

constexpr std::size_t kFtx140PacketSize = 554;
constexpr std::size_t kFtx140ChannelsPerPacket = 96;
constexpr std::size_t kFtx140Rows = 192;
constexpr std::size_t kFtx140Columns = 256;
constexpr std::size_t kFtx140RowsPerPacket = 6;
constexpr std::size_t kFtx140ColumnsPerPacket = 16;
constexpr std::size_t kFtx140RowBlockCount =
    kFtx140Rows / kFtx140RowsPerPacket;
constexpr std::size_t kFtx140ColumnBlockCount =
    kFtx140Columns / kFtx140ColumnsPerPacket;
constexpr std::size_t kFtx140PacketsPerFrame =
    kFtx140RowBlockCount * kFtx140ColumnBlockCount;
constexpr std::size_t kJt16PacketSize = 80;
constexpr std::size_t kJt16ChannelCount = 16;

constexpr std::size_t ftx140_frame_index(
    std::size_t row_block,
    std::size_t column_block,
    std::size_t channel) noexcept
{
    const std::size_t row =
        row_block * kFtx140RowsPerPacket + channel % kFtx140RowsPerPacket;
    const std::size_t column =
        column_block * kFtx140ColumnsPerPacket +
        channel / kFtx140RowsPerPacket;
    return row * kFtx140Columns + column;
}

struct LidarReturn
{
    double distance_m = 0.0;
    std::uint8_t reflectivity = 0;
    bool valid = false;
};

struct Ftx140PacketMetadata
{
    std::uint8_t row_id = 0;
    std::uint8_t column_id = 0;
    std::uint8_t frame_id = 0;
    std::uint32_t sequence = 0;
    std::uint64_t timestamp_ns = 0;
};

struct Jt16PacketMetadata
{
    std::uint16_t azimuth_centideg = 0;
    std::uint16_t sequence = 0;
    std::uint64_t timestamp_ns = 0;
};

std::vector<std::uint8_t> encode_ftx140_packet(
    const std::array<LidarReturn, kFtx140ChannelsPerPacket>& returns,
    const Ftx140PacketMetadata& metadata);

std::vector<std::uint8_t> encode_jt16_packet(
    const std::array<LidarReturn, kJt16ChannelCount>& returns,
    const Jt16PacketMetadata& metadata);

std::uint32_t crc32_mpeg2(const std::uint8_t* data, std::size_t size) noexcept;

}  // namespace sharpa::mujoco::plugins::lidar
