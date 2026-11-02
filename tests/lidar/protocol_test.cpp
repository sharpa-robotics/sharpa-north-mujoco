// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"
#include "sharpa_mujoco/plugins/lidar/lidar_protocol.h"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace
{

bool require(bool value, const char* message)
{
    if (!value)
    {
        std::cerr << message << '\n';
    }
    return value;
}

}  // namespace

int main()
{
    using namespace sharpa::mujoco::plugins;
    using namespace sharpa::mujoco::plugins::lidar;

    constexpr std::array<std::uint8_t, 9> known_input{
        '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    if (!require(
            crc32_mpeg2(known_input.data(), known_input.size()) == 0x0376E6E7U,
            "CRC-32/MPEG-2 known vector failed."))
    {
        return EXIT_FAILURE;
    }

    std::array<LidarReturn, kFtx140ChannelsPerPacket> ftx_returns{};
    ftx_returns[0] = {1.0, 42, true};
    const std::vector<std::uint8_t> ftx = encode_ftx140_packet(
        ftx_returns, {0, 0, 3, 7, 1'700'000'000'123'456'789ULL});
    if (!require(ftx.size() == kFtx140PacketSize, "FTX packet size is not 554.") ||
        !require(ftx[0] == 0xEE && ftx[1] == 0xFF && ftx[2] == 7 && ftx[3] == 3,
                 "FTX pre-header is invalid.") ||
        !require(common::read_le16(ftx.data() + 24) == 250, "FTX range quantization failed.") ||
        !require(ftx[26] == 42, "FTX reflectivity encoding failed."))
    {
        return EXIT_FAILURE;
    }

    if (!require(
            kFtx140PacketsPerFrame == 512,
            "FTX frame packet count is invalid.") ||
        !require(
            ftx140_frame_index(7, 11, 0) == 42 * 256 + 176,
            "FTX first channel mapping is invalid.") ||
        !require(
            ftx140_frame_index(7, 11, 5) == 47 * 256 + 176,
            "FTX row-interleaved channel mapping is invalid.") ||
        !require(
            ftx140_frame_index(7, 11, 6) == 42 * 256 + 177,
            "FTX column-interleaved channel mapping is invalid.") ||
        !require(
            ftx140_frame_index(7, 11, 95) == 47 * 256 + 191,
            "FTX last channel mapping is invalid."))
    {
        return EXIT_FAILURE;
    }

    std::array<LidarReturn, kJt16ChannelCount> jt_returns{};
    jt_returns[0] = {2.0, 23, true};
    const std::vector<std::uint8_t> jt = encode_jt16_packet(
        jt_returns, {160, 5, 1'700'000'000'123'456'789ULL});
    if (!require(jt.size() == kJt16PacketSize, "JT16 packet size is not 80.") ||
        !require(common::read_le16(jt.data() + 16) == 160, "JT16 azimuth encoding failed.") ||
        !require(common::read_le16(jt.data() + 18) == 500, "JT16 range quantization failed.") ||
        !require(common::read_le32(jt.data() + 76) == crc32_mpeg2(jt.data(), 76),
                 "JT16 CRC encoding failed."))
    {
        return EXIT_FAILURE;
    }

    common::McuEnvelope envelope;
    envelope.magic = common::kMcuMagicPoc2;
    envelope.source_code = common::make_source_code(
        common::kMcuDomainSensor, common::kMcuClassLidar);
    envelope.sequence = 9;
    envelope.timestamp_ns = 1'700'000'000'123'456'789ULL;
    envelope.gptp_ns = envelope.timestamp_ns;
    envelope.payload = jt;
    const std::vector<std::uint8_t> uplink = common::encode_mcu_uplink(envelope);
    return require(
               uplink.size() == common::kMcuEnvelopeSize + kJt16PacketSize +
                                    common::kMcuCrcSize + common::kMcuGptpSize,
               "JT16 POC2 uplink size is invalid.")
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
