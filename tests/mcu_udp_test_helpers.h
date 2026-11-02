// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"

#include <array>
#include <chrono>
#include <optional>
#include <string>

namespace sharpa::mujoco::test
{

inline bool wait_for_mcu_feedback(
    plugins::common::UdpSocket& host,
    const plugins::common::UdpEndpoint& plugin,
    std::string& error,
    std::optional<std::uint8_t> expected_can_interface = std::nullopt)
{
    using namespace plugins::common;
    error.clear();
    if (!host.is_open())
    {
        error = "Cannot wait for MCU feedback on a closed UDP socket.";
        return false;
    }
    const std::string route = plugin.ip + ":" + std::to_string(plugin.port) +
        " -> host port " + std::to_string(host.local_port());
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    std::array<std::uint8_t, kMcuMaxDatagramSize + 1> buffer{};
    // Receive the plugin's first packet before sending in the reverse direction.
    // Concurrent first packets can clash in Linux NAT connection tracking.
    // Callers bind their host sockets with a short receive timeout.
    while (std::chrono::steady_clock::now() < deadline)
    {
        std::size_t size = 0;
        UdpEndpoint source;
        std::string receive_error;
        if (!host.receive(buffer.data(), buffer.size(), size, &receive_error, &source))
        {
            if (!receive_error.empty())
            {
                error = "Receiving MCU feedback (" + route + "): " + receive_error;
                return false;
            }
            continue;
        }
        if (source.ip != plugin.ip || source.port != plugin.port ||
            size < kMcuEnvelopeSize + kMcuCrcSize + kMcuGptpSize ||
            size > kMcuMaxDatagramSize)
        {
            continue;
        }
        // Uplink uses the same CRC-protected envelope, followed by a gPTP trailer.
        McuEnvelope envelope;
        CanFdFrame frame;
        if (!decode_mcu_downlink(buffer.data(), size - kMcuGptpSize, envelope) ||
            envelope.magic != kMcuMagicPoc2 ||
            !decode_can_fd_payload(envelope.payload.data(), envelope.payload.size(), frame) ||
            (expected_can_interface && frame.can_interface != *expected_can_interface))
        {
            continue;
        }
        return true;
    }
    error = "Timed out after 2 seconds waiting for valid MCU CAN feedback (" + route + ").";
    return false;
}

}  // namespace sharpa::mujoco::test
