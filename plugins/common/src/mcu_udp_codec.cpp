// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"

#include <cstring>
#include <limits>
#include <stdexcept>

namespace sharpa::mujoco::plugins::common
{
namespace
{

bool fail(const char* message, std::string* error)
{
    if (error)
    {
        *error = message;
    }
    return false;
}

}  // namespace

std::uint16_t read_be16(const std::uint8_t* data) noexcept
{
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[0]) << 8U) | data[1]);
}

std::uint32_t read_be32(const std::uint8_t* data) noexcept
{
    return (static_cast<std::uint32_t>(data[0]) << 24U) |
           (static_cast<std::uint32_t>(data[1]) << 16U) |
           (static_cast<std::uint32_t>(data[2]) << 8U) | data[3];
}

std::uint64_t read_be64(const std::uint8_t* data) noexcept
{
    return (static_cast<std::uint64_t>(read_be32(data)) << 32U) |
           read_be32(data + 4);
}

std::uint16_t read_le16(const std::uint8_t* data) noexcept
{
    return static_cast<std::uint16_t>(
        data[0] | (static_cast<std::uint16_t>(data[1]) << 8U));
}

std::uint32_t read_le32(const std::uint8_t* data) noexcept
{
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
}

void write_be16(std::uint8_t* data, std::uint16_t value) noexcept
{
    data[0] = static_cast<std::uint8_t>(value >> 8U);
    data[1] = static_cast<std::uint8_t>(value);
}

void write_be32(std::uint8_t* data, std::uint32_t value) noexcept
{
    data[0] = static_cast<std::uint8_t>(value >> 24U);
    data[1] = static_cast<std::uint8_t>(value >> 16U);
    data[2] = static_cast<std::uint8_t>(value >> 8U);
    data[3] = static_cast<std::uint8_t>(value);
}

void write_be64(std::uint8_t* data, std::uint64_t value) noexcept
{
    write_be32(data, static_cast<std::uint32_t>(value >> 32U));
    write_be32(data + 4, static_cast<std::uint32_t>(value));
}

void write_le16(std::uint8_t* data, std::uint16_t value) noexcept
{
    data[0] = static_cast<std::uint8_t>(value);
    data[1] = static_cast<std::uint8_t>(value >> 8U);
}

void write_le32(std::uint8_t* data, std::uint32_t value) noexcept
{
    data[0] = static_cast<std::uint8_t>(value);
    data[1] = static_cast<std::uint8_t>(value >> 8U);
    data[2] = static_cast<std::uint8_t>(value >> 16U);
    data[3] = static_cast<std::uint8_t>(value >> 24U);
}

void write_le64(std::uint8_t* data, std::uint64_t value) noexcept
{
    write_le32(data, static_cast<std::uint32_t>(value));
    write_le32(data + 4, static_cast<std::uint32_t>(value >> 32U));
}

std::uint32_t mcu_crc32(const std::uint8_t* data, std::size_t size) noexcept
{
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t index = 0; index < size; ++index)
    {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc >> 1U) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

std::uint16_t modbus_crc16(const std::uint8_t* data, std::size_t size) noexcept
{
    std::uint16_t crc = 0xFFFFU;
    if (!data)
    {
        return crc;
    }
    for (std::size_t index = 0; index < size; ++index)
    {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = static_cast<std::uint16_t>(
                (crc >> 1U) ^ ((crc & 1U) ? 0xA001U : 0U));
        }
    }
    return crc;
}

bool has_valid_modbus_crc(
    const std::uint8_t* data,
    std::size_t size) noexcept
{
    return data && size >= 4 &&
           read_le16(data + size - 2) == modbus_crc16(data, size - 2);
}

bool decode_mcu_downlink(
    const std::uint8_t* data,
    std::size_t size,
    McuEnvelope& envelope,
    std::string* error)
{
    if (!data || size < kMcuEnvelopeSize + kMcuCrcSize)
    {
        return fail("MCU downlink is too short.", error);
    }
    if (data[4] != kMcuProtocolVersion)
    {
        return fail("MCU envelope version is unsupported.", error);
    }
    const std::uint16_t payload_size = read_be16(data + 22);
    const std::size_t expected =
        kMcuEnvelopeSize + static_cast<std::size_t>(payload_size) + kMcuCrcSize;
    if (expected != size || expected > kMcuMaxDatagramSize)
    {
        return fail("MCU downlink size does not match payload_len.", error);
    }
    const std::uint32_t expected_crc = read_be32(data + kMcuEnvelopeSize + payload_size);
    const std::uint32_t actual_crc =
        mcu_crc32(data + 4, (kMcuEnvelopeSize - 4) + payload_size);
    if (expected_crc != actual_crc)
    {
        return fail("MCU downlink CRC32 is invalid.", error);
    }

    McuEnvelope decoded;
    decoded.magic = read_be32(data);
    decoded.version = data[4];
    decoded.source_code = read_be32(data + 6);
    decoded.sequence = read_be32(data + 10);
    decoded.timestamp_ns = read_be64(data + 14);
    decoded.fragment_index = read_be16(data + 24);
    decoded.fragment_count = read_be16(data + 26);
    decoded.payload.assign(data + kMcuEnvelopeSize, data + kMcuEnvelopeSize + payload_size);
    envelope = std::move(decoded);
    return true;
}

std::vector<std::uint8_t> encode_mcu_uplink(const McuEnvelope& envelope)
{
    if (envelope.payload.size() > std::numeric_limits<std::uint16_t>::max())
    {
        throw std::length_error("MCU payload exceeds uint16 length.");
    }
    const std::size_t size = kMcuEnvelopeSize + envelope.payload.size() +
                             kMcuCrcSize + kMcuGptpSize;
    if (size > kMcuMaxDatagramSize)
    {
        throw std::length_error("MCU uplink exceeds maximum datagram size.");
    }
    std::vector<std::uint8_t> output(size, 0);
    write_be32(output.data(), envelope.magic);
    output[4] = kMcuProtocolVersion;
    write_be32(output.data() + 6, envelope.source_code);
    write_be32(output.data() + 10, envelope.sequence);
    write_be64(output.data() + 14, envelope.timestamp_ns);
    write_be16(
        output.data() + 22,
        static_cast<std::uint16_t>(envelope.payload.size()));
    write_be16(output.data() + 24, envelope.fragment_index);
    write_be16(output.data() + 26, envelope.fragment_count);
    if (!envelope.payload.empty())
    {
        std::memcpy(
            output.data() + kMcuEnvelopeSize,
            envelope.payload.data(),
            envelope.payload.size());
    }
    const std::uint32_t crc = mcu_crc32(
        output.data() + 4,
        (kMcuEnvelopeSize - 4) + envelope.payload.size());
    write_be32(output.data() + kMcuEnvelopeSize + envelope.payload.size(), crc);
    write_le64(
        output.data() + kMcuEnvelopeSize + envelope.payload.size() + kMcuCrcSize,
        envelope.gptp_ns);
    return output;
}

bool decode_can_fd_payload(
    const std::uint8_t* data,
    std::size_t size,
    CanFdFrame& frame,
    std::string* error)
{
    if (!data || size < 10)
    {
        return fail("CAN payload is too short.", error);
    }
    const std::uint8_t length = data[8];
    if (length > 64 || size != 10 + static_cast<std::size_t>(length))
    {
        return fail("CAN payload size or length is invalid.", error);
    }
    if (data[2] != 0 || data[3] != 0)
    {
        return fail("CAN payload reserved bytes are nonzero.", error);
    }
    CanFdFrame decoded;
    decoded.can_interface = data[0];
    decoded.transport_flags = data[1];
    decoded.can_id = read_le32(data + 4);
    decoded.can_flags = data[9];
    decoded.data.assign(data + 10, data + 10 + length);
    frame = std::move(decoded);
    return true;
}

std::vector<std::uint8_t> encode_can_fd_payload(const CanFdFrame& frame)
{
    if (frame.data.size() > 64)
    {
        throw std::length_error("CAN FD data exceeds 64 bytes.");
    }
    std::vector<std::uint8_t> output(10 + frame.data.size(), 0);
    output[0] = frame.can_interface;
    output[1] = frame.transport_flags;
    write_le32(output.data() + 4, frame.can_id);
    output[8] = static_cast<std::uint8_t>(frame.data.size());
    output[9] = frame.can_flags;
    if (!frame.data.empty())
    {
        std::memcpy(output.data() + 10, frame.data.data(), frame.data.size());
    }
    return output;
}

bool decode_modbus_rtu_payload(
    const std::uint8_t* data,
    std::size_t size,
    ModbusRtuFramePayload& frame,
    std::string* error)
{
    if (!data || size < 5)
    {
        return fail("Modbus RTU payload is too short.", error);
    }
    if (data[2] != 0 || data[3] != 0)
    {
        return fail("Modbus RTU payload reserved bytes are nonzero.", error);
    }
    const std::size_t length = data[4];
    if (length > kModbusRtuMaxFrameSize || size != 5 + length)
    {
        return fail("Modbus RTU payload size or length is invalid.", error);
    }
    ModbusRtuFramePayload decoded;
    decoded.rs485_interface = data[0];
    decoded.flags = data[1];
    decoded.data.assign(data + 5, data + size);
    frame = std::move(decoded);
    return true;
}

std::vector<std::uint8_t> encode_modbus_rtu_payload(
    const ModbusRtuFramePayload& frame)
{
    if (frame.data.size() > kModbusRtuMaxFrameSize ||
        frame.data.size() > std::numeric_limits<std::uint8_t>::max())
    {
        throw std::length_error("Modbus RTU frame exceeds 256 bytes.");
    }
    std::vector<std::uint8_t> output(5 + frame.data.size(), 0);
    output[0] = frame.rs485_interface;
    output[1] = frame.flags;
    output[4] = static_cast<std::uint8_t>(frame.data.size());
    if (!frame.data.empty())
    {
        std::memcpy(output.data() + 5, frame.data.data(), frame.data.size());
    }
    return output;
}

}  // namespace sharpa::mujoco::plugins::common
