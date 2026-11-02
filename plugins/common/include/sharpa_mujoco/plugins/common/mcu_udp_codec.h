// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sharpa::mujoco::plugins::common
{

constexpr std::size_t kMcuEnvelopeSize = 32;
constexpr std::size_t kMcuCrcSize = 4;
constexpr std::size_t kMcuGptpSize = 8;
constexpr std::size_t kMcuMaxDatagramSize = 1400;
constexpr std::uint8_t kMcuProtocolVersion = 1;
constexpr std::uint32_t kMcuMagicPoc2 = 0x504F4332U;
constexpr std::uint8_t kMcuDomainSensor = 0x01;
constexpr std::uint8_t kMcuDomainCommand = 0x02;
constexpr std::uint8_t kMcuClassMotor = 0x05;
constexpr std::uint8_t kMcuClassLidar = 0x10;
constexpr std::uint8_t kCanFlagExpectReply = 0x01;
constexpr std::uint8_t kCanFdFlagBrs = 0x01;
constexpr std::uint8_t kModbusRtuFlagExpectReply = 0x01;
constexpr std::size_t kModbusRtuMaxFrameSize = 256;

constexpr std::uint32_t make_source_code(
    std::uint8_t domain,
    std::uint8_t class_id,
    std::uint16_t operation = 0) noexcept
{
    return (static_cast<std::uint32_t>(domain) << 24U) |
           (static_cast<std::uint32_t>(class_id) << 16U) |
           operation;
}

struct McuEnvelope
{
    std::uint32_t magic = 0;
    std::uint8_t version = 0;
    std::uint32_t source_code = 0;
    std::uint32_t sequence = 0;
    std::uint64_t timestamp_ns = 0;
    std::uint16_t fragment_index = 0;
    std::uint16_t fragment_count = 1;
    std::vector<std::uint8_t> payload;
    std::uint64_t gptp_ns = 0;
};

struct CanFdFrame
{
    std::uint8_t can_interface = 0;
    std::uint8_t transport_flags = 0;
    std::uint32_t can_id = 0;
    std::uint8_t can_flags = 0;
    std::vector<std::uint8_t> data;
};

struct ModbusRtuFramePayload
{
    std::uint8_t rs485_interface = 0;
    std::uint8_t flags = 0;
    std::vector<std::uint8_t> data;
};

std::uint32_t mcu_crc32(const std::uint8_t* data, std::size_t size) noexcept;
std::uint16_t modbus_crc16(const std::uint8_t* data, std::size_t size) noexcept;
bool has_valid_modbus_crc(
    const std::uint8_t* data,
    std::size_t size) noexcept;

bool decode_mcu_downlink(
    const std::uint8_t* data,
    std::size_t size,
    McuEnvelope& envelope,
    std::string* error = nullptr);

std::vector<std::uint8_t> encode_mcu_uplink(
    const McuEnvelope& envelope);

bool decode_can_fd_payload(
    const std::uint8_t* data,
    std::size_t size,
    CanFdFrame& frame,
    std::string* error = nullptr);

std::vector<std::uint8_t> encode_can_fd_payload(const CanFdFrame& frame);

bool decode_modbus_rtu_payload(
    const std::uint8_t* data,
    std::size_t size,
    ModbusRtuFramePayload& frame,
    std::string* error = nullptr);

std::vector<std::uint8_t> encode_modbus_rtu_payload(
    const ModbusRtuFramePayload& frame);

std::uint16_t read_be16(const std::uint8_t* data) noexcept;
std::uint32_t read_be32(const std::uint8_t* data) noexcept;
std::uint64_t read_be64(const std::uint8_t* data) noexcept;
std::uint16_t read_le16(const std::uint8_t* data) noexcept;
std::uint32_t read_le32(const std::uint8_t* data) noexcept;
void write_be16(std::uint8_t* data, std::uint16_t value) noexcept;
void write_be32(std::uint8_t* data, std::uint32_t value) noexcept;
void write_be64(std::uint8_t* data, std::uint64_t value) noexcept;
void write_le16(std::uint8_t* data, std::uint16_t value) noexcept;
void write_le32(std::uint8_t* data, std::uint32_t value) noexcept;
void write_le64(std::uint8_t* data, std::uint64_t value) noexcept;

}  // namespace sharpa::mujoco::plugins::common
