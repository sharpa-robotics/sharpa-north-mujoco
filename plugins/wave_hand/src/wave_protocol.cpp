#include "sharpa_mujoco/plugins/wave_hand/wave_protocol.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace sharpa::mujoco::plugins::wave_hand
{
namespace
{

constexpr std::uint8_t kIdentifier0 = 0xBB;
constexpr std::uint8_t kIdentifier1 = 0xEE;
constexpr std::uint8_t kWaveProtocolVersion = 1;
constexpr std::size_t kPreHeaderSize = 4;
constexpr std::size_t kDataHeaderSize = 40;
constexpr std::size_t kOrientationSize = 28;
constexpr std::size_t kJointDataSize = 12;
constexpr std::size_t kTailSize = 12;
constexpr std::size_t kHeartbeatPayloadSize = 119;
constexpr std::uint16_t kKnownTypeFlagMask = 0xE07FU;

void write_u16_le(std::uint8_t* output, std::uint16_t value)
{
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
}

void write_u32_le(std::uint8_t* output, std::uint32_t value)
{
    for (std::size_t i = 0; i < 4; ++i)
    {
        output[i] = static_cast<std::uint8_t>(value >> (8U * i));
    }
}

void write_u64_le(std::uint8_t* output, std::uint64_t value)
{
    for (std::size_t i = 0; i < 8; ++i)
    {
        output[i] = static_cast<std::uint8_t>(value >> (8U * i));
    }
}

std::uint16_t read_u16_le(const std::uint8_t* input)
{
    return static_cast<std::uint16_t>(input[0]) |
           (static_cast<std::uint16_t>(input[1]) << 8U);
}

std::uint32_t read_u32_le(const std::uint8_t* input)
{
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < 4; ++i)
    {
        value |= static_cast<std::uint32_t>(input[i]) << (8U * i);
    }
    return value;
}

std::uint64_t read_u64_le(const std::uint8_t* input)
{
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < 8; ++i)
    {
        value |= static_cast<std::uint64_t>(input[i]) << (8U * i);
    }
    return value;
}

void write_float_le(std::uint8_t* output, float value)
{
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "32-bit float required");
    std::memcpy(&bits, &value, sizeof(bits));
    write_u32_le(output, bits);
}

float read_float_le(const std::uint8_t* input)
{
    const std::uint32_t bits = read_u32_le(input);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool finite(const WaveOrientation& orientation)
{
    for (const float value : orientation.angular_velocity)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    for (const float value : orientation.quaternion)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return true;
}

bool finite(const WaveDeviceStatus& status)
{
    return std::isfinite(status.temperature);
}

bool valid_common_header(const WaveDataHeader& header,
                         WavePacketType expected_type,
                         std::uint8_t expected_payload_version)
{
    const auto type = static_cast<std::uint8_t>(header.packet_type);
    return header.device_type == 0 &&
           type == static_cast<std::uint8_t>(expected_type) &&
           (type == 0U || type == 3U) &&
           header.payload_version == expected_payload_version &&
           header.joint_count == kWaveJointCount &&
           header.control_source <= 7U &&
           header.control_mode <= 15U &&
           static_cast<std::uint8_t>(header.side) <= 1U;
}

std::uint16_t encode_type_flag(const WaveDataHeader& header)
{
    return (static_cast<std::uint16_t>(header.control_source) & 0x07U) |
           ((static_cast<std::uint16_t>(header.control_mode) & 0x0FU) << 3U) |
           (static_cast<std::uint16_t>(header.running) << 13U) |
           (static_cast<std::uint16_t>(header.interpolate) << 14U) |
           ((static_cast<std::uint16_t>(header.side) & 0x01U) << 15U);
}

void encode_common_header(const WaveDataHeader& header, std::uint8_t* output)
{
    output[0] = kIdentifier0;
    output[1] = kIdentifier1;
    output[2] = kWaveProtocolVersion;
    output[3] = 0;
    output[4] = header.device_type;
    output[5] = static_cast<std::uint8_t>(header.packet_type);
    output[6] = header.payload_version;
    output[7] = header.joint_count;
    std::memcpy(output + 8, header.serial_number.data(), header.serial_number.size());
    write_u16_le(output + 24, encode_type_flag(header));
    write_u16_le(output + 26, header.sequence);
    write_u64_le(output + 28, header.timestamp);
    std::memcpy(output + 36, header.reserved.data(), header.reserved.size());
}

bool decode_common_header(const std::uint8_t* input,
                          WavePacketType expected_type,
                          std::uint8_t expected_payload_version,
                          std::uint8_t expected_protocol_version,
                          WaveDataHeader& header)
{
    if (input[0] != kIdentifier0 || input[1] != kIdentifier1 ||
        input[2] != expected_protocol_version || input[3] != 0)
    {
        return false;
    }

    const std::uint16_t type_flag = read_u16_le(input + 24);
    if ((type_flag & static_cast<std::uint16_t>(~kKnownTypeFlagMask)) != 0)
    {
        return false;
    }

    header.device_type = input[4];
    header.packet_type = static_cast<WavePacketType>(input[5]);
    header.payload_version = input[6];
    header.joint_count = input[7];
    std::memcpy(header.serial_number.data(), input + 8, header.serial_number.size());
    header.control_source = static_cast<std::uint8_t>(type_flag & 0x07U);
    header.control_mode = static_cast<std::uint8_t>((type_flag >> 3U) & 0x0FU);
    header.running = ((type_flag >> 13U) & 0x01U) != 0;
    header.interpolate = ((type_flag >> 14U) & 0x01U) != 0;
    header.side = static_cast<WaveHandSide>((type_flag >> 15U) & 0x01U);
    header.sequence = read_u16_le(input + 26);
    header.timestamp = read_u64_le(input + 28);
    std::memcpy(header.reserved.data(), input + 36, header.reserved.size());
    return valid_common_header(header, expected_type, expected_payload_version);
}

void encode_status(const WaveDeviceStatus& status, std::uint8_t* output)
{
    write_float_le(output, status.temperature);
    output[4] = status.battery;
    write_u16_le(output + 5, status.error_code);
    write_u32_le(output + 7, status.error_joint);
    write_u64_le(output + 11, status.joint_lock_status);
    write_u64_le(output + 19, status.temperature_levels);
}

void decode_status(const std::uint8_t* input, WaveDeviceStatus& status)
{
    status.temperature = read_float_le(input);
    status.battery = input[4];
    status.error_code = read_u16_le(input + 5);
    status.error_joint = read_u32_le(input + 7);
    status.joint_lock_status = read_u64_le(input + 11);
    status.temperature_levels = read_u64_le(input + 19);
}

}  // namespace

std::uint32_t ieee_crc32(const std::uint8_t* data, std::size_t size)
{
    if (data == nullptr && size != 0)
    {
        return 0;
    }
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
        {
            crc = (crc >> 1U) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

bool encode_joint_packet_v0(const WaveJointPacketV0& packet,
                            std::array<std::uint8_t, kWaveJointPacketV0Size>& output)
{
    if (!valid_common_header(packet.header, WavePacketType::kJointData, 0) ||
        !finite(packet.orientation))
    {
        return false;
    }
    for (const auto& joint : packet.joints)
    {
        if (!std::isfinite(joint.angle) || !std::isfinite(joint.velocity) ||
            !std::isfinite(joint.torque))
        {
            return false;
        }
    }

    output.fill(0);
    encode_common_header(packet.header, output.data());
    std::size_t offset = kPreHeaderSize + kDataHeaderSize;
    for (const float value : packet.orientation.angular_velocity)
    {
        write_float_le(output.data() + offset, value);
        offset += sizeof(float);
    }
    for (const float value : packet.orientation.quaternion)
    {
        write_float_le(output.data() + offset, value);
        offset += sizeof(float);
    }
    for (const auto& joint : packet.joints)
    {
        write_float_le(output.data() + offset, joint.angle);
        write_float_le(output.data() + offset + 4, joint.velocity);
        write_float_le(output.data() + offset + 8, joint.torque);
        offset += kJointDataSize;
    }
    std::memcpy(output.data() + offset, packet.tail_reserved.data(),
                packet.tail_reserved.size());
    output[offset + 3] = packet.tdm_id;
    write_u32_le(output.data() + offset + 4, packet.tdm_data);
    write_u32_le(output.data() + output.size() - 4,
                 ieee_crc32(output.data(), output.size() - 4));
    return offset + kTailSize == output.size();
}

bool decode_joint_packet_v0(const std::uint8_t* data,
                            std::size_t size,
                            WaveJointPacketV0& packet)
{
    if (data == nullptr || size != kWaveJointPacketV0Size ||
        read_u32_le(data + size - 4) != ieee_crc32(data, size - 4))
    {
        return false;
    }

    WaveJointPacketV0 decoded;
    // SharpaWaveSDK's downlink JointPacketSerializerV0 uses pre-header
    // protocol version 0. Device uplink packets use protocol version 1.
    if (!decode_common_header(data, WavePacketType::kJointData, 0, 0,
                              decoded.header))
    {
        return false;
    }
    std::size_t offset = kPreHeaderSize + kDataHeaderSize;
    for (float& value : decoded.orientation.angular_velocity)
    {
        value = read_float_le(data + offset);
        offset += sizeof(float);
    }
    for (float& value : decoded.orientation.quaternion)
    {
        value = read_float_le(data + offset);
        offset += sizeof(float);
    }
    for (auto& joint : decoded.joints)
    {
        joint.angle = read_float_le(data + offset);
        joint.velocity = read_float_le(data + offset + 4);
        joint.torque = read_float_le(data + offset + 8);
        offset += kJointDataSize;
    }
    std::memcpy(decoded.tail_reserved.data(), data + offset,
                decoded.tail_reserved.size());
    decoded.tdm_id = data[offset + 3];
    decoded.tdm_data = read_u32_le(data + offset + 4);

    if (!finite(decoded.orientation))
    {
        return false;
    }
    for (const auto& joint : decoded.joints)
    {
        if (!std::isfinite(joint.angle) || !std::isfinite(joint.velocity) ||
            !std::isfinite(joint.torque))
        {
            return false;
        }
    }
    packet = decoded;
    return true;
}

bool encode_heartbeat_v2(const WaveHeartbeatV2& heartbeat,
                         std::array<std::uint8_t, kWaveHeartbeatV2Size>& output)
{
    if (!valid_common_header(heartbeat.header, WavePacketType::kHeartbeat, 2) ||
        (heartbeat.feature_config & ~kWaveFeatureFingertipTactile) != 0 ||
        !finite(heartbeat.status))
    {
        return false;
    }

    output.fill(0);
    encode_common_header(heartbeat.header, output.data());
    std::size_t offset = kPreHeaderSize + kDataHeaderSize;
    auto copy = [&](const auto& field) {
        std::memcpy(output.data() + offset, field.data(), field.size());
        offset += field.size();
    };
    copy(heartbeat.part_number);
    copy(heartbeat.manufacturer);
    copy(heartbeat.firmware_version);
    write_u16_le(output.data() + offset, heartbeat.type_flag_minor);
    offset += 2;
    copy(heartbeat.mac);
    copy(heartbeat.device_ip);
    copy(heartbeat.joint_destination_ip);
    copy(heartbeat.tactile_destination_ip);
    copy(heartbeat.debug_destination_ip);
    copy(heartbeat.heartbeat_destination_ip);
    const std::uint16_t ports[] = {
        heartbeat.joint_destination_port, heartbeat.tactile_destination_port,
        heartbeat.debug_destination_port, heartbeat.heartbeat_destination_port,
        heartbeat.joint_listen_port, heartbeat.tcp_listen_port};
    for (const std::uint16_t port : ports)
    {
        write_u16_le(output.data() + offset, port);
        offset += 2;
    }
    copy(heartbeat.paired_serial_number);
    encode_status(heartbeat.status, output.data() + offset);
    offset += 27;
    output[offset++] = heartbeat.feature_config;
    copy(heartbeat.reserved);
    write_u32_le(output.data() + output.size() - 4,
                 ieee_crc32(output.data(), output.size() - 4));
    return offset == kPreHeaderSize + kDataHeaderSize + kHeartbeatPayloadSize;
}

bool decode_heartbeat_v2(const std::uint8_t* data,
                         std::size_t size,
                         WaveHeartbeatV2& heartbeat)
{
    if (data == nullptr || size != kWaveHeartbeatV2Size ||
        read_u32_le(data + size - 4) != ieee_crc32(data, size - 4))
    {
        return false;
    }

    WaveHeartbeatV2 decoded;
    if (!decode_common_header(data, WavePacketType::kHeartbeat, 2,
                              kWaveProtocolVersion, decoded.header))
    {
        return false;
    }
    std::size_t offset = kPreHeaderSize + kDataHeaderSize;
    auto copy = [&](auto& field) {
        std::memcpy(field.data(), data + offset, field.size());
        offset += field.size();
    };
    copy(decoded.part_number);
    copy(decoded.manufacturer);
    copy(decoded.firmware_version);
    decoded.type_flag_minor = read_u16_le(data + offset);
    offset += 2;
    copy(decoded.mac);
    copy(decoded.device_ip);
    copy(decoded.joint_destination_ip);
    copy(decoded.tactile_destination_ip);
    copy(decoded.debug_destination_ip);
    copy(decoded.heartbeat_destination_ip);
    std::uint16_t* ports[] = {
        &decoded.joint_destination_port, &decoded.tactile_destination_port,
        &decoded.debug_destination_port, &decoded.heartbeat_destination_port,
        &decoded.joint_listen_port, &decoded.tcp_listen_port};
    for (std::uint16_t* port : ports)
    {
        *port = read_u16_le(data + offset);
        offset += 2;
    }
    copy(decoded.paired_serial_number);
    decode_status(data + offset, decoded.status);
    offset += 27;
    decoded.feature_config = data[offset++];
    copy(decoded.reserved);

    if ((decoded.feature_config & ~kWaveFeatureFingertipTactile) != 0 ||
        !finite(decoded.status) ||
        offset != kPreHeaderSize + kDataHeaderSize + kHeartbeatPayloadSize)
    {
        return false;
    }
    heartbeat = decoded;
    return true;
}

static_assert(kPreHeaderSize + kDataHeaderSize + kOrientationSize +
                      kWaveJointCount * kJointDataSize + kTailSize ==
                  kWaveJointPacketV0Size,
              "Wave JointPacket V0 layout mismatch");
static_assert(kPreHeaderSize + kDataHeaderSize + kHeartbeatPayloadSize + 4 ==
                  kWaveHeartbeatV2Size,
              "Wave heartbeat V2 layout mismatch");
static_assert(std::numeric_limits<float>::is_iec559 && sizeof(float) == 4,
              "Wave protocol requires IEEE-754 binary32");

}  // namespace sharpa::mujoco::plugins::wave_hand
