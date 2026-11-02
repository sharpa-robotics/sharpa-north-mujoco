#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace sharpa::mujoco::plugins::wave_hand
{

constexpr std::size_t kWaveJointCount = 22;
constexpr std::size_t kWaveJointPacketV0Size = 348;
constexpr std::size_t kWaveHeartbeatV2Size = 167;
constexpr std::uint8_t kWaveFeatureFingertipTactile = 1U << 0U;

enum class WavePacketType : std::uint8_t
{
    kJointData = 0,
    kHeartbeat = 3,
};

enum class WaveHandSide : std::uint8_t
{
    kLeft = 0,
    kRight = 1,
};

struct WaveDataHeader
{
    std::uint8_t device_type = 0;
    WavePacketType packet_type = WavePacketType::kJointData;
    std::uint8_t payload_version = 0;
    std::uint8_t joint_count = static_cast<std::uint8_t>(kWaveJointCount);
    std::array<std::uint8_t, 16> serial_number{};
    std::uint8_t control_source = 0;
    std::uint8_t control_mode = 0;
    bool interpolate = false;
    WaveHandSide side = WaveHandSide::kRight;
    bool running = false;
    std::uint16_t sequence = 0;
    std::uint64_t timestamp = 0;
    std::array<std::uint8_t, 8> reserved{};
};

struct WaveOrientation
{
    std::array<float, 3> angular_velocity{};
    std::array<float, 4> quaternion{{0.0F, 0.0F, 0.0F, 1.0F}};
};

struct WaveJointData
{
    float angle = 0.0F;
    float velocity = 0.0F;
    float torque = 0.0F;
};

struct WaveJointPacketV0
{
    WaveDataHeader header{};
    WaveOrientation orientation{};
    std::array<WaveJointData, kWaveJointCount> joints{};
    std::array<std::uint8_t, 3> tail_reserved{};
    std::uint8_t tdm_id = 0;
    std::uint32_t tdm_data = 0;
};

struct WaveDeviceStatus
{
    float temperature = 0.0F;
    std::uint8_t battery = 0;
    std::uint16_t error_code = 0;
    std::uint32_t error_joint = 0;
    std::uint64_t joint_lock_status = 0;
    std::uint64_t temperature_levels = 0;
};

struct WaveHeartbeatV2
{
    WaveDataHeader header{};
    std::array<std::uint8_t, 16> part_number{};
    std::array<std::uint8_t, 8> manufacturer{};
    std::array<std::uint8_t, 4> firmware_version{};
    std::uint16_t type_flag_minor = 0;
    std::array<std::uint8_t, 6> mac{};
    std::array<std::uint8_t, 4> device_ip{};
    std::array<std::uint8_t, 4> joint_destination_ip{};
    std::array<std::uint8_t, 4> tactile_destination_ip{};
    std::array<std::uint8_t, 4> debug_destination_ip{};
    std::array<std::uint8_t, 4> heartbeat_destination_ip{};
    std::uint16_t joint_destination_port = 0;
    std::uint16_t tactile_destination_port = 0;
    std::uint16_t debug_destination_port = 0;
    std::uint16_t heartbeat_destination_port = 0;
    std::uint16_t joint_listen_port = 0;
    std::uint16_t tcp_listen_port = 0;
    std::array<std::uint8_t, 16> paired_serial_number{};
    WaveDeviceStatus status{};
    // Optional capabilities advertised by heartbeat V2.
    std::uint8_t feature_config = 0;
    std::array<std::uint8_t, 7> reserved{};
};

// IEEE 802.3 CRC-32: reflected polynomial 0xEDB88320, init/final XOR 0xFFFFFFFF.
std::uint32_t ieee_crc32(const std::uint8_t* data, std::size_t size);

bool encode_joint_packet_v0(const WaveJointPacketV0& packet,
                            std::array<std::uint8_t, kWaveJointPacketV0Size>& output);
bool decode_joint_packet_v0(const std::uint8_t* data,
                            std::size_t size,
                            WaveJointPacketV0& packet);

bool encode_heartbeat_v2(const WaveHeartbeatV2& heartbeat,
                         std::array<std::uint8_t, kWaveHeartbeatV2Size>& output);
bool decode_heartbeat_v2(const std::uint8_t* data,
                         std::size_t size,
                         WaveHeartbeatV2& heartbeat);

}  // namespace sharpa::mujoco::plugins::wave_hand
