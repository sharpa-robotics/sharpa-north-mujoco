// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"
#include "sharpa_mujoco/plugins/torso/juxie_motor_emulator.h"
#include "sharpa_mujoco/plugins/torso/parallel_waist_kinematics.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace
{

bool require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << message << '\n';
    }
    return condition;
}

std::vector<std::uint8_t> from_hex(const std::string& hex)
{
    std::vector<std::uint8_t> bytes;
    for (std::size_t offset = 0; offset < hex.size(); offset += 2)
    {
        bytes.push_back(static_cast<std::uint8_t>(
            std::stoul(hex.substr(offset, 2), nullptr, 16)));
    }
    return bytes;
}

const std::vector<std::uint8_t>& golden_downlink()
{
    static const auto bytes = from_hex(
        "504f433201000205000001020304010203040506070800120000000100000000"
        "0001000001060000080140416000000000000643322f");
    return bytes;
}

bool test_codec_golden_and_crc()
{
    using namespace sharpa::mujoco::plugins::common;
    McuEnvelope envelope;
    CanFdFrame frame;
    if (!require(
            decode_mcu_downlink(
                golden_downlink().data(), golden_downlink().size(), envelope),
            "Golden MCU downlink did not decode.") ||
        !require(envelope.magic == kMcuMagicPoc2, "POC2 magic changed.") ||
        !require(envelope.sequence == 0x01020304U, "Envelope sequence changed.") ||
        !require(
            decode_can_fd_payload(
                envelope.payload.data(), envelope.payload.size(), frame),
            "Golden CAN payload did not decode.") ||
        !require(
            frame.can_interface == 0 && frame.can_id == 0x601U &&
                frame.data[0] == 0x40,
            "Golden CAN fields changed."))
    {
        return false;
    }
    auto corrupt = golden_downlink();
    corrupt.back() ^= 1U;
    if (!require(
            !decode_mcu_downlink(corrupt.data(), corrupt.size(), envelope),
            "Bad MCU CRC was accepted."))
    {
        return false;
    }

    McuEnvelope uplink;
    uplink.magic = kMcuMagicPoc2;
    uplink.source_code = make_source_code(kMcuDomainSensor, kMcuClassMotor);
    uplink.sequence = 10;
    uplink.timestamp_ns = 11;
    uplink.gptp_ns = 0x1122334455667788ULL;
    frame.transport_flags = 0;
    frame.can_id = 0x581;
    frame.data = {0x4B, 0x41, 0x60, 0x00, 0x27, 0, 0, 0};
    uplink.payload = encode_can_fd_payload(frame);
    const auto expected = from_hex(
        "504f43320100010500000000000a000000000000000b00120000000100000000"
        "000000008105000008014b41600027000000672a0d1b8877665544332211");
    return require(
        encode_mcu_uplink(uplink) == expected,
        "MCU uplink golden bytes, CRC, or LE gPTP changed.");
}

bool test_sdo_and_periodic_frames()
{
    using namespace sharpa::mujoco::plugins;
    torso::JuxieMotorEmulator motor(3);
    common::CanFdFrame request;
    request.can_interface = 0;
    request.can_id = 0x603;
    request.data = {0x40, 0x41, 0x60, 0, 0, 0, 0, 0};
    auto response = motor.process(request);
    if (!require(
            response && response->can_id == 0x583 &&
                response->data[0] == 0x4B &&
                response->data[4] == 0x40,
            "0x6041 SDO read response is invalid."))
    {
        return false;
    }
    request.data = {0x2B, 0x17, 0x10, 0, 100, 0, 0, 0};
    response = motor.process(request);
    if (!require(
            response && response->data[0] == 0x60 &&
                motor.heartbeat_period_ms() == 100,
            "0x1017 SDO write was not ACKed.") )
    {
        return false;
    }
    request.data = {0x2B, 0x40, 0x60, 0, 0x0F, 0, 0, 0};
    motor.process(request);
    if (!require(motor.enabled(), "0x6040 enable was not applied."))
    {
        return false;
    }

    const std::uint8_t slot[7] = {0xC6, 0x12, 0x34, 0, 0, 0, 0};
    if (!require(
            motor.apply_csp_slot(slot, sizeof(slot)) &&
                motor.target_position() == 0x1234 &&
                motor.control_mode() == 3 &&
                motor.enabled() && motor.brake_released(),
            "0x200 CSP slot did not decode target int16 BE."))
    {
        return false;
    }
    motor.set_feedback(0x1234, -0.42, 20, -2);
    const common::CanFdFrame feedback = motor.feedback_frame(0);
    return require(
               feedback.can_id == 0x303 && feedback.data.size() == 16,
               "0x300 feedback CAN metadata is invalid.") &&
           require(
               feedback.data[0] == 0x12 && feedback.data[1] == 0x34 &&
                   feedback.data[2] == 0xFF && feedback.data[3] == 0xD6,
               "0x300 position or velocity is not int16 BE.") &&
           require(
               feedback.data[8] == 0 && feedback.data[9] == 250 &&
                   feedback.data[12] == 0xFF &&
                   feedback.data[13] == 0xFF &&
                   feedback.data[14] == 0xFE,
               "0x300 temperature or encoder encoding is invalid.");
}

bool test_feedback_velocity_units()
{
    using namespace sharpa::mujoco::plugins;
    struct Sample
    {
        double rotor_rad_s;
        std::int16_t lower_raw;
        std::int16_t upper_raw;
        std::int32_t sdo_rpm;
    };
    // Golden values cover direction, zero, PDO saturation, and the independent
    // int32 SDO RPM range. These are physical units, not a calibration factor.
    const std::array<Sample, 9> samples{{
        {0.0, 0, 0, 0},
        {1.0, 100, 10, 10},
        {-1.0, -100, -10, -10},
        {400.0, 32767, 3820, 3820},
        {-400.0, -32768, -3820, -3820},
        {4000.0, 32767, 32767, 38197},
        {-4000.0, -32768, -32768, -38197},
        {1e9, 32767, 32767, 2147483647},
        {-1e9, -32768, -32768, (-2147483647 - 1)},
    }};
    for (std::uint8_t id = 1; id <= 8; ++id)
    {
        torso::JuxieMotorEmulator motor(id);
        common::CanFdFrame request;
        request.can_id = 0x600U + id;
        request.data = {0x40, 0x6C, 0x60, 0, 0, 0, 0, 0};
        for (const auto& sample : samples)
        {
            motor.set_feedback(0, sample.rotor_rad_s, 0, 0);
            const auto frame = motor.feedback_frame(0);
            const auto raw = static_cast<std::int16_t>(
                common::read_be16(frame.data.data() + 2));
            if (!require(raw == (id <= 5 ? sample.lower_raw : sample.upper_raw),
                         "PDO velocity units or signed saturation are wrong."))
                return false;
            const auto response = motor.process(request);
            if (!require(response && response->data[0] == 0x43 &&
                             common::read_le32(response->data.data() + 4) ==
                                 static_cast<std::uint32_t>(sample.sdo_rpm),
                         "SDO 0x606C must report int32 rotor RPM independently of PDO units."))
                return false;
        }
    }
    return true;
}

bool test_sdo_dictionary()
{
    using namespace sharpa::mujoco::plugins;
    torso::JuxieMotorEmulator motor(3);
    const auto transact = [&](std::uint8_t command, std::uint16_t index,
                              std::uint8_t sub, std::uint32_t value = 0) {
        common::CanFdFrame frame;
        frame.can_id = 0x603;
        frame.data = {command, static_cast<std::uint8_t>(index),
                      static_cast<std::uint8_t>(index >> 8), sub, 0, 0, 0, 0};
        common::write_le32(frame.data.data() + 4, value);
        return motor.process(frame).value();
    };
    const auto raw = [](const common::CanFdFrame& frame) {
        std::uint32_t value = 0;
        for (int i = 0; i < 4; ++i) value |= std::uint32_t(frame.data[4+i]) << (8*i);
        return value;
    };
    const auto aborts = [&](std::uint8_t cmd, std::uint16_t index,
                            std::uint8_t sub, std::uint32_t value, std::uint32_t code) {
        const auto response = transact(cmd, index, sub, value);
        return require(response.data[0] == 0x80 && raw(response) == code,
                       "SDO abort code mismatch.");
    };
    if (!aborts(0x40, 0x9999, 0, 0, 0x06020000) ||
        !aborts(0x23, 0x9999, 0, 0, 0x06020000) ||
        !aborts(0x40, 0x2548, 7, 0, 0x06090011) ||
        !aborts(0x23, 0x2548, 1, 1500, 0x06070010) ||
        !aborts(0x2B, 0x2548, 1, 999, 0x06090030) ||
        !aborts(0x23, 0x6064, 0, 0, 0x06010002) ||
        !aborts(0x23, 0x1010, 1, 0, 0x06090030)) return false;
    const std::array<std::uint8_t, 6> commands{0x2B,0x2B,0x23,0x23,0x2F,0x23};
    const std::array<std::uint32_t, 6> values{1500,300,42000,12000,14,55000};
    for (std::size_t i=0; i<6; ++i)
    {
        if (!require(transact(commands[i],0x2548,i+1,values[i]).data[0] == 0x60 &&
                     raw(transact(0x40,0x2548,i+1)) == values[i],
                     "Torque-loop parameter readback failed.")) return false;
    }
    for (std::size_t i=0; i<4; ++i)
    {
        const std::array<std::uint32_t,4> values{0,2400,500,85};
        if (!require(transact(0x23,0x2547,i+1,values[i]).data[0] == 0x60 &&
                     raw(transact(0x40,0x2547,i+1)) == values[i],
                     "Notch parameter readback failed.")) return false;
    }
    if (!require(raw(transact(0x40,0x2548,5)) == 6, "Notch flags are inconsistent.")) return false;
    transact(0x2B,0x6040,0,0x0F);
    if (!aborts(0x2B,0x2548,1,1600,0x08000022) ||
        !aborts(0x23,0x2547,2,1800,0x08000022) ||
        !aborts(0x23,0x1011,3,0x64616F6C,0x08000022) ||
        !require(transact(0x23,0x2548,4,2000).data[0] == 0x60 &&
                 transact(0x23,0x2548,6,60000).data[0] == 0x60,
                 "Runtime-writable torque objects were rejected.")) return false;
    transact(0x23,0x1010,1,0x65766173);
    transact(0x23,0x2548,6,70000);
    motor.power_cycle();
    if (!require(!motor.enabled() && raw(transact(0x40,0x2548,6)) == 60000 &&
                 raw(transact(0x40,0x2548,4)) == 0 &&
                 raw(transact(0x40,0x2547,2)) == 2400,
                 "Save/reload persisted Jl or lost saved parameters.")) return false;
    transact(0x23,0x1011,3,0x64616F6C);
    motor.power_cycle();
    return require(raw(transact(0x40,0x2548,1)) == 1200 &&
                   raw(transact(0x40,0x2547,2)) == 1800,
                   "Factory restore failed.");
}

bool test_mit_slot()
{
    using sharpa::mujoco::plugins::torso::JuxieMotorEmulator;
    JuxieMotorEmulator motor(1);
    // Enabled, brake released, MIT mode 6. Position/velocity/torque are at
    // their midpoint; Kp/Kd are approximately half scale.
    const std::uint8_t slot[9] = {
        0xCC, 0x80, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00};
    if (!require(
            motor.apply_mit_slot(slot, sizeof(slot)),
            "0x210 MIT slot was rejected.") ||
        !require(
            motor.enabled() && motor.brake_released() &&
                motor.control_mode() == 6,
            "0x210 MIT control byte was not decoded."))
    {
        return false;
    }
    const auto& command = motor.mit_command();
    return require(
               std::abs(command.position_rad) < 1e-4 &&
                   std::abs(command.velocity_rad_s) < 1e-3 &&
                   std::abs(command.torque_ff_nm) < 0.1,
               "MIT symmetric midpoint decoding is incorrect.") &&
           require(
               std::abs(command.kp_nm_per_rad - 1250.305) < 0.5 &&
                   std::abs(command.kd_nm_s_per_rad - 250.061) < 0.2,
               "MIT gain decoding is incorrect.");
}

bool test_parallel_waist_round_trip()
{
    using sharpa::mujoco::plugins::torso::ParallelWaistKinematics;
    ParallelWaistKinematics kinematics;
    std::array<double, 2> motor{};
    std::array<double, 2> joint{};
    std::array<double, 2> zero_inertia{};
    std::array<double, 2> posed_inertia{};
    std::array<double, 2> joint_torque{};
    constexpr double kMotorInertia = 1.647253629;
    return require(
               kinematics.joint_to_motor({0.3, 0.1}, motor),
               "Production waist IK rejected a reachable pose.") &&
           // sharpa_platform 17ff671a, default Config, InverseKinematics
           // followed by Theta1ToMotor/Theta2ToMotor.
           require(std::abs(motor[0] + 0.226303822476952) < 1e-10 &&
                   std::abs(motor[1] - 0.383133568168498) < 1e-10,
                   "Waist geometry drifted from the platform golden pose.") &&
           require(
               kinematics.motor_to_joint(motor, joint),
               "Production waist FK rejected its IK result.") &&
           require(
               std::abs(joint[0] - 0.3) < 1e-5 &&
                   std::abs(joint[1] - 0.1) < 1e-5,
               "Production waist motor/joint round trip drifted.") &&
           require(
               kinematics.reflected_inertia_diagonal(
                   {0.0, 0.0},
                   {kMotorInertia, kMotorInertia},
                   zero_inertia),
               "Waist reflected inertia rejected the zero pose.") &&
           require(
               std::abs(zero_inertia[0] - 3.294507262) < 1e-6 &&
                   std::abs(zero_inertia[1] - 2.028046842) < 1e-6,
               "Waist zero-pose reflected inertia is incorrect.") &&
           require(
               kinematics.reflected_inertia_diagonal(
                   {0.3, 0.1},
                   {kMotorInertia, kMotorInertia},
                   posed_inertia),
               "Waist reflected inertia rejected a reachable pose.") &&
           require(
               std::abs(posed_inertia[0] - zero_inertia[0]) > 1e-4 ||
                   std::abs(posed_inertia[1] - zero_inertia[1]) > 1e-4,
               "Waist reflected inertia did not change with pose.") &&
           require(
               kinematics.motor_torque_to_joint_torque(
                   {0.3, 0.1}, {2.0, -1.0}, joint_torque),
               "Waist motor torque mapping rejected a reachable pose.") &&
           require(
               std::isfinite(joint_torque[0]) &&
                   std::isfinite(joint_torque[1]),
               "Waist motor torque mapping produced non-finite values.");
}

}  // namespace

int main()
{
    if (!test_codec_golden_and_crc() ||
        !test_sdo_and_periodic_frames() ||
        !test_feedback_velocity_units() ||
        !test_sdo_dictionary() ||
        !test_mit_slot() ||
        !test_parallel_waist_round_trip())
    {
        return EXIT_FAILURE;
    }
    std::cout << "MCU envelope, Juxie, and waist tests passed.\n";
    return EXIT_SUCCESS;
}
