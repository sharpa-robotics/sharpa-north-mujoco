// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/chassis/jiechang_motor_emulator.h"
#include "sharpa_mujoco/plugins/chassis/jiechang_stretch_emulator.h"
#include "sharpa_mujoco/plugins/chassis/kinco_motor_emulator.h"
#include "sharpa_mujoco/plugins/chassis/m42p_motor_emulator.h"
#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace
{

bool require(bool condition, const std::string& message)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << message << '\n';
    }
    return condition;
}

std::vector<std::uint8_t> modbus_request(
    std::initializer_list<std::uint8_t> bytes)
{
    using sharpa::mujoco::plugins::common::modbus_crc16;
    std::vector<std::uint8_t> frame(bytes);
    const std::uint16_t crc = modbus_crc16(frame.data(), frame.size());
    frame.push_back(static_cast<std::uint8_t>(crc));
    frame.push_back(static_cast<std::uint8_t>(crc >> 8U));
    return frame;
}

bool test_modbus_payload_and_crc()
{
    using namespace sharpa::mujoco::plugins::common;
    const auto request =
        modbus_request({1, 3, 0x03, 0xE8, 0, 8});
    if (!require(modbus_crc16(
                     reinterpret_cast<const std::uint8_t*>("123456789"), 9) ==
                     0x4B37U,
                 "Modbus CRC16 known vector changed.") ||
        !require(has_valid_modbus_crc(request.data(), request.size()),
                 "Valid Modbus request CRC was rejected."))
    {
        return false;
    }
    ModbusRtuFramePayload payload;
    payload.rs485_interface = 1;
    payload.flags = kModbusRtuFlagExpectReply;
    payload.data = request;
    const auto wire = encode_modbus_rtu_payload(payload);
    ModbusRtuFramePayload decoded;
    return require(
               wire.size() == 5 + request.size() && wire[4] == request.size(),
               "Variable Modbus payload size is invalid.") &&
           require(
               decode_modbus_rtu_payload(
                   wire.data(), wire.size(), decoded) &&
                   decoded.rs485_interface == 1 &&
                   decoded.data == request,
               "Variable Modbus payload did not round trip.");
}

bool test_jiechang_protocol()
{
    using namespace sharpa::mujoco::plugins;
    chassis::JiechangMotorEmulator motor(2);
    common::CanFdFrame sdo;
    sdo.can_id = 0x602;
    sdo.data = {0x2B, 0x40, 0x60, 0, 0x0F, 0, 0, 0};
    auto response = motor.process_sdo(sdo);
    if (!require(
            response && response->can_id == 0x582 &&
                response->data[0] == 0x60 && motor.enabled(),
            "Jiechang control-word SDO failed."))
    {
        return false;
    }
    const std::uint8_t pdo1[6] = {10, 1, 0x0F, 0, 0xD4, 0xFE};
    std::uint8_t pdo2[8] = {};
    common::write_le32(pdo2, static_cast<std::uint32_t>(-524288));
    common::write_le32(pdo2 + 4, static_cast<std::uint32_t>(-1500));
    std::uint8_t pdo3[8] = {};
    common::write_le32(pdo3, 1234U);
    common::write_le32(pdo3 + 4, 5678U);
    if (!require(
            motor.apply_pdo1_slot(pdo1, sizeof(pdo1)) &&
                motor.apply_pdo2_slot(pdo2, sizeof(pdo2)) &&
                motor.apply_pdo3_slot(pdo3, sizeof(pdo3)) &&
                motor.mode() == 10 && motor.brake_released() &&
                motor.control_word() == 0x000F &&
                motor.target_torque_permille() == -300 &&
                motor.target_position_count() == -524288 &&
                motor.target_velocity_count() == -1500 &&
                motor.mit_kp() == 1234 && motor.mit_kd() == 5678,
            "Jiechang signed broadcast PDO slots failed."))
    {
        return false;
    }
    motor.set_feedback(-2, 300, -10, 20);
    const auto feedback = motor.feedback_frame(0);
    return require(
               feedback.can_id == 0x182 && feedback.data.size() == 16,
               "Jiechang TPDO metadata failed.") &&
           require(
               static_cast<std::int32_t>(
                   common::read_le32(feedback.data.data() + 4)) == -2 &&
                   static_cast<std::int32_t>(
                       common::read_le32(feedback.data.data() + 8)) == 300 &&
                   static_cast<std::int16_t>(
                       common::read_le16(feedback.data.data() + 12)) == -10 &&
                   static_cast<std::int16_t>(
                       common::read_le16(feedback.data.data() + 14)) == 20,
               "Jiechang TPDO signed fields failed.") &&
           require(
               std::abs(
                   chassis::JiechangMotorEmulator::
                       position_counts_per_revolution() -
                   524288.0) < 1e-9 &&
                   std::abs(
                       chassis::JiechangMotorEmulator::
                           velocity_counts_per_rpm() -
                       5.0) < 1e-9,
               "Jiechang platform unit constants changed.");
}

bool test_m42p_protocol()
{
    using namespace sharpa::mujoco::plugins;
    chassis::M42pMotorEmulator motor(1);
    auto set_speed = modbus_request(
        {1, 0x10, 0x07, 0xDA, 0, 2, 4, 0x41, 0xA0, 0, 0});
    auto response = motor.process(set_speed.data(), set_speed.size());
    if (!require(
            response && common::has_valid_modbus_crc(
                            response->data(), response->size()) &&
                std::abs(motor.speed_rpm() - 20.0F) < 1e-6F,
            "M42P FC16 speed command failed."))
    {
        return false;
    }
    auto forward =
        modbus_request({1, 0x10, 0x07, 0xD4, 0, 2, 4, 0, 0, 0, 0});
    response = motor.process(forward.data(), forward.size());
    motor.advance(0.1);
    if (!require(
            response && motor.moving() && motor.position_steps() > 0,
            "M42P continuous forward motion failed."))
    {
        return false;
    }
    auto home = modbus_request({1, 0x06, 0x07, 0xD0, 0, 1});
    response = motor.process(home.data(), home.size());
    motor.advance(1.0);
    if (!require(
            response && motor.position_steps() == 0 && !motor.moving(),
            "M42P Home failed."))
    {
        return false;
    }
    auto read = modbus_request({1, 3, 0x03, 0xE8, 0, 8});
    response = motor.process(read.data(), read.size());
    return require(
        response && response->size() == 21 &&
            (*response)[1] == 3 && (*response)[2] == 16 &&
            common::has_valid_modbus_crc(response->data(), response->size()),
        "M42P FC03 snapshot response failed.");
}

bool test_kinco_protocol()
{
    using namespace sharpa::mujoco::plugins;
    chassis::KincoMotorEmulator motor(5);
    common::CanFdFrame frame;
    frame.can_id = 0;
    frame.data = {0x01, 5};
    motor.process(frame);
    if (!require(
            motor.nmt_state() ==
                chassis::KincoMotorEmulator::NmtState::operational,
            "Kinco NMT operational transition failed."))
    {
        return false;
    }

    frame.can_id = 0x605;
    frame.data = {0x40, 0x10, 0x65, 3, 0, 0, 0, 0};
    auto response = motor.process(frame);
    if (!require(
            response && response->can_id == 0x585 &&
                response->data[0] == 0x4B &&
                common::read_le16(response->data.data() + 4) == 354,
            "Kinco 0x6510:03 peak-current SDO failed."))
    {
        return false;
    }

    frame.data = {0x2F, 0x60, 0x60, 0, 0xFD, 0, 0, 0};
    response = motor.process(frame);
    frame.data = {0x2B, 0x40, 0x60, 0, 0x0F, 0, 0, 0};
    response = motor.process(frame);
    if (!require(
            response && response->data[0] == 0x60 &&
                motor.mode() == -3 && motor.enabled(),
            "Kinco CiA402 mode/enable SDO failed."))
    {
        return false;
    }

    frame.data = {0x2f, 0x07, 0x60, 0, 1, 0, 0, 0};
    response = motor.process(frame);
    if (!require(response && response->data[0] == 0x60,
                 "Kinco U8 communication interrupt mode failed.")) return false;
    frame.data = {0x2b, 0x00, 0x18, 5, 5, 0, 0, 0};
    response = motor.process(frame);
    if (!require(response && response->data[0] == 0x60,
                 "Kinco TPDO event timer configuration failed.")) return false;
    frame.data = {0x2b, 0x17, 0x10, 0, 100, 0, 0, 0};
    response = motor.process(frame);
    if (!require(response && response->data[0] == 0x60 && motor.heartbeat_period_ms() == 100,
                 "Kinco producer heartbeat configuration failed.")) return false;
    frame.data = {0x40, 0x17, 0x10, 0, 0, 0, 0, 0};
    response = motor.process(frame);
    if (!require(response && response->data[0] == 0x4b && response->data[4] == 100,
                 "Kinco producer heartbeat readback failed.")) return false;
    frame.can_id = 0x205;
    frame.data.assign(6, 0);
    common::write_le16(frame.data.data(), 0x000F);
    common::write_le32(
        frame.data.data() + 2, static_cast<std::uint32_t>(123456));
    motor.process(frame);
    motor.set_actual_position(100);
    motor.set_actual_velocity(200);
    motor.set_actual_current(300);
    const auto tpdo1 = motor.tpdo1_frame(1);
    frame.can_id = 0x40000705; // Platform preserves Linux CAN_RTR_FLAG in can_id.
    frame.data.clear();
    const auto guard1 = motor.process(frame);
    const auto guard2 = motor.process(frame);
    if (!require(guard1 && guard2 && guard1->can_id == 0x705 &&
                 (guard1->data[0] ^ guard2->data[0]) == 0x80,
                 "Kinco Linux RTR node-guard response/toggle failed.")) return false;
    return require(
               motor.target_velocity() == 123456,
               "Kinco RPDO velocity mapping failed.") &&
           require(
               tpdo1.can_id == 0x185 && tpdo1.data.size() <= 8,
               "Kinco TPDO mapping failed.");
}

bool test_kinco_store_control_parameters()
{
    using namespace sharpa::mujoco::plugins;
    for (std::uint8_t node = 1; node <= 8; ++node)
    {
        chassis::KincoMotorEmulator motor(node);
        common::CanFdFrame frame;
        frame.can_interface = node % 2;
        frame.can_id = 0x600U + node;
        const auto write = [&](std::uint8_t command, std::uint16_t index,
                               std::uint8_t subindex, std::uint32_t value) {
            frame.data = {command, static_cast<std::uint8_t>(index),
                          static_cast<std::uint8_t>(index >> 8U), subindex,
                          0, 0, 0, 0};
            common::write_le32(frame.data.data() + 4, value);
            return motor.process(frame);
        };
        const auto mode = static_cast<std::uint8_t>(node <= 4 ? 1 : 0xFD);
        auto reply = write(0x2F, 0x6060, 0, mode);
        if (!require(reply && reply->data[0] == 0x60,
                     "Store test could not configure drive mode.")) return false;
        reply = write(0x23, 0x6083, 0, 123456);
        if (!require(reply && reply->data[0] == 0x60,
                     "Store test could not configure acceleration.")) return false;
        const auto status = motor.status_word();
        const auto control = motor.control_word();

        // Platform saves parameters in Pre-Operational, before enabling.
        // Repeating the trigger must not reset parameters or drive state.
        for (int repeat = 0; repeat < 2; ++repeat)
        {
            reply = write(0x2F, 0x2FF0, 1, 1);
            if (!require(reply && reply->can_id == 0x580U + node &&
                         reply->can_interface == frame.can_interface &&
                         reply->data == std::vector<std::uint8_t>{
                             0x60, 0xF0, 0x2F, 1, 0, 0, 0, 0},
                         "Kinco store-control-parameters acknowledgement failed.")) return false;
            if (!require(!motor.enabled() &&
                         motor.nmt_state() == chassis::KincoMotorEmulator::NmtState::pre_operational &&
                         static_cast<std::uint8_t>(motor.mode()) == mode &&
                         motor.status_word() == status && motor.control_word() == control &&
                         motor.profile_acceleration() == 123456,
                         "Kinco save changed live configuration or drive state.")) return false;
        }

        const auto rejects = [&](std::uint8_t command, std::uint8_t subindex,
                                 std::uint32_t value, std::uint32_t error) {
            const auto response = write(command, 0x2FF0, subindex, value);
            return require(response && response->data[0] == 0x80 &&
                           common::read_le32(response->data.data() + 4) == error,
                           "Kinco store command returned the wrong abort code.");
        };
        if (!rejects(0x2B, 1, 1, 0x06070010U) ||
            !rejects(0x23, 1, 1, 0x06070010U) ||
            !rejects(0x2F, 0, 1, 0x06090011U) ||
            !rejects(0x2F, 2, 1, 0x06090011U) ||
            !rejects(0x2F, 1, 0, 0x06090030U) ||
            !rejects(0x2F, 1, 2, 0x06090030U)) return false;

        frame.can_id = 0;
        frame.data = {0x01, node};
        motor.process(frame);
        frame.can_id = 0x600U + node;
        reply = write(0x2B, 0x6040, 0, 0x0F);
        if (!require(reply && reply->data[0] == 0x60 && motor.enabled() &&
                     motor.profile_acceleration() == 123456,
                     "Kinco could not enable after saving parameters.")) return false;
    }
    return true;
}

bool test_kinco_platform_homing()
{
    using namespace sharpa::mujoco::plugins;
    chassis::KincoMotorEmulator motor(1);
    common::CanFdFrame frame;
    frame.can_id = 0;
    frame.data = {1, 1};
    motor.process(frame);
    frame.can_id = 0x601;
    const auto write = [&](std::uint16_t index, std::uint8_t sub,
                           std::uint32_t value, std::uint8_t command) {
        frame.data = {command, static_cast<std::uint8_t>(index),
                      static_cast<std::uint8_t>(index >> 8), sub, 0, 0, 0, 0};
        common::write_le32(frame.data.data() + 4, value);
        const auto reply = motor.process(frame);
        return require(reply && reply->data[0] == 0x60,
                       "Platform homing SDO write failed: " + std::to_string(index));
    };
    const auto read = [&](std::uint16_t index, std::uint8_t sub,
                          std::uint32_t value, std::uint8_t command) {
        frame.data = {0x40, static_cast<std::uint8_t>(index),
                      static_cast<std::uint8_t>(index >> 8), sub, 0, 0, 0, 0};
        const auto reply = motor.process(frame);
        return require(reply && reply->data[0] == command &&
                       common::read_le32(reply->data.data() + 4) == value,
                       "Platform homing SDO readback failed: " + std::to_string(index));
    };
    // KincoSteerProtocol::Home(): method -17, offset 275130, offset mode 0,
    // 2 output rpm at 9:1, acceleration 50 output rpm/s, power-on mode 2.
    if (!write(0x6098, 0, 0xef, 0x2f) || !write(0x607c, 0, 275130, 0x23) ||
        !write(0x6099, 5, 0, 0x2f) || !write(0x6099, 1, 322123, 0x23) ||
        !write(0x6099, 2, 322123, 0x23) || !write(0x609a, 0, 8053, 0x23) ||
        !write(0x6099, 3, 2, 0x2f) || !write(0x6060, 0, 6, 0x2f) ||
        !read(0x6061, 0, 6, 0x4f) || !read(0x6098, 0, 0xef, 0x4f) ||
        !read(0x607c, 0, 275130, 0x43) || !read(0x6099, 1, 322123, 0x43) ||
        !read(0x6099, 2, 322123, 0x43) || !read(0x6099, 3, 2, 0x4f) ||
        !read(0x6099, 5, 0, 0x4f) || !read(0x609a, 0, 8053, 0x43) ||
        !write(0x6040, 0, 0x0f, 0x2b)) return false;
    motor.set_actual_position(20000);
    motor.update_homing();
    if (!require(!motor.homing_active() && motor.status_word() == 0x27,
                 "Homing started before control-word trigger.")) return false;
    if (!write(0x6040, 0, 0x1f, 0x2b)) return false;
    motor.update_homing();
    if (!require(motor.homing_active() && motor.status_word() == 0x27,
                 "Homing acknowledged before physical arrival.")) return false;
    motor.set_actual_position(0);
    motor.set_actual_velocity(20000);
    motor.update_homing();
    if (!require(motor.status_word() == 0x27, "Homing ignored measured velocity.")) return false;
    motor.set_actual_velocity(0);
    motor.update_homing();
    if (!read(0x6041, 0, 0x1427, 0x4b)) return false;
    const auto feedback = motor.tpdo2_frame(0);
    if (!require(common::read_le16(feedback.data.data() + 2) == 0x1427,
                 "Homing completion missing from TPDO status.")) return false;
    // Platform restores profile-position mode and the measured position.
    if (!write(0x6060, 0, 1, 0x2f) || !write(0x607a, 0, 0, 0x23) ||
        !write(0x6040, 0, 0x0f, 0x2b) || !read(0x6061, 0, 1, 0x4f)) return false;
    if (!require(!motor.homing_active() && motor.status_word() == 0x27,
                 "Homing status leaked into profile-position mode.")) return false;

    for (const auto stop : {0x0006, 0x0002, 0x011f, 0x000f})
    {
        if (!write(0x6060, 0, 6, 0x2f) || !write(0x6040, 0, 0x0f, 0x2b) ||
            !write(0x6040, 0, 0x1f, 0x2b) || !write(0x6040, 0, stop, 0x2b)) return false;
        motor.update_homing();
        if (!require(!motor.homing_active() && !(motor.status_word() & 0x1000),
                     "Disable/quick stop/halt/cleared start did not cancel homing.")) return false;
    }
    if (!write(0x6040, 0, 0x0f, 0x2b) || !write(0x6040, 0, 0x1f, 0x2b)) return false;
    motor.set_fault(1);
    motor.update_homing();
    if (!require(!motor.homing_active() && motor.status_word() == 8,
                 "Fault did not cancel homing.")) return false;
    if (!write(0x6040, 0, 0x80, 0x2b) || !write(0x6040, 0, 0x0f, 0x2b) ||
        !write(0x6098, 0, 35, 0x2f) || !write(0x6040, 0, 0x1f, 0x2b)) return false;
    if (!require(!motor.homing_active() && motor.status_word() == 0x2027,
                 "Unsupported homing method reported success.")) return false;
    if (!write(0x6098, 0, 0xef, 0x2f) || !write(0x6040, 0, 0x0f, 0x2b) ||
        !write(0x6040, 0, 0x1f, 0x2b)) return false;
    frame.can_id = 0;
    frame.data = {0x82, 1};
    motor.process(frame);
    return require(!motor.homing_active(), "NMT reset did not cancel homing.");
}

bool test_jiechang_stretch_protocol()
{
    using namespace sharpa::mujoco::plugins;
    chassis::JiechangStretchEmulator motor(9);
    common::CanFdFrame frame;
    frame.can_interface = 0;
    frame.can_id = 0x609;
    frame.data = {0x23, 0x06, 0x10, 0, 0x90, 0xd0, 0x03, 0}; // 250000 us.
    auto reply = motor.process(frame);
    if (!require(reply && reply->data[0] == 0x60 && motor.command_timeout_s() == 0.75,
                 "Stretch communication period SDO failed.")) return false;
    frame.data = {0x23, 0x00, 0x16, 1, 0x10, 0, 0x40, 0x60};
    reply = motor.process(frame);
    if (!require(reply && reply->data[0] == 0x60, "Stretch PDO mapping SDO failed.")) return false;
    frame.data = {0x40, 0x00, 0x16, 1, 0, 0, 0, 0};
    reply = motor.process(frame);
    if (!require(reply && common::read_le32(reply->data.data() + 4) == 0x60400010,
                 "Stretch PDO mapping readback failed.")) return false;
    frame.data = {0x2f, 0x60, 0x60, 0, 9, 0, 0, 0};
    motor.process(frame);
    frame.data = {0x40, 0x61, 0x60, 0, 0, 0, 0, 0};
    reply = motor.process(frame);
    if (!require(reply && reply->data[0] == 0x4f && reply->data[4] == 9,
                 "Stretch CSV mode display failed.")) return false;
    frame.can_id = 0;
    frame.data = {1, 9};
    motor.process(frame);
    frame.can_id = 0x209;
    frame.data = {0x0f, 0, 0, 0, 4, 0}; // 262144 counts = 1 mm.
    motor.process(frame);
    frame.can_id = 0x309;
    frame.data = {0, 0, 8, 0}; // 524288 counts/s = 2 mm/s.
    motor.process(frame);
    if (!require(!motor.enabled(), "Stretch RPDO applied before SYNC.")) return false;
    frame.can_id = 0x80;
    frame.data.clear();
    motor.process(frame);
    if (!require(motor.enabled() && motor.target_position() == 262144 &&
                 motor.target_velocity() == 524288, "Stretch RPDO/SYNC command failed.")) return false;
    motor.set_feedback(262144, -524288, 100, 200);
    const auto feedback = motor.feedback(0);
    if (!require(feedback[0].can_id == 0x189 && feedback[1].can_id == 0x289 &&
                 feedback[0].data.size() == 8 && feedback[1].data.size() == 8 &&
                 feedback[0].can_flags == 0 &&
                 common::read_le16(feedback[0].data.data() + 2) == 0x27 &&
                 common::read_le32(feedback[0].data.data() + 4) == 262144 &&
                 static_cast<std::int32_t>(common::read_le32(feedback[1].data.data())) == -524288,
                 "Stretch classic feedback layout failed.")) return false;
    frame.can_id = 0x209;
    frame.data = {6, 0, 0, 0, 4, 0};
    motor.process(frame);
    frame.can_id = 0x80;
    frame.data.clear();
    motor.process(frame);
    return require(!motor.enabled(), "Stretch Shutdown did not disable the motor.");
}

}  // namespace

int main()
{
    if (!test_modbus_payload_and_crc() ||
        !test_jiechang_protocol() ||
        !test_m42p_protocol() ||
        !test_kinco_protocol() ||
        !test_kinco_store_control_parameters() ||
        !test_kinco_platform_homing() ||
        !test_jiechang_stretch_protocol())
    {
        return EXIT_FAILURE;
    }
    std::cout << "Chassis protocol tests passed.\n";
    return EXIT_SUCCESS;
}
