// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/device_plugin.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sharpa::mujoco::plugins::torso
{

enum class TorsoTransport
{
    UdpMcu,
    SocketCan,
};

struct TorsoMcuEndpointConfig
{
    std::string name;
    std::string bind_ip = "127.0.0.1";
    std::uint16_t bind_port = 0;
    std::string peer_ip = "127.0.0.1";
    std::uint16_t peer_port = 0;
    std::uint8_t can_interface = 0;
    std::vector<std::uint8_t> motor_ids;
};

struct TorsoPluginConfig
{
    TorsoTransport transport = TorsoTransport::UdpMcu;
    std::array<TorsoMcuEndpointConfig, 2> endpoints{{
        {"lower", "127.0.0.1", 45520, "127.0.0.1", 45120, 0, {1, 2, 3, 4, 5}},
        {"upper", "127.0.0.1", 45521, "127.0.0.1", 45121, 0, {6, 7, 8}},
    }};
    std::array<std::string, 2> can_interfaces{{"vcan0", "vcan1"}};
    double feedback_rate_hz = 200.0;
    double heartbeat_rate_hz = 10.0;
    // Fixed wall-clock transport delays in logical motor order, seconds.
    // Zero bypasses buffering. Coupled waist motors must use equal delays.
    // Supported range: [0, 0.1]. SDO/control state and heartbeat stay immediate.
    std::array<double, 8> command_delay_s{};
    std::array<double, 8> feedback_delay_s{};
    // Physical CAN IDs in logical torso motor order. Motors 3 and 4 are
    // cross-wired on production hardware.
    std::array<std::uint8_t, 8> motor_ids{1, 2, 4, 3, 5, 6, 7, 8};
    std::array<double, 8> motor_signs{
        -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
    std::array<double, 8> csp_kp{
        1248.0, 1248.0, 1248.0, 1482.5, 1482.5, 50.0, 50.0, 50.0};
    std::array<double, 8> csp_kd{
        5.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0};
    // Estimate the velocity of position-only command streams from their
    // wall-clock arrival cadence. This approximates the velocity feed-forward
    // present in a fixed-rate production servo. Zero disables the estimator.
    std::array<double, 8> command_velocity_filter_s{
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 8> command_velocity_scale{
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    // MIT gains and feed-forward arrive on the wire, but production firmware
    // contributes dynamics that are absent from the ideal MuJoCo torque motor.
    // Per-axis scales provide a black-box calibration layer without changing
    // the Platform command or the decoded wire values.
    std::array<double, 8> mit_kp_scale{
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::array<double, 8> mit_kd_scale{
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::array<double, 8> mit_torque_ff_scale{
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::array<double, 8> mit_torque_limit_nm{
        300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0};
    // First-order motor torque response used to approximate the finite
    // bandwidth of the production firmware/current loop. Zero keeps the
    // response instantaneous.
    std::array<double, 8> mit_torque_time_constant_s{
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Resample asynchronous MIT position updates in model time. Production
    // firmware consumes commands on a fixed control tick; these limits prevent
    // a burst of otherwise smooth wire commands from becoming one large target
    // step when the simulator temporarily falls behind. Zero disables it.
    std::array<double, 8> mit_reference_velocity_limit_rad_s{
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 8> mit_reference_acceleration_limit_rad_s2{
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Production position feedback is load-side, while the 0x300 velocity
    // field is rotor-side: IDs 1-5 use 0.01 rad/s/LSB, IDs 6-8 use RPM.
    // Keep the control law in model coordinates and apply this ratio only
    // when encoding feedback.
    std::array<double, 8> motor_reduction_ratios{
        161.0, 161.0, 161.0, 161.0, 161.0, 101.0, 101.0, 101.0};
    // Empirical velocity-only correction; never changes encoder geometry.
    std::array<double, 8> feedback_velocity_scale{
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    bool dynamic_waist_armature = true;
    std::array<double, 2> waist_motor_inertia{1.647253629, 1.647253629};
};

class TorsoPlugin final : public IDevicePlugin
{
public:
    explicit TorsoPlugin(TorsoPluginConfig config = {});
    ~TorsoPlugin() override;

    TorsoPlugin(const TorsoPlugin&) = delete;
    TorsoPlugin& operator=(const TorsoPlugin&) = delete;

    std::string name() const override;
    bool start(Runtime& runtime, std::string* error = nullptr) override;
    void stop() noexcept override;
    bool is_running() const noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sharpa::mujoco::plugins::torso
