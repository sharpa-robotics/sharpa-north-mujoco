// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include <north_sdk/NorthSDK.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

const char* phase_name(north::MasterPhase p) {
    switch (p) {
    case north::MasterPhase::kInit: return "INIT";
    case north::MasterPhase::kStandby: return "STANDBY";
    case north::MasterPhase::kRunning: return "RUNNING";
    case north::MasterPhase::kError: return "ERROR";
    case north::MasterPhase::kEstop: return "ESTOP";
    case north::MasterPhase::kOta: return "OTA";
    default: return "UNSPECIFIED";
    }
}
const char* mode_name(north::OperationMode m) {
    switch (m) {
    case north::OperationMode::kTeleoperation: return "TELEOPERATION";
    case north::OperationMode::kInference: return "INFERENCE";
    case north::OperationMode::kAutomation: return "AUTOMATION";
    default: return "UNSPECIFIED";
    }
}
int main() {
    using namespace std::chrono_literals;
    north::SdkConfig config;
    config.connect_endpoints = {"udp/127.0.0.1:7449?rel=1;multistream=1;mixed_rel=0"};
    config.time_sync.enabled = false;
    auto created = north::NorthSDK::Create(config);
    if (!created) { std::cerr << created.error().message << '\n'; return 1; }
    auto sdk = std::move(created.value());
    auto connected = sdk->connect();
    if (!connected) { std::cerr << connected.message() << '\n'; return 1; }
    auto master = sdk->master();
    auto arm = sdk->arm(north::Side::kLeft);
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
        auto status = master.latest_status(500ms);
        auto state = arm.latest_state(300ms);
        if (status && state) {
            const auto& q = state.value().joints.position;
            if (q.size() != 7) { std::cerr << "Expected 7 joints\n"; sdk->close(); return 1; }
            std::cout << "Master phase=" << phase_name(status.value().phase)
                      << " mode=" << mode_name(status.value().mode)
                      << " task='" << status.value().task_id << "'\n";
            std::cout << "Left arm positions (rad):";
            for (auto x : q) {
                if (!std::isfinite(x)) { std::cerr << "Nonfinite position\n"; sdk->close(); return 1; }
                std::cout << ' ' << x;
            }
            std::cout << "\nRead-only preflight complete. No commands were sent.\n";
            sdk->close();
            return 0;
        }
        std::this_thread::sleep_for(20ms);
    }
    std::cerr << "Timed out waiting for fresh Master and left-arm state\n";
    sdk->close();
    return 1;
}
