// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include <north_sdk/NorthSDK.h>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

volatile std::sig_atomic_t keep_running = 1;
void interrupted(int) { keep_running = 0; }

int main(int argc, char** argv) {
    using namespace std::chrono_literals;
    if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        std::cout << "Usage: sdk_home [--preview]\n"
                  << "Default: execute the Home task. --preview only reads state.\n";
        return 0;
    }
    if (argc > 2 || (argc == 2 && std::string(argv[1]) != "--execute" &&
                     std::string(argv[1]) != "--preview")) {
        std::cerr << "Usage: sdk_home [--preview]\n"; return 2;
    }
    const bool execute = argc == 1 || std::string(argv[1]) != "--preview";
    std::signal(SIGINT, interrupted);
    north::SdkConfig config;
    config.connect_endpoints = {"udp/127.0.0.1:7449?rel=1;multistream=1;mixed_rel=0"};
    config.time_sync.enabled = false;
    auto created = north::NorthSDK::Create(config);
    if (!created) { std::cerr << created.error().message << '\n'; return 1; }
    auto sdk = std::move(created.value());
    auto connected = sdk->connect();
    if (!connected) { std::cerr << connected.message() << '\n'; return 1; }
    auto master = sdk->master();
    auto status = master.latest_status(500ms);
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!status && keep_running && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(20ms);
        status = master.latest_status(500ms);
    }
    if (!status || status.value().phase != north::MasterPhase::kStandby) {
        std::cerr << "Fresh STANDBY required; no task requested.\n";
        sdk->close(); return 1;
    }
    std::cout << "Target: local simulation 127.0.0.1:7449\n"
              << "Master: STANDBY\n"
              << "Task: home (Platform generates and executes the trajectory)\n"
              << "This moves both arms and torso to configured targets.\n";
    if (!execute) {
        std::cout << "Preview only: no task, mode, enable or motion command sent.\n";
        sdk->close(); return 0;
    }
    if (!keep_running) { sdk->close(); return 130; }
    auto started = master.start_task("home");
    if (!started || started.value().result != north::MasterResult::kOk) {
        std::cerr << "Task start failed: "
                  << (started ? started.value().message : started.error().message) << '\n';
        // An RPC timeout may leave an executed server-side task. Never retry the
        // request; inspect the state, and only stop the requested task. This
        // example requires a single controller throughout the run.
        auto current = master.latest_status(500ms);
        if (current && current.value().phase == north::MasterPhase::kRunning &&
            current.value().task_id == "home") {
            auto stopped = master.stop_task();
            std::cerr << "Cleanup stop_task: "
                      << (stopped ? stopped.value().message : stopped.error().message) << '\n';
        }
        std::cerr << "Start outcome was not confirmed. Inspect Master with sdk_read_state "
                     "before another run; stop home from North Pilot if still running. "
                     "Do not retry start_task blindly.\n";
        sdk->close(); return 1;
    }
    std::cout << "home accepted; observing state (Ctrl+C requests stop_task).\n";
    bool observed_running = false;
    bool complete = false;
    const auto finish = std::chrono::steady_clock::now() + 12s;
    while (keep_running && std::chrono::steady_clock::now() < finish) {
        std::this_thread::sleep_for(50ms);
        auto current = master.latest_status(500ms);
        if (!current) { std::cerr << "Master status became stale.\n"; break; }
        if (current.value().phase == north::MasterPhase::kRunning) {
            if (current.value().task_id != "home") {
                std::cerr << "Another task observed; refusing to stop it.\n";
                sdk->close(); return 1;
            }
            if (!observed_running) std::cout << "Master RUNNING task=home\n";
            observed_running = true;
        } else if (observed_running) {
            complete = current.value().phase == north::MasterPhase::kStandby;
            std::cout << "Master left RUNNING; phase=" << static_cast<int>(current.value().phase)
                      << " (2=STANDBY, 4=ERROR)\n";
            break;
        }
    }
    if (!complete) {
        auto current = master.latest_status(500ms);
        if (current && current.value().phase == north::MasterPhase::kRunning &&
            current.value().task_id == "home") {
            auto stopped = master.stop_task();
            std::cerr << "stop_task: "
                      << (stopped ? stopped.value().message : stopped.error().message) << '\n';
        }
        std::cerr << "Completion not confirmed; run sdk_read_state to check Master.\n";
    } else {
        std::cout << "Task ended in STANDBY; inspect pose and Platform logs for arrival.\n";
    }
    sdk->close();
    return complete ? 0 : 1;
}
