// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/runtime.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>

// Exercise mj_step's real automatic reset, rather than Runtime::reset (which
// already resets the publication deadline). Repeat it before the old time can
// catch up, at both the normal device rate and a throttled snapshot rate.
int main()
{
    using namespace sharpa::mujoco;
    for (double rate : {500.0, 10.0, 0.0})
    {
        RuntimeConfig config;
        config.realtime = false;
        config.snapshot_rate_hz = rate;
        Runtime runtime(config);
        std::string error;
        if (!runtime.load(SHARPA_MUJOCO_TEST_MODEL, &error))
        {
            std::cerr << error << '\n';
            return EXIT_FAILURE;
        }
        std::atomic<bool> done{false};
        int reset_snapshots = 0;
        bool valid_snapshots = true;
        std::uint64_t previous_version = 0;
        const std::uint64_t generation_before_resets =
            runtime.reset_generation();
        runtime.set_snapshot_callback([&](const PoseSnapshot& snapshot) {
            valid_snapshots &= snapshot.version > previous_version;
            previous_version = snapshot.version;
            for (double q : snapshot.qpos) valid_snapshots &= std::isfinite(q);
            if (snapshot.step_index == 201 || snapshot.step_index == 261)
            {
                ++reset_snapshots;
                valid_snapshots &= std::abs(snapshot.simulation_time - 0.002) < 1e-12;
            }
        });
        runtime.set_step_callbacks(
            [](const mjModel*, mjData* data, const NativeStepContext& context) {
                if (context.step_index > 320) return StepDecision::Wait;
                data->ctrl[1] = 0.1;
                if (context.step_index == 201 || context.step_index == 261)
                    data->qvel[0] = std::numeric_limits<double>::quiet_NaN();
                return StepDecision::Proceed;
            },
            [&](const mjModel*, mjData*, const NativeStepContext& context) {
                if (context.step_index == 320) done.store(true);
            });
        if (!runtime.start(&error))
        {
            std::cerr << error << '\n';
            return EXIT_FAILURE;
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!done.load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        runtime.pause();
        const auto snapshot = runtime.latest_snapshot();
        double live_time = 0.0;
        int bad_velocity = 0;
        runtime.read_native([&](const mjModel*, const mjData* data) {
            live_time = data->time;
            bad_velocity = data->warning[mjWARN_BADQVEL].number;
        });
        const double maximum_lag = rate > 0.0 ? 1.0 / rate : 1e-12;
        if (!done.load() || !valid_snapshots || reset_snapshots != 2 ||
            bad_velocity == 0 || snapshot.step_index < 261 ||
            runtime.reset_generation() != generation_before_resets + 2 ||
            snapshot.simulation_time > live_time + 1e-12 ||
            live_time - snapshot.simulation_time > maximum_lag + 1e-12)
        {
            std::cerr << "Snapshot reset regression at rate " << rate
                      << ": resets=" << reset_snapshots
                      << " step=" << snapshot.step_index
                      << " snapshot_time=" << snapshot.simulation_time
                      << " live_time=" << live_time << '\n';
            return EXIT_FAILURE;
        }
    }

    // A numerical warning is not a reset when MuJoCo autoreset is disabled.
    // The Runtime lifecycle generation must therefore remain unchanged.
    Runtime no_autoreset;
    std::string error;
    if (!no_autoreset.load(SHARPA_MUJOCO_TEST_MODEL, &error))
    {
        std::cerr << error << '\n';
        return EXIT_FAILURE;
    }
    const std::uint64_t generation_before_warning =
        no_autoreset.reset_generation();
    const std::uint64_t updater = no_autoreset.add_model_updater(
        [](mjModel* model, mjData* data, const NativeStepContext&) {
            model->opt.disableflags |= mjDSBL_AUTORESET;
            data->qvel[0] = std::numeric_limits<double>::quiet_NaN();
        });
    if (updater == 0 || !no_autoreset.step(1, &error))
    {
        std::cerr << (error.empty() ? "Could not inject disabled-autoreset warning." : error)
                  << '\n';
        return EXIT_FAILURE;
    }
    no_autoreset.remove_model_updater(updater);
    int warning_count = 0;
    no_autoreset.read_native(
        [&](const mjModel*, const mjData* data) {
            warning_count = data->warning[mjWARN_BADQVEL].number;
        });
    if (warning_count == 0 ||
        no_autoreset.reset_generation() != generation_before_warning)
    {
        std::cerr << "Disabled autoreset warning was misclassified as a reset.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
