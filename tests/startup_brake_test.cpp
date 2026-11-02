// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/common/startup_brake.h"
#include "sharpa_mujoco/runtime.h"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
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

}  // namespace

int main()
{
    sharpa::mujoco::Runtime runtime;
    std::string error;
    if (!require(
            runtime.load(SHARPA_MUJOCO_NTH_HA1_MODEL, &error),
            error))
    {
        return EXIT_FAILURE;
    }

    sharpa::mujoco::plugins::common::StartupBrake brake;
    if (!require(
            brake.initialize(
                runtime,
                {"lower_body_joint_1", "lower_body_joint_2"},
                &error),
            error))
    {
        return EXIT_FAILURE;
    }

    bool first_active = false;
    bool second_active = false;
    runtime.read_native(
        [&](const mjModel* model, const mjData* data) {
            const int first =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_1");
            const int second =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_2");
            first_active = first >= 0 && data->eq_active[first] != 0;
            second_active = second >= 0 && data->eq_active[second] != 0;
        },
        &error);
    if (!require(first_active && second_active,
                 "Power-on brake constraints should start active."))
    {
        return EXIT_FAILURE;
    }

    if (!require(brake.release({0, 1}, &error), error) ||
        !require(brake.release({0, 1}, &error),
                 "Repeated brake release should be idempotent."))
    {
        return EXIT_FAILURE;
    }

    runtime.read_native(
        [&](const mjModel* model, const mjData* data) {
            const int first =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_1");
            const int second =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_2");
            first_active = first >= 0 && data->eq_active[first] != 0;
            second_active = second >= 0 && data->eq_active[second] != 0;
        },
        &error);
    if (!require(!first_active && !second_active,
                 "Released startup brakes should deactivate their constraints.") ||
        !require(brake.released(0) && brake.released(1),
                 "Startup brake should remember released axes."))
    {
        return EXIT_FAILURE;
    }

    const std::uint64_t reset_generation = runtime.reset_generation();
    if (!require(runtime.reset(&error), error) ||
        !require(runtime.reset_generation() == reset_generation + 1,
                 "Runtime reset generation did not advance."))
    {
        return EXIT_FAILURE;
    }
    runtime.read_native(
        [&](const mjModel* model, const mjData* data) {
            const int first =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_1");
            const int second =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_2");
            first_active = first >= 0 && data->eq_active[first] != 0;
            second_active = second >= 0 && data->eq_active[second] != 0;
        },
        &error);
    if (!require(first_active && second_active,
                 "Runtime reset should restore startup brake constraints.") ||
        !require(!brake.released(0) && !brake.released(1),
                 "Startup brake cache did not synchronize after runtime reset.") ||
        !require(brake.release({0, 1}, &error), error))
    {
        return EXIT_FAILURE;
    }
    runtime.read_native(
        [&](const mjModel* model, const mjData* data) {
            const int first =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_1");
            const int second =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_2");
            first_active = first >= 0 && data->eq_active[first] != 0;
            second_active = second >= 0 && data->eq_active[second] != 0;
        },
        &error);
    if (!require(!first_active && !second_active,
                 "Startup brakes were not released again after runtime reset."))
    {
        return EXIT_FAILURE;
    }

    // MuJoCo also resets mjData itself when dynamics become invalid. This is
    // the same lifecycle event as Runtime::reset for every device plugin.
    if (!require(runtime.step(5, &error), error))
    {
        return EXIT_FAILURE;
    }
    const std::uint64_t automatic_reset_generation =
        runtime.reset_generation();
    runtime.set_step_callbacks(
        [](const mjModel*, mjData* data, const sharpa::mujoco::NativeStepContext&) {
            data->qvel[0] = std::numeric_limits<double>::quiet_NaN();
            return sharpa::mujoco::StepDecision::Proceed;
        },
        {});
    if (!require(runtime.step(1, &error), error))
    {
        return EXIT_FAILURE;
    }
    runtime.clear_step_callbacks();
    runtime.read_native(
        [&](const mjModel* model, const mjData* data) {
            const int first =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_1");
            const int second =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_2");
            first_active = first >= 0 && data->eq_active[first] != 0;
            second_active = second >= 0 && data->eq_active[second] != 0;
        },
        &error);
    if (!require(
            runtime.reset_generation() == automatic_reset_generation + 1,
            "MuJoCo automatic reset did not advance the reset generation.") ||
        !require(first_active && second_active,
                 "MuJoCo automatic reset should restore startup brakes.") ||
        !require(!brake.released(0) && !brake.released(1),
                 "Startup brake cache did not synchronize after automatic reset.") ||
        !require(brake.release({0, 1}, &error), error))
    {
        return EXIT_FAILURE;
    }
    runtime.read_native(
        [&](const mjModel* model, const mjData* data) {
            const int first =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_1");
            const int second =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_2");
            first_active = first >= 0 && data->eq_active[first] != 0;
            second_active = second >= 0 && data->eq_active[second] != 0;
        },
        &error);
    if (!require(!first_active && !second_active,
                 "Startup brakes were not released after automatic reset."))
    {
        return EXIT_FAILURE;
    }

    double locked_position = 0.0;
    if (!require(
            runtime.write_native(
                [&](const mjModel* model, mjData* data) {
                    const int joint = mj_name2id(
                        model, mjOBJ_JOINT, "lower_body_joint_1");
                    const int qpos = model->jnt_qposadr[joint];
                    data->qpos[qpos] = 0.25;
                    locked_position = data->qpos[qpos];
                },
                true,
                &error),
            error) ||
        !require(brake.engage({0, 1}, &error), error) ||
        !require(brake.engage({0, 1}, &error),
                 "Repeated brake engagement should be idempotent."))
    {
        return EXIT_FAILURE;
    }

    double equality_target = 0.0;
    runtime.read_native(
        [&](const mjModel* model, const mjData* data) {
            const int first =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_1");
            const int second =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_2");
            first_active = first >= 0 && data->eq_active[first] != 0;
            second_active = second >= 0 && data->eq_active[second] != 0;
            equality_target = model->eq_data[mjNEQDATA * first];
        },
        &error);
    if (!require(first_active && second_active,
                 "Re-engaged brakes should reactivate their constraints.") ||
        !require(!brake.released(0) && !brake.released(1),
                 "Startup brake should remember engaged axes.") ||
        !require(std::abs(equality_target - locked_position) < 1e-12,
                 "Re-engaged brake should lock at the current position."))
    {
        return EXIT_FAILURE;
    }

    error.clear();
    if (!require(
            !runtime.set_equalities_active(
                {"brake_lower_body_joint_1", "missing_brake"},
                false,
                &error),
            "Invalid atomic brake update should fail."))
    {
        return EXIT_FAILURE;
    }
    runtime.read_native(
        [&](const mjModel* model, const mjData* data) {
            const int first =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_1");
            const int second =
                mj_name2id(model, mjOBJ_EQUALITY, "brake_lower_body_joint_2");
            first_active = first >= 0 && data->eq_active[first] != 0;
            second_active = second >= 0 && data->eq_active[second] != 0;
        },
        &error);
    if (!require(
            first_active && second_active,
            "Failed atomic brake update partially changed constraints."))
    {
        return EXIT_FAILURE;
    }

    std::cout << "Startup brake tests passed.\n";
    return EXIT_SUCCESS;
}
