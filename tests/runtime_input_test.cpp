// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/runtime.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void require(bool value, const std::string& message)
{
    if (!value) throw std::runtime_error(message);
}

void check_controls()
{
    using namespace sharpa::mujoco;
    Runtime runtime;
    std::string error;
    require(runtime.load(SHARPA_MUJOCO_TEST_MODEL, &error), error);
    for (ControlMode mode : {ControlMode::Actuator, ControlMode::DirectJointPosition})
    {
        for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()})
        {
            for (bool named : {false, true})
            {
                for (bool replace : {false, true})
                {
                    ControlCommand valid;
                    valid.mode = mode;
                    valid.ordered_values = {0.125, -0.25};
                    require(runtime.submit_control(valid, &error), error);
                    ControlCommand invalid;
                    invalid.mode = mode;
                    invalid.replace_all = replace;
                    invalid.ordered_values = {0.75, bad};
                    if (named)
                    {
                        invalid.ordered_values = {0.75};
                        invalid.named_values = {{"test_slide_actuator", bad}};
                    }
                    require(!runtime.submit_control(invalid, &error), "Accepted non-finite control");
                    require(error.find("finite") != std::string::npos, "Missing validation error");
                    require(runtime.step(1, &error), error);
                    const auto snapshot = runtime.latest_snapshot();
                    require(snapshot.controls == valid.ordered_values,
                            "Rejected command overwrote or partially merged into the pending command");
                    require(std::all_of(snapshot.body_positions.begin(), snapshot.body_positions.end(),
                                        [](double value) { return std::isfinite(value); }),
                            "Rejected command corrupted simulation state");
                }
            }
        }
    }
}

void check_mocap()
{
    using namespace sharpa::mujoco;
    Runtime runtime;
    std::string error;
    require(runtime.load(SHARPA_MUJOCO_TEST_MODEL, &error), error);
    require(runtime.edit_spec([](mjSpec* spec) {
        auto* body = mjs_addBody(mjs_findBody(spec, "world"), nullptr);
        mjs_setName(body->element, "test_mocap");
        body->mocap = true;
        return true;
    }, &error), error);
    require(runtime.recompile_spec(&error), error);
    Pose valid;
    valid.position = {0.1, 0.2, 0.3};
    valid.quaternion = {2.0, 0.0, 0.0, 2.0};
    require(runtime.set_mocap_pose("test_mocap", valid, &error), error);
    const auto before = runtime.latest_snapshot();
    const auto reject = [&](const Pose& pose) {
        require(!runtime.set_mocap_pose("test_mocap", pose, &error), "Accepted invalid mocap pose");
        const auto after = runtime.latest_snapshot();
        require(after.version == before.version && after.body_positions == before.body_positions &&
                after.body_quaternions == before.body_quaternions,
                "Invalid mocap pose changed the published snapshot");
        require(runtime.read_native([&](const mjModel*, const mjData* data) {
            require(std::equal(valid.position.begin(), valid.position.end(), data->mocap_pos),
                    "Invalid pose partially overwrote native mocap position");
        }, &error), error);
    };
    for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity()})
    {
        for (std::size_t index = 0; index < 7; ++index)
        {
            Pose invalid = valid;
            invalid.position[0] = 0.9;
            if (index < 3) invalid.position[index] = bad;
            else invalid.quaternion[index - 3] = bad;
            reject(invalid);
        }
    }
    Pose zero = valid;
    zero.quaternion.fill(0.0);
    reject(zero);
    for (double scale : {2.0, std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::denorm_min()})
    {
        Pose pose = valid;
        pose.quaternion = {scale, 0.0, 0.0, scale};
        require(runtime.set_mocap_pose("test_mocap", pose, &error), error);
        bool normalized = false;
        require(runtime.read_native([&](const mjModel*, const mjData* data) {
            normalized = std::abs(data->mocap_quat[0] - std::sqrt(0.5)) < 1e-12 &&
                         std::abs(data->mocap_quat[3] - std::sqrt(0.5)) < 1e-12;
        }, &error), error);
        require(normalized, "Finite quaternion was not normalized correctly");
    }
}
}

int main()
{
    try { check_controls(); check_mocap(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
