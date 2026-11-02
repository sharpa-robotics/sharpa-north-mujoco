// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/runtime.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

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

    if (!require(runtime.load(SHARPA_MUJOCO_TEST_MODEL, &error), error))
    {
        return EXIT_FAILURE;
    }

    const sharpa::mujoco::ModelInfo model = runtime.model_info();
    if (!require(model.body_count == 2, "Expected world and test body.") ||
        !require(model.geom_count == 2, "Expected floor and test geom.") ||
        !require(model.joint_count == 2, "Expected two joints.") ||
        !require(model.actuator_count == 2, "Expected two actuators.") ||
        !require(model.sensor_count == 1, "Expected one sensor.") ||
        !require(model.bodies[1].parent_id == 0, "Body parent topology is wrong.") ||
        !require(model.bodies[1].joint_address == 0, "Body joint address is wrong.") ||
        !require(model.bodies[1].joint_count == 2, "Body joint count is wrong.") ||
        !require(model.sensors.front().name == "test_joint_position", "Sensor name lookup failed.") ||
        !require(model.sensors.front().dimension == 1, "Sensor dimension lookup failed.") ||
        !require(model.actuators.front().name == "test_actuator", "Actuator name lookup failed."))
    {
        return EXIT_FAILURE;
    }

    const std::filesystem::path save_root =
        std::filesystem::temp_directory_path() / "sharpa_mujoco_runtime_test";
    std::filesystem::create_directories(save_root);
    if (!require(runtime.save_xml((save_root / "saved.xml").string(), &error), error) ||
        !require(runtime.save_mjb((save_root / "saved.mjb").string(), &error), error) ||
        !require(std::filesystem::file_size(save_root / "saved.xml") > 0, "Saved XML is empty.") ||
        !require(std::filesystem::file_size(save_root / "saved.mjb") > 0, "Saved MJB is empty."))
    {
        return EXIT_FAILURE;
    }

    const std::filesystem::path original_directory =
        std::filesystem::current_path();
    std::filesystem::current_path(save_root);
    const bool relative_save_ok =
        runtime.save_xml("saved-relative.xml", &error) &&
        runtime.save_mjb("saved-relative.mjb", &error);
    std::filesystem::current_path(original_directory);
    if (!require(relative_save_ok, error))
    {
        return EXIT_FAILURE;
    }

    sharpa::mujoco::Runtime robot_runtime;
    const std::filesystem::path robot_save_path =
        save_root / "saved-robot.xml";
    sharpa::mujoco::Runtime reloaded_robot_runtime;
    if (!require(
            robot_runtime.load(SHARPA_MUJOCO_RUNTIME_ROBOT_MODEL, &error),
            error) ||
        !require(robot_runtime.save_xml(robot_save_path.string(), &error), error) ||
        !require(
            reloaded_robot_runtime.load(robot_save_path.string(), &error),
            error))
    {
        return EXIT_FAILURE;
    }

    int updater_count = 0;
    const std::uint64_t updater_id = runtime.add_model_updater(
        [&updater_count](
            mjModel* native_model,
            mjData*,
            const sharpa::mujoco::NativeStepContext&) {
            ++updater_count;
            native_model->dof_armature[0] = 0.25;
        });
    if (!require(updater_id != 0, "Model updater registration failed.") ||
        !require(runtime.step(1, &error), error))
    {
        return EXIT_FAILURE;
    }
    double updated_armature = 0.0;
    if (!require(
            runtime.read_native(
                [&updated_armature](const mjModel* native_model, const mjData*) {
                    updated_armature = native_model->dof_armature[0];
                },
                &error),
            error) ||
        !require(
            updater_count == 1 && std::abs(updated_armature - 0.25) < 1e-12,
            "Model updater did not run before the manual step."))
    {
        return EXIT_FAILURE;
    }
    runtime.remove_model_updater(updater_id);
    if (!require(runtime.step(1, &error), error) ||
        !require(updater_count == 1, "Removed model updater still ran."))
    {
        return EXIT_FAILURE;
    }

    if (!require(runtime.has_editable_spec(), "XML model should retain an editable specification.") ||
        !require(
            runtime.edit_spec(
                [](mjSpec* spec)
                {
                    mjsBody* world = mjs_findBody(spec, "world");
                    if (!world)
                    {
                        world = mjs_asBody(mjs_firstElement(spec, mjOBJ_BODY));
                    }
                    if (!world)
                    {
                        return false;
                    }

                    mjsBody* body = mjs_addBody(world, nullptr);
                    if (!body)
                    {
                        return false;
                    }
                    mjs_setName(body->element, "runtime_added_body");

                    mjsGeom* geom = mjs_addGeom(body, nullptr);
                    if (!geom)
                    {
                        return false;
                    }
                    mjs_setName(geom->element, "runtime_added_geom");
                    geom->type = mjGEOM_SPHERE;
                    geom->size[0] = 0.1;
                    return true;
                },
                &error),
            error) ||
        !require(runtime.recompile_spec(&error), error))
    {
        return EXIT_FAILURE;
    }

    const sharpa::mujoco::ModelInfo edited_model = runtime.model_info();
    if (!require(edited_model.body_count == 3, "Spec edit did not add a body.") ||
        !require(edited_model.geom_count == 3, "Spec edit did not add a geom."))
    {
        return EXIT_FAILURE;
    }

    bool spec_serialized = false;
    if (!require(
            runtime.read_spec(
                [&spec_serialized](const mjSpec* spec)
                {
                    char xml[16384] = {};
                    char save_error[1024] = {};
                    spec_serialized =
                        mj_saveXMLString(
                            spec,
                            xml,
                            static_cast<int>(sizeof(xml)),
                            save_error,
                            static_cast<int>(sizeof(save_error))) == 0 &&
                        std::string(xml).find("runtime_added_body") != std::string::npos;
                },
                &error),
            error) ||
        !require(spec_serialized, "Spec read did not serialize the edited XML."))
    {
        return EXIT_FAILURE;
    }

    sharpa::mujoco::ControlCommand command;
    command.replace_all = false;
    command.named_values.push_back({"test_actuator", 0.25});
    sharpa::mujoco::ControlCommand second_command;
    second_command.replace_all = false;
    second_command.named_values.push_back({"test_slide_actuator", 0.125});
    if (!require(runtime.submit_control(command, &error), error) ||
        !require(runtime.submit_control(second_command, &error), error) ||
        !require(runtime.step(10, &error), error))
    {
        return EXIT_FAILURE;
    }

    const sharpa::mujoco::PoseSnapshot stepped = runtime.latest_snapshot();
    if (!require(stepped.valid(), "Manual-step snapshot is invalid.") ||
        !require(stepped.version >= 2, "Snapshot version did not advance.") ||
        !require(stepped.simulation_time > 0.0, "Simulation time did not advance.") ||
        !require(stepped.step_index == 12, "Manual step index is wrong.") ||
        !require(stepped.geom_count == 3, "Geom snapshot count is wrong.") ||
        !require(stepped.geom_positions.size() == 9, "Geom position snapshot size is wrong.") ||
        !require(stepped.geom_quaternions.size() == 12, "Geom quaternion snapshot size is wrong.") ||
        !require(stepped.controls.size() == 2, "Control snapshot size is wrong.") ||
        !require(stepped.sensor_values.size() == 1, "Sensor snapshot size is wrong.") ||
        !require(std::abs(stepped.controls[0] - 0.25) < 1e-9, "First merged control was not applied.") ||
        !require(std::abs(stepped.controls[1] - 0.125) < 1e-9, "Second merged control was not applied."))
    {
        return EXIT_FAILURE;
    }

    const std::uint64_t reset_generation = runtime.reset_generation();
    if (!require(runtime.reset(&error), error))
    {
        return EXIT_FAILURE;
    }
    const sharpa::mujoco::PoseSnapshot reset = runtime.latest_snapshot();
    if (!require(std::abs(reset.simulation_time) < 1e-12, "Reset did not restore simulation time.") ||
        !require(reset.step_index == 0, "Reset did not restore the step index.") ||
        !require(runtime.reset_generation() == reset_generation + 1,
                 "Reset generation did not advance."))
    {
        return EXIT_FAILURE;
    }

    bool native_read_called = false;
    if (!require(
            runtime.read_native(
                [&native_read_called](const mjModel* native_model, const mjData* native_data) {
                    native_read_called = native_model != nullptr && native_data != nullptr;
                },
                &error),
            error) ||
        !require(native_read_called, "Native read callback was not invoked."))
    {
        return EXIT_FAILURE;
    }

    if (!require(
            runtime.write_native(
                [](const mjModel*, mjData* native_data) {
                    native_data->qpos[0] = 0.125;
                },
                true,
                &error),
            error) ||
        !require(
            std::abs(runtime.latest_snapshot().qpos.front() - 0.125) < 1e-9,
            "Native write was not reflected in the snapshot."))
    {
        return EXIT_FAILURE;
    }

    if (!require(runtime.reset(&error), error))
    {
        return EXIT_FAILURE;
    }

    sharpa::mujoco::RuntimeConfig gated_config = runtime.config();
    gated_config.scheduler_mode = sharpa::mujoco::SchedulerMode::ActionGated;
    gated_config.action_hold_substeps = 2;
    gated_config.snapshot_rate_hz = 0.0;
    gated_config.action_wait_timeout_seconds = 0.01;
    if (!require(runtime.set_config(gated_config, &error), error))
    {
        return EXIT_FAILURE;
    }

    std::atomic<bool> action_ready{false};
    std::atomic<int> before_count{0};
    std::atomic<int> after_count{0};
    runtime.set_step_callbacks(
        [&action_ready, &before_count](
            const mjModel*,
            mjData*,
            const sharpa::mujoco::NativeStepContext&) {
            if (!action_ready.load())
            {
                return sharpa::mujoco::StepDecision::Wait;
            }
            ++before_count;
            return sharpa::mujoco::StepDecision::Proceed;
        },
        [&action_ready, &after_count](
            const mjModel*,
            mjData*,
            const sharpa::mujoco::NativeStepContext& context) {
            ++after_count;
            if (context.frame_substep_index + 1 == context.frame_substep_count)
            {
                action_ready.store(false);
            }
        });

    if (!require(runtime.start(&error), error))
    {
        return EXIT_FAILURE;
    }
    if (!require(
            !runtime.edit_spec([](mjSpec*) { return true; }, &error),
            "Spec edit should be rejected while the runtime is running."))
    {
        runtime.pause();
        return EXIT_FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!require(
            runtime.latest_snapshot().step_index == 0,
            "Action-gated runtime stepped without an action."))
    {
        runtime.pause();
        return EXIT_FAILURE;
    }

    action_ready.store(true);
    runtime.notify_step_ready();
    const auto action_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (after_count.load() < 2 && std::chrono::steady_clock::now() < action_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    runtime.pause();

    const sharpa::mujoco::PoseSnapshot gated = runtime.latest_snapshot();
    if (!require(before_count.load() == 2, "Action was not held for exactly two substeps.") ||
        !require(after_count.load() == 2, "After-step callback count is wrong.") ||
        !require(gated.step_index == 2, "Action-gated runtime advanced the wrong number of steps."))
    {
        return EXIT_FAILURE;
    }

    runtime.clear_step_callbacks();
    sharpa::mujoco::RuntimeConfig realtime_config = runtime.config();
    realtime_config.scheduler_mode = sharpa::mujoco::SchedulerMode::Realtime;
    realtime_config.snapshot_rate_hz = 60.0;
    if (!require(runtime.set_config(realtime_config, &error), error) ||
        !require(runtime.start(&error), error))
    {
        return EXIT_FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    runtime.pause();

    const sharpa::mujoco::PoseSnapshot threaded = runtime.latest_snapshot();
    if (!require(threaded.step_index > gated.step_index, "Realtime simulation thread did not advance."))
    {
        return EXIT_FAILURE;
    }

    std::cout << "Sharpa MuJoCo runtime smoke test passed.\n";
    return EXIT_SUCCESS;
}
