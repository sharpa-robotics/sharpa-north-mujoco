// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/c_api.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

namespace
{
bool scene_api_test()
{
    char directory[] = "/tmp/sharpa_mujoco_scene_api.XXXXXX";
    if (!mkdtemp(directory)) return false;
    const std::string xml = std::string(directory) + "/scene.xml";
    char error[2048] = {};
    sharpa_mujoco_scene_geom geom{};
    geom.name = "api_box_geom";
    geom.type = SHARPA_MUJOCO_BOX;
    geom.local_pose.quaternion[0] = 1.0;
    std::fill(std::begin(geom.size), std::end(geom.size), 0.1);
    geom.density = 1000.0;
    geom.mass = -1.0;
    sharpa_mujoco_scene_body body{};
    body.name = "api_box";
    body.world_pose.quaternion[0] = 1.0;
    body.geoms = &geom;
    body.geom_count = 1;
    sharpa_mujoco_scene scene{};
    scene.robot_xml_path = SHARPA_MUJOCO_TEST_MODEL;
    scene.robot_world_pose.quaternion[0] = 1.0;
    scene.bodies = &body;
    scene.body_count = 1;

    const auto read_contact = [&](std::array<double, 7>& values) {
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(xml.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) return false;
        const int id = mj_name2id(model.get(), mjOBJ_GEOM, geom.name);
        if (id < 0) return false;
        std::copy_n(model->geom_solref + 2 * id, 2, values.begin());
        std::copy_n(model->geom_solimp + 5 * id, 5, values.begin() + 2);
        return true;
    };
    const auto build_scene = [&] {
        return sharpa_mujoco_scene_build(
            &scene, xml.c_str(), nullptr, error, sizeof(error)) != 0;
    };
    const bool passed = [&] {
        std::array<double, 7> defaults{}, actual{};
        if (!build_scene() || !read_contact(defaults)) return false;

        geom.contact = {
            SHARPA_MUJOCO_SCENE_CONTACT_SOLREF | SHARPA_MUJOCO_SCENE_CONTACT_SOLIMP,
            {0.008, 1.0}, {0.95, 0.99, 0.0005, 0.5, 2.0}};
        if (!build_scene() || !read_contact(actual)) return false;
        const std::array<double, 7> expected{0.008, 1.0, 0.95, 0.99, 0.0005, 0.5, 2.0};
        for (std::size_t i = 0; i < actual.size(); ++i)
            if (std::abs(actual[i] - expected[i]) > 1e-12) return false;

        // Disabling overrides restores model defaults even if values remain.
        geom.contact.flags = 0;
        if (!build_scene() || !read_contact(actual) || defaults != actual)
            return false;
        geom.contact.flags = 1u << 31;
        if (build_scene() || error[0] == '\0') return false;
        geom.contact.flags = SHARPA_MUJOCO_SCENE_CONTACT_SOLREF;
        geom.contact.solref[0] = 0.0;
        if (build_scene() || error[0] == '\0') return false;
        geom.contact = {};
        const std::array<double, 12> vertices{
            0.0, 0.0, 0.0,
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1};
        std::array<int, 12> faces{0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
        geom.type = SHARPA_MUJOCO_MESH;
        geom.mesh_name = "api_inline_mesh";
        geom.mesh_vertices = vertices.data();
        geom.mesh_vertex_count = 4;
        geom.mesh_faces = faces.data();
        geom.mesh_face_count = 4;
        if (!build_scene() || !std::filesystem::is_regular_file(xml)) return false;
        std::filesystem::remove(xml);
        sharpa_mujoco_runtime* runtime = sharpa_mujoco_runtime_create();
        const bool loaded_from_memory = runtime &&
            sharpa_mujoco_runtime_load_scene(
                runtime, &scene, error, sizeof(error)) != 0 &&
            sharpa_mujoco_runtime_body_count(runtime) >= 3;
        sharpa_mujoco_runtime_destroy(runtime);
        if (!loaded_from_memory || std::filesystem::exists(xml)) return false;
        faces[0] = 4;
        runtime = sharpa_mujoco_runtime_create();
        const bool rejected_invalid_face = runtime &&
            sharpa_mujoco_runtime_load_scene(
                runtime, &scene, error, sizeof(error)) == 0 &&
            error[0] != '\0';
        sharpa_mujoco_runtime_destroy(runtime);
        if (!rejected_invalid_face) return false;
        return true;
    }();
    std::filesystem::remove_all(directory);
    if (!passed) std::cerr << "FAILED: scene C ABI regression: " << error << '\n';
    return passed;
}
}  // namespace

int main()
{
    if (!scene_api_test()) return EXIT_FAILURE;
    sharpa_mujoco_runtime* runtime = sharpa_mujoco_runtime_create();
    char error[2048] = {};
    if (!runtime ||
        !sharpa_mujoco_runtime_load(runtime, SHARPA_MUJOCO_TEST_MODEL, error, sizeof(error)))
    {
        std::cerr << "FAILED: " << error << '\n';
        sharpa_mujoco_runtime_destroy(runtime);
        return EXIT_FAILURE;
    }

    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity()})
    {
        if (sharpa_mujoco_runtime_submit_control(runtime, &invalid, 1, 1, error, sizeof(error)) ||
            std::string(error).find("finite") == std::string::npos)
        {
            std::cerr << "FAILED: C API accepted non-finite control or omitted its error\n";
            sharpa_mujoco_runtime_destroy(runtime);
            return EXIT_FAILURE;
        }
    }
    const int body_count = sharpa_mujoco_runtime_body_count(runtime);
    sharpa_mujoco_body_info body{};
    char body_name[256] = {};
    std::vector<double> positions(static_cast<size_t>(body_count * 3));
    std::vector<double> quaternions(static_cast<size_t>(body_count * 4));
    uint64_t version = 0;
    uint64_t step = 0;
    double time = 0.0;
    sharpa_mujoco_pose_timing timing{};
    const bool stepped = sharpa_mujoco_runtime_step(runtime, 1, error, sizeof(error)) != 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const bool timed_valid = stepped && body_count == 2 &&
        sharpa_mujoco_runtime_body_info(runtime, 1, &body, body_name, sizeof(body_name)) &&
        body.parent_id == 0 && body.joint_count == 2 &&
        sharpa_mujoco_runtime_copy_body_pose_timed(
            runtime, &version, &step, &time,
            positions.data(), positions.size(), quaternions.data(), quaternions.size(),
            &timing) &&
        timing.available == 1 && timing.version == version &&
        timing.snapshot_age_seconds > 0.0 &&
        timing.publish_interval_seconds > 0.0 &&
        timing.simulation_step_seconds > 0.0 && timing.physics_step_seconds > 0.0 &&
        timing.snapshot_build_seconds > 0.0 && timing.pose_copy_seconds > 0.0;
    uint64_t legacy_version = 0;
    uint64_t legacy_step = 0;
    double legacy_time = 0.0;
    const bool legacy_valid = sharpa_mujoco_runtime_copy_body_pose(
        runtime, &legacy_version, &legacy_step, &legacy_time,
        positions.data(), positions.size(), quaternions.data(), quaternions.size()) != 0;
    const bool legacy_matches_timed = legacy_version == version && legacy_step == step;

    sharpa_mujoco_runtime_set_pose_timing_enabled(runtime, 0);
    const bool disabled_step =
        sharpa_mujoco_runtime_step(runtime, 1, error, sizeof(error)) != 0;
    sharpa_mujoco_pose_timing disabled_timing{};
    const bool disabled_timing_valid = disabled_step &&
        sharpa_mujoco_runtime_copy_body_pose_timed(
            runtime, &version, &step, &time,
            positions.data(), positions.size(), quaternions.data(), quaternions.size(),
            &disabled_timing) != 0 &&
        disabled_timing.available == 0 && disabled_timing.version == version &&
        disabled_timing.snapshot_age_seconds == 0.0 &&
        disabled_timing.publish_interval_seconds == 0.0 &&
        disabled_timing.simulation_step_seconds == 0.0 &&
        disabled_timing.physics_step_seconds == 0.0 &&
        disabled_timing.snapshot_build_seconds == 0.0 &&
        disabled_timing.pose_copy_seconds > 0.0;
    sharpa_mujoco_runtime_destroy(runtime);
    if (!timed_valid || !legacy_valid || !legacy_matches_timed || !disabled_timing_valid)
    {
        std::cerr
            << "FAILED: C ABI topology or pose bridge is invalid"
            << " timed=" << timed_valid
            << " legacy=" << legacy_valid
            << " legacy_match=" << legacy_matches_timed
            << " disabled=" << disabled_timing_valid
            << " disabled_version=" << disabled_timing.version
            << " version=" << version
            << " disabled_age=" << disabled_timing.snapshot_age_seconds
            << " disabled_cadence=" << disabled_timing.publish_interval_seconds
            << " disabled_step=" << disabled_timing.simulation_step_seconds
            << " disabled_physics=" << disabled_timing.physics_step_seconds
            << " disabled_build=" << disabled_timing.snapshot_build_seconds
            << " disabled_copy=" << disabled_timing.pose_copy_seconds
            << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "Sharpa MuJoCo C ABI test passed.\n";
    return EXIT_SUCCESS;
}
