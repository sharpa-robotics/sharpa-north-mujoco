// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/runtime.h"
#include "sharpa_mujoco/scene_builder.h"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

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
    namespace sm = sharpa::mujoco;
    std::string error;
    const std::filesystem::path output_root =
        std::filesystem::temp_directory_path() / "sharpa_mujoco_scene_builder_test";
    std::filesystem::create_directories(output_root);

    sm::SceneDescription scene;
    scene.robot_xml_path = SHARPA_MUJOCO_TEST_MODEL;
    // An empty name selects the first direct worldbody child.
    scene.robot_root_body.clear();
    scene.robot_world_pose.position = {0.3, -0.2, 0.5};

    sm::SceneBodyDescription floor;
    floor.name = "scene_floor";
    floor.motion_type = sm::SceneMotionType::Static;
    sm::SceneGeomDescription floor_geom;
    floor_geom.name = "scene_floor_geom";
    floor_geom.type = sm::SceneGeomType::Box;
    floor_geom.size = {1.0, 1.0, 0.05};
    floor.geoms.push_back(floor_geom);
    scene.bodies.push_back(floor);

    sm::SceneBodyDescription cube;
    cube.name = "scene_dynamic_cube";
    cube.motion_type = sm::SceneMotionType::Dynamic;
    cube.world_pose.position = {0.0, 0.0, 0.5};
    sm::SceneGeomDescription cube_geom;
    cube_geom.name = "scene_dynamic_cube_geom";
    cube_geom.type = sm::SceneGeomType::Box;
    cube_geom.size = {0.05, 0.05, 0.05};
    cube_geom.mass = 0.25;
    cube_geom.override_solref = true;
    cube_geom.solref = {0.008, 1.0};
    cube_geom.override_solimp = true;
    cube_geom.solimp = {0.95, 0.99, 0.0005, 0.5, 2.0};
    cube.geoms.push_back(cube_geom);
    scene.bodies.push_back(cube);

    const std::filesystem::path mesh_path = output_root / "scene_mesh.obj";
    {
        std::ofstream mesh(mesh_path);
        mesh << "v 0 0 0\n"
             << "v 0.1 0 0\n"
             << "v 0 0.1 0\n"
             << "v 0 0 0.1\n"
             << "f 1 3 2\n"
             << "f 1 2 4\n"
             << "f 1 4 3\n"
             << "f 2 3 4\n";
    }
    if (!require(
            std::filesystem::file_size(mesh_path) > 0,
            "Could not create the scene mesh fixture."))
    {
        return EXIT_FAILURE;
    }

    sm::SceneBodyDescription mesh_body;
    mesh_body.name = "scene_mesh_body";
    sm::SceneGeomDescription mesh_geom;
    mesh_geom.name = "scene_mesh_geom";
    mesh_geom.type = sm::SceneGeomType::Mesh;
    mesh_geom.mesh_file = mesh_path.filename().string();
    mesh_body.geoms.push_back(mesh_geom);
    scene.bodies.push_back(mesh_body);

    const std::filesystem::path unexpected_runtime_xml =
        output_root / "runtime-load-scene-must-not-exist.xml";
    std::filesystem::remove(unexpected_runtime_xml);
    sm::Runtime direct_runtime;
    sm::SceneDescription direct_scene = scene;
    direct_scene.mesh_base_directory = output_root.string();
    if (!require(direct_runtime.load_scene(direct_scene, &error), error) ||
        !require(
            direct_runtime.model_info().body_count == 5,
            "Direct scene loading should compile the robot and scene bodies.") ||
        !require(
            !std::filesystem::exists(unexpected_runtime_xml),
            "Direct scene loading must not serialize an XML artifact."))
    {
        return EXIT_FAILURE;
    }

    sm::SceneBuildOptions options;
    options.output_xml_path = (output_root / "scene.xml").string();
    options.output_mjb_path = (output_root / "scene.mjb").string();
    if (!require(sm::SceneBuilder::build(scene, options, &error), error) ||
        !require(sm::SceneBuilder::validate_xml(options.output_xml_path, &error), error))
    {
        return EXIT_FAILURE;
    }
    std::ifstream generated_xml(options.output_xml_path);
    const std::string generated_xml_text(
        (std::istreambuf_iterator<char>(generated_xml)),
        std::istreambuf_iterator<char>());
    if (!require(
            generated_xml_text.find("file=\"scene_mesh.obj\"") !=
                std::string::npos,
            "Colocated scene mesh path was not serialized relative to the XML."))
    {
        return EXIT_FAILURE;
    }

    char load_error[1024] = {};
    mjModel* generated_model =
        mj_loadXML(options.output_xml_path.c_str(), nullptr, load_error, sizeof(load_error));
    if (!require(generated_model != nullptr, load_error))
    {
        return EXIT_FAILURE;
    }
    const int cube_geom_id =
        mj_name2id(generated_model, mjOBJ_GEOM, cube_geom.name.c_str());
    const bool contact_response_matches =
        cube_geom_id >= 0 &&
        std::abs(generated_model->geom_solref[2 * cube_geom_id] - 0.008) < 1e-12 &&
        std::abs(generated_model->geom_solref[2 * cube_geom_id + 1] - 1.0) < 1e-12 &&
        std::abs(generated_model->geom_solimp[5 * cube_geom_id] - 0.95) < 1e-12 &&
        std::abs(generated_model->geom_solimp[5 * cube_geom_id + 1] - 0.99) < 1e-12 &&
        std::abs(generated_model->geom_solimp[5 * cube_geom_id + 2] - 0.0005) < 1e-12 &&
        std::abs(generated_model->geom_solimp[5 * cube_geom_id + 3] - 0.5) < 1e-12 &&
        std::abs(generated_model->geom_solimp[5 * cube_geom_id + 4] - 2.0) < 1e-12;
    mj_deleteModel(generated_model);
    if (!require(
            contact_response_matches,
            "SceneBuilder did not preserve the per-geom contact response."))
    {
        return EXIT_FAILURE;
    }

    const std::filesystem::path reusable_robot_path =
        output_root / "reusable-robot.xml";
    {
        std::ofstream robot_xml(reusable_robot_path);
        robot_xml
            << "<mujoco model=\"reusable_mesh\">\n"
            << "  <asset><mesh name=\"robot_mesh\" file=\"scene_mesh.obj\"/></asset>\n"
            << "  <worldbody><body name=\"robot\"><geom type=\"mesh\" mesh=\"robot_mesh\"/></body></worldbody>\n"
            << "</mujoco>\n";
    }
    sm::SceneDescription reusable_scene;
    reusable_scene.robot_xml_path = reusable_robot_path.string();
    sm::SceneBodyDescription reused_mesh_body;
    reused_mesh_body.name = "reused_mesh_body";
    sm::SceneGeomDescription reused_mesh_geom;
    reused_mesh_geom.type = sm::SceneGeomType::Mesh;
    reused_mesh_geom.mesh_file = mesh_path.filename().string();
    reused_mesh_body.geoms.push_back(reused_mesh_geom);
    reusable_scene.bodies.push_back(reused_mesh_body);
    sm::SceneBodyDescription named_mesh_body;
    named_mesh_body.name = "named_mesh_body";
    sm::SceneGeomDescription named_mesh_geom;
    named_mesh_geom.type = sm::SceneGeomType::Mesh;
    named_mesh_geom.mesh_name = "robot_mesh";
    named_mesh_body.geoms.push_back(named_mesh_geom);
    reusable_scene.bodies.push_back(named_mesh_body);
    sm::SceneBuildOptions reusable_options;
    reusable_options.output_xml_path =
        (output_root / "reused-mesh-scene.xml").string();
    if (!require(
            sm::SceneBuilder::build(
                reusable_scene, reusable_options, &error),
            error) ||
        !require(
            sm::SceneBuilder::validate_xml(
                reusable_options.output_xml_path, &error),
            error))
    {
        return EXIT_FAILURE;
    }
    std::ifstream reused_xml(reusable_options.output_xml_path);
    const std::string reused_xml_text(
        (std::istreambuf_iterator<char>(reused_xml)),
        std::istreambuf_iterator<char>());
    const std::string mesh_asset_marker = "<mesh name=";
    const std::size_t first_mesh = reused_xml_text.find(mesh_asset_marker);
    if (!require(
            first_mesh != std::string::npos &&
                reused_xml_text.find(
                    mesh_asset_marker,
                    first_mesh + mesh_asset_marker.size()) ==
                    std::string::npos &&
                reused_xml_text.find("mesh=\"robot_mesh\"") !=
                    std::string::npos,
            "SceneBuilder duplicated an existing robot mesh asset."))
    {
        return EXIT_FAILURE;
    }

    sm::Runtime runtime;
    if (!require(runtime.load(options.output_xml_path, &error), error))
    {
        return EXIT_FAILURE;
    }
    const sm::ModelInfo model = runtime.model_info();
    if (!require(model.body_count == 5, "Generated model should contain world, robot and three scene bodies.") ||
        !require(model.joint_count == 3, "Dynamic scene body should add one free joint.") ||
        !require(std::filesystem::file_size(options.output_mjb_path) > 0, "Generated MJB is empty."))
    {
        return EXIT_FAILURE;
    }

    const sm::PoseSnapshot pose = runtime.latest_snapshot();
    if (!require(pose.valid(), "Generated scene did not publish body poses.") ||
        !require(std::abs(pose.body_positions[3] - 0.3) < 1e-9, "Robot root X pose was not baked."))
    {
        return EXIT_FAILURE;
    }

    const std::filesystem::path original_directory =
        std::filesystem::current_path();
    std::filesystem::current_path(output_root);
    sm::SceneBuildOptions relative_options;
    relative_options.output_xml_path = "scene-relative.xml";
    relative_options.output_mjb_path = "scene-relative.mjb";
    const bool relative_build_ok =
        sm::SceneBuilder::build(scene, relative_options, &error) &&
        sm::SceneBuilder::validate_xml(relative_options.output_xml_path, &error);
    std::filesystem::current_path(original_directory);
    if (!require(relative_build_ok, error))
    {
        return EXIT_FAILURE;
    }

    sm::SceneDescription robot_scene;
    robot_scene.robot_xml_path = SHARPA_MUJOCO_SCENE_ROBOT_MODEL;
    sm::SceneBuildOptions robot_options;
    robot_options.output_xml_path =
        (output_root / "robot-scene.xml").string();
    if (!require(sm::SceneBuilder::build(robot_scene, robot_options, &error), error) ||
        !require(sm::SceneBuilder::validate_xml(robot_options.output_xml_path, &error), error))
    {
        return EXIT_FAILURE;
    }

    std::cout << "Sharpa MuJoCo scene builder test passed.\n";
    return EXIT_SUCCESS;
}
