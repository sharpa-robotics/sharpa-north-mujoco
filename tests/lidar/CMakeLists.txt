# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

add_executable(
    sharpa_mujoco_lidar_protocol_test
    protocol_test.cpp
)
target_link_libraries(
    sharpa_mujoco_lidar_protocol_test
    PRIVATE
        SharpaMujoco::DeviceCommon
        SharpaMujoco::LidarPlugin
)
add_test(
    NAME sharpa_mujoco_lidar_protocol
    COMMAND sharpa_mujoco_lidar_protocol_test
)

add_executable(
    sharpa_mujoco_lidar_scan_pattern_test
    scan_pattern_test.cpp
)
target_compile_definitions(
    sharpa_mujoco_lidar_scan_pattern_test
    PRIVATE
        SHARPA_MUJOCO_FTX_CALIBRATION="${PROJECT_SOURCE_DIR}/resources/lidar/calibration/FTX140_angle_calib_file.csv"
        SHARPA_MUJOCO_JT16_CALIBRATION="${PROJECT_SOURCE_DIR}/resources/lidar/calibration/JT16_sample_angle.csv"
)
target_link_libraries(
    sharpa_mujoco_lidar_scan_pattern_test
    PRIVATE SharpaMujoco::LidarPlugin
)
add_test(
    NAME sharpa_mujoco_lidar_scan_pattern
    COMMAND sharpa_mujoco_lidar_scan_pattern_test
)

add_executable(
    sharpa_mujoco_lidar_model_attachment_test
    model_attachment_test.cpp
    "${PROJECT_SOURCE_DIR}/apps/simulator/lidar_config.cpp"
    "${PROJECT_SOURCE_DIR}/apps/simulator/model_assembler.cpp"
    "${PROJECT_SOURCE_DIR}/apps/simulator/simulator_config.cpp"
)
target_include_directories(
    sharpa_mujoco_lidar_model_attachment_test
    PRIVATE "${PROJECT_SOURCE_DIR}/apps/simulator"
)
target_link_libraries(
    sharpa_mujoco_lidar_model_attachment_test
    PRIVATE
        SharpaMujoco::Core
        SharpaMujoco::LidarPlugin
)
add_test(
    NAME sharpa_mujoco_lidar_model_attachment
    COMMAND
        sharpa_mujoco_lidar_model_attachment_test
        --config "${PROJECT_SOURCE_DIR}/tests/data/lidar_enabled.conf"
)

add_test(
    NAME sharpa_mujoco_lidar_test_scene_attachment
    COMMAND
        sharpa_mujoco_lidar_model_attachment_test
        --config "${PROJECT_SOURCE_DIR}/tests/data/lidar_enabled.conf"
        --model "${CMAKE_CURRENT_SOURCE_DIR}/lidar_test_scene.xml"
)

add_executable(
    sharpa_mujoco_lidar_config_validation_test
    lidar_config_validation_test.cpp
    "${PROJECT_SOURCE_DIR}/apps/simulator/lidar_config.cpp"
)
target_include_directories(
    sharpa_mujoco_lidar_config_validation_test
    PRIVATE "${PROJECT_SOURCE_DIR}/apps/simulator"
)
target_compile_definitions(
    sharpa_mujoco_lidar_config_validation_test
    PRIVATE
        SHARPA_MUJOCO_FTX_SENSOR_MODEL="${PROJECT_SOURCE_DIR}/mjcf/sensors/lidar/hesai_ftx140_frame.xml"
        SHARPA_MUJOCO_FTX_CALIBRATION="${PROJECT_SOURCE_DIR}/resources/lidar/calibration/FTX140_angle_calib_file.csv"
)
target_link_libraries(
    sharpa_mujoco_lidar_config_validation_test
    PRIVATE SharpaMujoco::LidarPlugin
)
add_test(
    NAME sharpa_mujoco_lidar_config_validation
    COMMAND sharpa_mujoco_lidar_config_validation_test
)

add_executable(
    sharpa_mujoco_lidar_raycast_regression_test
    raycast_regression_test.cpp
    "${PROJECT_SOURCE_DIR}/apps/simulator/lidar_config.cpp"
    "${PROJECT_SOURCE_DIR}/apps/simulator/model_assembler.cpp"
    "${PROJECT_SOURCE_DIR}/apps/simulator/simulator_config.cpp"
)
target_include_directories(
    sharpa_mujoco_lidar_raycast_regression_test
    PRIVATE "${PROJECT_SOURCE_DIR}/apps/simulator"
)
target_link_libraries(
    sharpa_mujoco_lidar_raycast_regression_test
    PRIVATE
        SharpaMujoco::Core
        SharpaMujoco::DeviceCommon
        SharpaMujoco::LidarPlugin
)
add_test(
    NAME sharpa_mujoco_lidar_raycast_regression
    COMMAND
        sharpa_mujoco_lidar_raycast_regression_test
        --config "${PROJECT_SOURCE_DIR}/tests/data/lidar_enabled.conf"
        --model "${CMAKE_CURRENT_SOURCE_DIR}/lidar_test_scene.xml"
)
set_tests_properties(
    sharpa_mujoco_lidar_raycast_regression
    PROPERTIES TIMEOUT 30
)

# Interactive black-box UDP receiver. Do not register it with CTest because it
# requires a display, a separately running simulator, and user input.
if(TARGET SharpaMujoco::Viewer)
    find_package(glfw3 3.3 REQUIRED)
    find_package(OpenGL REQUIRED)
    add_executable(
        sharpa_mujoco_lidar_udp_visualization_test
        udp_visualization_test.cpp
        "${PROJECT_SOURCE_DIR}/apps/simulator/lidar_config.cpp"
        "${PROJECT_SOURCE_DIR}/apps/simulator/simulator_config.cpp"
    )
    target_include_directories(
        sharpa_mujoco_lidar_udp_visualization_test
        PRIVATE "${PROJECT_SOURCE_DIR}/apps/simulator"
    )
    target_link_libraries(
        sharpa_mujoco_lidar_udp_visualization_test
        PRIVATE
            SharpaMujoco::Core
            SharpaMujoco::DeviceCommon
            SharpaMujoco::LidarPlugin
            glfw
            OpenGL::GL
    )

    add_executable(
        sharpa_mujoco_lidar_world_visualization_test
        world_visualization_test.cpp
        "${PROJECT_SOURCE_DIR}/apps/simulator/lidar_config.cpp"
        "${PROJECT_SOURCE_DIR}/apps/simulator/simulator_config.cpp"
    )
    target_include_directories(
        sharpa_mujoco_lidar_world_visualization_test
        PRIVATE "${PROJECT_SOURCE_DIR}/apps/simulator"
    )
    target_link_libraries(
        sharpa_mujoco_lidar_world_visualization_test
        PRIVATE
            SharpaMujoco::Core
            SharpaMujoco::DeviceCommon
            SharpaMujoco::LidarPlugin
            glfw
            OpenGL::GL
    )
endif()
