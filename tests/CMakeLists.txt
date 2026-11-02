# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

add_executable(sharpa_mujoco_companion_process_worker companion_process_worker.cpp)
add_executable(sharpa_mujoco_companion_process_test companion_process_test.cpp)
target_link_libraries(sharpa_mujoco_companion_process_test PRIVATE sharpa_mujoco_c Threads::Threads)
add_dependencies(sharpa_mujoco_companion_process_test sharpa_mujoco_companion_process_worker)
add_test(
    NAME sharpa_mujoco_companion_process
    COMMAND sharpa_mujoco_companion_process_test $<TARGET_FILE:sharpa_mujoco_companion_process_worker>)
find_package(Python3 3.10 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME sharpa_mujoco_build_script
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/build_script_test.py")
    add_test(NAME sharpa_mujoco_run_script
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/run_script_test.py")
    add_test(NAME sharpa_mujoco_prepare_environment
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/prepare_environment_test.py")
endif()
if(Python3_Interpreter_FOUND)
    add_test(NAME sharpa_mujoco_platform_archive
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/platform_runtime_test.py")
    add_test(NAME sharpa_mujoco_chassis_motion_driver
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/chassis_motion_driver_test.py")
endif()

add_executable(sharpa_mujoco_runtime_test runtime_test.cpp)
target_link_libraries(sharpa_mujoco_runtime_test PRIVATE SharpaMujoco::Core)
target_compile_definitions(
    sharpa_mujoco_runtime_test
    PRIVATE
        SHARPA_MUJOCO_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/data/minimal.xml"
        SHARPA_MUJOCO_RUNTIME_ROBOT_MODEL="${PROJECT_SOURCE_DIR}/mjcf/NTH-HA1-MJCF/NTH-HA1.xml"
)

add_test(NAME sharpa_mujoco_runtime COMMAND sharpa_mujoco_runtime_test)

add_executable(sharpa_mujoco_runtime_input_test runtime_input_test.cpp)
target_link_libraries(sharpa_mujoco_runtime_input_test PRIVATE SharpaMujoco::Core)
target_compile_definitions(sharpa_mujoco_runtime_input_test PRIVATE
    SHARPA_MUJOCO_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/data/minimal.xml")
add_test(NAME sharpa_mujoco_runtime_input COMMAND sharpa_mujoco_runtime_input_test)

add_executable(sharpa_mujoco_plugin_manager_test plugin_manager_test.cpp)
target_link_libraries(sharpa_mujoco_plugin_manager_test PRIVATE SharpaMujoco::Core)
add_test(NAME sharpa_mujoco_plugin_manager COMMAND sharpa_mujoco_plugin_manager_test)

add_executable(sharpa_mujoco_snapshot_reset_test snapshot_reset_test.cpp)
target_link_libraries(sharpa_mujoco_snapshot_reset_test PRIVATE SharpaMujoco::Core)
target_compile_definitions(
    sharpa_mujoco_snapshot_reset_test PRIVATE
    SHARPA_MUJOCO_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/data/minimal.xml"
)
add_test(NAME sharpa_mujoco_snapshot_reset COMMAND sharpa_mujoco_snapshot_reset_test)

add_executable(sharpa_mujoco_scene_builder_test scene_builder_test.cpp)
target_link_libraries(sharpa_mujoco_scene_builder_test PRIVATE SharpaMujoco::Core)
target_compile_definitions(
    sharpa_mujoco_scene_builder_test
    PRIVATE
        SHARPA_MUJOCO_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/data/minimal.xml"
        SHARPA_MUJOCO_SCENE_ROBOT_MODEL="${PROJECT_SOURCE_DIR}/mjcf/NTH-HA1-MJCF/NTH-HA1.xml"
)
add_test(NAME sharpa_mujoco_scene_builder COMMAND sharpa_mujoco_scene_builder_test)

add_executable(sharpa_mujoco_c_api_test c_api_test.cpp)
target_link_libraries(
    sharpa_mujoco_c_api_test
    PRIVATE sharpa_mujoco_c ${SHARPA_MUJOCO_MUJOCO_TARGET}
)
target_compile_definitions(
    sharpa_mujoco_c_api_test
    PRIVATE
        SHARPA_MUJOCO_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/data/minimal.xml"
)
add_test(NAME sharpa_mujoco_c_api COMMAND sharpa_mujoco_c_api_test)

if(TARGET SharpaMujoco::SimulatorService)
    add_executable(
        sharpa_mujoco_simulator_service_c_api_test
        simulator_service_c_api_test.cpp
    )
    target_link_libraries(
        sharpa_mujoco_simulator_service_c_api_test
        PRIVATE sharpa_mujoco_c ${SHARPA_MUJOCO_MUJOCO_TARGET}
    )
    target_compile_definitions(
        sharpa_mujoco_simulator_service_c_api_test
        PRIVATE
            SHARPA_MUJOCO_NTH_HA1_MODEL="${PROJECT_SOURCE_DIR}/mjcf/NTH-HA1-MJCF/NTH-HA1.xml"
            SHARPA_MUJOCO_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/data/minimal.xml"
            SHARPA_MUJOCO_SNAPSHOT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/data/embedded_snapshot.conf"
            SHARPA_MUJOCO_TORSO_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/data/embedded_torso.conf"
            SHARPA_MUJOCO_MISSING_CAN_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/data/embedded_missing_can.conf"
    )
    add_test(
        NAME sharpa_mujoco_simulator_service_c_api
        COMMAND sharpa_mujoco_simulator_service_c_api_test
    )
else()
    add_executable(
        sharpa_mujoco_simulator_disabled_c_api_test
        simulator_disabled_c_api_test.cpp
    )
    target_link_libraries(sharpa_mujoco_simulator_disabled_c_api_test PRIVATE sharpa_mujoco_c)
    target_compile_definitions(
        sharpa_mujoco_simulator_disabled_c_api_test
        PRIVATE SHARPA_MUJOCO_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/data/minimal.xml"
    )
    add_test(
        NAME sharpa_mujoco_simulator_disabled_c_api
        COMMAND sharpa_mujoco_simulator_disabled_c_api_test
    )
endif()

if(TARGET SharpaMujoco::DeviceCommon)
    add_executable(sharpa_mujoco_delayed_queue_test delayed_queue_test.cpp)
    target_link_libraries(sharpa_mujoco_delayed_queue_test PRIVATE SharpaMujoco::DeviceCommon)
    add_test(NAME sharpa_mujoco_delayed_queue COMMAND sharpa_mujoco_delayed_queue_test)
    add_executable(sharpa_mujoco_periodic_task_test periodic_task_test.cpp)
    target_link_libraries(sharpa_mujoco_periodic_task_test PRIVATE SharpaMujoco::DeviceCommon)
    add_test(NAME sharpa_mujoco_periodic_task COMMAND sharpa_mujoco_periodic_task_test)
    set_tests_properties(sharpa_mujoco_periodic_task PROPERTIES TIMEOUT 5)
    add_executable(
        sharpa_mujoco_startup_brake_test
        startup_brake_test.cpp
    )
    target_link_libraries(
        sharpa_mujoco_startup_brake_test
        PRIVATE SharpaMujoco::Core SharpaMujoco::DeviceCommon
    )
    target_compile_definitions(
        sharpa_mujoco_startup_brake_test
        PRIVATE
            SHARPA_MUJOCO_NTH_HA1_MODEL="${PROJECT_SOURCE_DIR}/mjcf/NTH-HA1-MJCF/NTH-HA1.xml"
    )
    add_test(
        NAME sharpa_mujoco_startup_brake
        COMMAND sharpa_mujoco_startup_brake_test
    )
endif()

if(TARGET SharpaMujoco::TorsoPlugin)
    add_executable(
        sharpa_mujoco_mcu_protocol_test
        mcu_udp_protocol_test.cpp
    )
    target_link_libraries(
        sharpa_mujoco_mcu_protocol_test
        PRIVATE
            SharpaMujoco::DeviceCommon
            SharpaMujoco::TorsoPlugin
    )
    add_test(
        NAME sharpa_mujoco_mcu_protocol
        COMMAND sharpa_mujoco_mcu_protocol_test
    )

    add_executable(
        sharpa_mujoco_device_plugin_test
        device_plugin_test.cpp
    )
    target_link_libraries(
        sharpa_mujoco_device_plugin_test
        PRIVATE
            SharpaMujoco::Core
            SharpaMujoco::DeviceCommon
            SharpaMujoco::TorsoPlugin
    )
    target_compile_definitions(
        sharpa_mujoco_device_plugin_test
        PRIVATE
            SHARPA_MUJOCO_POC2_MODEL="${PROJECT_SOURCE_DIR}/mjcf/NTH-HA1-MJCF/NTH-HA1.xml"
    )
    add_test(
        NAME sharpa_mujoco_device_plugin
        COMMAND sharpa_mujoco_device_plugin_test
    )

endif()

if(TARGET SharpaMujoco::ChassisPlugin)
    add_executable(sharpa_mujoco_steer_position_profile_test steer_position_profile_test.cpp)
    target_link_libraries(sharpa_mujoco_steer_position_profile_test PRIVATE SharpaMujoco::ChassisPlugin)
    add_test(NAME sharpa_mujoco_steer_position_profile COMMAND sharpa_mujoco_steer_position_profile_test)
    add_executable(sharpa_mujoco_wheel_velocity_pi_test wheel_velocity_pi_test.cpp)
    target_link_libraries(sharpa_mujoco_wheel_velocity_pi_test PRIVATE SharpaMujoco::ChassisPlugin)
    add_test(NAME sharpa_mujoco_wheel_velocity_pi COMMAND sharpa_mujoco_wheel_velocity_pi_test)

    add_executable(
        sharpa_mujoco_chassis_protocol_test
        chassis_protocol_test.cpp
    )
    target_link_libraries(
        sharpa_mujoco_chassis_protocol_test
        PRIVATE
            SharpaMujoco::DeviceCommon
            SharpaMujoco::ChassisPlugin
    )
    add_test(
        NAME sharpa_mujoco_chassis_protocol
        COMMAND sharpa_mujoco_chassis_protocol_test
    )

    add_executable(
        sharpa_mujoco_chassis_udp_test
        chassis_udp_test.cpp
    )
    target_link_libraries(
        sharpa_mujoco_chassis_udp_test
        PRIVATE
            SharpaMujoco::Core
            SharpaMujoco::DeviceCommon
            SharpaMujoco::ChassisPlugin
    )
    target_compile_definitions(
        sharpa_mujoco_chassis_udp_test
        PRIVATE
            SHARPA_MUJOCO_NTH_HA1_MODEL="${PROJECT_SOURCE_DIR}/mjcf/NTH-HA1-MJCF/NTH-HA1.xml"
            SHARPA_MUJOCO_NTH_HA1_SCENE="${PROJECT_SOURCE_DIR}/mjcf/NTH-HA1-MJCF/NTH-HA1-scene.xml"
    )
    add_test(
        NAME sharpa_mujoco_chassis_udp
        COMMAND sharpa_mujoco_chassis_udp_test
    )
endif()

if(TARGET SharpaMujoco::LidarPlugin)
    add_subdirectory(lidar)
endif()

if(TARGET SharpaMujoco::ArmPlugin)
    add_executable(sharpa_mujoco_arm_udp_test arm_udp_test.cpp)
    target_link_libraries(
        sharpa_mujoco_arm_udp_test
        PRIVATE
            SharpaMujoco::Core
            SharpaMujoco::DeviceCommon
            SharpaMujoco::ArmPlugin
    )
    target_compile_definitions(
        sharpa_mujoco_arm_udp_test
        PRIVATE
            SHARPA_MUJOCO_POC2_MODEL="${PROJECT_SOURCE_DIR}/mjcf/NTH-HA1-MJCF/NTH-HA1.xml"
    )
    add_test(NAME sharpa_mujoco_arm_udp COMMAND sharpa_mujoco_arm_udp_test)
endif()

if(TARGET SharpaMujoco::WaveHandPlugin)
    add_executable(
        sharpa_mujoco_wave_hand_controller_test
        wave_hand_controller_test.cpp
    )
    target_link_libraries(
        sharpa_mujoco_wave_hand_controller_test
        PRIVATE SharpaMujoco::WaveHandPlugin
    )
    target_include_directories(
        sharpa_mujoco_wave_hand_controller_test
        PRIVATE ${PROJECT_SOURCE_DIR}/plugins/wave_hand/src
    )
    add_test(
        NAME sharpa_mujoco_wave_hand_controller
        COMMAND sharpa_mujoco_wave_hand_controller_test
    )

    add_executable(
        sharpa_mujoco_wave_hand_test
        wave_hand_test.cpp
    )
    target_link_libraries(
        sharpa_mujoco_wave_hand_test
        PRIVATE
            SharpaMujoco::Core
            SharpaMujoco::DeviceCommon
            SharpaMujoco::WaveHandPlugin
    )
    target_compile_definitions(
        sharpa_mujoco_wave_hand_test
        PRIVATE
            SHARPA_MUJOCO_POC2_MODEL="${PROJECT_SOURCE_DIR}/mjcf/NTH-HA1-MJCF/NTH-HA1.xml"
    )
    add_test(NAME sharpa_mujoco_wave_hand COMMAND sharpa_mujoco_wave_hand_test)
endif()

if(TARGET SharpaMujoco::TactilePlugin)
    add_executable(
        sharpa_mujoco_tactile_test
        tactile_test.cpp
        ${PROJECT_SOURCE_DIR}/apps/simulator/model_assembler.cpp
    )
    target_include_directories(
        sharpa_mujoco_tactile_test
        PRIVATE ${PROJECT_SOURCE_DIR}/apps/simulator
    )
    target_link_libraries(
        sharpa_mujoco_tactile_test
        PRIVATE
            SharpaMujoco::Core
            SharpaMujoco::DeviceCommon
            SharpaMujoco::LidarPlugin
            SharpaMujoco::TactilePlugin
    )
    target_compile_definitions(
        sharpa_mujoco_tactile_test
        PRIVATE
            SHARPA_MUJOCO_TACTILE_MODEL="${PROJECT_SOURCE_DIR}/mjcf/NTH-HA1-MJCF/NTH-HA1.xml"
            SHARPA_MUJOCO_TACTILE_FRAME="${PROJECT_SOURCE_DIR}/mjcf/sensors/tactile/wave_fingertip_frame.xml"
            SHARPA_MUJOCO_TACTILE_CONTACT_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/tactile_contact_test.xml"
    )
    add_test(NAME sharpa_mujoco_tactile COMMAND sharpa_mujoco_tactile_test)
endif()

foreach(delay_test sharpa_mujoco_device_plugin sharpa_mujoco_arm_udp)
    if(TEST ${delay_test})
        set_tests_properties(${delay_test} PROPERTIES TIMEOUT 20)
    endif()
endforeach()
