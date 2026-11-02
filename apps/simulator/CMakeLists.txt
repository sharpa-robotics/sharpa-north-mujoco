# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

add_executable(
    sharpa_mujoco_simulator
    main.cpp
    ../../src/simulator_service.cpp
    lidar_config.cpp
    model_assembler.cpp
    simulator_config.cpp
    simulator_environment.cpp
)
target_compile_features(sharpa_mujoco_simulator PRIVATE cxx_std_17)
if(TARGET sharpa_mujoco_platform_runtime)
    add_dependencies(sharpa_mujoco_simulator sharpa_mujoco_platform_runtime)
endif()
target_include_directories(
    sharpa_mujoco_simulator
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(
    sharpa_mujoco_simulator
    PRIVATE
        SharpaMujoco::Core
        SharpaMujoco::TorsoPlugin
        SharpaMujoco::ChassisPlugin
        SharpaMujoco::LidarPlugin
        SharpaMujoco::ArmPlugin
        SharpaMujoco::WaveHandPlugin
        SharpaMujoco::TactilePlugin
)
if(TARGET SharpaMujoco::Viewer)
    target_link_libraries(
        sharpa_mujoco_simulator
        PRIVATE SharpaMujoco::Viewer
    )
    target_compile_definitions(
        sharpa_mujoco_simulator
        PRIVATE SHARPA_MUJOCO_HAS_VIEWER=1
    )
else()
    target_compile_definitions(
        sharpa_mujoco_simulator
        PRIVATE SHARPA_MUJOCO_HAS_VIEWER=0
    )
endif()

target_compile_options(sharpa_mujoco_simulator PRIVATE -Wall -Wextra -Wpedantic)

if(SHARPA_MUJOCO_PORTABLE_RUNTIME)
    set_target_properties(
        sharpa_mujoco_simulator
        PROPERTIES
            BUILD_WITH_INSTALL_RPATH YES
            BUILD_RPATH "$ORIGIN/../lib"
            INSTALL_RPATH "$ORIGIN/../lib"
    )
elseif(MuJoCo_RUNTIME_LIBRARY)
    get_filename_component(MUJOCO_RUNTIME_DIRECTORY "${MuJoCo_RUNTIME_LIBRARY}" DIRECTORY)
    set_target_properties(
        sharpa_mujoco_simulator
        PROPERTIES
            BUILD_RPATH "${MUJOCO_RUNTIME_DIRECTORY}"
            INSTALL_RPATH "${MUJOCO_RUNTIME_DIRECTORY}"
    )
endif()

install(
    TARGETS sharpa_mujoco_simulator
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
