# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

add_library(sharpa_mujoco_chassis_plugin STATIC)
add_library(SharpaMujoco::ChassisPlugin ALIAS sharpa_mujoco_chassis_plugin)
set_target_properties(
    sharpa_mujoco_chassis_plugin
    PROPERTIES EXPORT_NAME ChassisPlugin
)

target_sources(
    sharpa_mujoco_chassis_plugin
    PRIVATE
        src/chassis_plugin.cpp
        src/jiechang_motor_emulator.cpp
        src/jiechang_stretch_emulator.cpp
        src/kinco_motor_emulator.cpp
        src/m42p_motor_emulator.cpp
)

target_compile_features(sharpa_mujoco_chassis_plugin PUBLIC cxx_std_17)
target_include_directories(
    sharpa_mujoco_chassis_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(
    sharpa_mujoco_chassis_plugin
    PUBLIC
        SharpaMujoco::Core
        SharpaMujoco::DeviceCommon
)

target_compile_options(sharpa_mujoco_chassis_plugin PRIVATE -Wall -Wextra -Wpedantic)

install(
    TARGETS sharpa_mujoco_chassis_plugin
    EXPORT SharpaMujocoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
