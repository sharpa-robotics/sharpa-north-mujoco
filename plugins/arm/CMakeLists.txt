# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

add_library(sharpa_mujoco_arm_plugin STATIC)
add_library(SharpaMujoco::ArmPlugin ALIAS sharpa_mujoco_arm_plugin)
set_target_properties(sharpa_mujoco_arm_plugin PROPERTIES EXPORT_NAME ArmPlugin)

target_sources(
    sharpa_mujoco_arm_plugin
    PRIVATE
        src/arm_plugin.cpp
        src/arm_transmission.cpp
        src/arm_udp_codec.cpp
        src/cia402.cpp
)
target_compile_features(sharpa_mujoco_arm_plugin PUBLIC cxx_std_17)
target_include_directories(
    sharpa_mujoco_arm_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(
    sharpa_mujoco_arm_plugin
    PUBLIC SharpaMujoco::Core SharpaMujoco::DeviceCommon
)
target_compile_options(sharpa_mujoco_arm_plugin PRIVATE -Wall -Wextra -Wpedantic)

install(
    TARGETS sharpa_mujoco_arm_plugin
    EXPORT SharpaMujocoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
