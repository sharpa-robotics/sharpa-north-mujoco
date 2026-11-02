# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

add_library(sharpa_mujoco_lidar_plugin STATIC)
add_library(SharpaMujoco::LidarPlugin ALIAS sharpa_mujoco_lidar_plugin)
set_target_properties(
    sharpa_mujoco_lidar_plugin
    PROPERTIES EXPORT_NAME LidarPlugin
)

target_sources(
    sharpa_mujoco_lidar_plugin
    PRIVATE
        src/lidar_plugin.cpp
        src/lidar_protocol.cpp
        src/scan_pattern.cpp
)
target_compile_features(sharpa_mujoco_lidar_plugin PUBLIC cxx_std_17)
target_include_directories(
    sharpa_mujoco_lidar_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(
    sharpa_mujoco_lidar_plugin
    PUBLIC
        SharpaMujoco::Core
        SharpaMujoco::DeviceCommon
)

if(MSVC)
    target_compile_options(sharpa_mujoco_lidar_plugin PRIVATE /W4 /permissive-)
else()
    target_compile_options(sharpa_mujoco_lidar_plugin PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(
    TARGETS sharpa_mujoco_lidar_plugin
    EXPORT SharpaMujocoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
