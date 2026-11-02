# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

add_library(sharpa_mujoco_msgpack11 STATIC
    ${PROJECT_SOURCE_DIR}/third_party/msgpack11/msgpack11.cpp
)
set_target_properties(
    sharpa_mujoco_msgpack11
    PROPERTIES EXPORT_NAME Msgpack11
)
target_include_directories(
    sharpa_mujoco_msgpack11
    SYSTEM PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/msgpack11>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/sharpa_mujoco/third_party/msgpack11>
)
target_compile_features(sharpa_mujoco_msgpack11 PUBLIC cxx_std_17)
target_compile_options(sharpa_mujoco_msgpack11 PRIVATE -w)
install(
    TARGETS sharpa_mujoco_msgpack11
    EXPORT SharpaMujocoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(
    FILES ${PROJECT_SOURCE_DIR}/third_party/msgpack11/msgpack11.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/sharpa_mujoco/third_party/msgpack11
)
install(
    FILES ${PROJECT_SOURCE_DIR}/third_party/msgpack11/LICENSE.txt
    DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/sharpa_mujoco/msgpack11
)

add_library(sharpa_mujoco_tactile_plugin STATIC)
add_library(SharpaMujoco::TactilePlugin ALIAS sharpa_mujoco_tactile_plugin)
set_target_properties(
    sharpa_mujoco_tactile_plugin
    PROPERTIES EXPORT_NAME TactilePlugin
)
target_sources(
    sharpa_mujoco_tactile_plugin
    PRIVATE
        src/tactile_plugin.cpp
        src/tactile_protocol.cpp
)
target_compile_features(sharpa_mujoco_tactile_plugin PUBLIC cxx_std_17)
target_include_directories(
    sharpa_mujoco_tactile_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(
    sharpa_mujoco_tactile_plugin
    PUBLIC
        SharpaMujoco::Core
        SharpaMujoco::DeviceCommon
        sharpa_mujoco_msgpack11
)
target_compile_options(
    sharpa_mujoco_tactile_plugin
    PRIVATE -Wall -Wextra -Wpedantic
)
install(
    TARGETS sharpa_mujoco_tactile_plugin
    EXPORT SharpaMujocoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
