# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

add_library(sharpa_mujoco_wave_hand_plugin STATIC)
add_library(SharpaMujoco::WaveHandPlugin ALIAS sharpa_mujoco_wave_hand_plugin)
set_target_properties(
    sharpa_mujoco_wave_hand_plugin
    PROPERTIES EXPORT_NAME WaveHandPlugin
)

target_sources(
    sharpa_mujoco_wave_hand_plugin
    PRIVATE
        src/ltc_protocol.cpp
        src/wave_hand_controller.cpp
        src/wave_hand_plugin.cpp
        src/wave_protocol.cpp
)
target_compile_features(sharpa_mujoco_wave_hand_plugin PUBLIC cxx_std_17)
target_include_directories(
    sharpa_mujoco_wave_hand_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(
    sharpa_mujoco_wave_hand_plugin
    PUBLIC SharpaMujoco::Core SharpaMujoco::DeviceCommon
)
target_compile_options(
    sharpa_mujoco_wave_hand_plugin
    PRIVATE -Wall -Wextra -Wpedantic
)

install(
    TARGETS sharpa_mujoco_wave_hand_plugin
    EXPORT SharpaMujocoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
