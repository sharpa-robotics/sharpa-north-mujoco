# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

add_library(sharpa_mujoco_device_common STATIC)
add_library(SharpaMujoco::DeviceCommon ALIAS sharpa_mujoco_device_common)
set_target_properties(
    sharpa_mujoco_device_common
    PROPERTIES EXPORT_NAME DeviceCommon
)

target_sources(
    sharpa_mujoco_device_common
    PRIVATE
        src/mcu_udp_codec.cpp
        src/socketcan_transport.cpp
        src/startup_brake.cpp
        src/udp_transport.cpp
)

target_compile_features(sharpa_mujoco_device_common PUBLIC cxx_std_17)
target_include_directories(
    sharpa_mujoco_device_common
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
    sharpa_mujoco_device_common
    PUBLIC SharpaMujoco::Core Threads::Threads
)

target_compile_options(sharpa_mujoco_device_common PRIVATE -Wall -Wextra -Wpedantic)

install(
    TARGETS sharpa_mujoco_device_common
    EXPORT SharpaMujocoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
