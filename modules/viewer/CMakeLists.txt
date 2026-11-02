# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "Sharpa MuJoCo Viewer currently supports Linux only")
endif()

set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT}/simulate")
set(MUJOCO_VERSION_HEADER "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
if(NOT EXISTS "${MUJOCO_VERSION_HEADER}")
    message(FATAL_ERROR "MuJoCo header is missing: ${MUJOCO_VERSION_HEADER}")
endif()
file(
    STRINGS
    "${MUJOCO_VERSION_HEADER}"
    MUJOCO_HEADER_VERSION_LINE
    REGEX "^#define[ \t]+mjVERSION_HEADER[ \t]+[0-9]+"
    LIMIT_COUNT 1
)
if(NOT MUJOCO_HEADER_VERSION_LINE MATCHES "3008001$")
    message(
        FATAL_ERROR
        "Sharpa MuJoCo Viewer requires MuJoCo 3.8.1; found '${MUJOCO_HEADER_VERSION_LINE}'"
    )
endif()

set(
    MUJOCO_SIMULATE_SOURCES
    "${MUJOCO_SIMULATE_DIR}/simulate.cc"
    "${MUJOCO_SIMULATE_DIR}/glfw_adapter.cc"
    "${MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc"
    "${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc"
)
foreach(source_file IN LISTS MUJOCO_SIMULATE_SOURCES)
    if(NOT EXISTS "${source_file}")
        message(
            FATAL_ERROR
            "MuJoCo 3.8.1 simulate source is missing: ${source_file}. "
            "Set MUJOCO_ROOT to the extracted MuJoCo 3.8.1 directory."
        )
    endif()
endforeach()

find_package(glfw3 3.3 REQUIRED)

add_library(sharpa_mujoco_viewer STATIC)
add_library(SharpaMujoco::Viewer ALIAS sharpa_mujoco_viewer)
set_target_properties(sharpa_mujoco_viewer PROPERTIES EXPORT_NAME Viewer)

target_sources(
    sharpa_mujoco_viewer
    PRIVATE
        src/viewer.cpp
        ${MUJOCO_SIMULATE_SOURCES}
        "${PROJECT_SOURCE_DIR}/third_party/lodepng/lodepng.cpp"
)
target_compile_features(sharpa_mujoco_viewer PUBLIC cxx_std_20)
target_compile_definitions(
    sharpa_mujoco_viewer
    PRIVATE GLFW_INCLUDE_NONE
)
target_include_directories(
    sharpa_mujoco_viewer
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        "${MUJOCO_SIMULATE_DIR}"
        "${PROJECT_SOURCE_DIR}/third_party/lodepng"
)
target_link_libraries(
    sharpa_mujoco_viewer
    PUBLIC
        SharpaMujoco::Core
        glfw
    PRIVATE
        Threads::Threads
)

target_compile_options(
    sharpa_mujoco_viewer
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wno-missing-field-initializers
        -Wno-unused-parameter
)

install(
    TARGETS sharpa_mujoco_viewer
    EXPORT SharpaMujocoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
