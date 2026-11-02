# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

project(
    SharpaMujoco
    VERSION 0.1.0
    DESCRIPTION "MuJoCo runtime and hardware-compatible device simulation"
    LANGUAGES CXX
)

# Static SDK components are also linked into the shared C ABI bridge.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "Sharpa MuJoCo supports Linux only")
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(SHARPA_MUJOCO_BUILD_TESTS "Build SDK tests" ON)
option(SHARPA_MUJOCO_BUILD_DEVICE_PLUGINS "Build device simulation plugins" ON)
option(SHARPA_MUJOCO_BUILD_SIMULATOR "Build the generic device simulation server" ON)
option(SHARPA_MUJOCO_BUILD_EMBEDDED_SIMULATOR "Embed device simulator support in the C API library" OFF)
option(SHARPA_MUJOCO_BUILD_VIEWER "Build the MuJoCo 3.8.1 Viewer on Linux" ON)
option(SHARPA_MUJOCO_PORTABLE_RUNTIME "Use package-relative simulator RPATH" OFF)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(PlatformRuntime)

find_package(Threads REQUIRED)
find_package(mujoco CONFIG QUIET)
if(NOT mujoco_FOUND)
    find_package(MuJoCo REQUIRED MODULE)
endif()

set(SHARPA_MUJOCO_MUJOCO_TARGET "")
if(TARGET mujoco::mujoco)
    set(SHARPA_MUJOCO_MUJOCO_TARGET mujoco::mujoco)
elseif(TARGET MuJoCo::mujoco)
    set(SHARPA_MUJOCO_MUJOCO_TARGET MuJoCo::mujoco)
else()
    message(FATAL_ERROR "MuJoCo package did not provide mujoco::mujoco or MuJoCo::mujoco")
endif()

set(_SHARPA_MUJOCO_RUNTIME_CANDIDATE "")
if(MuJoCo_RUNTIME_LIBRARY AND EXISTS "${MuJoCo_RUNTIME_LIBRARY}")
    set(_SHARPA_MUJOCO_RUNTIME_CANDIDATE "${MuJoCo_RUNTIME_LIBRARY}")
else()
    foreach(_SHARPA_MUJOCO_LOCATION_PROPERTY
            IMPORTED_LOCATION
            IMPORTED_LOCATION_RELEASE
            IMPORTED_LOCATION_RELWITHDEBINFO
            IMPORTED_LOCATION_DEBUG)
        get_target_property(
            _SHARPA_MUJOCO_IMPORTED_LOCATION
            ${SHARPA_MUJOCO_MUJOCO_TARGET}
            ${_SHARPA_MUJOCO_LOCATION_PROPERTY}
        )
        if(_SHARPA_MUJOCO_IMPORTED_LOCATION AND
           EXISTS "${_SHARPA_MUJOCO_IMPORTED_LOCATION}")
            set(
                _SHARPA_MUJOCO_RUNTIME_CANDIDATE
                "${_SHARPA_MUJOCO_IMPORTED_LOCATION}"
            )
            break()
        endif()
    endforeach()
endif()
if(NOT _SHARPA_MUJOCO_RUNTIME_CANDIDATE)
    message(FATAL_ERROR "Could not resolve the MuJoCo runtime library")
endif()
file(
    REAL_PATH "${_SHARPA_MUJOCO_RUNTIME_CANDIDATE}"
    SHARPA_MUJOCO_MUJOCO_RUNTIME_LIBRARY
)
get_filename_component(
    SHARPA_MUJOCO_MUJOCO_RUNTIME_NAME
    "${SHARPA_MUJOCO_MUJOCO_RUNTIME_LIBRARY}"
    NAME
)

add_library(avatar_mujoco_core)
add_library(SharpaMujoco::Core ALIAS avatar_mujoco_core)
set_target_properties(
    avatar_mujoco_core
    PROPERTIES
        EXPORT_NAME Core
        POSITION_INDEPENDENT_CODE ON
)

target_sources(
    avatar_mujoco_core
    PRIVATE
        src/device_plugin.cpp
        src/companion_process.cpp
        src/runtime.cpp
        src/scene_builder.cpp
)

target_compile_features(avatar_mujoco_core PUBLIC cxx_std_17)
target_compile_definitions(
    avatar_mujoco_core
    PRIVATE SHARPA_MUJOCO_CORE_EXPORTS
)

if(NOT BUILD_SHARED_LIBS)
    target_compile_definitions(avatar_mujoco_core PUBLIC SHARPA_MUJOCO_STATIC)
endif()

target_include_directories(
    avatar_mujoco_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
    avatar_mujoco_core
    PUBLIC
        ${SHARPA_MUJOCO_MUJOCO_TARGET}
        Threads::Threads
        ${CMAKE_DL_LIBS}
)

target_compile_options(avatar_mujoco_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(sharpa_mujoco_c SHARED src/c_api.cpp src/companion_c_api.cpp)
if(TARGET sharpa_mujoco_platform_runtime)
    add_dependencies(sharpa_mujoco_c sharpa_mujoco_platform_runtime)
endif()
set_target_properties(
    sharpa_mujoco_c
    PROPERTIES
        OUTPUT_NAME sharpa_mujoco_c
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES
        BUILD_WITH_INSTALL_RPATH YES
        INSTALL_RPATH "$ORIGIN"
)
target_compile_features(sharpa_mujoco_c PRIVATE cxx_std_17)
target_compile_definitions(
    sharpa_mujoco_c
    PRIVATE
        SHARPA_MUJOCO_C_EXPORTS
        SHARPA_MUJOCO_VERSION="${PROJECT_VERSION}"
        SHARPA_MUJOCO_HAS_EMBEDDED_SIMULATOR=$<BOOL:${SHARPA_MUJOCO_BUILD_EMBEDDED_SIMULATOR}>
)
target_include_directories(
    sharpa_mujoco_c
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(sharpa_mujoco_c PRIVATE SharpaMujoco::Core)
target_link_options(
    sharpa_mujoco_c
    PRIVATE
        "LINKER:--exclude-libs,ALL"
        "LINKER:--no-undefined"
)
target_compile_options(sharpa_mujoco_c PRIVATE -Wall -Wextra -Wpedantic)
add_custom_command(
    TARGET sharpa_mujoco_c
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different
        "${SHARPA_MUJOCO_MUJOCO_RUNTIME_LIBRARY}"
        "$<TARGET_FILE_DIR:sharpa_mujoco_c>/${SHARPA_MUJOCO_MUJOCO_RUNTIME_NAME}"
    COMMENT "Copying MuJoCo next to the C ABI bridge"
)

if(SHARPA_MUJOCO_BUILD_DEVICE_PLUGINS)
    add_subdirectory(plugins/common)
    add_subdirectory(plugins/torso)
    add_subdirectory(plugins/chassis)
    add_subdirectory(plugins/lidar)
    add_subdirectory(plugins/arm)
    add_subdirectory(plugins/wave_hand)
    add_subdirectory(plugins/tactile)
endif()

if(SHARPA_MUJOCO_BUILD_EMBEDDED_SIMULATOR)
    install(
        PROGRAMS scripts/setup_vcan.sh scripts/prepare_environment.sh
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/sharpa_mujoco/scripts
        COMPONENT SharpaMujocoCSDK
    )
    if(NOT SHARPA_MUJOCO_BUILD_DEVICE_PLUGINS)
        message(FATAL_ERROR "SHARPA_MUJOCO_BUILD_EMBEDDED_SIMULATOR requires SHARPA_MUJOCO_BUILD_DEVICE_PLUGINS=ON")
    endif()
    add_library(
        sharpa_mujoco_simulator_service STATIC
        src/simulator_service.cpp
        apps/simulator/main.cpp
        apps/simulator/simulator_environment.cpp
        apps/simulator/lidar_config.cpp
        apps/simulator/model_assembler.cpp
        apps/simulator/simulator_config.cpp
    )
    add_library(
        SharpaMujoco::SimulatorService
        ALIAS sharpa_mujoco_simulator_service
    )
    set_target_properties(
        sharpa_mujoco_simulator_service
        PROPERTIES POSITION_INDEPENDENT_CODE ON
    )
    target_compile_features(sharpa_mujoco_simulator_service PUBLIC cxx_std_17)
    target_compile_definitions(
        sharpa_mujoco_simulator_service
        PRIVATE
            SHARPA_MUJOCO_SERVICE_LIBRARY=1
            SHARPA_MUJOCO_HAS_VIEWER=0
    )
    target_include_directories(
        sharpa_mujoco_simulator_service
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/apps/simulator
    )
    target_link_libraries(
        sharpa_mujoco_simulator_service
        PUBLIC
            SharpaMujoco::Core
            SharpaMujoco::TorsoPlugin
            SharpaMujoco::ChassisPlugin
            SharpaMujoco::LidarPlugin
            SharpaMujoco::ArmPlugin
            SharpaMujoco::WaveHandPlugin
            SharpaMujoco::TactilePlugin
    )
    target_compile_options(
        sharpa_mujoco_simulator_service PRIVATE -Wall -Wextra -Wpedantic
    )
    target_link_libraries(
        sharpa_mujoco_c
        PRIVATE SharpaMujoco::SimulatorService ${CMAKE_DL_LIBS}
    )

endif()

if(SHARPA_MUJOCO_BUILD_DEVICE_PLUGINS)
    add_executable(
        sharpa_mujoco_environment
        apps/environment/main.cpp
        apps/simulator/simulator_config.cpp
        apps/simulator/simulator_environment.cpp
    )
    target_compile_features(sharpa_mujoco_environment PRIVATE cxx_std_17)
    target_include_directories(
        sharpa_mujoco_environment
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/apps/simulator
    )
    target_link_libraries(
        sharpa_mujoco_environment
        PRIVATE SharpaMujoco::DeviceCommon
    )
    target_compile_options(
        sharpa_mujoco_environment PRIVATE -Wall -Wextra -Wpedantic
    )
    install(
        TARGETS sharpa_mujoco_environment
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT SharpaMujocoCSDK
    )
endif()

set(SHARPA_MUJOCO_VIEWER_BUILT OFF)
if(SHARPA_MUJOCO_BUILD_VIEWER)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        add_subdirectory(modules/viewer)
        set(SHARPA_MUJOCO_VIEWER_BUILT ON)
    else()
        message(
            WARNING
            "SHARPA_MUJOCO_BUILD_VIEWER is ignored because the Viewer currently supports Linux only"
        )
    endif()
endif()

if(SHARPA_MUJOCO_BUILD_SIMULATOR)
    if(NOT SHARPA_MUJOCO_BUILD_DEVICE_PLUGINS)
        message(FATAL_ERROR "SHARPA_MUJOCO_BUILD_SIMULATOR requires SHARPA_MUJOCO_BUILD_DEVICE_PLUGINS=ON")
    endif()
    add_subdirectory(apps/simulator)
endif()

if(SHARPA_MUJOCO_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

install(
    TARGETS avatar_mujoco_core
    EXPORT SharpaMujocoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    TARGETS sharpa_mujoco_c
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT SharpaMujocoCSDK
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT SharpaMujocoCSDK
)

install(
    FILES "${SHARPA_MUJOCO_MUJOCO_RUNTIME_LIBRARY}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT SharpaMujocoCSDK
)

install(
    FILES include/sharpa_mujoco/c_api.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/sharpa_mujoco
    COMPONENT SharpaMujocoCSDK
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    DIRECTORY
        mjcf/sensors/
        resources/lidar/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/sharpa_mujoco
)

if(SHARPA_MUJOCO_BUILD_EMBEDDED_SIMULATOR)
    install(
        FILES sharpa_mujoco.conf
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/sharpa_mujoco
        COMPONENT SharpaMujocoCSDK
    )
    install(
        DIRECTORY mjcf/sensors
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/sharpa_mujoco/mjcf
        COMPONENT SharpaMujocoCSDK
    )
    install(
        DIRECTORY resources/lidar
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/sharpa_mujoco/resources
        COMPONENT SharpaMujocoCSDK
    )
endif()

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/SharpaMujocoConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/SharpaMujocoConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/SharpaMujocoConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SharpaMujoco"
)

install(
    EXPORT SharpaMujocoTargets
    FILE SharpaMujocoTargets.cmake
    NAMESPACE SharpaMujoco::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SharpaMujoco"
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/SharpaMujocoConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/SharpaMujocoConfigVersion.cmake"
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindMuJoCo.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SharpaMujoco"
)
