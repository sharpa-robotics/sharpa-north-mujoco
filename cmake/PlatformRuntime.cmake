# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _platform_processor)
if(_platform_processor MATCHES "^(x86_64|amd64)$")
    set(_platform_arch "x86_64")
    set(_platform_target "GEN_X86")
elseif(_platform_processor MATCHES "^(aarch64|arm64)$")
    set(_platform_arch "aarch64")
    set(_platform_target "GEN_THOR")
else()
    set(_platform_arch "")
    set(_platform_target "")
endif()
set(_platform_bundle "${PROJECT_SOURCE_DIR}/third_party/sharpa_platform/linux-${_platform_arch}")
set(_platform_default OFF)
if(_platform_arch AND EXISTS "${_platform_bundle}/sharpa-platform.tar.gz")
    set(_platform_default ON)
endif()
option(SHARPA_MUJOCO_STAGE_PLATFORM "Stage the optional prebuilt Platform runtime" ${_platform_default})
set(SHARPA_MUJOCO_PLATFORM_STAGE "${PROJECT_BINARY_DIR}/sharpa_mujoco/companions/sharpa_platform")
if(SHARPA_MUJOCO_STAGE_PLATFORM)
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR NOT _platform_arch)
        message(FATAL_ERROR
            "The bundled Platform runtime supports Linux x86_64 and Linux aarch64 (Thor); "
            "the configured target is ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}")
    endif()
    if(NOT EXISTS "${_platform_bundle}/sharpa-platform.tar.gz")
        message(FATAL_ERROR
            "Platform bundle for ${_platform_target} is missing: ${_platform_bundle}. "
            "Provide the official Platform archive and manifest, or build without Platform.")
    endif()
    find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
    # Fail before any targets compile, including builds launched by the build
    # and packaging scripts. Recheck after a checkout replaces the LFS object.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${_platform_bundle}/sharpa-platform.tar.gz"
        "${_platform_bundle}/sharpa-platform.manifest.json"
        "${PROJECT_SOURCE_DIR}/scripts/platform_runtime.py")
    file(GLOB_RECURSE _platform_overlay_files CONFIGURE_DEPENDS
        "${PROJECT_SOURCE_DIR}/resources/platform_runtime/*")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        ${_platform_overlay_files})
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/scripts/platform_runtime.py"
            check --bundle "${_platform_bundle}"
        RESULT_VARIABLE _platform_check_result
        ERROR_VARIABLE _platform_check_error)
    if(NOT _platform_check_result EQUAL 0)
        message(FATAL_ERROR "${_platform_check_error}")
    endif()
    # Run the cheap stamp check even for an otherwise up-to-date binary. The
    # importer validates the checksum before deciding to reuse expanded files.
    add_custom_target(sharpa_mujoco_platform_runtime
        COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/scripts/platform_runtime.py"
            stage --bundle "${_platform_bundle}" --destination "${SHARPA_MUJOCO_PLATFORM_STAGE}"
        VERBATIM
        COMMENT "Validating and staging the optional Platform runtime")
    install(CODE "
        execute_process(COMMAND \"${Python3_EXECUTABLE}\" \"${PROJECT_SOURCE_DIR}/scripts/platform_runtime.py\"
            stage --bundle \"${_platform_bundle}\" --destination \"${SHARPA_MUJOCO_PLATFORM_STAGE}\"
            RESULT_VARIABLE _platform_result)
        if(NOT _platform_result EQUAL 0)
            message(FATAL_ERROR \"Cannot install an invalid Platform runtime\")
        endif()
    " COMPONENT SharpaMujocoCSDK)
    install(DIRECTORY "${SHARPA_MUJOCO_PLATFORM_STAGE}"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/sharpa_mujoco/companions"
        USE_SOURCE_PERMISSIONS COMPONENT SharpaMujocoCSDK)
else()
    # A build directory may previously have staged the optional companion.
    # Keep the configured OFF variant honest even when callers reuse a custom
    # build directory.
    file(REMOVE_RECURSE "${SHARPA_MUJOCO_PLATFORM_STAGE}")
endif()
