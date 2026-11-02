# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

set(
    MUJOCO_ROOT
    "$ENV{MUJOCO_ROOT}"
    CACHE PATH
    "Root directory of an official MuJoCo installation"
)

set(_SHARPA_MUJOCO_CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/../sharpa_mujoco.conf")
if(NOT MUJOCO_ROOT AND EXISTS "${_SHARPA_MUJOCO_CONFIG_FILE}")
    file(
        STRINGS
        "${_SHARPA_MUJOCO_CONFIG_FILE}"
        _SHARPA_MUJOCO_ROOT_ENTRIES
        REGEX "^[ \t]*mujoco_root[ \t]*="
        LIMIT_COUNT 1
    )
    if(_SHARPA_MUJOCO_ROOT_ENTRIES)
        list(GET _SHARPA_MUJOCO_ROOT_ENTRIES 0 _SHARPA_MUJOCO_CONFIGURED_ROOT)
        string(REGEX REPLACE "^[^=]*=[ \t]*" "" _SHARPA_MUJOCO_CONFIGURED_ROOT "${_SHARPA_MUJOCO_CONFIGURED_ROOT}")
        string(REGEX REPLACE "^\"(.*)\"$" "\\1" _SHARPA_MUJOCO_CONFIGURED_ROOT "${_SHARPA_MUJOCO_CONFIGURED_ROOT}")
        set(
            MUJOCO_ROOT
            "${_SHARPA_MUJOCO_CONFIGURED_ROOT}"
            CACHE PATH
            "Root directory of an official MuJoCo installation"
            FORCE
        )
    endif()
endif()

if(MUJOCO_ROOT)
    # MUJOCO_ROOT is an SDK selected explicitly by the caller.  In a cross
    # build it normally lives next to the source tree rather than inside the
    # target sysroot, so do not re-root this path through CMAKE_SYSROOT.
    find_path(
        MuJoCo_INCLUDE_DIR
        NAMES mujoco/mujoco.h
        PATHS "${MUJOCO_ROOT}/include"
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_library(
        MuJoCo_LIBRARY
        NAMES mujoco
        PATHS "${MUJOCO_ROOT}/lib" "${MUJOCO_ROOT}/bin"
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_file(
        MuJoCo_RUNTIME_LIBRARY
        NAMES
            libmujoco.so
            libmujoco.so.3
            libmujoco.so.3.8
            libmujoco.so.3.8.1
        PATHS "${MUJOCO_ROOT}/lib" "${MUJOCO_ROOT}/bin"
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )
else()
    find_path(MuJoCo_INCLUDE_DIR NAMES mujoco/mujoco.h)
    find_library(MuJoCo_LIBRARY NAMES mujoco)
    find_file(
        MuJoCo_RUNTIME_LIBRARY
        NAMES
            libmujoco.so
            libmujoco.so.3
            libmujoco.so.3.8
            libmujoco.so.3.8.1
    )
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    MuJoCo
    REQUIRED_VARS MuJoCo_INCLUDE_DIR MuJoCo_LIBRARY
    REASON_FAILURE_MESSAGE
        "Install the complete MuJoCo Linux release and set mujoco_root in sharpa_mujoco.conf or export MUJOCO_ROOT to its extracted directory. Then rerun scripts/build.sh. To discard stale CMake paths, use scripts/build.sh --clean with a verified build directory, or select a new build directory with -B. For direct CMake builds, configure in a new build directory."
)

if(MuJoCo_FOUND AND NOT TARGET MuJoCo::mujoco)
    add_library(MuJoCo::mujoco UNKNOWN IMPORTED)
    set_target_properties(
        MuJoCo::mujoco
        PROPERTIES
            IMPORTED_LOCATION "${MuJoCo_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${MuJoCo_INCLUDE_DIR}"
    )
endif()

mark_as_advanced(
    MuJoCo_INCLUDE_DIR
    MuJoCo_LIBRARY
    MuJoCo_RUNTIME_LIBRARY
)
