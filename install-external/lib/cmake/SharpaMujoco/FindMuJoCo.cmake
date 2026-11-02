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

find_path(
    MuJoCo_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    HINTS "${MUJOCO_ROOT}"
    PATH_SUFFIXES include
)

find_library(
    MuJoCo_LIBRARY
    NAMES mujoco
    HINTS "${MUJOCO_ROOT}"
    PATH_SUFFIXES lib bin
)

find_file(
    MuJoCo_RUNTIME_LIBRARY
    NAMES
        libmujoco.so
        libmujoco.so.3
        libmujoco.so.3.8
        libmujoco.so.3.8.1
    HINTS "${MUJOCO_ROOT}"
    PATH_SUFFIXES lib bin
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    MuJoCo
    REQUIRED_VARS MuJoCo_INCLUDE_DIR MuJoCo_LIBRARY
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
