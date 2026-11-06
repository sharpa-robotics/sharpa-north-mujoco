#!/usr/bin/env bash
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build-linux"
BUILD_DIR_EXPLICIT=false
PLATFORM_MODE=""
config_path="${SHARPA_MUJOCO_CONFIG:-${PROJECT_ROOT}/sharpa_mujoco.conf}"
args=()
has_config_argument=false
while (($#)); do
    case "$1" in
        -B|--build-dir|--config)
            if (($# < 2)) || [[ -z "$2" || "$2" == --* ]]; then
                echo "error: $1 requires a non-empty path" >&2
                exit 2
            fi
            if [[ "$1" == --config ]]; then
                config_path="$(realpath -m -- "$2")"
                args+=(--config "$config_path")
                has_config_argument=true
            else
                BUILD_DIR="$2"
                [[ "$BUILD_DIR" = /* ]] || BUILD_DIR="${PROJECT_ROOT}/${BUILD_DIR}"
                BUILD_DIR_EXPLICIT=true
            fi
            shift 2
            ;;
        --with-platform|--without-platform)
            if [[ -n "$PLATFORM_MODE" && "$PLATFORM_MODE" != "$1" ]]; then
                echo "error: --with-platform conflicts with --without-platform" >&2
                exit 2
            fi
            PLATFORM_MODE="$1"
            args+=("$1")
            shift
            ;;
        *) args+=("$1"); shift ;;
    esac
done
if [[ "$BUILD_DIR_EXPLICIT" == false && "$PLATFORM_MODE" == --without-platform ]]; then
    BUILD_DIR="${PROJECT_ROOT}/build-linux-no-platform"
fi
BUILD_DIR="$(realpath -m -- "$BUILD_DIR")"
SIMULATOR="$(realpath -m -- "${SHARPA_MUJOCO_SIMULATOR:-${BUILD_DIR}/apps/simulator/sharpa_mujoco_simulator}")"
config_path="$(realpath -m -- "$config_path")"
if [[ "$has_config_argument" == false ]]; then
    args=(--config "$config_path" "${args[@]}")
fi

if [[ ! -x "${SIMULATOR}" ]]; then
    echo "Simulator not found or not executable: ${SIMULATOR}" >&2
    if [[ "$BUILD_DIR_EXPLICIT" == false ]]; then
        echo "Build it first with: ${PROJECT_ROOT}/scripts/build.sh ${PLATFORM_MODE}" >&2
    else
        echo "Build the selected directory with: ${PROJECT_ROOT}/scripts/build.sh -B '${BUILD_DIR}' ${PLATFORM_MODE}" >&2
    fi
    exit 1
fi

checker="${SHARPA_MUJOCO_ENVIRONMENT_CHECKER:-}"
if [[ -z "$checker" ]]; then
    if [[ -n "${SHARPA_MUJOCO_SIMULATOR:-}" ]]; then
        # An overridden executable must use its own build/package's checker.
        simulator_dir="$(dirname -- "$SIMULATOR")"
        checker="${simulator_dir}/sharpa_mujoco_environment"
        [[ -x "$checker" ]] || checker="${simulator_dir}/../../sharpa_mujoco_environment"
    else
        checker="${BUILD_DIR}/sharpa_mujoco_environment"
    fi
fi
SHARPA_MUJOCO_ENVIRONMENT_CHECKER="$(realpath -m -- "$checker")" \
    bash "${SCRIPT_DIR}/prepare_environment.sh" --interactive --config "${config_path}"

cd "${PROJECT_ROOT}"
exec "${SIMULATOR}" "${args[@]}"
