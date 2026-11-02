#!/usr/bin/env bash
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIMULATOR="$(realpath -m -- "${SHARPA_MUJOCO_SIMULATOR:-${PACKAGE_ROOT}/bin/sharpa_mujoco_simulator}")"
CONFIG_PATH="$(realpath -m -- "${SHARPA_MUJOCO_CONFIG:-${PACKAGE_ROOT}/sharpa_mujoco.conf}")"

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "error: this Sharpa MuJoCo runtime package supports Linux only." >&2
    exit 1
fi

if [[ ! -x "${SIMULATOR}" ]]; then
    echo "error: simulator is missing or not executable: ${SIMULATOR}" >&2
    exit 1
fi

args=("$@")
has_config_argument=false
for ((index = 0; index < ${#args[@]}; ++index)); do
    if [[ "${args[index]}" == "--config" ]]; then
        if ((index + 1 >= ${#args[@]})) || [[ -z "${args[index + 1]}" || "${args[index + 1]}" == --* ]]; then
            echo "error: --config requires a non-empty path" >&2
            exit 2
        fi
        CONFIG_PATH="$(realpath -m -- "${args[index + 1]}")"
        args[index + 1]="$CONFIG_PATH"
        has_config_argument=true
        index=$((index + 1))
    fi
done

if [[ ! -f "${CONFIG_PATH}" ]]; then
    echo "error: configuration does not exist: ${CONFIG_PATH}" >&2
    exit 1
fi

if [[ "${has_config_argument}" == false ]]; then
    args=(--config "${CONFIG_PATH}" "${args[@]}")
fi

export LD_LIBRARY_PATH="${PACKAGE_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

missing_libraries="$(
    ldd "${SIMULATOR}" 2>/dev/null |
        awk '/not found/ { print $1 }' |
        sort -u || true
)"
if [[ -n "${missing_libraries}" ]]; then
    echo "error: required Linux runtime libraries are missing:" >&2
    while IFS= read -r library; do
        echo "  ${library}" >&2
    done <<< "${missing_libraries}"
    echo "Use this package on a compatible Linux desktop distribution." >&2
    exit 1
fi

bash "${PACKAGE_ROOT}/scripts/prepare_environment.sh" --interactive --config "${CONFIG_PATH}"

cd -- "${PACKAGE_ROOT}"
exec "${SIMULATOR}" "${args[@]}"
