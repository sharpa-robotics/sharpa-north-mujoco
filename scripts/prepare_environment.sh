#!/usr/bin/env bash
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
interactive=false
config_path=""

usage() {
    echo "Usage: bash $0 [--check|--interactive] --config /path/to/sharpa_mujoco.conf"
    echo "Default: read-only check. --interactive offers vcan setup with confirmation on a terminal."
    echo "Exit codes: 0 ready/not required; 1 environment not ready; 2 invalid arguments/configuration."
}

while (($#)); do
    case "$1" in
        --check) interactive=false; shift ;;
        --interactive) interactive=true; shift ;;
        --config) (($# >= 2)) || { usage >&2; exit 2; }; config_path="$2"; shift 2 ;;
        --help|-h) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
done
[[ -f "$config_path" ]] || { echo "Simulator configuration is missing: $config_path" >&2; exit 2; }

checker="${SHARPA_MUJOCO_ENVIRONMENT_CHECKER:-}"
if [[ -z "$checker" ]]; then
    for candidate in \
        "$script_dir/../bin/sharpa_mujoco_environment" \
        "$script_dir/../../../bin/sharpa_mujoco_environment"; do
        if [[ -x "$candidate" ]]; then
            checker="$candidate"
            break
        fi
    done
fi
if [[ ! -x "$checker" ]]; then
    echo "Native environment checker is missing: sharpa_mujoco_environment" >&2
    echo "Rebuild or reinstall sharpa_mujoco before launching." >&2
    exit 2
fi

set +e
required_vcan="$("$checker" --config "$config_path" --list-setup-vcan)"
check_status=$?
set -e
case "$check_status" in
    0) exit 0 ;;
    1) exit 1 ;;
    2) exit 2 ;;
    10) ;;
    *) echo "Environment checker failed with exit code $check_status." >&2; exit 2 ;;
esac

mapfile -t virtual_interfaces <<< "$required_vcan"
((${#virtual_interfaces[@]} > 0)) || {
    echo "Environment checker requested vcan setup without naming an interface." >&2
    exit 2
}
for interface in "${virtual_interfaces[@]}"; do
    [[ "$interface" =~ ^vcan[[:alnum:]_.-]{0,11}$ ]] || {
        echo "Environment checker returned an unsafe vcan name: $interface" >&2
        exit 2
    }
done

setup_script="$script_dir/setup_vcan.sh"
printf -v prepare_command 'sudo bash %q' "$setup_script"
for interface in "${virtual_interfaces[@]}"; do
    printf -v prepare_command '%s %q' "$prepare_command" "$interface"
done
echo "Configured virtual CAN interfaces are missing or DOWN." >&2
echo "Run in a terminal: $prepare_command" >&2
echo "Then restart simulation. Interfaces normally need preparation after reboot." >&2
if [[ "$interactive" != true || ! -t 0 || ! -t 2 ]]; then
    exit 1
fi

read -r -p "Prepare virtual CAN interfaces now (sudo may request your password)? [y/N] " answer || exit 1
[[ "$answer" =~ ^([yY]|[yY][eE][sS])$ ]] || exit 1
if ((EUID == 0)); then
    bash "$setup_script" "${virtual_interfaces[@]}"
else
    sudo bash "$setup_script" "${virtual_interfaces[@]}"
fi
export SHARPA_MUJOCO_ENVIRONMENT_CHECKER="$checker"
exec bash "$0" --check --config "$config_path"
