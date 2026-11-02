#!/usr/bin/env bash
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

CONFIG_PATH="${SHARPA_MUJOCO_CONFIG:-${PROJECT_ROOT}/sharpa_mujoco.conf}"
GRACE_SECONDS=2
DRY_RUN=false
FORCE_PORTS=false

usage() {
    cat <<'EOF'
Usage: ./scripts/cleanup_ports.sh [options]

Stop orphaned Sharpa MuJoCo processes and verify that simulator-side ports are
released.

Options:
  -c, --config PATH    Configuration to inspect (default: sharpa_mujoco.conf)
      --grace SECONDS  Seconds to wait after SIGTERM (default: 2)
      --force-ports    Also stop non-MuJoCo owners of configured local ports
      --dry-run        Print what would be stopped without sending signals
  -h, --help           Show this help

Examples:
  ./scripts/cleanup_ports.sh
  ./scripts/cleanup_ports.sh --force-ports
  sudo ./scripts/cleanup_ports.sh --force-ports

By default, only executables named sharpa_mujoco_* are stopped. If a configured
port is owned by another program (for example Platform), it is reported but not
stopped unless --force-ports is supplied.
EOF
}

while (($# > 0)); do
    case "$1" in
        -c|--config)
            if (($# < 2)); then
                echo "error: $1 requires a path" >&2
                exit 2
            fi
            CONFIG_PATH="$2"
            [[ "${CONFIG_PATH}" = /* ]] || CONFIG_PATH="$(pwd)/${CONFIG_PATH}"
            shift 2
            ;;
        --grace)
            if (($# < 2)); then
                echo "error: --grace requires a value" >&2
                exit 2
            fi
            GRACE_SECONDS="$2"
            shift 2
            ;;
        --force-ports)
            FORCE_PORTS=true
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if ! [[ "${GRACE_SECONDS}" =~ ^[0-9]+$ ]]; then
    echo "error: --grace must be a non-negative integer" >&2
    exit 2
fi
if [[ ! -f "${CONFIG_PATH}" ]]; then
    echo "error: configuration does not exist: ${CONFIG_PATH}" >&2
    exit 2
fi

process_description() {
    local pid="$1"
    local executable command_line
    executable="$(readlink -f "/proc/${pid}/exe" 2>/dev/null || true)"
    command_line="$(tr '\0' ' ' 2>/dev/null < "/proc/${pid}/cmdline" || true)"
    if [[ -n "${executable}" ]]; then
        printf '%s' "${executable}"
    elif [[ -n "${command_line}" ]]; then
        printf '%s' "${command_line}"
    else
        printf '%s' "unknown"
    fi
}

is_safe_target_pid() {
    local pid="$1"
    [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
    ((pid > 1)) || return 1
    ((pid != $$ && pid != PPID)) || return 1
    return 0
}

declare -A mujoco_pid_set=()
for proc_path in /proc/[0-9]*; do
    pid="${proc_path##*/}"
    is_safe_target_pid "${pid}" || continue
    executable_name=""
    IFS= read -r executable_name 2>/dev/null < "${proc_path}/comm" || true
    # Linux truncates comm to 15 characters. The common prefix still uniquely
    # identifies every executable produced by this repository.
    if [[ "${executable_name}" == sharpa_mujoco* ]]; then
        mujoco_pid_set["${pid}"]=1
    fi
done

declare -a mujoco_pids=()
if ((${#mujoco_pid_set[@]} > 0)); then
    mapfile -t mujoco_pids < <(
        printf '%s\n' "${!mujoco_pid_set[@]}" | sort -n
    )
fi

terminate_pids() {
    local label="$1"
    shift
    local -a candidates=("$@")
    local -a alive=()
    local pid attempt

    if ((${#candidates[@]} == 0)); then
        return
    fi

    echo "${label}:"
    for pid in "${candidates[@]}"; do
        is_safe_target_pid "${pid}" || continue
        if kill -0 "${pid}" 2>/dev/null; then
            printf '  PID %-7s %s\n' "${pid}" "$(process_description "${pid}")"
            alive+=("${pid}")
        fi
    done
    if ((${#alive[@]} == 0)); then
        return
    fi
    if [[ "${DRY_RUN}" == true ]]; then
        echo "  dry-run: no signals sent"
        return
    fi

    for pid in "${alive[@]}"; do
        if ! kill -TERM "${pid}" 2>/dev/null; then
            echo "  warning: could not send SIGTERM to PID ${pid}" >&2
        fi
    done

    for ((attempt = 0; attempt < GRACE_SECONDS * 10; ++attempt)); do
        remaining=false
        for pid in "${alive[@]}"; do
            if kill -0 "${pid}" 2>/dev/null; then
                remaining=true
                break
            fi
        done
        [[ "${remaining}" == false ]] && break
        sleep 0.1
    done

    for pid in "${alive[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            echo "  PID ${pid} did not exit; sending SIGKILL"
            if ! kill -KILL "${pid}" 2>/dev/null; then
                echo "  warning: could not send SIGKILL to PID ${pid}" >&2
            fi
        fi
    done
}

if ((${#mujoco_pids[@]} > 0)); then
    terminate_pids "Sharpa MuJoCo processes" "${mujoco_pids[@]}"
else
    echo "No running sharpa_mujoco_* process found."
fi

declare -A socket_set=()
while IFS=: read -r protocol port; do
    [[ "${protocol}" == tcp || "${protocol}" == udp ]] || continue
    [[ "${port}" =~ ^[1-9][0-9]{0,4}$ ]] || continue
    ((port <= 65535)) || continue
    socket_set["${protocol}:${port}"]=1
done < <(
    awk -F= '
        /^[[:space:]]*($|#)/ { next }
        /^[[:space:]]*\[/ { next }
        {
            key=$1
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", key)
            if (key == "bind_port" || key == "tcp_port" ||
                key == "command_udp_port" || key == "config_port" ||
                key == "feedback_source_port" || key ~ /_bind_port$/) {
                value=substr($0, index($0, "=") + 1)
                sub(/[[:space:]]*#.*/, "", value)
                gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
                protocol=(key == "tcp_port" || key == "config_port") ? "tcp" : "udp"
                print protocol ":" value
            }
        }
    ' "${CONFIG_PATH}"
)

if ! command -v ss >/dev/null 2>&1; then
    echo "warning: ss is unavailable; configured ports cannot be verified." >&2
    echo "Install the iproute2 package to enable port verification." >&2
    exit 0
fi

declare -a owner_records=()
declare -A owner_pid_set=()
collect_port_owners() {
    local line protocol state recv_queue send_queue local_address remainder
    local port pid search record_key found_pid
    declare -A record_set=()
    owner_records=()
    owner_pid_set=()
    while IFS= read -r line; do
        read -r protocol state recv_queue send_queue local_address remainder <<< "${line}"
        [[ "${protocol}" == tcp || "${protocol}" == udp ]] || continue
        port="${local_address##*:}"
        [[ -n "${socket_set["${protocol}:${port}"]+present}" ]] || continue

        found_pid=false
        search="${line}"
        while [[ "${search}" =~ pid=([0-9]+) ]]; do
            pid="${BASH_REMATCH[1]}"
            search="${search#*pid=${pid}}"
            is_safe_target_pid "${pid}" || continue
            found_pid=true
            record_key="${protocol}:${port}:${pid}"
            [[ -n "${record_set["${record_key}"]+present}" ]] && continue
            record_set["${record_key}"]=1
            owner_records+=("${record_key}")
            owner_pid_set["${pid}"]=1
        done

        if [[ "${found_pid}" == false ]]; then
            record_key="${protocol}:${port}:?"
            if [[ -z "${record_set["${record_key}"]+present}" ]]; then
                record_set["${record_key}"]=1
                owner_records+=("${record_key}")
            fi
        fi
    done < <(ss -H -ltnup 2>/dev/null || true)
}

collect_port_owners
if ((${#owner_records[@]} > 0)); then
    echo "Configured local ports still in use:"
    unknown_owner=false
    for record in "${owner_records[@]}"; do
        IFS=: read -r protocol port pid <<< "${record}"
        if [[ "${pid}" == "?" ]]; then
            printf '  %-3s %-5s PID unavailable (try sudo)\n' \
                "${protocol^^}" "${port}"
            unknown_owner=true
        else
            printf '  %-3s %-5s PID %-7s %s\n' \
                "${protocol^^}" "${port}" "${pid}" "$(process_description "${pid}")"
        fi
    done

    if [[ "${FORCE_PORTS}" == true ]]; then
        declare -a owner_pids=()
        if ((${#owner_pid_set[@]} > 0)); then
            mapfile -t owner_pids < <(
                printf '%s\n' "${!owner_pid_set[@]}" | sort -n
            )
        fi
        terminate_pids "Forced configured-port owners" "${owner_pids[@]}"
        if [[ "${unknown_owner}" == true ]]; then
            echo "  Some owner PIDs are hidden; re-run with sudo to stop them."
        fi
        [[ "${DRY_RUN}" == true ]] || sleep 0.1
        collect_port_owners
    elif [[ "${DRY_RUN}" == true ]]; then
        echo "These ports remain occupied because dry-run did not stop any process."
    else
        echo "Re-run with --force-ports to stop these non-MuJoCo owners."
    fi
fi

if ((${#owner_records[@]} == 0)); then
    echo "Cleanup complete: all configured Sharpa MuJoCo local ports are free."
    exit 0
fi

if [[ "${DRY_RUN}" == true ]]; then
    echo "Dry run complete: no processes were stopped."
else
    echo "Cleanup incomplete: one or more configured ports are still occupied." >&2
fi
exit 1
