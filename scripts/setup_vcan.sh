#!/usr/bin/env bash
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

if (($# == 0)); then
    interfaces=(vcan0 vcan1)
else
    interfaces=("$@")
fi
for interface in "${interfaces[@]}"; do
    if [[ ! "${interface}" =~ ^vcan[[:alnum:]_.-]*$ || ${#interface} -gt 15 ]]; then
        echo "Refusing to create non-vcan interface: ${interface}" >&2
        exit 2
    fi
done

if [[ ${EUID} -ne 0 ]]; then
    echo "Run as root: sudo bash $0 ${interfaces[*]}" >&2
    exit 1
fi

# Inspect every target before mutating any interface. A vcan-like name is not
# sufficient proof that an existing device is virtual CAN.
for interface in "${interfaces[@]}"; do
    if ip link show dev "${interface}" >/dev/null 2>&1 &&
       [[ -z "$(ip -details link show dev "${interface}" type vcan)" ]]; then
        echo "Refusing to modify an existing non-vcan device: ${interface}" >&2
        exit 1
    fi
done
modprobe vcan
for interface in "${interfaces[@]}"; do
    if ! ip link show dev "${interface}" >/dev/null 2>&1; then
        # Another launcher may have created the interface concurrently.
        ip link add dev "${interface}" type vcan ||
            [[ -n "$(ip -details link show dev "${interface}" type vcan)" ]]
    fi
    ip link set dev "${interface}" up
    ip link set dev "${interface}" txqueuelen 1000
done

for interface in "${interfaces[@]}"; do
    ip -details link show "${interface}"
done
