"""Load one arm's UDP transport configuration for the unified monitor."""

from __future__ import annotations

import configparser
import ipaddress
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple


AXIS_COUNT = 7
ENDPOINT_COUNT = 5
DEFAULT_MOTOR_INDICES = ((0,), (1,), (2,), (3, 4), (5, 6))


@dataclass(frozen=True)
class ArmEndpointConfig:
    index: int
    bind_ip: str
    bind_port: int
    peer_ip: str
    peer_port: int
    motor_indices: Tuple[int, ...]


@dataclass(frozen=True)
class ArmMonitorConfig:
    source_path: Path
    side: str
    endpoints: Tuple[ArmEndpointConfig, ...]


def _csv_ints(raw: str, key: str) -> Tuple[int, ...]:
    try:
        values = tuple(int(value.strip(), 0) for value in raw.split(",") if value.strip())
    except ValueError as exc:
        raise ValueError(f"{key} must be a comma-separated integer list") from exc
    if len(values) not in (1, 2):
        raise ValueError(f"{key} must contain one or two motor indices")
    return values


def _port(section: configparser.SectionProxy, key: str, default: int) -> int:
    value = section.getint(key, fallback=default)
    if value < 1 or value > 65535:
        raise ValueError(f"{section.name}.{key} must be in range 1..65535")
    return value


def _ip(section: configparser.SectionProxy, key: str, default: str) -> str:
    value = section.get(key, fallback=default).strip()
    try:
        parsed = ipaddress.ip_address(value)
    except ValueError as exc:
        raise ValueError(f"{section.name}.{key} is not a valid IP address: {value}") from exc
    if parsed.version != 4:
        raise ValueError(f"{section.name}.{key} must be an IPv4 address")
    return value


def load_arm_config(path: Path, side: str) -> ArmMonitorConfig:
    """Read one side's effective arm UDP settings from sharpa_mujoco.conf."""

    if side not in ("left", "right"):
        raise ValueError(f"invalid arm side: {side!r}")
    path = path.expanduser().resolve()
    parser = configparser.ConfigParser(
        interpolation=None, inline_comment_prefixes=("#",)
    )
    try:
        raw_config = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ValueError(f"cannot read MuJoCo configuration: {path}") from exc
    try:
        parser.read_string("[__root__]\n" + raw_config, source=str(path))
    except configparser.Error as exc:
        raise ValueError(f"cannot parse MuJoCo configuration {path}: {exc}") from exc

    section_name = f"devices.arm_{side}"
    if section_name not in parser:
        raise ValueError(f"{path} does not contain [{section_name}]")
    section = parser[section_name]
    if not section.getboolean("enabled", fallback=True):
        raise ValueError(f"{section_name} is disabled")
    driver = section.get("driver", fallback="udp_arm").strip().lower()
    if driver != "udp_arm":
        raise ValueError(
            f"{section_name}.driver must be 'udp_arm', got {driver!r}"
        )

    first_host = 101 if side == "left" else 106
    default_peer_port = 8080 if side == "left" else 8081
    endpoints = []
    covered = []
    for index in range(ENDPOINT_COUNT):
        prefix = f"endpoint{index}_"
        default_indices = DEFAULT_MOTOR_INDICES[index]
        motor_indices = _csv_ints(
            section.get(
                prefix + "motor_indices",
                fallback=",".join(map(str, default_indices)),
            ),
            f"{section_name}.{prefix}motor_indices",
        )
        if any(motor < 0 or motor >= AXIS_COUNT for motor in motor_indices):
            raise ValueError(
                f"{section_name}.{prefix}motor_indices values must be in range 0..6"
            )
        covered.extend(motor_indices)
        endpoints.append(
            ArmEndpointConfig(
                index=index,
                bind_ip=_ip(
                    section,
                    prefix + "bind_ip",
                    f"127.0.40.{first_host + index}",
                ),
                bind_port=_port(section, prefix + "bind_port", 21000),
                peer_ip=_ip(section, prefix + "peer_ip", "127.0.0.1"),
                peer_port=_port(
                    section, prefix + "peer_port", default_peer_port
                ),
                motor_indices=motor_indices,
            )
        )

    if sorted(covered) != list(range(AXIS_COUNT)):
        raise ValueError(
            f"{section_name} endpoint motor_indices must cover 0..6 exactly once"
        )
    return ArmMonitorConfig(
        source_path=path,
        side=side,
        endpoints=tuple(endpoints),
    )
