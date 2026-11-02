"""Load Wave hand endpoints for the unified monitor."""

from __future__ import annotations

import configparser
import ipaddress
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Tuple


SIDES = ("left", "right")


@dataclass(frozen=True)
class HandEndpointConfig:
    side: str
    device_ip: str
    sdk_ip: str
    command_udp_port: int
    sdk_joint_port: int
    feedback_source_port: int
    serial_number: str


@dataclass(frozen=True)
class HandMonitorConfig:
    source_path: Path
    endpoints: Tuple[HandEndpointConfig, ...]


def _ip(section: configparser.SectionProxy, key: str, default: str) -> str:
    value = section.get(key, fallback=default).strip()
    try:
        parsed = ipaddress.ip_address(value)
    except ValueError as exc:
        raise ValueError(f"{section.name}.{key} is not a valid IP address: {value}") from exc
    if parsed.version != 4:
        raise ValueError(f"{section.name}.{key} must be an IPv4 address")
    return value


def _port(section: configparser.SectionProxy, key: str, default: int) -> int:
    value = section.getint(key, fallback=default)
    if not 1 <= value <= 65535:
        raise ValueError(f"{section.name}.{key} must be in range 1..65535")
    return value


def load_hand_config(path: Path, sides: Iterable[str] = SIDES) -> HandMonitorConfig:
    """Read and validate the requested left/right Wave hand endpoints."""

    selected = tuple(dict.fromkeys(sides))
    if not selected or any(side not in SIDES for side in selected):
        raise ValueError("sides must contain left and/or right")

    path = path.expanduser().resolve()
    parser = configparser.ConfigParser(interpolation=None, inline_comment_prefixes=("#",))
    try:
        raw_config = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ValueError(f"cannot read MuJoCo configuration: {path}") from exc
    try:
        parser.read_string("[__root__]\n" + raw_config, source=str(path))
    except configparser.Error as exc:
        raise ValueError(f"cannot parse MuJoCo configuration {path}: {exc}") from exc

    endpoints = []
    for side in selected:
        section_name = f"devices.hand_{side}"
        if section_name not in parser:
            raise ValueError(f"{path} does not contain [{section_name}]")
        section = parser[section_name]
        if not section.getboolean("enabled", fallback=True):
            raise ValueError(f"{section_name} is disabled")
        driver = section.get("driver", fallback="wave_sdk_loopback").strip().lower()
        if driver != "wave_sdk_loopback":
            raise ValueError(
                f"{section_name}.driver must be 'wave_sdk_loopback', got {driver!r}"
            )
        left = side == "left"
        serial_number = section.get(
            "serial_number", fallback="SIM-WAVE-L-0001" if left else "SIM-WAVE-R-0001"
        ).strip()
        if not serial_number:
            raise ValueError(f"{section_name}.serial_number must not be empty")
        endpoints.append(
            HandEndpointConfig(
                side=side,
                device_ip=_ip(section, "device_ip", "127.0.50.10" if left else "127.0.50.20"),
                sdk_ip=_ip(section, "sdk_ip", "127.0.0.1"),
                command_udp_port=_port(section, "command_udp_port", 50020 if left else 50030),
                sdk_joint_port=_port(section, "sdk_joint_port", 50000 if left else 50010),
                feedback_source_port=_port(section, "feedback_source_port", 10000),
                serial_number=serial_number,
            )
        )

    endpoint_keys = {
        (endpoint.device_ip, endpoint.command_udp_port) for endpoint in endpoints
    }
    if len(endpoint_keys) != len(endpoints):
        raise ValueError("requested hand command endpoints must be unique")
    return HandMonitorConfig(source_path=path, endpoints=tuple(endpoints))
