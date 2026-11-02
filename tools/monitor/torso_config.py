"""Load torso transport configuration for the unified monitor."""

from __future__ import annotations

import configparser
import ipaddress
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple


AXIS_COUNT = 8
DEFAULT_MOTOR_IDS = (1, 2, 4, 3, 5, 6, 7, 8)
DEFAULT_MOTOR_SIGNS = (-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0)


@dataclass(frozen=True)
class EndpointConfig:
    name: str
    bind_ip: str
    bind_port: int
    peer_ip: str
    peer_port: int
    can_interface_id: int
    motor_ids: Tuple[int, ...]
    socketcan_interface: str


@dataclass(frozen=True)
class TorsoMonitorConfig:
    source_path: Path
    driver: str
    enabled: bool
    motor_ids: Tuple[int, ...]
    motor_signs: Tuple[float, ...]
    endpoints: Tuple[EndpointConfig, EndpointConfig]

    @property
    def transport_label(self) -> str:
        return "CAN" if self.driver == "can" else "UDP"


def _csv_ints(raw: str, key: str) -> Tuple[int, ...]:
    try:
        values = tuple(int(value.strip(), 0) for value in raw.split(",") if value.strip())
    except ValueError as exc:
        raise ValueError(f"{key} must be a comma-separated integer list") from exc
    if not values:
        raise ValueError(f"{key} must not be empty")
    return values


def _csv_floats(raw: str, key: str) -> Tuple[float, ...]:
    try:
        values = tuple(float(value.strip()) for value in raw.split(",") if value.strip())
    except ValueError as exc:
        raise ValueError(f"{key} must be a comma-separated number list") from exc
    if not values:
        raise ValueError(f"{key} must not be empty")
    return values


def _port(section: configparser.SectionProxy, key: str, default: int) -> int:
    value = section.getint(key, fallback=default)
    if value < 1 or value > 65535:
        raise ValueError(f"devices.torso.{key} must be in range 1..65535")
    return value


def _ip(section: configparser.SectionProxy, key: str, default: str) -> str:
    value = section.get(key, fallback=default).strip()
    try:
        parsed = ipaddress.ip_address(value)
    except ValueError as exc:
        raise ValueError(f"devices.torso.{key} is not a valid IP address: {value}") from exc
    if parsed.version != 4:
        raise ValueError(f"devices.torso.{key} must be an IPv4 address")
    return value


def load_torso_config(path: Path) -> TorsoMonitorConfig:
    """Read the effective torso transport settings from sharpa_mujoco.conf."""

    path = path.expanduser().resolve()
    parser = configparser.ConfigParser(interpolation=None, inline_comment_prefixes=("#",))
    try:
        raw_config = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ValueError(f"cannot read MuJoCo configuration: {path}")
    # sharpa_mujoco.conf intentionally permits SDK/build keys before its first
    # section. The C++ simulator ignores them; give ConfigParser a synthetic
    # section so this reader has the same behavior.
    try:
        parser.read_string("[__root__]\n" + raw_config, source=str(path))
    except configparser.Error as exc:
        raise ValueError(f"cannot parse MuJoCo configuration {path}: {exc}") from exc
    if "devices.torso" not in parser:
        raise ValueError(f"{path} does not contain [devices.torso]")

    section = parser["devices.torso"]
    enabled = section.getboolean("enabled", fallback=True)
    if not enabled:
        raise ValueError("devices.torso is disabled")
    driver = section.get("driver", fallback="udp").strip().lower()
    if driver not in ("can", "udp"):
        raise ValueError(
            "devices.torso.driver must be 'can' or 'udp', "
            f"got {driver!r}"
        )

    motor_ids = _csv_ints(
        section.get("motor_ids", fallback=",".join(map(str, DEFAULT_MOTOR_IDS))),
        "devices.torso.motor_ids",
    )
    if len(motor_ids) != AXIS_COUNT or len(set(motor_ids)) != AXIS_COUNT:
        raise ValueError("devices.torso.motor_ids must contain eight unique IDs")
    if any(value < 1 or value > 127 for value in motor_ids):
        raise ValueError("devices.torso.motor_ids values must be in range 1..127")

    motor_signs = _csv_floats(
        section.get("motor_signs", fallback=",".join(map(str, DEFAULT_MOTOR_SIGNS))),
        "devices.torso.motor_signs",
    )
    if len(motor_signs) != AXIS_COUNT:
        raise ValueError("devices.torso.motor_signs must contain exactly eight values")
    if any(value not in (-1.0, 1.0) for value in motor_signs):
        raise ValueError("devices.torso.motor_signs values must be -1 or 1")

    defaults = (
        ("lower", "127.0.0.1", 45520, "127.0.0.1", 45120, 0, (1, 2, 3, 4, 5), "vcan0"),
        ("upper", "127.0.0.1", 45521, "127.0.0.1", 45121, 0, (6, 7, 8), "vcan1"),
    )
    endpoints = []
    for name, bind_ip, bind_port, peer_ip, peer_port, can_if, endpoint_ids, can_name in defaults:
        prefix = f"{name}_"
        configured_ids = _csv_ints(
            section.get(prefix + "motor_ids", fallback=",".join(map(str, endpoint_ids))),
            f"devices.torso.{prefix}motor_ids",
        )
        if len(set(configured_ids)) != len(configured_ids):
            raise ValueError(f"devices.torso.{prefix}motor_ids contains duplicates")
        if any(value not in motor_ids for value in configured_ids):
            raise ValueError(
                f"devices.torso.{prefix}motor_ids contains an ID absent from motor_ids"
            )
        configured_can_if = section.getint(prefix + "can_if", fallback=can_if)
        if configured_can_if < 0 or configured_can_if > 255:
            raise ValueError(f"devices.torso.{prefix}can_if must be in range 0..255")
        socketcan_interface = section.get(
            prefix + "can_interface", fallback=can_name
        ).strip()
        if not socketcan_interface:
            raise ValueError(f"devices.torso.{prefix}can_interface must not be empty")
        endpoints.append(
            EndpointConfig(
                name=name,
                bind_ip=_ip(section, prefix + "bind_ip", bind_ip),
                bind_port=_port(section, prefix + "bind_port", bind_port),
                peer_ip=_ip(section, prefix + "peer_ip", peer_ip),
                peer_port=_port(section, prefix + "peer_port", peer_port),
                can_interface_id=configured_can_if,
                motor_ids=configured_ids,
                socketcan_interface=socketcan_interface,
            )
        )

    covered = set(endpoints[0].motor_ids) | set(endpoints[1].motor_ids)
    if covered != set(motor_ids):
        raise ValueError("lower_motor_ids and upper_motor_ids must cover all torso motor_ids")
    if set(endpoints[0].motor_ids) & set(endpoints[1].motor_ids):
        raise ValueError("lower_motor_ids and upper_motor_ids must not overlap")

    return TorsoMonitorConfig(
        source_path=path,
        driver=driver,
        enabled=enabled,
        motor_ids=motor_ids,
        motor_signs=motor_signs,
        endpoints=(endpoints[0], endpoints[1]),
    )
