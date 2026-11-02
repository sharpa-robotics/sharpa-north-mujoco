"""Configuration for the current Kinco UDP chassis simulation profile."""

from __future__ import annotations

import configparser
import ipaddress
import math
from dataclasses import dataclass
from pathlib import Path


# ChassisPluginConfig motor_can_interfaces; A/B/C/D = LF/RF/RR/LR.
MOTOR_IDS = tuple(range(1, 9))
CAN_INTERFACES = (1, 0, 0, 1, 1, 0, 0, 1)


@dataclass(frozen=True)
class ChassisMonitorConfig:
    source_path: Path
    bind_ip: str = "127.0.0.1"
    bind_port: int = 45620
    peer_ip: str = "127.0.0.1"
    peer_port: int = 45220
    encoder_resolution: int = 65536
    sample_timeout: float = 0.1
    name: str = "chassis"


def load_chassis_config(path: Path) -> ChassisMonitorConfig:
    path = path.expanduser().resolve()
    parser = configparser.ConfigParser(interpolation=None, inline_comment_prefixes=("#",))
    try:
        parser.read_string("[__root__]\n" + path.read_text(encoding="utf-8"), source=str(path))
    except (OSError, configparser.Error) as exc:
        raise ValueError(f"cannot read MuJoCo configuration {path}: {exc}") from exc
    if "devices.chassis" not in parser:
        raise ValueError(f"{path} does not contain [devices.chassis]")
    section = parser["devices.chassis"]
    if not section.getboolean("enabled", fallback=True):
        raise ValueError("devices.chassis is disabled")
    if section.get("driver", "").strip() != "udp_mcu_kinco":
        raise ValueError("chassis monitor supports only driver=udp_mcu_kinco")

    values = {"source_path": path}
    for key, default in (("bind_ip", "127.0.0.1"), ("peer_ip", "127.0.0.1")):
        value = section.get("kinco_" + key, default).strip()
        if ipaddress.ip_address(value).version != 4:
            raise ValueError(f"devices.chassis.kinco_{key} must be IPv4")
        values[key] = value
    for key, default in (("bind_port", 45620), ("peer_port", 45220)):
        value = section.getint("kinco_" + key, fallback=default)
        if not 1 <= value <= 65535:
            raise ValueError(f"devices.chassis.kinco_{key} must be in range 1..65535")
        values[key] = value
    resolution = section.getint("kinco_encoder_resolution", fallback=65536)
    if not 0 < resolution <= 0xFFFFFFFF:
        raise ValueError("kinco_encoder_resolution must be a positive uint32")
    values["encoder_resolution"] = resolution
    timeout = section.getfloat("command_timeout_s", fallback=0.1)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("devices.chassis.command_timeout_s must be finite and positive")
    values["sample_timeout"] = timeout
    return ChassisMonitorConfig(**values)
