"""Passive Zenoh and AF_PACKET arm data sources."""

from __future__ import annotations

import fcntl
import socket
import struct
import threading
from collections import defaultdict
from typing import Dict, Iterable, List, Optional, Tuple

from arm_config import ArmEndpointConfig, ArmMonitorConfig
from arm_history import MonitorHistory
from arm_protocol import DecodeError, decode_control, decode_feedback, extract_ipv4_udp
from zenoh_connection import ZENOH_ENDPOINT, client_config


ETH_P_IP = 0x0800
PACKET_OUTGOING = 4


class ZenohCapture:
    def __init__(
        self,
        zenoh_module,
        proto_module,
        side: str,
        history: MonitorHistory,
    ):
        self._zenoh = zenoh_module
        self._proto = proto_module
        self._side = side
        self._history = history
        self._session = None
        self._subscribers = []

    def start(self) -> None:
        config = client_config(self._zenoh)
        self._session = self._zenoh.open(config)
        self._subscribers = [
            self._session.declare_subscriber(
                f"action/{self._side}_arm", self._on_action
            ),
            self._session.declare_subscriber(
                f"state/{self._side}_arm", self._on_state
            ),
            self._session.declare_subscriber(
                f"system/heartbeat/{self._side}_arm", self._on_heartbeat
            ),
        ]

    def _on_action(self, sample) -> None:
        try:
            message = self._proto.ArmCommand()
            message.ParseFromString(bytes(sample.payload))
            self._history.add_zenoh("zenoh_action", message.joints.position)
        except Exception as exc:
            self._history.record_error(
                "zenoh_decode",
                f"{self._side} arm Zenoh action decode failed: {exc}",
            )

    def _on_state(self, sample) -> None:
        try:
            message = self._proto.ArmState()
            message.ParseFromString(bytes(sample.payload))
            self._history.add_zenoh("zenoh_state", message.joints.position)
        except Exception as exc:
            self._history.record_error(
                "zenoh_decode",
                f"{self._side} arm Zenoh state decode failed: {exc}",
            )

    def _on_heartbeat(self, sample) -> None:
        try:
            message = self._proto.Heartbeat()
            message.ParseFromString(bytes(sample.payload))
            self._history.set_control_mode(
                int(message.control_mode), source="heartbeat"
            )
        except Exception as exc:
            self._history.record_error(
                "zenoh_decode",
                f"{self._side} arm heartbeat decode failed: {exc}",
            )

    def stop(self) -> None:
        for subscriber in self._subscribers:
            try:
                subscriber.undeclare()
            except Exception:
                pass
        self._subscribers.clear()
        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass
            self._session = None


class WireCapture:
    def __init__(self, config: ArmMonitorConfig, history: MonitorHistory):
        self._config = config
        self._history = history
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._sockets: List[socket.socket] = []
        self.interfaces: Tuple[str, ...] = ()

    def start(self) -> None:
        grouped: Dict[str, List[ArmEndpointConfig]] = defaultdict(list)
        for endpoint in self._config.endpoints:
            grouped[_interface_for_endpoint(endpoint)].append(endpoint)
        opened = []
        try:
            for interface, endpoints in grouped.items():
                capture_socket = socket.socket(
                    socket.AF_PACKET, socket.SOCK_DGRAM, socket.htons(ETH_P_IP)
                )
                capture_socket.bind((interface, 0))
                capture_socket.settimeout(0.25)
                opened.append(capture_socket)
                self._threads.append(
                    threading.Thread(
                        target=self._udp_loop,
                        args=(capture_socket, interface, tuple(endpoints)),
                        name=f"arm-udp-{self._config.side}-{interface}",
                        daemon=True,
                    )
                )
            self._sockets.extend(opened)
            self.interfaces = tuple(grouped)
            for thread in self._threads:
                thread.start()
        except PermissionError as exc:
            for capture_socket in opened:
                capture_socket.close()
            raise RuntimeError(
                "arm UDP monitoring requires root or CAP_NET_RAW"
            ) from exc
        except Exception:
            for capture_socket in opened:
                capture_socket.close()
            raise

    def stop(self) -> None:
        self._stop.set()
        for capture_socket in self._sockets:
            try:
                capture_socket.close()
            except OSError:
                pass
        for thread in self._threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
        self._threads.clear()
        self._sockets.clear()

    def _udp_loop(
        self,
        capture_socket: socket.socket,
        interface: str,
        endpoints: Tuple[ArmEndpointConfig, ...],
    ) -> None:
        while not self._stop.is_set():
            try:
                packet, address = capture_socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    self._history.record_error("source", f"{interface}: {exc}")
                return
            if interface == "lo" and len(address) >= 3 and address[2] == PACKET_OUTGOING:
                continue
            try:
                datagram = extract_ipv4_udp(packet)
                if datagram is None:
                    continue
                matched = _match_endpoint(datagram, endpoints)
                if matched is None:
                    continue
                endpoint, channel = matched
                decoded = (
                    decode_control(datagram.payload)
                    if channel == "wire_action"
                    else decode_feedback(datagram.payload)
                )
                if len(decoded.axes) != len(endpoint.motor_indices):
                    raise DecodeError(
                        "arm frame axis count does not match endpoint configuration"
                    )
                if any(axis.node_id != slot for slot, axis in enumerate(decoded.axes)):
                    raise DecodeError("arm frame node IDs do not match endpoint slots")
                if channel == "wire_action":
                    self._history.observe_udp_mode(
                        decoded.axes[0].mode,
                        any(axis.kp > 0.0 or axis.kd > 0.0 for axis in decoded.axes),
                    )
                self._history.add_wire(
                    channel,
                    {
                        endpoint.motor_indices[slot]: axis.position_rad
                        for slot, axis in enumerate(decoded.axes)
                    },
                )
            except DecodeError as exc:
                self._history.record_error("wire_decode", f"{interface}: {exc}")


def _ip_matches(configured: str, actual: str) -> bool:
    return configured == "0.0.0.0" or configured == actual


def _match_endpoint(
    datagram, endpoints: Iterable[ArmEndpointConfig]
) -> Optional[Tuple[ArmEndpointConfig, str]]:
    for endpoint in endpoints:
        if (
            datagram.destination_port == endpoint.bind_port
            and datagram.source_port == endpoint.peer_port
            and _ip_matches(endpoint.bind_ip, datagram.destination_ip)
            and _ip_matches(endpoint.peer_ip, datagram.source_ip)
        ):
            return endpoint, "wire_action"
        if (
            datagram.source_port == endpoint.bind_port
            and datagram.destination_port == endpoint.peer_port
            and _ip_matches(endpoint.bind_ip, datagram.source_ip)
            and _ip_matches(endpoint.peer_ip, datagram.destination_ip)
        ):
            return endpoint, "wire_state"
    return None


def _interface_ipv4(interface: str) -> Optional[str]:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        request = struct.pack("256s", interface.encode("ascii")[:15])
        response = fcntl.ioctl(probe.fileno(), 0x8915, request)  # SIOCGIFADDR
        return socket.inet_ntoa(response[20:24])
    except OSError:
        return None
    finally:
        probe.close()


def _interface_for_endpoint(endpoint: ArmEndpointConfig) -> str:
    if endpoint.bind_ip.startswith("127.") or endpoint.peer_ip.startswith("127."):
        return "lo"
    target_ip = endpoint.bind_ip
    if target_ip == "0.0.0.0":
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((endpoint.peer_ip, endpoint.peer_port))
            target_ip = probe.getsockname()[0]
        finally:
            probe.close()
    for _index, interface in socket.if_nameindex():
        if _interface_ipv4(interface) == target_ip:
            return interface
    raise RuntimeError(
        f"cannot resolve capture interface for arm endpoint {endpoint.index} "
        f"local IP {target_ip}"
    )
