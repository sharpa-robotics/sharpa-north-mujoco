"""Passive Zenoh and AF_PACKET Wave hand data sources."""

from __future__ import annotations

import fcntl
import socket
import struct
import threading
from collections import defaultdict
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

from hand_config import HandEndpointConfig, HandMonitorConfig
from hand_history import MonitorHistory
from hand_protocol import DecodeError, decode_wave_joint_packet, extract_ipv4_udp
from zenoh_connection import ZENOH_ENDPOINT, client_config


ETH_P_IP = 0x0800
PACKET_OUTGOING = 4


class ZenohCapture:
    def __init__(self, zenoh_module, proto_module, histories: Mapping[str, MonitorHistory]):
        self._zenoh = zenoh_module
        self._proto = proto_module
        self._histories = histories
        self._session = None
        self._subscribers = []

    def start(self) -> None:
        config = client_config(self._zenoh)
        self._session = self._zenoh.open(config)
        for side, history in self._histories.items():
            self._subscribers.extend(
                (
                    self._session.declare_subscriber(
                        f"action/{side}_hand", self._action_callback(history, side)
                    ),
                    self._session.declare_subscriber(
                        f"state/{side}_hand", self._state_callback(history, side)
                    ),
                )
            )

    def _action_callback(self, history: MonitorHistory, side: str):
        def callback(sample) -> None:
            try:
                message = self._proto.HandCommand()
                message.ParseFromString(bytes(sample.payload))
                history.add("zenoh_action", message.joints.position)
            except Exception as exc:
                history.record_error(
                    "zenoh_decode", f"{side} Zenoh action decode failed: {exc}"
                )

        return callback

    def _state_callback(self, history: MonitorHistory, side: str):
        def callback(sample) -> None:
            try:
                message = self._proto.HandState()
                message.ParseFromString(bytes(sample.payload))
                history.add("zenoh_state", message.joints.position)
            except Exception as exc:
                history.record_error(
                    "zenoh_decode", f"{side} Zenoh state decode failed: {exc}"
                )

        return callback

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


class GloveZenohCapture:
    """Receive one raw glove command topic without touching the Wave UDP path."""

    def __init__(
        self, zenoh_module, proto_module, side: str, history: MonitorHistory
    ):
        self._zenoh = zenoh_module
        self._proto = proto_module
        self._side = side
        self._history = history
        self._session = None
        self._subscriber = None

    @property
    def topic(self) -> str:
        return f"action/{self._side}_glove/raw"

    def start(self) -> None:
        config = client_config(self._zenoh)
        self._session = self._zenoh.open(config)
        self._subscriber = self._session.declare_subscriber(
            self.topic, self._on_sample
        )

    def _on_sample(self, sample) -> None:
        try:
            message = self._proto.HandCommand()
            message.ParseFromString(bytes(sample.payload))
            self._history.add("zenoh_action", message.joints.position)
        except Exception as exc:
            self._history.record_error(
                "zenoh_decode", f"{self._side} raw glove decode failed: {exc}"
            )

    def stop(self) -> None:
        if self._subscriber is not None:
            try:
                self._subscriber.undeclare()
            except Exception:
                pass
            self._subscriber = None
        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass
            self._session = None


class WireCapture:
    def __init__(
        self, config: HandMonitorConfig, histories: Mapping[str, MonitorHistory]
    ):
        self._config = config
        self._histories = histories
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._sockets: List[socket.socket] = []
        self.interfaces: Tuple[str, ...] = ()

    def start(self) -> None:
        grouped: Dict[str, List[HandEndpointConfig]] = defaultdict(list)
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
                        name=f"hand-udp-{interface}",
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
            raise RuntimeError("Wave UDP monitoring requires root or CAP_NET_RAW") from exc
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
        endpoints: Tuple[HandEndpointConfig, ...],
    ) -> None:
        while not self._stop.is_set():
            try:
                packet, address = capture_socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    self._record_source_error(endpoints, f"{interface}: {exc}")
                return
            if interface == "lo" and len(address) >= 3 and address[2] == PACKET_OUTGOING:
                continue
            matched = None
            try:
                datagram = extract_ipv4_udp(packet)
                if datagram is None:
                    continue
                matched = _match_endpoint(datagram, endpoints)
                if matched is None:
                    continue
                endpoint, channel = matched
                expected_version = 0 if channel == "wire_action" else 1
                decoded = decode_wave_joint_packet(datagram.payload, expected_version)
                if decoded.side != endpoint.side:
                    raise DecodeError(
                        f"Wave packet side {decoded.side} does not match {endpoint.side} endpoint"
                    )
                self._histories[endpoint.side].add(channel, decoded.positions)
            except DecodeError as exc:
                side = matched[0].side if matched else "unknown"
                history = self._histories.get(side)
                if history is not None:
                    history.record_error("wire_decode", f"{interface}: {exc}")

    def _record_source_error(
        self, endpoints: Iterable[HandEndpointConfig], message: str
    ) -> None:
        for endpoint in endpoints:
            self._histories[endpoint.side].record_error("source", message)


def _ip_matches(configured: str, actual: str) -> bool:
    return configured == "0.0.0.0" or configured == actual


def _match_endpoint(datagram, endpoints: Iterable[HandEndpointConfig]):
    for endpoint in endpoints:
        if (
            datagram.destination_port == endpoint.command_udp_port
            and _ip_matches(endpoint.device_ip, datagram.destination_ip)
            and _ip_matches(endpoint.sdk_ip, datagram.source_ip)
        ):
            return endpoint, "wire_action"
        if (
            datagram.source_port == endpoint.feedback_source_port
            and datagram.destination_port == endpoint.sdk_joint_port
            and _ip_matches(endpoint.device_ip, datagram.source_ip)
            and _ip_matches(endpoint.sdk_ip, datagram.destination_ip)
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


def _interface_for_endpoint(endpoint: HandEndpointConfig) -> str:
    if endpoint.device_ip.startswith("127.") or endpoint.sdk_ip.startswith("127."):
        return "lo"
    target_ip = endpoint.sdk_ip
    if target_ip == "0.0.0.0":
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((endpoint.device_ip, endpoint.command_udp_port))
            target_ip = probe.getsockname()[0]
        finally:
            probe.close()
    for _index, interface in socket.if_nameindex():
        if _interface_ipv4(interface) == target_ip:
            return interface
    raise RuntimeError(
        f"cannot resolve capture interface for {endpoint.side} hand local IP {target_ip}"
    )
