"""Passive Zenoh, SocketCAN, and AF_PACKET torso data sources."""

from __future__ import annotations

import fcntl
import socket
import struct
import threading
from collections import defaultdict
from typing import Dict, Iterable, List, Optional, Set, Tuple

from torso_config import EndpointConfig, TorsoMonitorConfig
from torso_history import MonitorHistory
from torso_protocol import (
    MCU_COMMAND_SOURCE,
    MCU_SENSOR_SOURCE,
    CanFrame,
    DecodeError,
    decode_embedded_can,
    decode_mcu_envelope,
    decode_mit_position_range,
    decode_position_action,
    decode_position_feedback,
    decode_socketcan_frame,
    extract_ipv4_udp,
    position_action_control_mode,
)
from zenoh_connection import ZENOH_ENDPOINT, client_config


ZENOH_ACTION_TOPIC = "action/torso"
ZENOH_STATE_TOPIC = "state/torso"
ETH_P_IP = 0x0800
PACKET_OUTGOING = 4


class ZenohCapture:
    def __init__(self, zenoh_module, proto_module, history: MonitorHistory):
        self._zenoh = zenoh_module
        self._proto = proto_module
        self._history = history
        self._session = None
        self._subscribers = []

    def start(self) -> None:
        config = client_config(self._zenoh)
        self._session = self._zenoh.open(config)
        self._subscribers = [
            self._session.declare_subscriber(ZENOH_ACTION_TOPIC, self._on_action),
            self._session.declare_subscriber(ZENOH_STATE_TOPIC, self._on_state),
        ]

    def _on_action(self, sample) -> None:
        try:
            message = self._proto.TorsoCommand()
            message.ParseFromString(bytes(sample.payload))
            self._history.add_zenoh("zenoh_action", message.joints.position)
        except Exception as exc:
            self._history.record_error("zenoh_decode", f"Zenoh action decode failed: {exc}")

    def _on_state(self, sample) -> None:
        try:
            message = self._proto.TorsoState()
            message.ParseFromString(bytes(sample.payload))
            self._history.add_zenoh("zenoh_state", message.joints.position)
        except Exception as exc:
            self._history.record_error("zenoh_decode", f"Zenoh state decode failed: {exc}")

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
    def __init__(self, config: TorsoMonitorConfig, history: MonitorHistory):
        self._config = config
        self._history = history
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._sockets: List[socket.socket] = []
        self._mit_ranges: Dict[int, float] = {}
        self._mit_lock = threading.Lock()
        self.interfaces: Tuple[str, ...] = ()

    def start(self) -> None:
        if self._config.driver == "can":
            self._start_can()
        else:
            self._start_udp()

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

    def _ranges_snapshot(self) -> Dict[int, float]:
        with self._mit_lock:
            return dict(self._mit_ranges)

    def _process_frame(
        self,
        frame: CanFrame,
        valid_ids: Set[int],
        expected_channel: Optional[str] = None,
    ) -> None:
        try:
            learned = decode_mit_position_range(frame)
            if learned is not None and learned[0] in valid_ids:
                with self._mit_lock:
                    self._mit_ranges[learned[0]] = learned[1]
                return
            action = decode_position_action(frame, valid_ids, self._ranges_snapshot())
            if action is not None:
                if expected_channel not in (None, "wire_action"):
                    raise DecodeError("action CAN frame arrived in MCU sensor uplink")
                self._history.add_wire(
                    "wire_action",
                    action,
                    control_mode=position_action_control_mode(frame),
                )
                return
            state = decode_position_feedback(frame, valid_ids)
            if state is not None:
                if expected_channel not in (None, "wire_state"):
                    raise DecodeError("state CAN frame arrived in MCU command downlink")
                self._history.add_wire("wire_state", state)
        except DecodeError as exc:
            self._history.record_error("wire_decode", str(exc))

    def _start_can(self) -> None:
        interface_ids: Dict[str, Set[int]] = defaultdict(set)
        for endpoint in self._config.endpoints:
            interface_ids[endpoint.socketcan_interface].update(endpoint.motor_ids)
        opened = []
        try:
            for interface, valid_ids in interface_ids.items():
                capture_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                capture_socket.setsockopt(
                    getattr(socket, "SOL_CAN_RAW", 101),
                    getattr(socket, "CAN_RAW_FD_FRAMES", 5),
                    1,
                )
                capture_socket.settimeout(0.25)
                capture_socket.bind((interface,))
                opened.append(capture_socket)
                thread = threading.Thread(
                    target=self._can_loop,
                    args=(capture_socket, set(valid_ids), interface),
                    name=f"torso-can-{interface}",
                    daemon=True,
                )
                self._threads.append(thread)
            self._sockets.extend(opened)
            self.interfaces = tuple(interface_ids)
            for thread in self._threads:
                thread.start()
        except Exception:
            for capture_socket in opened:
                capture_socket.close()
            raise

    def _can_loop(self, capture_socket: socket.socket, valid_ids: Set[int], interface: str) -> None:
        while not self._stop.is_set():
            try:
                packet = capture_socket.recv(72)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    self._history.record_error("source", f"{interface}: {exc}")
                return
            try:
                self._process_frame(decode_socketcan_frame(packet), valid_ids)
            except DecodeError as exc:
                self._history.record_error("wire_decode", f"{interface}: {exc}")

    def _start_udp(self) -> None:
        endpoint_interfaces = {
            endpoint.name: _interface_for_endpoint(endpoint)
            for endpoint in self._config.endpoints
        }
        grouped: Dict[str, List[EndpointConfig]] = defaultdict(list)
        for endpoint in self._config.endpoints:
            grouped[endpoint_interfaces[endpoint.name]].append(endpoint)
        opened = []
        try:
            for interface, endpoints in grouped.items():
                capture_socket = socket.socket(
                    socket.AF_PACKET, socket.SOCK_DGRAM, socket.htons(ETH_P_IP)
                )
                capture_socket.bind((interface, 0))
                capture_socket.settimeout(0.25)
                opened.append(capture_socket)
                thread = threading.Thread(
                    target=self._udp_loop,
                    args=(capture_socket, interface, tuple(endpoints)),
                    name=f"torso-udp-{interface}",
                    daemon=True,
                )
                self._threads.append(thread)
            self._sockets.extend(opened)
            self.interfaces = tuple(grouped)
            for thread in self._threads:
                thread.start()
        except PermissionError as exc:
            for capture_socket in opened:
                capture_socket.close()
            raise RuntimeError("UDP monitoring requires root or CAP_NET_RAW") from exc
        except Exception:
            for capture_socket in opened:
                capture_socket.close()
            raise

    def _udp_loop(
        self,
        capture_socket: socket.socket,
        interface: str,
        endpoints: Tuple[EndpointConfig, ...],
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
                envelope = decode_mcu_envelope(datagram.payload)
                expected_source = MCU_COMMAND_SOURCE if channel == "wire_action" else MCU_SENSOR_SOURCE
                if envelope.source_code != expected_source:
                    raise DecodeError(f"unexpected MCU source code 0x{envelope.source_code:08x}")
                if channel == "wire_action" and envelope.gptp_ns is not None:
                    raise DecodeError("MCU command downlink unexpectedly has a gPTP trailer")
                if channel == "wire_state" and envelope.gptp_ns is None:
                    raise DecodeError("MCU sensor uplink is missing its gPTP trailer")
                frame = decode_embedded_can(envelope.payload)
                if frame.can_interface != endpoint.can_interface_id:
                    raise DecodeError("embedded CAN interface does not match configuration")
                self._process_frame(frame, set(endpoint.motor_ids), channel)
            except DecodeError as exc:
                self._history.record_error("wire_decode", f"{interface}: {exc}")


def _ip_matches(configured: str, actual: str) -> bool:
    return configured == "0.0.0.0" or configured == actual


def _match_endpoint(datagram, endpoints: Iterable[EndpointConfig]):
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


def _interface_for_endpoint(endpoint: EndpointConfig) -> str:
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
        f"cannot resolve capture interface for {endpoint.name} local IP {target_ip}"
    )
