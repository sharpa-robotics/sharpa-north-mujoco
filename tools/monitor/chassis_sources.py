"""Passive Zenoh subscriptions and AF_PACKET Kinco UDP capture."""

import socket
import threading

from chassis_protocol import decode_frame
from torso_protocol import (DecodeError,
                            decode_embedded_can, decode_mcu_envelope, extract_ipv4_udp)
from torso_sources import ETH_P_IP, PACKET_OUTGOING, _interface_for_endpoint, _match_endpoint
from zenoh_connection import client_config


# ChassisUdpEndpointConfig uses 0x02050000 for uplink as well as downlink;
# torso's sensor source (0x01050000) is not the chassis wire profile.
MCU_SOURCE = 0x02050000


class ZenohCapture:
    def __init__(self, zenoh_module, proto_module, history):
        self._zenoh = zenoh_module
        self._proto = proto_module
        self._history = history
        self._session = None
        self._subscribers = []
        self._last_state_stamp = None

    def start(self):
        self._session = self._zenoh.open(client_config(self._zenoh))
        # Append incrementally so partial startup also releases the first subscription.
        for topic, callback in (("action/chassis", self._on_action),
                                ("state/chassis", self._on_state)):
            self._subscribers.append(self._session.declare_subscriber(topic, callback))

    def _on_action(self, sample):
        try:
            message = self._proto.ChassisCommand()
            message.ParseFromString(bytes(sample.payload))
            self._history.add("zenoh_action", (message.vx, message.vy, message.vtheta))
        except Exception as exc:
            self._history.record_error("zenoh_decode", f"chassis action decode failed: {exc}")

    def _on_state(self, sample):
        try:
            message = self._proto.ChassisState()
            message.ParseFromString(bytes(sample.payload))
            if not message.HasField("velocity"):
                raise ValueError("chassis state is missing velocity")
            stamp = (message.header.stamp.seconds, message.header.stamp.nanos)
            if stamp == (0, 0):
                self._history.mark_unavailable("zenoh_state", "no valid feedback timestamp")
                return
            # Platform may republish a frozen feedback cache at 200 Hz. Repeated
            # timestamps are not fresh measurements; let this channel go stale.
            if stamp == self._last_state_stamp:
                return
            velocity = message.velocity
            if self._history.add("zenoh_state", (velocity.linear.x, velocity.linear.y,
                                                  velocity.angular.z)):
                self._last_state_stamp = stamp
        except Exception as exc:
            self._history.record_error("zenoh_decode", f"chassis state decode failed: {exc}")

    def stop(self):
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
    def __init__(self, config, history):
        self._config = config
        self._history = history
        self._stop = threading.Event()
        self._socket = None
        self._thread = None
        self.interfaces = ()

    def start(self):
        interface = _interface_for_endpoint(self._config)
        try:
            self._socket = socket.socket(socket.AF_PACKET, socket.SOCK_DGRAM,
                                         socket.htons(ETH_P_IP))
            self._socket.bind((interface, 0))
            self._socket.settimeout(0.25)
            self.interfaces = (interface,)
            self._thread = threading.Thread(target=self._loop, args=(self._socket, interface),
                                            name="chassis-udp", daemon=True)
            self._thread.start()
        except PermissionError as exc:
            self.stop()
            raise RuntimeError("UDP monitoring requires root or CAP_NET_RAW") from exc
        except Exception:
            self.stop()
            raise

    def process_packet(self, packet):
        """Decode one captured IPv4 packet; useful for offline replay as well."""
        try:
            datagram = extract_ipv4_udp(packet)
            if datagram is None:
                return
            matched = _match_endpoint(datagram, (self._config,))
            if matched is None:
                return
            _, channel = matched
            if datagram.payload[:4] != b"POC2":
                raise DecodeError("unexpected chassis MCU magic")
            envelope = decode_mcu_envelope(datagram.payload)
            if envelope.source_code != MCU_SOURCE:
                raise DecodeError("unexpected chassis MCU source code")
            if (envelope.gptp_ns is None) != (channel == "wire_action"):
                raise DecodeError("unexpected chassis MCU gPTP trailer")
            frame = decode_embedded_can(envelope.payload)
            for event in decode_frame(frame, channel):
                self._history.add_wire(event)
        except DecodeError as exc:
            self._history.record_error("wire_decode", str(exc))

    def _loop(self, capture_socket, interface):
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
            self.process_packet(packet)

    def stop(self):
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
