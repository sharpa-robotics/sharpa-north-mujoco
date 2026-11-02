"""Passive Zenoh source for raw glove commands."""

from __future__ import annotations

from hand_history import MonitorHistory
from zenoh_connection import ZENOH_ENDPOINT, client_config


class ZenohCapture:
    def __init__(
        self,
        zenoh_module,
        proto_module,
        side: str,
        history: MonitorHistory,
    ):
        if side not in ("left", "right"):
            raise ValueError(f"invalid glove side: {side}")
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
            self.topic, self._on_action
        )

    def _on_action(self, sample) -> None:
        try:
            message = self._proto.HandCommand()
            message.ParseFromString(bytes(sample.payload))
            self._history.add("zenoh_action", message.joints.position)
        except Exception as exc:
            self._history.record_error(
                "zenoh_decode",
                f"{self._side} glove raw action decode failed: {exc}",
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
