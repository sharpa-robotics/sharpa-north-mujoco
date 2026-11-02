"""Shared UDP client configuration for passive monitors."""

import json
from pathlib import Path

CLIENT_CONFIG = Path(__file__).resolve().parents[2] / "resources/zenoh/client.json"
ZENOH_ENDPOINT = json.loads(CLIENT_CONFIG.read_text())["connect"]["endpoints"][0]


def client_config(zenoh):
    return zenoh.Config.from_file(str(CLIENT_CONFIG))
