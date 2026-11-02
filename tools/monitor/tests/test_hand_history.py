import math
import sys
import unittest
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from hand_history import MonitorHistory


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value


class HistoryTest(unittest.TestCase):
    def test_rate_limit_degree_conversion_and_trim(self):
        clock = FakeClock()
        history = MonitorHistory(history_seconds=1.0, plot_rate_hz=20.0, clock=clock)
        history.add("wire_state", [math.pi / 2] * 22)
        clock.value += 0.01
        history.add("wire_state", [math.pi] * 22)
        snapshot = history.snapshot()
        self.assertEqual(snapshot.counts["wire_state"], 2)
        self.assertEqual(len(snapshot.samples["wire_state"][0]), 1)
        self.assertAlmostEqual(snapshot.samples["wire_state"][0][0][1], 90.0)
        clock.value += 0.05
        history.add("wire_state", [math.pi] * 22)
        self.assertEqual(len(history.snapshot().samples["wire_state"][0]), 2)
        clock.value += 1.1
        self.assertEqual(len(history.snapshot().samples["wire_state"][0]), 0)

    def test_short_and_nonfinite_messages_are_rejected(self):
        history = MonitorHistory()
        self.assertFalse(history.add("zenoh_action", [0.0] * 21))
        self.assertFalse(history.add("wire_action", [math.nan] * 22))
        snapshot = history.snapshot()
        self.assertEqual(snapshot.errors["zenoh_decode"], 1)
        self.assertEqual(snapshot.errors["wire_decode"], 1)


if __name__ == "__main__":
    unittest.main()
