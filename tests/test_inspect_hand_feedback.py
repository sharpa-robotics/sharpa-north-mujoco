"""Check passive hand diagnostics using real protobuf messages, without a robot."""

import contextlib
import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "examples" / "motion"))
sys.path.insert(0, str(ROOT / "proto" / "platform"))

import inspect_hand_feedback as inspect
import north_pb2 as proto


class InspectHandFeedbackTest(unittest.TestCase):
    def sample(self, joint5_deg, message_type=proto.HandState):
        message = message_type()
        values = [0.0] * 22
        values[4] = math.radians(joint5_deg)
        message.joints.position.extend(values)
        return SimpleNamespace(payload=message.SerializeToString())

    def test_span_uses_all_frames_and_report_identifies_fixed_j5(self):
        command = inspect.StreamStats(proto.HandCommand)
        state = inspect.StreamStats(proto.HandState)
        for value in (0, 20, 0):
            command.receive(self.sample(value, proto.HandCommand))
            state.receive(self.sample(120))
        self.assertEqual(state.snapshot()["count"], 3)
        self.assertEqual(state.snapshot()["span"][4], 0.0)
        self.assertAlmostEqual(command.snapshot()["span"][4], 20.0, places=4)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            inspect.report({"right": (command, state)}, summary=True)
        self.assertIn("largest_abs=J5(thumb_IP)", output.getvalue())
        self.assertIn("J5_span=0.0000deg", output.getvalue())
        self.assertIn("OUTSIDE_SCRIPT_LIMITS", output.getvalue())

    def test_invalid_sample_does_not_replace_last_valid_state(self):
        stream = inspect.StreamStats(proto.HandState)
        stream.receive(self.sample(10))
        stream.receive(self.sample(float("nan")))
        stream.receive(SimpleNamespace(payload=b"\xff"))
        self.assertEqual(stream.snapshot()["count"], 1)
        self.assertEqual(stream.snapshot()["invalid"], 2)
        self.assertAlmostEqual(stream.snapshot()["last"][4], 10, places=4)

    def test_observer_only_subscribes_and_closes_on_interrupt(self):
        # No put/get/service methods exist on this fake session.
        session = SimpleNamespace(declare_subscriber=Mock(return_value=Mock()), close=Mock())
        args = SimpleNamespace(platform_root=ROOT, connect="test", side="right", duration=1, interval=1)
        with patch.object(inspect, "load_dependencies", return_value=(None, proto)), \
                patch.object(inspect, "open_session", return_value=session), \
                patch.object(inspect.time, "sleep", side_effect=KeyboardInterrupt), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(inspect.observe(args), 1)
        self.assertEqual([call.args[0] for call in session.declare_subscriber.call_args_list],
                         ["action/right_hand", "state/right_hand"])
        self.assertEqual(session.declare_subscriber.return_value.undeclare.call_count, 2)
        session.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
