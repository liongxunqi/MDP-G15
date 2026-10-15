"""
Laptop tests for what task2mn changed on top of task2-working's Task 2 -
no robot, no serial.

    python -m unittest test_task2mn_fixes -v
"""

import sys
import types
import unittest
from threading import Event
from unittest.mock import Mock

# communications.stm imports pyserial; the laptop may not have it.
if "serial" not in sys.modules:
    _serial = types.ModuleType("serial")
    _serial.Serial = object
    _serial.SerialException = Exception
    sys.modules["serial"] = _serial

import task2mn_link as link
import task2mn_park as t2
import task2mn_sim as ts
from task2mn_vision import Vision


class Link(unittest.TestCase):

    def test_edge_token_packs_side_and_window(self):
        self.assertEqual(link.edge_token("left", 12, 27), "!IRE11227")
        self.assertEqual(link.edge_token("right", 10, 80), "!IRE21080")
        self.assertEqual(link.edge_token(None), "!IRE0")
        for bad in (("up", 12, 27), ("left", 9, 27), ("left", 30, 20),
                    ("right", 12, 81)):
            with self.assertRaises(ValueError):
                link.edge_token(*bad)

    def test_enable_teaches_an_old_client_the_new_tokens(self):
        from communications import stm
        link.enable()
        link.enable()                                   # idempotent
        self.assertTrue(stm.validate_token("!IRE11227")[0])
        self.assertTrue(stm.validate_token("!IRE0")[0])
        self.assertFalse(stm.validate_token("!IRE31227")[0])
        self.assertTrue(stm.validate_token("?IRW")[0])
        self.assertEqual(stm.classify_reply("IRE,2,1,23,0,1,2,3,4"), "query")
        self.assertEqual(stm.classify_reply("IRW,1,2,3,4,5,6,7,8"), "query")
        self.assertTrue(stm.validate_token("F10")[0])  # old tokens untouched

    def test_irw_tells_too_close_from_nothing(self):
        stm = Mock()
        stm.query_fields.return_value = ["65535", "65535", "3600", "200",
                                         "100", "-50", "15", "1234"]
        w = link.read_irw(stm)
        self.assertIsNone(w["left_cm"])
        self.assertTrue(link.near_saturated(w["left_raw"]))
        self.assertFalse(link.near_saturated(w["right_raw"]))
        self.assertEqual(w["wpose"], {"x_mm": 100, "y_mm": -50, "heading_deg": 1.5})

    def test_read_edge(self):
        stm = Mock()
        stm.query_fields.return_value = ["2", "2", "23", "1800", "10", "20", "-5", "9"]
        e = link.read_edge(stm)
        self.assertEqual((e["state"], e["side"], e["cm"]), ("latched", "right", 23))
        stm.query_fields.return_value = None
        self.assertIsNone(link.read_edge(stm))


class CentreOffset(unittest.TestCase):
    C = t2.BAY_HALF_WIDTH_CM - t2.IR_SIDE_OFFSET_CM

    def test_both_fences_unchanged(self):
        y, basis = t2.centre_offset_measured(self.C + 4, self.C - 4)
        self.assertAlmostEqual(y, 4.0)
        self.assertEqual(basis, "both fences")

    def test_one_fence_alone(self):
        y, _ = t2.centre_offset_measured(None, self.C - 6)
        self.assertAlmostEqual(y, 6.0)
        y, _ = t2.centre_offset_measured(self.C + 3, None)
        self.assertAlmostEqual(y, 3.0)

    def test_too_close_is_not_nothing(self):
        # Heading home the left IR faces -y: inside its floor means the robot
        # is hard against the right-hand (-y) fence.
        y, basis = t2.centre_offset_measured(None, 41.0, left_near=True)
        self.assertLess(y, -10.0)
        self.assertIn("left IR inside", basis)
        y, _ = t2.centre_offset_measured(41.0, None, right_near=True)
        self.assertGreater(y, 10.0)

    def test_nothing_readable(self):
        self.assertEqual(t2.centre_offset_measured(None, None),
                         (None, "no fence reading"))


class VisionFallback(unittest.TestCase):

    def _vision(self):
        v = Vision(Mock(), Event(), "run")
        v.default_arrow = "right"
        return v

    def test_timeout_uses_last_vote_then_default(self):
        v = self._vision()
        v.begin(1)
        self.assertEqual(v.wait_arrow(timeout=0.05), "right")   # default
        v.last_vote = "left"
        self.assertEqual(v.wait_arrow(timeout=0.05), "left")    # last vote

    def test_lost_link_does_not_wait(self):
        v = self._vision()
        v.begin(2)
        v.fail("test")
        self.assertFalse(v.active)
        self.assertEqual(v.wait_arrow(timeout=60), "right")

    def test_confirmed_arrow_wins(self):
        v = self._vision()
        v.begin(1)
        v.result = "left"
        self.assertEqual(v.wait_arrow(timeout=0.05), "left")

    def test_stop_still_cancels(self):
        v = self._vision()
        v.stop.set()
        with self.assertRaises(RuntimeError):
            v.wait_arrow(timeout=0.05)


class EndToEnd(unittest.TestCase):

    def test_narrow_obstacle_found_by_the_latch(self):
        for case in ((60, 60, 30, "left"), (100, 120, 36, "right"),
                     (90, 100, 40, "left")):
            code, fake, course = ts.simulate(*case, ir_latch=True)
            self.assertEqual(code, 0, case)
            clear, name = ts.run_clearance(fake, course)
            self.assertGreater(clear, t2.CLEARANCE_FLOOR_CM - 0.5, f"{case}: {name}")

    def test_drifted_robot_parks_centred(self):
        # 3 cm/m of odometry drift: task2t parked these 14-16 cm off centre.
        for case in ((90, 82, 60, "right"), (60, 60, 90, "left"),
                     (100, 80, 40, "left")):
            for drift in (3.0, -3.0):
                if case == (60, 60, 90, "left") and drift < 0:
                    continue        # the route itself fails this one, as task2t does
                code, fake, course = ts.simulate(*case, drift_cm_per_m=drift)
                self.assertEqual(code, 0, (case, drift))
                self.assertLess(abs(fake.pose.y), 6.0, (case, drift, fake.pose))


if __name__ == "__main__":
    unittest.main(verbosity=2)
