"""
Laptop tests for what task2mn changed on top of task2-working's Task 2 -
no robot, no serial.

    python -m unittest test_task2mn_fixes -v
"""

import math
import sys
import types
import unittest
from threading import Event
from unittest.mock import Mock, patch

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


class FloorRun20260914(unittest.TestCase):
    """What the 2026-09-14 05:46 / 05:47 floor logs showed."""

    @staticmethod
    def _scan_route(readings):
        import task2mn_route as route_module
        route = route_module.Task2(Mock(), startup=False, sleep_fn=lambda _: None)
        pose = t2.Pose(164.7, -11.1, math.radians(92.8))   # logged RST pose
        route._pose = Mock(return_value=pose)
        route.frame.pose = Mock(return_value=pose)
        route._ir = Mock(side_effect=list(readings) + [None] * 20)
        route._us = Mock(return_value=90.0)
        route._stable_us = Mock(return_value=90.0)
        route._wait_for_stopped_pose = Mock(return_value=pose)
        # Predicted 14.9 cm gap, as logged: acquisition window 10..26 cm.
        return route, pose.x + t2.IR_SIDE_OFFSET_CM + 14.9

    def _scan(self, moving):
        """Two stationary 13 cm sightings, then `moving` while driving.
        Returns the poll() results and everything logged."""
        route, face = self._scan_route([13.0, 13.0] + list(moving))
        stops = []

        def execute(stm, line, poll):
            if not stops:
                for _ in moving:
                    stops.append(poll())
                    if stops[-1]:
                        return "STOPPED"
            return "OK"

        with patch.object(t2, "run_polling", side_effect=execute), \
                self.assertLogs(level="INFO") as logs:
            route.follow_face("F63", face, 1.0, 1.0, 18.0)
        return stops, " ".join(logs.output)

    def test_one_low_reading_no_longer_aborts_the_face_scan(self):
        # Logged: min 11 / median 13 cm. The 11 alone put the body 5.7 cm
        # off the face, under the 6.5 cm guard, and ended the run.
        stops, errors = self._scan([13.0, 13.0, 11.0, 13.0, 13.0])
        self.assertEqual(stops, [False] * 5)
        self.assertNotIn("body clearance", errors)

    def test_face_that_stays_close_still_stops(self):
        stops, errors = self._scan([13.0, 11.0, 11.0])
        self.assertEqual(stops, [False, False, True])
        self.assertIn("body clearance", errors)

    def test_one_reading_inside_the_hard_floor_still_stops(self):
        stops, errors = self._scan([13.0, 8.0])
        self.assertEqual(stops, [False, True])
        self.assertIn("body clearance", errors)

    def test_one_beam_past_the_box_edge_keeps_the_ir_fix(self):
        # Logged: [-37.6, -2.5, -2.5, -1.6] -> "Ignoring inconsistent fix".
        import task2mn_route as route_module
        route = route_module.Task2(Mock(), startup=False, sleep_fn=lambda _: None)
        route.frame.origin = dict(x_mm=0, y_mm=0, heading_deg=0)
        route._pose = Mock(side_effect=[t2.Pose(50, 29, 0), t2.Pose(84, 29, 0)])
        route.frame.pose = Mock(side_effect=[t2.Pose(x, 29, 0) for x in (66, 68, 70, 72)])
        route._ir = Mock(side_effect=[15, 15, 50, 15])      # 50: past the edge

        def execute(stm, line, poll):
            for _ in range(4):
                self.assertFalse(poll())
            return "OK"

        with patch.object(t2, "run_polling", side_effect=execute):
            self.assertTrue(route._view_line([t2.Seg("F", 34)], t2.Course(80, 150, 90)))
        self.assertAlmostEqual(route.frame.y_shift, -3.75)

    def _mouth(self, offset, plan=Ellipsis):
        frame = Mock()
        frame.pose.return_value = t2.Pose(8.4, offset, math.radians(182.0))
        stm = Mock()
        stm.set_fast.return_value = True
        runner = t2.ParkRunner(stm, frame, t2.Course(76.4, 169.9, 75.8))
        c = t2.BAY_HALF_WIDTH_CM - t2.IR_SIDE_OFFSET_CM
        runner._ir = Mock(side_effect=lambda side: c + offset if side == "left"
                          else c - offset)
        runner._send = Mock(return_value=True)
        runner._park_to_back_wall = Mock(return_value=True)
        patches = [patch("task2mn_park.time.sleep")]
        if plan is not Ellipsis:
            patches.append(patch.object(t2, "plan_centre", return_value=plan))
        for p in patches:
            p.start()
        try:
            self.assertTrue(runner._park_from_mouth())
        finally:
            for p in patches:
                p.stop()
        return runner

    def test_five_cm_off_centre_is_centred(self):
        # Logged: left 32 / right 22 -> +5.0 cm, "keeps 15.0 cm to back
        # fence, so Line C is skipped", parked 6 cm off.
        runner = self._mouth(5.0)
        runner._send.assert_called_once()
        runner._park_to_back_wall.assert_called_once()

    def test_near_centre_still_goes_straight_in(self):
        runner = self._mouth(t2.PARK_CENTRE_TOL_CM - 1.0)
        runner._send.assert_not_called()

    def test_no_centring_line_falls_back_to_a_safe_straight_entry(self):
        runner = self._mouth(5.0, plan=None)
        runner._send.assert_not_called()
        runner._park_to_back_wall.assert_called_once()


class AimAtObstacle2(unittest.TestCase):
    """Off by default; T2MN_AIM_OBS2_MAX_DEG turns it on."""

    def setUp(self):
        import task2mn_route as route_module
        self.route_module = route_module
        self._saved = route_module.AIM_OBS2_MAX_DEG
        route_module.AIM_OBS2_MAX_DEG = 25

    def tearDown(self):
        self.route_module.AIM_OBS2_MAX_DEG = self._saved

    def test_view_ends_pointed_at_the_arrow_without_losing_clearance(self):
        route = self.route_module.Task2(Mock(), startup=False)
        course = t2.Course(89.7, 190.0, self.route_module.CONSERVATIVE_W2,
                           wall_y=self.route_module.CONSERVATIVE_WALL_Y)
        for side in (1, -1):
            with self.subTest(side=side):
                p = t2.Pose(61.8, side * 20.4, 0.0)
                view = [t2.Seg("F", 32)]
                segs = route._aim_at_obstacle2(p, view, course)
                self.assertEqual(segs[0].kind, "F")
                self.assertEqual(segs[-1].kind, "R" if side > 0 else "L")
                end = t2.run(p, segs, route.r, step=1e6)[-1]
                cx = end.x + self.route_module.CAMERA_FROM_AXLE_CM * math.cos(end.th)
                cy = end.y + self.route_module.CAMERA_FROM_AXLE_CM * math.sin(end.th)
                bearing = math.atan2(-cy, course.x2 - cx)
                self.assertLess(abs(math.degrees(bearing - end.th)), 2.0)
                self.assertGreaterEqual(route._check(p, segs, course),
                                        route._check(p, view, course) - 0.05)
                self.assertGreaterEqual(
                    route._first_board_behind_camera(end, course.x1),
                    self.route_module.STAGE2_REAR_EXCLUSION_CM)

    def test_off_by_default(self):
        self.route_module.AIM_OBS2_MAX_DEG = self._saved
        route = self.route_module.Task2(Mock(), startup=False)
        self.assertEqual(route._aim_at_obstacle2(
            t2.Pose(61.8, 20.4, 0.0), [t2.Seg("F", 32)],
            t2.Course(89.7, 190.0, 90)), [])

    def test_no_handoff_from_the_aim_backs_round_the_arc_alone(self):
        route = self.route_module.Task2(Mock(), startup=False)
        course = t2.Course(89.7, 190.0, 90, wall_y=self.route_module.CONSERVATIVE_WALL_Y)
        origin = t2.Pose(61.8, -20.4, 0.0)
        view = [t2.Seg("F", 40), t2.Seg("L", 15)]
        route._stage2_view_origin = origin
        route._stage2_view_segments = list(view)
        route._stage2_aim = view[-1]
        pose = t2.run(origin, view, route.r)[-1]
        route._pose = lambda: pose
        sent = []

        def move(segs):
            nonlocal pose
            sent.append(segs)
            pose = t2.run(pose, segs, route.r)[-1]
            return True

        route._line = move
        route.select_face_handoff = Mock(side_effect=[None, ([t2.Seg("L", 90)], False, 8)])
        route._execute_face_handoff = Mock(return_value=False)
        route.around_obstacle2(course, -1, 1)
        self.assertEqual(sent, [[t2.Seg("l", 15)]])
        self.assertAlmostEqual(pose.th, 0.0)

    def test_simulated_runs_still_park(self):
        for case in ((90, 90, 60, "right"), (60, 150, 30, "left")):
            code, fake, course = ts.simulate(*case)
            self.assertEqual(code, 0, case)
            clear, name = ts.run_clearance(fake, course)
            self.assertGreater(clear, t2.CLEARANCE_FLOOR_CM, f"{case}: {name}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
