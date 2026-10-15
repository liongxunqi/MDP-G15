"""Geometric route regressions, not hardware timing or braking certification."""

import math
import unittest
from unittest.mock import Mock, patch

import task2t as route_module
import task2t_park as geometry
from task2t_sim import FakeSTM, run_clearance


class WholeRoute(unittest.TestCase):
    def test_all_arrow_pairs_and_obstacle_lengths_finish_inside_bay(self):
        with patch("time.sleep"):
            for length in (30, 60, 90):
                for pair in ("LL", "LR", "RL", "RR"):
                    with self.subTest(length=length, pair=pair):
                        course = geometry.Course(71.7, 152.3, length)
                        stm = FakeSTM(course, geometry.WorldFrame(None).start,
                                      geometry.RADIUS_CM)
                        arrows = iter("left" if arrow == "L" else "right"
                                      for arrow in pair)
                        route = route_module.Task2(
                            stm, input_fn=lambda _: next(arrows),
                            sleep_fn=lambda _: None, startup=False, wait_for_go=False)
                        # task2.py uses measured WPOSE heading, not commanded heading.
                        route._pose = route.frame.pose
                        self.assertEqual(route.run(), 0)
                        gap, obstacle = run_clearance(stm, course)
                        self.assertGreaterEqual(gap, geometry.CLEARANCE_FLOOR_CM, obstacle)
                        self.assertTrue(all(-geometry.BAY_DEPTH_CM <= x <= 0
                                            and abs(y) <= geometry.BAY_HALF_WIDTH_CM
                                            for x, y in geometry.footprint(stm.pose)))
                        self.assertLessEqual(geometry.parking_heading_error(stm.pose.th),
                                             geometry.PARK_HEADING_TOL_DEG)

    def test_close_obstacle_uses_adaptive_lane_for_all_arrow_pairs(self):
        """Regression for the 01:09 floor runs rejected at 6.9-7.1 cm."""
        with patch("time.sleep"):
            for pair in ("LL", "LR", "RL", "RR"):
                with self.subTest(pair=pair):
                    course = geometry.Course(49.7, 119.7, 90)
                    stm = FakeSTM(course, geometry.WorldFrame(None).start,
                                  geometry.RADIUS_CM)
                    arrows = iter("left" if arrow == "L" else "right"
                                  for arrow in pair)
                    route = route_module.Task2(
                        stm, input_fn=lambda _: next(arrows),
                        sleep_fn=lambda _: None, startup=False,
                        wait_for_go=False)
                    self.assertEqual(route.run(), 0)
                    gap, obstacle = run_clearance(stm, course)
                    self.assertGreaterEqual(gap,
                                            geometry.CLEARANCE_FLOOR_CM,
                                            obstacle)
                    self.assertTrue(all(
                        -geometry.BAY_DEPTH_CM <= x <= 0
                        and abs(y) <= geometry.BAY_HALF_WIDTH_CM
                        for x, y in geometry.footprint(stm.pose)))


    def test_timeout_does_not_accept_below_floor_or_partial_sweep(self):
        for result in (2.0, route_module.HANDOFF_CLEARANCE_CM - .1):
            with self.subTest(clearance=result):
                route = route_module.Task2(Mock(), startup=False)
                options = [([geometry.Seg("F", 10)], False),
                           ([geometry.Seg("F", 12)], False)]
                with patch.object(route, "face_handoff_candidates", return_value=iter(options)), \
                        patch.object(route, "_face_handoff_endpoint_valid", return_value=True), \
                        patch.object(route, "_handoff_clearance", side_effect=[result, TimeoutError]):
                    self.assertIsNone(route.select_face_handoff(
                        geometry.Pose(80, 25, 0), geometry.Course(84, 163, 90), 1, 1))
                route.stm.send_line.assert_not_called()

    def test_low_ir_range_is_not_allowed_to_become_a_false_edge(self):
        route = route_module.Task2(Mock(), startup=False, sleep_fn=lambda _: None)
        pose = geometry.Pose(125, 0, math.pi / 2)
        route._pose = Mock(return_value=pose)
        route.frame.pose = Mock(return_value=pose)
        route._ir = Mock(return_value=14)
        route._wait_for_stopped_pose = Mock(return_value=pose)

        def move(stm, token, poll):
            route._ir.return_value = 10
            self.assertTrue(poll())
            return "STOPPED"

        with patch.object(geometry, "run_polling", side_effect=move):
            self.assertIsNone(route.follow_face("F58", 150, 1, 1, 14))
        route.stm.abort.assert_called_once()

    def test_short_echo_keeps_both_possible_faces_occupied(self):
        route = route_module.Task2(Mock(), startup=False)
        route.obs2_front_max = 150
        course = geometry.Course(80, 139, 90)
        box = next(b for b in route._handoff_boxes([course]) if b.name == "obstacle 2")
        self.assertEqual((box.x0, box.x1), (139, 160))
        self.assertLess(route._check(geometry.Pose(155, 0, 0), [], course), 0)

    def test_existing_close_pose_cannot_lower_collision_floor(self):
        route = route_module.Task2(Mock(), startup=False)
        route._check = Mock(return_value=2)
        self.assertIsNone(route._first_clear(
            geometry.Pose(80, 20, 0), [[geometry.Seg("F", 10)]],
            geometry.Course(84, 163, 90), "recovery"))
        route.stm.send_line.assert_not_called()


    def test_single_edge_measurement_preserves_return_landmark_fix(self):
        frame = Mock()
        frame.pose.return_value = geometry.Pose(180, 65, math.pi)
        frame.needs_lateral_fix = True
        runner = geometry.ParkRunner(Mock(), frame, geometry.Course(60, 150, 90))
        plan = Mock(seconds=10)
        direct = Mock(segs=[geometry.Seg("F", 100)], end=geometry.Pose(0, 0, math.pi))
        runner._run_line_a = Mock(return_value=True)
        runner._finish = Mock(return_value=True)
        runner._send = Mock()
        with patch.object(geometry, "plan_direct_return", return_value=direct), \
                patch.object(geometry, "plan_return", return_value=plan), \
                patch.object(geometry, "final_cost", return_value=1), \
                patch.object(geometry, "ir_fix", return_value=None):
            self.assertTrue(runner.run())
        runner._run_line_a.assert_called_once()
        runner._send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
