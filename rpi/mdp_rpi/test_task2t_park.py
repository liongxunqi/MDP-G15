"""
Laptop tests for task2t_park.py, the TIGHT copy - no robot, no serial.

    python -m unittest test_task2t_park -v

The sweep covers the brief's whole range: obstacle 1 at 60-150 cm from the
bay, obstacle 2 another 60-150 cm on, 30-90 cm wide, home on either side.
"""

import math
import unittest
from unittest.mock import Mock, call, patch

import task2t_park as t2
from task2t_park import Course, Pose


def _course(d1, d2, w2):
    return Course(x1=d1, x2=d1 + t2.OBS1_SIZE_CM + d2, w2=w2)


def _parked_ok(p: Pose):
    """Whole footprint inside the bay, with room to spare on every side."""
    pts = t2.footprint(p)
    xs = [q[0] for q in pts]
    ys = [q[1] for q in pts]
    return (max(xs) <= -5.0                                   # rear well inside the mouth
            and min(xs) >= -t2.BAY_DEPTH_CM + 5.0             # front off the back fence
            and max(abs(y) for y in ys) <= t2.BAY_HALF_WIDTH_CM - 5.0)


def _whole_path(p0, plan):
    poses = t2.run(p0, plan.line_a)
    poses += t2.run(poses[-1], plan.final.segs)[1:]
    straight, _ = t2.park_segments(poses[-1])
    poses += t2.run(poses[-1], straight)[1:]
    return poses


class Geometry(unittest.TestCase):

    def test_left_arc_quarter_turn(self):
        p = t2.arc(Pose(0, 0, 0), +1, 90, 30.0)
        self.assertAlmostEqual(p.x, 30.0, places=6)
        self.assertAlmostEqual(p.y, 30.0, places=6)
        self.assertAlmostEqual(p.th, math.pi / 2, places=6)

    def test_right_arc_heading_home(self):
        # Heading -x, a right turn swings toward +y.
        p = t2.arc(Pose(0, 0, math.pi), -1, 90, 30.0)
        self.assertAlmostEqual(p.x, -30.0, places=6)
        self.assertAlmostEqual(p.y, 30.0, places=6)

    def test_lane_change_lands_on_the_lane(self):
        for dy in (-60, -25, -8, 8, 25, 60, 75):
            p = Pose(100, 0, math.pi)
            end = t2.run(p, t2.lane_change(p.th, dy))[-1]
            self.assertAlmostEqual(end.y, dy, delta=0.5, msg=f"dy={dy}")
            self.assertAlmostEqual(end.th, math.pi, places=6)

    def test_clearance_sign(self):
        b = t2.Box(30, 40, -5, 5, "b")
        self.assertLess(t2.clearance(Pose(15, 0, 0), b), 0)            # front in it
        self.assertAlmostEqual(t2.clearance(Pose(0, 0, 0), b),
                               30 - t2.FRONT_FROM_AXLE_CM, places=6)

    def test_ir_side(self):
        # Home on the right (y<0), the centreline is on the robot's right.
        self.assertEqual(t2.ir_side(-40), "right")
        self.assertEqual(t2.ir_side(40), "left")


class Sweep(unittest.TestCase):

    def test_whole_brief_parks_clear(self):
        worst = (float("inf"), None)
        for d1 in (60, 75, 90, 120, 150):
            for d2 in (60, 90, 150):
                for w2 in (30, 50, 70, 90):
                    for side in ("left", "right"):
                        c = _course(d1, d2, w2)
                        p0 = t2.nominal_start(c, side)
                        case = (d1, d2, w2, side)
                        plan = t2.plan_return(p0, c)
                        poses = _whole_path(p0, plan)
                        clear, name = t2.path_clearance(poses, c.boxes())
                        self.assertGreaterEqual(clear, t2.CLEARANCE_CM - 0.5,
                                                f"{case}: {clear:.1f} cm off {name}")
                        self.assertTrue(_parked_ok(poses[-1]),
                                        f"{case}: parked at {poses[-1]}")
                        self.assertLess(abs(poses[-1].y), 1.0, case)
                        worst = min(worst, (clear, case))
        print(f"\nworst clearance over the brief: {worst[0]:.1f} cm at {worst[1]}")


class Robustness(unittest.TestCase):
    """Line A never lands exactly where planned. Line B is replanned from the
    measured pose; it must still be clear of everything."""

    def test_final_replan_absorbs_line_a_error(self):
        for d1 in (60, 90, 150):
            for w2 in (30, 90):
                for side in ("left", "right"):
                    c = _course(d1, 60, w2)
                    p0 = t2.nominal_start(c, side)
                    plan = t2.plan_return(p0, c)
                    stop = t2.run(p0, plan.line_a)[-1]
                    for dx in (-6, 0, 6):
                        for dy in (-8, 0, 8):
                            here = Pose(stop.x + dx, stop.y + dy, stop.th)
                            # An outward 8 cm drift can leave only 3.2 cm to
                            # the wall. No path can honour a 3.5 cm floor at
                            # its start; it must not be declared safe anyway.
                            if t2.path_clearance([here], c.boxes())[0] < t2.CLEARANCE_FLOOR_CM:
                                with self.assertRaises(t2.NoPlan):
                                    t2.plan_final(here, c)
                                continue
                            final = t2.plan_final(here, c)
                            case = (d1, w2, side, dx, dy)
                            self.assertGreaterEqual(final.clearance, t2.CLEARANCE_FLOOR_CM,
                                               f"{case}: {final.clearance:.1f} off {final.closest}")
                            self.assertTrue(_parked_ok(final.parked), case)

    def test_radius_off_by_two_cm_still_clear(self):
        # Plan on the configured radius, drive on a wrong one.
        for d1 in (60, 150):
            for side in ("left", "right"):
                c = _course(d1, 60, 60)
                p0 = t2.nominal_start(c, side)
                plan = t2.plan_return(p0, c)
                for r in (t2.RADIUS_CM - 2, t2.RADIUS_CM + 2):
                    stop = t2.run(p0, plan.line_a, r)[-1]
                    poses = t2.run(stop, plan.final.segs, r)
                    straight, _ = t2.park_segments(poses[-1])
                    poses += t2.run(poses[-1], straight, r)[1:]
                    clear, name = t2.path_clearance(poses, c.boxes())
                    self.assertGreater(clear, 2.0, f"{(d1, side, r)}: {name}")
                    self.assertTrue(_parked_ok(poses[-1]), (d1, side, r))


class IrFix(unittest.TestCase):

    def _samples(self, plan, c, true_y, x_err=0.0, lag_cm=3.0):
        """Fake a pass: the IR sees obstacle 1 while its beam is over it."""
        gap = abs(true_y - c.obs1_y) - t2.OBS1_SIZE_CM / 2 - t2.IR_SIDE_OFFSET_CM
        out = []
        x = plan.x_stop + 120
        while x > plan.x_stop:
            beam = x - t2.IR_FROM_AXLE_CM + lag_cm
            on = c.x1 <= beam <= c.x1 + t2.OBS1_SIZE_CM
            out.append((x + x_err, gap + (0.4 if on else 40.0)))
            x -= 1.5
        return out

    def test_recovers_lateral_and_longitudinal(self):
        c = _course(90, 80, 50)
        p0 = t2.nominal_start(c, "right")
        plan = t2.plan_return(p0, c)
        true_y = -plan.lane - 6.0               # drifted 6 cm wider
        samples = self._samples(plan, c, true_y, x_err=7.0, lag_cm=t2.IR_LAG_CM)
        fix = t2.ir_fix(samples, plan, c)
        self.assertIsNotNone(fix)
        self.assertAlmostEqual(fix.y, true_y, delta=1.0)
        self.assertIsNotNone(fix.x_shift)
        self.assertAlmostEqual(fix.x_shift, -7.0, delta=2.5)

    def test_nothing_seen_gives_none(self):
        c = _course(90, 80, 50)
        plan = t2.plan_return(t2.nominal_start(c, "left"), c)
        self.assertIsNone(t2.ir_fix([(100.0, 70.0)] * 20, plan, c))


class Obstacle2CornerExecution(unittest.TestCase):
    def make_route(self):
        import task2t
        route = task2t.Task2(Mock(), startup=False)
        route._set_fast = Mock()
        route._line = Mock(return_value=True)
        return route

    def test_mirrored_corners_are_sent_separately_at_precision_speed(self):
        for kind in ("L", "R"):
            with self.subTest(kind=kind):
                route = self.make_route()
                segs = [t2.Seg(kind, 90), t2.Seg("F", 4), t2.Seg(kind, 90)]
                p = Pose(0, 0, 0)
                samples = []
                for seg in segs:
                    end = t2.run(p, [seg], route.r)[-1]
                    samples.extend([p, end])
                    p = end
                route._pose = Mock(side_effect=samples)
                course = Mock()
                course.boxes.return_value = []
                self.assertTrue(route.execute_obstacle2_corner(segs, course))
                self.assertEqual([c.args[0] for c in route._line.call_args_list],
                                 [[s] for s in segs])
                route._set_fast.assert_called_once_with(False)

    def test_collision_blocks_first_command(self):
        route = self.make_route()
        route._pose = Mock(return_value=Pose(0, 0, 0))
        course = Mock()
        course.boxes.return_value = [t2.Box(-10, 40, -20, 20, "obstacle 2")]
        self.assertFalse(route.execute_obstacle2_corner([t2.Seg("L", 90)], course))
        route._line.assert_not_called()

    def test_finite_deviation_rechecks_and_continues_from_measured_pose(self):
        for offset, angle in ((11, 0), (0, 10)):
            with self.subTest(offset=offset, angle=angle):
                route = self.make_route()
                p = Pose(0, 0, 0)
                segs = [t2.Seg("L", 90), t2.Seg("L", 90)]
                expected = t2.run(p, segs[:1], route.r)[-1]
                measured = Pose(expected.x + offset, expected.y,
                                expected.th + math.radians(angle))
                finished = t2.run(measured, segs[1:], route.r)[-1]
                route._pose = Mock(side_effect=[p, measured, measured, finished])
                course = Mock()
                course.boxes.return_value = []
                self.assertTrue(route.execute_obstacle2_corner(segs, course))
                self.assertEqual(route._line.call_args_list,
                                 [call(segs[:1]), call(segs[1:])])

    def test_invalid_completion_pose_still_withholds_second_turn(self):
        route = self.make_route()
        p = Pose(0, 0, 0)
        segs = [t2.Seg("L", 90), t2.Seg("L", 90)]
        expected = t2.run(p, segs[:1], route.r)[-1]
        route._pose = Mock(side_effect=[
            p, Pose(float("nan"), expected.y, expected.th)])
        course = Mock()
        course.boxes.return_value = []

        self.assertFalse(route.execute_obstacle2_corner(segs, course))
        route._line.assert_called_once_with(segs[:1])

    def test_speed_refusal_blocks_motion(self):
        route = self.make_route()
        route.fast = True
        self.assertFalse(route.execute_obstacle2_corner([t2.Seg("L", 90)], Mock()))
        route._line.assert_not_called()

    def test_rst_wait_uses_the_final_stationary_pose(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, sleep_fn=lambda _: None)
        poses = [Pose(150, -10, math.radians(-85)),
                 Pose(150, -18, math.radians(-85)),
                 Pose(150, -23.5, math.radians(-85)),
                 Pose(150.1, -23.6, math.radians(-85)),
                 Pose(150.1, -23.6, math.radians(-85))]
        route._pose = Mock(side_effect=poses)
        self.assertEqual(route._wait_for_stopped_pose(), poses[-1])

    def test_logged_right_edge_has_forward_only_corner(self):
        """Regression for the 18:43 run: R2 must not precede this corner."""
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0
        near = -24.3
        width = task2.W2_RANGE[1] + 2 * task2.END_MARGIN_CM
        course = t2.Course(
            0.0, 181.6, width,
            wall_y=width / 2.0 + t2.WALL_CLEARANCE_CM
                   - 2 * task2.END_MARGIN_CM,
            obs2_y=near - task2.END_MARGIN_CM + width / 2.0)
        stopped = Pose(149.98, -23.62, math.radians(-84.9))
        segs = route.round_near_end(
            stopped, course, -1.0, False, near, forward_only=True)
        self.assertIsNotNone(segs)
        self.assertFalse(any(seg.kind == "B" for seg in segs), segs)
        self.assertEqual(segs[0].kind, "L")
        gap, _ = t2.path_clearance(t2.run(stopped, segs, route.r),
                                   course.boxes())
        self.assertGreaterEqual(gap, t2.CLEARANCE_CM)

    def test_latest_logged_right_edge_uses_checked_clearance_floor(self):
        """The 19:31 run cleared the IR edge; its 5 cm turn must not stall."""
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0
        near = -43.3
        width = task2.W2_RANGE[1] + 2 * task2.END_MARGIN_CM
        near_y = near - task2.END_MARGIN_CM
        course = t2.Course(
            0.0, 183.1, width,
            wall_y=width / 2.0 + t2.WALL_CLEARANCE_CM
                   - 2 * task2.END_MARGIN_CM,
            obs2_y=near_y + width / 2.0)
        stopped = Pose(149.8, -41.5, math.radians(-90.7))

        segs = route.round_near_end(
            stopped, course, -1.0, False, near, forward_only=True)

        self.assertIsNotNone(segs)
        self.assertEqual([seg.token() for seg in segs],
                         ["FL90", "F8", "FL90"])
        self.assertFalse(any(seg.kind == "B" for seg in segs))
        gap, _ = t2.path_clearance(t2.run(stopped, segs, route.r),
                                   course.boxes())
        self.assertGreaterEqual(gap, t2.CLEARANCE_FLOOR_CM)
        self.assertLess(gap, t2.CLEARANCE_CM)

        measured = [stopped]
        current = stopped
        for seg in segs:
            current = t2.run(current, [seg], route.r)[-1]
            measured.extend([current, current])
        route._set_fast = Mock()
        route.fast = False
        route._pose = Mock(side_effect=measured[:-1])
        route._line = Mock(return_value=True)

        self.assertTrue(route.execute_obstacle2_corner(segs, course))
        self.assertEqual(route._line.call_args_list,
                         [call([seg]) for seg in segs])

    def test_remaining_sweep_is_rechecked_after_small_pose_error(self):
        route = self.make_route()
        segs = [t2.Seg("L", 90), t2.Seg("L", 90)]
        p = Pose(0, 0, 0)
        expected = t2.run(p, segs[:1], route.r)[-1]
        measured = Pose(expected.x + 1, expected.y, expected.th)
        route._pose = Mock(side_effect=[p, measured, measured])
        course = Mock()
        course.boxes.return_value = []
        with patch.object(t2, "path_clearance", side_effect=[(8, "box"), (2, "box")]):
            self.assertFalse(route.execute_obstacle2_corner(segs, course))
        route._line.assert_called_once_with(segs[:1])


class ForwardOnlyHandoff(unittest.TestCase):
    def test_front_face_ir_route_precedes_direct_outer_route(self):
        import task2t as task2
        for side in (-1, 1):
            with self.subTest(side=side):
                route = task2.Task2(Mock(), startup=False)
                front = [t2.Seg("L" if side > 0 else "R", 90)]
                outer = [t2.Seg("F", 20), t2.Seg("R" if side > 0 else "L", 90)]
                route.forward_front_candidates = Mock(return_value=iter([front]))
                route.same_side_forward_candidates = Mock(return_value=iter([outer]))
                route.same_side_lane_candidates = Mock(return_value=iter([]))
                candidates = route._turn_handoff_candidates(Pose(0, 0, 0),
                                                            Course(60, 130, 90), side, side)
                self.assertEqual(next(candidates), (front, False))
                route.same_side_forward_candidates.assert_not_called()
                self.assertEqual(next(candidates), (outer, False))

    def test_early_same_direction_routes_complete_narrow_and_wide(self):
        import task2t as task2
        import task2t_sim as sim
        for width in (30, 90):
            for arrow in ("left", "right"):
                with self.subTest(width=width, arrow=arrow):
                    course = Course(60, 130, width)
                    frame = t2.WorldFrame(None)
                    stm = sim.FakeSTM(course, frame.start, t2.RADIUS_CM)
                    route = task2.Task2(stm, startup=False, wait_for_go=False,
                        sleep_fn=lambda _: None, input_fn=lambda _: arrow,
                        early_arrow_fn=lambda: arrow)
                    with patch.object(stm, "send_line", wraps=stm.send_line) as send:
                        self.assertEqual(route.run(), 0)
                        tokens = [token for call in send.call_args_list
                                  for token in call.args[0].split(",")]
                        self.assertFalse(any(token.startswith("R") for token in tokens), tokens)
                        self.assertFalse(any(token.startswith("FU") for token in tokens))
                    self.assertTrue(_parked_ok(stm.pose))
                    self.assertGreaterEqual(sim.run_clearance(stm, course)[0],
                                            t2.CLEARANCE_FLOOR_CM)

    def test_inflight_lane_detection_avoids_photo_move(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, input_fn=lambda _: "left",
                           sleep_fn=Mock(), early_arrow_fn=Mock(side_effect=[None, "left"]))
        route._pose = Mock(return_value=Pose(50, 21, 0))
        route._line = Mock()
        self.assertEqual(route.photo_obstacle2(Course(80, 150, 90), 1), 1)
        route._line.assert_not_called()
        route.sleep_fn.assert_called_once_with(0.1)

    def test_photo_vote_finishing_after_path_check_sends_no_motion(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, input_fn=lambda _: "right",
                           sleep_fn=lambda _: None,
                           early_arrow_fn=Mock(
                               side_effect=[None, None, None, None, "right"]))
        route._pose = Mock(return_value=Pose(55.8, 18.8,
                                             math.radians(-1.6)))
        segs = [t2.Seg("B", 6), t2.Seg("L", 45)]
        with patch.object(route, "photo_candidates",
                          return_value=[(segs, 34, 45)]), \
                patch.object(route, "_first_clear", return_value=segs), \
                patch.object(task2.tp, "run_polling") as polling:
            self.assertEqual(route.photo_obstacle2(
                Course(85.7, 151.0, 90), 1), -1)
        polling.assert_not_called()
        route.stm.send_line.assert_not_called()

    def test_logged_obstacle1_touch_uses_larger_turn_margin(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0
        pose = Pose(17.8, 2.8, math.radians(-1.5))
        course = Course(85.7, 155.7, 90,
                        wall_y=task2.CONSERVATIVE_WALL_Y)
        route._pose = Mock(return_value=pose)
        route._line = Mock(return_value=False)

        self.assertIsNone(route.pass_obstacle1(course.x1, 1.0))
        sent = route._line.call_args.args[0]
        self.assertEqual(sent[0].kind, "B")
        self.assertGreaterEqual(route._check(pose, sent, course),
                                task2.OBS1_TURN_CLEARANCE_CM)

    def test_sensing_lane_uses_confirmed_miss_as_width_bound(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.obs2_us_y = 21
        route._pose = Mock(return_value=Pose(50, 21, 0))
        route._line = Mock(return_value=True)
        route._us = Mock(return_value=None)
        self.assertTrue(route.prepare_forward_lane(Course(80, 150, 90), 1))
        self.assertEqual(route.obs2_us_y, 21)
        self.assertIsNotNone(route.obs2_half_max)
        route._line.assert_called_once()
        route._line.reset_mock()
        route._check = Mock(return_value=0)
        self.assertFalse(route.prepare_forward_lane(Course(80, 150, 90), 1))
        route._line.assert_not_called()

    def test_same_direction_candidates_never_request_reverse(self):
        import task2t as task2
        for side in (1, -1):
            route = task2.Task2(Mock(), startup=False)
            route.obs2_us_y = side * 40
            choices = list(route.face_handoff_candidates(
                Pose(60, -side * 20, 0), Course(80, 170, 90, wall_y=95), side, side))
            self.assertTrue(choices)
            for segs, reverse in choices:
                self.assertFalse(reverse)
                self.assertTrue(all(s.kind in ("F", "L", "R") for s in segs))

    def test_front_approach_is_checked_and_faces_requested_end(self):
        import task2t as task2
        for side in (1, -1):
            route = task2.Task2(Mock(), startup=False)
            route.obs2_us_y = side * 40
            p = Pose(60, -side * 20, 0)
            course = Course(80, 170, 90, wall_y=95)
            selected = route.select_face_handoff(p, course, side, side)
            self.assertIsNotNone(selected)
            segs, reverse, clearance = selected
            end = t2.run(p, segs, route.r)[-1]
            self.assertGreater(side * math.sin(end.th), 0.99)
            self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)
            self.assertFalse(reverse)
            route.stm.send_line.assert_not_called()

    def test_legacy_reverse_selection_cannot_be_executed(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route._pose = Mock(return_value=Pose(65, 21, 0))
        route.select_face_handoff = Mock(return_value=([t2.Seg("R", 90)], True, 8))
        route._line = Mock()
        self.assertIsNone(route.around_obstacle2(Course(84, 165, 90), 1, 1))
        route._line.assert_not_called()

    def test_front_guard_does_not_report_an_edge_or_start_motion(self):
        import task2t as task2
        for reading in (None, float("nan"), 0, 19, 20):
            route = task2.Task2(Mock(), startup=False, sleep_fn=Mock())
            route._pose = Mock(return_value=Pose(100, 0, math.pi / 2))
            route._us = Mock(return_value=reading)
            route._ir = Mock(return_value=20)
            with patch.object(t2, "run_polling") as run:
                self.assertIsNone(route.follow_face("F60", 130, 1, 1, 18, check_front=True))
                run.assert_not_called()

    def test_front_route_tracks_edge_before_planning_forward_corner(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route._pose = Mock(side_effect=[Pose(60, -20, 0)] +
                           [Pose(144, 10, math.pi / 2)] * 4)
        route.select_face_handoff = Mock(return_value=(
            [t2.Seg("F", 54), t2.Seg("L", 90)], False, 5.6))
        route._line = Mock(return_value=True)
        route._set_fast = Mock()
        route.follow_face = Mock(return_value=(45, 170))
        route.round_near_end = Mock(return_value=None)
        self.assertIsNone(route.around_obstacle2(Course(80, 170, 90), 1, 1))
        args, kwargs = route.follow_face.call_args
        self.assertTrue(args[0].startswith("F"))
        self.assertEqual(args[1:4], (170, 1, 1))
        self.assertTrue(kwargs["check_front"])
        self.assertTrue(route.round_near_end.call_args.kwargs["forward_only"])
        self.assertEqual(route._line.call_count, 1)

    def test_sensor_front_route_completes_both_directions_without_reverse(self):
        import task2t as task2
        import task2t_sim as sim
        for side in (1, -1):
            course = Course(80, 170, 90, wall_y=95)
            frame = t2.WorldFrame(None)
            stm = sim.FakeSTM(course, frame.start, t2.RADIUS_CM)
            route = task2.Task2(stm, startup=False, sleep_fn=lambda _: None)
            self.assertTrue(route.frame.anchor())
            stm.pose = Pose(60, -side * 20, 0)
            stm.trace = [stm.pose]
            route.obs2_us_y = side * 40
            with patch.object(stm, "send_line", wraps=stm.send_line) as send:
                self.assertIsNotNone(route.around_obstacle2(course, side, side))
                tokens = [token for call in send.call_args_list
                          for token in call.args[0].split(",")]
                self.assertFalse(any(token.startswith("R") for token in tokens), tokens)
            self.assertGreaterEqual(sim.run_clearance(stm, course)[0],
                                    t2.CLEARANCE_FLOOR_CM)


class EndToEnd(unittest.TestCase):
    def test_forward_alternative_precedes_original_pose_reverse(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        p = Pose(60, 25, math.radians(-21))
        fallback = [t2.Seg("R", 69)]
        forward = [t2.Seg("F", 12), t2.Seg("R", 90)]
        with patch.object(route, "_turn_handoff_candidates",
                          side_effect=[iter([(fallback, True)]),
                                       iter([(forward, False)])]):
            choices = list(route.face_handoff_candidates(p, Course(80, 165, 90), 1, 1))
        self.assertEqual(choices[0], ([t2.Seg("L", 21)] + forward, False))
        self.assertEqual(choices[1], (fallback, True))

    def test_handoff_combines_straights_without_changing_endpoint(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        p = Pose(60, 25, 0)
        lead = [t2.Seg("F", 10), t2.Seg("F", 12), t2.Seg("R", 90)]
        with patch.object(route, "_turn_handoff_candidates",
                          side_effect=lambda *args: iter([(lead, False), (lead, False)])):
            choices = list(route.face_handoff_candidates(p, Course(80, 165, 90), 1, 1))
        self.assertEqual(choices[0][0], [t2.Seg("F", 22), t2.Seg("R", 90)])
        self.assertEqual(t2.run(p, choices[0][0])[-1], t2.run(p, lead)[-1])
        self.assertEqual(len(choices), len(set(tuple(s) for s, _ in choices)))

    def test_interrupted_right_photo_turn_does_not_force_unsafe_recovery(self):
        import task2t as task2
        for mirror in (1, -1):
            route = task2.Task2(Mock(), startup=False)
            route.r = 30
            route.obs2_us_y = -22.11256875043172 * mirror
            p = Pose(65.568201025859, -25.42016366743875 * mirror,
                     math.radians(-21 * mirror))
            course = Course(79.7, 162.6, 90, wall_y=65)
            side = -mirror
            selected = route.select_face_handoff(p, course, side, side)
            self.assertIsNone(selected)
            route.stm.send_line.assert_not_called()

    def test_heading_recovery_is_rejected_when_its_prefix_collides(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        p = Pose(65.6, -25.4, math.radians(-21))
        course = Course(79.7, 162.6, 90, wall_y=65)
        route.obs2_us_y = -22.1
        # Force every complete sweep, including the new prefix, to fail.
        with patch.object(route, "_handoff_clearance", return_value=-1):
            self.assertIsNone(route.select_face_handoff(p, course, -1, -1))
        route.stm.send_line.assert_not_called()

    def _face_probe(self, readings):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, sleep_fn=Mock())
        p = Pose(150, 0, math.pi / 2)
        route._pose = Mock(return_value=p)
        route.frame.pose = Mock(return_value=p)
        route._ir = Mock(side_effect=readings)
        return route

    def test_face_acquired_at_rest_can_confirm_edge_after_ok(self):
        route = self._face_probe([19, 19, None, None, None])
        def completed(stm, token, poll):
            self.assertFalse(poll())
            return "OK"
        with patch.object(t2, "run_polling", side_effect=completed) as moving:
            self.assertIsNotNone(route.follow_face("F10", 126, -1, 1, 12))
        moving.assert_called_once()
        route.stm.send_line.assert_not_called()
        route.stm.abort.assert_not_called()
        self.assertEqual(route._ir.call_count, 5)

    def test_logged_front_face_28cm_reading_triggers_edge_and_stop(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, sleep_fn=lambda _: None)
        x_face = 156.0
        pose = Pose(x_face - t2.IR_SIDE_OFFSET_CM - 22.0, 58.3,
                    -math.pi / 2)
        route._pose = Mock(return_value=pose)
        route.frame.pose = Mock(return_value=pose)
        route._wait_for_stopped_pose = Mock(return_value=pose)
        # The floor run held 27-29 cm across the block, rose to 36 cm at its
        # edge, then went blank. Three out-of-window samples must stop F39.
        route._ir = Mock(side_effect=[28, 28, 28, 36, 36, 36])

        def edge(stm, token, poll):
            self.assertFalse(poll())
            self.assertFalse(poll())
            self.assertFalse(poll())
            self.assertTrue(poll())
            return "STOPPED"

        with patch.object(t2, "run_polling", side_effect=edge):
            measured = route.follow_face("F39", x_face, +1, -1, 18)
        self.assertIsNotNone(measured)
        route.stm.abort.assert_called_once()
        route._wait_for_stopped_pose.assert_called_once()

    def test_face_single_flash_is_not_confirmed(self):
        route = self._face_probe([19, None, None])
        def completed(stm, token, poll):
            self.assertFalse(poll())
            return "OK"
        with patch.object(t2, "run_polling", side_effect=completed):
            self.assertIsNone(route.follow_face("F10", 126, -1, 1, 12))
        self.assertIsNone(route.last_face_start_y)

    def test_face_still_visible_after_ok_is_not_an_edge(self):
        route = self._face_probe([19] * 5)
        with patch.object(t2, "run_polling", return_value="OK") as moving:
            self.assertIsNone(route.follow_face("F10", 126, -1, 1, 12))
        moving.assert_called_once()
        self.assertEqual(route._ir.call_count, 5)
        route.stm.send_line.assert_not_called()

    def test_face_wall_guard_still_rejects_scan(self):
        route = self._face_probe([19, 19, None])
        def blocked(stm, token, poll):
            route.frame.pose.return_value = Pose(150, 80, math.pi / 2)
            self.assertTrue(poll())
            return "STOPPED"
        with patch.object(t2, "run_polling", side_effect=blocked):
            self.assertIsNone(route.follow_face("F80", 126, -1, 1, 12))
        route.stm.abort.assert_called_once()

    def test_handoff_prefers_wider_checked_candidate(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        short, wide = [t2.Seg("F", 10)], [t2.Seg("F", 12)]
        with patch.object(route, "face_handoff_candidates",
                          return_value=iter([(short, False), (wide, False)])), \
                patch.object(route, "_handoff_clearance", side_effect=[4.0, 9.0]):
            chosen = route.select_face_handoff(Pose(80, 25, 0), Course(84, 163, 90), 1, 1)
        self.assertEqual(chosen, (wide, False, 9.0))

    def test_equal_clearance_does_not_add_a_needless_alignment_turn(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        simple = [t2.Seg("F", 20), t2.Seg("R", 85)]
        extra = [t2.Seg("L", 5), t2.Seg("F", 14), t2.Seg("R", 90)]
        with patch.object(route, "face_handoff_candidates",
                          return_value=iter([(simple, True), (extra, True)])), \
                patch.object(route, "_handoff_clearance",
                             side_effect=[4.550741155910053, 4.550741155910055]):
            result = route.select_face_handoff(Pose(85, 19, 0), Course(78, 159, 90), 1, 1)
        self.assertEqual(result[0], simple)

    def test_handoff_timeout_retains_only_fully_checked_candidate(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        safe, unchecked = [t2.Seg("F", 10)], [t2.Seg("F", 12)]
        with patch.object(route, "face_handoff_candidates",
                          return_value=iter([(safe, False), (unchecked, False)])), \
                patch.object(route, "_handoff_clearance", side_effect=[4.0, TimeoutError]):
            chosen = route.select_face_handoff(Pose(80, 25, 0), Course(84, 163, 90), 1, 1)
        self.assertEqual(chosen, (safe, False, 4.0))
        route.stm.send_line.assert_not_called()

    def test_logged_interrupted_pose_does_not_advance_without_clear_route(self):
        import task2t as task2
        for side in (1.0, -1.0):
            route = task2.Task2(Mock(), startup=False)
            route.r = 30.0
            route.frame.origin = dict(x_mm=-9, y_mm=1373, heading_deg=64.3)
            route.stm.read_wpose.return_value = dict(
                x_mm=220, y_mm=2404, heading_deg=64.2)
            route.obs2_us_y = side * route.frame.pose().y
            route.stm.read_wpose.return_value = dict(
                x_mm=196, y_mm=2354, heading_deg=64.8)
            measured = route.frame.pose()
            p = Pose(measured.x, side * measured.y, side * measured.th)
            course = Course(81.7, 162.5, 90, wall_y=65)
            selected = route.select_face_handoff(p, course, side, side)
            self.assertIsNone(selected)
            route.stm.send_line.assert_not_called()

    def test_second_turn_failed_search_does_not_move(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        original = Pose(65, 21, 0)
        route._pose = Mock(return_value=original)
        route._line = Mock()
        route.select_face_handoff = Mock(return_value=None)
        course = Course(84, 165, 90)
        self.assertIsNone(route.around_obstacle2(course, 1, 1))
        route.select_face_handoff.assert_called_once_with(original, course, 1, 1)
        route._line.assert_not_called()

    def test_second_turn_sends_complete_selected_plan(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        original = Pose(65, 21, 0)
        route._pose = Mock(return_value=original)
        selected = [t2.Seg("F", 12), t2.Seg("R", 90)]
        route.select_face_handoff = Mock(return_value=(selected, True, 8.0))
        route._line = Mock(return_value=False)
        self.assertIsNone(route.around_obstacle2(Course(84, 165, 90), 1, 1))
        route._line.assert_not_called()

    def _narrow_probe(self, readings):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, sleep_fn=Mock())
        route._pose = Mock(return_value=Pose(160, 43.4, 0))
        route.frame.pose = Mock(return_value=Pose(170, 43.4, 0))
        route._ir = Mock(side_effect=readings)
        route._first_clear = Mock(return_value=[])
        route._line = Mock(return_value=True)
        return route

    def test_narrow_first_hit_brakes_then_confirms_without_long_sleep(self):
        route = self._narrow_probe([19, 19, 19])
        def scan(stm, line, poll):
            self.assertTrue(poll())
            return "STOPPED"
        with patch.object(t2, "run_polling", side_effect=scan):
            face = route.find_narrow(90, 1)
        self.assertEqual(face, 170 + t2.IR_FROM_AXLE_CM - t2.IR_LAG_CM)
        route.stm.abort.assert_called_once()
        self.assertEqual(route._ir.call_count, 3)
        self.assertAlmostEqual(sum(c.args[0] for c in route.sleep_fn.call_args_list),
                               0.16)

    def test_narrow_false_hit_resumes_only_remaining_distance(self):
        route = self._narrow_probe([19, None, None, None, None, 19, 19, 19])
        route._pose.side_effect = [Pose(150, 43.4, 0), Pose(160, 43.4, 0),
                                  Pose(170, 43.4, 0)]
        route.frame.pose.side_effect = [Pose(165, 43.4, 0), Pose(175, 43.4, 0)]
        lines = []
        def scan(stm, line, poll):
            lines.append(line)
            self.assertTrue(poll())
            return "STOPPED"
        with patch.object(t2, "run_polling", side_effect=scan):
            face = route.find_narrow(90, 1)
        self.assertEqual(face, 175 + t2.IR_FROM_AXLE_CM - t2.IR_LAG_CM)
        self.assertEqual(int(lines[0][1:]) - int(lines[1][1:]), 10)
        self.assertEqual(route.stm.abort.call_count, 2)

    def test_narrow_unconfirmed_hits_have_bounded_retries(self):
        route = self._narrow_probe([19, None, None, None, None] * 3)
        def scan(stm, line, poll):
            self.assertTrue(poll())
            return "STOPPED"
        with patch.object(t2, "run_polling", side_effect=scan) as polling:
            self.assertIsNone(route.find_narrow(90, 1))
        self.assertEqual(polling.call_count, 3)
        self.assertIsNone(route.obs2_near_y)

    def test_logged_unknown_width_uses_forward_lane_then_turn(self):
        import task2t as task2
        for side in (1.0, -1.0):
            with self.subTest(side=side):
                route = task2.Task2(Mock(), startup=False)
                route.r = 30.0
                route.obs2_us_y = side * 26.44408467668212
                pose = Pose(86.0, side * 26.8, math.radians(-side * 0.5))
                course = Course(83.7, 162.56, 90.0, wall_y=65.0)
                selected = route.select_face_handoff(pose, course, side, side)
                self.assertIsNotNone(selected)
                segs, reverse, clearance = selected
                self.assertFalse(reverse)
                self.assertTrue(all(s.kind in ("F", "L", "R") for s in segs))
                self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)
                trace = t2.run(pose, segs, route.r)
                for checked in route.same_side_courses(course):
                    actual = t2.path_clearance(trace, checked.boxes())[0]
                    self.assertGreaterEqual(actual, t2.CLEARANCE_FLOOR_CM)
                route.stm.send_line.assert_not_called()

    def test_confirmed_photo_does_not_search_or_reposition(self):
        import task2t as task2
        route = task2.Task2(Mock(), input_fn=lambda _: "left",
                           early_arrow_fn=lambda: "left", startup=False)
        route._pose = Mock(return_value=Pose(86, 27, 0))
        with patch.object(route, "face_handoff_candidates") as handoff, \
                patch.object(route, "photo_candidates") as photo:
            self.assertEqual(route.photo_obstacle2(Course(84, 163, 90), 1), 1)
        handoff.assert_not_called()
        photo.assert_not_called()
        route.stm.send_line.assert_not_called()

    def test_handoff_timeout_never_sends_partial_plan(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        candidate = [t2.Seg("F", 10)]
        with patch.object(route, "face_handoff_candidates",
                          return_value=iter([(candidate, False)])), \
                patch.object(task2.time, "monotonic", side_effect=[0.0, 100.0]):
            result = route.select_face_handoff(Pose(80, 25, 0),
                                               Course(84, 163, 90), 1, 1)
        self.assertIsNone(result)
        route.stm.send_line.assert_not_called()

    def test_handoff_clearance_matches_original_for_valid_trace(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0
        route.obs2_us_y = 26.44408467668212
        course = Course(83.7, 162.56, 90.0, wall_y=65.0)
        pose = Pose(86, 26.8, math.radians(-0.5))
        segs = [t2.Seg("L", 30), t2.Seg("F", 48), t2.Seg("R", 30),
                t2.Seg("F", 16), t2.Seg("R", 90)]
        expected = route.face_handoff_clearance(pose, segs, course, 1, 1, False)
        actual = route._handoff_clearance(pose, segs,
                                         list(route.same_side_courses(course)),
                                         float("inf"))
        self.assertAlmostEqual(actual, expected)

    def test_outer_lane_unknown_width_does_not_invent_odometry_shift(self):
        import task2t as task2
        import task2t_sim as sim
        for side in (1.0, -1.0):
            for width in (54.0, 90.0):
                with self.subTest(side=side, width=width):
                    actual = Course(83.7, 162.56, width)
                    fake = sim.FakeSTM(actual, t2.WorldFrame(None).start, 30.0)
                    route = task2.Task2(fake, startup=False, sleep_fn=lambda _: None)
                    route.r = 30.0
                    route.frame.anchor()
                    pose = Pose(86.0, side * 26.8, math.radians(-side * 0.5))
                    fake.pose, fake.trace, route.th = pose, [pose], pose.th
                    route.obs2_us_y = side * 26.44408467668212
                    with patch("time.sleep"):
                        measured = route.around_obstacle2(
                            Course(83.7, 162.56, 90.0, wall_y=65.0), side, side)
                    self.assertIsNotNone(measured)
                    self.assertEqual(route.frame.y_shift, 0.0)
                    self.assertGreaterEqual(sim.run_clearance(fake, actual)[0],
                                            t2.CLEARANCE_FLOOR_CM)

    def test_interrupted_photo_does_not_repeat_search(self):
        import task2t as task2
        route = task2.Task2(Mock(), input_fn=lambda _: "right", startup=False,
                           sleep_fn=lambda _: None, early_arrow_fn=Mock(
                               side_effect=[None, None, None, None, None, "right"]))
        route._pose = Mock(return_value=Pose(86, 27, 0))
        route.frame.pose = Mock(return_value=Pose(90, 27, 0))
        segs = [t2.Seg("F", 10)]
        def interrupt(stm, line, poll):
            self.assertTrue(poll())
            return "STOPPED"
        with patch.object(route, "photo_candidates", return_value=[(segs, 30, 45)]), \
                patch.object(route, "_first_clear", return_value=segs), \
                patch.object(route, "face_handoff_candidates") as search, \
                patch.object(t2, "run_polling", side_effect=interrupt):
            self.assertEqual(route.photo_obstacle2(Course(84, 163, 90), 1), -1)
        search.assert_not_called()
        route.stm.abort.assert_called_once()

    def test_opposite_arrows_offer_checked_diagonal_handoff(self):
        import task2t as task2
        for side in (1.0, -1.0):
            with self.subTest(side=side):
                route = task2.Task2(Mock(), startup=False)
                route.r = 30.0
                route.obs2_near_y = side * 45.0
                pose, course = Pose(86, side * 60, 0), Course(83.7, 162.56, 90)
                segs, reverse, clearance = route.select_face_handoff(
                    pose, course, side, -side)
                self.assertFalse(reverse)
                turn = "R" if side > 0 else "L"
                self.assertEqual([s.kind for s in segs], ["F", turn, "F", turn])
                trace = t2.run(pose, segs, route.r)
                self.assertAlmostEqual(trace[-1].th, -side * math.pi / 2)
                self.assertGreaterEqual(t2.path_clearance(trace, course.boxes())[0],
                                        t2.CLEARANCE_FLOOR_CM)
    """task2t.Task2 against task2t_sim.FakeSTM: ray-cast sensors, the arc model
    on a radius the planner may have wrong, IR readings that lag."""

    CASES = [
        # d1, d2, w2, arrow, radius error
        (60, 60, 90, "right", 0),      # everything at its tightest, widest obs 2
        (60, 60, 30, "left", 0),       # narrow: ultrasound misses it from the lane
        (60, 60, 44, "left", 0),       # just past the narrow cut-off
        (150, 150, 90, "left", 0),
        (90, 100, 50, "right", 2),     # TIGHT wider than T2T_RADIUS_CM
        (60, 60, 40, "left", -2),      # and tighter
        (150, 60, 30, "right", -2),
    ]
    # The first floor run: ultrasound read obstacle 2 11 cm short from the
    # obstacle 1 lane.
    FLOOR = [(90, 82, 38, "left", 0, 11), (60, 60, 90, "right", -2, 11)]

    def test_same_side_handoff_prefers_symmetric_forward_sweeps(self):
        """Matching arrows use the open side instead of reversing by default."""
        import task2t as task2

        class NoWire:
            pass

        route = task2.Task2(NoWire(), startup=False, wait_for_go=False)
        course = t2.Course(85.7, 161.1, 90.0, wall_y=65.0)
        for side, turn in ((1.0, "FL"), (-1.0, "FR")):
            with self.subTest(side=side):
                route.obs2_us_y = side * 23.2
                route.obs2_near_y = side * 25.0
                pose = t2.Pose(91.0, side * 23.2,
                               math.radians(-side * 3.3))
                selected = route.select_face_handoff(
                    pose, course, side, side)
                self.assertIsNotNone(selected)
                segments, reverse, clearance = selected
                self.assertFalse(reverse)
                self.assertTrue(any(segment.token().startswith(turn)
                                    for segment in segments))
                self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)
                trace = t2.run(pose, segments, route.r)
                actual = min(t2.path_clearance(trace, checked.boxes())[0]
                             for checked in route.same_side_courses(course))
                self.assertAlmostEqual(clearance, actual)

    def test_same_side_handoff_keeps_reverse_fallback_when_sweep_is_blocked(self):
        """A blocked forward sweep must not silently restore reverse."""
        import task2t as task2

        class NoWire:
            pass

        route = task2.Task2(NoWire(), startup=False, wait_for_go=False)
        course = t2.Course(60.0, 180.0, 90.0, wall_y=65.0)
        pose = t2.Pose(120.0, 38.0, math.radians(-35.0))
        selected = route.select_face_handoff(pose, course, 1.0, 1.0)
        self.assertIsNone(selected)

    def test_outward_us_miss_bounds_width_and_completes_medium_box(self):
        """A 44 cm box is wider than narrow-mode but misses the outer US ray."""
        import task2t_sim as ts

        code, fake, course = ts.simulate(60, 60, 44, "left")

        self.assertEqual(code, 0)
        clearance, obstacle = ts.run_clearance(fake, course)
        self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM - 0.5,
                                obstacle)
        self.assertTrue(_parked_ok(fake.pose), fake.pose)

    def test_measured_width_completes_symmetric_outer_eight_without_reverse(self):
        """With the near edge known, the LL/RR floor pose is all-forward."""
        import task2t as task2
        import task2t_sim as ts

        class RecordingSTM(ts.FakeSTM):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.lines = []

            def send_line(self, line):
                self.lines.append(line)
                return super().send_line(line)

        for side in (1.0, -1.0):
            with self.subTest(side=side):
                actual = t2.Course(85.7, 161.1, 50.0, wall_y=75.0)
                frame = t2.WorldFrame(None)
                fake = RecordingSTM(actual, frame.start, t2.RADIUS_CM)
                route = task2.Task2(fake, sleep_fn=lambda _: None,
                                    startup=False, wait_for_go=False)
                self.assertTrue(route.frame.anchor())
                pose = t2.Pose(91.0, side * 23.2,
                               math.radians(-side * 3.3))
                fake.pose = pose
                fake.trace = [pose]
                route.th = pose.th
                route.obs2_us_y = side * 23.2
                route.obs2_near_y = side * actual.w2 / 2.0

                planned = t2.Course(85.7, 161.1, 90.0, wall_y=65.0)
                measured = route.around_obstacle2(planned, side, side)

                self.assertIsNotNone(measured)
                self.assertAlmostEqual(measured.w2, actual.w2, delta=8.0)
                self.assertGreaterEqual(ts.run_clearance(fake, actual)[0],
                                        t2.CLEARANCE_FLOOR_CM)
                tokens = ",".join(fake.lines).split(",")
                self.assertFalse(any(token.startswith("R")
                                     and token[1:].isdigit()
                                     and int(token[1:]) > 20
                                     for token in tokens), fake.lines)

    def test_obstacle1_fu_does_not_chase_a_changed_echo(self):
        """A floor run lost obstacle 1 after F111 and FU chased a farther wall."""
        import task2t as task2
        import task2t_sim as ts

        course = _course(150, 80, 50)

        class ChangedEcho(ts.FakeSTM):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.lines = []
                self.us_queries = 0

            def send_line(self, line):
                self.lines.append(line)
                return super().send_line(line)

            def query_fields(self, command, timeout=2.0):
                if command.upper().lstrip("?") == "US":
                    self.us_queries += 1
                    if self.us_queries == 2:
                        return ["180"]
                return super().query_fields(command, timeout)

        frame = t2.WorldFrame(None)
        fake = ChangedEcho(course, frame.start, t2.RADIUS_CM)
        route = task2.Task2(fake, input_fn=lambda _: self.fail("arrow requested"),
                            sleep_fn=lambda _: None, startup=False,
                            wait_for_go=False)
        self.assertTrue(route.frame.anchor())
        self.assertIsNone(route.obstacle1())
        self.assertTrue(any(line.startswith("F") for line in fake.lines))
        self.assertFalse(any("FU" in line for line in fake.lines))

    def test_obstacle1_closer_echo_reanchors_without_advancing_again(self):
        """A closer face at the turn standoff is obstacle 1, not a far wall."""
        import task2t as task2
        import task2t_sim as ts

        course = _course(150, 80, 50)

        class CloserEcho(ts.FakeSTM):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.lines = []
                self.us_queries = 0

            def send_line(self, line):
                self.lines.append(line)
                return super().send_line(line)

            def query_fields(self, command, timeout=2.0):
                if command.upper().lstrip("?") == "US":
                    self.us_queries += 1
                    if self.us_queries == 2:
                        return ["23"]
                return super().query_fields(command, timeout)

        frame = t2.WorldFrame(None)
        fake = CloserEcho(course, frame.start, t2.RADIUS_CM)
        route = task2.Task2(fake, input_fn=lambda _: "left",
                            sleep_fn=lambda _: None, startup=False,
                            wait_for_go=False)
        self.assertTrue(route.frame.anchor())

        hit = route.obstacle1()

        self.assertIsNotNone(hit)
        self.assertEqual(hit[1], 1.0)
        self.assertEqual(len([line for line in fake.lines
                              if line.startswith("F")]), 1)
        self.assertAlmostEqual(
            hit[0], route._front(route._pose()) + 23.0 - t2.US_BIAS_CM,
            delta=0.2)

    def test_confirmed_arrow_takes_first_clear_approach_turn(self):
        """A ready vision result skips the remaining straight and FU."""
        import task2t as task2
        import task2t_sim as ts

        course = _course(150, 80, 50)

        class RecordingSTM(ts.FakeSTM):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.lines = []

            def send_line(self, line):
                self.lines.append(line)
                return super().send_line(line)

        frame = t2.WorldFrame(None)
        fake = RecordingSTM(course, frame.start, t2.RADIUS_CM)
        answers = []

        def consume(_):
            answers.append("left")
            return "left"

        route = task2.Task2(fake, input_fn=consume, sleep_fn=lambda _: None,
                            startup=False, wait_for_go=False,
                            early_arrow_fn=lambda: "left")
        self.assertTrue(route.frame.anchor())
        hit = route.obstacle1()
        self.assertIsNotNone(hit)
        self.assertEqual(hit[1], 1.0)
        self.assertEqual(answers, ["left"])
        forward = [line for line in fake.lines if line.startswith("F")]
        self.assertEqual(len(forward), 1)
        self.assertFalse(any("FU" in line for line in fake.lines))

    def test_early_arrow_route_rejoins_and_parks(self):
        """An early lane change must rejoin the normal obstacle-2 route."""
        import task2t as task2
        import task2t_sim as ts

        # Geometry measured in the 17:10 floor run that previously failed at
        # the obstacle-2 photo handoff after a successful early lane change.
        course = t2.Course(132.7, 240.7, 50.0)
        frame = t2.WorldFrame(None)
        fake = ts.FakeSTM(course, frame.start, t2.RADIUS_CM)
        route = task2.Task2(fake, input_fn=lambda _: "left",
                            sleep_fn=lambda _: None, startup=False,
                            wait_for_go=False,
                            early_arrow_fn=lambda: "left")

        self.assertEqual(route.run(), 0)
        clearance, obstacle = ts.run_clearance(fake, course)
        self.assertGreater(clearance, t2.CLEARANCE_FLOOR_CM,
                           "route came too close to %s" % obstacle)
        self.assertTrue(_parked_ok(fake.pose), fake.pose)

    def test_runs_and_parks(self):
        import task2t_sim as ts
        for d1, d2, w2, arrow, rerr in self.CASES:
            case = (d1, d2, w2, arrow, rerr)
            code, fake, course = ts.simulate(d1, d2, w2, arrow, rerr)
            self.assertEqual(code, 0, case)
            clear, name = ts.run_clearance(fake, course)
            self.assertGreater(clear, t2.CLEARANCE_FLOOR_CM - 0.5,
                               f"{case}: {clear:.1f} cm off {name}")
            self.assertTrue(_parked_ok(fake.pose), f"{case}: {fake.pose}")

    def test_ultrasound_short_on_obstacle2(self):
        import task2t_sim as ts
        for d1, d2, w2, arrow, rerr, us_err in self.FLOOR:
            case = (d1, d2, w2, arrow, rerr, us_err)
            code, fake, course = ts.simulate(d1, d2, w2, arrow, rerr,
                                             us_obs2_err=us_err)
            if case == (60, 60, 90, "right", -2, 11):
                # This combination misses the *front* face at 30 cm. Keep
                # the strict front cutoff because empty floor gives the same
                # reading; the wider rear-face limit must not weaken it.
                self.assertEqual(code, 1)
                self.assertGreater(ts.run_clearance(fake, course)[0], 0)
                continue
            self.assertEqual(code, 0, case)
            clear, name = ts.run_clearance(fake, course)
            self.assertGreater(clear, t2.CLEARANCE_FLOOR_CM - 0.5, f"{case}: {name}")
            self.assertTrue(_parked_ok(fake.pose), f"{case}: {fake.pose}")

    def test_wide_rear_ir_reading_still_returns_and_parks(self):
        """Floor run 2026-10-14 read 28-33 cm along obstacle 2's back.

        The rear pass is geometrically isolated from empty floor, so it uses
        BACK_GAP_MAX_CM while the front face retains its stricter cutoff.
        """
        import task2t as task2
        import task2t_sim as ts

        course = _course(90, 82, 60)

        class WideRearReading(ts.FakeSTM):
            def _ir(self, p, side):
                value = super()._ir(p, side)
                if value is None or p.x <= self.course.obs2().x1:
                    return value
                k = 1.0 if side == "left" else -1.0
                sensor = t2.straight(p, t2.IR_FROM_AXLE_CM)
                ox = sensor.x - k * t2.IR_SIDE_OFFSET_CM * math.sin(p.th)
                oy = sensor.y + k * t2.IR_SIDE_OFFSET_CM * math.cos(p.th)
                hit = ts._ray_box(ox, oy, math.cos(p.th + k * math.pi / 2),
                                  math.sin(p.th + k * math.pi / 2),
                                  self.course.obs2())
                if hit is not None and abs(hit - value) < 0.5:
                    return value + 13.0
                return value

        frame = t2.WorldFrame(None)
        fake = WideRearReading(course, frame.start, t2.RADIUS_CM)
        arrows = iter(("left", "right"))
        route = task2.Task2(fake, input_fn=lambda _: next(arrows),
                            sleep_fn=lambda _: None, startup=False,
                            wait_for_go=False)
        self.assertEqual(route.run(), 0)
        clear, name = ts.run_clearance(fake, course)
        self.assertGreater(clear, t2.CLEARANCE_FLOOR_CM - 0.5, name)
        self.assertTrue(_parked_ok(fake.pose), fake.pose)



class Centring(unittest.TestCase):

    def test_offset_from_side_irs(self):
        c = 2 * (t2.BAY_HALF_WIDTH_CM - t2.IR_SIDE_OFFSET_CM) / 2
        self.assertAlmostEqual(t2.centre_offset(c + 4, c - 4), 4.0)
        self.assertIsNone(t2.centre_offset(c, None))
        self.assertIsNone(t2.centre_offset(c, c + 25))     # not both fences

    def test_parks_centred_despite_odometry_drift(self):
        # 3 cm of sideways drift per metre lands the robot ~9 cm off centre
        # at the mouth; the side IRs must bring it back within a cm or two.
        import task2t_sim as ts
        for case in ((60, 60, 90, "left"), (90, 82, 60, "right")):
            code, fake, course = ts.simulate(*case, drift_cm_per_m=3.0)
            self.assertEqual(code, 0, case)
            self.assertTrue(_parked_ok(fake.pose), f"{case}: {fake.pose}")
            self.assertLess(abs(fake.pose.y), 1.5, f"{case}: {fake.pose}")


if __name__ == "__main__":
    unittest.main()
