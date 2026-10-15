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
    def test_execution_clearance_tolerates_drift_but_never_lowers_floor(self):
        self.assertAlmostEqual(
            t2.execution_clearance(t2.CLEARANCE_CM),
            max(t2.CLEARANCE_FLOOR_CM,
                t2.CLEARANCE_CM - t2.EXECUTION_CLEARANCE_TOL_CM))
        self.assertEqual(
            t2.execution_clearance(t2.CLEARANCE_FLOOR_CM),
            t2.CLEARANCE_FLOOR_CM)


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

    def test_ir_uncertainty_expands_box_not_walls_or_calibration(self):
        route = self.make_route()
        course = Course(70, 160, 70)
        original = {b.name: b for b in course.boxes()}
        expanded = {b.name: b for b in route._corner_boxes(course)}
        self.assertEqual(expanded["obstacle 2"].x0, 157)
        self.assertEqual(expanded["obstacle 2"].x1, 173)
        for name in original.keys() - {"obstacle 2"}:
            self.assertEqual(expanded[name], original[name])
        self.assertEqual(course.x2, 160)

    def test_large_last_turn_error_requires_back_face_confirmation(self):
        for hits, expected_result in (([None, None], False), ([40, 40], False),
                                       ([29, 29], True), ([26, 26], True)):
            with self.subTest(hits=hits):
                route = self.make_route()
                p = Pose(190, -65, 0)
                segs = [t2.Seg("L", 90)]
                expected = t2.run(p, segs, route.r)[-1]
                measured = Pose(expected.x + 14.6, expected.y, expected.th)
                route._pose = Mock(side_effect=[p, measured])
                route._ir = Mock(side_effect=hits)
                self.assertEqual(route.execute_obstacle2_corner(
                    segs, Course(70, 190, 60)), expected_result)
                self.assertEqual(route._ir.call_count, 2)

    def test_last_corner_completion_cannot_return_success_inside_box(self):
        route = self.make_route()
        route._pose = Mock(side_effect=[Pose(190, -65, 0), Pose(195, 0, math.pi / 2)])
        self.assertFalse(route.execute_obstacle2_corner(
            [t2.Seg("L", 90)], Course(70, 190, 60)))

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

    def test_left_edge_brake_coast_gets_checked_straight_recovery(self):
        """Regression for 19:27: RST coast must not strand a clear corner."""
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0
        near = 33.7
        width = task2.W2_RANGE[1] + 2 * task2.END_MARGIN_CM
        near_y = near + task2.END_MARGIN_CM
        course = t2.Course(
            61.0, 155.1, width,
            obs2_y=near_y - width / 2.0,
            wall_y=width / 2.0 + t2.WALL_CLEARANCE_CM
                   - 2 * task2.END_MARGIN_CM)
        stopped = Pose(127.9, 37.7, math.pi / 2)

        recovered = route.round_near_end_with_brake_recovery(
            stopped, course, 1.0, False, near)

        self.assertIsNotNone(recovered)
        retreat, corner = recovered
        self.assertEqual([seg.token() for seg in retreat], ["R10"])
        self.assertEqual([seg.token() for seg in corner],
                         ["FR90", "F2", "FR90"])
        gap, _ = t2.path_clearance(
            t2.run(stopped, retreat + corner, route.r), course.boxes())
        self.assertGreaterEqual(gap, task2.TURN_CLEARANCE_CM)

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


class Stage2CameraGate(unittest.TestCase):
    def test_logged_bypass_opens_fresh_vision_with_checked_recovery_on_both_sides(self):
        import task2t as route_module
        for side in (-1, 1):
            with self.subTest(side=side):
                notify = Mock()
                route = route_module.Task2(Mock(), startup=False,
                                           lane_reached_fn=notify)
                pose = Pose(42.1, side * 18.6, -side * math.radians(.9))
                course = Course(71.7, 152.3, 90,
                                wall_y=route_module.CONSERVATIVE_WALL_Y)
                route._pose = lambda: pose
                sent = []

                def move(segs):
                    nonlocal pose
                    sent.append(segs)
                    clearance = route._check(pose, segs, course)
                    self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)
                    pose = t2.run(pose, segs, route.r)[-1]
                    return True

                route._line = move
                route._prepare_obstacle2_view(course)
                self.assertEqual(len(sent), 1)
                join = route.select_face_handoff(pose, course, side, -side)
                if join is None:
                    self.assertIsNotNone(route._stage2_view_origin)
                    inverse = {"F": "B", "L": "l", "R": "r"}
                    retreat = [t2.Seg(inverse[s.kind], s.value)
                               for s in reversed(sent[0])]
                    self.assertGreaterEqual(route._check(pose, retreat, course),
                                            t2.CLEARANCE_FLOOR_CM)
                else:
                    self.assertTrue(all(s.kind in ("F", "L", "R") for s in join[0]))
                self.assertGreaterEqual(route._first_board_behind_camera(pose, course.x1),
                                        route_module.STAGE2_REAR_EXCLUSION_CM)
                notify.assert_called_once_with()
                route._prepare_obstacle2_view(course)
                self.assertEqual(len(sent), 1)
                notify.assert_called_once_with()

    def test_direct_view_is_preserved_when_forward_handoff_is_available(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        p = Pose(42.1, -18.6, math.radians(.9))
        course = Course(71.7, 152.3, 90)
        with patch.object(route, "select_face_handoff",
                          return_value=([t2.Seg("L", 90)], False, 8.0)):
            expected = int(math.ceil(
                route_module.STAGE2_REAR_EXCLUSION_CM
                - route._first_board_behind_camera(
                    p, course.x1, course.obs1_y)))
            self.assertEqual(route._select_obstacle2_view(p, course),
                             [t2.Seg("F", expected)])

    def test_gate_keeps_vision_closed_when_all_routes_hit_clearance_floor(self):
        import task2t as route_module
        notify = Mock()
        route = route_module.Task2(Mock(), startup=False, lane_reached_fn=notify)
        route._pose = Mock(return_value=Pose(42.1, -17, math.radians(.9)))
        route._line = Mock()
        route._check = Mock(return_value=2.5)
        with self.assertRaisesRegex(RuntimeError, "camera-gate.*clearance"):
            route._prepare_obstacle2_view(Course(71.7, 152.3, 90))
        route._line.assert_not_called()
        notify.assert_not_called()

    def test_latest_logged_gate_has_checked_curved_exit_at_configured_radii(self):
        import task2t as route_module
        for radius in (29.1, 30.0, 31.5):
            for side in (-1, 1):
                with self.subTest(radius=radius, side=side):
                    notify = Mock()
                    route = route_module.Task2(Mock(), startup=False,
                                               lane_reached_fn=notify)
                    route.r = radius
                    route.fast = route_module.FAST
                    route.obs2_us_y = side * 15.5
                    pose = Pose(39.8, side * 17.8, -side * math.radians(1.2))
                    course = Course(69.7, 149.0, 90,
                                    wall_y=route_module.CONSERVATIVE_WALL_Y)
                    route._pose = lambda: pose
                    self.assertLess(route._check(pose, [t2.Seg("F", 34)], course),
                                    t2.CLEARANCE_FLOOR_CM)
                    sent = []

                    def move(segs):
                        nonlocal pose
                        self.assertTrue(all(s.kind in ("F", "L", "R") for s in segs))
                        self.assertGreaterEqual(route._check(pose, segs, course),
                                                t2.CLEARANCE_FLOOR_CM)
                        sent.append(segs)
                        pose = t2.run(pose, segs, radius)[-1]
                        return True

                    route._line = move
                    route._prepare_obstacle2_view(course)
                    self.assertEqual(len(sent), 1)
                    notify.assert_called_once_with()
                    self.assertGreaterEqual(route._first_board_behind_camera(pose, course.x1),
                                            route_module.STAGE2_REAR_EXCLUSION_CM)
                    join = route.select_face_handoff(pose, course, side, -side)
                    self.assertIsNotNone(join)
                    self.assertTrue(all(s.kind in ("F", "L", "R") for s in join[0]))
                    self.assertGreaterEqual(route._check(pose, join[0], course),
                                            t2.CLEARANCE_FLOOR_CM)
                    self.assertEqual(route.r, radius)

    def test_curved_view_recovery_retraces_arcs_not_blind_straight(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        origin = Pose(39.8, -17.8, math.radians(1.2))
        course = Course(69.7, 149, 90,
                        wall_y=route_module.CONSERVATIVE_WALL_Y)
        path = [t2.Seg("R", 45), t2.Seg("L", 45)]
        route._stage2_view_origin = origin
        route._stage2_view_segments = path
        pose = t2.run(origin, path, route.r)[-1]
        route._pose = lambda: pose
        sent = []

        def move(segs):
            nonlocal pose
            self.assertGreaterEqual(route._check(pose, segs, course),
                                    t2.CLEARANCE_FLOOR_CM)
            sent.append(segs)
            pose = t2.run(pose, segs, route.r)[-1]
            return True

        route._line = move
        route.select_face_handoff = Mock(side_effect=[None, ([t2.Seg("L", 90)], False, 8)])
        route._execute_face_handoff = Mock(return_value=False)
        route.around_obstacle2(course, -1, 1)
        self.assertEqual(sent, [[t2.Seg("l", 45), t2.Seg("r", 45)]])
        self.assertAlmostEqual(pose.x, origin.x)
        self.assertAlmostEqual(pose.y, origin.y)
        self.assertIsNone(route._stage2_view_segments)

    def test_drifted_curved_view_still_uses_checked_inverse(self):
        """Regression for 06:45: same arrows must not strand the camera pose."""
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        origin = Pose(39.0, -27.3, math.radians(.3))
        path = [t2.Seg("R", 8), t2.Seg("F", 45), t2.Seg("F", 3),
                t2.Seg("L", 90)]
        nominal = t2.run(origin, path, route.r, step=1e6)[-1]
        pose = Pose(nominal.x + 4.0, nominal.y, nominal.th)
        course = Course(35.6, 127.9, 50.0)
        route._stage2_view_origin = origin
        route._stage2_view_segments = path
        route._pose = lambda: pose
        sent = []

        def move(segs):
            nonlocal pose
            sent.append(segs)
            pose = t2.run(pose, segs, route.r, step=1e6)[-1]
            return True

        route._line = move
        route._check = Mock(return_value=8.0)
        route.select_face_handoff = Mock(
            side_effect=[None, ([t2.Seg("R", 90)], False, 8.0)])
        route._execute_face_handoff = Mock(return_value=False)

        route.around_obstacle2(course, -1, -1)

        self.assertEqual(sent, [[t2.Seg("l", 90), t2.Seg("B", 3),
                                 t2.Seg("B", 45), t2.Seg("r", 8)]])
        self.assertGreater(math.hypot(pose.x - origin.x, pose.y - origin.y),
                           3.0)
        route._execute_face_handoff.assert_called_once()

    def test_logged_pose_reaches_parking_for_all_four_arrow_pairs(self):
        import task2t as route_module
        from task2t_sim import FakeSTM, run_clearance
        for side1 in (1, -1):
            for side2 in (1, -1):
                with self.subTest(side1=side1, side2=side2):
                    actual = Course(69.7, 149, 60)
                    planned = Course(69.7, 149, 90,
                                     wall_y=route_module.CONSERVATIVE_WALL_Y)
                    stm = FakeSTM(actual, t2.WorldFrame(None).start, t2.RADIUS_CM)
                    route = route_module.Task2(
                        stm, input_fn=lambda _: "left" if side2 > 0 else "right",
                        sleep_fn=lambda _: None, startup=False, wait_for_go=False)
                    route.fast = route_module.FAST
                    stm.set_fast(route.fast)
                    self.assertTrue(route.frame.anchor())
                    stm.pose = Pose(39.8, side1 * 17.8, -side1 * math.radians(1.2))
                    stm.trace = [stm.pose]
                    route.obs2_us_y = side1 * 15.5
                    # Production task2.py uses measured, not commanded, heading.
                    route._pose = route.frame.pose
                    route.photo_obstacle2(planned, side1)
                    measured = route.around_obstacle2(planned, side1, side2)
                    self.assertIsNotNone(measured)
                    self.assertTrue(t2.ParkRunner(stm, route.frame, measured).run())
                    self.assertGreaterEqual(run_clearance(stm, actual)[0],
                                            t2.CLEARANCE_FLOOR_CM)
                    self.assertTrue(all(-t2.BAY_DEPTH_CM <= x <= 0
                                        and abs(y) <= t2.BAY_HALF_WIDTH_CM
                                        for x, y in t2.footprint(stm.pose)))

    def test_logged_view_can_restore_handoff_room_on_either_side(self):
        import task2t as route_module
        for side in (-1, 1):
            with self.subTest(side=side):
                route = route_module.Task2(Mock(), startup=False)
                course = Course(71.7, 152.3, 90,
                                wall_y=route_module.CONSERVATIVE_WALL_Y)
                origin = Pose(42.1, side * 18.6, -side * math.radians(.9))
                route._stage2_view_origin = origin
                pose = t2.run(origin, [t2.Seg("F", 34)], route.r)[-1]
                route._pose = lambda: pose
                sent = []

                def move(segs):
                    nonlocal pose
                    self.assertGreaterEqual(route._check(pose, segs, course),
                                            t2.CLEARANCE_FLOOR_CM)
                    sent.append(segs)
                    pose = t2.run(pose, segs, route.r)[-1]
                    return True

                route._line = move
                # Stop at dispatch: this test checks real planning and the
                # viewing retreat, not simulated IR measurements of the box.
                route._execute_face_handoff = Mock(return_value=False)
                route.around_obstacle2(course, side, -side)
                self.assertEqual(sent, [[t2.Seg("B", 34)]])
                route._execute_face_handoff.assert_called_once()
                self.assertIsNone(route._stage2_view_origin)


class FaceScanContinuation(unittest.TestCase):
    def make_route(self, facing=1, toward=1, heading_error=0):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False, sleep_fn=lambda _: None)
        pose = Pose(125.4 if facing > 0 else 180.6, toward * 43.2,
                    toward * (math.pi / 2 - facing * math.radians(heading_error)))
        route._pose = Mock(return_value=pose)
        route.frame.pose = Mock(return_value=pose)
        route._ir = Mock(return_value=14.0)
        route._us = Mock(return_value=90.0)
        route._stable_us = Mock(return_value=90.0)
        route._wait_for_stopped_pose = Mock(return_value=pose)
        face = pose.x + facing * (t2.IR_SIDE_OFFSET_CM + 14.0)
        return route, face

    def test_logged_persistent_14cm_echo_can_continue_to_real_edge_both_faces(self):
        for facing in (1, -1):
            for toward in (1, -1):
                with self.subTest(facing=facing, toward=toward):
                    route, face = self.make_route(facing, toward, heading_error=5.8)
                    commands = []

                    def move(stm, token, poll):
                        commands.append(token)
                        if len(commands) == 1:
                            return "OK"
                        self.assertGreaterEqual(int(token[1:]), 2)
                        self.assertLessEqual(int(token[1:]), 20)
                        route._ir.return_value = None
                        self.assertFalse(poll())
                        self.assertTrue(poll())
                        return "STOPPED"

                    with patch.object(t2, "run_polling", side_effect=move):
                        result = route.follow_face("F58", face, facing, toward, 14)
                    self.assertIsNotNone(result)
                    self.assertEqual(len(commands), 2)
                    route.stm.abort.assert_called_once()
                    route._us.assert_called()

    def test_missing_or_close_front_us_does_not_extend(self):
        for reading in (None, float("nan"), 20.0, 27.0):
            with self.subTest(reading=reading):
                route, face = self.make_route()
                route._stable_us.return_value = reading
                with patch.object(t2, "run_polling", return_value="OK") as move:
                    self.assertIsNone(route.follow_face("F58", face, 1, 1, 14))
                move.assert_called_once()

    def test_body_clearance_and_reverse_scan_do_not_allow_extra_travel(self):
        for token, gap in (("R58", 14), ("F58", 9)):
            with self.subTest(token=token, gap=gap):
                route, face = self.make_route()
                route._ir.return_value = gap
                with patch.object(t2, "run_polling", return_value="OK") as move:
                    self.assertIsNone(route.follow_face(token, face, 1, 1, 14))
                move.assert_called_once()

    def test_stuck_face_is_bounded_and_never_fabricates_edge(self):
        route, face = self.make_route()
        commands = []

        def move(stm, token, poll):
            commands.append(token)
            self.assertFalse(poll())
            return "OK"

        with patch.object(t2, "run_polling", side_effect=move):
            self.assertIsNone(route.follow_face("F58", face, 1, 1, 14))
        self.assertEqual(sum(int(token[1:]) for token in commands[1:]), 30)
        self.assertEqual(len(commands), 3)
        route.stm.abort.assert_not_called()

    def test_front_obstruction_during_extension_stops_instead_of_turning(self):
        route, face = self.make_route()
        calls = []

        def move(stm, token, poll):
            calls.append(token)
            if len(calls) == 1:
                return "OK"
            route._us.return_value = 25
            self.assertTrue(poll())
            return "STOPPED"

        with patch.object(t2, "run_polling", side_effect=move):
            self.assertIsNone(route.follow_face("F58", face, 1, 1, 14))
        route.stm.abort.assert_called_once()


class MeasuredForwardFlow(unittest.TestCase):
    def test_late_pose_uses_existing_forward_margin_on_both_sides(self):
        import task2t as route_module
        for side in (-1, 1):
            with self.subTest(side=side):
                route = route_module.Task2(Mock(), startup=False)
                start = Pose(85, side * 19, 0)
                course = Course(71.7, 152.3, 90)
                selected = route.select_face_handoff(start, course, side, side)
                self.assertIsNotNone(selected)
                segs, reverse, clearance = selected
                self.assertFalse(reverse)
                self.assertTrue(all(s.kind in ("F", "L", "R") for s in segs))
                self.assertGreaterEqual(clearance, route_module.HANDOFF_CLEARANCE_CM)
                self.assertLess(clearance, t2.CLEARANCE_CM)
                actual = min(route._check(start, segs, c)
                             for c in route.same_side_courses(course))
                self.assertAlmostEqual(actual, clearance)

    def test_measured_clear_turn_does_not_require_original_surplus_margin(self):
        import task2t as route_module
        for side in (-1, 1):
            with self.subTest(side=side):
                route = route_module.Task2(Mock(), startup=False)
                start = Pose(91, side * 23.2, 0)
                course = Course(71.7, 152.3, 90)
                segs, reverse, clearance = route.select_face_handoff(
                    start, course, side, side)
                self.assertGreaterEqual(clearance, route_module.HANDOFF_CONTINUOUS_CLEARANCE_CM)
                self.assertLess(clearance, 8.5)
                end = t2.run(start, segs, route.r)[-1]
                route.frame.pose = Mock(side_effect=[start, end, end])
                route._line = Mock(return_value=True)
                self.assertTrue(route._execute_face_handoff(
                    segs, course, side, side, reverse, planned_clearance=10))
                route._line.assert_called_once_with(segs)

    def test_forward_fallback_is_preferred_to_a_reverse_route(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        forward = [t2.Seg("F", 20), t2.Seg("L", 90)]
        reverse = [t2.Seg("B", 1), t2.Seg("L", 90)]
        with patch.object(route, "face_handoff_candidates",
                          return_value=[(reverse, False), (forward, False)]), \
                patch.object(route, "_face_handoff_endpoint_valid", return_value=True), \
                patch.object(route, "_handoff_clearance",
                             side_effect=lambda p, segs, cs, deadline:
                             4.6 if segs == forward else 10):
            selected = route.select_face_handoff(Pose(60, 20, 0),
                                                 Course(80, 170, 60), 1, -1)
        self.assertEqual(selected, (forward, False, 4.6))

    def test_below_fallback_margin_still_cannot_execute(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        start = Pose(91, 23.2, 0)
        route.frame.pose = Mock(return_value=start)
        route._line = Mock()
        with patch.object(route, "_face_handoff_endpoint_valid", return_value=True), \
                patch.object(t2, "path_clearance", return_value=(3.0, "box")):
            self.assertIsNone(route._execute_face_handoff(
                [t2.Seg("F", 20), t2.Seg("L", 90)],
                Course(71.7, 152.3, 90), 1, 1, False, 10))
        route._line.assert_not_called()


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
                        first_turn = next(i for i, token in enumerate(tokens)
                                          if token.startswith(("FL", "FR", "RL", "RR")))
                        reverse_after_bypass = [i for i in range(first_turn, len(tokens))
                                                if tokens[i].startswith("R")
                                                and tokens[i][1:].isdigit()]
                        # A checked camera-view retreat is allowed; the old
                        # opposite-turn-then-reverse handoff is not.
                        self.assertLessEqual(len(reverse_after_bypass), 1, tokens)
                        for i in reverse_after_bypass:
                            self.assertEqual(tokens[i - 1], "F" + tokens[i][1:], tokens)
                        self.assertFalse(any(token.startswith("FU") for token in tokens))
                    self.assertTrue(_parked_ok(stm.pose))
                    self.assertGreaterEqual(sim.run_clearance(stm, course)[0],
                                            t2.CLEARANCE_FLOOR_CM)

    def test_inflight_lane_detection_avoids_photo_move(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, input_fn=lambda _: "left",
                           sleep_fn=Mock(), early_arrow_fn=Mock(side_effect=[None, "left"]))
        route._pose = Mock(return_value=Pose(86, 21, 0))
        route._line = Mock()
        self.assertEqual(route.photo_obstacle2(Course(80, 150, 90), 1), 1)
        route._line.assert_not_called()
        route.sleep_fn.assert_not_called()

    def test_photo_vote_finishing_after_path_check_skips_positioning(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, input_fn=lambda _: "right",
                           sleep_fn=lambda _: None,
                           early_arrow_fn=Mock(
                               side_effect=[None, None, None, None, "right"]))
        route._pose = Mock(return_value=Pose(93, 18.8,
                                             math.radians(-1.6)))
        route._line = Mock(return_value=True)
        segs = [t2.Seg("B", 6), t2.Seg("L", 45)]
        with patch.object(route, "photo_candidates",
                          return_value=[(segs, 34, 45)]), \
                patch.object(route, "_first_clear", return_value=segs), \
                patch.object(task2.tp, "run_polling") as polling:
            self.assertEqual(route.photo_obstacle2(
                Course(85.7, 151.0, 90), 1), -1)
        polling.assert_not_called()
        route._line.assert_not_called()

    def test_logged_right_left_pose_reaches_a_forward_handoff(self):
        """Regression for 19:59: do not strand an early stage-2 result."""
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, center_stage2=True)
        route.obs2_us_y = -20.8
        start = Pose(36.6, -20.8, math.radians(1.0))
        course = Course(65.7, 156.8, 90,
                        wall_y=task2.CONSERVATIVE_WALL_Y)
        state = [start]
        route._pose = lambda: state[0]

        def move(segs):
            state[0] = t2.run(state[0], segs, route.r, step=1e6)[-1]
            return True

        route._line = move
        self.assertTrue(route.position_obstacle2_camera(course, -1.0))
        staged = state[0]

        selected = route.select_face_handoff(staged, course, -1.0, 1.0)

        self.assertIsNotNone(selected)
        segs, reverse, clearance = selected
        self.assertFalse(reverse)
        self.assertGreaterEqual(clearance,
                                task2.HANDOFF_CONTINUOUS_CLEARANCE_CM)
        self.assertTrue(route._face_handoff_endpoint_valid(
            staged, segs, course, -1.0, 1.0, reverse))

    def test_logged_obstacle1_touch_uses_larger_turn_margin(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0
        pose = Pose(17.8, 2.8, math.radians(-1.5))
        course = Course(85.7, 155.7, 90,
                        wall_y=task2.CONSERVATIVE_WALL_Y)
        route._pose = Mock(return_value=pose)
        route._stable_us = Mock(return_value=42.7)
        route._ir = Mock(return_value=20.0)
        route._line = Mock(return_value=False)

        self.assertIsNone(route.pass_obstacle1(course.x1, 1.0))
        sent = route._line.call_args.args[0]
        self.assertEqual(sent[0].kind, "B")
        self.assertGreaterEqual(route._check(pose, sent, course),
                                task2.OBS1_TURN_MIN_CM)

    def test_logged_95mm_obstacle1_margin_is_not_rejected_after_approach(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route._pose = Mock(return_value=Pose(20, 0, 0))
        route._check = Mock(return_value=9.5)
        route._stable_us = Mock(return_value=42.7)
        route._ir = Mock(return_value=20.0)
        route._line = Mock(return_value=False)

        self.assertIsNone(route.pass_obstacle1(80, 1.0))
        route._line.assert_called_once()
        self.assertGreaterEqual(9.5, task2.OBS1_TURN_MIN_CM)

    def test_logged_80mm_obstacle1_margin_defers_to_live_sensors(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route._pose = Mock(return_value=Pose(20, 0, 0))
        route._check = Mock(return_value=8.0)
        route._stable_us = Mock(return_value=42.7)
        route._ir = Mock(return_value=20.0)
        route._line = Mock(return_value=False)

        self.assertIsNone(route.pass_obstacle1(80, 1.0))
        route._line.assert_called_once()
        route._stable_us.assert_called_once()

    def test_obstacle1_live_sensor_stop_blocks_lane_change(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route._pose = Mock(return_value=Pose(20, 0, 0))
        route._check = Mock(return_value=6.4)
        route._stable_us = Mock(return_value=10.0)
        route._ir = Mock(return_value=20.0)
        route._line = Mock(return_value=True)

        self.assertIsNone(route.pass_obstacle1(80, 1.0))
        route._line.assert_not_called()

    def test_obstacle1_predicted_overlap_is_never_sent(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route._pose = Mock(return_value=Pose(20, 0, 0))
        route._check = Mock(return_value=-1.0)
        route._stable_us = Mock(return_value=42.7)
        route._ir = Mock(return_value=None)
        route._line = Mock(return_value=True)

        self.assertIsNone(route.pass_obstacle1(80, 1.0))
        route._line.assert_not_called()

    def test_stable_lane_echo_survives_reconstructed_spacing_drift(self):
        """A safe live range must not be confused with arena spacing."""
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0
        pose = Pose(58.3, 20.0, 0.0)
        route._pose = Mock(return_value=pose)
        route._check = Mock(return_value=12.0)
        route._line = Mock(return_value=True)
        route._us = Mock(return_value=63.7)
        route._ir = Mock(return_value=None)

        x2 = route.pass_obstacle1(80.0, 1.0)

        expected_x2 = pose.x + task2.tp.FRONT_FROM_AXLE_CM + 63.7
        self.assertAlmostEqual(x2, expected_x2)
        self.assertLess(x2 - (80.0 + task2.tp.OBS1_SIZE_CM), 60.0)
        self.assertAlmostEqual(route.obs2_us_y, pose.y)
        self.assertGreaterEqual(route._us.call_count, 2)

    def test_close_lane_echo_remains_a_hard_stop(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0
        route._pose = Mock(return_value=Pose(58.3, 20.0, 0.0))
        route._check = Mock(return_value=12.0)
        route._line = Mock(return_value=True)
        route._us = Mock(return_value=20.0)
        route._ir = Mock(return_value=None)

        self.assertIsNone(route.pass_obstacle1(80.0, 1.0))

    def test_sensing_lane_uses_confirmed_miss_as_width_bound(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.obs2_us_y = 21
        route._pose = Mock(side_effect=[Pose(50, 21, 0), Pose(87, 35, 0),
                                       Pose(50, 21, 0)])
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

    def test_logged_heading_drift_is_absorbed_by_sensing_lane(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.obs2_us_y = 14.6
        start = Pose(56.7, 14.6, math.radians(-6.7))
        route._pose = Mock(return_value=start)
        route._line = Mock(return_value=True)
        route._confirm_sensing_lane = Mock(return_value=True)
        course = Course(86.7, 177.7, 90,
                        wall_y=task2.CONSERVATIVE_WALL_Y)

        self.assertTrue(route.prepare_forward_lane(course, 1))

        sent = route._line.call_args_list
        self.assertEqual(len(sent), 1)
        segs = sent[0].args[0]
        self.assertIn(t2.Seg("L", 7), segs)
        clearance = min(route._check(start, segs, candidate)
                        for candidate in route.same_side_courses(course))
        self.assertGreaterEqual(
            clearance, t2.execution_clearance(t2.CLEARANCE_CM))
        route._confirm_sensing_lane.assert_called_once_with(course)

    def test_logged_two_degree_heading_uses_existing_lane_arcs(self):
        import task2t as task2
        for side in (-1, 1):
            with self.subTest(side=side):
                route = task2.Task2(Mock(), startup=False)
                route.r = 30.0
                route.obs2_us_y = side * 19.2
                pose = Pose(40.5, side * 19.2, math.radians(side * 2.1))
                course = Course(71.1, 162.7, 90, wall_y=65)
                route._pose = lambda: pose
                sent = []

                def move(segs):
                    nonlocal pose
                    clearance = min(route._check(pose, segs, c)
                                    for c in route.same_side_courses(course))
                    self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)
                    self.assertTrue(all(s.kind == "F" or s.value >= t2.MIN_ARC_DEG
                                        for s in segs))
                    sent.append(segs)
                    pose = t2.run(pose, segs, route.r)[-1]
                    return True

                route._line = move
                route._us = Mock(return_value=220)
                self.assertTrue(route.prepare_forward_lane(course, side))
                self.assertEqual(len(sent), 1)
                self.assertEqual(len(sent[0]), 2)
                self.assertLess(abs(math.degrees(pose.th)), 0.5)
                self.assertIsNotNone(route.obs2_half_max)

    def test_small_sensing_drift_does_not_disable_width_refinement(self):
        import task2t as task2
        for side in (-1, 1):
            with self.subTest(side=side):
                route = task2.Task2(Mock(), startup=False)
                route.r = 30.0
                route.obs2_us_y = side * 19.2
                route.obs2_half_max = 38.5
                course = Course(71.1, 162.7, 90, wall_y=65)
                pose = Pose(84.9, side * 34.4, math.radians(side * 3.1))
                route._pose = lambda: pose
                route._confirm_sensing_lane = Mock(return_value=True)
                sent = []

                def move(segs):
                    nonlocal pose
                    self.assertGreaterEqual(min(route._check(pose, segs, c)
                                                for c in route.same_side_courses(course)),
                                            t2.execution_clearance(t2.CLEARANCE_CM))
                    sent.extend(segs)
                    pose = t2.run(pose, segs, route.r)[-1]
                    return True

                route._line = move
                self.assertTrue(route.refine_forward_width(course, side))
                self.assertEqual(len(sent), 2)
                self.assertTrue(all(s.value >= t2.MIN_ARC_DEG for s in sent))
                self.assertLess(abs(math.degrees(pose.th)), 0.5)
                route._confirm_sensing_lane.assert_called_once_with(course)

    def test_logged_heading_drift_is_absorbed_into_one_sensing_queue(self):
        import task2t as task2
        for side in (-1, 1):
            with self.subTest(side=side):
                route = task2.Task2(Mock(), startup=False)
                route.r = 30.0
                route.obs2_us_y = side * 15.5
                pose = Pose(43.4, side * 15.5,
                            -side * math.radians(12.2))
                course = Course(71.0, 159.2, 90, wall_y=65)
                route._pose = Mock(return_value=pose)
                route._confirm_sensing_lane = Mock(return_value=True)
                route._line = Mock(return_value=True)

                self.assertTrue(route.prepare_forward_lane(course, side))

                route._line.assert_called_once()
                segs = route._line.call_args.args[0]
                self.assertEqual(segs[0], t2.Seg("B", 8))
                self.assertEqual(segs[1].kind, "L" if side > 0 else "R")
                self.assertTrue(all(seg.value >= t2.MIN_ARC_DEG
                                    for seg in segs if seg.kind in ("L", "R")))
                clearance = min(route._check(pose, segs, candidate)
                                for candidate in route.same_side_courses(course))
                self.assertGreaterEqual(
                    clearance, t2.execution_clearance(t2.CLEARANCE_CM))
                route._confirm_sensing_lane.assert_called_once_with(course)

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

    def test_same_direction_search_includes_wider_45_degree_multiples(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0

        choices = list(route.same_side_forward_candidates(
            Pose(40, -20, 0), Course(70, 140, 60), -1,
            straight_limit=100))

        tokens = [[seg.token() for seg in choice] for choice in choices]
        self.assertTrue(any("FR90" in choice and "FL180" in choice
                            for choice in tokens))

    def test_logged_same_face_pose_prefers_short_outer_eight(self):
        import task2t as task2
        for mirror in (1, -1):
            with self.subTest(mirror=mirror):
                route = task2.Task2(Mock(), startup=False)
                route.obs2_us_y = mirror * 34.4
                pose = Pose(91.5, mirror * 35.0,
                            math.radians(-mirror * 0.5))
                course = Course(83.7, 175.4, 90,
                                wall_y=task2.CONSERVATIVE_WALL_Y)

                selected = route.select_face_handoff(
                    pose, course, mirror, mirror)

                self.assertIsNotNone(selected)
                segs, reverse, clearance = selected
                self.assertFalse(reverse)
                self.assertLessEqual(len(segs), 4, [s.token() for s in segs])
                self.assertGreaterEqual(clearance, t2.CLEARANCE_CM)
                self.assertEqual(segs[-1].kind, "R" if mirror > 0 else "L")

    def test_same_face_handoff_is_derived_from_live_geometry(self):
        import task2t as task2
        routes = []
        for obstacle_x in (165.0, 185.0):
            route = task2.Task2(Mock(), startup=False)
            route.obs2_us_y = 34.4
            pose = Pose(91.5, 35.0, math.radians(-0.5))
            course = Course(83.7, obstacle_x, 90,
                            wall_y=task2.CONSERVATIVE_WALL_Y)

            selected = route.select_face_handoff(pose, course, 1, 1)

            self.assertIsNotNone(selected)
            segs, reverse, clearance = selected
            self.assertFalse(reverse)
            self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)
            routes.append([seg.token() for seg in segs])
        self.assertNotEqual(routes[0], routes[1])

    def test_logged_outer_arc_drift_has_forward_continuation_on_both_sides(self):
        """23:40 run: do not strand the car after the first checked arc."""
        import task2t as task2
        for side in (-1, 1):
            with self.subTest(side=side):
                route = task2.Task2(Mock(), startup=False)
                route.r = 30.0
                route.obs2_us_y = side * 19.4
                route.obs2_half_max = 35.5
                pose = Pose(128.0, side * 38.8,
                            side * math.radians(44.2))
                course = Course(69.6, 168.5, 90)

                selected = route.select_face_handoff(
                    pose, course, side, side)

                self.assertIsNotNone(selected)
                segs, reverse, clearance = selected
                self.assertFalse(reverse)
                self.assertTrue(all(seg.kind in ("F", "L", "R")
                                    for seg in segs))
                self.assertEqual(segs[0].kind, "R" if side > 0 else "L")
                self.assertEqual(segs[-1].kind, segs[0].kind)
                self.assertGreaterEqual(
                    clearance, task2.HANDOFF_CLEARANCE_CM)

    def test_measured_outer_recovery_is_queued_without_baby_steps(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.obs2_us_y = -19.4
        route.obs2_half_max = 35.5
        start = Pose(128.0, -38.8, math.radians(-44.2))
        segs = [t2.Seg("L", 35), t2.Seg("F", 24), t2.Seg("L", 99)]
        end = t2.run(start, segs, route.r, step=1e6)[-1]
        route.frame.pose = Mock(side_effect=[start, end, end])
        route._line = Mock(return_value=True)

        self.assertTrue(route._execute_face_handoff(
            segs, Course(69.6, 168.5, 90), -1, -1, False, 5.1))
        route._line.assert_called_once_with(segs)

    def test_clear_same_face_handoff_is_queued_as_one_group(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.obs2_us_y = 34.4
        segs = [t2.Seg("F", 36), t2.Seg("L", 45),
                t2.Seg("F", 16), t2.Seg("R", 134)]
        start = Pose(91.5, 35.0, math.radians(-0.5))
        end = t2.run(start, segs, route.r, step=1e6)[-1]
        route.frame.pose = Mock(side_effect=[start, end, end])
        route._line = Mock(return_value=True)
        course = Course(83.7, 175.4, 90,
                        wall_y=task2.CONSERVATIVE_WALL_Y)
        with patch.object(route, "_face_handoff_endpoint_valid",
                          return_value=True):
            self.assertTrue(route._execute_face_handoff(
                segs, course, 1, 1, False, 7.0))
        route._line.assert_called_once_with(segs)

    def test_clear_opposite_handoff_is_queued_continuously(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        segs = [t2.Seg("R", 30), t2.Seg("F", 30), t2.Seg("L", 118)]
        p0 = Pose(60, -20, 0)
        p1 = t2.run(p0, segs, route.r, step=1e6)[-1]
        route.frame.pose = Mock(side_effect=[p0, p1, p1])
        route._line = Mock(return_value=True)
        course = Course(80, 170, 60)
        with patch.object(route, "_face_handoff_endpoint_valid",
                          return_value=True), \
                patch.object(t2, "path_clearance", return_value=(9.0, "box")):
            self.assertTrue(route._execute_face_handoff(
                segs, course, -1, 1, False, 9.0))
        route._line.assert_called_once_with(segs)

    def test_logged_left_sensing_drift_is_aligned_before_measuring(self):
        import task2t as task2
        for mirror in (1, -1):
            route = task2.Task2(Mock(), startup=False)
            route.obs2_us_y = mirror * 23.7
            course = Course(72.7, 165.1, 90, wall_y=task2.CONSERVATIVE_WALL_Y)
            pose = Pose(80.1, mirror * 35.1, math.radians(-7.6 * mirror))
            route._pose = lambda: pose
            sent = []
            def move(segs):
                nonlocal pose
                self.assertGreaterEqual(min(route._check(pose, segs, c)
                                             for c in route.same_side_courses(course)),
                                        t2.CLEARANCE_CM)
                sent.extend(segs)
                pose = t2.run(pose, segs, route.r)[-1]
                return True
            route._line = move
            route._us = lambda: (course.x2 - pose.x) / math.cos(pose.th) - t2.FRONT_FROM_AXLE_CM
            self.assertTrue(route._confirm_sensing_lane(course))
            self.assertEqual([s.token() for s in sent], ["FL8" if mirror == 1 else "FR8"])
            selected = route.select_face_handoff(pose, course, mirror, mirror)
            self.assertIsNotNone(selected)
            self.assertFalse(selected[1])
            self.assertGreaterEqual(selected[2], t2.CLEARANCE_FLOOR_CM)

    def test_logged_four_degree_sensing_drift_is_projected_without_tiny_arc(self):
        import task2t as task2
        for mirror in (1, -1):
            route = task2.Task2(Mock(), startup=False)
            route.obs2_us_y = mirror * 23.7
            course = Course(72.7, 165.1, 90,
                            wall_y=task2.CONSERVATIVE_WALL_Y)
            pose = Pose(111.4, mirror * 31.6,
                        math.radians(-4.7 * mirror))
            route._pose = Mock(return_value=pose)
            route._line = Mock(return_value=True)
            echo = ((course.x2 - pose.x) / math.cos(pose.th)
                    - t2.FRONT_FROM_AXLE_CM)
            route._us = Mock(return_value=echo)

            self.assertTrue(route._confirm_sensing_lane(course))
            route._line.assert_not_called()
            expected_y = (pose.y
                          + (t2.FRONT_FROM_AXLE_CM + echo) * math.sin(pose.th))
            self.assertAlmostEqual(route.obs2_us_y, expected_y, places=6)

    def test_logged_background_echo_uses_bounded_probe_not_new_face(self):
        import task2t as task2
        for side in (-1, 1):
            route = task2.Task2(Mock(), startup=False)
            route.obs2_us_y = side * 17.9
            course = Course(62.6, 155.2, 90, wall_y=task2.CONSERVATIVE_WALL_Y)
            pose = Pose(75.1, side * 35.6, math.radians(-side * 2))
            route._pose = lambda: pose
            route._us = Mock(return_value=230.7)
            self.assertTrue(route._confirm_sensing_lane(course))
            self.assertEqual(course.x2, 155.2)
            self.assertGreater(route.obs2_half_max, 17.9)
            self.assertLess(route.obs2_half_max, 40)
            def move(segs):
                nonlocal pose
                self.assertTrue(all(s.kind in ("F", "L", "R") for s in segs))
                self.assertGreaterEqual(min(route._check(pose, segs, c)
                                             for c in route.same_side_courses(course)),
                                        t2.CLEARANCE_CM)
                pose = t2.run(pose, segs, route.r)[-1]
                return True
            route._line = move
            route._us = lambda: (course.x2 - pose.x) / math.cos(pose.th) - t2.FRONT_FROM_AXLE_CM
            self.assertTrue(route.refine_forward_width(course, side))
            selected = route.select_face_handoff(pose, course, side, side)
            self.assertIsNotNone(selected)
            self.assertFalse(selected[1])

    def test_sensing_does_not_narrow_on_unstable_or_closer_echo(self):
        import task2t as task2
        for hits in ([230, 190], [None, 230], [20, 20]):
            route = task2.Task2(Mock(), startup=False)
            route.obs2_us_y = -17.9
            route._pose = Mock(return_value=Pose(75.1, -35.6, 0))
            route._us = Mock(side_effect=hits)
            self.assertFalse(route._confirm_sensing_lane(Course(62.6, 155.2, 90)))
            self.assertIsNone(route.obs2_half_max)

    def test_sensing_alignment_never_bypasses_collision_check(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.obs2_us_y = 23.7
        route._pose = Mock(return_value=Pose(80.1, 35.1, math.radians(-7.6)))
        route._check = Mock(return_value=0)
        route._line = Mock()
        route._us = Mock()
        self.assertFalse(route._confirm_sensing_lane(Course(72.7, 165.1, 90)))
        route._line.assert_not_called()
        route._us.assert_not_called()

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
        route._execute_face_handoff = Mock(return_value=True)
        route._set_fast = Mock()
        route.follow_face = Mock(return_value=(45, 170))
        route.round_near_end = Mock(return_value=None)
        self.assertIsNone(route.around_obstacle2(Course(80, 170, 90), 1, 1))
        args, kwargs = route.follow_face.call_args
        self.assertTrue(args[0].startswith("F"))
        self.assertEqual(args[1:4], (170, 1, 1))
        self.assertTrue(kwargs["check_front"])
        self.assertTrue(route.round_near_end.call_args.kwargs["forward_only"])
        route._execute_face_handoff.assert_called_once()

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
        self.assertEqual(route._ir.call_count, 4)

    def test_front_face_range_change_alone_is_not_an_edge(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, sleep_fn=lambda _: None)
        x_face = 156.0
        pose = Pose(x_face - t2.IR_SIDE_OFFSET_CM - 22.0, 58.3,
                    -math.pi / 2)
        route._pose = Mock(return_value=pose)
        route.frame.pose = Mock(return_value=pose)
        route._wait_for_stopped_pose = Mock(return_value=pose)
        # A moderate jump above the acquisition limit is not proof of an end.
        # Continue tracking until the blank readings actually confirm it.
        route._ir = Mock(side_effect=[28, 28, 28, 36, 36, 36, None, None])

        def edge(stm, token, poll):
            for _ in range(5):
                self.assertFalse(poll())
            self.assertTrue(poll())
            return "STOPPED"

        with patch.object(t2, "run_polling", side_effect=edge):
            measured = route.follow_face("F39", x_face, +1, -1, 18)
        self.assertIsNotNone(measured)
        route.stm.abort.assert_called_once()
        route._wait_for_stopped_pose.assert_called_once()

    def test_220643_left_right_false_edge_is_not_accepted(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, sleep_fn=lambda _: None)
        pose = Pose(137, 15.5, -math.pi / 2)
        route._pose = Mock(return_value=pose)
        route.frame.pose = Mock(return_value=pose)
        route._wait_for_stopped_pose = Mock(return_value=pose)
        route._ir = Mock(side_effect=[25, 25, 26, 27, 29, 42, 33, 27, 26,
                                      None, None])
        def move(stm, token, poll):
            for _ in range(7):
                self.assertFalse(poll())
            route.frame.pose.return_value = Pose(137, -25, -math.pi / 2)
            self.assertFalse(poll())
            self.assertTrue(poll())
            return "STOPPED"
        with patch.object(t2, "run_polling", side_effect=move):
            measured = route.follow_face("F59", 165.2, 1, -1, 18)
        self.assertIsNotNone(measured)
        self.assertLess(measured[0], -15)
        route.stm.abort.assert_called_once()

    def test_edge_inside_known_face_extent_withholds_corner(self):
        route = self._face_probe([19, 19, None, None])
        route._wait_for_stopped_pose = Mock(return_value=route._pose())
        def edge(stm, token, poll):
            self.assertFalse(poll())
            self.assertTrue(poll())
            return "STOPPED"
        with patch.object(t2, "run_polling", side_effect=edge):
            self.assertIsNone(route.follow_face("F10", 175, 1, 1, 18,
                                               minimum_edge_y=40))
        route.stm.abort.assert_called_once()

    def test_fluctuating_right_ir_still_confirms_face_edge(self):
        """IRR stale face values must not cancel a majority-loss edge."""
        import task2t as task2
        route = task2.Task2(Mock(), startup=False, sleep_fn=lambda _: None)
        pose = Pose(150, 0, -math.pi / 2)
        route._pose = Mock(return_value=pose)
        route.frame.pose = Mock(return_value=pose)
        route._wait_for_stopped_pose = Mock(return_value=pose)
        # Video regression: a valid 20 cm echo flashes between empty-floor
        # values around 70-80 cm after the right IR has passed the object.
        route._ir = Mock(side_effect=[20, 20, 70, 20, 80, 74])

        def noisy_edge(stm, token, poll):
            self.assertFalse(poll())
            self.assertFalse(poll())
            self.assertTrue(poll())
            return "STOPPED"

        with patch.object(t2, "run_polling", side_effect=noisy_edge):
            measured = route.follow_face("F40", 126, -1, 1, 12)
        self.assertIsNotNone(measured)
        route._ir.assert_called_with("right")
        route.stm.abort.assert_called_once()

    def test_two_face_readings_cancel_a_false_edge_vote(self):
        """A real reacquisition must clear earlier one-sample IR noise."""
        route = self._face_probe([19, 19, None, 19, 19, 19, 19, 19, 19, 19])

        def completed(stm, token, poll):
            for _ in range(5):
                self.assertFalse(poll())
            return "OK"

        with patch.object(t2, "run_polling", side_effect=completed):
            self.assertIsNone(route.follow_face("F10", 126, -1, 1, 12))
        route.stm.abort.assert_not_called()

    def test_face_single_flash_is_not_confirmed(self):
        route = self._face_probe([19, None, None])
        def completed(stm, token, poll):
            self.assertFalse(poll())
            return "OK"
        with patch.object(t2, "run_polling", side_effect=completed):
            self.assertIsNone(route.follow_face("F10", 126, -1, 1, 12))
        self.assertIsNone(route.last_face_start_y)

    def test_ambiguous_reading_between_misses_does_not_reset_the_edge(self):
        """blank / leg-or-plateau / blank is two misses in three, an edge."""
        # Face at 19, window 12: 33 is neither the face (more than 12 off)
        # nor clearly past it (not over 19 + 12 + 3), so it is ambiguous.
        route = self._face_probe([19, 19, None, 33, None])
        route._wait_for_stopped_pose = Mock(return_value=route._pose())

        def edge(stm, token, poll):
            self.assertFalse(poll())
            self.assertFalse(poll())
            self.assertTrue(poll())
            return "STOPPED"

        with patch.object(t2, "run_polling", side_effect=edge):
            self.assertIsNotNone(route.follow_face("F10", 126, -1, 1, 12))
        route.stm.abort.assert_called_once()

    def test_ambiguous_readings_alone_never_make_an_edge(self):
        route = self._face_probe([19, 19] + [33] * 20)

        def completed(stm, token, poll):
            for _ in range(4):
                self.assertFalse(poll())
            return "OK"

        with patch.object(t2, "run_polling", side_effect=completed):
            route.follow_face("F10", 126, -1, 1, 12)
        route.stm.abort.assert_not_called()

    def test_face_edge_lag_scales_with_measured_speed(self):
        import task2t as task2
        results = {}
        for speed in (0.0, 60.0):
            route = task2.Task2(Mock(), startup=False, sleep_fn=lambda _: None)
            clock = [0.0]

            def tick():
                clock[0] += 0.01
                return clock[0]

            def pose():
                return Pose(150, speed * clock[0], math.pi / 2)

            route._pose = Mock(side_effect=pose)
            route.frame.pose = Mock(side_effect=pose)
            route._ir = Mock(side_effect=[19, 19, 19, 19, None, None])
            route._wait_for_stopped_pose = Mock(side_effect=pose)

            def edge(stm, token, poll):
                while not poll():
                    pass
                return "STOPPED"

            with patch.object(task2.time, "monotonic", side_effect=tick), \
                    patch.object(t2, "run_polling", side_effect=edge), \
                    self.assertLogs(level="INFO") as logs:
                results[speed] = route.follow_face("F40", 126, -1, 1, 12)
            results[speed] = (results[speed],
                              [m for m in logs.output if "Face edge lag" in m])
        self.assertIsNotNone(results[0.0][0])
        self.assertEqual(results[0.0][1], [])         # at rest: fixed IR_LAG_CM
        self.assertIsNotNone(results[60.0][0])
        self.assertEqual(len(results[60.0][1]), 1)
        self.assertIn("at 60 cm/s", results[60.0][1][0])

    def test_sensor_trace_records_each_poll_and_the_edge_decision(self):
        import csv
        import tempfile
        from sensor_trace import SensorTrace
        route = self._face_probe([19, 19, None, None])
        route._wait_for_stopped_pose = Mock(return_value=route._pose())
        with tempfile.TemporaryDirectory() as tmp:
            route.trace = SensorTrace(enabled=True, log_dir=tmp)

            def edge(stm, token, poll):
                self.assertFalse(poll())
                self.assertTrue(poll())
                return "STOPPED"

            with patch.object(t2, "run_polling", side_effect=edge):
                self.assertIsNotNone(route.follow_face("F10", 126, -1, 1, 12))
            route.trace.summary()
            with open(route.trace.path, newline="") as f:
                rows = list(csv.DictReader(f))
        self.assertEqual([r["phase"] for r in rows], ["rest", "rest", "move", "move"])
        self.assertEqual([r["decision"] for r in rows],
                         ["face", "face", "edge_vote", "stop:edge"])
        self.assertTrue(all(r["pose_rtt_ms"] != "" for r in rows))

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

    def test_live_selector_rejects_opposite_handoff_past_narrow_face(self):
        """Regression for 20:11: the live selector must apply the IR gate."""
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0
        pose = Pose(40.1, -20.4, math.radians(0.7))
        course = Course(69.7, 160.3, 90.0, wall_y=65.0)
        invalid = [t2.Seg("F", 65), t2.Seg("L", 89)]
        valid = [t2.Seg("R", 44), t2.Seg("L", 44),
                 t2.Seg("F", 22), t2.Seg("L", 89)]

        with patch.object(route, "face_handoff_candidates",
                          return_value=iter([(invalid, False),
                                             (valid, False)])), \
                patch.object(route, "_handoff_clearance",
                             return_value=9.5) as clearance:
            chosen = route.select_face_handoff(
                pose, course, -1.0, 1.0)

        self.assertEqual(chosen, (valid, False, 9.5))
        clearance.assert_called_once()

    def test_equal_clearance_does_not_add_a_needless_alignment_turn(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        simple = [t2.Seg("F", 20), t2.Seg("R", 85)]
        extra = [t2.Seg("L", 5), t2.Seg("F", 14), t2.Seg("R", 90)]
        with patch.object(route, "face_handoff_candidates",
                          return_value=iter([(simple, False), (extra, False)])), \
                patch.object(route, "_face_handoff_endpoint_valid", return_value=True), \
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
                patch.object(route, "_handoff_clearance", side_effect=[4.6, TimeoutError]), \
                patch.object(route, "_face_handoff_endpoint_valid", return_value=True):
            chosen = route.select_face_handoff(Pose(80, 25, 0), Course(84, 163, 90), 1, 1)
        self.assertEqual(chosen, (safe, False, 4.6))
        self.assertTrue(route.last_handoff_search["timed_out"])
        self.assertFalse(route.last_handoff_search["minimum_estimated_time_in_family"])
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

    def test_opposite_handoff_rejects_measured_heading_that_misses_face(self):
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.obs2_us_y = 5.7
        start = Pose(68.5, -22.3, 0)
        course = Course(60, 190, 44, wall_y=task2.CONSERVATIVE_WALL_Y)
        selected = route.select_face_handoff(start, course, -1, 1)
        self.assertIsNotNone(selected)
        segs, reverse, _ = selected
        nominal = t2.run(start, segs, route.r, step=1e6)[-1]
        self.assertTrue(route._face_handoff_endpoint_valid(
            nominal, [], course, -1, 1, reverse))
        measured = Pose(nominal.x, nominal.y,
                        nominal.th - math.radians(20))
        self.assertFalse(route._face_handoff_endpoint_valid(
            measured, [], course, -1, 1, reverse))

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
                               0.16 + 3 * __import__("task2t").STOP_POLL_S)  # three braking samples

    def test_narrow_two_moving_hits_then_clear_confirm_passed_short_edge(self):
        route = self._narrow_probe([30, 29, 56, 59])
        route._wait_for_stopped_pose = Mock(
            return_value=Pose(174, 43.4, 0))

        def scan(stm, line, poll):
            self.assertFalse(poll())
            self.assertTrue(poll())
            return "STOPPED"

        with patch.object(t2, "run_polling", side_effect=scan):
            face = route.find_narrow(90, 1)

        self.assertEqual(face, 170 + t2.IR_FROM_AXLE_CM - t2.IR_LAG_CM)
        route.stm.abort.assert_called_once()
        self.assertEqual(route._ir.call_count, 4)

    def test_narrow_one_hit_then_two_far_readings_confirm_trailing_edge(self):
        route = self._narrow_probe([24, 35, 36])
        route._wait_for_stopped_pose = Mock(
            return_value=Pose(174, 43.4, 0))

        def scan(stm, line, poll):
            self.assertTrue(poll())
            return "STOPPED"

        with patch.object(t2, "run_polling", side_effect=scan):
            face = route.find_narrow(90, 1)

        self.assertEqual(face, 170 + t2.IR_FROM_AXLE_CM - t2.IR_LAG_CM)
        route.stm.abort.assert_called_once()
        self.assertEqual(route._ir.call_count, 3)

    def test_narrow_false_hit_resumes_only_remaining_distance(self):
        route = self._narrow_probe([19, None, None, None, None, 19, 19, 19])
        route._pose.side_effect = [Pose(150, 43.4, 0), Pose(160, 43.4, 0),
                                  Pose(170, 43.4, 0)]
        route.frame.pose.side_effect = [Pose(165, 43.4, 0), Pose(175, 43.4, 0)]
        route._wait_for_stopped_pose = Mock(return_value=Pose(170, 43.4, 0))
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

    def test_logged_unknown_width_requires_sensing_before_forward_turn(self):
        import task2t as task2
        for side in (1.0, -1.0):
            with self.subTest(side=side):
                route = task2.Task2(Mock(), startup=False)
                route.r = 30.0
                route.obs2_us_y = side * 26.44408467668212
                pose = Pose(86.0, side * 26.8, math.radians(-side * 0.5))
                course = Course(83.7, 162.56, 90.0, wall_y=65.0)
                selected = route.select_face_handoff(pose, course, side, side)
                self.assertIsNone(selected)
                route._pose = Mock(return_value=pose)
                route.prepare_forward_lane = Mock(return_value=False)
                self.assertIsNone(route.around_obstacle2(course, side, side))
                route.prepare_forward_lane.assert_called_once_with(course, side)
                route.stm.send_line.assert_not_called()

    def test_confirmed_photo_does_not_search_or_reposition(self):
        import task2t as task2
        route = task2.Task2(Mock(), input_fn=lambda _: "left",
                           early_arrow_fn=lambda: "left", startup=False)
        route._pose = Mock(return_value=Pose(90, 27, 0))
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
                patch.dict("os.environ", {"T2_HANDOFF_PLAN_BUDGET_S": "1"}), \
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

    def test_parallel_photo_wait_sends_no_movement_or_abort(self):
        import task2t as task2
        route = task2.Task2(Mock(), input_fn=lambda _: "right", startup=False,
                           sleep_fn=lambda _: None, early_arrow_fn=Mock(
                               side_effect=[None, None, None, None, None, "right"]))
        route._pose = Mock(return_value=Pose(90, 27, 0))
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
        route.stm.abort.assert_not_called()

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
                self.assertEqual([s.kind for s in segs if s.kind != "F"], [turn, turn])
                self.assertTrue(any(s.kind == "F" for s in segs))
                trace = t2.run(pose, segs, route.r)
                self.assertAlmostEqual(trace[-1].th, -side * math.pi / 2)
                self.assertGreaterEqual(t2.path_clearance(trace, course.boxes())[0],
                                        t2.CLEARANCE_FLOOR_CM)

    def test_arrow_pairings_are_exact_left_right_mirrors(self):
        """Each arrow names the passing side; opposite arrows cross sides."""
        import task2t as task2

        course = Course(100.2, 182.2, 90,
                        wall_y=task2.CONSERVATIVE_WALL_Y)
        for first_side, first_turn, edge_ir in (
                (-1.0, "R", "right"), (1.0, "L", "left")):
            with self.subTest(first_side=first_side, pairing="same"):
                # After the checked outward sensing lane, matching arrows
                # advance first and begin toward their named passing side.
                same = task2.Task2(Mock(), startup=False)
                same.r = 30.0
                same.obs2_us_y = first_side * 24.6
                same.obs2_half_max = 31.2
                start = Pose(109.2, first_side * 31.1, 0.0)
                segs, reverse, clearance = same.select_face_handoff(
                    start, course, first_side, first_side)
                self.assertFalse(reverse)
                self.assertEqual(segs[0].kind, "F")
                self.assertEqual(next(seg.kind for seg in segs
                                      if seg.kind in ("L", "R")), first_turn)
                self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)

            with self.subTest(first_side=first_side, pairing="opposite"):
                # Opposite arrows cut across with a meaningful diagonal and
                # finish with the mirrored IR looking at the requested edge.
                crossing = task2.Task2(Mock(), startup=False)
                crossing.r = 30.0
                crossing.obs2_us_y = first_side * 24.6
                start = Pose(82.0, first_side * 24.6, 0.0)
                segs, reverse, clearance = crossing.select_face_handoff(
                    start, course, first_side, -first_side)
                turns = [seg.kind for seg in segs
                         if seg.kind in ("L", "R")]
                end = t2.run(start, segs, crossing.r, step=1e6)[-1]
                self.assertFalse(reverse)
                self.assertGreaterEqual(len(turns), 2)
                first_index = next(i for i, seg in enumerate(segs)
                                   if seg.kind in ("L", "R"))
                last_index = max(i for i, seg in enumerate(segs)
                                 if seg.kind in ("L", "R"))
                self.assertTrue(any(
                    seg.kind == "F" and seg.value >= 8
                    for seg in segs[first_index + 1:last_index]))
                self.assertAlmostEqual(end.th, -first_side * math.pi / 2)
                self.assertEqual(crossing._ir_facing(end.th, 1.0), edge_ir)
                self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)

    def test_matching_arrows_stop_on_the_mirrored_inward_ir(self):
        """A clear parallel lane advances until the IR facing obstacle 2 hits."""
        import task2t as task2

        course = Course(60, 140, 30)
        for side, sensor in ((1.0, "right"), (-1.0, "left")):
            with self.subTest(side=side):
                route = task2.Task2(Mock(), startup=False,
                                    sleep_fn=lambda _: None)
                start = Pose(100, side * 40, 0)
                hit = Pose(125, side * 40, 0)
                settled = Pose(126, side * 40, 0)
                route._pose = Mock(return_value=start)
                route.frame.pose = Mock(return_value=hit)
                route._check = Mock(return_value=10.0)
                route._ir = Mock(side_effect=[20.0, 20.0, 20.0])
                route._wait_for_stopped_pose = Mock(return_value=settled)

                def stopped(_stm, token, poll):
                    self.assertEqual(token, "F39")
                    self.assertTrue(poll())
                    return "STOPPED"

                with patch.object(task2.tp, "run_polling",
                                  side_effect=stopped):
                    measured = route.approach_same_side_until_ir(course, side)

                self.assertIsNotNone(measured)
                self.assertAlmostEqual(measured.x2, 140.0)
                self.assertAlmostEqual(route.obs2_near_y, side * 15.0,
                                       delta=0.3)
                self.assertEqual(route._ir.call_args_list,
                                 [call(sensor), call(sensor), call(sensor)])
                route.stm.abort.assert_called_once_with()

    def test_opposite_handoff_starts_ir_over_minimum_width_face(self):
        """Regression for the floor run whose right IR stayed at 42-68 cm."""
        import task2t as task2
        route = task2.Task2(Mock(), startup=False)
        route.r = 30.0
        route.obs2_us_y = -20.3
        pose = Pose(40.2, -20.3, 0.0)
        course = Course(69.7, 160.3, task2.CONSERVATIVE_W2,
                        wall_y=task2.CONSERVATIVE_WALL_Y)

        selected = route.select_face_handoff(pose, course, -1.0, 1.0)

        self.assertIsNotNone(selected)
        segs, reverse, clearance = selected
        end = t2.run(pose, segs, route.r)[-1]
        ir_y = end.y + t2.IR_FROM_AXLE_CM * math.sin(end.th)
        self.assertFalse(reverse)
        self.assertLessEqual(abs(ir_y - course.obs2_y),
                             task2.W2_RANGE[0] / 2.0 - 5.0)
        self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)
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
                    if self.lines and not self._pending:
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
                    if self.lines and not self._pending:
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



class MotionOptimisation(unittest.TestCase):
    def test_latest_oblique_us_hit_does_not_override_the_later_ir_edge(self):
        import task2t as route_module

        route = route_module.Task2(Mock(), startup=False)
        course = Course(58.4, 148.5, 90,
                        wall_y=route_module.CONSERVATIVE_WALL_Y)
        route._record_obs2_us_hit(-22.5, math.radians(-7.3))

        # The hit can still guide the immediate handoff, but an oblique,
        # drift-sensitive projection is not a hard width measurement.
        self.assertAlmostEqual(route._obs2_known_half(course), 22.5)
        self.assertAlmostEqual(route._obs2_minimum_edge_half(course), 15.0)

    def test_latest_camera_pose_keeps_opposite_arrow_forward(self):
        import task2t as route_module

        route = route_module.Task2(
            Mock(), startup=False, center_stage2=True)
        route.r = 30.0
        course = Course(58.4, 148.5, 90,
                        wall_y=route_module.CONSERVATIVE_WALL_Y)
        state = [Pose(59.0, 34.7, math.radians(-7.3))]
        route._record_obs2_us_hit(-22.5, state[0].th)
        sent = []
        route._pose = lambda: state[0]

        def move(segs):
            sent.append(list(segs))
            state[0] = t2.run(state[0], segs, route.r, step=1e6)[-1]
            return True

        route._line = move
        self.assertTrue(route.position_obstacle2_camera(course, 1.0))
        selected = route.select_face_handoff(state[0], course, 1.0, -1.0)

        self.assertIsNotNone(selected)
        segs, reverse, clearance = selected
        self.assertFalse(reverse)
        self.assertTrue(all(seg.kind in ("F", "L", "R") for seg in segs))
        self.assertGreaterEqual(clearance,
                                route_module.HANDOFF_CONTINUOUS_CLEARANCE_CM)
        # The complete camera move is checked; depending on available space
        # it may include a short final readjustment to create the turn pocket.
        self.assertGreaterEqual(route._check(
            Pose(59.0, 34.7, math.radians(-7.3)), sent[0], course),
            t2.CLEARANCE_FLOOR_CM)

    def test_0733_camera_standoff_absorbs_logged_hardware_drift(self):
        import task2t as route_module

        route = route_module.Task2(
            Mock(), startup=False, center_stage2=True)
        route.r = 30.0
        course = Course(60.6, 149.9, 90,
                        wall_y=route_module.CONSERVATIVE_WALL_Y)
        state = [Pose(78.2, 23.8, math.radians(-1.3))]
        route._record_obs2_us_hit(21.1, state[0].th)
        route._pose = lambda: state[0]

        def move(segs):
            state[0] = t2.run(state[0], segs, route.r, step=1e6)[-1]
            return True

        route._line = move
        self.assertTrue(route.position_obstacle2_camera(course, 1.0))

        # 07:33: nominal-to-measured camera error was +6.6 cm x, +4.8 cm y,
        # and -2 degrees. The old 30 cm pose had no route from this error.
        measured = Pose(state[0].x + 6.6, state[0].y + 4.8,
                        state[0].th - math.radians(2.0))
        route._handoff_cache = None
        selected = route.select_face_handoff(
            measured, course, 1.0, -1.0)

        self.assertIsNotNone(selected)
        segs, reverse, clearance = selected
        self.assertFalse(reverse)
        self.assertTrue(all(seg.kind in ("F", "L", "R") for seg in segs))
        self.assertGreaterEqual(clearance, 10.0)

    def test_short_handoff_preserves_narrow_face_and_clearance_both_sides(self):
        import task2t as route_module
        for side in (-1, 1):
            with self.subTest(side=side):
                route = route_module.Task2(Mock(), startup=False)
                route.r = 30.0
                start = Pose(42.0, side * 20.2, math.radians(-side * 1.2))
                course = Course(69.1, 159.2, 90,
                                wall_y=route_module.CONSERVATIVE_WALL_Y)
                selected = route.select_face_handoff(start, course, side, -side)
                self.assertIsNotNone(selected)
                segs, reverse, gap = selected
                self.assertFalse(reverse)
                for segs in [segs]:
                    self.assertEqual(sum(s.kind in ("L", "R") for s in segs), 2)
                    self.assertFalse(any(s.kind in ("B", "l", "r") for s in segs))
                    trace = t2.run(start, segs, route.r)
                    self.assertGreaterEqual(t2.path_clearance(trace, course.boxes())[0],
                                            route_module.HANDOFF_CLEARANCE_CM)
                    end = trace[-1]
                    ir_y = end.y + t2.IR_FROM_AXLE_CM * math.sin(end.th)
                    self.assertLessEqual(abs(ir_y), route_module.W2_RANGE[0] / 2 - 5)

    def test_logged_direct_return_is_shorter_clear_and_inside_garage(self):
        course = Course(69.1, 162, 74.7, wall_y=83.35)
        for side in (-1, 1):
            with self.subTest(side=side):
                start = Pose(167.1, side * 65.4, math.radians(180 - side * 0.7))
                direct = t2.plan_direct_return(start, course)
                self.assertIsNotNone(direct)
                self.assertEqual(sum(s.kind in ("L", "R") for s in direct.segs), 2)
                old = t2.plan_return(start, course)
                old_segs = old.line_a + old.final.segs + t2.park_segments(old.final.end)[0]
                new_segs = direct.segs + t2.park_segments(direct.end)[0]
                self.assertLess(t2.motion_cost(new_segs, t2.RADIUS_CM),
                                t2.motion_cost(old_segs, t2.RADIUS_CM))
                trace = t2.run(start, new_segs)
                self.assertGreaterEqual(t2.path_clearance(trace, course.boxes())[0], 8)
                self.assertTrue(_parked_ok(trace[-1]))

    def test_direct_return_rejects_blockage_or_bad_heading(self):
        course = Course(69.1, 162, 74.7, wall_y=83.35)
        self.assertIsNone(t2.plan_direct_return(Pose(167, -65, 0), course))
        with patch.object(t2, "path_clearance", return_value=(0, "obstacle")):
            self.assertIsNone(t2.plan_direct_return(Pose(167, -65, math.pi), course))

    def test_direct_return_retains_sensor_parking_and_fallback(self):
        frame, stm = Mock(), Mock()
        start = Pose(167.1, -65.4, math.radians(180.7))
        frame.pose.return_value = start
        course = Course(69.1, 162, 74.7, wall_y=83.35)
        runner = t2.ParkRunner(stm, frame, course)
        runner._send = Mock(return_value=True)
        runner._park_from_mouth = Mock(return_value=True)
        self.assertTrue(runner.run())
        runner._park_from_mouth.assert_called_once()
        with patch.object(t2, "plan_direct_return", return_value=None), \
                patch.object(t2, "plan_return", side_effect=t2.NoPlan("blocked")) as fallback:
            self.assertFalse(runner.run())
        self.assertEqual(fallback.call_count, 2)

    def test_logged_handoff_removes_reverse_and_preserves_ir_acquisition(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        route.r = 30.0
        angle = math.radians(-48.3)
        start = Pose(-45.5 + 40.3 * math.cos(angle) - 77.1 * math.sin(angle),
                     -40.3 * math.sin(angle) - 77.1 * math.cos(angle),
                     math.radians(-0.2))
        course = Course(69.4, 159.1, 90, wall_y=route_module.CONSERVATIVE_WALL_Y)
        selected = route.select_face_handoff(start, course, -1, 1)
        self.assertIsNotNone(selected)
        segs, reverse, clearance = selected
        self.assertFalse(reverse)
        self.assertFalse(any(s.kind in ("B", "l", "r") for s in segs))
        self.assertGreaterEqual(clearance, route_module.TURN_CLEARANCE_CM)
        self.assertTrue(route._face_handoff_endpoint_valid(start, segs, course, -1, 1, False))

    def test_opposing_straights_cancel_before_handoff_validation(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        start = Pose(80, 20, 0)
        original = [t2.Seg("F", 39), t2.Seg("B", 16), t2.Seg("L", 90)]
        with patch.object(route, "_turn_handoff_candidates",
                          return_value=[(original, False)]):
            reduced, reverse = next(route.face_handoff_candidates(
                start, Course(70, 170, 90), 1, 1))
        self.assertEqual(reduced, [t2.Seg("F", 23), t2.Seg("L", 90)])
        self.assertFalse(reverse)
        before, after = t2.run(start, original)[-1], t2.run(start, reduced)[-1]
        self.assertAlmostEqual(before.x, after.x)
        self.assertAlmostEqual(before.y, after.y)
        self.assertAlmostEqual(before.th, after.th)

    def test_pending_second_vote_retains_parallel_photo_pose(self):
        import task2t as route_module
        route = route_module.Task2(
            Mock(), startup=False, input_fn=lambda _: "left", sleep_fn=Mock(),
            early_arrow_fn=Mock(side_effect=[None] * 5 + ["left"]),
            arrow_pending_fn=lambda: True)
        route._pose = Mock(return_value=Pose(86, 21, 0))
        with patch.object(route, "photo_candidates") as photos, \
                patch.object(route, "_line") as move:
            self.assertEqual(route.photo_obstacle2(Course(80, 150, 90), 1), 1)
        photos.assert_not_called()
        move.assert_not_called()
        route.sleep_fn.assert_not_called()

    def test_logged_final_edge_coast_recovers_on_both_sides(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        course = Course(68.7, 161.5, 71, wall_y=81.5)
        for toward in (1, -1):
            with self.subTest(toward=toward):
                start = Pose(195.7, -toward * 37.8,
                             math.radians(-toward * 94.1))
                self.assertIsNone(route.turn_home(start, course, toward, True))
                recovery = route.turn_home_with_brake_recovery(start, course, toward)
                self.assertIsNotNone(recovery)
                self.assertEqual(recovery[0].kind, "B")
                poses = t2.run(start, recovery, route.r)
                self.assertGreaterEqual(t2.path_clearance(poses, course.boxes())[0],
                                        route_module.TURN_CLEARANCE_CM)
                error = math.atan2(math.sin(poses[-1].th - math.pi),
                                   math.cos(poses[-1].th - math.pi))
                self.assertLess(abs(math.degrees(error)), 0.5)

    def test_final_recovery_does_not_bypass_collision_check(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        with patch.object(route, "turn_home", side_effect=[None] + [[t2.Seg("R", 90)]] * 8), \
                patch.object(route_module, "_path_clear", return_value=None):
            self.assertIsNone(route.turn_home_with_brake_recovery(
                Pose(195, -40, -math.pi / 2), Course(70, 160, 70), 1))

    def test_215233_final_corner_keeps_safest_accepted_clearance(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        route.r = 30.0
        # Log: x2=161, edges -29.6..38.8, margin 2, y recentered by -4.6.
        course = Course(69.7, 161, 72.4, wall_y=82.2)
        for toward in (1, -1):
            with self.subTest(toward=toward):
                start = Pose(188.7, -toward * 37.2,
                             math.radians(-toward * 95.3))
                self.assertIsNone(route.turn_home(start, course, toward, True))
                recovered = route.turn_home_with_brake_recovery(start, course, toward)
                self.assertEqual(recovered, [t2.Seg("B", 2),
                                              t2.Seg("R" if toward > 0 else "L", 85)])
                trace = t2.run(start, recovered, route.r)
                clearance, _ = t2.path_clearance(trace, route._corner_boxes(course))
                self.assertAlmostEqual(clearance, 5.15, delta=0.05)
                # With the +/-3 cm face uncertainty, R4 is less clear than R2.
                old_trace = t2.run(start, [t2.Seg("B", 4), recovered[-1]], route.r)
                self.assertGreater(clearance, t2.path_clearance(
                    old_trace, route._corner_boxes(course))[0])
                self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)
                self.assertLess(clearance, route_module.TURN_CLEARANCE_CM)
                runner = t2.ParkRunner(Mock(), Mock(), course)
                runner.frame.pose.return_value = trace[-1]
                runner._send = Mock(return_value=True)
                runner._park_from_mouth = Mock(return_value=True)
                self.assertTrue(runner.run())
                runner._park_from_mouth.assert_called_once()

    def test_final_recovery_prefers_eight_over_floor_fallback(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        corner = [t2.Seg("R", 90)]
        with patch.object(route, "turn_home", side_effect=[None, corner, corner]), \
                patch.object(route_module, "_path_clear", side_effect=[5.6, 8.2]):
            self.assertEqual(route.turn_home_with_brake_recovery(
                Pose(195, -40, -math.pi / 2), Course(70, 160, 70), 1),
                [t2.Seg("B", 4)] + corner)

    def test_near_edge_recovery_keeps_accepted_floor_clearance(self):
        """Do not reject the 3.5 cm fallback with the preferred 8 cm again."""
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        corner = [t2.Seg("R", 90)]
        with patch.object(route, "round_near_end",
                          side_effect=[None] + [corner] * 8), \
                patch.object(route_module, "_path_clear",
                             return_value=5.6) as clear:
            recovered = route.round_near_end_with_brake_recovery(
                Pose(144.2, 38.0, math.radians(87.1)),
                Course(70, 160, 90), 1, False, 35.0)
        self.assertEqual(recovered, ([t2.Seg("B", 2)], corner))
        self.assertTrue(all(call.args[4] == t2.CLEARANCE_FLOOR_CM
                            for call in clear.call_args_list))

    def test_optional_diagonal_never_costs_more_or_relaxes_clearance(self):
        course = Course(90, 182, 50)
        for side in (1, -1):
            start = Pose(80, side * 45, math.pi)
            baseline = t2.plan_final(start, course)
            final = t2.shorten_final(start, course, baseline)
            def cost(plan):
                return t2._cost(plan.segs + t2.park_segments(plan.end)[0], t2.RADIUS_CM)
            self.assertLessEqual(cost(final), cost(baseline))
            poses = t2.run(start, final.segs + t2.park_segments(final.end)[0])
            self.assertGreaterEqual(t2.path_clearance(poses, course.boxes())[0],
                                    t2.CLEARANCE_CM)
            self.assertTrue(_parked_ok(poses[-1]))


class FlowConsistency(unittest.TestCase):
    def test_second_arrow_never_interrupts_required_bypass_positioning(self):
        import task2t as route_module
        for side, arrow in ((1, "left"), (-1, "right")):
            for already_ready in (False, True):
                with self.subTest(side=side, already_ready=already_ready):
                    result = [arrow if already_ready else None]
                    route = route_module.Task2(Mock(), startup=False,
                                              early_arrow_fn=lambda: result[0])
                    route._pose = Mock(side_effect=[Pose(0, 0, 0),
                        Pose(30, side * 21, 0), Pose(50, side * 21, 0)])
                    route._check = Mock(return_value=12)
                    route._us = Mock(return_value=79.5)
                    route._ir = Mock(return_value=None)
                    def move(segs):
                        if segs == [t2.Seg("F", 20)]:
                            result[0] = arrow
                        return True
                    route._line = Mock(side_effect=move)
                    with patch.object(t2, "run_polling") as polling:
                        self.assertEqual(route.pass_obstacle1(80, side), 150)
                    polling.assert_not_called()
                    route.stm.abort.assert_not_called()
                    self.assertEqual(route._line.call_args.args[0], [t2.Seg("F", 20)])
                    self.assertEqual(route.early_arrow_fn(), arrow)

    def test_return_ir_fix_updates_shared_frame_not_only_planning_pose(self):
        stm = Mock()
        frame = t2.WorldFrame(stm)
        frame.start = Pose(100, 20, 0)
        frame.origin = {"x_mm": 0, "y_mm": 0, "heading_deg": 0}
        stm.read_wpose.return_value = {"x_mm": 0, "y_mm": 0, "heading_deg": 180}
        runner = t2.ParkRunner(stm, frame, Course(60, 150, 60))
        runner._run_line_a = Mock(return_value=True)
        runner._finish = Mock(return_value=True)
        plan = Mock()
        plan.summary.return_value = "test return"
        fix = t2.IrFix(18, 2, 20, 5)
        with patch.object(t2, "plan_direct_return", return_value=None), \
                patch.object(t2, "plan_return", return_value=plan), \
                patch.object(t2, "ir_fix", return_value=fix):
            self.assertTrue(runner.run())
        self.assertEqual(frame.pose(), Pose(102, 18, math.pi))
        runner._finish.assert_called_once_with(frame.pose())

    def test_rejected_ir_fix_does_not_mutate_world_frame(self):
        stm = Mock()
        frame = t2.WorldFrame(stm)
        frame.start = Pose(100, 20, 0)
        frame.origin = {"x_mm": 0, "y_mm": 0, "heading_deg": 0}
        stm.read_wpose.return_value = {"x_mm": 0, "y_mm": 0, "heading_deg": 180}
        runner = t2.ParkRunner(stm, frame, Course(60, 150, 60))
        runner._run_line_a = Mock(return_value=True)
        runner._finish = Mock(return_value=True)
        plan = Mock()
        plan.summary.return_value = "test return"
        with patch.object(t2, "plan_direct_return", return_value=None), \
                patch.object(t2, "plan_return", return_value=plan), \
                patch.object(t2, "ir_fix", return_value=t2.IrFix(0, -35, 5, 5)):
            self.assertTrue(runner.run())
        self.assertEqual((frame.x_shift, frame.y_shift), (0, 0))

    def test_stable_us_requires_consecutive_valid_readings(self):
        for values, expected in [([15, None, 15, None], None),
                                  ([15, None, 15, 16], 15.5),
                                  ([176, 15, 16], 15.5),
                                  ([15, float("nan"), 15, 16], 15.5)]:
            with self.subTest(values=values):
                self.assertEqual(t2.stable_us(Mock(side_effect=values)), expected)

    def test_face_scan_failure_never_reenables_fast_mode(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        route._pose = Mock(return_value=Pose(100, 0, math.pi / 2))
        route.select_face_handoff = Mock(return_value=([t2.Seg("L", 90)], False, 10))
        route._execute_face_handoff = Mock(return_value=True)
        route.follow_face = Mock(return_value=None)
        route._set_fast = Mock()
        self.assertIsNone(route.around_obstacle2(Course(60, 150, 60), -1, 1))
        self.assertTrue(route._set_fast.call_args_list)
        self.assertTrue(all(c.args == (False,) for c in route._set_fast.call_args_list))

    def test_face_scan_refuses_failed_precision_switch(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        route.fast = True
        route._pose = Mock(return_value=Pose(100, 0, math.pi / 2))
        route.select_face_handoff = Mock(return_value=([t2.Seg("L", 90)], False, 10))
        route._execute_face_handoff = Mock(return_value=True)
        route.follow_face = Mock()
        route.stm.set_fast.return_value = False
        self.assertIsNone(route.around_obstacle2(Course(60, 150, 60), -1, 1))
        route.follow_face.assert_not_called()


class MeasuredCheckpointRegression(unittest.TestCase):
    def test_clear_forward_handoff_avoids_stop_induced_turn_error(self):
        import task2t as route_module
        route = route_module.Task2(Mock(), startup=False)
        end = t2.run(Pose(40, 20, 0),
                     [t2.Seg("R", 45), t2.Seg("F", 20)],
                     route.r, step=1e6)[-1]
        route.frame.pose = Mock(side_effect=[Pose(40, 20, 0), end, end])
        route._line = Mock(return_value=True)
        segs = [t2.Seg("R", 45), t2.Seg("F", 20)]
        course = Course(70, 160, 60)
        with patch.object(route, "_face_handoff_endpoint_valid", return_value=True), \
                patch.object(t2, "path_clearance", return_value=(9.0, "box")):
            self.assertTrue(route._execute_face_handoff(
                segs, course, 1, -1, False, 9.0))
        route._line.assert_called_once_with(segs)

    def test_long_return_straight_stays_one_continuously_monitored_move(self):
        frame = Mock()
        frame.pose.side_effect = [
            Pose(100, 50, math.pi),
            Pose(40, 50, math.pi),
        ]
        runner = t2.ParkRunner(Mock(), frame, Course(60, 130, 40))
        runner._ir = Mock(return_value=None)
        final = Mock(segs=[], end=Pose(0, 0, math.pi), clearance=9,
                     closest="wall", tokens=lambda: [])
        plan = t2.ReturnPlan(50, 1, "right", [t2.Seg("F", 60)], 40,
                             20, final, 9, "wall", 2)
        with patch.object(t2, "run_polling", return_value="OK"):
            self.assertTrue(runner._run_line_a(plan, []))
        self.assertEqual(t2.ParkRunner._checkpoint_segments(plan.line_a),
                         [t2.Seg("F", 60)])


class ContinuousReturnRegression(unittest.TestCase):
    def test_forward_return_stops_for_live_obstruction(self):
        frame = Mock()
        frame.pose.return_value = Pose(100, 50, math.pi)
        stm = Mock()
        runner = t2.ParkRunner(stm, frame, Course(60, 150, 40))
        runner._ir = Mock(return_value=None)
        runner._us = Mock(return_value=18)
        plan = Mock(line_a=[t2.Seg("F", 60)], side="right")

        def execute(stm, token, poll):
            self.assertEqual(token, "F60")
            self.assertTrue(poll())
            return "STOPPED"

        with patch.object(t2, "run_polling", side_effect=execute):
            self.assertFalse(runner._run_line_a(plan, []))
        stm.abort.assert_called_once()

    def test_forward_return_stops_if_remaining_sweep_drifts_into_box(self):
        frame = Mock()
        frame.pose.side_effect = [Pose(100, 50, math.pi), Pose(95, 15, math.pi)]
        stm = Mock()
        runner = t2.ParkRunner(stm, frame, Course(60, 150, 40))
        runner._ir = Mock(return_value=None)
        runner._us = Mock(return_value=100)
        plan = Mock(line_a=[t2.Seg("F", 60)], side="right")

        def execute(stm, token, poll):
            self.assertTrue(poll())
            return "STOPPED"

        with patch.object(t2, "run_polling", side_effect=execute):
            self.assertFalse(runner._run_line_a(plan, []))
        stm.abort.assert_called_once()

    def test_clear_forward_return_is_one_command_with_live_samples(self):
        frame = Mock()
        frame.pose.side_effect = [Pose(100, 50, math.pi), Pose(80, 50, math.pi),
                                  Pose(60, 50, math.pi), Pose(40, 50, math.pi)]
        stm = Mock()
        runner = t2.ParkRunner(stm, frame, Course(60, 150, 40))
        runner._ir = Mock(return_value=30)
        runner._us = Mock(return_value=100)
        plan = Mock(line_a=[t2.Seg("F", 60)], side="right")
        samples = []

        def execute(stm, token, poll):
            self.assertEqual(token, "F60")
            self.assertFalse(poll())
            self.assertFalse(poll())
            return "OK"

        with patch.object(t2, "run_polling", side_effect=execute) as motion:
            self.assertTrue(runner._run_line_a(plan, samples))
        motion.assert_called_once()
        self.assertEqual(len(samples), 2)
        stm.abort.assert_not_called()


class Centring(unittest.TestCase):
    def test_logged_short_return_acquires_mouth_once_then_parks(self):
        frame = Mock()
        frame.pose.return_value = Pose(7.2, 1.3, math.radians(183.5))
        runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
        runner._us = Mock(side_effect=[62.7, 62.7, 15.7, 15.7])
        runner._ir = Mock(return_value=None)
        runner._park_cruise = Mock(return_value=True)
        runner._send = Mock(return_value=True)
        self.assertTrue(runner._park_to_back_wall())
        runner._park_cruise.assert_called_once_with(
            47, frame.pose.return_value, 62.7, final=True)
        runner._send.assert_not_called()

    def test_logged_75cm_wall_echo_needs_bay_evidence_before_long_entry(self):
        frame = Mock()
        frame.pose.return_value = Pose(10.9, 0.0, math.radians(185.2))
        runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
        runner._us = Mock(side_effect=[74.7, 74.7, 15.4, 15.4])
        runner._ir = Mock(return_value=None)
        runner._park_cruise = Mock(return_value=True)

        self.assertFalse(runner._park_to_back_wall())
        runner._park_cruise.assert_not_called()

    def test_75cm_wall_echo_from_matching_approach_pose_enters_once(self):
        frame = Mock()
        frame.pose.return_value = Pose(45.2, 0, math.pi)
        runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
        runner._us = Mock(side_effect=[74.7, 74.7, 15.4, 15.4])
        runner._ir = Mock(return_value=None)
        runner._park_cruise = Mock(return_value=True)
        self.assertTrue(runner._park_to_back_wall())
        runner._park_cruise.assert_called_once_with(
            59, frame.pose.return_value, 74.7, final=True)

    def test_mouth_acquisition_rejects_background_drift_and_close_side(self):
        for gap, confirm, y, side in ((230, 230, 0, None), (63, 50, 0, None),
                                      (63, 63, 16, None), (63, 63, 0, 5)):
            frame = Mock()
            frame.pose.return_value = Pose(7.2, y, math.pi)
            runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
            runner._us = Mock(side_effect=[gap, confirm])
            runner._ir = Mock(return_value=side)
            runner._park_cruise = Mock()
            self.assertFalse(runner._park_to_back_wall())
            runner._park_cruise.assert_not_called()

    def test_logged_drifted_return_aligns_before_entering_bay(self):
        course = Course(72.7, 167.3, 65.6)
        for mirror in (-1, 1):
            p = Pose(1.4, mirror * 16.3, math.pi - mirror * math.radians(7.1))
            plan = t2.plan_centre(p, course)
            self.assertIsNotNone(plan)
            segs, _, clearance = plan
            self.assertEqual(segs[0].kind, "B")
            escape = t2.run(p, segs[:1])
            gaps = [t2.path_clearance([q], course.boxes())[0] for q in escape]
            self.assertGreaterEqual(min(gaps), 0)
            self.assertTrue(all(b >= a - .01 for a, b in zip(gaps, gaps[1:])))
            self.assertGreaterEqual(clearance, t2.CLEARANCE_FLOOR_CM)
            end = t2.run(p, segs)[-1]
            park, _ = t2.park_segments(end)
            self.assertTrue(_parked_ok(t2.run(end, park)[-1]))

    def test_logged_successful_mouth_does_not_gain_extra_motions(self):
        frame = Mock()
        frame.pose.return_value = Pose(8.4, 4.7, math.radians(182.1))
        runner = t2.ParkRunner(Mock(), frame, Course(76.4, 169.9, 75.8))
        runner._ir = Mock(return_value=None)
        runner._us = Mock(return_value=15)
        runner._send = Mock()
        runner._park_to_back_wall = Mock(return_value=True)
        with patch("task2t_park.time.sleep"):
            self.assertTrue(runner._park_from_mouth())
        runner._send.assert_not_called()
        runner._park_to_back_wall.assert_called_once()

    def test_safe_offset_mouth_skips_wobbly_centring_line(self):
        frame = Mock()
        frame.pose.return_value = Pose(8.0, 16.0, math.radians(184.0))
        stm = Mock()
        stm.set_fast.return_value = True
        runner = t2.ParkRunner(stm, frame, _course(60, 60, 90))
        runner._ir = Mock(side_effect=lambda side: 45.0 if side == "left" else 13.0)
        runner._send = Mock(return_value=True)
        runner._park_to_back_wall = Mock(return_value=True)

        with patch("task2t_park.time.sleep"):
            self.assertTrue(runner._park_from_mouth())

        runner._send.assert_not_called()
        runner._park_to_back_wall.assert_called_once()

    def test_missing_side_fences_go_directly_to_monitored_wall_approach(self):
        frame = Mock()
        frame.pose.return_value = Pose(3.5, 0, math.pi)
        stm = Mock()
        stm.set_fast.return_value = True
        runner = t2.ParkRunner(stm, frame, _course(60, 60, 90))
        runner._send = Mock(return_value=True)
        runner._ir = Mock(return_value=None)
        runner._park_to_back_wall = Mock(return_value=True)
        final = Mock(segs=[], end=Pose(3.5, 0, math.pi), clearance=10, closest="wall")
        with patch.object(t2, "plan_final", return_value=final), \
                patch.object(t2, "shorten_final", return_value=final), \
                patch("task2t_park.time.sleep"):
            self.assertTrue(runner._finish(frame.pose.return_value))
        self.assertEqual(runner._send.call_args_list, [unittest.mock.call([])])
        runner._park_to_back_wall.assert_called_once()

    def test_long_parking_approach_uses_one_cruise_and_one_trim(self):
        frame = Mock()
        frame.pose.return_value = Pose(0, 0, math.pi)
        runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
        runner._us = Mock(side_effect=[55.0, 55.0, 15.7, 15.7])
        runner._ir = Mock(return_value=24.0)
        runner._park_cruise = Mock(return_value=True)
        runner._send = Mock(return_value=True)
        with patch("task2t_park.time.sleep"):
            self.assertTrue(runner._park_to_back_wall())
        runner._park_cruise.assert_called_once_with(
            40, frame.pose.return_value, 55.0, final=True)
        runner._send.assert_not_called()

    def test_unstable_wall_echo_does_not_start_long_parking_move(self):
        frame = Mock()
        frame.pose.return_value = Pose(0, 0, math.pi)
        runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
        runner._us = Mock(side_effect=[55.0, 35.0])
        runner._ir = Mock(return_value=24.0)
        runner._park_cruise = Mock()
        self.assertFalse(runner._park_to_back_wall())
        runner._park_cruise.assert_not_called()

    def test_parking_cruise_monitors_sensors_and_aborts_on_fault(self):
        for gap, side, heading in ((45.0, 24.0, math.pi),
                                   (None, 24.0, math.pi),
                                   (10.0, 24.0, math.pi),
                                   (45.0, 5.0, math.pi),
                                   (45.0, 24.0, 0.0)):
            with self.subTest(gap=gap, side=side, heading=heading):
                frame = Mock()
                frame.pose.return_value = Pose(-10, 0, heading)
                runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
                runner._us = Mock(return_value=gap)
                runner._ir = Mock(return_value=side)
                def execute(stm, line, poll):
                    self.assertEqual(line, "F30")
                    return "STOPPED" if poll() else "OK"
                with patch.object(t2, "run_polling", side_effect=execute), \
                        patch("task2t_park.time.sleep"):
                    okay = runner._park_cruise(30, Pose(0, 0, math.pi), 55)
                expected = gap == 45.0 and side == 24.0 and heading == math.pi
                self.assertEqual(okay, expected)
                self.assertEqual(runner.stm.abort.call_count, 0 if expected else 1)

    def test_parking_confirms_far_spike_without_extending_or_restarting_motion(self):
        for echoes, expected in [([176.7, 45.0], True),
                                  ([176.7, 177.0], False),
                                  ([176.7, 10.0], False),
                                  ([176.7, None], False),
                                  ([10.0], False)]:
            with self.subTest(echoes=echoes):
                frame = Mock()
                frame.pose.return_value = Pose(-10, 0, math.pi)
                runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
                runner._us = Mock(side_effect=echoes)
                runner._ir = Mock(return_value=24.0)
                def execute(stm, line, poll):
                    return "STOPPED" if poll() else "OK"
                with patch.object(t2, "run_polling", side_effect=execute) as movement, \
                        patch("task2t_park.time.sleep"):
                    self.assertEqual(
                        runner._park_cruise(30, Pose(0, 0, math.pi), 55), expected)
                self.assertEqual(movement.call_count, 1)
                self.assertEqual(movement.call_args.args[1], "F30")
                self.assertEqual(runner._us.call_count, len(echoes))
                self.assertEqual(runner.stm.abort.call_count, 0 if expected else 1)

    def test_parking_far_echo_retry_cannot_delay_other_safety_stops(self):
        for side, heading in [(5.0, math.pi), (24.0, 0.0)]:
            with self.subTest(side=side, heading=heading):
                frame = Mock()
                frame.pose.return_value = Pose(-10, 0, heading)
                runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
                runner._us = Mock(return_value=176.7)
                runner._ir = Mock(return_value=side)
                def execute(stm, line, poll):
                    return "STOPPED" if poll() else "OK"
                with patch.object(t2, "run_polling", side_effect=execute), \
                        patch("task2t_park.time.sleep"):
                    self.assertFalse(runner._park_cruise(30, Pose(0, 0, math.pi), 55))
                runner._us.assert_called_once()
                runner.stm.abort.assert_called_once()

    def test_parking_cruise_timeout_aborts(self):
        runner = t2.ParkRunner(Mock(), Mock(), _course(60, 60, 90))
        with patch.object(t2, "run_polling", return_value=None):
            self.assertFalse(runner._park_cruise(30, Pose(0, 0, math.pi), 55))
        runner.stm.abort.assert_called_once()


    def test_missing_side_fence_uses_one_confirmed_final_move(self):
        frame = Mock()
        frame.pose.return_value = t2.Pose(-2.0, 0.0, math.pi)
        runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
        runner._us = Mock(side_effect=[31.0, 31.0, None, 15.7, 15.7])
        runner._ir = Mock(side_effect=[None, 60.0])
        runner._park_cruise = Mock(return_value=True)

        with patch("task2t_park.time.sleep"):
            self.assertTrue(runner._park_to_back_wall())

        runner._park_cruise.assert_called_once_with(
            16, frame.pose.return_value, 31.0, final=True)

    def test_final_parking_move_refuses_unstable_echo_or_side_fence(self):
        for echoes, y in [([31.0, 25.0], 0.0), ([31.0, 31.0], 22.0)]:
            with self.subTest(echoes=echoes, y=y):
                frame = Mock()
                frame.pose.return_value = t2.Pose(-2.0, y, math.pi)
                runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
                runner._us = Mock(side_effect=echoes)
                runner._ir = Mock(return_value=None if y else 24.0)
                runner._send = Mock(return_value=True)
                self.assertFalse(runner._park_to_back_wall())
                runner._send.assert_not_called()

    def test_us_parking_keeps_present_side_ir_as_hard_stop(self):
        frame = Mock()
        frame.pose.return_value = t2.Pose(-2.0, 0.0, math.pi)
        runner = t2.ParkRunner(Mock(), frame, _course(60, 60, 90))
        runner._us = Mock(return_value=25.0)
        runner._ir = Mock(side_effect=[9.0, None])
        runner._send = Mock(return_value=True)

        self.assertFalse(runner._park_to_back_wall())
        runner._send.assert_not_called()

    def test_offset_from_side_irs(self):
        c = 2 * (t2.BAY_HALF_WIDTH_CM - t2.IR_SIDE_OFFSET_CM) / 2
        self.assertAlmostEqual(t2.centre_offset(c + 4, c - 4), 4.0)
        self.assertIsNone(t2.centre_offset(c, None))
        self.assertIsNone(t2.centre_offset(c, c + 25))     # not both fences
        self.assertAlmostEqual(t2.single_fence_offset(None, c + 10), -10.0)
        self.assertAlmostEqual(t2.single_fence_offset(c + 8, None), 8.0)

    def test_parks_safely_without_centring_wobble_despite_odometry_drift(self):
        # 3 cm of sideways drift per metre can leave the robot off centre at
        # the mouth. If its complete straight sweep still clears both fences,
        # parking should remain one forward push instead of adding an S.
        import task2t_sim as ts
        for case in ((60, 60, 90, "left"), (90, 82, 60, "right")):
            code, fake, course = ts.simulate(*case, drift_cm_per_m=3.0)
            self.assertEqual(code, 0, case)
            self.assertTrue(_parked_ok(fake.pose), f"{case}: {fake.pose}")
            clearance, _ = t2.path_clearance([fake.pose], course.fence())
            self.assertGreaterEqual(clearance, t2.PARK_STRAIGHT_CLEARANCE_CM,
                                    f"{case}: {fake.pose}")


if __name__ == "__main__":
    unittest.main()
