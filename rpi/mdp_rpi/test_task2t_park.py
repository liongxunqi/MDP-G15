"""
Laptop tests for task2t_park.py, the TIGHT copy - no robot, no serial.

    python -m unittest test_task2t_park -v

The sweep covers the brief's whole range: obstacle 1 at 60-150 cm from the
bay, obstacle 2 another 60-150 cm on, 30-90 cm wide, home on either side.
"""

import math
import unittest

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
                            final = t2.plan_final(here, c)
                            case = (d1, w2, side, dx, dy)
                            self.assertGreater(final.clearance, 3.0,
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


class EndToEnd(unittest.TestCase):
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

    def test_runs_and_parks(self):
        # Obstacle 2's arrow both ways: away from the robot's side (it runs
        # along the face forward) and toward it (in reverse, then the
        # reverse-arc opening round that end).
        import task2t_sim as ts
        for d1, d2, w2, arrow, rerr in self.CASES:
            for arrow2 in ("left", "right"):
                case = (d1, d2, w2, arrow, arrow2, rerr)
                code, fake, course = ts.simulate(d1, d2, w2, arrow, rerr,
                                                 arrow2=arrow2)
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
            self.assertEqual(code, 0, case)
            clear, name = ts.run_clearance(fake, course)
            self.assertGreater(clear, t2.CLEARANCE_FLOOR_CM - 0.5, f"{case}: {name}")
            self.assertTrue(_parked_ok(fake.pose), f"{case}: {fake.pose}")



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
