"""
Laptop tests for task2mn_gordon.py (Task 2 version 2) - no robot, no serial.

    python -m unittest test_task2mn_gordon -v
    T2G_FULL_SWEEP=1 python -m unittest test_task2mn_gordon -v   # all 108 layouts

Run them in a checkout of task2-working's rpi/mdp_rpi with the task2mn files
added (they need its communications/ and run_log.py).
"""

import logging
import math
import os
import sys
import types
import unittest
from unittest.mock import patch

# communications.stm imports pyserial; the laptop may not have it.
if "serial" not in sys.modules:
    _serial = types.ModuleType("serial")
    _serial.Serial = object
    _serial.SerialException = Exception
    sys.modules["serial"] = _serial

import task2mn_gordon as g
import task2mn_gordon_sim as gs
import task2mn_park as tp
from task2mn_park import Pose, Seg

ARROWS = gs.ARROWS
FULL = os.getenv("T2G_FULL_SWEEP", "0") == "1"
# A spread of the sweep that covers every branch: narrow / wide, same /
# opposite arrows, obstacle 1 close to the bay, obstacle 2 close and far.
SAMPLE = [(60, 60, 30), (60, 150, 60), (90, 90, 60), (90, 60, 90),
          (150, 90, 30), (150, 150, 90), (90, 150, 30), (60, 90, 90)]


def _layouts():
    if FULL:
        return [(d1, d2, w2) for d1 in gs.LAYOUT_D for d2 in gs.LAYOUT_D
                for w2 in gs.LAYOUT_W]
    return SAMPLE


class Geometry(unittest.TestCase):

    def test_compact_merges_straights_and_same_way_arcs_only(self):
        segs = [Seg("F", 10), Seg("F", 5), Seg("R", 90), Seg("R", 90),
                Seg("R", 30), Seg("L", 20), Seg("F", 0)]
        self.assertEqual(g.tokens(g.compact(segs)), "F15,FR180,FR30,FL20")

    def test_line_limits_follow_the_protocol(self):
        self.assertTrue(g.fits_one_line([Seg("F", 100)] * 16))
        self.assertFalse(g.fits_one_line([Seg("F", 100)] * 17))

    def test_arc_to_turns_the_short_or_the_asked_way(self):
        p = Pose(0, 0, 0.0)
        segs, q = g._arc_to(p, math.pi / 2, "L", 30.0)
        self.assertEqual(g.tokens(segs), "FL90")
        self.assertAlmostEqual(q.th, math.pi / 2, places=6)
        segs, q = g._arc_to(p, math.pi / 2, "R", 30.0)
        self.assertEqual(g.tokens(segs), "FR270")
        segs, _ = g._arc_to(Pose(0, 0, math.radians(2)), 0.0, "L", 30.0)
        self.assertEqual(segs, [])            # already just past: no arc

    def test_obstacle_box_is_the_widest_and_walls_the_narrowest(self):
        info = g.Obs2(200.0, 26.0, 45.0)
        boxes = {b.name: b for b in info.boxes(90.0)}
        self.assertEqual(boxes["obstacle 2"].y1, 45.0)
        self.assertEqual(boxes["left wall"].y0, 76.0)
        self.assertEqual(boxes["right wall"].y1, -76.0)

    def test_an_echo_says_where_obstacle_2_is_not_how_wide(self):
        # The beam is a cone (floor, 2026-10-15): any width is still open.
        info = g.GordonRoute._wide_info({"x2": 200.0, "us_y": 28.0})
        self.assertEqual((info.h_lo, info.h_hi), (g.HALF_W2_MIN, g.HALF_W2_MAX))
        # A miss would still leave a half-width the narrow scan lane clears.
        narrow = g.narrow_info(90.0)
        self.assertGreaterEqual(g.NARROW_LANE_CM - tp.HALF_WIDTH_CM - narrow.h_hi,
                                g.PLAN_CLEAR_CM)

    def test_the_face_scan_line_is_clear_for_every_width(self):
        info = g.Obs2(200.0, g.HALF_W2_MIN, g.HALF_W2_MAX)
        for side1 in (1.0, -1.0):
            with self.subTest(side1=side1):
                p = Pose(200.0 - 75.0, side1 * g.BYPASS_LANE_CM, 0.0)
                got = g.face_line(p, 90.0, info, -side1)
                self.assertIsNotNone(got)
                end = tp.run(p, got[0])[-1]
                # Turned in toward the centreline, IR on the face.
                self.assertAlmostEqual(math.degrees(end.th), -side1 * 90.0, delta=1)

    def test_planned_loops_keep_the_clearance_they_claim(self):
        x1 = 90.0
        # After the face scan: stopped heading in, the end measured.
        cases = [(Pose(173, -12, -math.pi / 2), g.Obs2(200, 29, 33, measured=True),
                  -1.0, ("front", "outer")),           # opposite: on round it
                 (Pose(173, -12, -math.pi / 2), g.Obs2(200, 29, 33, measured=True),
                  1.0, ("reverse_out",)),              # same: back out to +y
                 (Pose(173, 30, math.pi / 2), g.Obs2(200, 44, 48, measured=True),
                  -1.0, ("reverse_out",)),
                 (Pose(173, -4, -math.pi / 2), g.Obs2(200, 15, 23, measured=True),
                  1.0, ("reverse_out",))]              # narrow: no face seen
        for p, info, side, fams in cases:
            with self.subTest(p=str(p), side=side, fams=fams):
                plan = g.plan_go_around(p, x1, info, side, families=fams,
                                        allow_reverse=True)
                self.assertIsNotNone(plan)
                self.assertGreaterEqual(plan.clearance, g.MIN_CLEAR_CM - 0.05)
                poses = tp.run(p, plan.segments())
                worst, _ = tp.path_clearance(
                    poses, [b for b in info.boxes(x1) if b.name != "back fence"])
                self.assertGreaterEqual(worst, tp.CLEARANCE_FLOOR_CM)
                for line in plan.lines():
                    self.assertTrue(g.fits_one_line(line), g.tokens(line))

    def test_reverse_out_never_drives_toward_the_wall_first(self):
        p = Pose(173, -12, -math.pi / 2)
        info = g.Obs2(200, 29, 33, measured=True)
        plan = g.plan_go_around(p, 90.0, info, 1.0, families=("reverse_out",))
        self.assertIsNotNone(plan)
        self.assertEqual(plan.segments()[0].kind, "B")
        self.assertIn(plan.segments()[1].kind, "lr")


class EndToEnd(unittest.TestCase):
    """Whole runs on the simulator: obstacle 1 to parked."""

    @classmethod
    def setUpClass(cls):
        cls.results = {}
        for d1, d2, w2 in _layouts():
            for a1, a2 in ARROWS:
                cls.results[(d1, d2, w2, a1, a2)] = gs.simulate(d1, d2, w2, a1, a2)

    def test_every_layout_parks_centred_without_touching_anything(self):
        for key, res in self.results.items():
            with self.subTest(layout=key):
                self.assertEqual(res["code"], 0)
                self.assertGreaterEqual(res["clear"], tp.CLEARANCE_FLOOR_CM,
                                        "%s at %.1f cm" % (res["closest"], res["clear"]))
                self.assertLessEqual(abs(res["y"]), 4.0)
                self.assertLessEqual(abs(res["gap"] - tp.PARK_GAP_CM),
                                     tp.PARK_GAP_TOL_CM)
                self.assertLessEqual(res["heading_err"], tp.PARK_ALIGN_TOL_DEG)

    def test_every_line_respects_the_firmware_line_limits(self):
        for key, res in self.results.items():
            for rec in res["fake"].lines:
                with self.subTest(layout=key, line=rec["line"]):
                    self.assertLessEqual(len(rec["segs"]), 16)
                    self.assertLessEqual(len(rec["line"]) + 1, 128)

    def test_no_task2mn_fallback_was_needed(self):
        for key, res in self.results.items():
            with self.subTest(layout=key):
                self.assertNotIn("fallback", [e[0] for e in res["events"]])

    def test_far_fewer_lines_than_task2mn(self):
        for key, res in self.results.items():
            with self.subTest(layout=key):
                self.assertLessEqual(res["lines"], 7)


class Faster(unittest.TestCase):

    def test_estimated_time_beats_task2mn_on_the_sample(self):
        saved = []
        for d1, d2, w2 in SAMPLE[:4]:
            for a1, a2 in (("left", "right"), ("right", "right")):
                base = gs.simulate(d1, d2, w2, a1, a2, "baseline")
                new = gs.simulate(d1, d2, w2, a1, a2, "gordon")
                with self.subTest(layout=(d1, d2, w2, a1, a2)):
                    self.assertEqual(new["code"], 0)
                    if base["code"] != 0:
                        # task2mn did not finish (the wide ultrasound beam):
                        # its time is not a time to beat.
                        continue
                    # Same arrows on a wide obstacle can cost ~0.5 s on
                    # protocol 9 (measuring the width, then reversing out);
                    # task2mn saves it by passing at the 3.5 cm floor.
                    self.assertLess(new["physics_s"], base["physics_s"] + 1.0)
                saved.append(base["physics_s"] - new["physics_s"])
        self.assertGreater(sum(saved) / len(saved), 2.0)


class Robustness(unittest.TestCase):

    def test_no_plan_stops_rather_than_hand_over_by_default(self):
        with patch.object(g, "face_line", return_value=None):
            res = gs.simulate(90, 90, 60, "left", "right")
        self.assertEqual(res["code"], 1)
        self.assertNotIn("fallback", [e[0] for e in res["events"]])
        self.assertGreaterEqual(res["clear"], tp.CLEARANCE_FLOOR_CM)

    def test_no_measuring_line_hands_over_to_task2mn_from_the_lane(self):
        # T2G_FALLBACK=1: gordon cannot plan its measuring line, the robot is
        # still on the bypass lane, where task2mn's own route starts from.
        # (With the cone ultrasound that route may not finish - the point is
        # that the hand-over happens and nothing is touched.)
        for arrows in (("left", "right"), ("left", "left")):
            with self.subTest(arrows=arrows), \
                    patch.object(g, "face_line", return_value=None), \
                    patch.object(g, "FALLBACK", True):
                res = gs.simulate(90, 90, 60, *arrows)
                self.assertIn("fallback", [e[0] for e in res["events"]])
                self.assertGreaterEqual(res["clear"], tp.CLEARANCE_FLOOR_CM)

    def test_late_vote_stops_and_waits_instead_of_guessing(self):
        res = gs.simulate(90, 90, 60, "left", "right", vote_cm=400.0)
        self.assertEqual(res["code"], 0)
        self.assertGreater(res["wait_s"], 0.0)
        self.assertIn("wait_arrow", [e[0] for e in res["events"]])

    def test_slow_background_planner_falls_back_to_a_safe_stop(self):
        res = gs.simulate(90, 150, 90, "right", "right", plan_latency_cm=1000.0)
        self.assertEqual(res["code"], 0)
        self.assertGreaterEqual(res["clear"], tp.CLEARANCE_FLOOR_CM)

    def test_drift_two_cm_per_metre_is_survived_on_a_spread(self):
        for d1, d2, w2 in SAMPLE[:4]:
            for drift in (2.0, -2.0):
                res = gs.simulate(d1, d2, w2, "left", "right", drift=drift)
                with self.subTest(layout=(d1, d2, w2), drift=drift):
                    if res["code"] == 0:
                        self.assertGreaterEqual(res["clear"], 0.0)
                        self.assertLessEqual(abs(res["y"]), 6.0)


class Protocol10(unittest.TestCase):
    """task2mn_gordon_link: the firmware's protocol 10 rules and the model
    of a continuous line (PROTOCOL.md section 13)."""

    def test_tokens_round_trip_and_old_tokens_stay_old(self):
        import task2mn_gordon_link as gl
        for tok in ("F120V20C", "FR90V15C", "RL45V10", "R30V8", "F120", "FR90"):
            self.assertEqual(gl.parse_token(tok).token(), tok)
        self.assertEqual(gl.parse_token("FR90"), gl.Move(Seg("R", 90)))

    def test_line_rules_match_the_firmware_parser(self):
        import task2mn_gordon_link as gl
        good = (["F120V20C", "FR90V15C", "F40V15"], ["F120", "FR90"],
                ["R20V8C", "RR30V10"], ["F50V20C", "F30V25"])
        bad = (["F50V20C"],                       # C on the last token
               ["F50V20C", "FR90"],               # next token has no V
               ["F50V20C", "R10V10"],             # C into a reversal
               ["F50C", "F10V10"],                # C without V
               ["FR90V18"], ["FR90V5"],           # arc speed out of range
               ["F50V26"], ["F50V3"],             # straight speed out of range
               ["F0V10"], ["FU20V10"],            # F0 / FU take no suffix
               ["F050V10"], ["F50V08"],           # leading zeros
               ["F10V10C"] * 16 + ["F10V10"])     # 17 primitives
        for line in good:
            with self.subTest(line=line):
                self.assertTrue(gl.check_line(line)[0], gl.check_line(line)[1])
        for line in bad:
            with self.subTest(line=line):
                self.assertFalse(gl.check_line(line)[0])

    def test_hand_over_speed_is_the_lower_of_the_two(self):
        import task2mn_gordon_link as gl
        moves = [gl.parse_token(t) for t in ("F120V25C", "FR90V15C", "F40V20")]
        self.assertEqual(gl.exit_speed(moves, 0), 15)
        self.assertEqual(gl.exit_speed(moves, 1), 15)
        self.assertEqual(gl.exit_speed(moves, 2), 0)

    def test_a_line_of_stops_flies_exactly_the_planned_path(self):
        import task2mn_gordon_link as gl
        p = Pose(10, -5, 0.3)
        segs = [Seg("F", 50), Seg("R", 90), Seg("F", 30), Seg("L", 45),
                Seg("l", 30), Seg("B", 20)]
        ideal = tp.run(p, segs)[-1]
        flown = gl.fly_path(p, [gl.Move(s) for s in segs])[-1]
        self.assertAlmostEqual(ideal.x, flown.x, places=1)
        self.assertAlmostEqual(ideal.y, flown.y, places=1)
        self.assertAlmostEqual(ideal.th, flown.th, places=3)

    def test_continued_hand_overs_keep_the_commanded_heading(self):
        import task2mn_gordon_link as gl
        p = Pose(0, 0, 0.0)
        segs = [Seg("F", 50), Seg("R", 90), Seg("F", 30), Seg("L", 45),
                Seg("R", 45), Seg("F", 40)]
        moves = gl.speedify(p, segs, [], 5.0)
        self.assertTrue(all(m.carry for m in moves[:-1]))
        self.assertFalse(moves[-1].carry)
        end = gl.fly_path(p, moves)[-1]
        self.assertAlmostEqual(math.degrees(end.th), -90.0, delta=1.0)

    def test_speedify_stops_a_hand_over_whose_spiral_cuts_clearance(self):
        import task2mn_gordon_link as gl
        p = Pose(0, 0, 0.0)
        segs = [Seg("F", 40), Seg("R", 90), Seg("F", 40)]
        ideal = tp.run(p, segs)
        # A box just outside the ideal sweep: the ideal (all stops) clears
        # it, a continued hand-over must not be allowed to.
        free = gl.speedify(p, segs, [], 5.0)
        worst_c, _ = tp.path_clearance(gl.fly_path(p, free),
                                       [tp.Box(55, 75, -60, -20, "x")])
        need = tp.path_clearance(ideal, [tp.Box(55, 75, -60, -20, "x")])[0]
        if worst_c < need - 0.01:
            moves = gl.speedify(p, segs, [tp.Box(55, 75, -60, -20, "x")], need - 0.01)
            self.assertFalse(all(m.carry for m in moves[:-1]))
            flown = tp.path_clearance(gl.fly_path(p, moves),
                                      [tp.Box(55, 75, -60, -20, "x")])[0]
            self.assertGreaterEqual(flown, need - 0.01)

    def test_lane_landing_trim_puts_the_kick_on_its_lane(self):
        import task2mn_gordon_link as gl
        p = Pose(0, 0, 0.0)
        segs = tp.lane_change(0.0, 28.0) + [Seg("F", 60)]
        loose = gl.speedify(p, segs, [], 5.0)
        landed = gl.speedify(p, segs, [], 5.0, land_y=28.0)
        self.assertLess(abs(gl.fly_path(p, landed)[-1].y - 28.0),
                        abs(gl.fly_path(p, loose)[-1].y - 28.0) + 1e-9)
        self.assertLess(abs(gl.fly_path(p, landed)[-1].y - 28.0), 1.0)

    def test_speed_model_prices_a_stopped_arc_as_motion_cost_does(self):
        import task2mn_gordon_link as gl
        arc = gl.line_seconds([gl.Move(Seg("R", 90))])
        self.assertAlmostEqual(arc, tp.motion_cost([Seg("R", 90)], tp.RADIUS_CM),
                               places=2)
        stops = gl.line_seconds([gl.Move(s) for s in (Seg("F", 60), Seg("R", 90),
                                                      Seg("F", 60))])
        flying = gl.line_seconds(gl.speedify(Pose(0, 0, 0.0),
                                             [Seg("F", 60), Seg("R", 90), Seg("F", 60)],
                                             [], 5.0))
        self.assertLess(flying, stops - 0.9)

    def test_enable_teaches_the_shared_client_the_new_tokens(self):
        import task2mn_gordon_link as gl
        from communications import stm
        gl.enable()
        gl.enable()                                  # idempotent
        self.assertTrue(stm.validate_token("FR90V15C")[0])
        self.assertFalse(stm.validate_token("FR90V18")[0])
        self.assertTrue(stm.validate_token("F10")[0])
        self.assertTrue(stm.validate_token("?SEG")[0])
        line = ["FR90V15C"] * 15 + ["F10V10"]
        self.assertTrue(stm.validate_line(line)[0])  # 136 bytes: protocol 10 allows

    def test_gordon10_parks_with_far_fewer_stops(self):
        for d1, d2, w2 in SAMPLE[:4]:
            for a1, a2 in (("left", "right"), ("right", "right")):
                with self.subTest(layout=(d1, d2, w2, a1, a2)):
                    old = gs.simulate(d1, d2, w2, a1, a2, "gordon")
                    new = gs.simulate(d1, d2, w2, a1, a2, "gordon10")
                    self.assertEqual(new["code"], 0)
                    self.assertGreaterEqual(new["clear"], tp.CLEARANCE_FLOOR_CM)
                    self.assertLessEqual(abs(new["y"]), 4.0)
                    self.assertLess(new["stops"], old["stops"] - 5)
                    self.assertLess(new["physics_s"], old["physics_s"])
                    for rec in new["fake"].lines:
                        import task2mn_gordon_link as gl
                        toks = rec["line"].split(",")
                        if rec["p10"]:
                            self.assertTrue(gl.check_line(toks)[0], rec["line"])


class OnTheRobot(unittest.TestCase):

    def test_task2mn_entry_point_runs_the_gordon_route(self):
        """task2mn.Task2 (vision, status, parking verification) over
        GordonRoute, as main() builds it, end to end on the fake STM."""
        import task2mn
        import task2mn_sim as ts

        class Vision:
            def __init__(self, arrows):
                self.arrows = iter(arrows)
                self.stage = 0

            def begin(self, stage):
                self.stage = stage

            def idle(self):
                self.stage = 0

            def wait_arrow(self, timeout=8.0):
                return next(self.arrows)

            def peek_arrow(self):
                return "right" if self.stage == 2 else None

            def arrow_pending(self):
                return False

        class Stop:
            def wait(self, _):
                return False

            def is_set(self):
                return False

        class GordonTask2(task2mn.Task2, g.GordonRoute):
            pass

        self.assertLess(GordonTask2.__mro__.index(task2mn.Task2),
                        GordonTask2.__mro__.index(g.GordonRoute))
        course = tp.Course(x1=90, x2=190, w2=60)
        fake = ts.FakeSTM(course, tp.WorldFrame(None).start, tp.RADIUS_CM)
        phases = []
        logging.disable(logging.CRITICAL)
        try:
            task = GordonTask2(fake, Vision(["left", "right"]), Stop(),
                               phases.append, ir_latch=True)
            code = task.run()
        finally:
            logging.disable(logging.NOTSET)
        self.assertEqual(code, 0)
        self.assertTrue(task.route_finished)
        for phase in ("BYPASS_1", "ROUND_2", "RETURN_AND_PARK"):
            self.assertIn(phase, phases)
        clear, name = ts.run_clearance(fake, course)
        self.assertGreaterEqual(clear, tp.CLEARANCE_FLOOR_CM, name)


if __name__ == "__main__":
    unittest.main()
