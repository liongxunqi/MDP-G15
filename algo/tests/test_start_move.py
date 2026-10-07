"""
algo/tests/test_start_move.py  —  how the plan leaves the start, and what the RPi will see
─────────────────────────────────────────────────────────────────────────────────────────
    python3 algo/tests/test_start_move.py

WHAT IT PROTECTS
  1. The robot is placed 10 mm from two lines. A first instruction that reverses
     (or any move that takes the body well past a line near the start) puts it
     outside the arena. The planner used to do exactly that in 12 of 35 layouts
     (R5 .. R25), because its edge-overhang allowance applied at the start corner.
  2. The RPi reads ?WPOSE after every instruction and STOPS the mission if the
     position is outside the arena (stm/PROTOCOL.md 12). Its frame is anchored at
     (RPI_ANCHOR_X_MM, RPI_ANCHOR_Y_MM), not at the planner's start, so a plan that
     is fine in the planner's frame can still halt the RPi. Before the guard, 27
     of 35 plans would have.
  3. PATH's "expected" poses line up with the segments and match a replayed robot.
  4. A dead-end obstacle does not eat the whole planning budget.

Layouts: the team's fixed ones plus a seeded random set (same generator as
test_mission_sim.py). Planned once and shared.
"""

import logging
import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))      # algo/
sys.path.insert(0, str(Path(__file__).resolve().parent))          # algo/tests/

import grid_search                                                # noqa: E402
import path_planner as pp                                         # noqa: E402
import test_mission_sim as sim                                    # noqa: E402
from stm_tokens import PROFILE_TIGHT, TURN_RADIUS_MM              # noqa: E402

N_RANDOM = 10
START_REGION_MM = 700.0         # = pp.START_GUARD_MM
ON_THE_FLOOR_TOL_MM = 25.0      # = pp.START_GUARD_TOL_MM

# Cramped start: obstacle 1 sits in the start lane and obstacle 6 just past it, so
# there is no way out of the corner that stays on the floor. (Was empty-plan
# territory for a strict rule; must fall back, loudly, instead.)
CRAMPED = [
    {"id": 1, "x": 1, "y": 5, "d": 6}, {"id": 2, "x": 16, "y": 13, "d": 2},
    {"id": 3, "x": 7, "y": 16, "d": 0}, {"id": 4, "x": 12, "y": 18, "d": 4},
    {"id": 5, "x": 13, "y": 1, "d": 4}, {"id": 6, "x": 4, "y": 6, "d": 0},
]
# Seven obstacles, several hugging walls: used to retry every dead end at all 8
# standoffs and spend the whole 20 s budget.
DEAD_ENDS = [
    {"id": 1, "x": 0, "y": 17, "d": 0}, {"id": 2, "x": 12, "y": 6, "d": 6},
    {"id": 3, "x": 7, "y": 14, "d": 6}, {"id": 4, "x": 17, "y": 7, "d": 4},
    {"id": 5, "x": 7, "y": 7, "d": 6}, {"id": 6, "x": 9, "y": 0, "d": 6},
    {"id": 7, "x": 17, "y": 3, "d": 2},
]

_PLANS = []


class _Capture(logging.Handler):
    def __init__(self):
        super().__init__(logging.WARNING)
        self.msgs = []

    def emit(self, record):
        self.msgs.append(record.getMessage())


def setUpModule():
    layouts = list(sim.FIXED_LAYOUTS.items())
    rng = random.Random(1)
    layouts += [(f"random-{i + 1}", sim.random_layout(rng, rng.randint(5, 8))) for i in range(N_RANDOM)]
    logging.disable(logging.CRITICAL)
    for name, lay in layouts:
        det = {}
        plan = pp.plan_mission(lay, arc_profile=PROFILE_TIGHT, details=det)
        _PLANS.append((name, lay, plan, det))
    logging.disable(logging.NOTSET)


def _overhang(rep):
    pts = sim.corners(rep.x, rep.y, rep.th)
    return max(0.0, max(max(-px, -py, px - 2000, py - 2000) for px, py in pts))


class StartMove(unittest.TestCase):
    def test_the_first_instruction_is_never_a_reverse(self):
        for name, lay, plan, det in _PLANS:
            if det["start_relaxed"]:
                continue
            with self.subTest(layout=name):
                self.assertTrue(plan["segments"], "empty plan")
                first = plan["segments"][0][0].upper()
                self.assertFalse(first.startswith("R"), f"starts with {first}")

    def test_the_body_stays_on_the_floor_near_the_start(self):
        for name, lay, plan, det in _PLANS:
            if det["start_relaxed"]:
                continue
            with self.subTest(layout=name):
                rep = sim.Replay(lay, TURN_RADIUS_MM[PROFILE_TIGHT])
                worst = 0.0
                for seg, target in zip(plan["segments"], plan["segment_obstacles"]):
                    for tok in seg:
                        rep.run(tok)
                        if math.hypot(rep.x - pp.START_X_MM, rep.y - pp.START_Y_MM) < START_REGION_MM:
                            worst = max(worst, _overhang(rep))
                    if target is not None:
                        break                                    # first leg only
                self.assertLessEqual(worst, ON_THE_FLOOR_TOL_MM + 1.0)

    def test_the_rpi_never_sees_a_position_outside_the_arena(self):
        if not pp.RPI_ARENA_GUARD:
            self.skipTest("RPI_ARENA_GUARD=0")
        for name, lay, plan, det in _PLANS:
            with self.subTest(layout=name):
                sx, sy = det["frame_shift_mm"]
                rep = sim.Replay(lay, TURN_RADIUS_MM[PROFILE_TIGHT])
                for seg in plan["segments"]:
                    for tok in seg:
                        rep.run(tok)
                        x, y = rep.x + sx, rep.y + sy
                        self.assertTrue(0 <= x < 2000 and 0 <= y < 2000,
                                        f"after {tok}: the RPi would see ({x:.0f}, {y:.0f}) mm")

    def test_cramped_start_falls_back_loudly_instead_of_planning_nothing(self):
        cap = _Capture()
        logging.getLogger().addHandler(cap)
        det = {}
        try:
            plan = pp.plan_mission(CRAMPED, arc_profile=PROFILE_TIGHT, details=det)
        finally:
            logging.getLogger().removeHandler(cap)
        self.assertTrue(plan["segments"], "planned nothing")
        self.assertTrue(det["start_relaxed"])
        self.assertTrue(any("could not leave the start cleanly" in m for m in cap.msgs), cap.msgs)


class StartBoxes(unittest.TestCase):
    """The blockers themselves, against the same footprint test the search uses."""

    def blocked(self, x, y, theta, boxes):
        return grid_search._point_blocked(x, y, theta, boxes, pp.ARENA_MM,
                                          pp.ROBOT_HALF_LENGTH_MM, pp.ROBOT_HALF_WIDTH_MM)

    def setUp(self):
        self.start = pp.Pose(pp.START_X_MM, pp.START_Y_MM, pp.START_THETA)
        self.boxes = pp._start_boxes(self.start)

    def test_driving_straight_out_of_the_corner_is_allowed(self):
        for d in (0, 50, 200):
            self.assertFalse(self.blocked(self.start.x, self.start.y + d, self.start.theta, self.boxes), d)

    def test_any_reverse_from_the_start_is_blocked(self):
        for d in (5, 20, 50, 250):
            self.assertTrue(self.blocked(self.start.x, self.start.y - d, self.start.theta, self.boxes), d)

    def test_a_forward_left_arc_out_of_the_corner_is_blocked_and_a_right_one_is_not(self):
        r = TURN_RADIUS_MM[PROFILE_TIGHT]

        def swept(kappa):
            for i in range(1, 13):
                dx, dy, th = grid_search._arc_delta(self.start.theta, +1, kappa, math.pi / 2 * i / 12, r)
                if self.blocked(self.start.x + dx, self.start.y + dy, th, self.boxes):
                    return True
            return False
        self.assertTrue(swept(+1), "FL90 sweeps the front 302 mm past the left line")
        self.assertFalse(swept(-1), "FR90 only brushes the line by ~7 mm")

    def test_rpi_limit_bounds_the_reference_point_exactly_at_every_heading(self):
        s = pp.Pose(pp.START_X_MM, pp.START_Y_MM, pp.START_THETA)
        boxes = grid_search.Boxes()
        boxes.ref_bounds = pp._rpi_ref_bounds(s)
        lo_x, hi_x, lo_y, hi_y = boxes.ref_bounds
        for k in range(0, 360, 15):                       # incl. the diagonals where the body is widest
            th = math.radians(k)
            for x, y, want in ((lo_x + 1, 1000, False), (lo_x - 1, 1000, True),
                               (hi_x - 1, 1000, False), (hi_x + 1, 1000, True),
                               (1000, lo_y + 1, False), (1000, lo_y - 1, True),
                               (1000, hi_y - 1, False), (1000, hi_y + 1, True)):
                self.assertEqual(self.blocked(x, y, th, boxes), want, f"heading {k} deg at ({x:.0f},{y:.0f})")

    def test_the_bounds_are_the_arena_as_the_rpi_sees_it(self):
        s = pp.Pose(pp.START_X_MM, pp.START_Y_MM, pp.START_THETA)
        lo_x, hi_x, lo_y, hi_y = pp._rpi_ref_bounds(s)
        sx, sy = pp._frame_shift(s)
        self.assertAlmostEqual(lo_x + sx, pp.RPI_ARENA_MARGIN_MM)
        self.assertAlmostEqual(hi_x + sx, 2000 - pp.RPI_ARENA_MARGIN_MM)
        self.assertAlmostEqual(lo_y + sy, pp.RPI_ARENA_MARGIN_MM)
        self.assertAlmostEqual(hi_y + sy, 2000 - pp.RPI_ARENA_MARGIN_MM)


class ExpectedPoses(unittest.TestCase):
    def test_shapes_line_up_with_the_segments(self):
        for name, lay, plan, det in _PLANS:
            with self.subTest(layout=name):
                exp = plan["expected"]
                self.assertEqual(len(exp), len(plan["segments"]))
                for e, seg in zip(exp, plan["segments"]):
                    self.assertEqual(len(e), len(seg))
                    for x, y, h in e:
                        self.assertTrue(0.0 <= h < 360.0)

    def test_they_match_a_perfect_robot(self):
        """Snap to the planned pose after each photo (FU stops where the SONAR says,
        so the replay and the plan legitimately part company there - the team's
        test_mission_sim does the same). Within a segment the gap is the 25 mm
        approach-line tolerance: the search may end a leg up to that far off the
        line, while "expected" assumes it ends exactly on it. Measured: median
        0.0 mm, max 24 mm, heading 0.000 deg."""
        tol = pp.APPROACH_LATERAL_TOL_MM + 1.0
        for name, lay, plan, det in _PLANS:
            with self.subTest(layout=name):
                sx, sy = det["frame_shift_mm"]
                rep = sim.Replay(lay, TURN_RADIUS_MM[PROFILE_TIGHT])
                for si, (seg, target) in enumerate(zip(plan["segments"], plan["segment_obstacles"])):
                    for ii, tok in enumerate(seg):
                        rep.run(tok)
                        if tok.upper().startswith("FU") or tok.upper() == "S":
                            continue
                        ex, ey, eh = plan["expected"][si][ii]
                        self.assertLess(math.hypot(rep.x + sx - ex, rep.y + sy - ey), tol, (si, ii, tok))
                        got = (90.0 - math.degrees(rep.th)) % 360.0
                        self.assertLess(abs((got - eh + 180.0) % 360.0 - 180.0), 1.0, (si, ii, tok))
                    if target is not None:
                        ex, ey, eh = plan["expected"][si][-1]
                        rep.x, rep.y, rep.th = ex - sx, ey - sy, math.radians(90.0 - eh)

    def test_start_mm_is_the_planners_real_start(self):
        for name, lay, plan, det in _PLANS[:3]:
            self.assertEqual(plan["start_mm"], {"x": round(pp.START_X_MM, 1), "y": round(pp.START_Y_MM, 1),
                                                "heading": "N"})


class Escalation(unittest.TestCase):
    def test_dead_ends_do_not_trigger_a_matrix_of_searches_at_every_standoff(self):
        calls = []
        real = pp._search_leg
        pp._search_leg = lambda *a, **k: (calls.append(1), real(*a, **k))[1]
        logging.disable(logging.CRITICAL)
        try:
            pp.plan_mission(DEAD_ENDS, arc_profile=PROFILE_TIGHT)
        finally:
            pp._search_leg = real
            logging.disable(logging.NOTSET)
        # 78+ before the bound (and the 20 s budget gone); 32 after.
        self.assertLess(len(calls), 50, f"{len(calls)} leg searches")


if __name__ == "__main__":
    unittest.main(verbosity=1)
