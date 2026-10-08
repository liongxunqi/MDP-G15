"""
test_path_planner.py  —  the tour search must always return a tour
───────────────────────────────────────────────────────────────────
    python3 algo/tests/test_path_planner.py      # from the repo root

Fast. Most of these drive _visit_order() and _reachable() against a
hand-made edge cache; the rest plan one-obstacle scenes. For whole layouts,
replayed and collision-checked, run test_mission_sim.py.

WHAT IT IS PROTECTING
─────────────────────
_visit_order() used to return None when every permutation cost infinity, and
then iterate that None in its own logging line:

    best_order, best_cost = None, math.inf
    for perm in itertools.permutations(visitable):
        if cost < best_cost:          # inf < inf is False, so never
            ...
    logging.info(f"... {[o['id'] for o in best_order]} ...")   # TypeError

Every permutation costs infinity as soon as ONE obstacle has no finite
approach, which happens whenever a viewing pose lands outside the arena — a
block within about 65 cm of a wall. A.5 declares all four faces, so it meets
that case whenever the obstacle is anywhere near an edge.

The crash is worse than it looks from the PC. task1_pc.py takes the exception
and never sends PATH, and on the Pi nothing blocks on PATH — so there is no
deadlock and no error, just a robot standing still and a tablet showing
nothing wrong.
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # algo/

import path_planner as pp


def obstacles(*ids):
    return [{"id": i, "x": 5, "y": 5, "d": 0} for i in ids]


def cache_from(edges):
    """{(from_id, to_id): cost} -> the (tokens, poses, cost) shape the real one has."""
    return {key: ([], [], cost) for key, cost in edges.items()}


class VisitOrderTests(unittest.TestCase):

    def test_never_returns_none_when_nothing_is_reachable(self):
        # The regression. An empty cache means every edge is infinite.
        order = pp._visit_order(obstacles(1, 2, 3), {})
        self.assertIsNotNone(order, "_visit_order returned None — the old crash")
        self.assertEqual(len(order), 3)

    def test_one_unreachable_obstacle_does_not_lose_the_others(self):
        # 3 is unreachable; 1 and 2 are fine. Every permutation still costs
        # infinity because all of them contain 3 — which is exactly how one
        # bad obstacle used to take the whole plan down.
        edges = cache_from({
            ("START", 1): 100.0, ("START", 2): 200.0,
            (1, 2): 50.0, (2, 1): 50.0,
        })
        order = pp._visit_order(obstacles(1, 2, 3), edges)
        self.assertIsNotNone(order)
        self.assertEqual({o["id"] for o in order}, {1, 2, 3})

    def test_a_reachable_set_still_gets_the_cheap_tour(self):
        # The optimiser must still optimise: going 1 -> 2 is cheaper than 2 -> 1.
        edges = cache_from({
            ("START", 1): 10.0, ("START", 2): 900.0,
            (1, 2): 10.0, (2, 1): 900.0,
        })
        order = pp._visit_order(obstacles(1, 2), edges)
        self.assertEqual([o["id"] for o in order], [1, 2])

    def test_single_obstacle_is_returned_as_is(self):
        self.assertEqual(len(pp._visit_order(obstacles(1), {})), 1)

    def test_empty_is_empty(self):
        self.assertEqual(pp._visit_order([], {}), [])


class ReachabilityTests(unittest.TestCase):

    def test_nothing_reachable_when_the_cache_is_empty(self):
        ok, dropped = pp._reachable(obstacles(1, 2), {})
        self.assertEqual(ok, [])
        self.assertEqual(len(dropped), 2)

    def test_reachable_from_start_is_kept(self):
        ok, dropped = pp._reachable(obstacles(1), cache_from({("START", 1): 5.0}))
        self.assertEqual([o["id"] for o in ok], [1])
        self.assertEqual(dropped, [])

    def test_reachable_only_from_another_obstacle_is_kept(self):
        # Unreachable from the start but fine once the robot is at 1. Dropping
        # it would throw away a face it could actually photograph.
        ok, _ = pp._reachable(obstacles(1, 2), cache_from({
            ("START", 1): 5.0,
            (1, 2): 5.0,
        }))
        self.assertEqual({o["id"] for o in ok}, {1, 2})

    def test_the_wall_facing_one_is_the_only_one_dropped(self):
        # The A.5 shape: four faces, one of them pointing into a wall.
        edges = cache_from({
            ("START", 1): 10.0, ("START", 2): 10.0, ("START", 4): 10.0,
            (1, 2): 5.0, (2, 4): 5.0, (4, 1): 5.0,
        })
        ok, dropped = pp._reachable(obstacles(1, 2, 3, 4), edges)
        self.assertEqual({o["id"] for o in ok}, {1, 2, 4})
        self.assertEqual([o["id"] for o in dropped], [3])


class PlanMissionTests(unittest.TestCase):

    def test_an_all_skip_scene_returns_a_dict_not_none(self):
        # d=8 is SKIP, so nothing has a viewing pose. The caller needs the
        # four keys it always reads, not None.
        result = pp.plan_mission([{"id": 1, "x": 5, "y": 5, "d": 8}])
        self.assertIsInstance(result, dict)
        for key in ("segments", "obstacle_ids", "segment_obstacles", "dirs"):
            self.assertIn(key, result)
        self.assertEqual(result["segments"], [])


def options_for(target, *others):
    every = [target, *others]
    boxes = [pp._obstacle_aabb_mm(o) for o in every]
    return [op.standoff_cm for op in pp._photo_options(target, boxes, every)]


class PhotoDistanceTests(unittest.TestCase):
    """The ultrasonic reading at each photo: 30 cm first, then further out up
    to 45, with close 28/20 cm fallbacks when nothing else fits."""

    def test_open_floor_uses_30(self):
        self.assertEqual(options_for({"id": 1, "x": 10, "y": 10, "d": 0})[0], 30)

    def test_path_exposes_only_safe_camera_retry_distances(self):
        result = pp.plan_mission([{"id": 1, "x": 10, "y": 10, "d": 0}])
        self.assertEqual(result["selected_standoffs"], {"1": 30})
        self.assertIn(33, result["photo_standoffs"]["1"])
        self.assertIn(45, result["photo_standoffs"]["1"])

    def test_close_distances_are_tried_only_after_normal_options(self):
        # Facing the top edge: 30 still wins; close poses remain fallbacks.
        self.assertEqual(
            options_for({"id": 1, "x": 10, "y": 15, "d": 0}),
            [30, 28, 20],
        )

    def test_20_cm_fallback_keeps_wall_facing_rear_axle_inside_guard(self):
        # The west face of cell x=5 is at 500 mm. At 30 cm the rear-axle pose
        # is -30 mm, but FU20 leaves it at +70 mm and inside the 50 mm RPi guard.
        obstacle = {"id": 1, "x": 5, "y": 6, "d": 6}
        boxes = pp.grid_search.Boxes([pp._obstacle_aabb_mm(obstacle)])
        start = pp.Pose(pp.START_X_MM, pp.START_Y_MM, pp.START_THETA)
        boxes.ref_bounds = pp._rpi_ref_bounds(start)
        options = pp._photo_options(obstacle, boxes, [obstacle])
        self.assertEqual([option.standoff_cm for option in options], [20])
        self.assertAlmostEqual(options[0].pose.x, 70.0)

    def test_a_blocked_30_moves_further_out(self):
        # A neighbour diagonally behind blocks the preferred close standoffs.
        self.assertEqual(
            options_for({"id": 1, "x": 10, "y": 10, "d": 0}, {"id": 2, "x": 10, "y": 15, "d": 0}),
            [42, 45],
        )

    def test_no_room_at_any_distance_is_skipped_not_fudged(self):
        details = {}
        result = pp.plan_mission([{"id": 1, "x": 10, "y": 17, "d": 0}], details=details)
        self.assertEqual(result["segments"], [])
        self.assertEqual(details["skipped"], {1: "no photo spot"})

    def test_viewing_pose_uses_rear_axle_to_front_sensor_offset(self):
        obstacle = {"id": 1, "x": 10, "y": 10, "d": 0}
        pose = pp._viewing_pose(obstacle, 300.0)
        expected_y = 1050.0 + 50.0 + 300.0 + pp.REAR_AXLE_TO_SENSOR_MM
        self.assertAlmostEqual(pose.x, 1050.0)
        self.assertAlmostEqual(pose.y, expected_y)
        self.assertAlmostEqual(pose.theta, -math.pi / 2)


class LegOutputTests(unittest.TestCase):

    def test_straights_are_merged(self):
        poses = [pp.Pose(i, 0, 0) for i in range(5)]
        tokens, merged = pp._merge_runs(["F5", "F5", "FR90", "R5", "R5"], poses)
        self.assertEqual(tokens, ["F10", "FR90", "R10"])
        self.assertIs(merged[0], poses[1])   # pose after the LAST merged token

    def test_same_kind_arcs_are_merged_and_mixed_ones_are_not(self):
        poses = [pp.Pose(i, 0, 0) for i in range(6)]
        tokens, merged = pp._merge_runs(["FR45", "FR45", "RL45", "FL45", "FL45", "FU30"], poses)
        self.assertEqual(tokens, ["FR90", "RL45", "FL90", "FU30"])
        self.assertIs(merged[2], poses[4])

    def test_the_search_only_emits_45_degree_arcs(self):
        details = {}
        plan = pp.plan_mission([{"id": 1, "x": 15, "y": 3, "d": 6},
                                {"id": 2, "x": 5, "y": 15, "d": 4}], details=details)
        arcs = [t for line in plan["segments"] for t in line if t[:2] in ("FR", "FL", "RR", "RL")]
        self.assertTrue(arcs)
        self.assertTrue(all(int(t[2:]) % 45 == 0 for t in arcs), arcs)

    def test_a_simple_plan_ends_each_photo_with_fu_at_the_chosen_distance(self):
        details = {}
        result = pp.plan_mission([{"id": 1, "x": 10, "y": 10, "d": 0}], details=details)
        self.assertEqual(result["segment_obstacles"][-1], "1")
        self.assertEqual(result["segments"][-1][-2:], ["FU30", "S"])
        self.assertEqual(details["standoff_cm"], {1: 30})
        self.assertEqual(result["selected_standoffs"], {"1": 30})


class SegmentRecoveryTests(unittest.TestCase):
    OBSTACLES = [
        {"id": 1, "x": 2.0, "y": 16.0, "d": 2},
        {"id": 2, "x": 2.0, "y": 6.0, "d": 0},
        {"id": 4, "x": 15.0, "y": 3.0, "d": 6},
        {"id": 5, "x": 16.0, "y": 6.0, "d": 0},
        {"id": 6, "x": 12.0, "y": 13.0, "d": 6},
        {"id": 7, "x": 17.0, "y": 15.0, "d": 4},
    ]

    def test_logged_fu_overshoot_recovers_current_endpoint_and_preserves_tail(self):
        later_segment = ["F15", "FL90", "FL90", "R15", "RL90", "FU30", "S"]
        later_expected = [[680, 1650, 270], [385, 1356, 180], [680, 1062, 90],
                          [530, 1062, 90], [236, 1356, 180], [250, 1230, 180],
                          [250, 1230, 180]]
        plan = {
            "segments": [["FL90", "F30", "RR90", "F28", "FU30", "S"], later_segment],
            "segment_obstacles": ["1", "2"],
            "expected": [
                [[964, 1644, 0], [964, 1944, 0], [1258, 1650, 270],
                 [978, 1650, 270], [830, 1650, 270], [830, 1650, 270]],
                later_expected,
            ],
            "frame_shift_mm": {"x": 0, "y": 0},
            "start_mm": {"x": 104, "y": 125, "heading": "N"},
        }
        progress = {
            "segment_index": 0,
            "instruction_index": 4,
            "x_grid": 6.60969892,
            "y_grid": 16.5837032,
            "heading_deg": 270.2,
            "remaining_photo_ids": ["1", "2"],
        }

        replacement = pp.plan_segment_recovery(plan, self.OBSTACLES, progress)

        self.assertIsNotNone(replacement)
        self.assertEqual(replacement["segments"][0], ["R17", "S"])
        self.assertEqual(replacement["segment_obstacles"], ["1", "2"])
        self.assertEqual(replacement["segments"][1], later_segment)
        self.assertEqual(replacement["expected"][1], later_expected)
        final = replacement["expected"][0][-1]
        self.assertLess(math.hypot(final[0] - 830, final[1] - 1650), 10)

    def test_pending_photo_mismatch_refuses_replacement(self):
        plan = {
            "segments": [["F10", "S"]],
            "segment_obstacles": ["7"],
            "expected": [[[500, 500, 0], [500, 500, 0]]],
        }
        progress = {"segment_index": 0, "instruction_index": 0, "x_grid": 4,
                    "y_grid": 5, "heading_deg": 0, "remaining_photo_ids": ["99"]}
        self.assertIsNone(pp.plan_segment_recovery(plan, self.OBSTACLES, progress))

    def test_boundary_recovery_shortens_an_obstructed_approach_line(self):
        # Physical run 20261008_112200. The nominal next FL90 would carry the
        # measured x error to 2009 mm. The old recovery search found a point on
        # its 600 mm approach line whose final straight crossed obstacle 2, then
        # gave up without trying a closer, clear point on the same line.
        obstacles = [
            {"id": 1, "x": 5.0, "y": 6.0, "d": 6},
            {"id": 2, "x": 16.0, "y": 6.0, "d": 0},
            {"id": 3, "x": 2.0, "y": 16.0, "d": 2},
            {"id": 4, "x": 17.0, "y": 15.0, "d": 4},
            {"id": 5, "x": 12.0, "y": 13.0, "d": 6},
        ]
        plan = {
            "segments": [["F10", "FL90", "R50", "FL90", "R20", "FU30", "S"]],
            "segment_obstacles": ["4"],
            "expected": [[
                [1650, 1130, 180], [1944, 836, 90], [1444, 836, 90],
                [1738, 1130, 0], [1738, 930, 0], [1750, 970, 0],
                [1750, 970, 0],
            ]],
            "frame_shift_mm": {"x": 0, "y": 0},
            "start_mm": {"x": 104, "y": 125, "heading": "N"},
        }
        progress = {
            "segment_index": 0,
            "instruction_index": 0,
            "x_grid": 17.16426389,
            "y_grid": 11.02233924,
            "heading_deg": 179.8,
            "remaining_photo_ids": ["4"],
        }

        replacement = pp.plan_segment_recovery(plan, obstacles, progress)

        self.assertIsNotNone(replacement)
        self.assertEqual(replacement["segment_obstacles"][-1], "4")
        self.assertEqual(replacement["segments"][-1][-1], "S")
        final = replacement["expected"][-1][-1]
        self.assertLess(math.hypot(final[0] - 1750, final[1] - 970), 30)


class AndroidPoseTests(unittest.TestCase):
    """dirs carry the bottom-left cell of the 2 x 2 block Android draws."""

    def entry(self, x_mm, y_mm, theta=math.pi / 2):
        return pp._pose_to_dir_entry(pp.Pose(x_mm, y_mm, theta))

    def test_start_pose_draws_centred_on_the_start_zone(self):
        # Robot centre (20, 20) cm -> block over cells 1-2 = 10-30 cm.
        self.assertEqual(self.entry(200, 200), {"x": 1, "y": 1, "dir": "N"})

    def test_block_is_centred_on_the_robot(self):
        # Centre (104, 156) cm -> the 2x2 block nearest to centred on it starts
        # at (9, 15): it covers 90-110 and 150-170 cm, centred at (100, 160).
        self.assertEqual(self.entry(1040, 1560, 0.0), {"x": 9, "y": 15, "dir": "E"})

    def test_halves_round_the_same_way_on_both_axes(self):
        e = self.entry(1050, 1550)
        self.assertEqual((e["x"], e["y"]), (10, 15))

    def test_overhanging_robot_is_kept_on_the_grid(self):
        # Photo spots may hang past the edge; Android only accepts 0..18.
        self.assertEqual(self.entry(-50, 2150, math.pi)["x"], 0)
        self.assertEqual(self.entry(-50, 2150, math.pi)["y"], 18)


if __name__ == "__main__":
    unittest.main(verbosity=2)
