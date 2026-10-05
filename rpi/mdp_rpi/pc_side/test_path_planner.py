"""
test_path_planner.py  —  the tour search must always return a tour
───────────────────────────────────────────────────────────────────
    python3 test_path_planner.py          # from pc_side/

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

sys.path.insert(0, str(Path(__file__).resolve().parent))

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
    to 45, and 28 only when nothing else fits."""

    def test_open_floor_uses_30(self):
        self.assertEqual(options_for({"id": 1, "x": 10, "y": 10, "d": 0})[0], 30)

    def test_28_is_tried_last(self):
        # Facing the top edge: 35+ would hang the robot too far off the arena.
        self.assertEqual(options_for({"id": 1, "x": 10, "y": 16, "d": 0}), [30, 32, 28])

    def test_a_blocked_30_moves_further_out(self):
        # A neighbour diagonally behind blocks 28-42; only 45 clears it.
        self.assertEqual(
            options_for({"id": 1, "x": 10, "y": 10, "d": 0}, {"id": 2, "x": 11, "y": 14, "d": 0}),
            [45],
        )

    def test_no_room_at_any_distance_is_skipped_not_fudged(self):
        details = {}
        result = pp.plan_mission([{"id": 1, "x": 10, "y": 17, "d": 0}], details=details)
        self.assertEqual(result["segments"], [])
        self.assertEqual(details["skipped"], {1: "no photo spot"})


class LegOutputTests(unittest.TestCase):

    def test_straights_are_merged(self):
        poses = [pp.Pose(i, 0, 0) for i in range(5)]
        tokens, merged = pp._merge_straights(["F5", "F5", "FR90", "R5", "R5"], poses)
        self.assertEqual(tokens, ["F10", "FR90", "R10"])
        self.assertIs(merged[0], poses[1])   # pose after the LAST merged token

    def test_a_simple_plan_ends_each_photo_with_fu_at_the_chosen_distance(self):
        details = {}
        result = pp.plan_mission([{"id": 1, "x": 10, "y": 10, "d": 0}], details=details)
        self.assertEqual(result["segment_obstacles"][-1], "1")
        self.assertEqual(result["segments"][-1][-2:], ["FU30", "S"])
        self.assertEqual(details["standoff_cm"], {1: 30})


if __name__ == "__main__":
    unittest.main(verbosity=2)
