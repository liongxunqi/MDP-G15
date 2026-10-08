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
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # algo/

import path_planner as pp


def obstacles(*ids):
    return [{"id": i, "x": 5, "y": 5, "d": 0} for i in ids]


def cache_from(edges):
    """{(from_id, to_id): cost} -> the (tokens, poses, cost) shape the real one has."""
    return {key: ([], [], cost) for key, cost in edges.items()}


class VisitOrderTests(unittest.TestCase):

    def test_open_target_keeps_normal_tour_and_head_on_photo(self):
        with patch.object(pp, "_plan_compact_sequence",
                          side_effect=AssertionError("open layout used compact routing")):
            result = pp.plan_mission([{"id": 1, "x": 10, "y": 10, "d": 0}])
        self.assertEqual(result["obstacle_ids"], [1])
        self.assertEqual(result["selected_view_angles"]["1"], 0)

    def test_edge_target_does_not_force_open_target_into_compact_tour(self):
        with patch.object(pp, "_choose_tour", wraps=pp._choose_tour) as normal:
            result = pp.plan_mission([
                {"id": 1, "x": 10, "y": 10, "d": 0},
                {"id": 2, "x": 16, "y": 16, "d": 2},
            ])
        normal.assert_called_once()
        self.assertIn(1, result["obstacle_ids"])
        self.assertEqual(result["selected_view_angles"]["1"], 0)

    def test_compact_search_tries_alternate_head_on_range_before_diagonal(self):
        obstacle = {"id": 1}
        preferred, alternate, diagonal = object(), object(), object()
        start = pp.Pose(500, 500, 0)
        boxes = pp._collision_boxes([])
        leg = (["F5"], [start, pp.Pose(550, 500, 0)], 50)
        with patch.object(pp, "_compact_diagonal_candidates", return_value=[diagonal]), \
                patch.object(pp, "_search_any_approach", side_effect=[
                    None, (obstacle, alternate, leg),
                ]) as search, \
                patch.object(pp, "_terminal_after_leg", return_value=leg[1][-1]):
            sequence, remaining = pp._plan_compact_sequence(
                start, [obstacle], {1: [preferred, alternate]}, {1: [diagonal]},
                293.5, boxes, boxes, pp.time.monotonic() + 30,
            )
        self.assertEqual(remaining, [])
        self.assertIs(sequence[0][1], alternate)
        self.assertEqual(search.call_args_list[0].args[1], [(obstacle, preferred)])
        self.assertEqual(search.call_args_list[1].args[1], [(obstacle, alternate)])

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
        self.assertEqual(result["ultrasonic_adjustments"], {"1": True})
        self.assertIn(33, result["photo_standoffs"]["1"])
        self.assertIn(45, result["photo_standoffs"]["1"])

    def test_top_facing_pose_uses_the_closest_safe_head_on_standoff(self):
        self.assertEqual(options_for({"id": 1, "x": 10, "y": 15, "d": 0})[0], 28)

    def test_measured_camera_centre_opens_the_west_photo_pose(self):
        obstacle = {"id": 1, "x": 5, "y": 6, "d": 6}
        boxes = pp._collision_boxes([pp._obstacle_aabb_mm(obstacle)])
        start = pp.Pose(pp.START_X_MM, pp.START_Y_MM, pp.START_THETA)
        boxes.ref_bounds = pp._rpi_ref_bounds(start)
        options = pp._photo_options(obstacle, boxes, [obstacle])
        self.assertTrue(options)
        self.assertEqual(options[0].standoff_cm, 30)

    def test_facing_images_without_head_on_space_use_known_face_diagonals(self):
        result = pp.plan_mission([
            {"id": 1, "x": 6, "y": 10, "d": 2},
            {"id": 2, "x": 10, "y": 10, "d": 6},
        ])
        self.assertEqual(set(result["obstacle_ids"]), {1, 2})
        self.assertEqual(
            {abs(angle) for angle in result["selected_view_angles"].values()},
            {45.0},
        )

    def test_a_blocker_on_the_view_axis_rejects_every_head_on_standoff(self):
        # Moving farther back does not help when another obstacle remains between
        # the camera and the intended image; accepting it associates the other
        # obstacle's target with this ID.
        self.assertEqual(
            options_for({"id": 1, "x": 10, "y": 10, "d": 0},
                        {"id": 2, "x": 10, "y": 12, "d": 0}),
            [],
        )

    def test_failed_head_on_connection_falls_back_to_safe_diagonal(self):
        # The second box blocks every head-on camera pose but is itself marked
        # SKIP. A 45-degree view of obstacle 1 remains physically reachable.
        details = {}
        result = pp.plan_mission([
            {"id": 1, "x": 10, "y": 10, "d": 0},
            {"id": 2, "x": 9, "y": 15, "d": 8},
        ], details=details)
        self.assertEqual(result["obstacle_ids"], [1])
        self.assertEqual(abs(result["selected_view_angles"]["1"]), 45)
        self.assertNotIn(1, details["skipped"])
        layout = [{"id": 1, "x": 10, "y": 10, "d": 0},
                  {"id": 2, "x": 9, "y": 15, "d": 8}]
        boxes = pp._collision_boxes([pp._obstacle_aabb_mm(o) for o in layout])
        replay = pp._replay_motion(
            pp.Pose(pp.START_X_MM, pp.START_Y_MM, pp.START_THETA),
            [token for line in result["segments"] for token in line if token != "S"],
            pp.TURN_RADIUS_MM[pp.PROFILE_TIGHT], boxes,
        )
        self.assertIsNotNone(replay)

    def test_facing_images_with_thirty_cm_gap_keep_safe_retry_distances(self):
        # Boxes occupy x=600..700 and x=1000..1100, leaving exactly 300 mm
        # between their facing image planes. A head-on rear-axle pose does not
        # fit, but an external 45-degree view of each face does.
        result = pp.plan_mission([
            {"id": 1, "x": 6, "y": 10, "d": 2},
            {"id": 2, "x": 10, "y": 10, "d": 6},
        ])
        self.assertEqual(set(result["obstacle_ids"]), {1, 2})
        self.assertTrue(all(
            distance >= 25
            for distances in result["photo_standoffs"].values()
            for distance in distances
        ))
        self.assertFalse(any(
            token.startswith("FU")
            for segment in result["segments"] for token in segment
        ))

    def test_thirty_cm_edge_target_gets_safe_diagonal_candidates(self):
        target = {"id": 6, "x": 16, "y": 16, "d": 2}
        boxes = pp._collision_boxes([pp._obstacle_aabb_mm(target)])
        options = pp._photo_options(target, boxes, [target], allow_oblique=True)
        self.assertEqual({option.view_angle_deg for option in options}, {-45.0, 45.0})
        self.assertTrue(any(option.standoff_cm == 25 and option.line.length_mm >= 50
                            for option in options))
        self.assertTrue(all(pp._pose_clear(option.pose.x, option.pose.y,
                                           option.pose.theta, boxes)
                            for option in options))

    def test_nine_target_compact_layout_completes_with_bounded_diagonals(self):
        xs, ys = (3, 7, 12), (4, 8, 13)
        directions = ((2, 0, 6), (2, 0, 6), (2, 4, 6))
        layout = [
            {"id": row * 3 + column + 1, "x": x, "y": y,
             "d": directions[row][column]}
            for row, y in enumerate(ys)
            for column, x in enumerate(xs)
        ]
        result = pp.plan_mission(layout)
        self.assertEqual(set(result["obstacle_ids"]), set(range(1, 10)))
        self.assertEqual(len(result["selected_standoffs"]), 9)
        self.assertTrue(all(
            distance >= 25
            for distances in result["photo_standoffs"].values()
            for distance in distances
        ))
        self.assertFalse(any(
            token.startswith("FU")
            for segment in result["segments"] for token in segment
        ))

    def test_unreachable_edge_target_is_skipped_instead_of_using_fine_turns(self):
        target = {"id": 6, "x": 16, "y": 16, "d": 2}
        result = pp.plan_mission([target])
        self.assertEqual(result["segments"], [])
        self.assertEqual(result["selected_view_angles"], {})

    def test_viewing_pose_uses_rear_axle_to_camera_offset(self):
        obstacle = {"id": 1, "x": 10, "y": 10, "d": 0}
        pose = pp._viewing_pose(obstacle, 300.0)
        expected_y = 1050.0 + 50.0 + 300.0 + pp.REAR_AXLE_TO_CAMERA_MM
        self.assertAlmostEqual(pose.x, 1050.0)
        self.assertAlmostEqual(pose.y, expected_y)
        self.assertAlmostEqual(pose.theta, -math.pi / 2)


class LegOutputTests(unittest.TestCase):

    def test_search_does_not_invent_small_turns_after_coarse_failure(self):
        goal = pp.grid_search.ApproachLine(1000.0, 1000.0, 0.0, 0.0, 1.0)
        calls = []

        def fake_search(*args, **kwargs):
            steps = kwargs.get("turn_steps_deg", (pp.grid_search.TURN_STEP_DEG,))
            calls.append(tuple(steps))
            raise pp.grid_search.NoPathFound("coarse route blocked")

        with patch.object(pp.grid_search, "search_leg", side_effect=fake_search):
            with self.assertRaises(pp.grid_search.NoPathFound):
                pp._search_motion(
                    900.0, 1000.0, 0.0, goal,
                    pp.TURN_RADIUS_MM[pp.PROFILE_TIGHT], pp.grid_search.Boxes(),
                )

        self.assertEqual(calls, [(45,)])

    def test_fine_lattice_can_stop_at_a_15_degree_checkpoint(self):
        radius = pp.TURN_RADIUS_MM[pp.PROFILE_TIGHT]
        dx, dy, theta = pp.grid_search._arc_delta(
            0.0, +1, -1, math.radians(15.0), radius,
        )
        goal = pp.grid_search.ApproachLine(
            1000.0 + dx, 1000.0 + dy, theta, 1.0, 2.0,
        )
        tokens, poses, _ = pp.grid_search.search_leg(
            1000.0, 1000.0, 0.0, goal, radius,
            pp.grid_search.Boxes(), pp.ARENA_MM,
            pp.ROBOT_HALF_LENGTH_MM, pp.ROBOT_HALF_WIDTH_MM,
            turn_steps_deg=(15, 30, 45),
        )
        self.assertEqual(tokens, ["FR15"])
        self.assertAlmostEqual(math.degrees(poses[-1][2]), -15.0)

    def test_non_45_degree_endpoint_is_rejected_without_fine_fallback(self):
        goal = pp.grid_search.ApproachLine(
            1000.0, 1000.0, math.radians(75.0), 0.0, 1.0,
        )
        calls = []

        def fake_search(*args, **kwargs):
            calls.append(tuple(kwargs.get(
                "turn_steps_deg", (pp.grid_search.TURN_STEP_DEG,)
            )))
            return ["FR15"], [(1000.0, 1000.0, math.radians(75.0))], 1.0

        with patch.object(pp.grid_search, "search_leg", side_effect=fake_search):
            with self.assertRaises(pp.grid_search.NoPathFound):
                pp._search_motion(
                    900.0, 1000.0, math.radians(90.0), goal,
                    pp.TURN_RADIUS_MM[pp.PROFILE_TIGHT], pp.grid_search.Boxes(),
                )

        self.assertEqual(calls, [])

    def test_straights_are_merged(self):
        poses = [pp.Pose(i, 0, 0) for i in range(5)]
        tokens, merged = pp._merge_runs(["F5", "F5", "FR90", "R5", "R5"], poses)
        self.assertEqual(tokens, ["F10", "FR90", "R10"])
        self.assertIs(merged[0], poses[1])   # pose after the LAST merged token

    def test_adjacent_45_degree_arcs_merge_to_calibrated_90(self):
        poses = [pp.Pose(i, 0, 0) for i in range(6)]
        tokens, merged = pp._merge_runs(["FR45", "FR45", "RL45", "FL45", "FL45", "F30"], poses)
        self.assertEqual(tokens, ["FR90", "RL45", "FL90", "F30"])
        self.assertEqual(merged, [poses[1], poses[2], poses[4], poses[5]])

    def test_the_search_only_emits_calibrated_45_or_90_degree_arcs(self):
        details = {}
        plan = pp.plan_mission([{"id": 1, "x": 15, "y": 3, "d": 6},
                                {"id": 2, "x": 5, "y": 15, "d": 4}], details=details)
        arcs = [t for line in plan["segments"] for t in line if t[:2] in ("FR", "FL", "RR", "RL")]
        self.assertTrue(arcs)
        self.assertTrue(all(int(t[2:]) in (45, 90) for t in arcs), arcs)

    def test_oblique_photo_options_use_calibrated_diagonal_headings(self):
        obstacle = {"id": 1, "x": 10, "y": 10, "d": 0}
        boxes = pp.grid_search.Boxes([pp._obstacle_aabb_mm(obstacle)])
        options = pp._photo_options(
            obstacle, boxes, [obstacle], allow_oblique=True,
        )
        angles = {round(option.view_angle_deg) for option in options}
        self.assertEqual(angles, {-45, 45})
        for option in options:
            heading = pp.grid_search._heading_index(option.pose.theta)
            self.assertAlmostEqual(
                pp.grid_search._HEADINGS[heading], option.pose.theta, places=6
            )

    def test_photo_option_rejects_another_obstacle_blocking_the_image(self):
        target = {"id": 1, "x": 10, "y": 10, "d": 0}
        blocker = {"id": 2, "x": 10, "y": 13, "d": 4}
        every = [target, blocker]
        boxes = pp.grid_search.Boxes([pp._obstacle_aabb_mm(o) for o in every])
        options = pp._photo_options(target, boxes, every, allow_oblique=False)
        self.assertEqual(options, [])

    def test_swept_guard_checks_45_degree_arc_not_only_endpoint(self):
        plan = {
            "frame_shift_mm": {"x": 0.0, "y": 0.0},
            "start_mm": {"x": pp.START_X_MM, "y": pp.START_Y_MM},
        }
        report = {"x_grid": 20.5, "y_grid": 10.0, "heading_deg": 0.0}
        risk = pp.assess_primitive_safety(plan, [], report, "FR45")
        self.assertIsNotNone(risk)
        self.assertEqual(risk["next_token"], "FR45")

    def test_a_simple_plan_uses_only_odometry_plus_us_metadata_for_photo(self):
        details = {}
        result = pp.plan_mission([{"id": 1, "x": 10, "y": 10, "d": 0}], details=details)
        self.assertEqual(result["segment_obstacles"][-1], "1")
        self.assertEqual(result["segments"][-1][-1], "S")
        self.assertFalse(any(
            token.startswith("FU")
            for segment in result["segments"] for token in segment
        ))
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

    def test_logged_longitudinal_error_recovers_current_endpoint_and_preserves_tail(self):
        later_segment = ["F15", "FL90", "FL90", "R15", "RL90", "F3", "S"]
        later_expected = [[680, 1650, 270], [385, 1356, 180], [680, 1062, 90],
                          [530, 1062, 90], [236, 1356, 180], [250, 1230, 180],
                          [250, 1230, 180]]
        plan = {
            "segments": [["FL90", "F30", "RR90", "F28", "F15", "S"], later_segment],
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
            "segments": [["F10", "FL90", "R50", "FL90", "R20", "F4", "S"]],
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
            "blocked_token": "FL90",
        }

        replacement = pp.plan_segment_recovery(plan, obstacles, progress)

        self.assertIsNotNone(replacement)
        self.assertEqual(replacement["segment_obstacles"][-1], "4")
        self.assertEqual(replacement["segments"][-1][-1], "S")
        self.assertNotEqual(replacement["segments"][0][0], "FL45")
        final = replacement["expected"][-1][-1]
        self.assertLess(math.hypot(final[0] - 1750, final[1] - 970), 30)

        progress["recovery_reason"] = "IR side clearance too small"
        ir_replacement = pp.plan_segment_recovery(plan, obstacles, progress)
        self.assertIsNotNone(ir_replacement)
        self.assertRegex(ir_replacement["segments"][0][0], r"^[FR]\d+$")


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
