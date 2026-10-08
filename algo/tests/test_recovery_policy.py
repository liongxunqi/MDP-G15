import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import recovery_policy as rp


class RecoveryPolicyTests(unittest.TestCase):
    def test_logged_buffer_escape_and_live_preflight(self):
        obstacles = [{"id": i + 1, "x": x, "y": y, "d": d} for i, (x, y, d) in enumerate([
            (5, 13, 6), (1, 9, 4), (6, 5, 0), (14, 8, 6),
            (16, 3, 6), (11, 12, 2), (15, 16, 2)])]
        plan = dict(self.plan, frame_shift_mm={"x": -114, "y": -125})
        report = dict(self.report, x_grid=2.53395471, y_grid=10.59110823, heading_deg=90.9)
        result = rp.recover_route(plan, obstacles, report)
        self.assertEqual(result["recovery_strategy"], "buffer_escape")
        token = result["buffer_escape"]
        self.assertTrue(token.startswith("R"))
        self.assertLessEqual(int(token[1:]), 20)
        self.assertEqual(result["segment_obstacles"], [None, "a", "b"])
        preflight = dict(report, event="instruction_preflight", instruction_index=-1)
        self.assertIsNone(rp.pp.assess_primitive_safety(result, obstacles, preflight, token))
        self.assertIsNotNone(rp.pp.assess_primitive_safety(plan, obstacles, preflight, token))
        self.assertIsNotNone(rp.pp.assess_primitive_safety(result, obstacles, preflight, "FR45"))
        end = result["expected"][0][-1]
        normal = dict(report, x_grid=end[0] / 100, y_grid=end[1] / 100, heading_deg=end[2])
        self.assertIsNone(rp.pp.buffer_escape_endpoint(plan, obstacles, normal, token))
        with patch.object(rp.pp, "_pose_clear", return_value=False):
            self.assertIsNone(rp.pp.buffer_escape_endpoint(plan, obstacles, report, token))
        self.assertIsNone(rp.pp.buffer_escape_endpoint(plan, obstacles, report, "R21"))
        self.assertIsNone(rp.pp.buffer_escape_endpoint(plan, obstacles, report, "F15"))

    def setUp(self):
        self.plan = {"segments": [["F10", "F10", "S"], ["F10", "S"]],
                     "segment_obstacles": ["a", "b"],
                     "frame_shift_mm": {"x": 0, "y": 0},
                     "expected": [[[500, 600, 0], [500, 700, 0], [500, 700, 0]],
                                  [[500, 800, 0], [500, 800, 0]]]}
        self.report = {"segment_index": 0, "instruction_index": 0,
                       "x_grid": 5, "y_grid": 6, "heading_deg": 0,
                       "previous_pose": [500, 500, 0]}

    @staticmethod
    def leg(*tokens):
        return {"segments": [list(tokens)]}

    def test_local_recovery_keeps_suffix_and_order(self):
        with patch.object(rp.pp, "plan_segment_recovery", return_value=self.leg("F10", "S")):
            result = rp.recover_route(self.plan, [], self.report)
        self.assertEqual(result["recovery_strategy"], "instruction_endpoint")
        self.assertEqual(result["segment_obstacles"], ["a", "b"])
        self.assertEqual(result["segments"][1], self.plan["segments"][1])

    def test_segment_fallback(self):
        with patch.object(rp.pp, "plan_segment_recovery",
                          side_effect=[None, self.leg("F10", "S")]):
            result = rp.recover_route(self.plan, [], self.report)
        self.assertEqual(result["recovery_strategy"], "segment_endpoint")

    def test_backtrack_both_legs_are_planned(self):
        with patch.object(rp.pp, "plan_segment_recovery", side_effect=[
                None, None, self.leg("R10", "S"), self.leg("F20", "S")]) as search:
            result = rp.recover_route(self.plan, [], self.report)
        self.assertEqual(result["recovery_strategy"], "previous_pose")
        self.assertEqual(search.call_args_list[-1].args[2]["y_grid"], 5)
        self.assertEqual(result["segment_obstacles"], ["a", "b"])

    def test_failed_escape_returns_no_route(self):
        with patch.object(rp.pp, "plan_segment_recovery", return_value=None):
            self.assertIsNone(rp.recover_route(self.plan, [], self.report))

    def test_mismatched_pending_order_is_rejected(self):
        report = dict(self.report, remaining_photo_ids=["b", "a"])
        with patch.object(rp.pp, "plan_segment_recovery") as search:
            self.assertIsNone(rp.recover_route(self.plan, [], report))
        search.assert_not_called()

    def test_real_search_recovers_straight_endpoint(self):
        result = rp.recover_route(self.plan, [], self.report)
        self.assertIsNotNone(result)
        self.assertEqual(result["segment_obstacles"], ["a", "b"])
        self.assertEqual(result["segments"][1], self.plan["segments"][1])

    def test_defers_blocked_target_and_retries_from_new_endpoint(self):
        with patch.object(rp.pp, "plan_segment_recovery", side_effect=[
                None, None, None, self.leg("F20", "S"), self.leg("R10", "S")]) as search:
            result = rp.recover_route(self.plan, [], self.report)
        self.assertEqual(result["segment_obstacles"], ["b", "a"])
        self.assertEqual(result["skipped_photo_ids"], [])
        self.assertEqual(result["recovery_strategy"], "defer_and_retry")
        self.assertEqual(search.call_args_list[-1].args[2]["y_grid"], 6)
        self.assertEqual(result["deferred_segments"], [1])

    def test_unreachable_target_is_explicit_not_a_fake_detection(self):
        report = dict(self.report, force_defer=True)
        with patch.object(rp.pp, "plan_segment_recovery", side_effect=[
                self.leg("F20", "S"), None]):
            result = rp.recover_route(self.plan, [], report)
        self.assertEqual(result["segment_obstacles"], ["b", "a"])
        self.assertEqual(result["skipped_photo_ids"], [])
        self.assertEqual(result["deferred_segments"], [1])

    def test_deferral_limit_prevents_repeated_cycling(self):
        plan = dict(self.plan, deferral_counts={"a": 2})
        with patch.object(rp.pp, "plan_segment_recovery", return_value=self.leg("F20", "S")) as search:
            result = rp.recover_route(plan, [], dict(self.report, force_defer=True))
        self.assertIsNone(result)
        self.assertEqual(search.call_count, 0)

    def test_real_reordered_route_uses_current_pose(self):
        result = rp.recover_route(self.plan, [], dict(self.report, force_defer=True))
        self.assertEqual(result["segment_obstacles"], ["b", "a"])
        self.assertEqual(result["segments"], [["F20", "S"], ["S"]])
        report = dict(self.report, segment_index=1, instruction_index=-1,
                      y_grid=8.1, remaining_photo_ids=["a"])
        retry = rp.recover_route(result, [], report)
        self.assertEqual(retry["segments"], [["R11", "S"]])
        self.assertEqual(retry["deferred_segments"], [])

    def test_later_segment_does_not_invalidate_local_repair(self):
        self.plan["segments"][1] = ["F999", "S"]
        result = rp.recover_route(self.plan, [], self.report)
        self.assertEqual(result["recovery_strategy"], "instruction_endpoint")
        self.assertEqual(result["segments"][1], ["F999", "S"])
        self.assertEqual(result["expected"][1], self.plan["expected"][1])

    def test_original_endpoint_survives_replacement_drift(self):
        self.plan["original_photo_endpoints"] = {"a": [500, 700, 0], "b": [500, 800, 0]}
        self.plan["expected"][0][-1] = [500, 760, 0]
        result = rp.recover_route(self.plan, [], self.report)
        self.assertAlmostEqual(result["expected"][0][-1][1], 700)
        self.assertEqual(result["original_photo_endpoints"]["a"], [500, 700, 0])

    def test_bad_endpoint_is_rejected_even_if_planner_returns_tokens(self):
        with patch.object(rp.pp, "plan_segment_recovery", return_value=self.leg("F1", "S")):
            self.assertIsNone(rp.recover_route(self.plan, [], self.report))

    def test_actual_heading_replay_rejects_collision(self):
        with patch.object(rp.pp, "plan_segment_recovery", return_value=self.leg("F10", "S")), \
             patch.object(rp.pp, "_replay_motion", return_value=None):
            self.assertIsNone(rp.recover_route(self.plan, [], self.report))

    def test_alignment_at_endpoint_requires_new_approach(self):
        report = dict(self.report, y_grid=7, alignment_recovery=True)
        with patch.object(rp.pp, "plan_segment_recovery", side_effect=[
                self.leg("R20", "S"), self.leg("F20", "S")]):
            result = rp.recover_route(self.plan, [], report)
        self.assertEqual(result["recovery_strategy"], "previous_pose")
