"""
test_calibration.py — offline checks for frozen calibration (protocol 4)

    python3 -m pytest -q test_calibration.py      # from rpi/mdp_rpi/

No robot needed. FakeFirmware answers the wire exactly the way rpilink.c does
for the commands these scripts use, so what is being tested is the RPi side:
token validation, the task startup sequence, the profile store, and the
arithmetic calibrate.py does with what you measure on the floor.
"""

import csv
import math
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

if "dotenv" not in sys.modules:
    dotenv = types.ModuleType("dotenv")
    dotenv.load_dotenv = lambda: None
    sys.modules["dotenv"] = dotenv

if "serial" not in sys.modules:
    serial = types.ModuleType("serial")
    serial.Serial = object
    serial.SerialException = Exception
    sys.modules["serial"] = serial

import calibrate  # noqa: E402
from communications import cal_profile  # noqa: E402
from communications.stm import STM, validate_token  # noqa: E402


class FakeFirmware(STM):
    """A protocol 8 robot (or older, via proto=), as far as the wire can tell."""

    LIMITS = {"D": (501, 29999), "L": (0, 2500), "T": (-80, 80),
              "G": (9000, 11000), "SL": (300, 574), "SR": (300, 626)}

    def __init__(self, proto=8):
        self.proto = proto
        self.cal = {"D": 6360, "L": 30, "T": 0, "G": 10000, "SL": 574, "SR": 574}
        self.learn = 0
        self.profile = 0
        self.zeroed = 0
        self.sent = []
        self._reply = None
        self.hdg = [0, 0, 0, 0, 0]          # what ?HDG answers, x10 fields
        self.after_move = None              # hook: token -> None
        self.has_safe = True
        self.fast = None                    # !FAST state, None = never sent
        self.imu_ready = 1
        self.imu_rate_x10 = 0               # ?IMU rate at rest, deg/s x10
        self.last_decel = 0                 # ?CAL field 8 (protocol 6)
        self.wpose = [0, 0, 0]              # ?WPOSE x_mm, y_mm, heading x10
        self.enc = [0, 0]                   # ?ENC counts A, B

    # Movement path ----------------------------------------------------------
    def send_line(self, tokens):
        tok = tokens[0] if isinstance(tokens, list) else tokens
        self.sent.append(tok)
        up = tok.upper()
        if up.startswith("!FAST"):
            self.fast = int(up[5:])
            self._reply = "OK" if self.proto >= 8 else "RESEND"
        elif up.startswith("!LEARN"):
            self.learn = int(up[6:])
            self._reply = "OK"
        elif up.startswith("!PROF"):
            # Locked to TIGHT and SAFE, as in rpilink.c; a board from before
            # SAFE (has_safe=False) refuses !PROF3 too.
            ok = up == "!PROF0" or (up == "!PROF3" and self.has_safe)
            if ok:
                self.profile = int(up[5:])
            self._reply = "OK" if ok else "RESEND"
        elif up == "!ZERO":
            self.zeroed += 1
            self.hdg = [0, 0, self.hdg[2], self.hdg[3], 0]
            self._reply = "OK"
        elif up.startswith("!CAL"):
            kind = up[4:6] if up[4] == "S" else up[4]
            val = int(up[4 + len(kind):])
            lo, hi = self.LIMITS[kind]
            if kind in ("SL", "SR") and self.proto < 5:
                self._reply = "RESEND"          # unknown token on old firmware
            elif lo <= val <= hi:
                self.cal[kind] = val
                self._reply = "OK"
            else:
                self._reply = "RESEND"
        else:
            if self.after_move:
                self.after_move(tok)
            self._reply = "OK"
        return True

    def wait_reply(self, timeout=None):
        r, self._reply = self._reply, None
        return r

    busy = False

    def is_busy(self, timeout=2.0):
        return self.busy

    # Query path --------------------------------------------------------------
    def query(self, command, timeout=2.0):
        tag = command.strip().upper().lstrip("?")
        if tag == "VER":
            return f"VER,MDPG15-STM32,{self.proto}"
        if tag == "CAL":
            c = self.cal
            reply = f"CAL,{c['D']},{c['L']},{c['T']},{c['G']},{self.learn}"
            if self.proto >= 5:
                reply += f",{c['SL']},{c['SR']}"
            if self.proto >= 6:
                reply += f",{self.last_decel}"
            return reply
        if tag == "WPOSE":
            return "WPOSE," + ",".join(str(v) for v in self.wpose)
        if tag == "ENC":
            return f"ENC,{self.enc[0]},{self.enc[1]}"
        if tag == "IMU":
            return (f"IMU,{self.imu_ready},0,{self.imu_rate_x10},"
                    f"{1 - self.imu_ready},0")
        if tag == "HDG":
            return "HDG," + ",".join(str(v) for v in self.hdg)
        return None


class TokenValidationTests(unittest.TestCase):
    def test_protocol_4_tokens_are_accepted(self):
        for tok in ("!LEARN0", "!LEARN1", "!CALG10000", "!CALG9000",
                    "!CALG11000", "?HDG", "!PROF0"):
            ok, reason = validate_token(tok)
            self.assertTrue(ok, f"{tok}: {reason}")

    def test_protocol_5_steering_tokens(self):
        for tok in ("!CALSL574", "!CALSL300", "!CALSR626", "!calsr500"):
            ok, reason = validate_token(tok)
            self.assertTrue(ok, f"{tok}: {reason}")
        for tok in ("!CALSL575", "!CALSR627", "!CALSL299", "!CALSL-5"):
            ok, _ = validate_token(tok)
            self.assertFalse(ok, tok)
        # The lag setter must not be mistaken for the left steering one.
        self.assertTrue(validate_token("!CALL42")[0])

    def test_out_of_range_is_refused_before_the_wire(self):
        for tok in ("!LEARN2", "!CALG8999", "!CALG11001", "!CALG-10000"):
            ok, _ = validate_token(tok)
            self.assertFalse(ok, tok)


class ProfileStoreMixin:
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._patch = mock.patch.multiple(
            cal_profile, PROFILE_DIR=os.path.join(self._tmp.name, "cal"),
            HEADING_LOG_DIR=os.path.join(self._tmp.name, "hdg"))
        self._patch.start()

    def tearDown(self):
        self._patch.stop()
        self._tmp.cleanup()

    def saved_arena(self, **extra):
        p = {"decel_x10": 6012, "lag_ms_x10": 42, "trim_us": -3,
             "gyro_x10000": 10087, "verified": True, "note": "test"}
        p.update(extra)
        cal_profile.save("arena", p)


class PrepareForTaskTests(ProfileStoreMixin, unittest.TestCase):
    def test_restores_freezes_and_zeroes_in_order(self):
        self.saved_arena()
        fw = FakeFirmware()
        fw.learn = 1                     # left on by someone - must be switched off

        self.assertTrue(cal_profile.prepare_for_task(fw, name="arena", required=True))

        self.assertEqual(fw.cal, {"D": 6012, "L": 42, "T": -3, "G": 10087,
                                  "SL": 574, "SR": 574})
        self.assertEqual(fw.learn, 0)
        self.assertEqual(fw.zeroed, 1)
        # Learning off before anything else, zero last.
        self.assertEqual(fw.sent[0], "!LEARN0")
        self.assertEqual(fw.sent[-1], "!ZERO")

    def test_missing_profile_refuses_to_start(self):
        fw = FakeFirmware()
        self.assertFalse(cal_profile.prepare_for_task(fw, name="nowhere", required=True))
        self.assertEqual(fw.zeroed, 0)

    def test_no_profile_named_refuses_unless_not_required(self):
        fw = FakeFirmware()
        self.assertFalse(cal_profile.prepare_for_task(fw, name="", required=True))
        self.assertTrue(cal_profile.prepare_for_task(fw, name="", required=False))
        self.assertEqual(fw.learn, 0)

    def test_old_firmware_is_refused(self):
        self.saved_arena()
        fw = FakeFirmware(proto=3)
        self.assertFalse(cal_profile.prepare_for_task(fw, name="arena", required=True))
        self.assertEqual(fw.sent, [])

    def test_per_side_steering_is_restored(self):
        self.saved_arena(steer_left_us=574, steer_right_us=541)
        fw = FakeFirmware()
        self.assertTrue(cal_profile.prepare_for_task(fw, name="arena", required=True))
        self.assertEqual((fw.cal["SL"], fw.cal["SR"]), (574, 541))

    def test_protocol_4_board_skips_default_steering(self):
        self.saved_arena()                       # no steering fields: defaults
        fw = FakeFirmware(proto=4)
        self.assertTrue(cal_profile.prepare_for_task(fw, name="arena", required=True))
        self.assertFalse(any(t.upper().startswith("!CALS") for t in fw.sent))

    def test_protocol_4_board_refuses_measured_steering(self):
        self.saved_arena(steer_left_us=574, steer_right_us=541)
        fw = FakeFirmware(proto=4)
        self.assertFalse(cal_profile.prepare_for_task(fw, name="arena", required=True))
        self.assertEqual(fw.zeroed, 0)

    def test_profile_without_gyro_scale_restores_as_one(self):
        self.saved_arena()
        p = cal_profile.load("arena")
        del p["gyro_x10000"]
        cal_profile.save("arena", p)
        fw = FakeFirmware()
        fw.cal["G"] = 10300
        self.assertTrue(cal_profile.prepare_for_task(fw, name="arena", required=True))
        self.assertEqual(fw.cal["G"], 10000)


class ArcProfileTests(ProfileStoreMixin, unittest.TestCase):
    """TIGHT and SAFE each keep their own calibration."""

    def test_old_calibration_without_arc_is_tight(self):
        self.saved_arena()
        path = cal_profile.profile_path("arena")
        import json
        with open(path, encoding="utf-8") as fh:
            raw = json.load(fh)
        raw.pop("arc_profile", None)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(raw, fh)
        fw = FakeFirmware()
        fw.profile = 3
        self.assertTrue(cal_profile.prepare_for_task(fw, name="arena", required=True))
        self.assertIn("!PROF0", fw.sent)
        self.assertEqual(fw.profile, 0)
        self.assertEqual(cal_profile.selected_arc, cal_profile.ARC_TIGHT)

    def test_safe_calibration_selects_safe(self):
        self.saved_arena(arc_profile=cal_profile.ARC_SAFE)
        fw = FakeFirmware()
        self.assertTrue(cal_profile.prepare_for_task(fw, name="arena", required=True))
        self.assertIn("!PROF3", fw.sent)
        self.assertEqual(fw.profile, 3)
        self.assertEqual(cal_profile.selected_arc, cal_profile.ARC_SAFE)

    def test_safe_calibration_on_firmware_without_safe_is_refused(self):
        self.saved_arena(arc_profile=cal_profile.ARC_SAFE)
        fw = FakeFirmware()
        fw.has_safe = False
        self.assertFalse(cal_profile.prepare_for_task(fw, name="arena", required=True))
        self.assertEqual(fw.zeroed, 0)

    def test_tight_only_task_refuses_safe_calibration(self):
        self.saved_arena(arc_profile=cal_profile.ARC_SAFE)
        fw = FakeFirmware()
        self.assertFalse(cal_profile.prepare_for_task(
            fw, name="arena", required=True, allowed_arcs={cal_profile.ARC_TIGHT}))
        self.assertEqual(fw.sent, [])

    def test_step_on_existing_profile_keeps_its_arc(self):
        self.saved_arena(arc_profile=cal_profile.ARC_SAFE)
        fw = FakeFirmware()
        fw.after_move = lambda tok: setattr(fw, "hdg", [3600, 3600, 900, 900, 0])
        with mock.patch("builtins.input", side_effect=["", "3.6", "", "3.6"]):
            self.assertTrue(calibrate.run_one_step(fw, "arena", "gyro"))
        self.assertEqual(fw.profile, 3)
        self.assertEqual(cal_profile.arc_of(cal_profile.load("arena")),
                         cal_profile.ARC_SAFE)

    def test_step_cannot_mix_arcs_in_one_profile(self):
        self.saved_arena()                                   # TIGHT
        before = cal_profile.load("arena")
        fw = FakeFirmware()
        with mock.patch("builtins.input", return_value=""):
            self.assertFalse(calibrate.run_one_step(
                fw, "arena", "gyro", arc=cal_profile.ARC_SAFE))
        self.assertEqual(cal_profile.load("arena"), before)

    def test_new_safe_profile_is_saved_as_safe(self):
        fw = FakeFirmware()
        fw.after_move = lambda tok: setattr(fw, "hdg", [3600, 3600, 900, 900, 0])
        with mock.patch("builtins.input", side_effect=["", "3.6", "", "3.6"]):
            self.assertTrue(calibrate.run_one_step(
                fw, "arena_safe", "gyro", arc=cal_profile.ARC_SAFE))
        self.assertEqual(cal_profile.arc_of(cal_profile.load("arena_safe")),
                         cal_profile.ARC_SAFE)

    def test_safe_radius_is_found_not_matched_to_tight(self):
        self.assertIsNone(calibrate.default_target(cal_profile.ARC_SAFE))
        self.assertEqual(calibrate.default_target(cal_profile.ARC_TIGHT),
                         calibrate.RADIUS_TARGET_CM)


class ZeroHeadingTests(unittest.TestCase):
    def test_zero_clears_commanded_and_actual(self):
        fw = FakeFirmware()
        fw.hdg = [-1800, -1823, -900, -912, 0]    # robot straightened by hand
        self.assertTrue(cal_profile.zero_heading(fw))
        self.assertEqual(fw.sent, ["!ZERO"])
        self.assertEqual(fw.hdg[:2], [0, 0])

    def test_refuses_while_moving(self):
        fw = FakeFirmware()
        fw.busy = True
        self.assertFalse(cal_profile.zero_heading(fw))
        self.assertEqual(fw.sent, [])

    def test_reports_failure_if_heading_did_not_zero(self):
        fw = FakeFirmware()
        fw.zero_odometry = lambda timeout=None: True   # OK, but nothing happened
        fw.hdg = [0, 45, 0, 0, 0]
        self.assertFalse(cal_profile.zero_heading(fw))

    def test_task_startup_zeroes_last(self):
        fw = FakeFirmware()
        fw.hdg = [900, 870, 0, 0, 0]
        self.assertTrue(cal_profile.prepare_for_task(fw, name="", required=False))
        self.assertEqual(fw.sent[-1], "!ZERO")
        self.assertEqual(fw.hdg[:2], [0, 0])


class TaskGyroCheckTests(ProfileStoreMixin, unittest.TestCase):
    def test_task_refuses_to_start_on_encoder_heading(self):
        self.saved_arena()
        fw = FakeFirmware()
        fw.imu_ready = 0
        with self.assertLogs(level="ERROR") as logs:
            self.assertFalse(cal_profile.prepare_for_task(fw, name="arena",
                                                          required=True))
        self.assertTrue(any("ENCODER heading" in m and "not starting" in m
                            for m in logs.output))

    def test_a_drifting_gyro_is_refused(self):
        # 6.4 deg/s standing still: bias measured while it was being handled.
        self.saved_arena()
        fw = FakeFirmware()
        fw.imu_rate_x10 = 64
        with self.assertLogs(level="ERROR") as logs:
            self.assertFalse(cal_profile.prepare_for_task(fw, name="arena",
                                                          required=True))
        self.assertTrue(any("+6.4°/s" in m for m in logs.output))

    def test_tasks_switch_fast_mode_off_unless_asked(self):
        self.saved_arena()
        fw = FakeFirmware()
        fw.fast = 1                                  # left on by a Task 2 run
        self.assertTrue(cal_profile.prepare_for_task(fw, name="arena", required=True))
        self.assertEqual(fw.fast, 0)
        self.assertTrue(cal_profile.prepare_for_task(fw, name="arena", required=True,
                                                     fast=True))
        self.assertEqual(fw.fast, 1)

    def test_fast_mode_on_older_firmware_warns_and_runs(self):
        self.saved_arena()
        fw = FakeFirmware(proto=7)
        with self.assertLogs(level="WARNING"):
            self.assertTrue(cal_profile.prepare_for_task(fw, name="arena",
                                                         required=True, fast=True))
        self.assertIsNone(fw.fast)                   # nothing sent

    def test_task_starts_with_the_gyro_in_use(self):
        self.saved_arena()
        self.assertTrue(cal_profile.prepare_for_task(FakeFirmware(), name="arena",
                                                     required=True))


class HeadingLogTests(ProfileStoreMixin, unittest.TestCase):
    def test_writes_one_row_per_move_and_reports_error(self):
        fw = FakeFirmware()
        fw.hdg = [-900, -895, -900, -895, 0]      # 0.5 short on the last arc
        log = cal_profile.HeadingLog(fw, "unit")
        h = log.record("segment 0")
        log.close()

        self.assertAlmostEqual(h["error"], -0.5)
        self.assertAlmostEqual(h["last_stop_err"], 0.5)
        with open(log.path, newline="", encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["label"], "segment 0")
        self.assertEqual(rows[0]["error"], "-0.5")
        self.assertEqual(rows[0]["gyro"], "ok")
        self.assertFalse(log.gyro_lost)

    def test_gyro_lost_mid_run_is_logged_once_and_recorded(self):
        fw = FakeFirmware()
        log = cal_profile.HeadingLog(fw, "unit")
        log.record("segment 0")
        fw.imu_ready = 0
        with self.assertLogs(level="ERROR") as logs:
            log.record("segment 1")
            log.record("segment 2")
        log.close()
        self.assertTrue(log.gyro_lost)
        self.assertEqual(sum("GYRO LOST" in m for m in logs.output), 1)
        with open(log.path, newline="", encoding="utf-8") as fh:
            gyro = [r["gyro"] for r in csv.DictReader(fh)]
        self.assertEqual(gyro, ["ok", "LOST", "LOST"])

    def test_carry_cap_is_warned_once(self):
        fw = FakeFirmware()
        fw.hdg = [0, 70, 0, 0, 1]
        log = cal_profile.HeadingLog(fw, "unit")
        with self.assertLogs(level="WARNING"):
            log.record("segment 3")
        log.close()


class GyroStepTests(unittest.TestCase):
    def run_gyro(self, fw, answers):
        result = {"checks": {}}
        with mock.patch("builtins.input", side_effect=answers):
            self.assertTrue(calibrate.step_gyro(fw, result))
        return result

    def test_turning_past_the_edge_means_the_gyro_reads_low(self):
        # The gyro said 360 each time; the floor says 3.6 degrees past, both
        # ways - so the gyro under-reads by 1% and the scale goes up 1%.
        fw = FakeFirmware()
        # Each loop starts with !ZERO; the gyro total is what the moves leave.
        fw.after_move = lambda tok: setattr(fw, "hdg", [3600, 3600, 900, 900, 0])
        result = self.run_gyro(fw, ["", "3.6", "", "3.6"])
        self.assertEqual(fw.cal["G"], 10100)
        self.assertAlmostEqual(result["checks"]["gyro"]["to"], 1.01, places=4)

    def test_offset_over_span_is_an_angle(self):
        # 12 mm over 200 mm along the robot's side is atan(0.06) = 3.43 degrees.
        with mock.patch("builtins.input", return_value="-12/200"):
            self.assertAlmostEqual(calibrate.ask_angle("x"), -3.4336, places=3)
        with mock.patch("builtins.input", return_value="1.5"):
            self.assertAlmostEqual(calibrate.ask_angle("x"), 1.5)


class OneStepTests(ProfileStoreMixin, unittest.TestCase):
    def gyro_robot(self):
        fw = FakeFirmware()
        fw.after_move = lambda tok: setattr(fw, "hdg", [3600, 3600, 900, 900, 0])
        return fw

    def test_rerun_keeps_the_other_steps(self):
        self.saved_arena(checks={"radius": {"target_cm": 29.1, "rounds": []},
                                 "brakes": {"worst_deg": 0.3}},
                         steer_left_us=425, steer_right_us=586)
        fw = self.gyro_robot()
        with mock.patch("builtins.input", side_effect=["", "3.6", "", "3.6"]):
            self.assertTrue(calibrate.run_one_step(fw, "arena", "gyro"))
        p = cal_profile.load("arena")
        self.assertEqual(set(p["checks"]), {"radius", "brakes", "gyro"})
        self.assertEqual(p["gyro_x10000"], round(10087 * 1.01))
        self.assertEqual((p["steer_left_us"], p["steer_right_us"]), (425, 586))
        self.assertEqual(p["note"], "test")
        self.assertFalse(p["verified"])

    def test_missing_profile_starts_from_the_robot(self):
        fw = self.gyro_robot()
        with mock.patch("builtins.input", side_effect=["", "3.6", "", "3.6"]):
            self.assertTrue(calibrate.run_one_step(fw, "fresh", "gyro", note="hpl"))
        p = cal_profile.load("fresh")
        self.assertEqual(set(p["checks"]), {"gyro"})
        self.assertEqual(p["note"], "hpl")
        self.assertIn("decel_x10", p)

    def test_failed_step_saves_nothing(self):
        self.saved_arena()
        before = cal_profile.load("arena")
        fw = FakeFirmware(proto=4)
        with mock.patch("builtins.input", return_value=""):
            self.assertFalse(calibrate.run_one_step(fw, "arena", "radius"))
        self.assertEqual(cal_profile.load("arena"), before)


class RadiusStepTests(unittest.TestCase):
    """
    A robot whose radius is K / deflection per side - wider the less it
    steers, like the real linkage over this range. The 'person' answers each
    chord prompt with what a tape would read for the side just driven.
    """

    def make(self, k_left_cm, k_right_cm, rev_scale=1.0):
        # rev_scale: reverse radius / forward radius on the same steering.
        fw = FakeFirmware()
        fw.last = None

        def moved(tok):
            fw.last = tok
            deg = int(tok[2:]) * 10
            # Body rotation: FR and RL clockwise, FL and RR anticlockwise.
            fw.hdg = [0, 0, deg, -deg if tok[:2] in ("FR", "RL") else deg, 0]
        fw.after_move = moved

        def person(prompt):
            if "distance" in prompt.lower():
                side = "S" + fw.last[1]
                k = k_right_cm if side == "SR" else k_left_cm
                if fw.last[0] == "R":
                    k *= rev_scale
                half = math.radians(int(fw.last[2:]) / 2.0)
                return f"{k / fw.cal[side] * 2 * math.sin(half):.2f}"
            if "[y/n]" in prompt.lower():
                return "y"
            return ""
        return fw, person

    def test_narrower_left_is_steered_less_until_both_match(self):
        fw, person = self.make(k_left_cm=28.0 * 574, k_right_cm=29.1 * 574)
        result = {"checks": {}}
        with mock.patch("builtins.input", side_effect=person):
            self.assertTrue(calibrate.step_radius(fw, result))
        self.assertEqual(fw.cal["SR"], 574)
        self.assertLess(fw.cal["SL"], 574)
        last = result["checks"]["radius"]["rounds"][-1]
        self.assertAlmostEqual(last["radius_l_cm"], 29.1, delta=0.5)
        self.assertAlmostEqual(last["radius_r_cm"], 29.1, delta=0.5)
        self.assertEqual(result["steer_left_us"], fw.cal["SL"])

    def test_wider_left_at_its_limit_matches_right_to_it_on_request(self):
        # Left is already at the 574 limit and still 31 cm: it cannot get
        # tighter, so the right side is brought out to 31 cm instead.
        fw, person = self.make(k_left_cm=31.0 * 574, k_right_cm=29.1 * 574)
        result = {"checks": {}}
        with mock.patch("builtins.input", side_effect=person),                 self.assertLogs(level="WARNING") as logs:
            self.assertTrue(calibrate.step_radius(fw, result))
        self.assertEqual(fw.cal["SL"], 574)
        self.assertLess(fw.cal["SR"], 574)
        self.assertEqual(result["checks"]["radius"]["target_cm"], 31.0)
        self.assertTrue(any("TURN_RADIUS_MM" in m for m in logs.output))

    def test_reverse_arcs_are_measured_on_the_same_steering(self):
        fw, person = self.make(k_left_cm=29.1 * 574, k_right_cm=29.1 * 574)
        result = {"checks": {}}
        with mock.patch("builtins.input", side_effect=person):
            self.assertTrue(calibrate.step_radius(fw, result))
        deg = calibrate.RADIUS_ARC_DEG
        for tok in (f"FR{deg}", f"RR{deg}", f"FL{deg}", f"RL{deg}"):
            self.assertEqual(fw.sent.count(tok), calibrate.RADIUS_RUNS)
        last = result["checks"]["radius"]["rounds"][-1]
        self.assertAlmostEqual(last["radius_rr_cm"], 29.1, delta=0.1)
        self.assertAlmostEqual(last["radius_lr_cm"], 29.1, delta=0.1)

    def test_reverse_wider_than_forward_is_warned_and_split_down_the_middle(self):
        # Reverse 6% wider: steering moves both together, so the side is
        # tuned on the mean and the mismatch is reported, not chased.
        fw, person = self.make(k_left_cm=29.1 * 574, k_right_cm=29.1 * 574,
                               rev_scale=1.06)
        result = {"checks": {}}
        with mock.patch("builtins.input", side_effect=person),                 self.assertLogs(level="WARNING") as logs:
            self.assertTrue(calibrate.step_radius(fw, result))
        rad = result["checks"]["radius"]
        last = rad["rounds"][-1]
        self.assertAlmostEqual(last["radius_r_cm"], rad["target_cm"], delta=0.5)
        self.assertAlmostEqual(last["radius_l_cm"], rad["target_cm"], delta=0.5)
        self.assertGreater(last["radius_rr_cm"], last["radius_rf_cm"] + 1.0)
        self.assertTrue(any("reverse" in m for m in logs.output))

    def test_auto_finds_the_tightest_radius_both_sides_can_turn(self):
        # Full lock: left 574 -> 31.0 cm, right 626 -> 28.0 cm. Left cannot
        # go tighter, so 31.0 is the target and the right is widened to it.
        fw, person = self.make(k_left_cm=31.0 * 574, k_right_cm=28.0 * 626)
        fw.cal["SL"], fw.cal["SR"] = 425, 586
        result = {"checks": {}}
        with mock.patch("builtins.input", side_effect=person), \
                self.assertLogs(level="INFO") as logs:
            self.assertTrue(calibrate.step_radius(fw, result, target_cm=None))
        rad = result["checks"]["radius"]
        self.assertEqual((rad["rounds"][0]["steer_l"], rad["rounds"][0]["steer_r"]),
                         (574, 626))
        self.assertEqual(rad["target_cm"], 31.0)
        self.assertEqual(fw.cal["SL"], 574)
        self.assertLess(fw.cal["SR"], 626)
        self.assertAlmostEqual(rad["rounds"][-1]["radius_r_cm"], 31.0, delta=0.5)
        self.assertTrue(any("TURN_RADIUS_MM" in m and "310" in m for m in logs.output))

    def test_needs_protocol_5(self):
        fw = FakeFirmware(proto=4)
        with mock.patch("builtins.input", return_value=""):
            self.assertFalse(calibrate.step_radius(fw, {"checks": {}}))


class DriftTests(unittest.TestCase):
    def test_drift_after_handling_is_flagged(self):
        # Left alone: 0.1° in 10 s. Just handled: 2° in 10 s.
        fw = FakeFirmware()
        rates = iter([1, 20])                  # x10 degrees per reading
        clock = [0.0]

        def zero_then_rate(*_):
            fw.step, fw.t0 = next(rates), clock[0]
            fw.hdg = [0, 0, 0, 0, 0]
            return True

        real_query = fw.query

        def query(cmd, timeout=2.0):
            if cmd == "?HDG":
                clock[0] += 1.0
                fw.hdg[1] = int((clock[0] - fw.t0) * fw.step / 10)
            return real_query(cmd, timeout)
        fw.query = query

        with mock.patch.object(calibrate.cal_profile, "zero_heading",
                               side_effect=zero_then_rate), \
                mock.patch.object(calibrate.time, "sleep"), \
                mock.patch.object(calibrate.time, "monotonic",
                                  side_effect=lambda: clock[0]), \
                mock.patch("builtins.input", return_value=""), \
                self.assertLogs(level="WARNING") as logs:
            self.assertTrue(calibrate.gyro_drift(fw, seconds=10))
        self.assertTrue(any("bias" in m for m in logs.output))


class ReverseCheckTests(unittest.TestCase):
    """
    A robot whose reverse arc sign and reverse heading-hold sign can each be
    built wrong. The 'person' twists it 5 degrees whenever asked.
    """

    def make(self, arc_wrong=False, hold_wrong=False, fwd_hold_wrong=False,
             cross_wrong=False):
        fw = FakeFirmware()
        fw.pose = (0, 0)
        real_send = fw.send_line
        real_query = fw.query

        def query(command, timeout=2.0):
            if command.strip().upper() == "?POSE":
                return f"POSE,{fw.pose[0]},{fw.pose[1]},0"
            return real_query(command, timeout)
        fw.query = query

        def send_line(tokens):
            tok = tokens[0] if isinstance(tokens, list) else tokens
            real_send(tokens)
            up = tok.upper()
            if up[:2] in ("FR", "FL", "RR", "RL"):
                want = calibrate.REV_ARC_EXPECT[up[:2]]
                if arc_wrong and up[0] == "R":
                    # Firmware aims the wrong way, the body rotates the right
                    # way, so it aborts after 20 degrees - and, like the
                    # firmware, leaves last_turned at the PREVIOUS arc.
                    fw.hdg[1] += want * 200
                    fw._reply = "FAIL,WRONGWAY"
                else:
                    fw.hdg[1] += want * 450
                    fw.hdg[3] = want * 450
            elif up[0] in ("F", "R") and up[1:].isdigit():
                wrong = hold_wrong if up[0] == "R" else fwd_hold_wrong
                fw.hdg[1] = fw.hdg[1] * 3 if wrong else fw.hdg[1] // 10
                # Offset from the line at the end: the twist pushed it ~8 mm
                # off; right sign pulls it back, wrong sign carries it away.
                fw.pose = (0, 30 if (cross_wrong and up[0] == "R") else 3)
            return True
        fw.send_line = send_line

        def person(prompt):
            if "twist" in prompt.lower():
                fw.hdg[1] = 50 if "LEFT" in prompt else -50
            return ""
        return fw, person

    def run_check(self, fw, person):
        with mock.patch("builtins.input", side_effect=person), \
                self.assertLogs(level="INFO") as logs:
            ok = calibrate.reverse_check(fw)
        return ok, logs.output

    def test_correct_signs_pass(self):
        fw, person = self.make()
        ok, _ = self.run_check(fw, person)
        self.assertTrue(ok)
        for tok in ("FR45", "RR45", "FL45", "RL45", "F100", "R100"):
            self.assertIn(tok, fw.sent)

    def test_wrong_reverse_arc_sign_is_caught(self):
        fw, person = self.make(arc_wrong=True)
        ok, logs = self.run_check(fw, person)
        self.assertFalse(ok)
        self.assertTrue(any("Motion_DriveArc" in m for m in logs))

    def test_wrong_reverse_hold_sign_is_caught(self):
        fw, person = self.make(hold_wrong=True)
        ok, logs = self.run_check(fw, person)
        self.assertFalse(ok)
        self.assertTrue(any("odom.c" in m for m in logs))

    def test_wrong_reverse_cross_track_sign_is_caught(self):
        fw, person = self.make(cross_wrong=True)
        ok, logs = self.run_check(fw, person)
        self.assertFalse(ok)
        self.assertTrue(any("cross = -cross" in m for m in logs))

    def test_cross_offset_is_measured_from_the_held_line(self):
        # Held -5 deg, drove 1000 mm straight along that line: 0 off it.
        fw = FakeFirmware()
        x = 1000.0
        y = x * calibrate.math.tan(calibrate.math.radians(-5.0))
        fw.query = lambda c, timeout=2.0: f"POSE,{x:.0f},{y:.1f},0"
        self.assertAlmostEqual(calibrate._cross_offset_mm(fw, -7.0), 0.0, delta=1.0)

    def test_overshoot_the_right_way_is_a_pass(self):
        # Floor run 2026-10-09: F100 twisted to -5.0 finished at +4.5, R100
        # +5.0 -> -3.7. Swung past straight, but turned the right way.
        fw, person = self.make()
        real_send = fw.send_line

        def send_line(tokens):
            tok = tokens[0] if isinstance(tokens, list) else tokens
            twist = fw.hdg[1]
            real_send(tokens)
            if tok in ("F100", "R100"):
                fw.hdg[1] = -twist * 9 // 10
            return True
        fw.send_line = send_line
        ok, logs = self.run_check(fw, person)
        self.assertTrue(ok)
        self.assertTrue(any("overshot" in m for m in logs))

    def test_failed_forward_control_distrusts_the_result(self):
        fw, person = self.make(fwd_hold_wrong=True)
        ok, logs = self.run_check(fw, person)
        self.assertFalse(ok)
        self.assertTrue(any("FORWARD control failed" in m for m in logs))


class StopSummaryTests(unittest.TestCase):
    def test_worst_and_per_direction(self):
        out = calibrate.summarise_stops([("FR90", 0.3), ("RR90", -0.2),
                                         ("FR90", 0.5), ("FL90", -0.1)])
        self.assertEqual(out["worst_deg"], 0.5)
        self.assertEqual(out["by_direction"]["FR"], 0.4)


class WireValidationTests(unittest.TestCase):
    def test_fast_mode_tokens(self):
        for tok in ("!FAST0", "!FAST1"):
            self.assertTrue(validate_token(tok)[0], tok)
        self.assertFalse(validate_token("!FAST2")[0])

    def test_new_queries_are_known(self):
        for tok in ("?WPOSE", "?ENC"):
            self.assertTrue(validate_token(tok)[0], tok)

    def test_safe_profile_can_be_selected(self):
        # Was refused before reaching the wire: SAFE could never be selected.
        self.assertTrue(validate_token("!PROF3")[0])
        self.assertFalse(validate_token("!PROF4")[0])

    def test_leading_zeros_are_refused(self):
        # "F00" is "F100" with a byte lost; as F0 it would drive to a wall.
        for tok in ("F00", "F050", "FR090", "R007"):
            self.assertFalse(validate_token(tok)[0], tok)
        for tok in ("F0", "F100", "FR180", "R7"):
            self.assertTrue(validate_token(tok)[0], tok)


class StraightStepTests(unittest.TestCase):
    def test_trim_is_learned_going_out_not_coming_back(self):
        fw = FakeFirmware()
        learning = []
        fw.after_move = lambda tok: learning.append((tok, fw.learn))
        answers = iter(["", "0.2", "100.0"] * calibrate.STRAIGHT_MAX_RUNS)
        with mock.patch("builtins.input", side_effect=lambda _p: next(answers)):
            self.assertTrue(calibrate.step_straight(fw, {"checks": {}}))
        self.assertTrue(learning)
        for tok, on in learning:
            self.assertEqual(on, 1 if tok.startswith("F") else 0, tok)
        self.assertEqual(fw.learn, 0)


class OdometryCrossCheckTests(unittest.TestCase):
    def test_tape_close_to_odometry_is_quiet(self):
        with self.assertNoLogs(level="WARNING"):
            odo = calibrate.check_chord("FR180", 58.0, {"x_mm": 0, "y_mm": 0},
                                        {"x_mm": 30, "y_mm": 595})
        self.assertAlmostEqual(odo, 59.6, places=1)

    def test_misread_tape_is_flagged(self):
        # 48 read for 58: a misread 10 cm.
        with self.assertLogs(level="WARNING") as logs:
            calibrate.check_chord("FL180", 48.0, {"x_mm": 0, "y_mm": 0},
                                  {"x_mm": 0, "y_mm": 580})
        self.assertTrue(any("tape" in m for m in logs.output))

    def test_radius_step_queries_odometry_around_each_arc(self):
        fw = FakeFirmware()
        queried = []
        real_query = fw.query
        fw.query = lambda c, timeout=2.0: (queried.append(c.upper()),
                                           real_query(c, timeout))[1]
        fw.after_move = lambda tok: setattr(fw, "hdg", [0, 0, 1800, 1800, 0])
        with mock.patch("builtins.input",
                        side_effect=lambda p: "58.2" if "distance" in p.lower() else ""):
            calibrate.step_radius(fw, {"checks": {}})
        self.assertGreater(queried.count("?WPOSE"), 0)

    def test_no_odometry_query_before_protocol_6(self):
        fw = FakeFirmware(proto=5)
        fw.after_move = lambda tok: setattr(fw, "hdg", [0, 0, 1800, 1800, 0])
        fw.read_wpose = mock.Mock(side_effect=AssertionError("queried ?WPOSE"))
        with mock.patch("builtins.input",
                        side_effect=lambda p: "58.2" if "distance" in p.lower() else ""):
            calibrate.step_radius(fw, {"checks": {}})


class DistanceOutlierTests(unittest.TestCase):
    def straight(self, answers, runs):
        fw = FakeFirmware()
        it = iter(answers)
        with mock.patch.object(calibrate, "STRAIGHT_MAX_RUNS", runs), \
                mock.patch("builtins.input", side_effect=lambda _p: next(it)), \
                self.assertLogs(level="WARNING") as logs:
            self.assertTrue(calibrate.step_straight(fw, {"checks": {}}))
        return logs.output

    def test_one_long_run_of_three_is_named(self):
        logs = self.straight(["", "2.0", "100.0", "", "2.0", "100.2",
                              "", "0.2", "104.0"], runs=3)
        self.assertTrue(any("encoder" in m and "3 (104.0 cm)" in m for m in logs))

    def test_two_runs_that_disagree_are_flagged(self):
        logs = self.straight(["", "2.0", "100.0", "", "2.0", "104.0"], runs=2)
        self.assertTrue(any("encoder" in m and "apart" in m for m in logs))


class EncoderCheckTests(unittest.TestCase):
    def run_check(self, counts_a, counts_b):
        fw = FakeFirmware()
        turns = iter([(counts_a, 0), (0, counts_b)])

        def person(_prompt):
            da, db = next(turns)
            fw.enc = [fw.enc[0] + da, fw.enc[1] + db]
            return ""
        with mock.patch("builtins.input", side_effect=person):
            return calibrate.encoder_check(fw, turns=5)

    def test_both_wheels_counting_pass(self):
        self.assertTrue(self.run_check(7800, 7790))

    def test_a_wheel_dropping_counts_fails(self):
        with self.assertLogs(level="ERROR") as logs:
            self.assertFalse(self.run_check(7400, 7800))
        self.assertTrue(any("dropping counts" in m for m in logs.output))

    def test_a_backwards_encoder_is_named(self):
        with self.assertLogs(level="ERROR") as logs:
            self.assertFalse(self.run_check(7800, -7800))
        self.assertTrue(any("ENCODER_B_INVERT" in m for m in logs.output))

    def test_needs_protocol_7(self):
        with self.assertLogs(level="ERROR"):
            self.assertFalse(calibrate.encoder_check(FakeFirmware(proto=6)))


class FastRadiusTests(unittest.TestCase):
    """
    radius --fast on a robot whose radius is K / deflection per side and
    whose odometry reads 1% short of the tape. `bad` names arcs (by count
    of that token so far) that stop off their aim, like a surge would.
    """

    def make(self, k_left_cm, k_right_cm, bad=None):
        fw = FakeFirmware()
        fw.last, fw.count, fw.x = None, {}, 0
        tapes = []

        def chord_cm(tok):
            k = k_right_cm if tok[1] == "R" else k_left_cm
            half = math.radians(int(tok[2:]) / 2.0)
            return k / fw.cal["S" + tok[1]] * 2 * math.sin(half)

        def moved(tok):
            fw.last = tok
            fw.count[tok] = fw.count.get(tok, 0) + 1
            deg = int(tok[2:]) * 10
            sign = -1 if tok[:2] in ("FR", "RL") else 1
            off = 50 if (bad and (tok, fw.count[tok]) in bad) else 0
            fw.hdg = [0, 0, sign * deg, sign * (deg - off), 0]
            fw.x += round(chord_cm(tok) * 10 / 1.01)     # odometry 1% short
            fw.wpose = [fw.x, 0, 0]

        def person(prompt):
            if "distance" in prompt.lower():
                tapes.append(fw.last)
                return f"{chord_cm(fw.last):.2f}"
            return ""
        fw.after_move = moved
        return fw, person, tapes

    def run_radius(self, fw, person, arc=cal_profile.ARC_SAFE, **kw):
        result = {"checks": {}, "arc_profile": arc}
        with mock.patch("builtins.input", side_effect=person):
            ok = calibrate.step_radius(fw, result, fast=True, **kw)
        return ok, result

    def test_each_kind_is_taped_once_however_many_rounds(self):
        fw, person, tapes = self.make(28.0 * 574, 29.1 * 574)
        ok, result = self.run_radius(fw, person, target_cm=29.1)
        self.assertTrue(ok)
        rounds = result["checks"]["radius"]["rounds"]
        self.assertGreater(len(rounds), 1)              # it did iterate
        self.assertEqual(sorted(tapes), ["FL180", "FR180", "RL180", "RR180"])
        self.assertAlmostEqual(rounds[-1]["radius_l_cm"], 29.1, delta=0.5)
        self.assertAlmostEqual(
            result["checks"]["radius"]["odometry_factor"]["FR"], 1.01, delta=0.005)

    def test_a_bad_arc_the_robot_was_measuring_stops_the_step(self):
        # The second FL180 is the robot's to measure, and it stops 5° off.
        fw, person, _ = self.make(28.0 * 574, 29.1 * 574, bad={("FL180", 2)})
        with self.assertLogs(level="ERROR") as logs:
            ok, _ = self.run_radius(fw, person, target_cm=29.1)
        self.assertFalse(ok)
        self.assertTrue(any("re-run the step" in m for m in logs.output))

    def test_a_bad_check_arc_keeps_that_kind_on_the_tape(self):
        fw, person, tapes = self.make(28.0 * 574, 29.1 * 574, bad={("RL180", 1)})
        ok, result = self.run_radius(fw, person, target_cm=29.1)
        self.assertTrue(ok)
        self.assertGreater(tapes.count("RL180"), 1)
        self.assertEqual(tapes.count("FR180"), 1)
        self.assertIsNone(result["checks"]["radius"]["odometry_factor"]["RL"])

    def test_tight_is_taped_every_arc(self):
        fw, person, tapes = self.make(28.0 * 574, 29.1 * 574)
        with self.assertLogs(level="WARNING") as logs:
            ok, _ = self.run_radius(fw, person, arc=cal_profile.ARC_TIGHT,
                                    target_cm=29.1)
        self.assertTrue(ok)
        self.assertTrue(any("SAFE only" in m for m in logs.output))
        self.assertGreater(len(tapes), 4)


class WheelCountTests(unittest.TestCase):
    def test_a_normal_safe_arc_is_quiet(self):
        with self.assertNoLogs(level="ERROR"):
            self.assertIsNone(calibrate.log_wheels("FL180", (0, 0), (2900, 5000)))

    def test_a_wheel_that_quit_is_named(self):
        # RL180 that pivoted: left barely turned, right drove on.
        with self.assertLogs(level="ERROR") as logs:
            self.assertEqual(calibrate.log_wheels("RL180", (0, 0), (-300, -4000)),
                             "A (LEFT)")
        self.assertTrue(any("motor's power" in m for m in logs.output))

    def test_arcs_are_logged_only_when_the_firmware_has_enc(self):
        fw = FakeFirmware()
        fw.after_move = lambda tok: setattr(fw, "enc", [fw.enc[0] + 300,
                                                        fw.enc[1] + 4000])
        fw.read_enc = mock.Mock(wraps=fw.read_enc)
        with mock.patch.object(calibrate, "_wheel_counts", True), \
                self.assertLogs(level="ERROR"):
            calibrate.send_move(fw, "FL180")
        self.assertEqual(fw.read_enc.call_count, 2)
        with mock.patch.object(calibrate, "_wheel_counts", False):
            calibrate.send_move(fw, "FL180")
        self.assertEqual(fw.read_enc.call_count, 2)        # older firmware: none


class EncoderReverseTests(unittest.TestCase):
    def run_check(self, counts_a, counts_b):
        fw = FakeFirmware()
        turns = iter([(counts_a, 0), (0, counts_b)])

        def person(_prompt):
            da, db = next(turns)
            fw.enc = [fw.enc[0] + da, fw.enc[1] + db]
            return ""
        with mock.patch("builtins.input", side_effect=person):
            return calibrate.encoder_check(fw, turns=5, reverse=True)

    def test_backwards_counts_pass(self):
        self.assertTrue(self.run_check(-7800, -7795))

    def test_dropping_counts_backwards_fails(self):
        with self.assertLogs(level="ERROR") as logs:
            self.assertFalse(self.run_check(-7800, -6900))
        self.assertTrue(any("dropping counts" in m for m in logs.output))

    def test_forward_counts_in_a_reverse_test_are_named(self):
        with self.assertLogs(level="ERROR") as logs:
            self.assertFalse(self.run_check(7800, -7800))
        self.assertTrue(any("ENCODER_A_INVERT" in m for m in logs.output))


class SecantRadiusTests(unittest.TestCase):
    def test_one_point_is_the_geometry(self):
        self.assertAlmostEqual(calibrate.steer_for_radius([(574, 30.0)], 29.1),
                               574 * 30.0 / 29.1)

    def test_two_points_follow_the_measured_response(self):
        # 0.02 cm per µs - a tenth of what the geometry would predict.
        pts = [(574, 27.0), (533, 27.82)]
        self.assertAlmostEqual(calibrate.steer_for_radius(pts, 29.1), 469.0, places=0)

    def test_a_backwards_slope_is_noise_and_falls_back(self):
        pts = [(574, 27.0), (533, 26.9)]       # less steering, TIGHTER? no.
        self.assertAlmostEqual(calibrate.steer_for_radius(pts, 29.1),
                               533 * 26.9 / 29.1)

    def test_a_weakly_responding_side_still_settles(self):
        # Left 2.1 cm too tight and barely responding (scrubbing rear wheels):
        # stepping on the geometry alone ends 4 rounds still outside ±0.5.
        fw = FakeFirmware()

        def moved(tok):
            fw.last = tok
            deg = int(tok[2:]) * 10
            fw.hdg = [0, 0, deg, -deg if tok[:2] in ("FR", "RL") else deg, 0]
        fw.after_move = moved

        def radius(side):
            s = fw.cal[side]
            return 27.0 + 0.02 * (574 - s) if side == "SL" else 29.1

        def person(prompt):
            if "distance" in prompt.lower():
                half = math.radians(int(fw.last[2:]) / 2.0)
                return f"{radius('S' + fw.last[1]) * 2 * math.sin(half):.2f}"
            return ""

        result = {"checks": {}}
        with mock.patch("builtins.input", side_effect=person):
            self.assertTrue(calibrate.step_radius(fw, result))
        rounds = result["checks"]["radius"]["rounds"]
        self.assertAlmostEqual(rounds[-1]["radius_l_cm"], 29.1, delta=0.5)
        self.assertLessEqual(len(rounds), 3)


class BrakeStepTests(unittest.TestCase):
    def make(self, raw_x10, proto=6):
        fw = FakeFirmware(proto=proto)
        fw.cal["D"] = 6360                     # an old prior, far from the floor
        it = iter(raw_x10)

        def moved(tok):
            # Learning arcs report a raw measurement; checks report none.
            fw.last_decel = next(it, 0) if fw.learn else 0
            fw.hdg = [0, 0, 900, 902, 0]
        fw.after_move = moved
        return fw

    def test_averages_raw_arcs_and_fixes_lag(self):
        fw = self.make([5000, 5200, 4800, 5000])
        result = {"checks": {}}
        with mock.patch("builtins.input", return_value=""):
            self.assertTrue(calibrate.step_brakes(fw, result))
        self.assertEqual(fw.cal["D"], 5000)    # none of the 6360 prior left
        self.assertEqual(fw.cal["L"], 0)
        arcs = [t for t in fw.sent if t[:2] in ("FR", "RR", "FL", "RL")]
        self.assertEqual(len(arcs), len(calibrate.BRAKE_LEARN_SEQ)
                         + len(calibrate.BRAKE_CHECK_SEQ))
        self.assertEqual(fw.learn, 0)

    def test_rejected_arcs_fail_the_step(self):
        fw = self.make([5000, 0, 0, 5000])
        with mock.patch("builtins.input", return_value=""), \
                self.assertLogs(level="ERROR"):
            self.assertFalse(calibrate.step_brakes(fw, {"checks": {}}))
        self.assertEqual(fw.cal["D"], 6360)

    def test_older_firmware_averages_the_filtered_value(self):
        fw = self.make([], proto=5)
        with mock.patch("builtins.input", return_value=""), \
                self.assertLogs(level="WARNING"):
            self.assertTrue(calibrate.step_brakes(fw, {"checks": {}}))
        arcs = [t for t in fw.sent if t[:2] in ("FR", "RR", "FL", "RL")]
        self.assertEqual(len(arcs), len(calibrate.BRAKE_LEARN_SEQ_OLD)
                         + len(calibrate.BRAKE_CHECK_SEQ))


class GyroHealthTests(ProfileStoreMixin, unittest.TestCase):
    def test_encoder_fallback_is_refused(self):
        fw = FakeFirmware()
        fw.imu_ready = 0
        with self.assertLogs(level="ERROR") as logs:
            self.assertFalse(calibrate.gyro_ok(fw))
        self.assertTrue(any("ENCODER heading" in m for m in logs.output))

    def test_step_measured_after_a_dropout_is_not_saved(self):
        fw = FakeFirmware()

        def moved(tok):
            fw.hdg = [3600, 3600, 900, 900, 0]
            fw.imu_ready = 0                   # gyro went quiet mid-step
        fw.after_move = moved
        with mock.patch("builtins.input", side_effect=["", "3.6", "", "3.6"]), \
                self.assertLogs(level="ERROR"):
            self.assertFalse(calibrate.run_one_step(fw, "arena", "gyro"))
        self.assertFalse(os.path.exists(cal_profile.profile_path("arena")))


if __name__ == "__main__":
    unittest.main(verbosity=2)
