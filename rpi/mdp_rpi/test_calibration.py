"""
test_calibration.py — offline checks for frozen calibration (protocol 4)

    python3 -m pytest -q test_calibration.py      # from rpi/mdp_rpi/

No robot needed. FakeFirmware answers the wire exactly the way rpilink.c does
for the commands these scripts use, so what is being tested is the RPi side:
token validation, the task startup sequence, the profile store, and the
arithmetic calibrate.py does with what you measure on the floor.
"""

import csv
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
    """A protocol 5 robot (or older, via proto=), as far as the wire can tell."""

    LIMITS = {"D": (501, 29999), "L": (0, 2500), "T": (-80, 80),
              "G": (9000, 11000), "SL": (300, 574), "SR": (300, 626)}

    def __init__(self, proto=5):
        self.proto = proto
        self.cal = {"D": 6360, "L": 30, "T": 0, "G": 10000, "SL": 574, "SR": 574}
        self.learn = 0
        self.profile = 0
        self.zeroed = 0
        self.sent = []
        self._reply = None
        self.hdg = [0, 0, 0, 0, 0]          # what ?HDG answers, x10 fields
        self.after_move = None              # hook: token -> None

    # Movement path ----------------------------------------------------------
    def send_line(self, tokens):
        tok = tokens[0] if isinstance(tokens, list) else tokens
        self.sent.append(tok)
        up = tok.upper()
        if up.startswith("!LEARN"):
            self.learn = int(up[6:])
            self._reply = "OK"
        elif up.startswith("!PROF"):
            # Locked to TIGHT: anything else is refused, as in rpilink.c.
            self._reply = "OK" if up == "!PROF0" else "RESEND"
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
            return reply
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


class RadiusStepTests(unittest.TestCase):
    """
    A robot whose radius is K / deflection per side - wider the less it
    steers, like the real linkage over this range. The 'person' answers each
    chord prompt with what a tape would read for the side just driven.
    """

    def make(self, k_left_cm, k_right_cm):
        fw = FakeFirmware()
        fw.last = None

        def moved(tok):
            fw.last = tok
            fw.hdg = [0, 0, 900, -900 if tok == "FR90" else 900, 0]
        fw.after_move = moved

        def person(prompt):
            if "distance" in prompt.lower():
                side = "SR" if fw.last == "FR90" else "SL"
                k = k_right_cm if side == "SR" else k_left_cm
                return f"{k / fw.cal[side] * 2 ** 0.5:.2f}"
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

    def test_needs_protocol_5(self):
        fw = FakeFirmware(proto=4)
        with mock.patch("builtins.input", return_value=""):
            self.assertFalse(calibrate.step_radius(fw, {"checks": {}}))


class StopSummaryTests(unittest.TestCase):
    def test_worst_and_per_direction(self):
        out = calibrate.summarise_stops([("FR90", 0.3), ("RR90", -0.2),
                                         ("FR90", 0.5), ("FL90", -0.1)])
        self.assertEqual(out["worst_deg"], 0.5)
        self.assertEqual(out["by_direction"]["FR"], 0.4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
