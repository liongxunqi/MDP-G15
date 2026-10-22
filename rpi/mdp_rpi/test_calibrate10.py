"""
test_calibrate10.py — offline checks for the protocol 10 calibration table

    python3 -m unittest test_calibrate10 -v      # from rpi/mdp_rpi/

No robot. SimRobot answers the wire like the protocol 10 firmware (!K / ?K /
?MV / V tokens) and drives a toy robot with KNOWN faults - one wheel a bit
bigger, motors with different lines, a different trim each way, each arc its
own braking and radius. The tests check that calibrate10 finds those faults
from the move reports alone and that what it applies converges.
"""

import math
import os
import re
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

import calibrate10 as c10  # noqa: E402
from communications import cal_profile  # noqa: E402
from communications.stm import k_field_ok, validate_token  # noqa: E402
from test_calibration import FakeFirmware  # noqa: E402

MMPC = 65.9 * math.pi / 1560.0


class SimRobot(FakeFirmware):
    """Protocol 10 on the wire, a robot with known faults underneath."""

    W_TRUE = 1.0040                       # B ground per count / A
    TRIM_TRUE = {"F": lambda rpm: 30 + 0.05 * (rpm - 100), "R": lambda rpm: -12.0}
    COAST_TRUE = {"F": 12.0, "R": 9.0}    # mm, by odometry
    SCALE_TRUE = {"F": 1.02, "R": 1.03}   # true / odometry
    MOTOR = {("a", "f"): (620, 350), ("a", "r"): (640, 340),   # (deadband, ff rpm)
             ("b", "f"): (580, 390), ("b", "r"): (590, 380)}
    DECEL_TRUE = {"FR": 520.0, "FL": 480.0, "RR": 560.0, "RL": 500.0}
    LAG_TRUE = 0.004
    RADIUS_OFF = {"FR": 0.0, "FL": -2.0, "RR": 0.3, "RL": -1.8}
    SCRUB = 1.06                          # odometry chord / true chord

    def __init__(self):
        super().__init__(proto=10)
        self.ktab = {}                     # move -> anchor -> {field: v}
        self.kg = {}
        self.seq = 0
        self.mv = None
        self.true_last = None

    # -- table, as caltab.c --------------------------------------------------
    def _k_set(self, up):
        if up == "!KCLR":
            self.ktab, self.kg = {}, {}
            return True
        m = re.match(r"^!KG\.([A-Z]+)=(-?\d+)$", up)
        if m:
            f, v = m.group(1).lower(), int(m.group(2))
            if not k_field_ok(None, f, v)[0]:
                return False
            self.kg[f] = v
            return True
        m = re.match(r"^!K(FR|FL|RR|RL|F|R)([0-2])\.([A-Z]+)=(-?\d+)$", up)
        if not m:
            return False
        move, a, f, v = m.group(1), int(m.group(2)), m.group(3).lower(), int(m.group(4))
        if not k_field_ok(move, f, v)[0]:
            return False
        anchors = self.ktab.setdefault(move, {})
        if f == "rpm":
            if v == 0:
                anchors.pop(a, None)
                return True
            if any(b != a and x.get("rpm") == v for b, x in anchors.items()):
                return False
            anchors.setdefault(a, {})["rpm"] = v
            return True
        if "rpm" not in anchors.get(a, {}):
            return False
        anchors[a][f] = v
        return True

    def _tab(self):
        return {m: [dict(x) for x in a.values()] for m, a in self.ktab.items()}

    def _look(self, move, rpm, field):
        return c10.effective(self._tab(), move, rpm, field)

    # -- wire ----------------------------------------------------------------
    def send_line(self, tokens):
        tok = tokens[0] if isinstance(tokens, list) else tokens
        up = tok.upper()
        if up.startswith("!K"):
            self.sent.append(tok)
            self._reply = "OK" if self._k_set(up) else "RESEND"
            return True
        m = re.match(r"^(FR|FL|RR|RL|F|R)(\d+)(?:V(\d+))?$", up)
        if m:
            self.sent.append(tok)
            self._drive(m.group(1), int(m.group(2)), int(m.group(3) or 0) * 10)
            self._reply = "OK"
            return True
        return super().send_line(tokens)

    def query(self, command, timeout=2.0):
        tag = command.strip().upper().lstrip("?")
        if tag == "MV":
            if self.mv is None:
                return "MV," + ",".join(["0"] * 26)
            return "MV," + ",".join(str(int(round(self.mv[k] * 10)))
                                    if k in c10.STM._MV_X10 else str(int(self.mv[k]))
                                    for k in c10.STM.MV_FIELDS)
        if tag == "KG":
            return "KG," + ",".join(str(self.kg.get(f, 0))
                                    for f in c10.CAL_TABLE_GLOBALS)
        m = re.match(r"^K(FR|FL|RR|RL|F|R)([0-2])$", tag)
        if m:
            move, a = m.group(1), int(m.group(2))
            row = self.ktab.get(move, {}).get(a, {})
            allowed = (c10.cal_profile.CAL_TABLE_MOVES and
                       ("rpm", "steer", "decel", "lag", "radius", "boost")
                       if move in c10.ARC_MOVES else ("rpm", "trim", "coast", "dist"))
            out = []
            for f in ("rpm", "trim", "coast", "dist", "steer", "decel", "lag",
                      "radius", "boost"):
                if f not in allowed:
                    out.append(".")
                else:
                    out.append(str(row[f]) if f in row and "rpm" in row else "-")
            return f"K,{move.lower()},{a}," + ",".join(out)
        return super().query(command, timeout)

    # -- the toy robot ----------------------------------------------------------
    def _drive(self, move, amount, rpm):
        self.seq += 1
        mv = {k: 0 for k in c10.STM.MV_FIELDS}
        mv["seq"] = self.seq
        mv["move"] = c10.CAL_TABLE_MOVES.index(move) + 1
        if move in ("F", "R"):
            rpm = rpm or 100
            w_force = (self.kg.get("wheel") or 10000) / 10000.0
            trim = self._look(move, rpm, "trim")
            trim = trim if trim is not None else self.cal["T"]
            coast = self._look(move, rpm, "coast")
            coast = coast / 10.0 if coast is not None else 8.0
            dist = self._look(move, rpm, "dist")
            dist = dist / 10000.0 if dist is not None else 1.0
            target = amount * 10.0 / dist - coast
            odo = target + self.COAST_TRUE[move]
            enc_a = odo / MMPC
            corr = (self.TRIM_TRUE[move](rpm) - trim) + (self.W_TRUE - w_force) * 2000
            sign = 1 if move == "F" else -1
            d = "f" if move == "F" else "r"
            mv.update(rpm=rpm, n=100, enc_a=sign * int(enc_a),
                      enc_b=sign * int(enc_a / self.W_TRUE),
                      rpm_a=rpm, rpm_b=rpm,
                      duty_a=self._duty("a", d, rpm), duty_b=self._duty("b", d, rpm),
                      corr=corr, max_err=0.8, dist_brake=target, dist=odo,
                      y=0.3 * corr, x=sign * odo, servo=1474 + int(trim),
                      trim=trim, target=target)
            self.true_last = odo * self.SCALE_TRUE[move]
        else:
            rpm = rpm or 150
            right = move in ("FR", "RR")
            steer = self._look(move, rpm, "steer")
            if steer is None:
                steer = self.cal["SR"] if right else self.cal["SL"]
            decel = self._look(move, rpm, "decel")
            decel = decel / 10.0 if decel is not None else self.cal["D"] / 10.0
            lag = self._look(move, rpm, "lag")
            lag = lag / 10000.0 if lag is not None else self.cal["L"] / 10000.0
            w = min(rpm, 100) * 0.9
            coast = w * w / (2 * self.DECEL_TRUE[move]) + w * self.LAG_TRUE
            lead = w * w / (2 * decel) + w * lag
            turned = amount + (coast - lead)
            radius = (29.1 + (600 - steer) * 0.04 + 0.01 * (rpm - 100)
                      + self.RADIUS_OFF[move])
            chord = 2 * radius * math.sin(math.radians(turned) / 2)
            odo_chord = chord * self.SCRUB * 10
            mv.update(rpm=rpm, n=80, aim=amount, turned=turned, brake_rate=w,
                      brake_ang=turned - coast, x=odo_chord / math.sqrt(2),
                      y=odo_chord / math.sqrt(2),
                      servo=1474 + int(steer) if right else 1474 - int(steer),
                      rpm_a=rpm, rpm_b=rpm, duty_a=700, duty_b=700, target=amount)
            self.true_last = chord
            self.true_radius = radius
        self.mv = mv

    def _duty(self, wheel, d, rpm):
        db, ff = self.MOTOR[(wheel, d)]
        return int(round(db + rpm / ff * (1000 - db)))


class Store(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._patch = mock.patch.multiple(
            cal_profile, PROFILE_DIR=os.path.join(self._tmp.name, "cal"),
            HEADING_LOG_DIR=os.path.join(self._tmp.name, "hdg"))
        self._patch.start()
        # No human: every prompt takes its default (apply all, run again).
        self._quiet = [mock.patch.object(c10.c9, "pause", lambda msg: None),
                       mock.patch.object(c10.c9, "ask", lambda prompt: ""),
                       mock.patch("builtins.print", lambda *a, **k: None)]
        for p in self._quiet:
            p.start()

    def tearDown(self):
        for p in self._quiet:
            p.stop()
        self._patch.stop()
        self._tmp.cleanup()

    def profile(self):
        return {"decel_x10": 6000, "lag_ms_x10": 0, "trim_us": 10,
                "gyro_x10000": 10000, "steer_left_us": 574, "steer_right_us": 574,
                "arc_profile": 0, "checks": {}, "table": {}, "note": ""}


class TokenTests(unittest.TestCase):
    def test_v_suffix_and_table_tokens(self):
        for tok in ("F100V20", "R50V4", "FR90V12", "RL90V17", "!KCLR", "?MV",
                    "?KG", "?KRL2", "!KFR1.STEER=626", "!KF0.TRIM=-80",
                    "!KG.FFBR=0", "!KR2.RPM=0"):
            self.assertTrue(validate_token(tok)[0], tok)
        for tok in ("F100V26", "FR90V18", "FU20V5", "F0V5", "!KFL0.STEER=600",
                    "!KF0.STEER=500", "!KFR3.RPM=100", "?KFR3", "!KG.WHEEL=5",
                    "!KF0.RPM=30"):
            self.assertFalse(validate_token(tok)[0], tok)


class TableModelTests(unittest.TestCase):
    def test_effective_matches_the_firmware_interpolation(self):
        # Same numbers as stm/tests/test_caltab.c.
        tab = {}
        c10.set_value(tab, "FR", 150, "steer", 600)
        c10.set_value(tab, "FR", 80, "steer", 560)
        self.assertAlmostEqual(c10.effective(tab, "FR", 115, "steer"), 580)
        self.assertAlmostEqual(c10.effective(tab, "FR", 40, "steer"), 560)
        self.assertAlmostEqual(c10.effective(tab, "FR", 250, "steer"), 600)
        c10.set_value(tab, "FR", 120, "steer", 590)
        self.assertAlmostEqual(c10.effective(tab, "FR", 100, "steer"), 575)
        self.assertAlmostEqual(c10.effective(tab, "FR", 135, "steer"), 595)

    def test_set_refuses_what_the_firmware_would(self):
        tab = {}
        with self.assertRaises(ValueError):
            c10.set_value(tab, "FL", 150, "steer", 600)      # past the left servo
        with self.assertRaises(ValueError):
            c10.set_value(tab, "F", 100, "steer", 500)       # arc field on F
        for rpm in (60, 100, 150):
            c10.set_value(tab, "FR", rpm, "decel", 5000)
        with self.assertRaises(ValueError):
            c10.set_value(tab, "FR", 170, "decel", 5000)     # fourth anchor
        c10.set_value(tab, "FR", 60, "rpm", 0)               # drop one
        c10.set_value(tab, "FR", 170, "decel", 5000)
        self.assertEqual([a["rpm"] for a in tab["FR"]], [100, 150, 170])

    def test_keys(self):
        self.assertEqual(c10.parse_key("FR@150.steer"), ("FR", 150, "steer"))
        self.assertEqual(c10.parse_key("g.wheel"), ("G", None, "wheel"))
        with self.assertRaises(ValueError):
            c10.parse_key("FR.steer")

    def test_table_entries_send_rpm_first(self):
        tab = {}
        c10.set_value(tab, "F", 100, "trim", 30)
        c10.set_value(tab, "G", None, "wheel", 10040)
        e = cal_profile.table_entries(tab)
        self.assertEqual(e[0], (None, 0, "wheel", 10040))
        self.assertEqual(e[1], ("F", 0, "rpm", 100))
        self.assertEqual(e[2], ("F", 0, "trim", 30))

    def test_speeds(self):
        self.assertEqual(c10.parse_speeds("80,150", arc=True), [80, 150])
        with self.assertRaises(ValueError):
            c10.parse_speeds("85", arc=False)
        with self.assertRaises(ValueError):
            c10.parse_speeds("200", arc=True)
        with self.assertRaises(ValueError):
            c10.parse_speeds("60,100,150,170", arc=True)


class PushTests(Store):
    def test_table_restores_and_reads_back(self):
        fw = SimRobot()
        p = self.profile()
        c10.set_value(p["table"], "FR", 150, "steer", 610)
        c10.set_value(p["table"], "R", 100, "trim", -12)
        c10.set_value(p["table"], "G", None, "ffbr", 380)
        self.assertTrue(cal_profile.push(fw, p, 10))
        self.assertEqual(fw.ktab["FR"][0], {"rpm": 150, "steer": 610})
        self.assertEqual(fw.kg["ffbr"], 380)
        self.assertIn("!KCLR", fw.sent)

    def test_empty_table_still_clears(self):
        fw = SimRobot()
        fw.ktab = {"F": {0: {"rpm": 100, "trim": 5}}}
        self.assertTrue(cal_profile.push(fw, self.profile(), 10))
        self.assertEqual(fw.ktab, {})

    def test_table_on_protocol_9_is_refused(self):
        fw = FakeFirmware(proto=9)
        p = self.profile()
        c10.set_value(p["table"], "F", 100, "trim", 30)
        self.assertFalse(cal_profile.push(fw, p, 9))

    def test_old_profile_on_protocol_10(self):
        fw = SimRobot()
        p = self.profile()
        del p["table"]
        self.assertTrue(cal_profile.push(fw, p, 10))


class StraightTests(Store):
    def test_finds_the_robots_faults_and_converges(self):
        fw = SimRobot()
        p = self.profile()
        cal_profile.save("t", p)
        self.assertTrue(cal_profile.push(fw, p, 10))
        log = c10.RunLog("t")
        self.assertTrue(c10.step_straights(fw, p, "t", log, [100, 180], 2, 100,
                                           tape=False))
        tab = p["table"]
        # wheel ratio exactly from the counts
        self.assertAlmostEqual(tab["G"]["wheel"], 10040, delta=2)
        # each motor's own line
        # The firmware has one deadband per wheel for both directions, so the
        # line is judged by the duty it predicts, not by its parameters.
        for (wheel, d), (db, ff) in SimRobot.MOTOR.items():
            gdb, gff = tab["G"][f"db{wheel}"], tab["G"][f"ff{wheel}{d}"]
            for rpm in (100, 180):
                want = db + rpm / ff * (1000 - db)
                got = gdb + rpm / gff * (1000 - gdb)
                self.assertAlmostEqual(got, want, delta=6, msg=f"{wheel}{d}@{rpm}")
        # trim per direction and speed, near the truth after the rounds
        for move in ("F", "R"):
            for rpm in (100, 180):
                got = c10.get_value(tab, move, rpm, "trim")
                self.assertAlmostEqual(got, SimRobot.TRIM_TRUE[move](rpm), delta=3,
                                       msg=f"{move}@{rpm}")
                # within COAST_OK_MM of the truth, table or protocol 9 default
                coast = c10.get_value(tab, move, rpm, "coast") or c10.P9_COAST_MM * 10
                self.assertAlmostEqual(coast, SimRobot.COAST_TRUE[move] * 10,
                                       delta=c10.COAST_OK_MM * 10)
        # what the robot holds is what the profile says
        self.assertEqual(fw.kg["wheel"], tab["G"]["wheel"])
        saved = cal_profile.load("t")
        self.assertEqual(saved["table"], tab)
        log.close()
        with open(log.path) as fh:
            self.assertGreater(len(fh.readlines()), 8)

    def test_tape_sets_the_distance_scale(self):
        fw = SimRobot()
        p = self.profile()
        cal_profile.save("t", p)
        cal_profile.push(fw, p, 10)
        with mock.patch.object(c10.c9, "ask_float",
                               lambda prompt: round(fw.true_last / 10.0, 1)):
            self.assertTrue(c10.step_straights(fw, p, "t", c10.RunLog("t"),
                                               [100], 2, 100, tape=True))
        self.assertAlmostEqual(c10.get_value(p["table"], "F", 100, "dist"),
                               10200, delta=15)
        self.assertAlmostEqual(c10.get_value(p["table"], "R", 100, "dist"),
                               10300, delta=15)

    def test_nothing_is_applied_without_approval(self):
        fw = SimRobot()
        p = self.profile()
        cal_profile.save("t", p)
        cal_profile.push(fw, p, 10)
        with mock.patch.object(c10.c9, "ask", lambda prompt: "n"):
            self.assertTrue(c10.step_straights(fw, p, "t", c10.RunLog("t"),
                                               [100], 1, 100, tape=False))
        self.assertEqual(p["table"], {})
        self.assertEqual(fw.ktab, {})

    def test_only_the_chosen_numbers(self):
        rows, props, _ = c10.analyse_straights(self._batch(), {}, self.profile())
        answers = iter(["2"])
        got = c10.approve(props, ask=lambda prompt: next(answers))
        self.assertEqual(got, [props[1]])
        answers = iter(["e 1 10000", ""])
        got = c10.approve(props, ask=lambda prompt: next(answers))
        self.assertEqual(got[0].new, 10000)

    def _batch(self):
        fw = SimRobot()
        out = []
        for move in ("F", "R"):
            for rpm in (100, 180):
                fw._drive(move, 100, rpm)
                mv = fw.read_mv()
                mv["tape_cm"] = None
                out.append(mv)
        return out

    def test_weaker_motor_is_named(self):
        _, _, notes = c10.analyse_straights(self._batch(), {}, self.profile())
        self.assertTrue(any("A (left)" in n for n in notes), notes)


class ArcTests(Store):
    def test_braking_and_steering_per_arc_converge(self):
        fw = SimRobot()
        p = self.profile()
        cal_profile.save("t", p)
        cal_profile.push(fw, p, 10)
        p["checks"]["arc_chord_factor"] = {m: 1 / SimRobot.SCRUB for m in c10.ARC_MOVES}
        self.assertTrue(c10.step_arcs(fw, p, "t", c10.RunLog("t"), list(c10.ARC_MOVES),
                                      [60, 150], 2, 90, tape=False, target="29.1"))
        tab = p["table"]
        for move in c10.ARC_MOVES:
            for rpm in (60, 150):
                fw._drive(move, 90, rpm)
                stop = fw.mv["turned"] - 90
                self.assertLess(abs(stop), 0.3, f"{move}@{rpm} stops {stop:+.2f}")
                self.assertAlmostEqual(fw.true_radius, 29.1, delta=0.6,
                                       msg=f"{move}@{rpm}")
            self.assertIsNotNone(c10.get_value(tab, move, 150, "steer"))

    def test_brake_fit_separates_lag_across_speeds(self):
        a, lag = 500.0, 0.006
        samples = [(w, w * w / (2 * a) + w * lag) for w in (40, 60, 90, 90, 60)]
        decel, got_lag, joint = c10.fit_brakes(samples, 0.0)
        self.assertTrue(joint)
        self.assertAlmostEqual(decel, a, delta=1)
        self.assertAlmostEqual(got_lag, lag, delta=0.0005)
        # one speed: lag held, decel absorbs it
        decel, got_lag, joint = c10.fit_brakes([(90, 9.0), (90, 9.2)], 0.0)
        self.assertFalse(joint)
        self.assertEqual(got_lag, 0.0)

    def test_tapered_arcs_are_not_refitted(self):
        # MOTION_ARC_TAPER: braking from ~25 rpm coasts a fraction of a degree.
        fw = SimRobot()
        runs = []
        for _ in range(3):
            fw._drive("FR", 90, 150)
            mv = fw.read_mv()
            mv["brake_rate"], mv["brake_ang"] = 22.0, abs(mv["turned"]) - 0.2
            runs.append(mv)
        _, props, notes = c10.analyse_arcs(runs, {}, self.profile(), "keep", {}, {})
        self.assertFalse([p for p in props if p.key[2] in ("decel", "lag")])
        self.assertTrue(any("not refitted" in n for n in notes), notes)

    def test_tape_corrects_the_odometry_chord(self):
        fw = SimRobot()
        p = self.profile()
        cal_profile.save("t", p)
        cal_profile.push(fw, p, 10)
        with mock.patch.object(c10.c9, "ask_float",
                               lambda prompt: round(fw.true_last, 2)), \
                mock.patch.object(c10.c9, "ask", lambda prompt: "n"):
            self.assertTrue(c10.step_arcs(fw, p, "t", c10.RunLog("t"), ["FR", "RR"],
                                          [150], 1, 90, tape=True, target="keep"))
        k = p["checks"]["arc_chord_factor"]["FR"]
        self.assertAlmostEqual(k, 1 / SimRobot.SCRUB, delta=0.002)


class SetTests(Store):
    def test_set_by_hand(self):
        p = self.profile()
        cal_profile.save("t", p)
        self.assertTrue(c10.cmd_set(p, "t", ["FR@150.steer=610", "G.wheel=10030",
                                             "F@100.trim=-5"]))
        saved = cal_profile.load("t")
        self.assertEqual(c10.get_value(saved["table"], "FR", 150, "steer"), 610)
        self.assertEqual(saved["table"]["G"]["wheel"], 10030)
        self.assertFalse(c10.cmd_set(p, "t", ["FL@150.steer=600"]))
        self.assertTrue(c10.cmd_set(p, "t", ["FR@150.rpm=0"]))
        self.assertEqual(cal_profile.load("t")["table"]["FR"], [])


if __name__ == "__main__":
    unittest.main()
