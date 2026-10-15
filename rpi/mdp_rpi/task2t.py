"""
task2t.py - Task 2, the fastest car, on the TIGHT arc profile.

TIGHT version of task2.py, planned with task2t_park.py (TIGHT's radius and
arc speed) and run on a TIGHT calibration, in the firmware's fast mode.

    python3 task2t.py                 # on the robot
    python3 task2t_sim.py 90 80 50    # the same code against a simulated STM

The route, in the world frame of task2t_park.py (origin the bay mouth, +x out):

  1. Approach obstacle 1, recognise its arrow, then bypass it with one
     continuous two-arc lane change on the requested side.
  2. Turn inward immediately from the measured parallel pose, stop at a
     collision-checked angled view of obstacle 2, and recognise arrow 2.
  3. Turn directly in arrow 2's direction. LR/RL use one inward arc whenever
     it fits; they never reverse to manufacture a turn pocket. LL/RR retain
     one bounded straight recovery only when the measured camera endpoint
     otherwise has no collision-checked forward continuation.
  4. Use the side IR to measure obstacle 2's requested edge. If the IR is
     already beyond that edge, three stationary no-face readings start the
     corner immediately; no fixed forward scan is added.
  5. Round obstacle 2 and head home. ParkRunner prefers one fully swept,
     collision-checked arc/diagonal/arc return, then performs one measured
     parking push. The obstacle-1 IR landmark route is only the fallback when
     the direct diagonal does not fit.

Speed: protocol-10 fast mode (!FAST1) runs open straights at 200 rpm;
T2T_FAST=0 turns it off. Checked handoffs and corners are queued continuously
when their complete swept path has the full turn margin.

Calibration: T2T_CAL_PROFILE in .env (falls back to T2_CAL_PROFILE, then
CAL_PROFILE). A SAFE calibration is refused - the plan assumes TIGHT's radius.
"""

import argparse
import logging
import math
import os
import sys
import time
from typing import Callable, Iterator, List, Optional, Sequence, Tuple

import task2t_park as tp
from task2t_park import Course, Pose, Seg

# Body gap kept past obstacle 1 on the way out. The lane is chosen as tight
# as this allows so the ultrasound, at the robot's centreline, still lands on
# obstacle 2's face when it is only 30 cm wide.
OBS1_PASS_GAP_CM = tp._env_f("T2_OBS1_PASS_GAP_CM", 6.0)
# A 6 cm lane keeps a narrow obstacle 2 in the ultrasonic beam, but when
# obstacle 1 is close to the bay that same lane can leave less clearance than
# the measured-turn gate accepts.  Search outward only as far as needed; this
# preserves the established lane on ordinary layouts and gives close layouts
# a collision-checked alternative instead of stopping or reversing.
OBS1_PASS_GAP_MAX_CM = max(
    OBS1_PASS_GAP_CM,
    tp._env_f("T2_OBS1_PASS_GAP_MAX_CM", 10.0))
# Target for the continuous obstacle-1 S-turn.  This remains well inside the
# stated 50 cm side allowance (this is the axle centre, not the body edge).
OBS1_COMPACT_LANE_CM = min(
    32.0, max(20.0, tp._env_f("T2_OBS1_COMPACT_LANE_CM", 31.0)))
# The STM cannot replace a movement primitive in flight.  During obstacle 1's
# approach we therefore run one fast straight, poll the already-running vision
# vote, and use RST as soon as the requested lane change is collision-safe.
# This avoids braking after every short straight while retaining an RPi-side
# reaction to an arrow detected well before the planned photo position.
SENSOR_POLL_S = tp._env_f("T2_SENSOR_POLL_S", 0.02)
# Turns at obstacle 1 and into the photo pose are this sharp.
KICK_DEG = 45
# Prefer the angles exercised by the normal 45-degree command family when
# geometry permits. They expand the search; measured-angle corrections remain
# available because quantising every recovery can remove its only clear path.
CALIBRATED_TURN_ANGLES = (45, 90, 135, 180)
# Photo of obstacle 2: the camera is reliable about 20--30 cm from the face.
# Aim at the measured face centre and stop at 30 cm; after recognition, plan
# only the continuation required by the arrow that was actually observed.
PHOTO_DIST_CM = tp._env_f("T2T_PHOTO_DIST_CM", 30.0)
PHOTO_MIN_CM = tp._env_f("T2T_PHOTO_MIN_CM", 20.0)
# Stage 2 must turn inward as soon as obstacle 1's lane change is complete.
# The required angle is solved from the measured obstacle-2 centre: a distant
# face naturally needs a shallower angle than a close face.  A former fixed
# 30-degree floor forced a long parallel F move before the turn (for example
# F58,FR31,F8), which is the opposite of the required manoeuvre.  Keep only
# the firmware's real minimum arc as the lower bound.
STAGE2_CAMERA_MIN_INWARD_DEG = max(
    float(tp.MIN_ARC_DEG),
    tp._env_f("T2T_CAMERA_MIN_INWARD_DEG", float(tp.MIN_ARC_DEG)))
CAMERA_FROM_AXLE_CM = tp._env_f("T2_CAMERA_FROM_AXLE_CM", tp.FRONT_FROM_AXLE_CM)
OBS1_CAMERA_CHECKPOINT_CM = tp._env_f("T2_OBS1_CAMERA_CHECKPOINT_CM", 45.0)
# Position reserve after the entire first board is behind the forward camera.
STAGE2_REAR_EXCLUSION_CM = max(0.0, tp._env_f("T2_STAGE2_REAR_EXCLUSION_CM", 6.0))
# The physical straight can finish several degrees off its commanded heading.
# Vision and the handoff generator both tolerate this angle; rejecting at five
# degrees caused a complete stop after an otherwise clear camera-gate move.
STAGE2_MAX_HEADING_DEG = min(
    15.0, max(5.0, tp._env_f("T2_STAGE2_MAX_HEADING_DEG", 15.0)))
# Running along obstacle 2's face: the IR this far off it.
FACE_GAP_CM = tp._env_f("T2T_FACE_GAP_CM", 22.0)
# The most an IR reading can say and still be believed as a face. With
# nothing in range the IR does NOT read blank: its background output maps to
# ~30-50 cm (the right unit read 29-48 for 90 cm of empty floor). Anything
# past this is treated as nothing there.
IR_FACE_MAX_CM = tp._env_f("T2T_IR_FACE_MAX_CM", 26.0)
# A checked handoff can finish a few centimetres farther from the front face
# than its nominal endpoint when the calibrated arc under-turns.  Accept a
# stable reading near the pose-predicted gap, but keep a hard ceiling so the
# IR's 30-50 cm empty-floor plateau is not treated as a face indiscriminately.
FRONT_FACE_GAP_TOL_CM = tp._env_f("T2T_FRONT_FACE_GAP_TOL_CM", 8.0)
FRONT_FACE_HARD_MAX_CM = tp._env_f("T2T_FRONT_FACE_HARD_MAX_CM", 34.0)
# Long primitives are vulnerable to the deployed STM's FAIL,SHORT response.
# Splitting preserves the same ideal path while keeping each encoder target
# bounded; the measured handoff checks still run around the resulting groups.
HANDOFF_MAX_STRAIGHT_CM = max(
    20.0, tp._env_f("T2T_HANDOFF_MAX_STRAIGHT_CM", 60.0))
HANDOFF_MAX_ARC_DEG = max(
    90.0, tp._env_f("T2T_HANDOFF_MAX_ARC_DEG", 135.0))
HANDOFF_HEADING_TOL_DEG = tp._env_f("T2T_HANDOFF_HEADING_TOL_DEG", 8.0)
# Stop an opposite-arrow handoff slightly before mathematical parallel. The
# measured floor arcs continue 2--4 degrees past their ideal endpoint; using
# the full nominal 90 consumed the corner pocket and forced a reverse. This is
# a bounded planner preference, not a fixed command: every 84..90 candidate is
# still generated and collision checked from the measured camera pose.
OPPOSITE_HANDOFF_RESERVE_DEG = min(
    6.0, max(0.0, tp._env_f("T2T_OPPOSITE_HANDOFF_RESERVE_DEG", 4.0)))
# Behind obstacle 2, the IR on its back face at most this far off it.
BACK_GAP_MAX_CM = tp._env_f("T2T_BACK_GAP_MAX_CM", 34.0)
# Coming round behind obstacle 2, the IR at most this far past its near end.
# The side IR is 17 cm ahead of the axle. At a measured 5-degree heading
# error, a clean two-arc corner places it about 23.5 cm into the rear face.
# Twenty-four still leaves 6 cm of the brief's narrowest 30 cm obstacle for
# the stationary acquisition and moving edge confirmation.
BACK_START_MAX_CM = tp._env_f("T2T_BACK_START_MAX_CM", 24.0)
# The side IR must begin the edge scan this far inside the guaranteed 30 cm
# face. Two centimetres is enough for the two stationary acquisition reads;
# the logged right-then-left pose lands within millimetres of a 3 cm cutoff.
FACE_ENTRY_MIN_VISIBLE_CM = tp._env_f("T2T_FACE_ENTRY_MIN_VISIBLE_CM", 2.0)
# Obstacle 2's measured centre is taken as odometry drift, up to this much.
Y_RESYNC_MAX_CM = tp._env_f("T2T_Y_RESYNC_MAX_CM", 15.0)
# Heading home past obstacle 2, the body this far off its side.
HOME_GAP_CM = tp._env_f("T2T_HOME_GAP_CM", 15.0)
# Added to the measured half-width before planning round it.
END_MARGIN_CM = tp._env_f("T2T_END_MARGIN_CM", 2.0)
# Measurement uncertainty, not a signed correction to either IR calibration.
IR_FACE_UNCERTAINTY_CM = max(0.0, tp._env_f("T2T_IR_FACE_UNCERTAINTY_CM", 3.0))
# A side edge can be reported several centimetres late by the filtered IR.
# Keep this much of the stated 50 cm side allowance unused while rounding the
# first measured edge; the opposite edge later supplies the accurate width.
EDGE_LIMIT_RESERVE_CM = max(
    IR_FACE_UNCERTAINTY_CM,
    min(15.0, tp._env_f("T2T_EDGE_LIMIT_RESERVE_CM", 4.0)))
# A protocol-10 IR distance is blank both below the calibrated 10 cm range and
# when nothing is present.  The raw ADC distinguishes those cases.  Treat a
# high-raw blank as dangerously close, never as a cleared obstacle edge.
IR_TOO_CLOSE_RAW = int(round(tp._env_f("T2T_IR_TOO_CLOSE_RAW", 2500.0)))
# RST on the deployed firmware can coast about 16 cm even at precision speed.
# Extra edge seeking is bounded and reserves stopping room, not just travel.
FACE_EXTENSION_MAX_CM = 30
FACE_EXTENSION_BATCH_CM = 20
FACE_EXTENSION_STOP_CM = 20.0
CORNER_RECHECK_CM = 8.0
TURN_CLEARANCE_CM = max(tp.CLEARANCE_FLOOR_CM,
                        tp._env_f("T2T_TURN_CLEARANCE_CM", 8.0))
HANDOFF_CLEARANCE_CM = max(tp.CLEARANCE_FLOOR_CM,
                           tp._env_f("T2T_HANDOFF_CLEARANCE_CM", 4.5))
# A route replanned from a measured pose can run as one STM queue when it has
# this much clearance.  The measured route has already had its complete body
# sweep checked; keeping a separate 5 cm queue threshold made the robot stop
# after every primitive even though the selector accepted the route at the
# execution-tolerant 3.9 cm handoff threshold.
HANDOFF_CONTINUOUS_CLEARANCE_CM = max(
    HANDOFF_CLEARANCE_CM,
    tp._env_f("T2T_HANDOFF_CONTINUOUS_CLEARANCE_CM", 5.0))
# Only LL/RR may use this escape, and the search always takes the first
# collision-checked centimetre.  At the minimum 60 cm face gap this is 22 cm
# for a 50 cm obstacle and 30 cm for the widest supported obstacle.  The hard
# cap still prevents the old full camera-route reversal from returning.
SAME_SIDE_RECOVERY_MAX_CM = min(
    30, max(1, int(round(tp._env_f(
        "T2T_SAME_SIDE_RECOVERY_MAX_CM", 30.0)))))
# Obstacle 1 is encountered before its true swept-path error can be measured.
# Keep a larger hardware margin there than the generic close-pass floor; the
# floor run touched the box when the model reported 6.4 cm of clearance.
OBS1_TURN_CLEARANCE_CM = max(
    TURN_CLEARANCE_CM, tp._env_f("T2T_OBS1_TURN_CLEARANCE_CM", 10.0))
# WPOSE and swept-envelope logs are quantised to 0.1 cm.  Keep the 10 cm
# target, but do not reject at execution a route the approach accepted at
# 9.5-9.6 cm because of one sample/rounding step.
OBS1_TURN_TOLERANCE_CM = max(
    0.0, tp._env_f("T2T_OBS1_TURN_TOLERANCE_CM", 0.6))
OBS1_TURN_MIN_CM = max(
    tp.CLEARANCE_CM, OBS1_TURN_CLEARANCE_CM - OBS1_TURN_TOLERANCE_CM)
# If pose/US re-anchoring leaves no route at the preferred obstacle-1 margin,
# permit the best checked lane change down to this still-positive fallback.
# Floor traces touched the box at 6.4 cm, so the default remains materially
# above that observed failure instead of relaxing to the global floor.
OBS1_TURN_FALLBACK_CM = max(
    tp.CLEARANCE_FLOOR_CM,
    tp._env_f("T2T_OBS1_TURN_FALLBACK_CM", 8.0))
# Apply the same bounded execution tolerance used by the rest of the measured
# replanner.  RST coast moved the logged turn from 9.5 cm to 7.9 cm, and a
# strict comparison with the nominal 8 cm target stopped an otherwise checked
# continuation by one quantised pose sample.
OBS1_TURN_FALLBACK_MIN_CM = tp.execution_clearance(
    OBS1_TURN_FALLBACK_CM)
# A floor placement or calibrated short straight can leave the rear axle a few
# centimetres farther inside the bay mouth than the nominal pose.  In that
# case the first S-turn is limited by the side fence, not obstacle 1.  Search
# for the smallest forward lead that restores the preferred margin and queue
# it with the arcs as one continuous line.  This is a bound, not a commanded
# distance: most runs use zero and the measured pose selects the actual lead.
OBS1_FORWARD_LEAD_MAX_CM = min(
    25, max(0, int(round(tp._env_f("T2T_OBS1_FORWARD_LEAD_MAX_CM", 20.0)))))
# This check is made at obstacle 1's final camera/turn standoff.  Reusing the
# 30 cm obstacle-2 lane threshold contradicted the normal 26-29 cm standoff.
OBS1_LIVE_US_MIN_CM = max(
    tp.CLEARANCE_FLOOR_CM,
    tp._env_f("T2T_OBS1_LIVE_US_MIN_CM", 20.0))
FAST = os.getenv("T2T_FAST", "1").strip() != "0"
# Protocol 10 permits 40..250 rpm straights. Use its ceiling on meaningful
# open-path forwards; its ramp automatically limits the peak of shorter moves.
# IR scans, tight turns and the parking approach select precision mode first.
MAX_STRAIGHT_V = max(4, min(25, int(os.getenv("T2T_MAX_STRAIGHT_V", "25"))))
MAX_STRAIGHT_MIN_CM = max(
    30, int(os.getenv("T2T_MAX_STRAIGHT_MIN_CM", "30")))
# Diagram: spacing 60-150 cm; prose: obstacle 2 width at least 30 cm.
# 90 cm is a practice-course cap, NOT a numeric limit stated in either PDF.
# Set the maximum to the actual allowed arena span before using wider layouts.
# Keep 5 cm of measurement tolerance at either end. The old 45 cm lower bound admitted a
# short floor/edge echo as obstacle 2 and shifted its collision model forward
# by more than the robot's remaining clearance.
W2_RANGE = (30.0, tp._env_f("T2_OBS2_MAX_WIDTH_CM", 90.0))
if W2_RANGE[1] < W2_RANGE[0]:
    raise ValueError("Obstacle 2 maximum width must be at least 30 cm")
GAP12_RANGE = (55.0, 155.0)
# The 60 cm brief spacing is obstacle-to-obstacle. It is not a minimum live US
# reading after the robot has turned and its world pose has drifted. A stable
# echo this far ahead permits continued sensing. Route geometry retains the
# measured face, never a farther face invented from the brief's spacing.
LANE_US_MIN_CLEAR_CM = tp._env_f("T2_LANE_US_MIN_CLEAR_CM", 30.0)
# Before obstacle 2's width is measured: plan against its widest case and
# the walls of its narrowest, together.
CONSERVATIVE_W2 = W2_RANGE[1]
CONSERVATIVE_WALL_Y = W2_RANGE[0] / 2.0 + tp.WALL_CLEARANCE_CM
# The ultrasound missed obstacle 2 from the obstacle 1 lane, so its half-width
# is at most about the lane's: NARROW_HALF_MAX. The robot then widens to
# NARROW_LANE, clear of any such obstacle and of a wall 50 cm past the
# narrowest, and finds obstacle 2 with the inner IR going by.
NARROW = -1.0
NARROW_HALF_MAX = tp._env_f("T2_NARROW_HALF_MAX_CM", 22.0)

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [T2] %(levelname)s - %(message)s",
                    datefmt="%H:%M:%S")


def _straight(d: float) -> List[Seg]:
    n = int(round(abs(d)))
    if n < 1:
        return []
    return [Seg("F" if d > 0 else "B", n)]


def _other(turn: str) -> str:
    return "R" if turn == "L" else "L"


def kick(dy: float, r: float) -> List[Seg]:
    """
    Shift sideways by dy (world y) from heading +x and end on heading +x:
    KICK_DEG out, a diagonal, KICK_DEG back. A shift too small for two full
    KICK_DEG arcs falls back to two shallower ones.
    """
    if abs(dy) < 0.5:
        return []
    k = math.radians(KICK_DEG)
    arcs = 2 * r * (1 - math.cos(k))
    if abs(dy) < arcs:
        return tp.lane_change(0.0, dy, r)
    out = tp._turn_toward(0.0, dy)
    return [Seg(out, KICK_DEG)] + _straight((abs(dy) - arcs) / math.sin(k)) \
        + [Seg(_other(out), KICK_DEG)]


def compact_kick(dy: float, r: float) -> List[Seg]:
    """One continuous S-turn with no straight or reverse between its arcs.

    Two opposite arcs of ``angle`` shift the axle by
    ``2*r*(1-cos(angle))`` and restore the original heading.  This is used
    immediately after image one: the older fixed 45-degree pair needed an
    F5--F11 spacer and sometimes selected a preliminary reverse merely to
    recover the missing lateral displacement.
    """
    if abs(dy) < 0.5:
        return []
    ratio = 1.0 - abs(dy) / (2.0 * r)
    if not -1.0 <= ratio <= 1.0:
        return []
    angle = int(round(math.degrees(math.acos(ratio))))
    if not tp.MIN_ARC_DEG <= angle <= 90:
        return []
    out = tp._turn_toward(0.0, dy)
    return [Seg(out, angle), Seg(_other(out), angle)]


def _ask_arrow(input_fn: Callable[[str], str], which: str) -> float:
    while True:
        ans = input_fn(f"{which} arrow - left or right? ").strip().lower()
        if ans in ("l", "left"):
            return 1.0
        if ans in ("r", "right"):
            return -1.0
        print("Type left or right (l / r).")


def _path_clear(p: Pose, segs: Sequence[Seg], r: float, boxes, need: float,
                step: float = 2.0) -> Optional[float]:
    """Closest approach along segs, or None the moment it dips below need -
    the loop planner tries hundreds of candidates and most fail early."""
    best = float("inf")
    for q in tp.run(p, segs, r, step):
        for b in boxes:
            c = tp.clearance(q, b)
            if c < need:
                return None
            best = min(best, c)
    return best


class Task2:

    def __init__(self, stm, input_fn: Callable[[str], str] = input,
                 sleep_fn: Callable[[float], None] = time.sleep,
                 startup: bool = True, wait_for_go: bool = True,
                 early_arrow_fn: Optional[Callable[[], Optional[str]]] = None,
                 lane_reached_fn: Optional[Callable[[], None]] = None,
                 arrow_pending_fn: Optional[Callable[[], bool]] = None):
        self.stm = stm
        self.input_fn = input_fn
        self.sleep_fn = sleep_fn
        self.startup = startup
        self.wait_for_go = wait_for_go
        self.early_arrow_fn = early_arrow_fn
        self.lane_reached_fn = lane_reached_fn
        self.arrow_pending_fn = arrow_pending_fn
        self.frame = tp.WorldFrame(stm)
        self.th = 0.0                 # commanded heading, radians
        self.r = tp.RADIUS_CM
        # YOLO can refine this from obstacle 1's centred arrow.  Both boards
        # share the course axis; zero remains the compatibility/default value.
        self.obs1_y = 0.0
        self.fast = False             # what the firmware is set to
        # Y coordinate of the ultrasonic ray that actually hit obstacle 2.
        # It supplies a measured lower bound on that obstacle's width for the
        # same-side outer-eight handoff. None means obstacle 2 was found by IR.
        self.obs2_us_y: Optional[float] = None
        # A front-US hit proves that obstacle 2 exists, but its projected y is
        # only a useful width bound when the chassis is genuinely parallel.
        # WPOSE heading drift made a 7-degree ray overstate the half-width by
        # about 10 cm in the 07:15 floor run, after which a real IR edge was
        # rejected as "contradicting" that projection.
        self.obs2_us_extent_reliable = True
        # A checked US miss from a farther parallel lane supplies the matching
        # upper half-width bound. Together these bounds avoid treating every
        # medium obstacle as the 90 cm worst case.
        self.obs2_half_max: Optional[float] = None
        self.obs2_near_y: Optional[float] = None
        self.last_face_start_y: Optional[float] = None
        self.last_stopped_pose: Optional[Pose] = None
        self._handoff_cache = None
        self._stage2_view_open = False
        self.obs2_front_max: Optional[float] = None
        self.stage2_centered = False
        self._stage2_camera_positioned = False

    def _obs2_known_half(self, course: Course) -> float:
        """Conservative face half-width established by live sensing."""
        known = W2_RANGE[0] / 2.0
        if self.obs2_us_y is not None:
            known = max(known, abs(self.obs2_us_y - course.obs2_y))
        if self.obs2_near_y is not None:
            known = max(known, abs(self.obs2_near_y - course.obs2_y))
        return min(W2_RANGE[1] / 2.0, known)

    def _obs2_minimum_edge_half(self, course: Course) -> float:
        """Width lower bound strict enough to reject a later measured edge."""
        # A US hit locates the front plane.  Its projected y is useful for
        # choosing a conservative pre-image route, but it is not an edge
        # measurement: the cone can return from a person/background or from
        # another part of the face.  In the 08:52 LR run the y=+32 cm ray was
        # reflected into a supposed symmetric -32 cm minimum edge; the live
        # IR then correctly measured -22.7 cm and this guard aborted the
        # otherwise valid obstacle-2 circuit.  Only a side-IR observation may
        # strengthen the physical 30 cm minimum for this hard consistency
        # check.
        if self.obs2_near_y is None:
            return W2_RANGE[0] / 2.0
        return min(W2_RANGE[1] / 2.0,
                   max(W2_RANGE[0] / 2.0,
                       abs(self.obs2_near_y - course.obs2_y)))

    def _record_obs2_us_hit(self, ray_y: float, heading: float) -> None:
        """Record a hit without turning an oblique projection into width."""
        self.obs2_us_y = ray_y
        self.obs2_us_extent_reliable = (
            abs(math.degrees(math.atan2(math.sin(heading), math.cos(heading))))
            <= 3.0)

    # ── wire ──────────────────────────────────────────────────────────────────

    def _line(self, segs: Sequence) -> bool:
        """Send one line and wait for it. Tracks the commanded heading."""
        tokens = []
        for segment in segs:
            token = segment.token() if isinstance(segment, Seg) else segment
            if (isinstance(segment, Seg) and self.fast
                    and getattr(self.stm, "supports_v_speed", False)
                    and segment.kind == "F"
                    and segment.value >= MAX_STRAIGHT_MIN_CM):
                token += "V%d" % MAX_STRAIGHT_V
            tokens.append(token)
        if not tokens:
            return True
        line = ",".join(tokens)
        if not self.stm.send_line(line):
            return False
        reply = self.stm.wait_reply()
        if reply is None or not reply.startswith("OK"):
            logging.error("STM answered %r to %s - stopping.", reply, line)
            return False
        self._track(segs)
        return True

    def _track(self, segs: Sequence) -> None:
        for s in segs:
            if isinstance(s, Seg):
                self.th += math.radians(s.turn_deg)

    def _set_fast(self, on: bool) -> None:
        """Fast mode, if the run uses it and the firmware has it."""
        on = on and FAST
        if on == self.fast or not hasattr(self.stm, "set_fast"):
            return
        if self.stm.set_fast(on):
            self.fast = on
        else:
            logging.warning("!FAST%d refused - carrying on at %s speed.",
                            int(on), "fast" if self.fast else "normal")

    def _ir(self, side: str) -> Optional[float]:
        fields = self.stm.query_fields("?IR", timeout=0.5)
        if not fields or len(fields) < 2:
            return None
        try:
            v = int(fields[0] if side == "left" else fields[1])
        except ValueError:
            return None
        return None if v == 0xFFFF else float(v)

    def _ir_pose_sample(self, side: str):
        """Read side IR and measured pose atomically when protocol 10 allows.

        Older firmware and laptop fakes retain the established ?IR + ?WPOSE
        fallback.  The returned raw ADC is used only to distinguish an object
        closer than the calibrated IR range from genuinely empty space.
        """
        reader = getattr(self.stm, "read_irw", None)
        if reader is not None and getattr(self.stm, "supports_irw", None) is True:
            sample = reader(timeout=0.5)
            if sample is not None:
                q = self.frame.pose_from_wpose(sample)
                key = "left" if side == "left" else "right"
                cm = sample.get(key + "_cm")
                raw = sample.get(key + "_raw")
                return (None if cm is None else float(cm), raw, q)
        return self._ir(side), None, self.frame.pose()

    def _us(self) -> Optional[float]:
        """True gap from the front bumper, cm (bias removed)."""
        fields = self.stm.query_fields("?US")
        try:
            v = int(fields[0])
        except (TypeError, ValueError, IndexError):
            return None
        return None if v == 0xFFFF else v - tp.US_BIAS_CM

    def _stable_us(self, attempts: int = 4,
                   tolerance: float = 2.0) -> Optional[float]:
        return tp.stable_us(self._us, attempts, tolerance)

    def _pose(self) -> Pose:
        p = self.frame.pose()
        if p is None:
            raise RuntimeError("No ?WPOSE.")
        return Pose(p.x, p.y, self.th)     # the carry-over holds commanded

    def _wait_for_stopped_pose(self, samples: int = 40,
                               interval_s: float = 0.05) -> Optional[Pose]:
        """Return only after two consecutive WPOSE samples are stationary.

        RST starts braking but does not wait for the chassis to stop. Planning
        from the first post-RST sample caused the floor run to gain another
        13 cm before its corner was rechecked. Later fast-mode traces coasted
        for just over one second and became stationary one sample after the
        old 20-sample limit. Keep the early stationary return, but allow up to
        two seconds for the deployed firmware to finish coasting.
        """
        previous = None
        stable = 0
        for _ in range(samples):
            self.sleep_fn(interval_s)
            try:
                current = self._pose()
            except RuntimeError:
                continue
            if previous is not None:
                moved = math.hypot(current.x - previous.x,
                                   current.y - previous.y)
                turned = abs(math.degrees(math.atan2(
                    math.sin(current.th - previous.th),
                    math.cos(current.th - previous.th))))
                if moved <= 0.3 and turned <= 0.5:
                    stable += 1
                    if stable >= 2:
                        logging.info("RST settled at %s.", current)
                        return current
                else:
                    stable = 0
            previous = current
        logging.error("Robot did not produce a stable WPOSE after RST; "
                      "corner movement withheld.")
        return None

    @staticmethod
    def _front(p: Pose) -> float:
        return p.x + tp.FRONT_FROM_AXLE_CM * math.cos(p.th)

    def _check(self, p: Pose, segs, course: Course, fence: bool = False) -> float:
        """Closest approach along segs. The back fence is left out on the way
        out: the robot starts 2 cm off it and only drives away from it. The
        side fences stay in - with obstacle 1 close to the bay, the 45 deg
        turn starts with the tail still in the mouth."""
        boxes = self._course_boxes(course)
        if not fence:
            boxes = [b for b in boxes if b.name != "back fence"]
        return tp.path_clearance(tp.run(p, segs, self.r), boxes)[0]

    def _course_boxes(self, course: Course):
        boxes = course.boxes()
        if self.obs2_front_max is not None:
            # A short US echo may be pose error OR an early surface return.
            # Keep both possible faces occupied until IR identifies the box.
            boxes = [tp.Box(b.x0, max(b.x1, self.obs2_front_max + tp.OBS2_DEPTH_CM),
                            b.y0, b.y1, b.name) if b.name == "obstacle 2" else b
                     for b in boxes]
        return boxes

    @staticmethod
    def _obs1_lanes() -> List[float]:
        """Candidate centreline offsets, narrow/established lane first."""
        start = int(math.ceil(OBS1_PASS_GAP_CM))
        stop = int(math.floor(OBS1_PASS_GAP_MAX_CM))
        gaps = [OBS1_PASS_GAP_CM]
        gaps.extend(float(gap) for gap in range(start, stop + 1)
                    if gap > OBS1_PASS_GAP_CM + 1e-6)
        if gaps[-1] < OBS1_PASS_GAP_MAX_CM - 1e-6:
            gaps.append(OBS1_PASS_GAP_MAX_CM)
        return [tp.OBS1_SIZE_CM / 2 + tp.HALF_WIDTH_CM + gap
                for gap in gaps]

    def _first_clear(self, p: Pose, candidates, course: Course, what: str):
        """
        The first candidate keeping CLEARANCE_CM. Failing that, the best one
        keeping CLEARANCE_FLOOR_CM. An already-close pose is not permission
        to reduce the collision floor for the next movement.
        """
        floor = tp.CLEARANCE_FLOOR_CM
        best, best_c = None, -1.0
        for segs in candidates:
            c = self._check(p, segs, course)
            if c >= tp.execution_clearance(tp.CLEARANCE_CM):
                return segs
            if c > best_c:
                best, best_c = segs, c
        if best is not None and best_c >= floor:
            logging.warning("%s keeps only %.1f cm (want %.0f) - taking it.",
                            what, best_c, tp.CLEARANCE_CM)
            return best
        logging.error("No clear %s from %s (best %.1f cm).", what, p, best_c)
        return None

    # ── outbound ──────────────────────────────────────────────────────────────

    def obstacle1(self) -> Optional[Tuple[float, float]]:
        """Approach and ask. Returns (x1, side) or None."""
        p = self._pose()
        g = self._stable_us()
        if g is None or not math.isfinite(g) or g <= tp.CLEARANCE_FLOOR_CM:
            logging.error("No safe obstacle-1 ultrasound reading at the start: %s.", g)
            return None
        if not (40.0 <= g <= 200.0):
            logging.warning("Obstacle-1 ultrasound %.1f cm is outside the nominal "
                            "course range; using the live reading.", g)
        x1 = self._front(p) + g
        # Recognition is deliberately stationary. Live vision used to abort a
        # moving approach; the deployed RST then coasted far enough that US,
        # IR and WPOSE all described a different pose. Convert the requested
        # camera-to-board distance to the equivalent ultrasonic reading and
        # make one normal, reply-terminated straight move instead.
        sensor_ahead_of_camera = tp.FRONT_FROM_AXLE_CM - CAMERA_FROM_AXLE_CM
        target_us_gap = OBS1_CAMERA_CHECKPOINT_CM - sensor_ahead_of_camera
        if target_us_gap <= 0:
            logging.error("Invalid obstacle-1 camera checkpoint %.1f cm for "
                          "the configured camera/US offsets.",
                          OBS1_CAMERA_CHECKPOINT_CM)
            return None
        remaining = g - target_us_gap
        final_move = _straight(remaining)
        if final_move:
            logging.info("Obstacle-1 camera checkpoint: %s targets %.1f cm "
                         "camera-to-face (US target %.1f cm).",
                         ",".join(seg.token() for seg in final_move),
                         OBS1_CAMERA_CHECKPOINT_CM, target_us_gap)
            if not self._line(final_move):
                return None
        final_pose = self._pose()
        final_gap = self._stable_us()
        if final_gap is None or not math.isfinite(final_gap):
            logging.error("Obstacle-1 ultrasound was unavailable after the camera move.")
            return None
        if final_gap <= tp.CLEARANCE_FLOOR_CM:
            logging.error("Obstacle 1 is only %.1f cm from the front sensor.", final_gap)
            return None
        if final_gap > target_us_gap + 6.0:
            logging.error("Obstacle-1 echo changed after the camera move: "
                          "expected %.1f cm, read %.1f cm; refusing to chase "
                          "a different surface.", target_us_gap, final_gap)
            return None
        if abs(final_gap - target_us_gap) > 6.0:
            logging.warning("Obstacle-1 camera move ended at US %.1f cm instead "
                            "of %.1f cm; re-anchoring the face from the live "
                            "reading and continuing.", final_gap, target_us_gap)
        x1 = self._front(final_pose) + final_gap
        logging.info("Obstacle 1 stable at the %.1f cm camera checkpoint; "
                     "starting recognition now.", OBS1_CAMERA_CHECKPOINT_CM)
        return x1, _ask_arrow(self.input_fn, "Obstacle 1")

    def pass_obstacle1(self, x1: float, side1: float) -> Optional[float]:
        """45 deg onto the lane on side1, ultrasound obstacle 2. Returns its
        face x, NARROW if the ultrasound missed it, or None."""
        p = self._pose()
        course = Course(x1, x1 + tp.OBS1_SIZE_CM + 60.0, CONSERVATIVE_W2,
                        obs1_y=self.obs1_y, wall_y=CONSERVATIVE_WALL_Y,
                        obs2_y=self.obs1_y)
        # Image one is already taken from a clear approach lane.  Use a
        # contiguous pair of arcs from that measured pose: no R19/R20 setup
        # and no F5/F11 between the turns.  Slightly larger equal arcs supply
        # the same lateral shift while restoring the parallel heading.
        segs = None
        best_candidate = None
        best_clearance = -1.0
        compact_lanes = [OBS1_COMPACT_LANE_CM]
        compact_lanes.extend(reversed(self._obs1_lanes()))
        for lane in compact_lanes:
            candidate = compact_kick(side1 * lane - p.y, self.r)
            if not candidate:
                continue
            clearance = self._check(p, candidate, course)
            if clearance > best_clearance:
                best_candidate, best_clearance = candidate, clearance
            if (lane == OBS1_COMPACT_LANE_CM
                    and clearance >= OBS1_TURN_FALLBACK_MIN_CM):
                segs = candidate
                logging.warning(
                    "Using the requested wide continuous lane %s at %.1f cm "
                    "clearance without a reverse or inter-turn straight.",
                    ",".join(s.token() for s in candidate), clearance)
                break
            if clearance >= OBS1_TURN_MIN_CM:
                segs = candidate
                log = (logging.info
                       if clearance >= OBS1_TURN_CLEARANCE_CM
                       else logging.warning)
                log("Continuous obstacle-1 S-turn %s: clearance %.1f cm "
                    "(target %.1f cm, bounded minimum %.1f cm).",
                    ",".join(s.token() for s in candidate), clearance,
                    OBS1_TURN_CLEARANCE_CM, OBS1_TURN_MIN_CM)
                break
        if segs is None and best_clearance >= OBS1_TURN_FALLBACK_MIN_CM:
            segs = best_candidate
            logging.warning("Obstacle-1 lane change has %.1f cm clearance, "
                            "below the preferred %.1f cm but above the %.1f cm "
                            "measured execution fallback; taking the best "
                            "checked route.",
                            best_clearance, OBS1_TURN_MIN_CM,
                            OBS1_TURN_FALLBACK_MIN_CM)
        if segs is None:
            # The camera stop can be slightly inside the nominal bay-mouth
            # pose even when US correctly locates obstacle 1.  Moving the
            # complete S-turn forward clears the side fence while retaining
            # ample front standoff.  Prefer the first lead that reaches the
            # normal obstacle-1 margin; retain the first fallback only if the
            # preferred margin cannot be reached within the bounded search.
            fallback = None
            # Bound the search by the live obstacle-1 standoff, not by a
            # fixed commanded lead.  In the 13:32 floor run F20 physically
            # travelled only 16.5 cm; the measured axle pose therefore
            # needed F13 before the same continuous S-turn could clear the
            # bay fence.  The old 10 cm search cap aborted even though US
            # still left 40.7 cm in front.  Search only the distance that
            # retains OBS1_LIVE_US_MIN_CM, and select the first safe result.
            front_standoff = x1 - self._front(p)
            measured_lead_max = max(
                0, int(math.floor(front_standoff - OBS1_LIVE_US_MIN_CM)))
            lead_max = min(OBS1_FORWARD_LEAD_MAX_CM, measured_lead_max)
            for lead in range(1, lead_max + 1):
                lead_best = None
                lead_clearance = -1.0
                for lane in compact_lanes:
                    turns = compact_kick(side1 * lane - p.y, self.r)
                    if not turns:
                        continue
                    candidate = [Seg("F", lead)] + turns
                    clearance = self._check(p, candidate, course)
                    if clearance > lead_clearance:
                        lead_best, lead_clearance = candidate, clearance
                if (fallback is None
                        and lead_clearance >= OBS1_TURN_FALLBACK_MIN_CM):
                    fallback = (lead_best, lead_clearance, lead)
                if lead_clearance >= OBS1_TURN_MIN_CM:
                    segs, best_clearance = lead_best, lead_clearance
                    logging.info(
                        "Measured bay-mouth pose needs F%d before the "
                        "obstacle-1 S-turn; queued continuously as %s "
                        "(clearance %.1f cm).",
                        lead, ",".join(s.token() for s in segs),
                        best_clearance)
                    break
            if segs is None and fallback is not None:
                segs, best_clearance, lead = fallback
                logging.warning(
                    "Measured bay-mouth pose needs the minimum checked F%d "
                    "lead before the obstacle-1 S-turn; queued continuously "
                    "as %s (clearance %.1f cm).",
                    lead, ",".join(s.token() for s in segs), best_clearance)
        if segs is None:
            logging.error("No obstacle-1 lane change keeps the %.1f cm measured "
                          "fallback (%.1f cm preferred; best %.1f cm; "
                          "sensor-bounded lead searched through F%d).",
                          OBS1_TURN_FALLBACK_MIN_CM, OBS1_TURN_MIN_CM,
                          best_clearance,
                          locals().get("lead_max", 0))
            return None
        if best_clearance < OBS1_TURN_MIN_CM:
            front = self._stable_us()
            side_gap = self._ir("left" if side1 > 0 else "right")
            side_floor = (tp.HALF_WIDTH_CM - tp.IR_SIDE_OFFSET_CM
                          + tp.CLEARANCE_FLOOR_CM + IR_FACE_UNCERTAINTY_CM)
            if (front is None or not math.isfinite(front)
                    or front < OBS1_LIVE_US_MIN_CM
                    or (side_gap is not None and
                        (not math.isfinite(side_gap) or side_gap < side_floor))):
                logging.error("Obstacle-1 fallback lacks live clearance "
                              "(front=%s, turning-side IR=%s).", front, side_gap)
                return None
        logging.info("Obstacle-1 lane change is one continuous command; no "
                     "pre-turn reverse or inter-turn straight.")
        if not self._line(segs):
            return None
        p = self._pose()
        # Recognition may finish as soon as the robot becomes parallel, but
        # that pose is still beside obstacle 1 and its forward ultrasound can
        # report the wrong surface.  Complete only the checked bypass lead to
        # the fixed handoff station before locating/planning obstacle 2.  The
        # arrow remains latched; it must not cancel this safety positioning.
        handoff_x = x1 - self.r
        cruise_cm = int(math.floor(handoff_x - p.x))
        if cruise_cm > 0:
            cruise = _straight(cruise_cm)
            clearance = self._check(p, cruise, course)
            if clearance < tp.CLEARANCE_FLOOR_CM:
                logging.error("The bypass-lane cruise to obstacle 2 keeps only "
                              "%.1f cm clearance; refusing it.", clearance)
                return None
            logging.info("Stage-2 arrow may already be latched; completing "
                         "checked bypass positioning F%d before branching "
                         "(clearance %.1f cm).", cruise_cm, clearance)
            if not self._line(cruise):
                return None
            p = self._pose()

        g = self._stable_us()
        distance_from_axle = None if g is None else tp.FRONT_FROM_AXLE_CM + g
        x2 = (None if distance_from_axle is None else
              p.x + distance_from_axle * math.cos(p.th))
        gap12 = None if x2 is None else x2 - (x1 + tp.OBS1_SIZE_CM)
        if gap12 is not None and GAP12_RANGE[0] <= gap12 <= GAP12_RANGE[1]:
            self._record_obs2_us_hit(
                p.y + distance_from_axle * math.sin(p.th), p.th)
            logging.info("Obstacle 2 face at x=%.1f, %.1f cm past obstacle 1.",
                         x2, gap12)
            return x2
        if gap12 is not None and gap12 < GAP12_RANGE[0]:
            if g < LANE_US_MIN_CLEAR_CM:
                logging.error("Ultrasound on the lane reads %.1f cm, below the "
                              "%.1f cm live-clearance floor; not moving toward "
                              "that surface.", g, LANE_US_MIN_CLEAR_CM)
                return None
            if gap12 <= 0:
                logging.error("Lane echo overlaps obstacle 1 in the measured "
                              "frame; cannot identify obstacle 2 (x=%.1f).", x2)
                return None
            self._record_obs2_us_hit(
                p.y + distance_from_axle * math.sin(p.th), p.th)
            self.obs2_front_max = x1 + tp.OBS1_SIZE_CM + 60.0
            logging.warning("Measured spacing %.1f cm is shorter than the brief; "
                            "retaining the closer face x=%.1f from US %.1f cm. "
                            "Every following sweep is checked against this face.",
                            gap12, x2, g)
            return x2
        # From this lane the robot's centreline is outside the face of any
        # obstacle 2 narrower than the measured US ray. Preserve that live
        # upper bound: the bypass lane can be slightly outside a 60 cm face,
        # so treating every miss as a <=44 cm obstacle makes the later IR
        # lane too close to a medium board.
        ray_y = p.y + tp.FRONT_FROM_AXLE_CM * math.sin(p.th)
        self.obs2_half_max = min(
            W2_RANGE[1] / 2.0,
            max(NARROW_HALF_MAX, abs(ray_y - course.obs2_y)))
        logging.info("Ultrasound sees nothing plausible on the lane (%s) - "
                     "obstacle 2 half-width is below %.1f cm.", g,
                     self.obs2_half_max)
        return NARROW

    def find_narrow(self, x1: float, side1: float) -> Optional[float]:
        """Locate obstacle 2 with the inward side IR after a parallel US miss."""
        scan_side = side1
        p = self._pose()
        half_max = min(
            W2_RANGE[1] / 2.0,
            max(NARROW_HALF_MAX, self.obs2_half_max or NARROW_HALF_MAX))
        # Put the inward IR at least at its reliable minimum distance from the
        # widest face still compatible with the US miss. This remains inside
        # the 50 cm side boundary even for the medium-width edge case.
        scan_lane = half_max + tp.IR_SIDE_OFFSET_CM + tp.IR_MIN_GAP_CM
        course = Course(x1, x1 + tp.OBS1_SIZE_CM + GAP12_RANGE[0],
                        2 * half_max, obs1_y=self.obs1_y,
                        wall_y=CONSERVATIVE_WALL_Y,
                        obs2_y=self.obs1_y)

        movement = "widening to the narrow lane"
        # The US ray can miss a medium face by less than a centimetre. Its
        # wider IR lane is then valid, but the lane-change arcs may initially
        # sweep the rear corner of obstacle 1. Queue only the smallest forward
        # lead that clears that measured board; do not stop or add a separate
        # positioning motion.
        widen_options = []
        # The 50 cm side boundary can make the ideal 14 cm IR gap
        # unreachable for a medium board. Reduce the lane only as far as the
        # sensor's physical 10 cm floor, choosing the widest swept-clear lane.
        minimum_lane = int(math.ceil(
            half_max + tp.IR_SIDE_OFFSET_CM + 10.0))
        for lane in range(int(math.floor(scan_lane)), minimum_lane - 1, -1):
            turns = tp.lane_change(0.0, scan_side * lane - p.y, self.r)
            for lead in range(0, 36):
                candidate = _straight(lead) + turns
                if self._check(p, candidate, course) >= tp.CLEARANCE_FLOOR_CM:
                    widen_options.append(candidate)
                    scan_lane = float(lane)
                    break
            if widen_options:
                break
        selected_widen = self._first_clear(
            p, widen_options, course, movement)
        if selected_widen is None:
            return None
        widen = selected_widen
        logging.info("Narrow obstacle: %s (%s); the %s IR will watch inward.",
                     movement, ",".join(seg.token() for seg in widen),
                     "right" if scan_side > 0 else "left")
        if not self._line(widen):
            return None

        # Heading +x on the selected lane, the centreline is on the right for
        # the left lane, and on the left for the right lane.
        inner = "right" if scan_side > 0 else "left"
        far = x1 + tp.OBS1_SIZE_CM + GAP12_RANGE[1] + tp.OBS2_DEPTH_CM
        # Obstacle 2's end face, if it is 30..2*NARROW_HALF_MAX wide.
        expect_min = scan_lane - tp.IR_SIDE_OFFSET_CM - half_max - 4
        expect_max = scan_lane - tp.IR_SIDE_OFFSET_CM - W2_RANGE[0] / 2.0 + 4
        hits = []
        extended_hits = []

        def poll() -> bool:
            # Pair a moving IR sighting with the pose before its serial
            # query. Reading pose afterwards added another query's travel
            # to x2, placing the front face too far away before the turn.
            q = self.frame.pose()
            if q is not None:
                self._notify_obstacle2_view(q, x1)
            g = self._ir(inner)
            if q is None or g is None:
                extended_hits.clear()
                self.sleep_fn(SENSOR_POLL_S)
                return False
            candidate_x = q.x + tp.IR_FROM_AXLE_CM - tp.IR_LAG_CM
            # Obstacle 1 and its trailing-edge clutter show first. The brief's
            # spacing is face-to-face, so a valid obstacle-2 face cannot occur
            # before obstacle 1's far face plus GAP12_RANGE[0]. The previous
            # 15 cm gate accepted the logged x=111.7 echo, then reversed 67 cm
            # to photograph a location where obstacle 2 could not legally be.
            min_x2 = x1 + tp.OBS1_SIZE_CM + GAP12_RANGE[0]
            if candidate_x >= min_x2 and expect_min <= g <= expect_max:
                hits.append((q, g))
                # The side is only 10 cm thick. Three moving samples can
                # miss it entirely; brake on evidence and confirm at rest.
                self.stm.abort()                      # RST: no reply comes
                self.sleep_fn(0.10)
                return True
            if (candidate_x >= min_x2
                    and expect_max < g <= FRONT_FACE_HARD_MAX_CM):
                # The physical lane can sit a few centimetres farther from
                # the board than its nominal geometry.  A single 30-34 cm IR
                # sample can also be floor clutter, so extend the window only
                # after two consecutive moving readings agree.  This keeps
                # the normal close-range trigger immediate while allowing a
                # real board at the measured 33-34 cm offset to stop F115.
                extended_hits.append((q, g))
                extended_hits[:] = extended_hits[-2:]
                if (len(extended_hits) == 2
                        and abs(extended_hits[1][1]
                                - extended_hits[0][1]) <= 3.0):
                    hits.extend(extended_hits)
                    self.stm.abort()                  # RST: no reply comes
                    self.sleep_fn(0.10)
                    return True
            else:
                extended_hits.clear()
            # The STM's IR conversion does not gain information when queried
            # hundreds of times per second. Keep the chassis moving, but let
            # a miss wait for a fresh sensor interval. A valid first hit still
            # brakes immediately and pays no extra polling delay.
            self.sleep_fn(SENSOR_POLL_S)
            return False

        confirmed = False
        for attempt in range(3):
            hits.clear()
            extended_hits.clear()
            remaining = int(far - (self._pose().x + tp.IR_FROM_AXLE_CM))
            if remaining <= 0:
                break
            reply = tp.run_polling(self.stm, f"F{remaining}", poll)
            if reply is None or not (reply.startswith("OK") or reply == "STOPPED"):
                logging.error("STM answered %r while looking for obstacle 2.", reply)
                return None
            if not hits:
                logging.error("Narrow-lane scan finished without a plausible "
                              "obstacle-2 sighting on the %s IR.", inner)
                return None
            settled = None
            if reply == "STOPPED":
                settled = self._wait_for_stopped_pose()
                if settled is None:
                    return None
            first_pose, first_gap = hits[0]
            confirm_max = (FRONT_FACE_HARD_MAX_CM
                           if first_gap > expect_max else expect_max)
            logging.info("Narrow obstacle candidate on %s IR: %.1f cm; "
                         "confirming at rest (attempt %d/3).",
                         inner, first_gap, attempt + 1)
            # At most four IR queries (0.5s timeout each). Normally the first
            # two agree: only 0.10s braking + 0.06s sample spacing is added.
            consecutive = 0
            clear_consecutive = 0
            finite_edge_consecutive = 0
            # RST currently coasts before stopping. If that measured coast
            # carries the side sensor beyond obstacle 2's complete 10 cm
            # depth, a clear IR at rest confirms the trailing edge instead of
            # disproving the moving hit. Short one-sample spikes still fail
            # this spatial check and use the bounded retry below.
            coast = 0.0
            if settled is not None:
                coast = ((settled.x - first_pose.x) * math.cos(first_pose.th)
                         + (settled.y - first_pose.y) * math.sin(first_pose.th))
            edge_pass = max(4.0, tp.OBS2_DEPTH_CM - tp.IR_LAG_CM)
            for _ in range(4):
                self.sleep_fn(0.03)
                gap = self._ir(inner)
                if (gap is not None and expect_min <= gap <= confirm_max
                        and abs(gap - first_gap) <= 3.0):
                    consecutive += 1
                    clear_consecutive = 0
                    if consecutive == 2:
                        confirmed = True
                        break
                else:
                    consecutive = 0
                    finite_edge = (gap is not None and math.isfinite(gap)
                                   and gap > first_gap
                                   + max(4.0, IR_FACE_UNCERTAINTY_CM))
                    if gap is None or finite_edge:
                        clear_consecutive += 1
                        finite_edge_consecutive = (
                            finite_edge_consecutive + 1 if finite_edge else 0)
                        # Two agreeing samples while moving followed by two
                        # clear samples are the complete near-edge/far-edge
                        # signature of this 10 cm-deep board.  The 05:17:30
                        # floor run produced 30,29 cm, then 56,59 cm after
                        # braking; requiring the whole nominal depth as
                        # post-RST coast rejected that real board even though
                        # the beam had already crossed part of it before RST.
                        moving_edge_pair = (
                            len(hits) >= 2
                            and abs(hits[-1][1] - hits[-2][1]) <= 3.0)
                        if (clear_consecutive == 2
                                and (coast >= edge_pass
                                     or moving_edge_pair
                                     or (finite_edge_consecutive >= 2
                                         and coast >= 1.0))):
                            confirmed = True
                            logging.info(
                                "Narrow obstacle edge confirmed by %d "
                                "consistent moving sighting(s), %.1f cm "
                                "post-RST travel, and clear %s IR.",
                                len(hits), coast, inner)
                            break
                    else:
                        clear_consecutive = 0
                        finite_edge_consecutive = 0
            if confirmed:
                # Keep the first moving pose for the front-edge estimate;
                # using the braking endpoint would bias it farther forward.
                logging.info("Narrow obstacle confirmed on %s IR; retaining "
                             "first-sighting pose %s.", inner, first_pose)
                break
            logging.warning("Side IR candidate was seen but not confirmed; %s.",
                            "resuming only the remaining narrow-lane scan"
                            if attempt < 2 else "retry limit reached")
        if not confirmed:
            logging.error("Narrow obstacle IR confirmation exhausted; "
                          "not treating an unconfirmed reading as its edge.")
            return None
        # A moving side-IR hit can be reported a few centimetres after the
        # sensor reaches the front edge. Place the collision face at the near
        # end of that uncertainty interval; planning it farther away produced
        # a nominal 4.5 cm recovery that passed the real face by 3.44 cm.
        x2 = (hits[0][0].x + tp.IR_FROM_AXLE_CM - tp.IR_LAG_CM
              - IR_FACE_UNCERTAINTY_CM)
        edge_samples = [q.y - scan_side * (tp.IR_SIDE_OFFSET_CM + gap)
                        for q, gap in hits]
        edge_samples.sort()
        self.obs2_near_y = edge_samples[len(edge_samples) // 2]
        logging.info("Obstacle 2 side seen %.0f cm off the %s IR: face at "
                     "x=%.1f; near edge y=%.1f.", min(g for _, g in hits),
                     inner, x2, self.obs2_near_y)
        return x2

    def photo_candidates(self, p: Pose, course: Course, side1: float
                         ) -> Iterator[Tuple[List[Seg], float, int]]:
        """
        Moves to the photo pose, best first: KICK_DEG out to side1, a
        diagonal, and a swing back in, sized so the camera ends PHOTO_DIST_CM
        from the arrow (obstacle 2's face centre) looking straight at it.
        With the full swing of 90 it looks in from 45 deg. Yields (segs,
        distance, swing); nearer and shallower ones follow for when walls
        leave no room for the ideal.
        """
        out = tp._turn_toward(p.th, side1)
        inward = _other(out)

# The second obstacle may be up to 150 cm beyond the first. Keep the car
# parallel only until obstacle 1 is behind it, then calculate where a single
# inward arc and straight camera ray meet the centre of obstacle 2.
        # Two-centimetre/degree sampling is finer than the hardware and pose
        # uncertainty while avoiding thousands of near-duplicate candidates
        # before the robot can issue its inward camera move.
        for lead in range(0, 111, 2):
            prefix = _straight(lead)
            # Start with the requested 45-degree move toward obstacle 2, then
            # partially straighten so the remaining forward motion follows
            # the actual camera ray.  This is the visible/continuous S entry
            # that the floor run expected instead of F90 followed by FR25.
            for recover in range(15, 45, 2):
                turn = (prefix + [Seg(inward, KICK_DEG),
                                  Seg(out, recover)])
                turned = tp.run(p, turn, self.r, step=1e6)[-1]
                cam = tp.straight(turned, CAMERA_FROM_AXLE_CM)
                c, s = math.cos(turned.th), math.sin(turned.th)
                dx = course.x2 - cam.x
                dy = course.obs2_y - cam.y
                ahead = dx * c + dy * s
                lateral = -dx * s + dy * c
                if abs(lateral) <= 4.0 and ahead >= PHOTO_MIN_CM - 6.0:
                    for standoff in (PHOTO_DIST_CM + 8.0,
                                      PHOTO_DIST_CM + 4.0,
                                      PHOTO_DIST_CM, 30.0, PHOTO_MIN_CM):
                        candidate = turn + _straight(ahead - standoff)
                        yield candidate, math.hypot(ahead, lateral), recover
            # Once obstacle 1 has been cleared, aim at obstacle 2 directly.
            # Calculate the direct centre ray at the earliest safe pose.  For
            # a distant obstacle this is intentionally shallow rather than a
            # long parallel straight followed by an artificial 30-degree arc.
            for look in range(tp.MIN_ARC_DEG, 61):
                turn = prefix + [Seg(inward, look)]
                turned = tp.run(p, turn, self.r, step=1e6)[-1]
                cam = tp.straight(turned, CAMERA_FROM_AXLE_CM)
                c, s = math.cos(turned.th), math.sin(turned.th)
                dx = course.x2 - cam.x
                dy = course.obs2_y - cam.y
                ahead = dx * c + dy * s
                lateral = -dx * s + dy * c
                # A one-degree integer arc step can leave a few centimetres
                # of lateral residual on a distant target.  The final camera
                # check still enforces the angular limit.
                if (abs(lateral) > 4.0
                        or ahead < PHOTO_MIN_CM - 6.0):
                    continue
                for standoff in (PHOTO_DIST_CM + 8.0,
                                  PHOTO_DIST_CM + 4.0,
                                  PHOTO_DIST_CM, 30.0, PHOTO_MIN_CM):
                    candidate = turn + _straight(ahead - standoff)
                    yield candidate, math.hypot(ahead, lateral), look

        e = (math.cos(p.th), math.sin(p.th))           # along the lead
        # A few centimetres of extra standoff can turn a nominal R2 before
        # the handoff into no reverse at all.  Search those poses first; the
        # final view gate already permits PHOTO_DIST_CM + 12 cm.
        d = PHOTO_DIST_CM + 8.0
        while d >= PHOTO_MIN_CM - 0.01:
            # A fixed 45-degree outward kick consumes too much x room when
            # the first bypass lane is only 20--25 cm off centre.  That was
            # the source of the post-photo inverse in the 07:52 floor run:
            # the car could obtain the image, but not retain both forward
            # exits.  Search shallower calibrated S entries as well.  They
            # move outward before looking inward, so camera positioning and
            # creation of the turn pocket are the same continuous motion.
            for kick in (15, 25, 35, KICK_DEG):
                k = math.radians(kick) * side1
                u = (math.cos(p.th + k), math.sin(p.th + k))
                for look in (15, 25, 35, 45):
                    swing = kick + look
                    end0 = tp.run(
                        p, [Seg(out, kick), Seg(inward, swing)],
                        self.r)[-1]
                    cam0 = (
                        end0.x + CAMERA_FROM_AXLE_CM * math.cos(end0.th),
                        end0.y + CAMERA_FROM_AXLE_CM * math.sin(end0.th))
                    want = (course.x2 - d * math.cos(end0.th),
                            course.obs2_y - d * math.sin(end0.th))
                    # cam0 + lead*e + diag*u = want, for lead and diag.
                    dx, dy = want[0] - cam0[0], want[1] - cam0[1]
                    det = e[0] * u[1] - e[1] * u[0]
                    if abs(det) < 1e-9:
                        continue
                    lead = (dx * u[1] - dy * u[0]) / det
                    diag = (e[0] * dy - e[1] * dx) / det
                    if lead < -0.5 or diag < -0.5:
                        continue
                    yield (_straight(lead) + [Seg(out, kick)]
                           + _straight(diag) + [Seg(inward, swing)],
                           d, swing)
            d -= 4.0
        # Already out wide (the narrow-obstacle lane), or walls too close for
        # any of those: no kick, just on or back and swing in, looking at the
        # arrow from whatever distance that gives. swing - 45 is the angle it
        # looks in from, as above.
        # A narrow-board side scan stops only about one IR lever-arm beyond
        # its front edge.  Include a 50-degree inward look: it reaches the
        # requested 30 cm camera distance with a materially shorter retreat
        # than the old 45-degree-only choice.  Shallower choices remain for
        # longer gaps and wall-limited layouts.
        for look in (50, 45, 35, 25, 15):
            end0 = tp.run(p, [Seg(inward, look)], self.r)[-1]
            cam0 = (end0.x + CAMERA_FROM_AXLE_CM * math.cos(end0.th),
                    end0.y + CAMERA_FROM_AXLE_CM * math.sin(end0.th))
            v = (math.cos(end0.th), math.sin(end0.th))
            # cam0 + lead*e = arrow - dist*v, i.e. lead*e + dist*v = arrow - cam0.
            ax, ay = course.x2 - cam0[0], 0.0 - cam0[1]
            det = e[0] * v[1] - e[1] * v[0]
            if abs(det) < 1e-9:
                continue
            lead = (ax * v[1] - ay * v[0]) / det
            dist = (e[0] * ay - e[1] * ax) / det
            if PHOTO_MIN_CM - 0.01 <= dist <= PHOTO_DIST_CM + 12.0:
                yield _straight(lead) + [Seg(inward, look)], dist, look + KICK_DEG

    @staticmethod
    def _first_board_behind_camera(p: Pose, x1: float, obs1_y: float = 0.0) -> float:
        """Smallest distance of obstacle 1 behind the camera's forward plane."""
        c, s = math.cos(p.th), math.sin(p.th)
        cx = p.x + CAMERA_FROM_AXLE_CM * c
        cy = p.y + CAMERA_FROM_AXLE_CM * s
        half = tp.OBS1_SIZE_CM / 2.0
        return -max((x - cx) * c + (y - cy) * s
                    for x in (x1, x1 + tp.OBS1_SIZE_CM)
                    for y in (obs1_y - half, obs1_y + half))

    def _notify_obstacle2_view(self, p: Pose, x1: float,
                              obs1_y: float = 0.0) -> None:
        if self._stage2_view_open:
            return
        if not self._stage2_camera_positioned:
            return
        if (abs(math.atan2(math.sin(p.th), math.cos(p.th))) >
                math.radians(STAGE2_MAX_HEADING_DEG)
                or self._first_board_behind_camera(p, x1, obs1_y)
                < STAGE2_REAR_EXCLUSION_CM):
            return
        self._stage2_view_open = True
        if self.lane_reached_fn is not None:
            self.lane_reached_fn()

    def photo_obstacle2(self, course: Course, side1: float) -> float:
        """Read obstacle 2 at its checked stationary camera pose."""
        if not self._stage2_view_open:
            self._stage2_view_open = True
            if self.lane_reached_fn is not None:
                self.lane_reached_fn()
        if self.stage2_centered:
            logging.info("Obstacle 2 recognition head-on from the course "
                         "centreline; obstacle 1 is already behind the camera.")
        else:
            logging.info("Obstacle 2 recognition from the checked angled "
                         "camera pose; obstacle 1 is behind the lens plane.")
        return _ask_arrow(self.input_fn, "Obstacle 2")

    def position_obstacle2_camera(self, course: Course,
                                  side1: float) -> bool:
        """Move once to a collision-checked pose aimed at obstacle 2.

        The camera ray, not US after the turn, controls this move.  A narrow
        side-IR stop may require a bounded retreat because its camera has
        already crossed the face; ordinary runs turn inward first and follow
        the calculated ray continuously.
        """
        p = self._pose()
        inward = tp._turn_toward(p.th, -side1)

        # A narrow obstacle found by the side IR is a different geometric
        # state from the ordinary parallel-US case.  The IR is mounted ahead
        # of the axle and RST adds measured coast; consequently the camera can
        # already be beyond the front plane when the chassis settles.  An
        # immediate inward arc from there points beside/behind the sign (the
        # 11:43 run produced null detections for every frame after FR24).
        # Reposition only as far as the measured geometry requires, then turn
        # toward the face.  Select the shortest such view that also retains a
        # complete all-forward handoff for either possible second arrow.
        if self.obs2_near_y is not None:
            narrow_options = []
            for candidate, camera_distance, _ in self.photo_candidates(
                    p, course, side1):
                if (not candidate
                        or any(seg.kind not in ("F", "B", "L", "R")
                               for seg in candidate)):
                    continue
                endpoint = tp.run(p, candidate, self.r, step=1e6)[-1]
                camera = tp.straight(endpoint, CAMERA_FROM_AXLE_CM)
                target = math.atan2(course.obs2_y - camera.y,
                                    course.x2 - camera.x)
                view_error = abs(math.degrees(math.atan2(
                    math.sin(endpoint.th - target),
                    math.cos(endpoint.th - target))))
                if view_error > 6.0:
                    continue
                candidate_clearance = self._check(p, candidate, course)
                if candidate_clearance < tp.CLEARANCE_FLOOR_CM:
                    continue

                exits = True
                worst_recovery = 0
                for possible_side2 in (side1, -side1):
                    possible = False
                    for handoff, reverse in self.face_handoff_candidates(
                            endpoint, course, side1, possible_side2,
                            straight_limit=60):
                        if (reverse or any(seg.kind not in ("F", "L", "R")
                                           for seg in handoff)):
                            continue
                        if self.face_handoff_clearance(
                                endpoint, handoff, course, side1,
                                possible_side2, False
                                ) >= tp.CLEARANCE_FLOOR_CM:
                            possible = True
                            break
                    if not possible and possible_side2 == side1:
                        self._handoff_cache = None
                        recovery = self.select_short_same_side_recovery(
                            endpoint, course, side1)
                        if recovery is not None:
                            possible = True
                            worst_recovery = max(
                                worst_recovery,
                                sum(seg.value for seg in recovery[0]
                                    if seg.kind == "B"))
                    if not possible:
                        exits = False
                        break
                if not exits:
                    continue

                reverse_cm = sum(seg.value for seg in candidate
                                 if seg.kind == "B")
                narrow_options.append((
                    worst_recovery,
                    reverse_cm,
                    tp.motion_cost(candidate, self.r, fast=self.fast),
                    view_error, -candidate_clearance, camera_distance,
                    candidate))

            if not narrow_options:
                logging.error(
                    "Narrow side-IR stop left no camera view with two "
                    "forward exits; refusing to turn away from the sign.")
                return False

            (_, _, _, view_error, negative_clearance, camera_distance,
             camera_route) = min(narrow_options)
            logging.info(
                "Narrow obstacle camera recovery: %s; %.1f cm view, %.1f "
                "degree aim error, %.1f cm clearance; both arrow outcomes "
                "pre-validated with at most one bounded straight recovery.",
                ",".join(seg.token() for seg in camera_route),
                camera_distance, view_error, -negative_clearance)
            if not self._line(camera_route):
                return False
            measured_pose = self.frame.pose()
            if measured_pose is not None:
                self.th = measured_pose.th
            self._handoff_cache = None
            self.stage2_centered = False
            self._stage2_camera_positioned = True
            self._stage2_view_open = True
            if self.lane_reached_fn is not None:
                self.lane_reached_fn()
            logging.info(
                "Obstacle 2 camera is stationary and aimed at the measured "
                "narrow face; recognition starts now.")
            return True

        # Ordinary US-localised runs use the same aimed route selector.  The
        # former implementation chose the largest collision-free arc without
        # checking where the camera pointed.  That produced FR24/FR45 poses
        # whose camera ray missed (or had already passed) the sign, followed
        # by seconds of null detections.  A clear turn is not automatically a
        # usable photograph: solve the ray to the measured face centre.
        return self._position_obstacle2_camera_with_approach(course, side1)

    def _position_obstacle2_camera_with_approach(self, course: Course,
                                                 side1: float) -> bool:
        """Choose one aimed diagonal view from the obstacle-1 bypass lane.

        Obstacle 2 has already been localised before this method runs. Vision
        remains idle throughout that sensing movement, so neither obstacle 1
        nor a fleeting parallel view can become the stage-2 answer.
        """
        p = self._pose()
        options = []
        inward = tp._turn_toward(p.th, -side1)

        # Long-gap preferred route: leave the obstacle-1 lane immediately
        # with a real 45-degree inward turn, cross to the course centre, then
        # restore heading for a square obstacle-2 photograph.  In the 09:15
        # run the mathematically direct ray was only FR12; although centred on
        # paper, that was not a useful physical transition/view.  This merge
        # is the requested visible inward turn and uses the obstacle-1 style
        # turn/diagonal/turn geometry.  It is offered only when the complete
        # sweep and the remaining 30 cm photo approach fit.
        outward = _other(inward)
        lateral = abs(p.y - course.obs2_y)
        # A narrow-face recovery may already have completed the centre merge.
        # In that case do not manufacture a 45-degree loop merely to satisfy
        # the angled-camera family; finish the short square photo approach.
        if lateral <= 2.0:
            cam = tp.straight(p, CAMERA_FROM_AXLE_CM)
            forward = ((course.x2 - cam.x) * math.cos(p.th)
                       + (course.obs2_y - cam.y) * math.sin(p.th)
                       - PHOTO_DIST_CM)
            centred_candidate = _straight(forward)
            if centred_candidate:
                centred_end = tp.run(
                    p, centred_candidate, self.r, step=1e6)[-1]
                clearance = self._check(p, centred_candidate, course)
                if (clearance >= tp.CLEARANCE_FLOOR_CM
                        and abs(centred_end.y - course.obs2_y) <= 2.0):
                    options.append((0, 0, 0, 0.0,
                                    tp.motion_cost(centred_candidate, self.r,
                                                   fast=self.fast),
                                    -clearance, centred_candidate, True))
        arc_shift = 2.0 * self.r * (1.0 - math.cos(math.radians(KICK_DEG)))
        diagonal = max(0.0, (lateral - arc_shift)
                       / math.sin(math.radians(KICK_DEG)))
        centre_prefix = [Seg(inward, KICK_DEG)]
        centre_prefix += _straight(diagonal)
        centre_prefix.append(Seg(outward, KICK_DEG))
        centred_pose = tp.run(p, centre_prefix, self.r, step=1e6)[-1]
        centred_cam = tp.straight(centred_pose, CAMERA_FROM_AXLE_CM)
        centred_forward = ((course.x2 - centred_cam.x)
                           * math.cos(centred_pose.th)
                           + (course.obs2_y - centred_cam.y)
                           * math.sin(centred_pose.th)
                           - PHOTO_DIST_CM)
        if centred_forward >= 0.5:
            centred_candidate = centre_prefix + _straight(centred_forward)
            centred_end = tp.run(
                p, centred_candidate, self.r, step=1e6)[-1]
            centred_clearance = self._check(p, centred_candidate, course)
            centred_error = abs(math.degrees(math.atan2(
                math.sin(centred_end.th), math.cos(centred_end.th))))
            if (centred_clearance >= tp.CLEARANCE_FLOOR_CM
                    and abs(centred_end.y - course.obs2_y) <= 2.0
                    and self._first_board_behind_camera(
                        centred_end, course.x1, course.obs1_y) >= 0.0):
                # Keep this as a geometry fallback.  A single angled photo
                # turn is preferred when it fits, per the requested motion.
                options.append((2, 0, 0, centred_error,
                                tp.motion_cost(centred_candidate, self.r,
                                               fast=self.fast),
                                -centred_clearance,
                                centred_candidate, True))

        # Aim diagonally from the cleared side. Normally the inward turn is
        # the first primitive. If the rear of obstacle 1 still occupies its
        # swept circle, permit only the shortest checked forward lead needed
        # to clear that board, then turn inward in the same command.
        for candidate, _, _ in self.photo_candidates(p, course, side1):
            if (not candidate or any(
                    seg.kind not in ("F", "L", "R") for seg in candidate)):
                continue
            candidate_lead = (candidate[0].value
                              if candidate[0].kind in ("F", "B") else 0)
            if candidate_lead > tp.OBS1_SIZE_CM + tp.ROBOT_LENGTH_CM:
                continue
            first_turn = next(
                (seg for seg in candidate if seg.kind in ("L", "R")), None)
            if (first_turn is None or first_turn.kind != inward
                    or any(seg.kind != "F"
                           for seg in candidate[:candidate.index(first_turn)])):
                continue
            turn_index = candidate.index(first_turn)
            if not any(seg.kind == "F" and seg.value >= 3
                       for seg in candidate[turn_index + 1:]):
                # The image must follow an actual diagonal approach, not be
                # taken immediately at the end of the arc.  Besides matching
                # the intended motion, this retains room for either arrow's
                # obstacle-2 continuation.
                continue
            end = tp.run(p, candidate, self.r, step=1e6)[-1]
            heading = math.atan2(math.sin(end.th), math.cos(end.th))
            inward_heading = math.degrees(-side1 * heading)
            if inward_heading < STAGE2_CAMERA_MIN_INWARD_DEG:
                continue
            # An aimed oblique view only needs obstacle 1 fully behind the
            # lens plane. The additional 6 cm reserve is for a parallel view;
            # enforcing it here removes the only safe photo pose on a 60 cm
            # narrow course even though every corner of obstacle 1 is behind
            # the camera.
            if self._first_board_behind_camera(
                    end, course.x1, course.obs1_y) < 0.0:
                continue
            clearance = self._check(p, candidate, course)
            if clearance < tp.CLEARANCE_FLOOR_CM:
                continue
            cam = tp.straight(end, CAMERA_FROM_AXLE_CM)
            target = math.atan2(course.obs2_y - cam.y,
                                course.x2 - cam.x)
            view_error = abs(math.degrees(math.atan2(
                math.sin(end.th - target), math.cos(end.th - target))))
            camera_distance = math.hypot(
                course.x2 - cam.x, course.obs2_y - cam.y)
            if (view_error > STAGE2_MAX_HEADING_DEG
                    or not PHOTO_MIN_CM - 6.0 <= camera_distance
                    <= PHOTO_DIST_CM + 12.0):
                continue
            leading_straight = candidate_lead
            clearance_penalty = 0 if clearance >= tp.CLEARANCE_CM else 1
            # Use the directly calculated centre ray.  Its turn grows for a
            # nearby obstacle and shrinks for a distant one; the camera still
            # points at the arrow centre and stops at the configured standoff.
            options.append((1, clearance_penalty, leading_straight, view_error,
                            tp.motion_cost(candidate, self.r, fast=self.fast),
                            -clearance, candidate, False))

        if not options:
            logging.error("No collision-checked immediate inward angled "
                          "camera view of obstacle 2 from %s; refusing a "
                          "parallel straight before the turn.", p)
            return False

        # Camera positioning must be immediate.  Do not run the obstacle-2
        # handoff search for both hypothetical arrows here: on the Pi that
        # blocked motion for 12 seconds (10:27:18--10:27:30).  Choose the
        # earliest checked visible pose now.  Once image two is known,
        # around_obstacle2 plans only its one real continuation.
        # Every option has already passed the six-degree camera gate.  Within
        # that usable cone prefer the shortest continuous motion: choosing a
        # cosmetically smaller error can consume the turn pocket needed by
        # the still-unknown second arrow.
        chosen = min(options, key=lambda item: (
            item[0], item[1], item[2], item[4], item[3], item[5]))
        _, _, _, view_error, _, negative_clearance, segs, centred = chosen
        logging.info("Obstacle-2 %s camera route: %s; view error %.1f deg, "
                     "clearance %.1f cm.",
                     "centred" if centred else "angled",
                     ",".join(seg.token() for seg in segs),
                     view_error,
                     -negative_clearance)

        # Send the checked turn/diagonal as one continuous command.  US was
        # already used to locate the face; reading it again from an oblique
        # heading made people/background objects alter the route and split a
        # fluid camera move into stop/zoom/reverse corrections.
        if not self._line(segs):
            return False

        q = self._pose()
        cam = tp.straight(q, CAMERA_FROM_AXLE_CM)
        target_heading = math.atan2(course.obs2_y - cam.y,
                                    course.x2 - cam.x)
        heading_error = math.atan2(math.sin(q.th - target_heading),
                                   math.cos(q.th - target_heading))
        distance = math.hypot(course.x2 - cam.x, course.obs2_y - cam.y)
        rear_need = STAGE2_REAR_EXCLUSION_CM if centred else 0.0
        if (abs(math.degrees(heading_error)) > STAGE2_MAX_HEADING_DEG
                or not PHOTO_MIN_CM - 6.0 <= distance <= PHOTO_DIST_CM + 12.0
                or self._first_board_behind_camera(
                    q, course.x1, course.obs1_y) < rear_need):
            logging.error("Measured obstacle-2 camera pose is not usable: %s, "
                          "distance %.1f cm, heading error %.1f deg.", q,
                          distance, math.degrees(heading_error))
            return False

        self.stage2_centered = centred
        self._stage2_camera_positioned = True
        self._stage2_view_open = True
        if self.lane_reached_fn is not None:
            self.lane_reached_fn()
        logging.info("Obstacle 2 camera is %s and stationary; recognition may "
                     "start now.", "head-on" if centred else "aimed from an angle")
        return True

    # ── obstacle 2 ────────────────────────────────────────────────────────────

    def face_candidates(self, p: Pose, course: Course, side1: float,
                        reverse: bool, straight_limit: int = 30
                        ) -> Iterator[List[Seg]]:
        """
        From the photo pose to running along obstacle 2's face: turn on to
        heading -side1*90, the IR FACE_GAP_CM off the face. Forward runs
        toward -side1 from there; a reverse run toward side1 must start with
        the IR well inside the face, so it creeps that way first.
        """
        inward = tp._turn_toward(p.th, -side1)
        turn = int(round(90 - abs(math.degrees(p.th))
                         - (0.0 if reverse else
                            OPPOSITE_HANDOFF_RESERVE_DEG)))
        th_t = -side1 * math.pi / 2
        options = []
        for s in range(-straight_limit, straight_limit + 2, 2):
            head = _straight(s) + ([Seg(inward, turn)] if turn >= 1 else [])
            end = tp.run(p, head, self.r, step=1e6)[-1]
            gap = course.x2 - (end.x + tp.IR_SIDE_OFFSET_CM)
            if not (tp.IR_MIN_GAP_CM <= gap <= IR_FACE_MAX_CM - 4):
                continue
            creep = []
            if reverse:
                ir_y = end.y + tp.IR_FROM_AXLE_CM * math.sin(th_t)
                creep = _straight(side1 * ir_y - (-5.0))
                creep = creep if creep and creep[0].kind == "F" else []
            options.append((abs(gap - FACE_GAP_CM), head + creep))
        for _, segs in sorted(options, key=lambda t: t[0]):
            yield segs

    def forward_front_candidates(self, p: Pose, course: Course,
                                 side: float) -> Iterator[List[Seg]]:
        """Face the requested end, then measure it by driving along the front."""
        delta = math.atan2(math.sin(side * math.pi / 2 - p.th),
                           math.cos(side * math.pi / 2 - p.th))
        angle = int(round(abs(math.degrees(delta))))
        if not tp.MIN_ARC_DEG <= angle <= 180:
            return
        turn = Seg("L" if delta > 0 else "R", angle)
        options = []
        for lead in range(0, 62, 2):
            segs = _straight(lead) + [turn]
            end = tp.run(p, segs, self.r, step=1e6)[-1]
            gap = course.x2 - (end.x + tp.IR_SIDE_OFFSET_CM)
            ir_y = end.y + tp.IR_FROM_AXLE_CM * math.sin(end.th)
            # Start over a portion of the face supported by the US hit,
            # not beyond an assumed maximum-width edge.
            known_half = self._obs2_known_half(course)
            if (tp.IR_MIN_GAP_CM <= gap <= IR_FACE_MAX_CM - 4
                    and abs(ir_y - course.obs2_y) <= known_half - 4):
                options.append((abs(gap - FACE_GAP_CM), segs))
        for _, segs in sorted(options, key=lambda item: item[0]):
            yield segs

    def opposite_direct_candidates(self, p: Pose, course: Course,
                                   side1: float) -> Iterator[List[Seg]]:
        """Join the front face with one forward lead and one inward arc.

        A US hit from the bypass lane proves that obstacle 2 reaches that
        lateral coordinate. Use that evidence instead of always assuming a
        30 cm face: opposite-arrow runs can keep moving and turn toward the
        detected arrow without first making an outward S. A small heading
        range absorbs bypass drift while remaining nearly parallel to the
        face for the live IR scan.
        """
        heading = math.atan2(math.sin(p.th), math.cos(p.th))
        if abs(heading) > math.radians(STAGE2_MAX_HEADING_DEG):
            return
        toward = -side1
        options = []
        for target_deg in range(90, 83, -1):
            target = toward * math.radians(target_deg)
            delta = math.atan2(math.sin(target - heading),
                               math.cos(target - heading))
            angle = int(round(abs(math.degrees(delta))))
            if not tp.MIN_ARC_DEG <= angle <= 180:
                continue
            turn = Seg("L" if delta > 0 else "R", angle)
            for lead in range(0, 101):
                segs = _straight(lead) + [turn]
                end = tp.run(p, segs, self.r, step=1e6)[-1]
                gap = course.x2 - (end.x + tp.IR_SIDE_OFFSET_CM)
                if not (tp.IR_MIN_GAP_CM <= gap <= IR_FACE_MAX_CM):
                    continue
                if not self._face_handoff_endpoint_valid(
                        p, segs, course, side1, -side1, False):
                    continue
                # Keep a small measured-turn reserve before parallel so arc
                # overshoot does not consume the obstacle-end corner pocket.
                options.append((abs((90 - OPPOSITE_HANDOFF_RESERVE_DEG)
                                    - target_deg),
                                tp.motion_cost(segs, self.r, fast=self.fast),
                                abs(gap - FACE_GAP_CM), segs))
        # Expose only the heading closest to the calibrated reserve; cost then
        # chooses the smallest lead at that heading without accidental FR84.
        best_heading_error = min((item[0] for item in options), default=None)
        for heading_error, _, _, segs in sorted(
                options, key=lambda item: (item[0], item[1], item[2])):
            if heading_error != best_heading_error:
                continue
            yield segs

    def same_side_forward_candidates(self, p: Pose, course: Course,
                                     side: float,
                                     straight_limit: int = 60,
                                     direct_only: bool = False
                                     ) -> Iterator[List[Seg]]:
        """Sweep from the same-side photo pose directly behind obstacle 2.

        When both arrows point to the same side, the bypass lane is already
        on the end of obstacle 2 that the robot needs to round. Move closer,
        arc outward, then use a wider opposite arc to arrive behind it. This
        is one continuous, all-forward outer eight: no front-face reverse and
        no second maneuver from a wall-tight +/-90 degree intermediate pose.
        """
        outward = "L" if side > 0 else "R"
        inward = "R" if side > 0 else "L"
        checked = self.same_side_course(course)
        back = checked.x2 + tp.OBS2_DEPTH_CM
        lo = back + tp.IR_SIDE_OFFSET_CM + tp.IR_MIN_GAP_CM + 4.0
        hi = back + tp.IR_SIDE_OFFSET_CM + BACK_GAP_MAX_CM
        options = []
        heading_error = math.degrees(p.th)
        # IR may already have placed us outside the near end. In that case
        # a straight lead and one inward arc need no additional lane shift.
        direct_turn = int(round(90.0 + side * heading_error))
        if tp.MIN_ARC_DEG <= direct_turn <= 180:
            for lead in range(0, straight_limit + 1, 2):
                segs = _straight(lead) + [Seg(inward, direct_turn)]
                end = tp.run(p, segs, self.r, step=1e6)[-1]
                if lo <= end.x <= hi:
                    options.append((tp.motion_cost(segs, self.r),
                                    abs(end.x - (lo + hi) / 2.0), segs))
        if direct_only:
            for _, _, segs in sorted(options, key=lambda item: (item[0], item[1])):
                yield segs
            return
        # Net turn is -side*90 degrees. The second arc therefore exceeds the
        # first by 90 degrees (with the small measured heading error folded
        # in). Candidate distances are searched, not fixed to a floor run.
        # Keep the established compact choices first so working routes retain
        # their ranking, then consider the wider calibrated 90-degree arc.
        first_angles = (30, 45, 60, 75) + tuple(
            angle for angle in CALIBRATED_TURN_ANGLES
            if 75 < angle <= 90)
        for first in first_angles:
            second = int(round(first + 90.0 + side * heading_error))
            if not tp.MIN_ARC_DEG <= second <= 180:
                continue
            # Final x is affine in the straight lead and diagonal length.
            # Solve the diagonal rather than trying every 4 cm grid cell.
            base = [Seg(outward, first), Seg(inward, second)]
            end0 = tp.run(p, base, self.r, step=1e6)[-1]
            dx_lead = math.cos(p.th)
            dx_middle = math.cos(p.th + side * math.radians(first))
            for lead in range(0, straight_limit + 1, 4):
                if abs(dx_middle) < 0.05:
                    middles = range(0, straight_limit + 1, 4)
                else:
                    middles = set()
                    for target_x in (lo, (lo + hi) / 2, hi):
                        mid = int(round((target_x - end0.x - lead * dx_lead)
                                        / dx_middle))
                        middles.update((mid - 1, mid, mid + 1))
                for middle in sorted(middles):
                    if not 0 <= middle <= straight_limit:
                        continue
                    segs = (_straight(lead) + [Seg(outward, first)]
                            + _straight(middle) + [Seg(inward, second)])
                    end = tp.run(p, segs, self.r, step=1e6)[-1]
                    if not (lo <= end.x <= hi):
                        continue
                    options.append((tp.motion_cost(segs, self.r),
                                    abs(end.x - (lo + hi) / 2.0), segs))
        for _, _, segs in sorted(options, key=lambda item: (item[0], item[1])):
            yield segs

    def same_side_continuation_candidates(
            self, p: Pose, course: Course, side: float
            ) -> Iterator[List[Seg]]:
        """Recover an outer arc from its measured pose without reversing.

        A same-side outer route starts by steering away from obstacle 2. If
        that calibrated arc lands a few centimetres off its nominal endpoint,
        the remaining nominal straighten-and-turn can lose its clearance.
        From that measured diagonal pose, finish with two arcs in the inward
        direction. Their angles are derived from the measured heading and the
        requested final heading; the straight and split are searched against
        the measured obstacle/wall geometry.
        """
        outward_component = side * math.sin(p.th)
        if outward_component <= math.sin(math.radians(10.0)):
            return
        inward = "R" if side > 0 else "L"
        target = -side * math.pi / 2.0
        if inward == "L":
            total = math.degrees((target - p.th) % (2 * math.pi))
        else:
            total = math.degrees((p.th - target) % (2 * math.pi))
        total = int(round(total))
        if not 2 * tp.MIN_ARC_DEG <= total <= 180:
            return

        back = course.x2 + tp.OBS2_DEPTH_CM
        lo = back + tp.IR_SIDE_OFFSET_CM + tp.IR_MIN_GAP_CM + 4.0
        hi = back + tp.IR_SIDE_OFFSET_CM + BACK_GAP_MAX_CM
        options = []
        for first in range(tp.MIN_ARC_DEG,
                           total - tp.MIN_ARC_DEG + 1, 2):
            second = total - first
            for middle in range(0, 61, 4):
                segs = ([Seg(inward, first)] + _straight(middle)
                        + [Seg(inward, second)])
                end = tp.run(p, segs, self.r, step=1e6)[-1]
                if not lo <= end.x <= hi:
                    continue
                # A roughly quarter/three-quarter split keeps the first arc
                # away from both the wall and the obstacle corner. Exact
                # swept clearance is still computed by select_face_handoff;
                # keeping this generator geometric avoids spending the whole
                # planning budget checking candidates twice.
                split_error = abs(first - 0.26 * total)
                options.append((split_error, tp.motion_cost(segs, self.r), segs))
        for _, _, segs in sorted(options, key=lambda item: (item[0], item[1])):
            yield segs

    def same_side_lane_candidates(self, p: Pose, course: Course,
                                 side: float) -> Iterator[List[Seg]]:
        """Shift outward, straighten, pass the end, then turn behind it.

        Unlike a single large inward arc, the straight between the lane
        shift and final turn controls where the robot crosses the front face.
        Endpoint calculations are cheap; selection checks the full trace.
        """
        courses = list(self.same_side_courses(course))
        edge = max(c.w2 / 2.0 for c in courses)
        wall = min((c.wall_y if c.wall_y is not None else
                    c.w2 / 2.0 + tp.WALL_CLEARANCE_CM) for c in courses)
        low = edge + tp.HALF_WIDTH_CM + tp.CLEARANCE_FLOOR_CM
        high = wall - tp.HALF_WIDTH_CM - tp.CLEARANCE_FLOOR_CM
        if any(c.walls for c in courses) and low > high:
            return
        outward, inward = ("L", "R") if side > 0 else ("R", "L")
        final = int(round(90 + side * math.degrees(p.th)))
        if not tp.MIN_ARC_DEG <= final <= 180:
            return
        back = course.x2 + tp.OBS2_DEPTH_CM + tp.IR_SIDE_OFFSET_CM
        options = []
        targets = (low, (low + high) / 2.0, high)
        lane_angles = (15, 30, 45) + tuple(
            angle for angle in CALIBRATED_TURN_ANGLES
            if 45 < angle <= 90)
        for angle in lane_angles:
            diagonal_y = side * math.sin(p.th + side * math.radians(angle))
            if diagonal_y <= 0.1:
                continue
            for lead in (0, 4, 8, 12):
                base = (_straight(lead) + [Seg(outward, angle),
                                          Seg(inward, angle)])
                end0 = tp.run(p, base, self.r, step=1e6)[-1]
                for target in targets:
                    diagonal = int(round((target - side *
                        (end0.y - course.obs2_y)) / diagonal_y))
                    for middle in (diagonal - 1, diagonal, diagonal + 1):
                        if not 0 <= middle <= 80:
                            continue
                        head = (_straight(lead) + [Seg(outward, angle)]
                                + _straight(middle) + [Seg(inward, angle)])
                        end = tp.run(p, head + [Seg(inward, final)],
                                     self.r, step=1e6)[-1]
                        if math.cos(p.th) < 0.5:
                            continue
                        for gap in range(int(tp.IR_MIN_GAP_CM + 4),
                                         int(BACK_GAP_MAX_CM) + 1, 4):
                            forward = int(round((back + gap - end.x)
                                                / math.cos(p.th)))
                            if 0 <= forward <= 60:
                                segs = head + _straight(forward) + [Seg(inward, final)]
                                options.append((tp.motion_cost(segs, self.r), segs))
        seen = set()
        for _, segs in sorted(options, key=lambda item: item[0]):
            key = tuple(segs)
            if key not in seen:
                seen.add(key)
                yield segs

    def same_side_course(self, course: Course) -> Course:
        """Couple obstacle width and wall position for the outer-eight test.

        The generic pre-measurement course deliberately combines the widest
        obstacle with the nearest wall, even though those cannot occur
        together. If ultrasound hit obstacle 2 from y, its half-width reaches
        at least |y|; under the brief the wall is then another 50 cm out.
        Wider real obstacles also move that wall farther out, so this lower
        bound is conservative for the outward sweep.
        """
        if self.obs2_us_y is None:
            return course
        # The legal side boundary is 50 cm beyond the obstacle edge.  A US
        # ray proves only that the face reaches that ray; it cannot shrink a
        # wider configured/observed obstacle and drag the boundary inward.
        width = max(course.w2, 2.0 * self._obs2_known_half(course))
        wall_y = width / 2.0 + tp.WALL_CLEARANCE_CM
        return Course(course.x1, course.x2, width,
                      obs1_y=course.obs1_y, walls=course.walls,
                      wall_y=wall_y,
                      obs2_y=course.obs2_y)

    def same_side_courses(self, course: Course) -> Iterator[Course]:
        """All obstacle widths compatible with the measurements so far.

        A US hit at y only supplies a lower width bound. Check the outer
        eight against every wider brief width, coupling each obstacle width
        to its corresponding wall position. A narrow-lane IR hit measures
        the edge itself and can use that width directly.
        """
        if self.obs2_near_y is not None:
            width = max(W2_RANGE[0], min(W2_RANGE[1],
                        2.0 * abs(self.obs2_near_y - course.obs2_y)))
            widths = [width]
        elif self.obs2_us_y is not None:
            minimum = 2.0 * self._obs2_known_half(course)
            # Wall clearance improves monotonically as width grows, while
            # obstacle clearance worsens. Their respective worst cases are
            # therefore the two endpoints; intermediate widths cannot add a
            # stricter constraint.
            maximum = W2_RANGE[1]
            if self.obs2_half_max is not None:
                maximum = min(maximum, 2.0 * self.obs2_half_max)
            widths = [minimum, max(minimum, maximum)]
        else:
            yield course
            return
        for width in sorted(set(widths)):
            wall_y = width / 2.0 + tp.WALL_CLEARANCE_CM
            yield Course(course.x1, course.x2, width,
                         obs1_y=course.obs1_y, walls=course.walls,
                         wall_y=wall_y,
                         obs2_y=course.obs2_y)

    def opposite_diagonal_candidates(self, p: Pose, course: Course,
                                     side1: float) -> Iterator[List[Seg]]:
        """Cross diagonally, then align with the front face toward side 2."""
        total = int(round(90 + side1 * math.degrees(p.th)))
        if not 2 * tp.MIN_ARC_DEG <= total <= 180 or math.cos(p.th) < 0.5:
            return
        inward = "R" if side1 > 0 else "L"
        # The actual width is not known yet. Keep the front-mounted side IR
        # over the guaranteed 30 cm face, not merely over the conservative
        # 90 cm planning box.
        minimum_width = W2_RANGE[0]
        options = []
        for angle in (30, 45):
            if total - angle < tp.MIN_ARC_DEG:
                continue
            for diagonal in range(4, 45, 4):
                tail = [Seg(inward, angle), Seg("F", diagonal),
                        Seg(inward, total - angle)]
                end0 = tp.run(p, tail, self.r, step=1e6)[-1]
                for gap in (FACE_GAP_CM, tp.IR_MIN_GAP_CM + 4, IR_FACE_MAX_CM - 4):
                    lead = int(round((course.x2 - tp.IR_SIDE_OFFSET_CM - gap
                                      - end0.x) / math.cos(p.th)))
                    if not 0 <= lead <= 60:
                        continue
                    segs = _straight(lead) + tail
                    end = tp.run(p, segs, self.r, step=1e6)[-1]
                    ir_y = end.y + tp.IR_FROM_AXLE_CM * math.sin(end.th)
                    # Finish before the opposite edge, leaving IR a visible
                    # stretch of face before it tracks that edge.
                    if -side1 * (ir_y - course.obs2_y) > minimum_width / 2 - 5:
                        continue
                    options.append((tp.motion_cost(segs, self.r), segs))
        for _, segs in sorted(options, key=lambda item: item[0]):
            yield segs

    def opposite_two_arc_candidates(self, p: Pose, course: Course,
                                    side1: float) -> Iterator[List[Seg]]:
        """Reach the front-face IR lane with two arcs instead of three."""
        heading = math.atan2(math.sin(p.th), math.cos(p.th))
        if abs(heading) > math.radians(15):
            return
        outward = "L" if side1 > 0 else "R"
        inward = _other(outward)
        total = int(round(90 + side1 * math.degrees(heading)))
        options = []
        for angle in range(15, 46, 3):
            for lead in (0, 2, 5):
                prefix = _straight(lead) + [Seg(outward, angle)]
                last = Seg(inward, total + angle)
                at = tp.run(p, prefix, self.r, step=1e6)[-1]
                end0 = tp.run(at, [last], self.r, step=1e6)[-1]
                for gap in (FACE_GAP_CM, FACE_GAP_CM + 2, IR_FACE_MAX_CM):
                    distance = int(round((course.x2 - tp.IR_SIDE_OFFSET_CM
                                          - gap - end0.x) / math.cos(at.th)))
                    if not 0 <= distance <= 100:
                        continue
                    segs = prefix + _straight(distance) + [last]
                    end = tp.run(p, segs, self.r, step=1e6)[-1]
                    ir_y = end.y + tp.IR_FROM_AXLE_CM * math.sin(end.th)
                    if abs(ir_y - course.obs2_y) > W2_RANGE[0] / 2.0 - 5.0:
                        continue
                    if self._face_handoff_endpoint_valid(
                            p, segs, course, side1, -side1, False):
                        options.append((tp.motion_cost(segs, self.r), segs))
        for _, segs in sorted(options, key=lambda item: item[0]):
            # Global cost ordering and collision checking happen once in the
            # selector; do not truncate this family to its first four joins.
            yield segs

    def face_handoff_candidates(self, p: Pose, course: Course, side1: float,
                                side2: float, straight_limit: int = 30
                                ) -> Iterator[Tuple[List[Seg], bool]]:
        """Include approach alternatives in the same swept-path validation."""
        if side1 != side2:
            # From the aimed image-two pose, first try the direct inward arc.
            # It turns immediately in the detected arrow direction and puts
            # the side IR either on the front face or already beyond the
            # requested edge.  The latter is now a valid zero-travel edge
            # handoff: follow_face() confirms three stationary no-face reads
            # and rounds the end without the old reverse/outward S detour.
            for segs in self.opposite_direct_candidates(p, course, side1):
                yield segs, False
            # Retain diagonal crossings as collision-checked fallbacks for a
            # camera pose from which the single inward arc genuinely cannot
            # clear the obstacle or the 50 cm side boundary.
            crossing = list(self.opposite_two_arc_candidates(
                p, course, side1))
            crossing.extend(self.opposite_diagonal_candidates(
                p, course, side1))
            for segs in crossing:
                yield segs, False
        approaches = [0]
        heading = math.atan2(math.sin(p.th), math.cos(p.th))
        if abs(heading) <= math.radians(15):
            reserve = max(2 * self.r, tp.FRONT_FROM_AXLE_CM + tp.CLEARANCE_CM)
            distance = int(math.floor(
                (course.x2 - self._front(p)) / math.cos(heading) - reserve))
            if distance >= 2:
                approaches.extend(sorted(set((distance // 2, distance))))
        prefixes = [_straight(distance) for distance in approaches]
        if side1 != side2 and abs(heading) <= math.radians(15):
            # A 90-degree front-face turn moves the axle one radius across
            # the centreline and puts the forward-mounted side IR another
            # IR_FROM_AXLE centimetres on. From the ordinary bypass lane that
            # beam can already be beyond a minimum-width (30 cm) obstacle,
            # producing only the IR's 40-60 cm background plateau. Move
            # outward first so the beam begins over the guaranteed face.
            visible_half = W2_RANGE[0] / 2.0 - 5.0
            entry_y = side1 * (self.r + tp.IR_FROM_AXLE_CM - visible_half)
            outward = tp.lane_change(heading, entry_y - p.y, self.r)
            if side1 * (entry_y - p.y) > 1.0 and outward:
                # A distant obstacle needs both operations: first place the
                # IR on a lane that intersects the minimum-width face, then
                # consume the safe longitudinal approach. Treating these as
                # separate alternatives left no valid candidate on long
                # courses and fell through to a face-missing shortcut.
                prefixes.extend(outward + _straight(distance)
                                for distance in reversed(approaches))
        # Vision may interrupt a photo arc before the robot is parallel.
        # Only after the original choices, try a forward heading recovery.
        # Its entire sweep stays part of the candidate, never a blind move.
        angle = int(round(abs(math.degrees(heading))))
        if tp.MIN_ARC_DEG <= angle <= 45:
            prefixes.append([Seg("L" if heading < 0 else "R", angle)])
        deferred = []
        seen = set()
        for prefix in prefixes:
            staged = tp.run(p, prefix, self.r, step=1e6)[-1] if prefix else p
            for segs, reverse in self._turn_handoff_candidates(
                    staged, course, side1, side2, straight_limit):
                combined = []
                for seg in prefix + segs:
                    # Adjacent collinear moves can be combined, including
                    # F39,R16 -> F23. The reduced sweep is checked below.
                    if (combined and seg.kind in ("F", "B")
                            and combined[-1].kind in ("F", "B")):
                        prev = combined.pop()
                        distance = ((prev.value if prev.kind == "F" else -prev.value)
                                    + (seg.value if seg.kind == "F" else -seg.value))
                        if distance:
                            combined.extend(_straight(distance))
                    else:
                        combined.append(seg)
                key = (tuple(combined), reverse)
                if key in seen:
                    continue
                seen.add(key)
                if reverse:
                    deferred.append((combined, reverse))
                else:
                    yield combined, reverse
        # Do not let a reverse choice at the original pose pre-empt a
        # forward route from another checked approach or alignment prefix.
        yield from deferred

    def _turn_handoff_candidates(self, p: Pose, course: Course, side1: float,
                                side2: float, straight_limit: int = 30
                                ) -> Iterator[Tuple[List[Seg], bool]]:
        """Yield ``(segments, reverse_face_run)`` in preferred order."""
        same_side = side2 == side1
        if same_side:
            # Keep enough forward reach for distant or wide courses. The
            # limit comes from the measured face/back geometry, not a fixed
            # motion sequence.
            straight_limit = max(straight_limit, int(math.ceil(
                course.x2 + tp.OBS2_DEPTH_CM + tp.IR_SIDE_OFFSET_CM
                + BACK_GAP_MAX_CM + self.r - p.x)))
            # After a measured outer arc, complete the maneuver from where the
            # chassis actually landed. The ordinary generator assumes a
            # parallel starting heading and cannot represent this recovery.
            for segs in self.same_side_continuation_candidates(
                    p, course, side2):
                yield segs, False
            # Prefer measuring the requested edge along the front face. A
            # direct route behind the box is a fallback, not the first choice.
            for segs in self.forward_front_candidates(p, course, side2):
                yield segs, False
            for segs in self.same_side_forward_candidates(
                    p, course, side2, straight_limit=max(60, straight_limit),
                    direct_only=True):
                yield segs, False
            # Once a direct single turn does not fit, prefer one continuous
            # outer-eight (straight, outward arc, straight, wide inward arc).
            # The lane family below adds another opposing-arc S before the
            # same final turn and was responsible for the repeated LL/RR
            # wobble in the physical traces.
            for segs in self.same_side_forward_candidates(
                    p, course, side2, straight_limit=max(60, straight_limit)):
                yield segs, False
            for segs in self.same_side_lane_candidates(p, course, side2):
                yield segs, False
        else:
            for segs in self.opposite_diagonal_candidates(p, course, side1):
                yield segs, False
        if same_side:
            return
        for segs in self.face_candidates(p, course, side1, same_side,
                                         straight_limit=straight_limit):
            yield segs, same_side

    def face_handoff_clearance(self, p: Pose, segs: Sequence[Seg],
                               course: Course, side1: float, side2: float,
                               reverse: bool) -> float:
        if not self._face_handoff_endpoint_valid(
                p, segs, course, side1, side2, reverse):
            return -float("inf")
        if side1 == side2 and not reverse:
            return min(self._check(p, segs, checked)
                       for checked in self.same_side_courses(course))
        return self._check(p, segs, course)

    def _handoff_boxes(self, courses: Sequence[Course]):
        """Obstacles used by a handoff before obstacle 2 is IR-measured."""
        return list(dict.fromkeys(
            box for checked in courses for box in self._course_boxes(checked)
            if box.name != "back fence"))

    def _face_handoff_endpoint_valid(self, p: Pose, segs: Sequence[Seg],
                                     course: Course, side1: float,
                                     side2: float, reverse: bool) -> bool:
        if side1 == side2 or reverse:
            return True
        end = tp.run(p, segs, self.r, step=1e6)[-1]
        target_heading = side2 * math.pi / 2.0
        heading_error = math.atan2(math.sin(end.th - target_heading),
                                   math.cos(end.th - target_heading))
        if abs(heading_error) > math.radians(HANDOFF_HEADING_TOL_DEG):
            return False
        ir_y = end.y + tp.IR_FROM_AXLE_CM * math.sin(end.th)
        # The edge scan must start over a face extent known to exist. Before a
        # sensor hit that is only the guaranteed 30 cm width. A US hit from
        # the bypass lane is stronger evidence: the face necessarily reaches
        # that ray, so a direct opposite-arrow turn may use that measured
        # extent without treating empty-floor IR values as obstacle evidence.
        known_half = self._obs2_known_half(course)
        on_known_face = (abs(ir_y - course.obs2_y)
                         <= known_half - FACE_ENTRY_MIN_VISIBLE_CM)
        # An opposite-arrow inward arc may legitimately finish just beyond
        # the requested end, especially after the measured camera heading is
        # folded into the turn.  Requiring its IR to remain over the face made
        # the 12:55 LR run reverse 33 cm, execute an outward S, and then miss
        # the face anyway.  The complete arc is collision checked; if the IR
        # is already beyond the guaranteed edge, the stationary no-face rule
        # in follow_face() turns around that end immediately.  If the real
        # obstacle is wider, the IR sees it and performs the normal live scan.
        requested_edge = side2 * (ir_y - course.obs2_y)
        already_past_edge = (
            requested_edge >= self._obs2_minimum_edge_half(course))
        return on_known_face or already_past_edge

    def select_face_handoff(self, p: Pose, course: Course, side1: float,
                            side2: float
                            ) -> Optional[Tuple[List[Seg], bool, float]]:
        """Bound planning time; never execute a partially checked candidate."""
        key = (p, course.x1, course.x2, course.w2, course.obs1_y,
               course.obs2_y, course.walls, course.wall_y, side1, side2,
               self.obs2_us_y, self.obs2_half_max, self.obs2_near_y,
               self.obs2_front_max, self.r, self.fast)
        if self._handoff_cache is not None and self._handoff_cache[0] == key:
            return self._handoff_cache[1]
        started = time.monotonic()
        # Default: finish enumeration and check in increasing cost order.
        # A deadline may return a fully checked fallback. Optimality is not
        # safety: an untested cheaper candidate cannot invalidate a safe one.
        budget = max(0.0, tp._env_f("T2_HANDOFF_PLAN_BUDGET_S", 0.0))
        deadline = started + budget if budget else float("inf")
        minimum = max(tp.CLEARANCE_CM, HANDOFF_CONTINUOUS_CLEARANCE_CM)
        options, seen = [], set()
        checked = 0
        fallback = None
        timed_out = False
        candidate_courses = [course]
        if self.obs2_front_max is not None and self.obs2_front_max > course.x2:
            candidate_courses.append(Course(
                course.x1, self.obs2_front_max, course.w2,
                obs1_y=course.obs1_y, walls=course.walls,
                wall_y=course.wall_y, obs2_y=course.obs2_y))
        try:
            candidates = (item for candidate_course in candidate_courses
                          for item in self.face_handoff_candidates(
                              p, candidate_course, side1, side2, straight_limit=60))
            for segs, reverse in candidates:
                if time.monotonic() >= deadline:
                    raise TimeoutError
                signature = (tuple(segs), reverse)
                if signature in seen:
                    continue
                seen.add(signature)
                if not self._face_handoff_endpoint_valid(
                        p, segs, course, side1, side2, reverse):
                    continue
                options.append((tp.motion_cost(segs, self.r, fast=self.fast),
                                tuple(seg.token() for seg in segs), segs, reverse))
            def forward_only(item):
                return not item[3] and all(seg.kind in ("F", "L", "R")
                                          for seg in item[2])

            def opposite_crossing(item):
                if side1 == side2:
                    return False
                turn_indexes = [index for index, seg in enumerate(item[2])
                                if seg.kind in ("L", "R")]
                if len(turn_indexes) < 2:
                    return False
                # A real side change contains a visible diagonal between two
                # arcs.  Merely counting turns also promoted routes whose two
                # arcs were adjacent (or separated by a token F1/F2); those
                # looked on hardware like a long straight followed by a late
                # turn.  Keep those only as geometric fallbacks.
                first, last = turn_indexes[0], turn_indexes[-1]
                return any(seg.kind == "F" and seg.value >= 8
                           for seg in item[2][first + 1:last])

            def opposite_direct(item):
                """Immediate one-arc turn from the angled image-two pose."""
                return (side1 != side2 and forward_only(item)
                        and sum(seg.kind in ("L", "R")
                                for seg in item[2]) == 1)

            options.sort(key=lambda item: (not opposite_direct(item),
                                           not opposite_crossing(item),
                                           not forward_only(item),
                                           item[0], item[1], item[3]))
            for cost, _, segs, reverse in options:
                if (fallback is not None and
                        (reverse or any(seg.kind not in ("F", "L", "R")
                                        for seg in segs))):
                    break
                courses = (list(self.same_side_courses(course))
                           if side1 == side2 and not reverse else [course])
                clearance = self._handoff_clearance(p, segs, courses, deadline)
                checked += 1
                if clearance + 0.01 < minimum:
                    if (fallback is None and not reverse
                            and all(seg.kind in ("F", "L", "R") for seg in segs)
                            and clearance >= HANDOFF_CLEARANCE_CM):
                        fallback = (segs, reverse, clearance, cost)
                    continue
                result = segs, reverse, clearance
                self._handoff_cache = key, result
                self.last_handoff_search = dict(candidates=len(options), checked=checked,
                    complete_enumeration=True, minimum_estimated_time_in_family=True,
                    estimated_s=cost, scope="handoff_only")
                logging.info("Forward-preferred handoff among %d generated candidates: "
                             "%s; %.2fs, %.2f cm clearance; %d candidates "
                             "checked in %.3fs.", len(options), ",".join(seg.token() for seg in segs),
                             cost, clearance, checked, time.monotonic() - started)
                return result
        except TimeoutError:
            timed_out = True
            if fallback is None:
                self.last_handoff_search = dict(candidates=len(options), checked=checked,
                    complete_enumeration=False, minimum_estimated_time_in_family=False,
                    scope="handoff_only", timed_out=True)
                logging.error("Handoff search timed out without a fully checked "
                              "route; no partial candidate will execute.")
                return None
            logging.warning("Handoff search timed out; retaining only the "
                            "completed, collision-checked fallback.")
        if fallback is not None:
            segs, reverse, clearance, cost = fallback
            result = segs, reverse, clearance
            self._handoff_cache = key, result
            self.last_handoff_search = dict(candidates=len(options), checked=checked,
                complete_enumeration=not timed_out,
                minimum_estimated_time_in_family=not timed_out,
                estimated_s=cost, scope="handoff_only", measured_fallback=True,
                timed_out=timed_out)
            logging.info("Forward handoff from measured pose %s uses the "
                         "contextual %.1f cm fallback: %s; %.2fs, %.2f cm "
                         "clearance.",
                         p, HANDOFF_CLEARANCE_CM,
                         ",".join(seg.token() for seg in segs), cost, clearance)
            return result
        self._handoff_cache = key, None
        self.last_handoff_search = dict(candidates=len(options), checked=checked,
            complete_enumeration=True, minimum_estimated_time_in_family=False,
            no_feasible_candidate=True, scope="handoff_only")
        logging.info("No checked handoff at %.1f cm from %s (%d candidates, %.3fs).",
                     minimum, p, checked, time.monotonic() - started)
        return None

    def select_short_same_side_recovery(
            self, p: Pose, course: Course, side: float
            ) -> Optional[Tuple[List[Seg], Tuple[List[Seg], bool, float], float]]:
        """Smallest straight reverse that restores an LL/RR forward handoff.

        This deliberately cannot retrace camera arcs.  LR/RL never call it,
        and the hard distance cap prevents a failed search from growing into
        the former full camera-path reversal.
        """
        courses = list(self.same_side_courses(course))
        # Leave a small sampling reserve above the hard floor. The planner's
        # 2 cm sweep can otherwise accept a tangent that the 1 cm execution
        # trace measures a few hundredths below the floor.
        recovery_floor = tp.CLEARANCE_FLOOR_CM + 0.25
        for distance in range(1, SAME_SIDE_RECOVERY_MAX_CM + 1):
            recovery = [Seg("B", distance)]
            recovered = tp.run(p, recovery, self.r, step=1e6)[-1]
            recovery_clearance = min(
                self._check(p, recovery, checked) for checked in courses)
            if recovery_clearance < recovery_floor:
                continue
            self._handoff_cache = None
            join = self.select_face_handoff(recovered, course, side, side)
            if (join is None or join[1]
                    or any(seg.kind not in ("F", "L", "R")
                           for seg in join[0])):
                continue
            combined_clearance = min(
                self._check(p, recovery + join[0], checked)
                for checked in courses)
            if combined_clearance < recovery_floor:
                continue
            return recovery, join, min(recovery_clearance,
                                       combined_clearance, join[2])
        return None

    def _handoff_clearance(self, p: Pose, segs: Sequence[Seg],
                           courses: Sequence[Course], deadline: float) -> float:
        # A circumscribed circle gives a lower distance bound. Skip exact
        # polygon checks only when that bound cannot lower the current minimum.
        boxes = self._handoff_boxes(courses)
        reach = math.hypot(max(tp.FRONT_FROM_AXLE_CM, tp.REAR_FROM_AXLE_CM),
                           tp.HALF_WIDTH_CM)
        # Reject impossible joins before sampling the full sweep. Passing
        # this inexpensive filter never replaces the full collision check.
        for pose in tp.run(p, segs, self.r, step=1e6):
            if time.monotonic() >= deadline:
                raise TimeoutError
            for box in boxes:
                dx = max(box.x0 - pose.x, pose.x - box.x1, 0.0)
                dy = max(box.y0 - pose.y, pose.y - box.y1, 0.0)
                if math.hypot(dx, dy) - reach >= tp.CLEARANCE_FLOOR_CM:
                    continue
                clearance = tp.clearance(pose, box)
                if clearance < tp.CLEARANCE_FLOOR_CM:
                    return clearance
        best = float("inf")
        for pose in tp.run(p, segs, self.r):
            if time.monotonic() >= deadline:
                raise TimeoutError
            for box in boxes:
                dx = max(box.x0 - pose.x, pose.x - box.x1, 0.0)
                dy = max(box.y0 - pose.y, pose.y - box.y1, 0.0)
                if math.hypot(dx, dy) - reach >= best:
                    continue
                best = min(best, tp.clearance(pose, box))
                if best < tp.CLEARANCE_FLOOR_CM:
                    return best
        return best

    def _execute_face_handoff(self, segs: Sequence[Seg], course: Course,
                              side1: float, side2: float,
                              reverse: bool,
                              planned_clearance: float) -> Optional[bool]:
        """Execute a checked handoff, retaining checkpoints where they matter."""
        original = list(segs)
        bounded: List[Seg] = []
        for seg in segs:
            limit = (HANDOFF_MAX_STRAIGHT_CM if seg.kind in ("F", "B")
                     else HANDOFF_MAX_ARC_DEG)
            parts = max(1, int(math.ceil(seg.value / limit)))
            total = int(round(seg.value))
            base, extra = divmod(total, parts)
            for index in range(parts):
                # Keep the total exact so token rounding cannot lengthen the
                # route merely because it was divided into safe primitives.
                value = base + (1 if index < extra else 0)
                bounded.append(Seg(seg.kind, value))
        segs = bounded
        was_bounded = len(segs) != len(original)

        courses = (list(self.same_side_courses(course))
                   if side1 == side2 and not reverse else [course])
        boxes = self._handoff_boxes(courses)
        required = (max(tp.CLEARANCE_FLOOR_CM,
                        min(HANDOFF_CLEARANCE_CM, planned_clearance - 1.5))
                    if side1 == side2 and not reverse
                    else tp.CLEARANCE_FLOOR_CM)

        # Queue any fully checked all-forward transition as one continuous
        # command. Opposite-arrow diagonals need this just as much as the
        # matching-arrow outer route; stopping between the two halves of an
        # S-curve amplified steering backlash in the 07:15 floor run.
        if (not reverse
                and all(seg.kind in ("F", "L", "R") for seg in segs)
                and planned_clearance + 0.01 >=
                HANDOFF_CONTINUOUS_CLEARANCE_CM and not was_bounded):
            chunks = [list(segs)]
        else:
            # A direction change still needs a physical stop, but preserve
            # continuous same-direction runs instead of checkpointing every
            # individual arc.
            chunks = []
            for seg in segs:
                forward = seg.kind in ("F", "L", "R")
                if (chunks and
                        ((chunks[-1][-1].kind in ("F", "L", "R")) == forward)):
                    chunks[-1].append(seg)
                else:
                    chunks.append([seg])

        def measured_pose() -> Optional[Pose]:
            measured = self.frame.pose()
            if isinstance(measured, Pose):
                return measured
            # Test/manual runners can provide the route pose directly without
            # constructing a WorldFrame. Production always takes this branch
            # only when the WPOSE read itself was unavailable.
            try:
                return self._pose()
            except (RuntimeError, TypeError):
                return None

        remaining = list(segs)
        for index, chunk in enumerate(chunks, 1):
            measured = measured_pose()
            if measured is None:
                logging.error("No measured pose before handoff group %d/%d; "
                              "movement withheld.", index, len(chunks))
                return False
            start = Pose(measured.x, measured.y, measured.th)
            if not self._face_handoff_endpoint_valid(
                    start, remaining, course, side1, side2, reverse):
                logging.error("Measured pose invalidates the obstacle-2 handoff "
                              "before %s.", ",".join(s.token() for s in chunk))
                return None
            clearance = tp.path_clearance(
                tp.run(start, remaining, self.r), boxes)[0]
            if clearance < required - 0.01:
                logging.error("Measured pose leaves only %.2f cm for the "
                              "remaining handoff %s (need %.1f); movement "
                              "withheld.", clearance,
                              ",".join(s.token() for s in remaining), required)
                return None
            logging.info("Handoff group %d/%d: %s; remaining measured "
                         "clearance %.2f cm.", index, len(chunks),
                         ",".join(s.token() for s in chunk), clearance)
            if not self._line(chunk):
                return False
            measured = measured_pose()
            if measured is None:
                logging.error("No measured pose after handoff group %s.",
                              ",".join(s.token() for s in chunk))
                return False
            self.th = measured.th
            del remaining[:len(chunk)]

        measured = measured_pose()
        if measured is None:
            return False
        end = Pose(measured.x, measured.y, measured.th)
        if not self._face_handoff_endpoint_valid(
                end, [], course, side1, side2, reverse):
            logging.error("Measured handoff endpoint does not put the side IR "
                          "over obstacle 2; face scan withheld.")
            return None
        return True

    def follow_face(self, token: str, x_face: float, facing: float,
                    toward: float, window: float,
                    check_front: bool = False,
                    minimum_edge_y: Optional[float] = None) -> Optional[Tuple[float, float]]:
        """
        Drive `token` (a straight, forward or reverse) along one face of
        obstacle 2 with the side IR on it, and RST where the face ends.

        x_face: the prior face estimate, used for the acquisition window.
        facing: +1 if the IR looks along +x at it (the front face), -1
        along -x (the back face). toward: which way along y the robot is
        going. window: a reading this far off the face's running median is
        no longer the face.

        Acquire a face in the expected range, then track its measured gap.
        Leaving the acquisition window alone is not an edge. A confirmed
        disappearance or substantial outward jump is required, and a known
        minimum face extent can veto an implausibly early edge.

        Returns (y where the face ends, the face's x as the IR measured it),
        both in odometry, or None.

        Wall guard: a wall can stand 50 cm past obstacle 2's end, and the end
        is at least as far on as the face the IR last saw. So whatever the
        readings do, the robot's leading edge never goes more than 44 cm (50
        less the 6 cm margin) past that - before anything is seen, past where
        the IR started, which was put in front of the face.
        """
        self.last_face_start_y = None
        self.last_stopped_pose = None
        p = self._pose()
        side = self._ir_facing(p.th, facing)
        ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)     # IR y - axle y
        forward = not token.upper().startswith("R")
        lead_off = tp.FRONT_FROM_AXLE_CM if forward else tp.REAR_FROM_AXLE_CM
        seen, lost, raw = [], [], []
        acquisition: List[float] = []
        face_gap_anchor: List[float] = []
        # The right IR on the floor can alternate between its real face gap,
        # an empty-floor plateau and blank after it passes an edge.  Counting
        # only consecutive misses lets one stale in-range sample erase the
        # whole edge.  Keep a three-sample majority instead; two consecutive
        # face readings are required to declare that the face was reacquired.
        edge_votes: List[bool] = []
        reacquired = 0
        ref = [p.y + ahead]                              # face known to here
        guard = []
        front_blocked = []
        extending = False
        # Empty floor can look like a 30-50 cm target, so the front face keeps
        # the strict cutoff.  Behind obstacle 2 the collision-checked turn has
        # already placed this IR beside the back face; floor testing measured
        # that face at 28-33 cm.  BACK_GAP_MAX_CM exists for precisely this
        # wider rear lane and must not be clipped by the front-face limit.
        expected_gap = facing * (
            x_face - (p.x + facing * tp.IR_SIDE_OFFSET_CM))
        if facing < 0:
            lo = tp.IR_MIN_GAP_CM - 4.0
            hi = min(BACK_GAP_MAX_CM, tp.IR_MAX_GAP_CM[side])
        else:
            lo = max(tp.IR_MIN_GAP_CM - 4.0,
                     expected_gap - FRONT_FACE_GAP_TOL_CM)
            hi = min(tp.IR_MAX_GAP_CM[side], FRONT_FACE_HARD_MAX_CM,
                     max(IR_FACE_MAX_CM,
                         expected_gap + FRONT_FACE_GAP_TOL_CM))
            logging.info("Front-face IR acquisition window %.0f..%.0f cm "
                         "from predicted %.1f cm gap.", lo, hi, expected_gap)

        def stop() -> bool:
            self.stm.abort()                             # RST: no reply comes
            self.last_stopped_pose = self._wait_for_stopped_pose()
            return True

        def poll(moving: bool = True) -> bool:
            nonlocal reacquired
            if moving:
                self.sleep_fn(SENSOR_POLL_S)
            # Protocol 10 returns this IR value and WPOSE from one control
            # tick.  Separate ?IR then ?WPOSE queries let the moving robot
            # travel between the two and produced false edge coordinates.
            g, adc, q = self._ir_pose_sample(side)
            if g is None and adc is not None and adc >= IR_TOO_CLOSE_RAW:
                front_blocked.append(g)
                logging.error("%s IR is below its calibrated range (raw %d); "
                              "treating it as too close, not as empty floor.",
                              side, adc)
                return stop() if moving else True
            if g is not None and (not math.isfinite(g)
                                  or g < tp.CLEARANCE_FLOOR_CM + IR_FACE_UNCERTAINTY_CM):
                front_blocked.append(g)
                logging.error("Face scan has invalid or insufficient side clearance: %s.", g)
                return stop() if moving else True
            if check_front or extending:
                distance = self._us()
                stop_gap = (FACE_EXTENSION_STOP_CM + tp.CLEARANCE_CM
                            if extending else 20.0)
                if distance is None or not math.isfinite(distance) or distance <= stop_gap:
                    front_blocked.append(distance)
                    logging.error("Forward face scan blocked by US reading %r cm.", distance)
                    return stop() if moving else True
            if q is None or not all(math.isfinite(v) for v in (q.x, q.y, q.th)):
                front_blocked.append(None)
                return stop() if moving else True
            raw.append(g)
            sensor_side = 1.0 if side == "left" else -1.0
            ray_dx = -sensor_side * math.sin(q.th)
            ray_dy = sensor_side * math.cos(q.th)
            ir_x = (q.x + tp.IR_FROM_AXLE_CM * math.cos(q.th)
                    + tp.IR_SIDE_OFFSET_CM * ray_dx)
            ir_y = (q.y + tp.IR_FROM_AXLE_CM * math.sin(q.th)
                    + tp.IR_SIDE_OFFSET_CM * ray_dy)
            if seen and g is not None and math.isfinite(g):
                surface = ir_x + ray_dx * g
                body_gap = min(facing * (surface - x) for x, _ in tp.footprint(q))
                # `surface` is reconstructed from the live IR measurement,
                # so adding IR_FACE_UNCERTAINTY_CM again double-counts that
                # uncertainty. The 05:48 floor run had eleven stable 11..13
                # cm face readings and a measured 5.7 cm body gap, but the
                # old 6.5 cm inflated threshold aborted it. Keep uncertainty
                # in route planning; enforce the physical floor on live data.
                if body_gap < tp.CLEARANCE_FLOOR_CM:
                    front_blocked.append(g)
                    logging.error("Face scan has only %.1f cm measured body clearance.", body_gap)
                    return stop() if moving else True
            edge_y = ir_y + (g if g is not None and math.isfinite(g)
                            else seen[-1][2] if seen else 0.0) * ray_dy
            steady = True
            edge_evidence = g is None
            if seen:                                     # the gap it has been reading
                recent = sorted(gap for _, _, gap in seen[-5:])
                baseline = recent[len(recent) // 2]
                steady = g is not None and abs(g - baseline) <= window
                # Acquisition limits reject background before a face is seen.
                # They are not an edge detector: 29 -> 42 -> 33 can still be
                # the same face. Only a substantial outward jump or no echo
                # votes for an edge; a closer surface never does.
                edge_baseline = (face_gap_anchor[0]
                                 if face_gap_anchor else baseline)
                edge_evidence = (g is None or (math.isfinite(g)
                                 and g > edge_baseline + window
                                 + IR_FACE_UNCERTAINTY_CM))
            tracking_hi = tp.IR_MAX_GAP_CM[side] if len(seen) >= 2 else hi
            if not seen and g is not None and math.isfinite(g):
                acquisition.append(g)
                del acquisition[:-3]
            # The predicted window remains the normal acquisition rule. If
            # odometry leaves the robot farther off the face, also accept two
            # consecutive, converged readings inside the calibrated hard
            # ceiling while moving. This distinguishes an approaching face
            # (for example 42, 33, 33) from the stationary 30-50 cm empty-floor
            # plateau, and lets the IR establish the real stopping geometry.
            converged_acquisition = (
                moving and len(acquisition) >= 2
                and lo <= acquisition[-1] <= FRONT_FACE_HARD_MAX_CM
                and lo <= acquisition[-2] <= FRONT_FACE_HARD_MAX_CM
                and abs(acquisition[-1] - acquisition[-2]) <= 3.0)
            valid_face = (g is not None and math.isfinite(g) and steady
                          and (not seen or not edge_evidence)
                          and (lo <= g <= tracking_hi
                               or converged_acquisition))
            if valid_face:
                if not seen and g > hi:
                    logging.info("%s IR acquired obstacle 2 at %.1f cm beyond "
                                 "the %.1f cm predicted window; using the live "
                                 "face for the edge stop.", side, g, hi)
                seen.append((edge_y, ir_x + ray_dx * g, g))
                if not face_gap_anchor:
                    face_gap_anchor.append(g)
                ref[0] = edge_y
                if edge_votes:
                    reacquired += 1
                    edge_votes.append(False)
                    del edge_votes[:-3]
                    if reacquired >= 2:
                        edge_votes.clear()
                        lost.clear()
                        reacquired = 0
                else:
                    lost.clear()
            elif seen and edge_evidence:
                # Two misses among the latest three readings confirm an edge.
                # One isolated dropout cannot trigger it, while the measured
                # blank/stale/blank IRR sequence brakes before wall clearance
                # is consumed by another sensor-poll interval.
                reacquired = 0
                lost.append(edge_y)
                edge_votes.append(True)
                del edge_votes[:-3]
                if len(seen) >= 2 and sum(edge_votes) >= 2:
                    return stop() if moving else True
            elif seen:
                # Ambiguous/closer returns cannot accumulate old edge votes.
                edge_votes.clear()
                lost.clear()
                reacquired = 0
            wall_reserve = FACE_EXTENSION_STOP_CM if extending else 0.0
            if toward * (q.y + toward * lead_off - ref[0]) > tp.WALL_CLEARANCE_CM - tp.CLEARANCE_CM - wall_reserve:
                guard.append(q.y)
                return stop() if moving else True
            return False

        def extension_length(remaining: int) -> int:
            # Only continue a forward scan that is still positively tracking
            # the face. No rear range sensor certifies a reverse extension.
            if not forward or len(seen) < 2 or lost or guard or front_blocked:
                return 0
            g = raw[-1] if raw else None
            if g is None or not math.isfinite(g) or not lo <= g <= hi:
                return 0
            q = self.frame.pose()
            distance = self._stable_us()
            if (q is None or not all(math.isfinite(v) for v in (q.x, q.y, q.th))
                    or distance is None or not math.isfinite(distance)
                    or toward * math.sin(q.th) < math.cos(math.radians(10))):
                logging.warning("No face extension: cannot confirm forward clearance "
                                "and along-face heading (US=%s, pose=%s).", distance, q)
                return 0
            # The live IR reconstructs the local face plane. Check the full
            # body against it, including the distance it may coast after RST.
            xs = sorted(x for _, x, _ in seen[-5:])
            face = xs[len(xs) // 2]
            body = tp.footprint(q)
            gap = min(facing * (face - x) for x, _ in body)
            need = tp.CLEARANCE_FLOOR_CM + IR_FACE_UNCERTAINTY_CM
            if gap < need:
                logging.warning("No face extension: measured body gap %.1f cm "
                                "is below %.1f cm including IR uncertainty.", gap, need)
                return 0
            limit = min(remaining, FACE_EXTENSION_BATCH_CM,
                        distance - FACE_EXTENSION_STOP_CM - tp.CLEARANCE_CM)
            closing = facing * math.cos(q.th)
            if closing > 1e-6:
                limit = min(limit, (gap - need) / closing - FACE_EXTENSION_STOP_CM)
            front_y = max(toward * y for _, y in body)
            limit = min(limit, (toward * ref[0] + tp.WALL_CLEARANCE_CM
                               - tp.CLEARANCE_CM - front_y)
                        / (toward * math.sin(q.th)) - FACE_EXTENSION_STOP_CM)
            return max(0, int(math.floor(limit)))

        logging.info("Along obstacle 2's %s face, %s IR on it: %s",
                     "front" if facing > 0 else "back", side, token)
        # A narrow back face can leave the beam before two moving polls.
        # Confirm what is already alongside us before beginning the scan.
        for _ in range(3):
            self.sleep_fn(0.03)
            if poll(moving=False):
                logging.error("Face scan cannot start: clearance/pose guard triggered.")
                return None
        logging.info("Face scan stationary acquisition: %d valid sightings on %s IR.",
                     len(seen), side)
        # The camera/route planner already tells us which end of obstacle 2
        # we are going round.  A calibrated arc can finish a little farther
        # along that direction than its ideal endpoint; in the 12:47 floor
        # run the axle stopped at y=-59.7 cm and the forward-mounted IR was
        # already near y=-76 cm, beyond the known minimum right edge.  Both
        # stationary reads were empty-floor values (56 and 65 cm), but the
        # old code still drove F36 because it refused to recognise an edge
        # until it had first acquired the face.  That extra travel is neither
        # useful nor safe.
        #
        # Only take this shortcut for the front face, when the caller supplied
        # a known minimum edge and the IR is physically at/past that edge in
        # the requested travel direction.  Require two definite no-face
        # readings (blank or beyond the calibrated hard ceiling), matching the
        # firmware edge filter, so one or two dropouts cannot cause an early
        # corner. If the IR did acquire the
        # face, the normal live two-of-three loss detector below remains in
        # charge and brakes at its measured edge.
        q = self.frame.pose()
        stationary_absent = (
            len(raw) >= 3
            and all(g is None or (math.isfinite(g)
                                  and g > FRONT_FACE_HARD_MAX_CM)
                    for g in raw[-3:]))
        if (facing > 0 and minimum_edge_y is not None and not seen
                and stationary_absent and q is not None
                and all(math.isfinite(v) for v in (q.x, q.y, q.th))):
            ir_y = q.y + tp.IR_FROM_AXLE_CM * math.sin(q.th)
            if toward * (ir_y - minimum_edge_y) >= 0:
                self.last_stopped_pose = q
                logging.info(
                    "%s IR is already past obstacle 2's known %s edge and "
                    "three stationary reads show no face; skipping %s and "
                    "turning around the end immediately.",
                    side, "left" if toward > 0 else "right", token)
                return minimum_edge_y, x_face

        # Let the STM observe every 10 ms control tick as well as the Pi's
        # slower polls. Its latch requires three consecutive in-window samples,
        # so a narrow face cannot pass between Linux serial queries unnoticed.
        latch_armed = False
        edge_setter = getattr(self.stm, "set_ir_edge", None)
        if (edge_setter is not None
                and getattr(self.stm, "supports_irw", None) is True):
            latch_armed = edge_setter(
                side, max(10, int(math.ceil(lo))),
                min(80, int(math.floor(hi))))
            if not latch_armed:
                logging.warning("Firmware IR edge latch was unavailable; "
                                "continuing with synchronized Pi polling.")
        reply = tp.run_polling(self.stm, token, poll)
        if reply is None or not (reply.startswith("OK") or reply == "STOPPED"):
            logging.error("STM answered %r along obstacle 2.", reply)
            return None

        # If the Pi missed a short acquisition entirely, recover the STM's
        # confirmed first hit and let the normal at-rest loss confirmation
        # establish that the beam has since passed the edge.
        if latch_armed and reply.startswith("OK") and not seen:
            edge_reader = getattr(self.stm, "read_ir_edge", None)
            latched = edge_reader(timeout=0.5) if edge_reader is not None else None
            if (latched is not None and latched.get("state") == "latched"
                    and latched.get("side") == side):
                g = float(latched["cm"])
                ql = self.frame.pose_from_wpose(latched)
                if ql is not None:
                    sensor_side = 1.0 if side == "left" else -1.0
                    ray_dx = -sensor_side * math.sin(ql.th)
                    ray_dy = sensor_side * math.cos(ql.th)
                    ir_x = (ql.x + tp.IR_FROM_AXLE_CM * math.cos(ql.th)
                            + tp.IR_SIDE_OFFSET_CM * ray_dx)
                    ir_y = (ql.y + tp.IR_FROM_AXLE_CM * math.sin(ql.th)
                            + tp.IR_SIDE_OFFSET_CM * ray_dy)
                    hit = (ir_y + g * ray_dy, ir_x + g * ray_dx, g)
                    # One latch represents at least three consecutive firmware
                    # ticks, satisfying the same acquisition confirmation as
                    # two independent Pi observations.
                    seen.extend((hit, hit))
                    face_gap_anchor.append(g)
                    ref[0] = hit[0]
                    logging.info("Recovered %s IR acquisition from firmware "
                                 "latch at %.1f cm.", side, g)
        # OK means the requested distance finished, not that the IR found an
        # edge. Confirm at rest first, then permit bounded sensor-led travel.
        remaining = FACE_EXTENSION_MAX_CM
        for _ in range(FACE_EXTENSION_MAX_CM // 2 + 1):
            if not (reply.startswith("OK") and len(seen) >= 2
                    and len(lost) < 2 and not guard and not front_blocked):
                break
            logging.info("Face scan completed; confirming its edge at rest "
                         "without additional movement.")
            for _ in range(3):
                self.sleep_fn(0.03)
                if poll(moving=False):
                    break
            distance = extension_length(remaining)
            if distance < 2:
                break
            extending = True
            remaining -= distance
            self._set_fast(False)
            if self.fast:
                logging.error("Precision speed was not acknowledged; face extension withheld.")
                return None
            logging.info("Face still present after %s: extending F%d with live "
                         "US/IR guards, %.0f cm stopping reserve, %d cm budget remaining.",
                         token, distance, FACE_EXTENSION_STOP_CM, remaining)
            reply = tp.run_polling(self.stm, f"F{distance}", poll)
            if reply is None or not (reply.startswith("OK") or reply == "STOPPED"):
                logging.error("STM answered %r during bounded face extension.", reply)
                return None
        failed = None
        if front_blocked:
            failed = "clearance unsafe or pose unavailable; not an obstacle edge"
        elif guard:
            failed = "stopped short of where a wall could be, never seeing its end"
        elif len(seen) < 2 or len(lost) < 2:
            failed = "never lost it" if seen else "never saw it"
        if failed:
            # The poll lines are filtered out of the log; say what it read.
            vals = sorted(g for g in raw if g is not None)
            logging.error(
                "The %s IR saw obstacle 2's face %d times and %s. It read %d "
                "times: %d blank, %s. Expected about %.0f cm if the face is at "
                "x=%.0f; it only counts %.0f..%.0f cm.", side, len(seen), failed,
                len(raw), len(raw) - len(vals),
                (f"min {vals[0]:.0f} / median {vals[len(vals) // 2]:.0f} / max "
                 f"{vals[-1]:.0f} cm") if vals else "no numbers",
                facing * (x_face - (p.x + facing * tp.IR_SIDE_OFFSET_CM)),
                x_face, lo, hi)
            return None
        if reply == "STOPPED" and self.last_stopped_pose is None:
            logging.error("Face edge was seen, but the robot did not settle; "
                          "not planning a corner from a moving pose.")
            return None
        # A reading describes the beam IR_LAG_CM back along the way it went.
        end_y = (seen[-1][0] + lost[0]) / 2.0 - toward * tp.IR_LAG_CM
        if (minimum_edge_y is not None
                and toward * (end_y - minimum_edge_y) < -5.0):
            logging.error("IR edge y=%.1f contradicts the previously observed "
                          "face extent y=%.1f; withholding the corner.",
                          end_y, minimum_edge_y)
            return None
        self.last_face_start_y = seen[0][0] - toward * tp.IR_LAG_CM
        xs = sorted(x for _, x, _ in seen)
        return end_y, xs[len(xs) // 2]

    @staticmethod
    def _ir_facing(th: float, facing: float) -> str:
        """The side IR looking along facing * x, at heading +-90."""
        return "right" if math.sin(th) * facing > 0 else "left"

    @staticmethod
    def _corner_boxes(course: Course):
        """Keep front/back turn clearance for either sign of IR range error."""
        return [tp.Box(b.x0 - IR_FACE_UNCERTAINTY_CM,
                       b.x1 + IR_FACE_UNCERTAINTY_CM, b.y0, b.y1, b.name)
                if b.name == "obstacle 2" else b for b in course.boxes()]

    def round_near_end(self, p: Pose, course: Course, toward: float,
                       reverse: bool, near_y: float,
                       forward_only: bool = False) -> Optional[List[Seg]]:
        """
        Round obstacle 2's `toward` end and on to behind it: two 90 deg turns
        the same way (clockwise for toward = +y), ending heading -toward*90
        with the IR on the back face within BACK_GAP_MAX_CM of it, ready to run along
        it. The shortest clear one wins.

        The transition is forward-only. Once the side IR has found the edge,
        the robot continues around that edge instead of steering toward the
        obstacle and reversing back out.

        near_y: where that end is. The IR, ahead of the axle, must start the
        back-face run no more than BACK_START_MAX_CM past it, or a 30 cm
        obstacle has gone by before the IR has seen enough of it.
        """
        same = "R" if toward > 0 else "L"
        boxes = [b for b in self._corner_boxes(course) if "fence" not in b.name]
        back = course.x2 + tp.OBS2_DEPTH_CM
        lo = back + tp.IR_SIDE_OFFSET_CM + tp.IR_MIN_GAP_CM + 4.0
        hi = back + tp.IR_SIDE_OFFSET_CM + BACK_GAP_MAX_CM

        clearances = {}

        def clear_at(segs, need):
            key = tuple(segs)
            if key not in clearances:
                clearances[key] = _path_clear(
                    p, segs, self.r, boxes, tp.CLEARANCE_FLOOR_CM)
            value = clearances[key]
            return value is not None and value >= need

        def firsts():
            # The edge has been measured and RST has settled. Continue around
            # that edge; never turn the wrong way and reverse back toward it.
            for a in range(0, 46, 2):
                if not reverse:
                    yield a, _straight(a) + [Seg(same, 90)]
                    continue
                # Retain compatibility with old reverse-face callers without
                # adding a backward movement to the edge transition itself.
                for th1 in (45, 30, 60, 0):
                    yield a, (_straight(a)
                              + ([Seg(same, th1)] if th1 else [])
                              + [Seg(_other(same), 90 - th1)])

        # The brief permits only 50 cm between obstacle 2 and the wall. With
        # the 30 cm TIGHT radius, a measured edge pose can leave about 5 cm
        # body clearance even though the nominal 6 cm preference cannot fit.
        # This is exactly what CLEARANCE_FLOOR_CM is for: retain collision
        # checking, but do not stop beside a cleared edge over one centimetre.
        needs = (max(TURN_CLEARANCE_CM, tp.CLEARANCE_CM),
                 tp.CLEARANCE_CM, tp.CLEARANCE_FLOOR_CM)
        options = []
        for a, first in firsts():
            for b in range(0, 80, 2):
                segs = first + _straight(b) + [Seg(same, 90)]
                end = tp.run(p, segs, self.r, step=1e6)[-1]
                ir_y = end.y + tp.IR_FROM_AXLE_CM * math.sin(end.th)
                if toward * (ir_y - near_y) < -BACK_START_MAX_CM:
                    break
                if end.x < lo:
                    continue
                if end.x > hi:
                    break
                options.append((tp.motion_cost(segs, self.r, fast=False), segs))
        options.sort(key=lambda item: item[0])
        for need in dict.fromkeys(needs):
            for _, segs in options:
                if clear_at(segs, need):
                    return segs
        return None

    def round_near_end_with_brake_recovery(
            self, p: Pose, course: Course, toward: float, reverse: bool,
            near_y: float) -> Optional[Tuple[List[Seg], List[Seg]]]:
        """Plan the measured corner, recovering only from RST coast.

        A long face scan can coast several centimetres after its IR first
        loses the edge. If that leaves no wall clearance for the first arc,
        move straight back along the already-cleared face and retry the same
        corner. This is not the old turn-toward-and-reverse route: no steering
        occurs until a complete corner has been collision checked.
        """
        corner = self.round_near_end(
            p, course, toward, reverse, near_y, forward_only=True)
        if corner is not None:
            return [], corner

        retreat_kind = "F" if reverse else "B"
        boxes = self._corner_boxes(course)
        fallback = None
        # The filtered IR can report an edge up to EDGE_LIMIT_RESERVE_CM late
        # and RST can add further coast. Search only the short distance needed
        # to recover those two bounded errors; this remains a straight move
        # along already-cleared face, never a camera-path reversal.
        recovery_max = min(24, int(math.ceil(
            EDGE_LIMIT_RESERVE_CM + CORNER_RECHECK_CM)))
        for distance in range(2, recovery_max + 1, 2):
            retreat = [Seg(retreat_kind, distance)]
            recovered = tp.run(p, retreat, self.r, step=1e6)[-1]
            corner = self.round_near_end(
                recovered, course, toward, reverse, near_y,
                forward_only=True)
            if corner is None:
                continue
            # round_near_end() deliberately permits the configured positive
            # floor when the brief's 50 cm wall gap cannot provide the
            # preferred 8 cm. Requiring 8 cm again here rejected the exact
            # fallback it had just accepted and stranded the robot after RST.
            clearance = _path_clear(
                p, retreat + corner, self.r, boxes,
                tp.CLEARANCE_FLOOR_CM)
            if clearance is None:
                continue
            if clearance >= TURN_CLEARANCE_CM:
                return retreat, corner
            if fallback is None or clearance > fallback[0]:
                fallback = clearance, retreat, corner
        if fallback is not None:
            clearance, retreat, corner = fallback
            logging.warning("No %.1f cm edge recovery fits; using the safest "
                            "checked fallback at %.1f cm clearance.",
                            TURN_CLEARANCE_CM, clearance)
            return retreat, corner
        return None

    def execute_obstacle2_corner(self, segs: Sequence[Seg], course: Course) -> bool:
        """Execute a fully swept corner continuously when its margin permits.

        Tight fallbacks retain measured checkpoints. A corner with the full
        configured turn clearance is already checked as one swept path, so
        stopping between its arcs only adds settle time and steering backlash.
        """
        self._set_fast(False)
        if self.fast:
            logging.error("Obstacle 2 corner: precision speed unavailable; not moving.")
            return False
        boxes = self._corner_boxes(course)
        required = tp.execution_clearance(tp.CLEARANCE_CM)
        start = self._pose()
        initial_gap, _ = tp.path_clearance(tp.run(start, segs, self.r), boxes)
        continuous = (initial_gap + 0.01 >= TURN_CLEARANCE_CM
                      and all(seg.kind in ("F", "L", "R") for seg in segs))
        chunks = [list(segs)] if continuous else [[seg] for seg in segs]
        consumed = 0
        for i, chunk in enumerate(chunks):
            p = start if i == 0 else self._pose()
            if not all(math.isfinite(v) for v in (p.x, p.y, p.th)):
                logging.error("Obstacle 2 corner: invalid measured pose.")
                return False
            remaining = list(segs[consumed:])
            gap, name = tp.path_clearance(tp.run(p, remaining, self.r), boxes)
            if i == 0 and gap < required:
                if gap < tp.CLEARANCE_FLOOR_CM:
                    logging.error("Obstacle 2 corner has only %.1f cm "
                                  "clearance to %s; below the %.1f cm floor. "
                                  "Not moving.", gap, name,
                                  tp.CLEARANCE_FLOOR_CM)
                    return False
                required = tp.CLEARANCE_FLOOR_CM
                logging.warning("Obstacle 2 corner is using its checked %.1f "
                                "cm clearance to %s (%.1f cm floor).",
                                gap, name, required)
            if gap < required:
                logging.error("Obstacle 2 corner: remaining sweep has %.1f cm "
                              "clearance to %s; requires %.1f cm. Not sending %s.",
                              gap, name, required,
                              ",".join(seg.token() for seg in chunk))
                return False
            expected = tp.run(p, chunk, self.r)[-1]
            logging.info("Obstacle 2 corner %d/%d: %s, measured pose %s, "
                         "remaining clearance %.1f cm to %s (precision speed).",
                         i + 1, len(chunks),
                         ",".join(seg.token() for seg in chunk), p, gap, name)
            if not self._line(chunk):
                return False
            consumed += len(chunk)
            actual = self._pose()
            if not all(math.isfinite(v) for v in (actual.x, actual.y, actual.th)):
                logging.error("Obstacle 2 corner: invalid completion pose.")
                return False
            offset = math.hypot(actual.x - expected.x, actual.y - expected.y)
            angle = abs(math.degrees(math.atan2(math.sin(actual.th - expected.th),
                                                math.cos(actual.th - expected.th))))
            logging.info("Obstacle 2 corner %s measured error: %.1f cm / %.1f deg.",
                         ",".join(seg.token() for seg in chunk), offset, angle)
            actual_clearance, closest = tp.path_clearance([actual], boxes)
            if actual_clearance < tp.CLEARANCE_FLOOR_CM:
                logging.error("Corner completion has only %.1f cm to %s; "
                              "withholding further movement.", actual_clearance, closest)
                return False
            if (consumed == len(segs)
                    and (offset > CORNER_RECHECK_CM or angle > 8.0)
                    and any(b.name == "obstacle 2" for b in boxes)):
                if abs(math.cos(actual.th)) > math.sin(math.radians(8)):
                    logging.error("Large final-corner error without a square "
                                  "back-face view; withholding further movement.")
                    return False
                side = self._ir_facing(actual.th, -1.0)
                expected_gap = (actual.x - course.x2 - tp.OBS2_DEPTH_CM
                                + tp.IR_FROM_AXLE_CM * math.cos(actual.th))
                expected_gap = (expected_gap / abs(math.sin(actual.th))
                                - tp.IR_SIDE_OFFSET_CM)
                hits = [self._ir(side)]
                self.sleep_fn(SENSOR_POLL_S)
                hits.append(self._ir(side))
                if (any(g is None or not math.isfinite(g) or g <= 0
                        or g > min(BACK_GAP_MAX_CM, tp.IR_MAX_GAP_CM[side])
                        or abs(g - expected_gap) > IR_FACE_UNCERTAINTY_CM + 3
                        for g in hits) or abs(hits[0] - hits[1]) > 3):
                    logging.error("Large final-corner error: back-face %s IR %s "
                                  "does not confirm %.1f cm; withholding the scan.",
                                  side, hits, expected_gap)
                    return False
                logging.info("Large final-corner error revalidated against "
                             "back-face %s IR %s and body clearance.", side, hits)
            # Small errors use the measured pose for the next sweep. Large
            # final errors need the independent face confirmation above.
            if offset > 3.0 or angle > 5.0:
                logging.warning("Obstacle 2 corner deviated from its nominal "
                                "endpoint; completion clearance checked and "
                                "measured WPOSE retained for the next sweep.")
        return True

    def turn_home(self, p: Pose, course: Course, toward: float,
                  forward_only: bool = False
                  ) -> Optional[List[Seg]]:
        """
        From running along the back face past obstacle 2's far end: on (or
        back) and the last 90 deg turn, heading home HOME_GAP_CM off that end.
        """
        same = "R" if toward > 0 else "L"
        rotation = ((p.th - tp.HEADING_HOME) if same == "R"
                    else (tp.HEADING_HOME - p.th)) % (2 * math.pi)
        angle = int(round(math.degrees(rotation)))
        if not 45 <= angle <= 135:
            return None
        turn = Seg(same, angle)
        far = course.obs2_y - toward * course.w2 / 2.0
        y_home = far - toward * (tp.HALF_WIDTH_CM + HOME_GAP_CM)
        boxes = [b for b in self._corner_boxes(course) if "fence" not in b.name]
        end = tp.run(p, [turn], self.r)[-1]
        c = toward * (end.y - y_home)
        options = [c + d for d in (0, 3, 6, 9, -3)]
        if forward_only:
            options = sorted(set(max(0.0, cc) for cc in options))
        options = sorted(set(options), key=lambda cc:
                         tp.motion_cost(_straight(cc) + [turn], self.r, fast=False))
        for need in (TURN_CLEARANCE_CM, tp.CLEARANCE_CM - 0.01, tp.CLEARANCE_FLOOR_CM):
            for cc in options:
                segs = _straight(cc) + [turn]
                if _path_clear(p, segs, self.r, boxes, need) is not None:
                    return segs
        return None

    def turn_home_with_brake_recovery(self, p: Pose, course: Course,
                                       toward: float) -> Optional[List[Seg]]:
        segs = self.turn_home(p, course, toward, forward_only=True)
        if segs is not None:
            return segs
        # As at the near edge, RST coast can consume the wall-side turn room.
        # Retrace only a short straight, with retreat AND turn checked together.
        fallback = None
        for distance in range(2, 17, 2):
            retreat = [Seg("B", distance)]
            recovered = tp.run(p, retreat, self.r, step=1e6)[-1]
            corner = self.turn_home(recovered, course, toward, forward_only=True)
            if corner is None:
                continue
            clearance = _path_clear(
                p, retreat + corner, self.r, self._corner_boxes(course),
                tp.CLEARANCE_FLOOR_CM)
            if clearance is None:
                continue
            if clearance >= TURN_CLEARANCE_CM:
                logging.warning("Final edge coast recovery: R%d before the "
                                "home turn; full sweep keeps %.1f cm.",
                                distance, clearance)
                return retreat + corner
            # Match turn_home's accepted floor, as the near-edge wrapper does.
            # Prefer 8 cm; otherwise choose the largest verified clearance.
            if fallback is None or clearance > fallback[0]:
                fallback = clearance, retreat + corner
        if fallback is not None:
            clearance, segs = fallback
            logging.warning("Final edge recovery cannot keep %.1f cm; taking "
                            "the safest checked route %s at %.1f cm "
                            "(configured floor %.1f cm).",
                            TURN_CLEARANCE_CM,
                            ",".join(s.token() for s in segs), clearance,
                            tp.CLEARANCE_FLOOR_CM)
            return segs
        return None

    def around_obstacle2(self, course: Course, side1: float, side2: float
                         ) -> Optional[Course]:
        """
        Go round obstacle 2 in side2's direction, measuring both its ends on
        the way. Returns the measured course, or None.

          1. Turn on to its front face and run forward toward its side2 end.
          2. Round that end to behind it.
          3. Run along its back face: its far end.
          4. The last turn, heading home off that far end.

        Both ends are measured in odometry, which has drifted by the time the
        robot gets here, so nothing assumes obstacle 2 sits on the centreline.
        """
        p = self._pose()
        # Candidate sequences already include their straight leads. Validate
        # the approach AND turn from this pose before sending either one.
        selected = self.select_face_handoff(p, course, side1, side2)
        # LR/RL turn in the second arrow's direction directly from the camera
        # pose. Do not reverse or turn outward to manufacture a turn pocket;
        # stop if no collision-checked forward path exists.
        if ((selected is None or selected[1] or any(
                seg.kind not in ("F", "L", "R") for seg in selected[0]))
                and side1 == side2):
            short = self.select_short_same_side_recovery(
                p, course, side1)
            if short is not None:
                recovery, planned_join, clearance = short
                logging.warning(
                    "Same-side image-two turn needs the minimum checked "
                    "recovery %s (%.1f cm clearance; hard cap %d cm).",
                    ",".join(seg.token() for seg in recovery), clearance,
                    SAME_SIDE_RECOVERY_MAX_CM)
                if not self._line(recovery):
                    return None
                p = self._pose()
                self._handoff_cache = None
                planned_segs, planned_reverse, planned_clearance = planned_join
                measured_clearance = self.face_handoff_clearance(
                    p, planned_segs, course, side1, side2, planned_reverse)
                if measured_clearance >= tp.CLEARANCE_FLOOR_CM:
                    selected = (planned_segs, planned_reverse,
                                min(planned_clearance, measured_clearance))
                else:
                    selected = self.select_face_handoff(
                        p, course, side1, side2)
        if (selected is None or selected[1] or any(
                seg.kind not in ("F", "L", "R") for seg in selected[0])):
            logging.error("Measured diagonal photo pose has no all-forward "
                          "%s continuation; reversal is disabled.",
                          "same-arrow" if side1 == side2 else "opposite-arrow")
            return None
        def require_same_side_margin(candidate):
            # Candidate generation already rejects anything below the hard
            # collision floor. Do not force another lane change merely to
            # reach a preferred margin; that created the LL/RR snake motion.
            if (candidate is not None and side1 == side2
                    and candidate[2] < HANDOFF_CONTINUOUS_CLEARANCE_CM):
                logging.warning("Same-side handoff keeps %.2f cm; executing "
                                "the shortest collision-checked route.",
                                candidate[2])
            return candidate

        selected = require_same_side_margin(selected)
        if selected is None:
            logging.error("No checked forward obstacle-2 route available; not moving.")
            return None
        segs, reverse, clearance = selected
        if reverse or any(s.kind in ("B", "l", "r") for s in segs):
            logging.error("Obstacle-2 handoff rejected: reverse is disabled.")
            return None
        for handoff_attempt in range(3):
            logging.info("turn onto obstacle 2's face: clearance %.1f cm", clearance)
            executed = self._execute_face_handoff(
                segs, course, side1, side2, reverse, clearance)
            if executed is True:
                break
            if executed is False:
                return None
            if handoff_attempt >= 2:
                logging.error("Obstacle-2 handoff exceeded its measured "
                              "replan limit.")
                return None
            p = self._pose()
            logging.warning("Measured handoff drifted; replanning its remaining "
                            "path from %s (attempt %d/3).", p,
                            handoff_attempt + 2)
            selected = require_same_side_margin(
                self.select_face_handoff(p, course, side1, side2))
            if selected is None:
                logging.error("No checked continuation from the measured "
                              "handoff pose; face scan withheld.")
                return None
            segs, reverse, clearance = selected
            if reverse or any(s.kind in ("B", "l", "r") for s in segs):
                logging.error("Replanned obstacle-2 handoff requires reverse; "
                              "movement withheld.")
                return None
        if side2 == side1:
            logging.info("Same-side obstacle arrows: the post-recovery handoff "
                         "is forward-only.")

        # The ends have to be found accurately, not quickly.
        self._set_fast(False)
        if self.fast:
            logging.error("Obstacle 2 face scan: precision speed unavailable; not moving.")
            return None
        try:
            outer = (side2 == side1 and not reverse
                     and side2 * math.sin(self._pose().th) < 0)
            if outer:
                self.frame.needs_lateral_fix = True
                # The selected handoff is the complete all-forward outer
                # eight and already ends behind obstacle 2. Traverse its back
                # face now; the old front-face reverse is not part of this
                # branch.
                p = self._pose()
                ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)
                limit = course.obs2_y - side2 * (W2_RANGE[1] / 2.0 + 15.0)
                run = max(10.0, -side2 * (limit - (p.y + ahead)))
                got = self.follow_face(
                    f"F{int(round(run))}",
                    course.x2 + tp.OBS2_DEPTH_CM, -1.0, -side2, 12.0)
                if got is None or self.last_face_start_y is None:
                    return None
                far, back_x = got
                self.obs2_front_max = None
                near = self.last_face_start_y
                if self.obs2_near_y is not None:
                    near = self.obs2_near_y
                else:
                    # Entering the back face midway does not measure its near
                    # edge. The course puts the cuboid on the centreline;
                    # reflect the measured far edge, without inventing a
                    # lateral odometry correction from the first IR sighting.
                    near = 2.0 * course.obs2_y - far
                    logging.info("Outer-eight near edge inferred from the "
                                 "course centreline, not measured by IR: %.1f.",
                                 near)
                x2 = back_x - tp.OBS2_DEPTH_CM
                logging.info("Outer-eight measured obstacle 2 from its back: "
                             "%s-end bound y=%.1f, far end y=%.1f, front "
                             "x=%.1f.",
                             "left" if side2 > 0 else "right", near, far, x2)
            else:
                p = self._pose()
                limit = side2 * (W2_RANGE[1] / 2.0 + 15.0)   # IR past any end
                ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)
                run = max(10.0, side2 * (limit - (p.y + ahead)))
                got = self.follow_face(
                    f"{'R' if reverse else 'F'}{int(round(run))}",
                    course.x2, +1.0, side2, 18.0,
                    check_front=side1 == side2,
                    minimum_edge_y=(course.obs2_y + side2
                                    * self._obs2_minimum_edge_half(course)))
                if got is None:
                    return None
                near, x2 = got
                self.obs2_front_max = None
                # A narrow-lane acquisition may already have measured the
                # opposite edge. Both obstacle centres share the YOLO-derived
                # course axis, so mirror that live edge when the filtered
                # front-face scan reports this edge implausibly farther out.
                # This removes late IR/RST coast without guessing a width.
                if (self.obs2_near_y is not None
                        and side2 * (self.obs2_near_y - course.obs2_y) < 0):
                    mirrored = 2.0 * course.obs2_y - self.obs2_near_y
                    outward_error = side2 * (near - mirrored)
                    if outward_error > IR_FACE_UNCERTAINTY_CM:
                        logging.warning(
                            "Filtered IR edge %.1f cm is %.1f cm beyond the "
                            "mirrored live edge %.1f cm; using the mirrored "
                            "edge for the 50 cm boundary.",
                            near, outward_error, mirrored)
                        near = mirrored
                if abs(x2 - course.x2) > 8.0:
                    logging.warning("IR puts obstacle 2's face at x=%.1f, the "
                                    "ultrasound said %.1f - going by the IR.",
                                    x2, course.x2)
                logging.info("Obstacle 2's %s end at y=%.1f; face at x=%.1f "
                             "(IR; ultrasound said %.1f).",
                             "left" if side2 > 0 else "right", near, x2,
                             course.x2)

                near_y = near + side2 * END_MARGIN_CM
                wide = W2_RANGE[1] + 2 * END_MARGIN_CM
                nominal_wall = (course.wall_y if course.wall_y is not None
                                else course.w2 / 2.0
                                + tp.WALL_CLEARANCE_CM)
                wall_min = (near - tp.WALL_CLEARANCE_CM
                            + EDGE_LIMIT_RESERVE_CM
                            if side2 < 0
                            else course.obs2_y - nominal_wall)
                wall_max = (near + tp.WALL_CLEARANCE_CM
                            - EDGE_LIMIT_RESERVE_CM
                            if side2 > 0
                            else course.obs2_y + nominal_wall)
                partial = Course(
                    course.x1, x2, wide,
                    obs2_y=near_y - side2 * wide / 2.0,
                    wall_y=(wide / 2.0 + tp.WALL_CLEARANCE_CM
                            - 2 * END_MARGIN_CM),
                    wall_min_y=wall_min, wall_max_y=wall_max)
                p = self.last_stopped_pose or self._pose()
                recovered = self.round_near_end_with_brake_recovery(
                    p, partial, side2, reverse, near)
                if recovered is None:
                    logging.error("No clear way round obstacle 2's end from %s.", p)
                    return None
                retreat, segs = recovered
                if retreat:
                    logging.warning(
                        "RST coast left insufficient corner clearance; "
                        "recovering %s cm straight along the cleared face.",
                        retreat[0].value)
                    if not self._line(retreat):
                        return None
                    p = self._pose()
                    segs = self.round_near_end(
                        p, partial, side2, reverse, near,
                        forward_only=True)
                    if segs is None:
                        logging.error("Measured pose after brake recovery still "
                                      "has no clear obstacle-2 corner: %s.", p)
                        return None
                if not self.execute_obstacle2_corner(segs, partial):
                    return None

                self._set_fast(False)
                p = self._pose()
                ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)
                limit = near - side2 * (W2_RANGE[1] + 15.0)
                run = max(10.0, -side2 * (limit - (p.y + ahead)))
                got = self.follow_face(
                    f"F{int(round(run))}", x2 + tp.OBS2_DEPTH_CM,
                    -1.0, -side2, 12.0)
                if got is None:
                    return None
                far, _ = got
        finally:
            # The final turn still needs precision speed. Restore fast mode
            # only after that turn succeeds, not between two slow sections
            # or while unwinding a failed face scan.
            self._set_fast(False)

        near_y = near + side2 * END_MARGIN_CM
        far_y = far - side2 * END_MARGIN_CM
        w2 = abs(near_y - far_y)
        centre = (near + far) / 2.0
        logging.info("Obstacle 2 runs y=%.1f..%.1f: %.0f cm wide, centred %.1f "
                     "off the centreline (odometry).", min(near, far),
                     max(near, far), abs(near - far), centre)
        if not (W2_RANGE[0] - 10 <= abs(near - far) <= W2_RANGE[1] + 10):
            logging.warning("Measured obstacle-2 width %.1f cm is outside the "
                            "nominal range; retaining the two live IR edges.",
                            abs(near - far))
        # The brief puts obstacle 2 on the centreline, so where its centre
        # reads is odometry drift - ~3 m of it by now, and the IR fix on
        # obstacle 1 on the way home only looks a window either side of
        # where it expects. Take it out. Capped: more than that and it is
        # more likely the obstacle really is off the line.
        shift = -max(-Y_RESYNC_MAX_CM, min(Y_RESYNC_MAX_CM, centre))
        if abs(shift) >= 0.5:
            self.frame.y_shift += shift
            near_y, far_y = near_y + shift, far_y + shift
            logging.info("Odometry y moved %+.1f cm to put obstacle 2 on the "
                         "centreline.", shift)
        measured = Course(course.x1, x2, w2, obs2_y=(near_y + far_y) / 2.0,
                          wall_y=w2 / 2.0 + tp.WALL_CLEARANCE_CM - 2 * END_MARGIN_CM)
        p = self._pose()
        segs = self.turn_home_with_brake_recovery(p, measured, side2)
        if segs is None:
            logging.error("No clear last turn home from %s; obstacle 2 face "
                          "x=%.1f, width=%.1f, centre y=%.1f, wall offset=%.1f, "
                          "radius=%.1f cm, clearance floor=%.1f cm.",
                          p, measured.x2, measured.w2, measured.obs2_y,
                          measured.wall_y, self.r, tp.CLEARANCE_FLOOR_CM)
            return None
        logging.info("Round obstacle 2 to the %s; heading home: %s",
                     "left" if side2 > 0 else "right",
                     ",".join(s.token() for s in segs))
        if not self.execute_obstacle2_corner(segs, measured):
            return None
        self._set_fast(FAST)
        return measured

    # ── run ───────────────────────────────────────────────────────────────────

    def run(self) -> int:
        if self.startup and not self._startup():
            return 2
        if self.wait_for_go:
            self.input_fn("Robot in the bay, square, on the centreline. "
                          "Press Enter to GO. ")
            if self.startup:
                from communications import cal_profile
                if not cal_profile.zero_heading(self.stm):
                    return 2
        if not self.frame.anchor():
            logging.error("No ?WPOSE - flash firmware with protocol >= 6.")
            return 2
        t0 = time.monotonic()
        try:
            hit = self.obstacle1()
            if hit is None:
                return 1
            x1, side1 = hit
            x2 = self.pass_obstacle1(x1, side1)
            if x2 is None:
                return 1
            w2 = CONSERVATIVE_W2
            if x2 == NARROW:
                # Localise a narrow face before committing to the camera
                # turn, then use the measured width for its swept path.
                x2 = self.find_narrow(x1, side1)
                if x2 is None:
                    return 1
                if self.obs2_near_y is None and self.obs2_half_max is None:
                    w2 = CONSERVATIVE_W2
                elif self.obs2_near_y is None:
                    w2 = 2 * self.obs2_half_max
                else:
                    measured_half = (abs(self.obs2_near_y)
                                     + IR_FACE_UNCERTAINTY_CM)
                    w2 = max(W2_RANGE[0], min(
                        2 * (self.obs2_half_max or NARROW_HALF_MAX),
                        2 * measured_half))
            course = Course(
                x1, x2, w2, obs1_y=self.obs1_y,
                wall_y=w2 / 2.0 + tp.WALL_CLEARANCE_CM,
                obs2_y=self.obs1_y)
            # Always turn inward to the checked angled image-two pose before
            # asking for arrow two. There is no parallel-photo alternative.
            if not self.position_obstacle2_camera(course, side1):
                return 1
            side2 = self.photo_obstacle2(course, side1)
            # A head-on camera endpoint has no obstacle-1-side approach to
            # preserve. Start obstacle 2 in its own arrow's direction;
            # angled endpoints retain the side from which they were reached.
            approach_side = (side2 if self.stage2_centered else
                             side1)
            course = self.around_obstacle2(course, approach_side, side2)
            if course is None:
                return 1
            if not tp.ParkRunner(self.stm, self.frame, course).run():
                return 1
        except KeyboardInterrupt:
            logging.warning("Interrupted - RST.")
            self.stm.abort()
            return 2
        except RuntimeError as exc:
            logging.error("%s", exc)
            return 1
        logging.info("Task 2 route finished in %.1f s; caller must verify parking.",
                     time.monotonic() - t0)
        return 0

    def _startup(self) -> bool:
        from communications import cal_profile
        from communications.stm import WPOSE_MIN_PROTOCOL
        try:
            self.stm.connect()
        except Exception as exc:
            logging.error("Could not open the serial link: %s", exc)
            return False
        proto = cal_profile.firmware_protocol(self.stm)
        if proto is None or proto < WPOSE_MIN_PROTOCOL:
            logging.error("Firmware protocol %s; Task 2 needs >= %d (?WPOSE).",
                          proto, WPOSE_MIN_PROTOCOL)
            return False
        name = (os.getenv("T2T_CAL_PROFILE", "").strip()
                or os.getenv("T2_CAL_PROFILE", "").strip() or None)
        if not cal_profile.prepare_for_task(self.stm, name=name, required=True):
            logging.error("STM startup failed (see above) - not running Task 2.")
            return False
        from communications.stm import FAST_MIN_PROTOCOL, IRW_MIN_PROTOCOL
        self.stm.supports_irw = proto >= IRW_MIN_PROTOCOL
        if FAST and (proto < FAST_MIN_PROTOCOL or not self.stm.set_fast(True)):
            logging.error("Task 2 fast mode is unavailable; not starting.")
            return False
        self.fast = FAST
        return True


class _QuietPolling(logging.Filter):
    """The STM client logs every query; the IR watches poll ?IR and ?WPOSE ~50
    times a second, which buries everything else. Drop those lines."""

    def filter(self, record):
        msg = record.getMessage()
        return not (msg.startswith("STM → stm: ?IR")
                    or msg.startswith("STM → stm: ?WPOSE")
                    or msg.startswith("STM ← stm: IR,")
                    or msg.startswith("STM ← stm: WPOSE,"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--verbose", action="store_true",
                    help="log every ?IR / ?WPOSE poll too")
    if not ap.parse_args().verbose:
        for h in logging.getLogger().handlers:
            h.addFilter(_QuietPolling())
    from communications.stm import STM
    stm = STM()
    try:
        return Task2(stm).run()
    finally:
        try:
            stm.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
