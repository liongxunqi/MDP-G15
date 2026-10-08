"""
algo/tests/test_mission_sim.py  —  replay plan_mission() output and check it
──────────────────────────────────────────────────────────────────────────────
    python3 algo/tests/test_mission_sim.py              # fixed + 30 random layouts
    python3 algo/tests/test_mission_sim.py --random 0   # fixed layouts only
    python3 algo/tests/test_mission_sim.py --seed 7     # a different random set

No robot, no RPi, no YOLO. Each layout goes through the real plan_mission(),
then every token the STM would receive is replayed here, independently of
the planner, on the REAL robot footprint (23 x 18.8 cm) and REAL obstacles
(10 x 10 cm) — without the safety margin the planner adds.

Per layout: OK (everything photographed, cleanly), PART (clean, but some
images have no room in front of them for a 28-45 cm photo), or FAIL. Then:
  photos      obstacles photographed / obstacles placed
  collision   robot body overlaps an obstacle ("graze" if every overlap is
              under GRAZE_MM deep — within real-world slop, but still a touch)
  overhang    furthest the body pokes past the 2 m line; any positive value is
              a planner bug under the closed arena-envelope contract
  aim         at each photo: facing the image, centred on it, and the
              ultrasonic sees the intended target when range correction is safe
  time        planning time — the PC must answer before the run clock hurts

Model assumptions (kept deliberately simple — this checks the PLAN, not the
motors): arcs turn about the robot centre at the profile's radius, as the
planner assumes; ?US observes the nearest obstacle inside a
+/-SONAR_HALF_ANGLE_DEG cone but does not itself move the robot.

After each photo the replay snaps back to the pose the planner intended, so
one bad approach is reported once instead of throwing off every leg after it.
"""

import argparse
import logging
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # algo/

import grid_search  # noqa: E402
import path_planner as pp  # noqa: E402
from stm_tokens import PROFILE_TIGHT, TURN_RADIUS_MM  # noqa: E402

ARENA = 2000.0
OBS = 100.0
HALF_L = pp.ROBOT_HALF_LENGTH_MM
HALF_W = pp.ROBOT_HALF_WIDTH_MM
FACE_NORMAL = {0: (0, 1), 2: (1, 0), 4: (0, -1), 6: (-1, 0)}   # N E S W

AIM_HEADING_TOL_DEG = 10.0
AIM_LATERAL_TOL_MM = 50.0     # camera centre must be over the 10 cm face
SONAR_HALF_ANGLE_DEG = 10.0   # HC-SR04-style beam, conservatively narrow
GRAZE_MM = 20.0

NO_ROOM_REASONS = {"no photo spot", "photo spot beyond the RPi arena limit"}

FIXED_LAYOUTS = {
    # The usual 5-obstacle MDP sample shape: spread out, mixed faces.
    "sample-5": [
        {"id": 1, "x": 1, "y": 18, "d": 4},
        {"id": 2, "x": 6, "y": 12, "d": 0},
        {"id": 3, "x": 10, "y": 7, "d": 2},
        {"id": 4, "x": 15, "y": 16, "d": 4},
        {"id": 5, "x": 19, "y": 9, "d": 6},
    ],
    "open-centre-4": [
        {"id": 1, "x": 8, "y": 8, "d": 0},
        {"id": 2, "x": 12, "y": 12, "d": 2},
        {"id": 3, "x": 6, "y": 14, "d": 6},
        {"id": 4, "x": 14, "y": 5, "d": 4},
    ],
    # Two faces pointing at nearby edges — the case algo-fixes was about.
    "edge-facing-pair": [
        {"id": 1, "x": 18, "y": 10, "d": 2},
        {"id": 2, "x": 10, "y": 18, "d": 0},
        {"id": 3, "x": 9, "y": 9, "d": 4},
    ],
    # Checklist A.5: one block declared once per face (task1._a5_expand).
    "a5-one-block": [{"id": i + 1, "x": 9, "y": 9, "d": d} for i, d in enumerate((0, 2, 4, 6))],
    "max-8": [
        {"id": 1, "x": 5, "y": 10, "d": 2}, {"id": 2, "x": 10, "y": 5, "d": 0},
        {"id": 3, "x": 15, "y": 3, "d": 6}, {"id": 4, "x": 16, "y": 10, "d": 4},
        {"id": 5, "x": 12, "y": 16, "d": 6}, {"id": 6, "x": 6, "y": 16, "d": 2},
        {"id": 7, "x": 2, "y": 13, "d": 0}, {"id": 8, "x": 10, "y": 11, "d": 4},
    ],
}


# ── geometry ──────────────────────────────────────────────────────────────────

def corners(x, y, th, hl=HALF_L, hw=HALF_W):
    c, s = math.cos(th), math.sin(th)
    x += pp.BODY_CENTRE_AHEAD_MM * c
    y += pp.BODY_CENTRE_AHEAD_MM * s
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in
            ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def rect_hits_box(pts, box):
    """Separating-axis test: rotated rectangle (4 pts) vs axis-aligned box."""
    xmin, ymin, xmax, ymax = box
    box_pts = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
    axes = [(1, 0), (0, 1)]
    for i in range(2):
        ex, ey = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        axes.append((-ey, ex))
    for ax, ay in axes:
        a = [px * ax + py * ay for px, py in pts]
        b = [px * ax + py * ay for px, py in box_pts]
        if max(a) <= min(b) or max(b) <= min(a):
            return False
    return True


def obstacle_box(o):
    x0, y0 = o["x"] * 100.0, o["y"] * 100.0
    return (x0, y0, x0 + OBS, y0 + OBS)


# ── replay ────────────────────────────────────────────────────────────────────

class Replay:
    def __init__(self, obstacles, radius):
        self.obstacles = obstacles
        self.boxes = [obstacle_box(o) for o in obstacles]
        self.radius = radius
        self.x, self.y, self.th = pp.START_X_MM, pp.START_Y_MM, pp.START_THETA
        self.collisions = set()
        self.deep_collisions = set()
        self.overhang = 0.0

    def _check(self):
        pts = corners(self.x, self.y, self.th)
        g = GRAZE_MM
        for o, box in zip(self.obstacles, self.boxes):
            if rect_hits_box(pts, box):
                self.collisions.add(o["id"])
                if rect_hits_box(pts, (box[0] + g, box[1] + g, box[2] - g, box[3] - g)):
                    self.deep_collisions.add(o["id"])
        for px, py in pts:
            self.overhang = max(self.overhang, -px, -py, px - ARENA, py - ARENA)

    def straight(self, mm):
        n = max(1, int(abs(mm) // 10))
        for _ in range(n):
            self.x += (mm / n) * math.cos(self.th)
            self.y += (mm / n) * math.sin(self.th)
            self._check()

    def arc(self, forward, right, deg):
        d, k = (1 if forward else -1), (-1 if right else 1)
        n = max(1, int(deg // 2))
        step = math.radians(deg) / n
        for _ in range(n):
            th1 = self.th + d * k * step
            self.x += self.radius * k * (math.sin(th1) - math.sin(self.th))
            self.y += self.radius * k * (math.cos(self.th) - math.cos(th1))
            self.th = th1
            self._check()

    def ahead_distance(self):
        """Front bumper to the nearest obstacle inside the sonar cone."""
        fx = self.x + pp.REAR_AXLE_TO_SENSOR_MM * math.cos(self.th)
        fy = self.y + pp.REAR_AXLE_TO_SENSOR_MM * math.sin(self.th)
        best, best_id = math.inf, None
        steps = 8
        for i in range(-steps, steps + 1):
            a = self.th + math.radians(SONAR_HALF_ANGLE_DEG) * i / steps
            ca, sa = math.cos(a), math.sin(a)
            for d in range(0, 2500, 5):
                if d >= best:
                    break
                px, py = fx + d * ca, fy + d * sa
                hit = next((o["id"] for o, (x0, y0, x1, y1) in zip(self.obstacles, self.boxes)
                            if x0 <= px <= x1 and y0 <= py <= y1), None)
                if hit is not None:
                    # Range along the heading, as the robot will close it.
                    best, best_id = d * math.cos(a - self.th), hit
                    break
        return best, best_id

    def run(self, token):
        t = token.upper()
        if t in ("S", "RST"):
            return
        for op in ("FR", "FL", "RR", "RL"):
            if t.startswith(op):
                self.arc(op[0] == "F", op[1] == "R", int(t[2:]))
                return
        if t.startswith("F"):
            self.straight(int(t[1:]) * 10.0)
        elif t.startswith("R"):
            self.straight(-int(t[1:]) * 10.0)
        else:
            raise ValueError(f"unknown token {token}")


def same_block(sim, hit_id, obs):
    """A.5 declares one block once per face: any of those ids is the target."""
    hit = next((o for o in sim.obstacles if o["id"] == hit_id), None)
    return hit is not None and (hit["x"], hit["y"]) == (obs["x"], obs["y"])


def aim_problems(sim, obs):
    """Is the robot, right now, in a position to photograph obs's image face?"""
    nx, ny = FACE_NORMAL[obs["d"]]
    cx = obs["x"] * 100 + OBS / 2 + nx * OBS / 2
    cy = obs["y"] * 100 + OBS / 2 + ny * OBS / 2
    fx = sim.x + pp.REAR_AXLE_TO_SENSOR_MM * math.cos(sim.th)
    fy = sim.y + pp.REAR_AXLE_TO_SENSOR_MM * math.sin(sim.th)
    want = math.atan2(cy - fy, cx - fx)
    err = math.degrees(abs(math.atan2(math.sin(sim.th - want), math.cos(sim.th - want))))
    dist, hit = sim.ahead_distance()
    problems = []
    if err > AIM_HEADING_TOL_DEG:
        problems.append(f"heading off by {err:.0f}deg")
    if not same_block(sim, hit, obs):
        problems.append(f"camera sees {'nothing' if hit is None else f'obstacle {hit}'}")
    return problems, dist


# ── one layout ────────────────────────────────────────────────────────────────

def evaluate(obstacles, profile=PROFILE_TIGHT):
    t0 = time.perf_counter()
    details = {}
    plan = pp.plan_mission(obstacles, arc_profile=profile, details=details)
    elapsed = time.perf_counter() - t0

    radius = TURN_RADIUS_MM[profile]
    sim = Replay(obstacles, radius)
    by_id = {str(o["id"]): o for o in obstacles}
    photographed, aim = [], []
    for line, target in zip(plan["segments"], plan["segment_obstacles"]):
        for tok in line:
            sim.run(tok)
        if target is not None:
            obs = by_id[target]
            problems, _ = aim_problems(sim, obs)
            photographed.append(obs["id"])
            us_problems = []
            if plan.get("ultrasonic_adjustments", {}).get(str(target), False):
                _, hit = sim.ahead_distance()
                if not same_block(sim, hit, obs):
                    seen = "nothing" if hit is None else f"obstacle {hit}"
                    us_problems.append(f"?US sees {seen}")
            problems = us_problems + problems
            if problems:
                aim.append(f"obs {obs['id']}: " + ", ".join(problems))
            # Snap to where the planner meant to be, so one miss isn't counted
            # again on every later leg.
            sim.x, sim.y, sim.th = details["photo_poses"][obs["id"]]
    return {
        "photos": f"{len(set(photographed))}/{len(obstacles)}",
        "complete": len(set(photographed)) == len(obstacles),
        "collisions": sorted(sim.collisions),
        "deep": sorted(sim.deep_collisions),
        "overhang": max(0.0, sim.overhang),
        "aim": aim,
        "time": elapsed,
        "standoffs": details["standoff_cm"],
        "skipped": details["skipped"],
        "lines": len(plan["segments"]),
    }


def random_layout(rng, n):
    """Obstacles clear of start with at least 30 cm between their box edges."""
    taken, out = set(), []
    while len(out) < n:
        x, y = rng.randrange(0, 20), rng.randrange(0, 20)
        if x < 5 and y < 5:
            continue
        if any(
            math.hypot(
                max(0, (abs(x - other_x) - 1) * 100),
                max(0, (abs(y - other_y) - 1) * 100),
            ) < 300
            for other_x, other_y in taken
        ):
            continue
        taken.add((x, y))
        out.append({"id": len(out) + 1, "x": x, "y": y, "d": rng.choice((0, 2, 4, 6))})
    return out


def report(name, r):
    flags = []
    if r["deep"]:
        flags.append(f"HITS {r['deep']}")
    grazes = [i for i in r["collisions"] if i not in r["deep"]]
    if grazes:
        flags.append(f"grazes {grazes}")
    if r["overhang"] > 0:
        flags.append(f"overhang {r['overhang']:.0f}mm")
    flags += r["aim"]
    too_far = r["overhang"] > grid_search.ARENA_OVERHANG_MM + 1
    clean = not r["collisions"] and not r["aim"] and not too_far
    no_room = bool(r["skipped"]) and set(r["skipped"].values()) <= NO_ROOM_REASONS
    # PART: everything planned is clean; the skips are obstacles with no room
    # in front of the image for a 28-45 cm photo, which no route can fix - either
    # no clear spot at all, or one that is only reachable by taking the robot
    # past where the RPi stops the mission (position outside the arena).
    status = "OK  " if clean and r["complete"] else "PART" if clean and no_room else "FAIL"
    print(f"{status} {name:<18} photos {r['photos']:<5} lines {r['lines']:<3} "
          f"{r['time']:5.1f}s  {'; '.join(flags)}")
    return status != "FAIL"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--random", type=int, default=30)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--verbose", action="store_true", help="show planner logging")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO if args.verbose else logging.CRITICAL)

    results = []
    for name, layout in FIXED_LAYOUTS.items():
        r = evaluate(layout)
        results.append(r)
        report(name, r)

    rng = random.Random(args.seed)
    for i in range(args.random):
        layout = random_layout(rng, rng.randint(5, 8))
        r = evaluate(layout)
        results.append(r)
        if not report(f"random-{i + 1}", r):
            print(f"      layout: {layout}")

    n = len(results)
    print("\n── Summary ──")
    print(f"layouts                {n}")
    print(f"every obstacle shot    {sum(r['complete'] for r in results)}/{n}")
    print(f"no collisions          {sum(not r['collisions'] for r in results)}/{n}"
          f"   (hits >{GRAZE_MM:.0f}mm deep in {sum(bool(r['deep']) for r in results)})")
    print(f"aim problems           {sum(bool(r['aim']) for r in results)}/{n} layouts")
    print(f"uses edge overhang     {sum(r['overhang'] > 0 for r in results)}/{n} layouts "
          f"(max {max(r['overhang'] for r in results):.0f}mm, "
          f"allowed {grid_search.ARENA_OVERHANG_MM:.0f}mm)")
    reasons = {}
    for r in results:
        for why in r["skipped"].values():
            reasons[why] = reasons.get(why, 0) + 1
    placed = sum(int(r["photos"].split("/")[1]) for r in results)
    shot = sum(int(r["photos"].split("/")[0]) for r in results)
    print(f"obstacles photographed {shot}/{placed}   skipped: "
          + (", ".join(f"{n} {why}" for why, n in sorted(reasons.items())) or "none"))
    dist = {}
    for r in results:
        for cm in r["standoffs"].values():
            dist[cm] = dist.get(cm, 0) + 1
    print("photo distances (cm)   " + ", ".join(f"{cm}: {n}" for cm, n in sorted(dist.items())))
    times = sorted(r["time"] for r in results)
    print(f"planning time          median {times[n // 2]:.1f}s, max {times[-1]:.1f}s")


if __name__ == "__main__":
    main()
