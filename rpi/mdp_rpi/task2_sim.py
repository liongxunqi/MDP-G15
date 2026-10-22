"""
task2_sim.py - a fake STM for running task2.py on a laptop.

    python task2_sim.py 90 80 50            # d1 d2 w2, you type the arrow
    python task2_sim.py 60 60 90 --arrow right --radius-err 2 --plot

The fake answers the same calls task2.py makes on the real STM class. Motion
follows the same arc model the planner uses, optionally on a different radius
than the planner believes; the ultrasound and IRs are ray-cast against the
true course. Every pose the robot passes through is recorded, so the run can
be checked for contact with anything afterwards.
"""

import argparse
import math
import sys
from typing import List, Optional

import task2_park as tp
from task2_park import Box, Course, Pose, Seg


def _ray_box(ox, oy, dx, dy, b: Box) -> Optional[float]:
    t0, t1 = 0.0, float("inf")
    for o, d, lo, hi in ((ox, dx, b.x0, b.x1), (oy, dy, b.y0, b.y1)):
        if abs(d) < 1e-12:
            if not (lo <= o <= hi):
                return None
            continue
        a, c = (lo - o) / d, (hi - o) / d
        if a > c:
            a, c = c, a
        t0, t1 = max(t0, a), min(t1, c)
        if t0 > t1:
            return None
    return t0


class FakeSTM:
    IR_MAX = {"left": 80.0, "right": 65.0}

    def __init__(self, course: Course, start: Pose, radius: float,
                 cm_per_query: float = 1.5, us_obs2_err: float = 0.0,
                 drift_cm_per_m: float = 0.0):
        self.course = course
        self.boxes = course.boxes()
        self.pose = start
        self.origin = start
        self.radius = radius
        self.step = cm_per_query
        # ?US on obstacle 2 reads this much short (seen on the floor: 11 cm
        # from the obstacle 1 lane, the beam off the face's edge).
        self.us_obs2_err = us_obs2_err
        # ?WPOSE y wanders off the truth by this much per metre driven -
        # odometry drift, which only the IR fixes can see.
        self.drift = drift_cm_per_m
        self.odo = 0.0
        # The first query after a movement line collides with it, as on the
        # floor: the query is dropped and a RESEND joins the movement replies.
        self._fresh = False
        self._stray = False
        self.trace: List[Pose] = [start]
        self._pending: List[Pose] = []
        self._reply: Optional[str] = None

    # ── movement ──────────────────────────────────────────────────────────────
    def send_line(self, line: str) -> bool:
        assert self._reply is None, "line sent while another is outstanding"
        p = self.pose
        poses = []
        for tok in line.split(","):
            tok = tok.strip().upper()
            if tok.startswith("FU"):
                n = int(tok[2:])
                gap = self._us_true(p)
                travel = gap - (n - tp.US_BIAS_CM)
                travel = max(travel, -20.0)
                seg = [Seg("F" if travel >= 0 else "B", int(round(abs(travel))))]
            elif tok.startswith("FL") or tok.startswith("FR"):
                seg = [Seg("L" if tok[1] == "L" else "R", int(tok[2:]))]
            elif tok.startswith("F"):
                seg = [Seg("F", int(tok[1:]))]
            elif tok.startswith("R"):
                seg = [Seg("B", int(tok[1:]))]
            else:
                raise ValueError(f"fake STM cannot run {tok}")
            path = tp.run(p, seg, self.radius)
            poses += path[1:]
            p = path[-1]
        self._pending = poses
        self._reply = "OK"
        self._fresh = True
        return True

    def _advance(self, cm: float) -> None:
        n = max(1, int(round(cm)))
        moved, self._pending = self._pending[:n], self._pending[n:]
        for q in moved:
            self.odo += math.hypot(q.x - self.pose.x, q.y - self.pose.y)
            self.pose = q
        if moved:
            self.pose = moved[-1]
            self.trace += moved

    def receive(self) -> Optional[str]:
        if self._stray:
            self._stray = False
            return "RESEND"
        if self._pending:
            return None
        self._fresh = False
        r, self._reply = self._reply, None
        return r

    def wait_reply(self, timeout=None) -> Optional[str]:
        self._fresh = False
        self._advance(len(self._pending))
        r, self._reply = self._reply, None
        return r

    def abort(self) -> None:
        self._pending, self._reply = [], None
        self._fresh = self._stray = False

    # ── sensors ───────────────────────────────────────────────────────────────
    def _cast(self, ox, oy, th) -> Optional[float]:
        dx, dy = math.cos(th), math.sin(th)
        hits = [_ray_box(ox, oy, dx, dy, b) for b in self.boxes]
        hits = [t for t in hits if t is not None]
        return min(hits) if hits else None

    def _us_true(self, p: Pose) -> float:
        f = tp.straight(p, tp.FRONT_FROM_AXLE_CM)
        d = self._cast(f.x, f.y, p.th)
        return d if d is not None else 999.0

    def _ir(self, p: Pose, side: str) -> Optional[float]:
        s = tp.straight(p, tp.IR_FROM_AXLE_CM)
        k = 1.0 if side == "left" else -1.0
        ox = s.x - k * tp.IR_SIDE_OFFSET_CM * math.sin(p.th)
        oy = s.y + k * tp.IR_SIDE_OFFSET_CM * math.cos(p.th)
        d = self._cast(ox, oy, p.th + k * math.pi / 2)
        if d is None or d < 10.0 or d > self.IR_MAX[side]:
            return None
        return d

    def query_fields(self, cmd: str, timeout: float = 2.0):
        if self._fresh:
            self._fresh = False
            self._stray = True
            return None                     # the query timed out
        self._advance(self.step)            # time passes between queries
        cmd = cmd.upper().lstrip("?")
        p = self.pose
        if cmd == "US":
            d = self._us_true(p)
            f = tp.straight(p, tp.FRONT_FROM_AXLE_CM)
            b2 = self.course.obs2()
            if _ray_box(f.x, f.y, math.cos(p.th), math.sin(p.th), b2) is not None                     and abs(_ray_box(f.x, f.y, math.cos(p.th), math.sin(p.th), b2) - d) < 0.5:
                d -= self.us_obs2_err
            return [str(65535 if d > 400 else int(round(d + tp.US_BIAS_CM)))]
        if cmd == "IR":
            # The reading describes the robot IR_LAG_CM back up the trace.
            lag = int(round(tp.IR_LAG_CM))
            if self._pending and len(self.trace) > lag:      # moving only
                p = self.trace[-1 - lag]
            vals = [self._ir(p, "left"), self._ir(p, "right")]
            return [str(65535 if v is None else int(round(v))) for v in vals]
        raise ValueError(cmd)

    def read_wpose(self, timeout: float = 2.0):
        p, o = self.pose, self.origin
        c, s = math.cos(-o.th), math.sin(-o.th)
        dx, dy = p.x - o.x, p.y - o.y + self.drift * self.odo / 100.0
        return {"x_mm": int(round((c * dx - s * dy) * 10)),
                "y_mm": int(round((s * dx + c * dy) * 10)),
                "heading_deg": round(math.degrees(p.th - o.th), 1)}


def run_clearance(fake: "FakeSTM", course: Course):
    """Closest approach once the robot has left the bay (it starts 2 cm off
    the back fence by design)."""
    out = next((i for i, q in enumerate(fake.trace) if q.x > 0), len(fake.trace))
    return tp.path_clearance(fake.trace[out:], course.boxes())


def simulate(d1, d2, w2, arrow: str, radius_err: float = 0.0, quiet=True,
             us_obs2_err: float = 0.0, drift_cm_per_m: float = 0.0):
    """Run Task2 end to end on a fake course. Returns (code, fake, course)."""
    import logging
    import task2
    course = Course(x1=d1, x2=d1 + tp.OBS1_SIZE_CM + d2, w2=w2)
    frame = tp.WorldFrame(None)
    fake = FakeSTM(course, frame.start, tp.RADIUS_CM + radius_err,
                   us_obs2_err=us_obs2_err, drift_cm_per_m=drift_cm_per_m)
    if quiet:
        logging.disable(logging.CRITICAL)
    try:
        t = task2.Task2(fake, input_fn=lambda _: arrow, sleep_fn=lambda _: None,
                        startup=False, wait_for_go=False)
        code = t.run()
    finally:
        logging.disable(logging.NOTSET)
    return code, fake, course


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("d1", type=float)
    ap.add_argument("d2", type=float)
    ap.add_argument("w2", type=float)
    ap.add_argument("--arrow", choices=("left", "right"))
    ap.add_argument("--radius-err", type=float, default=0.0)
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()

    import task2
    course = Course(x1=a.d1, x2=a.d1 + tp.OBS1_SIZE_CM + a.d2, w2=a.w2)
    frame = tp.WorldFrame(None)
    fake = FakeSTM(course, frame.start, tp.RADIUS_CM + a.radius_err)
    ask = (lambda _: a.arrow) if a.arrow else input
    code = task2.Task2(fake, input_fn=ask, sleep_fn=lambda _: None,
                       startup=False, wait_for_go=False).run()
    clear, name = run_clearance(fake, course)
    print(f"exit {code}; closest approach {clear:.1f} cm ({name}); "
          f"parked at {fake.pose}")
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Rectangle
        fig, ax = plt.subplots(figsize=(11, 6))
        for b in course.boxes():
            ax.add_patch(Rectangle((b.x0, b.y0), b.x1 - b.x0, b.y1 - b.y0, color="0.3"))
        ax.plot([q.x for q in fake.trace], [q.y for q in fake.trace], "b-", lw=1)
        for q in fake.trace[::15] + [fake.trace[-1]]:
            ax.add_patch(Polygon(tp.footprint(q), fill=False, ec="tab:blue", lw=0.4))
        ax.set_aspect("equal")
        ax.set_title(f"d1={a.d1} d2={a.d2} w2={a.w2} arrow={a.arrow} "
                     f"r_err={a.radius_err}: closest {clear:.1f} cm ({name})", fontsize=8)
        fig.savefig("task2_sim.png", dpi=150, bbox_inches="tight")
        print("wrote task2_sim.png")
    return code


if __name__ == "__main__":
    sys.exit(main())
