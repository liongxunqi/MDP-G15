"""
task2mn_gordon_sim.py - run gordon (task2mn_gordon.py) and task2mn side by side
on the task2mn_sim fake STM, and price both runs with task2mn_park.motion_cost.

    python task2mn_gordon_sim.py 90 90 60 left right          # one layout, both routes
    python task2mn_gordon_sim.py --sweep                       # 108 layouts
    python task2mn_gordon_sim.py --sweep --drift 2 --drift -2  # and with odometry drift

What the fake adds to task2mn_sim.FakeSTM:
  - RST coasts on: RST_COAST_FAST_CM / _NORMAL_CM of the pending path is
    still driven (and collision-checked) after an abort.
  - ?IRW raw counts saturate when a side IR is closer than its 10 cm floor.
  - Every movement line is logged with how much of it actually ran.

ESTIMATED SECONDS for a run (the table's time columns):
    sum over executed primitives of task2mn_park.motion_cost (fast flag as
    sent; a primitive cut by RST is priced pro rata plus RST_STOP_S)
  + task2mn_park.LINE_OVERHEAD_S per movement line
  + background plans asked for at a stop before PLAN_LATENCY_CM of driving
    since they were started (standing still waiting for the thread)
  + the stage-2 vote wait: the arrow is confirmed VOTE_CM of driving after
    obstacle 1 leaves the camera view (1.1 s at cruise, the 2026-09-14 05:46
    floor log's wait); a route asking for it sooner waits out the rest
The obstacle-1 approach and arrow are identical in both routes and priced the
same way. Pi planning while the robot stands still is NOT in the estimate:
it is reported separately as laptop wall-clock ("idle plan s").

Background planning is modelled deterministically: a plan submitted while
the robot drives becomes available PLAN_LATENCY_CM of travel later.
"""

import argparse
import logging
import math
import statistics
import sys
import time
from typing import Dict, List, Optional

import task2mn_park as tp
import task2mn_sim as sim
from task2mn_park import Course, Seg

# Half-angle of the ultrasound's beam in the fake. 0 = task2mn_sim's single
# ray. The floor says wide: see TimedFakeSTM._us_true.
US_CONE_DEG = 15.0
US_FACING_DEG = 30.0
RST_COAST_FAST_CM = 8.0
RST_COAST_NORMAL_CM = 4.0
RST_STOP_S = 0.4
# Stage-2 vote: confirmed this far of driving after obstacle 1 leaves the
# camera view (1.1 s at the model's 34.5 cm/s cruise: the 05:46 floor log's
# wait). A route that stops before then waits out the rest.
VOTE_CM = 38.0
VOTE_CM_S = tp.FAST_CRUISE_CM_S
# Background planning time, as driving distance: ~0.9 s at cruise (the
# laptop's plan times run ~4x slower on a Pi 4).
PLAN_LATENCY_CM = 30.0

LAYOUT_D = (60, 90, 150)
LAYOUT_W = (30, 60, 90)
ARROWS = (("left", "left"), ("left", "right"), ("right", "left"), ("right", "right"))


def _token_segs(tok: str) -> List[Seg]:
    tok = tok.strip().upper()
    if tok.startswith("FL") or tok.startswith("FR"):
        return [Seg("L" if tok[1] == "L" else "R", int(tok[2:]))]
    if tok.startswith("RL") or tok.startswith("RR"):
        return [Seg("l" if tok[1] == "L" else "r", int(tok[2:]))]
    if tok.startswith("F"):
        return [Seg("F", int(tok[1:]))]
    if tok.startswith("R"):
        return [Seg("B", int(tok[1:]))]
    return []


class TimedFakeSTM(sim.FakeSTM):
    """protocol 10 too: a line with V/C tokens is driven along
    task2mn_gordon_link.fly_path with the fake's OWN steering-swing time and
    understeer (true_slew_s, true_understeer), so a planner that assumed the
    link's defaults sees the hand-overs land a little off, as on the floor."""

    def __init__(self, *args, coast: bool = True, true_slew_s: Optional[float] = None,
                 true_understeer: Optional[float] = None,
                 us_cone_deg: float = US_CONE_DEG, **kwargs):
        super().__init__(*args, **kwargs)
        self.us_cone_deg = us_cone_deg
        import task2mn_gordon_link as gl
        self.fast = True
        self.coast = coast
        self.true_slew_s = gl.STEER_SLEW_S * 1.25 if true_slew_s is None else true_slew_s
        self.true_understeer = (gl.UNDERSTEER_PER_100RPM * 1.5
                                if true_understeer is None else true_understeer)
        self.lines: List[dict] = []
        self._current: Optional[dict] = None
        self._idle_since: Optional[float] = None
        self.idle_s = 0.0

    def send_line(self, line) -> bool:
        import task2mn_gordon_link as gl
        if isinstance(line, (list, tuple)):
            line = ",".join(line)
        toks = [t.strip() for t in line.split(",") if t.strip()]
        moves = []
        for t in toks:
            try:
                moves.append(gl.parse_token(t))
            except ValueError:
                moves = None
                break
        p10 = bool(moves) and any(m.speed or m.carry for m in moves)
        if p10:
            assert self._reply is None, "line sent while another is outstanding"
            ok, why = gl.check_line(toks)
            if not ok:
                self._reply = "RESEND"          # the firmware's answer, nothing moves
                return True
            r_true = lambda v: gl.radius_at(v, self.radius, self.true_understeer)
            path = gl.fly_path(self.pose, moves, r_true, self.true_slew_s)
            self._pending = path[1:]
            self._reply = "OK"
            self._fresh = True
        else:
            ok = super().send_line(line)
        movement = [m.seg for m in moves] if moves else []
        if movement:
            if self._idle_since is not None:
                self.idle_s += time.perf_counter() - self._idle_since
                self._idle_since = None
            counts = []
            p = self.pose
            for s in movement:
                path = tp.run(p, [s], self.radius)
                counts.append(len(path) - 1)
                p = path[-1]
            if p10:
                counts = None               # priced by distance, not per primitive
            self._current = {"line": line, "segs": movement, "counts": counts,
                             "moves": moves, "p10": p10, "total": len(self._pending),
                             "fast": bool(self.fast), "done": 0,
                             "aborted": False}
            self.lines.append(self._current)
        return ok

    def _advance(self, cm: float) -> None:
        before = len(self._pending)
        super()._advance(cm)
        if self._current is not None:
            self._current["done"] += before - len(self._pending)

    def _finish(self):
        self._current = None
        self._idle_since = time.perf_counter()

    def wait_reply(self, timeout=None):
        r = super().wait_reply(timeout)
        if self._current is not None:
            self._finish()
        return r

    def receive(self):
        r = super().receive()
        if r is not None and r != "RESEND" and self._current is not None \
                and not self._pending:
            self._finish()
        return r

    def abort(self) -> None:
        if self._pending and self._current is not None:
            if self.coast:
                self._advance(RST_COAST_FAST_CM if self.fast else RST_COAST_NORMAL_CM)
            self._current["aborted"] = bool(self._pending)
            self._finish()
        super().abort()

    def _us_true(self, p) -> float:
        """The nearest echo anywhere in the beam's cone, not along one ray:
        on the floor a 30 cm obstacle 28 cm off the line read 93 cm, the same
        as a 60 cm one straight ahead (2026-10-15). A surface only answers if
        it faces the sensor (within US_FACING_DEG of its normal) or the ray
        lands on a corner - sound grazing a side wall reflects away."""
        if self.us_cone_deg <= 0:
            return super()._us_true(p)
        f = tp.straight(p, tp.FRONT_FROM_AXLE_CM)
        best = 999.0
        n = int(self.us_cone_deg)
        facing = math.cos(math.radians(US_FACING_DEG))
        for k in range(-n, n + 1):
            th = p.th + math.radians(k)
            dx, dy = math.cos(th), math.sin(th)
            for b in self.boxes:
                d = sim._ray_box(f.x, f.y, dx, dy, b)
                if d is None or d >= best:
                    continue
                hx, hy = f.x + d * dx, f.y + d * dy
                corner = (min(abs(hx - b.x0), abs(hx - b.x1)) < 2.0
                          and min(abs(hy - b.y0), abs(hy - b.y1)) < 2.0)
                # Which face was hit: the one whose plane the point lies on.
                on_x = min(abs(hx - b.x0), abs(hx - b.x1)) < 0.05
                normal_dot = abs(dx) if on_x else abs(dy)
                if corner or normal_dot >= facing:
                    best = d
        return best

    def query_fields(self, cmd: str, timeout: float = 2.0):
        out = super().query_fields(cmd, timeout)
        if out is not None and cmd.upper().lstrip("?") == "IRW":
            q = self._lagged() if self._pending else self.pose
            for i, side in enumerate(("left", "right")):
                if out[i] == "65535":
                    s = tp.straight(q, tp.IR_FROM_AXLE_CM)
                    k = 1.0 if side == "left" else -1.0
                    ox = s.x - k * tp.IR_SIDE_OFFSET_CM * math.sin(q.th)
                    oy = s.y + k * tp.IR_SIDE_OFFSET_CM * math.cos(q.th)
                    d = self._cast(ox, oy, q.th + k * math.pi / 2)
                    if d is not None and d < 10.0:
                        out[2 + i] = "3500"
        return out


def estimate(fake: TimedFakeSTM, arrow2_wait_s: float = 0.0) -> float:
    total = 0.0
    for rec in fake.lines:
        total += tp.LINE_OVERHEAD_S
        if rec["p10"]:
            total += _physics(rec)
            continue
        done = rec["done"]
        start = 0
        for s, n in zip(rec["segs"], rec["counts"]):
            if done >= start + n:
                total += tp.motion_cost([s], tp.RADIUS_CM, fast=rec["fast"])
            elif done > start:
                frac = (done - start) / max(1, n)
                total += frac * tp._cost([s], tp.RADIUS_CM, rec["fast"]) + RST_STOP_S
                break
            else:
                break
            start += n
    return total + arrow2_wait_s


def _physics(rec: dict) -> float:
    import task2mn_gordon_link as gl
    moves = rec["moves"] or [gl.Move(s) for s in rec["segs"]]
    return gl.line_seconds(moves, rec["fast"],
                           done_cm=float(rec["done"]) if rec["aborted"] else None)


def physics(fake: TimedFakeSTM, wait_s: float = 0.0) -> float:
    """The same run under task2mn_gordon_link.line_seconds' speed model (fast
    straights at the firmware's 200 rpm, ramps, per-stop settles) - for
    protocol 9 and 10 lines alike."""
    return sum(tp.LINE_OVERHEAD_S + _physics(r) for r in fake.lines) + wait_s


class SimFuture:
    def __init__(self, value, owner, start):
        self._value, self._owner, self._start = value, owner, start

    def _left_cm(self) -> float:
        return self._owner.latency - (self._owner.fake.odo - self._start)

    def done(self) -> bool:
        return self._left_cm() <= 0

    def result(self, timeout=None):
        # Asked for before it is ready: the robot waits (standing still, as
        # every result() call here follows a stop) for the rest.
        self._owner.wait_s += max(0.0, self._left_cm()) / VOTE_CM_S
        return self._value


class SimExecutor:
    """Background planning, deterministic: the result is computed at once but
    only handed over after `latency_cm` of driving."""

    def __init__(self, fake: TimedFakeSTM, latency_cm: float):
        self.fake, self.latency = fake, latency_cm
        self.wall_s = 0.0
        self.wait_s = 0.0

    def submit(self, fn, *args, **kwargs):
        t = time.perf_counter()
        value = fn(*args, **kwargs)
        self.wall_s += time.perf_counter() - t
        return SimFuture(value, self, self.fake.odo)

    def shutdown(self, wait=False):
        pass


def _parked(fake: TimedFakeSTM) -> dict:
    p = fake.pose
    front = p.x - tp.FRONT_FROM_AXLE_CM
    return {"y": p.y, "gap": front - (-tp.BAY_DEPTH_CM),
            "heading_err": tp.parking_heading_error(p.th)}


def simulate(d1, d2, w2, arrow1: str, arrow2: str, route_name: str = "gordon",
             drift: float = 0.0, radius_err: float = 0.0, quiet: bool = True,
             coast: bool = True, vote_cm: float = VOTE_CM,
             plan_latency_cm: float = PLAN_LATENCY_CM,
             us_cone_deg: float = US_CONE_DEG) -> dict:
    course = Course(x1=d1, x2=d1 + tp.OBS1_SIZE_CM + d2, w2=w2)
    frame = tp.WorldFrame(None)
    fake = TimedFakeSTM(course, frame.start, tp.RADIUS_CM + radius_err,
                        drift_cm_per_m=drift, coast=coast, us_cone_deg=us_cone_deg)
    arrows = iter((arrow1, arrow2))
    clock = {"gate": None, "asked2": None, "calls": 0}

    def ask(_):
        clock["calls"] += 1
        if clock["calls"] == 2:
            clock["asked2"] = fake.odo
        return next(arrows)

    def gate():
        if clock["gate"] is None:
            clock["gate"] = fake.odo
    if quiet:
        logging.disable(logging.CRITICAL)
    t0 = time.perf_counter()
    events = []
    ex = None
    try:
        if route_name in ("gordon", "gordon10"):
            import task2mn_gordon as gordon

            def peek():
                g = clock["gate"]
                return arrow2 if g is not None and fake.odo - g >= vote_cm else None

            ex = SimExecutor(fake, plan_latency_cm)
            task = gordon.GordonRoute(fake, input_fn=ask, sleep_fn=lambda _: None,
                                      startup=False, wait_for_go=False,
                                      lane_reached_fn=gate,
                                      arrow2_peek=peek, executor=ex)
        else:
            import task2mn_route as route
            task = route.Task2(fake, input_fn=ask, sleep_fn=lambda _: None,
                               startup=False, wait_for_go=False,
                               lane_reached_fn=gate)
        task.ir_latch = True
        task.fast = True
        if route_name.startswith("gordon"):
            task.p10 = route_name == "gordon10"
        code = task.run()
        if route_name.startswith("gordon"):
            events = list(task.events)
    finally:
        logging.disable(logging.NOTSET)
    wall = time.perf_counter() - t0
    clear, name = sim.run_clearance(fake, course)
    wait_s = 0.0
    if clock["asked2"] is not None:
        driven = clock["asked2"] - (clock["gate"] if clock["gate"] is not None
                                    else clock["asked2"])
        wait_s = max(0.0, vote_cm - driven) / VOTE_CM_S
    out = {"code": code, "clear": clear, "closest": name,
           "seconds": estimate(fake, wait_s) + (ex.wait_s if ex is not None else 0.0),
           "physics_s": physics(fake, wait_s + (ex.wait_s if ex is not None else 0.0)),
           "stops": sum(1 for r in fake.lines for m in (r["moves"] or [])
                        if not m.carry),
           "plan_wait_s": ex.wait_s if ex is not None else 0.0,
           "lines": len(fake.lines),
           "rst": sum(1 for r in fake.lines if r["aborted"]),
           "idle_s": fake.idle_s, "wall_s": wall, "wait_s": wait_s,
           "events": events, "fake": fake, "course": course}
    out.update(_parked(fake))
    return out


def _row(args) -> dict:
    (d1, d2, w2, a1, a2), routes, drift, kw = args
    row = {"d1": d1, "d2": d2, "w2": w2, "a1": a1, "a2": a2}
    for r in routes:
        try:
            row[r] = simulate(d1, d2, w2, a1, a2, r, drift=drift, **kw)
        except Exception as exc:          # report, keep sweeping
            row[r] = {"code": -1, "error": repr(exc), "clear": float("nan"),
                      "seconds": float("nan"), "y": float("nan"),
                      "lines": 0, "rst": 0, "idle_s": 0.0, "wait_s": 0.0,
                      "closest": "", "gap": float("nan"), "events": [],
                      "physics_s": float("nan"), "stops": 0}
        row[r].pop("fake", None)
        row[r].pop("course", None)
    return row


def sweep(drift: float = 0.0, routes=("baseline", "gordon"), jobs: int = 1,
          **kw) -> List[dict]:
    layouts = [(d1, d2, w2, a1, a2) for d1 in LAYOUT_D for d2 in LAYOUT_D
               for w2 in LAYOUT_W for a1, a2 in ARROWS]
    work = [(lay, routes, drift, kw) for lay in layouts]
    if jobs > 1:
        import multiprocessing
        with multiprocessing.Pool(jobs) as pool:
            return pool.map(_row, work)
    return [_row(w) for w in work]


def ok(res: dict) -> bool:
    return (res.get("code") == 0 and res["clear"] >= tp.CLEARANCE_FLOOR_CM
            and abs(res["y"]) <= 4.0)


def table(rows: List[dict], routes=("baseline", "gordon")) -> str:
    head = "| d1 | d2 | w2 | arrows |"
    sep = "|---:|---:|---:|---|"
    for r in routes:
        head += f" {r} s | {r} clear | {r} park y |"
        sep += "---:|---:|---:|"
    if len(routes) == 2:
        head += " saved s |"
        sep += "---:|"
    out = [head, sep]
    for row in rows:
        line = f"| {row['d1']} | {row['d2']} | {row['w2']} | {row['a1'][0].upper()}{row['a2'][0].upper()} |"
        for r in routes:
            res = row[r]
            mark = "" if ok(res) else " FAIL"
            line += (f" {res['seconds']:.1f}{mark} | {res['clear']:.1f} | "
                     f"{res['y']:+.1f} |")
        if len(routes) == 2:
            a, b = row[routes[0]], row[routes[1]]
            line += f" {a['seconds'] - b['seconds']:.1f} |"
        out.append(line)
    return "\n".join(out)


def summary(rows: List[dict], routes=("baseline", "gordon")) -> str:
    out = []
    for r in routes:
        res = [row[r] for row in rows]
        good = [x for x in res if ok(x)]
        secs = [x["seconds"] for x in good]
        phys = [x.get("physics_s", float("nan")) for x in good]
        out.append(
            f"{r}: {len(good)}/{len(res)} parked within 4 cm with >= 3.5 cm "
            f"clearance; min clearance {min(x['clear'] for x in res):.1f} cm; "
            f"est. {statistics.median(secs) if secs else float('nan'):.1f} s median, "
            f"{min(secs) if secs else float('nan'):.1f}-{max(secs) if secs else float('nan'):.1f} s; "
            f"speed model {statistics.median(phys) if phys else float('nan'):.1f} s median "
            f"({min(phys) if phys else float('nan'):.1f}-{max(phys) if phys else float('nan'):.1f}); "
            f"{statistics.mean(x.get('stops', 0) for x in res):.1f} stops, "
            f"{statistics.mean(x['lines'] for x in res):.1f} lines, "
            f"{statistics.mean(x['rst'] for x in res):.1f} RST, "
            f"idle planning {statistics.mean(x['idle_s'] for x in res):.2f} s (laptop) per run")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("layout", nargs="*", help="d1 d2 w2 arrow1 arrow2")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--drift", type=float, action="append", default=None)
    ap.add_argument("--radius-err", type=float, default=0.0)
    ap.add_argument("--only", choices=("baseline", "gordon", "gordon10"))
    ap.add_argument("--routes", default=None,
                    help="comma list of baseline,gordon,gordon10 (default the first two)")
    ap.add_argument("--jobs", type=int, default=1, help="parallel sweep workers")
    ap.add_argument("--json", help="also write the sweep rows to this file")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    routes = ((a.only,) if a.only else tuple(a.routes.split(",")) if a.routes
              else ("baseline", "gordon"))
    drifts = a.drift or [0.0]
    if a.sweep:
        dump = {}
        for drift in drifts:
            rows = sweep(drift, routes, jobs=a.jobs, radius_err=a.radius_err)
            dump[str(drift)] = rows
            print(f"\n## drift {drift:+.1f} cm/m\n")
            print(table(rows, routes))
            print()
            print(summary(rows, routes))
        if a.json:
            import json
            with open(a.json, "w", encoding="utf-8") as fh:
                json.dump(dump, fh, default=str)
        return 0
    if len(a.layout) != 5:
        ap.error("give d1 d2 w2 arrow1 arrow2, or --sweep")
    d1, d2, w2 = (float(v) for v in a.layout[:3])
    if a.verbose:
        logging.basicConfig(level=logging.INFO, format="%(message)s", force=True)
    for r in routes:
        for drift in drifts:
            res = simulate(d1, d2, w2, a.layout[3], a.layout[4], r, drift=drift,
                           radius_err=a.radius_err, quiet=not a.verbose)
            print(f"{r} drift {drift:+.1f}: exit {res['code']}, est {res['seconds']:.1f} s, "
                  f"physics {res['physics_s']:.1f} s, {res['stops']} stops, "
                  f"{res['lines']} lines, {res['rst']} RST, clearance {res['clear']:.1f} cm "
                  f"({res['closest']}), parked y {res['y']:+.1f} gap {res['gap']:.1f} "
                  f"hdg {res['heading_err']:.1f}; idle plan {res['idle_s']:.2f} s")
            for rec in res["fake"].lines:
                frac = rec["done"] / max(1, rec["total"] if rec["counts"] is None
                                         else sum(rec["counts"]))
                print(f"   {'F' if rec['fast'] else 'n'} {rec['line']}"
                      f"{'  (RST at %.0f%%)' % (100 * frac) if rec['aborted'] else ''}")
            if a.plot:
                _plot(res, f"task2mn_gordon_sim_{r}.png")
    return 0


def _plot(res: dict, path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle
    fake, course = res["fake"], res["course"]
    fig, ax = plt.subplots(figsize=(11, 6))
    for b in course.boxes():
        ax.add_patch(Rectangle((b.x0, b.y0), b.x1 - b.x0, b.y1 - b.y0, color="0.3"))
    ax.plot([q.x for q in fake.trace], [q.y for q in fake.trace], "b-", lw=1)
    for q in fake.trace[::15] + [fake.trace[-1]]:
        ax.add_patch(Polygon(tp.footprint(q), fill=False, ec="tab:blue", lw=0.4))
    ax.set_aspect("equal")
    ax.set_title(f"est {res['seconds']:.1f} s, closest {res['clear']:.1f} cm ({res['closest']})",
                 fontsize=8)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print("wrote", path)


if __name__ == "__main__":
    sys.exit(main())
