# MDP Algorithm Simulator

An original implementation of the SC2079 MDP "Robot Path Planning Module"
(checklist items B.1–B.3), built directly from the course's own algorithms
briefing and assessment checklist — not adapted from another team's
codebase. Every design decision below is grounded in a specific slide,
with the reasoning spelled out so it's easy to explain in a demo/viva.

## Run it

```bash
cd server
python -m venv .venv
. .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
uvicorn app:app --reload
```

Open **http://127.0.0.1:8000** — the server serves the web UI itself, no
separate frontend process needed.

## How it maps to the checklist

- **B.1 (Robot Movement Area Simulator)** — `web/` displays the 200x200cm
  (20x20 cell) arena, the 40x40cm start zone, obstacles with their image
  facing, and animates the robot's position/facing step by step.
- **B.2 (Hamiltonian Path Computation)** — `server/hamiltonian.py` plans a
  route visiting every obstacle's viewpoint exactly once, animated on the
  same grid.
- **B.3 (Shortest-time Hamiltonian Path)** — the route is chosen by
  exhaustive search over every visiting order (briefing slide 16: "given
  that we have only 5 obstacles to visit, we can afford the cost of the
  exhaustive search"), scored by real pathfinding cost between every pair
  of stops, so the order returned is the cheapest one found, not a greedy
  approximation.

## Design decisions, and where they come from

| Decision | Source | Notes |
|---|---|---|
| 20x20 grid of 10cm cells | briefing slide 9, "option 2" | Simpler than a 5cm sub-grid; the briefing offers both, this is the one it walks through in most detail. |
| Robot = 3x3 cells, identified by bottom-left cell + facing | briefing slide 9 | The footprint box is the same 3x3 square regardless of facing — only which edge is the camera side changes (see the slide 9 diagram). That's what makes collision-checking simple (see below). |
| Obstacle viewpoint formula: e.g. South-facing image at (a,b) → robot at (a-1, b-(3+CAMERA_CLEARANCE)) facing North | briefing slide 10, worked example (slide used CAMERA_CLEARANCE=2) | The other 3 directions (`arena.py: viewpoint_for`) are my own derivation of the same idea — center the 3x3 footprint on the obstacle, back the camera edge off by `CAMERA_CLEARANCE` cells in the approach direction. Calibrated to our own sensor's measured ~20cm recognition distance (2 cells), matching the slide's own example value. |
| Collision check = does the robot's plain 3x3 box overlap any obstacle's box | briefing slide 36 ("consider the robot as a dot… virtual obstacle") | Because the footprint box doesn't rotate (see above), a direct axis-aligned box-overlap test is exact — no rotated-rectangle geometry needed, unlike a continuous-space model. |
| Exhaustive permutation search for visiting order | briefing slide 16 | Only a handful of obstacles in this task, so brute force is both simplest and provably optimal — no multiprocessing needed either, it's fast enough in plain Python. |
| Grid pathfinding (A\*) between two poses, not full Dubins curves | briefing slide 41–42 flags a simpler discrete alternative to Euclidean/Dubins nearest-neighbour scoring | I extended that spirit to the *real* motion too: 6 discrete moves (forward, backward, and 4 diagonal quarter-turns) over the same grid, rather than continuous-space circular arcs. Turn moves cost more than straight ones (`pathfinding.py: TURN_COST`), a simple tunable stand-in for "turning takes longer", which is what the *shortest-time* requirement (B.3) is about. |
| Turn moves cover `TURN_RADIUS_CELLS` (2) cells forward + 2 sideways, not 1 of each, approximated as two straight legs | our robot's own measured ~20cm turning radius | A 90° pivot isn't a 1-cell diagonal step at this radius — modeling it as a drive-then-swing L-shape (rather than a true arc) keeps both legs collision-checkable with the same box-overlap test, and lets the simulator animate the swept path instead of jumping straight to the end pose. |
| Robot cannot spot-turn | checklist A.4: "the robot cannot do on the spot turn" | Reflected directly in the motion model — there is no in-place-rotate move, only forward/backward-while-turning. |

## Layout

```
server/
  arena.py         grid, obstacle, robot-pose types; collision checks; viewpoint geometry
  pathfinding.py    A* between two poses
  hamiltonian.py    exhaustive visiting-order search
  app.py            FastAPI: POST /api/plan, serves web/ statically
web/
  index.html, style.css, app.js   canvas-based UI, no build step, no dependencies
```

## What this doesn't cover

This is the algorithm/simulator module only (checklist section B). The
Raspberry Pi (A.1–A.5) and Android (C.1–C.10) modules are separate
hardware/mobile deliverables outside this folder's scope.
