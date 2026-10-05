"""HTTP API for the MDP algorithm simulator.

One endpoint, one job: given a start pose and a list of obstacles, plan the
shortest-time Hamiltonian path (checklist B.3) and hand back every step the
robot takes so the browser can animate it (checklist B.1/B.2).

Schema below is my own — not a copy of anything else's wire protocol —
since this server only ever needs to talk to `web/app.js` next to it.
"""

import os

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from arena import Arena, Facing, Obstacle, RobotPose
from hamiltonian import plan_route

app = FastAPI(title="MDP Algorithm Simulator")

# Local demo tool only — never deployed, so a permissive CORS policy is fine.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class ObstacleIn(BaseModel):
    id: int
    x: int
    y: int
    facing: Facing  # which side the image is on


class PoseIn(BaseModel):
    x: int = 0
    y: int = 0
    facing: Facing = Facing.N


class PlanRequest(BaseModel):
    obstacles: list[ObstacleIn]
    start: PoseIn = PoseIn()


class StepOut(BaseModel):
    action: str
    x: int
    y: int
    facing: Facing
    scanned_obstacle_id: int | None = None


class BlockedObstacleOut(BaseModel):
    id: int
    reason: str


class PlanResponse(BaseModel):
    visit_order: list[int]
    steps: list[StepOut]
    total_cost: float
    runtime_seconds: float
    blocked: list[BlockedObstacleOut] = []


@app.get("/api/health")
async def health():
    return {"status": "ok"}


@app.post("/api/plan", response_model=PlanResponse)
async def plan(request: PlanRequest):
    obstacles = [Obstacle(o.id, o.x, o.y, o.facing) for o in request.obstacles]
    arena = Arena(obstacles)
    start = RobotPose(request.start.x, request.start.y, request.start.facing)

    result = plan_route(arena, start)

    steps: list[StepOut] = [StepOut(action="START", x=start.x, y=start.y, facing=start.facing)]
    for obstacle_id, leg in zip(result.visit_order, result.legs):
        # Skip the leg's own "START" entry — it duplicates the previous leg's last pose.
        for action, pose in leg.steps[1:]:
            steps.append(StepOut(action=action, x=pose.x, y=pose.y, facing=pose.facing))
        steps.append(StepOut(action="SCAN", x=steps[-1].x, y=steps[-1].y, facing=steps[-1].facing,
                              scanned_obstacle_id=obstacle_id))

    return PlanResponse(
        visit_order=result.visit_order,
        steps=steps,
        total_cost=result.total_cost,
        runtime_seconds=result.runtime_seconds,
        blocked=[BlockedObstacleOut(id=b.id, reason=b.reason) for b in result.blocked],
    )


# Serve the static web UI (web/index.html etc.) from the same server, so
# `uvicorn app:app` is the only command needed to run the whole thing.
# Resolved relative to this file (not the current working directory) so it
# works no matter where `uvicorn` is launched from.
_WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "web")
app.mount("/", StaticFiles(directory=_WEB_DIR, html=True), name="web")
