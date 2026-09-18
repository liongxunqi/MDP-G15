"use strict";

/* ---------- Constants (mirrors server/arena.py) ---------- */
const GRID_SIZE = 20;
const ROBOT_SIZE = 3;
const START_ZONE_SIZE = 4;
const API_BASE = ""; // same origin — server serves this page itself

const FACINGS = ["N", "E", "S", "W"];

// Mirrors server/pathfinding.py TURN_RADIUS_CELLS — used only for the
// animated arc drawn during playback; the planner's own cost/collision
// model is unaffected by this (it still reasons in the discrete two-leg
// waypoints the server already returns).
const TURN_RADIUS_CELLS = 2;
const FACING_ANGLE = { E: 0, N: Math.PI / 2, W: Math.PI, S: -Math.PI / 2 };

const PRESETS = {
  basic: [
    { id: 1, x: 5, y: 9, facing: "S" },
    { id: 2, x: 12, y: 9, facing: "N" },
    { id: 3, x: 8, y: 15, facing: "W" },
  ],
  spread: [
    { id: 1, x: 3, y: 16, facing: "S" },
    { id: 2, x: 9, y: 3, facing: "N" },
    { id: 3, x: 16, y: 6, facing: "W" },
    { id: 4, x: 6, y: 10, facing: "E" },
    { id: 5, x: 14, y: 15, facing: "S" },
  ],
  corners: [
    { id: 1, x: 4, y: 16, facing: "S" },
    { id: 2, x: 15, y: 16, facing: "W" },
    { id: 3, x: 4, y: 4, facing: "E" },
    { id: 4, x: 15, y: 4, facing: "N" },
    { id: 5, x: 10, y: 10, facing: "S" },
  ],
  diagonal: [
    { id: 1, x: 4, y: 4, facing: "E" },
    { id: 2, x: 8, y: 8, facing: "W" },
    { id: 3, x: 11, y: 11, facing: "S" },
    { id: 4, x: 14, y: 14, facing: "W" },
    { id: 5, x: 17, y: 17, facing: "W" },
  ],
  cluster: [
    { id: 1, x: 13, y: 9, facing: "S" },
    { id: 2, x: 7, y: 12, facing: "S" },
    { id: 3, x: 11, y: 10, facing: "S" },
    { id: 4, x: 12, y: 12, facing: "N" },
    { id: 5, x: 14, y: 11, facing: "S" },
  ],
};

/* ---------- State ---------- */
let obstacles = [];

let planSteps = null;   // [{action,x,y,facing,scanned_obstacle_id}]
let animSegments = null; // built from planSteps — see buildSegments()
let currentStep = 0;
let playTimer = null;      // truthy while Play is active (no longer an interval id)
let playState = null;      // { seg, startTs, arc? } — current in-flight segment
let animFrameHandle = null;
let renderPose = null;     // {x, y, angle} in box-center coords, set only mid-animation
let speedMultiplier = 1;

let selectedObstacleId = null;

// While an obstacle is being scanned: its id, and how far through the scan
// pause we are (0..1 while animating during Play; -1 means "paused/stepped
// here manually", which renders a fixed-size highlight instead of a pulse).
let highlightObstacleId = null;
let highlightProgress = -1;

/* ---------- DOM ---------- */
const canvas = document.getElementById("grid");
const ctx = canvas.getContext("2d");
const MARGIN = 30; // space reserved for axis number labels, left + bottom
const GRID_PX = canvas.width - MARGIN;
const CELL_PX = GRID_PX / GRID_SIZE;

const statusEl = document.getElementById("status");
const logEl = document.getElementById("log");
const stepLabelEl = document.getElementById("step-label");
const stepSlider = document.getElementById("step-slider");
const playBtn = document.getElementById("play-btn");
const stepBackBtn = document.getElementById("step-back-btn");
const stepFwdBtn = document.getElementById("step-fwd-btn");
const speedSelect = document.getElementById("speed-select");
const runBtn = document.getElementById("run-btn");
const clearBtn = document.getElementById("clear-btn");
const presetSelect = document.getElementById("preset");
const noSelectionHint = document.getElementById("no-selection-hint");
const selectedPanel = document.getElementById("selected-obstacle-panel");
const selectedIdEl = document.getElementById("selected-obstacle-id");
const facingButtons = document.querySelectorAll("#selected-obstacle-panel .facing-buttons button");
const removeObstacleBtn = document.getElementById("remove-obstacle-btn");
const startXInput = document.getElementById("start-x");
const startYInput = document.getElementById("start-y");
const startFacingSelect = document.getElementById("start-facing");

/* ---------- Grid <-> canvas coordinate conversion ----------
 * Grid (0,0) is bottom-left (per the briefing); canvas (0,0) is top-left,
 * so the y axis is flipped when drawing. The grid itself is inset by
 * MARGIN on the left to leave room for axis number labels. */
function cellToCanvas(x, y) {
  return { cx: MARGIN + x * CELL_PX, cy: (GRID_SIZE - 1 - y) * CELL_PX };
}
function canvasToCell(px, py) {
  const x = Math.floor((px - MARGIN) / CELL_PX);
  const y = GRID_SIZE - 1 - Math.floor(py / CELL_PX);
  return { x, y };
}

/* ---------- Drawing ---------- */
function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Start zone
  const style = getComputedStyle(document.documentElement);
  ctx.fillStyle = style.getPropertyValue("--start-zone").trim();
  ctx.fillRect(MARGIN, (GRID_SIZE - START_ZONE_SIZE) * CELL_PX, START_ZONE_SIZE * CELL_PX, START_ZONE_SIZE * CELL_PX);

  // Grid lines
  ctx.strokeStyle = style.getPropertyValue("--grid-line").trim();
  ctx.lineWidth = 1;
  for (let i = 0; i <= GRID_SIZE; i++) {
    ctx.beginPath();
    ctx.moveTo(MARGIN + i * CELL_PX, 0);
    ctx.lineTo(MARGIN + i * CELL_PX, GRID_PX);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(MARGIN, i * CELL_PX);
    ctx.lineTo(MARGIN + GRID_PX, i * CELL_PX);
    ctx.stroke();
  }

  drawAxisLabels(style);
  obstacles.forEach(drawObstacle);
  drawPath(style);

  const selected = obstacles.find((o) => o.id === selectedObstacleId);
  if (selected) drawSelectionRing(style, selected);

  if (renderPose) {
    drawRobotAt(renderPose.x, renderPose.y, renderPose.angle);
  } else if (planSteps && planSteps.length > 0) {
    const pose = planSteps[currentStep];
    drawRobot(pose.x, pose.y, pose.facing);
  } else {
    const sx = Number(startXInput.value) || 0;
    const sy = Number(startYInput.value) || 0;
    drawRobot(sx, sy, startFacingSelect.value);
  }
}

function drawAxisLabels(style) {
  ctx.fillStyle = style.getPropertyValue("--muted").trim();
  ctx.font = "9px sans-serif";

  // x axis: column numbers along the bottom, below the grid.
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let x = 0; x < GRID_SIZE; x++) {
    const { cx } = cellToCanvas(x, 0);
    ctx.fillText(String(x), cx + CELL_PX / 2, GRID_PX + 2);
  }

  // y axis: row numbers to the left of the grid.
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let y = 0; y < GRID_SIZE; y++) {
    const { cy } = cellToCanvas(0, y);
    ctx.fillText(String(y), MARGIN - 4, cy + CELL_PX / 2);
  }
}

// Traces the robot footprint's center through the whole planned route, so
// it's visible on the grid as soon as a plan comes back (not just wherever
// the playback head currently is). Reuses the same arc math the animation
// uses for turns (computeArcSegment/sampleArc, defined below) so the drawn
// path matches the actual swept quarter-circle rather than the raw
// two-leg L-shaped waypoints the server emits for collision-checking.
function drawPath(style) {
  if (!planSteps || !animSegments || animSegments.length === 0) return;

  const offset = (ROBOT_SIZE - 1) / 2;
  const canvasCenter = (gx, gy) => {
    const { cx, cy } = cellToCanvas(gx, gy);
    return { px: cx + CELL_PX / 2, py: cy + CELL_PX / 2 };
  };
  const centerOfStep = (step) => canvasCenter(step.x + offset, step.y + offset);

  ctx.strokeStyle = style.getPropertyValue("--path-line").trim();
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  const start = centerOfStep(planSteps[0]);
  ctx.moveTo(start.px, start.py);

  animSegments.forEach((seg) => {
    if (seg.type === "none" || seg.type === "scan") return;
    const fromPose = planSteps[seg.from];
    const toPose = planSteps[seg.to];
    if (seg.type === "arc") {
      const arc = computeArcSegment(fromPose, toPose, seg.action);
      const ARC_STEPS = 12;
      for (let s = 1; s <= ARC_STEPS; s++) {
        const { x, y } = sampleArc(arc, s / ARC_STEPS);
        const { px, py } = canvasCenter(x, y);
        ctx.lineTo(px, py);
      }
    } else {
      const { px, py } = centerOfStep(toPose);
      ctx.lineTo(px, py);
    }
  });
  ctx.stroke();
  ctx.setLineDash([]);

  // Mark each SCAN stop along the route.
  ctx.fillStyle = style.getPropertyValue("--path-line").trim();
  planSteps.forEach((step) => {
    if (step.action !== "SCAN") return;
    const { px, py } = centerOfStep(step);
    ctx.beginPath();
    ctx.arc(px, py, 4, 0, Math.PI * 2);
    ctx.fill();
  });
}

function drawObstacle(obs) {
  const style = getComputedStyle(document.documentElement);
  const { cx, cy } = cellToCanvas(obs.x, obs.y);

  if (obs.id === highlightObstacleId) {
    // Pulses out and back during Play (progress 0..1 across the scan
    // pause); a fixed-size ring when landed on a SCAN step manually.
    const pulse = highlightProgress < 0 ? 7 : 3 + 8 * Math.sin(Math.min(highlightProgress, 1) * Math.PI);
    ctx.save();
    ctx.strokeStyle = "#fbbf24";
    ctx.lineWidth = 3;
    ctx.strokeRect(cx - pulse, cy - pulse, CELL_PX + pulse * 2, CELL_PX + pulse * 2);
    ctx.restore();
  }

  ctx.fillStyle = style.getPropertyValue("--obstacle").trim();
  ctx.fillRect(cx + 1, cy + 1, CELL_PX - 2, CELL_PX - 2);

  // thick edge on the side the image faces
  ctx.strokeStyle = style.getPropertyValue("--obstacle-face").trim();
  ctx.lineWidth = 4;
  ctx.beginPath();
  if (obs.facing === "N") { ctx.moveTo(cx, cy); ctx.lineTo(cx + CELL_PX, cy); }
  if (obs.facing === "S") { ctx.moveTo(cx, cy + CELL_PX); ctx.lineTo(cx + CELL_PX, cy + CELL_PX); }
  if (obs.facing === "E") { ctx.moveTo(cx + CELL_PX, cy); ctx.lineTo(cx + CELL_PX, cy + CELL_PX); }
  if (obs.facing === "W") { ctx.moveTo(cx, cy); ctx.lineTo(cx, cy + CELL_PX); }
  ctx.stroke();

  ctx.fillStyle = "white";
  ctx.font = "bold 12px sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(String(obs.id), cx + CELL_PX / 2, cy + CELL_PX / 2);
}

function drawSelectionRing(style, obs) {
  const { cx, cy } = cellToCanvas(obs.x, obs.y);
  ctx.save();
  ctx.strokeStyle = style.getPropertyValue("--accent").trim();
  ctx.lineWidth = 2;
  ctx.setLineDash([4, 3]);
  ctx.strokeRect(cx - 3, cy - 3, CELL_PX + 6, CELL_PX + 6);
  ctx.restore();
}

function drawRobot(x, y, facing) {
  const offset = (ROBOT_SIZE - 1) / 2;
  drawRobotAt(x + offset, y + offset, FACING_ANGLE[facing]);
}

// Box position is always the axis-aligned 3x3 square (it never rotates —
// same footprint-doesn't-rotate model as server/arena.py); only the camera
// marker's position around its perimeter depends on `angleRad`, which is
// why this can render both the 4 discrete facings and every angle in
// between during a smooth turn animation with the same code path.
function drawRobotAt(centerX, centerY, angleRad) {
  const style = getComputedStyle(document.documentElement);
  const offset = (ROBOT_SIZE - 1) / 2;
  const anchorX = centerX - offset;
  const anchorY = centerY - offset;
  const { cx, cy } = cellToCanvas(anchorX, anchorY + ROBOT_SIZE - 1); // top-left cell of the footprint
  const size = ROBOT_SIZE * CELL_PX;

  ctx.fillStyle = style.getPropertyValue("--robot-body").trim();
  ctx.globalAlpha = 0.85;
  ctx.fillRect(cx + 1, cy + 1, size - 2, size - 2);
  ctx.globalAlpha = 1;

  // camera marker: sits `offset` cells out from center, in the direction of angleRad
  const camX = centerX + offset * Math.cos(angleRad);
  const camY = centerY + offset * Math.sin(angleRad);
  const cam = cellToCanvas(camX, camY);
  ctx.fillStyle = style.getPropertyValue("--robot-camera").trim();
  ctx.fillRect(cam.cx + 3, cam.cy + 3, CELL_PX - 6, CELL_PX - 6);

  ctx.strokeStyle = "#065f46";
  ctx.lineWidth = 2;
  ctx.strokeRect(cx + 1, cy + 1, size - 2, size - 2);
}

/* ---------- Obstacle editing ---------- */
canvas.addEventListener("click", (e) => {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const { x, y } = canvasToCell((e.clientX - rect.left) * scaleX, (e.clientY - rect.top) * scaleY);
  if (x < 0 || y < 0 || x >= GRID_SIZE || y >= GRID_SIZE) return;

  const existing = obstacles.find((o) => o.x === x && o.y === y);
  if (existing) {
    selectedObstacleId = existing.id;
  } else {
    const obs = { id: 0, x, y, facing: "N" };
    obstacles.push(obs);
    renumberObstacles();
    selectedObstacleId = obs.id;
  }
  resetPlan();
  updateSelectionPanel();
  draw();
});

canvas.addEventListener("contextmenu", (e) => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const { x, y } = canvasToCell((e.clientX - rect.left) * scaleX, (e.clientY - rect.top) * scaleY);
  const selectedObs = obstacles.find((o) => o.id === selectedObstacleId);
  obstacles = obstacles.filter((o) => !(o.x === x && o.y === y));
  renumberObstacles();
  selectedObstacleId = obstacles.includes(selectedObs) ? selectedObs.id : null;
  resetPlan();
  updateSelectionPanel();
  draw();
});

// Obstacle ids are always kept dense and sequential (1..N in placement
// order), rather than ever-incrementing — so removing #2 out of #1-#3
// leaves #1, #2 (formerly #3), not a gap like #1, #3.
function renumberObstacles() {
  obstacles.forEach((o, i) => {
    o.id = i + 1;
  });
}

/* ---------- Selected-obstacle panel ---------- */
function updateSelectionPanel() {
  const obs = obstacles.find((o) => o.id === selectedObstacleId);
  if (!obs) {
    selectedObstacleId = null;
    selectedPanel.style.display = "none";
    noSelectionHint.style.display = "";
    return;
  }
  selectedPanel.style.display = "flex";
  noSelectionHint.style.display = "none";
  selectedIdEl.textContent = String(obs.id);
  facingButtons.forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.facing === obs.facing);
  });
}

facingButtons.forEach((btn) => {
  btn.addEventListener("click", () => {
    const obs = obstacles.find((o) => o.id === selectedObstacleId);
    if (!obs) return;
    obs.facing = btn.dataset.facing;
    resetPlan();
    updateSelectionPanel();
    draw();
  });
});

removeObstacleBtn.addEventListener("click", () => {
  obstacles = obstacles.filter((o) => o.id !== selectedObstacleId);
  renumberObstacles();
  selectedObstacleId = null;
  resetPlan();
  updateSelectionPanel();
  draw();
});

presetSelect.addEventListener("change", () => {
  const key = presetSelect.value;
  if (!key) return;
  obstacles = PRESETS[key].map((o) => ({ ...o }));
  selectedObstacleId = null;
  resetPlan();
  updateSelectionPanel();
  draw();
});

clearBtn.addEventListener("click", () => {
  obstacles = [];
  presetSelect.value = "";
  selectedObstacleId = null;
  resetPlan();
  updateSelectionPanel();
  draw();
});

speedSelect.addEventListener("change", () => {
  speedMultiplier = Number(speedSelect.value) || 1;
});

/* ---------- Planning ---------- */
runBtn.addEventListener("click", async () => {
  resetPlan();
  setStatus("Planning…", "");
  runBtn.disabled = true;

  const body = {
    obstacles: obstacles.map(({ id, x, y, facing }) => ({ id, x, y, facing })),
    start: {
      x: Number(startXInput.value) || 0,
      y: Number(startYInput.value) || 0,
      facing: startFacingSelect.value,
    },
  };

  try {
    const res = await fetch(`${API_BASE}/api/plan`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    if (!res.ok) throw new Error(`server returned ${res.status}`);
    const data = await res.json();

    if (obstacles.length > 0 && data.visit_order.length === 0) {
      setStatus(describeFailure(data.blocked, obstacles.length), "error");
    } else {
      setStatus(
        `Visit order: ${data.visit_order.join(" → ")} — ${data.steps.length} steps, ` +
          `cost ${data.total_cost.toFixed(1)}, planned in ${data.runtime_seconds.toFixed(3)}s`,
        "ok"
      );
    }

    planSteps = data.steps;
    animSegments = buildSegments(planSteps);
    currentStep = 0;
    stepSlider.max = String(planSteps.length - 1);
    stepSlider.value = "0";
    [playBtn, stepBackBtn, stepFwdBtn, stepSlider].forEach((el) => (el.disabled = false));
    updateStepLabel();
    draw();
  } catch (err) {
    setStatus(`Failed to plan: ${err.message}. Is the server running?`, "error");
  } finally {
    runBtn.disabled = false;
  }
});

function resetPlan() {
  stopPlaying();
  planSteps = null;
  animSegments = null;
  currentStep = 0;
  highlightObstacleId = null;
  highlightProgress = -1;
  logEl.innerHTML = "";
  [playBtn, stepBackBtn, stepFwdBtn, stepSlider].forEach((el) => (el.disabled = true));
  stepSlider.max = "0";
  stepSlider.value = "0";
  updateStepLabel();
  setStatus("", "");
}

// The planner only ever returns a *complete* tour or nothing (never a
// partial one), so "reached 0 of N" is always literally true and never
// useful. `blocked` (from the server's own diagnosis) names the specific
// obstacle(s) that had no valid scanning spot, so we can say why instead.
function describeFailure(blocked, obstacleCount) {
  if (blocked && blocked.length > 0) {
    const parts = blocked.map((b) => `obstacle #${b.id} (${b.reason})`);
    return `Can't find a route — ${parts.join("; ")}.`;
  }
  return `Can't find a route that visits all ${obstacleCount} obstacles together — try repositioning them.`;
}

function setStatus(text, kind) {
  statusEl.textContent = text;
  statusEl.className = "status" + (kind ? " " + kind : "");
}

/* ---------- Playback ---------- */
function updateStepLabel() {
  const total = planSteps ? planSteps.length : 0;
  stepLabelEl.textContent = `Step ${total ? currentStep + 1 : 0} / ${total}`;
}

function goToStep(index) {
  if (!planSteps) return;
  currentStep = Math.max(0, Math.min(index, planSteps.length - 1));
  stepSlider.value = String(currentStep);
  updateStepLabel();
  syncLogToStep();
  const step = planSteps[currentStep];
  highlightObstacleId = step.action === "SCAN" ? step.scanned_obstacle_id : null;
  highlightProgress = -1; // manual positioning -> fixed-size highlight, not a pulse
  draw();
}

function syncLogToStep() {
  // Rebuild the recognition log so it always matches steps up to `currentStep`
  // (handles scrubbing backward too, not just forward playback).
  logEl.innerHTML = "";
  for (let i = 0; i <= currentStep; i++) {
    const step = planSteps[i];
    if (step.action === "SCAN") {
      const li = document.createElement("li");
      li.textContent = `Recognized image on obstacle #${step.scanned_obstacle_id} at (${step.x}, ${step.y})`;
      logEl.appendChild(li);
    }
  }
}

stepFwdBtn.addEventListener("click", () => goToStep(currentStep + 1));
stepBackBtn.addEventListener("click", () => goToStep(currentStep - 1));
stepSlider.addEventListener("input", () => goToStep(Number(stepSlider.value)));

/* ---------- Smooth turn animation ----------
 * The server emits two consecutive waypoints per turn move (a straight leg
 * of TURN_RADIUS_CELLS, then a lateral swing of the same length — see
 * pathfinding.py: _diagonal_turn) so it can collision-check both legs.
 * For playback we merge that pair back into a single segment and animate
 * it as the real quarter-circle arc the two legs are a stand-in for,
 * rather than visibly jogging through the L-shaped corner. */
function buildSegments(steps) {
  const segments = [];
  let i = 1;
  while (i < steps.length) {
    const cur = steps[i];
    const prev = steps[i - 1];
    const isTurnName = cur.action.includes("LEFT") || cur.action.includes("RIGHT");
    const nextContinuesTurn =
      isTurnName &&
      cur.facing === prev.facing &&
      i + 1 < steps.length &&
      steps[i + 1].action === cur.action &&
      steps[i + 1].facing !== cur.facing;

    if (nextContinuesTurn) {
      segments.push({ from: i - 1, to: i + 1, type: "arc", action: cur.action });
      i += 2;
      continue;
    }
    const type = cur.action === "START" ? "none" : cur.action === "SCAN" ? "scan" : "straight";
    segments.push({ from: i - 1, to: i, type });
    i++;
  }
  return segments;
}

function computeArcSegment(fromPose, toPose, action) {
  const goingForward = action.startsWith("FORWARD");
  const turnLeft = action.endsWith("LEFT");
  const theta0 = FACING_ANGLE[fromPose.facing];
  const theta1 = theta0 + (turnLeft ? 1 : -1) * (Math.PI / 2);

  const offset = (ROBOT_SIZE - 1) / 2;
  const c0 = { x: fromPose.x + offset, y: fromPose.y + offset };
  const c1 = { x: toPose.x + offset, y: toPose.y + offset };

  // Tangent directions at each end: leg 1 moves forward-or-backward along
  // the start facing (matching `action`'s direction), leg 2 always moves
  // "forward" along the new facing (see _diagonal_turn — its lateral step
  // is never negated). Control points placed at the standard cubic-Bezier
  // circular-arc distance (~0.5523 * radius) approximate a true 90° arc.
  const startDir = goingForward ? 1 : -1;
  const k = TURN_RADIUS_CELLS * 0.5523;
  const p1 = { x: c0.x + startDir * Math.cos(theta0) * k, y: c0.y + startDir * Math.sin(theta0) * k };
  const p2 = { x: c1.x - Math.cos(theta1) * k, y: c1.y - Math.sin(theta1) * k };

  return { c0, c1, p1, p2, theta0, theta1 };
}

function sampleArc(arc, t) {
  const { c0, c1, p1, p2, theta0, theta1 } = arc;
  const mt = 1 - t;
  const x = mt * mt * mt * c0.x + 3 * mt * mt * t * p1.x + 3 * mt * t * t * p2.x + t * t * t * c1.x;
  const y = mt * mt * mt * c0.y + 3 * mt * mt * t * p1.y + 3 * mt * t * t * p2.y + t * t * t * c1.y;
  return { x, y, angle: theta0 + (theta1 - theta0) * t };
}

function sampleStraight(fromPose, toPose, t) {
  const offset = (ROBOT_SIZE - 1) / 2;
  return {
    x: fromPose.x + (toPose.x - fromPose.x) * t + offset,
    y: fromPose.y + (toPose.y - fromPose.y) * t + offset,
    angle: FACING_ANGLE[fromPose.facing],
  };
}

// "scan" is deliberately much longer than the plain "none" pause (START's
// brief settle) — it's the moment the robot is meant to be recognizing the
// image, so it should read as a real stop, not a blip.
const SEGMENT_MS = { none: 150, straight: 260, arc: 480, scan: 900 };

function stopPlaying() {
  if (playTimer) {
    playTimer = null;
    if (animFrameHandle) cancelAnimationFrame(animFrameHandle);
    animFrameHandle = null;
    playState = null;
    renderPose = null;
  }
  playBtn.textContent = "Play";
}

function playAnimate(ts) {
  if (!playTimer) return;

  if (!playState) {
    if (currentStep >= planSteps.length - 1) {
      stopPlaying();
      draw();
      return;
    }
    const seg = animSegments.find((s) => s.from === currentStep);
    if (!seg) {
      stopPlaying();
      return;
    }
    playState = { seg, startTs: ts };
    highlightObstacleId = seg.type === "scan" ? planSteps[seg.to].scanned_obstacle_id : null;
    highlightProgress = 0;
  }

  const { seg, startTs } = playState;
  const duration = SEGMENT_MS[seg.type] / speedMultiplier;
  const t = Math.min(1, (ts - startTs) / duration);
  const fromPose = planSteps[seg.from];
  const toPose = planSteps[seg.to];

  if (seg.type === "arc") {
    if (!playState.arc) playState.arc = computeArcSegment(fromPose, toPose, seg.action);
    renderPose = sampleArc(playState.arc, t);
  } else if (seg.type === "straight") {
    renderPose = sampleStraight(fromPose, toPose, t);
  } else {
    renderPose = sampleStraight(fromPose, fromPose, 0);
    if (seg.type === "scan") highlightProgress = t;
  }
  draw();

  if (t >= 1) {
    currentStep = seg.to;
    stepSlider.value = String(currentStep);
    updateStepLabel();
    syncLogToStep();
    playState = null;
    renderPose = null;
    if (seg.type === "scan") highlightObstacleId = null;
  }

  animFrameHandle = requestAnimationFrame(playAnimate);
}

playBtn.addEventListener("click", () => {
  if (playTimer) {
    stopPlaying();
    return;
  }
  if (!planSteps) return;
  if (currentStep >= planSteps.length - 1) currentStep = 0; // restart from the beginning
  playBtn.textContent = "Pause";
  playTimer = true;
  playState = null;
  animFrameHandle = requestAnimationFrame(playAnimate);
});

/* ---------- Init ---------- */
resetPlan();
updateSelectionPanel();
draw();
