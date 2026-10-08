"""Bounded local recovery, then a best-effort tour of pending targets."""
import logging
import math
import time

import path_planner as pp


def recover_route(plan, obstacles, progress, arc_profile=0):
    seg = int(progress["segment_index"])
    goals = plan.get("original_photo_endpoints") or {
        str(oid): list(poses[-1]) for oid, poses in
        zip(plan["segment_obstacles"], plan["expected"]) if oid is not None}
    remaining = progress.get("remaining_photo_ids")
    planned = [str(oid) for oid in plan["segment_obstacles"][seg:] if oid is not None]
    if remaining is not None and [str(oid) for oid in remaining] != planned:
        logging.error("Recovery rejected: pending photo order does not match the active plan")
        return None
    oid = plan["segment_obstacles"][seg]
    endpoint = goals.get(str(oid), progress.get("segment_goal", plan["expected"][seg][-1]))
    started = time.monotonic()
    deadline = started + 3.0  # Reserve the rest of the decision window for other targets.
    sx, sy = pp._plan_frame_shift(plan)
    boxes = pp._collision_boxes([pp._obstacle_aabb_mm(o) for o in obstacles])
    if pp.RPI_ARENA_GUARD:
        m = pp.RPI_ARENA_MARGIN_MM
        boxes.ref_bounds = (m - sx, pp.ARENA_MM - m - sx,
                            m - sy, pp.ARENA_MM - m - sy)
    radius = pp.TURN_RADIUS_MM[arc_profile]

    def wire(pose):
        return [pose.x + sx, pose.y + sy,
                (90.0 - math.degrees(pose.theta)) % 360.0]

    def replay(start, tokens):
        pose = pp.Pose(start[0] - sx, start[1] - sy,
                       math.radians(90.0 - start[2]))
        if not pp._pose_clear(pose.x, pose.y, pose.theta, boxes):
            return None
        result = []
        for token in tokens:
            if token != "S":
                poses = pp._replay_motion(pose, [token], radius, boxes)
                if poses is None:
                    return None
                pose = poses[-1]
            result.append(wire(pose))
        return result

    def near(actual, desired):
        return (math.hypot(actual[0] - desired[0], actual[1] - desired[1]) <= 25
                and abs((actual[2] - desired[2] + 180) % 360 - 180) <= 6)

    def leg(start, goal, first, budget):
        single = dict(plan, segments=[["S"]], segment_obstacles=[None],
                      expected=[[list(goal)]])
        report = dict(progress, segment_index=0, remaining_photo_ids=[],
                      x_grid=start[0] / 100, y_grid=start[1] / 100,
                      heading_deg=start[2], recovery_deadline=min(deadline, time.monotonic() + budget))
        if not first:
            report.pop("blocked_token", None)
            report.pop("recovery_reason", None)
        candidate = pp.plan_segment_recovery(single, obstacles, report, arc_profile)
        if candidate is None:
            return None
        tokens = [t for line in candidate["segments"] for t in line]
        poses = replay(start, tokens)
        if not poses or not near(poses[-1], goal):
            return None
        return tokens, poses

    start = [float(progress["x_grid"]) * 100,
             float(progress["y_grid"]) * 100, float(progress["heading_deg"])]

    if plan.get("buffer_escape_count", 0) < 2:
        for cm in range(1, 21):
            token = "R%d" % cm
            escaped = pp.buffer_escape_endpoint(plan, obstacles, progress, token)
            if escaped is None:
                continue
            logging.warning("Buffer escape %s: temporary 20 mm obstacle clearance; "
                            "endpoint restores 30 mm, then live-pose replanning", token)
            return dict(plan, segments=[[token, "S"]] + [["S"] for _ in planned],
                        segment_obstacles=[None] + planned,
                        expected=[[escaped, escaped]] + [[list(goals[oid])] for oid in planned],
                        deferred_segments=list(range(1, len(planned) + 1)),
                        original_photo_endpoints=goals, skipped_photo_ids=[],
                        buffer_escape=token, buffer_escape_count=plan.get("buffer_escape_count", 0) + 1,
                        recovery_strategy="buffer_escape")

    def assemble(tokens, strategy):
        if not tokens or tokens[-1] != "S":
            tokens = list(tokens) + ["S"]
        poses = replay(start, tokens)
        if not poses or not near(poses[-1], endpoint):
            return None
        lines = pp.chunk_tokens(tokens)
        expected, offset = [], 0
        for line in lines:
            expected.append(poses[offset:offset + len(line)])
            offset += len(line)
        mappings = [None] * len(lines)
        mappings[-1] = plan["segment_obstacles"][seg]
        deferred = []
        for old_index, (line, old_poses, oid) in enumerate(zip(plan["segments"][seg + 1:],
                                       plan["expected"][seg + 1:],
                                       plan["segment_obstacles"][seg + 1:]), seg + 1):
            if old_index in plan.get("deferred_segments", []):
                deferred.append(len(lines))
            lines.append(list(line))
            expected.append([list(p) for p in old_poses])
            mappings.append(oid)
        logging.warning("Recovery strategy %s; obstacle order retained", strategy)
        return dict(plan, segments=lines, expected=expected,
                    segment_obstacles=mappings, recovery_strategy=strategy,
                    skipped_photo_ids=[], original_photo_endpoints=goals,
                    deferred_segments=deferred, buffer_escape=None)

    alignment = progress.get("alignment_recovery", False)
    ins = int(progress["instruction_index"]) + 1
    force_defer = progress.get("force_defer", False)
    if not force_defer and not alignment and ins < len(plan["segments"][seg]):
        goal = endpoint if seg in plan.get("deferred_segments", []) else plan["expected"][seg][ins]
        candidate = leg(start, goal, True, 0.6)
        if candidate:
            result = assemble(candidate[0][:-1] + plan["segments"][seg][ins + 1:],
                              "instruction_endpoint")
            if result:
                return result
    # A range mismatch at the estimated endpoint needs a different approach,
    # not a no-op route back to the very same estimated pose.
    if not force_defer and (not alignment or not near(start, endpoint)):
        candidate = leg(start, endpoint, True, 0.9)
        if candidate:
            result = assemble(candidate[0], "segment_endpoint")
            if result:
                return result
    previous = progress.get("previous_pose")
    if not force_defer and previous and not near(start, previous) and time.monotonic() < deadline:
        back = leg(start, previous, True, 0.75)
        if back:
            onward = leg(back[1][-1], endpoint, False, 0.75)
            if onward:
                result = assemble(back[0] + onward[0], "previous_pose")
                if result:
                    return result
    logging.warning("Local recovery exhausted; trying other pending targets before the blocked target")
    deadline = started + 6.0
    targets = [(str(oid), goals[str(oid)]) for oid, poses in
               zip(plan["segment_obstacles"][seg:], plan["expected"][seg:])
               if oid is not None]
    if not targets:
        return None
    blocked = targets[0][0]
    counts = dict(plan.get("deferral_counts", {}))
    counts[blocked] = counts.get(blocked, 0) + 1
    if counts[blocked] > 2:
        logging.error("Deferral limit reached for %s; retaining pending targets", blocked)
        return None
    # Plan only the next visit. Later targets require a new measured-pose preflight.
    queue = targets[1:]
    lines, expected, mappings = [], [], []
    current = start

    def visit(target):
        nonlocal current
        oid, goal = target
        if time.monotonic() >= deadline:
            return False
        candidate = leg(current, goal, not lines, min(0.5, deadline - time.monotonic()))
        if candidate is None:
            return False
        tokens, poses = candidate
        chunks = pp.chunk_tokens(tokens)
        offset = 0
        for chunk in chunks:
            lines.append(chunk)
            expected.append(poses[offset:offset + len(chunk)])
            mappings.append(None)
            offset += len(chunk)
        mappings[-1] = oid
        current = poses[-1]
        return True

    chosen = None
    for target in queue:
        if visit(target):
            chosen = target[0]
            break
    if not lines:
        logging.error("No safe escape to any pending target; refusing blind movement")
        return None
    deferred = []
    for oid, goal in targets[1:] + targets[:1]:
        if oid == chosen:
            continue
        deferred.append(len(lines))
        lines.append(["S"])
        expected.append([list(goal)])
        mappings.append(oid)
    logging.warning("Recovery to %s; targets %s remain pending for live-pose planning",
                    chosen, [mappings[i] for i in deferred])
    return dict(plan, segments=lines, expected=expected, segment_obstacles=mappings,
                recovery_strategy="defer_and_retry", deferral_counts=counts,
                skipped_photo_ids=[], original_photo_endpoints=goals,
                deferred_segments=deferred, buffer_escape=None)
