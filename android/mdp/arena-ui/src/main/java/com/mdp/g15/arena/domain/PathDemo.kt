package com.mdp.g15.arena.domain

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sign
import kotlin.math.sin

enum class PathSegmentType { STRAIGHT, ARC }

/**
 * One scripted move. Poses use the arena anchor convention (see [RobotFootprint]): the anchor is
 * the lower-left of the north-facing reference rectangle, so the footprint centre is always
 * anchor + (width / 2, length / 2) and a curve of the anchor is the same curve as the centre's.
 */
data class PathSegment(
    val startPose: RobotPose,
    val endPose: RobotPose,
    val durationMs: Long,
    val type: PathSegmentType,
) {
    init {
        require(durationMs > 0) { "Path segment duration must be positive." }
    }

    /** Signed heading change, clockwise positive, in (-180, 180]. */
    private val turnDeg: Double =
        (((endPose.heading - startPose.heading) % 360.0 + 540.0) % 360.0 - 180.0).let {
            if (it == -180.0) 180.0 else it
        }

    private val isArc: Boolean get() = type == PathSegmentType.ARC && abs(turnDeg) > 1e-6

    /** Chord length and compass bearing (clockwise from north) from start to end anchor. */
    private val chord: Double = hypot(endPose.x - startPose.x, endPose.y - startPose.y)
    private val chordBearing: Double = atan2(endPose.x - startPose.x, endPose.y - startPose.y)

    /** Radius of the circle through both positions whose arc sweeps exactly [turnDeg]. */
    private val radius: Double = if (isArc) chord / (2.0 * sin(Math.toRadians(abs(turnDeg)) / 2.0)) else 0.0

    val length: Double get() = if (isArc) radius * Math.toRadians(abs(turnDeg)) else chord

    /**
     * Pose at [fraction] of the move (constant speed). Straight moves interpolate linearly. Arcs
     * follow the circle whose swept angle equals the heading change, so position and heading
     * change together; the direction of travel at the ends is the chord bearing ∓ half the turn.
     */
    fun poseAt(fraction: Double): RobotPose {
        val f = fraction.coerceIn(0.0, 1.0)
        val heading = startPose.heading + turnDeg * f
        if (!isArc) {
            return RobotPose(startPose.x + (endPose.x - startPose.x) * f,
                startPose.y + (endPose.y - startPose.y) * f, normalise(heading), source = PoseSource.SIMULATED)
        }
        if (f == 1.0) return endPose.copy(heading = normalise(endPose.heading), source = PoseSource.SIMULATED)
        val turn = Math.toRadians(turnDeg)
        val tangent0 = chordBearing - turn / 2.0
        val tangent = tangent0 + turn * f
        val signedRadius = radius * sign(turn)
        // Integral of the unit travel vector (sin τ, cos τ) over the swept angle.
        return RobotPose(
            startPose.x + signedRadius * (cos(tangent0) - cos(tangent)),
            startPose.y + signedRadius * (sin(tangent) - sin(tangent0)),
            normalise(heading),
            source = PoseSource.SIMULATED,
        )
    }

    /** Footprint centre along the move, start to end, for the direction-of-travel arrow. */
    fun centreline(config: ArenaConfig): List<RobotFootprint.Point> {
        val samples = sampleCount()
        return (0..samples).map { step ->
            val pose = poseAt(step.toDouble() / samples)
            RobotFootprint.Point(pose.x + config.robotWidthCells / 2.0, pose.y + config.robotLengthCells / 2.0)
        }
    }

    /** Every in-arena cell the footprint overlaps (positive area) at any point of the move. */
    fun sweptCells(config: ArenaConfig): Set<GridCoordinate> {
        val samples = sampleCount()
        val cells = mutableSetOf<GridCoordinate>()
        for (step in 0..samples) {
            val pose = poseAt(step.toDouble() / samples)
            val corners = RobotFootprint.corners(pose, config)
            val minX = max(0, floor(corners.minOf { it.x }).toInt())
            val maxX = minOf(config.columns - 1, floor(corners.maxOf { it.x }).toInt())
            val minY = max(0, floor(corners.minOf { it.y }).toInt())
            val maxY = minOf(config.rows - 1, floor(corners.maxOf { it.y }).toInt())
            for (x in minX..maxX) for (y in minY..maxY) {
                val cell = GridCoordinate(x, y)
                if (cell !in cells && RobotFootprint.overlapsCell(pose, config, cell)) cells += cell
            }
        }
        return cells
    }

    // Fine enough that consecutive footprints overlap: 0.05 cell of travel or 1° of turn.
    private fun sampleCount(): Int = max(8, max(ceil(length / 0.05), ceil(abs(turnDeg))).toInt())

    private fun normalise(heading: Double): Double = ((heading % 360.0) + 360.0) % 360.0
}

/** What the arena draws under the robot for the active move only. */
data class PathOverlay(
    val cells: Set<GridCoordinate>,
    /** Footprint-centre path in arena cells; the arrowhead sits on the last point. */
    val centreline: List<RobotFootprint.Point>,
)

/**
 * A scripted sequence on one clock: every frame is computed from the absolute time since the
 * start, so a late frame never shifts later moves.
 */
class PathDemoPlayback(
    val initialPose: RobotPose,
    val segments: List<PathSegment>,
    config: ArenaConfig,
) {
    data class Frame(val pose: RobotPose, val activeSegment: Int?, val overlay: PathOverlay?, val finished: Boolean)

    private val overlays = segments.map { PathOverlay(it.sweptCells(config), it.centreline(config)) }
    private val startTimes = segments.runningFold(0L) { start, segment -> start + segment.durationMs }
    val totalDurationMs: Long = startTimes.last()

    fun frameAt(elapsedMs: Long): Frame {
        if (segments.isEmpty()) return Frame(initialPose, null, null, finished = true)
        if (elapsedMs >= totalDurationMs) {
            return Frame(segments.last().poseAt(1.0), null, null, finished = true)
        }
        val elapsed = elapsedMs.coerceAtLeast(0L)
        val index = segments.indices.last { startTimes[it] <= elapsed }
        val fraction = (elapsed - startTimes[index]).toDouble() / segments[index].durationMs
        return Frame(segments[index].poseAt(fraction), index, overlays[index], finished = false)
    }
}

object PathDemo {
    private fun pose(x: Double, y: Double, heading: Double) = RobotPose(x, y, heading, source = PoseSource.SIMULATED)

    /** Move 0: where every playback starts. */
    val INITIAL_POSE: RobotPose = pose(0.0, 0.0, 0.0)

    val SCRIPT: List<PathSegment> = listOf(
        // 1: straight forward.
        PathSegment(INITIAL_POSE, pose(0.0, 10.7, 0.0), 3_000L, PathSegmentType.STRAIGHT),
        // 2: reverse 1.7 cells while turning to NE.
        PathSegment(pose(0.0, 10.7, 0.0), pose(0.0, 9.0, 45.0), 5_000L, PathSegmentType.ARC),
        // 3: forward-right arc to E.
        PathSegment(pose(0.0, 9.0, 45.0), pose(3.0, 10.5, 90.0), 8_000L, PathSegmentType.ARC),
    )
}
