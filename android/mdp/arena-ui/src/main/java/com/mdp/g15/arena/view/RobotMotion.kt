package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.ArenaState
import com.mdp.g15.arena.domain.DrivePath
import com.mdp.g15.arena.domain.PoseSource
import com.mdp.g15.arena.domain.RobotFootprint
import com.mdp.g15.arena.domain.RobotPose
import kotlin.math.abs

/** A rendering state only. Receipt, not animation completion, owns the robot position. */
internal class RobotMotion {
    data class Pose(val x: Float, val y: Float, val angle: Float)
    private var from: Pose? = null
    private var to: Pose? = null
    private var startedAt = 0L
    private var duration = 0L
    private var path: DrivePath? = null

    fun sample(now: Long): Pose? {
        val end = to ?: return null
        val start = from ?: return end
        val fraction = if (duration == 0L) 1f else ((now - startedAt).toFloat() / duration).coerceIn(0f, 1f)
        path?.let {
            val point = it.at(fraction.toDouble())
            return Pose(point.x.toFloat(), point.y.toFloat(), point.heading.toFloat())
        }
        val turn = ((end.angle - start.angle + 540f) % 360f) - 180f
        val shortestTurn = if (turn == -180f) 180f else turn
        return Pose(start.x + (end.x - start.x) * fraction,
            start.y + (end.y - start.y) * fraction,
            ((start.angle + shortestTurn * fraction) % 360f + 360f) % 360f)
    }

    fun isRunning(now: Long): Boolean = to != null && now < startedAt + duration

    fun snap(pose: RobotPose?) {
        path = null
        to = pose?.let { Pose(it.x.toFloat(), it.y.toFloat(), it.heading.toFloat()) }
        from = to
        duration = 0L
    }

    fun retarget(state: ArenaState, now: Long) {
        val robot = state.robot ?: return snap(null)
        val commanded = robot.commandedPath
        if (robot.source == PoseSource.ESTIMATED && commanded != null && commanded.fits(state)) {
            path = commanded
            val start = commanded.start
            val end = commanded.at(1.0)
            from = Pose(start.x.toFloat(), start.y.toFloat(), start.heading.toFloat())
            to = Pose(end.x.toFloat(), end.y.toFloat(), end.heading.toFloat())
            startedAt = now
            duration = commanded.previewDurationMillis
            return
        }
        if (robot.source != PoseSource.REPORTED) return snap(robot)
        val end = Pose(robot.x.toFloat(), robot.y.toFloat(), robot.heading.toFloat())
        val start = sample(now) ?: return snap(robot)
        if (abs(end.x - start.x) < 1e-6f && abs(end.y - start.y) < 1e-6f &&
            abs(((end.angle - start.angle + 540f) % 360f) - 180f) < 1e-6f) return

        // A straight visual chord is only used if it stays clear. It is not a route claim.
        val chordIsClear = (0..24).all { step ->
            val t = step / 24f
            val turn = ((end.angle - start.angle + 540f) % 360f) - 180f
            val angle = start.angle + (if (turn == -180f) 180f else turn) * t
            val pose = RobotPose((start.x + (end.x - start.x) * t).toDouble(),
                (start.y + (end.y - start.y) * t).toDouble(), angle.toDouble())
            RobotFootprint.fitsArena(pose, state.config) && state.obstacles.values.none {
                RobotFootprint.overlapsCell(pose, state.config, it.position)
            }
        }
        if (!chordIsClear) return snap(robot)
        path = null
        from = start
        to = end
        startedAt = now
        duration = 150L
    }
}
