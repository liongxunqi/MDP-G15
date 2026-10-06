package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.ArenaState
import com.mdp.g15.arena.domain.RobotPose
import com.mdp.g15.arena.domain.DrivePose
import com.mdp.g15.arena.domain.DrivePath

/** Display-only animation of a validated, explicitly estimated manual path. */
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
        return Pose(
            start.x + (end.x - start.x) * fraction,
            start.y + (end.y - start.y) * fraction,
            start.angle + (end.angle - start.angle) * fraction,
        )
    }

    fun isRunning(now: Long): Boolean = to != null && now < startedAt + duration

    fun snap(pose: RobotPose?) {
        path = null
        to = pose?.let { DrivePose.from(it) }?.let { Pose(it.x.toFloat(), it.y.toFloat(), it.heading.toFloat()) }
        from = to
        duration = 0L
    }

    fun retarget(state: ArenaState, now: Long) {
        val robot = state.robot ?: return snap(null)
        // Only an explicitly commanded local preview animates. Wire poses are authoritative.
        val commanded = robot.commandedPath ?: return snap(robot)
        if (!commanded.fits(state)) return snap(robot)
        path = commanded
        val start = commanded.start
        val end = commanded.at(1.0)
        from = Pose(start.x.toFloat(), start.y.toFloat(), start.heading.toFloat())
        to = Pose(end.x.toFloat(), end.y.toFloat(), end.heading.toFloat())
        startedAt = now
        duration = commanded.previewDurationMillis
    }
}
