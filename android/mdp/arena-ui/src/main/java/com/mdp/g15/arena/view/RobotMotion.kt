package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.ArenaState
import com.mdp.g15.arena.domain.RobotPose
import com.mdp.g15.arena.domain.DrivePose
import com.mdp.g15.arena.domain.DrivePath
import kotlin.math.max
import kotlin.math.min

/** Display-only interpolation of telemetry or a validated, explicitly estimated manual path. */
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
        robot.commandedPath?.let { commanded ->
            if (commanded.fits(state)) {
                path = commanded
                val start = commanded.start
                val end = commanded.at(1.0)
                from = Pose(start.x.toFloat(), start.y.toFloat(), start.heading.toFloat())
                to = Pose(end.x.toFloat(), end.y.toFloat(), end.heading.toFloat())
                startedAt = now
                // Readable preview timing, not a claim about measured physical velocity.
                duration = commanded.previewDurationMillis
                return
            }
        }
        val start = sample(now) ?: return snap(robot)
        path = null
        val precise = DrivePose.from(robot)
        val end = Pose(precise.x.toFloat(), precise.y.toFloat(), precise.heading.toFloat())
        // Sparse endpoints do not specify a route around a corner. Only interpolate
        // an axis-aligned segment whose entire swept 2x2 footprint is clear.
        val size = state.config.robotFootprintCells
        val left = min(start.x, end.x)
        val right = max(start.x, end.x) + size
        val bottom = min(start.y, end.y)
        val top = max(start.y, end.y) + size
        val blocked = state.obstacles.values.any {
            it.position.x < right && it.position.x + 1 > left &&
                it.position.y < top && it.position.y + 1 > bottom
        }
        if ((start.x != end.x && start.y != end.y) || blocked ||
            left < 0 || bottom < 0 || right > state.config.columns || top > state.config.rows) {
            return snap(robot)
        }
        val turn = ((end.angle - start.angle) % 360f + 540f) % 360f - 180f
        from = start
        to = end.copy(angle = start.angle + turn)
        startedAt = now
        duration = 180L
    }
}
