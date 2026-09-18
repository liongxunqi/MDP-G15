package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.ArenaState
import com.mdp.g15.arena.domain.RobotPose
import kotlin.math.max
import kotlin.math.min

/** Display-only interpolation. Never changes the authoritative pose or predicts commands. */
internal class RobotMotion {
    data class Pose(val x: Float, val y: Float, val angle: Float)
    private var from: Pose? = null
    private var to: Pose? = null
    private var startedAt = 0L
    private var duration = 0L

    fun sample(now: Long): Pose? {
        val end = to ?: return null
        val start = from ?: return end
        val fraction = if (duration == 0L) 1f else ((now - startedAt).toFloat() / duration).coerceIn(0f, 1f)
        return Pose(
            start.x + (end.x - start.x) * fraction,
            start.y + (end.y - start.y) * fraction,
            start.angle + (end.angle - start.angle) * fraction,
        )
    }

    fun isRunning(now: Long): Boolean = to != null && now < startedAt + duration

    fun snap(pose: RobotPose?) {
        to = pose?.let { Pose(it.position.x.toFloat(), it.position.y.toFloat(), it.direction.ordinal * 90f) }
        from = to
        duration = 0L
    }

    fun retarget(state: ArenaState, now: Long) {
        val robot = state.robot ?: return snap(null)
        val start = sample(now) ?: return snap(robot)
        val end = Pose(robot.position.x.toFloat(), robot.position.y.toFloat(), robot.direction.ordinal * 90f)
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
