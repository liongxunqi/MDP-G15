package com.mdp.g15.arena.domain

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/** The pose anchor is the lower-left of the north-facing reference rectangle. */
object RobotFootprint {
    data class Point(val x: Double, val y: Double)

    fun corners(pose: RobotPose, config: ArenaConfig): List<Point> {
        val halfWidth = config.robotWidthCells / 2.0
        val halfLength = config.robotLengthCells / 2.0
        val cx = pose.x + halfWidth
        val cy = pose.y + halfLength
        val angle = Math.toRadians(pose.heading)
        val c = cos(angle)
        val s = sin(angle)
        return listOf(-halfWidth to -halfLength, halfWidth to -halfLength,
            halfWidth to halfLength, -halfWidth to halfLength).map { (dx, dy) ->
            Point(cx + dx * c + dy * s, cy - dx * s + dy * c)
        }
    }

    fun fitsArena(pose: RobotPose, config: ArenaConfig, tolerance: Double = 1e-8): Boolean =
        corners(pose, config).all {
            it.x >= -tolerance && it.y >= -tolerance &&
                it.x <= config.columns + tolerance && it.y <= config.rows + tolerance
        }

    fun contains(pose: RobotPose, config: ArenaConfig, point: Point): Boolean {
        val cx = pose.x + config.robotWidthCells / 2.0
        val cy = pose.y + config.robotLengthCells / 2.0
        val angle = Math.toRadians(pose.heading)
        val dx = point.x - cx
        val dy = point.y - cy
        val localX = dx * cos(angle) - dy * sin(angle)
        val localY = dx * sin(angle) + dy * cos(angle)
        return abs(localX) <= config.robotWidthCells / 2.0 &&
            abs(localY) <= config.robotLengthCells / 2.0
    }

    /** Positive-area overlap; edge contact alone is allowed. Margin adds physical clearance. */
    fun overlapsCell(pose: RobotPose, config: ArenaConfig, cell: GridCoordinate, margin: Double = 0.0): Boolean {
        val robot = corners(pose, config)
        val square = listOf(
            Point(cell.x - margin, cell.y - margin), Point(cell.x + 1 + margin, cell.y - margin),
            Point(cell.x + 1 + margin, cell.y + 1 + margin), Point(cell.x - margin, cell.y + 1 + margin),
        )
        val axes = listOf(
            Point(robot[1].y - robot[0].y, robot[0].x - robot[1].x),
            Point(robot[3].y - robot[0].y, robot[0].x - robot[3].x),
            Point(1.0, 0.0), Point(0.0, 1.0),
        )
        return axes.all { axis ->
            val a = robot.map { it.x * axis.x + it.y * axis.y }
            val b = square.map { it.x * axis.x + it.y * axis.y }
            a.max() > b.min() + 1e-8 && b.max() > a.min() + 1e-8
        }
    }

    fun clearForPlacement(pose: RobotPose, state: ArenaState): Boolean =
        fitsArena(pose, state.config) && state.obstacles.values.none {
            overlapsCell(pose, state.config, it.position)
        }
}
