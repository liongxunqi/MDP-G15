package com.mdp.g15.arena.domain

import kotlin.math.*

/**
 * Distance one Forward/Reverse press travels, in 10 cm cells. Must equal the RPi manual
 * bridge's MANUAL_MOVE_CM / 10, or the estimated pose drifts from the real robot.
 */
const val MANUAL_STEP_CELLS = 1.0

/**
 * Defaults from the RPi manual bridge and STM TIGHT profile. Units are 10 cm cells.
 * [pivot] commands (Left/Right) rotate the robot on the spot: same cell, new heading.
 * Every other turn is an arc that also moves the robot.
 */
enum class ManualCommand(val wire: String, val reverse: Boolean = false, val turn: Double = 0.0, val pivot: Boolean = false) {
    FORWARD("f"), REVERSE("r", true), LEFT("tl", turn = -90.0, pivot = true), RIGHT("tr", turn = 90.0, pivot = true),
    FORWARD_LEFT("fl", turn = -45.0), FORWARD_RIGHT("fr", turn = 45.0),
    BACK_LEFT("bl", true, 45.0), BACK_RIGHT("br", true, -45.0);
}

data class DrivePose(val x: Double, val y: Double, val heading: Double) {
    fun samePose(other: DrivePose): Boolean = abs(x - other.x) < 1e-8 && abs(y - other.y) < 1e-8 &&
        abs(((heading - other.heading) % 360.0 + 540.0) % 360.0 - 180.0) < 1e-8
    fun asRobotPose() = RobotPose(GridCoordinate(floor(x + 1e-8).toInt(), floor(y + 1e-8).toInt()),
        Direction.entries[((heading / 90.0).roundToInt() % 4 + 4) % 4], this)
    fun bounds(size: Int): DoubleArray {
        val half = size / 2.0
        val a = Math.toRadians(heading)
        val extent = half * (abs(sin(a)) + abs(cos(a)))
        return doubleArrayOf(x + half - extent, y + half - extent, x + half + extent, y + half + extent)
    }
    fun overlaps(cell: GridCoordinate, size: Int): Boolean {
        val b = bounds(size)
        return cell.x < b[2] - 1e-8 && cell.x + 1 > b[0] + 1e-8 && cell.y < b[3] - 1e-8 && cell.y + 1 > b[1] + 1e-8
    }
    companion object {
        fun from(pose: RobotPose) = pose.estimate ?: DrivePose(pose.position.x.toDouble(), pose.position.y.toDouble(), pose.direction.ordinal * 90.0)
    }
}

data class DrivePath(val start: DrivePose, val command: ManualCommand, val radius: Double = 2.91) {
    val previewDurationMillis: Long get() = if (command.turn == 0.0) 650L else 1000L
    fun at(fraction: Double): DrivePose {
        val t = fraction.coerceIn(0.0, 1.0)
        val h = Math.toRadians(start.heading)
        val sign = if (command.reverse) -1.0 else 1.0
        if (command.pivot) return DrivePose(start.x, start.y, start.heading + command.turn * t)
        if (command.turn == 0.0) return DrivePose(start.x + sin(h) * MANUAL_STEP_CELLS * t * sign, start.y + cos(h) * MANUAL_STEP_CELLS * t * sign, start.heading)
        val delta = Math.toRadians(command.turn) * t
        val r = radius * sign * command.turn.sign
        return DrivePose(start.x + r * (cos(h) - cos(h + delta)), start.y + r * (sin(h + delta) - sin(h)), start.heading + command.turn * t)
    }

    /**
     * Conservative swept AABBs between closely spaced samples, including a rotating square.
     * Pivots (Left/Right) never leave their cell, so there is no path to sweep and nothing to check.
     */
    fun fits(state: ArenaState): Boolean {
        if (command.pivot) return true
        fun bounds(p: DrivePose) = p.bounds(state.config.robotFootprintCells)
        var previous = bounds(at(0.0))
        for (i in 1..180) {
            val next = bounds(at(i / 180.0))
            // Arc sagitta and rotation between samples are below 0.001 cell.
            val margin = if (command.turn == 0.0) 0.0 else 0.001
            val l = min(previous[0], next[0]) - margin
            val b = min(previous[1], next[1]) - margin
            val r = max(previous[2], next[2]) + margin
            val t = max(previous[3], next[3]) + margin
            if (l < -1e-8 || b < -1e-8 || r > state.config.columns + 1e-8 || t > state.config.rows + 1e-8) return false
            if (state.obstacles.values.any { it.position.x < r - 1e-8 && it.position.x + 1 > l + 1e-8 && it.position.y < t - 1e-8 && it.position.y + 1 > b + 1e-8 }) return false
            previous = next
        }
        return true
    }
}

/** No timer releases a real command. STATUS,OK completes it; uncertainty fails closed. */
class ManualDriveGuard {
    var pose: DrivePose? = null
        private set
    var pending: DrivePath? = null
        private set
    var uncertain = false
        private set
    var reportedDuringMove: RobotPose? = null
        private set
    fun report(robot: RobotPose): Boolean {
        // A pose report is not a command completion acknowledgement.
        if (pending == null) { pose = DrivePose.from(robot); uncertain = false }
        else {
            if (DrivePose.from(robot).samePose(pending!!.start)) return false
            reportedDuringMove = robot
        }
        return true
    }
    fun allowed(state: ArenaState, command: ManualCommand): Boolean =
        !uncertain && pending == null && pose?.let { DrivePath(it, command).fits(state) } == true
    fun reserve(state: ArenaState, command: ManualCommand): DrivePath? {
        if (!allowed(state, command)) return null
        return DrivePath(requireNotNull(pose), command).also { pending = it; reportedDuringMove = null }
    }
    fun complete() { pending?.let { pose = reportedDuringMove?.let(DrivePose::from) ?: it.at(1.0) }; pending = null }
    fun flagUncertain() { uncertain = true }
    fun invalidate() { pending = null; pose = null; uncertain = true; reportedDuringMove = null }
}
