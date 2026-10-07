package com.mdp.g15.arena.domain

import kotlin.math.*


const val MANUAL_STEP_CELLS = 2.0

typealias DrivePose = RobotPose
/** Defaults from the RPi manual bridge and STM TIGHT profile. Units are 10 cm cells. */
enum class ManualCommand(val wire: String, val reverse: Boolean = false, val turn: Double = 0.0) {
    FORWARD("f"), REVERSE("r", true), LEFT("tl", turn = -90.0), RIGHT("tr", turn = 90.0),
    FORWARD_LEFT("fl", turn = -45.0), FORWARD_RIGHT("fr", turn = 45.0),
    BACK_LEFT("bl", true, 45.0), BACK_RIGHT("br", true, -45.0);

    fun previewDurationMillis(amdToolMode: Boolean): Long =
        if (amdToolMode) 180L else if (turn == 0.0) 650L else 1000L

}

data class DrivePath(val start: DrivePose, val command: ManualCommand, val radius: Double = 2.91, val amdToolMode: Boolean = false) {
    val previewDurationMillis: Long get() = command.previewDurationMillis(amdToolMode)
    fun at(fraction: Double): DrivePose {
        val t = fraction.coerceIn(0.0, 1.0)
        // AMD arrows follow grid directions, without car-like arcs or rotating footprints.
        if (amdToolMode) {
            val (dx, dy) = when (command) {
                ManualCommand.FORWARD -> 0 to 1
                ManualCommand.REVERSE -> 0 to -1
                ManualCommand.LEFT -> -1 to 0
                ManualCommand.RIGHT -> 1 to 0
                ManualCommand.FORWARD_LEFT -> -1 to 1
                ManualCommand.FORWARD_RIGHT -> 1 to 1
                ManualCommand.BACK_LEFT -> -1 to -1
                ManualCommand.BACK_RIGHT -> 1 to -1
            }
            val heading = (Math.toDegrees(atan2(dx.toDouble(), dy.toDouble())) + 360.0) % 360.0
            return DrivePose(start.x + dx * t, start.y + dy * t, heading)
        }
        val h = Math.toRadians(start.heading)
        val sign = if (command.reverse) -1.0 else 1.0
        if (command.turn == 0.0) return DrivePose(start.x + sin(h) * MANUAL_STEP_CELLS * t * sign, start.y + cos(h) * MANUAL_STEP_CELLS * t * sign, start.heading)
        val delta = Math.toRadians(command.turn) * t
        val r = radius * sign * command.turn.sign
        return DrivePose(start.x + r * (cos(h) - cos(h + delta)), start.y + r * (sin(h + delta) - sin(h)),
            start.heading + command.turn * t)
    }

    /** Sweep with a clearance covering maximum corner travel between adjacent samples. */
    fun fits(state: ArenaState): Boolean {
        val steps = 180
        val distance = if (amdToolMode) sqrt(2.0) else if (command.turn == 0.0) MANUAL_STEP_CELLS
            else radius * Math.toRadians(abs(command.turn))
        val cornerRadius = hypot(state.config.robotWidthCells, state.config.robotLengthCells) / 2.0
        val rotation = if (amdToolMode) 0.0 else Math.toRadians(abs(command.turn))
        val sweepMargin = (distance + cornerRadius * rotation) / (steps * 2)
        for (i in 0..steps) {
            val sampled = at(i.toDouble() / steps)
            val pose = if (amdToolMode) sampled.copy(heading = 0.0) else sampled
            if (!RobotFootprint.fitsArena(pose, state.config)) return false
            if (state.obstacles.values.any {
                RobotFootprint.overlapsCell(pose, state.config, it.position,
                    if (i == 0 || i == steps) 0.0 else sweepMargin)
            }) return false
        }
        return true
    }
}

/** Reserves a path synchronously until the caller completes it; uncertainty blocks movement. */
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
        pose = DrivePose.from(robot)
        uncertain = false
        if (pending != null) reportedDuringMove = robot
        return true
    }
    /** Operator-placed pose (drag and drop): trusted as the new known pose. Never mid-move. */
    fun seed(robot: RobotPose) {
        check(pending == null) { "Cannot seed the pose while a command is pending" }
        pose = DrivePose.from(robot); uncertain = false; reportedDuringMove = null
    }
    fun allowed(state: ArenaState, command: ManualCommand, amdToolMode: Boolean = false): Boolean =
        !uncertain && pending == null && pose?.let { DrivePath(it, command, amdToolMode = amdToolMode).fits(state) } == true
    fun reserve(state: ArenaState, command: ManualCommand, amdToolMode: Boolean = false): DrivePath? {
        if (!allowed(state, command, amdToolMode)) return null
        return DrivePath(requireNotNull(pose), command, amdToolMode = amdToolMode).also { pending = it; reportedDuringMove = null }
    }
    fun complete() { pending?.let { pose = reportedDuringMove?.let(DrivePose::from) ?: it.at(1.0) }; pending = null }
    fun flagUncertain() { uncertain = true }
    fun invalidate() { pending = null; pose = null; uncertain = true; reportedDuringMove = null }
}
