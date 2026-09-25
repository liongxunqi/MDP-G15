package com.mdp.g15.arena.domain

import kotlin.math.*


const val MANUAL_STEP_CELLS = 1.0

/** How far (cells) a turning robot's bounding box may poke past the arena boundary. */
const val TURN_EDGE_OVERHANG_CELLS = 0.3
/** Defaults from the RPi manual bridge and STM TIGHT profile. Units are 10 cm cells. */
enum class ManualCommand(val wire: String, val reverse: Boolean = false, val turn: Double = 0.0) {
    FORWARD("f"), REVERSE("r", true), LEFT("tl", turn = -90.0), RIGHT("tr", turn = 90.0),
    FORWARD_LEFT("fl", turn = -45.0), FORWARD_RIGHT("fr", turn = 45.0),
    BACK_LEFT("bl", true, 45.0), BACK_RIGHT("br", true, -45.0);

    fun previewDurationMillis(amdToolMode: Boolean): Long =
        if (amdToolMode) 180L else if (turn == 0.0) 650L else 1000L

    /** Every arc command (forward or back, left or right) ends on a cardinal heading, a 90° step from the start. */
    val snapsToQuarterTurn: Boolean get() = this == FORWARD_LEFT || this == FORWARD_RIGHT || this == BACK_LEFT || this == BACK_RIGHT
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
            // Diagonals face their lateral direction; all stored headings remain cardinal.
            val heading = if (dx < 0) 270.0 else if (dx > 0) 90.0 else if (dy > 0) 0.0 else 180.0
            return DrivePose(start.x + dx * t, start.y + dy * t, heading)
        }
        val h = Math.toRadians(start.heading)
        val sign = if (command.reverse) -1.0 else 1.0
        if (command.turn == 0.0) return DrivePose(start.x + sin(h) * MANUAL_STEP_CELLS * t * sign, start.y + cos(h) * MANUAL_STEP_CELLS * t * sign, start.heading)
        val delta = Math.toRadians(command.turn) * t
        val r = radius * sign * command.turn.sign
        return DrivePose(start.x + r * (cos(h) - cos(h + delta)), start.y + r * (sin(h + delta) - sin(h)), headingAt(t))
    }

    /**
     * Forward/back left/right arcs: the drawn heading sweeps through intermediate angles, but the
     * end heading is derived from the starting cardinal direction (±90°), never
     * accumulated from the animation angle. Position keeps using the arc geometry above.
     */
    private fun headingAt(t: Double): Double {
        if (!command.snapsToQuarterTurn) return start.heading + command.turn * t
        val end = (start.heading / 90.0).roundToInt() * 90.0 + command.turn.sign * 90.0
        return if (t >= 1.0) end else start.heading + (end - start.heading) * t
    }

    /** Conservative swept AABBs between closely spaced samples, including a rotating square. */
    fun fits(state: ArenaState): Boolean {
        fun bounds(p: DrivePose): DoubleArray {
            val size = state.config.robotFootprintCells
            return if (amdToolMode) doubleArrayOf(p.x, p.y, p.x + size, p.y + size) else p.bounds(size)
        }

        var previous = bounds(at(0.0))
        for (i in 1..180) {
            val next = bounds(at(i / 180.0))
            // Arc sagitta and rotation between samples are below 0.001 cell.
            val margin = if (amdToolMode || command.turn == 0.0) 0.0 else 0.001
            val l = min(previous[0], next[0]) - margin
            val b = min(previous[1], next[1]) - margin
            val r = max(previous[2], next[2]) + margin
            val t = max(previous[3], next[3]) + margin
            // The rotating square's bounding box overhangs the arena edge near the start of a turn
            // from an edge cell; allow that for turns only. Obstacles below stay strict.
            val edge = 1e-8 + if (amdToolMode || command.turn == 0.0) 0.0 else TURN_EDGE_OVERHANG_CELLS
            if (l < -edge || b < -edge || r > state.config.columns + edge || t > state.config.rows + edge) return false
            if (state.obstacles.values.any { it.position.x < r - 1e-8 && it.position.x + 1 > l + 1e-8 && it.position.y < t - 1e-8 && it.position.y + 1 > b + 1e-8 }) return false
            previous = next
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
        if (pending == null) { pose = DrivePose.from(robot); uncertain = false }
        else {
            if (DrivePose.from(robot).samePose(pending!!.start)) return false
            reportedDuringMove = robot
        }
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
