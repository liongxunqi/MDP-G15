package com.mdp.g15.arena.domain

data class ArenaConfig(
    val columns: Int = 20,
    val rows: Int = 20,
    val robotWidthCells: Double = 2.0,
    val robotLengthCells: Double = 2.1,
    val maxObstacles: Int = 50,
) {
    init {
        require(columns > 0) { "Arena columns must be positive." }
        require(rows > 0) { "Arena rows must be positive." }
        require(robotWidthCells.isFinite() && robotWidthCells > 0) { "Robot width must be positive and finite." }
        require(robotLengthCells.isFinite() && robotLengthCells > 0) { "Robot length must be positive and finite." }
        require(maxObstacles > 0) { "Arena must allow at least one obstacle." }
    }

    fun contains(coordinate: GridCoordinate): Boolean =
        coordinate.x in 0 until columns && coordinate.y in 0 until rows
}

data class GridCoordinate(val x: Int, val y: Int)

/** The adjacent cell one step in [direction] (NORTH = +y, per the grid's bottom-left origin). */
fun GridCoordinate.step(direction: Direction): GridCoordinate = when (direction) {
    Direction.NORTH -> copy(y = y + 1)
    Direction.EAST -> copy(x = x + 1)
    Direction.SOUTH -> copy(y = y - 1)
    Direction.WEST -> copy(x = x - 1)
}

/** All cells of a [size] x [size] square footprint centred on this cell. */
fun GridCoordinate.footprint(size: Int): List<GridCoordinate> =
    (-size / 2..size / 2).flatMap { dx -> (-size / 2..size / 2).map { dy -> GridCoordinate(x + dx, y + dy) } }

enum class Direction(val wireValue: String) {
    NORTH("N"),
    EAST("E"),
    SOUTH("S"),
    WEST("W");

    fun opposite(): Direction = when (this) {
        NORTH -> SOUTH
        EAST -> WEST
        SOUTH -> NORTH
        WEST -> EAST
    }

    companion object {
        fun fromWire(value: String): Direction? = when (value.trim().uppercase()) {
            "N", "NORTH" -> NORTH
            "E", "EAST" -> EAST
            "S", "SOUTH" -> SOUTH
            "W", "WEST" -> WEST
            else -> null
        }
    }
}

data class RobotPose(
    val x: Double,
    val y: Double,
    val heading: Double,
    /** Local command estimate; never encoded as measured telemetry. */
    val estimate: RobotPose? = null,
    val commandedPath: DrivePath? = null,
    val source: PoseSource = PoseSource.REPORTED,
) {
    init {
        require(x.isFinite() && y.isFinite() && heading.isFinite()) { "Robot pose must be finite." }
    }

    /** Kept as an adapter for integer placement and older callers; not the pose source of truth. */
    constructor(position: GridCoordinate, direction: Direction, estimate: RobotPose? = null,
                commandedPath: DrivePath? = null) : this(position.x.toDouble(), position.y.toDouble(),
        direction.ordinal * 90.0, estimate, commandedPath)

    val position: GridCoordinate get() = GridCoordinate(kotlin.math.round(x).toInt(), kotlin.math.round(y).toInt())
    val direction: Direction get() = Direction.entries[
        ((kotlin.math.round(heading / 90.0).toInt() % 4) + 4) % 4
    ]
    val normalizedHeading: Double get() = ((heading % 360.0) + 360.0) % 360.0
    fun withoutPreview(): RobotPose = if (estimate == null && commandedPath == null) this else copy(estimate = null, commandedPath = null)
    fun asRobotPose(): RobotPose = withoutPreview()
    fun samePose(other: RobotPose): Boolean = kotlin.math.abs(x - other.x) < 1e-8 &&
        kotlin.math.abs(y - other.y) < 1e-8 &&
        kotlin.math.abs(((heading - other.heading) % 360.0 + 540.0) % 360.0 - 180.0) < 1e-8

    companion object {
        fun from(pose: RobotPose): RobotPose = pose.estimate ?: pose.withoutPreview()
    }
}

enum class PoseSource { REPORTED, OPERATOR, ESTIMATED }

fun RobotPose.compassLabel(): String {
    val labels = arrayOf("North", "Northeast", "East", "Southeast", "South", "Southwest", "West", "Northwest")
    return labels[(kotlin.math.floor((normalizedHeading + 22.5) / 45.0).toInt()) % 8]
}

fun RobotPose.readableHeading(): String {
    val angle = (kotlin.math.round(normalizedHeading * 10.0) / 10.0) % 360.0
    val number = if (angle == kotlin.math.floor(angle)) angle.toInt().toString()
        else String.format(java.util.Locale.US, "%.1f", angle)
    return "$number° · ${copy(heading = angle).compassLabel()}"
}

data class Obstacle(
    val id: Int,
    val position: GridCoordinate,
    val targetFace: Direction? = null,
    val targetId: String? = null,
) {
    init {
        require(id > 0) { "Obstacle IDs must be positive." }
    }
}

data class ArenaState(
    val config: ArenaConfig = ArenaConfig(),
    val robot: RobotPose? = null,
    val reportedRobot: RobotPose? = null,
    val reportRevision: Long = 0,
    val obstacles: Map<Int, Obstacle> = emptyMap(),
    val selectedObstacleId: Int? = null,
) {
    val selectedObstacle: Obstacle?
        get() = selectedObstacleId?.let(obstacles::get)

    /** Smallest positive ID not currently in use — removed IDs are reused before new ones. */
    fun nextFreeObstacleId(): Int {
        var candidate = 1
        while (candidate in obstacles) candidate++
        return candidate
    }
}

data class ArenaEditSnapshot(
    val obstacles: Map<Int, Obstacle>,
    val selectedObstacleId: Int?,
    val robot: RobotPose?,
    val poseRevision: Long,
) {
    fun applyTo(state: ArenaState, currentPoseRevision: Long): ArenaState = state.copy(
        obstacles = obstacles,
        selectedObstacleId = selectedObstacleId?.takeIf(obstacles::containsKey),
        robot = if (poseRevision == currentPoseRevision) robot else state.robot,
    )

    companion object {
        fun from(state: ArenaState, poseRevision: Long): ArenaEditSnapshot = ArenaEditSnapshot(
            obstacles = state.obstacles,
            selectedObstacleId = state.selectedObstacleId,
            robot = state.robot,
            poseRevision = poseRevision,
        )
    }
}
