package com.mdp.g15.arena.domain

data class ArenaConfig(
    val columns: Int = 20,
    val rows: Int = 20,
    val robotFootprintCells: Int = 1,
) {
    init {
        require(columns > 0) { "Arena columns must be positive." }
        require(rows > 0) { "Arena rows must be positive." }
        require(robotFootprintCells > 0) { "Robot footprint must be positive." }
    }

    fun contains(coordinate: GridCoordinate): Boolean =
        coordinate.x in 0 until columns && coordinate.y in 0 until rows
}

data class GridCoordinate(val x: Int, val y: Int)

enum class Direction(val wireValue: String) {
    NORTH("N"),
    EAST("E"),
    SOUTH("S"),
    WEST("W");

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
    val position: GridCoordinate,
    val direction: Direction,
)

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
    val obstacles: Map<Int, Obstacle> = emptyMap(),
    val selectedObstacleId: Int? = null,
    val nextObstacleId: Int = 1,
) {
    val selectedObstacle: Obstacle?
        get() = selectedObstacleId?.let(obstacles::get)
}

data class ArenaEditSnapshot(
    val obstacles: Map<Int, Obstacle>,
    val selectedObstacleId: Int?,
    val nextObstacleId: Int,
    val robot: RobotPose?,
) {
    fun applyTo(state: ArenaState): ArenaState = state.copy(
        obstacles = obstacles,
        selectedObstacleId = selectedObstacleId?.takeIf(obstacles::containsKey),
        nextObstacleId = nextObstacleId,
        robot = robot,
    )

    companion object {
        fun from(state: ArenaState): ArenaEditSnapshot = ArenaEditSnapshot(
            obstacles = state.obstacles,
            selectedObstacleId = state.selectedObstacleId,
            nextObstacleId = state.nextObstacleId,
            robot = state.robot,
        )
    }
}
