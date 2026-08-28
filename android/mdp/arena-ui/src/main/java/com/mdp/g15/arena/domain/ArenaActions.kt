package com.mdp.g15.arena.domain

sealed interface ArenaAction {
    data class AddObstacle(val position: GridCoordinate) : ArenaAction
    data class MoveObstacle(val obstacleId: Int, val destination: GridCoordinate) : ArenaAction
    data class RemoveObstacle(val obstacleId: Int) : ArenaAction
    data class SelectObstacle(val obstacleId: Int?) : ArenaAction
    data class SetTargetFace(val obstacleId: Int, val face: Direction) : ArenaAction
    data class ApplyTarget(
        val obstacleId: Int,
        val targetId: String,
        val face: Direction? = null,
    ) : ArenaAction

    data class ApplyRobotPose(val pose: RobotPose) : ArenaAction
    data object Reset : ArenaAction
}

sealed interface ArenaOutboundEvent {
    data class UpsertObstacle(val obstacle: Obstacle) : ArenaOutboundEvent
    data class RemoveObstacle(val obstacleId: Int) : ArenaOutboundEvent
}

sealed interface ArenaReduction {
    data class Success(
        val state: ArenaState,
        val outboundEvent: ArenaOutboundEvent? = null,
    ) : ArenaReduction

    data class Failure(val reason: String) : ArenaReduction
}
