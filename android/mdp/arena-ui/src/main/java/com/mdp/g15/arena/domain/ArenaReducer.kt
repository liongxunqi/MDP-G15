package com.mdp.g15.arena.domain

class ArenaReducer {
    fun reduce(state: ArenaState, action: ArenaAction): ArenaReduction = when (action) {
        is ArenaAction.AddObstacle -> addObstacle(state, action.position)
        is ArenaAction.MoveObstacle -> moveObstacle(state, action.obstacleId, action.destination)
        is ArenaAction.RemoveObstacle -> removeObstacle(state, action.obstacleId)
        is ArenaAction.SelectObstacle -> selectObstacle(state, action.obstacleId)
        is ArenaAction.SetTargetFace -> setTargetFace(state, action.obstacleId, action.face)
        is ArenaAction.ApplyTarget -> applyTarget(state, action)
        is ArenaAction.ApplyRobotPose -> applyRobotPose(state, action.pose)
        is ArenaAction.MoveRobot -> moveRobot(state, action.destination)
        ArenaAction.Reset -> ArenaReduction.Success(ArenaState(config = state.config))
    }

    private fun addObstacle(state: ArenaState, position: GridCoordinate): ArenaReduction {
        validateFreeCell(state, position)?.let { return ArenaReduction.Failure(it) }

        val obstacle = Obstacle(id = state.nextObstacleId, position = position)
        return ArenaReduction.Success(
            state = state.copy(
                obstacles = state.obstacles + (obstacle.id to obstacle),
                selectedObstacleId = obstacle.id,
                nextObstacleId = obstacle.id + 1,
            ),
            outboundEvent = ArenaOutboundEvent.UpsertObstacle(obstacle),
        )
    }

    private fun moveObstacle(
        state: ArenaState,
        obstacleId: Int,
        destination: GridCoordinate,
    ): ArenaReduction {
        val obstacle = state.obstacles[obstacleId]
            ?: return ArenaReduction.Failure("Obstacle $obstacleId does not exist.")
        if (!state.config.contains(destination)) {
            return ArenaReduction.Failure("Destination is outside the arena.")
        }
        if (state.obstacles.values.any { it.id != obstacleId && it.position == destination }) {
            return ArenaReduction.Failure("Another obstacle already occupies that cell.")
        }
        if (obstacle.position == destination) {
            return ArenaReduction.Success(state.copy(selectedObstacleId = obstacleId))
        }

        val moved = obstacle.copy(position = destination)
        return ArenaReduction.Success(
            state = state.copy(
                obstacles = state.obstacles + (obstacleId to moved),
                selectedObstacleId = obstacleId,
            ),
            outboundEvent = ArenaOutboundEvent.UpsertObstacle(moved),
        )
    }

    private fun removeObstacle(state: ArenaState, obstacleId: Int): ArenaReduction {
        if (obstacleId !in state.obstacles) {
            return ArenaReduction.Failure("Obstacle $obstacleId does not exist.")
        }
        return ArenaReduction.Success(
            state = state.copy(
                obstacles = state.obstacles - obstacleId,
                selectedObstacleId = state.selectedObstacleId.takeUnless { it == obstacleId },
            ),
            outboundEvent = ArenaOutboundEvent.RemoveObstacle(obstacleId),
        )
    }

    private fun selectObstacle(state: ArenaState, obstacleId: Int?): ArenaReduction {
        if (obstacleId != null && obstacleId !in state.obstacles) {
            return ArenaReduction.Failure("Obstacle $obstacleId does not exist.")
        }
        return ArenaReduction.Success(state.copy(selectedObstacleId = obstacleId))
    }

    private fun setTargetFace(
        state: ArenaState,
        obstacleId: Int,
        face: Direction,
    ): ArenaReduction {
        val obstacle = state.obstacles[obstacleId]
            ?: return ArenaReduction.Failure("Select an obstacle before choosing its target face.")
        val updated = obstacle.copy(targetFace = face)
        return ArenaReduction.Success(
            state = state.copy(
                obstacles = state.obstacles + (obstacleId to updated),
                selectedObstacleId = obstacleId,
            ),
            outboundEvent = ArenaOutboundEvent.UpsertObstacle(updated),
        )
    }

    private fun applyTarget(state: ArenaState, action: ArenaAction.ApplyTarget): ArenaReduction {
        val obstacle = state.obstacles[action.obstacleId]
            ?: return ArenaReduction.Failure("Target references unknown obstacle ${action.obstacleId}.")
        if (action.targetId.isBlank()) {
            return ArenaReduction.Failure("Target ID cannot be blank.")
        }
        val updated = obstacle.copy(
            targetId = action.targetId,
            targetFace = action.face ?: obstacle.targetFace,
        )
        return ArenaReduction.Success(
            state.copy(obstacles = state.obstacles + (updated.id to updated)),
        )
    }

    private fun applyRobotPose(state: ArenaState, pose: RobotPose): ArenaReduction {
        if (!state.config.contains(pose.position)) {
            return ArenaReduction.Failure(
                "Robot coordinate (${pose.position.x}, ${pose.position.y}) is outside the arena.",
            )
        }
        return ArenaReduction.Success(state.copy(robot = pose))
    }

    private fun moveRobot(state: ArenaState, destination: GridCoordinate): ArenaReduction {
        val robot = state.robot ?: return ArenaReduction.Failure("Robot position is not set.")
        if (!state.config.contains(destination)) {
            return ArenaReduction.Failure("Destination is outside the arena.")
        }
        if (state.obstacles.values.any { it.position == destination }) {
            return ArenaReduction.Failure("An obstacle already occupies that cell.")
        }
        if (robot.position == destination) {
            return ArenaReduction.Success(state)
        }
        return ArenaReduction.Success(state.copy(robot = robot.copy(position = destination)))
    }

    private fun validateFreeCell(state: ArenaState, position: GridCoordinate): String? = when {
        !state.config.contains(position) -> "Selected cell is outside the arena."
        state.obstacles.values.any { it.position == position } ->
            "Another obstacle already occupies that cell."
        else -> null
    }
}
