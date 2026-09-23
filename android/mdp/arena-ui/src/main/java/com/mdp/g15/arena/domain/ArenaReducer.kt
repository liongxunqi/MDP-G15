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
        if (state.obstacles.size >= state.config.maxObstacles) {
            return ArenaReduction.Failure("Cannot place more than ${state.config.maxObstacles} obstacles.")
        }
        validateFreeCell(state, position, ignoreObstacleId = null)?.let { return ArenaReduction.Failure(it) }

        val obstacle = Obstacle(id = state.nextFreeObstacleId(), position = position)
        return ArenaReduction.Success(
            state = state.copy(
                obstacles = state.obstacles + (obstacle.id to obstacle),
                selectedObstacleId = obstacle.id,
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
        validateFreeCell(state, destination, ignoreObstacleId = obstacleId)?.let {
            return ArenaReduction.Failure(it)
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
        TargetId.error(action.targetId)?.let { return ArenaReduction.Failure(it) }
        val updated = obstacle.copy(
            targetId = action.targetId,
            targetFace = action.face ?: obstacle.targetFace,
        )
        return ArenaReduction.Success(
            state.copy(obstacles = state.obstacles + (updated.id to updated)),
        )
    }

    private fun applyRobotPose(state: ArenaState, pose: RobotPose): ArenaReduction {
        val cells = pose.position.footprint(state.config.robotFootprintCells)
        if (cells.any { !state.config.contains(it) }) {
            return ArenaReduction.Failure(
                "Robot footprint at (${pose.position.x}, ${pose.position.y}) doesn't fit inside the arena.",
            )
        }
        if (state.obstacles.values.any { it.position in cells }) {
            return ArenaReduction.Failure("Robot footprint overlaps an obstacle.")
        }
        return ArenaReduction.Success(state.copy(robot = pose))
    }

    /** Read-only check: would moving the robot to [destination] succeed, without applying it. */
    fun canMoveRobot(state: ArenaState, destination: GridCoordinate): Boolean =
        moveRobot(state, destination) is ArenaReduction.Success

    private fun moveRobot(state: ArenaState, destination: GridCoordinate): ArenaReduction {
        val robot = state.robot ?: return ArenaReduction.Failure("Robot position is not set.")
        val cells = destination.footprint(state.config.robotFootprintCells)
        if (cells.any { !state.config.contains(it) }) {
            return ArenaReduction.Failure("Destination is outside the arena.")
        }
        if (state.obstacles.values.any { obstacle -> cells.any { it == obstacle.position } }) {
            return ArenaReduction.Failure("An obstacle already occupies that cell.")
        }
        if (robot.position == destination) {
            return ArenaReduction.Success(state)
        }
        return ArenaReduction.Success(state.copy(robot = robot.copy(position = destination, estimate = null, commandedPath = null)))
    }

    private fun validateFreeCell(
        state: ArenaState,
        position: GridCoordinate,
        ignoreObstacleId: Int?,
    ): String? = when {
        !state.config.contains(position) -> "Selected cell is outside the arena."
        state.obstacles.values.any { it.id != ignoreObstacleId && it.position == position } ->
            "Another obstacle already occupies that cell."
        state.robot != null && DrivePose.from(state.robot).overlaps(position, state.config.robotFootprintCells) ->
            "The robot occupies that cell."
        else -> null
    }
}
