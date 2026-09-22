package com.mdp.g15.arena.domain

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ArenaReducerTest {
    private val reducer = ArenaReducer()

    @Test
    fun `add assigns stable increasing IDs and rejects occupied cell`() {
        val first = reducer.success(ArenaState(), ArenaAction.AddObstacle(GridCoordinate(4, 5)))
        val second = reducer.success(first, ArenaAction.AddObstacle(GridCoordinate(8, 9)))

        assertEquals(setOf(1, 2), second.obstacles.keys)
        assertEquals(3, second.nextObstacleId)
        assertTrue(
            reducer.reduce(second, ArenaAction.AddObstacle(GridCoordinate(4, 5)))
                is ArenaReduction.Failure,
        )

        val removed = reducer.success(second, ArenaAction.RemoveObstacle(1))
        val third = reducer.success(removed, ArenaAction.AddObstacle(GridCoordinate(1, 1)))
        assertEquals(setOf(2, 3), third.obstacles.keys)
    }

    @Test
    fun `add is rejected once the configured obstacle limit is reached`() {
        val capped = ArenaState(config = ArenaConfig(maxObstacles = 2))
        val first = reducer.success(capped, ArenaAction.AddObstacle(GridCoordinate(0, 0)))
        val second = reducer.success(first, ArenaAction.AddObstacle(GridCoordinate(1, 0)))
        assertEquals(2, second.obstacles.size)

        val result = reducer.reduce(second, ArenaAction.AddObstacle(GridCoordinate(2, 0)))
        assertTrue(result is ArenaReduction.Failure)
        assertEquals(2, second.obstacles.size)
    }

    @Test
    fun `move preserves obstacle metadata and rejects collisions`() {
        var state = reducer.success(ArenaState(), ArenaAction.AddObstacle(GridCoordinate(1, 1)))
        state = reducer.success(state, ArenaAction.SetTargetFace(1, Direction.WEST))
        state = reducer.success(state, ArenaAction.ApplyTarget(1, "11"))
        state = reducer.success(state, ArenaAction.AddObstacle(GridCoordinate(2, 2)))

        val collided = reducer.reduce(state, ArenaAction.MoveObstacle(1, GridCoordinate(2, 2)))
        assertTrue(collided is ArenaReduction.Failure)

        val moved = reducer.success(state, ArenaAction.MoveObstacle(1, GridCoordinate(7, 3)))
        assertEquals(GridCoordinate(7, 3), moved.obstacles.getValue(1).position)
        assertEquals(Direction.WEST, moved.obstacles.getValue(1).targetFace)
        assertEquals("11", moved.obstacles.getValue(1).targetId)
    }

    @Test
    fun `remove clears selection without renumbering remaining obstacle`() {
        var state = reducer.success(ArenaState(), ArenaAction.AddObstacle(GridCoordinate(1, 1)))
        state = reducer.success(state, ArenaAction.AddObstacle(GridCoordinate(2, 2)))
        state = reducer.success(state, ArenaAction.SelectObstacle(1))

        val removed = reducer.success(state, ArenaAction.RemoveObstacle(1))

        assertNull(removed.selectedObstacleId)
        assertEquals(setOf(2), removed.obstacles.keys)
        assertEquals(3, removed.nextObstacleId)
    }

    @Test
    fun `target requires an existing obstacle and optional face updates rendering state`() {
        val emptyResult = reducer.reduce(ArenaState(), ArenaAction.ApplyTarget(1, "4", Direction.NORTH))
        assertTrue(emptyResult is ArenaReduction.Failure)

        val withObstacle = reducer.success(
            ArenaState(),
            ArenaAction.AddObstacle(GridCoordinate(10, 6)),
        )
        val targeted = reducer.success(
            withObstacle,
            ArenaAction.ApplyTarget(1, "4", Direction.NORTH),
        )
        assertEquals("4", targeted.obstacles.getValue(1).targetId)
        assertEquals(Direction.NORTH, targeted.obstacles.getValue(1).targetFace)
    }

    @Test
    fun `robot accepts only in-bounds pose`() {
        // Default footprint is 2x2 anchored bottom-left, so (18,0) is the last column that fits.
        val valid = RobotPose(GridCoordinate(18, 0), Direction.SOUTH)
        val validState = reducer.success(ArenaState(), ArenaAction.ApplyRobotPose(valid))
        assertEquals(valid, validState.robot)

        val invalid = reducer.reduce(
            validState,
            ArenaAction.ApplyRobotPose(RobotPose(GridCoordinate(20, 0), Direction.NORTH)),
        )
        assertTrue(invalid is ArenaReduction.Failure)
        assertFalse((invalid as ArenaReduction.Failure).reason.isBlank())
    }

    @Test
    fun `obstacle cannot be placed or moved onto a cell the robot occupies`() {
        // Robot at (5,5) with the default 2x2 footprint occupies (5,5) (6,5) (5,6) (6,6).
        val withRobot = reducer.success(
            ArenaState(),
            ArenaAction.ApplyRobotPose(RobotPose(GridCoordinate(5, 5), Direction.NORTH)),
        )

        assertTrue(
            reducer.reduce(withRobot, ArenaAction.AddObstacle(GridCoordinate(6, 6)))
                is ArenaReduction.Failure,
        )

        val withObstacle = reducer.success(withRobot, ArenaAction.AddObstacle(GridCoordinate(0, 0)))
        assertTrue(
            reducer.reduce(withObstacle, ArenaAction.MoveObstacle(1, GridCoordinate(6, 5)))
                is ArenaReduction.Failure,
        )
    }

    @Test
    fun `robot pose rejected when its footprint would stick out past the edge`() {
        // (19,0) is itself a valid single cell, but a 2x2 footprint anchored there needs
        // column 20, which doesn't exist.
        val result = reducer.reduce(
            ArenaState(),
            ArenaAction.ApplyRobotPose(RobotPose(GridCoordinate(19, 0), Direction.SOUTH)),
        )
        assertTrue(result is ArenaReduction.Failure)
    }

    @Test
    fun `all four robot cells reject obstacles and received poses reject collisions`() {
        val pose = RobotPose(GridCoordinate(5, 5), Direction.NORTH)
        val state = ArenaState(robot = pose, obstacles = mapOf(1 to Obstacle(1, GridCoordinate(0, 0))))
        assertEquals(2, state.config.robotFootprintCells)
        for (cell in pose.position.footprint(2)) {
            assertTrue(reducer.reduce(state, ArenaAction.AddObstacle(cell)) is ArenaReduction.Failure)
            assertTrue(reducer.reduce(state, ArenaAction.MoveObstacle(1, cell)) is ArenaReduction.Failure)
            val occupied = ArenaState(obstacles = mapOf(1 to Obstacle(1, cell)))
            assertTrue(reducer.reduce(occupied, ArenaAction.ApplyRobotPose(pose)) is ArenaReduction.Failure)
            assertFalse(reducer.canMoveRobot(occupied.copy(robot = RobotPose(GridCoordinate(0, 0), Direction.NORTH)), pose.position))
        }
        assertTrue(reducer.reduce(state, ArenaAction.AddObstacle(GridCoordinate(7, 5))) is ArenaReduction.Success)
    }

    @Test
    fun `invalid robot moves preserve the last valid pose at every edge`() {
        val pose = RobotPose(GridCoordinate(5, 5), Direction.EAST)
        val state = ArenaState(robot = pose)
        for (destination in listOf(GridCoordinate(-1, 5), GridCoordinate(5, -1), GridCoordinate(19, 5), GridCoordinate(5, 19))) {
            assertTrue(reducer.reduce(state, ArenaAction.MoveRobot(destination)) is ArenaReduction.Failure)
            assertEquals(pose, state.robot)
        }
        assertTrue(reducer.canMoveRobot(state, GridCoordinate(18, 18)))
    }

    private fun ArenaReducer.success(state: ArenaState, action: ArenaAction): ArenaState =
        (reduce(state, action) as ArenaReduction.Success).state
}
