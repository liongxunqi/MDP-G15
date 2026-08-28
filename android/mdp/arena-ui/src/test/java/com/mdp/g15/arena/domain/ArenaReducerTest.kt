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
        val valid = RobotPose(GridCoordinate(19, 0), Direction.SOUTH)
        val validState = reducer.success(ArenaState(), ArenaAction.ApplyRobotPose(valid))
        assertEquals(valid, validState.robot)

        val invalid = reducer.reduce(
            validState,
            ArenaAction.ApplyRobotPose(RobotPose(GridCoordinate(20, 0), Direction.NORTH)),
        )
        assertTrue(invalid is ArenaReduction.Failure)
        assertFalse((invalid as ArenaReduction.Failure).reason.isBlank())
    }

    private fun ArenaReducer.success(state: ArenaState, action: ArenaAction): ArenaState =
        (reduce(state, action) as ArenaReduction.Success).state
}
