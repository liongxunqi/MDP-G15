package com.mdp.g15.arena.domain

import org.junit.Assert.*
import org.junit.Test

class ManualDriveTest {
    @Test fun `heading wraparound represents the same physical pose`() {
        assertTrue(DrivePose(8.0, 8.0, -90.0).samePose(DrivePose(8.0, 8.0, 270.0)))
        assertTrue(DrivePose(8.0, 8.0, 360.0).samePose(DrivePose(8.0, 8.0, 0.0)))
        assertFalse(DrivePose(8.0, 8.0, 0.0).samePose(DrivePose(8.0, 8.0, 45.0)))
    }
    @Test fun `turns preserve fractional coordinates and a u turn covers two radii`() {
        val start = DrivePose(8.0, 8.0, 0.0)
        val quarter = DrivePath(start, ManualCommand.RIGHT).at(1.0)
        assertEquals(10.91, quarter.x, 0.0001)
        assertEquals(10.91, quarter.y, 0.0001)
        val uturn = DrivePath(quarter, ManualCommand.RIGHT).at(1.0)
        assertEquals(13.82, uturn.x, 0.0001)
        assertEquals(8.0, uturn.y, 0.0001)
        assertEquals(180.0, uturn.heading, 0.0001)
        val reverseLeft = DrivePath(start, ManualCommand.BACK_LEFT).at(1.0)
        assertTrue(reverseLeft.x < start.x && reverseLeft.y < start.y)
        assertEquals(45.0, reverseLeft.heading, 0.0001)
        assertEquals(reverseLeft, DrivePose.from(reverseLeft.asRobotPose()))
    }
    @Test fun `forward left and right end on a cardinal direction a quarter turn away`() {
        val left = listOf(Direction.NORTH to Direction.WEST, Direction.WEST to Direction.SOUTH,
            Direction.SOUTH to Direction.EAST, Direction.EAST to Direction.NORTH)
        val right = listOf(Direction.NORTH to Direction.EAST, Direction.EAST to Direction.SOUTH,
            Direction.SOUTH to Direction.WEST, Direction.WEST to Direction.NORTH)
        for ((command, cases) in listOf(ManualCommand.FORWARD_LEFT to left, ManualCommand.FORWARD_RIGHT to right)) {
            for ((from, to) in cases) {
                val start = DrivePose(8.0, 8.0, from.ordinal * 90.0)
                val end = DrivePath(start, command).at(1.0)
                assertEquals("$command $from", 0.0, end.heading % 90.0, 0.0)
                assertEquals("$command $from", to, end.asRobotPose().direction)
                // Chained turns start from the stored cardinal heading, so nothing drifts.
                assertEquals(0.0, DrivePath(end, command).at(1.0).heading % 90.0, 0.0)
            }
        }
    }
    @Test fun `forward left sweeps through intermediate headings`() {
        val path = DrivePath(DrivePose(8.0, 8.0, 0.0), ManualCommand.FORWARD_LEFT)
        assertEquals(-45.0, path.at(0.5).heading, 0.0001)
    }
    private fun state(x: Int = 8, y: Int = 8, direction: Direction = Direction.NORTH) = ArenaState(robot = RobotPose(GridCoordinate(x, y), direction))
    @Test fun `all cardinal boundaries account for two cell distance and footprint`() {
        for ((direction, xy) in listOf(Direction.NORTH to GridCoordinate(8,17), Direction.EAST to GridCoordinate(17,8), Direction.SOUTH to GridCoordinate(8,1), Direction.WEST to GridCoordinate(1,8))) {
            val s = state(xy.x, xy.y, direction)
            assertFalse(DrivePath(DrivePose.from(s.robot!!), ManualCommand.FORWARD).fits(s))
        }
    }
    @Test fun `swept path catches obstacles even when endpoint is clear`() {
        val s = state(8, 8).copy(obstacles = mapOf(1 to Obstacle(1, GridCoordinate(8,10))))
        assertFalse(DrivePath(DrivePose.from(s.robot!!), ManualCommand.FORWARD).fits(s))
        val corner = state(17, 8)
        assertFalse(DrivePath(DrivePose.from(corner.robot!!), ManualCommand.RIGHT).fits(corner))
    }
    @Test fun `spam and stale reports cannot release pending command`() {
        val s = state()
        val gate = ManualDriveGuard()
        assertNull(gate.reserve(s, ManualCommand.FORWARD))
        gate.report(s.robot!!)
        assertNotNull(gate.reserve(s, ManualCommand.FORWARD))
        repeat(100) { gate.report(s.robot); assertNull(gate.reserve(s, ManualCommand.FORWARD)) }
        gate.complete()
        assertEquals(10.0, gate.pose!!.y, 0.00001)
        assertNotNull(gate.reserve(s, ManualCommand.REVERSE))
        gate.invalidate()
        gate.complete()
        assertNull(gate.reserve(s, ManualCommand.FORWARD))
    }
    @Test fun `turns into the arena are allowed from an edge cell but straight moves stay strict`() {
        val left = state(0, 8, Direction.NORTH)
        for (c in listOf(ManualCommand.RIGHT, ManualCommand.FORWARD_RIGHT)) {
            assertTrue(c.name, DrivePath(DrivePose.from(left.robot!!), c).fits(left))
        }
        val right = state(18, 8, Direction.NORTH)
        for (c in listOf(ManualCommand.LEFT, ManualCommand.FORWARD_LEFT)) {
            assertTrue(c.name, DrivePath(DrivePose.from(right.robot!!), c).fits(right))
        }
        // Turning out of the arena, or a straight move past the edge, is still rejected.
        assertFalse(DrivePath(DrivePose.from(left.robot!!), ManualCommand.LEFT).fits(left))
        assertFalse(DrivePath(DrivePose(0.0, 8.0, 270.0), ManualCommand.FORWARD).fits(left))
    }
    @Test fun `seeding after an invalidate re-arms driving from the placed pose`() {
        val s = state()
        val gate = ManualDriveGuard()
        gate.invalidate()
        assertFalse(gate.allowed(s, ManualCommand.FORWARD))
        gate.seed(s.robot!!)
        assertTrue(gate.allowed(s, ManualCommand.FORWARD))
        assertEquals(8.0, gate.pose!!.x, 0.0)
    }
    @Test fun `all eight commands have clear paths from arena center`() {
        val s = state()
        ManualCommand.entries.forEach { assertTrue(it.name, DrivePath(DrivePose.from(s.robot!!), it).fits(s)) }
    }
}
