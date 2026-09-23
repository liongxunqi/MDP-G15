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
    @Test fun `all eight commands have clear paths from arena center`() {
        val s = state()
        ManualCommand.entries.forEach { assertTrue(it.name, DrivePath(DrivePose.from(s.robot!!), it).fits(s)) }
    }
}
