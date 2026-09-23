package com.mdp.g15.arena.domain

import org.junit.Assert.*
import org.junit.Test

class ManualDriveTest {
    @Test fun `heading wraparound represents the same physical pose`() {
        assertTrue(DrivePose(8.0, 8.0, -90.0).samePose(DrivePose(8.0, 8.0, 270.0)))
        assertTrue(DrivePose(8.0, 8.0, 360.0).samePose(DrivePose(8.0, 8.0, 0.0)))
        assertFalse(DrivePose(8.0, 8.0, 0.0).samePose(DrivePose(8.0, 8.0, 45.0)))
    }
    private fun arc(from: DrivePose, command: ManualCommand) = DrivePath(from, command).at(1.0)
    @Test fun `left and right rotate on the spot and update the direction`() {
        val start = DrivePose(8.0, 8.0, 0.0)
        val right = arc(start, ManualCommand.RIGHT)
        assertEquals(8.0, right.x, 1e-9)
        assertEquals(8.0, right.y, 1e-9)
        assertEquals(90.0, right.heading, 1e-9)
        assertEquals(Direction.EAST, right.asRobotPose().direction)
        assertEquals(GridCoordinate(8, 8), right.asRobotPose().position)

        val left = arc(start, ManualCommand.LEFT)
        assertEquals(GridCoordinate(8, 8), left.asRobotPose().position)
        assertEquals(Direction.WEST, left.asRobotPose().direction)

        // Four right turns come back to facing north, and the next Forward heads north again.
        var pose = start
        repeat(4) { pose = arc(pose, ManualCommand.RIGHT) }
        assertEquals(Direction.NORTH, pose.asRobotPose().direction)
        assertEquals(9.0, arc(pose, ManualCommand.FORWARD).y, 1e-9)
        // Turned east, Forward now moves along +x.
        assertEquals(9.0, arc(right, ManualCommand.FORWARD).x, 1e-9)
    }
    @Test fun `rotating on the spot is allowed against walls and obstacles but moving still is not`() {
        // Flush in the corner: a rotating square would sweep past the wall, but a pivot doesn't sweep.
        val corner = state(0, 0)
        assertTrue(DrivePath(DrivePose.from(corner.robot!!), ManualCommand.LEFT).fits(corner))
        assertTrue(DrivePath(DrivePose.from(corner.robot!!), ManualCommand.RIGHT).fits(corner))
        assertFalse(DrivePath(DrivePose.from(corner.robot!!), ManualCommand.REVERSE).fits(corner))

        // Obstacle touching the robot's side (robot 2x2 at 8,8 covers x 8..10): turning is fine,
        // driving into it or arcing past it is not.
        val boxed = state(8, 8).copy(obstacles = mapOf(1 to Obstacle(1, GridCoordinate(10, 8)), 2 to Obstacle(2, GridCoordinate(8, 10))))
        val pose = DrivePose.from(boxed.robot!!)
        assertTrue(DrivePath(pose, ManualCommand.RIGHT).fits(boxed))
        assertTrue(DrivePath(pose, ManualCommand.LEFT).fits(boxed))
        assertFalse(DrivePath(pose, ManualCommand.FORWARD).fits(boxed))
        assertFalse(DrivePath(pose, ManualCommand.FORWARD_RIGHT).fits(boxed))
    }
    @Test fun `arcs preserve fractional coordinates and four 45 degree arcs make a u turn`() {
        val start = DrivePose(8.0, 8.0, 0.0)
        val quarter = arc(arc(start, ManualCommand.FORWARD_RIGHT), ManualCommand.FORWARD_RIGHT)
        assertEquals(10.91, quarter.x, 0.0001)
        assertEquals(10.91, quarter.y, 0.0001)
        val uturn = arc(arc(quarter, ManualCommand.FORWARD_RIGHT), ManualCommand.FORWARD_RIGHT)
        assertEquals(13.82, uturn.x, 0.0001)
        assertEquals(8.0, uturn.y, 0.0001)
        assertEquals(180.0, uturn.heading, 0.0001)
        val reverseLeft = DrivePath(start, ManualCommand.BACK_LEFT).at(1.0)
        assertTrue(reverseLeft.x < start.x && reverseLeft.y < start.y)
        assertEquals(45.0, reverseLeft.heading, 0.0001)
        assertEquals(reverseLeft, DrivePose.from(reverseLeft.asRobotPose()))
    }
    private fun state(x: Int = 8, y: Int = 8, direction: Direction = Direction.NORTH) = ArenaState(robot = RobotPose(GridCoordinate(x, y), direction))
    @Test fun `all cardinal boundaries account for one cell distance and footprint`() {
        // A 2x2 robot on the last cell that still fits: one more step forward leaves the arena.
        for ((direction, xy) in listOf(Direction.NORTH to GridCoordinate(8,18), Direction.EAST to GridCoordinate(18,8), Direction.SOUTH to GridCoordinate(8,0), Direction.WEST to GridCoordinate(0,8))) {
            val s = state(xy.x, xy.y, direction)
            assertFalse(DrivePath(DrivePose.from(s.robot!!), ManualCommand.FORWARD).fits(s))
        }
        // One cell further in from each edge, the same step is still allowed.
        for ((direction, xy) in listOf(Direction.NORTH to GridCoordinate(8,17), Direction.EAST to GridCoordinate(17,8), Direction.SOUTH to GridCoordinate(8,1), Direction.WEST to GridCoordinate(1,8))) {
            val s = state(xy.x, xy.y, direction)
            assertTrue(DrivePath(DrivePose.from(s.robot!!), ManualCommand.FORWARD).fits(s))
        }
    }
    @Test fun `forward and reverse each move exactly one cell`() {
        val start = DrivePose(8.0, 8.0, 0.0)
        assertEquals(9.0, DrivePath(start, ManualCommand.FORWARD).at(1.0).y, 1e-9)
        assertEquals(7.0, DrivePath(start, ManualCommand.REVERSE).at(1.0).y, 1e-9)
    }
    @Test fun `swept path catches obstacles even when endpoint is clear`() {
        val s = state(8, 8).copy(obstacles = mapOf(1 to Obstacle(1, GridCoordinate(8,10))))
        assertFalse(DrivePath(DrivePose.from(s.robot!!), ManualCommand.FORWARD).fits(s))
        val corner = state(17, 8)
        assertFalse(DrivePath(DrivePose.from(corner.robot!!), ManualCommand.FORWARD_RIGHT).fits(corner))
    }
    @Test fun `spam and stale reports cannot release pending command`() {
        val s = state()
        val gate = ManualDriveGuard()
        assertNull(gate.reserve(s, ManualCommand.FORWARD))
        gate.report(s.robot!!)
        assertNotNull(gate.reserve(s, ManualCommand.FORWARD))
        repeat(100) { gate.report(s.robot); assertNull(gate.reserve(s, ManualCommand.FORWARD)) }
        gate.complete()
        assertEquals(9.0, gate.pose!!.y, 0.00001)
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
