package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.*
import org.junit.Assert.*
import org.junit.Test

class RobotMotionTest {
    @Test fun `AMD lateral preview translates directly with its new heading`() {
        val motion = RobotMotion()
        val path = DrivePath(DrivePose(8.0,8.0,0.0), ManualCommand.LEFT, amdToolMode = true)
        motion.snap(state(8,8).robot)
        motion.retarget(ArenaState(robot = path.at(1.0).asRobotPose().copy(commandedPath = path)), 0)
        assertEquals(7.5f, motion.sample(90)!!.x, 0.001f)
        assertEquals(8f, motion.sample(90)!!.y, 0.001f)
        assertEquals(270f, motion.sample(90)!!.angle, 0.001f)
        assertEquals(7f, motion.sample(180)!!.x, 0.001f)
        assertFalse(motion.isRunning(180))
    }

    @Test fun `manual turn follows an arc with intermediate diagonal heading`() {
        val motion = RobotMotion()
        val path = DrivePath(DrivePose(8.0, 8.0, 0.0), ManualCommand.RIGHT)
        motion.snap(state(8, 8).robot)
        motion.retarget(ArenaState(robot = path.at(1.0).asRobotPose().copy(commandedPath = path)), 0)
        val halfway = motion.sample(500)!!
        assertEquals(45f, halfway.angle, 0.001f)
        assertEquals(path.at(0.5).x.toFloat(), halfway.x, 0.001f)
        assertTrue(halfway.x < (8f + 2.91f / 2)) // curved rather than a diagonal chord
        assertEquals(10.91f, motion.sample(1000)!!.x, 0.001f)
        assertEquals(10.91f, motion.sample(1000)!!.y, 0.001f)
    }
    @Test fun `straight preview moves smoothly and snaps cleanly on reset`() {
        val motion = RobotMotion()
        val path = DrivePath(DrivePose(8.0, 8.0, 0.0), ManualCommand.FORWARD)
        motion.snap(state(8, 8).robot)
        motion.retarget(ArenaState(robot = path.at(1.0).asRobotPose().copy(commandedPath = path)), 0)
        assertEquals(8.5f, motion.sample(325)!!.y, 0.001f)
        motion.snap(null)
        assertNull(motion.sample(650))
    }
    private fun state(x: Int, y: Int = 2, direction: Direction = Direction.NORTH) =
        ArenaState(robot = RobotPose(GridCoordinate(x, y), direction))

    @Test fun `rapid reports retarget continuously and settle at latest authoritative pose`() {
        val motion = RobotMotion()
        motion.retarget(state(2), 0)
        motion.retarget(state(4), 10)
        assertEquals(3f, motion.sample(100)!!.x, 0.001f)
        motion.retarget(state(6), 100)
        assertEquals(3f, motion.sample(100)!!.x, 0.001f)
        assertEquals(6f, motion.sample(280)!!.x, 0.001f)
        assertFalse(motion.isRunning(280))
    }

    @Test fun `obstacles and ambiguous corners are never crossed by interpolation`() {
        val motion = RobotMotion()
        motion.snap(state(2).robot)
        motion.retarget(state(8).copy(obstacles = mapOf(1 to Obstacle(1, GridCoordinate(5, 2)))), 0)
        assertEquals(8f, motion.sample(0)!!.x, 0f)
        motion.retarget(state(10, 4), 0)
        assertEquals(4f, motion.sample(0)!!.y, 0f)
        assertFalse(motion.isRunning(0))
    }

    @Test fun `rotation takes shortest arc and reset clears animation`() {
        val motion = RobotMotion()
        motion.snap(state(2, direction = Direction.WEST).robot)
        motion.retarget(state(2), 0)
        assertEquals(315f, motion.sample(90)!!.angle, 0.001f)
        motion.snap(null)
        assertNull(motion.sample(100))
        assertFalse(motion.isRunning(100))
    }
}
