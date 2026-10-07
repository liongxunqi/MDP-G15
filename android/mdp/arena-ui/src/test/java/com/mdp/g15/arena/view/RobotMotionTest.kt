package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.*
import org.junit.Assert.*
import org.junit.Test

class RobotMotionTest {
    private fun state(x: Double, y: Double = 8.0, heading: Double = 0.0) =
        ArenaState(robot = RobotPose(x, y, heading))

    @Test fun `first report appears immediately and next report animates only to its target`() {
        val motion = RobotMotion()
        motion.retarget(state(4.0), 0)
        assertEquals(4f, motion.sample(0)!!.x, 0f)
        motion.retarget(state(6.0), 10)
        assertEquals(4f, motion.sample(10)!!.x, 0f)
        assertEquals(5f, motion.sample(85)!!.x, 0.001f)
        assertEquals(6f, motion.sample(160)!!.x, 0.001f)
        assertEquals(6f, motion.sample(1000)!!.x, 0.001f)
        assertFalse(motion.isRunning(1000))
    }

    @Test fun `rapid reports retarget from current display sample`() {
        val motion = RobotMotion()
        motion.snap(state(4.0).robot)
        motion.retarget(state(6.0), 0)
        motion.retarget(state(8.0), 75)
        assertEquals(5f, motion.sample(75)!!.x, 0.001f)
        assertEquals(8f, motion.sample(225)!!.x, 0.001f)
    }

    @Test fun `reported heading rotates through shortest wraparound`() {
        val motion = RobotMotion()
        motion.snap(state(8.0, heading = 359.0).robot)
        motion.retarget(state(8.0, heading = 1.0), 0)
        assertEquals(0f, motion.sample(75)!!.angle, 0.001f)
        assertEquals(1f, motion.sample(150)!!.angle, 0.001f)
    }

    @Test fun `blocked visual chord snaps to actual report`() {
        val motion = RobotMotion()
        motion.snap(state(2.0).robot)
        motion.retarget(state(8.0).copy(obstacles = mapOf(1 to Obstacle(1, GridCoordinate(5, 8)))), 0)
        assertEquals(8f, motion.sample(0)!!.x, 0f)
        assertFalse(motion.isRunning(0))
    }

    @Test fun `manual preview follows configured path and report supersedes it`() {
        val motion = RobotMotion()
        val path = DrivePath(DrivePose(8.0, 8.0, 0.0), ManualCommand.FORWARD_RIGHT)
        motion.snap(state(8.0).robot)
        motion.retarget(ArenaState(robot = path.at(1.0).copy(source = PoseSource.ESTIMATED,
            estimate = path.at(1.0), commandedPath = path)), 0)
        assertEquals(22.5f, motion.sample(500)!!.angle, 0.001f)
        motion.retarget(state(9.0, 9.0, 40.0), 500)
        assertEquals(9f, motion.sample(650)!!.x, 0.001f)
        motion.snap(null)
        assertNull(motion.sample(700))
    }
}
