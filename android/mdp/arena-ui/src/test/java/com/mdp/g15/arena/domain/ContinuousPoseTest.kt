package com.mdp.g15.arena.domain

import com.mdp.g15.arena.protocol.ArenaDecodeResult
import com.mdp.g15.arena.protocol.ArenaInboundEvent
import com.mdp.g15.arena.protocol.CsvArenaMessageCodec
import org.junit.Assert.*
import org.junit.Test

class ContinuousPoseTest {
    private val codec = CsvArenaMessageCodec()

    @Test fun `decimal position and numeric heading survive parsing`() {
        val decoded = codec.decode("ROBOT,3.25,4.5,46") as ArenaDecodeResult.Decoded
        val pose = (decoded.event as ArenaInboundEvent.Robot).pose
        assertEquals(3.25, pose.x, 0.0)
        assertEquals(4.5, pose.y, 0.0)
        assertEquals(46.0, pose.heading, 0.0)
        assertEquals("46° · Northeast", pose.readableHeading())
        assertEquals("3° · North", RobotPose(0.0, 0.0, 3.0).readableHeading())
        assertEquals("0° · North", RobotPose(0.0, 0.0, 359.99).readableHeading())
    }

    @Test fun `cardinal input remains compatible while numeric bearing normalizes`() {
        for ((wire, angle) in listOf("N" to 0.0, "E" to 90.0, "S" to 180.0, "W" to 270.0,
            "NORTH" to 0.0, "-1" to 359.0, "360" to 0.0, "405" to 45.0)) {
            val event = (codec.decode("ROBOT,2,3,$wire") as ArenaDecodeResult.Decoded).event
            assertEquals(wire, angle, (event as ArenaInboundEvent.Robot).pose.heading, 0.0)
        }
        assertEquals("22.5° · Northeast", RobotPose(0.0, 0.0, 22.5).readableHeading())
        assertEquals("337.5° · North", RobotPose(0.0, 0.0, 337.5).readableHeading())
    }

    @Test fun `start status shares decimal pose rules and malformed numbers preserve state`() {
        assertEquals(ArenaDecodeResult.Decoded(ArenaInboundEvent.Status("START,3.25,4.5,46")),
            codec.decode("STATUS,START,3.25,4.5,46"))
        for (wire in listOf("ROBOT,NaN,2,0", "ROBOT,2,Infinity,0", "ROBOT,2,3,NaN",
            "ROBOT,1001,2,0", "ROBOT,2,3", "ROBOT,2,3,45,extra",
            "STATUS,START,2,3,Infinity", "STATUS,START,1001,3,0")) {
            assertTrue(wire, codec.decode(wire) is ArenaDecodeResult.Malformed)
        }
    }

    @Test fun `twenty by twenty one centimetre robot rotates around fixed centre`() {
        val config = ArenaConfig()
        val north = RobotPose(0.0, 0.0, 0.0)
        assertTrue(RobotFootprint.fitsArena(north, config))
        assertEquals(1.0, RobotFootprint.corners(north, config).map { it.x }.average(), 1e-8)
        assertEquals(1.05, RobotFootprint.corners(north, config).map { it.y }.average(), 1e-8)
        assertFalse(RobotFootprint.fitsArena(north.copy(heading = 45.0), config))
        assertTrue(RobotFootprint.fitsArena(RobotPose(4.0, 4.0, 45.0), config))
    }

    @Test fun `rotated footprint accepts near miss but blocks actual overlap`() {
        val state = ArenaState(robot = RobotPose(4.0, 4.0, 45.0))
        val pose = state.robot!!
        assertFalse(RobotFootprint.overlapsCell(pose, state.config, GridCoordinate(3, 3)))
        assertTrue(RobotFootprint.overlapsCell(pose, state.config, GridCoordinate(5, 5)))
        val reducer = ArenaReducer()
        assertTrue(reducer.reduce(state, ArenaAction.AddObstacle(GridCoordinate(3, 3))) is ArenaReduction.Success)
        assertTrue(reducer.reduce(state, ArenaAction.AddObstacle(GridCoordinate(5, 5))) is ArenaReduction.Failure)
    }

    @Test fun `reported overlap is retained while operator placement is rejected`() {
        val reducer = ArenaReducer()
        val state = ArenaState(robot = RobotPose(0.0, 0.0, 0.0),
            obstacles = mapOf(1 to Obstacle(1, GridCoordinate(5, 5))))
        val incoming = RobotPose(5.25, 5.5, 45.0)
        val reported = (reducer.reduce(state, ArenaAction.ApplyRobotPose(incoming)) as ArenaReduction.Success).state
        assertEquals(incoming, reported.reportedRobot)
        assertEquals(1L, reported.reportRevision)
        val repeated = (reducer.reduce(reported, ArenaAction.ApplyRobotPose(incoming)) as ArenaReduction.Success).state
        assertEquals(2L, repeated.reportRevision)
        assertFalse(reducer.canMoveRobot(reported, GridCoordinate(5, 5)))
    }
}
