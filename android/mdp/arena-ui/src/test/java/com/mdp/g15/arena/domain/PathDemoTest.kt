package com.mdp.g15.arena.domain

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class PathDemoTest {
    private val config = ArenaConfig()
    private val playback = PathDemoPlayback(PathDemo.INITIAL_POSE, PathDemo.SCRIPT, config)

    private fun assertPose(x: Double, y: Double, heading: Double, pose: RobotPose) {
        assertEquals(x, pose.x, 1e-6)
        assertEquals(y, pose.y, 1e-6)
        assertEquals(heading, pose.heading, 1e-6)
    }

    @Test fun `end poses are reached at 3 8 and 16 seconds`() {
        assertPose(0.0, 0.0, 0.0, playback.frameAt(0L).pose)
        assertPose(0.0, 10.7, 0.0, playback.frameAt(3_000L).pose)
        assertPose(0.0, 9.0, 45.0, playback.frameAt(8_000L).pose)
        val last = playback.frameAt(16_000L)
        assertPose(3.0, 10.5, 90.0, last.pose)
        assertTrue(last.finished)
        assertEquals(16_000L, playback.totalDurationMs)
    }

    @Test fun `straight move interpolates linearly`() {
        assertPose(0.0, 10.7 / 2, 0.0, playback.frameAt(1_500L).pose)
    }

    @Test fun `arcs are continuous into their end pose and turn heading with position`() {
        for (segment in PathDemo.SCRIPT.drop(1)) {
            val nearEnd = segment.poseAt(0.999)
            val end = segment.endPose
            assertTrue(kotlin.math.hypot(nearEnd.x - end.x, nearEnd.y - end.y) < 0.02)
            val halfway = segment.poseAt(0.5)
            assertEquals((segment.startPose.heading + end.heading) / 2, halfway.heading, 1e-6)
        }
    }

    @Test fun `only the active move is highlighted and it advances at each boundary`() {
        assertEquals(0, playback.frameAt(2_999L).activeSegment)
        assertEquals(1, playback.frameAt(3_000L).activeSegment)
        assertEquals(1, playback.frameAt(7_999L).activeSegment)
        assertEquals(2, playback.frameAt(8_000L).activeSegment)
        val done = playback.frameAt(16_000L)
        assertNull(done.activeSegment)
        assertNull(done.overlay)
        assertFalse(playback.frameAt(15_999L).finished)
    }

    @Test fun `straight move sweeps exactly the cells under its footprint`() {
        // 2.0 x 2.1 footprint from y 0 to y 12.8 covers columns 0-1, rows 0-12.
        val expected = (0..1).flatMap { x -> (0..12).map { y -> GridCoordinate(x, y) } }.toSet()
        assertEquals(expected, PathDemo.SCRIPT[0].sweptCells(config))
    }

    @Test fun `arrow follows the direction of travel, not the heading`() {
        fun travel(segment: PathSegment): RobotFootprint.Point {
            val line = segment.centreline(config)
            val a = line[line.size - 2]
            val b = line.last()
            return RobotFootprint.Point(b.x - a.x, b.y - a.y)
        }
        assertTrue(travel(PathDemo.SCRIPT[0]).y > 0)            // forward, north
        travel(PathDemo.SCRIPT[1]).let { assertTrue(it.y < 0 && -it.y > kotlin.math.abs(it.x)) } // reversing, south
        travel(PathDemo.SCRIPT[2]).let { assertTrue(it.x > 0 && it.x > kotlin.math.abs(it.y)) }  // ends heading east
        // The arrow runs along the footprint centre, which is offset from the anchor.
        val start = PathDemo.SCRIPT[0].centreline(config).first()
        assertEquals(config.robotWidthCells / 2, start.x, 1e-9)
        assertEquals(config.robotLengthCells / 2, start.y, 1e-9)
    }
}
