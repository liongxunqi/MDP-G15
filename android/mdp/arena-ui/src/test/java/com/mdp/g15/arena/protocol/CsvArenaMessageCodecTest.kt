package com.mdp.g15.arena.protocol

import com.mdp.g15.arena.domain.ArenaOutboundEvent
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.domain.Obstacle
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class CsvArenaMessageCodecTest {
    private val codec = CsvArenaMessageCodec()

    @Test
    fun `status accepts exactly the five forms`() {
        listOf(
            "CONNECTED TO RPI", "RUNNING,3,12", "START,0,19,N", "FAILED", "DONE",
        ).forEach {
            assertEquals(
                ArenaDecodeResult.Decoded(ArenaInboundEvent.Status(it)),
                codec.decode("STATUS,$it"),
            )
        }
    }

    @Test
    fun `status rejects everything else as malformed`() {
        listOf(
            "STATUS", "STATUS,", "STATUS,Exploring", "STATUS,DONE,extra", "STATUS,FAILED,1",
            "STATUS,RUNNING,3", "STATUS,RUNNING,3,12,4", "STATUS,RUNNING,-1,12",
            "STATUS,RUNNING,+3,12", "STATUS,RUNNING,3.0,12", "STATUS,RUNNING,a,12",
            "STATUS,START,1,2", "STATUS,START,1,2,X", "STATUS,START,1,2,n",
            "STATUS,START,1,2,NORTH", "STATUS,START,-1,2,N", "STATUS,START,1,2,N,extra",
            "STATUS,done", "STATUS,CONNECTED  TO RPI", "STATUS, DONE",
        ).forEach {
            assertEquals(it, ArenaDecodeResult.Malformed("Malformed status received"), codec.decode(it))
        }
    }

    @Test
    fun `MSG removes briefing brackets`() {
        assertEquals(
            ArenaDecodeResult.Decoded(ArenaInboundEvent.Status("Moving")),
            codec.decode("MSG,[Moving]"),
        )
    }

    @Test
    fun `target supports checklist format and optional face`() {
        assertEquals(codec.decode("TARGET,2,11,N"), codec.decode("TARGET,B2,11,N"))
        assertEquals(
            ArenaDecodeResult.Decoded(ArenaInboundEvent.Target(2, "11", null)),
            codec.decode("TARGET,2,11"),
        )
        assertEquals(
            ArenaDecodeResult.Decoded(ArenaInboundEvent.Target(2, "11", Direction.NORTH)),
            codec.decode("TARGET,2,11,N"),
        )
    }

    @Test
    fun `robot supports short and long direction values`() {
        val short = codec.decode("ROBOT,7,2,W") as ArenaDecodeResult.Decoded
        val long = codec.decode("ROBOT,7,2,WEST") as ArenaDecodeResult.Decoded
        assertEquals(short, long)
    }

    @Test
    fun `malformed relevant messages report error while unrelated messages are ignored`() {
        assertTrue(codec.decode("TARGET,hello,4") is ArenaDecodeResult.Malformed)
        assertTrue(codec.decode("ROBOT,7,2,UP") is ArenaDecodeResult.Malformed)
        assertEquals(ArenaDecodeResult.Ignored, codec.decode("MOVE,10,FORWARD"))
    }

    @Test
    fun `outbound messages contain complete obstacle state`() {
        val obstacle = Obstacle(
            id = 3,
            position = GridCoordinate(10, 6),
            targetFace = Direction.EAST,
        )
        assertEquals(
            "OBSTACLE,UPSERT,3,10,6,E",
            codec.encode(ArenaOutboundEvent.UpsertObstacle(obstacle)),
        )
        assertEquals(
            "OBSTACLE,REMOVE,3",
            codec.encode(ArenaOutboundEvent.RemoveObstacle(3)),
        )
    }
}
