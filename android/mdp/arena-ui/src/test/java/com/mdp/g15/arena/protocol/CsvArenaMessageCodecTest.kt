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
    fun `status keeps commas and MSG removes briefing brackets`() {
        assertEquals(
            ArenaDecodeResult.Decoded(ArenaInboundEvent.Status("Moving, slowly")),
            codec.decode("STATUS,Moving, slowly"),
        )
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
