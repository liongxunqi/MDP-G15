package com.mdp.g15.arena.protocol

import com.mdp.g15.arena.domain.*
import org.junit.Assert.*
import org.junit.Test

class ObstacleSyncTest {
    @Test fun `map uses existing RPi units direction names and stable IDs`() {
        assertEquals("CLEAR\nOBSTACLE,2,190,0,SKIP\nOBSTACLE,7,30,40,WEST", ObstacleSync.encode(mapOf(
            7 to Obstacle(7, GridCoordinate(3, 4), Direction.WEST),
            2 to Obstacle(2, GridCoordinate(19, 0)),
        )))
        assertEquals("CLEAR", ObstacleSync.encode(emptyMap()))
    }

    @Test fun `only the first connect sends the full map — a later reconnect sends nothing`() {
        val sent = mutableListOf<String>()
        val sync = ObstacleSync(sent::add)
        val a = Obstacle(1, GridCoordinate(3, 4), Direction.NORTH)
        val b = Obstacle(2, GridCoordinate(5, 6), Direction.EAST)
        sync.update(mapOf(1 to a, 2 to b))
        assertTrue(sent.isEmpty())
        sync.connectionChanged(true)
        assertEquals(ObstacleSync.encode(mapOf(1 to a, 2 to b)), sent.last())
        sync.update(mapOf(2 to b))
        assertEquals("CLEAR\nOBSTACLE,2,50,60,EAST", sent.last())
        val beforeReconnect = sent.size
        sync.connectionChanged(false)
        sync.update(emptyMap())          // offline edit — not transmitted
        assertEquals(beforeReconnect, sent.size)
        sync.connectionChanged(true)
        // A reconnect must NOT resend the map — the RPi's own state is untouched by a drop,
        // and resending CLEAR would wipe and replan it for no reason.
        assertEquals(beforeReconnect, sent.size)
        assertTrue(sync.status.contains("unconfirmed"))
    }

    @Test fun `a reconnect never re-sends the map, even if it changed while offline`() {
        val sent = mutableListOf<String>()
        val sync = ObstacleSync(sent::add)
        val a = Obstacle(1, GridCoordinate(3, 4), Direction.NORTH)
        sync.update(mapOf(1 to a))
        sync.connectionChanged(true)   // first connect: sends the initial map once
        sync.update(mapOf(1 to a.copy(targetFace = Direction.EAST)))                 // live edit: sends
        sync.update(mapOf(1 to a.copy(position = GridCoordinate(9, 9))), transmit = false) // offline-style edit: not sent
        assertEquals(2, sent.size)
        sync.connectionChanged(false)
        sync.connectionChanged(true)   // reconnect: no resend, even though the map changed offline
        assertEquals(2, sent.size)
    }
}
