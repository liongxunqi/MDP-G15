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

    @Test fun `deletion replaces remaining map and reconnect sends latest offline state`() {
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
        sync.connectionChanged(false)
        sync.update(emptyMap())
        assertEquals(2, sent.size)
        sync.connectionChanged(true)
        assertEquals("CLEAR", sent.last())
        assertTrue(sync.status.contains("unconfirmed"))
    }

    @Test fun `received targets and unchanged map never cause an echo or needless rebuild`() {
        val sent = mutableListOf<String>()
        val sync = ObstacleSync(sent::add)
        val a = Obstacle(1, GridCoordinate(3, 4), Direction.NORTH)
        sync.update(mapOf(1 to a))
        sync.connectionChanged(true)
        sync.update(mapOf(1 to a.copy(targetId = "11")))
        sync.update(mapOf(1 to a.copy(targetFace = Direction.EAST)), transmit = false)
        sync.connectionChanged(true)
        assertEquals(1, sent.size)
        sync.connectionChanged(false)
        sync.connectionChanged(true)
        assertEquals("CLEAR\nOBSTACLE,1,30,40,EAST", sent.last())
    }
}
