package com.example.mdp

import kotlinx.coroutines.runBlocking
import org.junit.Assert.*
import org.junit.Test

class SessionOutboxTest {
    @Test fun `all control commands are rejected offline and discarded at disconnect`() = runBlocking {
        val outbox = SessionOutbox()
        RobotCommand.entries.forEach { assertFalse(outbox.submit(it.wire)) }
        outbox.open()
        RobotCommand.entries.forEach { assertTrue(outbox.submit(it.wire)) }
        assertEquals(RobotCommand.entries.size, outbox.close())
        assertFalse(outbox.submit("BEGIN"))
        outbox.open()
        outbox.submit("s")
        assertEquals("s", outbox.receive())
        assertEquals(0, outbox.close())
    }

    @Test fun `map batch stays ordered before Begin and cannot interleave with another command`() = runBlocking {
        val outbox = SessionOutbox()
        outbox.open()
        val map = "CLEAR\nOBSTACLE,2,30,40,WEST"
        outbox.submit(map)
        outbox.submit("BEGIN")
        assertEquals(map, outbox.receive())
        assertEquals("BEGIN", outbox.receive())
        outbox.submit("f")
        outbox.open()
        outbox.submit("r")
        assertEquals("r", outbox.receive())
    }
}
