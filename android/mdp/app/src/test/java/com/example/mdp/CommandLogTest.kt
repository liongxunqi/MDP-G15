package com.example.mdp

import org.junit.Assert.*
import org.junit.Test

class CommandLogTest {
    @Test fun `history retains chronological bounded entries with timestamps`() {
        val log = CommandLog(3) { 123L }
        repeat(5) { log.record(CommandLog.Kind.RX, "ROBOT,$it,2,N") }
        assertEquals(listOf(3L, 4L, 5L), log.entries.value.map { it.id })
        assertTrue(log.entries.value.all { it.timestamp == 123L })
        log.clear()
        assertTrue(log.entries.value.isEmpty())
        log.record(CommandLog.Kind.TX, "f")
        assertEquals(6L, log.entries.value.single().id)
    }

    @Test fun `concurrent capture loses no events within capacity`() {
        val log = CommandLog(500)
        val threads = List(4) { Thread { repeat(100) { log.record(CommandLog.Kind.RX, "test") } } }
        threads.forEach { it.start() }
        threads.forEach { it.join() }
        assertEquals(400, log.entries.value.size)
        assertEquals((1L..400L).toList(), log.entries.value.map { it.id })
    }

    @Test fun `oversized messages are explicitly truncated`() {
        val log = CommandLog()
        log.record(CommandLog.Kind.ERROR, "x".repeat(10000))
        assertTrue(log.entries.value.single().message.endsWith("[truncated]"))
        assertTrue(log.entries.value.single().message.length < 4200)
    }
}
