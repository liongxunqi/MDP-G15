package com.example.mdp

import org.junit.Assert.*
import org.junit.Test

class ChecklistProtocolTest {
    @Test fun `target validation rejects invalid labels and accepts the incoming uppercase rule`() {
        for (target in listOf("-1", "100", "abc", "a", "+1", "A B", "é", "💡")) {
            assertTrue(target, RobotMessageParser.parse("TARGET,1,$target") { true } is RobotMessage.TargetRejected)
        }
        for (target in listOf("0", "9", "11", "A", "NW", "A9")) {
            assertEquals(RobotMessage.Target(1, target), RobotMessageParser.parse("TARGET,B1,$target") { true })
        }
    }
    @Test fun `every command is delimited for the RPi stream reader`() {
        assertEquals("\n", bluetoothPayload(""))
        assertEquals(listOf("f\n", "r\n", "tl\n", "tr\n", "s\n", "fl\n", "fr\n", "bl\n", "br\n", "BEGIN\n", "PATH\n"), RobotCommand.entries.map { bluetoothPayload(it.wire) })
        assertEquals("CLEAR\nOBSTACLE,1,30,40,NORTH\n", bluetoothPayload("CLEAR\nOBSTACLE,1,30,40,NORTH"))
        assertEquals("BEGIN\n", bluetoothPayload("BEGIN\n"))
    }

    @Test fun `status target and robot reports remain recognized`() {
        assertEquals(RobotMessage.Status("OK"), RobotMessageParser.parse("STATUS,OK"))
        assertEquals(RobotMessage.Status("RUNNING,3,12"), RobotMessageParser.parse("STATUS,RUNNING,3,12"))
        assertEquals(RobotMessage.Status("START,1,2,N"), RobotMessageParser.parse("STATUS,START,1,2,N"))
        assertEquals(RobotMessage.MalformedStatus, RobotMessageParser.parse("STATUS,Exploring"))
        assertEquals(RobotMessage.MalformedStatus, RobotMessageParser.parse("STATUS,DONE,extra"))
        assertEquals(RobotMessage.MalformedStatus, RobotMessageParser.parse("STATUS"))
        assertEquals(RobotMessage.Status("Moving"), RobotMessageParser.parse("MSG,[Moving]"))
        assertEquals(RobotMessage.Target(1, "11"), RobotMessageParser.parse("TARGET,1,11"))
        assertEquals(RobotMessage.Target(2, "11"), RobotMessageParser.parse("TARGET,B2,11,N"))
        assertEquals(RobotMessage.Position(7, 2, "W"), RobotMessageParser.parse("ROBOT,7,2,W"))
        assertTrue(RobotMessageParser.parse("ROBOT,bad,2,W") is RobotMessage.Unknown)
    }

    @Test fun `target applies the same format rules as CsvArenaMessageCodec`() {
        // Malformed — same checks and wording as CsvArenaMessageCodec.decodeTarget.
        assertEquals(
            RobotMessage.TargetRejected("TARGET must be TARGET,<obstacleId>,<targetId>[,<face>]."),
            RobotMessageParser.parse("TARGET,1"),
        )
        assertEquals(
            RobotMessage.TargetRejected("TARGET obstacle ID must be a positive integer."),
            RobotMessageParser.parse("TARGET,0,11"),
        )
        assertEquals(
            RobotMessage.TargetRejected("TARGET ID cannot be blank."),
            RobotMessageParser.parse("TARGET,1,"),
        )
        assertEquals(
            RobotMessage.TargetRejected("TARGET face must be N, E, S, or W."),
            RobotMessageParser.parse("TARGET,1,11,NW"),
        )

        // Well-formed but references an obstacle the caller says doesn't exist —
        // same wording as ArenaReducer.applyTarget.
        assertEquals(
            RobotMessage.TargetRejected("Target references unknown obstacle 1231."),
            RobotMessageParser.parse("TARGET,1231,NW") { false },
        )

        // Well-formed and the obstacle exists.
        assertEquals(
            RobotMessage.Target(1231, "NW"),
            RobotMessageParser.parse("TARGET,1231,NW") { id -> id == 1231 },
        )
    }
}
