package com.example.mdp

import org.junit.Assert.*
import org.junit.Test

class ChecklistProtocolTest {
    @Test fun `every command is delimited for the RPi stream reader`() {
        assertEquals("\n", bluetoothPayload(""))
        assertEquals(listOf("f\n", "r\n", "tl\n", "tr\n", "s\n", "fl\n", "fr\n", "bl\n", "br\n", "BEGIN\n", "PATH\n"), RobotCommand.entries.map { bluetoothPayload(it.wire) })
        assertEquals("CLEAR\nOBSTACLE,1,30,40,NORTH\n", bluetoothPayload("CLEAR\nOBSTACLE,1,30,40,NORTH"))
        assertEquals("BEGIN\n", bluetoothPayload("BEGIN\n"))
    }

    @Test fun `status target and robot reports remain recognized`() {
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
}
