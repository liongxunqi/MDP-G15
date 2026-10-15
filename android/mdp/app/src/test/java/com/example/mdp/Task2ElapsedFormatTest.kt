package com.example.mdp

import org.junit.Assert.assertEquals
import org.junit.Test

class Task2ElapsedFormatTest {
    @Test
    fun hundredthsRollOverWithoutLosingMinutes() {
        assertEquals("00:00:00", formatTask2Elapsed(0))
        assertEquals("00:59:99", formatTask2Elapsed(59_999))
        assertEquals("01:00:00", formatTask2Elapsed(60_000))
        assertEquals("02:30:00", formatTask2Elapsed(150_000))
        assertEquals("03:00:00", formatTask2Elapsed(180_000))
        assertEquals("60:00:00", formatTask2Elapsed(3_600_000))
        assertEquals("100:00:00", formatTask2Elapsed(6_000_000))
    }

    @Test
    fun unavailableTimeAndPartialHundredthsAreUnambiguous() {
        assertEquals("--:--:--", formatTask2Elapsed(null))
        assertEquals("00:00:00", formatTask2Elapsed(9))
        assertEquals("00:00:01", formatTask2Elapsed(10))
    }
}
