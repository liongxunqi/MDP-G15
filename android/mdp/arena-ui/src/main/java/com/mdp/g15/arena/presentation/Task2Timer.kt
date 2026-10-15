package com.mdp.g15.arena.presentation

import android.os.SystemClock
import kotlin.math.abs

/** Elapsed realtime includes sleep, so returning to the app does not pause a live attempt. */
interface Task2TimerClock {
    fun elapsedMillis(): Long
    fun wallMillis(): Long
}

object AndroidTask2TimerClock : Task2TimerClock {
    override fun elapsedMillis(): Long = SystemClock.elapsedRealtime()
    override fun wallMillis(): Long = System.currentTimeMillis()
}

data class Task2Timer(
    val startedAtElapsedMillis: Long? = null,
    val bootWallOffsetMillis: Long? = null,
    val stoppedAtElapsedMillis: Long? = null,
    val timeUnavailable: Boolean = false,
) {
    val started: Boolean get() = startedAtElapsedMillis != null || timeUnavailable

    fun start(clock: Task2TimerClock): Task2Timer {
        val elapsed = clock.elapsedMillis()
        return Task2Timer(
            startedAtElapsedMillis = elapsed,
            bootWallOffsetMillis = clock.wallMillis() - elapsed,
        )
    }

    fun elapsedMillis(clock: Task2TimerClock): Long? {
        if (timeUnavailable) return null
        stoppedAtElapsedMillis?.let { return it }
        val start = startedAtElapsedMillis ?: return 0L
        return (clock.elapsedMillis() - start).coerceAtLeast(0L)
    }

    fun stop(clock: Task2TimerClock): Task2Timer = if (started && stoppedAtElapsedMillis == null && !timeUnavailable) {
        copy(stoppedAtElapsedMillis = elapsedMillis(clock))
    } else this

    /** elapsedRealtime resets on reboot; do not present a duration from a different boot. */
    fun validateAfterRestore(clock: Task2TimerClock): Task2Timer {
        if (stoppedAtElapsedMillis != null || startedAtElapsedMillis == null) return this
        val offset = bootWallOffsetMillis ?: return copy(timeUnavailable = true)
        val currentOffset = clock.wallMillis() - clock.elapsedMillis()
        return if (clock.elapsedMillis() < startedAtElapsedMillis ||
            abs(currentOffset - offset) > MAX_BOOT_OFFSET_DRIFT_MILLIS) {
            copy(timeUnavailable = true)
        } else this
    }

    companion object {
        private const val MAX_BOOT_OFFSET_DRIFT_MILLIS = 2_000L
    }
}
