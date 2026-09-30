package com.mdp.g15.arena.protocol

import com.mdp.g15.arena.domain.Obstacle

/** Replaces the RPi's append-only list. Its existing protocol has no ACK/readback. */
class ObstacleSync(private val send: (String) -> Unit) {
    private var snapshot = "CLEAR"
    private var connected = false

    // True once the full map has gone out at least once. The RPi's own obstacle state is
    // independent of the Bluetooth link — a drop never clears it there — so resending
    // CLEAR + the whole batch on every RECONNECT is redundant at best, and worse: CLEAR
    // wipes the RPi's obstacle list and forces a replan purely because the link blipped.
    // Only the very first connect (including obstacles added before ever connecting) needs
    // to send the full map; a later reconnect sends nothing here — a SYNC from Android
    // covers catching the RPi's TARGET/ROBOT/STATUS answers back up to Android instead.
    private var hasSyncedOnce = false

    val status: String get() = if (connected) {
        "Map delivery unconfirmed — RPi has no acknowledgement"
    } else {
        "Map offline — edits saved locally"
    }

    fun update(obstacles: Map<Int, Obstacle>, transmit: Boolean = true) {
        val next = encode(obstacles)
        if (next == snapshot) return
        snapshot = next
        if (connected && transmit) send(snapshot)
    }

    fun connectionChanged(value: Boolean) {
        if (value == connected) return
        connected = value
        if (connected && !hasSyncedOnce) {
            send(snapshot)
            hasSyncedOnce = true
        }
    }

    companion object {
        fun encode(obstacles: Map<Int, Obstacle>): String = buildList {
            add("CLEAR")
            obstacles.toSortedMap().values.forEach {
                // task1.py divides coordinates by ten and expects full direction names.
                add("OBSTACLE,${it.id},${it.position.x * 10},${it.position.y * 10},${it.targetFace?.name ?: "SKIP"}")
            }
        }.joinToString("\n")
    }
}
