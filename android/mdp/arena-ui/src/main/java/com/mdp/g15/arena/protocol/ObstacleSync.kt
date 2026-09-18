package com.mdp.g15.arena.protocol

import com.mdp.g15.arena.domain.Obstacle

/** Replaces the RPi's append-only list. Its existing protocol has no ACK/readback. */
class ObstacleSync(private val send: (String) -> Unit) {
    private var snapshot = "CLEAR"
    private var connected = false
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
        if (connected) send(snapshot)
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
