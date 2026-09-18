package com.example.mdp

import kotlinx.coroutines.channels.Channel

/** A command belongs to one live socket only. Offline map edits live in ArenaViewModel. */
internal class SessionOutbox {
    private val channel = Channel<String>(Channel.UNLIMITED)
    private var active = false

    @Synchronized fun open() {
        clear()
        active = true
    }

    @Synchronized fun close(): Int {
        active = false
        return clear()
    }

    @Synchronized fun submit(message: String): Boolean = active && channel.trySend(message).isSuccess

    suspend fun receive(): String = channel.receive()

    private fun clear(): Int {
        var count = 0
        while (channel.tryReceive().isSuccess) count++
        return count
    }
}
