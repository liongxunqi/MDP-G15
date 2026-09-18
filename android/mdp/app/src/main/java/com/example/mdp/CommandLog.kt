package com.example.mdp

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow

/** In-memory session history; capture is independent of the currently visible tab. */
class CommandLog(private val capacity: Int = 500, private val clock: () -> Long = System::currentTimeMillis) {
    enum class Kind { QUEUED, TX, RX, CONNECTION, ERROR }
    data class Entry(val id: Long, val timestamp: Long, val kind: Kind, val message: String)
    private var sequence = 0L
    private val _entries = MutableStateFlow<List<Entry>>(emptyList())
    val entries = _entries.asStateFlow()

    init { require(capacity > 0) }

    @Synchronized
    fun record(kind: Kind, message: String) {
        val bounded = if (message.length > 4096) message.take(4096) + "… [truncated]" else message
        _entries.value = (_entries.value + Entry(++sequence, clock(), kind, bounded)).takeLast(capacity)
    }

    @Synchronized
    fun clear() { _entries.value = emptyList() }
}
