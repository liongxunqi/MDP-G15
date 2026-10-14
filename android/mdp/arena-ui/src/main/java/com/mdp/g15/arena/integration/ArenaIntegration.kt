package com.mdp.g15.arena.integration

interface ArenaInboundConsumer {
    fun accept(message: String)
}

fun interface ArenaOutboundSink {
    fun submit(message: String)
}

/** The transport proved a command never entered its live-session queue. */
class CommandQueueRejected : RuntimeException()
