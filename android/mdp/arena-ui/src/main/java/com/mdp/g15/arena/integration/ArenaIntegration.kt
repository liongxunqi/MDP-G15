package com.mdp.g15.arena.integration

interface ArenaInboundConsumer {
    fun accept(message: String)
}

fun interface ArenaOutboundSink {
    fun submit(message: String)
}
