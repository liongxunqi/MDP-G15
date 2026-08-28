package com.mdp.g15.arena.protocol

import com.mdp.g15.arena.domain.ArenaOutboundEvent
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.RobotPose

interface ArenaMessageCodec {
    fun decode(message: String): ArenaDecodeResult
    fun encode(event: ArenaOutboundEvent): String
}

sealed interface ArenaInboundEvent {
    data class Status(val text: String) : ArenaInboundEvent
    data class Target(
        val obstacleId: Int,
        val targetId: String,
        val face: Direction?,
    ) : ArenaInboundEvent

    data class Robot(val pose: RobotPose) : ArenaInboundEvent
}

sealed interface ArenaDecodeResult {
    data class Decoded(val event: ArenaInboundEvent) : ArenaDecodeResult
    data object Ignored : ArenaDecodeResult
    data class Malformed(val reason: String) : ArenaDecodeResult
}
