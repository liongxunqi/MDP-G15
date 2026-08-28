package com.mdp.g15.arena.protocol

import com.mdp.g15.arena.domain.ArenaOutboundEvent
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.domain.RobotPose

class CsvArenaMessageCodec : ArenaMessageCodec {
    override fun decode(message: String): ArenaDecodeResult {
        val clean = message.trim().trimEnd('\r', '\n')
        if (clean.isBlank()) return ArenaDecodeResult.Ignored

        val command = clean.substringBefore(',').trim().uppercase()
        return when (command) {
            "STATUS", "MSG" -> decodeStatus(clean)
            "TARGET" -> decodeTarget(clean)
            "ROBOT" -> decodeRobot(clean)
            else -> ArenaDecodeResult.Ignored
        }
    }

    override fun encode(event: ArenaOutboundEvent): String = when (event) {
        is ArenaOutboundEvent.UpsertObstacle -> with(event.obstacle) {
            listOf(
                "OBSTACLE",
                "UPSERT",
                id.toString(),
                position.x.toString(),
                position.y.toString(),
                targetFace?.wireValue.orEmpty(),
            ).joinToString(",")
        }
        is ArenaOutboundEvent.RemoveObstacle ->
            "OBSTACLE,REMOVE,${event.obstacleId}"
    }

    private fun decodeStatus(message: String): ArenaDecodeResult {
        val separator = message.indexOf(',')
        if (separator < 0 || separator == message.lastIndex) {
            return ArenaDecodeResult.Malformed("Status message has no display text.")
        }
        val text = message.substring(separator + 1).trim().removeSurrounding("[", "]")
        if (text.isBlank()) return ArenaDecodeResult.Malformed("Status message has no display text.")
        return ArenaDecodeResult.Decoded(ArenaInboundEvent.Status(text))
    }

    private fun decodeTarget(message: String): ArenaDecodeResult {
        val parts = message.split(',').map(String::trim)
        if (parts.size !in 3..4) {
            return ArenaDecodeResult.Malformed(
                "TARGET must be TARGET,<obstacleId>,<targetId>[,<face>].",
            )
        }
        val obstacleId = parts[1].toIntOrNull()?.takeIf { it > 0 }
            ?: return ArenaDecodeResult.Malformed("TARGET obstacle ID must be positive.")
        val targetId = parts[2]
        if (targetId.isBlank()) return ArenaDecodeResult.Malformed("TARGET ID cannot be blank.")
        val face = parts.getOrNull(3)?.let {
            Direction.fromWire(it)
                ?: return ArenaDecodeResult.Malformed("TARGET face must be N, E, S, or W.")
        }
        return ArenaDecodeResult.Decoded(ArenaInboundEvent.Target(obstacleId, targetId, face))
    }

    private fun decodeRobot(message: String): ArenaDecodeResult {
        val parts = message.split(',').map(String::trim)
        if (parts.size != 4) {
            return ArenaDecodeResult.Malformed("ROBOT must be ROBOT,<x>,<y>,<direction>.")
        }
        val x = parts[1].toIntOrNull()
            ?: return ArenaDecodeResult.Malformed("ROBOT x-coordinate must be an integer.")
        val y = parts[2].toIntOrNull()
            ?: return ArenaDecodeResult.Malformed("ROBOT y-coordinate must be an integer.")
        val direction = Direction.fromWire(parts[3])
            ?: return ArenaDecodeResult.Malformed("ROBOT direction must be N, E, S, or W.")
        return ArenaDecodeResult.Decoded(
            ArenaInboundEvent.Robot(RobotPose(GridCoordinate(x, y), direction)),
        )
    }
}
