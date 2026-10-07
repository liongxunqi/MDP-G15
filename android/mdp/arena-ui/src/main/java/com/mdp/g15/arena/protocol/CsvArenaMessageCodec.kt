package com.mdp.g15.arena.protocol

import com.mdp.g15.arena.domain.ArenaOutboundEvent
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.TargetId

class CsvArenaMessageCodec : ArenaMessageCodec {
    override fun decode(message: String): ArenaDecodeResult {
        val clean = message.trim().trimEnd('\r', '\n')
        if (clean.isBlank()) return ArenaDecodeResult.Ignored

        if (StatusMessage.isStatus(clean)) return decodeStrictStatus(clean)

        val command = clean.substringBefore(',').trim().uppercase()
        return when (command) {
            "PLANNER" -> {
                val match = PLANNER_READY.matchEntire(clean)
                if (match == null) {
                    ArenaDecodeResult.Malformed("Invalid planning readiness update.")
                } else ArenaDecodeResult.Decoded(ArenaInboundEvent.PlannerReady(
                    match.groupValues[1].takeIf(String::isNotEmpty),
                ))
            }
            "MSG" -> decodeStatus(clean)
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

    private fun decodeStrictStatus(message: String): ArenaDecodeResult =
        if (StatusMessage.isValid(message)) {
            ArenaDecodeResult.Decoded(ArenaInboundEvent.Status(StatusMessage.text(message)))
        } else {
            ArenaDecodeResult.Malformed(StatusMessage.MALFORMED_FEEDBACK)
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
        val obstacleId = parts[1].replaceFirst(Regex("^[bB]"), "").toIntOrNull()?.takeIf { it > 0 }
            ?: return ArenaDecodeResult.Malformed("TARGET obstacle ID must be a positive integer.")
        val targetId = parts[2]
        TargetId.error(targetId)?.let { return ArenaDecodeResult.Malformed(it) }
        val face = parts.getOrNull(3)?.let {
            Direction.fromWire(it)
                ?: return ArenaDecodeResult.Malformed("TARGET face must be N, E, S, or W.")
        }
        return ArenaDecodeResult.Decoded(ArenaInboundEvent.Target(obstacleId, targetId, face))
    }

    private fun decodeRobot(message: String): ArenaDecodeResult {
        val parts = message.split(',').map(String::trim)
        if (parts.size != 4) {
            return ArenaDecodeResult.Malformed("ROBOT must be ROBOT,<x>,<y>,<bearing>.")
        }
        val pose = RobotPoseFields.parse(parts[1], parts[2], parts[3])
            ?: return ArenaDecodeResult.Malformed("ROBOT coordinates and bearing must be finite numbers (or N/E/S/W).")
        return ArenaDecodeResult.Decoded(
            ArenaInboundEvent.Robot(pose),
        )
    }
    companion object {
        // Validate the entire framed payload, never a substring or just its tag.
        // Transport whitespace/CRLF is normalized above; internal whitespace is invalid.
        private val PLANNER_READY = Regex("PLANNER,READY(?:,([0-9a-f]{64}))?")
    }

}
