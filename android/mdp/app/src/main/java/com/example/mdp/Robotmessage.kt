package com.example.mdp

import com.mdp.g15.arena.protocol.ArenaDecodeResult
import com.mdp.g15.arena.protocol.ArenaInboundEvent
import com.mdp.g15.arena.protocol.CsvArenaMessageCodec
import com.mdp.g15.arena.protocol.StatusMessage
import com.mdp.g15.arena.protocol.RobotPoseFields

/**
 * Parsed form of an incoming Bluetooth line. Both this controller (for the C.4
 * status box) and, later, the arena view (for robot position + target IDs) can
 * share this one parser — so there's a single place that knows the wire format.
 *
 * Incoming formats (agree these with the RPi / algorithm team on day one):
 *   ROBOT,<decimal-x>,<decimal-y>,<bearing> -> Position; degrees clockwise from north,
 *                                               with legacy N/E/S/W accepted.
 *   TARGET,<obstacleId>,<id>[,<face>] -> Target, or TargetRejected if malformed or the
 *                                  obstacle is unknown (same rules as CsvArenaMessageCodec
 *                                  / ArenaReducer.applyTarget)
 *   STATUS,<one of 5 forms>      -> Status     (strict, see StatusMessage); else MalformedStatus
 *   anything else                -> Unknown
 */
sealed interface RobotMessage {
    data class Position(val x: Double, val y: Double, val headingDeg: Double) : RobotMessage
    data class Target(val obstacle: Int, val targetId: String) : RobotMessage
    /** TARGET was malformed, or named an obstacle that doesn't exist on the arena. */
    data class TargetRejected(val reason: String) : RobotMessage
    data class Status(val text: String) : RobotMessage
    /** Started with STATUS but matched none of the five allowed forms. */
    data object MalformedStatus : RobotMessage
    data class Unknown(val raw: String) : RobotMessage
}

object RobotMessageParser {
    private val arenaCodec = CsvArenaMessageCodec()

    /**
     * [obstacleExists] lets the caller reject a TARGET for an obstacle it doesn't know
     * about, mirroring ArenaReducer.applyTarget. Defaults to "assume it exists" for
     * callers with no arena data to check against.
     */
    fun parse(line: String, obstacleExists: (Int) -> Boolean = { true }): RobotMessage {
        if (StatusMessage.isStatus(line)) {
            return if (StatusMessage.isValid(line)) {
                RobotMessage.Status(StatusMessage.text(line))
            } else {
                RobotMessage.MalformedStatus
            }
        }
        val parts = line.split(",").map { it.trim() }
        return when (parts.firstOrNull()?.uppercase()) {
            "ROBOT" -> {
                if (parts.size != 4) RobotMessage.Unknown(line)
                else RobotPoseFields.parse(parts[1], parts[2], parts[3])?.let {
                    RobotMessage.Position(it.x, it.y, it.heading)
                } ?: RobotMessage.Unknown(line)
            }

            "TARGET" -> parseTarget(line, obstacleExists)

            "MSG" -> parts.drop(1).joinToString(", ").removeSurrounding("[", "]")
                .takeIf { it.isNotBlank() }?.let(RobotMessage::Status) ?: RobotMessage.Unknown(line)

            else -> RobotMessage.Unknown(line)
        }
    }

    /** Same format checks as CsvArenaMessageCodec.decodeTarget, plus an existence check. */
    private fun parseTarget(line: String, obstacleExists: (Int) -> Boolean): RobotMessage =
        when (val result = arenaCodec.decode(line)) {
            is ArenaDecodeResult.Malformed -> RobotMessage.TargetRejected(result.reason)
            is ArenaDecodeResult.Decoded -> {
                val event = result.event as? ArenaInboundEvent.Target ?: return RobotMessage.Unknown(line)
                if (obstacleExists(event.obstacleId)) {
                    RobotMessage.Target(event.obstacleId, event.targetId)
                } else {
                    RobotMessage.TargetRejected("Target references unknown obstacle ${event.obstacleId}.")
                }
            }
            ArenaDecodeResult.Ignored -> RobotMessage.Unknown(line)
        }
}

/** Rejected updates are diagnostics, never a replacement for the last valid robot status. */
internal fun RobotMessage.displayStatusOr(previous: String): String = when (this) {
    is RobotMessage.Status -> text
    is RobotMessage.Target -> "Target $targetId found at obstacle $obstacle"
    else -> previous
}
