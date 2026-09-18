package com.example.mdp

/**
 * Parsed form of an incoming Bluetooth line. Both this controller (for the C.4
 * status box) and, later, the arena view (for robot position + target IDs) can
 * share this one parser — so there's a single place that knows the wire format.
 *
 * Incoming formats (agree these with the RPi / algorithm team on day one):
 *   ROBOT, <x>, <y>, <dir>      -> Position   (fixed by spec, C.10)
 *   TARGET, <obstacle>, <id>    -> Target     (fixed by spec, C.9)
 *   STATUS, <free text>         -> Status     (your convention for C.4)
 *   anything else               -> Unknown
 */
sealed interface RobotMessage {
    data class Position(val x: Int, val y: Int, val direction: String) : RobotMessage
    data class Target(val obstacle: Int, val targetId: String) : RobotMessage
    data class Status(val text: String) : RobotMessage
    data class Unknown(val raw: String) : RobotMessage
}

object RobotMessageParser {

    fun parse(line: String): RobotMessage {
        val parts = line.split(",").map { it.trim() }
        return when (parts.firstOrNull()?.uppercase()) {
            "ROBOT" -> {
                val x = parts.getOrNull(1)?.toIntOrNull()
                val y = parts.getOrNull(2)?.toIntOrNull()
                val dir = parts.getOrNull(3)?.uppercase()
                if (x != null && y != null && dir != null) {
                    RobotMessage.Position(x, y, dir)
                } else {
                    RobotMessage.Unknown(line)
                }
            }

            "TARGET" -> {
                val obstacle = parts.getOrNull(1)?.replaceFirst(Regex("^[bB]"), "")?.toIntOrNull()
                val id = parts.getOrNull(2)
                if (obstacle != null && !id.isNullOrEmpty()) {
                    RobotMessage.Target(obstacle, id)
                } else {
                    RobotMessage.Unknown(line)
                }
            }

            "MSG" -> parts.drop(1).joinToString(", ").removeSurrounding("[", "]")
                .takeIf { it.isNotBlank() }?.let(RobotMessage::Status) ?: RobotMessage.Unknown(line)

            "STATUS" -> RobotMessage.Status(
                parts.drop(1).joinToString(", ").ifEmpty { line }
            )

            else -> RobotMessage.Unknown(line)
        }
    }
}
