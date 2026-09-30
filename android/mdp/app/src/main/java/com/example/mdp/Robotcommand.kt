package com.example.mdp

/**
 * Sent once per transition into [ConnectionState.Connected] (first connect and every
 * reconnect). The RPi replies by resending its latest TARGET/ROBOT/STATUS snapshot as
 * ordinary lines, so the map self-heals after whatever was lost while the link was down.
 * Team contract with the RPi side — keep this the only place the literal string lives.
 */
const val SYNC = "SYNC"

/**
 * Manual movement commands (checklist C.3).
 *
 * IMPORTANT: [wire] is the exact string put on the Bluetooth link. These values
 * MUST match what the STM/RPi team parses on their end — agree them on day one.
 * The strings below are common MDP conventions; change them to match your team's
 * protocol, don't assume theirs matches mine.
 */
enum class RobotCommand(val wire: String) {
    FORWARD("f"),
    REVERSE("r"),
    TURN_LEFT("tl"),
    TURN_RIGHT("tr"),
    STOP("s"),
    FORWARD_LEFT("fl"),
    FORWARD_RIGHT("fr"),
    BACK_LEFT("bl"),
    BACK_RIGHT("br"),
    BEGIN("BEGIN"),
    PATH("PATH"),
}