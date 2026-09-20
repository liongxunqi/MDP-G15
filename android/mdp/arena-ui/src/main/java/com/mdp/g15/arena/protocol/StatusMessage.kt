package com.mdp.g15.arena.protocol

/** Strict grammar for RPi -> Android STATUS messages. Anything else starting STATUS is malformed. */
object StatusMessage {
    const val MALFORMED_FEEDBACK = "Malformed status received"

    // Regex.matches() requires the WHOLE input to match, so extra or missing fields fail.
    // [0-9] (not \d or toIntOrNull) keeps integers to plain ASCII digits: no "+3", "-3" or " 3".
    private val VALID = Regex(
        "STATUS,(?:CONNECTED TO RPI|RUNNING,[0-9]+,[0-9]+|START,[0-9]+,[0-9]+,[NSEW]|FAILED|DONE)",
    )

    /** True when the first comma-separated field is exactly "STATUS". */
    fun isStatus(message: String): Boolean = message.substringBefore(',') == "STATUS"

    fun isValid(message: String): Boolean = VALID.matches(message)

    /** The part after "STATUS,", e.g. "RUNNING,3,12". Call only after [isValid]. */
    fun text(message: String): String = message.substringAfter(',')
}
