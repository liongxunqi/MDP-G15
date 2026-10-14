package com.example.mdp

/**
 * RFCOMM is a stream. Newline framing is the default; the RPi's legacy receiver
 * also accepts bare commands used by the AMD tool, so it can be switched off.
 * An empty heartbeat remains a blank line in either mode so the socket sees a write.
 */
internal fun bluetoothPayload(message: String, appendNewline: Boolean = true): String {
    val payload = message.trimEnd('\r', '\n')
    return payload + if (appendNewline || payload.isEmpty()) "\n" else ""
}
