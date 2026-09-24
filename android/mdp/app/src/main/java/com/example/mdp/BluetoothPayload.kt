package com.example.mdp

/**
 * RFCOMM is a stream. RPi receive() requires one newline per complete command, so [appendNewline]
 * is true by default. The AMD tool needs the bare message, so it can be switched off.
 */
internal fun bluetoothPayload(message: String, appendNewline: Boolean = true): String =
    message.trimEnd('\r', '\n') + if (appendNewline) "\n" else ""
