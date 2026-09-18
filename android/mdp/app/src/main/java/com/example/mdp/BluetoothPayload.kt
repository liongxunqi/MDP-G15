package com.example.mdp

/** RFCOMM is a stream. RPi receive() requires one newline per complete command. */
internal fun bluetoothPayload(message: String): String = message.trimEnd('\r', '\n') + "\n"
