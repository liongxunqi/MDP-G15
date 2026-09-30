package com.example.mdp

import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch

/** A writer failure must close the transport to release a blocking socket read. */
internal suspend fun runBluetoothSession(
    close: () -> Unit,
    read: suspend () -> Unit,
    write: suspend () -> Unit,
    heartbeat: suspend () -> Unit,
) = coroutineScope {
    val writer = launch {
        try {
            write()
        } finally {
            close()
        }
    }
    val keepalive = launch { heartbeat() }
    try {
        read()
    } finally {
        close()
        writer.cancel()
        keepalive.cancel()
    }
}

/**
 * Runs exactly once per transition into Connected, right after the state flips and before
 * [runBluetoothSession] starts — see [BluetoothConnectionManager.maintainConnection]. Split out
 * as a plain function (rather than inlined) so "SYNC is sent, and only once" is unit-testable
 * without a real BluetoothAdapter/-Socket, which this module has no way to mock.
 */
internal fun onConnected(send: (String) -> Unit) {
    send(SYNC)
}
