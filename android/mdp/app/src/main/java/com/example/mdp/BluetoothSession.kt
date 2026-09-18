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
