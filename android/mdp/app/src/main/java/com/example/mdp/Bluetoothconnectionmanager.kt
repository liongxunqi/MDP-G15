package com.example.mdp

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.currentCoroutineContext
import kotlinx.coroutines.ensureActive
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.UUID
import android.util.Log
/**
 * Owns the RFCOMM/SPP Bluetooth link to the Raspberry Pi.
 *
 * This app is the CLIENT; the RPi runs the rfcomm SERVER. Covers the Person A
 * checklist items:
 *   C.1  transmit/receive text strings
 *   C.2  (the connect half — the scan/pick UI lives in your Activity/ViewModel)
 *   C.3  sending manual control commands
 *   C.8  robust auto-reconnect without restarting the app
 *
 * Threading: everything runs on an internal IO scope. send() is non-blocking and
 * safe to call from the UI thread. Collect [state] and [incoming] from your UI.
 */
class BluetoothConnectionManager(
    private val adapter: BluetoothAdapter,
) {
    companion object {
        // Standard Serial Port Profile UUID. The RPi's server must advertise this.
        private val SPP_UUID: UUID =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

        // Bluetooth serial is a raw byte stream with NO message boundaries, so we
        // frame every message with this delimiter and reassemble on receive.
        private const val DELIMITER = '\n'

        private const val RECONNECT_DELAY_MS = 2_000L

        // Keepalive. A remote disconnect often leaves read() blocked forever with no
        // exception, so the link looks "connected" when it's dead. Writing to a dead
        // socket throws quickly, so we send a tiny frame on this interval to force
        // that failure and tear the session down. An empty frame (just the delimiter)
        // is a no-op for the RPi/AMD parser, which ignores empty lines.
        private const val HEARTBEAT_INTERVAL_MS = 1_000L
    }

    val commandLog = CommandLog()

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)

    private val _state = MutableStateFlow<ConnectionState>(ConnectionState.Disconnected)
    val state: StateFlow<ConnectionState> = _state.asStateFlow()

    /** Complete incoming messages, one per emission, already split on the delimiter. */
    private val _incoming = MutableSharedFlow<String>(extraBufferCapacity = 256)
    val incoming: SharedFlow<String> = _incoming.asSharedFlow()

    /** Never replay queued movement/Begin commands on a replacement socket. */
    private val outgoing = SessionOutbox()

    private var connectionJob: Job? = null

    @Volatile
    private var currentSocket: BluetoothSocket? = null

    /** Start connecting to [device] and keep the link alive until [disconnect]. */
    fun connect(device: BluetoothDevice) {
        commandLog.record(CommandLog.Kind.CONNECTION, "Connect requested: ${safeName(device) ?: "device"}")
        val previous = connectionJob
        clearPending()
        previous?.cancel()
        closeSocketQuietly()
        connectionJob = scope.launch {
            previous?.join()
            maintainConnection(device)
        }
    }

    /** Stop maintaining the link and close the socket. */
    fun disconnect() {
        commandLog.record(CommandLog.Kind.CONNECTION, "Disconnected by client")
        connectionJob?.cancel()
        // Retain the cancelled job so a rapid Connect waits for its cleanup.
        closeSocketQuietly()                          // unblocks a blocked read()
        _state.value = ConnectionState.Disconnected
        clearPending()
    }

    /** The arena republishes its map after reconnect; individual commands never replay. */
    fun send(message: String) {
        if (outgoing.submit(message)) {
            commandLog.record(CommandLog.Kind.QUEUED, message)
        } else {
            commandLog.record(CommandLog.Kind.ERROR, "Not connected; not queued: $message")
        }
    }

    /** Convenience for the D-pad buttons (C.3). */
    fun sendCommand(command: RobotCommand) = send(command.wire)

    /** Call once when tearing the app down (e.g. ViewModel.onCleared). */
    fun shutdown() {
        disconnect()
        scope.coroutineContext[Job]?.cancel()
    }

    // --- internals -----------------------------------------------------------

    /**
     * The heart of C.8. Loops forever (until cancelled by disconnect): connect,
     * run a session until the link dies, wait, reconnect. A dropped socket throws
     * IOException out of the session, which lands us back at the top of the loop.
     */
    @SuppressLint("MissingPermission")
    private suspend fun maintainConnection(device: BluetoothDevice) {
        var attempt = 0
        while (currentCoroutineContext().isActive) {
            _state.value =
                if (attempt == 0) ConnectionState.Connecting(safeName(device))
                else ConnectionState.Reconnecting(safeName(device), attempt)

            val socket = try {
                openSocket(device)
            } catch (e: IOException) {
                commandLog.record(CommandLog.Kind.ERROR, "Connection failed: ${e.message}")
                attempt++
                delay(RECONNECT_DELAY_MS)
                continue
            }

            commandLog.record(CommandLog.Kind.CONNECTION, "Connected: ${safeName(device) ?: "device"}")
            attempt = 0
            outgoing.open()
            _state.value = ConnectionState.Connected(safeName(device))

            try {
                runSession(socket)
            } catch (e: CancellationException) {
                throw e                               // user disconnected — exit loop
            } catch (e: IOException) {
                commandLog.record(CommandLog.Kind.ERROR, "Link dropped; reconnecting: ${e.message}")
                // link dropped — fall through and reconnect
            } finally {
                closeSocketQuietly(socket)
                clearPending()
            }

            if (!currentCoroutineContext().isActive) break
            attempt++
            _state.value = ConnectionState.Reconnecting(safeName(device), attempt)
            delay(RECONNECT_DELAY_MS)
        }
    }

    @SuppressLint("MissingPermission")
    private suspend fun openSocket(device: BluetoothDevice): BluetoothSocket {
        // Discovery cripples the connect handshake — always cancel it first.
        if (adapter.isDiscovering) adapter.cancelDiscovery()
        val socket = device.createRfcommSocketToServiceRecord(SPP_UUID)
        // Allow Disconnect to cancel a blocking connect as well as a read.
        synchronized(this) { currentSocket = socket }
        try {
            currentCoroutineContext().ensureActive()
            socket.connect()
            currentCoroutineContext().ensureActive()
        } catch (error: Exception) {
            closeSocketQuietly(socket)
            throw error
        }
        return socket
    }

    /**
     * One live connection. Runs the reader and writer together; when the reader
     * ends (socket closed/dropped) we cancel the writer and return, and the
     * exception (if any) propagates up to trigger a reconnect.
     */
    private suspend fun runSession(socket: BluetoothSocket) {
        val input: InputStream = socket.inputStream
        val output: OutputStream = socket.outputStream

        runBluetoothSession(
            close = { runCatching { socket.close() } },
            read = { readLoop(input) },
            write = { writeLoop(output) },
            heartbeat = { heartbeatLoop() },
        )
    }

    /**
     * Periodically queues an empty frame. If the link is silently dead, the write
     * inside writeLoop throws, which tears down the session and triggers a real
     * reconnect — instead of sitting on a false "Connected". The empty line is
     * ignored by the receiver (readLoop skips empty lines too, so a heartbeat that
     * loops back would be dropped rather than mistaken for a real message).
     */
    private suspend fun heartbeatLoop() {
        while (currentCoroutineContext().isActive) {
            delay(HEARTBEAT_INTERVAL_MS)
            outgoing.submit("")   // writeLoop appends the delimiter -> just "\n"
        }
    }

    /** Read bytes, accumulate, and emit each complete delimiter-terminated message. */
    private suspend fun readLoop(input: InputStream) {
        val buffer = ByteArray(1024)
        val assembled = StringBuilder()

        while (currentCoroutineContext().isActive) {
            val count = input.read(buffer)            // blocks; -1 or throws when closed
            if (count < 0) throw IOException("stream closed")

            assembled.append(String(buffer, 0, count, Charsets.UTF_8))
            Log.d("BT", "buffered so far: [${assembled}]")  

            var idx = assembled.indexOf(DELIMITER)
            while (idx >= 0) {
                val line = assembled.substring(0, idx).trim()
                assembled.delete(0, idx + 1)
                if (line.isNotEmpty()) {
                    commandLog.record(CommandLog.Kind.RX, line)
                    _incoming.emit(line)
                }
                idx = assembled.indexOf(DELIMITER)
            }
        }
    }

    /** Drain the outgoing queue to the socket, framing each message. */
    private suspend fun writeLoop(output: OutputStream) {
        while (currentCoroutineContext().isActive) {
            val message = outgoing.receive()
            val framed = bluetoothPayload(message)
            try {
                output.write(framed.toByteArray(Charsets.UTF_8))
                output.flush()
                if (message.isNotEmpty()) commandLog.record(CommandLog.Kind.TX, message)
            } catch (e: IOException) {
                commandLog.record(CommandLog.Kind.ERROR, "Write failed (${e.message}): $message")
                // Deliberately NOT requeued: a control command that failed to send
                // should not replay late after a reconnect. Let the session tear down.
                throw e
            }
        }
    }

    private fun clearPending() {
        val dropped = outgoing.close()
        if (dropped > 0) commandLog.record(CommandLog.Kind.ERROR, "Discarded $dropped pending messages after connection change; commands will not replay.")
    }

    private fun closeSocketQuietly(socket: BluetoothSocket? = currentSocket) {
        synchronized(this) {
            if (currentSocket === socket) currentSocket = null
        }
        try {
            socket?.close()
        } catch (_: IOException) {
        }
    }

    @SuppressLint("MissingPermission")
    private fun safeName(device: BluetoothDevice): String? =
        try { device.name } catch (_: SecurityException) { null }
}
