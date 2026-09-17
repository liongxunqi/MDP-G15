package com.example.mdp

import android.annotation.SuppressLint
import android.app.Application
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.Context
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

/**
 * Example wiring only — adapt to your app's UI. Shows how Person A's manager
 * plugs into the connect flow (C.2), the D-pad (C.3), and the curated status
 * box (C.4).
 *
 * Call BluetoothPermissions.allGranted() and request permissions BEFORE using any
 * of this, otherwise scanning/connecting will silently fail on Android 12+.
 */
class ControllerViewModel(app: Application) : AndroidViewModel(app) {

    private val adapter =
        (app.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter

    private val bt = BluetoothConnectionManager(adapter)

    // Connection state -> status label + button enable/disable.
    val state = bt.state

    // Raw incoming stream: debug readout now, Person B's arena parser later.
    val incoming = bt.incoming

    // C.4: curated, human-readable robot status — NOT the raw firehose.
    private val _robotStatus = MutableStateFlow("—")
    val robotStatus: StateFlow<String> = _robotStatus.asStateFlow()

    init {
        // Filter the incoming stream down to status-worthy events only. This is
        // the "selective information" the C.4 spec asks for: STATUS messages and
        // target detections surface here; ROBOT position spam and anything
        // unrecognised deliberately do NOT.
        viewModelScope.launch {
            bt.incoming.collect { line ->
                when (val msg = RobotMessageParser.parse(line)) {
                    is RobotMessage.Status ->
                        _robotStatus.value = msg.text
                    is RobotMessage.Target ->
                        _robotStatus.value =
                            "Target ${msg.targetId} found at obstacle ${msg.obstacle}"
                    is RobotMessage.Position -> Unit   // arena data, not status
                    is RobotMessage.Unknown -> Unit    // ignore noise
                }
            }
        }
    }

    /** For the Connect dialog (C.2): already-paired devices. */
    @SuppressLint("MissingPermission")
    fun pairedDevices(): List<BluetoothDevice> =
        adapter?.bondedDevices?.toList().orEmpty()

    fun connect(device: BluetoothDevice) = bt.connect(device)
    fun disconnect() = bt.disconnect()

    // D-pad (C.3)
    fun forward() = bt.sendCommand(RobotCommand.FORWARD)
    fun reverse() = bt.sendCommand(RobotCommand.REVERSE)
    fun turnLeft() = bt.sendCommand(RobotCommand.TURN_LEFT)
    fun turnRight() = bt.sendCommand(RobotCommand.TURN_RIGHT)
    fun stop() = bt.sendCommand(RobotCommand.STOP)
    fun forwardLeft() = bt.sendCommand(RobotCommand.FORWARD_LEFT)
    fun forwardRight() = bt.sendCommand(RobotCommand.FORWARD_RIGHT)
    fun backLeft() = bt.sendCommand(RobotCommand.BACK_LEFT)
    fun backRight() = bt.sendCommand(RobotCommand.BACK_RIGHT)
    fun begin() = bt.sendCommand(RobotCommand.BEGIN)

    /** Hands a complete arena application message to the existing Bluetooth send queue. */
    fun sendArenaMessage(message: String) = bt.send(message)

    override fun onCleared() {
        bt.shutdown()
    }
}
