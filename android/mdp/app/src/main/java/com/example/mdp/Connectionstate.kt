package com.example.mdp



/**
 * The link's current state. Collect [BluetoothConnectionManager.state] in your UI
 * to drive the status TextView (checklist C.4) and enable/disable controls.
 */
sealed interface ConnectionState {
    data object Disconnected : ConnectionState
    data class Connecting(val deviceName: String?) : ConnectionState
    data class Connected(val deviceName: String?) : ConnectionState
    data class Reconnecting(val deviceName: String?, val attempt: Int) : ConnectionState
}