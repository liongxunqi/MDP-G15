package com.example.mdp

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.example.mdp.BluetoothPermissions
import com.example.mdp.ConnectionState
import com.example.mdp.ControllerViewModel
import com.example.mdp.ui.theme.MdpTheme

/**
 * NOTE ON IMPORTS: this assumes BluetoothPermissions / ConnectionState /
 * ControllerViewModel live in the `com.example.mdp.bluetooth` sub-package (as
 * originally generated). If you moved those classes into the root
 * `com.example.mdp` package, just delete the three `com.example.mdp.bluetooth.*`
 * imports above.
 */

/** Lightweight UI model so composables never touch permission-gated BT APIs. */
data class DeviceItem(
    val name: String,
    val address: String,
    val device: BluetoothDevice,
)

class MainActivity : ComponentActivity() {

    private val viewModel: ControllerViewModel by viewModels()

    private val adapter: BluetoothAdapter? by lazy {
        (getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
    }

    // --- Compose-observable UI state -----------------------------------------
    private var isBluetoothEnabled by mutableStateOf(false)   // permissions granted?
    private var showPicker by mutableStateOf(false)
    private val pairedItems = mutableStateListOf<DeviceItem>()
    private val discovered = mutableStateListOf<DeviceItem>()

    // --- Permission request --------------------------------------------------
    private val requestPerms = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        if (permissions.entries.all { it.value }) {
            enableBluetoothFeatures()
        } else {
            // Denied: Connect stays disabled. Show a Toast/snackbar if you like.
        }
    }

    // --- Discovery receiver (the "scan" half of C.2) -------------------------
    private val discoveryReceiver = object : BroadcastReceiver() {
        @SuppressLint("MissingPermission")
        override fun onReceive(context: Context?, intent: Intent?) {
            if (intent?.action == BluetoothDevice.ACTION_FOUND) {
                val device = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                    intent.getParcelableExtra(
                        BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java
                    )
                } else {
                    @Suppress("DEPRECATION")
                    intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                }
                device?.let { d ->
                    val item = d.toItem()
                    if (discovered.none { it.address == item.address }) discovered.add(item)
                }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        ContextCompat.registerReceiver(
            this,
            discoveryReceiver,
            IntentFilter(BluetoothDevice.ACTION_FOUND),
            ContextCompat.RECEIVER_EXPORTED,
        )

        setContent {
            MdpTheme {
                val state by viewModel.state.collectAsStateWithLifecycle()
                val latest by viewModel.incoming.collectAsStateWithLifecycle(initialValue = "")
                val robotStatus by viewModel.robotStatus.collectAsStateWithLifecycle()

                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    ControllerScreen(
                        status = statusLabel(state),
                        robotStatus = robotStatus,
                        isConnected = state is ConnectionState.Connected,
                        isBusy = state is ConnectionState.Connecting ||
                            state is ConnectionState.Reconnecting,
                        permissionsGranted = isBluetoothEnabled,
                        latestMessage = latest,
                        onConnectClick = ::openPicker,
                        onDisconnectClick = viewModel::disconnect,
                        onForward = viewModel::forward,
                        onReverse = viewModel::reverse,
                        onTurnLeft = viewModel::turnLeft,
                        onTurnRight = viewModel::turnRight,
                        onStop = viewModel::stop,
                        modifier = Modifier.padding(innerPadding),
                    )

                    if (showPicker) {
                        DevicePickerDialog(
                            paired = pairedItems,
                            discovered = discovered,
                            onScan = ::startScan,
                            onSelect = ::onDeviceChosen,
                            onDismiss = ::closePicker,
                        )
                    }
                }
            }
        }

        if (!BluetoothPermissions.allGranted(this)) {
            requestPerms.launch(BluetoothPermissions.required)
        } else {
            enableBluetoothFeatures()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        cancelDiscovery()
        runCatching { unregisterReceiver(discoveryReceiver) }
    }

    private fun enableBluetoothFeatures() {
        isBluetoothEnabled = true
    }

    // --- Picker actions ------------------------------------------------------

    private fun openPicker() {
        pairedItems.clear()
        pairedItems.addAll(viewModel.pairedDevices().map { it.toItem() })
        discovered.clear()
        showPicker = true
    }

    private fun closePicker() {
        cancelDiscovery()
        showPicker = false
    }

    private fun onDeviceChosen(item: DeviceItem) {
        cancelDiscovery()                 // must stop discovery before connecting
        viewModel.connect(item.device)
        showPicker = false
    }

    @SuppressLint("MissingPermission")
    private fun startScan() {
        discovered.clear()
        adapter?.let {
            if (it.isDiscovering) it.cancelDiscovery()
            it.startDiscovery()
        }
    }

    @SuppressLint("MissingPermission")
    private fun cancelDiscovery() {
        adapter?.let { if (it.isDiscovering) it.cancelDiscovery() }
    }

    @SuppressLint("MissingPermission")
    private fun BluetoothDevice.toItem() = DeviceItem(
        name = name ?: "(no name)",
        address = address,
        device = this,
    )
}

/** Maps the connection state to a short, human-readable status (seeds C.4). */
private fun statusLabel(state: ConnectionState): String = when (state) {
    is ConnectionState.Disconnected -> "Disconnected"
    is ConnectionState.Connecting -> "Connecting to ${state.deviceName ?: "device"}…"
    is ConnectionState.Connected -> "Connected to ${state.deviceName ?: "device"}"
    is ConnectionState.Reconnecting -> "Reconnecting… (attempt ${state.attempt})"
}

@Composable
fun ControllerScreen(
    status: String,
    robotStatus: String,
    isConnected: Boolean,
    isBusy: Boolean,
    permissionsGranted: Boolean,
    latestMessage: String,
    onConnectClick: () -> Unit,
    onDisconnectClick: () -> Unit,
    onForward: () -> Unit,
    onReverse: () -> Unit,
    onTurnLeft: () -> Unit,
    onTurnRight: () -> Unit,
    onStop: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(24.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        // --- Connection status + Connect/Disconnect ---
        Text(text = "Bluetooth", style = MaterialTheme.typography.titleMedium)
        Spacer(Modifier.height(8.dp))
        Text(text = status, style = MaterialTheme.typography.bodyLarge)
        Spacer(Modifier.height(16.dp))

        when {
            isConnected -> Button(onClick = onDisconnectClick) { Text("Disconnect") }
            isBusy -> OutlinedButton(onClick = onDisconnectClick) { Text("Cancel") }
            else -> Button(onClick = onConnectClick, enabled = permissionsGranted) {
                Text(if (permissionsGranted) "Connect" else "Permissions required")
            }
        }

        Spacer(Modifier.height(24.dp))

        // --- C.4: curated robot status (selective, not the raw stream) ---
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(16.dp)) {
                Text(
                    text = "Robot status",
                    style = MaterialTheme.typography.labelMedium,
                )
                Spacer(Modifier.height(4.dp))
                Text(
                    text = robotStatus,
                    style = MaterialTheme.typography.titleMedium,
                )
            }
        }

        Spacer(Modifier.height(24.dp))

        // --- C.3: manual control D-pad ---
        Text(text = "Manual control", style = MaterialTheme.typography.labelMedium)
        Spacer(Modifier.height(8.dp))
        DPad(
            enabled = isConnected,
            onForward = onForward,
            onReverse = onReverse,
            onTurnLeft = onTurnLeft,
            onTurnRight = onTurnRight,
            onStop = onStop,
        )

        // --- Debug readout — handy while testing C.1. Not the C.4 deliverable. ---
        if (latestMessage.isNotEmpty()) {
            Spacer(Modifier.height(24.dp))
            Text(
                text = "Last message: $latestMessage",
                style = MaterialTheme.typography.bodySmall,
            )
        }
    }
}

/**
 * Cross-shaped movement pad (C.3). Buttons are disabled until connected so a tap
 * can't silently no-op. Each maps to a RobotCommand via the ViewModel; the wire
 * strings live in RobotCommand.kt and must match the STM/RPi parser.
 */
@Composable
fun DPad(
    enabled: Boolean,
    onForward: () -> Unit,
    onReverse: () -> Unit,
    onTurnLeft: () -> Unit,
    onTurnRight: () -> Unit,
    onStop: () -> Unit,
) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Button(onClick = onForward, enabled = enabled) { Text("▲") }
        Spacer(Modifier.height(8.dp))
        Row {
            Button(onClick = onTurnLeft, enabled = enabled) { Text("◀") }
            Spacer(Modifier.width(8.dp))
            OutlinedButton(onClick = onStop, enabled = enabled) { Text("■") }
            Spacer(Modifier.width(8.dp))
            Button(onClick = onTurnRight, enabled = enabled) { Text("▶") }
        }
        Spacer(Modifier.height(8.dp))
        Button(onClick = onReverse, enabled = enabled) { Text("▼") }
    }
}

@Composable
fun DevicePickerDialog(
    paired: List<DeviceItem>,
    discovered: List<DeviceItem>,
    onScan: () -> Unit,
    onSelect: (DeviceItem) -> Unit,
    onDismiss: () -> Unit,
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Select a device") },
        text = {
            Column(modifier = Modifier.verticalScroll(rememberScrollState())) {
                Text("Paired", style = MaterialTheme.typography.labelMedium)
                if (paired.isEmpty()) {
                    Text("No paired devices", style = MaterialTheme.typography.bodySmall)
                }
                paired.forEach { DeviceRow(it, onSelect) }

                if (discovered.isNotEmpty()) {
                    Spacer(Modifier.height(16.dp))
                    Text("Found", style = MaterialTheme.typography.labelMedium)
                    discovered.forEach { DeviceRow(it, onSelect) }
                }
            }
        },
        confirmButton = { TextButton(onClick = onScan) { Text("Scan") } },
        dismissButton = { TextButton(onClick = onDismiss) { Text("Close") } },
    )
}

@Composable
private fun DeviceRow(item: DeviceItem, onSelect: (DeviceItem) -> Unit) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .clickable { onSelect(item) }
            .padding(vertical = 12.dp),
    ) {
        Text(text = item.name, style = MaterialTheme.typography.bodyLarge)
        Text(text = item.address, style = MaterialTheme.typography.bodySmall)
    }
}

@Preview(showBackground = true)
@Composable
fun ControllerScreenPreview() {
    MdpTheme {
        ControllerScreen(
            status = "Connected to laptop",
            robotStatus = "Looking for target 2",
            isConnected = true,
            isBusy = false,
            permissionsGranted = true,
            latestMessage = "",
            onConnectClick = {},
            onDisconnectClick = {},
            onForward = {},
            onReverse = {},
            onTurnLeft = {},
            onTurnRight = {},
            onStop = {},
        )
    }
}