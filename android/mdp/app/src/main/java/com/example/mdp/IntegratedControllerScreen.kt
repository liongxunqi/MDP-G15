package com.example.mdp

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.material3.Button
import androidx.compose.material3.OutlinedButton
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.PrimaryTabRow
import androidx.compose.material3.Tab
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.lifecycle.viewmodel.compose.viewModel
import com.mdp.g15.arena.domain.ManualCommand
import com.mdp.g15.arena.integration.ArenaOutboundSink
import com.mdp.g15.arena.presentation.ArenaScreen
import com.mdp.g15.arena.presentation.ArenaViewModel
import kotlinx.coroutines.flow.Flow

private enum class ControllerTab(val title: String) {
    CONTROLS("Controls"),
    ARENA("Arena"),
    LOGS("Logs"),
}

@Composable
fun IntegratedControllerScreen(
    status: String,
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
    onForwardLeft: () -> Unit,
    onForwardRight: () -> Unit,
    onBackLeft: () -> Unit,
    onBackRight: () -> Unit,
    onBegin: () -> Unit,
    onPath: () -> Unit,
    onSendCustomMessage: (String) -> Unit,
    onObstacleLookupAvailable: ((Int) -> Boolean) -> Unit,
    incomingMessages: Flow<String>,
    onArenaOutbound: (String) -> Unit,
    modifier: Modifier = Modifier,
    commandLogs: List<CommandLog.Entry> = emptyList(),
    onClearLogs: () -> Unit = {},
    appendNewline: Boolean = true,
    onAppendNewlineChange: (Boolean) -> Unit = {},
) {
    var selectedTab by rememberSaveable { mutableIntStateOf(ControllerTab.CONTROLS.ordinal) }
    val currentOutbound = rememberUpdatedState(onArenaOutbound)
    val outboundSink = remember {
        ArenaOutboundSink { message -> currentOutbound.value(message) }
    }
    val arenaFactory = remember(outboundSink) { ArenaViewModel.Factory(outboundSink, useRpiMapSync = true) }
    val arenaViewModel: ArenaViewModel = viewModel(factory = arenaFactory)

    LaunchedEffect(incomingMessages, arenaViewModel) {
        incomingMessages.collect(arenaViewModel::accept)
    }
    LaunchedEffect(appendNewline, arenaViewModel) {
        arenaViewModel.setAmdToolMode(!appendNewline)
    }
    LaunchedEffect(isConnected, arenaViewModel) {
        arenaViewModel.connectionChanged(isConnected)
    }

    val arenaState by arenaViewModel.uiState.collectAsState()
    DisposableEffect(arenaViewModel, onObstacleLookupAvailable) {
        onObstacleLookupAvailable { id -> id in arenaViewModel.uiState.value.arena.obstacles }
        onDispose { onObstacleLookupAvailable { false } }
    }
    val allowedCommands = arenaState.let { ManualCommand.entries.filter(arenaViewModel::manualAllowed).toSet() }
    val forwardAllowed = arenaViewModel.manualAllowed(ManualCommand.FORWARD)
    val reverseAllowed = arenaViewModel.manualAllowed(ManualCommand.REVERSE)
    val movementCallbacks = mapOf(
        ManualCommand.FORWARD to onForward, ManualCommand.REVERSE to onReverse,
        ManualCommand.LEFT to onTurnLeft, ManualCommand.RIGHT to onTurnRight,
        ManualCommand.FORWARD_LEFT to onForwardLeft, ManualCommand.FORWARD_RIGHT to onForwardRight,
        ManualCommand.BACK_LEFT to onBackLeft, ManualCommand.BACK_RIGHT to onBackRight,
    )

    Column(modifier = modifier.fillMaxSize()) {
        PrimaryTabRow(selectedTabIndex = selectedTab) {
            ControllerTab.entries.forEach { tab ->
                Tab(
                    selected = selectedTab == tab.ordinal,
                    onClick = { selectedTab = tab.ordinal },
                    text = {
                        Column(horizontalAlignment = androidx.compose.ui.Alignment.CenterHorizontally) {
                            Text(tab.title)
                            Text(
                                when (tab) {
                                    ControllerTab.CONTROLS -> "Bluetooth & commands"
                                    ControllerTab.ARENA -> "Map & driving"
                                    ControllerTab.LOGS -> "TX / RX history"
                                },
                                style = MaterialTheme.typography.labelSmall,
                            )
                        }
                    },
                )
            }
        }

        Box(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f),
        ) {
            when (ControllerTab.entries[selectedTab]) {
                ControllerTab.CONTROLS -> ControllerScreen(
                    status = status,
                    robotStatus = arenaState.manualStatus ?: arenaState.status,
                    isConnected = isConnected,
                    isBusy = isBusy,
                    permissionsGranted = permissionsGranted,
                    latestMessage = latestMessage,
                    forwardEnabled = forwardAllowed,
                    reverseEnabled = reverseAllowed,
                    onConnectClick = onConnectClick,
                    onDisconnectClick = onDisconnectClick,
                    onForward = { arenaViewModel.drive(ManualCommand.FORWARD, onForward) },
                    onReverse = { arenaViewModel.drive(ManualCommand.REVERSE, onReverse) },
                    onTurnLeft = { arenaViewModel.drive(ManualCommand.LEFT, onTurnLeft) },
                    onTurnRight = { arenaViewModel.drive(ManualCommand.RIGHT, onTurnRight) },
                    onStop = { arenaViewModel.stopManual(onStop) },
                    onForwardLeft = { arenaViewModel.drive(ManualCommand.FORWARD_LEFT, onForwardLeft) },
                    onForwardRight = { arenaViewModel.drive(ManualCommand.FORWARD_RIGHT, onForwardRight) },
                    onBackLeft = { arenaViewModel.drive(ManualCommand.BACK_LEFT, onBackLeft) },
                    onBackRight = { arenaViewModel.drive(ManualCommand.BACK_RIGHT, onBackRight) },
                    onBegin = { arenaViewModel.startRun(onBegin) },
                    onPath = { arenaViewModel.startRun(onPath) },
                    onSendCustomMessage = { message ->
                        val command = ManualCommand.entries.firstOrNull { it.wire == message.trim().lowercase() }
                        when {
                            '\n' in message || '\r' in message -> arenaViewModel.rejectUnsafeCustomMessage()
                            command != null -> arenaViewModel.drive(command, movementCallbacks.getValue(command))
                            message.trim().equals("s", true) -> arenaViewModel.stopManual(onStop)
                            message.trim().equals("BEGIN", true) -> arenaViewModel.startRun(onBegin)
                            message.trim().equals("PATH", true) -> arenaViewModel.startRun(onPath)
                            else -> onSendCustomMessage(message)
                        }
                    },
                    modifier = Modifier.fillMaxSize(),
                    showManualPad = false,
                )

                ControllerTab.LOGS -> CommandLogScreen(commandLogs, onClearLogs)

                ControllerTab.ARENA -> ArenaScreen(
                    viewModel = arenaViewModel,
                    robotStatus = arenaState.manualStatus ?: arenaState.status,
                    modifier = Modifier.fillMaxSize(),
                    belowResetControls = {
                        NewlineModeButtons(
                            appendNewline = !arenaState.amdToolMode,
                            enabled = !arenaState.manualPending && !arenaState.manualAnimating && !arenaState.autonomousRunning,
                            onChange = { value ->
                                if (arenaViewModel.setAmdToolMode(!value)) onAppendNewlineChange(value)
                            },
                        )
                    },
                    drivingControls = {
                        ArenaDrivePad(
                            status = arenaState.manualStatus ?: arenaState.status,
                            connected = isConnected,
                            allowed = allowedCommands,
                            onMove = { command ->
                                arenaViewModel.drive(command, movementCallbacks.getValue(command))
                            },
                            onStop = { arenaViewModel.stopManual(onStop) },
                        )
                    },
                )
            }
        }
    }
}

/**
 * "Amd tool" uses grid-direction steps and no trailing newline; normal keeps robot arcs and framing.
 * The active mode is filled like "Add obstacle"; the inactive one is outlined like "Reset arena".
 * Pressing the already-active button is a no-op.
 */
@Composable
private fun NewlineModeButtons(appendNewline: Boolean, enabled: Boolean, onChange: (Boolean) -> Unit) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        if (!appendNewline) {
            Button(enabled = enabled, onClick = { onChange(false) }, modifier = Modifier.weight(1f)) { Text("Amd tool") }
            OutlinedButton(enabled = enabled, onClick = { onChange(true) }, modifier = Modifier.weight(1f)) { Text("normal") }
        } else {
            OutlinedButton(enabled = enabled, onClick = { onChange(false) }, modifier = Modifier.weight(1f)) { Text("Amd tool") }
            Button(enabled = enabled, onClick = { onChange(true) }, modifier = Modifier.weight(1f)) { Text("normal") }
        }
    }
}
