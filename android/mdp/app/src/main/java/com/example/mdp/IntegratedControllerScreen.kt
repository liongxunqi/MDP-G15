package com.example.mdp

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.TextButton
import androidx.compose.material3.Button
import androidx.compose.material3.FilterChip
import androidx.compose.material3.OutlinedButton
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.PrimaryTabRow
import androidx.compose.material3.Tab
import androidx.compose.material3.Text
import androidx.compose.material3.Surface
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.platform.LocalView
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.produceState
import androidx.compose.runtime.setValue
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.Alignment
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.platform.testTag
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.ui.unit.dp
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.BorderStroke
import androidx.compose.material3.TabRowDefaults
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.ui.unit.sp
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.lifecycle.viewmodel.compose.viewModel
import com.mdp.g15.arena.domain.ManualCommand
import com.mdp.g15.arena.integration.ArenaOutboundSink
import com.mdp.g15.arena.presentation.ArenaScreen
import com.mdp.g15.arena.presentation.PixelMaskIcon
import com.mdp.g15.arena.presentation.Task1Phase
import com.mdp.g15.arena.presentation.TaskMode
import com.mdp.g15.arena.presentation.ArenaViewModel
import com.mdp.g15.arena.presentation.AndroidTask2TimerClock
import com.mdp.g15.arena.presentation.Task2Timer
import com.mdp.g15.arena.presentation.DEFAULT_PLANNER_INFO
import com.mdp.g15.arena.presentation.PixelButton
import com.mdp.g15.arena.presentation.PixelTone
import com.mdp.g15.arena.presentation.SpiderMap
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.delay
import java.util.Locale

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
    // Keep the foreground controller awake through setup, disconnects and result viewing.
    // Scope this to the host view rather than a tab that gets disposed on navigation.
    val hostView = LocalView.current
    DisposableEffect(hostView) {
        val previouslyAwake = hostView.keepScreenOn
        hostView.keepScreenOn = true
        onDispose { hostView.keepScreenOn = previouslyAwake }
    }
    val beginSelectedTask: () -> Unit = {
        if (arenaViewModel.startRun(onBegin)) selectedTab = ControllerTab.ARENA.ordinal
    }
    LaunchedEffect(arenaState.task1Phase) {
        if (arenaState.task1Phase in setOf(Task1Phase.START_REQUESTED, Task1Phase.RUNNING,
                Task1Phase.UNKNOWN, Task1Phase.COMPLETED, Task1Phase.FAILED)) {
            selectedTab = ControllerTab.ARENA.ordinal
        }
    }
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
        PrimaryTabRow(selectedTabIndex = selectedTab,
            containerColor = SpiderMap.NavyDeep,
            contentColor = SpiderMap.OnBackdrop,
            indicator = { TabRowDefaults.PrimaryIndicator(
                Modifier.tabIndicatorOffset(selectedTab), color = SpiderMap.Frame,
                width = 72.dp) },
            modifier = Modifier.height(48.dp)) {
            ControllerTab.entries.forEach { tab ->
                Tab(
                    selected = selectedTab == tab.ordinal,
                    onClick = { selectedTab = tab.ordinal },
                    text = { Text(tab.title, style = SpiderMap.pixel(10.sp,
                        if (selectedTab == tab.ordinal) SpiderMap.OnBackdrop else SpiderMap.PanelBlue)) },
                    modifier = Modifier.height(48.dp),
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
                    onBegin = beginSelectedTask,
                    onPath = { arenaViewModel.preparePath(onPath) },
                    onSendCustomMessage = { message ->
                        val command = ManualCommand.entries.firstOrNull { it.wire == message.trim().lowercase() }
                        when {
                            '\n' in message || '\r' in message -> arenaViewModel.rejectUnsafeCustomMessage()
                            command != null -> arenaViewModel.drive(command, movementCallbacks.getValue(command))
                            message.trim().equals("s", true) -> arenaViewModel.stopManual(onStop)
                            message.trim().equals("BEGIN", true) -> beginSelectedTask()
                            message.trim().equals("PATH", true) -> arenaViewModel.preparePath(onPath)
                            else -> onSendCustomMessage(message)
                        }
                    },
                    modifier = Modifier.fillMaxSize(),
                    showManualPad = false,
                    showTaskButtons = false,
                )

                ControllerTab.LOGS -> CommandLogScreen(commandLogs, onClearLogs)

                ControllerTab.ARENA -> ArenaScreen(
                    viewModel = arenaViewModel,
                    robotStatus = if (arenaState.taskMode == TaskMode.TASK2) {
                        when {
                            !isConnected && arenaState.task1Phase == Task1Phase.UNKNOWN ->
                                "Unknown — Bluetooth disconnected"
                            !isConnected && arenaState.task1Phase !in setOf(
                                Task1Phase.COMPLETED, Task1Phase.FAILED, Task1Phase.STOPPED) ->
                                "Bluetooth disconnected"
                            arenaState.task1Phase == Task1Phase.STOPPED ->
                                arenaState.manualStatus ?: arenaState.status
                            else -> arenaState.status
                        }
                    } else arenaState.manualStatus ?: arenaState.status,
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
                    taskControls = {
                        TaskControls(
                            mode = arenaState.taskMode,
                            phase = arenaState.task1Phase,
                            connected = isConnected,
                            canRequest = if (arenaState.taskMode == TaskMode.TASK1)
                                arenaViewModel.canRequestTask1() else arenaViewModel.canRequestTask2(),
                            startIssue = if (arenaState.taskMode == TaskMode.TASK1)
                                arenaViewModel.task1StartIssue() else arenaViewModel.task2StartIssue(),
                            plannerInfo = arenaState.plannerInfo,
                            hasResults = arenaState.taskMode == TaskMode.TASK1 &&
                                arenaState.arena.obstacles.values.any { it.targetId != null },
                            canNewAttempt = (!arenaState.autonomousRunning ||
                                (arenaState.taskMode == TaskMode.TASK2 && arenaState.task1Phase == Task1Phase.UNKNOWN)) &&
                                !arenaState.manualPending && !arenaState.manualAnimating,
                            canSelectTask = !arenaState.autonomousRunning && !arenaState.manualPending && !arenaState.manualAnimating,
                            onSelectTask = arenaViewModel::selectTask,
                            onStart = beginSelectedTask,
                            onStop = { arenaViewModel.stopManual(onStop) },
                            onNewAttempt = arenaViewModel::newAttempt,
                        )
                    },
                    taskStatus = {
                        TaskStatusDetails(
                            mode = arenaState.taskMode, phase = arenaState.task1Phase,
                            plannerInfo = arenaState.plannerInfo,
                            startIssue = if (arenaState.taskMode == TaskMode.TASK1)
                                arenaViewModel.task1StartIssue() else arenaViewModel.task2StartIssue(),
                        )
                    },
                    drivingControls = {
                        if (arenaState.taskMode == TaskMode.TASK1) {
                            ArenaDrivePad(
                                status = arenaState.manualStatus ?: arenaState.status,
                                connected = isConnected,
                                allowed = allowedCommands,
                                onMove = { command ->
                                    arenaViewModel.drive(command, movementCallbacks.getValue(command))
                                },
                                onStop = { arenaViewModel.stopManual(onStop) },
                            )
                        }
                    },
                    task2TimerDisplay = {
                        Task2TimerPanel(arenaState.task2Timer, arenaState.task1Phase)
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
        PixelButton("Amd tool", { onChange(false) }, Modifier.weight(1f), PixelTone.ORANGE,
            enabled = enabled, selected = !appendNewline)
        PixelButton("normal", { onChange(true) }, Modifier.weight(1f), PixelTone.ORANGE,
            enabled = enabled, selected = appendNewline)
    }
}

@Composable
private fun TaskControls(
    mode: TaskMode,
    phase: Task1Phase,
    connected: Boolean,
    canRequest: Boolean,
    hasResults: Boolean,
    canNewAttempt: Boolean,
    startIssue: String?,
    plannerInfo: String,
    canSelectTask: Boolean,
    onSelectTask: (TaskMode) -> Unit,
    onStart: () -> Unit,
    onStop: () -> Unit,
    onNewAttempt: () -> Unit,
) {
    var confirmNewAttempt by rememberSaveable { mutableStateOf(false) }
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically) {
            // The active task is filled orange; the other is a hollow tab.
            TaskMode.entries.forEach { task ->
                PixelButton(
                    text = if (task == TaskMode.TASK1) "Task 1" else "Task 2",
                    onClick = { onSelectTask(task) },
                    tone = PixelTone.ORANGE,
                    enabled = canSelectTask,
                    selected = mode == task,
                    contentPadding = PaddingValues(horizontal = 14.dp, vertical = 6.dp),
                )
            }
        BoxWithConstraints(Modifier.weight(1f).height(40.dp), contentAlignment = Alignment.Center) {
            if (maxWidth >= 48.dp) PixelMaskIcon(Modifier.size(38.dp))
        }
        val task2Active = mode == TaskMode.TASK2 && phase in setOf(
            Task1Phase.START_REQUESTED, Task1Phase.RUNNING, Task1Phase.UNKNOWN)
        if (mode == TaskMode.TASK2) {
            if (task2Active) {
                PixelButton("Stop Task 2", onStop, tone = PixelTone.CORAL,
                    enabled = connected)
            }
            if (phase in setOf(Task1Phase.COMPLETED, Task1Phase.FAILED, Task1Phase.STOPPED,
                    Task1Phase.UNKNOWN)) {
                PixelButton("New attempt", { confirmNewAttempt = true },
                    enabled = canNewAttempt, selected = false)
            } else if (!task2Active) {
                PixelButton("Start Task 2", onStart, tone = PixelTone.GREEN, enabled = canRequest)
            }
        } else if (phase in setOf(Task1Phase.COMPLETED, Task1Phase.FAILED, Task1Phase.STOPPED) ||
            (hasResults && phase in setOf(Task1Phase.SETUP, Task1Phase.PATH_REQUESTED))) {
            PixelButton("New attempt", { confirmNewAttempt = true },
                enabled = canNewAttempt, selected = false)
        } else {
            PixelButton("Start Task 1", onStart, tone = PixelTone.GREEN, enabled = canRequest)
        }
    }
    if (confirmNewAttempt) {
        AlertDialog(
            onDismissRequest = { confirmNewAttempt = false },
            shape = SpiderMap.PanelShape,
            containerColor = SpiderMap.PanelBlue,
            titleContentColor = SpiderMap.Ink,
            textContentColor = SpiderMap.Ink,
            title = { Text("Prepare a new attempt?", style = SpiderMap.Title) },
            text = { Text(if (mode == TaskMode.TASK1)
                "Save the result-map screenshot first. Previous image results will be cleared; obstacle positions and faces will be kept. No command will be sent to the robot."
                else "Before another Task 2 attempt, verify the robot is stationary and the previous run has ended. Saved Task 1 markers and results will be kept. Preparing does not send BEGIN or STOP. For local tests, restart the Task 2 Pi; leave the PC service running and wait for READY before Start. Competition retries are shared across tasks; confirm eligibility with the supervisor.") },
            confirmButton = {
                PixelButton(if (mode == TaskMode.TASK1) "Clear results" else "Prepare attempt", {
                    confirmNewAttempt = false
                    onNewAttempt()
                }, enabled = canNewAttempt, tone = PixelTone.CORAL)
            },
            dismissButton = { PixelButton("Cancel", { confirmNewAttempt = false }, selected = false) },
        )
    }
}

@Composable
private fun TaskStatusDetails(mode: TaskMode, phase: Task1Phase, plannerInfo: String,
                              startIssue: String?) {
    val phaseLabel = if (mode == TaskMode.TASK1) phase.label else when (phase) {
        Task1Phase.COMPLETED -> "Completed — check image evidence"
        Task1Phase.FAILED -> "Run ended with a failure"
        Task1Phase.STOPPED -> "Stop requested"
        else -> phase.label
    }
    Text("${if (mode == TaskMode.TASK1) "Task 1" else "Task 2"} • $phaseLabel",
        style = SpiderMap.pixel(7.sp))
    if (phase in setOf(Task1Phase.SETUP, Task1Phase.PATH_REQUESTED, Task1Phase.START_REQUESTED)) {
        if (mode == TaskMode.TASK1 && plannerInfo != DEFAULT_PLANNER_INFO)
            Text(plannerInfo, style = SpiderMap.pixel(7.sp))
        if (startIssue != null && phase != Task1Phase.START_REQUESTED)
            Text(startIssue, style = SpiderMap.pixel(7.sp), color = SpiderMap.Warning)
    }
}

@Composable
private fun Task2TimerPanel(timer: Task2Timer, phase: Task1Phase) {
    val elapsed by produceState(initialValue = timer.elapsedMillis(AndroidTask2TimerClock), timer) {
        while (timer.startedAtElapsedMillis != null && timer.stoppedAtElapsedMillis == null &&
            !timer.timeUnavailable) {
            delay(20)
            value = timer.elapsedMillis(AndroidTask2TimerClock)
        }
    }
    val elapsedMillis = elapsed
    val display = formatTask2Elapsed(elapsedMillis)
    val label = when {
        elapsedMillis == null -> "Task 2 time unavailable"
        !timer.started -> "Task 2 ready; elapsed time $display"
        phase == Task1Phase.COMPLETED -> "Task 2 completed in $display"
        phase == Task1Phase.FAILED -> "Task 2 failed after $display"
        phase == Task1Phase.STOPPED -> "Task 2 Stop requested at $display; robot stop unconfirmed"
        phase == Task1Phase.UNKNOWN -> "Task 2 elapsed $display; run status unknown"
        else -> "Task 2 elapsed $display"
    }
    val heading = when (phase) {
        Task1Phase.COMPLETED -> "Completed"
        Task1Phase.FAILED -> "Failed"
        Task1Phase.STOPPED -> "Stop requested"
        Task1Phase.UNKNOWN -> "Run status unknown"
        Task1Phase.RUNNING -> "Running"
        Task1Phase.START_REQUESTED -> "Start requested"
        else -> "Ready to start"
    }
    // Same blue as the status card; the time-limit warnings still change it.
    val color = when {
        elapsedMillis != null && elapsedMillis >= 180_000L -> Color(0xFFD32F2F)
        elapsedMillis == null || elapsedMillis >= 150_000L -> PixelTone.ORANGE.fill
        else -> SpiderMap.PanelBlue
    }
    val contentColor = if (elapsedMillis != null && elapsedMillis >= 180_000L) Color.White else SpiderMap.Ink
    Surface(shape = SpiderMap.PanelShape, color = color, contentColor = contentColor,
        modifier = Modifier.fillMaxSize().semantics { contentDescription = label }
            .testTag("task2_timer_panel")) {
        BoxWithConstraints(Modifier.fillMaxSize().padding(16.dp), contentAlignment = Alignment.Center) {
            val fontScale = LocalDensity.current.fontScale
            // Measure against the actual glyph width so the timer fills its panel.
            val size = minOf(128f,
                maxWidth.value / (display.length * 0.75f * fontScale),
                maxHeight.value / (2.4f * fontScale)).coerceAtLeast(1f)
            Column(horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.spacedBy(10.dp)) {
                Text("Task 2 • $heading", style = SpiderMap.pixel(11.sp, contentColor), textAlign = TextAlign.Center)
                Text(display, style = SpiderMap.pixel(size.sp, contentColor), maxLines = 1, softWrap = false)
                Text("minutes : seconds : hundredths", style = SpiderMap.pixel(7.sp, contentColor),
                    textAlign = TextAlign.Center)
            }
        }
    }
}

internal fun formatTask2Elapsed(elapsedMillis: Long?): String {
    if (elapsedMillis == null) return "--:--:--"
    val hundredths = elapsedMillis.coerceAtLeast(0L) / 10L
    return String.format(Locale.US, "%02d:%02d:%02d",
        hundredths / 6_000L, hundredths / 100L % 60L, hundredths % 100L)
}
