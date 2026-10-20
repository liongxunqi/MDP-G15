package com.mdp.g15.arena.presentation

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.LocalContentColor
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.draw.clipToBounds
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.viewinterop.AndroidView
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.mdp.g15.arena.domain.PoseSource
import com.mdp.g15.arena.domain.readableHeading
import com.mdp.g15.arena.domain.RobotFootprint
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.view.ArenaGridView
import com.mdp.g15.arena.view.ArenaInteractionListener

@Composable
fun ArenaScreen(
    viewModel: ArenaViewModel,
    robotStatus: String? = null,
    modifier: Modifier = Modifier,
    drivingControls: (@Composable () -> Unit)? = null,
    taskControls: (@Composable () -> Unit)? = null,
    belowResetControls: (@Composable () -> Unit)? = null,
    task2TimerDisplay: (@Composable () -> Unit)? = null,
) {
    val state by viewModel.uiState.collectAsState()
    val displayState = if (state.taskMode == TaskMode.TASK2) state.copy(
        arena = state.arena.copy(obstacles = emptyMap(), selectedObstacleId = null),
    ) else state
    var confirmReset by remember { mutableStateOf(false) }
    val interactions = remember(viewModel) {
        object : ArenaInteractionListener {
            override fun onAddObstacle(position: GridCoordinate) = viewModel.placeObstacle(position)
            override fun onSelectObstacle(obstacleId: Int?) = viewModel.selectObstacle(obstacleId)
            override fun onMoveObstacle(obstacleId: Int, destination: GridCoordinate) =
                viewModel.moveObstacle(obstacleId, destination)

            override fun onRemoveObstacle(obstacleId: Int) = viewModel.removeObstacle(obstacleId)
            override fun onMoveRobot(destination: GridCoordinate) = viewModel.moveRobot(destination)
        }
    }

    // Panels set their own dark text; anything placed straight on the navy backdrop is light.
    BoxWithConstraints(modifier = modifier.fillMaxSize().background(SpiderMap.Backdrop)) {
        CompositionLocalProvider(LocalContentColor provides SpiderMap.OnBackdrop) {
            if (maxWidth >= maxHeight) {
                Row(
                    modifier = Modifier
                        .fillMaxSize()
                        .padding(12.dp),
                    horizontalArrangement = Arrangement.spacedBy(16.dp),
                ) {
                    if (state.taskMode == TaskMode.TASK2) {
                        Task2TimerCard(task2TimerDisplay, Modifier.weight(3f).fillMaxHeight())
                    } else {
                        ArenaCanvas(
                            state = displayState,
                            interactions = interactions,
                            modifier = Modifier.weight(3f).fillMaxHeight(),
                        )
                    }
                    // The panel's frame stays put; only its contents scroll.
                    ArenaStatusPanel(
                        state = displayState,
                        robotStatus = robotStatus,
                        viewModel = viewModel,
                        onReset = { confirmReset = true },
                        taskControls = taskControls,
                        belowResetControls = belowResetControls,
                        drivingControls = drivingControls,
                        modifier = Modifier.weight(2f).fillMaxHeight(),
                    )
                }
            } else {
                Column(
                    modifier = Modifier
                        .fillMaxSize()
                        .padding(12.dp),
                    verticalArrangement = Arrangement.spacedBy(16.dp),
                ) {
                    if (state.taskMode == TaskMode.TASK2) {
                        Task2TimerCard(task2TimerDisplay, Modifier.fillMaxWidth().weight(3f))
                    } else {
                        ArenaCanvas(
                            state = displayState,
                            interactions = interactions,
                            modifier = Modifier.fillMaxWidth().weight(3f),
                        )
                    }
                    // The panel's frame stays put; only its contents scroll.
                    ArenaStatusPanel(
                        state = displayState,
                        robotStatus = robotStatus,
                        viewModel = viewModel,
                        onReset = { confirmReset = true },
                        taskControls = taskControls,
                        belowResetControls = belowResetControls,
                        drivingControls = drivingControls,
                        modifier = Modifier.fillMaxWidth().weight(2f),
                    )
                }
            }
        }
    }

    if (confirmReset) {
        AlertDialog(
            onDismissRequest = { confirmReset = false },
            title = { Text("Reset arena?") },
            text = { Text("All obstacles and the robot pose will be cleared.") },
            confirmButton = {
                TextButton(
                    onClick = {
                        confirmReset = false
                        viewModel.resetArena()
                    },
                ) { Text("Reset") }
            },
            dismissButton = {
                TextButton(onClick = { confirmReset = false }) { Text("Cancel") }
            },
        )
    }
}

/** Task 2 replaces the map with the timer, framed like the status card below it. */
@Composable
private fun Task2TimerCard(content: (@Composable () -> Unit)?, modifier: Modifier) {
    SpiderMapPanel(modifier = modifier) {
        Box(Modifier.fillMaxSize().padding(4.dp), contentAlignment = Alignment.Center) {
            content?.invoke() ?: Text("Task 2 timer unavailable", style = SpiderMap.pixel(12.sp))
        }
    }
}

@Composable
private fun ArenaCanvas(
    state: ArenaUiState,
    interactions: ArenaInteractionListener,
    modifier: Modifier,
    squareViewport: Boolean = false,
) {
    var gridView by remember { mutableStateOf<ArenaGridView?>(null) }
    LaunchedEffect(gridView, state.task1Phase) {
        if (state.task1Phase !in setOf(Task1Phase.SETUP, Task1Phase.PATH_REQUESTED)) {
            gridView?.fitToView()
        }
    }

    SpiderMapPanel(modifier = modifier.testTag("arena_grid_card")) {
        Column(modifier = Modifier.fillMaxSize().clipToBounds()) {
            val viewportModifier = if (squareViewport) {
                Modifier.fillMaxWidth().aspectRatio(1f)
            } else {
                Modifier.fillMaxWidth().weight(1f)
            }
            AndroidView(
                factory = { context ->
                    ArenaGridView(context).apply {
                        interactionListener = interactions
                        gridView = this
                    }
                },
                update = { view ->
                    view.interactionListener = interactions
                    view.render(state.arena, state.placementMode, state.pathOverlay)
                },
                onRelease = { view ->
                    view.interactionListener = null
                    gridView = null
                },
                modifier = viewportModifier.testTag("arena_grid"),
            )
            Row(
                modifier = Modifier
                    .align(Alignment.End)
                    .padding(horizontal = 8.dp, vertical = 4.dp),
                horizontalArrangement = Arrangement.spacedBy(4.dp),
            ) {
                PixelButton("− Zoom out", { gridView?.zoomOut() }, Modifier.height(40.dp),
                    contentPadding = PaddingValues(horizontal = 12.dp))
                PixelButton("+ Zoom in", { gridView?.zoomIn() }, Modifier.height(40.dp),
                    contentPadding = PaddingValues(horizontal = 12.dp))
            }
        }
    }
}

@Composable
private fun ArenaStatusPanel(
    state: ArenaUiState,
    robotStatus: String?,
    viewModel: ArenaViewModel,
    onReset: () -> Unit,
    taskControls: (@Composable () -> Unit)? = null,
    belowResetControls: (@Composable () -> Unit)? = null,
    drivingControls: (@Composable () -> Unit)? = null,
    modifier: Modifier = Modifier,
) {
    val selected = state.arena.selectedObstacle
    SpiderMapPanel(modifier = modifier) { PixelTextScope {
        Column(
            // Inset by the 4 dp border before scrolling, so contents scroll beneath the
            // frame's inner edge instead of over it.
            modifier = Modifier.fillMaxWidth().padding(4.dp)
                .verticalScroll(rememberScrollState()).padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            taskControls?.invoke()
            Text(if (state.taskMode == TaskMode.TASK2) "Task 2 status" else "Arena status",
                style = SpiderMap.Title)
            Text("Latest action", style = SpiderMap.Label)
            Text(
                text = state.feedback,
                color = if (state.feedbackIsError) SpiderMap.Warning else SpiderMap.Text,
                style = SpiderMap.Body,
            )
            if (state.taskMode == TaskMode.TASK1 && state.latestReceived.isNotEmpty()) {
                StatusValue("Last received", state.latestReceived)
            }
            if (state.taskMode == TaskMode.TASK2) {
                StatusValue("Robot status", robotStatus?.takeIf(String::isNotBlank) ?: state.status)
            } else {
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                    StatusValue(
                        "Robot status",
                        robotStatus?.takeIf(String::isNotBlank) ?: state.status,
                        Modifier.weight(1f),
                    )
                    StatusValue(
                        "Robot pose",
                        state.arena.robot?.let {
                            val source = when (it.source) {
                                PoseSource.REPORTED -> "RPi reported"
                                PoseSource.OPERATOR -> "operator placed"
                                PoseSource.ESTIMATED -> "estimated"
                                PoseSource.SIMULATED -> "path demo"
                            }
                            val warning = when {
                                !RobotFootprint.fitsArena(it, state.arena.config) -> " · outside arena"
                                state.arena.obstacles.values.any { obstacle ->
                                    RobotFootprint.overlapsCell(it, state.arena.config, obstacle.position)
                                } -> " · overlaps obstacle"
                                else -> ""
                            }
                            String.format(java.util.Locale.US, "(%.2f, %.2f) • %s • %s",
                                it.x, it.y, it.readableHeading(), source) + warning
                        } ?: "Not available",
                        Modifier.weight(1f),
                    )
                    StatusValue(
                        "Obstacles",
                        "${state.arena.obstacles.size}/${state.arena.config.maxObstacles}",
                        Modifier.weight(0.6f),
                    )
                }
                StatusValue(
                    "Selected obstacle",
                    selected?.let {
                        buildString {
                            append("#${it.id} • (${it.position.x}, ${it.position.y})")
                            append(" • face ${it.targetFace?.wireValue ?: "unset"}")
                            it.targetId?.let { target -> append(" • target $target") }
                        }
                    } ?: "None",
                )
            }

            if (state.taskMode == TaskMode.TASK1) {
                HorizontalDivider(thickness = 2.dp, color = SpiderMap.FrameDark)
                Text("Target face", style = SpiderMap.Label)
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(6.dp),
                ) {
                    // Coral tabs, like the reference's O / S / T; filled = the chosen face.
                    Direction.entries.forEach { direction ->
                        PixelButton(
                            text = direction.wireValue,
                            onClick = { viewModel.setTargetFace(direction) },
                            tone = PixelTone.CORAL,
                            enabled = selected != null,
                            selected = selected?.targetFace == direction,
                            contentPadding = PaddingValues(horizontal = 14.dp, vertical = 6.dp),
                        )
                    }
                }

                val atObstacleLimit = state.arena.obstacles.size >= state.arena.config.maxObstacles
                PixelButton(
                    text = when {
                        state.placementMode -> "Cancel placement"
                        atObstacleLimit -> "Obstacle limit reached"
                        else -> "Add obstacle"
                    },
                    onClick = { viewModel.setPlacementMode(!state.placementMode) },
                    tone = if (state.placementMode) PixelTone.CORAL else PixelTone.GREEN,
                    enabled = state.placementMode || (!atObstacleLimit && !state.manualPending && !state.manualAnimating &&
                        !state.autonomousRunning && !state.pathDemoRunning),
                    modifier = Modifier.fillMaxWidth(),
                )
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp),
                ) {
                    PixelButtonCell("Undo", viewModel::undo, PixelTone.GREEN, enabled = state.canUndo)
                    PixelButtonCell("Redo", viewModel::redo, PixelTone.GREEN, enabled = state.canRedo)
                }
                PixelButton("Reset arena", onReset, Modifier.fillMaxWidth(), PixelTone.ORANGE)
            }
            if (state.taskMode == TaskMode.TASK1) {
                ResyncOnReconnectControl(
                    phase = state.task1Phase,
                    armed = state.forceMapResyncOnReconnect,
                    onArm = viewModel::armMapResyncOnReconnect,
                    onCancel = viewModel::cancelMapResyncOnReconnect,
                )
            }
            drivingControls?.invoke()
            // Driving-related extras sit under the D-pad: the Amd tool / normal mode switch
            // (passed in as belowResetControls), then the path demo.
            if (state.taskMode == TaskMode.TASK1) {
                belowResetControls?.invoke()
                // Visual only: disabled while playing, so a press always starts from move 0.
                PixelButton(
                    text = if (state.pathDemoRunning) "Playing path demo…" else "Play Path Demo",
                    onClick = viewModel::playPathDemo,
                    tone = PixelTone.ORANGE,
                    enabled = !state.pathDemoRunning && !state.autonomousRunning && !state.manualPending &&
                        !state.manualAnimating && !state.placementMode,
                    modifier = Modifier.fillMaxWidth().testTag("play_path_demo"),
                )
            }
        }
    } }
}

/**
 * Recovery override for [ArenaViewModel.connectionChanged]'s normal rule: once BEGIN has been
 * pressed, a reconnect no longer resends CLEAR + the obstacle list on its own (it would wipe
 * the RPi's mid-mission state for no reason). This lets the operator explicitly ask for that
 * resend anyway, armed for one connect only. Disabled before BEGIN (phase SETUP/PATH_REQUESTED),
 * since a reconnect there already resends the map automatically.
 */
@Composable
private fun ResyncOnReconnectControl(phase: Task1Phase, armed: Boolean, onArm: () -> Unit, onCancel: () -> Unit) {
    val eligible = phase !in setOf(Task1Phase.SETUP, Task1Phase.PATH_REQUESTED)
    Column(verticalArrangement = Arrangement.spacedBy(2.dp)) {
        if (armed) {
            PixelButton("Map resync armed for next connect — tap to cancel", onCancel,
                Modifier.fillMaxWidth(), PixelTone.CORAL)
        } else {
            PixelButton("Resend map on next reconnect", onArm, Modifier.fillMaxWidth(),
                PixelTone.ORANGE, enabled = eligible)
        }
        Text(
            "Sends CLEAR + every obstacle to the RPi the next time it (re)connects — one time only. " +
                "Use after BEGIN only if the RPi's map is known to be out of sync; it can disrupt an active mission.",
            style = SpiderMap.Label.copy(fontWeight = FontWeight.Normal),
        )
    }
}

@Composable
private fun StatusValue(label: String, value: String, modifier: Modifier = Modifier) {
    Column(modifier) {
        Text(label, style = SpiderMap.Label)
        Text(value, style = SpiderMap.Body)
    }
}
