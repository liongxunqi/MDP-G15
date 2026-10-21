package com.mdp.g15.arena.presentation

import androidx.compose.foundation.background
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.VerticalDivider
import androidx.compose.material3.LocalContentColor
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Surface
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
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
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
    taskStatus: (@Composable () -> Unit)? = null,
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

    // Task actions stay above the scrolling map and status content in both orientations.
    BoxWithConstraints(modifier = modifier.fillMaxSize().background(SpiderMap.Backdrop)) {
        val landscape = maxWidth >= maxHeight
        val portraitScroll = rememberScrollState()
        LaunchedEffect(state.taskMode, maxWidth, maxHeight) { portraitScroll.scrollTo(0) }
        CompositionLocalProvider(LocalContentColor provides SpiderMap.OnBackdrop) {
            Column(Modifier.fillMaxSize().padding(vertical = 4.dp)) {
                SpiderMapPanel(Modifier.fillMaxWidth().padding(horizontal = 4.dp, vertical = 2.dp)
                    .testTag("task_action_bar")) {
                    Column(Modifier.fillMaxWidth().padding(8.dp)) { taskControls?.invoke() }
                }
            if (landscape) {
                BoxWithConstraints(Modifier.fillMaxWidth().weight(1f)) {
                    val paneHeight = maxHeight - 8.dp
                    Row(Modifier.fillMaxSize().padding(4.dp),
                        horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                    if (state.taskMode == TaskMode.TASK2) {
                        Task2TimerCard(task2TimerDisplay,
                            Modifier.width(paneHeight).fillMaxHeight())
                    } else {
                        ArenaCanvas(
                            state = displayState,
                            interactions = interactions,
                            modifier = Modifier.width(paneHeight).fillMaxHeight(),
                        )
                    }
                    Column(Modifier.weight(1f).fillMaxHeight().verticalScroll(rememberScrollState()),
                        verticalArrangement = Arrangement.spacedBy(4.dp)) {
                        ArenaStatusPanel(displayState, robotStatus, viewModel, taskStatus,
                            drivingControls, Modifier.fillMaxWidth().height(paneHeight))
                        ArenaSecondaryPanel(displayState, viewModel, { confirmReset = true },
                            belowResetControls)
                    }
                    }
                }
            } else {
                BoxWithConstraints(Modifier.fillMaxWidth().weight(1f)) {
                    if (state.taskMode == TaskMode.TASK2) {
                        val statusHeight = minOf(190.dp, maxHeight * 0.24f)
                        Column(Modifier.fillMaxSize().padding(horizontal = 4.dp),
                            verticalArrangement = Arrangement.spacedBy(4.dp)) {
                            Task2TimerCard(task2TimerDisplay, Modifier.fillMaxWidth().weight(1f))
                            ArenaStatusPanel(displayState, robotStatus, viewModel, taskStatus,
                                drivingControls, Modifier.fillMaxWidth().height(statusHeight))
                        }
                    } else {
                        val availableHeight = maxHeight
                        // Keep the essential controls compact on taller tablets; the map uses
                        // the remaining height and is never smaller than 60% of this area.
                        val mapHeight = availableHeight - minOf(availableHeight * 0.40f, 306.dp)
                        val statusHeight = availableHeight - mapHeight - 4.dp
                        Column(Modifier.fillMaxSize().verticalScroll(portraitScroll)
                            .padding(horizontal = 4.dp),
                            verticalArrangement = Arrangement.spacedBy(4.dp)) {
                            ArenaCanvas(displayState, interactions,
                                Modifier.fillMaxWidth().height(mapHeight))
                            ArenaStatusPanel(displayState, robotStatus, viewModel, taskStatus,
                                drivingControls, Modifier.fillMaxWidth().height(statusHeight))
                            ArenaSecondaryPanel(displayState, viewModel, { confirmReset = true },
                                belowResetControls)
                        }
                    }
                }
            }
            }
        }
    }

    if (confirmReset) {
        AlertDialog(
            onDismissRequest = { confirmReset = false },
            shape = SpiderMap.PanelShape,
            containerColor = SpiderMap.PanelBlue,
            titleContentColor = SpiderMap.Ink,
            textContentColor = SpiderMap.Ink,
            title = { Text("Reset arena?", style = SpiderMap.Title) },
            text = { Text("All obstacles and the robot pose will be cleared.") },
            confirmButton = {
                PixelButton("Reset", {
                        confirmReset = false
                        viewModel.resetArena()
                    }, tone = PixelTone.CORAL)
            },
            dismissButton = { PixelButton("Cancel", { confirmReset = false }, selected = false) },
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
                    .fillMaxWidth()
                    .padding(horizontal = 8.dp, vertical = 4.dp),
                horizontalArrangement = Arrangement.spacedBy(4.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                PixelSpiderIcon(Modifier.size(44.dp))
                Spacer(Modifier.weight(1f))
                PixelWebIcon(Modifier.size(40.dp))
                Spacer(Modifier.weight(1f))
                PixelButton("−", { gridView?.zoomOut() },
                    Modifier.size(48.dp).semantics { contentDescription = "Zoom out" },
                    contentPadding = PaddingValues(0.dp))
                PixelButton("+", { gridView?.zoomIn() },
                    Modifier.size(48.dp).semantics { contentDescription = "Zoom in" },
                    contentPadding = PaddingValues(0.dp))
            }
        }
    }
}

@Composable
private fun ArenaStatusPanel(
    state: ArenaUiState,
    robotStatus: String?,
    viewModel: ArenaViewModel,
    taskStatus: (@Composable () -> Unit)? = null,
    drivingControls: (@Composable () -> Unit)? = null,
    modifier: Modifier = Modifier,
) {
    Column(modifier) {
        BoxWithConstraints(Modifier.fillMaxSize()) {
            if (state.taskMode == TaskMode.TASK2) {
                SpiderMapPanel(Modifier.fillMaxSize().testTag("status_card")) {
                    StatusDetails(state, robotStatus, taskStatus)
                }
            } else if (maxWidth >= 350.dp) {
                val compactRight = maxHeight < 290.dp
                SpiderMapPanel(Modifier.fillMaxSize().testTag("status_card")) {
                    Row(Modifier.fillMaxSize().padding(8.dp),
                        horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                        ControlInset(Modifier.weight(1f).fillMaxHeight()) {
                            StatusDetails(state, robotStatus, taskStatus)
                        }
                        val rightModifier = Modifier.weight(1f).fillMaxHeight()
                            .testTag("obstacle_card")
                        Column(if (compactRight) rightModifier.verticalScroll(rememberScrollState())
                               else rightModifier,
                            verticalArrangement = Arrangement.spacedBy(4.dp)) {
                            Row(if (compactRight) Modifier.fillMaxWidth().height(152.dp)
                                else Modifier.fillMaxWidth().weight(1f),
                                horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                              ControlInset(Modifier.weight(1f).fillMaxHeight()) {
                                ObstacleSelection(state)
                              }
                              ControlInset(Modifier.weight(1f).fillMaxHeight()) {
                                drivingControls?.invoke()
                              }
                            }
                            ControlInset(Modifier.fillMaxWidth()) {
                                ObstacleButtons(state, viewModel)
                            }
                        }
                    }
                }
            } else {
                SpiderMapPanel(Modifier.fillMaxSize().testTag("status_card")) {
                    ControlInset(Modifier.fillMaxWidth().padding(4.dp)) {
                        StatusDetails(state, robotStatus, taskStatus)
                    }
                    Column(Modifier.fillMaxWidth().testTag("obstacle_card")) {
                        ControlInset(Modifier.fillMaxWidth().padding(4.dp)) { drivingControls?.invoke() }
                        ControlInset(Modifier.fillMaxWidth().padding(4.dp)) { ObstacleSelection(state) }
                        ControlInset(Modifier.fillMaxWidth().padding(4.dp)) { ObstacleButtons(state, viewModel) }
                    }
                }
            }
        }
    }
}

@Composable
private fun ControlInset(modifier: Modifier = Modifier, content: @Composable () -> Unit) {
    Surface(modifier, shape = SpiderMap.PanelShape,
        color = Color(0xFF54B8EB), contentColor = SpiderMap.Ink,
        border = BorderStroke(2.dp, SpiderMap.FrameDark)) { content() }
}

@Composable
private fun ArenaSecondaryPanel(state: ArenaUiState, viewModel: ArenaViewModel,
                                onReset: () -> Unit, belowResetControls: (@Composable () -> Unit)?) {
        if (state.taskMode == TaskMode.TASK1) {
            SpiderMapPanel(Modifier.fillMaxWidth().testTag("secondary_card")) {
                Column(Modifier.fillMaxWidth().padding(8.dp),
                    verticalArrangement = Arrangement.spacedBy(4.dp)) {
                    PixelButton("Reset arena", onReset, Modifier.fillMaxWidth(), PixelTone.CORAL)
                    ResyncOnReconnectControl(
                        phase = state.task1Phase,
                        armed = state.forceMapResyncOnReconnect,
                        onArm = viewModel::armMapResyncOnReconnect,
                        onCancel = viewModel::cancelMapResyncOnReconnect,
                    )
                    belowResetControls?.invoke()
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
        }
}

@Composable
private fun StatusDetails(state: ArenaUiState, robotStatus: String?,
                          taskStatus: (@Composable () -> Unit)?) {
    Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(4.dp)) {
        Text(if (state.taskMode == TaskMode.TASK2) "Task 2 status" else "Arena status",
            style = SpiderMap.pixel(10.sp))
        taskStatus?.invoke()
        StatusValue("Latest action", state.feedback, error = state.feedbackIsError)
        if (state.taskMode == TaskMode.TASK1 && state.latestReceived.isNotEmpty()) {
            StatusValue("Last received", state.latestReceived)
        }
        StatusValue("Robot status", robotStatus?.takeIf(String::isNotBlank) ?: state.status)
        if (state.taskMode == TaskMode.TASK1) {
            StatusValue("Robot pose", state.arena.robot?.let {
                val source = when (it.source) {
                    PoseSource.REPORTED -> "RPi reported"
                    PoseSource.OPERATOR -> "operator placed"
                    PoseSource.ESTIMATED -> "estimated"
                    PoseSource.SIMULATED -> "preview"
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
            } ?: "Not available")
            StatusValue("Obstacles", "${state.arena.obstacles.size}/${state.arena.config.maxObstacles}")
        }
    }
}

@Composable
private fun ObstacleSelection(state: ArenaUiState) {
    val selected = state.arena.selectedObstacle
    Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(5.dp)) {
        Text("Obstacle", style = SpiderMap.pixel(9.sp))
        if (selected == null) {
            Text("None", style = SpiderMap.pixel(7.sp))
        } else {
            StatusValue("ID", "#${selected.id}")
            StatusValue("Coordinates", "(${selected.position.x}, ${selected.position.y})")
            StatusValue("Face", selected.targetFace?.wireValue ?: "unset",
                Modifier.testTag("selected_obstacle_face").semantics(mergeDescendants = true) {})
            selected.targetId?.let { StatusValue("Target", "$it",
                Modifier.testTag("selected_obstacle_target").semantics(mergeDescendants = true) {}) }
        }
    }
}

@Composable
private fun ObstacleButtons(state: ArenaUiState, viewModel: ArenaViewModel) {
    val selected = state.arena.selectedObstacle
    Column(Modifier.fillMaxWidth().padding(8.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
        Text("Target face", style = SpiderMap.pixel(9.sp))
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(4.dp)) {
            Direction.entries.forEach { direction ->
                PixelButton(direction.wireValue, { viewModel.setTargetFace(direction) },
                    Modifier.weight(1f), PixelTone.CORAL, enabled = selected != null,
                    selected = selected?.targetFace == direction,
                    contentPadding = PaddingValues(0.dp))
            }
        }
        val atObstacleLimit = state.arena.obstacles.size >= state.arena.config.maxObstacles
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(4.dp),
            verticalAlignment = Alignment.CenterVertically) {
            PixelButton(
                text = when {
                    state.placementMode -> "Cancel placement"
                    atObstacleLimit -> "Obstacle limit reached"
                    else -> "Add obstacle"
                },
                onClick = { viewModel.setPlacementMode(!state.placementMode) },
                tone = if (state.placementMode) PixelTone.CORAL else PixelTone.GREEN,
                enabled = state.placementMode || (!atObstacleLimit && !state.manualPending &&
                    !state.manualAnimating && !state.autonomousRunning && !state.pathDemoRunning),
                modifier = Modifier.weight(2f).height(44.dp),
                contentPadding = PaddingValues(horizontal = 2.dp),
                fontSize = 7.sp,
            )
            PixelHistoryButton("Undo", false, state.canUndo, viewModel::undo, Modifier.weight(1f))
            PixelHistoryButton("Redo", true, state.canRedo, viewModel::redo, Modifier.weight(1f))
        }
    }
}

@Composable
private fun PixelHistoryButton(label: String, redo: Boolean, enabled: Boolean,
                               onClick: () -> Unit, modifier: Modifier = Modifier) {
    Button(onClick = onClick, enabled = enabled,
        modifier = modifier.height(44.dp).semantics { contentDescription = label },
        shape = SpiderMap.ButtonShape, border = BorderStroke(3.dp, SpiderMap.Ink),
        contentPadding = PaddingValues(0.dp),
        colors = ButtonDefaults.buttonColors(containerColor = PixelTone.GREEN.fill,
            contentColor = SpiderMap.Ink, disabledContainerColor = Color(0xFF3E5F7A),
            disabledContentColor = Color(0xFF7FA3BF))) {
        Canvas(Modifier.size(22.dp)) {
            val cell = size.width / 11f
            fun pixel(x: Int, y: Int) = drawRect(SpiderMap.Ink,
                topLeft = Offset(x * cell, y * cell), size = Size(cell, cell))
            // A bent return shaft distinguishes history from the D-pad's straight arrows.
            val cells = listOf(2 to 7, 3 to 6, 3 to 7, 3 to 8, 4 to 5, 4 to 7, 4 to 9,
                5 to 4, 5 to 7, 6 to 3, 6 to 7, 7 to 3, 7 to 7, 8 to 3,
                8 to 7, 9 to 4, 9 to 5, 9 to 6)
            cells.forEach { (x, y) -> pixel(if (redo) 10 - x else x, y) }
        }
    }
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
            PixelButton("Map resync armed — tap to cancel", onCancel,
                Modifier.fillMaxWidth(), PixelTone.CORAL, fontSize = 7.sp)
        } else {
            PixelButton("Resend map on next reconnect", onArm, Modifier.fillMaxWidth(),
                PixelTone.ORANGE, enabled = eligible, fontSize = 7.sp)
        }
    }
}

@Composable
private fun StatusValue(label: String, value: String, modifier: Modifier = Modifier,
                        error: Boolean = false) {
    Column(modifier, verticalArrangement = Arrangement.spacedBy(5.dp)) {
        StatusHeading(label)
        Text(value, style = SpiderMap.pixel(7.sp),
            color = if (error) SpiderMap.Warning else SpiderMap.Text)
    }
}

@Composable
private fun StatusHeading(label: String) {
    Text(label, modifier = Modifier.background(SpiderMap.NavyDeep, SpiderMap.PanelShape)
        .padding(horizontal = 6.dp, vertical = 3.dp),
        style = SpiderMap.pixel(6.sp, SpiderMap.PanelBlue))
}
