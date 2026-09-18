package com.mdp.g15.arena.presentation

import androidx.compose.foundation.BorderStroke
import androidx.compose.material3.CardDefaults
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.aspectRatio
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
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.FilterChip
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
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
import androidx.compose.ui.unit.dp
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.view.ArenaGridView
import com.mdp.g15.arena.view.ArenaInteractionListener

@Composable
fun ArenaScreen(
    viewModel: ArenaViewModel,
    robotStatus: String? = null,
    modifier: Modifier = Modifier,
) {
    val state by viewModel.uiState.collectAsState()
    var confirmReset by remember { mutableStateOf(false) }
    val interactions = remember(viewModel) {
        object : ArenaInteractionListener {
            override fun onAddObstacle(position: GridCoordinate) = viewModel.addObstacle(position)
            override fun onSelectObstacle(obstacleId: Int?) = viewModel.selectObstacle(obstacleId)
            override fun onMoveObstacle(obstacleId: Int, destination: GridCoordinate) =
                viewModel.moveObstacle(obstacleId, destination)

            override fun onRemoveObstacle(obstacleId: Int) = viewModel.removeObstacle(obstacleId)
            override fun onMoveRobot(destination: GridCoordinate) = viewModel.moveRobot(destination)
        }
    }

    BoxWithConstraints(modifier = modifier.fillMaxSize()) {
        if (maxWidth >= maxHeight) {
            Row(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(12.dp),
                horizontalArrangement = Arrangement.spacedBy(16.dp),
            ) {
                ArenaCanvas(
                    state = state,
                    interactions = interactions,
                    modifier = Modifier
                        .weight(2f)
                        .fillMaxHeight(),
                )
                ArenaStatusPanel(
                    state = state,
                    robotStatus = robotStatus,
                    viewModel = viewModel,
                    onReset = { confirmReset = true },
                    modifier = Modifier
                        .weight(1f)
                        .fillMaxHeight()
                        .verticalScroll(rememberScrollState()),
                )
            }
        } else {
            Column(
                modifier = Modifier
                    .fillMaxSize()
                    .verticalScroll(rememberScrollState())
                    .padding(12.dp),
                verticalArrangement = Arrangement.spacedBy(16.dp),
            ) {
                ArenaCanvas(
                    state = state,
                    interactions = interactions,
                    squareViewport = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                ArenaStatusPanel(
                    state = state,
                    robotStatus = robotStatus,
                    viewModel = viewModel,
                    onReset = { confirmReset = true },
                    modifier = Modifier.fillMaxWidth(),
                )
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

@Composable
private fun ArenaCanvas(
    state: ArenaUiState,
    interactions: ArenaInteractionListener,
    modifier: Modifier,
    squareViewport: Boolean = false,
) {
    var gridView by remember { mutableStateOf<ArenaGridView?>(null) }

    Card(
        modifier = modifier.testTag("arena_grid_card"),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface),
        border = BorderStroke(1.dp, MaterialTheme.colorScheme.outlineVariant),
    ) {
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
                    view.render(state.arena, state.placementMode)
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
                OutlinedButton(
                    onClick = { gridView?.zoomOut() },
                    modifier = Modifier.height(40.dp),
                    contentPadding = PaddingValues(horizontal = 12.dp),
                ) { Text("− Zoom out") }
                OutlinedButton(
                    onClick = { gridView?.zoomIn() },
                    modifier = Modifier.height(40.dp),
                    contentPadding = PaddingValues(horizontal = 12.dp),
                ) { Text("+ Zoom in") }
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
    modifier: Modifier = Modifier,
) {
    val selected = state.arena.selectedObstacle
    Card(
        modifier = modifier,
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface),
        border = BorderStroke(1.dp, MaterialTheme.colorScheme.outlineVariant),
    ) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            Text("Arena status", style = MaterialTheme.typography.titleLarge)
            Text("Latest action", style = MaterialTheme.typography.labelLarge)
            Text(
                text = state.feedback,
                color = if (state.feedbackIsError) {
                    MaterialTheme.colorScheme.error
                } else {
                    MaterialTheme.colorScheme.onSurfaceVariant
                },
                style = MaterialTheme.typography.bodyMedium,
            )
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                StatusValue(
                    "Robot status",
                    robotStatus?.takeIf(String::isNotBlank) ?: state.status,
                    Modifier.weight(1f),
                )
                StatusValue(
                    "Robot pose",
                    state.arena.robot?.let {
                        "(${it.position.x}, ${it.position.y}) • ${it.direction.name}"
                    } ?: "Not available",
                    Modifier.weight(1f),
                )
                StatusValue("Obstacles", state.arena.obstacles.size.toString(), Modifier.weight(0.6f))
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

            HorizontalDivider()
            Text("Target face", style = MaterialTheme.typography.labelLarge)
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(6.dp),
            ) {
                Direction.entries.forEach { direction ->
                    FilterChip(
                        selected = selected?.targetFace == direction,
                        onClick = { viewModel.setTargetFace(direction) },
                        enabled = selected != null,
                        label = { Text(direction.wireValue) },
                    )
                }
            }

            Button(
                onClick = { viewModel.setPlacementMode(!state.placementMode) },
                modifier = Modifier.fillMaxWidth(),
            ) {
                Text(if (state.placementMode) "Cancel adding" else "Add obstacle")
            }
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp),
            ) {
                OutlinedButton(
                    onClick = viewModel::undo,
                    enabled = state.canUndo,
                    modifier = Modifier.weight(1f),
                ) { Text("Undo") }
                OutlinedButton(
                    onClick = viewModel::redo,
                    enabled = state.canRedo,
                    modifier = Modifier.weight(1f),
                ) { Text("Redo") }
            }
            OutlinedButton(onClick = onReset, modifier = Modifier.fillMaxWidth()) {
                Text("Reset arena")
            }

        }
    }
}

@Composable
private fun StatusValue(label: String, value: String, modifier: Modifier = Modifier) {
    Column(modifier) {
        Text(label, style = MaterialTheme.typography.labelMedium, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, style = MaterialTheme.typography.bodyMedium)
    }
}
