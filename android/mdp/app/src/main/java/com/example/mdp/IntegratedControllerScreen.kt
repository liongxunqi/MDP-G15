package com.example.mdp

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.material3.PrimaryTabRow
import androidx.compose.material3.Tab
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.lifecycle.viewmodel.compose.viewModel
import com.mdp.g15.arena.domain.ArenaReducer
import com.mdp.g15.arena.domain.step
import com.mdp.g15.arena.integration.ArenaOutboundSink
import com.mdp.g15.arena.presentation.ArenaScreen
import com.mdp.g15.arena.presentation.ArenaViewModel
import kotlinx.coroutines.flow.Flow

private enum class ControllerTab(val title: String) {
    CONTROLS("Controls"),
    ARENA("Arena"),
}

@Composable
fun IntegratedControllerScreen(
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
    onForwardLeft: () -> Unit,
    onForwardRight: () -> Unit,
    onBackLeft: () -> Unit,
    onBackRight: () -> Unit,
    onBegin: () -> Unit,
    incomingMessages: Flow<String>,
    onArenaOutbound: (String) -> Unit,
    modifier: Modifier = Modifier,
) {
    var selectedTab by rememberSaveable { mutableIntStateOf(ControllerTab.CONTROLS.ordinal) }
    val currentOutbound = rememberUpdatedState(onArenaOutbound)
    val outboundSink = remember {
        ArenaOutboundSink { message -> currentOutbound.value(message) }
    }
    val arenaFactory = remember(outboundSink) { ArenaViewModel.Factory(outboundSink) }
    val arenaViewModel: ArenaViewModel = viewModel(factory = arenaFactory)

    LaunchedEffect(incomingMessages, arenaViewModel) {
        incomingMessages.collect(arenaViewModel::accept)
    }

    // Guard only — this does NOT move the robot locally. It just disables Forward/Reverse
    // when the arena data we already have (from the last ROBOT report + placed obstacles)
    // shows the next cell is off-grid or occupied, reusing ArenaReducer's own validation
    // so there's exactly one place that knows what a "valid move" is.
    val arenaState by arenaViewModel.uiState.collectAsState()
    val arenaReducer = remember { ArenaReducer() }
    val robot = arenaState.arena.robot
    val forwardAllowed = robot == null ||
        arenaReducer.canMoveRobot(arenaState.arena, robot.position.step(robot.direction))
    val reverseAllowed = robot == null ||
        arenaReducer.canMoveRobot(arenaState.arena, robot.position.step(robot.direction.opposite()))

    Column(modifier = modifier.fillMaxSize()) {
        PrimaryTabRow(selectedTabIndex = selectedTab) {
            ControllerTab.entries.forEach { tab ->
                Tab(
                    selected = selectedTab == tab.ordinal,
                    onClick = { selectedTab = tab.ordinal },
                    text = { Text(tab.title) },
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
                    robotStatus = robotStatus,
                    isConnected = isConnected,
                    isBusy = isBusy,
                    permissionsGranted = permissionsGranted,
                    latestMessage = latestMessage,
                    forwardEnabled = forwardAllowed,
                    reverseEnabled = reverseAllowed,
                    onConnectClick = onConnectClick,
                    onDisconnectClick = onDisconnectClick,
                    onForward = onForward,
                    onReverse = onReverse,
                    onTurnLeft = onTurnLeft,
                    onTurnRight = onTurnRight,
                    onStop = onStop,
                    onForwardLeft = onForwardLeft,
                    onForwardRight = onForwardRight,
                    onBackLeft = onBackLeft,
                    onBackRight = onBackRight,
                    onBegin = onBegin,
                    modifier = Modifier.fillMaxSize(),
                )

                ControllerTab.ARENA -> ArenaScreen(
                    viewModel = arenaViewModel,
                    modifier = Modifier.fillMaxSize(),
                )
            }
        }
    }
}
