package com.mdp.g15.arena.presentation

import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import androidx.lifecycle.createSavedStateHandle
import androidx.lifecycle.viewModelScope
import androidx.lifecycle.viewmodel.CreationExtras
import com.mdp.g15.arena.domain.ArenaAction
import com.mdp.g15.arena.domain.ArenaEditSnapshot
import com.mdp.g15.arena.domain.ArenaOutboundEvent
import com.mdp.g15.arena.domain.ArenaReducer
import com.mdp.g15.arena.domain.ArenaReduction
import com.mdp.g15.arena.domain.ArenaState
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.domain.Obstacle
import com.mdp.g15.arena.domain.RobotPose
import com.mdp.g15.arena.integration.ArenaInboundConsumer
import com.mdp.g15.arena.integration.ArenaOutboundSink
import com.mdp.g15.arena.protocol.ArenaDecodeResult
import com.mdp.g15.arena.protocol.ArenaInboundEvent
import com.mdp.g15.arena.protocol.ArenaMessageCodec
import com.mdp.g15.arena.protocol.CsvArenaMessageCodec
import java.util.ArrayDeque
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

class ArenaViewModel(
    private val savedStateHandle: SavedStateHandle,
    private val outboundSink: ArenaOutboundSink,
    private val reducer: ArenaReducer = ArenaReducer(),
    private val codec: ArenaMessageCodec = CsvArenaMessageCodec(),
) : ViewModel(), ArenaInboundConsumer {
    private val undoHistory = ArrayDeque<ArenaEditSnapshot>()
    private val redoHistory = ArrayDeque<ArenaEditSnapshot>()

    private val _uiState = MutableStateFlow(restoreUiState())
    val uiState: StateFlow<ArenaUiState> = _uiState.asStateFlow()

    override fun accept(message: String) {
        viewModelScope.launch {
            when (val decoded = codec.decode(message)) {
                is ArenaDecodeResult.Decoded -> handleInbound(decoded.event)
                is ArenaDecodeResult.Malformed -> setFeedback(decoded.reason, isError = true)
                ArenaDecodeResult.Ignored -> Unit
            }
        }
    }

    fun setPlacementMode(enabled: Boolean) {
        _uiState.value = _uiState.value.copy(
            placementMode = enabled,
            feedback = if (enabled) "Touch an empty cell to place an obstacle." else "Placement cancelled.",
            feedbackIsError = false,
        )
    }

    fun addObstacle(position: GridCoordinate) = dispatchLocal(
        action = ArenaAction.AddObstacle(position),
        successMessage = "Obstacle placed at (${position.x}, ${position.y}).",
        leavePlacementMode = true,
    )

    fun moveObstacle(obstacleId: Int, destination: GridCoordinate) = dispatchLocal(
        ArenaAction.MoveObstacle(obstacleId, destination),
        "Obstacle $obstacleId moved to (${destination.x}, ${destination.y}).",
    )

    fun removeObstacle(obstacleId: Int) = dispatchLocal(
        ArenaAction.RemoveObstacle(obstacleId),
        "Obstacle $obstacleId removed.",
    )

    fun selectObstacle(obstacleId: Int?) {
        applyReduction(reducer.reduce(_uiState.value.arena, ArenaAction.SelectObstacle(obstacleId))) {
            _uiState.value = _uiState.value.copy(
                arena = it,
                feedback = obstacleId?.let { id -> "Obstacle $id selected." } ?: "Selection cleared.",
                feedbackIsError = false,
            )
            persist()
        }
    }

    fun setTargetFace(face: Direction) {
        val obstacleId = _uiState.value.arena.selectedObstacleId
        if (obstacleId == null) {
            setFeedback("Select an obstacle before choosing its target face.", isError = true)
            return
        }
        dispatchLocal(
            ArenaAction.SetTargetFace(obstacleId, face),
            "Obstacle $obstacleId target face set to ${face.name.lowercase()}.",
        )
    }

    fun resetArena() {
        val before = _uiState.value.arena
        if (before.obstacles.isEmpty() && before.robot == null) {
            setFeedback("Arena is already empty.", isError = false)
            return
        }
        pushUndo(before)
        val reset = (reducer.reduce(before, ArenaAction.Reset) as ArenaReduction.Success).state
        _uiState.value = _uiState.value.copy(
            arena = reset,
            placementMode = false,
            canUndo = undoHistory.isNotEmpty(),
            canRedo = false,
            feedback = "Arena reset.",
            feedbackIsError = false,
        )
        before.obstacles.keys.sorted().forEach {
            outboundSink.submit(codec.encode(ArenaOutboundEvent.RemoveObstacle(it)))
        }
        persist()
    }

    fun undo() {
        if (undoHistory.isEmpty()) return
        val before = _uiState.value.arena
        val target = undoHistory.removeLast()
        redoHistory.addLast(ArenaEditSnapshot.from(before))
        val restored = target.applyTo(before)
        syncObstacleDiff(before, restored)
        updateAfterHistory(restored, "Last arena edit undone.")
    }

    fun redo() {
        if (redoHistory.isEmpty()) return
        val before = _uiState.value.arena
        val target = redoHistory.removeLast()
        undoHistory.addLast(ArenaEditSnapshot.from(before))
        val restored = target.applyTo(before)
        syncObstacleDiff(before, restored)
        updateAfterHistory(restored, "Arena edit restored.")
    }

    private fun dispatchLocal(
        action: ArenaAction,
        successMessage: String,
        leavePlacementMode: Boolean = false,
    ) {
        val before = _uiState.value.arena
        when (val reduction = reducer.reduce(before, action)) {
            is ArenaReduction.Failure -> setFeedback(reduction.reason, isError = true)
            is ArenaReduction.Success -> {
                if (reduction.state != before) pushUndo(before)
                reduction.outboundEvent?.let { outboundSink.submit(codec.encode(it)) }
                _uiState.value = _uiState.value.copy(
                    arena = reduction.state,
                    placementMode = if (leavePlacementMode) false else _uiState.value.placementMode,
                    canUndo = undoHistory.isNotEmpty(),
                    canRedo = false,
                    feedback = successMessage,
                    feedbackIsError = false,
                )
                persist()
            }
        }
    }

    private fun handleInbound(event: ArenaInboundEvent) {
        when (event) {
            is ArenaInboundEvent.Status -> {
                val history = (_uiState.value.statusHistory + event.text).takeLast(MAX_STATUS_HISTORY)
                _uiState.value = _uiState.value.copy(
                    status = event.text,
                    statusHistory = history,
                    feedback = "Status updated.",
                    feedbackIsError = false,
                )
                persist()
            }
            is ArenaInboundEvent.Target -> {
                val action = ArenaAction.ApplyTarget(event.obstacleId, event.targetId, event.face)
                applyReduction(reducer.reduce(_uiState.value.arena, action)) { state ->
                    undoHistory.clear()
                    redoHistory.clear()
                    _uiState.value = _uiState.value.copy(
                        arena = state,
                        canUndo = false,
                        canRedo = false,
                        feedback = "Target ${event.targetId} applied to obstacle ${event.obstacleId}.",
                        feedbackIsError = false,
                    )
                    persist()
                }
            }
            is ArenaInboundEvent.Robot -> {
                applyReduction(
                    reducer.reduce(_uiState.value.arena, ArenaAction.ApplyRobotPose(event.pose)),
                ) { state ->
                    _uiState.value = _uiState.value.copy(
                        arena = state,
                        feedback = "Robot pose updated.",
                        feedbackIsError = false,
                    )
                    persist()
                }
            }
        }
    }

    private inline fun applyReduction(
        reduction: ArenaReduction,
        onSuccess: (ArenaState) -> Unit,
    ) {
        when (reduction) {
            is ArenaReduction.Success -> onSuccess(reduction.state)
            is ArenaReduction.Failure -> setFeedback(reduction.reason, isError = true)
        }
    }

    private fun pushUndo(state: ArenaState) {
        if (undoHistory.size == MAX_UNDO_HISTORY) undoHistory.removeFirst()
        undoHistory.addLast(ArenaEditSnapshot.from(state))
        redoHistory.clear()
    }

    private fun updateAfterHistory(state: ArenaState, feedback: String) {
        _uiState.value = _uiState.value.copy(
            arena = state,
            placementMode = false,
            canUndo = undoHistory.isNotEmpty(),
            canRedo = redoHistory.isNotEmpty(),
            feedback = feedback,
            feedbackIsError = false,
        )
        persist()
    }

    private fun syncObstacleDiff(before: ArenaState, after: ArenaState) {
        (before.obstacles.keys - after.obstacles.keys).sorted().forEach {
            outboundSink.submit(codec.encode(ArenaOutboundEvent.RemoveObstacle(it)))
        }
        after.obstacles.toSortedMap().forEach { (id, obstacle) ->
            if (before.obstacles[id] != obstacle) {
                outboundSink.submit(codec.encode(ArenaOutboundEvent.UpsertObstacle(obstacle)))
            }
        }
    }

    private fun setFeedback(message: String, isError: Boolean) {
        _uiState.value = _uiState.value.copy(feedback = message, feedbackIsError = isError)
    }

    private fun restoreUiState(): ArenaUiState {
        val obstacles = savedStateHandle.get<ArrayList<String>>(KEY_OBSTACLES)
            .orEmpty()
            .mapNotNull(::decodeObstacle)
            .associateBy(Obstacle::id)
        val robot = savedStateHandle.get<String>(KEY_ROBOT)?.let(::decodeRobot)
        val nextId = savedStateHandle.get<Int>(KEY_NEXT_ID)
            ?.coerceAtLeast((obstacles.keys.maxOrNull() ?: 0) + 1)
            ?: ((obstacles.keys.maxOrNull() ?: 0) + 1)
        val selected = savedStateHandle.get<Int>(KEY_SELECTED)?.takeIf(obstacles::containsKey)
        val status = savedStateHandle.get<String>(KEY_STATUS).orEmpty().ifBlank { "Ready to start" }
        return ArenaUiState(
            arena = ArenaState(
                robot = robot,
                obstacles = obstacles,
                selectedObstacleId = selected,
                nextObstacleId = nextId,
            ),
            status = status,
            statusHistory = listOf(status),
        )
    }

    private fun persist() {
        val state = _uiState.value
        savedStateHandle[KEY_OBSTACLES] = ArrayList(
            state.arena.obstacles.toSortedMap().values.map(::encodeObstacle),
        )
        savedStateHandle[KEY_ROBOT] = state.arena.robot?.let(::encodeRobot)
        savedStateHandle[KEY_NEXT_ID] = state.arena.nextObstacleId
        savedStateHandle[KEY_SELECTED] = state.arena.selectedObstacleId
        savedStateHandle[KEY_STATUS] = state.status
    }

    private fun encodeObstacle(obstacle: Obstacle): String = listOf(
        obstacle.id,
        obstacle.position.x,
        obstacle.position.y,
        obstacle.targetFace?.wireValue.orEmpty(),
        obstacle.targetId.orEmpty().replace('|', '_'),
    ).joinToString("|")

    private fun decodeObstacle(encoded: String): Obstacle? {
        val parts = encoded.split('|', limit = 5)
        if (parts.size != 5) return null
        val id = parts[0].toIntOrNull()?.takeIf { it > 0 } ?: return null
        val x = parts[1].toIntOrNull() ?: return null
        val y = parts[2].toIntOrNull() ?: return null
        val position = GridCoordinate(x, y).takeIf(ArenaState().config::contains) ?: return null
        val face = parts[3].takeIf(String::isNotBlank)?.let(Direction::fromWire) ?: run {
            if (parts[3].isBlank()) null else return null
        }
        return Obstacle(id, position, face, parts[4].ifBlank { null })
    }

    private fun encodeRobot(robot: RobotPose): String =
        "${robot.position.x}|${robot.position.y}|${robot.direction.wireValue}"

    private fun decodeRobot(encoded: String): RobotPose? {
        val parts = encoded.split('|')
        if (parts.size != 3) return null
        val position = GridCoordinate(
            parts[0].toIntOrNull() ?: return null,
            parts[1].toIntOrNull() ?: return null,
        ).takeIf(ArenaState().config::contains) ?: return null
        return RobotPose(position, Direction.fromWire(parts[2]) ?: return null)
    }

    class Factory(
        private val outboundSink: ArenaOutboundSink,
    ) : ViewModelProvider.Factory {
        @Suppress("UNCHECKED_CAST")
        override fun <T : ViewModel> create(
            modelClass: Class<T>,
            extras: CreationExtras,
        ): T {
            require(modelClass.isAssignableFrom(ArenaViewModel::class.java))
            return ArenaViewModel(extras.createSavedStateHandle(), outboundSink) as T
        }
    }

    companion object {
        private const val MAX_STATUS_HISTORY = 20
        private const val MAX_UNDO_HISTORY = 50
        private const val KEY_OBSTACLES = "arena.obstacles"
        private const val KEY_ROBOT = "arena.robot"
        private const val KEY_NEXT_ID = "arena.nextId"
        private const val KEY_SELECTED = "arena.selected"
        private const val KEY_STATUS = "arena.status"
    }
}
