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
import com.mdp.g15.arena.domain.TargetId
import com.mdp.g15.arena.domain.ManualCommand
import com.mdp.g15.arena.domain.ManualDriveGuard
import com.mdp.g15.arena.integration.ArenaInboundConsumer
import com.mdp.g15.arena.integration.ArenaOutboundSink
import com.mdp.g15.arena.protocol.ArenaDecodeResult
import com.mdp.g15.arena.protocol.ArenaInboundEvent
import com.mdp.g15.arena.protocol.ArenaMessageCodec
import com.mdp.g15.arena.protocol.CsvArenaMessageCodec
import com.mdp.g15.arena.protocol.ObstacleSync
import java.util.ArrayDeque
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay

class ArenaViewModel(
    private val savedStateHandle: SavedStateHandle,
    private val outboundSink: ArenaOutboundSink,
    private val reducer: ArenaReducer = ArenaReducer(),
    private val codec: ArenaMessageCodec = CsvArenaMessageCodec(),
    private val useRpiMapSync: Boolean = false,
) : ViewModel(), ArenaInboundConsumer {
    private val undoHistory = ArrayDeque<ArenaEditSnapshot>()
    private val redoHistory = ArrayDeque<ArenaEditSnapshot>()

    private val _uiState = MutableStateFlow(restoreUiState())
    val uiState: StateFlow<ArenaUiState> = _uiState.asStateFlow()
    private val manualDrive = ManualDriveGuard()
    private var connected = false
    private var previewJob: Job? = null

    fun manualAllowed(command: ManualCommand): Boolean = connected && !_uiState.value.autonomousRunning &&
        !_uiState.value.manualAnimating && manualDrive.allowed(_uiState.value.arena, command)

    fun startRun(send: () -> Unit) {
        if (!connected || blockWhileDriving()) return
        manualDrive.invalidate()
        _uiState.value = _uiState.value.copy(autonomousRunning = true, manualStatus = "Autonomous run — manual driving paused")
        setFeedback("Run requested.", false)
        send()
    }

    fun rejectUnsafeCustomMessage() = setFeedback("Send one command at a time; movement must pass the arena checks.", true)

    fun drive(command: ManualCommand, send: () -> Unit) {
        if (!manualAllowed(command) || manualDrive.reserve(_uiState.value.arena, command) == null) {
            setFeedback("Movement blocked: wait for completion and a valid pose; check the swept path.", true)
            return
        }
        // Reserve synchronously before calling the transport, even if Compose has not recomposed.
        _uiState.value = _uiState.value.copy(
            manualPending = true,
            manualAnimating = true,
            manualStatus = "Moving (estimated) — awaiting completion",
            arena = _uiState.value.arena.copy(robot = manualDrive.pending!!.let { it.at(1.0).asRobotPose().copy(commandedPath = it) }),
        )
        setFeedback("Sent ${command.wire} — waiting for movement completion.", false)
        val previewDuration = requireNotNull(manualDrive.pending).previewDurationMillis
        previewJob = viewModelScope.launch {
            delay(previewDuration + 50L) // Allow the next rendered frame to finish the preview.
            _uiState.value = _uiState.value.copy(manualAnimating = false)
        }
        try { send() } catch (error: Exception) {
            cancelPreviewGate()
            manualDrive.invalidate()
            _uiState.value = _uiState.value.copy(manualPending = false, manualStatus = "Send failed — pose unconfirmed")
            setFeedback("Send failed; robot position needs confirmation.", true)
        }
    }

    fun stopManual(send: () -> Unit) {
        cancelPreviewGate()
        manualDrive.invalidate()
        _uiState.value = _uiState.value.copy(manualPending = false, autonomousRunning = false, manualStatus = "Stopped — pose unconfirmed")
        setFeedback("Stop requested — await a fresh robot position before driving.", false)
        if (connected) send()
    }
    private val obstacleSync = ObstacleSync(outboundSink::submit).also {
        it.update(_uiState.value.arena.obstacles)
    }

    fun connectionChanged(connected: Boolean) {
        if (this.connected != connected) {
            cancelPreviewGate()
            manualDrive.invalidate()
            _uiState.value = _uiState.value.copy(manualPending = false, autonomousRunning = false,
                manualStatus = if (connected) "Awaiting fresh robot position" else "Disconnected — pose unconfirmed")
        }
        this.connected = connected
        if (!useRpiMapSync) return
        obstacleSync.connectionChanged(connected)
        updateSyncStatus()
    }

    private fun updateSyncStatus() {
        _uiState.value = _uiState.value.copy(
            obstacleSyncStatus = obstacleSync.status,
        )
    }

    override fun accept(message: String) {
        viewModelScope.launch {
            // Transport normally frames lines; this also accepts batched test-tool input.
            message.lineSequence().map(String::trim).filter(String::isNotEmpty).forEach { line ->
                _uiState.value = _uiState.value.copy(latestReceived = line)
                when (val decoded = codec.decode(line)) {
                    is ArenaDecodeResult.Decoded -> {
                        setFeedback("Received: $line", isError = false)
                        handleInbound(decoded.event, line)
                    }
                    is ArenaDecodeResult.Malformed -> {
                        if (line.substringBefore(',').trim().equals("ROBOT", true)) invalidateReportedPose()
                        setFeedback(decoded.reason, isError = true)
                    }
                    ArenaDecodeResult.Ignored -> setFeedback("Received: $line", isError = false)
                }
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

    fun moveRobot(destination: GridCoordinate) = dispatchLocal(
        ArenaAction.MoveRobot(destination),
        "Robot moved to (${destination.x}, ${destination.y}).",
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
        if (blockWhileDriving()) return
        manualDrive.invalidate()
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
        if (!useRpiMapSync) before.obstacles.keys.sorted().forEach {
            outboundSink.submit(codec.encode(ArenaOutboundEvent.RemoveObstacle(it)))
        }
        persist()
    }

    fun undo() {
        if (blockWhileDriving()) return
        manualDrive.invalidate()
        if (undoHistory.isEmpty()) return
        val before = _uiState.value.arena
        val target = undoHistory.removeLast()
        redoHistory.addLast(ArenaEditSnapshot.from(before))
        val restored = target.applyTo(before)
        syncObstacleDiff(before, restored)
        updateAfterHistory(restored, "Last arena edit undone.")
    }

    fun redo() {
        if (blockWhileDriving()) return
        manualDrive.invalidate()
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
        if (blockWhileDriving()) return
        if (action is ArenaAction.MoveRobot) manualDrive.invalidate()
        val before = _uiState.value.arena
        when (val reduction = reducer.reduce(before, action)) {
            is ArenaReduction.Failure -> setFeedback(reduction.reason, isError = true)
            is ArenaReduction.Success -> {
                if (reduction.state != before) pushUndo(before)
                if (!useRpiMapSync) reduction.outboundEvent?.let { outboundSink.submit(codec.encode(it)) }
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

    private fun blockWhileDriving(): Boolean {
        if (manualDrive.pending == null && !_uiState.value.autonomousRunning && !_uiState.value.manualAnimating) return false
        setFeedback("Wait for movement completion before editing the arena.", true)
        return true
    }

    private fun cancelPreviewGate() {
        previewJob?.cancel()
        previewJob = null
        _uiState.value = _uiState.value.copy(manualAnimating = false)
    }

    private fun handleInbound(event: ArenaInboundEvent, rawMessage: String) {
        when (event) {
            is ArenaInboundEvent.Status -> {
                val wasPending = manualDrive.pending != null
                val measuredCompletion = manualDrive.reportedDuringMove != null
                val autonomous = when (event.text.substringBefore(',')) {
                    "START", "RUNNING" -> true
                    "DONE", "FAILED" -> false
                    else -> _uiState.value.autonomousRunning
                }
                when (event.text.trim().uppercase()) {
                    "OK" -> if (rawMessage == "STATUS,OK") manualDrive.complete()
                    "FAILED", "ERROR", "STOPPED" -> manualDrive.invalidate()
                }
                val history = (_uiState.value.statusHistory + event.text).takeLast(MAX_STATUS_HISTORY)
                _uiState.value = _uiState.value.copy(
                    status = event.text,
                    statusHistory = history,
                    manualPending = manualDrive.pending != null,
                    autonomousRunning = autonomous,
                    manualStatus = when (event.text.trim().uppercase()) {
                        "OK" -> if (wasPending && manualDrive.pending == null) {
                            if (measuredCompletion) "Ready — reported pose" else "Ready — estimated pose"
                        } else _uiState.value.manualStatus
                        "FAILED" -> "Movement failed — pose unconfirmed"
                        "DONE" -> null
                        else -> if (autonomous) "Autonomous run — manual driving paused" else _uiState.value.manualStatus
                    },
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
                    persist(transmitMap = false) // Received data is not a new Android edit.
                }
            }
            is ArenaInboundEvent.Robot -> {
                val reduction = reducer.reduce(_uiState.value.arena, ArenaAction.ApplyRobotPose(event.pose))
                if (reduction is ArenaReduction.Failure) invalidateReportedPose()
                applyReduction(
                    reduction,
                ) { state ->
                    val accepted = manualDrive.report(event.pose)
                    _uiState.value = _uiState.value.copy(
                        arena = if (accepted) state else _uiState.value.arena,
                        manualStatus = when {
                            _uiState.value.autonomousRunning -> _uiState.value.manualStatus
                            manualDrive.pending == null -> null
                            accepted -> "Moving — reported pose; awaiting completion"
                            else -> _uiState.value.manualStatus
                        },
                    )
                    persist()
                }
            }
        }
    }

    private fun invalidateReportedPose() {
        // Bad telemetry must not leave driving enabled against an older, apparently safe pose.
        // Keep any outstanding command reserved: a parse error does not stop physical movement.
        manualDrive.flagUncertain()
        _uiState.value = _uiState.value.copy(manualStatus = "Invalid robot position — driving paused")
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
        if (useRpiMapSync) return // persist() sends the full map, including undo/redo/reset.
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
        val selected = savedStateHandle.get<Int>(KEY_SELECTED)?.takeIf(obstacles::containsKey)
        val status = savedStateHandle.get<String>(KEY_STATUS).orEmpty().ifBlank { "Awaiting robot status" }
        return ArenaUiState(
            arena = ArenaState(
                robot = robot,
                obstacles = obstacles,
                selectedObstacleId = selected,
            ),
            status = status,
            statusHistory = listOf(status),
        )
    }

    private fun persist(transmitMap: Boolean = true) {
        val state = _uiState.value
        if (useRpiMapSync) {
            obstacleSync.update(state.arena.obstacles, transmit = transmitMap)
            updateSyncStatus()
        }
        savedStateHandle[KEY_OBSTACLES] = ArrayList(
            state.arena.obstacles.toSortedMap().values.map(::encodeObstacle),
        )
        savedStateHandle[KEY_ROBOT] = state.arena.robot?.takeIf { it.estimate == null }?.let(::encodeRobot)
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
        return Obstacle(id, position, face, parts[4].takeIf { TargetId.error(it) == null })
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
        private val useRpiMapSync: Boolean = false,
    ) : ViewModelProvider.Factory {
        @Suppress("UNCHECKED_CAST")
        override fun <T : ViewModel> create(
            modelClass: Class<T>,
            extras: CreationExtras,
        ): T {
            require(modelClass.isAssignableFrom(ArenaViewModel::class.java))
            return ArenaViewModel(extras.createSavedStateHandle(), outboundSink, useRpiMapSync = useRpiMapSync) as T
        }
    }

    companion object {
        private const val MAX_STATUS_HISTORY = 20
        private const val MAX_UNDO_HISTORY = 50
        private const val KEY_OBSTACLES = "arena.obstacles"
        private const val KEY_ROBOT = "arena.robot"
        private const val KEY_SELECTED = "arena.selected"
        private const val KEY_STATUS = "arena.status"
    }
}
