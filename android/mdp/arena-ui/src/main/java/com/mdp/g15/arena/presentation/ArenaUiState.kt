package com.mdp.g15.arena.presentation

import com.mdp.g15.arena.domain.ArenaState
import com.mdp.g15.arena.domain.PathOverlay

enum class TaskMode { TASK1, TASK2 }

enum class Task1Phase(val label: String) {
    SETUP("Setup"),
    PATH_REQUESTED("Path requested — readiness unconfirmed"),
    START_REQUESTED("Start requested — awaiting robot"),
    RUNNING("Running"),
    UNKNOWN("Connection interrupted — run status unknown"),
    COMPLETED("Completed — results retained"),
    FAILED("Run ended with a failure — results retained"),
    STOPPED("Stop requested — results retained"),
}

const val DEFAULT_PLANNER_INFO = "Planning confirmation unavailable — Start can proceed once setup is complete"

data class ArenaUiState(
    val arena: ArenaState = ArenaState(),
    val status: String = "Awaiting robot status",
    val statusHistory: List<String> = listOf("Awaiting robot status"),
    val feedback: String = "Ready",
    val feedbackIsError: Boolean = false,
    val latestReceived: String = "",
    val placementMode: Boolean = false,
    val canUndo: Boolean = false,
    val canRedo: Boolean = false,
    val amdToolMode: Boolean = false,
    val manualPending: Boolean = false,
    val manualAnimating: Boolean = false,
    val manualStatus: String? = null,
    val lastPoseReceivedAtMillis: Long? = null,
    val autonomousRunning: Boolean = false,
    val plannerInfo: String = DEFAULT_PLANNER_INFO,
    val task1Phase: Task1Phase = Task1Phase.SETUP,
    val taskMode: TaskMode = TaskMode.TASK1,
    val task2Ready: Boolean = false,
    val task2Timer: Task2Timer = Task2Timer(),
    val obstacleSyncStatus: String = "Map offline — edits saved locally",
    /** One-shot: the operator has explicitly asked the next reconnect to resend CLEAR + the
     *  full obstacle list, overriding the normal post-BEGIN suppression. See
     *  [ArenaViewModel.armMapResyncOnReconnect]. */
    val forceMapResyncOnReconnect: Boolean = false,
    /** Local "Play Path Demo" playback; it drives [ArenaState.robot] but sends nothing. */
    val pathDemoRunning: Boolean = false,
    /** Swept cells and travel arrow of the demo's active move only; null between runs. */
    val pathOverlay: PathOverlay? = null,
)
