package com.mdp.g15.arena.presentation

import com.mdp.g15.arena.domain.ArenaState

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
    val autonomousRunning: Boolean = false,
    /** True from a Start tap until STATUS,ACK/START/RUNNING/DONE/FAILED resolves it. Distinct
     *  from [autonomousRunning]: a tap alone must never claim the robot is moving. */
    val startPending: Boolean = false,
    val plannerInfo: String = DEFAULT_PLANNER_INFO,
    val task1Phase: Task1Phase = Task1Phase.SETUP,
    val obstacleSyncStatus: String = "Map offline — edits saved locally",
)
