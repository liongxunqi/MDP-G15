package com.mdp.g15.arena.presentation

import com.mdp.g15.arena.domain.ArenaState

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
    val freeMoveMode: Boolean = false,
    val amdToolMode: Boolean = false,
    val manualPending: Boolean = false,
    val manualAnimating: Boolean = false,
    val manualStatus: String? = null,
    val autonomousRunning: Boolean = false,
    val obstacleSyncStatus: String = "Map offline — edits saved locally",
)
