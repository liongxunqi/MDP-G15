package com.mdp.g15.arena.presentation

import com.mdp.g15.arena.domain.ArenaState

data class ArenaUiState(
    val arena: ArenaState = ArenaState(),
    val status: String = "Ready to start",
    val statusHistory: List<String> = listOf("Ready to start"),
    val feedback: String = "Ready",
    val feedbackIsError: Boolean = false,
    val placementMode: Boolean = false,
    val canUndo: Boolean = false,
    val canRedo: Boolean = false,
)
