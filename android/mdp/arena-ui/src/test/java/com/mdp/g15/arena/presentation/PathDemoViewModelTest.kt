package com.mdp.g15.arena.presentation

import androidx.lifecycle.SavedStateHandle
import com.mdp.g15.arena.domain.PoseSource
import com.mdp.g15.arena.integration.ArenaOutboundSink
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class PathDemoViewModelTest {
    private val dispatcher = StandardTestDispatcher()
    private lateinit var sent: MutableList<String>
    private lateinit var viewModel: ArenaViewModel

    @Before
    fun setUp() {
        Dispatchers.setMain(dispatcher)
        sent = mutableListOf()
        viewModel = ArenaViewModel(SavedStateHandle(), ArenaOutboundSink(sent::add),
            pathDemoClockMillis = { dispatcher.scheduler.currentTime })
    }

    @After
    fun tearDown() {
        Dispatchers.resetMain()
    }

    @Test fun `demo plays all moves on the one robot and sends nothing`() = runTest(dispatcher) {
        viewModel.playPathDemo()
        val first = viewModel.uiState.value
        assertTrue(first.pathDemoRunning)
        assertEquals(0.0, first.arena.robot!!.y, 1e-9)
        assertEquals(PoseSource.SIMULATED, first.arena.robot!!.source)
        val move1Cells = first.pathOverlay!!.cells

        advanceTimeBy(4_000L); runCurrent()
        val during2 = viewModel.uiState.value
        assertNotEquals(move1Cells, during2.pathOverlay!!.cells)

        advanceTimeBy(12_100L); runCurrent()
        val done = viewModel.uiState.value
        assertFalse(done.pathDemoRunning)
        assertNull(done.pathOverlay)
        assertEquals(3.0, done.arena.robot!!.x, 1e-9)
        assertEquals(10.5, done.arena.robot!!.y, 1e-9)
        assertEquals(90.0, done.arena.robot!!.heading, 1e-9)
        assertTrue(sent.isEmpty())
    }

    @Test fun `a second press while playing does not restart`() = runTest(dispatcher) {
        viewModel.playPathDemo()
        advanceTimeBy(1_000L); runCurrent()
        val y = viewModel.uiState.value.arena.robot!!.y
        viewModel.playPathDemo()
        assertEquals(y, viewModel.uiState.value.arena.robot!!.y, 1e-9)
        assertTrue(viewModel.uiState.value.feedbackIsError)
    }

    @Test fun `a real pose report ends the demo`() = runTest(dispatcher) {
        viewModel.playPathDemo()
        advanceTimeBy(1_000L); runCurrent()
        viewModel.accept("ROBOT,8,8,N")
        runCurrent()
        val state = viewModel.uiState.value
        assertFalse(state.pathDemoRunning)
        assertNull(state.pathOverlay)
        assertEquals(PoseSource.REPORTED, state.arena.robot!!.source)
        advanceUntilIdle()
        assertEquals(8.0, viewModel.uiState.value.arena.robot!!.x, 1e-9)
    }

    @Test fun `arena edits are blocked while the demo plays`() = runTest(dispatcher) {
        viewModel.playPathDemo()
        viewModel.setPlacementMode(true)
        assertFalse(viewModel.uiState.value.placementMode)
        assertNotNull(viewModel.uiState.value.pathOverlay)
    }
}
