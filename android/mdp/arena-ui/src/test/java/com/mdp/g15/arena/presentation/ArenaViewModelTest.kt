package com.mdp.g15.arena.presentation

import androidx.lifecycle.SavedStateHandle
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.integration.ArenaOutboundSink
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class ArenaViewModelTest {
    private val dispatcher = StandardTestDispatcher()
    private lateinit var sent: MutableList<String>
    private lateinit var viewModel: ArenaViewModel

    @Before
    fun setUp() {
        Dispatchers.setMain(dispatcher)
        sent = mutableListOf()
        viewModel = ArenaViewModel(SavedStateHandle(), ArenaOutboundSink(sent::add))
    }

    @After
    fun tearDown() {
        Dispatchers.resetMain()
    }

    @Test
    fun `local placement and removal emit exactly one message each`() {
        viewModel.addObstacle(GridCoordinate(10, 6))
        assertEquals(listOf("OBSTACLE,UPSERT,1,10,6,"), sent)

        viewModel.removeObstacle(1)
        assertEquals("OBSTACLE,REMOVE,1", sent.last())
        assertEquals(2, sent.size)
    }

    @Test
    fun `selective status ignores unrelated message`() = runTest(dispatcher) {
        viewModel.accept("MOVE,10,FORWARD")
        advanceUntilIdle()
        assertEquals("Awaiting robot status", viewModel.uiState.value.status)

        viewModel.accept("STATUS,Looking for target 2")
        advanceUntilIdle()
        assertEquals("Looking for target 2", viewModel.uiState.value.status)
    }

    @Test
    fun `routine status and robot telemetry preserve latest editing feedback`() = runTest(dispatcher) {
        viewModel.setPlacementMode(true)
        val feedback = viewModel.uiState.value.feedback

        viewModel.accept("STATUS,Moving")
        viewModel.accept("ROBOT,7,2,W")
        advanceUntilIdle()

        assertEquals("Moving", viewModel.uiState.value.status)
        assertEquals(feedback, viewModel.uiState.value.feedback)
        assertEquals(GridCoordinate(7, 2), viewModel.uiState.value.arena.robot?.position)
    }

    @Test
    fun `target and robot messages update render state without outbound traffic`() = runTest(dispatcher) {
        viewModel.addObstacle(GridCoordinate(1, 1))
        sent.clear()

        viewModel.accept("TARGET,1,11,N")
        viewModel.accept("ROBOT,7,2,W")
        advanceUntilIdle()

        val state = viewModel.uiState.value.arena
        assertEquals("11", state.obstacles.getValue(1).targetId)
        assertEquals(Direction.NORTH, state.obstacles.getValue(1).targetFace)
        assertEquals(GridCoordinate(7, 2), state.robot?.position)
        assertEquals(Direction.WEST, state.robot?.direction)
        assertTrue(sent.isEmpty())
    }

    @Test
    fun `undo and redo synchronize obstacle diff through sink`() {
        viewModel.addObstacle(GridCoordinate(3, 4))
        sent.clear()

        viewModel.undo()
        assertFalse(viewModel.uiState.value.arena.obstacles.containsKey(1))
        assertEquals(listOf("OBSTACLE,REMOVE,1"), sent)

        sent.clear()
        viewModel.redo()
        assertTrue(viewModel.uiState.value.arena.obstacles.containsKey(1))
        assertEquals(listOf("OBSTACLE,UPSERT,1,3,4,"), sent)
    }

    @Test
    fun `RPi mode resends full map for every local edit reset and history operation`() = runTest(dispatcher) {
        val handle = SavedStateHandle()
        viewModel = ArenaViewModel(handle, ArenaOutboundSink(sent::add), useRpiMapSync = true)
        viewModel.addObstacle(GridCoordinate(3, 4))
        assertTrue(sent.isEmpty())
        viewModel.connectionChanged(true)
        assertEquals("CLEAR\nOBSTACLE,1,30,40,SKIP", sent.last())
        viewModel.moveObstacle(1, GridCoordinate(5, 6))
        assertEquals("CLEAR\nOBSTACLE,1,50,60,SKIP", sent.last())
        viewModel.selectObstacle(1)
        viewModel.setTargetFace(Direction.SOUTH)
        assertEquals("CLEAR\nOBSTACLE,1,50,60,SOUTH", sent.last())
        viewModel.addObstacle(GridCoordinate(7, 8))
        viewModel.removeObstacle(1)
        assertEquals("CLEAR\nOBSTACLE,2,70,80,SKIP", sent.last())
        viewModel.undo()
        assertEquals("CLEAR\nOBSTACLE,1,50,60,SOUTH\nOBSTACLE,2,70,80,SKIP", sent.last())
        viewModel.redo()
        assertEquals("CLEAR\nOBSTACLE,2,70,80,SKIP", sent.last())
        viewModel.resetArena()
        assertEquals("CLEAR", sent.last())
        viewModel.undo()
        val beforeReports = sent.size
        viewModel.accept("TARGET,2,11,W")
        viewModel.accept("ROBOT,0,0,N")
        advanceUntilIdle()
        assertEquals(beforeReports, sent.size)
        viewModel.connectionChanged(false)
        viewModel.moveObstacle(2, GridCoordinate(8, 9))
        assertEquals(beforeReports, sent.size)
        val restored = ArenaViewModel(handle, ArenaOutboundSink(sent::add), useRpiMapSync = true)
        restored.connectionChanged(true)
        assertEquals("CLEAR\nOBSTACLE,2,80,90,WEST", sent.last())
    }
}
