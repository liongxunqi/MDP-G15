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
        assertEquals("Ready to start", viewModel.uiState.value.status)

        viewModel.accept("STATUS,Looking for target 2")
        advanceUntilIdle()
        assertEquals("Looking for target 2", viewModel.uiState.value.status)
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
}
