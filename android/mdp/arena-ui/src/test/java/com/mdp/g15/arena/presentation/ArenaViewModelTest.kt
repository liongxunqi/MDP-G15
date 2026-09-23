package com.mdp.g15.arena.presentation

import androidx.lifecycle.SavedStateHandle
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.domain.ManualCommand
import com.mdp.g15.arena.integration.ArenaOutboundSink
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.advanceTimeBy
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
    @Test fun `recycled IDs survive legacy restoration undo redo and new obstacle identities`() = runTest(dispatcher) {
        val handle = SavedStateHandle(mapOf(
            "arena.nextId" to 999,
            "arena.obstacles" to arrayListOf("1|0|0|N|11", "3|4|4|E|ABC"),
        ))
        val vm = ArenaViewModel(handle, ArenaOutboundSink(sent::add), useRpiMapSync = true)
        assertEquals(null, vm.uiState.value.arena.obstacles[3]!!.targetId)
        vm.addObstacle(GridCoordinate(8,8))
        assertEquals(setOf(1,2,3), vm.uiState.value.arena.obstacles.keys)
        vm.undo()
        assertEquals(setOf(1,3), vm.uiState.value.arena.obstacles.keys)
        vm.redo()
        assertEquals(setOf(1,2,3), vm.uiState.value.arena.obstacles.keys)
        vm.removeObstacle(1)
        vm.addObstacle(GridCoordinate(6,6))
        val recycled = vm.uiState.value.arena.obstacles.getValue(1)
        assertEquals(null, recycled.targetId)
        assertEquals(null, recycled.targetFace)
        vm.accept("TARGET,B1,A9,W")
        advanceUntilIdle()
        assertEquals("A9", vm.uiState.value.arena.obstacles.getValue(1).targetId)
        val restored = ArenaViewModel(handle, ArenaOutboundSink(sent::add), useRpiMapSync = true)
        assertEquals(vm.uiState.value.arena.obstacles, restored.uiState.value.arena.obstacles)
        restored.connectionChanged(true)
        assertTrue(sent.last().contains("OBSTACLE,1,60,60,WEST"))
    }

    @Test fun `recycled obstacles still block manual paths and invalid targets preserve valid detection`() = runTest(dispatcher) {
        viewModel.addObstacle(GridCoordinate(1,1))
        viewModel.removeObstacle(1)
        viewModel.addObstacle(GridCoordinate(8,10))
        viewModel.connectionChanged(true)
        viewModel.accept("ROBOT,8,8,N")
        viewModel.accept("TARGET,1,11,N")
        advanceUntilIdle()
        assertFalse(viewModel.manualAllowed(ManualCommand.FORWARD))
        viewModel.accept("TARGET,1,-1,N")
        advanceUntilIdle()
        assertEquals("11", viewModel.uiState.value.arena.obstacles.getValue(1).targetId)
        assertTrue(viewModel.uiState.value.feedbackIsError)
        viewModel.accept("TARGET,2,11")
        advanceUntilIdle()
        assertEquals("Target references unknown obstacle 2.", viewModel.uiState.value.feedback)
    }
    @Test fun `fast acknowledgement waits for preview and preview alone never releases a command`() = runTest(dispatcher) {
        viewModel.connectionChanged(true)
        viewModel.accept("ROBOT,8,8,N")
        runCurrent()
        var sends = 0
        viewModel.drive(ManualCommand.FORWARD) { sends++ }
        viewModel.accept("MSG,[OK]")
        runCurrent()
        assertTrue(viewModel.uiState.value.manualPending)
        viewModel.accept("STATUS,OK")
        runCurrent()
        assertFalse(viewModel.uiState.value.manualPending)
        assertTrue(viewModel.uiState.value.manualAnimating)
        viewModel.drive(ManualCommand.FORWARD) { sends++ }
        assertEquals(1, sends)
        advanceTimeBy(701)
        runCurrent()
        assertTrue(viewModel.manualAllowed(ManualCommand.FORWARD))
        viewModel.drive(ManualCommand.FORWARD) { sends++ }
        advanceUntilIdle()
        assertEquals(2, sends)
        assertTrue(viewModel.uiState.value.manualPending)
        assertFalse(viewModel.uiState.value.manualAnimating)
        assertFalse(viewModel.manualAllowed(ManualCommand.FORWARD))
    }
    @Test fun `bad robot telemetry locks driving while preserving last visible pose`() = runTest(dispatcher) {
        viewModel.connectionChanged(true)
        for (bad in listOf("ROBOT,8,9", "ROBOT,19,19,N")) {
            viewModel.accept("ROBOT,8,8,N")
            advanceUntilIdle()
            assertTrue(viewModel.manualAllowed(ManualCommand.FORWARD))
            viewModel.accept(bad)
            advanceUntilIdle()
            assertFalse(viewModel.manualAllowed(ManualCommand.FORWARD))
            assertEquals(GridCoordinate(8,8), viewModel.uiState.value.arena.robot!!.position)
        }
    }
    @Test fun `genuine telemetry during a move corrects the estimate without releasing the lock`() = runTest(dispatcher) {
        viewModel.connectionChanged(true)
        viewModel.accept("ROBOT,8,8,N")
        advanceUntilIdle()
        viewModel.drive(ManualCommand.FORWARD) {}
        viewModel.accept("ROBOT,8,9,N")
        advanceUntilIdle()
        assertEquals(GridCoordinate(8,9), viewModel.uiState.value.arena.robot!!.position)
        assertEquals(null, viewModel.uiState.value.arena.robot!!.estimate)
        assertFalse(viewModel.manualAllowed(ManualCommand.FORWARD))
        viewModel.accept("STATUS,OK")
        advanceUntilIdle()
        viewModel.drive(ManualCommand.FORWARD) {}
        assertEquals(11.0, viewModel.uiState.value.arena.robot!!.estimate!!.y, 0.0001)
    }

    @Test fun `manual and autonomous commands cannot interleave`() = runTest(dispatcher) {
        viewModel.connectionChanged(true)
        viewModel.accept("ROBOT,8,8,N")
        advanceUntilIdle()
        var runs = 0
        viewModel.drive(ManualCommand.FORWARD) {}
        viewModel.startRun { runs++ }
        assertEquals(0, runs)
        viewModel.accept("STATUS,OK")
        advanceUntilIdle()
        viewModel.startRun { runs++ }
        assertEquals(1, runs)
        viewModel.accept("ROBOT,8,10,N")
        advanceUntilIdle()
        assertFalse(viewModel.manualAllowed(ManualCommand.FORWARD))
        viewModel.resetArena()
        assertEquals(GridCoordinate(8,10), viewModel.uiState.value.arena.robot!!.position)
        viewModel.accept("STATUS,DONE")
        advanceUntilIdle()
        assertTrue(viewModel.manualAllowed(ManualCommand.FORWARD))
    }
    @Test fun `raw latest received survives malformed parsing and a valid message clears the error`() = runTest(dispatcher) {
        viewModel.accept("ROBOT,4,5")
        advanceUntilIdle()
        assertTrue(viewModel.uiState.value.feedbackIsError)
        assertEquals("ROBOT,4,5", viewModel.uiState.value.latestReceived)
        assertEquals(null, viewModel.uiState.value.arena.robot)
        viewModel.accept("ROBOT,4,5,N\r\nSTATUS,DONE\n")
        advanceUntilIdle()
        assertFalse(viewModel.uiState.value.feedbackIsError)
        assertEquals("STATUS,DONE", viewModel.uiState.value.latestReceived)
        assertEquals("Received: STATUS,DONE", viewModel.uiState.value.feedback)
        assertEquals(GridCoordinate(4,5), viewModel.uiState.value.arena.robot!!.position)
        viewModel.accept("debug reply")
        advanceUntilIdle()
        assertEquals("debug reply", viewModel.uiState.value.latestReceived)
        assertEquals("DONE", viewModel.uiState.value.status)
    }
    @Test fun `manual movement reserves before send and cannot queue or edit while pending`() = runTest(dispatcher) {
        viewModel.connectionChanged(true)
        viewModel.accept("ROBOT,8,16,N")
        advanceUntilIdle()
        var transmissions = 0
        repeat(100) { viewModel.drive(ManualCommand.FORWARD) { transmissions++ } }
        assertEquals(1, transmissions)
        assertEquals(18.0, viewModel.uiState.value.arena.robot!!.estimate!!.y, 0.0001)
        viewModel.accept("ROBOT,8,16,N")
        advanceUntilIdle()
        assertEquals(18.0, viewModel.uiState.value.arena.robot!!.estimate!!.y, 0.0001)
        viewModel.resetArena()
        assertTrue(viewModel.uiState.value.manualPending)
        viewModel.accept("STATUS,OK")
        advanceUntilIdle()
        assertFalse(viewModel.uiState.value.manualPending)
        assertTrue(viewModel.manualAllowed(ManualCommand.REVERSE))
        viewModel.drive(ManualCommand.FORWARD) { transmissions++ }
        assertEquals(1, transmissions)
        viewModel.connectionChanged(false)
        viewModel.drive(ManualCommand.REVERSE) { transmissions++ }
        assertEquals(1, transmissions)
    }

    @Test fun `failed movement and stop keep controls locked until a fresh pose`() = runTest(dispatcher) {
        viewModel.connectionChanged(true)
        viewModel.accept("ROBOT,8,8,N")
        advanceUntilIdle()
        viewModel.drive(ManualCommand.FORWARD) {}
        viewModel.accept("STATUS,FAILED")
        advanceUntilIdle()
        assertFalse(viewModel.manualAllowed(ManualCommand.FORWARD))
        viewModel.accept("STATUS,OK")
        advanceUntilIdle()
        assertFalse(viewModel.manualAllowed(ManualCommand.FORWARD))
        viewModel.accept("ROBOT,8,9,N")
        advanceUntilIdle()
        assertTrue(viewModel.manualAllowed(ManualCommand.FORWARD))
        assertEquals(null, viewModel.uiState.value.arena.robot!!.estimate)
        viewModel.stopManual {}
        assertFalse(viewModel.manualAllowed(ManualCommand.FORWARD))
    }
    @Test fun `rapid spawn presses stop at cap and preserve history on rejected press`() {
        repeat(100) { viewModel.spawnObstacle() }
        assertEquals(50, viewModel.uiState.value.arena.obstacles.size)
        assertEquals(50, sent.size)
        assertFalse(viewModel.uiState.value.placementMode)
        assertTrue(viewModel.uiState.value.feedbackIsError)
        viewModel.undo()
        assertEquals(49, viewModel.uiState.value.arena.obstacles.size)
        viewModel.redo()
        assertEquals(50, viewModel.uiState.value.arena.obstacles.size)
        viewModel.removeObstacle(17)
        viewModel.spawnObstacle()
        assertEquals(17, viewModel.uiState.value.arena.selectedObstacleId)
        assertFalse(viewModel.uiState.value.canRedo)
    }

    @Test fun `automatic additions persist and sync edits and reconnect without extra taps`() {
        val handle = SavedStateHandle()
        val vm = ArenaViewModel(handle, ArenaOutboundSink(sent::add), useRpiMapSync = true)
        repeat(3) { vm.spawnObstacle() }
        assertTrue(sent.isEmpty())
        vm.connectionChanged(true)
        assertEquals("CLEAR\nOBSTACLE,1,0,0,SKIP\nOBSTACLE,2,10,0,SKIP\nOBSTACLE,3,20,0,SKIP", sent.last())
        vm.moveObstacle(3, GridCoordinate(5, 5))
        vm.setTargetFace(Direction.WEST)
        assertTrue(sent.last().endsWith("OBSTACLE,3,50,50,WEST"))
        val restored = ArenaViewModel(handle, ArenaOutboundSink(sent::add), useRpiMapSync = true)
        assertEquals(vm.uiState.value.arena.obstacles, restored.uiState.value.arena.obstacles)
        restored.connectionChanged(true)
        assertTrue(sent.last().endsWith("OBSTACLE,3,50,50,WEST"))
    }

    @Test fun `spawn cannot bypass manual pending preview or autonomous locks`() = runTest(dispatcher) {
        viewModel.connectionChanged(true)
        viewModel.accept("ROBOT,8,8,N")
        runCurrent()
        viewModel.drive(ManualCommand.FORWARD) {}
        viewModel.spawnObstacle()
        assertTrue(viewModel.uiState.value.arena.obstacles.isEmpty())
        viewModel.accept("STATUS,OK")
        runCurrent()
        viewModel.spawnObstacle()
        assertTrue(viewModel.uiState.value.arena.obstacles.isEmpty())
        advanceUntilIdle()
        viewModel.startRun {}
        viewModel.spawnObstacle()
        assertTrue(viewModel.uiState.value.arena.obstacles.isEmpty())
        viewModel.accept("STATUS,DONE")
        advanceUntilIdle()
        viewModel.spawnObstacle()
        assertEquals(1, viewModel.uiState.value.arena.obstacles.size)
    }

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

        viewModel.accept("STATUS,RUNNING,2,5")
        advanceUntilIdle()
        assertEquals("RUNNING,2,5", viewModel.uiState.value.status)
    }

    @Test
    fun `malformed status is surfaced and not acted on`() = runTest(dispatcher) {
        viewModel.accept("STATUS,DONE")
        advanceUntilIdle()

        viewModel.accept("STATUS,START,1,2,X")
        advanceUntilIdle()

        assertEquals("DONE", viewModel.uiState.value.status)
        assertEquals("Malformed status received", viewModel.uiState.value.feedback)
        assertTrue(viewModel.uiState.value.feedbackIsError)
    }

    @Test
    fun `routine status and robot telemetry refresh latest action without cancelling editing`() = runTest(dispatcher) {
        viewModel.setPlacementMode(true)

        viewModel.accept("STATUS,RUNNING,1,4")
        viewModel.accept("ROBOT,7,2,W")
        advanceUntilIdle()

        assertEquals("RUNNING,1,4", viewModel.uiState.value.status)
        assertEquals("Received: ROBOT,7,2,W", viewModel.uiState.value.feedback)
        assertTrue(viewModel.uiState.value.placementMode)
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
