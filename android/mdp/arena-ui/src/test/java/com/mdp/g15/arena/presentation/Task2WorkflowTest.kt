package com.mdp.g15.arena.presentation

import androidx.lifecycle.SavedStateHandle
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.integration.ArenaOutboundSink
import com.mdp.g15.arena.integration.CommandQueueRejected
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.After
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class Task2WorkflowTest {
    private val dispatcher = StandardTestDispatcher()
    private val handle = SavedStateHandle()
    private val outbound = mutableListOf<String>()
    private lateinit var vm: ArenaViewModel

    @Before fun setup() {
        Dispatchers.setMain(dispatcher)
        vm = ArenaViewModel(handle, ArenaOutboundSink(outbound::add), useRpiMapSync = true)
    }

    @After fun teardown() = Dispatchers.resetMain()

    @Test fun `empty task2 arena starts with one begin and no path`() = runTest(dispatcher) {
        vm.selectTask(TaskMode.TASK2)
        assertFalse(vm.startRun { fail("offline BEGIN") })
        vm.connectionChanged(true)
        assertEquals(listOf("TASK2,STATUS?"), outbound)
        assertFalse(vm.canRequestTask2())
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        assertTrue(vm.canRequestTask2())
        assertTrue(vm.startRun { outbound.add("BEGIN") })
        assertFalse(vm.startRun { outbound.add("BEGIN") })
        assertFalse(vm.preparePath { outbound.add("PATH") })
        assertEquals(listOf("TASK2,STATUS?", "BEGIN"), outbound)
        vm.accept("STATUS,RUNNING,0,5\nSTATUS,RUNNING,1,5\nSTATUS,DONE")
        advanceUntilIdle()
        assertEquals(Task1Phase.COMPLETED, vm.uiState.value.task1Phase)
        assertFalse(vm.uiState.value.autonomousRunning)
        assertFalse(vm.canRequestTask2())
    }

    @Test fun `task2 ignores saved task1 markers and restores them when switching back`() = runTest(dispatcher) {
        vm.addObstacle(GridCoordinate(5, 5))
        vm.setTargetFace(Direction.NORTH)
        vm.connectionChanged(true)
        assertTrue(outbound.last().contains("OBSTACLE,1,50,50,NORTH"))
        val beforeTask2 = outbound.size
        vm.selectTask(TaskMode.TASK2)
        assertEquals(listOf("TASK2,STATUS?"), outbound.drop(beforeTask2))
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        assertTrue(vm.canRequestTask2())
        assertEquals(1, vm.uiState.value.arena.obstacles.size)
        val beforeStart = outbound.toList()
        assertTrue(vm.startRun { outbound.add("BEGIN") })
        assertEquals(beforeStart + "BEGIN", outbound)
        vm.accept("TASK2,ARROW,1,LEFT\nTARGET,1,38\nSTATUS,DONE")
        advanceUntilIdle()
        assertNull(vm.uiState.value.arena.obstacles.getValue(1).targetId)
        assertTrue(vm.uiState.value.statusHistory.contains("Obstacle 1 arrow: LEFT"))
        vm.selectTask(TaskMode.TASK1)
        assertTrue(outbound.last().contains("OBSTACLE,1,50,50,NORTH"))
        assertFalse(vm.canRequestTask1())
    }

    @Test fun `task2 permits delimiter free mode and reconnect keeps uncertain run`() = runTest(dispatcher) {
        vm.selectTask(TaskMode.TASK2)
        vm.connectionChanged(true)
        vm.setPlacementMode(true)
        assertFalse(vm.uiState.value.placementMode)
        vm.setAmdToolMode(true)
        assertFalse(vm.canRequestTask2())
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        assertTrue(vm.canRequestTask2())
        vm.startRun { outbound.add("BEGIN") }
        val beforeReconnect = outbound.toList()
        vm.connectionChanged(false)
        vm.connectionChanged(true)
        assertEquals(Task1Phase.UNKNOWN, vm.uiState.value.task1Phase)
        assertEquals(beforeReconnect + "TASK2,STATUS?", outbound)
        vm.selectTask(TaskMode.TASK1)
        assertEquals(TaskMode.TASK2, vm.uiState.value.taskMode)
        val restored = ArenaViewModel(handle, ArenaOutboundSink(outbound::add), useRpiMapSync = true)
        restored.connectionChanged(true)
        assertEquals(TaskMode.TASK2, restored.uiState.value.taskMode)
        assertEquals(Task1Phase.UNKNOWN, restored.uiState.value.task1Phase)
        assertEquals(beforeReconnect + listOf("TASK2,STATUS?", "TASK2,STATUS?"), outbound)
    }

    @Test fun `unknown task2 outcome needs explicit recovery and fresh readiness`() = runTest(dispatcher) {
        vm.selectTask(TaskMode.TASK2)
        vm.connectionChanged(true)
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        assertTrue(vm.startRun { outbound.add("BEGIN") })
        vm.connectionChanged(false)
        assertEquals(Task1Phase.UNKNOWN, vm.uiState.value.task1Phase)
        assertFalse(vm.canRequestTask2())
        vm.connectionChanged(true)
        assertEquals("TASK2,STATUS?", outbound.last())
        vm.accept("TASK2,RUNNING")
        advanceUntilIdle()
        assertFalse(vm.canRequestTask2())
        vm.newAttempt() // Operator has verified the previous run ended.
        assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
        assertFalse(vm.uiState.value.autonomousRunning)
        assertFalse(vm.canRequestTask2())
        assertEquals("TASK2,STATUS?", outbound.last())
        vm.accept("TASK2,RUNNING")
        advanceUntilIdle()
        assertEquals(Task1Phase.UNKNOWN, vm.uiState.value.task1Phase)
        assertFalse(vm.canRequestTask2())
    }

    @Test fun `uncertain delivery is not retried and stale setup status is ignored`() = runTest(dispatcher) {
        vm.selectTask(TaskMode.TASK2)
        vm.connectionChanged(true)
        vm.accept("STATUS,DONE")
        advanceUntilIdle()
        assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        assertTrue(vm.startRun { error("possibly partial BEGIN") })
        assertEquals(Task1Phase.UNKNOWN, vm.uiState.value.task1Phase)
        assertFalse(vm.startRun { fail("must not retry BEGIN") })
        assertEquals(listOf("TASK2,STATUS?"), outbound)
    }

    @Test fun `failure ends task2 without enabling an implicit retry`() = runTest(dispatcher) {
        vm.selectTask(TaskMode.TASK2)
        vm.connectionChanged(true)
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        vm.startRun { outbound.add("BEGIN") }
        vm.accept("STATUS,FAILED\nSTATUS,DONE")
        advanceUntilIdle()
        assertEquals(Task1Phase.FAILED, vm.uiState.value.task1Phase)
        assertFalse(vm.uiState.value.autonomousRunning)
        assertFalse(vm.startRun { fail("retry requires a new attempt") })
        vm.newAttempt()
        assertFalse(vm.canRequestTask2())
        assertEquals("TASK2,STATUS?", outbound.last())
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        assertTrue(vm.canRequestTask2())
    }

    @Test fun `task2 stop and new attempt do not erase task1 results`() = runTest(dispatcher) {
        vm.addObstacle(GridCoordinate(5, 5))
        vm.setTargetFace(Direction.NORTH)
        vm.connectionChanged(true)
        vm.accept("TARGET,1,A")
        advanceUntilIdle()
        vm.selectTask(TaskMode.TASK2)
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        vm.startRun { outbound.add("BEGIN") }
        vm.stopManual { outbound.add("s") }
        assertEquals(Task1Phase.STOPPED, vm.uiState.value.task1Phase)
        vm.newAttempt()
        assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
        assertEquals("A", vm.uiState.value.arena.obstacles.getValue(1).targetId)
        assertFalse(vm.canRequestTask2())
        vm.selectTask(TaskMode.TASK1)
        assertFalse(vm.canRequestTask1())
    }

    @Test fun `late connection queries readiness and rejected begin can recover`() = runTest(dispatcher) {
        vm.selectTask(TaskMode.TASK2)
        vm.connectionChanged(true)
        assertEquals("TASK2,STATUS?", outbound.last())
        vm.accept("TASK2,NOT_READY\nSTATUS,OK")
        advanceUntilIdle()
        assertFalse(vm.canRequestTask2()) // Generic OK also acknowledges manual movement.
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        assertTrue(vm.startRun { outbound.add("BEGIN") })
        vm.accept("TASK2,NOT_READY")
        advanceUntilIdle()
        assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
        assertFalse(vm.uiState.value.autonomousRunning)
        assertFalse(vm.canRequestTask2())
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        assertTrue(vm.canRequestTask2())
    }

    @Test fun `definite queue rejection does not strand start while startup failure clears readiness`() = runTest(dispatcher) {
        vm.selectTask(TaskMode.TASK2)
        vm.connectionChanged(true)
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        assertFalse(vm.startRun { throw CommandQueueRejected() })
        assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
        assertFalse(vm.uiState.value.autonomousRunning)
        assertTrue(vm.canRequestTask2())
        vm.accept("STATUS,FAILED")
        advanceUntilIdle()
        assertFalse(vm.canRequestTask2())
        assertTrue(vm.uiState.value.feedback.contains("startup failure"))
    }

    @Test fun `failed stop queue does not claim the active run stopped`() = runTest(dispatcher) {
        vm.selectTask(TaskMode.TASK2)
        vm.connectionChanged(true)
        vm.accept("TASK2,READY")
        advanceUntilIdle()
        vm.startRun { outbound.add("BEGIN") }
        vm.accept("STATUS,RUNNING,0,5")
        advanceUntilIdle()
        vm.stopManual { throw CommandQueueRejected() }
        assertTrue(vm.uiState.value.autonomousRunning)
        assertEquals(Task1Phase.RUNNING, vm.uiState.value.task1Phase)
        vm.connectionChanged(false)
        vm.stopManual { fail("offline stop must not send") }
        assertEquals(Task1Phase.UNKNOWN, vm.uiState.value.task1Phase)
    }

    @Test fun `unsolicited running status before start disables another begin`() = runTest(dispatcher) {
        vm.selectTask(TaskMode.TASK2)
        vm.connectionChanged(true)
        vm.accept("TASK2,READY\nSTATUS,RUNNING,0,5")
        advanceUntilIdle()
        assertEquals(Task1Phase.UNKNOWN, vm.uiState.value.task1Phase)
        assertFalse(vm.canRequestTask2())
        assertFalse(vm.startRun { fail("must not enqueue second BEGIN") })
    }
}
