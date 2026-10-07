package com.mdp.g15.arena.presentation

import androidx.lifecycle.SavedStateHandle
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.integration.ArenaOutboundSink
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.setMain
import org.junit.After
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class Task1WorkflowTest {
    private val dispatcher = StandardTestDispatcher()
    private val sent = mutableListOf<String>()
    private val handle = SavedStateHandle()
    private lateinit var vm: ArenaViewModel

    @Before fun setup() {
        Dispatchers.setMain(dispatcher)
        vm = ArenaViewModel(handle, ArenaOutboundSink(sent::add), useRpiMapSync = true)
        repeat(4) {
            vm.addObstacle(GridCoordinate(it, 0))
            vm.setTargetFace(com.mdp.g15.arena.domain.Direction.NORTH)
        }
    }
    @After fun teardown() = Dispatchers.resetMain()

    @Test fun `path request does not start run and begin is single shot while awaiting confirmation`() = runTest(dispatcher) {
        vm.connectionChanged(true)
        assertTrue(vm.preparePath { sent.add("PATH") })
        assertEquals(Task1Phase.PATH_REQUESTED, vm.uiState.value.task1Phase)
        assertFalse(vm.uiState.value.autonomousRunning)
        assertTrue(vm.startRun { sent.add("BEGIN") })
        assertFalse(vm.startRun { sent.add("BEGIN") })
        assertFalse(vm.preparePath { sent.add("PATH") })
        advanceUntilIdle() // no timer may resend BEGIN or pretend that planning finished
        assertEquals(listOf("PATH", "BEGIN"), sent.drop(1))
        assertEquals(Task1Phase.START_REQUESTED, vm.uiState.value.task1Phase)
        vm.accept("STATUS,RUNNING,1,3")
        advanceUntilIdle()
        assertEquals(Task1Phase.RUNNING, vm.uiState.value.task1Phase)
    }

    @Test fun `offline and unfinished placement cannot send planning or start`() {
        assertFalse(vm.preparePath { fail("offline PATH") })
        assertFalse(vm.startRun { fail("offline BEGIN") })
        vm.connectionChanged(true)
        vm.setPlacementMode(true)
        assertFalse(vm.preparePath { fail("unfinished placement PATH") })
        assertFalse(vm.startRun { fail("unfinished placement BEGIN") })
        vm.setPlacementMode(false)
        assertTrue(vm.canRequestTask1())
    }

    @Test fun `map edits invalidate planning label`() {
        vm.connectionChanged(true)
        vm.preparePath {}
        vm.moveObstacle(1, GridCoordinate(3,3))
        assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
        vm.preparePath {}
        vm.undo()
        assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
    }

    @Test fun `completion survives late messages reconnect and restoration without map retransmission`() = runTest(dispatcher) {
        vm.moveObstacle(1, GridCoordinate(3,3))
        vm.connectionChanged(true)
        vm.startRun {}
        vm.accept("ROBOT,2,2,N\nTARGET,1,20\nSTATUS,DONE")
        advanceUntilIdle()
        val result = vm.uiState.value.arena
        sent.clear()
        vm.accept("STATUS,RUNNING,1,3\nSTATUS,FAILED\nMSG,Connected\nROBOT,NaN,99,N")
        advanceUntilIdle()
        vm.connectionChanged(false)
        vm.connectionChanged(true)
        assertEquals(Task1Phase.COMPLETED, vm.uiState.value.task1Phase)
        assertEquals(result, vm.uiState.value.arena)
        assertFalse(vm.startRun { fail("completed run restarted") })
        val restored = ArenaViewModel(handle, ArenaOutboundSink(sent::add), useRpiMapSync = true)
        restored.connectionChanged(true)
        assertEquals(Task1Phase.COMPLETED, restored.uiState.value.task1Phase)
        assertEquals(result.obstacles, restored.uiState.value.arena.obstacles)
        assertNull(restored.uiState.value.arena.robot)
        assertTrue(sent.isEmpty())
    }

    @Test fun `explicit new attempt clears only results and cannot undo old detections`() = runTest(dispatcher) {
        vm.moveObstacle(1, GridCoordinate(3,3))
        vm.connectionChanged(true)
        vm.startRun {}
        vm.accept("TARGET,1,20,N\nSTATUS,DONE")
        advanceUntilIdle()
        val obstacle = vm.uiState.value.arena.obstacles.getValue(1)
        sent.clear()
        vm.newAttempt()
        assertEquals(obstacle.copy(targetId = null), vm.uiState.value.arena.obstacles.getValue(1))
        assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
        assertTrue(vm.canRequestTask1())
        vm.undo()
        assertNull(vm.uiState.value.arena.obstacles.getValue(1).targetId)
        assertTrue(sent.isEmpty())
    }

    @Test fun `existing detections require explicit clearing even without completed status`() = runTest(dispatcher) {
        vm.moveObstacle(1, GridCoordinate(3,3))
        vm.connectionChanged(true)
        vm.accept("TARGET,1,20")
        advanceUntilIdle()
        assertFalse(vm.canRequestTask1())
        vm.newAttempt()
        assertTrue(vm.canRequestTask1())
    }

    @Test fun `disconnect during start does not unlock commands or send clear and restore remains unknown`() {
        vm.connectionChanged(true)
        vm.startRun {}
        sent.clear()
        vm.connectionChanged(false)
        vm.connectionChanged(true)
        vm.newAttempt()
        assertTrue(vm.uiState.value.autonomousRunning)
        assertEquals(Task1Phase.UNKNOWN, vm.uiState.value.task1Phase)
        assertFalse(vm.canRequestTask1())
        val restored = ArenaViewModel(handle, ArenaOutboundSink(sent::add), useRpiMapSync = true)
        restored.connectionChanged(true)
        assertEquals(Task1Phase.UNKNOWN, restored.uiState.value.task1Phase)
        assertTrue(restored.uiState.value.autonomousRunning)
        assertTrue(sent.isEmpty())
    }

    @Test fun `throwing transport cannot crash or cause automatic start retry`() {
        vm.connectionChanged(true)
        assertFalse(vm.preparePath { error("send failed") })
        assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
        vm.startRun { error("possibly partial write") }
        assertEquals(Task1Phase.UNKNOWN, vm.uiState.value.task1Phase)
        assertTrue(vm.uiState.value.autonomousRunning)
        assertFalse(vm.startRun { fail("retry") })
    }

    @Test fun `manual animation blocks path request and start until its preview completes`() = runTest(dispatcher) {
        vm.connectionChanged(true)
        vm.accept("ROBOT,8,8,N")
        advanceUntilIdle()
        vm.drive(com.mdp.g15.arena.domain.ManualCommand.FORWARD) {}
        assertFalse(vm.preparePath { fail("PATH during movement") })
        assertFalse(vm.startRun { fail("BEGIN during movement") })
        advanceUntilIdle()
        assertTrue(vm.preparePath {})
        assertTrue(vm.startRun {})
    }

    @Test fun `invalid progress cannot change a pending run and new attempt cannot erase active results`() = runTest(dispatcher) {
        vm.moveObstacle(1, GridCoordinate(3,3))
        vm.connectionChanged(true)
        vm.startRun {}
        vm.accept("TARGET,1,20\nSTATUS,START,NaN,99,N\nSTATUS,DONE,extra")
        advanceUntilIdle()
        assertEquals(Task1Phase.START_REQUESTED, vm.uiState.value.task1Phase)
        vm.newAttempt()
        assertEquals("20", vm.uiState.value.arena.obstacles.getValue(1).targetId)
        assertTrue(vm.uiState.value.autonomousRunning)
        vm.stopManual {}
        vm.accept("STATUS,DONE")
        advanceUntilIdle()
        assertEquals(Task1Phase.STOPPED, vm.uiState.value.task1Phase)
        assertEquals("20", vm.uiState.value.arena.obstacles.getValue(1).targetId)
    }

    @Test fun `failure preserves partial results and delayed done does not claim success`() = runTest(dispatcher) {
        vm.moveObstacle(1, GridCoordinate(3,3))
        vm.connectionChanged(true)
        vm.startRun {}
        vm.accept("TARGET,1,20\nSTATUS,FAILED\nSTATUS,DONE")
        advanceUntilIdle()
        assertEquals(Task1Phase.FAILED, vm.uiState.value.task1Phase)
        assertEquals("20", vm.uiState.value.arena.obstacles.getValue(1).targetId)
        assertFalse(vm.uiState.value.autonomousRunning)
    }
}
