package com.mdp.g15.arena.presentation

import androidx.lifecycle.SavedStateHandle
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.integration.ArenaOutboundSink
import com.mdp.g15.arena.protocol.*
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class Task1ReadinessTest {
    private val dispatcher = StandardTestDispatcher()
    private val sent = mutableListOf<String>()
    private lateinit var vm: ArenaViewModel
    @Before fun setup() {
        Dispatchers.setMain(dispatcher)
        vm = ArenaViewModel(SavedStateHandle(), ArenaOutboundSink(sent::add), useRpiMapSync = true)
        vm.connectionChanged(true)
    }
    @After fun teardown() = Dispatchers.resetMain()
    private fun add(count: Int, face: Boolean = true) {
        repeat(count) { i ->
            vm.addObstacle(GridCoordinate(i, 0))
            if (face) vm.setTargetFace(Direction.NORTH)
        }
    }
    private fun hash() = java.security.MessageDigest.getInstance("SHA-256")
        .digest(ObstacleSync.encode(vm.uiState.value.arena.obstacles).toByteArray(Charsets.UTF_8))
        .joinToString("") { "%02x".format(it) }

    @Test fun `count boundaries and missing faces guard the handler as well as UI`() {
        for (count in 0..9) {
            vm.resetArena()
            add(count)
            assertEquals("count $count", count in 4..8, vm.canRequestTask1())
            if (count !in 4..8) assertFalse(vm.startRun { fail("invalid count sent BEGIN") })
        }
        vm.resetArena()
        add(4, face = false)
        assertTrue(vm.task1StartIssue()!!.contains("1, 2, 3, 4"))
        assertFalse(vm.startRun { fail("missing faces sent BEGIN") })
        for (id in 1..4) { vm.selectObstacle(id); vm.setTargetFace(Direction.EAST) }
        assertTrue(vm.canRequestTask1())
        assertTrue(vm.startRun {}) // No planner report is required.
    }

    @Test fun `planner telemetry never unlocks invalid setup or starts a mission`() = runTest(dispatcher) {
        sent.clear()
        vm.accept("PLANNER,READY")
        advanceUntilIdle()
        assertFalse(vm.canRequestTask1())
        assertFalse(vm.uiState.value.autonomousRunning)
        assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
        assertTrue(vm.uiState.value.plannerInfo.contains("unverified"))
        assertEquals(vm.uiState.value.plannerInfo, vm.uiState.value.feedback)
        assertEquals("PLANNER,READY", vm.uiState.value.latestReceived)
        assertTrue(sent.isEmpty())
        add(4)
        assertEquals(DEFAULT_PLANNER_INFO, vm.uiState.value.plannerInfo)
        assertTrue(vm.canRequestTask1())
        assertTrue(vm.startRun { sent.add("BEGIN") })
    }

    @Test fun `matching map report survives start and repeated report does not send begin`() = runTest(dispatcher) {
        add(4)
        val report = "PLANNER,READY,${hash()}"
        vm.accept(report)
        advanceUntilIdle()
        assertTrue(vm.uiState.value.plannerInfo.contains("for this obstacle map"))
        assertEquals(vm.uiState.value.plannerInfo, vm.uiState.value.feedback)
        sent.clear()
        vm.startRun { sent.add("BEGIN") }
        vm.accept("$report\n$report")
        advanceUntilIdle()
        assertEquals(Task1Phase.START_REQUESTED, vm.uiState.value.task1Phase)
        assertEquals(listOf("BEGIN"), sent)
    }

    @Test fun `edits undo reconnect and new attempt invalidate readiness`() = runTest(dispatcher) {
        add(4)
        val oldReport = "PLANNER,READY,${hash()}"
        vm.accept(oldReport)
        advanceUntilIdle()
        vm.moveObstacle(1, GridCoordinate(0, 2))
        vm.accept(oldReport)
        advanceUntilIdle()
        assertEquals(DEFAULT_PLANNER_INFO, vm.uiState.value.plannerInfo)
        vm.undo()
        assertEquals(DEFAULT_PLANNER_INFO, vm.uiState.value.plannerInfo)
        vm.accept("PLANNER,READY")
        advanceUntilIdle()
        vm.connectionChanged(false)
        vm.accept("PLANNER,READY")
        advanceUntilIdle()
        assertEquals(DEFAULT_PLANNER_INFO, vm.uiState.value.plannerInfo)
        vm.connectionChanged(true)
        assertEquals(DEFAULT_PLANNER_INFO, vm.uiState.value.plannerInfo)
        vm.accept("PLANNER,READY")
        advanceUntilIdle()
        vm.newAttempt()
        assertEquals(DEFAULT_PLANNER_INFO, vm.uiState.value.plannerInfo)
    }

    @Test fun `malformed stale and post-completion reports cannot change accepted readiness or run state`() = runTest(dispatcher) {
        add(4)
        vm.accept("PLANNER,READY,${hash()}")
        advanceUntilIdle()
        val info = vm.uiState.value.plannerInfo
        for (line in listOf("PLANNER", "PLANNER,READY,", "PLANNER,READY,no", "PLANNER,READY,${hash()},extra", "PLANNER,RUNNING", "PLANNER,READY," + "0".repeat(64))) {
            vm.accept(line)
            advanceUntilIdle()
            assertEquals(info, vm.uiState.value.plannerInfo)
            assertTrue(vm.canRequestTask1())
        }
        vm.startRun {}
        vm.accept("STATUS,DONE")
        advanceUntilIdle()
        vm.accept("PLANNER,READY")
        advanceUntilIdle()
        assertEquals(Task1Phase.COMPLETED, vm.uiState.value.task1Phase)
        assertEquals(info, vm.uiState.value.plannerInfo)
    }

    @Test fun `planner regex rejects malformed payloads without replacing readiness`() = runTest(dispatcher) {
        add(4)
        vm.accept("PLANNER,READY,${hash()}")
        advanceUntilIdle()
        val accepted = vm.uiState.value.plannerInfo
        val codec = CsvArenaMessageCodec()
        val malformed = listOf(
            "PLANNER", "PLANNER,", "PLANNER,READY,", "PLANNER,READY,,",
            "PLANNER,READY,extra", "PLANNER,READY," + "a".repeat(63),
            "PLANNER,READY," + "a".repeat(65), "PLANNER,READY," + "A".repeat(64),
            "PLANNER,READY," + "g".repeat(64), "PLANNER,READY," + "０".repeat(64),
            "PLANNER,ready", "planner,READY", "PLANNER, READY", "PLANNER ,READY",
            "PLANNER,READY,${hash()},extra", "PLANNER,READY, ${hash()}",
            "PLANNER,READY\t,${hash()}", "PLANNER,READY\u0000",
            "PLANNER,READY;STATUS,DONE",
        )
        for (payload in malformed) {
            assertTrue(payload, codec.decode(payload) is ArenaDecodeResult.Malformed)
            vm.accept(payload)
            advanceUntilIdle()
            assertEquals(payload, accepted, vm.uiState.value.plannerInfo)
            assertEquals(Task1Phase.SETUP, vm.uiState.value.task1Phase)
            assertTrue(vm.canRequestTask1())
        }
        // Multiple Bluetooth lines are individually framed by the receiver; a codec payload
        // containing an embedded line break is never accepted as one planner message.
        assertTrue(codec.decode("PLANNER,READY\nSTATUS,DONE") is ArenaDecodeResult.Malformed)
        assertTrue(codec.decode("PLANNER,READY,${hash()}\rSTATUS,DONE") is ArenaDecodeResult.Malformed)
    }

    @Test fun `codec accepts only documented optional notification format`() {
        val codec = CsvArenaMessageCodec()
        assertEquals(ArenaDecodeResult.Decoded(ArenaInboundEvent.PlannerReady()), codec.decode("PLANNER,READY"))
        assertTrue(codec.decode("PLANNER,READY," + "a".repeat(64)) is ArenaDecodeResult.Decoded)
        assertTrue(codec.decode("PLANNER,READY," + "a".repeat(65)) is ArenaDecodeResult.Malformed)
        assertTrue(codec.decode("MSG,PLANNER,READY") is ArenaDecodeResult.Decoded)
    }
}
