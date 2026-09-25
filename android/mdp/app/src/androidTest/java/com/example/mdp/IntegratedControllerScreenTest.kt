package com.example.mdp

import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.setValue
import androidx.compose.runtime.mutableStateOf
import com.example.mdp.ui.theme.MdpTheme
import androidx.compose.material3.Surface
import androidx.compose.ui.graphics.asAndroidBitmap
import androidx.compose.ui.test.captureToImage
import androidx.compose.ui.test.onRoot
import androidx.test.platform.app.InstrumentationRegistry
import java.io.File
import android.graphics.Bitmap
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.performScrollTo
import androidx.compose.ui.test.junit4.StateRestorationTester
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import kotlinx.coroutines.flow.MutableSharedFlow
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class IntegratedControllerScreenTest {
    @get:Rule
    val composeRule = createComposeRule()

    private lateinit var incoming: MutableSharedFlow<String>
    private lateinit var outbound: MutableList<String>
    private var connectClicks = 0
    private lateinit var commandLog: CommandLog
    private var connected by mutableStateOf(false)
    private var appendNewline by mutableStateOf(true)
    private val movement = mutableListOf<String>()
    private var targetLookup: (Int) -> Boolean = { false }
    private val targetChecksAtMapWrite = mutableListOf<RobotMessage>()

    @Before
    fun setUp() {
        incoming = MutableSharedFlow(extraBufferCapacity = 8)
        outbound = mutableListOf()
        connectClicks = 0
        commandLog = CommandLog()
        connected = false
        appendNewline = true
        movement.clear()
        targetLookup = { false }
        targetChecksAtMapWrite.clear()
    }

    @Test
    fun connectionAllowsFreeCommandsUntilValidPoseArrives() {
        connected = true
        setScreen()
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("◀ Left").assertIsEnabled().performClick()
        composeRule.runOnIdle { assertEquals(listOf("tl"), movement) }
        composeRule.waitUntil(5_000) {
            runCatching { composeRule.onNodeWithText("▲ Forward").assertIsEnabled() }.isSuccess
        }
        composeRule.runOnIdle { assertTrue(incoming.tryEmit("ROBOT,99,99,N")) }
        composeRule.waitForIdle()
        composeRule.onNodeWithText("▲ Forward").assertIsEnabled()
        composeRule.runOnIdle { assertTrue(incoming.tryEmit("ROBOT,8,18,N")) }
        composeRule.waitForIdle()
        composeRule.onNodeWithText("▲ Forward").assertIsNotEnabled()
        composeRule.onNodeWithText("▼ Reverse").assertIsEnabled()
    }

    @Test
    fun amdToggleControlsMovementAndRejectedUpdatesPreserveDriveButtons() {
        connected = true
        setScreen()
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("Amd tool").performScrollTo().performClick()
        composeRule.runOnIdle { assertEquals(false, appendNewline) }
        composeRule.runOnIdle { assertTrue(incoming.tryEmit("STATUS,DONE\nROBOT,8,8,N")) }
        composeRule.waitForIdle()
        composeRule.onNodeWithText("◀ Left").assertIsEnabled().performClick()
        composeRule.onNodeWithText("(7.00, 8.00) • W • estimated").performScrollTo().assertIsDisplayed()
        composeRule.waitUntil(5_000) {
            runCatching { composeRule.onNodeWithText("Right ▶").assertIsEnabled() }.isSuccess
        }
        composeRule.runOnIdle {
            assertTrue(incoming.tryEmit("ROBOT,19,19,N\nSTATUS,START,9999999999999999,8,N\nSTATUS,INVALID"))
        }
        composeRule.waitForIdle()
        composeRule.onNodeWithText("Right ▶").assertIsEnabled().performClick()
        composeRule.onNodeWithText("(8.00, 8.00) • E • estimated").performScrollTo().assertIsDisplayed()
        composeRule.waitUntil(5_000) {
            runCatching { composeRule.onNodeWithText("normal").assertIsEnabled() }.isSuccess
        }
        composeRule.onNodeWithText("normal").performScrollTo().performClick()
        composeRule.runOnIdle { assertEquals(true, appendNewline) }
        composeRule.onNodeWithText("Right ▶").assertIsEnabled().performClick()
        composeRule.onNodeWithText("(10.91, 5.09) • S • estimated").performScrollTo().assertIsDisplayed()
        composeRule.runOnIdle { assertEquals(listOf("tl", "tr", "tr"), movement) }
    }

    @Test
    fun targetLookupSeesEditsImmediatelyBeforeAnotherComposition() {
        connected = true
        setScreen()
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("Add obstacle").performScrollTo().performClick()
        // Captured synchronously inside the outgoing map callback, not after recomposition.
        assertEquals(RobotMessage.Target(1, "11"), targetChecksAtMapWrite.last())
        composeRule.onNodeWithText("Undo").performScrollTo().performClick()
        assertEquals(RobotMessage.TargetRejected("Target references unknown obstacle 1."), targetChecksAtMapWrite.last())
        composeRule.onNodeWithText("Redo").performScrollTo().performClick()
        assertEquals(RobotMessage.Target(1, "11"), targetChecksAtMapWrite.last())
    }

    @Test
    fun controlsAreDefaultAndKeepExistingCallback() {
        setScreen()

        composeRule.onNodeWithText("Bluetooth").assertIsDisplayed()
        composeRule.onNodeWithText("▲ Forward").assertDoesNotExist()
        composeRule.onNodeWithText("■ Stop").assertDoesNotExist()
        composeRule.onNodeWithText("Stop robot").assertDoesNotExist()
        capture("controls")
        composeRule.onNodeWithText("Connect").performClick()

        assertEquals(1, connectClicks)
    }

    @Test
    fun selectedArenaTabSurvivesStateRestoration() {
        val restoration = StateRestorationTester(composeRule)
        restoration.setContent { ScreenContent() }
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("Arena status").assertIsDisplayed()

        restoration.emulateSavedInstanceStateRestore()

        composeRule.onNodeWithText("Arena status").assertIsDisplayed()
    }

    @Test
    fun robotStatusIsSharedByControlsAndArena() {
        setScreen()
        composeRule.onNodeWithText("Awaiting robot status").assertIsDisplayed()
        composeRule.runOnIdle { assertTrue(incoming.tryEmit("MSG,[Exploring]")) }
        composeRule.onNodeWithText("Exploring").assertIsDisplayed()
        composeRule.runOnIdle { assertTrue(incoming.tryEmit("STATUS,INVALID")) }
        composeRule.waitForIdle()
        composeRule.onNodeWithText("Exploring").assertIsDisplayed()
        composeRule.onNodeWithText("Arena").performClick()

        composeRule.onNodeWithText("Exploring").assertIsDisplayed()
        composeRule.onNodeWithText("20 × 20", substring = true).assertDoesNotExist()
        composeRule.onNodeWithText("Latest action").assertIsDisplayed()
    }

    @Test
    fun obstaclePlacementUsesHostOutboundCallbackOnce() {
        connected = true
        setScreen()
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("Add obstacle").performScrollTo().performClick()

        composeRule.waitForIdle()

        assertEquals(2, outbound.size) // initial map, then completed placement
        assertEquals("CLEAR", outbound.first())
        assertEquals("CLEAR\nOBSTACLE,1,0,0,SKIP", outbound.last())
        repeat(2) { composeRule.onNodeWithText("Add obstacle").performScrollTo().performClick() }
        assertEquals(4, outbound.size)
        assertEquals("CLEAR\nOBSTACLE,1,0,0,SKIP\nOBSTACLE,2,10,0,SKIP\nOBSTACLE,3,20,0,SKIP", outbound.last())
        composeRule.onNodeWithText("W").performScrollTo().performClick()
        assertTrue(outbound.last().endsWith("OBSTACLE,3,20,0,WEST"))
    }

    @Test
    fun immediateAddStopsAtLimitAndUndoRedoRestoresAvailability() {
        connected = true
        setScreen()
        composeRule.onNodeWithText("Arena").performClick()
        repeat(50) {
            composeRule.onNodeWithText("Add obstacle").performScrollTo().performClick()
        }
        composeRule.onNodeWithText("Obstacle limit reached").assertIsNotEnabled()
        assertEquals(51, outbound.size) // connection snapshot plus 50 additions
        assertEquals(51, outbound.last().lines().size) // CLEAR plus 50 obstacles
        composeRule.onNodeWithText("Undo").performScrollTo().performClick()
        composeRule.onNodeWithText("Add obstacle").assertIsEnabled()
        assertEquals(50, outbound.last().lines().size)
        composeRule.onNodeWithText("Redo").performScrollTo().performClick()
        composeRule.onNodeWithText("Obstacle limit reached").assertIsNotEnabled()
        assertEquals(51, outbound.last().lines().size)
    }

    @Test
    fun logsPauseDisplayButContinueCaptureAndCanFilterAndClear() {
        setScreen()
        composeRule.runOnIdle { commandLog.record(CommandLog.Kind.RX, "first incoming") }
        composeRule.onNodeWithText("Logs").performClick()
        composeRule.onNodeWithText("first incoming", substring = true).assertIsDisplayed()
        capture("logs")
        composeRule.onNodeWithText("Pause display").performClick()
        composeRule.runOnIdle { commandLog.record(CommandLog.Kind.TX, "second outgoing") }
        composeRule.onNodeWithText("second outgoing", substring = true).assertDoesNotExist()
        composeRule.onNodeWithText("Resume display").performClick()
        composeRule.onNodeWithText("second outgoing", substring = true).assertIsDisplayed()
        composeRule.onNodeWithText("RX").performClick()
        composeRule.onNodeWithText("second outgoing", substring = true).assertDoesNotExist()
        composeRule.onNodeWithText("Clear").performClick()
        composeRule.onNodeWithText("No matching events.").assertIsDisplayed()
        composeRule.runOnIdle { assertTrue(commandLog.entries.value.isEmpty()) }
    }

    @Test
    fun receivedRobotPoseIsRenderedEvenWhenReceivedOnControlsTab() {
        setScreen()
        composeRule.waitForIdle()
        assertTrue(incoming.tryEmit("ROBOT,7,2,W"))
        composeRule.waitForIdle()
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("(7, 2) • WEST").assertIsDisplayed()
        capture("arena")
        assertTrue(incoming.tryEmit("ROBOT,19,2,W"))
        composeRule.waitForIdle()
        composeRule.onNodeWithText("(7, 2) • WEST").assertIsDisplayed()
    }

    @Test
    fun longStatusAndScrollingDetailsKeepGridAndStopVisible() {
        connected = true
        setScreen()
        assertTrue(incoming.tryEmit("ROBOT,8,8,N"))
        composeRule.waitForIdle()
        composeRule.runOnIdle { assertTrue(incoming.tryEmit("MSG," + "A detailed status message from the robot. ".repeat(60))) }
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("■ Stop").assertIsDisplayed().assertIsEnabled()
        composeRule.onNodeWithTag("arena_grid").assertIsDisplayed()
        composeRule.onNodeWithText("Add obstacle").performScrollTo().assertIsDisplayed()
        composeRule.onNodeWithText("■ Stop").assertIsDisplayed()
        composeRule.onNodeWithTag("arena_grid").assertIsDisplayed()
        capture("checklist-long-status")
    }

    @Test
    fun labeledMovementAndPersistentStopKeepTheirCallbacksAndBoundaryGuards() {
        connected = true
        setScreen()
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("▲ Forward").assertIsEnabled()
        for (label in listOf("▲ Forward", "◀ Left", "Right ▶", "▼ Reverse", "↖ Forward left", "Forward right ↗", "↙ Back left", "Back right ↘")) {
            assertTrue(incoming.tryEmit("ROBOT,8,8,N"))
            composeRule.waitForIdle()
            composeRule.waitUntil(5_000) {
                runCatching { composeRule.onNodeWithText(label).assertIsEnabled() }.isSuccess
            }
            composeRule.onNodeWithText(label).assertIsDisplayed().assertIsEnabled().performClick()
            // Preview completion is time-based; deterministic spam gating is covered by VM tests.
            composeRule.onNodeWithText("■ Stop").assertIsEnabled()
            if (label == "▲ Forward") {
                composeRule.onNodeWithText("(8.00, 9.00) • N • estimated").performScrollTo().assertIsDisplayed()
                composeRule.onNodeWithTag("arena_grid").assertIsDisplayed()
                assertEquals(listOf("f"), movement)
            }
            assertTrue(incoming.tryEmit("STATUS,OK"))
            composeRule.waitForIdle()
        }
        assertEquals(listOf("f", "tl", "tr", "r", "fl", "fr", "bl", "br"), movement)
        composeRule.onNodeWithText("■ Stop").performClick()
        assertTrue(incoming.tryEmit("ROBOT,5,18,N"))
        composeRule.waitForIdle()
        composeRule.onNodeWithText("▲ Forward").assertIsNotEnabled()
        composeRule.onNodeWithText("▼ Reverse").assertIsEnabled()
        composeRule.onNodeWithTag("arena_grid").assertIsDisplayed()
        capture("checklist-connected-controls")
        for (tab in listOf("Arena", "Logs")) {
            composeRule.onNodeWithText(tab).performClick()
            composeRule.onNodeWithText("Stop robot").assertDoesNotExist()
        }
        assertEquals(1, movement.count { it == "s" })
    }

    @Test
    fun targetFaceTransmitsCoordinatesAndReceivedTargetUpdatesSelectedObstacle() {
        connected = true
        setScreen()
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("Add obstacle").performScrollTo().assertIsDisplayed().performClick()
        composeRule.onNodeWithText("N").performScrollTo().assertIsDisplayed().performClick()
        assertEquals(3, outbound.size)
        assertEquals(outbound[1].replace("SKIP", "NORTH"), outbound[2])
        assertTrue(incoming.tryEmit("TARGET,B1,11,E"))
        composeRule.waitForIdle()
        composeRule.onNodeWithText(" • face E • target 11", substring = true).performScrollTo().assertIsDisplayed()
        composeRule.onNodeWithText("+ Zoom in").assertIsDisplayed()
        composeRule.onNodeWithText("− Zoom out").assertIsDisplayed()
        capture("checklist-arena")
        assertEquals(3, outbound.size) // Received messages must never echo to the robot.
    }

    @Test
    fun offlineMapIsSentOnConnectAndResentOnReconnectWithoutMovementReplay() {
        setScreen()
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("Add obstacle").performScrollTo().performClick()
        assertTrue(outbound.isEmpty())
        composeRule.runOnIdle { connected = true }
        composeRule.waitForIdle()
        assertEquals(1, outbound.size)
        assertTrue(outbound.single().startsWith("CLEAR\nOBSTACLE,1,"))
        composeRule.onNodeWithText("Map delivery unconfirmed", substring = true).assertDoesNotExist()
        composeRule.runOnIdle { connected = false }
        composeRule.waitForIdle()
        composeRule.onNodeWithText("N").performScrollTo().performClick()
        assertEquals(1, outbound.size)
        composeRule.runOnIdle { connected = true }
        composeRule.waitForIdle()
        assertEquals(2, outbound.size)
        assertTrue(outbound.last().endsWith(",NORTH"))
        assertTrue(movement.isEmpty())
    }

    @Test
    fun devicePickerExposesScanningSelectionAndDismissCallbacks() {
        val adapter = android.bluetooth.BluetoothAdapter.getDefaultAdapter()
        val item = DeviceItem("Test robot", "00:11:22:33:44:55", adapter.getRemoteDevice("00:11:22:33:44:55"))
        var scans = 0
        var selected: DeviceItem? = null
        var dismissed = false
        composeRule.setContent {
            MdpTheme {
                DevicePickerDialog(
                    paired = listOf(item), discovered = emptyList(),
                    onScan = { scans++ }, onSelect = { selected = it }, onDismiss = { dismissed = true },
                )
            }
        }
        composeRule.onNodeWithText("Scan").assertIsDisplayed().performClick()
        composeRule.onNodeWithText("Test robot").assertIsDisplayed().performClick()
        composeRule.onNodeWithText("Close").performClick()
        assertEquals(1, scans)
        assertEquals(item, selected)
        assertTrue(dismissed)
    }

    private fun capture(name: String) {
        val image = composeRule.onRoot().captureToImage().asAndroidBitmap()
        val directory = InstrumentationRegistry.getInstrumentation().targetContext.getExternalFilesDir(null)!!
        File(directory, "$name.png").outputStream().use { image.compress(Bitmap.CompressFormat.PNG, 100, it) }
    }

    private fun setScreen() {
        composeRule.setContent { ScreenContent() }
    }

    @androidx.compose.runtime.Composable
    private fun ScreenContent() {
        MdpTheme {
          Surface {
            val logs by commandLog.entries.collectAsState()
            IntegratedControllerScreen(
                status = if (connected) "Connected to test robot" else "Disconnected",
                appendNewline = appendNewline,
                onAppendNewlineChange = { appendNewline = it },
                isConnected = connected,
                isBusy = false,
                permissionsGranted = true,
                latestMessage = "",
                commandLogs = logs,
                onClearLogs = commandLog::clear,
                onConnectClick = { connectClicks += 1 },
                onDisconnectClick = {},
                onForward = { movement.add("f") },
                onReverse = { movement.add("r") },
                onTurnLeft = { movement.add("tl") },
                onTurnRight = { movement.add("tr") },
                onStop = { movement.add("s") },
                onForwardLeft = { movement.add("fl") },
                onForwardRight = { movement.add("fr") },
                onBackLeft = { movement.add("bl") },
                onBackRight = { movement.add("br") },
                onBegin = { movement.add("BEGIN") },
                onPath = { movement.add("PATH") },
                onSendCustomMessage = { movement.add("CUSTOM:$it") },
                onObstacleLookupAvailable = { targetLookup = it },
                incomingMessages = incoming,
                onArenaOutbound = {
                    outbound.add(it)
                    targetChecksAtMapWrite.add(RobotMessageParser.parse("TARGET,1,11", targetLookup))
                },
            )
          }
        }
    }
}
