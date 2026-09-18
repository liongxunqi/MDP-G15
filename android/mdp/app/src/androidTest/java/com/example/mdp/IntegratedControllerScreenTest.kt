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
import androidx.compose.ui.test.performTouchInput
import androidx.compose.ui.test.click
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
    private var robotStatusText by mutableStateOf("Awaiting robot status")
    private val movement = mutableListOf<String>()

    @Before
    fun setUp() {
        incoming = MutableSharedFlow(extraBufferCapacity = 8)
        outbound = mutableListOf()
        connectClicks = 0
        commandLog = CommandLog()
        connected = false
        robotStatusText = "Awaiting robot status"
        movement.clear()
    }

    @Test
    fun controlsAreDefaultAndKeepExistingCallback() {
        setScreen()

        composeRule.onNodeWithText("Bluetooth").assertIsDisplayed()
        composeRule.onNodeWithText("▲ Forward").assertIsNotEnabled()
        composeRule.onNodeWithText("■ Stop").assertIsNotEnabled()
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
        composeRule.runOnIdle { robotStatusText = "Exploring" }
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
        composeRule.onNodeWithText("Add obstacle").performClick()

        composeRule.onNodeWithTag("arena_grid").performTouchInput { click(center) }
        composeRule.waitForIdle()

        assertEquals(2, outbound.size) // initial map, then completed placement
        assertEquals("CLEAR", outbound.first())
        assertTrue(outbound.last().startsWith("CLEAR\nOBSTACLE,1,"))
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
    fun labeledMovementAndPersistentStopKeepTheirCallbacksAndBoundaryGuards() {
        connected = true
        setScreen()
        for (label in listOf("▲ Forward", "◀ Left", "Right ▶", "▼ Reverse", "■ Stop")) {
            composeRule.onNodeWithText(label).performScrollTo().performClick()
        }
        assertEquals(listOf("f", "tl", "tr", "r", "s"), movement)
        for (label in listOf("↖ Forward left", "Forward right ↗", "↙ Back left", "Back right ↘", "Begin")) {
            composeRule.onNodeWithText(label).performScrollTo().performClick()
        }
        assertEquals(listOf("fl", "fr", "bl", "br", "BEGIN"), movement.takeLast(5))
        capture("checklist-connected-controls")
        assertTrue(incoming.tryEmit("ROBOT,5,18,N"))
        composeRule.waitForIdle()
        composeRule.onNodeWithText("▲ Forward").assertIsNotEnabled()
        composeRule.onNodeWithText("▼ Reverse").assertIsEnabled()
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
        composeRule.onNodeWithText("Add obstacle").assertIsDisplayed().performClick()
        composeRule.onNodeWithTag("arena_grid").performTouchInput { click(center) }
        composeRule.onNodeWithText("N").assertIsDisplayed().performClick()
        assertEquals(3, outbound.size)
        assertEquals(outbound[1].replace("SKIP", "NORTH"), outbound[2])
        assertTrue(incoming.tryEmit("TARGET,B1,11,E"))
        composeRule.waitForIdle()
        composeRule.onNodeWithText(" • face E • target 11", substring = true).assertIsDisplayed()
        composeRule.onNodeWithText("+ Zoom in").assertIsDisplayed()
        composeRule.onNodeWithText("− Zoom out").assertIsDisplayed()
        capture("checklist-arena")
        assertEquals(3, outbound.size) // Received messages must never echo to the robot.
    }

    @Test
    fun offlineMapIsSentOnConnectAndResentOnReconnectWithoutMovementReplay() {
        setScreen()
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("Add obstacle").performClick()
        composeRule.onNodeWithTag("arena_grid").performTouchInput { click(center) }
        assertTrue(outbound.isEmpty())
        composeRule.runOnIdle { connected = true }
        composeRule.waitForIdle()
        assertEquals(1, outbound.size)
        assertTrue(outbound.single().startsWith("CLEAR\nOBSTACLE,1,"))
        composeRule.onNodeWithText("Map delivery unconfirmed", substring = true).assertDoesNotExist()
        composeRule.runOnIdle { connected = false }
        composeRule.waitForIdle()
        composeRule.onNodeWithText("N").performClick()
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
                robotStatus = robotStatusText,
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
                incomingMessages = incoming,
                onArenaOutbound = outbound::add,
            )
          }
        }
    }
}
