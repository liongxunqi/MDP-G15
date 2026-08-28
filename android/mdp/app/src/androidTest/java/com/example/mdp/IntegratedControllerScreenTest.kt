package com.example.mdp

import androidx.compose.material3.MaterialTheme
import androidx.compose.ui.test.assertIsDisplayed
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

    @Before
    fun setUp() {
        incoming = MutableSharedFlow(extraBufferCapacity = 8)
        outbound = mutableListOf()
        connectClicks = 0
    }

    @Test
    fun controlsAreDefaultAndKeepExistingCallback() {
        setScreen()

        composeRule.onNodeWithText("Bluetooth").assertIsDisplayed()
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
    fun arenaCollectsMessagesWhileControlsAreVisible() {
        setScreen()
        composeRule.waitForIdle()

        assertTrue(incoming.tryEmit("STATUS,Exploring"))
        composeRule.waitForIdle()
        composeRule.onNodeWithText("Arena").performClick()

        composeRule.onNodeWithText("Exploring").assertIsDisplayed()
    }

    @Test
    fun obstaclePlacementUsesHostOutboundCallbackOnce() {
        setScreen()
        composeRule.onNodeWithText("Arena").performClick()
        composeRule.onNodeWithText("Add obstacle").performClick()

        composeRule.onNodeWithTag("arena_grid").performTouchInput { click(center) }
        composeRule.waitForIdle()

        assertEquals(1, outbound.size)
        assertTrue(outbound.single().startsWith("OBSTACLE,UPSERT,1,"))
    }

    private fun setScreen() {
        composeRule.setContent { ScreenContent() }
    }

    @androidx.compose.runtime.Composable
    private fun ScreenContent() {
        MaterialTheme {
            IntegratedControllerScreen(
                status = "Disconnected",
                robotStatus = "—",
                isConnected = false,
                isBusy = false,
                permissionsGranted = true,
                latestMessage = "",
                onConnectClick = { connectClicks += 1 },
                onDisconnectClick = {},
                onForward = {},
                onReverse = {},
                onTurnLeft = {},
                onTurnRight = {},
                onStop = {},
                incomingMessages = incoming,
                onArenaOutbound = outbound::add,
            )
        }
    }
}
