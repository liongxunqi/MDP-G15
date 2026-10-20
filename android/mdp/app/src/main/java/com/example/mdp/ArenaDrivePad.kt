package com.example.mdp

import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.semantics.clearAndSetSemantics
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.unit.dp
import androidx.compose.ui.text.style.TextOverflow
import com.mdp.g15.arena.domain.ManualCommand

// Same green as the arena's pixel buttons, with its dark-green label for contrast. Disabled
// buttons keep these colours too; they still ignore taps, they just are not greyed out.
private val DriveGreen = Color(0xFF5FD27A)
private val DriveGreenInk = Color(0xFF0E4A26)
private val StopRed = Color(0xFFD32F2F)

@Composable
fun ArenaDrivePad(status: String, connected: Boolean, allowed: Set<ManualCommand>,
                  onMove: (ManualCommand) -> Unit, onStop: () -> Unit) {
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(8.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text("Manual driving • $status", style = MaterialTheme.typography.labelLarge,
                maxLines = 2, overflow = TextOverflow.Ellipsis)
            val commands = listOf(
                listOf(ManualCommand.FORWARD_LEFT, ManualCommand.FORWARD, ManualCommand.FORWARD_RIGHT),
                listOf(ManualCommand.LEFT, null, ManualCommand.RIGHT),
                listOf(ManualCommand.BACK_LEFT, ManualCommand.REVERSE, ManualCommand.BACK_RIGHT),
            )
            commands.forEach { row ->
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                    row.forEach { command ->
                        val buttonModifier = Modifier.weight(1f).heightIn(min = 48.dp)
                        if (command == null) {
                            // Square only on screen; "Stop" is its spoken name.
                            Button(onClick = onStop, enabled = connected,
                                modifier = buttonModifier.semantics { contentDescription = "Stop" },
                                colors = ButtonDefaults.buttonColors(containerColor = StopRed, contentColor = Color.White,
                                    disabledContainerColor = StopRed, disabledContentColor = Color.White)) {
                                Text("■", style = MaterialTheme.typography.titleLarge,
                                    modifier = Modifier.clearAndSetSemantics { })
                            }
                        } else {
                            val (arrow, name) = when (command) {
                                ManualCommand.FORWARD -> "▲" to "Forward"
                                ManualCommand.REVERSE -> "▼" to "Reverse"
                                ManualCommand.LEFT -> "◀" to "Left"
                                ManualCommand.RIGHT -> "▶" to "Right"
                                ManualCommand.FORWARD_LEFT -> "↖" to "Forward left"
                                ManualCommand.FORWARD_RIGHT -> "↗" to "Forward right"
                                ManualCommand.BACK_LEFT -> "↙" to "Back left"
                                ManualCommand.BACK_RIGHT -> "↘" to "Back right"
                            }
                            // Arrow only on screen; the name is what screen readers (and tests) use.
                            Button(onClick = { onMove(command) }, enabled = connected && command in allowed,
                                modifier = buttonModifier.semantics { contentDescription = name },
                                colors = ButtonDefaults.buttonColors(containerColor = DriveGreen, contentColor = DriveGreenInk,
                                    disabledContainerColor = DriveGreen, disabledContentColor = DriveGreenInk),
                                contentPadding = PaddingValues(4.dp)) {
                                Text(arrow, style = MaterialTheme.typography.titleLarge,
                                    modifier = Modifier.clearAndSetSemantics { })
                            }
                        }
                    }
                }
            }
        }
    }
}
