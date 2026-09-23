package com.example.mdp

import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.text.style.TextOverflow
import com.mdp.g15.arena.domain.ManualCommand

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
                            Button(onClick = onStop, enabled = connected, modifier = buttonModifier,
                                colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error)) { Text("■ Stop") }
                        } else Button(onClick = { onMove(command) }, enabled = connected && command in allowed,
                            modifier = buttonModifier, contentPadding = PaddingValues(4.dp)) {
                            Text(when (command) {
                                ManualCommand.FORWARD -> "▲ Forward"
                                ManualCommand.REVERSE -> "▼ Reverse"
                                ManualCommand.LEFT -> "◀ Left"
                                ManualCommand.RIGHT -> "Right ▶"
                                ManualCommand.FORWARD_LEFT -> "↖ Forward left"
                                ManualCommand.FORWARD_RIGHT -> "Forward right ↗"
                                ManualCommand.BACK_LEFT -> "↙ Back left"
                                ManualCommand.BACK_RIGHT -> "Back right ↘"
                            }, style = MaterialTheme.typography.labelMedium)
                        }
                    }
                }
            }
        }
    }
}
