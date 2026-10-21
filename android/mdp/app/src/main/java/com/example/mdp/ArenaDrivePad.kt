package com.example.mdp

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.unit.dp
import com.mdp.g15.arena.domain.ManualCommand
import com.mdp.g15.arena.presentation.SpiderMap

private val DriveGreen = Color(0xFF5FD27A)
private val StopCoral = Color(0xFFF2735C)

@Composable
fun ArenaDrivePad(status: String, connected: Boolean, allowed: Set<ManualCommand>,
                  onMove: (ManualCommand) -> Unit, onStop: () -> Unit) {
        Box(Modifier.fillMaxWidth().padding(8.dp), contentAlignment = Alignment.Center) {
            Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
                val commands = listOf(
                    listOf(ManualCommand.FORWARD_LEFT, ManualCommand.FORWARD, ManualCommand.FORWARD_RIGHT),
                    listOf(ManualCommand.LEFT, null, ManualCommand.RIGHT),
                    listOf(ManualCommand.BACK_LEFT, ManualCommand.REVERSE, ManualCommand.BACK_RIGHT),
                )
                commands.forEach { row ->
                    Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                        row.forEach { command ->
                            val name = when (command) {
                                null -> "Stop"
                                ManualCommand.FORWARD -> "Forward"
                                ManualCommand.REVERSE -> "Reverse"
                                ManualCommand.LEFT -> "Left"
                                ManualCommand.RIGHT -> "Right"
                                ManualCommand.FORWARD_LEFT -> "Forward left"
                                ManualCommand.FORWARD_RIGHT -> "Forward right"
                                ManualCommand.BACK_LEFT -> "Back left"
                                ManualCommand.BACK_RIGHT -> "Back right"
                            }
                            Button(
                                onClick = { if (command == null) onStop() else onMove(command) },
                                enabled = connected && (command == null || command in allowed),
                                modifier = Modifier.size(42.dp).semantics { contentDescription = name },
                                shape = SpiderMap.ButtonShape,
                                border = BorderStroke(3.dp, SpiderMap.Ink),
                                contentPadding = PaddingValues(0.dp),
                                colors = ButtonDefaults.buttonColors(
                                    containerColor = if (command == null) StopCoral else DriveGreen,
                                    contentColor = SpiderMap.Ink,
                                    disabledContainerColor = if (command == null) StopCoral else DriveGreen,
                                    disabledContentColor = SpiderMap.Ink,
                                ),
                            ) { PixelDriveSymbol(command) }
                        }
                    }
                }
            }
        }
}

@Composable
private fun PixelDriveSymbol(command: ManualCommand?) {
    Canvas(Modifier.size(22.dp)) {
        val unit = size.width / 11f
        fun pixel(x: Int, y: Int) = drawRect(SpiderMap.Ink,
            topLeft = Offset(x * unit, y * unit), size = androidx.compose.ui.geometry.Size(unit, unit))
        if (command == null) {
            for (x in 3..7) for (y in 3..7) pixel(x, y)
        } else {
            val vector = when (command) {
                ManualCommand.FORWARD -> 0 to -1
                ManualCommand.REVERSE -> 0 to 1
                ManualCommand.LEFT -> -1 to 0
                ManualCommand.RIGHT -> 1 to 0
                ManualCommand.FORWARD_LEFT -> -1 to -1
                ManualCommand.FORWARD_RIGHT -> 1 to -1
                ManualCommand.BACK_LEFT -> -1 to 1
                ManualCommand.BACK_RIGHT -> 1 to 1
            }
            val (dx, dy) = vector
            for (step in 0..4) pixel(5 + dx * step, 5 + dy * step)
            val tipX = 5 + dx * 4
            val tipY = 5 + dy * 4
            if (dx == 0) {
                pixel(tipX - 2, tipY - dy * 2)
                pixel(tipX + 2, tipY - dy * 2)
                pixel(tipX - 1, tipY - dy)
                pixel(tipX + 1, tipY - dy)
            } else if (dy == 0) {
                pixel(tipX - dx * 2, tipY - 2)
                pixel(tipX - dx * 2, tipY + 2)
                pixel(tipX - dx, tipY - 1)
                pixel(tipX - dx, tipY + 1)
            } else {
                pixel(tipX - dx * 2, tipY)
                pixel(tipX, tipY - dy * 2)
                pixel(tipX - dx, tipY)
                pixel(tipX, tipY - dy)
            }
        }
    }
}
