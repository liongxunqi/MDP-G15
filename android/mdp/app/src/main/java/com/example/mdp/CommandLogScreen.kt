package com.example.mdp

import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.selection.SelectionContainer
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalClipboardManager
import androidx.compose.ui.text.AnnotatedString
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

@Composable
fun CommandLogScreen(entries: List<CommandLog.Entry>, onClear: () -> Unit) {
    var frozen by remember { mutableStateOf<List<CommandLog.Entry>?>(null) }
    var filter by remember { mutableStateOf<CommandLog.Kind?>(null) }
    val displayed = (frozen ?: entries).filter { filter == null || it.kind == filter }.asReversed()
    val clipboard = LocalClipboardManager.current
    val timeFormat = remember { SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault()) }
    fun format(entry: CommandLog.Entry) =
        "#${entry.id} ${timeFormat.format(Date(entry.timestamp))} ${entry.kind}\n${entry.message}"

    Column(Modifier.fillMaxSize().padding(16.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
        Text("Command logs", style = MaterialTheme.typography.headlineSmall)
        Text(
            "Last 500 session events • newest first. TX means written, not acknowledged by the robot.",
            style = MaterialTheme.typography.bodySmall,
        )
        Row(Modifier.horizontalScroll(rememberScrollState()), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = { frozen = if (frozen == null) entries.toList() else null }) {
                Text(if (frozen == null) "Pause display" else "Resume display")
            }
            OutlinedButton(onClick = { clipboard.setText(AnnotatedString(displayed.joinToString("\n\n", transform = ::format))) }, enabled = displayed.isNotEmpty()) {
                Text("Copy")
            }
            OutlinedButton(onClick = { onClear(); if (frozen != null) frozen = emptyList() }) { Text("Clear") }
        }
        Row(Modifier.horizontalScroll(rememberScrollState()), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            FilterChip(selected = filter == null, onClick = { filter = null }, label = { Text("All") })
            CommandLog.Kind.entries.forEach { kind ->
                FilterChip(selected = filter == kind, onClick = { filter = kind }, label = { Text(kind.name) })
            }
        }
        Text(if (frozen == null) "Live • ${displayed.size} entries" else "Display paused • capture continues", style = MaterialTheme.typography.labelMedium)
        if (displayed.isEmpty()) Text("No matching events.")
        LazyColumn(Modifier.weight(1f), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            items(displayed, key = { it.id }) { entry ->
                Card(Modifier.fillMaxWidth(), colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceContainer)) {
                    SelectionContainer {
                        Text(
                            format(entry), modifier = Modifier.padding(12.dp),
                            fontFamily = FontFamily.Monospace, style = MaterialTheme.typography.bodySmall,
                            color = if (entry.kind == CommandLog.Kind.ERROR) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.onSurface,
                        )
                    }
                }
            }
        }
    }
}
