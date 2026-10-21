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
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.foundation.background
import com.mdp.g15.arena.presentation.SpiderMap
import com.mdp.g15.arena.presentation.SpiderMapPanel
import com.mdp.g15.arena.presentation.PixelButton
import com.mdp.g15.arena.presentation.PixelTone
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

    Column(Modifier.fillMaxSize().background(SpiderMap.Backdrop).padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(5.dp)) {
        Text("Command logs", style = SpiderMap.pixel(10.sp, SpiderMap.OnBackdrop))
        Text(
            "Last 500 session events • newest first. TX means written, not acknowledged by the robot.",
            style = SpiderMap.pixel(6.sp, SpiderMap.OnBackdrop),
        )
        Row(Modifier.horizontalScroll(rememberScrollState()), horizontalArrangement = Arrangement.spacedBy(5.dp)) {
            PixelButton(if (frozen == null) "Pause display" else "Resume display",
                { frozen = if (frozen == null) entries.toList() else null })
            PixelButton("Copy", { clipboard.setText(AnnotatedString(displayed.joinToString("\n\n", transform = ::format))) },
                enabled = displayed.isNotEmpty(), tone = PixelTone.GREEN)
            PixelButton("Clear", { onClear(); if (frozen != null) frozen = emptyList() }, tone = PixelTone.CORAL)
        }
        Row(Modifier.horizontalScroll(rememberScrollState()), horizontalArrangement = Arrangement.spacedBy(4.dp)) {
            PixelButton("All", { filter = null }, selected = filter == null)
            CommandLog.Kind.entries.forEach { kind ->
                PixelButton(kind.name, { filter = kind }, selected = filter == kind)
            }
        }
        Text(if (frozen == null) "Live • ${displayed.size} entries" else "Display paused • capture continues",
            style = SpiderMap.pixel(6.sp, SpiderMap.OnBackdrop))
        if (displayed.isEmpty()) Text("No matching events.", style = SpiderMap.pixel(7.sp, SpiderMap.OnBackdrop))
        LazyColumn(Modifier.weight(1f), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            items(displayed, key = { it.id }) { entry ->
                SpiderMapPanel(Modifier.fillMaxWidth()) {
                    SelectionContainer {
                        Text(
                            format(entry), modifier = Modifier.padding(6.dp),
                            style = SpiderMap.pixel(6.sp),
                            color = if (entry.kind == CommandLog.Kind.ERROR) SpiderMap.Warning else SpiderMap.Text,
                        )
                    }
                }
            }
        }
    }
}
