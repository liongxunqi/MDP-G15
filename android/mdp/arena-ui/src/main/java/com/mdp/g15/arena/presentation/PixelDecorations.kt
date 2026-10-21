package com.mdp.g15.arena.presentation

import androidx.compose.foundation.Canvas
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color

/** Small, decorative sprites drawn on the same pixel grid as the Spider Tracker controls. */
@Composable
fun PixelMaskIcon(modifier: Modifier = Modifier) {
    PixelSprite(
        listOf(
            ".....KKKKKK.....",
            "...KKRRRRRRKK...",
            "..KRRRRRRRRRRK..",
            ".KRRRRRRRRRRRRK.",
            ".KRRWWRRRRWWRRK.",
            "KRRWWWWRRWWWWRRK",
            "KRRRWWWRRWWWRRRK",
            "KRRRRWWRRWWRRRRK",
            "KRRRRRRRRRRRRRRK",
            ".KRRRRRRRRRRRRK.",
            "..KRRRRRRRRRRK..",
            "...KKRRRRRRKK...",
            ".....KKKKKK.....",
        ), modifier,
    )
}

@Composable
fun PixelSpiderIcon(modifier: Modifier = Modifier) {
    PixelSprite(
        listOf(
            "................",
            "..K..........K..",
            "..KK........KK..",
            "...KK..KK..KK...",
            "....KK.KK.KK....",
            "K....KKRRKK....K",
            "KKKKKRRRRRRKKKKK",
            "..KKRRRRRRRRKK..",
            "..KKRRRRRRRRKK..",
            "KKKKKRRRRRRKKKKK",
            "K....KKRRKK....K",
            "....KK.KK.KK....",
            "...KK..KK..KK...",
            "..KK........KK..",
            "..K..........K..",
            "................",
        ), modifier,
    )
}

@Composable
fun PixelWebIcon(modifier: Modifier = Modifier) {
    PixelSprite(
        listOf(
            ".......K.......",
            "...K...K...K...",
            "....K..K..K....",
            ".K...K.K.K...K.",
            "..K...KKK...K..",
            "...K.K.K.K.K...",
            "KKKKKKKKKKKKKKK",
            "......KRK......",
            "KKKKKKKKKKKKKKK",
            "...K.K.K.K.K...",
            "..K...KKK...K..",
            ".K...K.K.K...K.",
            "....K..K..K....",
            "...K...K...K...",
            ".......K.......",
        ), modifier,
    )
}

@Composable
private fun PixelSprite(rows: List<String>, modifier: Modifier) {
    Canvas(modifier) {
        val side = rows.size.coerceAtLeast(rows.maxOf { it.length })
        val cell = minOf(size.width, size.height) / side
        val left = (size.width - cell * side) / 2f
        val top = (size.height - cell * side) / 2f
        rows.forEachIndexed { y, row ->
            row.forEachIndexed { x, pixel ->
                val color = when (pixel) {
                    'K' -> SpiderMap.Ink
                    'R' -> Color(0xFFF2735C)
                    'W' -> Color(0xFFE6F4FF)
                    else -> null
                }
                if (color != null) drawRect(color,
                    topLeft = Offset(left + x * cell, top + y * cell),
                    size = Size(cell, cell))
            }
        }
    }
}
