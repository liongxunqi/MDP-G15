package com.example.mdp.ui.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Shapes
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import com.mdp.g15.arena.presentation.SpiderMap

private val SpiderColorScheme = darkColorScheme(
    primary = SpiderMap.PanelBlue, onPrimary = SpiderMap.Ink,
    secondary = Color(0xFF5FD27A), onSecondary = SpiderMap.Ink,
    tertiary = Color(0xFFFFB547), onTertiary = SpiderMap.Ink,
    error = Color(0xFFF2735C), onError = SpiderMap.Ink,
    background = SpiderMap.Backdrop, onBackground = SpiderMap.OnBackdrop,
    surface = SpiderMap.PanelBlue, onSurface = SpiderMap.Ink,
    surfaceContainer = SpiderMap.PanelBlue,
    surfaceContainerHigh = SpiderMap.PanelEdge,
    surfaceVariant = SpiderMap.PanelEdge,
    onSurfaceVariant = SpiderMap.Ink,
    outline = SpiderMap.Ink, outlineVariant = SpiderMap.FrameDark,
)

/** The controller always uses the Spider Tracker palette, including dialogs and tabs. */
@Suppress("UNUSED_PARAMETER")
@Composable
fun MdpTheme(
    darkTheme: Boolean = true,
    dynamicColor: Boolean = false,
    content: @Composable () -> Unit,
) {
    MaterialTheme(
        colorScheme = SpiderColorScheme,
        typography = Typography,
        shapes = Shapes(
            small = SpiderMap.ButtonShape,
            medium = SpiderMap.PanelShape,
            large = SpiderMap.PanelShape,
        ),
        content = content,
    )
}
