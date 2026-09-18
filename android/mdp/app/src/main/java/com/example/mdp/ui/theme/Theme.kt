package com.example.mdp.ui.theme

import android.os.Build
import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.dynamicDarkColorScheme
import androidx.compose.material3.dynamicLightColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.graphics.Color
import androidx.compose.material3.Shapes
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.ui.unit.dp

private val DarkColorScheme = darkColorScheme(
    primary = Color(0xFF75D8C6),
    onPrimary = Color(0xFF00382F),
    primaryContainer = Color(0xFF164D45),
    onPrimaryContainer = Color(0xFFA2F2E1),
    secondary = Color(0xFFA9C9DD),
    secondaryContainer = Color(0xFF284452),
    onSecondaryContainer = Color(0xFFD5EAF5),
    background = Color(0xFF101B24),
    surface = Color(0xFF14232E),
    surfaceContainer = Color(0xFF1B2D39),
    surfaceVariant = Color(0xFF304550),
    onBackground = Color(0xFFE0EBF1),
    onSurface = Color(0xFFE0EBF1),
    onSurfaceVariant = Color(0xFFBACCD6),
    outlineVariant = Color(0xFF3A515E),
)

private val LightColorScheme = lightColorScheme(
    primary = Color(0xFF006B5E),
    onPrimary = Color.White,
    primaryContainer = Color(0xFFD3F3EB),
    onPrimaryContainer = Color(0xFF0B453B),
    secondary = Color(0xFF42657C),
    secondaryContainer = Color(0xFFDCEAF2),
    onSecondaryContainer = Color(0xFF264C63),
    background = Color(0xFFF0F5F8),
    surface = Color(0xFFFCFEFF),
    surfaceContainer = Color(0xFFE7EFF4),
    surfaceVariant = Color(0xFFE2EBF0),
    onBackground = Color(0xFF152F40),
    onSurface = Color(0xFF152F40),
    onSurfaceVariant = Color(0xFF4B6371),
    outlineVariant = Color(0xFFC6D6DF),
)

@Composable
fun MdpTheme(
    darkTheme: Boolean = isSystemInDarkTheme(),
    // Dynamic color is available on Android 12+
    dynamicColor: Boolean = false,
    content: @Composable () -> Unit
) {
    val colorScheme = when {
        dynamicColor && Build.VERSION.SDK_INT >= Build.VERSION_CODES.S -> {
            val context = LocalContext.current
            if (darkTheme) dynamicDarkColorScheme(context) else dynamicLightColorScheme(context)
        }

        darkTheme -> DarkColorScheme
        else -> LightColorScheme
    }

    MaterialTheme(
        colorScheme = colorScheme,
        typography = Typography,
        shapes = Shapes(
            small = RoundedCornerShape(10.dp),
            medium = RoundedCornerShape(16.dp),
            large = RoundedCornerShape(22.dp),
        ),
        content = content
    )
}