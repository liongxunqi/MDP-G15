package com.example.mdp.ui.theme

import android.os.Build
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

// Spider-Verse palette: violet-black space, portal orange, Miles red, magenta and electric blue.
private val DarkColorScheme = darkColorScheme(
    primary = PortalOrange,
    onPrimary = Color(0xFF2B0B00),
    primaryContainer = Color(0xFF7A2410),
    onPrimaryContainer = Color(0xFFFFDCC2),
    secondary = HotMagenta,
    onSecondary = Color(0xFF3D0024),
    secondaryContainer = Color(0xFF5E1247),
    onSecondaryContainer = Color(0xFFFFD6EC),
    tertiary = ElectricBlue,
    onTertiary = Color(0xFF00105C),
    tertiaryContainer = Color(0xFF26338F),
    onTertiaryContainer = Color(0xFFDDE1FF),
    error = Color(0xFFFF4D5E),
    onError = Color(0xFF3F0009),
    errorContainer = Color(0xFF8C1027),
    onErrorContainer = Color(0xFFFFDADD),
    background = VoidPurple,
    onBackground = Color(0xFFFBEAF6),
    surface = Color(0xFF1C0A2E),
    onSurface = Color(0xFFFBEAF6),
    surfaceContainer = Color(0xFF27103E),
    surfaceContainerHigh = Color(0xFF32154D),
    surfaceVariant = Color(0xFF3B1854),
    onSurfaceVariant = Color(0xFFE6BEDD),
    outline = Color(0xFFB46CA8),
    outlineVariant = Color(0xFF6B2C70),
)

private val LightColorScheme = lightColorScheme(
    primary = Color(0xFFB4410A),
    onPrimary = Color.White,
    primaryContainer = Color(0xFFFFDCC2),
    onPrimaryContainer = Color(0xFF4A1500),
    secondary = Color(0xFFB0186E),
    onSecondary = Color.White,
    secondaryContainer = Color(0xFFFFD6EC),
    onSecondaryContainer = Color(0xFF3D0024),
    tertiary = Color(0xFF3448C5),
    onTertiary = Color.White,
    error = Color(0xFFBA1A2A),
    errorContainer = Color(0xFFFFDADD),
    onErrorContainer = Color(0xFF3F0009),
    background = Color(0xFFFFF3F8),
    onBackground = Color(0xFF2A0F2E),
    surface = Color(0xFFFFFBFD),
    onSurface = Color(0xFF2A0F2E),
    surfaceContainer = Color(0xFFF8E6F1),
    surfaceVariant = Color(0xFFF3DCEB),
    onSurfaceVariant = Color(0xFF5E3A57),
    outlineVariant = Color(0xFFE0BCD3),
)

@Composable
fun MdpTheme(
    // The Spider-Verse look is a dark theme, so it is used regardless of the system setting.
    darkTheme: Boolean = true,
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