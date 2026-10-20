package com.mdp.g15.arena.presentation

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.RowScope
import androidx.compose.foundation.layout.defaultMinSize
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.LocalContentColor
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

/**
 * Spider-Society map look for the arena: navy blueprint panels in a thick sky-blue frame,
 * chunky outlined buttons in green / orange / coral, and a monospace "pixel" type.
 * Colours here mirror res/values/colors.xml, which the grid view draws with.
 */
internal object SpiderMap {
    val Navy = Color(0xFF0B2350)
    val NavyDeep = Color(0xFF071838)
    val Frame = Color(0xFF29A9F0)
    val FrameDark = Color(0xFF0E5FA8)
    val Ink = Color(0xFF0A1630)
    val Text = Color(0xFFE6F4FF)
    val TextDim = Color(0xFF9CC7EE)
    val Warning = Color(0xFFFF8B78)

    val Font = FontFamily.Monospace
    val Title = TextStyle(fontFamily = Font, fontWeight = FontWeight.Bold, fontSize = 18.sp,
        letterSpacing = 1.sp, color = Text)
    val Label = TextStyle(fontFamily = Font, fontWeight = FontWeight.Bold, fontSize = 12.sp,
        letterSpacing = 0.5.sp, color = TextDim)
    val Body = TextStyle(fontFamily = Font, fontSize = 13.sp, color = Text)

    val PanelShape = RoundedCornerShape(6.dp)
    val ButtonShape = RoundedCornerShape(10.dp)
}

/** Button fill and label colours, taken from the reference's three button families. */
internal enum class PixelTone(val fill: Color, val label: Color) {
    GREEN(Color(0xFF5FD27A), Color(0xFF0E4A26)),
    ORANGE(Color(0xFFFFB547), Color(0xFF5A2A00)),
    CORAL(Color(0xFFF2735C), Color(0xFF3D0D05)),
}

/** A navy map panel inside the thick blue frame. */
@Composable
internal fun SpiderMapPanel(
    modifier: Modifier = Modifier,
    content: @Composable ColumnScope.() -> Unit,
) {
    Card(
        modifier = modifier,
        shape = SpiderMap.PanelShape,
        colors = CardDefaults.cardColors(containerColor = SpiderMap.Navy, contentColor = SpiderMap.Text),
        border = BorderStroke(5.dp, SpiderMap.Frame),
    ) {
        CompositionLocalProvider(LocalContentColor provides SpiderMap.Text) { content() }
    }
}

/**
 * Chunky outlined button. Disabled buttons go slate, like the reference's greyed "3D VIEW",
 * so it is obvious at a glance which actions are available.
 */
@Composable
internal fun PixelButton(
    text: String,
    onClick: () -> Unit,
    modifier: Modifier = Modifier,
    tone: PixelTone = PixelTone.ORANGE,
    enabled: Boolean = true,
    selected: Boolean = true,
    contentPadding: PaddingValues = PaddingValues(horizontal = 12.dp, vertical = 6.dp),
) {
    // An unselected chip is the same shape, hollow, so a set of them reads as one choice.
    val fill = if (selected) tone.fill else SpiderMap.NavyDeep
    val label = if (selected) tone.label else tone.fill
    Button(
        onClick = onClick,
        enabled = enabled,
        modifier = modifier.defaultMinSize(minHeight = 40.dp),
        shape = SpiderMap.ButtonShape,
        border = BorderStroke(3.dp, if (enabled) SpiderMap.Ink else SpiderMap.FrameDark),
        contentPadding = contentPadding,
        colors = ButtonDefaults.buttonColors(
            containerColor = fill,
            contentColor = label,
            disabledContainerColor = Color(0xFF3E5F7A),
            disabledContentColor = Color(0xFF7FA3BF),
        ),
    ) {
        Text(text, fontFamily = SpiderMap.Font, fontWeight = FontWeight.Bold, fontSize = 13.sp)
    }
}

/** Row-scoped convenience so button rows share width evenly. */
@Composable
internal fun RowScope.PixelButtonCell(
    text: String,
    onClick: () -> Unit,
    tone: PixelTone,
    enabled: Boolean = true,
) = PixelButton(text, onClick, Modifier.weight(1f), tone, enabled)
