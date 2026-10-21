package com.example.mdp.ui.theme

import androidx.compose.material3.Typography
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp
import com.mdp.g15.arena.presentation.SpiderMap

val Typography = Typography(
    bodyLarge = TextStyle(
        fontFamily = SpiderMap.PixelFont,
        fontWeight = FontWeight.Normal,
        fontSize = 9.sp,
        lineHeight = 14.sp,
    ),
    bodyMedium = SpiderMap.Body,
    bodySmall = SpiderMap.pixel(7.sp),
    titleLarge = SpiderMap.pixel(12.sp),
    titleMedium = SpiderMap.pixel(10.sp),
    titleSmall = SpiderMap.pixel(9.sp),
    headlineSmall = SpiderMap.pixel(12.sp),
    labelLarge = SpiderMap.pixel(8.sp),
    labelMedium = SpiderMap.pixel(7.sp),
    labelSmall = SpiderMap.pixel(6.sp),
)
