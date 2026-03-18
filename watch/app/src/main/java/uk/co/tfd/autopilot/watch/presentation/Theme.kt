package uk.co.tfd.autopilot.watch.presentation

import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.wear.compose.material.MaterialTheme
import androidx.wear.compose.material.Colors

private val WatchColors = Colors(
    primary = Color(0xFF4FC3F7),       // Light blue — visible at night
    primaryVariant = Color(0xFF0288D1),
    secondary = Color(0xFFFFB74D),     // Amber for warnings
    secondaryVariant = Color(0xFFF57C00),
    error = Color(0xFFEF5350),         // Red for faults/standby
    background = Color.Black,
    surface = Color(0xFF1A1A1A),
    onPrimary = Color.Black,
    onSecondary = Color.Black,
    onError = Color.White,
    onBackground = Color.White,
    onSurface = Color.White,
    onSurfaceVariant = Color(0xFFBDBDBD),
)

@Composable
fun AutopilotWatchTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colors = WatchColors,
        content = content,
    )
}
