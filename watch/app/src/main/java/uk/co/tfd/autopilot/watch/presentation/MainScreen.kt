package uk.co.tfd.autopilot.watch.presentation

import android.content.Context
import android.os.VibrationEffect
import android.os.Vibrator
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.onSizeChanged
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.wear.compose.material.*
import uk.co.tfd.autopilot.watch.ble.AutopilotState
import uk.co.tfd.autopilot.watch.ble.PilotMode
import uk.co.tfd.autopilot.watch.ble.PilotType
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

private val BTN = 46.dp
private val PILL_WIDTH = 92.dp // 2x button = full circle each end + straight middle

@Composable
fun MainScreen(
    state: AutopilotState,
    onAdjust: (Float) -> Unit,
    onStandby: () -> Unit,
    onCycleMode: () -> Unit,
    onCyclePilot: () -> Unit,
) {
    val context = LocalContext.current
    val vibrator = remember { context.getSystemService(Context.VIBRATOR_SERVICE) as Vibrator }
    val density = LocalDensity.current

    fun haptic() {
        vibrator.vibrate(VibrationEffect.createOneShot(30, VibrationEffect.DEFAULT_AMPLITUDE))
    }

    var lastFault by remember { mutableIntStateOf(0) }
    LaunchedEffect(state.faultCode) {
        if (state.faultCode != 0 && state.faultCode != lastFault) {
            vibrator.vibrate(VibrationEffect.createOneShot(200, VibrationEffect.DEFAULT_AMPLITUDE))
        }
        lastFault = state.faultCode
    }

    val isFault = state.faultCode != 0
    val bgColor = if (isFault) Color(0xFF330000) else Color.Black

    val activeModeLabel = when (state.pilotMode) {
        PilotMode.STANDBY -> "STANDBY"
        PilotMode.COMPASS -> "HDG"
        PilotMode.WIND_AWA -> "AWA"
        PilotMode.WIND_TWA -> "TWA"
        else -> state.pilotMode.displayName
    }

    val targetText = when (state.pilotMode) {
        PilotMode.STANDBY -> "---"
        PilotMode.COMPASS -> String.format("%.0f\u00B0", state.targetValue)
        else -> {
            val side = if (state.targetValue >= 0) "S" else "P"
            val angle = Math.abs(state.targetValue)
            String.format("%s %.0f\u00B0", side, angle)
        }
    }

    val pilotShortName = when (state.pilotType) {
        PilotType.PD -> "PD"
        PilotType.PID -> "PID"
        PilotType.SMOOTH -> "Smth"
        PilotType.ADAPTIVE -> "Adpt"
    }

    val primaryColor = MaterialTheme.colors.primary
    val errorColor = MaterialTheme.colors.error
    val surfaceColor = MaterialTheme.colors.surface
    val onSurfaceVar = MaterialTheme.colors.onSurfaceVariant
    val dark = Color(0xFF2A2A2A)

    var sizePx by remember { mutableIntStateOf(0) }

    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(bgColor)
            .onSizeChanged { sizePx = it.width },
    ) {
        if (sizePx > 0) {
            val btnPx = with(density) { BTN.toPx() }
            val pillWPx = with(density) { PILL_WIDTH.toPx() }
            val centerPx = sizePx / 2f
            val radiusPx = centerPx - btnPx / 2f - with(density) { 4.dp.toPx() }

            // Helper: circle center at given angle on the ring
            fun ringPos(angleDeg: Float): Pair<Float, Float> {
                val rad = Math.toRadians(angleDeg.toDouble())
                return Pair(
                    centerPx + radiusPx * sin(rad).toFloat(),
                    centerPx - radiusPx * cos(rad).toFloat(),
                )
            }

            // ---- Circle buttons ----
            data class CBtn(val label: String, val angle: Float, val bg: Color, val fg: Color, val onClick: () -> Unit)
            val circleButtons = listOf(
                CBtn(pilotShortName, 0f, surfaceColor, onSurfaceVar) { haptic(); onCyclePilot() },
                CBtn("+1", 45f, dark, Color.White) { haptic(); onAdjust(1f) },
                CBtn("+10", 90f, dark, Color.White) { haptic(); onAdjust(10f) },
                CBtn("-10", 270f, dark, Color.White) { haptic(); onAdjust(-10f) },
                CBtn("-1", 315f, dark, Color.White) { haptic(); onAdjust(-1f) },
            )

            circleButtons.forEach { btn ->
                val (cx, cy) = ringPos(btn.angle)
                Box(
                    modifier = Modifier
                        .offset { IntOffset((cx - btnPx / 2f).roundToInt(), (cy - btnPx / 2f).roundToInt()) }
                        .size(BTN)
                        .clip(CircleShape)
                        .background(btn.bg)
                        .clickable { btn.onClick() },
                    contentAlignment = Alignment.Center,
                ) {
                    Text(
                        text = btn.label,
                        fontSize = 12.sp,
                        fontWeight = FontWeight.Bold,
                        color = btn.fg,
                        textAlign = TextAlign.Center,
                        maxLines = 1,
                        overflow = TextOverflow.Clip,
                    )
                }
            }

            // ---- STBY pill: outer (left) circle at 225°, extends right ----
            val (stbyOcx, stbyOcy) = ringPos(225f)
            // Pill left edge = outer circle left edge
            val stbyX = stbyOcx - btnPx / 2f
            val stbyY = stbyOcy - btnPx / 2f
            val stbyBg = if (state.pilotMode == PilotMode.STANDBY) errorColor else Color(0xFF4A1A1A)

            Box(
                modifier = Modifier
                    .offset { IntOffset(stbyX.roundToInt(), stbyY.roundToInt()) }
                    .width(PILL_WIDTH)
                    .height(BTN)
                    .clip(RoundedCornerShape(50))
                    .background(stbyBg)
                    .clickable { haptic(); onStandby() },
                contentAlignment = Alignment.Center,
            ) {
                Text(
                    text = "STBY",
                    fontSize = 13.sp,
                    fontWeight = FontWeight.Bold,
                    color = Color.White,
                    textAlign = TextAlign.Center,
                )
            }

            // ---- Auto pill: outer (right) circle at 135°, extends left ----
            val (autoOcx, autoOcy) = ringPos(135f)
            // Pill right edge = outer circle right edge, so left edge = right - pillWidth
            val autoX = autoOcx + btnPx / 2f - pillWPx
            val autoY = autoOcy - btnPx / 2f
            val autoBg = if (state.pilotMode != PilotMode.STANDBY) primaryColor else Color(0xFF1A3A4A)
            val autoFg = if (state.pilotMode != PilotMode.STANDBY) Color.Black else Color.White

            Box(
                modifier = Modifier
                    .offset { IntOffset(autoX.roundToInt(), autoY.roundToInt()) }
                    .width(PILL_WIDTH)
                    .height(BTN)
                    .clip(RoundedCornerShape(50))
                    .background(autoBg)
                    .clickable { haptic(); onCycleMode() },
                contentAlignment = Alignment.Center,
            ) {
                Text(
                    text = "Auto",
                    fontSize = 13.sp,
                    fontWeight = FontWeight.Bold,
                    color = autoFg,
                    textAlign = TextAlign.Center,
                )
            }
        }

        // ---- Center: mode + target ----
        Column(
            modifier = Modifier.align(Alignment.Center),
            horizontalAlignment = Alignment.CenterHorizontally,
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Text(
                    text = activeModeLabel,
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Bold,
                    color = when (state.pilotMode) {
                        PilotMode.STANDBY -> MaterialTheme.colors.error
                        else -> MaterialTheme.colors.primary
                    },
                )
                if (isFault) {
                    Spacer(modifier = Modifier.width(6.dp))
                    Text(
                        text = "F${state.faultCode}",
                        fontSize = 12.sp,
                        fontWeight = FontWeight.Bold,
                        color = MaterialTheme.colors.error,
                    )
                }
            }

            Text(
                text = targetText,
                fontSize = 36.sp,
                fontWeight = FontWeight.Bold,
                color = Color.White,
                textAlign = TextAlign.Center,
            )
        }
    }
}
