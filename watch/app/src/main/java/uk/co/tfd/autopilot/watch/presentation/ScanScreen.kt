package uk.co.tfd.autopilot.watch.presentation

import androidx.compose.foundation.layout.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.wear.compose.foundation.lazy.ScalingLazyColumn
import androidx.wear.compose.foundation.lazy.itemsIndexed
import androidx.wear.compose.material.*
import uk.co.tfd.autopilot.watch.ble.ConnectionState
import uk.co.tfd.autopilot.watch.ble.ScanItem

@Composable
fun ScanScreen(
    scanItems: List<ScanItem>,
    connectionState: ConnectionState,
    onScan: () -> Unit,
    onConnect: (Int) -> Unit,
) {
    LaunchedEffect(Unit) {
        onScan()
    }

    ScalingLazyColumn(
        modifier = Modifier.fillMaxSize(),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        item {
            Text(
                text = when (connectionState) {
                    ConnectionState.SCANNING -> "Scanning..."
                    ConnectionState.CONNECTING -> "Connecting..."
                    else -> "Pilot"
                },
                style = MaterialTheme.typography.title3,
                textAlign = TextAlign.Center,
            )
        }

        if (scanItems.isEmpty() && connectionState == ConnectionState.SCANNING) {
            item {
                CircularProgressIndicator(
                    modifier = Modifier.size(24.dp),
                    strokeWidth = 2.dp,
                )
            }
        }

        itemsIndexed(scanItems) { index, item ->
            Chip(
                modifier = Modifier.fillMaxWidth(0.9f),
                onClick = { onConnect(index) },
                label = {
                    Text(item.name)
                },
                secondaryLabel = {
                    Text(item.detail)
                },
                colors = ChipDefaults.secondaryChipColors(),
            )
        }

        if (connectionState == ConnectionState.DISCONNECTED) {
            item {
                Spacer(modifier = Modifier.height(8.dp))
                CompactChip(
                    onClick = onScan,
                    label = { Text("Rescan") },
                    colors = ChipDefaults.primaryChipColors(),
                )
            }
        }
    }
}
