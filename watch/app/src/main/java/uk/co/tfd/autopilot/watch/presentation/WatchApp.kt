package uk.co.tfd.autopilot.watch.presentation

import androidx.compose.runtime.*
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.wear.compose.navigation.SwipeDismissableNavHost
import androidx.wear.compose.navigation.composable
import androidx.wear.compose.navigation.rememberSwipeDismissableNavController
import uk.co.tfd.autopilot.watch.ble.ConnectionState
import uk.co.tfd.autopilot.watch.viewmodel.AutopilotViewModel

@Composable
fun WatchApp(vm: AutopilotViewModel = viewModel()) {
    val navController = rememberSwipeDismissableNavController()
    val connectionState by vm.connectionState.collectAsStateWithLifecycle()
    val apState by vm.state.collectAsStateWithLifecycle()
    val scanItems by vm.scanItems.collectAsStateWithLifecycle()

    // Navigate based on connection state
    LaunchedEffect(connectionState) {
        when (connectionState) {
            ConnectionState.CONNECTED -> {
                if (navController.currentDestination?.route != "main") {
                    navController.navigate("main") {
                        popUpTo("scan") { inclusive = true }
                    }
                }
            }
            ConnectionState.DISCONNECTED -> {
                if (navController.currentDestination?.route == "main") {
                    navController.navigate("scan") {
                        popUpTo("main") { inclusive = true }
                    }
                }
            }
            else -> {} // scanning/connecting — stay on scan screen
        }
    }

    AutopilotWatchTheme {
        SwipeDismissableNavHost(
            navController = navController,
            startDestination = "scan",
        ) {
            composable("scan") {
                ScanScreen(
                    scanItems = scanItems,
                    connectionState = connectionState,
                    onScan = { vm.startScan() },
                    onConnect = { index -> vm.connectItem(index) },
                )
            }

            composable("main") {
                MainScreen(
                    state = apState,
                    onAdjust = { delta -> vm.adjustTarget(delta) },
                    onStandby = { vm.standby() },
                    onCycleMode = { vm.cycleMode() },
                    onCyclePilot = { vm.cyclePilotType() },
                )
            }
        }
    }
}
