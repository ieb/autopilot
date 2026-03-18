package uk.co.tfd.autopilot.watch.ble

import kotlinx.coroutines.flow.StateFlow

/**
 * Common interface for autopilot BLE communication.
 * Implemented by AutopilotBleManager (real BLE) and FakeBleClient (simulator).
 */
interface PilotBleClient {
    val state: StateFlow<AutopilotState>
    val connectionState: StateFlow<ConnectionState>

    /** Items to display on the scan screen. Null name = use display text directly. */
    val scanItems: StateFlow<List<ScanItem>>

    fun startScan()
    fun stopScan()
    fun connectItem(index: Int)
    fun disconnect()
    fun sendCommand(data: ByteArray)
    fun destroy()
}

/** Scannable device — abstracted so fake client doesn't need android.bluetooth types. */
data class ScanItem(
    val name: String,
    val detail: String,
)
