package uk.co.tfd.autopilot.watch.ble

import android.util.Log
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

/**
 * Fake BLE client for emulator testing. Simulates a connected autopilot
 * with a drifting heading target so the UI has live-updating data.
 */
class FakeBleClient : PilotBleClient {

    companion object {
        private const val TAG = "FakeBLE"
    }

    private val _state = MutableStateFlow(AutopilotState())
    override val state: StateFlow<AutopilotState> = _state

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    override val connectionState: StateFlow<ConnectionState> = _connectionState

    private val _scanItems = MutableStateFlow<List<ScanItem>>(emptyList())
    override val scanItems: StateFlow<List<ScanItem>> = _scanItems

    private val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    private var stateJob: Job? = null

    // Simulated autopilot state
    private var simMode = PilotMode.STANDBY
    private var simType = PilotType.PD
    private var simTarget = 275.0f
    private var simFault = 0
    private var simClutch = false

    override fun startScan() {
        _connectionState.value = ConnectionState.SCANNING
        _scanItems.value = emptyList()

        // Simulate discovery after a short delay
        scope.launch {
            delay(800)
            _scanItems.value = listOf(
                ScanItem(name = "Pilot (fake)", detail = "Simulated device"),
            )
        }

        // Auto-stop scan after 10s
        scope.launch {
            delay(10_000)
            if (_connectionState.value == ConnectionState.SCANNING) {
                _connectionState.value = ConnectionState.DISCONNECTED
            }
        }
    }

    override fun stopScan() {
        if (_connectionState.value == ConnectionState.SCANNING) {
            _connectionState.value = ConnectionState.DISCONNECTED
        }
    }

    override fun connectItem(index: Int) {
        _connectionState.value = ConnectionState.CONNECTING
        scope.launch {
            delay(500) // simulate connection delay
            simMode = PilotMode.STANDBY
            simTarget = 275.0f
            simClutch = false
            simFault = 0
            _connectionState.value = ConnectionState.CONNECTED
            Log.d(TAG, "Connected to fake device")
            startStateUpdates()
        }
    }

    override fun disconnect() {
        stateJob?.cancel()
        _connectionState.value = ConnectionState.DISCONNECTED
        _state.value = AutopilotState()
        Log.d(TAG, "Disconnected")
    }

    override fun sendCommand(data: ByteArray) {
        if (data.isEmpty()) return
        when (data[0].toInt() and 0xFF) {
            0x01 -> { // set_mode
                if (data.size >= 4) {
                    val newMode = PilotMode.fromValue(data[1].toInt() and 0xFF)
                    val raw = (data[2].toInt() and 0xFF) or ((data[3].toInt() and 0xFF) shl 8)
                    val signed = if (raw > 32767) raw - 65536 else raw
                    simTarget = signed / 10f
                    simMode = newMode
                    // Wrap wind angles to ±180
                    if (simMode == PilotMode.WIND_AWA || simMode == PilotMode.WIND_TWA) {
                        while (simTarget > 180f) simTarget -= 360f
                        while (simTarget < -180f) simTarget += 360f
                    }
                    simClutch = simMode != PilotMode.STANDBY
                    Log.d(TAG, "CMD set_mode: $simMode target=$simTarget")
                }
            }
            0x02 -> { // adjust_target
                if (data.size >= 3) {
                    val raw = (data[1].toInt() and 0xFF) or ((data[2].toInt() and 0xFF) shl 8)
                    val signed = if (raw > 32767) raw - 65536 else raw
                    val delta = signed / 10f
                    simTarget += delta
                    if (simMode == PilotMode.COMPASS) {
                        while (simTarget < 0) simTarget += 360f
                        while (simTarget >= 360f) simTarget -= 360f
                    } else if (simMode == PilotMode.WIND_AWA || simMode == PilotMode.WIND_TWA) {
                        while (simTarget > 180f) simTarget -= 360f
                        while (simTarget < -180f) simTarget += 360f
                    }
                    Log.d(TAG, "CMD adjust: delta=$delta -> target=$simTarget")
                }
            }
            0x03 -> { // set_pilot_type
                if (data.size >= 2) {
                    simType = PilotType.fromValue(data[1].toInt() and 0xFF)
                    Log.d(TAG, "CMD set_pilot_type: $simType")
                }
            }
            0x04 -> { // standby
                simMode = PilotMode.STANDBY
                simClutch = false
                Log.d(TAG, "CMD standby")
            }
        }
        pushState()
    }

    override fun destroy() {
        stateJob?.cancel()
        scope.cancel()
    }

    private fun startStateUpdates() {
        stateJob?.cancel()
        stateJob = scope.launch {
            var tick = 0
            while (isActive) {
                // Trigger a transient fault every 30 seconds for 3 ticks
                simFault = if (tick % 30 in 0..2 && tick >= 30) 1 else 0

                pushState()
                tick++
                delay(1000) // 1 Hz like the real BLE characteristic
            }
        }
    }

    private fun pushState() {
        _state.value = AutopilotState(
            pilotMode = simMode,
            pilotType = simType,
            targetValue = simTarget,
            faultCode = simFault,
            clutchEngaged = simClutch,
        )
    }
}
