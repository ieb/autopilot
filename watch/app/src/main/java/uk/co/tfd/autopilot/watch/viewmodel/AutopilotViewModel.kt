package uk.co.tfd.autopilot.watch.viewmodel

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import uk.co.tfd.autopilot.watch.BuildConfig
import uk.co.tfd.autopilot.watch.ble.*
import kotlinx.coroutines.flow.StateFlow

class AutopilotViewModel(application: Application) : AndroidViewModel(application) {

    private val client: PilotBleClient = if (BuildConfig.FAKE_BLE) {
        FakeBleClient()
    } else {
        AutopilotBleManager(application)
    }

    val state: StateFlow<AutopilotState> = client.state
    val connectionState: StateFlow<ConnectionState> = client.connectionState
    val scanItems: StateFlow<List<ScanItem>> = client.scanItems

    fun startScan() = client.startScan()
    fun stopScan() = client.stopScan()
    fun connectItem(index: Int) = client.connectItem(index)
    fun disconnect() = client.disconnect()

    fun standby() {
        client.sendCommand(BleCommand.standby())
    }

    fun setMode(mode: PilotMode, target: Float) {
        client.sendCommand(BleCommand.setMode(mode, target))
    }

    fun adjustTarget(delta: Float) {
        client.sendCommand(BleCommand.adjustTarget(delta))
    }

    fun setPilotType(type: PilotType) {
        client.sendCommand(BleCommand.setPilotType(type))
    }

    fun cyclePilotType() {
        val next = state.value.pilotType.next()
        setPilotType(next)
    }

    /** Cycle through HDG → AWA → TWA. From standby, enters HDG. */
    fun cycleMode() {
        val target = state.value.targetValue
        val nextMode = when (state.value.pilotMode) {
            PilotMode.STANDBY -> PilotMode.COMPASS
            PilotMode.COMPASS -> PilotMode.WIND_AWA
            PilotMode.WIND_AWA -> PilotMode.WIND_TWA
            PilotMode.WIND_TWA -> PilotMode.COMPASS
            else -> PilotMode.COMPASS
        }
        setMode(nextMode, target)
    }

    override fun onCleared() {
        super.onCleared()
        client.destroy()
    }
}
