package uk.co.tfd.autopilot.watch.ble

import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.Context
import android.os.ParcelUuid
import android.util.Log
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import java.util.UUID
import java.util.concurrent.ConcurrentLinkedQueue

@SuppressLint("MissingPermission")
class AutopilotBleManager(private val context: Context) : PilotBleClient {

    companion object {
        private const val TAG = "AutopilotBLE"
        val SERVICE_UUID: UUID = UUID.fromString("4f490000-b5a3-f393-e0a9-e50e24dcca9e")
        val STATE_CHAR_UUID: UUID = UUID.fromString("4f490001-b5a3-f393-e0a9-e50e24dcca9e")
        val COMMAND_CHAR_UUID: UUID = UUID.fromString("4f490002-b5a3-f393-e0a9-e50e24dcca9e")
        val FAULT_CHAR_UUID: UUID = UUID.fromString("4f490003-b5a3-f393-e0a9-e50e24dcca9e")
        val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        private const val MAX_RECONNECT_DELAY_MS = 10_000L
    }

    private val _state = MutableStateFlow(AutopilotState())
    override val state: StateFlow<AutopilotState> = _state

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    override val connectionState: StateFlow<ConnectionState> = _connectionState

    private val _scanResults = MutableStateFlow<List<ScanResult>>(emptyList())

    private val _scanItems = MutableStateFlow<List<ScanItem>>(emptyList())
    override val scanItems: StateFlow<List<ScanItem>> = _scanItems

    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())
    private var bluetoothGatt: BluetoothGatt? = null
    private var commandChar: BluetoothGattCharacteristic? = null
    private val writeQueue = ConcurrentLinkedQueue<ByteArray>()
    private var writeInProgress = false
    private var reconnectJob: Job? = null
    private var reconnectDelay = 1000L
    private var lastConnectedAddress: String? = null

    private val adapter: BluetoothAdapter?
        get() = (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter

    private val scanner: BluetoothLeScanner?
        get() = adapter?.bluetoothLeScanner

    // ========================================================================
    // Scanning
    // ========================================================================

    override fun startScan() {
        val s = scanner ?: return
        _connectionState.value = ConnectionState.SCANNING
        _scanResults.value = emptyList()
        _scanItems.value = emptyList()

        val filter = ScanFilter.Builder()
            .setServiceUuid(ParcelUuid(SERVICE_UUID))
            .build()
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        s.startScan(listOf(filter), settings, scanCallback)

        // Stop scan after 10 seconds
        scope.launch {
            delay(10_000)
            stopScan()
        }
    }

    override fun stopScan() {
        scanner?.stopScan(scanCallback)
        if (_connectionState.value == ConnectionState.SCANNING) {
            _connectionState.value = ConnectionState.DISCONNECTED
        }
    }

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val current = _scanResults.value.toMutableList()
            val idx = current.indexOfFirst { it.device.address == result.device.address }
            if (idx >= 0) current[idx] = result else current.add(result)
            _scanResults.value = current
            _scanItems.value = current.map { r ->
                ScanItem(
                    name = r.device.name ?: "Unknown",
                    detail = "RSSI: ${r.rssi} dBm",
                )
            }
        }
    }

    // ========================================================================
    // Connect / Disconnect
    // ========================================================================

    fun connect(device: BluetoothDevice) {
        stopScan()
        reconnectJob?.cancel()
        _connectionState.value = ConnectionState.CONNECTING
        lastConnectedAddress = device.address
        reconnectDelay = 1000L
        bluetoothGatt = device.connectGatt(context, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
    }

    override fun connectItem(index: Int) {
        val result = _scanResults.value.getOrNull(index) ?: return
        connect(result.device)
    }

    override fun disconnect() {
        reconnectJob?.cancel()
        lastConnectedAddress = null
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
        commandChar = null
        _connectionState.value = ConnectionState.DISCONNECTED
    }

    private fun scheduleReconnect() {
        val address = lastConnectedAddress ?: return
        reconnectJob?.cancel()
        reconnectJob = scope.launch {
            delay(reconnectDelay)
            reconnectDelay = (reconnectDelay * 2).coerceAtMost(MAX_RECONNECT_DELAY_MS)
            Log.d(TAG, "Reconnecting to $address...")
            val device = adapter?.getRemoteDevice(address) ?: return@launch
            _connectionState.value = ConnectionState.CONNECTING
            bluetoothGatt = device.connectGatt(context, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
        }
    }

    // ========================================================================
    // GATT Callbacks
    // ========================================================================

    private val gattCallback = object : BluetoothGattCallback() {

        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    Log.d(TAG, "Connected, discovering services")
                    reconnectDelay = 1000L
                    gatt.discoverServices()
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    Log.d(TAG, "Disconnected (status=$status)")
                    commandChar = null
                    gatt.close()
                    bluetoothGatt = null
                    _connectionState.value = ConnectionState.DISCONNECTED
                    if (lastConnectedAddress != null) {
                        scheduleReconnect()
                    }
                }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.e(TAG, "Service discovery failed: $status")
                return
            }

            val service = gatt.getService(SERVICE_UUID)
            if (service == null) {
                Log.e(TAG, "Autopilot service not found")
                return
            }

            // Cache command characteristic
            commandChar = service.getCharacteristic(COMMAND_CHAR_UUID)

            // Enable notifications on State characteristic
            val stateChar = service.getCharacteristic(STATE_CHAR_UUID)
            if (stateChar != null) {
                gatt.setCharacteristicNotification(stateChar, true)
                val desc = stateChar.getDescriptor(CCCD_UUID)
                desc?.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                gatt.writeDescriptor(desc)
            }

            // Enable notifications on Fault characteristic (after state descriptor write completes)
            scope.launch {
                delay(500) // Wait for descriptor write
                val faultChar = service.getCharacteristic(FAULT_CHAR_UUID)
                if (faultChar != null) {
                    gatt.setCharacteristicNotification(faultChar, true)
                    val desc = faultChar.getDescriptor(CCCD_UUID)
                    desc?.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                    gatt.writeDescriptor(desc)
                }
            }

            _connectionState.value = ConnectionState.CONNECTED
            Log.d(TAG, "Ready")
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            val data = characteristic.value ?: return
            when (characteristic.uuid) {
                STATE_CHAR_UUID -> {
                    AutopilotState.fromBytes(data)?.let { _state.value = it }
                }
                FAULT_CHAR_UUID -> {
                    if (data.isNotEmpty()) {
                        _state.value = _state.value.copy(faultCode = data[0].toInt() and 0xFF)
                    }
                }
            }
        }

        override fun onCharacteristicWrite(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int
        ) {
            writeInProgress = false
            processWriteQueue()
        }
    }

    // ========================================================================
    // Command Sending
    // ========================================================================

    override fun sendCommand(data: ByteArray) {
        writeQueue.add(data)
        processWriteQueue()
    }

    private fun processWriteQueue() {
        if (writeInProgress) return
        val gatt = bluetoothGatt ?: return
        val char = commandChar ?: return
        val data = writeQueue.poll() ?: return

        writeInProgress = true
        char.value = data
        char.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
        gatt.writeCharacteristic(char)
    }

    override fun destroy() {
        disconnect()
        scope.cancel()
    }
}
