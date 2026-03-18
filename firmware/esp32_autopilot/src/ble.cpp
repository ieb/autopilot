#ifndef NATIVE_BUILD

#include "ble.h"
#include "config.h"
#include "pilot_manager.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

// Service and characteristic UUIDs
#define BLE_SERVICE_UUID        "4f490000-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_STATE_CHAR_UUID     "4f490001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_COMMAND_CHAR_UUID   "4f490002-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_FAULT_CHAR_UUID     "4f490003-b5a3-f393-e0a9-e50e24dcca9e"

// Command types
#define BLE_CMD_SET_MODE        0x01
#define BLE_CMD_ADJUST_TARGET   0x02
#define BLE_CMD_SET_PILOT_TYPE  0x03
#define BLE_CMD_STANDBY         0x04

// Pending commands from BLE (applied in main loop)
static volatile bool cmd_mode_change = false;
static volatile PilotMode cmd_mode = MODE_STANDBY;
static volatile float cmd_target = 0.0f;
static volatile bool cmd_type_change = false;
static volatile PilotType cmd_pilot_type = PILOT_PD;
static volatile bool cmd_adjust = false;
static volatile float cmd_delta = 0.0f;

// Connection state
static volatile bool client_connected = false;

// Rate limiting
static uint32_t last_state_ms = 0;
static const uint32_t STATE_INTERVAL_MS = 1000;  // 1 Hz
static uint8_t last_fault_code = 0xFF;  // impossible value to force first send

// GATT objects
static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pStateChar = nullptr;
static NimBLECharacteristic* pFaultChar = nullptr;

// ============================================================================
// Server Callbacks
// ============================================================================

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        client_connected = true;
        Serial.println("BLE: client connected");
    }

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        client_connected = false;
        last_fault_code = 0xFF;  // reset so next connect gets fault
        Serial.println("BLE: client disconnected, restarting advertising");
        NimBLEDevice::startAdvertising();
    }
};

// ============================================================================
// Command Write Callback
// ============================================================================

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) override {
        NimBLEAttValue val = pChar->getValue();
        if (val.size() < 1) return;

        const uint8_t* data = val.data();
        uint8_t cmd_type = data[0];

        switch (cmd_type) {
            case BLE_CMD_SET_MODE:
                if (val.size() >= 4) {
                    cmd_mode = (PilotMode)data[1];
                    cmd_target = (float)((int16_t)(data[2] | (data[3] << 8))) / 10.0f;
                    cmd_mode_change = true;
                }
                break;

            case BLE_CMD_ADJUST_TARGET:
                if (val.size() >= 3) {
                    cmd_delta = (float)((int16_t)(data[1] | (data[2] << 8))) / 10.0f;
                    cmd_adjust = true;
                }
                break;

            case BLE_CMD_SET_PILOT_TYPE:
                if (val.size() >= 2) {
                    cmd_pilot_type = (PilotType)data[1];
                    cmd_type_change = true;
                }
                break;

            case BLE_CMD_STANDBY:
                cmd_mode = MODE_STANDBY;
                cmd_target = 0.0f;
                cmd_mode_change = true;
                break;
        }
    }
};

// ============================================================================
// Public API
// ============================================================================

void ble_init() {
    NimBLEDevice::init("Pilot");
    NimBLEDevice::setPower(ESP_PWR_LVL_P6);  // +6dBm for cockpit range

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(BLE_SERVICE_UUID);

    // State characteristic — Read + Notify
    pStateChar = pService->createCharacteristic(
        BLE_STATE_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    // Command characteristic — Write (no response)
    NimBLECharacteristic* pCmdChar = pService->createCharacteristic(
        BLE_COMMAND_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE_NR
    );
    pCmdChar->setCallbacks(new CommandCallbacks());

    // Fault characteristic — Read + Notify
    pFaultChar = pService->createCharacteristic(
        BLE_FAULT_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    pService->start();

    // Start advertising with service UUID
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("BLE: initialized, advertising as Pilot");
}

void ble_apply_commands(AppState& state) {
    if (cmd_adjust) {
        cmd_adjust = false;
        if (state.pilot_mode == MODE_COMPASS) {
            float new_target = state.target_value + cmd_delta;
            while (new_target < 0) new_target += 360.0f;
            while (new_target >= 360.0f) new_target -= 360.0f;
            state.target_value = new_target;
        } else if (state.pilot_mode == MODE_WIND_AWA || state.pilot_mode == MODE_WIND_TWA) {
            float new_target = state.target_value + cmd_delta;
            while (new_target > 180.0f) new_target -= 360.0f;
            while (new_target < -180.0f) new_target += 360.0f;
            state.target_value = new_target;
        }
    }

    if (cmd_mode_change) {
        cmd_mode_change = false;
        state.pilot_mode = cmd_mode;
        state.target_value = cmd_target;
        state.clutch_requested = (cmd_mode != MODE_STANDBY);
        state.last_pilot_ms = millis();
        if (cmd_mode == MODE_VMG_UP || cmd_mode == MODE_VMG_DOWN) {
            pilot_manager_latch_tack(state.twa);
        }
        pilot_manager_set_mode(cmd_mode, cmd_target);
    }

    if (cmd_type_change) {
        cmd_type_change = false;
        state.pilot_type = cmd_pilot_type;
        pilot_manager_set_type(cmd_pilot_type);
    }
}

void ble_send_state(const AppState& state) {
    if (!client_connected) return;

    uint32_t now = millis();

    // State notification at 1 Hz
    if (now - last_state_ms >= STATE_INTERVAL_MS) {
        last_state_ms = now;

        int16_t target_x10 = (int16_t)(state.target_value * 10.0f);
        uint8_t flags = state.clutch_engaged ? 0x01 : 0x00;

        uint8_t buf[6];
        buf[0] = (uint8_t)state.pilot_mode;
        buf[1] = (uint8_t)state.pilot_type;
        buf[2] = (uint8_t)(target_x10 & 0xFF);
        buf[3] = (uint8_t)((target_x10 >> 8) & 0xFF);
        buf[4] = state.fault_code;
        buf[5] = flags;

        pStateChar->setValue(buf, 6);
        pStateChar->notify();
    }

    // Fault notification on change only
    if (state.fault_code != last_fault_code) {
        last_fault_code = state.fault_code;
        uint8_t fault = state.fault_code;
        pFaultChar->setValue(&fault, 1);
        pFaultChar->notify();
    }
}

bool ble_client_connected() {
    return client_connected;
}

#endif // NATIVE_BUILD
