#ifndef BLE_H
#define BLE_H

#include "app_state.h"

void ble_init();
void ble_apply_commands(AppState& state);
void ble_send_state(const AppState& state);
bool ble_client_connected();

#endif // BLE_H
