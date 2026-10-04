#pragma once

// Starts a BLE GATT server (Nordic UART service) that toggles blinking.
void ble_control_start();
bool ble_control_blinking_enabled();
