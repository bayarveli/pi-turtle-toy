#pragma once

// Starts a BLE GATT server (Nordic UART + HM-10 services) driven by the BLE Joystick app.
void ble_control_start();
bool ble_control_blinking_enabled();

// Normalized D-pad command: linear in [-1, 1] (forward +), yaw in [-1, 1] (left +).
struct DriveInput {
    float linear;
    float yaw;
};
DriveInput ble_control_drive_input();
