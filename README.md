# JoyBot — ESP32-S3 Firmware

This repository's `esp32-s3-port` branch is the ESP-IDF firmware project for JoyBot. The previous Raspberry Pi/Linux project is preserved under [`legacy/`](legacy/).

The first firmware milestone blinks the onboard addressable RGB LED green/off. Motor and joystick functionality will be ported in later steps.

## Hardware

This app targets an Espressif **ESP32-S3 development board** with the onboard addressable RGB LED connected to GPIO48 (such as ESP32-S3-DevKitC-1 or ESP32-S3-DevKitM-1). The LED is driven using Espressif's `led_strip` component over RMT. Check the board marking and revision before flashing; other ESP32-S3 boards may use a different LED pin or may not have an onboard RGB LED. See the [ESP32-S3-DevKitC-1 guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/index.html) and [DevKitM-1 guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitm-1/user_guide.html).

Connect the board to the PC using its USB-to-UART port and a data-capable USB cable. The board's native USB port may also support flashing and serial/JTAG, depending on board setup; USB-to-UART is the default for these instructions.

## Build with VS Code

1. Install Espressif's **ESP-IDF** extension and an ESP-IDF toolchain that supports ESP32-S3 (the workspace is set up for ESP-IDF v5.5.1).
2. Open this repository root as the ESP-IDF project.
3. Run **ESP-IDF: Set Espressif Device Target** and select `esp32s3`.
4. Run **ESP-IDF: Build your project**.
5. Connect the board and run **ESP-IDF: Flash your project**. Select the board's COM port if prompted.
6. Run **ESP-IDF: Monitor your device**. The onboard RGB LED should blink green/off every half-second.

The component manager downloads `espressif/led_strip` as a managed dependency during the first build.

## Build and flash from the ESP-IDF terminal

From this directory in an ESP-IDF PowerShell terminal:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

Replace `COMx` with the board's actual serial port, such as `COM5`. Press `Ctrl+]` to exit the serial monitor.

## Legacy Raspberry Pi project

The previous Linux/Raspberry Pi source, tests, README, and CMake configuration are kept under [`legacy/`](legacy/), with their relative paths preserved. That snapshot should be built in a Linux/Raspberry Pi environment; it is not part of this ESP-IDF firmware build.
