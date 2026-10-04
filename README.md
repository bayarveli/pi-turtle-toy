# JoyBot — ESP32-S3 Firmware

This repository's `esp32-s3-port` branch is the ESP-IDF firmware project for JoyBot. The previous Raspberry Pi/Linux project is preserved under [`legacy/`](legacy/).

The firmware drives the onboard addressable RGB LED with alternating double red and blue flashes. A reusable ESP-IDF driver for the 4tronix L298N dual H-bridge is also included; joystick input and board-specific motor pin assignments are not configured yet.

## Hardware

This app targets an Espressif **ESP32-S3 development board** with the onboard addressable RGB LED connected to GPIO48 (such as ESP32-S3-DevKitC-1 or ESP32-S3-DevKitM-1). The LED is driven using Espressif's `led_strip` component over RMT. Check the board marking and revision before flashing; other ESP32-S3 boards may use a different LED pin or may not have an onboard RGB LED. See the [ESP32-S3-DevKitC-1 guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/index.html) and [DevKitM-1 guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitm-1/user_guide.html).

Connect the board to the PC using its USB-to-UART port and a data-capable USB cable. The board's native USB port may also support flashing and serial/JTAG, depending on board setup; USB-to-UART is the default for these instructions.

## Build with VS Code

1. Install Espressif's **ESP-IDF** extension and an ESP-IDF toolchain that supports ESP32-S3 (the workspace is set up for ESP-IDF v5.5.1).
2. Open this repository root as the ESP-IDF project.
3. Run **ESP-IDF: Set Espressif Device Target** and select `esp32s3`.
4. Run **ESP-IDF: Build your project**.
5. Connect the board and run **ESP-IDF: Flash your project**. Select the board's COM port if prompted.
6. Run **ESP-IDF: Monitor your device**. The onboard RGB LED should flash red twice, pause, then flash blue twice, repeating.

The component manager downloads `espressif/led_strip` as a managed dependency during the first build.

## L298N motor driver

`main/motor_driver.hpp` provides `L298NMotorDriver` for two brushed DC motors. Supply the ESP32-S3 GPIO pins connected to each channel's `ENA`/`ENB`, `IN1`/`IN2`, and `IN3`/`IN4` inputs. The driver uses 20 kHz, 8-bit LEDC PWM on the enable pins; `set_speed(MotorSide::Left, speed)` and `set_speed(MotorSide::Right, speed)` accept signed values from `-255` to `255` (negative is reverse). Call `init()` before controlling motors and check each returned `esp_err_t`.

Remove the module's ENA/ENB jumpers when using PWM. Power the motors from the module's motor supply, connect the ESP32-S3 ground to the module ground, and do not power motors from an ESP32 GPIO or its 3.3 V pin. The module's listed maximum is 2 A continuous per channel (3 A peak); observe the motor and module thermal limits. Assign pins for the specific development board before constructing the driver; the application does not start the motors automatically.

## Differential-drive velocity control

`main/differential_drive_controller.hpp` adds `DifferentialDriveController`, which accepts chassis linear velocity in m/s and yaw rate in rad/s. It converts these to left/right wheel angular-speed targets, proportionally scales both if either exceeds the configured maximum, reads encoder pulse rates with ESP-IDF PCNT, and uses an independent PID state per wheel to set signed L298N PWM. Call `init()`, set velocity with `set_velocity(linear_mps, yaw_radps)`, and call `update()` from one control task at the configured period; call `set_velocity()`, `update()`, and `stop()` on the same task or serialize them externally. Early `update()` calls before the configured interval are harmless no-ops. `stop()` coasts both motors.

All physical calibration is explicit: configure the effective wheel radius, wheel center-to-center track, maximum wheel angular speed, and measured encoder rising-edge counts per wheel revolution. The ROB0005/FIT0003 wheel's published diameter is 65 mm (nominal radius 0.0325 m), but rolling radius and track should be measured on the assembled robot. Measure encoder counts by rotating a wheel through one full revolution; this implementation counts rising edges only. Set nonnegative PID gains for the wheel-speed loop and tune them on hardware, starting with the wheels raised and a low speed command. A zero-gain PID is rejected.

The SEN0038 is single-channel: pulse frequency gives speed magnitude, while direction is inferred from the commanded motor direction. It cannot detect a wheel being externally driven backward, and this controller is not odometry. Wheel slip, motor variation, supply voltage, and L298N voltage drop mean chassis speed still requires calibration. The current application deliberately has no board-specific motor/encoder pin assignment and does not instantiate this controller.

Example setup (replace GPIOs and measured/tuned values for the actual board):

```cpp
#include "differential_drive_controller.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern const MotorPins kLeftMotorPins;
extern const MotorPins kRightMotorPins;
extern const WheelEncoderPins kEncoderPins;
extern const float kMeasuredWheelTrackMeters;
extern const int kMeasuredEncoderRisingEdgesPerRevolution;
extern const float kCalibratedMaximumWheelRadiansPerSecond;
extern const float kTunedKp;
extern const float kTunedKi;
extern const float kTunedKd;

DifferentialDriveConfig config{};
config.wheel_radius_meters = 0.0325f;
config.wheel_track_meters = kMeasuredWheelTrackMeters;
config.encoder_counts_per_wheel_revolution = kMeasuredEncoderRisingEdgesPerRevolution;
config.max_wheel_radians_per_second = kCalibratedMaximumWheelRadiansPerSecond;
config.control_period_ms = 20;
config.kp = kTunedKp;
config.ki = kTunedKi;
config.kd = kTunedKd;

L298NMotorDriver motors(kLeftMotorPins, kRightMotorPins);
DifferentialDriveController drive(motors, kEncoderPins, config);
ESP_ERROR_CHECK(drive.init());
ESP_ERROR_CHECK(drive.set_velocity(0.15f, 0.0f));

while (true) {
	ESP_ERROR_CHECK(drive.update());
	vTaskDelay(pdMS_TO_TICKS(config.control_period_ms));
}
```

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
