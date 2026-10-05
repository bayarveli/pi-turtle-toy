# JoyBot — ESP32-S3 Firmware

This repository's `esp32-s3-port` branch is the ESP-IDF firmware project for JoyBot. The previous Raspberry Pi/Linux project is preserved under [`legacy/`](legacy/).

The firmware drives the onboard addressable RGB LED with alternating double red and blue flashes. Its BLE joystick controls both L298N motors in open loop; encoders are not required. A reusable ESP-IDF driver for the 4tronix L298N dual H-bridge and a board-specific motor/encoder pin map are included.

## Hardware

This app targets the diymore **ESP32-S3-DevKitC-1 N16R8** with its onboard addressable RGB LED on GPIO48. The LED is driven using Espressif's `led_strip` component over RMT. Check the board marking and revision before flashing; other ESP32-S3 boards may use a different LED pin or may not have an onboard RGB LED. See the [ESP32-S3-DevKitC-1 guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/index.html).

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

`main/motor_driver.hpp` provides `L298NMotorDriver` for two brushed DC motors. The selected ESP32-S3-DevKitC-1 pin map is:

| Board GPIO | Connect to |
|---|---|
| GPIO4 | L298N ENA (right motor PWM, channel A) |
| GPIO5 | L298N IN1 (right motor) |
| GPIO6 | L298N IN2 (right motor) |
| GPIO7 | L298N ENB (left motor PWM, channel B) |
| GPIO15 | L298N IN3 (left motor) |
| GPIO16 | L298N IN4 (left motor) |
| GPIO17 | Left wheel encoder pulse output |
| GPIO18 | Right wheel encoder pulse output |
| GPIO48 | Onboard RGB LED (reserved by this app) |

The pin map is defined and printed at boot in `main/main.cpp`. The robot's right motor is connected to L298N channel A (ENA/IN1/IN2), and its left motor to channel B (ENB/IN3/IN4). The left motor's drive direction is inverted in software to compensate for its mounting/wiring orientation. Connect to the BLE device `JoyBot` with a BLE joystick app configured for the existing letter protocol: `A` drives forward, `C` backward, `B` turns right, and `D` turns left. Each press latches the motion (no press-and-hold): the robot keeps moving until the opposite button is pressed, which stops it (forward/backward and left/right are opposites). Pressing a different axis switches directly to it. Lowercase release letters are ignored. Assign two additional buttons to `F` (increase speed) and `H` (decrease speed). The drive starts at 20% PWM; each speed press adjusts the speed limit by 20 percentage points, up to 100% and down to the 20% minimum. Motors also stop on BLE disconnection. Direction pins and PWM change immediately, with no ramping. The commands mix linear and turning input to drive the two motors independently; encoders are not used. Verify the physical board pin labels and wire encoder outputs so they never exceed the ESP32-S3's 3.3 V GPIO limit. The driver uses 20 kHz, 8-bit LEDC PWM on the enable pins; `set_speed(MotorSide::Left, speed)` and `set_speed(MotorSide::Right, speed)` accept signed values from `-255` to `255` (negative selects the opposite direction). Call `init()` before controlling motors and check each returned `esp_err_t`.

Remove both ENA and ENB jumpers so GPIO4 and GPIO7 can control PWM. Connect the right motor to OUT1/OUT2 and the left motor to OUT3/OUT4. Before powering on, secure the robot or raise its wheels. Power motors from the module's motor supply, connect ESP32-S3 ground to module ground, and do not power motors from an ESP32 GPIO or its 3.3 V pin. The module's listed maximum is 2 A continuous per channel (3 A peak); observe motor and module thermal limits. If a motor turns opposite to the intended direction, power off and swap that motor's leads.

## Differential-drive velocity control

`main/differential_drive_controller.hpp` adds `DifferentialDriveController`, which accepts chassis linear velocity in m/s and yaw rate in rad/s. It converts these to left/right wheel angular-speed targets, proportionally scales both if either exceeds the configured maximum, reads encoder pulse rates with ESP-IDF PCNT, and uses an independent PID state per wheel to set signed L298N PWM. Call `init()`, set velocity with `set_velocity(linear_mps, yaw_radps)`, and call `update()` from one control task at the configured period; call `set_velocity()`, `update()`, and `stop()` on the same task or serialize them externally. Early `update()` calls before the configured interval are harmless no-ops. `stop()` coasts both motors.

All physical calibration is explicit: configure the effective wheel radius, wheel center-to-center track, maximum wheel angular speed, and measured encoder rising-edge counts per wheel revolution. The ROB0005/FIT0003 wheel's published diameter is 65 mm (nominal radius 0.0325 m), but rolling radius and track should be measured on the assembled robot. Measure encoder counts by rotating a wheel through one full revolution; this implementation counts rising edges only. Set nonnegative PID gains for the wheel-speed loop and tune them on hardware, starting with the wheels raised and a low speed command. A zero-gain PID is rejected.

The SEN0038 is single-channel: pulse frequency gives speed magnitude, while direction is inferred from the commanded motor direction. It cannot detect a wheel being externally driven backward, and this controller is not odometry. Wheel slip, motor variation, supply voltage, and L298N voltage drop mean chassis speed still requires calibration. The current application does not instantiate this controller; measure encoder counts and tune the physical control parameters before enabling it.

Example setup (use the configured GPIO map above and replace the measured/tuned values for the robot):

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
