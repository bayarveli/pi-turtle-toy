// #include <iostream>

// int main()
// {
//     std::cout << "Hello, from JoyBot!\n";
// }

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <cstdint>
#include <chrono>

#include "hal/gpio_pin.h"
#include "hal/memory_mapped_pin.h"
#include "motor.h"
#include "hal/pwm.h"
#include "joystick.h"
#include "encoder.h"

/* https://www.arduino.cc/reference/en/language/functions/math/map/ */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

constexpr const char* JOYSTICK_DEVICE_PATH  = "/dev/input/js0";
constexpr int JOYSTICK_AXIS_MIN = -32767;
constexpr int JOYSTICK_AXIS_MAX = 32767;
constexpr int DEADZONE_THRESHOLD = 100;
constexpr int MOTOR_SPEED_MIN = 0;
constexpr int MOTOR_SPEED_MAX = 255;

int main()
{
	/* PWM pin mapping (RPi 2 Model B)
	 * The Raspberry Pi PWM peripheral has two channels (PWM0, PWM1). Those
	 * channels are used on the following pins using their required ALT modes:
	 *   +----------+---------+-----------+-----------+
	 *   | Function | Channel |  BCM GPIO | Phys Pin  |
	 *   +----------+---------+-----------+-----------+
	 *   |   PWM0   |   ch0   |   GPIO18  |    12     | (ALT5)
	 *   |   PWM1   |   ch1   |   GPIO13  |    33     | (ALT0)
	 *   +----------+---------+-----------+-----------+
	 */
	MemoryMappedPin pwm_channel0("18");
	pwm_channel0.set_alt_function(AltFunction::Alt5);
	MemoryMappedPin pwm_channel1("13");
	pwm_channel1.set_alt_function(AltFunction::Alt0);
	PWM motor_pwm(pwm_channel0, pwm_channel1, 1000.0, 256, 80.0, PWM::MSMODE);

	/* Motor direction pins
	 *
	 * Right motor:
	 *   +------+---------+-----------+-----------+----------+
	 *   | Role |  Side   |  BCM GPIO | Phys Pin  | sysfs id |
	 *   +------+---------+-----------+-----------+----------+
	 *   | IN1  | right   |   GPIO23  |    16     |   535    |
	 *   | IN2  | right   |   GPIO24  |    18     |   536    |
	 *   +------+---------+-----------+-----------+----------+
	 * Left motor:
	 *   +------+---------+-----------+-----------+----------+
	 *   | IN3  | left    |   GPIO5   |    29     |   517    |
	 *   | IN4  | left    |   GPIO6   |    31     |   518    |
	 *   +------+---------+-----------+-----------+----------+
	 * Note: sysfs IDs on this system appear to be (base 512 + BCM).
	 */
	GpioPin right_motor_in1("535");
	GpioPin right_motor_in2("536");
	Motor right_motor(right_motor_in1, right_motor_in2, motor_pwm, 1);

	GpioPin left_motor_in1("517");
	GpioPin left_motor_in2("518");
	Motor left_motor(left_motor_in1, left_motor_in2, motor_pwm, 0);

    /* Wheel encoder pin mapping:
     *   +-------------+-----------+-----------+----------+
     *   |    Role     |  BCM GPIO | Phys Pin  | sysfs id |
     *   +-------------+-----------+-----------+----------+
     *   | Left Wheel  |   GPIO17  |    11     |   529    |
     *   | Right Wheel |   GPIO27  |    13     |   539    |
     *   +-------------+-----------+-----------+----------+
     * Note: Use BCM GPIO numbers for the Encoder constructor
     */
    Encoder left_encoder("17", GpioInputPin::EDGE_RISING);
    Encoder right_encoder("27", GpioInputPin::EDGE_RISING);
	
	Joystick usb_joystick;
	usb_joystick.open(JOYSTICK_DEVICE_PATH );
	
	left_motor.stop();
	right_motor.stop();
    left_encoder.start();
    right_encoder.start();

	std::cout << "[INFO] Encoders started. Monitoring pulses...\n";
	using clock_t = std::chrono::steady_clock;
	auto last_sample = clock_t::now();
	constexpr std::chrono::milliseconds SAMPLE_PERIOD{100}; // 100ms window

	while (true)
	{
		if (!usb_joystick.is_connected())
		{
			std::cerr << "[WARN] Input device lost. Attempting reconnection..." << std::endl;
			left_motor.stop();
			right_motor.stop();
			while (!usb_joystick.try_reconnect(JOYSTICK_DEVICE_PATH)) {
				std::cerr << "[INFO] Reconnect attempt failed. Retrying in 1s..." << std::endl;
				usleep(1000000);
			}
			std::cout << "[INFO] Input device reconnected." << std::endl;
		}
		
		usb_joystick.poll();

		auto steering_axis_value = usb_joystick.axis_state(0);
		auto throttle_axis_value = usb_joystick.axis_state(1);
		int motor_speed_left = 0;
		int motor_speed_right = 0;

		if (throttle_axis_value < -DEADZONE_THRESHOLD)
		{
			motor_speed_left = map(
				std::abs(throttle_axis_value), std::abs(-DEADZONE_THRESHOLD), std::abs(JOYSTICK_AXIS_MIN), MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
			motor_speed_right = map(
				std::abs(throttle_axis_value), std::abs(-DEADZONE_THRESHOLD), std::abs(JOYSTICK_AXIS_MIN), MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
		}
		else if (throttle_axis_value > DEADZONE_THRESHOLD)
		{
			motor_speed_left = -map(
				throttle_axis_value, DEADZONE_THRESHOLD, JOYSTICK_AXIS_MAX, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
			motor_speed_right = -map(
				throttle_axis_value, DEADZONE_THRESHOLD, JOYSTICK_AXIS_MAX, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
		}

		if (steering_axis_value < -DEADZONE_THRESHOLD)
		{
			int xMapped = map(
				std::abs(steering_axis_value), std::abs(-DEADZONE_THRESHOLD), std::abs(JOYSTICK_AXIS_MIN), MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
			motor_speed_left -= xMapped;
			motor_speed_right += xMapped;
		}
		else if (steering_axis_value > DEADZONE_THRESHOLD)
		{
			int xMapped = map(
				steering_axis_value, DEADZONE_THRESHOLD, JOYSTICK_AXIS_MAX, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
			motor_speed_left += xMapped;
			motor_speed_right -= xMapped;
		}

		left_motor.set_speed(motor_speed_left);
		right_motor.set_speed(motor_speed_right);

		// Time-based sampling (no assumption about exact loop duration)
		auto now = clock_t::now();
		if (now - last_sample >= SAMPLE_PERIOD) {
			auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sample).count();
			last_sample = now;
			// Get pulses in this elapsed window (atomic fetch & reset)
			std::uint32_t left_pulses = left_encoder.get_and_reset();
			std::uint32_t right_pulses = right_encoder.get_and_reset();
			// Print only raw pulse counts for this sampling window
			std::cout << "[Enc 100ms] L:" << left_pulses << " | R:" << right_pulses << std::endl;
		}

		// Loop pacing (~50Hz) - not critical for timing accuracy
		usleep(20000); // 20ms
	}
}
