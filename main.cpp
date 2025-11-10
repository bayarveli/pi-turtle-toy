// #include <iostream>

// int main()
// {
//     std::cout << "Hello, from JoyBot!\n";
// }

#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include "hal/gpio_pin.h"
#include "hal/memory_mapped_pin.h"
#include "motor.h"
#include "hal/pwm.h"
#include "joystick.h"

/* https://www.arduino.cc/reference/en/language/functions/math/map/ */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Draw an ascii art to show the axis and numbers

constexpr int AXIS0_MIN{ -32767 }; /* Left joystick - Horizontal Left */
constexpr int AXIS0_MAX{ 32767 }; /* Left joystick - Horizontal Right */
constexpr int AXIS1_MIN{ -32767 }; /* Left joystick - Vertical Up */
constexpr int AXIS1_MAX{ 32767 }; /* Left joystick - Vertical Down */

constexpr int AXIS2_MIN{ -32767 }; /* Left joystick - Horizontal Left */
constexpr int AXIS2_MAX{ 32767 }; /* Left joystick - Horizontal Right */
constexpr int AXIS3_MIN{ -32767 }; /* Left joystick - Vertical Up */
constexpr int AXIS3_MAX{ 32767 }; /* Left joystick - Vertical Down */

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

	left_motor.stop();
	right_motor.stop();

	Joystick usb_joystick;
	usb_joystick.open("/dev/input/js0");

	while (true)
	{
		if (!usb_joystick.is_connected())
		{
			std::cerr << "[WARN] Input device lost. Attempting reconnection..." << std::endl;
			left_motor.stop();
			right_motor.stop();
			while (!usb_joystick.try_reconnect("/dev/input/js0")) {
				std::cerr << "[INFO] Reconnect attempt failed. Retrying in 1s..." << std::endl;
				usleep(1000000);
			}
			std::cout << "[INFO] Input device reconnected." << std::endl;
		}
		
		usb_joystick.poll();

		/* Drive */
		auto xAxis = usb_joystick.axis_state(0);
		auto yAxis = usb_joystick.axis_state(1);
		int motor_speed_left = 0;
		int motor_speed_right = 0;

		// Calculate base speed from Y axis
		if (yAxis < -100) // forward
		{
			motor_speed_left = map(std::abs(yAxis), std::abs(-100), std::abs(-32767), 0, 255);
			motor_speed_right = map(std::abs(yAxis), std::abs(-100), std::abs(-32767), 0, 255);
		}
		else if (yAxis > 100) // reverse
		{
			motor_speed_left = -map(yAxis, 100, 32767, 0, 255);
			motor_speed_right = -map(yAxis, 100, 32767, 0, 255);
		}

		// Apply turning adjustments from X axis
		if (xAxis < -100) // left turn
		{
			int xMapped = map(std::abs(xAxis), std::abs(-100), std::abs(-32767), 0, 255);
			motor_speed_left -= xMapped;
			motor_speed_right += xMapped;
		}
		else if (xAxis > 100) // right turn
		{
			int xMapped = map(xAxis, 100, 32767, 0, 255);
			motor_speed_left += xMapped;
			motor_speed_right -= xMapped;
		}

		left_motor.set_speed(motor_speed_left);
		right_motor.set_speed(motor_speed_right);

		std::cout << "[TRACE] Motor L=" << motor_speed_left << " R=" << motor_speed_right << std::endl;

		fflush(stdout);
		usleep(16000);
	}
}
