// #include <iostream>

// int main()
// {
//     std::cout << "Hello, from JoyBot!\n";
// }

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <linux/input.h>

#include "hal/gpio_pin.h"
#include "hal/memory_mapped_pin.h"
#include "motor.h"
#include "hal/pwm.h"

struct Joystick
{
	bool connected;
	char buttonCount;
	short* buttonStates;
	char axisCount;
	short* axisStates;
	char name[128];
	int file;
};

Joystick openJoystick(const char* fileName)
{
	Joystick j = {0};
	int file = open(fileName, O_RDONLY | O_NONBLOCK);
	if (file != -1)
	{
		ioctl(file, JSIOCGBUTTONS, &j.buttonCount);
		j.buttonStates = (short*)calloc(j.buttonCount, sizeof(short));
		ioctl(file, JSIOCGAXES, &j.axisCount);
		j.axisStates = (short*)calloc(j.axisCount, sizeof(short));
		ioctl(file, JSIOCGNAME(sizeof(j.name)), j.name);
		j.file = file;
		j.connected = true;
	}
	return j;
}

void readJoystickInput(Joystick* joystick)
{
	while (1)
	{
		js_event event;
		int bytesRead = read(joystick->file, &event, sizeof(event));
		if (bytesRead == 0 || bytesRead == -1) return;

		if (event.type == JS_EVENT_BUTTON && event.number < joystick->buttonCount) {
			joystick->buttonStates[event.number] = event.value;
		}
		if (event.type == JS_EVENT_AXIS && event.number < joystick->axisCount) {
			joystick->axisStates[event.number] = event.value;
		}
	}
}

/* https://www.arduino.cc/reference/en/language/functions/math/map/ */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Draw an ascii art to show the axis and numbers

constexpr u_int8_t JOYSTICK_ID{ 0 };

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
	const unsigned int maxJoysticks = 32;
	Joystick joysticks[maxJoysticks] = {0};

	char fileName[32];
	for (unsigned int i=0; i<maxJoysticks; ++i)
	{
		sprintf(fileName, "/dev/input/js%d", i);
		joysticks[i] = openJoystick(fileName);
	}

	/* Raspberry Pin 12 -> ENA (Gray Cable) */
	/* Raspberry Pin 33 -> ENB (Purple Cable) */
	MemoryMappedPin pwm_channel0("18");  // GPIO18 -> PWM0 (Pin 12)
	pwm_channel0.set_alt_function(AltFunction::Alt5);
	MemoryMappedPin pwm_channel1("13");  // GPIO13 -> PWM1 (Pin 33)
	pwm_channel1.set_alt_function(AltFunction::Alt0);
	PWM motor_pwm(pwm_channel0, pwm_channel1, 1000.0, 256, 80.0, PWM::MSMODE);

	GpioPin right_motor_in1("535"); // Raspberry Pin 16 -> IN1 (Red Cable)
	GpioPin right_motor_in2("536"); // Raspberry Pin 18 -> IN2 (Green Cable)
	right_motor_in1.set_direction(GpioPin::OUTPUT);
	right_motor_in2.set_direction(GpioPin::OUTPUT);
	Motor right_motor(right_motor_in1, right_motor_in2, motor_pwm, 1);

	GpioPin left_motor_in1("517"); // Raspberry Pin 29 -> IN3 (Blue Cable)
	GpioPin left_motor_in2("518"); // Raspberry Pin 31 -> IN4 (Orange Cable)
	left_motor_in1.set_direction(GpioPin::OUTPUT);
	left_motor_in2.set_direction(GpioPin::OUTPUT);
	Motor left_motor(left_motor_in1, left_motor_in2, motor_pwm, 0);

	left_motor.stop();
	right_motor.stop();

	while (1)
	{
		for (unsigned int i=0; i<maxJoysticks; ++i)
		{
			if (joysticks[i].connected)
			{
				readJoystickInput(&joysticks[i]);
				/*
				printf("%s - Axes: ", joysticks[i].name);
				for (char axisIndex=0; axisIndex<joysticks[i].axisCount; ++axisIndex) {
					printf("%d:% 6d ", axisIndex, joysticks[i].axisStates[axisIndex]);
				}
				printf("Buttons: ");
				for (char buttonIndex=0; buttonIndex<joysticks[i].buttonCount; ++buttonIndex) {
					if (joysticks[i].buttonStates[buttonIndex]) printf("%d ", buttonIndex);
				}
				printf("\n");
				*/
			}
		}

        /* Drive */
        auto xAxis = joysticks[JOYSTICK_ID].axisStates[0];
        auto yAxis = joysticks[JOYSTICK_ID].axisStates[1];
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

        // Clamp speeds to valid range
        if (motor_speed_left > 255) motor_speed_left = 255;
        if (motor_speed_left < -255) motor_speed_left = -255;
        if (motor_speed_right > 255) motor_speed_right = 255;
        if (motor_speed_right < -255) motor_speed_right = -255;

        // Apply to motors (right motor inverted for correct wiring)
        left_motor.set_speed(motor_speed_left);
        right_motor.set_speed(motor_speed_right);  // Inverted polarity

		std::cout << "Left Speed: " << motor_speed_left << " - Right Speed: " << motor_speed_right << "\n";

		fflush(stdout);
		usleep(16000);
	}
}
