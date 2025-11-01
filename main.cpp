// #include <iostream>

// int main()
// {
//     std::cout << "Hello, from JoyBot!\n";
// }

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <linux/input.h>

#include "PupaClass/gpioLib.h"
#include "PupaClass/pwmLib.h"

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

	/*
	 * Raspberry Pin 16 -> IN1 (Red Cable)
	 * Raspberry Pin 18 -> IN2 (Green Cable)
	 * */
	IMGPIO rightMotorInput1("535");
	IMGPIO rightMotorInput2("536");

	/*
	 * Raspberry Pin 29 -> IN3 (Blue Cable)
	 * Raspberry Pin 31 -> IN4 (Orange Cable)
	 * */
	IMGPIO leftMotorInput1("517");
	IMGPIO leftMotorInput2("518");
	/*Set direction pins as output*/
	rightMotorInput1.SetPinDirection(IMGPIO::OUTPUT);
	rightMotorInput2.SetPinDirection(IMGPIO::OUTPUT);
	leftMotorInput1.SetPinDirection(IMGPIO::OUTPUT);
	leftMotorInput2.SetPinDirection(IMGPIO::OUTPUT);

	/*PWM */
	/* Raspberry Pin 12 -> ENA (Gray Cable) */
	/* Raspberry Pin 33 -> ENB (Purple Cable) */
	PWM MotorPWM(1000.0,256,80.0,PWM::MSMODE);

	rightMotorInput1.SetPinValue(IMGPIO::LOW);
	rightMotorInput2.SetPinValue(IMGPIO::LOW);
	leftMotorInput1.SetPinValue(IMGPIO::LOW);
	leftMotorInput2.SetPinValue(IMGPIO::LOW);

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
        auto motorSpeedLeft{ 0 };
        auto motorSpeedRight{ 0 };

        if (yAxis < -100) // forward
        {
			leftMotorInput1.SetPinValue(IMGPIO::HIGH);
			leftMotorInput2.SetPinValue(IMGPIO::LOW);
			rightMotorInput1.SetPinValue(IMGPIO::LOW);
			rightMotorInput2.SetPinValue(IMGPIO::HIGH);
            motorSpeedLeft = map(std::abs(yAxis), std::abs(-100), std::abs(-32767), 0, 255);
            motorSpeedRight = map(std::abs(yAxis), std::abs(-100), std::abs(-32767), 0, 255);
        }
        else if (yAxis > 100) // reverse
        {
			leftMotorInput1.SetPinValue(IMGPIO::LOW);
			leftMotorInput2.SetPinValue(IMGPIO::HIGH);
			rightMotorInput1.SetPinValue(IMGPIO::HIGH);
			rightMotorInput2.SetPinValue(IMGPIO::LOW);
            motorSpeedLeft = map(yAxis, 100, 32767, 0, 255);
            motorSpeedRight = map(yAxis, 100, 32767, 0, 255);
        }
        else
        {
            motorSpeedLeft = 0;
            motorSpeedRight = 0;
			rightMotorInput1.SetPinValue(IMGPIO::LOW);
			rightMotorInput2.SetPinValue(IMGPIO::LOW);
			leftMotorInput1.SetPinValue(IMGPIO::LOW);
			leftMotorInput2.SetPinValue(IMGPIO::LOW);
        }

        if (xAxis < -100) // left 
        {
            auto xMapped = map(std::abs(xAxis), std::abs(-100), std::abs(-32767), 0, 255);
            motorSpeedLeft = motorSpeedLeft - xMapped;
            motorSpeedRight = motorSpeedRight + xMapped;
            if (motorSpeedLeft < 0) { motorSpeedLeft = 0; }
            if (motorSpeedRight > 255) { motorSpeedRight = 255; }
        }

        if (xAxis > 100) // Right
        {
            auto xMapped = map(xAxis, 100, 32767, 0, 255);
            motorSpeedLeft = motorSpeedLeft + xMapped;
            motorSpeedRight = motorSpeedRight - xMapped;
            if (motorSpeedLeft > 255) { motorSpeedLeft = 255; }
            if (motorSpeedRight < 0) { motorSpeedRight = 0; }
        }

		MotorPWM.SetDutyCycleCount(motorSpeedLeft,0); // increase Duty Cycle by 16 counts every two seconds
		MotorPWM.SetDutyCycleCount(motorSpeedRight,1); // increase Duty Cycle by 16 counts every two seconds

		// printf("Left Speed: %d - Right Speed: %d \n", motorSpeedLeft, motorSpeedRight);

		fflush(stdout);
		usleep(16000);
	}
}