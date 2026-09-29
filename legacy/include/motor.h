#ifndef MOTOR_H_
#define MOTOR_H_

#include "../hal/digital_pin.h"

class PWM;

class Motor {
 public:
  // channel: PWM channel index for this motor (e.g., 0 or 1)
  Motor(IDigitalPin& in1, IDigitalPin& in2, PWM& pwm, int channel);

  // -255 to +255; negative = reverse, 0 = stop, positive = forward
  void set_speed(int speed);

  // Stop motor (coast)
  void stop();

  // Drive forward at speed 0..255
  void forward(int speed);

  // Drive reverse at speed 0..255
  void reverse(int speed);

  // Non-copyable
  Motor(const Motor&) = delete;
  Motor& operator=(const Motor&) = delete;

 private:
  IDigitalPin* in1;
  IDigitalPin* in2;
  PWM* pwm;
  int channel;
};

#endif // MOTOR_H_
