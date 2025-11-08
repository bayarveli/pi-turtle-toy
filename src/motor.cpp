#include "motor.h"

#include <algorithm>

#include "hal/gpio_pin.h"
#include "hal/pwm.h"

Motor::Motor(GpioPin& pin_in1, GpioPin& pin_in2, PWM& pwm_ref, int pwm_channel)
  : in1(&pin_in1), in2(&pin_in2), pwm(&pwm_ref), channel(pwm_channel) {
  in1->set_direction(GpioPin::OUTPUT);
  in2->set_direction(GpioPin::OUTPUT);
  stop();
}

void Motor::set_speed(int speed) {
  speed = std::clamp(speed, -255, 255);
  if (speed > 0) {
    forward(speed);
  } else if (speed < 0) {
    reverse(-speed);
  } else {
    stop();
  }
}

void Motor::stop() {
  in1->set_value(GpioPin::LOW);
  in2->set_value(GpioPin::LOW);
  pwm->set_duty_cycle_count(0, channel);
}

void Motor::forward(int speed) {
  speed = std::clamp(speed, 0, 255);
  in1->set_value(GpioPin::HIGH);
  in2->set_value(GpioPin::LOW);
  pwm->set_duty_cycle_count(speed, channel);
}

void Motor::reverse(int speed) {
  speed = std::clamp(speed, 0, 255);
  in1->set_value(GpioPin::LOW);
  in2->set_value(GpioPin::HIGH);
  pwm->set_duty_cycle_count(speed, channel);
}
