
#include <algorithm>

#include "../hal/digital_pin.h"
#include "../hal/pwm.h"

#include "motor.h"

Motor::Motor(IDigitalPin& pin_in1, IDigitalPin& pin_in2, PWM& pwm_ref, int pwm_channel)
  : in1(&pin_in1), in2(&pin_in2), pwm(&pwm_ref), channel(pwm_channel) {
  in1->set_direction(PinDirection::OUTPUT);
  in2->set_direction(PinDirection::OUTPUT);
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
  in1->set_value(PinValue::LOW);
  in2->set_value(PinValue::LOW);
  pwm->set_duty_cycle_count(0, channel);
}

void Motor::forward(int speed) {
  speed = std::clamp(speed, 0, 255);
  in1->set_value(PinValue::HIGH);
  in2->set_value(PinValue::LOW);
  pwm->set_duty_cycle_count(speed, channel);
}

void Motor::reverse(int speed) {
  speed = std::clamp(speed, 0, 255);
  in1->set_value(PinValue::LOW);
  in2->set_value(PinValue::HIGH);
  pwm->set_duty_cycle_count(speed, channel);
}
