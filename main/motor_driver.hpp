#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

struct MotorPins {
    gpio_num_t enable;
    gpio_num_t input_a;
    gpio_num_t input_b;
};

enum class MotorSide {
    Left,
    Right,
};

class L298NMotorDriver {
public:
    L298NMotorDriver(MotorPins left, MotorPins right);

    L298NMotorDriver(const L298NMotorDriver&) = delete;
    L298NMotorDriver& operator=(const L298NMotorDriver&) = delete;

    esp_err_t init();
    esp_err_t set_speed(MotorSide side, int speed);
    esp_err_t stop();
    bool uses_gpio(gpio_num_t pin) const;

private:
    esp_err_t set_motor_speed(const MotorPins& pins, int speed, int channel);

    MotorPins left_;
    MotorPins right_;
    bool initialized_ = false;
};