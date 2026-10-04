#include "motor_driver.hpp"

#include <algorithm>
#include <cstdlib>

#include "driver/ledc.h"

namespace {
constexpr ledc_mode_t kPwmMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kPwmTimer = LEDC_TIMER_0;
constexpr uint32_t kPwmFrequencyHz = 20'000;

esp_err_t configure_pwm_channel(gpio_num_t pin, ledc_channel_t channel)
{
    ledc_channel_config_t channel_config{};
    channel_config.gpio_num = pin;
    channel_config.speed_mode = kPwmMode;
    channel_config.channel = channel;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = kPwmTimer;
    channel_config.duty = 0;
    channel_config.hpoint = 0;
    channel_config.flags.output_invert = 0;
    return ledc_channel_config(&channel_config);
}
} // namespace

L298NMotorDriver::L298NMotorDriver(MotorPins left, MotorPins right)
    : left_(left), right_(right)
{
}

esp_err_t L298NMotorDriver::init()
{
    gpio_config_t direction_config{};
    direction_config.pin_bit_mask = (1ULL << left_.input_a) |
                                    (1ULL << left_.input_b) |
                                    (1ULL << right_.input_a) |
                                    (1ULL << right_.input_b);
    direction_config.mode = GPIO_MODE_OUTPUT;
    direction_config.pull_up_en = GPIO_PULLUP_DISABLE;
    direction_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    direction_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t error = gpio_config(&direction_config);
    if (error != ESP_OK) {
        return error;
    }

    error = gpio_set_level(left_.input_a, 0);
    if (error != ESP_OK) return error;
    error = gpio_set_level(left_.input_b, 0);
    if (error != ESP_OK) return error;
    error = gpio_set_level(right_.input_a, 0);
    if (error != ESP_OK) return error;
    error = gpio_set_level(right_.input_b, 0);
    if (error != ESP_OK) return error;

    ledc_timer_config_t timer_config{};
    timer_config.speed_mode = kPwmMode;
    timer_config.duty_resolution = LEDC_TIMER_8_BIT;
    timer_config.timer_num = kPwmTimer;
    timer_config.freq_hz = kPwmFrequencyHz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    error = ledc_timer_config(&timer_config);
    if (error != ESP_OK) {
        return error;
    }

    error = configure_pwm_channel(left_.enable, LEDC_CHANNEL_0);
    if (error != ESP_OK) {
        return error;
    }
    error = configure_pwm_channel(right_.enable, LEDC_CHANNEL_1);
    if (error != ESP_OK) {
        return error;
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t L298NMotorDriver::set_speed(MotorSide side, int speed)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    speed = std::clamp(speed, -255, 255);
    if (side == MotorSide::Left) {
        return set_motor_speed(left_, speed, LEDC_CHANNEL_0);
    }
    return set_motor_speed(right_, speed, LEDC_CHANNEL_1);
}

esp_err_t L298NMotorDriver::stop()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t error = set_motor_speed(left_, 0, LEDC_CHANNEL_0);
    if (error != ESP_OK) {
        return error;
    }
    return set_motor_speed(right_, 0, LEDC_CHANNEL_1);
}

bool L298NMotorDriver::uses_gpio(gpio_num_t pin) const
{
    return pin == left_.enable || pin == left_.input_a || pin == left_.input_b ||
           pin == right_.enable || pin == right_.input_a || pin == right_.input_b;
}

esp_err_t L298NMotorDriver::set_motor_speed(const MotorPins& pins, int speed, int channel)
{
    const auto ledc_channel = static_cast<ledc_channel_t>(channel);
    esp_err_t error = ledc_set_duty(kPwmMode, ledc_channel, 0);
    if (error != ESP_OK) {
        return error;
    }
    error = ledc_update_duty(kPwmMode, ledc_channel);
    if (error != ESP_OK) {
        return error;
    }

    const int magnitude = std::abs(speed);
    const int input_a_level = speed > 0 ? 1 : 0;
    const int input_b_level = speed < 0 ? 1 : 0;
    error = gpio_set_level(pins.input_a, input_a_level);
    if (error != ESP_OK) {
        return error;
    }
    error = gpio_set_level(pins.input_b, input_b_level);
    if (error != ESP_OK) {
        return error;
    }

    error = ledc_set_duty(kPwmMode, ledc_channel, magnitude);
    if (error != ESP_OK) {
        return error;
    }
    return ledc_update_duty(kPwmMode, ledc_channel);
}