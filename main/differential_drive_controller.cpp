#include "differential_drive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "differential_drive_kinematics.hpp"
#include "esp_timer.h"

namespace {
constexpr int kPcntLowLimit = -32'768;
constexpr int kPcntHighLimit = 32'767;
constexpr float kRadiansPerRevolution = 6.28318530717958647692f;
constexpr float kMaximumPwm = 255.0f;

bool encoder_pins_are_valid(const L298NMotorDriver& motors, WheelEncoderPins encoders)
{
    return GPIO_IS_VALID_GPIO(encoders.left) && GPIO_IS_VALID_GPIO(encoders.right) &&
           encoders.left != encoders.right && !motors.uses_gpio(encoders.left) &&
           !motors.uses_gpio(encoders.right);
}
} // namespace

DifferentialDriveController::DifferentialDriveController(
    L298NMotorDriver& motors,
    WheelEncoderPins encoder_pins,
    DifferentialDriveConfig config)
    : motors_(motors), encoder_pins_(encoder_pins), config_(config)
{
}

DifferentialDriveController::~DifferentialDriveController()
{
    if (initialized_) {
        motors_.stop();
    }
    release_encoder(left_encoder_);
    release_encoder(right_encoder_);
}

bool DifferentialDriveController::configuration_is_valid() const
{
    if (!std::isfinite(config_.wheel_radius_meters) || config_.wheel_radius_meters <= 0.0f ||
        !std::isfinite(config_.wheel_track_meters) || config_.wheel_track_meters <= 0.0f ||
        config_.encoder_counts_per_wheel_revolution <= 0 ||
        !std::isfinite(config_.max_wheel_radians_per_second) ||
        config_.max_wheel_radians_per_second <= 0.0f || config_.control_period_ms == 0 ||
        !std::isfinite(config_.kp) || config_.kp < 0.0f ||
        !std::isfinite(config_.ki) || config_.ki < 0.0f ||
        !std::isfinite(config_.kd) || config_.kd < 0.0f ||
        (config_.kp == 0.0f && config_.ki == 0.0f && config_.kd == 0.0f)) {
        return false;
    }

    const float counts_per_period = config_.max_wheel_radians_per_second *
                                    config_.encoder_counts_per_wheel_revolution *
                                    (config_.control_period_ms / 1000.0f) /
                                    kRadiansPerRevolution;
    return std::isfinite(counts_per_period) && counts_per_period < kPcntHighLimit;
}

esp_err_t DifferentialDriveController::initialize_encoder(gpio_num_t pin, EncoderCounter& encoder)
{
    pcnt_unit_config_t unit_config{};
    unit_config.low_limit = kPcntLowLimit;
    unit_config.high_limit = kPcntHighLimit;
    unit_config.intr_priority = 0;
    unit_config.flags.accum_count = false;

    esp_err_t error = pcnt_new_unit(&unit_config, &encoder.unit);
    if (error != ESP_OK) {
        return error;
    }

    pcnt_chan_config_t channel_config{};
    channel_config.edge_gpio_num = pin;
    channel_config.level_gpio_num = -1;
    error = pcnt_new_channel(encoder.unit, &channel_config, &encoder.channel);
    if (error != ESP_OK) {
        return error;
    }

    error = pcnt_channel_set_edge_action(
        encoder.channel,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (error != ESP_OK) {
        return error;
    }
    error = pcnt_channel_set_level_action(
        encoder.channel,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (error != ESP_OK) {
        return error;
    }

    error = pcnt_unit_enable(encoder.unit);
    if (error != ESP_OK) {
        return error;
    }
    encoder.enabled = true;

    error = pcnt_unit_clear_count(encoder.unit);
    if (error != ESP_OK) {
        return error;
    }
    error = pcnt_unit_start(encoder.unit);
    if (error == ESP_OK) {
        encoder.running = true;
    }
    return error;
}

void DifferentialDriveController::release_encoder(EncoderCounter& encoder)
{
    if (encoder.running) {
        pcnt_unit_stop(encoder.unit);
        encoder.running = false;
    }
    if (encoder.enabled) {
        pcnt_unit_disable(encoder.unit);
        encoder.enabled = false;
    }
    if (encoder.channel != nullptr) {
        pcnt_del_channel(encoder.channel);
        encoder.channel = nullptr;
    }
    if (encoder.unit != nullptr) {
        pcnt_del_unit(encoder.unit);
        encoder.unit = nullptr;
    }
}

esp_err_t DifferentialDriveController::init()
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!configuration_is_valid() || !encoder_pins_are_valid(motors_, encoder_pins_)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = motors_.init();
    if (error != ESP_OK) {
        return error;
    }

    error = initialize_encoder(encoder_pins_.left, left_encoder_);
    if (error != ESP_OK) {
        release_encoder(left_encoder_);
        motors_.stop();
        return error;
    }
    error = initialize_encoder(encoder_pins_.right, right_encoder_);
    if (error != ESP_OK) {
        release_encoder(left_encoder_);
        release_encoder(right_encoder_);
        motors_.stop();
        return error;
    }

    last_update_time_us_ = esp_timer_get_time();
    initialized_ = true;
    return motors_.stop();
}

esp_err_t DifferentialDriveController::set_velocity(
    float linear_meters_per_second,
    float yaw_radians_per_second)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    WheelAngularVelocities targets{};
    if (!calculate_wheel_angular_velocities(
            linear_meters_per_second,
            yaw_radians_per_second,
            config_.wheel_radius_meters,
            config_.wheel_track_meters,
            config_.max_wheel_radians_per_second,
            targets)) {
        return ESP_ERR_INVALID_ARG;
    }

    target_left_radians_per_second_ = targets.left_rad_per_second;
    target_right_radians_per_second_ = targets.right_rad_per_second;
    if (target_left_radians_per_second_ == 0.0f && target_right_radians_per_second_ == 0.0f) {
        return stop();
    }
    return ESP_OK;
}

esp_err_t DifferentialDriveController::update_motor(
    MotorSide side,
    float target_radians_per_second,
    float measured_radians_per_second,
    float elapsed_seconds,
    PidState& state)
{
    if (target_radians_per_second == 0.0f) {
        state = {};
        return motors_.set_speed(side, 0);
    }

    const float target_magnitude = std::abs(target_radians_per_second);
    const float error = target_magnitude - measured_radians_per_second;
    const float candidate_integral = state.integral + error * elapsed_seconds;
    const float derivative = state.has_previous_measurement
                                 ? (measured_radians_per_second - state.previous_measurement) /
                                       elapsed_seconds
                                 : 0.0f;
    float output = config_.kp * error + config_.ki * candidate_integral - config_.kd * derivative;
    const bool saturating_high = output > kMaximumPwm && error > 0.0f;
    const bool saturating_low = output < 0.0f && error < 0.0f;
    if (!saturating_high && !saturating_low) {
        state.integral = candidate_integral;
        output = config_.kp * error + config_.ki * state.integral - config_.kd * derivative;
    }

    state.previous_measurement = measured_radians_per_second;
    state.has_previous_measurement = true;

    const int pwm_magnitude = static_cast<int>(std::lround(std::clamp(output, 0.0f, kMaximumPwm)));
    const int signed_pwm = target_radians_per_second < 0.0f ? -pwm_magnitude : pwm_magnitude;
    return motors_.set_speed(side, signed_pwm);
}

esp_err_t DifferentialDriveController::update()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t now_us = esp_timer_get_time();
    const int64_t elapsed_us = now_us - last_update_time_us_;
    if (elapsed_us < static_cast<int64_t>(config_.control_period_ms) * 1000) {
        return ESP_OK;
    }
    const float elapsed_seconds = elapsed_us / 1'000'000.0f;

    int left_count = 0;
    int right_count = 0;
    esp_err_t error = pcnt_unit_get_count(left_encoder_.unit, &left_count);
    if (error != ESP_OK) {
        stop();
        return error;
    }
    error = pcnt_unit_get_count(right_encoder_.unit, &right_count);
    if (error != ESP_OK) {
        stop();
        return error;
    }
    error = pcnt_unit_clear_count(left_encoder_.unit);
    if (error != ESP_OK) {
        stop();
        return error;
    }
    error = pcnt_unit_clear_count(right_encoder_.unit);
    if (error != ESP_OK) {
        stop();
        return error;
    }

    last_update_time_us_ = now_us;
    const float radians_per_count = kRadiansPerRevolution /
                                    config_.encoder_counts_per_wheel_revolution;
    const float left_speed = std::max(left_count, 0) * radians_per_count / elapsed_seconds;
    const float right_speed = std::max(right_count, 0) * radians_per_count / elapsed_seconds;

    error = update_motor(
        MotorSide::Left,
        target_left_radians_per_second_,
        left_speed,
        elapsed_seconds,
        left_pid_);
    if (error != ESP_OK) {
        stop();
        return error;
    }
    error = update_motor(
        MotorSide::Right,
        target_right_radians_per_second_,
        right_speed,
        elapsed_seconds,
        right_pid_);
    if (error != ESP_OK) {
        stop();
    }
    return error;
}

esp_err_t DifferentialDriveController::stop()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    target_left_radians_per_second_ = 0.0f;
    target_right_radians_per_second_ = 0.0f;
    left_pid_ = {};
    right_pid_ = {};
    return motors_.stop();
}