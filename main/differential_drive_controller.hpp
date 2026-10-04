#pragma once

#include <cstdint>

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "motor_driver.hpp"

struct WheelEncoderPins {
    gpio_num_t left;
    gpio_num_t right;
};

struct DifferentialDriveConfig {
    float wheel_radius_meters = 0.0f;
    float wheel_track_meters = 0.0f;
    int encoder_counts_per_wheel_revolution = 0;
    float max_wheel_radians_per_second = 0.0f;
    uint32_t control_period_ms = 0;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

class DifferentialDriveController {
public:
    DifferentialDriveController(
        L298NMotorDriver& motors,
        WheelEncoderPins encoder_pins,
        DifferentialDriveConfig config);
    ~DifferentialDriveController();

    DifferentialDriveController(const DifferentialDriveController&) = delete;
    DifferentialDriveController& operator=(const DifferentialDriveController&) = delete;

    esp_err_t init();
    esp_err_t set_velocity(float linear_meters_per_second, float yaw_radians_per_second);
    esp_err_t update();
    esp_err_t stop();

private:
    struct EncoderCounter {
        pcnt_unit_handle_t unit = nullptr;
        pcnt_channel_handle_t channel = nullptr;
        bool enabled = false;
        bool running = false;
    };

    struct PidState {
        float integral = 0.0f;
        float previous_measurement = 0.0f;
        bool has_previous_measurement = false;
    };

    esp_err_t initialize_encoder(gpio_num_t pin, EncoderCounter& encoder);
    void release_encoder(EncoderCounter& encoder);
    esp_err_t update_motor(
        MotorSide side,
        float target_radians_per_second,
        float measured_radians_per_second,
        float elapsed_seconds,
        PidState& state);
    bool configuration_is_valid() const;

    L298NMotorDriver& motors_;
    WheelEncoderPins encoder_pins_;
    DifferentialDriveConfig config_;
    EncoderCounter left_encoder_;
    EncoderCounter right_encoder_;
    PidState left_pid_;
    PidState right_pid_;
    float target_left_radians_per_second_ = 0.0f;
    float target_right_radians_per_second_ = 0.0f;
    int64_t last_update_time_us_ = 0;
    bool initialized_ = false;
};