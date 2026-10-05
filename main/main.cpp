#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "ble_control.h"
#include "led_strip.h"
#include "motor_driver.hpp"
#include <algorithm>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr gpio_num_t kRgbLedGpio = GPIO_NUM_48;
constexpr MotorPins kLeftMotorPins{GPIO_NUM_7, GPIO_NUM_15, GPIO_NUM_16};
constexpr MotorPins kRightMotorPins{GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6};
constexpr gpio_num_t kLeftEncoderGpio = GPIO_NUM_17;
constexpr gpio_num_t kRightEncoderGpio = GPIO_NUM_18;
constexpr gpio_num_t kObstacleSensorGpio = GPIO_NUM_1;
constexpr gpio_num_t kLineSensorGpios[3] = {GPIO_NUM_2, GPIO_NUM_8, GPIO_NUM_9};
constexpr uint32_t kLineChargeUs = 10;
constexpr int64_t kLineTimeoutUs = 2500;
constexpr uint32_t kLineDarkThresholdUs = 1000;
constexpr uint32_t kLinePollMs = 50;
constexpr uint32_t kMotorControlPollMs = 20;
constexpr uint32_t kEncoderLogMs = 500;
constexpr float kEncoderPulsesPerRev = 20.0f;
constexpr float kWheelCircumferenceM = 3.14159265f * 0.065f;
constexpr uint32_t kFlashDurationMs = 100;
constexpr uint32_t kFlashGapMs = 100;
constexpr uint32_t kColorGapMs = 250;
constexpr uint8_t kFlashBrightness = 48;
constexpr char kTag[] = "joybot";

led_strip_handle_t create_rgb_led()
{
    led_strip_config_t strip_config{};
    strip_config.strip_gpio_num = kRgbLedGpio;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.flags.invert_out = false;

    led_strip_rmt_config_t rmt_config{};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10'000'000;
    rmt_config.mem_block_symbols = 64;
    rmt_config.flags.with_dma = false;

    led_strip_handle_t strip = nullptr;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
    return strip;
}

int drive_value_to_pwm(float value)
{
    const float bounded = std::clamp(value, -1.0f, 1.0f);
    return static_cast<int>(std::lround(bounded * 255.0f));
}

pcnt_unit_handle_t create_encoder(gpio_num_t gpio)
{
    pcnt_unit_config_t unit_config{};
    unit_config.low_limit = -32768;
    unit_config.high_limit = 32767;
    pcnt_unit_handle_t unit = nullptr;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));

    pcnt_glitch_filter_config_t filter{};
    filter.max_glitch_ns = 1000;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));

    pcnt_chan_config_t channel_config{};
    channel_config.edge_gpio_num = gpio;
    channel_config.level_gpio_num = -1;
    pcnt_channel_handle_t channel = nullptr;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &channel_config, &channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

struct EncoderLog {
    const char* name;
    pcnt_unit_handle_t unit;
    int last_count;
    int last_delta;
};

void log_encoder(EncoderLog& encoder)
{
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder.unit, &count));
    if (count == encoder.last_count && encoder.last_delta == 0) {
        return;
    }
    const int delta = count - encoder.last_count;
    const float meters_per_second = static_cast<float>(delta) * 1000.0f / kEncoderLogMs /
                                    kEncoderPulsesPerRev * kWheelCircumferenceM;
    ESP_LOGI(kTag, "%s encoder count=%d (+%d) speed=%.3f m/s", encoder.name, count, delta,
             static_cast<double>(meters_per_second));
    encoder.last_count = count;
    encoder.last_delta = delta;
}

void drive_control_task(void* context)
{
    auto* motors = static_cast<L298NMotorDriver*>(context);
    EncoderLog left_encoder{"Left", create_encoder(kLeftEncoderGpio), 0, 0};
    EncoderLog right_encoder{"Right", create_encoder(kRightEncoderGpio), 0, 0};
    int last_left = 0;
    int last_right = 0;
    uint32_t log_elapsed_ms = 0;

    gpio_config_t sensor_config{};
    sensor_config.pin_bit_mask = 1ULL << kObstacleSensorGpio;
    sensor_config.mode = GPIO_MODE_INPUT;
    sensor_config.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&sensor_config));
    int last_obstacle = -1;

    while (true) {
        // E18-D80NK output is active low: 0 means an obstacle is detected.
        const int obstacle = gpio_get_level(kObstacleSensorGpio) == 0;
        if (obstacle != last_obstacle) {
            ESP_LOGI(kTag, "Obstacle sensor: %s", obstacle ? "DETECTED" : "clear");
            last_obstacle = obstacle;
        }
        log_elapsed_ms += kMotorControlPollMs;
        if (log_elapsed_ms >= kEncoderLogMs) {
            log_elapsed_ms = 0;
            log_encoder(left_encoder);
            log_encoder(right_encoder);
        }

        DriveInput input = ble_control_drive_input();
        if (obstacle && input.linear > 0.0f) {
            // The latched command is kept, so forward motion resumes once the path is clear.
            input = DriveInput{};
        }
        const int left = -drive_value_to_pwm(input.linear - input.yaw);
        const int right = drive_value_to_pwm(input.linear + input.yaw);
        const bool changed = left != last_left || right != last_right;

        if (left != last_left) {
            ESP_ERROR_CHECK(motors->set_speed(MotorSide::Left, left));
            last_left = left;
        }
        if (right != last_right) {
            ESP_ERROR_CHECK(motors->set_speed(MotorSide::Right, right));
            last_right = right;
        }
        if (changed) {
            ESP_LOGI(kTag, "Drive PWM left=%d right=%d", left, right);
        }
        vTaskDelay(pdMS_TO_TICKS(kMotorControlPollMs));
    }
}

void line_sensor_task(void*)
{
    bool last_dark[3] = {false, false, false};
    bool first = true;

    while (true) {
        uint32_t decay_us[3];
        bool done[3] = {false, false, false};

        // Charge each RC capacitor, then time how long the pin takes to fall low.
        for (gpio_num_t pin : kLineSensorGpios) {
            gpio_set_direction(pin, GPIO_MODE_OUTPUT);
            gpio_set_level(pin, 1);
        }
        esp_rom_delay_us(kLineChargeUs);
        const int64_t start = esp_timer_get_time();
        for (gpio_num_t pin : kLineSensorGpios) {
            gpio_set_direction(pin, GPIO_MODE_INPUT);
        }
        int remaining = 3;
        while (remaining > 0) {
            const int64_t elapsed = esp_timer_get_time() - start;
            for (int i = 0; i < 3; ++i) {
                if (!done[i] && (gpio_get_level(kLineSensorGpios[i]) == 0 || elapsed >= kLineTimeoutUs)) {
                    decay_us[i] = static_cast<uint32_t>(std::min<int64_t>(elapsed, kLineTimeoutUs));
                    done[i] = true;
                    --remaining;
                }
            }
        }

        bool dark[3];
        bool changed = first;
        for (int i = 0; i < 3; ++i) {
            dark[i] = decay_us[i] >= kLineDarkThresholdUs;
            changed = changed || dark[i] != last_dark[i];
            last_dark[i] = dark[i];
        }
        if (changed) {
            first = false;
            ESP_LOGI(kTag, "Line sensor L/C/R: %s/%s/%s (decay %lu/%lu/%lu us)",
                     dark[0] ? "BLACK" : "white", dark[1] ? "BLACK" : "white",
                     dark[2] ? "BLACK" : "white", static_cast<unsigned long>(decay_us[0]),
                     static_cast<unsigned long>(decay_us[1]), static_cast<unsigned long>(decay_us[2]));
        }
        vTaskDelay(pdMS_TO_TICKS(kLinePollMs));
    }
}

void flash_color(led_strip_handle_t strip, uint8_t red, uint8_t green, uint8_t blue)
{
    if (!ble_control_blinking_enabled()) {
        return;
    }
    ESP_ERROR_CHECK(led_strip_set_pixel(strip, 0, red, green, blue));
    ESP_ERROR_CHECK(led_strip_refresh(strip));
    vTaskDelay(pdMS_TO_TICKS(kFlashDurationMs));

    ESP_ERROR_CHECK(led_strip_clear(strip));
    ESP_ERROR_CHECK(led_strip_refresh(strip));
    vTaskDelay(pdMS_TO_TICKS(kFlashGapMs));
}
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "Starting JoyBot RGB LED blink on GPIO %d", kRgbLedGpio);
    ESP_LOGI(kTag,
             "DevKitC-1 pin map: right motor (L298N A) ENA/IN1/IN2=%d/%d/%d, left "
             "motor (L298N B) ENB/IN3/IN4=%d/%d/%d; encoder left/right=%d/%d",
             kRightMotorPins.enable, kRightMotorPins.input_a, kRightMotorPins.input_b,
             kLeftMotorPins.enable, kLeftMotorPins.input_a, kLeftMotorPins.input_b,
             kLeftEncoderGpio, kRightEncoderGpio);
    led_strip_handle_t rgb_led = create_rgb_led();
    ESP_LOGI(kTag, "QTR-3RC line sensor GPIOs=%d/%d/%d", kLineSensorGpios[0],
             kLineSensorGpios[1], kLineSensorGpios[2]);

    L298NMotorDriver motors(kLeftMotorPins, kRightMotorPins);
    ESP_ERROR_CHECK(motors.init());
    ble_control_start();
    if (xTaskCreate(drive_control_task, "drive_control", 4096, &motors, 5, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to start drive control task");
        ESP_ERROR_CHECK(motors.stop());
    }
    xTaskCreate(line_sensor_task, "line_sensor", 3072, nullptr, 4, nullptr);

    while (true) {
        if (!ble_control_blinking_enabled()) {
            ESP_ERROR_CHECK(led_strip_clear(rgb_led));
            ESP_ERROR_CHECK(led_strip_refresh(rgb_led));
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Double red flash, then double blue flash, like a police beacon.
        flash_color(rgb_led, kFlashBrightness, 0, 0);
        flash_color(rgb_led, kFlashBrightness, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(kColorGapMs));

        flash_color(rgb_led, 0, 0, kFlashBrightness);
        flash_color(rgb_led, 0, 0, kFlashBrightness);
        vTaskDelay(pdMS_TO_TICKS(kColorGapMs));
    }
}
