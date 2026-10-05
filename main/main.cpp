#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"
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

    while (true) {
        log_elapsed_ms += kMotorControlPollMs;
        if (log_elapsed_ms >= kEncoderLogMs) {
            log_elapsed_ms = 0;
            log_encoder(left_encoder);
            log_encoder(right_encoder);
        }

        const DriveInput input = ble_control_drive_input();
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

    L298NMotorDriver motors(kLeftMotorPins, kRightMotorPins);
    ESP_ERROR_CHECK(motors.init());
    ble_control_start();
    if (xTaskCreate(drive_control_task, "drive_control", 4096, &motors, 5, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to start drive control task");
        ESP_ERROR_CHECK(motors.stop());
    }

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
