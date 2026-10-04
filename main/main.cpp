#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ble_control.h"
#include "differential_drive_controller.hpp"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr gpio_num_t kRgbLedGpio = GPIO_NUM_48;
constexpr MotorPins kLeftMotorPins{GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6};
constexpr MotorPins kRightMotorPins{GPIO_NUM_7, GPIO_NUM_15, GPIO_NUM_16};
constexpr WheelEncoderPins kEncoderPins{GPIO_NUM_17, GPIO_NUM_18};
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
             "DevKitC-1 pin map: L298N left ENA/IN1/IN2=%d/%d/%d, right "
             "ENB/IN3/IN4=%d/%d/%d; encoder left/right=%d/%d",
             kLeftMotorPins.enable, kLeftMotorPins.input_a, kLeftMotorPins.input_b,
             kRightMotorPins.enable, kRightMotorPins.input_a, kRightMotorPins.input_b,
             kEncoderPins.left, kEncoderPins.right);
    led_strip_handle_t rgb_led = create_rgb_led();
    ble_control_start();

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

