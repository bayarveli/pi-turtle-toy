#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr gpio_num_t kRgbLedGpio = GPIO_NUM_48;
constexpr uint32_t kBlinkPeriodMs = 500;
constexpr uint8_t kGreenBrightness = 32;
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
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "Starting JoyBot RGB LED blink on GPIO %d", kRgbLedGpio);
    led_strip_handle_t rgb_led = create_rgb_led();

    while (true) {
        ESP_ERROR_CHECK(led_strip_set_pixel(rgb_led, 0, 0, kGreenBrightness, 0));
        ESP_ERROR_CHECK(led_strip_refresh(rgb_led));
        vTaskDelay(pdMS_TO_TICKS(kBlinkPeriodMs));

        ESP_ERROR_CHECK(led_strip_clear(rgb_led));
        vTaskDelay(pdMS_TO_TICKS(kBlinkPeriodMs));
    }
}
