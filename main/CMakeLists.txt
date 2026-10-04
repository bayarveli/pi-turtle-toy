idf_component_register(
    SRCS "main.cpp" "ble_control.cpp" "motor_driver.cpp" "differential_drive_controller.cpp"
    INCLUDE_DIRS "."
    REQUIRES led_strip esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_timer bt nvs_flash
)

