#include <iostream>
#include <gpiod.hpp>
#include <chrono>
#include <thread>

int main() {
    try {
        std::cout << "Testing libgpiod 2.x C++ API\n";

        // Chip aç
        ::gpiod::chip chip("/dev/gpiochip0");
    
        // Line settings: output, başlangıç HIGH
        ::gpiod::line_settings settings;
        settings.set_direction(::gpiod::line::direction::OUTPUT)
                .set_output_value(::gpiod::line::value::ACTIVE);

        // Line config: GPIO17 için settings
        ::gpiod::line_config line_cfg;
        line_cfg.add_line_settings(17, settings);

        // Builder pattern ile request
        auto request = chip.prepare_request()
                           .set_consumer("example")
                           .set_line_config(line_cfg)
                           .do_request();

        // 100ms bekle
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // LOW yap
        request.set_value(17, ::gpiod::line::value::INACTIVE);

        std::cout << "Pulse done\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}