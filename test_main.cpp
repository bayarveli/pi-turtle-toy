#include <iostream>
#include <gpiod.hpp>
#include <chrono>
#include <thread>

#include "hal/gpiod_pin.h"

int main() {
    try {
      std::cout << "Testing libgpiod 2.x C++ API\n";
      std::cout << "Toggling GPIO17 at ~1kHz (500us HIGH, 500us LOW)\n";
      std::cout << "Press Ctrl+C to stop\n";
      //   // Chip aç
      //   ::gpiod::chip chip("/dev/gpiochip0");
    
      //   // Line settings: output, başlangıç HIGH
      //   ::gpiod::line_settings settings;
      //   settings.set_direction(::gpiod::line::direction::OUTPUT)
      //           .set_output_value(::gpiod::line::value::ACTIVE);

      //   // Line config: GPIO17 için settings
      //   ::gpiod::line_config line_cfg;
      //   line_cfg.add_line_settings(17, settings);

      //   // Builder pattern ile request
      //   auto request = chip.prepare_request()
      //                      .set_consumer("example")
      //                      .set_line_config(line_cfg)
      //                      .do_request();

      //   // 100ms bekle
      //   std::this_thread::sleep_for(std::chrono::milliseconds(100));

      //   // LOW yap
      //   request.set_value(17, ::gpiod::line::value::INACTIVE);

      GpiodPin pin(17, "GpiodTest", "/dev/gpiochip0");
      pin.set_direction(GpiodPin::OUTPUT);

      while (true) {
        pin.set_value(GpiodPin::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        pin.set_value(GpiodPin::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}