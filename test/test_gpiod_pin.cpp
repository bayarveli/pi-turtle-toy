#include <iostream>
#include <chrono>
#include <thread>

#include <gpiod.hpp>

#include "../hal/gpiod_pin.h"

int main() {
    try {
      std::cout << "Testing libgpiod 2.x C++ API\n";
      std::cout << "Toggling GPIO17 at ~1kHz (500us HIGH, 500us LOW)\n";
      std::cout << "Press Ctrl+C to stop\n";

      GpiodPin pin(17, "GpiodTest", "/dev/gpiochip0");
      pin.set_direction(PinDirection::OUTPUT);

      while (true) {
        pin.set_value(PinValue::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        pin.set_value(PinValue::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}