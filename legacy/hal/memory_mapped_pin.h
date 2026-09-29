// Memory-mapped pin control for Raspberry Pi peripherals (PWM, SPI, I2C, etc.)

#ifndef HAL_MEMORY_MAPPED_PIN_H_
#define HAL_MEMORY_MAPPED_PIN_H_

#include <string>

// GPIO alternate functions for peripheral modes
enum class AltFunction {
  Input = 0,
  Output = 1,
  Alt0 = 4,
  Alt1 = 5,
  Alt2 = 6,
  Alt3 = 7,
  Alt4 = 3,
  Alt5 = 2
};

class MemoryMappedPin {
 public:
  // Construct with pin number (e.g., "18" for GPIO18)
  explicit MemoryMappedPin(const std::string& pin_number);

  // Set pin to alternate function mode (PWM, SPI, I2C, etc.)
  void set_alt_function(AltFunction function);

  // Non-copyable, non-movable
  MemoryMappedPin(const MemoryMappedPin&) = delete;
  MemoryMappedPin& operator=(const MemoryMappedPin&) = delete;
  MemoryMappedPin(MemoryMappedPin&&) = delete;
  MemoryMappedPin& operator=(MemoryMappedPin&&) = delete;

 private:
  std::string pin;
};

#endif /* HAL_MEMORY_MAPPED_PIN_H_ */
