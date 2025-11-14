// GPIO Input Pin with interrupt/edge detection support
// Extends GpioPin to add interrupt capabilities for encoders, buttons, etc.

#ifndef HAL_GPIO_INPUT_PIN_H_
#define HAL_GPIO_INPUT_PIN_H_

#include <string>

#include "gpio_pin.h"

class GpioInputPin : public GpioPin {
 public:
  // Edge trigger types for interrupts
  static inline const std::string EDGE_NONE = "none";
  static inline const std::string EDGE_RISING = "rising";
  static inline const std::string EDGE_FALLING = "falling";
  static inline const std::string EDGE_BOTH = "both";

  // Constructor: gpio_pin is the sysfs GPIO number
  explicit GpioInputPin(const std::string& gpio_pin);

  ~GpioInputPin() = default;

  // Set edge detection mode for interrupts
  void set_edge(const std::string& edge_type);

  // Get the GPIO pin number (for constructing file paths)
  std::string get_pin_number() const { return pin_number; }

 private:
  std::string pin_number;
};

#endif /* HAL_GPIO_INPUT_PIN_H_ */
