// GPIO pin control for Raspberry Pi using Linux sysfs interface.
// Provides safe hardware abstraction for GPIO operations.

#ifndef HAL_GPIO_PIN_H_
#define HAL_GPIO_PIN_H_

#include <string>

class GpioPin {
 public:
  static inline const std::string OUTPUT = "out";
  static inline const std::string INPUT = "in";

  static inline const std::string HIGH = "1";
  static inline const std::string LOW = "0";

  static inline const std::string GPIO0 = "0";
  static inline const std::string GPIO1 = "1";
  static inline const std::string GPIO2 = "2";
  static inline const std::string GPIO3 = "3";
  static inline const std::string GPIO4 = "4";
  static inline const std::string GPIO5 = "5";
  static inline const std::string GPIO6 = "6";
  static inline const std::string GPIO7 = "7";
  static inline const std::string GPIO8 = "8";
  static inline const std::string GPIO9 = "9";
  static inline const std::string GPIO10 = "10";
  static inline const std::string GPIO11 = "11";
  static inline const std::string GPIO12 = "12";
  static inline const std::string GPIO13 = "13";

  // Constructor with specific GPIO pin number
  explicit GpioPin(const std::string& pin_number);

  ~GpioPin();

  // Delete copy and move operations - GPIO pins represent hardware resources
  GpioPin(const GpioPin&) = delete;
  GpioPin& operator=(const GpioPin&) = delete;
  GpioPin(GpioPin&&) = delete;
  GpioPin& operator=(GpioPin&&) = delete;

  // Set GPIO pin direction (INPUT or OUTPUT)
  void set_direction(const std::string& direction);

  // Set GPIO pin value (HIGH or LOW)
  void set_value(const std::string& value);

  // Get current GPIO pin value
  void get_value(std::string& value);

 private:
  // Export GPIO pin to make it available for use
  void export_gpio();

  // Unexport GPIO pin when done
  void unexport_gpio();

  std::string pin_number;
};

#endif /* HAL_GPIO_PIN_H_ */
