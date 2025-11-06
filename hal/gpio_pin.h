// GPIO pin control for Raspberry Pi using Linux sysfs interface.
// Provides safe hardware abstraction for GPIO operations.

#ifndef HAL_GPIO_PIN_H_
#define HAL_GPIO_PIN_H_

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <array>

class GpioPin {
 public:
  static inline const std::string RISING = "rising";
  static inline const std::string FALLING = "falling";
  static inline const std::string BOTH = "both";

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

  static constexpr int INVALID_FD = -1;

  // Constructor with specific GPIO pin number
  explicit GpioPin(const std::string& pin_number);

  ~GpioPin();

  // Set GPIO pin direction (INPUT or OUTPUT)
  // Returns negative value on error, >=0 on success
  int set_direction(const std::string& direction);

  // Set GPIO pin value (HIGH or LOW)
  // Returns negative value on error, >=0 on success
  int set_value(const std::string& value);

  // Get current GPIO pin value
  // Returns negative value on error, >=0 on success
  int get_value(std::string& value);

  // Get the GPIO pin number set in constructor
  std::string get_number() const;

 private:
  // Export GPIO pin to make it available for use
  int export_gpio();

  // Unexport GPIO pin when done
  int unexport_gpio();

  int value_fd;
  int direction_fd;
  int export_fd;
  int unexport_fd;
  int edge_fd;
  std::string pin_number;
};

#endif /* HAL_GPIO_PIN_H_ */
