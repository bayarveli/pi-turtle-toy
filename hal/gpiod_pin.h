#ifndef HAL_GPIOD_PIN_H_
#define HAL_GPIOD_PIN_H_

#include <string>
#include <memory>

#include <gpiod.hpp>

#include "digital_pin.h"

class GpiodPin : public IDigitalPin {
public:
  static constexpr const char* OUTPUT = "out";
  static constexpr const char* INPUT = "in";
  static constexpr const char* HIGH = "1";
  static constexpr const char* LOW = "0";

  explicit GpiodPin(int line_offset, const std::string& consumer = "JoyBot", const std::string& chip_name = "gpiochip0");
  ~GpiodPin() = default;

  GpiodPin(const GpiodPin&) = delete;
  GpiodPin& operator=(const GpiodPin&) = delete;
  GpiodPin(GpiodPin&&) = default;
  GpiodPin& operator=(GpiodPin&&) = default;

  void set_direction(PinDirection dir) override;
  void set_value(PinValue val) override;
  PinValue read_value() override;

private:
  std::string consumer_;
  std::string chip_path_;
  int line_offset_;
  bool output_ = false;
  std::unique_ptr<::gpiod::chip> chip_;
  std::unique_ptr<::gpiod::line_request> request_;
};

#endif /* HAL_GPIOD_PIN_H_ */
