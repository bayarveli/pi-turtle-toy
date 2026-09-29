// GPIO pin control implementation for Raspberry Pi using Linux sysfs interface.
#include <errno.h>
#include <iostream>

#include "file_descriptor.h"

#include "gpio_pin.h"

GpioPin::GpioPin(const std::string& pin_number) : pin_number(pin_number)
{
  export_gpio();
}

GpioPin::~GpioPin()
{
  try {
    unexport_gpio();
  } catch (const std::exception& e) {
    // Log the error but do not throw from destructor
    std::cerr << "[GpioPin] destructor: failed to unexport GPIO "
              << pin_number << ": " << e.what() << '\n';
  } catch (...) {
    // Log unknown errors as well
    std::cerr << "[GpioPin] destructor: unknown error while unexporting GPIO "
              << pin_number << '\n';
  }
}

void GpioPin::export_gpio()
{
  std::string export_path = "/sys/class/gpio/export";
  FileDescriptor fd(export_path, O_WRONLY | O_SYNC);
  int status_value = write(fd.get(), pin_number.c_str(), pin_number.length());
  if (status_value < 0) {
    if (errno == EBUSY) {
      // GPIO already exported - safe to continue
      return;
    }
    throw std::runtime_error("Failed to export GPIO " + pin_number);
  }
}

void GpioPin::unexport_gpio()
{
  std::string unexport_path = "/sys/class/gpio/unexport";
  FileDescriptor fd(unexport_path, O_WRONLY | O_SYNC);
  int status_value = write(fd.get(), pin_number.c_str(), pin_number.length());
  if (status_value < 0) {
    throw std::runtime_error("Failed to unexport GPIO " + pin_number);
  }
}

void GpioPin::set_direction(PinDirection dir)
{
  std::string direction_path = "/sys/class/gpio/gpio" + pin_number + "/direction";
  std::string direction_str;

  if (dir == PinDirection::INPUT) {
    direction_str = INPUT;
  } else if (dir == PinDirection::OUTPUT) {
    direction_str = OUTPUT;
  } else {
    throw std::invalid_argument("Invalid PinDirection value");
  }

  FileDescriptor fd(direction_path, O_WRONLY | O_SYNC);
  int status_value = write(fd.get(), direction_str.c_str(), direction_str.length());
  if (status_value < 0) {
    throw std::runtime_error("Failed to set direction for GPIO " + pin_number);
  }
}

void GpioPin::set_value(PinValue val)
{
  std::string set_value_path = "/sys/class/gpio/gpio" + pin_number + "/value";
  std::string value_str;

  if (val == PinValue::HIGH) {
    value_str = HIGH;
  } else if (val == PinValue::LOW) {
    value_str = LOW;
  } else {
    throw std::invalid_argument("Invalid PinValue value");
  }

  FileDescriptor fd(set_value_path, O_WRONLY | O_SYNC);
  int status_value = write(fd.get(), value_str.c_str(), value_str.length());
  if (status_value < 0) {
    throw std::runtime_error("Failed to set value for GPIO " + pin_number);
  }
}

PinValue GpioPin::read_value()
{
  std::string value_path = "/sys/class/gpio/gpio" + pin_number + "/value";
  FileDescriptor fd(value_path, O_RDONLY | O_SYNC);

  char buffer;
  if (read(fd.get(), &buffer, 1) < 0) {
    throw std::runtime_error("Failed to read value from GPIO " + pin_number);
  }
  std::string value(1, buffer);
  if (value == HIGH) {
    return PinValue::HIGH;
  } else if (value == LOW) {
    return PinValue::LOW;
  } else {
    throw std::runtime_error("Invalid value read from GPIO " + pin_number);
  }
}
