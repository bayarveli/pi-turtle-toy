// GPIO pin control implementation for Raspberry Pi using Linux sysfs interface.

#include "gpio_pin.h"

GpioPin::GpioPin(const std::string& pin_number) 
    : value_fd(INVALID_FD), 
      direction_fd(INVALID_FD), 
      export_fd(INVALID_FD),
      unexport_fd(INVALID_FD), 
      edge_fd(INVALID_FD),
      pin_number(pin_number)
{
  export_gpio();
}

GpioPin::~GpioPin()
{
  unexport_gpio();
}

int GpioPin::export_gpio()
{
  std::string export_path = "/sys/class/gpio/export";

  export_fd = open(export_path.c_str(), O_WRONLY | O_SYNC);
  if (export_fd < 0) {
    throw std::runtime_error("GpioPin: Failed to open GPIO export device");
  }

  int status_value = write(export_fd, pin_number.c_str(),
                           pin_number.length());
  if (status_value < 0) {
    if (errno == EBUSY) {
      printf("GPIO %s is already exported - continuing.\n", pin_number.c_str());
      close(export_fd);
      return 0; // success
    }
    throw std::runtime_error("GpioPin: SYSFS GPIO export aygitina yazarken hata olustu.");
  }

  status_value = close(export_fd);
  if (status_value < 0) {
    throw std::runtime_error("GpioPin: SYSFS GPIO export aygiti kapatilamadi.");
  }

  return status_value;
}

int GpioPin::unexport_gpio()
{
  std::string unexport_path = "/sys/class/gpio/unexport";

  unexport_fd = open(unexport_path.c_str(), O_WRONLY|O_SYNC);
  if (unexport_fd < 0) {
    throw std::runtime_error("GpioPin: SYSFS GPIO unexport aygiti acilamadi");
  }

  int status_value = write(unexport_fd, pin_number.c_str(),
                           pin_number.length());
  if (status_value < 0) {
    throw std::runtime_error("GpioPin: SYSFS GPIO unexport aygitina yazarken hata oluştu.");
  }

  status_value = close(unexport_fd);
  if (status_value < 0) {
    throw std::runtime_error("GpioPin: SYSFS GPIO unexport aygiti kapatilamadi.");
  }

  return status_value;
}

int GpioPin::set_direction(const std::string& direction)
{
  std::string direction_path = "/sys/class/gpio/gpio" + pin_number + "/direction";

  direction_fd = open(direction_path.c_str(), O_WRONLY | O_SYNC);
  if (direction_fd < 0) {
    throw std::runtime_error("GpioPin: Failed to open GPIO direction device");
  }

  if (direction != INPUT && direction != OUTPUT) {
    throw std::invalid_argument("GpioPin: Invalid pin direction. Must be INPUT or OUTPUT.");
  }

  int status_value = write(direction_fd, direction.c_str(), direction.length());
  if (status_value < 0) {
    throw std::runtime_error("GpioPin: Failed to write to GPIO direction device");
  }

  status_value = close(direction_fd);
  if (status_value < 0) {
    throw std::runtime_error("GpioPin: Failed to close GPIO direction device");
  }

  return status_value;
}

int GpioPin::set_value(const std::string& value)
{
  std::string set_value_path = "/sys/class/gpio/gpio" + pin_number + "/value";

  value_fd = open(set_value_path.c_str(), O_WRONLY|O_SYNC);
  if (value_fd < 0) {
    throw std::runtime_error("GpioPin: SYSFS GPIO value aygiti acilamadi.");
  }

  if (value != HIGH && value != LOW) {
    throw std::invalid_argument("GpioPin: Gecersiz deger girildi. HIGH ya da LOW olmali");
  }

  int status_value = write(value_fd, value.c_str(), value.length());
  if (status_value < 0) {
    throw std::runtime_error("GpioPin: SYSFS GPIO value aygitina yazilamadi.");
  }

  status_value = close(value_fd);
  if (status_value < 0) {
    throw std::runtime_error("GpioPin: SYSFS GPIO value aygiti kapatilamadi.");
  }
  return status_value;
}

int GpioPin::get_value(std::string& value)
{
  std::string str_get_value = "/sys/class/gpio/gpio" + pin_number + "/value";

  std::array<char, 2> c_buff;
  value_fd = open(str_get_value.c_str(), O_RDONLY|O_SYNC);
  if (value_fd < 0) {
    throw std::runtime_error("GpioPin: SYSFS GPIO value aygiti acilamadi");
  }

  int status_value = read(value_fd, c_buff.data(), 1);
  if (status_value < 0) {
    throw std::runtime_error("GpioPin: SYSFS GPIO value aygiti okunamadi");
  }

  c_buff[1] = '\0';

  value = std::string(c_buff.data());

  if (value != HIGH && value != LOW) {
    throw std::runtime_error("GpioPin: Gecersiz deger okundu. HIGH ya da LOW olmali.");
  }

  status_value = close(value_fd);
  if (status_value < 0) {
    throw std::runtime_error("GpioPin: SYSFS GPIO value aygiti kapatilamadi.");
  }

  return status_value;
}

std::string GpioPin::get_number() const
{
  return pin_number;
}
