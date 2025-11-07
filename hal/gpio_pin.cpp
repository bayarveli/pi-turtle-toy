// GPIO pin control implementation for Raspberry Pi using Linux sysfs interface.

#include "gpio_pin.h"
#include "file_descriptor.h"
#include <errno.h>
#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdexcept>

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

void GpioPin::set_direction(const std::string& direction)
{
  std::string direction_path = "/sys/class/gpio/gpio" + pin_number + "/direction";
  if (direction != INPUT && direction != OUTPUT) {
    throw std::invalid_argument("Invalid direction: must be INPUT or OUTPUT");
  }

  FileDescriptor fd(direction_path, O_WRONLY | O_SYNC);
  int status_value = write(fd.get(), direction.c_str(), direction.length());
  if (status_value < 0) {
    throw std::runtime_error("Failed to set direction for GPIO " + pin_number);
  }
}

void GpioPin::set_value(const std::string& value)
{
  std::string set_value_path = "/sys/class/gpio/gpio" + pin_number + "/value";
  if (value != HIGH && value != LOW) {
    throw std::invalid_argument("Invalid value: must be HIGH or LOW");
  }

  FileDescriptor fd(set_value_path, O_WRONLY | O_SYNC);
  int status_value = write(fd.get(), value.c_str(), value.length());
  if (status_value < 0) {
    throw std::runtime_error("Failed to set value for GPIO " + pin_number);
  }
}

void GpioPin::get_value(std::string& value)
{
  std::string value_path = "/sys/class/gpio/gpio" + pin_number + "/value";
  FileDescriptor fd(value_path, O_RDONLY | O_SYNC);

  char buffer;
  if (read(fd.get(), &buffer, 1) < 0) {
    throw std::runtime_error("Failed to read value from GPIO " + pin_number);
  }
  value = std::string(1, buffer);
  if (value != HIGH && value != LOW) {
    throw std::runtime_error("Invalid value read from GPIO " + pin_number);
  }
}

void GpioPin::set_alt_function(AltFunction function)
{
  // GPIO alternate function requires direct register access (not available via sysfs)
  static const unsigned long BCM2708_PERI_BASE = 0x3F000000;  // RPi 2/3, use 0x20000000 for RPi 1
  static const unsigned long GPIO_BASE = BCM2708_PERI_BASE + 0x200000;
  static const int BLOCK_SIZE = 4096;

  int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (mem_fd < 0) {
    throw std::runtime_error("Failed to open /dev/mem for GPIO alternate function");
  }

  void* gpio_map = mmap(
    nullptr,
    BLOCK_SIZE,
    PROT_READ | PROT_WRITE,
    MAP_SHARED,
    mem_fd,
    GPIO_BASE
  );

  close(mem_fd);  // Can close after mmap

  if (gpio_map == MAP_FAILED) {
    throw std::runtime_error("Failed to mmap GPIO registers");
  }

  volatile unsigned* gpio = static_cast<volatile unsigned*>(gpio_map);

  // Parse pin number
  int pin = std::stoi(pin_number);
  
  // Each register controls 10 pins (3 bits per pin)
  // GPFSEL0 controls pins 0-9, GPFSEL1 controls 10-19, etc.
  int reg_index = pin / 10;
  int bit_offset = (pin % 10) * 3;

  // Clear the 3 bits for this pin and set new function
  unsigned int func_bits = static_cast<unsigned int>(function);
  *(gpio + reg_index) = (*(gpio + reg_index) & ~(7 << bit_offset)) | (func_bits << bit_offset);

  munmap(gpio_map, BLOCK_SIZE);
}
