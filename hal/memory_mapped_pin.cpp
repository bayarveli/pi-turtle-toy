// Memory-mapped pin control implementation for Raspberry Pi.

#include "memory_mapped_pin.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdexcept>

MemoryMappedPin::MemoryMappedPin(const std::string& pin_number)
  : pin(pin_number) {
  // Constructor just stores pin number; actual configuration happens in set_alt_function
}

void MemoryMappedPin::set_alt_function(AltFunction function) {
  // Hardware register addresses for Raspberry Pi
  static const unsigned long BCM2708_PERI_BASE = 0x3F000000;
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
  int pin_num = std::stoi(pin);
  
  // Each GPFSEL register controls 10 pins (3 bits per pin)
  // GPFSEL0 controls pins 0-9, GPFSEL1 controls 10-19, etc.
  int reg_index = pin_num / 10;
  int bit_offset = (pin_num % 10) * 3;

  // Clear the 3 bits for this pin and set new function
  unsigned int func_bits = static_cast<unsigned int>(function);
  *(gpio + reg_index) = (*(gpio + reg_index) & ~(7 << bit_offset)) | (func_bits << bit_offset);

  munmap(gpio_map, BLOCK_SIZE);
}
