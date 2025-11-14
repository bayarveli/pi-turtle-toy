// Wheel encoder with interrupt support for Raspberry Pi
// Uses Linux sysfs edge detection for efficient pulse counting

#ifndef ENCODER_H_
#define ENCODER_H_

#include <string>
#include <atomic>
#include <thread>
#include <functional>
#include <memory>
#include <cstdint>

#include "hal/gpio_input_pin.h"

class Encoder {
 public:
  // Constructor: gpio_pin is the sysfs GPIO number (e.g., "535" for your system)
  // Default to EDGE_RISING for 20 counts/rev (matching 20 PPR spec)
  // Use EDGE_BOTH for 40 counts/rev (2x resolution) if needed
  explicit Encoder(const std::string& gpio_pin, 
                   const std::string& edge_type = GpioInputPin::EDGE_RISING);
  
  ~Encoder();

  // Delete copy and move operations
  Encoder(const Encoder&) = delete;
  Encoder& operator=(const Encoder&) = delete;
  Encoder(Encoder&&) = delete;
  Encoder& operator=(Encoder&&) = delete;

  // Start listening for interrupts
  void start();

  // Stop listening for interrupts
  void stop();

  // Get current pulse count (thread-safe)
  std::uint32_t get_count() const;

  // Atomically get current pulse count and reset to zero (no lost pulses)
  std::uint32_t get_and_reset();

  // Reset pulse count to zero
  void reset_count();

  // Set a callback function to be called on each pulse (optional)
  void set_callback(std::function<void(std::uint32_t)> callback);

 private:
  void poll_thread_func();

  std::unique_ptr<GpioInputPin> gpio_pin;
  std::atomic<std::uint32_t> pulse_count{0};
  std::atomic<bool> running{false};
  std::thread poll_thread;
  std::function<void(std::uint32_t)> user_callback;
  int value_fd;
};

#endif /* ENCODER_H_ */
