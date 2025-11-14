// Wheel encoder implementation with interrupt support using Linux sysfs

#include "encoder.h"
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <iostream>
#include <stdexcept>
#include <cstring>

Encoder::Encoder(const std::string& gpio_pin, const std::string& edge_type)
    : gpio_pin(std::make_unique<GpioInputPin>(gpio_pin)),
      pulse_count(0),
      running(false),
      value_fd(-1)
{
  gpio_pin->set_edge(edge_type);
}

Encoder::~Encoder()
{
  stop();
  
  if (value_fd >= 0) {
    close(value_fd);
  }
}

void Encoder::start()
{
  if (running.load()) {
    std::cerr << "[Encoder] Already running\n";
    return;
  }

  std::string value_path = "/sys/class/gpio/gpio" + gpio_pin->get_pin_number() + "/value";
  value_fd = open(value_path.c_str(), O_RDONLY | O_NONBLOCK);
  if (value_fd < 0) {
    throw std::runtime_error("Failed to open value file for GPIO " + gpio_pin->get_pin_number());
  }

  char dummy;
  read(value_fd, &dummy, 1);

  running.store(true);
  poll_thread = std::thread(&Encoder::poll_thread_func, this);
}

void Encoder::stop()
{
  if (running.load()) {
    running.store(false);
    if (poll_thread.joinable()) {
      poll_thread.join();
    }
  }
}

void Encoder::poll_thread_func()
{
  struct pollfd pfd;
  pfd.fd = value_fd;
  pfd.events = POLLPRI | POLLERR;

  char buffer[2];

  while (running.load()) {
    // Wait for edge event with timeout
    int ret = poll(&pfd, 1, 100); // 100ms timeout
    
    if (ret > 0) {
      if (pfd.revents & POLLPRI) {
        // Edge detected - read and clear the value
        lseek(value_fd, 0, SEEK_SET);
        if (read(value_fd, buffer, sizeof(buffer)) > 0) {
          // Increment pulse count
          std::uint32_t new_count = pulse_count.fetch_add(1) + 1;
          
          // Call user callback if set
          if (user_callback) {
            user_callback(new_count);
          }
        }
      }
    } else if (ret < 0 && errno != EINTR) {
      std::cerr << "[Encoder] Poll error: " << strerror(errno) << '\n';
      break;
    }
    // ret == 0 means timeout, continue polling
  }
}

std::uint32_t Encoder::get_count() const
{
  return pulse_count.load();
}

std::uint32_t Encoder::get_and_reset()
{
  return pulse_count.exchange(0);
}

void Encoder::reset_count()
{
  pulse_count.store(0);
}

void Encoder::set_callback(std::function<void(std::uint32_t)> callback)
{
  user_callback = callback;
}
