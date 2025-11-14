// GPIO Input Pin implementation with edge detection

#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <cstring>

#include "file_descriptor.h"

#include "gpio_input_pin.h"

GpioInputPin::GpioInputPin(const std::string& gpio_pin)
    : GpioPin(gpio_pin), pin_number(gpio_pin)
{
  set_direction(GpioPin::INPUT);
}

void GpioInputPin::set_edge(const std::string& edge_type)
{
  if (edge_type != EDGE_NONE && 
      edge_type != EDGE_RISING && 
      edge_type != EDGE_FALLING && 
      edge_type != EDGE_BOTH) {
    throw std::invalid_argument("Invalid edge type: " + edge_type);
  }

  std::string edge_path = "/sys/class/gpio/gpio" + pin_number + "/edge";
  FileDescriptor fd(edge_path, O_WRONLY | O_SYNC);
  
  int status = write(fd.get(), edge_type.c_str(), edge_type.length());
  if (status < 0) {
    throw std::runtime_error("Failed to set edge trigger for GPIO " + pin_number);
  }
}
