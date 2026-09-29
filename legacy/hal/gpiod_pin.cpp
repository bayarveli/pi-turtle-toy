// libgpiod-backed implementation of GpiodPin using C++ API (libgpiod 2.x style)
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "gpiod_pin.h"

GpiodPin::GpiodPin(int line_offset, const std::string& consumer, const std::string& chip_path)
    : consumer_(consumer), 
    chip_path_(chip_path), 
    line_offset_(line_offset),
    chip_(std::make_unique<::gpiod::chip>(chip_path))
{
    if (line_offset < 0) {
        throw std::invalid_argument("GPIO offset must be non-negative");
    }
}

void GpiodPin::set_direction(PinDirection dir)
{
    request_.reset();
    
    ::gpiod::line_settings settings;

    if (dir == PinDirection::INPUT) {
        settings.set_direction(::gpiod::line::direction::INPUT);
        output_ = false;
    } else if (dir == PinDirection::OUTPUT) {
        settings.set_direction(::gpiod::line::direction::OUTPUT)
                .set_output_value(::gpiod::line::value::INACTIVE);
        output_ = true;
    } else {
        throw std::invalid_argument("Invalid PinDirection value");
    }
    
    // Create line config
    ::gpiod::line_config line_cfg;
    line_cfg.add_line_settings(line_offset_, settings);
    
    // Request lines using builder pattern
    request_ = std::make_unique<::gpiod::line_request>(
        chip_->prepare_request()
              .set_consumer(consumer_)
              .set_line_config(line_cfg)
              .do_request()
    );
}

void GpiodPin::set_value(PinValue val)
{
    if (!request_) {
        throw std::runtime_error("GPIO line not requested; call set_direction first");
    }
    if (!output_) {
        throw std::runtime_error("Cannot set value when direction is INPUT");
    }
    auto line_value = (val == PinValue::HIGH) ? ::gpiod::line::value::ACTIVE : ::gpiod::line::value::INACTIVE;
    request_->set_value(line_offset_, line_value);
}

PinValue GpiodPin::read_value()
{
    if (!request_) {
        throw std::runtime_error("GPIO line not requested; call set_direction first");
    }

    auto line_value = request_->get_value(line_offset_);
    return (line_value == ::gpiod::line::value::ACTIVE) ? PinValue::HIGH : PinValue::LOW;
}
