// libgpiod-backed implementation of GpiodPin using C++ API (libgpiod 2.x style)

#include "gpiod_pin.h"
#include <stdexcept>
#include <string>
#include <cstdlib>

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

void GpiodPin::set_direction(const std::string& direction)
{
    if (direction != INPUT && direction != OUTPUT) {
        throw std::invalid_argument("Invalid direction: must be 'in' or 'out'");
    }

    request_.reset();
    
    ::gpiod::line_settings settings;

    if (direction == INPUT) {
        settings.set_direction(::gpiod::line::direction::INPUT);
        output_ = false;
    } else {
        settings.set_direction(::gpiod::line::direction::OUTPUT)
                .set_output_value(::gpiod::line::value::INACTIVE);
        output_ = true;
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

void GpiodPin::set_value(const std::string& value)
{
    if (!request_) {
        throw std::runtime_error("GPIO line not requested; call set_direction first");
    }
    if (!output_) {
        throw std::runtime_error("Cannot set value when direction is INPUT");
    }
    if (value != HIGH && value != LOW) {
        throw std::invalid_argument("Invalid value: must be '1' or '0'");
    }
    auto line_value = (value == HIGH) ? ::gpiod::line::value::ACTIVE : ::gpiod::line::value::INACTIVE;
    request_->set_value(line_offset_, line_value);
}

void GpiodPin::get_value(std::string& value)
{
    if (!request_) {
        throw std::runtime_error("GPIO line not requested; call set_direction first");
    }

    auto line_value = request_->get_value(line_offset_);
    value = (line_value == ::gpiod::line::value::ACTIVE) ? HIGH : LOW;
}
