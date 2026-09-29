#ifndef HAL_DIGITAL_PIN_H_
#define HAL_DIGITAL_PIN_H_

#include <string>

enum class PinDirection { INPUT, OUTPUT };
enum class PinValue { LOW, HIGH };

class IDigitalPin {
public:
    virtual ~IDigitalPin() = default;
    virtual void set_direction(PinDirection dir) = 0;
    virtual void set_value(PinValue val) = 0;
    virtual PinValue read_value() = 0;
};

#endif /* HAL_DIGITAL_PIN_H_ */