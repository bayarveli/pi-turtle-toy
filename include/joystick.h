#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <vector>
#include <string>

class Joystick
{
public:
    Joystick();
    ~Joystick();
    
    void open(const char* fileName);
    // Attempt to reconnect to the joystick device without throwing.
    // Returns true on success, false if the device is still unavailable.
    bool try_reconnect(const char* fileName);
    void close();
    void poll();
    
    bool is_connected() const { return connected; }
    size_t button_count() const { return button_states.size(); }
    size_t axis_count() const { return axis_states.size(); }
    const std::string& name() const { return device_name; }
    
    short button_state(size_t index) const;
    short axis_state(size_t index) const;

private:
    bool connected;
    std::vector<short> button_states;
    std::vector<short> axis_states;
    std::string device_name;
    int file_descriptor;
};

#endif // JOYSTICK_H
