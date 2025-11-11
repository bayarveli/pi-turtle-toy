#include "joystick.h"
#include <stdexcept>
#include <cstring>
#include <cerrno>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <linux/input.h>

Joystick::Joystick()
    : connected(false)
    , file_descriptor(-1)
{
}

Joystick::~Joystick()
{
    close();
}

void Joystick::open(const char* fileName)
{
    file_descriptor = ::open(fileName, O_RDONLY | O_NONBLOCK);
    if (file_descriptor == -1) {
        // Do not throw: remain in a disconnected state
        connected = false;
        return;
    }
    
    char buttonCount = 0;
    char axisCount = 0;
    char deviceName[128] = {0};
    
    ioctl(file_descriptor, JSIOCGBUTTONS, &buttonCount);
    button_states.resize(buttonCount, 0);
    ioctl(file_descriptor, JSIOCGAXES, &axisCount);
    axis_states.resize(axisCount, 0);   
    ioctl(file_descriptor, JSIOCGNAME(sizeof(deviceName)), deviceName);
    device_name = deviceName;
    
    connected = true;
}

bool Joystick::try_reconnect(const char* fileName)
{
    if (file_descriptor != -1) {
        ::close(file_descriptor);
        file_descriptor = -1;
    }

    open(fileName);
    return connected;
}

void Joystick::close()
{
    if (file_descriptor != -1) {
        ::close(file_descriptor);
        file_descriptor = -1;
    }
    // Clear runtime state
    button_states.clear();
    axis_states.clear();
    device_name.clear();
    connected = false;
}

void Joystick::poll()
{
    if (!connected) {
        return;
    } 
    
    while (true) {
        js_event event;
        int bytes_read = read(file_descriptor, &event, sizeof(event));
        
        if (bytes_read == 0) {
            return;
        }
        
        if (bytes_read < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            if (errno == ENODEV || errno == EIO || errno == EBADF) {
                connected = false;
            }
            return;
        }

        // Update button state
        if (event.type == JS_EVENT_BUTTON && event.number < button_states.size()) {
            button_states[event.number] = event.value;
        }
        
        // Update axis state
        if (event.type == JS_EVENT_AXIS && event.number < axis_states.size()) {
            axis_states[event.number] = event.value;
        }
    }
}

short Joystick::button_state(size_t index) const
{
    if (index < button_states.size())
    {
        return button_states[index];
    }
    return 0;
}

short Joystick::axis_state(size_t index) const
{
    if (index < axis_states.size())
    {
        return axis_states[index];
    }
    return 0;
}
