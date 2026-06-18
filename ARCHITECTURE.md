# JoyBot - Raspberry Pi Turtle Robot Controller

A clean, testable C++ application for controlling a turtle robot using joystick input on Raspberry Pi.

## Architecture Overview

This project follows a **balanced, interface-based architecture** focused on **testability** and **maintainability** while keeping complexity minimal.

### System Architecture

```
                           ┌─────────────────────────┐
                           │       main.cpp          │
                           │   (Entry Point)         │
                           │ - Creates Application   │
                           │ - Calls app.run()       │
                           └───────────┬─────────────┘
                                       │
                                       │ instantiates
                                       ▼
                           ┌─────────────────────────┐
                           │     Application         │
                           ├─────────────────────────┤
                           │ + initialize()          │
                           │ + run()                 │
                           │ + update()              │
                           │ + shutdown()            │
                           │ - readJoystick()        │
                           │ - controlMotors()       │
                           └─────┬─────────────┬─────┘
                                 │             │
                           uses  │             │ uses
                                 ▼             ▼
                   ┌─────────────────────┐   ┌─────────────────────┐
                   │     IJoystick       │   │      IMotor         │
                   │   (interface)       │   │   (interface)       │
                   ├─────────────────────┤   ├─────────────────────┤
                   │ + getX()            │   │ + setSpeed(speed)   │
                   │ + getY()            │   │ + stop()            │
                   │ + isConnected()     │   │ + forward(speed)    │
                   │ + getButton(id)     │   │ + reverse(speed)    │
                   └─────────┬───────────┘   └─────────┬───────────┘
                             │                         │
                        implements                implements
                             ▼                         ▼
                   ┌─────────────────────┐   ┌─────────────────────┐
                   │   LinuxJoystick     │   │      DCMotor        │
                   │   MockJoystick      │   │     MockMotor       │
                   │   WebGamepad        │   │     ServoMotor      │
                   └─────────────────────┘   └─────────────────────┘
                                                       │
                                                  depends on
                                                       ▼
                           ┌─────────────────────────────────────────┐
                           │          Hardware Layer                 │
                           ├─────────────────┬───────────────────────┤
                           │    IMGPIO       │         PWM           │
                           │  (existing)     │     (existing)        │
                           ├─────────────────┼───────────────────────┤
                           │ + SetPinValue() │ + SetDutyCycleCount() │
                           │ + SetDirection()│ + configPWM()         │
                           │ - Pin 535,536   │ - Channel 0,1         │
                           │ - Pin 517,518   │ - Frequency: 1000Hz   │
                           └─────────────────┴───────────────────────┘
```

## Core Design Principles

### 1. **Testability First**
- **Interfaces enable mocking** - Test business logic without hardware
- **Dependency injection** - Easy to swap implementations
- **Unit testable** - Each class can be tested in isolation

### 2. **Domain-Specific Interfaces**
- `IJoystick` instead of generic `IInputDevice`
- `IMotor` instead of generic `IActuator`  
- **Clear semantic meaning** in the code

### 3. **Balanced Complexity**
- **Only 2 key interfaces** (IJoystick, IMotor)
- **Simple method signatures** 
- **No over-engineering** - Interfaces only where testability matters

## Class Responsibilities

### Application Class
**Role**: Coordinator and main control loop
```cpp
class Application {
    IJoystick* joystick;
    IMotor* leftMotor;
    IMotor* rightMotor;
    
public:
    bool initialize();
    void run();           // Main game loop
    void update();        // Read joystick → control motors  
    void shutdown();
};
```

**Responsibilities**:
- Initialize all components
- Main control loop (60Hz)
- Convert joystick input to motor commands
- Handle application lifecycle

### IJoystick Interface
**Role**: Input abstraction with gaming semantics
```cpp
class IJoystick {
public:
    virtual int getX() = 0;           // -32767 to +32767
    virtual int getY() = 0;           // -32767 to +32767
    virtual bool isConnected() = 0;
    virtual bool getButton(int id) = 0;
    virtual const char* getName() = 0;
};
```

**Implementations**:
- `LinuxJoystick` - Real hardware via `/dev/input/js*`
- `MockJoystick` - For unit testing
- `WebGamepad` - Future web interface

### IMotor Interface  
**Role**: Motor control abstraction
```cpp
class IMotor {
public:
    virtual void setSpeed(int speed) = 0;  // -255 to +255 (negative = reverse)
    virtual void stop() = 0;
    virtual void forward(int speed) = 0;   // 0-255
    virtual void reverse(int speed) = 0;   // 0-255
};
```

**Implementations**:
- `DCMotor` - Real hardware using IMGPIO + PWM classes
- `MockMotor` - For unit testing  
- `ServoMotor` - Future servo support

## Hardware Configuration

### GPIO Pin Mapping
```cpp
// Right Motor (Channel 1)
Pin 16 (GPIO 535) → IN1 (Red Cable)
Pin 18 (GPIO 536) → IN2 (Green Cable) 
Pin 12            → ENA (Gray Cable, PWM Channel 1)

// Left Motor (Channel 0)  
Pin 29 (GPIO 517) → IN3 (Blue Cable)
Pin 31 (GPIO 518) → IN4 (Orange Cable)
Pin 33            → ENB (Purple Cable, PWM Channel 0)
```

### PWM Configuration
- **Frequency**: 1000Hz
- **Resolution**: 256 steps (8-bit)
- **Max Duty Cycle**: 80%
- **Channels**: 0 (left motor), 1 (right motor)

## Testing Strategy

### Unit Testing Examples
```cpp
TEST(ApplicationTest, ForwardMovement) {
    // Arrange
    MockJoystick joystick;
    MockMotor leftMotor, rightMotor;
    Application app(&joystick, &leftMotor, &rightMotor);
    
    // Act  
    joystick.setY(-32767);  // Full forward
    app.update();
    
    // Assert
    EXPECT_EQ(leftMotor.getSpeed(), 255);
    EXPECT_EQ(rightMotor.getSpeed(), 255);
}

TEST(MotorTest, SpeedControl) {
    MockGPIO gpio1, gpio2;
    MockPWM pwm;
    DCMotor motor(&gpio1, &gpio2, &pwm, 0);
    
    motor.setSpeed(150);
    
    EXPECT_EQ(gpio1.getValue(), HIGH);  // Forward direction
    EXPECT_EQ(pwm.getDutyCycle(0), 150);
}
```

## Future Extensions

### PID Control Integration
```cpp
class PIDController {
    float kp, ki, kd;
public:
    float compute(float setpoint, float input);
};

class DCMotorWithPID : public IMotor {
    PIDController pid;
    ISpeedSensor* encoder;
public:
    void setTargetSpeed(int speed) override;
    void updatePID();  // Call in control loop
};
```

### Additional Input Methods
```cpp
class KeyboardJoystick : public IJoystick {
    // WASD keys as joystick input
};

class NetworkJoystick : public IJoystick {
    // Remote control via network
};
```

## Building and Running

### Prerequisites
- Raspberry Pi 2/3/4
- CMake 3.16+
- C++17 compiler
- Hardware: Motor driver, joystick

### Build Commands
```bash
mkdir build && cd build
cmake ..
make
sudo ./bin/JoyBot  # Requires sudo for GPIO access
```

### Project Structure
```
pi-turtle-toy/
├── main.cc               # Entry point (minimal) - Google prefers .cc
├── include/
│   ├── i_joystick.h      # Joystick interface
│   ├── i_motor.h         # Motor interface
│   ├── application.h     # Main application class
│   ├── linux_joystick.h  # Linux joystick implementation
│   └── dc_motor.h        # DC motor implementation
├── src/
│   ├── application/
│   │   └── application.cc
│   └── components/
│       ├── motors/
│       │   └── dc_motor.cc
│       └── joysticks/
│           └── linux_joystick.cc
├── hal/                  # Hardware Abstraction Layer
│   ├── gpio/
│   │   ├── gpio_lib.cc
│   │   └── gpio_lib.h
│   └── pwm/
│       ├── pwm_lib.cc
│       └── pwm_lib.h
├── tests/                # Unit tests
│   ├── application_test.cc
│   ├── motor_test.cc
│   ├── joystick_test.cc
│   └── mocks/
│       ├── mock_motor.h
│       └── mock_joystick.h
└── CMakeLists.txt
```

## Design Benefits

### ✅ **Maintainable**
- Clear separation of concerns
- Single responsibility per class
- Domain-specific interfaces

### ✅ **Testable**
- Mock all hardware dependencies
- Unit test business logic
- Integration testing support

### ✅ **Extensible**
- Easy to add new joystick types
- Easy to add new motor types
- Easy to add sensors (encoders, IMU)

### ✅ **Practical**
- Not over-engineered
- Uses existing GPIO/PWM code
- Simple to understand and modify

This architecture provides the right balance between **simplicity** and **flexibility**, enabling both rapid development and long-term maintainability.