# Speed and Odometry Calculations - Detailed Guide

## Quick Reference Formulas

### Speed Calculation
```
speed (m/s) = distance / time
distance = (pulses / PPR) × wheel_circumference
wheel_circumference = π × wheel_diameter
```

### Odometry (Position Tracking)
```
left_distance = (left_pulses / PPR) × wheel_circumference
right_distance = (right_pulses / PPR) × wheel_circumference
center_distance = (left_distance + right_distance) / 2

// Straight motion:
x += center_distance × cos(heading)
y += center_distance × sin(heading)

// Turning:
heading_change = (right_distance - left_distance) / wheelbase
heading += heading_change
```

## Part 1: Speed Calculation

### The Basics

**Speed = How far you moved / How long it took**

For a wheel encoder:
1. Count pulses over a time period (e.g., 100ms)
2. Convert pulses → rotations → distance
3. Divide distance by time

### Example with Real Numbers

Given your setup:
- **Encoder**: 20 PPR (pulses per revolution)
- **Wheel diameter**: 65mm = 0.065m
- **Wheel circumference**: π × 0.065 = 0.204m (distance per rotation)

**Scenario**: You count 10 pulses in 100ms

```
Step 1: Pulses → Rotations
  rotations = 10 pulses / 20 PPR = 0.5 rotations

Step 2: Rotations → Distance
  distance = 0.5 rotations × 0.204m = 0.102m

Step 3: Distance → Speed
  speed = 0.102m / 0.1s = 1.02 m/s (3.67 km/h)
```

### In Code

```cpp
// Configuration
const double PPR = 20.0;
const double WHEEL_DIAMETER = 0.065;  // meters
const double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;

// Every 100ms:
long current_count = encoder.get_count();
long pulse_delta = current_count - previous_count;
double time_delta = 0.1;  // 100ms = 0.1 seconds

// Calculate speed
double distance = (pulse_delta / PPR) * WHEEL_CIRCUMFERENCE;
double speed = distance / time_delta;  // m/s

previous_count = current_count;
```

### Speed Ranges

With your 65mm wheels:

| Pulses/100ms | Rotations/s | Speed (m/s) | Speed (km/h) |
|--------------|-------------|-------------|--------------|
| 5            | 2.5         | 0.51        | 1.84         |
| 10           | 5.0         | 1.02        | 3.67         |
| 20           | 10.0        | 2.04        | 7.34         |
| 50           | 25.0        | 5.10        | 18.36        |

---

## Part 2: Odometry (Position Tracking)

### What is Odometry?

**Odometry** = Tracking where your robot is by measuring wheel rotations.

Think of it like a car's odometer, but it also tracks:
- **Position** (x, y coordinates)
- **Heading** (which direction you're facing)

### Differential Drive Geometry

Your robot has two wheels:

```
    Left Wheel (L)          Right Wheel (R)
         ↓                        ↓
    ┌────●────────────────────────●────┐
    │                                  │
    │         Robot Center             │  → heading (θ)
    │                                  │
    └──────────────────────────────────┘
              
    ←────── Wheelbase (W) ──────→
```

### Three Motion Cases

#### Case 1: Straight Forward (L = R)

Both wheels move the same distance:

```
left_distance = right_distance = 0.5m

// Robot moves straight forward
forward_distance = 0.5m
x += forward_distance × cos(heading)
y += forward_distance × sin(heading)
heading stays the same
```

#### Case 2: Pure Rotation (L = -R)

Wheels move opposite directions:

```
left_distance = 0.2m
right_distance = -0.2m

// Robot spins in place
distance_diff = -0.2 - 0.2 = -0.4m
heading_change = -0.4 / wheelbase
heading += heading_change
x and y stay the same (spinning in place)
```

#### Case 3: Arc Motion (L ≠ R)

Wheels move different distances:

```
left_distance = 0.3m
right_distance = 0.5m

// Robot follows a curved path
The robot turns while moving forward
```

### The Math for Arc Motion

This is the trickiest part. When wheels move different distances, the robot follows a circular arc:

```
1. Calculate average distance (center of robot moves this far):
   center_distance = (left + right) / 2

2. Calculate heading change:
   heading_change = (right - left) / wheelbase

3. Update heading:
   new_heading = old_heading + heading_change

4. Calculate new position along the arc:
   radius = center_distance / heading_change
   x += radius × (sin(new_heading) - sin(old_heading))
   y += radius × (cos(old_heading) - cos(new_heading))
```

### Practical Example

**Given**:
- Wheelbase: 150mm = 0.15m
- Left wheel: 40 pulses (= 0.408m with 65mm wheel)
- Right wheel: 60 pulses (= 0.612m)

**Calculate**:

```
Step 1: Distances
  left_distance = (40/20) × 0.204 = 0.408m
  right_distance = (60/20) × 0.204 = 0.612m

Step 2: Center distance and heading change
  center_distance = (0.408 + 0.612) / 2 = 0.510m
  heading_change = (0.612 - 0.408) / 0.15 = 1.36 radians (78°)

Step 3: Robot turned right while moving forward 0.51m
  The robot followed a curved path, turning 78° to the right
```

---

## Part 3: Using the Odometry Class

### Setup

```cpp
#include "odometry.h"

// Configure your robot
Odometry::RobotParams params;
params.wheel_diameter = 0.065;    // 65mm wheels
params.wheelbase = 0.15;          // 150mm between wheels (MEASURE!)
params.pulses_per_rev = 20.0;     // 20 PPR with EDGE_RISING

Odometry odom(params);
```

### Main Loop

```cpp
while (running) {
  // Get encoder counts
  long left_count = left_encoder.get_count();
  long right_count = right_encoder.get_count();
  
  // Update odometry (call every 50-100ms)
  odom.update(left_count, right_count);
  
  // Read results
  auto pose = odom.get_pose();
  auto speed = odom.get_speed();
  
  std::cout << "Position: (" << pose.x << ", " << pose.y << ")\n";
  std::cout << "Heading: " << pose.heading << " radians\n";
  std::cout << "Speed: " << speed.linear << " m/s\n";
  
  usleep(100000);  // 100ms
}
```

### What You Get

```cpp
// Position
pose.x          // X coordinate in meters
pose.y          // Y coordinate in meters  
pose.heading    // Direction in radians (0 = east, π/2 = north)

// Speed
speed.left      // Left wheel speed (m/s)
speed.right     // Right wheel speed (m/s)
speed.linear    // Forward speed (m/s)
speed.angular   // Rotation speed (rad/s)

// Total
total_distance  // Total meters traveled
```

---

## Part 4: Important Measurements

### You MUST Measure These

1. **Wheel Diameter**: Measure or check spec sheet
   - Use calipers for accuracy
   - Measure actual rolling diameter (tire may compress)

2. **Wheelbase**: Distance between wheel centers
   - Measure from center of left wheel to center of right wheel
   - Critical for accurate turning calculations

3. **Encoder PPR**: Check encoder specification
   - SEN0038 = 20 PPR
   - If using EDGE_BOTH, effective PPR = 40

### Calibration Tips

**Test straight line**:
- Drive robot forward 1 meter
- Check if odometry says 1 meter
- If off, adjust wheel_diameter

**Test rotation**:
- Spin robot 360° in place
- Check if heading returns to ~0
- If off, adjust wheelbase

---

## Part 5: Common Pitfalls

### ❌ Wrong Update Rate
```cpp
// BAD: Updating too fast (< 10ms)
usleep(5000);  // Noise dominates, poor speed calculation

// BAD: Updating too slow (> 500ms)  
usleep(1000000);  // Lose accuracy, jerky updates

// GOOD: 50-100ms sweet spot
usleep(100000);  // Smooth, accurate
```

### ❌ Not Resetting Between Readings
```cpp
// BAD: Forgetting to store previous count
long current_count = encoder.get_count();
// Missing: previous_count update!

// GOOD: Proper delta calculation
long delta = current_count - previous_count;
previous_count = current_count;  // Don't forget!
```

### ❌ Units Confusion
```cpp
// BAD: Mixing units
double diameter = 65;  // Is this mm? meters? inches??

// GOOD: Be explicit, use SI units (meters)
double diameter_m = 0.065;  // 65mm in meters
```

### ❌ Integer Division
```cpp
// BAD: Integer division loses precision
int pulses = 15;
int PPR = 20;
double rotations = pulses / PPR;  // = 0 (integer division!)

// GOOD: Use floating point
double rotations = pulses / 20.0;  // = 0.75
```

---

## Summary Checklist

- ✅ Update odometry every 50-100ms
- ✅ Measure wheel diameter and wheelbase accurately
- ✅ Use consistent units (meters, radians)
- ✅ Store previous counts for delta calculation
- ✅ Use floating-point math, not integer division
- ✅ Test and calibrate with known distances

## Build and Test

```bash
cd build
cmake ..
make OdometryExample
sudo ./OdometryExample
```

Drive your robot around and watch the position update in real-time!
