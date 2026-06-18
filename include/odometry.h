// Odometry and speed calculation for differential drive robot
// Tracks robot position (x, y, heading) and wheel speeds

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <cmath>
#include <chrono>

class Odometry {
 public:
  // Robot physical parameters (all in meters)
  struct RobotParams {
    double wheel_diameter;    // Wheel diameter in meters (e.g., 0.065 for 65mm)
    double wheelbase;         // Distance between left and right wheels
    double pulses_per_rev;    // Encoder PPR (20 for SEN0038 with EDGE_RISING)
  };

  // Robot pose (position and orientation)
  struct Pose {
    double x;         // X position in meters
    double y;         // Y position in meters
    double heading;   // Heading angle in radians (0 = facing +X axis)
  };

  // Wheel speeds
  struct Speed {
    double left;      // Left wheel speed in m/s
    double right;     // Right wheel speed in m/s
    double linear;    // Robot linear speed (forward) in m/s
    double angular;   // Robot angular speed (rotation) in rad/s
  };

  explicit Odometry(const RobotParams& params);

  // Update odometry with new encoder counts
  // Call this periodically (e.g., every 50-100ms)
  void update(long left_encoder_count, long right_encoder_count);

  // Get current robot pose
  Pose get_pose() const { return pose_; }

  // Get current speeds (based on last update)
  Speed get_speed() const { return speed_; }

  // Get total distance traveled
  double get_total_distance() const { return total_distance_; }

  // Reset odometry to origin
  void reset();

  // Set current pose (for localization corrections)
  void set_pose(double x, double y, double heading);

 private:
  RobotParams params_;
  Pose pose_;
  Speed speed_;
  
  double wheel_circumference_;
  double total_distance_;
  
  // Previous encoder values for delta calculation
  long prev_left_count_;
  long prev_right_count_;
  
  // Timing for speed calculation
  std::chrono::steady_clock::time_point prev_time_;
};

#endif /* ODOMETRY_H_ */
