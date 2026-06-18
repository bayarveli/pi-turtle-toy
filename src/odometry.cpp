// Odometry implementation for differential drive robot

#include "odometry.h"
#include <cmath>

Odometry::Odometry(const RobotParams& params)
    : params_(params),
      pose_{0.0, 0.0, 0.0},
      speed_{0.0, 0.0, 0.0, 0.0},
      total_distance_(0.0),
      prev_left_count_(0),
      prev_right_count_(0),
      prev_time_(std::chrono::steady_clock::now())
{
  wheel_circumference_ = M_PI * params_.wheel_diameter;
}

void Odometry::update(long left_encoder_count, long right_encoder_count)
{
  // Get current time
  auto current_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_diff = current_time - prev_time_;
  double dt = time_diff.count();
  
  // Avoid division by zero on first call or very fast updates
  if (dt < 0.001) {
    return;
  }
  
  // Calculate pulse deltas
  long left_pulses = left_encoder_count - prev_left_count_;
  long right_pulses = right_encoder_count - prev_right_count_;
  
  // Convert pulses to distance (meters)
  double left_distance = (left_pulses / params_.pulses_per_rev) * wheel_circumference_;
  double right_distance = (right_pulses / params_.pulses_per_rev) * wheel_circumference_;
  
  // Calculate wheel speeds (m/s)
  speed_.left = left_distance / dt;
  speed_.right = right_distance / dt;
  
  // Calculate robot motion
  double center_distance = (left_distance + right_distance) / 2.0;
  double heading_change = (right_distance - left_distance) / params_.wheelbase;
  
  // Calculate linear and angular speeds
  speed_.linear = center_distance / dt;
  speed_.angular = heading_change / dt;
  
  // Update pose using simple integration
  // For small time steps, this approximation is quite accurate
  if (std::abs(heading_change) < 0.001) {
    // Moving nearly straight - avoid division by small numbers
    pose_.x += center_distance * std::cos(pose_.heading);
    pose_.y += center_distance * std::sin(pose_.heading);
  } else {
    // Turning - use arc motion equations
    double turning_radius = center_distance / heading_change;
    double old_heading = pose_.heading;
    
    pose_.heading += heading_change;
    
    // Normalize heading to [-π, π]
    while (pose_.heading > M_PI) pose_.heading -= 2.0 * M_PI;
    while (pose_.heading < -M_PI) pose_.heading += 2.0 * M_PI;
    
    // Update position along arc
    pose_.x += turning_radius * (std::sin(pose_.heading) - std::sin(old_heading));
    pose_.y += turning_radius * (std::cos(old_heading) - std::cos(pose_.heading));
  }
  
  // Update total distance
  total_distance_ += std::abs(center_distance);
  
  // Store current values for next iteration
  prev_left_count_ = left_encoder_count;
  prev_right_count_ = right_encoder_count;
  prev_time_ = current_time;
}

void Odometry::reset()
{
  pose_ = {0.0, 0.0, 0.0};
  speed_ = {0.0, 0.0, 0.0, 0.0};
  total_distance_ = 0.0;
  prev_left_count_ = 0;
  prev_right_count_ = 0;
  prev_time_ = std::chrono::steady_clock::now();
}

void Odometry::set_pose(double x, double y, double heading)
{
  pose_.x = x;
  pose_.y = y;
  pose_.heading = heading;
  
  // Normalize heading
  while (pose_.heading > M_PI) pose_.heading -= 2.0 * M_PI;
  while (pose_.heading < -M_PI) pose_.heading += 2.0 * M_PI;
}
