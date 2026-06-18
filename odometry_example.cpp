// Complete example: Encoders + Odometry for differential drive robot
// Shows real-time speed and position tracking

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cmath>
#include "encoder.h"
#include "odometry.h"

std::atomic<bool> keep_running(true);

void signal_handler(int signum) {
  keep_running.store(false);
}

// Convert radians to degrees for display
double rad_to_deg(double rad) {
  return rad * 180.0 / M_PI;
}

int main()
{
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  try {
    // Robot physical parameters (ADJUST THESE FOR YOUR ROBOT!)
    Odometry::RobotParams robot_params;
    robot_params.wheel_diameter = 0.065;    // 65mm wheels
    robot_params.wheelbase = 0.15;          // 150mm between wheels (MEASURE THIS!)
    robot_params.pulses_per_rev = 20.0;     // 20 PPR encoder with EDGE_RISING

    // Create odometry tracker
    Odometry odom(robot_params);

    // Create encoders (adjust GPIO pins for your wiring)
    Encoder left_encoder("17", GpioInputPin::EDGE_RISING);   // GPIO17
    Encoder right_encoder("27", GpioInputPin::EDGE_RISING);  // GPIO27

    // Start encoders
    std::cout << "Starting encoders and odometry...\n";
    left_encoder.start();
    right_encoder.start();

    std::cout << "\nRobot Configuration:\n";
    std::cout << "  Wheel diameter: " << (robot_params.wheel_diameter * 1000) << " mm\n";
    std::cout << "  Wheelbase: " << (robot_params.wheelbase * 1000) << " mm\n";
    std::cout << "  Encoder PPR: " << robot_params.pulses_per_rev << "\n";
    std::cout << "\nDrive the robot and watch odometry update!\n";
    std::cout << "Press Ctrl+C to exit.\n\n";

    // Main loop - update odometry periodically
    while (keep_running.load()) {
      // Get encoder counts
      long left_count = left_encoder.get_count();
      long right_count = right_encoder.get_count();

      // Update odometry
      odom.update(left_count, right_count);

      // Get current state
      auto pose = odom.get_pose();
      auto speed = odom.get_speed();
      double total_dist = odom.get_total_distance();

      // Display odometry (clear and rewrite for clean output)
      std::cout << "\r" << std::string(100, ' ') << "\r";  // Clear line
      std::cout << std::fixed << std::setprecision(3);
      
      std::cout << "Pos: (" 
                << std::setw(6) << pose.x << "m, "
                << std::setw(6) << pose.y << "m) "
                << "Hdg: " << std::setw(6) << rad_to_deg(pose.heading) << "° | "
                << "Speed: L=" << std::setw(5) << speed.left << " R=" << std::setw(5) << speed.right << " m/s | "
                << "Total: " << std::setw(6) << total_dist << "m | "
                << "Counts: L=" << left_count << " R=" << right_count
                << std::flush;

      // Update at 10 Hz (every 100ms)
      usleep(100000);
    }

    std::cout << "\n\nStopping encoders...\n";
    left_encoder.stop();
    right_encoder.stop();

    // Final summary
    auto final_pose = odom.get_pose();
    std::cout << "\n=== Final Odometry ===\n";
    std::cout << "Position: (" << final_pose.x << " m, " << final_pose.y << " m)\n";
    std::cout << "Heading: " << rad_to_deg(final_pose.heading) << " degrees\n";
    std::cout << "Total distance: " << odom.get_total_distance() << " m\n";
    std::cout << "Left encoder: " << left_encoder.get_count() << " pulses\n";
    std::cout << "Right encoder: " << right_encoder.get_count() << " pulses\n";

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
