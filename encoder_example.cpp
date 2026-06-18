// Example: Using wheel encoders with GPIO interrupts on Raspberry Pi
// Demonstrates how to track wheel rotation for odometry

#include <iostream>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cstdint>
#include "encoder.h"

// Global flag for graceful shutdown
std::atomic<bool> keep_running(true);

void signal_handler(int signum) {
  std::cout << "\nReceived signal " << signum << ", shutting down...\n";
  keep_running.store(false);
}

int main()
{
  // Setup signal handler for clean exit (Ctrl+C)
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  try {
    /* Wheel encoder pin mapping example:
     *   +-------------+-----------+-----------+----------+
     *   |    Role     |  BCM GPIO | Phys Pin  | sysfs id |
     *   +-------------+-----------+-----------+----------+
     *   | Left Wheel  |   GPIO17  |    11     |   529    |
     *   | Right Wheel |   GPIO27  |    13     |   539    |
     *   +-------------+-----------+-----------+----------+
     * Note: Use BCM GPIO numbers for the Encoder constructor
     */

    // Create encoder objects for left and right wheels
    // Using EDGE_RISING for 20 counts/rev (matches 20 PPR spec)
    // Change to EDGE_BOTH for 40 counts/rev if higher resolution needed
    Encoder left_encoder("17", GpioInputPin::EDGE_RISING);
    Encoder right_encoder("27", GpioInputPin::EDGE_RISING);

    // Optional: Set callbacks for real-time processing
    left_encoder.set_callback([](std::uint32_t count) {
      // This runs on every pulse - keep it fast!
      // You could update odometry calculations here
    });

    // Start interrupt monitoring
    std::cout << "Starting encoder monitoring...\n";
    left_encoder.start();
    right_encoder.start();

    // Main loop - monitor encoder counts
    std::cout << "Encoders active. Press Ctrl+C to exit.\n";
    std::cout << "Format: [Left Count] [Right Count]\n\n";

    unsigned long loop_count = 0;
    while (keep_running.load()) {
      // Read encoder counts (thread-safe)
      std::uint32_t left_count = left_encoder.get_count();
      std::uint32_t right_count = right_encoder.get_count();

      // Print status every 10 loops (~1 second)
      if (loop_count % 10 == 0) {
        std::cout << "Left: " << left_count 
                  << "  Right: " << right_count << "\r" << std::flush;
      }

      loop_count++;
      usleep(100000); // 100ms delay
    }

    std::cout << "\n\nStopping encoders...\n";
    left_encoder.stop();
    right_encoder.stop();

    // Final counts
    std::cout << "Final counts - Left: " << left_encoder.get_count() 
              << "  Right: " << right_encoder.get_count() << "\n";

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
  }

  std::cout << "Shutdown complete.\n";
  return 0;
}
