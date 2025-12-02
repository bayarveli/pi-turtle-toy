#include <iostream>
#include <chrono>
#include <thread>
#include <poll.h>

#include <gpiod.hpp>

int main() {
    try {
        auto request =
            ::gpiod::chip("/dev/gpiochip0")
            .prepare_request()
            .set_consumer("async_pulse_counter")
            .add_line_settings(
                27,
                ::gpiod::line_settings()
                    .set_direction(::gpiod::line::direction::INPUT)
                    .set_edge_detection(::gpiod::line::edge::RISING)
                    .set_bias(::gpiod::line::bias::PULL_UP)
                    .set_debounce_period(std::chrono::microseconds(2000))
            )
            .do_request();

        ::gpiod::edge_event_buffer event_buffer(64);

        struct pollfd pollfd;
        pollfd.fd = request.fd();
        pollfd.events = POLLIN;

        int pulse_count = 0;
        auto last_print = std::chrono::steady_clock::now();
        const auto print_interval = std::chrono::seconds(1);

        for (;;) {
            auto ret = poll(&pollfd, 1, 100); // 100ms timeout for periodic print
            if (ret == -1) {
                ::std::cerr << "error waiting for event: " << '\n';
                return EXIT_FAILURE;
            }

            if (ret > 0) {
                request.read_edge_events(event_buffer);
                for (const auto& event : event_buffer) {
                    ++pulse_count;
                }
            }

            auto now = std::chrono::steady_clock::now();
            if (now - last_print >= print_interval) {
                std::cout << "Pulse count in last second: " << pulse_count << std::endl;
                pulse_count = 0;
                last_print = now;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
