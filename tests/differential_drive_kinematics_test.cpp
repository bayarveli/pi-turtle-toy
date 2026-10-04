#include <cmath>
#include <cstdio>
#include <limits>

#include "differential_drive_kinematics.hpp"

namespace {
bool nearly_equal(float actual, float expected)
{
    return std::abs(actual - expected) < 0.0001f;
}

bool check(bool condition, const char* name)
{
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", name);
    }
    return condition;
}
} // namespace

int main()
{
    int failures = 0;
    WheelAngularVelocities wheels{};

    bool valid = calculate_wheel_angular_velocities(1.0f, 0.0f, 0.05f, 0.2f, 100.0f, wheels);
    failures += !check(valid && nearly_equal(wheels.left_rad_per_second, 20.0f) &&
                           nearly_equal(wheels.right_rad_per_second, 20.0f),
                       "straight motion produces equal wheel speeds");

    valid = calculate_wheel_angular_velocities(0.0f, 1.0f, 0.05f, 0.2f, 100.0f, wheels);
    failures += !check(valid && nearly_equal(wheels.left_rad_per_second, -2.0f) &&
                           nearly_equal(wheels.right_rad_per_second, 2.0f),
                       "in-place turn drives wheels in opposite directions");

    valid = calculate_wheel_angular_velocities(1.0f, 2.0f, 0.1f, 0.2f, 6.0f, wheels);
    failures += !check(valid && nearly_equal(wheels.left_rad_per_second, 4.0f) &&
                           nearly_equal(wheels.right_rad_per_second, 6.0f),
                       "speed saturation scales both wheels proportionally");

    const WheelAngularVelocities unchanged = wheels;
    valid = calculate_wheel_angular_velocities(1.0f, 0.0f, 0.0f, 0.2f, 6.0f, wheels);
    failures += !check(!valid && wheels.left_rad_per_second == unchanged.left_rad_per_second &&
                           wheels.right_rad_per_second == unchanged.right_rad_per_second,
                       "invalid geometry is rejected without changing the result");

    valid = calculate_wheel_angular_velocities(
        std::numeric_limits<float>::infinity(), 0.0f, 0.05f, 0.2f, 6.0f, wheels);
    failures += !check(!valid, "non-finite commands are rejected");

    return failures == 0 ? 0 : 1;
}