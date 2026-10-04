#pragma once

#include <algorithm>
#include <cmath>

struct WheelAngularVelocities {
    float left_rad_per_second;
    float right_rad_per_second;
};

inline bool calculate_wheel_angular_velocities(
    float linear_meters_per_second,
    float yaw_radians_per_second,
    float wheel_radius_meters,
    float wheel_track_meters,
    float max_wheel_radians_per_second,
    WheelAngularVelocities& result)
{
    if (!std::isfinite(linear_meters_per_second) ||
        !std::isfinite(yaw_radians_per_second) ||
        !std::isfinite(wheel_radius_meters) || wheel_radius_meters <= 0.0f ||
        !std::isfinite(wheel_track_meters) || wheel_track_meters <= 0.0f ||
        !std::isfinite(max_wheel_radians_per_second) || max_wheel_radians_per_second <= 0.0f) {
        return false;
    }

    const float half_track_yaw = yaw_radians_per_second * wheel_track_meters * 0.5f;
    float left = (linear_meters_per_second - half_track_yaw) / wheel_radius_meters;
    float right = (linear_meters_per_second + half_track_yaw) / wheel_radius_meters;
    if (!std::isfinite(left) || !std::isfinite(right)) {
        return false;
    }

    const float peak_speed = std::max(std::abs(left), std::abs(right));
    if (peak_speed > max_wheel_radians_per_second) {
        const float scale = max_wheel_radians_per_second / peak_speed;
        left *= scale;
        right *= scale;
    }

    result = {left, right};
    return true;
}