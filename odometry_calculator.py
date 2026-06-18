#!/usr/bin/env python3
"""
Visual demonstration of odometry calculations
Shows how encoder pulses translate to robot position
"""

import math

def print_header(title):
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60)

# Robot configuration
WHEEL_DIAMETER = 0.065  # meters (65mm)
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
WHEELBASE = 0.15  # meters (150mm)
PPR = 20.0  # Pulses per revolution

print_header("ENCODER TO ODOMETRY CONVERSION")

# Example 1: Straight line motion
print_header("Example 1: Straight Line Motion")
print("Scenario: Both wheels rotate same amount")
print(f"  Left encoder:  40 pulses")
print(f"  Right encoder: 40 pulses")
print()

left_pulses = 40
right_pulses = 40

left_dist = (left_pulses / PPR) * WHEEL_CIRCUMFERENCE
right_dist = (right_pulses / PPR) * WHEEL_CIRCUMFERENCE
center_dist = (left_dist + right_dist) / 2.0
heading_change = (right_dist - left_dist) / WHEELBASE

print(f"Calculations:")
print(f"  Left distance:  {left_dist:.3f} m")
print(f"  Right distance: {right_dist:.3f} m")
print(f"  Center distance: {center_dist:.3f} m")
print(f"  Heading change: {heading_change:.3f} rad ({math.degrees(heading_change):.1f}°)")
print()
print(f"Result: Robot moved {center_dist:.3f}m straight forward")

# Example 2: Turning right
print_header("Example 2: Turning Right")
print("Scenario: Right wheel moves more than left")
print(f"  Left encoder:  30 pulses")
print(f"  Right encoder: 50 pulses")
print()

left_pulses = 30
right_pulses = 50

left_dist = (left_pulses / PPR) * WHEEL_CIRCUMFERENCE
right_dist = (right_pulses / PPR) * WHEEL_CIRCUMFERENCE
center_dist = (left_dist + right_dist) / 2.0
heading_change = (right_dist - left_dist) / WHEELBASE

print(f"Calculations:")
print(f"  Left distance:  {left_dist:.3f} m")
print(f"  Right distance: {right_dist:.3f} m")
print(f"  Center distance: {center_dist:.3f} m")
print(f"  Heading change: {heading_change:.3f} rad ({math.degrees(heading_change):.1f}°)")
print()
print(f"Result: Robot moved {center_dist:.3f}m forward while turning {math.degrees(heading_change):.1f}° left")

# Example 3: Spinning in place
print_header("Example 3: Spinning in Place")
print("Scenario: Wheels move opposite directions")
print(f"  Left encoder:  -20 pulses (backward)")
print(f"  Right encoder: +20 pulses (forward)")
print()

left_pulses = -20
right_pulses = 20

left_dist = (left_pulses / PPR) * WHEEL_CIRCUMFERENCE
right_dist = (right_pulses / PPR) * WHEEL_CIRCUMFERENCE
center_dist = (left_dist + right_dist) / 2.0
heading_change = (right_dist - left_dist) / WHEELBASE

print(f"Calculations:")
print(f"  Left distance:  {left_dist:.3f} m (backward)")
print(f"  Right distance: {right_dist:.3f} m (forward)")
print(f"  Center distance: {center_dist:.3f} m")
print(f"  Heading change: {heading_change:.3f} rad ({math.degrees(heading_change):.1f}°)")
print()
print(f"Result: Robot rotated {math.degrees(heading_change):.1f}° in place (no forward motion)")

# Speed calculation example
print_header("Example 4: Speed Calculation")
print("Scenario: Robot moving forward for 100ms")
print(f"  Pulses counted: 10")
print(f"  Time period: 100ms (0.1 seconds)")
print()

pulses = 10
time_period = 0.1  # seconds

distance = (pulses / PPR) * WHEEL_CIRCUMFERENCE
speed_ms = distance / time_period
speed_kmh = speed_ms * 3.6

print(f"Calculations:")
print(f"  Distance: {distance:.3f} m")
print(f"  Speed: {speed_ms:.3f} m/s")
print(f"  Speed: {speed_kmh:.2f} km/h")

# Quick reference table
print_header("Quick Reference: Pulses → Distance")
print(f"With {WHEEL_DIAMETER*1000:.0f}mm wheels ({WHEEL_CIRCUMFERENCE:.3f}m circumference):\n")
print(f"{'Pulses':<10} {'Rotations':<12} {'Distance':<15}")
print("-" * 40)
for pulses in [1, 5, 10, 20, 40, 100]:
    rotations = pulses / PPR
    distance = rotations * WHEEL_CIRCUMFERENCE
    print(f"{pulses:<10} {rotations:<12.2f} {distance*1000:<12.1f} mm")

print_header("Quick Reference: Pulses/sec → Speed")
print(f"{'Pulses/s':<12} {'RPM':<12} {'m/s':<12} {'km/h':<12}")
print("-" * 50)
for pulses_per_sec in [10, 20, 50, 100, 200]:
    rpm = (pulses_per_sec / PPR) * 60
    mps = (pulses_per_sec / PPR) * WHEEL_CIRCUMFERENCE
    kmh = mps * 3.6
    print(f"{pulses_per_sec:<12} {rpm:<12.1f} {mps:<12.2f} {kmh:<12.2f}")

print("\n" + "="*60)
print("  Configuration used in examples:")
print(f"    Wheel diameter: {WHEEL_DIAMETER*1000:.0f} mm")
print(f"    Wheelbase: {WHEELBASE*1000:.0f} mm")
print(f"    Encoder PPR: {PPR:.0f}")
print("="*60 + "\n")
