# Single Vehicle PID Control

This script implements PID-based autonomous control of a single vehicle in CARLA simulator.

## File
- `PID_control.py`

## Objective
- Follow a predefined waypoint path using:
  - Longitudinal PID control for throttle
  - Lateral PID control for steering
- Stop smoothly at a designated location using distance-based trigger

## Key Components
- `PIDController` class: Generic PID control logic
- `find_closest_index()`: Finds nearest waypoint to current position
- `load_waypoints()`: Reads waypoints from `.csv` file
- Hardcoded stop point (`stop_location`) with brake logic
- Target speed: 8 m/s (adjustable)

## Usage Notes
- Meant to test control behavior in isolation
- Works best with straight or mild curve paths

