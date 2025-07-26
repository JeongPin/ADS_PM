# Double-Vehicle PID Control with Collision Avoidance

Extends single vehicle control to two vehicles operating simultaneously in CARLA.

## File
- `PID_control_2.py`

## Objective
- Control two vehicles independently with:
  - Separate waypoint paths
  - Independent stop points
  - Collision avoidance via FOV + distance detection

## Key Features
- Per-vehicle PID controller for speed + steering
- Dynamic `heading_error`-based curve slowdown
- Stop timer logic using `time.time()` per vehicle
- Vehicle-to-vehicle detection:
  - Front detection within ±45° FOV and < 8m range
  - If detected → stop, else continue
- Smooth resume after pause (non-blocking)

## Usage Notes
- Useful for logistics yard or intersection-style testing
- Realistic behavior emerges with speed + curve tuning

## Future Work
- Priority resolution (e.g. time-based fairness)
- Dynamic path re-routing
