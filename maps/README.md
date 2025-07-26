This file sets up a simplified container yard in CARLA using object placement.

## File
- `container_yard.py`

## Description
- Based on Town07 map in CARLA
- Spawns:
  - Containers
  - Pergolas (structures)
- Used to simulate a logistics scenario for vehicle maneuvering

## Highlights
- Uses CARLA API to spawn static props
- Placement is deterministic, useful for repeatable tests

## Tips
- Run this before starting control scripts
- Combine with `PID_control_.py` to test narrow path + curve behavior
