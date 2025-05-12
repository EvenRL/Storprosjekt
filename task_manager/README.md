# task_manager

This package implements the task logic for deciding what the robot should do based on detected cubes.

## Nodes

### `task_manager.py`

- Reads cube information from `/color_cubes`.
- Uses `tf2_ros` to transform cube poses from `camera_link` to `base_link`.
- Sends a movement request to the robot controller to interact with selected cubes.

## Dependencies

- `cube_interfaces`
- `tf2_ros`
- `geometry_msgs`
- `rclpy`

## Services

- `/move_to_pose`