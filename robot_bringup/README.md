# robot_bringup

This package launches the full robot system including cube detection, task management, and robot control.

## Launch Files

### `bringup.launch.py`

Launches:
- Cube detection node
- Task manager node
- Robot controller node

## Dependencies

- `cube_detection`
- `task_manager`
- `robot_controller`

## Usage

```bash
ros2 launch robot_bringup bringup.launch.py