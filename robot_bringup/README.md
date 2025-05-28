# robot_bringup

This package launches the full robot system including cube detection, task management, and robot control.

## Launch Files

### `bringup.launch.py`

Launches:
- Static transform from camera frame to tool frame
- Cube detection node
- Task manager node

## Dependencies

- `cube_detection`
- `task_manager`
- `tf2`

## Usage

```bash
ros2 launch robot_bringup bringup.launch.py