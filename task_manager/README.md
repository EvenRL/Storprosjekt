# Task Manager

A ROS2 node for coordinating the detection and interaction with colored cubes using a robot arm ( work with ur3).

## Overview

The Task Manager node coordinates a robot's actions to:
1. Move to different observation positions
2. Process image data to detect green, blue, and black cubes
3. Point to each detected cube in sequence
4. Perform additional searches if some cubes are not initially detected

## Installation

### Prerequisites
- ROS2 (Humble or later)
- Python 3.8+

### Building from Source

1. Create a ROS2 workspace (if you don't have one):
   ```bash
   mkdir -p ~/ws_moveit/src
   cd ~/ws_moveit/src
   ```

2. Clone the repository:
   ```bash
   git clone git@github.com:EvenRL/Storprosjekt.git
   ```

3. Build the package:
   ```bash
   cd ~/ws_moveit
   colcon build --packages-select task_manager
   ```

4. Source the workspace:
   ```bash
   source ~/ws_moveit/install/setup.bash
   ```

## Usage



### Running the Node

`task_manager` directory structure:

task_manager/
├── launch/
│   └── task_manager.launch.py
├── package.xml
├── resource/
│   └── task_manager
├── setup.cfg
├── setup.py
└── task_manager/
    ├── __init__.py
    └── task_manager_node.py

To start the Task Manager:

```bash
ros2 run task_manager task_manager_node
```

### Starting the Task

The task can be started by publishing a message to the task_command topic:

```bash
ros2 topic pub /task_command std_msgs/String "data: 'start'" -1
```

## Topics

### Subscribed Topics
- `/detected_cubes` (geometry_msgs/PoseArray): Cube detection results
  - The frame_id contains color information in format: "color:green,blue,black"
  - Each pose in the array represents a detected cube
- `/robot_status` (std_msgs/String): Status updates from the robot controller
  - "movement_complete": Indicates that a robot movement has completed

### Published Topics
- `/task_status` (std_msgs/String): Current status of the task
- `/robot_command` (std_msgs/String): Commands to control the robot

## Robot Commands

The task manager sends robot commands as String messages with the following formats:

- **Joint Movement**: `move_joints:j1,j2,j3,j4,j5,j6`
  - Example: `move_joints:0.0,-1.57,0.0,-1.57,0.0,0.0`
  
- **Point To Position**: `point_to:x,y,z`
  - Example: `point_to:0.3,0.2,0.05`

## State Machine

The task manager implements a state machine with the following states:

1. **idle**: Waiting to start
2. **moving_home**: Moving robot to home position
3. **moving_to_overview**: Moving to overview position to detect cubes
4. **capturing_image**: Waiting for camera to capture image
5. **detecting_cubes**: Processing image to detect cubes
6. **pointing_sequence**: Pointing to each detected cube in sequence
7. **search_init**: Planning search for missing cubes
8. **searching**: Moving to search position
9. **processing_search**: Processing search results
10. **returning_home**: Returning to home position
11. **task_complete**: Task finished

## Configuration

Robot positions can be configured by modifying the following parameters in the code:

```python
self.home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
self.overview_position = [0.0, -1.0, 0.5, -1.0, 0.0, 0.0]
self.search_positions = [
    [0.3, -0.8, 0.5, -1.0, 0.0, 0.0],
    [-0.3, -0.8, 0.5, -1.0, 0.0, 0.0]
]
```

The cube colors to detect can be configured with:

```python
self.required_colors = ['green', 'blue', 'black']
```

## Troubleshooting

### No Cube Detections

If the task manager reports missing cubes:

1. Verify that the cube detector is publishing to the `/detected_cubes` topic
2. Check that the frame_id format matches the expected pattern: "color:green,blue,black"
3. Ensure the cube detector can recognize the colors you're looking for

### Robot Movement Issues

If the robot doesn't move as expected:

1. Check that the robot controller is subscribing to the `/robot_command` topic
2. Verify the robot controller is publishing "movement_complete" on the `/robot_status` topic
3. Ensure the joint positions are within the robot's range of motion
