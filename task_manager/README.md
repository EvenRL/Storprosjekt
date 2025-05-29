# Task Manager

A ROS2 node for coordinating the detection and interaction with colored cubes using a robot arm (works with UR3).
detects green, blue, and black cubes, points to them in sequence, and performs additional searches if some cubes are not initially detected.
## Overview
The Task Manager node coordinates a robot's actions to:
1. Move to a home position
2. Move to an overview position to detect cubes
3. Capture an image for cube detection
4. Process the image to find colored cubes
5. Point to each detected cube in sequence
6. If some cubes are not detected, it will initiate a search sequence to find them
7. Return to the home position after completing the task
8. Report task status and robot commands via ROS2 topics
9. Handle robot movements using a state machine
10. Log status updates and errors
## Directory Structure

```
task_manager/
├── launch/
│   └── task_manager.launch.py
├── package.xml
├── tests/
│   ├── integrated_test.py
├── task_manager/
│   └── task_manager
├── setup.cfg
├── setup.py
└── task_manager/
    ├── __init__.py
    └── task_manager_node.py
        /core/
            ├── cube_detector.py
            ├── robot_controller.py
            └── state_machine.py
            ├── robot_controller.py
            └── state_machine.py
            
```

## Installation

### Prerequisites
- ROS2 (Humble or later)
- Python 3.8+

- Required ROS2 packages installed:
  - `geometry_msgs`
  - `std_msgs`
  - `rclpy`
  - `sensor_msgs`
  - `image_transport`
  - `cv_bridge`
  - `opencv-python`
  - `opencv-python-headless`
  - `moveit_commander` 
  - `moveit_msgs`
  - `tf2_ros`
  - `tf2_geometry_msgs`
  - `tf2_py`
  - `tf2_sensor_msgs`
  - `tf2_tools`
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
5. Install additional dependencies:
   ```bash
   rosdep install -i --from-path src --rosdistro humble -y
   ```
6. Install moveit python packages:
    ```bash
    sudo apt update
    sudo apt install ros-rolling-moveit
    sudo apt install python3-moveit-commander
    sudo apt install ros-rolling-rmw-cyclonedds-cpp
    ```

## Usage

# Running the Node

To start the Task Manager:

```bash
ros2 launch task_manager task_manager.launch.py
```
```bash
ros2 launch task_manager task_manager.launch.py enable_integrated_test:=true enable_auto_start:=true debug:=true
```
This will launch the Task Manager node along with any required dependencies.
# Testing services

### Start the task execution
```bash
ros2 service call /task_control std_srvs/srv/Trigger {}
````
### Move to home position
```bash
ros2 service call /move_to_home std_srvs/srv/Trigger {}
````
### Pause the task
```bash
ros2 service call /pause_task std_srvs/srv/SetBool "{data: true}"
````
### Resume the task
```bash
ros2 service call /pause_task std_srvs/srv/SetBool "{data: false}"
````
### Stop the task
```bash
ros2 service call /task_manager/stop_task std_srvs/srv/Trigger
```
### View task status messages
```bash
ros2 topic echo /task_status
```
### View detected cubes
```bash
ros2 topic echo /detected_cubes
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
- `/move_to_home` (std_srvs/srv/Trigger): Service to move the robot to home position
- `/move_to_overview` (std_srvs/srv/Trigger): Service to move the robot to overview position
- `/capture_overview_image` (std_srvs/srv/Trigger): Service to capture an image for cube detection

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

## Example Output

When running the task manager, you should see output similar to this:

```
[INFO] [launch]: All log files can be found below /home/wahid-pc/.ros/log/2025-05-29-21-17-08-154602-wahid-pc-OptiPlex-3070-94768
[INFO] [launch]: Default logging verbosity is set to INFO
/opt/ros/rolling/lib/python3.12/site-packages/launch/conditions/launch_configuration_equals.py:53: UserWarning: The 'LaunchConfigurationEquals' and 'LaunchConfigurationNotEquals' Conditions are  deprecated. Use the 'EqualsSubstitution' and 'NotEqualsSubstitution' substitutions instead! E.g.:
  IfCondition(
  	EqualsSubstitution(LaunchConfiguration('some_launch_arg'), "some_equality_check")
  )
  warnings.warn(
[INFO] [launch.user]: Starting Task Manager with sim_mode=true
[INFO] [launch.user]: Required colors: [green,black,blue]
[INFO] [launch.user]: Auto-start enabled: true
[INFO] [task_manager_node-1]: process started with pid [94771]
[INFO] [integrated_test-2]: process started with pid [94772]
[INFO] [rqt_console-3]: process started with pid [94773]
[INFO] [rqt_graph-4]: process started with pid [94774]
[INFO] [rqt_topic-5]: process started with pid [94775]
[rqt_topic-5] QSocketNotifier: Can only be used with threads started with QThread
[rqt_graph-4] QSocketNotifier: Can only be used with threads started with QThread
[rqt_console-3] QSocketNotifier: Can only be used with threads started with QThread
[integrated_test-2] [INFO] [1748546228.903234783] [integrated_tester]: Integrated tester started
[task_manager_node-1] no moveit !
[task_manager_node-1] [INFO] [1748546229.022295169] [task_manager]: Initializing Task Manager Node
[task_manager_node-1] [INFO] [1748546229.057009810] [task_manager]: Auto-starting detection and pointing task
[task_manager_node-1] [INFO] [1748546229.057601069] [task_manager]: Task Manager Node initialized with:
[task_manager_node-1] [INFO] [1748546229.058153817] [task_manager]: - Required colors: green, black, blue
[task_manager_node-1] [INFO] [1748546229.058710783] [task_manager]: - 2 search positions configured
[task_manager_node-1] [INFO] [1748546229.059277333] [task_manager]: - Running in simulation mode
[task_manager_node-1] [INFO] [1748546229.059965068] [task_manager]: System initialized and ready
[integrated_test-2] [INFO] [1748546229.061317438] [integrated_tester]: Task status: System initialized and ready
[task_manager_node-1] [INFO] [1748546229.386021301] [task_manager]: Detected green cube at position: (0.30, 0.10, 0.05)
[task_manager_node-1] [INFO] [1748546229.386542703] [task_manager]: Detected black cube at position: (0.25, -0.15, 0.05)
[task_manager_node-1] [INFO] [1748546229.387001355] [task_manager]: Detected blue cube at position: (0.35, 0.00, 0.05)
```
## Troubleshooting
If you encounter issues, check the following:
- Ensure all required ROS2 packages are installed
- Verify that the robot arm is properly configured and connected
- Check the camera is functioning and can capture images