# AIS2105 Mega-project
This repository is for the Mega-project assignment in subject AIS2105.

## Project Overview

The following project features the UR3 robot, where the group is focusing on programming and allowing it to detect, identify and point to colored cubes. The robot is assigned to:

* Capture an image with the connected camera
* Detect different cube colors
* Orientating around its workspace and point to its sequential cubes of color
* Use search mechanism if a cube is not captured, providing feedback of the missing cube.

## Project Structure

* Ros2-nodes is organized with launch files for configuration
* Camera for detection sweeping utilizes the implemented system camera using pipeline for color detection
* Movements are controlled with moveit, using ros2 for implementation
* Error handling by a search mechanism through attempted detection sweeping

## Requirements

* Ubuntu
* 2x.xx with Ros2 and move it
* UR-robot
* Surface tablet
* Camera
* ros2 packages for image processing and color detection
* moveit configuration for UR3 robot
* Python for node implementation
* OpenCV for image processing
* UR3 robot drivers and configuration files
```bash
sudo apt-get install ros-humble-moveit
sudo apt-get install ros-humble-image-transport
sudo apt-get install ros-humble-vision-msgs
sudo apt-get install ros-humble-image-pipeline
sudo apt-get install ros-humble-camera-info-manager
sudo apt-get install ros-humble-camera-info-manager-py
sudo apt-get install ros-humble-image-geometry
sudo apt-get install ros-humble-image-proc
sudo apt-get install ros-humble-cv-bridge
```

## Features
* Real-time color detection using camera input
  * UR3 robot control for movement and pointing
  * Sequential color identification and pointing
  * Search mechanism for missing cubes
  * Error handling and feedback system
  * Modular design for easy updates and maintenance

## Contribution

## License 