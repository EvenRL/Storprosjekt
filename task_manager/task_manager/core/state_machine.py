import time
from typing import Optional
from .states import TaskState

class TaskStateMachine:
    def __init__(self, node, robot_movement, cube_detection, config):
        self.node = node
        self.robot = robot_movement
        self.cube_detection = cube_detection
        self.home_position = config['home_position']# Configuration
        self.overview_position = config['overview_position']
        self.search_positions = config['search_positions']
        self.required_colors = config['required_colors']
        self.pointing_distance = config['pointing_distance']
        self.movement_timeout = config['movement_timeout']
        self.retry_attempts = config['retry_attempts']
        self.recovery_behavior = config['recovery_behavior']
        self.state = TaskState.IDLE  # State variables
        self.current_color_index = 0
        self.search_index = 0
        self.movement_start_time: Optional[float] = None
        self.retry_count = 0
        self._paused_state = None

    def update(self):
        if (self.movement_start_time and# Check for movement timeout if applicable
                time.time() - self.movement_start_time > self.movement_timeout):
            self.handle_movement_timeout()
        # Hanle state transitions
        if self.state == TaskState.IDLE:
            # Nothing to do, waiting for command
            pass

        elif self.state == TaskState.INITIALIZING:
            self.node.publish_status("Initializing task")
            self.cube_detection.reset()
            self.current_color_index = 0
            self.search_index = 0
            self.retry_count = 0
            self.transition_to(TaskState.MOVING_HOME)
        elif self.state == TaskState.MOVING_HOME:
            if not self.movement_start_time:
                self.node.publish_status("Moving to home position")
                self.node.send_robot_command(f"move_joints:{self._format_position(self.home_position)}")
                self.movement_start_time = time.time()
        elif self.state == TaskState.MOVING_TO_OVERVIEW:
            if not self.movement_start_time:
                self.node.publish_status("Moving to overview position")
                self.node.send_robot_command(f"move_joints:{self._format_position(self.overview_position)}")
                self.movement_start_time = time.time()
        elif self.state == TaskState.SEARCHING:
            if not self.movement_start_time:
                if self.search_index >= len(self.search_positions):
                    self.handle_search_completion()# All search positions tried
                else:
                    pos = self.search_positions[self.search_index]# Move to next search position
                    self.node.publish_status(f"Moving to search position {self.search_index + 1}")
                    self.node.send_robot_command(f"move_joints:{self._format_position(pos)}")
                    self.movement_start_time = time.time()
        elif self.state == TaskState.PROCESSING_DETECTION:
            pass# This state is primarily entered via callbacks, not the state machine
        elif self.state == TaskState.POINTING:
            if not self.movement_start_time:
                if self.current_color_index >= len(self.required_colors):# All colors pointed to
                    self.transition_to(TaskState.RETURNING_HOME)
                else:
                    # Point to next color
                    color = self.required_colors[self.current_color_index]
                    if color in self.cube_detection.detected_cubes:
                        self.point_to_cube(color)
                    else:
                        self.node.get_logger().warn(f"Cube color {color} not detected, skipping")
                        self.current_color_index += 1
        elif self.state == TaskState.RETURNING_HOME:
            if not self.movement_start_time:
                self.node.publish_status("Returning to home position")
                self.node.send_robot_command(f"move_joints:{self._format_position(self.home_position)}")
                self.movement_start_time = time.time()
        elif self.state == TaskState.COMPLETED:
            self.node.publish_status("Task completed successfully")
            self.transition_to(TaskState.IDLE)

    def handle_movement_timeout(self):
        current_state = self.state
        self.node.get_logger().error(f"Movement timeout in state: {current_state}")
        self.node.publish_status(f"ALERT: Movement timeout in state: {current_state}")
        # Attempt recovery
        self.movement_start_time = None
        if self.retry_count < self.retry_attempts:
            self.retry_count += 1
            self.node.get_logger().warn(f"Retrying operation (attempt {self.retry_count}/{self.retry_attempts})")
            self.node.publish_status(f"Retrying operation (attempt {self.retry_count}/{self.retry_attempts})")
            # Stop current movement
            self.node.send_robot_command("stop")
            # Wait a moment before retrying
            self.node.create_one_shot_timer(2.0, lambda: self.retry_movement(current_state))
        else:  # Exceeded retry attempts
            self.handle_recovery()

    def retry_movement(self, previous_state):
        # Transition back to the state that had the timeout
        self.transition_to(previous_state)

    def handle_movement_complete(self):
        self.movement_start_time = None
        self.retry_count = 0

        if self.state == TaskState.MOVING_HOME:
            if self.search_index == 0:
                # Initial home position reached, start searching
                self.transition_to(TaskState.MOVING_TO_OVERVIEW)
            else:
                # Returning home after task
                self.transition_to(TaskState.COMPLETED)

        elif self.state == TaskState.MOVING_TO_OVERVIEW:
            # Overview position reached, start searching
            self.transition_to(TaskState.SEARCHING)

        elif self.state == TaskState.SEARCHING:
            # Let the cube detection run for a moment before processing
            self.node.create_one_shot_timer(1.0, self.process_detections)

        elif self.state == TaskState.POINTING:
            # Move to next color
            self.current_color_index += 1
            self.movement_start_time = None

        elif self.state == TaskState.RETURNING_HOME:
            self.transition_to(TaskState.COMPLETED)

    def handle_robot_error(self, error_msg):
        self.node.get_logger().error(f"Robot error: {error_msg}")
        self.node.publish_status(f"ALERT: Robot error: {error_msg}")

        if self.retry_count < self.retry_attempts:
            self.retry_count += 1
            self.node.get_logger().warn(f"Retrying after error (attempt {self.retry_count}/{self.retry_attempts})")
            # Wait a moment before retrying the current state
            self.node.create_one_shot_timer(2.0, lambda: self.retry_movement(self.state))
        else:
            # Exceeded retry attempts
            self.handle_recovery()

    def handle_recovery(self):
        if self.recovery_behavior == 'home':
            self.node.get_logger().warn("Recovery: returning to home position")
            self.node.publish_status("Recovery: returning to home position")
            self.retry_count = 0
            self.transition_to(TaskState.RETURNING_HOME)
        elif self.recovery_behavior == 'continue':
            self.node.get_logger().warn("Recovery: continuing with next operation")
            self.node.publish_status("Recovery: continuing with next operation")
            self.retry_count = 0
            # Skip to next operation based on current state
            if self.state == TaskState.SEARCHING:
                self.search_index += 1
                self.movement_start_time = None
            elif self.state == TaskState.POINTING:
                self.current_color_index += 1
                self.movement_start_time = None
            else:  # For other states, just go home
                self.transition_to(TaskState.RETURNING_HOME)
        else:  # 'abort'
            self.node.get_logger().error("Recovery: aborting task due to failures")
            self.node.publish_status("ALERT: Task aborted due to failures")
            self.transition_to(TaskState.ERROR)

    def handle_search_completion(self):
        # Check if all required colors were found
        missing_colors = self.cube_detection.missing_colors
        if not missing_colors:
            self.node.get_logger().info("All required cubes detected")
            # All colors found, proceed to pointing
            self.node.publish_status("All required cubes detected")
            self.transition_to(TaskState.POINTING)
        else:
            missing_str = ", ".join(missing_colors)
            # Some colors still missing
            self.node.get_logger().warn(f"Could not find all required cubes. Missing: {missing_str}")
            self.node.publish_status(f"ALERT: Could not find cubes of colors: {missing_str}")

            # Continue with the cubes we have or return home based on config
            if (self.recovery_behavior == 'continue' and
                    any(c in self.cube_detection.detected_cubes for c in self.required_colors)):
                self.node.get_logger().info("Continuing with detected cubes")
                self.node.publish_status("Continuing with detected cubes")
                self.transition_to(TaskState.POINTING)
            else:
                self.transition_to(TaskState.RETURNING_HOME)

    def process_detections(self):
        self.movement_start_time = None
        self.node.publish_status(
            f"Processing detections at search position {self.search_index + 1}")
        # Check if all required colors were found
        if self.cube_detection.all_cubes_found():
            self.node.get_logger().info("All required cubes detected")
            self.node.publish_status("All required cubes detected")# All colors found
            self.transition_to(TaskState.POINTING)
        else:
            self.search_index += 1# Move to next search position
            self.movement_start_time = None
            # Keep searching

    def point_to_cube(self, color):
        if color not in self.cube_detection.detected_cubes:
            self.node.get_logger().error(f"Cannot point to {color} cube - not detected")
            return

        pose = self.cube_detection.detected_cubes[color]
        position = pose.position
        self.node.publish_status(f"Pointing to {color} cube")
        self.node.send_robot_command(f"point_to:{position.x},{position.y},{position.z},{self.pointing_distance}")
        self.movement_start_time = time.time()

    def transition_to(self, new_state):
        old_state = self.state
        self.state = new_state
        self.node.get_logger().debug(f"State transition: {old_state.name} -> {new_state.name}")
        # Reset movement timeout timer on state change
        self.movement_start_time = None
        # Special handling for certain state transitions
        if new_state == TaskState.IDLE:
            self.retry_count = 0
        elif new_state == TaskState.ERROR:
            self.node.publish_status("System in ERROR state. Reset required.")

    def _format_position(self, position):
        return ','.join(map(str, position))