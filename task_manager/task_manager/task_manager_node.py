import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from cube_interfaces.msg import ColorCubeArray, ColorCube
import time

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        
        # Declare configurable parameters
        self.declare_parameter('required_colors', ['green', 'black', 'blue'])
        self.declare_parameter('home_position', [0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
        self.declare_parameter('overview_position', [0.0, -1.0, 0.5, -1.0, 0.0, 0.0])
        
        # For search positions, we need to handle them differently since ROS2 doesn't support nested lists
        # Declare individual search positions
        self.declare_parameter('search_position_1', [0.3, -0.8, 0.5, -1.0, 0.0, 0.0])
        self.declare_parameter('search_position_2', [-0.3, -0.8, 0.5, -1.0, 0.0, 0.0])
        self.declare_parameter('num_search_positions', 2)
        
        self.declare_parameter('movement_timeout', 30.0)  # Timeout in seconds
        self.declare_parameter('pointing_distance', 0.15)
        
        # Get parameters
        self.home_position = self.get_parameter('home_position').value
        self.overview_position = self.get_parameter('overview_position').value
        self.required_colors = self.get_parameter('required_colors').value
        self.movement_timeout = self.get_parameter('movement_timeout').value
        self.pointing_distance = self.get_parameter('pointing_distance').value
        
        # Reconstruct search positions list
        num_positions = self.get_parameter('num_search_positions').value
        self.search_positions = []
        for i in range(1, num_positions + 1):
            position = self.get_parameter(f'search_position_{i}').value
            if position:
                self.search_positions.append(position)
        
        # Task variables
        self.state = 'idle'
        self.detected_cubes = {}
        self.missing_colors = []
        self.current_color = None
        self.search_index = 0
        self.movement_complete = False
        self.movement_start_time = None
        self._active_timers = []  # Changed from 'timers' to '_active_timers' to avoid conflict
        
        # Publishers and subscribers
        self.status_pub = self.create_publisher(String, 'task_status', 10)
        self.robot_cmd_pub = self.create_publisher(String, 'robot_command', 10)
        
        # Use ColorCubeArray message type from cube_interfaces
        self.cube_sub = self.create_subscription(
            ColorCubeArray, 'detected_cubes', self.cube_callback, 10)
            
        self.robot_status_sub = self.create_subscription(
            String, 'robot_status', self.robot_status_callback, 10)
        
        # Add a service to start/stop the task - using Trigger service now (not String)
        self.task_service = self.create_service(
            Trigger, 'task_control', self.task_control_callback)
        
        # Main state machine timer
        self.task_timer = self.create_timer(1.0, self.task_loop)
        self.get_logger().info('Task Manager node initialized')
        self.get_logger().info(f'Using {len(self.search_positions)} search positions')
        self.get_logger().info(f'Required colors: {", ".join(self.required_colors)}')
        
        # Publish topic information to help debugging
        self.get_logger().info(f'Subscribed to: detected_cubes')
        self.get_logger().info(f'Publishing to: task_status, robot_command')
    
    def create_one_shot_timer(self, timeout_sec, callback):
        """Create a timer that will fire only once after the specified timeout"""
        timer = self.create_timer(timeout_sec, lambda: self._one_shot_callback(timer, callback))
        self._active_timers.append(timer)  # Changed from 'timers' to '_active_timers'
        return timer
        
    def _one_shot_callback(self, timer, callback):
        """Execute the callback and cancel the timer"""
        # Cancel timer first to prevent any chance of it firing again
        timer.cancel()
        if timer in self._active_timers:  # Changed from 'timers' to '_active_timers'
            self._active_timers.remove(timer)
        # Call the original callback
        callback()
    
    def task_control_callback(self, request, response):
        """Service callback to control tasks"""
        # Start the task (we could expand this with command in the request message)
        self.start_task()
        response.success = True
        response.message = 'Task started'
        return response

    def robot_status_callback(self, msg):
        """Handle robot status updates."""
        if msg.data == "movement_complete":
            self.movement_complete = True
            self.handle_movement_complete()
        elif msg.data.startswith("error:"):
            self.handle_robot_error(msg.data[6:])  # Extract error message
    
    def handle_robot_error(self, error_msg):
        """Handle robot error conditions"""
        self.get_logger().error(f"Robot error: {error_msg}")
        self.publish_status(f"ALERT: Robot error: {error_msg}")
        # Attempt recovery by returning to home position
        self.state = 'returning_home'
        self.send_robot_command(f"move_joints:{self._format_position(self.home_position)}")
    
    def handle_movement_complete(self):
        """Handle state transitions when robot movement completes."""
        self.movement_start_time = None  # Reset timeout timer
        
        if self.state == 'moving_home':
            self.state = 'moving_to_overview'
            self.send_robot_command(f"move_joints:{self._format_position(self.overview_position)}")
        elif self.state == 'moving_to_overview':
            self.state = 'capturing_image'
            # Transition to detecting after 1 second (simulating camera capture)
            self.create_one_shot_timer(1.0, lambda: setattr(self, 'state', 'detecting_cubes'))
        elif self.state == 'pointing_sequence':
            # Process the color sequence - updated for green, black, blue colors
            if self.current_color == self.required_colors[0]:  # green
                self.current_color = self.required_colors[1]  # black
                self.point_to_cube(self.required_colors[1])
            elif self.current_color == self.required_colors[1]:  # black
                self.current_color = self.required_colors[2]  # blue
                self.point_to_cube(self.required_colors[2])
            elif self.current_color == self.required_colors[2]:  # blue
                self.current_color = 'done'
                self.state = 'returning_home'
                self.send_robot_command(f"move_joints:{self._format_position(self.home_position)}")
        elif self.state == 'searching':
            self.state = 'processing_search'
        elif self.state == 'returning_home':
            self.state = 'task_complete'
    
    def task_loop(self):
        """Main state machine for task execution."""
        # Check for movement timeout
        if self.movement_start_time and time.time() - self.movement_start_time > self.movement_timeout:
            self.handle_movement_timeout()
            return
        
        # State machine logic
        if self.state == 'idle':
            self.publish_status("System ready. Use 'task_control' service to start task.")
        elif self.state == 'moving_home':
            if not self.movement_start_time:  # Only send command once
                self.publish_status("Moving to home position")
                self.send_robot_command(f"move_joints:{self._format_position(self.home_position)}")
                self.movement_start_time = time.time()
        elif self.state == 'moving_to_overview':
            if not self.movement_start_time:  # Only send command once
                self.publish_status("Moving to overview position")
                self.movement_start_time = time.time()
        elif self.state == 'capturing_image':
            self.publish_status("Capturing overview image")
        elif self.state == 'detecting_cubes':
            self.publish_status("Processing image to detect cubes")
            self._check_detected_cubes()
        elif self.state == 'search_init':
            self._handle_search_init()
        elif self.state == 'searching':
            if not self.movement_start_time:  # Only send command once
                self.publish_status(f"Capturing image from search position {self.search_index+1}")
                self.movement_start_time = time.time()
        elif self.state == 'processing_search':
            self._check_detected_cubes()
        elif self.state == 'returning_home':
            if not self.movement_start_time:  # Only send command once
                self.publish_status("Returning to home position")
                self.movement_start_time = time.time()
        elif self.state == 'task_complete':
            self.publish_status("Task completed! System ready for next task.")
            self.state = 'idle'
            self.detected_cubes = {}
    
    def handle_movement_timeout(self):
        """Handle timeout during robot movement"""
        self.get_logger().error(f"Movement timeout in state: {self.state}")
        self.publish_status(f"ALERT: Movement timeout in state: {self.state}")
        self.movement_start_time = None
        
        # Attempt recovery by returning to home position
        self.state = 'returning_home'
        self.send_robot_command("stop")  # Stop current movement
        
        # Use our custom one-shot timer
        self.create_one_shot_timer(2.0, 
            lambda: self.send_robot_command(f"move_joints:{self._format_position(self.home_position)}"))
    
    def _check_detected_cubes(self):
        """Check which cubes were detected and decide next action."""
        self.missing_colors = [c for c in self.required_colors if c not in self.detected_cubes]
        
        if not self.missing_colors:
            # All cubes found
            self.state = 'pointing_sequence'
            self.current_color = self.required_colors[0]
            self.point_to_cube(self.required_colors[0])
            self.publish_status("All cubes detected! Beginning pointing sequence")
        else:
            # Missing cubes, start/continue search
            self.state = 'search_init'
            if self.state == 'detecting_cubes':  # First detection
                self.search_index = 0
                self.publish_status(f"Missing cubes: {', '.join(self.missing_colors)}. Starting search")
            else:  # After a search position
                self.search_index += 1
    
    def _handle_search_init(self):
        """Initialize or continue the search for missing cubes."""
        if self.search_index >= len(self.search_positions):
            self.publish_status(f"ALERT: Could not find all cubes. Missing: {', '.join(self.missing_colors)}")
            self.state = 'returning_home'
            self.send_robot_command(f"move_joints:{self._format_position(self.home_position)}")
            self.movement_start_time = time.time()
        else:
            search_pos = self.search_positions[self.search_index]
            self.publish_status(f"Moving to search position {self.search_index+1}")
            self.send_robot_command(f"move_joints:{self._format_position(search_pos)}")
            self.state = 'searching'
            self.movement_start_time = time.time()
    
    def point_to_cube(self, color):
        """Command robot to point at the specified cube"""
        if color not in self.detected_cubes:
            self.get_logger().error(f"Cannot point to {color} cube - not detected")
            return
            
        position = self.detected_cubes[color]['position']
        self.publish_status(f"Pointing to {color} cube at {position}")
        self.send_robot_command(f"point_to:{position[0]},{position[1]},{position[2]},{self.pointing_distance}")
        self.movement_start_time = time.time()
    
    def start_task(self):
        """Start the cube detection and pointing task."""
        self.state = 'moving_home'
        self.publish_status("Starting task")
        self.movement_complete = False
        self.movement_start_time = None
        self.detected_cubes = {}
        self.missing_colors = []
    
    def send_robot_command(self, cmd):
        """Send command to the robot."""
        self.robot_cmd_pub.publish(self.create_string_msg(cmd))
    
    def publish_status(self, message):
        """Publish status message."""
        msg = self.create_string_msg(message)
        self.status_pub.publish(msg)
        self.get_logger().info(message)
    
    def create_string_msg(self, text):
        """Create a String message."""
        msg = String()
        msg.data = text
        return msg
    
    def _format_position(self, position):
        """Format position array as comma-separated string."""
        return ','.join(map(str, position))
    
    def cube_callback(self, msg):
        """Process detected cubes from ColorCubeArray message."""
        if self.state not in ['detecting_cubes', 'processing_search']:
            return
            
        self.detected_cubes = {}
        detected_colors = []
        
        # Process each cube in the array
        for cube in msg.cubes:
            if cube.detected:
                detected_colors.append(cube.color)
                self.detected_cubes[cube.color] = {
                    'pose': cube.pose,
                    'position': (cube.pose.position.x, cube.pose.position.y, cube.pose.position.z)
                }
                self.get_logger().info(f"Detected {cube.color} cube at position: "
                                      f"({cube.pose.position.x:.2f}, {cube.pose.position.y:.2f}, {cube.pose.position.z:.2f})")
            else:
                self.get_logger().info(f"Cube {cube.color} was NOT detected")
        
        # Find missing cubes by comparing with required colors
        missing_colors = [c for c in self.required_colors if c not in detected_colors]
        
        if missing_colors:
            for color in missing_colors:
                if color not in detected_colors:
                    self.get_logger().info(f"Required cube {color} was NOT detected")
        
        self.get_logger().info(f"Detected {len(self.detected_cubes)} cubes: {', '.join(self.detected_cubes.keys())}")

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
