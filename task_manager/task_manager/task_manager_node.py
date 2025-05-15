import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        
        # Robot positions and parameters
        self.home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self.overview_position = [0.0, -1.0, 0.5, -1.0, 0.0, 0.0]
        self.search_positions = [
            [0.3, -0.8, 0.5, -1.0, 0.0, 0.0],
            [-0.3, -0.8, 0.5, -1.0, 0.0, 0.0]
        ]
        
        # Task variables
        self.required_colors = ['green', 'blue', 'black']
        self.state = 'idle'
        self.detected_cubes = {}
        self.missing_colors = []
        self.current_color = None
        self.search_index = 0
        self.movement_complete = False
        
        # Publishers and subscribers
        self.status_pub = self.create_publisher(String, 'task_status', 10)
        self.robot_cmd_pub = self.create_publisher(String, 'robot_command', 10)
        self.cube_sub = self.create_subscription(
            PoseArray, 'detected_cubes', self.cube_detection_callback, 10)
        self.robot_status_sub = self.create_subscription(
            String, 'robot_status', self.robot_status_callback, 10)
        
        # Main state machine timer
        self.create_timer(1.0, self.task_loop)
        self.get_logger().info('Task Manager node initialized')
    
    def cube_detection_callback(self, msg):
        """Process detected cubes from PoseArray."""
        if self.state not in ['detecting_cubes', 'processing_search']:
            return
            
        self.detected_cubes = {}
        colors = []
        if msg.header.frame_id.startswith("color:"):
            colors = msg.header.frame_id[6:].split(",")
                
        # Match the colors with poses
        for i, pose in enumerate(msg.poses):
            if i < len(colors):
                color = colors[i]
                self.detected_cubes[color] = {
                    'pose': pose,
                    'position': (pose.position.x, pose.position.y, pose.position.z)
                }
        
        self.get_logger().info(f"Detected {len(self.detected_cubes)} cubes: {', '.join(self.detected_cubes.keys())}")

    def robot_status_callback(self, msg):
        """Handle robot status updates."""
        if msg.data == "movement_complete":
            self.movement_complete = True
            self.handle_movement_complete()
    
    def handle_movement_complete(self):
        """Handle state transitions when robot movement completes."""
        if self.state == 'moving_home':
            self.state = 'moving_to_overview'
            self.send_robot_command(f"move_joints:{self._format_position(self.overview_position)}")
        elif self.state == 'moving_to_overview':
            self.state = 'capturing_image'
            # Transition to detecting after 1 second (simulating camera capture)
            self.create_timer(1.0, lambda: setattr(self, 'state', 'detecting_cubes'), one_shot=True)
        elif self.state == 'pointing_sequence':
            if self.current_color == 'green':
                self.current_color = 'blue'
                self.point_to_cube('blue')
            elif self.current_color == 'blue':
                self.current_color = 'black'
                self.point_to_cube('black')
            elif self.current_color == 'black':
                self.current_color = 'done'
                self.state = 'returning_home'
                self.send_robot_command(f"move_joints:{self._format_position(self.home_position)}")
        elif self.state == 'searching':
            self.state = 'processing_search'
        elif self.state == 'returning_home':
            self.state = 'task_complete'
    
    def task_loop(self):
        """Main state machine for task execution."""
        self.movement_complete = False
        
        # State machine logic
        if self.state == 'idle':
            self.publish_status("System ready. Press 'Start' to begin task.")
        elif self.state == 'moving_home':
            self.publish_status("Moving to home position")
            self.send_robot_command(f"move_joints:{self._format_position(self.home_position)}")
        elif self.state == 'moving_to_overview':
            self.publish_status("Moving to overview position")
        elif self.state == 'capturing_image':
            self.publish_status("Capturing overview image")
        elif self.state == 'detecting_cubes':
            self.publish_status("Processing image to detect cubes")
            self._check_detected_cubes()
        elif self.state == 'search_init':
            self._handle_search_init()
        elif self.state == 'searching':
            self.publish_status(f"Capturing image from search position {self.search_index+1}")
        elif self.state == 'processing_search':
            self._check_detected_cubes()
        elif self.state == 'returning_home':
            self.publish_status("Returning to home position")
        elif self.state == 'task_complete':
            self.publish_status("Task completed! System ready for next task.")
            self.state = 'idle'
            self.detected_cubes = {}
    
    def _check_detected_cubes(self):
        """Check which cubes were detected and decide next action."""
        self.missing_colors = [c for c in self.required_colors if c not in self.detected_cubes]
        
        if not self.missing_colors:
            # All cubes found
            self.state = 'pointing_sequence'
            self.current_color = 'green'
            self.point_to_cube('green')
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
        else:
            search_pos = self.search_positions[self.search_index]
            self.publish_status(f"Moving to search position {self.search_index+1}")
            self.send_robot_command(f"move_joints:{self._format_position(search_pos)}")
            self.state = 'searching'
    
    def point_to_cube(self, color):
        """Command robot to point at the specified cube."""
        if color not in self.detected_cubes:
            self.get_logger().error(f"Cannot point to {color} cube - not detected")
            return
            
        position = self.detected_cubes[color]['position']
        self.publish_status(f"Pointing to {color} cube at {position}")
        self.send_robot_command(f"point_to:{position[0]},{position[1]},{position[2]}")
    
    def start_task(self):
        """Start the cube detection and pointing task."""
        self.state = 'moving_home'
        self.publish_status("Starting task")
    
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
