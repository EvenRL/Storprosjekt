import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cube_interfaces.msg import DetectedCube, DetectedCubeArray
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
import time
import random
class FakeHardwareNode(Node):
    def __init__(self):
        super().__init__('fake_hardware')
        # Publishers
        self.robot_status_pub = self.create_publisher(String, 'robot_status', 10)
        self.cubes_pub = self.create_publisher(DetectedCubeArray, 'detected_cubes', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        # Subscribers
        self.command_sub = self.create_subscription(
            String, 'robot_command', self.command_callback, 10)
        # Get required colors from parameters or use default
        self.required_colors = self.declare_parameter('required_colors', ['green', 'black', 'blue']).value
        self.additional_colors = ['red', 'yellow', 'white', 'orange', 'purple']
        # Timer for publishing fake data
        self.timer = self.create_timer(2.0, self.publish_fake_cubes)
        # Initialize random number generator
        random.seed()
        self.get_logger().info('Fake hardware node started')
        self.get_logger().info(f'Required colors: {", ".join(self.required_colors)}')
        
    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        # Simulate delay for movement
        time.sleep(1.0)
        # Send movement complete status
        status_msg = String()
        status_msg.data = "movement_complete"
        self.robot_status_pub.publish(status_msg)
        
    def publish_fake_cubes(self):
        cubes_msg = DetectedCubeArray()
        marker_array = MarkerArray()
        
        # Start with the required colors
        colors_to_use = list(self.required_colors)
        
        # Add some random additional colors
        num_additional = random.randint(0, 2)
        if num_additional > 0:
            additional = random.sample(self.additional_colors, k=min(num_additional, len(self.additional_colors)))
            colors_to_use.extend(additional)
        # Create cubes for each color
        marker_id = 0
        for color in colors_to_use:
            # Create cube message
            cube = DetectedCube()
            cube.color = color
            # Random position in a reasonable workspace
            cube.pose = Pose()
            cube.pose.position.x = random.uniform(0.2, 0.6)
            cube.pose.position.y = random.uniform(-0.3, 0.3)
            cube.pose.position.z = random.uniform(0.1, 0.3)
            # Simple orientation - just identity
            cube.pose.orientation.w = 1.0
            cubes_msg.cubes.append(cube)
            # Create visualization marker
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cubes"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            # Set position and orientation
            marker.pose = cube.pose
            # Set scale (size of the cube)
            marker.scale.x = 0.05  # 5cm cube
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            # Set color based on cube color
            marker.color.a = 1.0  # Alpha (transparency)
            if color == 'green':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif color == 'black':
                marker.color.r = 0.1
                marker.color.g = 0.1
                marker.color.b = 0.1
            elif color == 'blue':
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif color == 'red':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif color == 'yellow':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif color == 'white':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            elif color == 'orange':
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            elif color == 'purple':
                marker.color.r = 0.5
                marker.color.g = 0.0
                marker.color.b = 0.5
            else:
                marker.color.r = 0.5 # Default color
                marker.color.g = 0.5
                marker.color.b = 0.5
            
            marker_array.markers.append(marker)
        # Publish both the cube messages and visualization markers
        self.cubes_pub.publish(cubes_msg)
        self.marker_pub.publish(marker_array)
        # Log which colors were published
        color_list = [cube.color for cube in cubes_msg.cubes]
        self.get_logger().info(f'Published {len(cubes_msg.cubes)} cubes: {", ".join(color_list)}')

def main(args=None):
    rclpy.init(args=args)
    node = FakeHardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
