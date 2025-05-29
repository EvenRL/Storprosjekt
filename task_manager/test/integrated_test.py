import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Header, Bool
from cube_interfaces.msg import DetectedCube, DetectedCubeArray
import threading
import time
class IntegratedTester(Node):
    def __init__(self):
        super().__init__('integrated_tester')
        self.cube_pub = self.create_publisher(# Publisher for cube detections
            DetectedCubeArray, 
            '/detected_cubes', 
            10
        )
        self.status_pub = self.create_publisher( # Publisher for robot status
            String,
            '/robot_status',
            10
        )
        self.cmd_sub = self.create_subscription( # Subscriber for robot commands
            String,
            '/robot_command',
            self.cmd_callback,
            10
        )
        self.task_status_sub = self.create_subscription(# Subscriber for task status
            String,
            '/task_status',
            self.task_status_callback,
            10
        )
        self.timer = self.create_timer(0.5, self.publish_cubes) # Start cube publishing
        self.get_logger().info('Integrated tester started')
        
        # Variables to track state
        self.last_command = None
        self.last_status = None
        self.movement_complete_timer = None
    
    def publish_cubes(self):
        msg = DetectedCubeArray()
        msg.header = Header(frame_id='base_link')
        msg.header.stamp = self.get_clock().now().to_msg()
        # Green cube
        green_cube = DetectedCube()
        green_cube.color = 'green'
        green_cube.pose.position.x = 0.3
        green_cube.pose.position.y = 0.1
        green_cube.pose.position.z = 0.05
        green_cube.pose.orientation.w = 1.0
        msg.cubes.append(green_cube)
        # Black cube
        black_cube = DetectedCube()
        black_cube.color = 'black'
        black_cube.pose.position.x = 0.25
        black_cube.pose.position.y = -0.15
        black_cube.pose.position.z = 0.05
        black_cube.pose.orientation.w = 1.0
        msg.cubes.append(black_cube)
        # Blue cube
        blue_cube = DetectedCube()
        blue_cube.color = 'blue'
        blue_cube.pose.position.x = 0.35
        blue_cube.pose.position.y = 0.0
        blue_cube.pose.position.z = 0.05
        blue_cube.pose.orientation.w = 1.0
        msg.cubes.append(blue_cube)
        self.cube_pub.publish(msg)
        self.get_logger().debug('Published test cubes')
    def cmd_callback(self, msg):
        self.last_command = msg.data
        self.get_logger().info(f'Received command: {msg.data}')
        if self.movement_complete_timer:# Cancel any pending timer
            self.movement_complete_timer.cancel()
        # Schedule a movement completion after a delay
        self.movement_complete_timer = self.create_timer(
            2.0,  # 2 second delay
            lambda: self.send_movement_complete(),
            oneshot=True
        )
    def send_movement_complete(self):
        msg = String()
        msg.data = 'movement_complete'
        self.status_pub.publish(msg)
        self.get_logger().info('Sent movement_complete')

    def task_status_callback(self, msg):
        self.last_status = msg.data
        self.get_logger().info(f'Task status: {msg.data}')

def main():
    rclpy.init()
    tester = IntegratedTester()
    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(tester)
    # Run the test
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    print("Integrated test running. Ctrl+C to exit.")
    
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
