import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.timer import Timer
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from cube_interfaces.msg import DetectedCubeArray
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
# Import from our core module
from .core import (TaskState, ParameterManager, create_cube_markers, create_pointing_marker,
                   RobotMovementManager, CubeDetectionHandler, TaskStateMachine, ServiceManager)

try:
    from moveit import MoveItPy

    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    print("no moveit !")

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.get_logger().info('Initializing Task Manager Node')
        self.service_group = ReentrantCallbackGroup() # Set up callback groups
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.action_group = ReentrantCallbackGroup()
        # Initialize parameters
        self.param_manager = ParameterManager(self)
        self.config = self.param_manager.config
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Active timers management
        self._active_timers = []
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        self.robot_cmd_pub = self.create_publisher(String, '/robot_command', 10)
        #  Initialize components
        self.cube_detection = CubeDetectionHandler(self, self.config['required_colors'])
        self.robot_movement = RobotMovementManager(
            self,
            self.config['traj_action_name'],
            self.config['joint_names'],
            self.config['sim_mode']
        )
        #  State machine
        self.state_machine = TaskStateMachine(self, self.robot_movement, self.cube_detection, self.config)
        #  Services
        self.service_manager = ServiceManager(self, self.state_machine)
        #  Subscribers
        self.setup_subscribers()
        #  MoveIt setup (if available)
        self._init_moveit()
        #  Main state machine timer
        # Run at 10Hz for more responsive state transitions
        self.state_machine_timer = self.create_timer(0.1, self.state_machine.update, callback_group=self.timer_group)
        #  Auto-start
        if self.config['enable_auto_start']:
            self.get_logger().info('Auto-starting detection and pointing task')
            self.start_task()
        # Log initialization complete
        self.get_logger().info('Task Manager Node initialized with:')
        self.get_logger().info(f'- Required colors: {", ".join(self.config["required_colors"])}')
        self.get_logger().info(f'- {len(self.config["search_positions"])} search positions configured')
        self.get_logger().info(f'- Running in {"simulation" if self.config["sim_mode"] else "real robot"} mode')
        self.publish_status("System initialized and ready")

    def setup_subscribers(self):
        self.create_subscription(DetectedCubeArray,'/detected_cubes',self.cube_callback,10)
        self.create_subscription(Bool,'/emergency_stop', self.estop_cb,10)
        self.create_subscription(String,'/robot_status',self.robot_status_callback,10)

    def _init_moveit(self):
        self.moveit = None
        self.arm = None
        if not self.config['sim_mode'] and MOVEIT_AVAILABLE:
            try:
                pkg_tm = get_package_share_directory('task_manager')
                moveit_config_file = os.path.join(pkg_tm, 'config', 'move_group_params.yaml')
                self.moveit = MoveItPy(node_name=self.get_name() + '_moveit',name_space='',
                    launch_params_filepaths=[moveit_config_file],provide_planning_service=True)
                self.arm = self.moveit.get_planning_component('manipulator')
                self.get_logger().info('MoveIt initialized successfully')
            except Exception as e:
                self.get_logger().error(f'MoveIt initialization failed: {str(e)}')

    def create_one_shot_timer(self, timeout_sec: float, callback) -> Timer:
        timer = self.create_timer(
            timeout_sec,
            lambda: self._one_shot_callback(timer, callback),
            callback_group=self.timer_group
        )
        self._active_timers.append(timer)
        return timer

    def _one_shot_callback(self, timer: Timer, callback) -> None:
        timer.cancel()
        if timer in self._active_timers:
            self._active_timers.remove(timer)
        callback()

    def cube_callback(self, msg: DetectedCubeArray) -> None:
        # Process the detection
        self.cube_detection.process_detection(msg)
        # Update visualization
        if self.config['enable_visualization']:
            self.visualize_cubes()
        # Process detections during search if needed
        if (self.state_machine.state == TaskState.SEARCHING and
                self.cube_detection.all_cubes_found() and
                self.state_machine.movement_start_time is None):
            self.state_machine.process_detections()

    def robot_status_callback(self, msg: String) -> None:
        status = msg.data
        if status == "movement_complete":
            self.state_machine.handle_movement_complete()
        elif status.startswith("error:"):
            error_msg = status[6:]  # Remove
            self.state_machine.handle_robot_error(error_msg)

    def estop_cb(self, msg: Bool) -> None:
        if msg.data and not hasattr(self, 'estopped'):
            self.get_logger().fatal('Emergency stop triggered!')
            self.estopped = True
            self.publish_status("EMERGENCY STOP ACTIVATED")
            self.send_robot_command("stop")  # Stop all movements
            self.state_machine.transition_to(TaskState.ERROR)  # Set state to error
        elif not msg.data and hasattr(self, 'estopped'):
            self.get_logger().info('Emergency stop cleared')
            delattr(self, 'estopped')
            self.publish_status("Emergency stop cleared. System ready.")

    def visualize_cubes(self) -> None:
        if not self.config['enable_visualization']:
            return

        marker = MarkerArray()
        now = self.get_clock().now().to_msg()
        cube_markers = create_cube_markers(self.cube_detection.detected_cubes,# Create cube markers
            self.config['output_frame'],now)
        # Add all cube markers to the marker array
        for marker in cube_markers:
            marker.markers.append(marker)
        # Add pointing line if in pointing state
        if (self.state_machine.state == TaskState.POINTING and
                self.state_machine.current_color_index < len(self.config['required_colors'])):
            color = self.config['required_colors'][self.state_machine.current_color_index]
            pointing_marker = create_pointing_marker(
                color,
                self.cube_detection.detected_cubes,
                self.config['output_frame'],
                now)
            if pointing_marker:
                marker.markers.append(pointing_marker)
        self.marker_pub.publish(marker)

    def start_task(self) -> bool:
        if hasattr(self, 'estopped'):
            self.get_logger().error("emergency stop is active")
            return False

        self.state_machine.transition_to(TaskState.INITIALIZING)
        return True

    def send_robot_command(self, cmd: str) -> None:
        if self.config['sim_mode']:
            self.get_logger().info(f"[SIM] Robot command: {cmd}")
            # In simulation, simulate movement completion after a delay
            self.create_one_shot_timer(2.0, self.state_machine.handle_movement_complete)
        else:
            # Send actual command
            msg = String()
            msg.data = cmd
            self.robot_cmd_pub.publish(msg)
            self.get_logger().info(f"Sent robot command: {cmd}")

    def publish_status(self, message: str) -> None:
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(message)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TaskManagerNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unhandled exception: {e}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()