import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose


class CubeTransformNode(Node):
    """
    Listens for cube poses in the camera frame and republishes them in a
    robot–centric frame so that a task manager can use them directly.
    """

    def __init__(self) -> None:
        super().__init__('cube_transform')

        # Parameters that can be overridden from a launch file
        self.declare_parameter('target_frame', 'robot_base_link')
        self.declare_parameter('input_topic',  '/detected_cube_pose')
        self.declare_parameter('output_topic', '/cube_pose_in_base')

        self.target_frame = (self.get_parameter('target_frame').get_parameter_value().string_value)
        in_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # TF2 one buffer and one dedicated listener thread
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # ROS I/O
        # Subscribes raw cube pose from vision node
        self.create_subscription(PoseStamped, in_topic, self.pose_callback, qos_profile=10)

        # Publishes cube pose in robot base frame
        self.pose_pub = self.create_publisher(PoseStamped, out_topic, 10)

        self.get_logger().info(
            f'CubeTransformNode is running.  Input: "{in_topic}"  →  Output: "{out_topic}"\n'
            f'Transforming every pose into frame "{self.target_frame}".'
        )

    # Callback
    def pose_callback(self, msg: PoseStamped) -> None:
        """
        Transforms an incoming pose into `self.target_frame` and republishes it.
        """
        src_frame = msg.header.frame_id
        stamp     = msg.header.stamp  # Uses the camera timestamp

        try:
            # TF2 has up to 50 ms to find the requested transform
            if not self.tf_buffer.can_transform(
                self.target_frame, src_frame, stamp, timeout=Duration(seconds=0.05)
            ):
                self.get_logger().warn(
                    f'Waiting for transform {self.target_frame} ← {src_frame}'
                )
                return

            transform = self.tf_buffer.lookup_transform(
                self.target_frame, src_frame, stamp
            )

            # Apply the transform to the pose
            pose_in_target = do_transform_pose(msg, transform)
            pose_in_target.header.frame_id = self.target_frame

            # Publish for downstream nodes
            self.pose_pub.publish(pose_in_target)

        except TransformException as ex:
            # Log and continue
            self.get_logger().error(f'Transform error: {ex}')

# Initialize the node
def main(args=None) -> None:
    rclpy.init(args=args)
    node = CubeTransformNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
