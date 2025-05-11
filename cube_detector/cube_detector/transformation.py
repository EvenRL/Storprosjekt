import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from sensor_msgs.msg import CameraInfo
import numpy as np

class TransformationNode(Node):
    def __init__(self):
        super().__init__('transformation_node')
        
        # TF2-tool
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # camera subscription
        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)
        
        # pose subscription
        self.create_subscription(
            PoseStamped,
            '/detected_cube_poses',
            self.pose_callback,
            10)
        
        # publish transformed pose
        self.transformed_publisher = self.create_publisher(
            PoseArray,
            '/transformed_cube_poses',
            10)

    def camera_info_callback(self, msg):
        #NB need to get info
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def pose_callback(self, msg):
        try:
            # Transform pose
            transform = self.tf_buffer.lookup_transform(
                'robot_base_link',      # Goal frame
                msg.header.frame_id,    # source frame
                rclpy.time.Time())
            
            transformed_pose = do_transform_pose(msg, transform)
            
            # publish all
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = 'robot_base_link'
            pose_array.poses.append(transformed_pose.pose)
            
            self.transformed_publisher.publish(pose_array)
            
        except TransformException as e:
            self.get_logger().error(f'Transformasjonsfeil: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TransformationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
