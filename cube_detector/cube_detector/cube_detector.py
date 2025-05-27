import rclpy
from rclpy.node import Node, NodeOptions
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import tf_transformations
from cube_interfaces.msg import DetectedCubeArray, DetectedCube, ColorRangeArray
import cv2 as cv
import numpy as np

from detector_core import findCubePoses

class CubeDetectorNode(Node):
    '''
    Node for detecting and estimating pose of colored cubes.
    Publishes poses of cubes in array of type DetectedCubeArray in /detected_cubes topic.
    '''

    # Colors to search for in hsv format
    colors = {'blue': [np.array([90, 50, 70]), np.array([128, 255, 255])],
        'red': [np.array([159, 50, 70]), np.array([180, 255, 255])],
        'yellow': [np.array([25, 50, 70]), np.array([35, 255, 255])],
        'green': [np.array([36, 50, 70]), np.array([89, 255, 255])],
        'black': [np.array([0,0,0]), np.array([180,255,30])]}
    
    opts = NodeOptions(automatically_declare_parameters_from_overrides=True)

    def __init__(self):
        super().__init__('cube_detector', options=opts)

        self.calibration_loaded = False
        self.frame_id = 'camera_frame'
        
        # Subscribe to camera info topic to get camera parameters
        self.cam_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10
        )

        self.timer = self.create_timer(10.0, self.check_camera_info)

        # Subscribe to color topic
        self.color_subsrber = self.create_subscription(
            ColorRangeArray,
            'colors',
            self.color_callback,
            10)

        # Subscribe to image topic
        self.img_subscriber = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)

        # Publish image with cube poses drawn
        self.img_publisher = self.create_publisher(
            Image,
            'image_output',
            10)
        
        # Publish cube poses
        self.cube_publisher = self.create_publisher(
            DetectedCubeArray,
            'detected_cubes',
            10)
  
        # Initialize CVBridge
        self.bridge = CvBridge()

    def camera_info_callback(self, msg):
        self.get_logger().info(f'Loading camera parameters...')

        # Load camera parameters
        self.K = np.array(msg.k).reshape(3, 3) # Camera matrix
        self.D = np.array(msg.d) # Distortion coefficients

        # Unsubscribe after loading parameters
        self.calibration_loaded = True

        # stopper timer om den blir aktivert
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
            self.timer = None

        self.destroy_subscription(self.cam_info_subscriber)
        self.get_logger().info('Camera parameters loaded, unsubscribing from /camera_info.')

    def color_callback(self, msg):
        #Update color ranges from topic
        updated_colors = {}
        for color in msg.colors:
            lower = np.array(color.lower, dtype=np.uint8)
            upper = np.array(color.upper, dtype=np.uint8)
            updated_colors[color.name] = [lower,upper]
        self.colors = updated_colors
        self.get_logger().info('Updated the color set!')

    def image_callback(self, msg):
        # Abort if camera parameters not loaded
        if not self.calibration_loaded:
            self.get_logger().error('Image callback called before camera parameters loaded!')
            return
        
        # Attempt to retrieve image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8') # Convert image from ros2 format to opencv format
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        cubes, overlay = findCubePoses(
        cv_image,
        self.colors,
        self.K, self.D,
        {
          'cubeSize': self.get_parameter('cubeSize').value,
          'minArea': self.get_parameter('minArea').value,
          'maxArea': self.get_parameter('maxArea').value,
          'alpha': self.get_parameter('alpha').value,
          'morphOpen': self.get_parameter('morphOpen').value,
          'morphClose': self.get_parameter('morphClose').value,
          'reprojMax': self.get_parameter('reprojMax').value,
          'enableBlur'  : self.get_parameter('enableBlur').value,
          'blurKernelSize': self.get_parameter('blurKernelSize').value,
          'displayColorMask': self.get_parameter('displayColorMask').value,
          'preprocessing': self.get_parameter('preprocessing').value,
          'claheClip': self.get_parameter('claheClip').value,
          'claheTile': self.get_parameter('claheTile').value,
          'maskCleanup': self.get_parameter('maskCleanup').value,
          'drawContours': self.get_parameter('drawContours').value,
          'drawPoints': self.get_parameter('drawPoints').value,
          'drawPointOrder': self.get_parameter('drawPointOrder').value,
          'drawPose': self.get_parameter('drawPose').value,
          'minSolidity': self.get_parameter('minSolidity').value,
          'minExtent': self.get_parameter('minExtent').value,
          'minAspect': self.get_parameter('minAspect').value,
          'maxAspect': self.get_parameter('maxAspect').value
          })

        # Publish new image with cube poses drawn
        try:
            cube_img_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
        
        cube_msg = DetectedCubeArray()
        cube_msg.header.stamp = self.get_clock().now().to_msg()
        cube_msg.header.frame_id = self.frame_id

        for cube in cubes:
            tvec = cube['tvec']
            # Convert rot vector to quaternion
            rot_mat,_ = cv.Rodrigues(np.array(cube["rvec"])) # Rodrigues vector to rotation matrix
            transform = np.eye(4) 
            transform[:3, :3] = rot_mat
            transform[:3,  3] = tvec.reshape(3)
            quat = tf_transformations.quaternion_from_matrix(transform)

            dc = DetectedCube()
            dc.color = cube['color']
            dc.pose.position.x = float(tvec[0])
            dc.pose.position.y = float(tvec[1])
            dc.pose.position.z = float(tvec[2])
            dc.pose.orientation.x = float(quat[0])
            dc.pose.orientation.y = float(quat[1])
            dc.pose.orientation.z = float(quat[2])
            dc.pose.orientation.w = float(quat[3])

            cube_msg.cubes.append(dc)

        self.cube_publisher.publish(cube_msg)
        self.img_publisher.publish(cube_img_msg)
    
    def check_camera_info(self):
        if not self.calibration_loaded:
            self.get_logger().error(
                'Camera info is not received for 10 sec; check camera connection.'
            )
        elif hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
            self.timer = None
    
# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    cube_detector_node = CubeDetectorNode()
    rclpy.spin(cube_detector_node)
    cube_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()