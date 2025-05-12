import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import cv2 as cv
import numpy as np

class CubeDetectorNode(Node):
    """
    Node for detecting and estimating pose of colored cubes.
    Publishes poses of cubes in array with information like color and whether cube is detected or not.
    """

    # Colors to search for in hsv format
    colors = {'blue': [np.array([90, 50, 70]), np.array([128, 255, 255])],
        'red': [np.array([159, 50, 70]), np.array([180, 255, 255])],
        'yellow': [np.array([25, 50, 70]), np.array([35, 255, 255])],
        'green': [np.array([36, 50, 70]), np.array([89, 255, 255])],
        'black': [np.array([0,0,0]), np.array([180,255,30])]}

    def __init__(self):
        super().__init__('cube_detector')

        # Subscribe to camera info topic to get camera parameters
        self.cam_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10
        )

        # Subscribe to image topic
        self.img_subscriber = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)

        # Publish image with cube poses drawn
        self.img_publisher = self.create_publisher(
            Image,
            'image_output',
            10)
  
        # Initialize CVBridge
        self.bridge = CvBridge()

    def camera_info_callback(self, msg):
        self.get_logger().info(f"Loading camera parameters...")

        # Load camera parameters
        self.K = np.array(msg.k).reshape(3, 3) # Camera matrix
        self.D = np.array(msg.d) # Distortion coefficients

        # Unsubscribe after loading parameters
        self.calibration_loaded = True
        self.destroy_subscription(self.cam_info_subscriber)
        self.get_logger().info("Camera parameters loaded, unsubscribing from /camera_info.")

    def image_callback(self, msg):
        # Abort if camera parameters not loaded
        if not self.calibration_loaded:
            self.get_logger().error('Image callback called before camera parameters loaded!')
            return
        
        # Attempt to retrieve image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convert image from ros2 format to opencv format
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        # Find cube poses
        new_image = self.findCubePoses(cv_image)

        # Publish new image with cube poses drawn
        try:
            cube_msg = self.bridge.cv2_to_imgmsg(new_image, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
  
        self.publisher.publish(cube_msg)

    def findCubePoses(self, image):
        """
        Finds poses of cubes in the image

        First each color is iterated and contours of that color within a certain size are found.
        If any contours look like a square the cube pose is estimated with PnP.
        The estimated poses are then drawn on the image.
        """
        square_points = np.array([[0,0,0],[0.05,0,0],[0.05,0.05,0],[0,0.05,0]], dtype=np.float32) # Define Square geometry to compare with contour, 5x5cm Square.
        cube_points = np.array([[0,0,0],[0.05,0,0],[0.05,0.05,0],[0,0.05,0],
                                [0,0,-0.05],[0.05,0,-0.05],[0.05,0.05,-0.05],[0,0.05,-0.05]]) # Define Cube geometry, 5x5 Cube

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) # Create image copy in hsv format
        for name, clr in self.colors.items():
            mask = cv.inRange(hsv, clr[0], clr[1]) # Create mask of identified color
            cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            
            for cnt in cnts:
                area = cv.contourArea(cnt)
                if 2500 < area < 10000:
                    epsilon = 0.05 * cv.arcLength(cnt, True)
                    approx = cv.approxPolyDP(cnt, epsilon, True)

                    if len(approx) == 4: # If rectangle (4 sides)
                        corners = approx.reshape(-1,2)
                        for point in corners:
                            cv.circle(image, tuple(point), radius=5, color=(0,255,0), thickness=-1)
                        image_points = np.array(corners, dtype=np.float32)

                        # Estimate pose
                        success, rvec, tvec = cv.solvePnP(square_points, image_points, self.K, self.D)

                        if success:
                            imgpts, _ = cv.projectPoints(cube_points, rvec, tvec, self.K, self.D)
                            imgpts = np.int32(imgpts).reshape(-1,2)

                            cv.drawContours(image,[imgpts[:4]], -1, (255,0,0), 2)
                            for i in range(4):
                                cv.line(image, tuple(imgpts[i]), tuple(imgpts[i+4]), (0,255,0), 2)

                            cv.drawContours(image, [imgpts[4:]], -1, (0,0,255), 2)

        return image
    
# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    cube_detector_node = CubeDetectorNode()
    rclpy.spin(cube_detector_node)
    cube_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()