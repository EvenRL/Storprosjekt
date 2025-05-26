import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import tf_transformations
from cube_interfaces.msg import DetectedCubeArray, DetectedCube, ColorRangeArray
import cv2 as cv
import numpy as np

class CubeDetectorNode(Node):
    """
    Node for detecting and estimating pose of colored cubes.
    Publishes poses of cubes in array of type DetectedCubeArray in /detected_cubes topic.
    """

    # Colors to search for in hsv format
    colors = {'blue': [np.array([90, 50, 70]), np.array([128, 255, 255])],
        'red': [np.array([159, 50, 70]), np.array([180, 255, 255])],
        'yellow': [np.array([25, 50, 70]), np.array([35, 255, 255])],
        'green': [np.array([36, 50, 70]), np.array([89, 255, 255])],
        'black': [np.array([0,0,0]), np.array([180,255,30])]}

    def __init__(self):
        super().__init__('cube_detector')

        # Parameter declaration
        self.declare_parameter('minArea', 2500) # Minimum contour area
        self.declare_parameter('maxArea', 30000) # Maximum contour area
        self.declare_parameter('alpha', 0.05) # Epsilon factor for contour approximation, % of contour perimeter
        self.declare_parameter('enableBlur', False) # Enables preprocessing image with gaussian blur
        self.declare_parameter('blurKernelSize', 5) # Kernel size for gaussian blur
        self.declare_parameter('debug', False) # Enable debug mode
        self.declare_parameter('debug_draw_contours', False) # Draw select contours in debug mode
        self.declare_parameter('debug_draw_all_contours', True) # Draw all contours in debug mode
        self.declare_parameter('debug_draw_points', True) # Draw select points from poly approximation
        self.declare_parameter('debug_draw_all_points', False) # Draw all points from poly approximation
        self.declare_parameter('debug_draw_pose', False) # Draw estimated pose in debug mode

        self.minArea = self.get_parameter('minArea').value
        self.maxArea = self.get_parameter('maxArea').value
        self.alpha = self.get_parameter('alpha').value
        self.enableBlur = self.get_parameter('enableBlur').value
        self.blurKernelSize = self.get_parameter('blurKernelSize').value
        self.debug = self.get_parameter('debug').value
        self.debug_draw_contours = self.get_parameter('debug_draw_contours').value
        self.debug_draw_all_contours = self.get_parameter('debug_draw_all_contours').value
        self.debug_draw_points = self.get_parameter('debug_draw_points').value
        self.debug_draw_all_points = self.get_parameter('debug_draw_all_points').value
        self.debug_draw_pose = self.get_parameter('debug_draw_pose').value

        ##### Testing ######
        self.declare_parameter('clahe_clip_limit', 2.0)
        self.declare_parameter('clahe_tile_size', 8)

        self.clahe_clip = self.get_parameter('clahe_clip_limit').value
        self.clahe_tile = self.get_parameter('clahe_tile_size').value

        ####################

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
            "colors",
            self.color_callback,
            10)

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
        
        # Publish cube poses
        self.cube_publisher = self.create_publisher(
            DetectedCubeArray,
            'detected_cubes',
            10)
  
        # Initialize CVBridge
        self.bridge = CvBridge()

    ##### TESTING ###########
    def colour_preprocess(self, img_bgr: np.ndarray) -> np.ndarray:
        """Return a normalised BGR image using clahe method."""
        hsv = cv.cvtColor(img_bgr, cv.COLOR_BGR2HSV)
        v    = hsv[:,:,2]

        clahe = cv.createCLAHE(clipLimit=self.clahe_clip, tileGridSize=(self.clahe_tile, self.clahe_tile))
        v_eq = clahe.apply(v)

        hsv[:,:,2] = v_eq
        return cv.cvtColor(hsv, cv.COLOR_HSV2BGR)
    #####################

    def camera_info_callback(self, msg):
        self.get_logger().info(f"Loading camera parameters...")

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
        self.get_logger().info("Camera parameters loaded, unsubscribing from /camera_info.")

    def color_callback(self, msg):
        #Update color ranges from topic
        updated_colors = {}
        for color in msg.colors:
            lower = np.array(color.lower, dtype=np.uint8)
            upper = np.array(color.upper, dtype=np.uint8)
            updated_colors[color.name] = [lower,upper]
        self.colors = updated_colors
        self.get_logger().info("Updated the color set!")

    def image_callback(self, msg):
        # Abort if camera parameters not loaded
        if not self.calibration_loaded:
            self.get_logger().error('Image callback called before camera parameters loaded!')
            return
        
        # Attempt to retrieve image
        try:
            #cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convert image from ros2 format to opencv format
            cv_image_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image     = self.colour_preprocess(cv_image_raw)
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        if self.enableBlur:
            cv_image = cv.blur(cv_image,(self.blurKernelSize,self.blurKernelSize))

        if self.debug:
            # Debug mode
            new_image = self.debugMode(cv_image)
        else:
            # Find cube poses
            new_image = self.findCubePoses(cv_image)

        # Publish new image with cube poses drawn
        try:
            cube_img_msg = self.bridge.cv2_to_imgmsg(new_image, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
  
        self.img_publisher.publish(cube_img_msg)

    def sortCorners(self, corners):
        '''
        Order the points from poly approximation to make the following solvePnP function more stable.
        Order is: bottom left, bottom right, top right, top left.
        '''
        sorted_points = np.zeros((4,2), dtype=int)

        sum = corners.sum(axis=1)
        difference = corners[:,0] - corners[:,1]

        sorted_points[0] = corners[np.argmin(difference)] # Bottom left is smallest difference x - y
        sorted_points[1] = corners[np.argmax(sum)] # Bottom right is biggest sum x + y
        sorted_points[2] = corners[np.argmax(difference)] # Top right is biggest difference x - y
        sorted_points[3] = corners[np.argmin(sum)] # Top left is smallest sum x + y
        
        return sorted_points

    def debugMode(self, image):
        '''
        This function is for debugging parameters and color values.
        It allows the user to inspect every step of the cube detection process in real time.
        The function only considers one color at a time, the first color in the color dictionary.
        What the function displays can be configured with the ros parameters prefixed with debug_.
        This function does not publish pose estimates to /detected_cubes topic.
        '''
        color = list(self.colors.keys())[0]
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.colors[color][0], self.colors[color][1])
        bgr_mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
        
        # Find and draw contours
        okContour = list() # Contours within area limits
        oobContour = list() # Contours outside area limits

        cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for cnt in cnts:
            area = cv.contourArea(cnt)
            if self.minArea < area < self.maxArea:
                okContour.append(cnt)
            else:
                oobContour.append(cnt)
        if okContour and (self.debug_draw_contours or self.debug_draw_all_contours):
            cv.drawContours(bgr_mask, okContour, -1, (0,255,0), 3)
        if self.debug_draw_all_contours and oobContour:
            cv.drawContours(bgr_mask, oobContour, -1, (0,0,255), 3)

        if not self.debug_draw_all_contours:
            cnts = okContour

        # Find and draw poly approximations
        for cnt in cnts:
            epsilon = self.alpha * cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt, epsilon, True)
            if self.debug_draw_all_points:
                corners = self.sortCorners(approx.reshape(-1,2))
                for point in corners:
                    cv.circle(bgr_mask, tuple(point), radius=5, color=(255,0,0), thickness=-1)
                image_points = np.array(corners, dtype=np.float32)
            
            elif self.debug_draw_points:
                if len(approx) == 4: # If rectangle (4 sides)
                        corners = self.sortCorners(approx.reshape(-1,2))
                        i = 0
                        for point in corners:
                            i += 1
                            cv.circle(bgr_mask, tuple(point), radius=5, color=(255,0,0), thickness=-1)
                            cv.putText(bgr_mask, str(i), tuple(point), cv.FONT_HERSHEY_SIMPLEX, 1, color=(255,255,255))
        
        # Draw pose
        if self.debug_draw_pose:
            square_points = np.array([[0,0,0],[0.05,0,0],[0.05,0.05,0],[0,0.05,0]], dtype=np.float32) # Define Square geometry to compare with contour, 5x5cm Square.
            cube_points = np.array([[0,0,0],[0.05,0,0],[0.05,0.05,0],[0,0.05,0],
                                    [0,0,-0.05],[0.05,0,-0.05],[0.05,0.05,-0.05],[0,0.05,-0.05]]) # Define Cube geometry, 5x5 Cube
            for cnt in cnts:
                epsilon = self.alpha * cv.arcLength(cnt, True)
                approx = cv.approxPolyDP(cnt, epsilon, True)
                if len(approx) == 4: # If rectangle (4 sides)
                        corners = self.sortCorners(approx.reshape(-1,2))
                        image_points = np.array(corners, dtype=np.float32)

                        # Estimate pose
                        success, rvec, tvec, inliers = cv.solvePnPRansac(square_points, image_points, self.K, self.D, reprojectionError=8.0, confidence=0.99, flags=cv.SOLVEPNP_ITERATIVE)
                        if success:
                            imgpts, _ = cv.projectPoints(cube_points, rvec, tvec, self.K, self.D)
                            imgpts = np.int32(imgpts).reshape(-1,2)

                            cv.drawContours(bgr_mask,[imgpts[:4]], -1, (255,0,0), 2)
                            for i in range(4):
                                cv.line(bgr_mask, tuple(imgpts[i]), tuple(imgpts[i+4]), (0,255,0), 2)
                            cv.drawContours(bgr_mask, [imgpts[4:]], -1, (0,0,255), 2)

        return bgr_mask




    def findCubePoses(self, image):
        """
        Finds and publishes poses of cubes in the image

        First each color is iterated and contours of that color within a certain size are found.
        If any contours look like a square the cube pose is estimated with PnP.
        The estimated poses are then drawn on the image and published.
        """

        # Configure cube array msg
        cube_msg = DetectedCubeArray()
        cube_msg.header.stamp = self.get_clock().now().to_msg()
        cube_msg.header.frame_id = self.frame_id

        square_points = np.array([[0,0,0],[0.05,0,0],[0.05,0.05,0],[0,0.05,0]], dtype=np.float32) # Define Square geometry to compare with contour, 5x5cm Square.
        cube_points = np.array([[0,0,0],[0.05,0,0],[0.05,0.05,0],[0,0.05,0],
                                [0,0,-0.05],[0.05,0,-0.05],[0.05,0.05,-0.05],[0,0.05,-0.05]]) # Define Cube geometry, 5x5 Cube

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) # Create image copy in hsv format
        for name, clr in self.colors.items():
            mask = cv.inRange(hsv, clr[0], clr[1]) # Create mask of identified color
            cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            
            for cnt in cnts:
                area = cv.contourArea(cnt)
                if self.minArea < area < self.maxArea:
                    epsilon = self.alpha * cv.arcLength(cnt, True)
                    approx = cv.approxPolyDP(cnt, epsilon, True)

                    if len(approx) == 4: # If rectangle (4 sides)
                        corners = self.sortCorners(approx.reshape(-1,2))
                        for point in corners:
                            cv.circle(image, tuple(point), radius=5, color=(0,255,0), thickness=-1)
                        image_points = np.array(corners, dtype=np.float32)

                        # Estimate pose
                        success, rvec, tvec, inliers = cv.solvePnPRansac(square_points, image_points, self.K, self.D, reprojectionError=8.0, confidence=0.99, flags=cv.SOLVEPNP_ITERATIVE)

                        if success:
                            imgpts, _ = cv.projectPoints(cube_points, rvec, tvec, self.K, self.D)
                            imgpts = np.int32(imgpts).reshape(-1,2)

                            cv.drawContours(image,[imgpts[:4]], -1, (255,0,0), 2)
                            for i in range(4):
                                cv.line(image, tuple(imgpts[i]), tuple(imgpts[i+4]), (0,255,0), 2)
                            cv.drawContours(image, [imgpts[4:]], -1, (0,0,255), 2)

                            # Convert rot vector to quaternion
                            rot_mat, _ = cv.Rodrigues(rvec) # Rodrigues vector to rotation matrix
                            transform = np.eye(4) 
                            transform[:3, :3] = rot_mat
                            transform[:3,  3] = tvec.reshape(3)
                            quat = tf_transformations.quaternion_from_matrix(transform)

                            # Create cube message and populate array
                            cube = DetectedCube()
                            cube.color = name
                            cube.pose.position.x = float(tvec[0])
                            cube.pose.position.y = float(tvec[1])
                            cube.pose.position.z = float(tvec[2])
                            cube.pose.orientation.x = float(quat[0])
                            cube.pose.orientation.y = float(quat[1])
                            cube.pose.orientation.z = float(quat[2])
                            cube.pose.orientation.w = float(quat[3])

                            cube_msg.cubes.append(cube)
                            
                        else:
                            self.get_logger().warn(f"Pose estimation failed for color {name}")

        self.cube_publisher.publish(cube_msg)
        return image
    
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