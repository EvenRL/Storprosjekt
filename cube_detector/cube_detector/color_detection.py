import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class ColorDetectionNode(Node):
    """
    Node for detecting colored cubes
    """
    color = (255,255,255)

    # Colors to search for. hsv format
    colors = {'blue': [np.array([90, 50, 70]), np.array([128, 255, 255])],
        'red': [np.array([159, 50, 70]), np.array([180, 255, 255])],
        'yellow': [np.array([25, 50, 70]), np.array([35, 255, 255])],
        'green': [np.array([36, 50, 70]), np.array([89, 255, 255])]}
    
    def find_color(self, frame, points):
        mask = cv.inRange(frame, points[0], points[1])#create mask with boundaries 
        cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, 
                               cv.CHAIN_APPROX_SIMPLE) # find contours from mask
        for c in cnts:
            area = cv.contourArea(c) # find how big countour is
            if area > 5000:       # only if countour is big enough, then
                M = cv.moments(c)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position
                return c, cx, cy

    def findColorContours(self, image):
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) # create image copy in hsv format

        for name, clr in self.colors.items():
            if self.find_color(hsv, clr):
                c, cx, cy = self.find_color(hsv, clr)
                cv.drawContours(image, [c], -1, self.color, 3) #draw contours
                cv.circle(image, (cx, cy), 7, self.color, -1)  # draw circle
                cv.putText(image, name, (cx,cy), 
                            cv.FONT_HERSHEY_SIMPLEX, 1, self.color, 1) # put text
        return image

    def findCubeContours(self, image):
        camera_matrix = np.array([[623.60721, 0.0, 297.00864],
                         [0.0, 620.86526, 244.34018],
                         [0.0, 0.0, 1.0]], dtype=np.float32)
        object_points = np.array([[0,0,0],[0.05,0,0],[0.05,0.05,0],[0,0.05,0]], dtype=np.float32)
        dist_coeffs = np.array([-0.019917, -0.022530, 0.002542, -0.009390, 0.000000])
        cube_points = np.array([[0,0,0],[0.05,0,0],[0.05,0.05,0],[0,0.05,0],
                                [0,0,-0.05],[0.05,0,-0.05],[0.05,0.05,-0.05],[0,0.05,-0.05]])

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) # create image copy in hsv format
        for name, clr in self.colors.items():
            mask = cv.inRange(hsv, clr[0], clr[1])
            cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for cnt in cnts:
                area = cv.contourArea(cnt) # find how big countour is
                if 2500 < area < 10000:       # only if countour is big enough
                    epsilon = 0.05 * cv.arcLength(cnt, True)
                    approx = cv.approxPolyDP(cnt, epsilon, True)

                    if len(approx) == 4:
                        corners = approx.reshape(-1,2)
                        for point in corners:
                            cv.circle(image, tuple(point), radius=5, color=(0,255,0), thickness=-1)
                        image_points = np.array(corners, dtype=np.float32)

                        success, rvec, tvec = cv.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
                        imgpts, _ = cv.projectPoints(cube_points, rvec, tvec, camera_matrix, dist_coeffs)
                        imgpts = np.int32(imgpts).reshape(-1,2)

                        cv.drawContours(image,[imgpts[:4]], -1, (255,0,0), 2)
                        for i in range(4):
                            cv.line(image, tuple(imgpts[i]), tuple(imgpts[i+4]), (0,255,0), 2)
                        
                        cv.drawContours(image, [imgpts[4:]], -1, (0,0,255), 2)

        return image


        

    def __init__(self):
        super().__init__('color_detection')

        # Subscribe to image topic
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish the filtered image
        self.publisher = self.create_publisher(
            Image,
            'image_output',
            10)
  
        # Initialize CVBridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        new_image = self.findCubeContours(cv_image)

        try:
            cube_msg = self.bridge.cv2_to_imgmsg(new_image, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
  
        self.publisher.publish(cube_msg)

# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()