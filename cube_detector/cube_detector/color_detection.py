import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class ColorDetectionNode(Node):
    """
    Node for detecting colors blobs
    """
    color = (255,255,255)

    # Colors to check search for in hsv format
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

        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        for name, clr in self.colors.items():
            if self.find_color(hsv, clr):
                c, cx, cy = self.find_color(hsv, clr)
                cv.drawContours(cv_image, [c], -1, self.color, 3) #draw contours
                cv.circle(cv_image, (cx, cy), 7, self.color, -1)  # draw circle
                cv.putText(cv_image, name, (cx,cy), 
                            cv.FONT_HERSHEY_SIMPLEX, 1, self.color, 1) # put text

        try:
            blur_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
  
        self.publisher.publish(blur_msg)

# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()