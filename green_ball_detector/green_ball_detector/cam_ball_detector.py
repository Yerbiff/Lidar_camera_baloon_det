#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy



class CamBallDetector(Node):
    def __init__(self):
        super().__init__('cam_ball_detector')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        
        self.subscription = self.create_subscription(
            Image,
            '/sensing/camera/front/image_raw',
            self.image_callback,
            qos_profile 
        )
        self.subscription 
        self.bridge = CvBridge()
        self.window_name = "Cam Ball Detection"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Błąd konwersji obrazu: {e}")
            return

        processed_image = self.detect_green_ball(cv_image)
        
        cv2.imshow(self.window_name, processed_image)
        cv2.waitKey(1)

    def detect_green_ball(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            if radius > 10:
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.putText(image, "Green Ball", (int(x - radius), int(y - radius)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return image

def main(args=None):
    rclpy.init(args=args)
    detector = CamBallDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()