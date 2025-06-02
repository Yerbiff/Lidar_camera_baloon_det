#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

class CamBallDetector(Node):
    def __init__(self):
        super().__init__('cam_ball_detector')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.image_sub = self.create_subscription(
            Image, '/sensing/camera/front/image_raw', self.image_callback, qos
        )
        self.gt_sub = self.create_subscription(
            PointStamped, '/ground_truth_ball_point', self.gt_callback, 10
        )

        self.bridge = CvBridge()
        cv2.namedWindow("Ball Detection", cv2.WINDOW_NORMAL)

        self.K = np.array([[533.0181, 0.0, 533.1950],
                           [0.0, 484.9178, 309.9587],
                           [0.0, 0.0, 1.0]])

        self.ball_diameter = 0.58
        self.gt_point = None
        self.errors = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def gt_callback(self, msg):
        self.gt_point = (msg.point.x, msg.point.y, msg.point.z)

    def detect_green_ball(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None
        largest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest)
        if radius < 10:
            return None, None
        return (int(x), int(y)), int(radius)

    def estimate_3d_position(self, center, radius, stamp):
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        u, v = center
        depth = (fx * self.ball_diameter) / (2 * radius)
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        try:
            pt = PointStamped()
            pt.header.frame_id = 'camera_front_optical_link'
            pt.header.stamp = stamp
            pt.point = Point(x=x, y=y, z=z)
            transform = self.tf_buffer.lookup_transform(
                'lidar_top_link', 'camera_front_optical_link', stamp,
                timeout=Duration(seconds=0.1)
            )
            pt_lidar = do_transform_point(pt, transform)
            return (pt_lidar.point.x, pt_lidar.point.y, pt_lidar.point.z)
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return None

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image = cv2.resize(image, (960, 600))
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        center, radius = self.detect_green_ball(image)

        if center and radius:
            cv2.circle(image, center, radius, (0, 255, 255), 2)
            cv2.circle(image, center, 3, (0, 255, 255), -1)
            cv2.putText(image, f"R: {radius}px", (center[0] - 30, center[1] - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            estimated_pos = self.estimate_3d_position(center, radius, msg.header.stamp)

            if estimated_pos and self.gt_point:
                error = np.linalg.norm(np.array(estimated_pos) - np.array(self.gt_point))
                self.errors.append(error)
                if len(self.errors) > 50:
                    self.errors.pop(0)
                avg_error = np.mean(self.errors)

                y = 30
                cv2.putText(image, f"Error: {error:.2f}m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                y += 30
                cv2.putText(image, f"Avg: {avg_error:.2f}m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                y += 30
                cv2.putText(image, f"Est: ({estimated_pos[0]:.2f}, {estimated_pos[1]:.2f}, {estimated_pos[2]:.2f})",
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                y += 25
                cv2.putText(image, f"GT:  ({self.gt_point[0]:.2f}, {self.gt_point[1]:.2f}, {self.gt_point[2]:.2f})",
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)


        else:
            cv2.putText(image, "No ball detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Ball Detection", image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CamBallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
