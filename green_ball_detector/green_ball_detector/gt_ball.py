#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PointStamped, Vector3Stamped
import rclpy.duration


class GroundTruthBallPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_ball_publisher')
        
        self.marker_pub = self.create_publisher(Marker, '/ground_truth_ball', 1)
        self.point_pub = self.create_publisher(PointStamped, '/ground_truth_ball_point', 10)
        
        self.annotation_sub = self.create_subscription(
            Vector3Stamped,
            'SlyAnnotations',
            self.annotation_callback,
            10
        )
        
        self.declare_parameter('marker_alpha', 0.6)
        self.declare_parameter('frame_id', 'lidar_top_link') 
        self.declare_parameter('color_r', 1.0)  
        self.declare_parameter('color_g', 0.0)
        self.declare_parameter('color_b', 0.0)
        
        self.marker_alpha = self.get_parameter('marker_alpha').get_parameter_value().double_value
        self.target_frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.color_r = self.get_parameter('color_r').get_parameter_value().double_value
        self.color_g = self.get_parameter('color_g').get_parameter_value().double_value
        self.color_b = self.get_parameter('color_b').get_parameter_value().double_value
        
        self.current_position = None
        self.current_dimensions = None
        self.current_timestamp = None
        self.message_count_for_timestamp = 0
        
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_ground_truth)
        
        self.get_logger().info("Ground Truth Cube Publisher uruchomiony.")
        self.get_logger().info("Nasłuchuje adnotacji na topicu 'SlyAnnotations'...")
        self.get_logger().info("Format oczekiwany: pozycja -> wymiary -> separator (0,0,0)")

    def annotation_callback(self, msg):
        """
        Parsuje sekwencję wiadomości SlyAnnotations:
        1. Pozycja (x,y,z centrum sześcianu)
        2. Wymiary (szerokość, wysokość, głębokość)  
        3. Separator (0,0,0)
        """
        current_frame_timestamp = msg.header.frame_id
        
        if self.current_timestamp != current_frame_timestamp:
            self.current_timestamp = current_frame_timestamp
            self.message_count_for_timestamp = 0
        
        self.message_count_for_timestamp += 1
        
        if self.message_count_for_timestamp == 1:
            self.current_position = (-msg.vector.x, msg.vector.y, msg.vector.z)
            self.get_logger().debug(
                f"Pozycja: x={msg.vector.x:.3f}, y={msg.vector.y:.3f}, z={msg.vector.z:.3f}"
            )
            
        elif self.message_count_for_timestamp == 2:
            self.current_dimensions = (msg.vector.x, msg.vector.y, msg.vector.z)
            self.get_logger().debug(
                f"Wymiary: x={msg.vector.x:.3f}, y={msg.vector.y:.3f}, z={msg.vector.z:.3f}"
            )
            
        elif self.message_count_for_timestamp == 3:
            # Trzecia wiadomość = separator (powinno być 0,0,0)
            if abs(msg.vector.x) < 0.001 and abs(msg.vector.y) < 0.001 and abs(msg.vector.z) < 0.001:
                self.get_logger().debug("Kompletny zestaw adnotacji otrzymany")
            else:
                self.get_logger().warn("Oczekiwano separatora (0,0,0), ale otrzymano inne wartości")

    def publish_ground_truth(self):
        """Publikuje ground truth marker jeśli mamy kompletne dane"""
        if self.current_position is None or self.current_dimensions is None:
            return
            
        marker = Marker()
        marker.header = Header(
            frame_id=self.target_frame_id,
            stamp=self.get_clock().now().to_msg()
        )
        marker.ns = "ground_truth"
        marker.id = 100
        marker.type = Marker.CUBE 
        marker.action = Marker.ADD

        marker.pose.position.x = self.current_position[0]
        marker.pose.position.y = self.current_position[1] 
        marker.pose.position.z = self.current_position[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = self.current_dimensions[0]
        marker.scale.y = self.current_dimensions[1]
        marker.scale.z = self.current_dimensions[2]
    
        marker.color = ColorRGBA(
            r=self.color_r,
            g=self.color_g, 
            b=self.color_b,
            a=self.marker_alpha
        )
        
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        
        self.marker_pub.publish(marker)
        
        point_msg = PointStamped()
        point_msg.header = marker.header
        point_msg.point.x = self.current_position[0]
        point_msg.point.y = self.current_position[1]
        point_msg.point.z = self.current_position[2]
        self.point_pub.publish(point_msg)
        
        if hasattr(self, '_last_log_time'):
            if (self.get_clock().now().nanoseconds - self._last_log_time) > 2e9:  
                self._log_current_state()
        else:
            self._log_current_state()
            
    def _log_current_state(self):
        """Loguje aktualny stan"""
        self._last_log_time = self.get_clock().now().nanoseconds
        pos = self.current_position
        dim = self.current_dimensions
        self.get_logger().info(
            f"GT Cube - Pozycja: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), "
            f"Wymiary: ({dim[0]:.3f}, {dim[1]:.3f}, {dim[2]:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GroundTruthBallPublisher()
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nZamykanie...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()