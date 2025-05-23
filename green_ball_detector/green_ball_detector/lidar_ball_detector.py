#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs_py import point_cloud2

class GreenBallDetector(Node):
    def __init__(self):
        super().__init__('green_ball_detector')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_distance', 8.5),
                ('min_distance', 4.5),
                ('ball_radius', 0.35),
                ('cluster_eps', 0.1), 
                ('cluster_min_samples', 8),  
                ('fov_degrees', 60),
                ('max_z', 2.0) 
            ]
        )
        
        self.max_distance = self.get_parameter('max_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        self.ball_radius = self.get_parameter('ball_radius').value
        self.cluster_eps = self.get_parameter('cluster_eps').value
        self.cluster_min_samples = self.get_parameter('cluster_min_samples').value
        self.fov_rad = np.deg2rad(self.get_parameter('fov_degrees').value / 2)
        self.max_z = self.get_parameter('max_z').value
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/concatenated/pointcloud',
            self.pc_callback,
            10
        )
        self.marker_pub = self.create_publisher(Marker, '/detected_green_ball', 10)
        self.debug_pub = self.create_publisher(PointCloud2, '/filtered_points', 10)
        
        self.get_logger().info("Node initialized. Waiting for point cloud data...")

    def create_ball_marker(self, position):
        marker = Marker()
        marker.header = Header(frame_id="lidar_top_link", stamp=self.get_clock().now().to_msg())
        marker.ns = "green_ball"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = self.ball_radius * 2
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        return marker

    def pc_callback(self, msg):
        # Konwersja PointCloud2 do numpy
        pc_data = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = np.array([[-p[0], -p[1], p[2]] for p in pc_data], dtype=np.float32)

        
        if len(points) == 0:
            return

        # Filtracja kątowa (60 stopni z przodu)
        yaw = np.arctan2(points[:,1], points[:,0])
        angle_mask = np.abs(yaw) < self.fov_rad
        
        # Filtracja odległościowa
        distances = np.linalg.norm(points[:,:2], axis=1) 
        distance_mask = (distances > self.min_distance) & (distances < self.max_distance)
        
        # Filtracja wysokości
        z_mask = np.abs(points[:,2]) < self.max_z
        
        # Połączone maski
        combined_mask = angle_mask & distance_mask & z_mask
        filtered_points = points[combined_mask]
        
        # Publikuj przefiltrowane punkty do debugowania
        debug_header = Header(frame_id="lidar_top_link", stamp=msg.header.stamp)
        debug_msg = point_cloud2.create_cloud_xyz32(debug_header, filtered_points)
        self.debug_pub.publish(debug_msg)
        
        if len(filtered_points) == 0:
            return

        # Klasteryzacja DBSCAN
        clustering = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples).fit(filtered_points)
        
        # Analiza klastrów
        clusters = []
        for label in np.unique(clustering.labels_):
            if label == -1:
                continue
                
            cluster_mask = (clustering.labels_ == label)
            cluster_points = filtered_points[cluster_mask]
            centroid = np.mean(cluster_points, axis=0)
            
            # Oblicz maksymalną odległość od centroidu
            distances_from_centroid = np.linalg.norm(cluster_points - centroid, axis=1)
            max_distance = np.max(distances_from_centroid)
            
            # Warunek walidacji klastra
            if (self.ball_radius * 0.7 < max_distance < self.ball_radius * 1.3 and
                len(cluster_points) > self.cluster_min_samples and
                abs(centroid[2]) < self.max_z):
                
                clusters.append({
                    'centroid': centroid,
                    'size': len(cluster_points),
                    'max_distance': max_distance
                })
        
        if clusters:
            best_cluster = max(clusters, key=lambda x: x['size'])
            self.get_logger().info(f"Detected ball at: {best_cluster['centroid']}")
            self.marker_pub.publish(self.create_ball_marker(best_cluster['centroid']))

def main(args=None):
    rclpy.init(args=args)
    detector = GreenBallDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()