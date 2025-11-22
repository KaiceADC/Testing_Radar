#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from radar_msgs.msg import RadarScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class RadarPathTracker(Node):
    def __init__(self):
        super().__init__('radar_path_tracker')
        
        self.subscription = self.create_subscription(
            RadarScan,
            'radar/scan',
            self.radar_callback,
            10
        )
        
        self.path_publisher = self.create_publisher(Path, 'radar/path', 10)
        
        self.path = Path()
        self.path.header.frame_id = "base_link"
        self.max_path_length = 500
        
        self.get_logger().info("Radar Path Tracker started")
    
    def radar_callback(self, msg):
        for detection in msg.returns:
            # Convert to Cartesian
            x = detection.range * math.cos(detection.azimuth)
            y = detection.range * math.sin(detection.azimuth)
            
            # Create pose
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            self.path.poses.append(pose)
            
            if len(self.path.poses) > self.max_path_length:
                self.path.poses.pop(0)
        
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)
        self.get_logger().debug(f"Path length: {len(self.path.poses)} points")

def main(args=None):
    rclpy.init(args=args)
    node = RadarPathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

