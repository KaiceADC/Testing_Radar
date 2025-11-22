#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from radar_msgs.msg import RadarScan
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import math
from collections import deque

class RadarPointCloudNode(Node):
    def __init__(self):
        super().__init__('radar_pointcloud')
        
        self.subscription = self.create_subscription(
            RadarScan,
            'radar/scan',
            self.radar_callback,
            10
        )
        
        self.publisher = self.create_publisher(PointCloud2, 'radar/pointcloud', 10)
        
        # IMPROVED: Increased buffer for smoother visualization
        self.recent_points = deque(maxlen=300)
        self.current_batch = []
        self.publish_interval = 0.05  # 20Hz
        self.timer = self.create_timer(self.publish_interval, self.publish_batch)
        
        # Filter parameters
        self.min_distance = 0.5  # Changed from 0.3 to match Arduino filter
        self.max_distance = 200.0
        self.max_angle = math.radians(90)  # ±90 degrees
        
        self.get_logger().info("Radar PointCloud Node started")
    
    def radar_callback(self, msg):
        """Convert radar detections to points"""
        for detection in msg.returns:
            # Apply filters
            if detection.range < self.min_distance or detection.range > self.max_distance:
                continue
            
            if abs(detection.azimuth) > self.max_angle:
                continue
            
            # Convert to Cartesian
            x = detection.range * math.cos(detection.azimuth)
            y = detection.range * math.sin(detection.azimuth)
            z = 0.0  # Assume flat ground plane
            
            # Velocity magnitude for color intensity
            velocity = abs(detection.doppler_velocity)
            intensity = min(255, int(velocity * 25))  # Scale velocity to 0-255
            
            # IMPROVED: Store point with timestamp
            point = {
                'x': x,
                'y': y,
                'z': z,
                'intensity': intensity,
                'velocity': detection.doppler_velocity,
                'timestamp': self.get_clock().now()
            }
            
            self.recent_points.append(point)
            self.current_batch.append(point)
    
    def publish_batch(self):
        """Publish accumulated points as PointCloud2"""
        if not self.recent_points:
            return
        
        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        
        # Define fields (XYZI + Velocity)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack point data
        cloud_data = []
        for point in self.recent_points:
            cloud_data.append(struct.pack('fffff', 
                                         point['x'], 
                                         point['y'], 
                                         point['z'], 
                                         float(point['intensity']),
                                         point['velocity']))
        
        # Create PointCloud2
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(cloud_data)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 20  # 5 fields * 4 bytes
        pc2.row_step = pc2.point_step * pc2.width
        pc2.is_dense = True
        pc2.data = b''.join(cloud_data)
        
        self.publisher.publish(pc2)
        self.get_logger().info(f'Published pointcloud with {len(cloud_data)} points')
        
        # Clear current batch
        self.current_batch = []

def main(args=None):
    rclpy.init(args=args)
    node = RadarPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

