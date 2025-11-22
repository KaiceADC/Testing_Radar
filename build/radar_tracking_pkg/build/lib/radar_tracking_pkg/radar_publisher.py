import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
import random

class RadarSimulatorNode(Node):
    """ROS 2 Node that simulates RSV82 radar sensor"""
    
    def __init__(self):
        super().__init__('radar_simulator')
        
        # QoS profile for real-time updates
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publisher for radar objects
        self.publisher = self.create_publisher(
            Point,
            'radar/objects',
            qos_profile
        )
        
        # Timer for 50Hz update rate (20ms)
        self.timer = self.create_timer(0.02, self.publish_radar_data)
        self.object_counter = 0
        
        self.get_logger().info('🚗 Radar Simulator Node started')
        self.get_logger().info('📡 Publishing to topic: /radar/objects')
        self.get_logger().info('⏱️  Update rate: 50Hz (20ms)')
    
    def publish_radar_data(self):
        """Simulate and publish radar object detections"""
        
        # Randomly detect 0-3 objects per scan
        num_objects = random.randint(0, 3)
        
        if num_objects == 0:
            self.get_logger().debug('💤 Keep-alive (no objects)')
        else:
            # Publish detected objects
            for i in range(num_objects):
                self.object_counter += 1
                
                # Generate random object data
                distance = random.uniform(0, 30)      # 0-30 meters
                velocity = random.uniform(-10, 10)    # -10 to +10 m/s
                angle = random.uniform(-90, 90)       # -90 to +90 degrees
                rcs = random.uniform(0, 10)           # 0-10 dB
                
                # Create message (x=distance, y=angle, z=velocity)
                msg = Point()
                msg.x = distance
                msg.y = angle
                msg.z = velocity
                
                # Publish
                self.publisher.publish(msg)
                
                self.get_logger().info(
                    f'🎯 Object {i}: {distance:.2f}m, {velocity:.2f}m/s, {angle:.1f}°'
                )

def main(args=None):
    rclpy.init(args=args)
    node = RadarSimulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n⏹️  Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
