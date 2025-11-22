import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import random
import math

# Import custom message
from radar_simulator.msg import RadarObject

class RadarSimulatorEnhancedNode(Node):
    """ROS 2 Node that simulates RSV82 radar with custom message and filtering"""
    
    def __init__(self):
        super().__init__('radar_simulator_enhanced')
        
        # Declare parameters for data filtering
        self.declare_parameter('min_distance', 0.5)
        self.declare_parameter('max_distance', 30.0)
        self.declare_parameter('min_rcs', -10.0)
        self.declare_parameter('max_angle', 90.0)
        
        # Get parameters
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.min_rcs = self.get_parameter('min_rcs').value
        self.max_angle = self.get_parameter('max_angle').value
        
        # QoS profile for real-time updates
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher for custom radar objects
        self.publisher = self.create_publisher(
            RadarObject,
            'radar/objects_v2',
            qos_profile
        )
        
        # Timer for 50Hz update rate (20ms)
        self.timer = self.create_timer(0.02, self.publish_radar_data)
        self.object_counter = 0
        self.filtered_count = 0
        
        self.get_logger().info('🚗 Radar Simulator Enhanced Node started')
        self.get_logger().info('📡 Publishing to topic: /radar/objects_v2')
        self.get_logger().info(f'📏 Distance filter: {self.min_distance}m - {self.max_distance}m')
        self.get_logger().info(f'📊 RCS filter: {self.min_rcs}dB+')
        self.get_logger().info('⏱️  Update rate: 50Hz (20ms)')
    
    def publish_radar_data(self):
        """Simulate and publish radar object detections with filtering"""
        
        # Randomly detect 0-3 objects per scan
        num_objects = random.randint(0, 3)
        
        if num_objects == 0:
            self.get_logger().debug('💤 Keep-alive (no objects)')
        else:
            # Publish detected objects
            for i in range(num_objects):
                self.object_counter += 1
                
                # Generate random object data
                distance = random.uniform(0, 35)      # 0-35 meters
                velocity = random.uniform(-15, 15)    # -15 to +15 m/s
                angle = random.uniform(-90, 90)       # -90 to +90 degrees
                rcs = random.uniform(-20, 10)         # -20 to +10 dB
                
                # Apply filtering
                if self.is_valid_object(distance, rcs, angle):
                    # Create custom message
                    msg = RadarObject()
                    msg.distance = distance
                    msg.velocity = velocity
                    msg.angle = angle
                    msg.rcs = rcs
                    msg.object_id = i
                    
                    # Publish
                    self.publisher.publish(msg)
                    
                    self.get_logger().info(
                        f'✅ Object {i}: {distance:.2f}m, {velocity:.2f}m/s, {angle:.1f}°, {rcs:.1f}dB'
                    )
                else:
                    self.filtered_count += 1
                    self.get_logger().debug(
                        f'❌ Filtered Object {i}: {distance:.2f}m, {rcs:.1f}dB'
                    )
    
    def is_valid_object(self, distance, rcs, angle):
        """Apply data filtering rules"""
        
        # Distance filter
        if distance < self.min_distance or distance > self.max_distance:
            return False
        
        # RCS filter (ignore weak reflections)
        if rcs < self.min_rcs:
            return False
        
        # Angle filter (ignore out-of-range angles)
        if abs(angle) > self.max_angle:
            return False
        
        return True

def main(args=None):
    rclpy.init(args=args)
    node = RadarSimulatorEnhancedNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n⏹️  Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
