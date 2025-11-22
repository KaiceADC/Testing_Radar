import rclpy
from rclpy.node import Node
from radar_simulator.msg import RadarObject

class RadarSubscriberEnhancedNode(Node):
    """ROS 2 Node that receives and processes custom radar objects"""
    
    def __init__(self):
        super().__init__('radar_subscriber_enhanced')
        
        # Subscribe to custom radar objects topic
        self.subscription = self.create_subscription(
            RadarObject,
            'radar/objects_v2',
            self.listener_callback,
            10
        )
        
        self.object_count = 0
        self.get_logger().info('📡 Radar Subscriber Enhanced Node started')
        self.get_logger().info('✅ Listening on topic: /radar/objects_v2')
    
    def listener_callback(self, msg):
        """Callback when custom message received"""
        self.object_count += 1
        
        distance = msg.distance
        velocity = msg.velocity
        angle = msg.angle
        rcs = msg.rcs
        object_id = msg.object_id
        
        self.get_logger().info(
            f'[{self.object_count}] 🎯 ID:{object_id} | Distance:{distance:.2f}m | '
            f'Velocity:{velocity:.2f}m/s | Angle:{angle:.1f}° | RCS:{rcs:.1f}dB'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RadarSubscriberEnhancedNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
