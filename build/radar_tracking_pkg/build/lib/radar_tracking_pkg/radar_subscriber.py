import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class RadarSubscriberNode(Node):
    """ROS 2 Node that receives and processes radar data"""
    
    def __init__(self):
        super().__init__('radar_subscriber')
        
        # Subscribe to radar objects topic
        self.subscription = self.create_subscription(
            Point,
            'radar/objects',
            self.listener_callback,
            10
        )
        
        self.get_logger().info('📡 Radar Subscriber Node started')
        self.get_logger().info('✅ Listening on topic: /radar/objects')
    
    def listener_callback(self, msg):
        """Callback when message received"""
        distance = msg.x
        angle = msg.y
        velocity = msg.z
        
        self.get_logger().info(
            f'✅ Received: Distance={distance:.2f}m, Angle={angle:.1f}°, Velocity={velocity:.2f}m/s'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RadarSubscriberNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
