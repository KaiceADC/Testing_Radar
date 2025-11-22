#!/usr/bin/env python3
import rclpy, serial, struct, math
from rclpy.node import Node
from std_msgs.msg import Header
from radar_msgs.msg import RadarScan, RadarReturn

class RadarSerialPublisher(Node):
    def __init__(self):
        super().__init__('radar_serial_publisher')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'radar_1')
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        self.ser = serial.Serial(port, baudrate, timeout=1.0)
        self.publisher = self.create_publisher(RadarScan, 'radar/scan', 10)
        self.timer = self.create_timer(0.02, self.read_and_publish)
        self.buffer = bytearray()
        
        self.get_logger().info(f"Connected - SYNC-BASED PARSER")
    
    def read_and_publish(self):
        detections = []
        
        # Read available data into buffer
        if self.ser.in_waiting > 0:
            self.buffer.extend(self.ser.read(self.ser.in_waiting))
        
        # Search for track sync pattern: 08 XX 04 where XX is E0-FF (track messages)
        i = 0
        while i < len(self.buffer) - 10:
            # Look for pattern: 08 EX 04
            if self.buffer[i] == 0x08 and self.buffer[i+2] == 0x04:
                can_id_byte = self.buffer[i+1]
                can_id = 0x0400 | can_id_byte  # Combine to get 0x04E0, 0x04E1, etc.
                
                if 0x4E0 <= can_id <= 0x4FF:
                    # Found a track message!
                    # Data should be BEFORE the sync pattern
                    if i >= 6:
                        data = self.buffer[i-6:i]
                        track_id = can_id - 0x4E0
                        
                        try:
                            # Parse track data
                            range_raw = struct.unpack('<H', data[0:2])[0]
                            range_m = range_raw * 0.1
                            
                            bearing_raw = struct.unpack('b', data[2:3])[0]
                            bearing_deg = bearing_raw * 0.1
                            bearing_rad = math.radians(bearing_deg)
                            
                            velocity_raw = struct.unpack('b', data[3:4])[0]
                            doppler_velocity = velocity_raw * 0.25
                            
                            status = struct.unpack('B', data[5:6])[0]
                            
                            if status > 0 and -40 <= bearing_deg <= 40:
                                detection = RadarReturn()
                                detection.range = range_m
                                detection.azimuth = bearing_rad
                                detection.doppler_velocity = doppler_velocity
                                
                                detections.append(detection)
                                
                                self.get_logger().info(
                                    f"✓ Track {track_id}: {range_m:.2f}m @ {bearing_deg:.1f}°, v={doppler_velocity:.2f}m/s"
                                )
                        except:
                            pass
            
            i += 1
        
        # Remove processed data (keep last 20 bytes for safety)
        if len(self.buffer) > 20:
            self.buffer = self.buffer[-20:]
        
        if detections:
            scan_msg = RadarScan()
            scan_msg.header = Header()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = self.frame_id
            scan_msg.returns = detections
            
            self.publisher.publish(scan_msg)
            self.get_logger().info(f"📡 Published {len(detections)} detections")

def main(args=None):
    rclpy.init(args=args)
    node = RadarSerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
