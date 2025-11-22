#!/usr/bin/env python3
"""
DEBUG VERSION: Displays raw hex data to reverse-engineer Arduino format
"""

import rclpy
from rclpy.node import Node
import serial
import struct

class RadarSerialDebug(Node):
    """
    Debug node - shows raw serial data in hex format
    """
    
    def __init__(self):
        super().__init__('radar_serial_debug')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f"Connected to {port} at {baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise
        
        # Timer to read and display data
        self.timer = self.create_timer(0.5, self.read_and_display)
        self.frame_count = 0
    
    def read_and_display(self):
        """Read serial data and print in hex format"""
        
        if self.ser.in_waiting >= 13:
            # Read up to 13 bytes
            frame = self.ser.read(13)
            self.frame_count += 1
            
            # Print raw hex
            hex_str = ' '.join(f'{b:02X}' for b in frame)
            print(f"\n[Frame {self.frame_count}] Raw (hex): {hex_str}")
            
            # Print as decimal
            dec_str = ' '.join(f'{b:3d}' for b in frame)
            print(f"[Frame {self.frame_count}] Raw (dec): {dec_str}")
            
            # Try to interpret as little-endian CAN ID + data
            try:
                can_id = struct.unpack('<I', frame[0:4])[0]
                print(f"  → Bytes 0:4 (LE CAN ID):  0x{can_id:04X} ({can_id})")
                
                data = frame[4:12]
                print(f"  → Bytes 4:12 (Data):      {' '.join(f'{b:02X}' for b in data)}")
                
                length = frame[12]
                print(f"  → Byte 12 (Length):       {length}")
            except Exception as e:
                print(f"  → Parse error: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RadarSerialDebug()
        print("\n=== RADAR SERIAL DEBUG MODE ===")
        print("Press Ctrl+C to stop\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nStopped.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
