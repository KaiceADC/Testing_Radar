from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Serial publisher node (connects to Arduino serial)
        Node(
            package='radar_tracking_pkg',
            executable='radar_serial_publisher',
            name='radar_serial_publisher',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baud_rate': 115200},
            ],
        ),
        # EKF tracker node
        Node(
            package='radar_tracking_pkg',
            executable='radar_ekf_tracker_node',
            name='radar_ekf_tracker',
            output='screen',
        ),
        # PointCloud visualization node
        Node(
            package='radar_tracking_pkg',
            executable='radar_pointcloud_node',
            name='radar_pointcloud',
            output='screen',
        ),
    ])
