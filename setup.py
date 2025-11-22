from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'radar_tracking_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # THIS LINE ADDS YOUR LAUNCH FILES!
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaice',
    maintainer_email='ktmanoj29@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'radar_path_tracker = radar_tracking_pkg.radar_path_tracker:main',
            'radar_ekf_tracker_node = radar_tracking_pkg.radar_ekf_tracker_node:main',
            'radar_subscriber = radar_tracking_pkg.radar_subscriber:main',
            'radar_publisher = radar_tracking_pkg.radar_publisher:main',
            'radar_publisher_enhanced = radar_tracking_pkg.radar_publisher_enhanced:main',
            'radar_subscriber_enhanced = radar_tracking_pkg.radar_subscriber_enhanced:main',
            'radar_serial_publisher = radar_tracking_pkg.radar_serial_publisher:main',
            'radar_serial_debug = radar_tracking_pkg.radar_serial_debug:main',
            'radar_pointcloud_node = radar_tracking_pkg.radar_pointcloud_node:main',
        ],
    },
)

