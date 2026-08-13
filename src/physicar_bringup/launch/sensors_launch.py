"""Bring up all sensors together: lidar (/scan), camera (/image_raw), imu (/imu/data, /imu/mag).

Usage:
    ros2 launch physicar_bringup sensors_launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py',
            )
        ),
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('physicar_imu'),
                'launch',
                'imu_launch.py',
            )
        ),
    )

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'pixel_format': 'mjpeg2rgb',
            'image_width': 1280,
            'image_height': 720,
            'framerate': 30.0,
        }],
    )

    return LaunchDescription([sllidar_launch, imu_launch, camera_node])
