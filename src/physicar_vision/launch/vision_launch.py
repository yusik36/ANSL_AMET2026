"""Brings up the camera-based perception nodes (lane following + traffic
light gate). Assumes /image_raw is already being published (e.g. by
physicar_bringup's sensors_launch.py, which starts usb_cam).

Usage:
    ros2 launch physicar_vision vision_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lane_node = Node(
        package='physicar_vision',
        executable='lane_follow_node',
        name='lane_follow_node',
        output='screen',
    )

    traffic_node = Node(
        package='physicar_vision',
        executable='traffic_light_node',
        name='traffic_light_node',
        output='screen',
    )

    return LaunchDescription([lane_node, traffic_node])
