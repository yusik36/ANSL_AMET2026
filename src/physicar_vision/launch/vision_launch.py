"""Brings up the camera-based perception nodes (lane following + traffic
light gate). Assumes a camera image topic is already being published.

Practice platform (usb_cam) publishes /image_raw -- that's the default here.
The real Physicar publishes /camera/image_raw instead (confirmed against the
official physicar-ros source, 2026-08-16) -- pass image_topic:=/camera/image_raw
when launching on the real vehicle (real_autonomy_launch.py does this for you).

Usage:
    ros2 launch physicar_vision vision_launch.py
    ros2 launch physicar_vision vision_launch.py image_topic:=/camera/image_raw
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='image_raw',
        description='Camera image topic to subscribe to.',
    )
    image_topic = LaunchConfiguration('image_topic')
    remap = [('image_raw', image_topic)]

    lane_node = Node(
        package='physicar_vision',
        executable='lane_follow_node',
        name='lane_follow_node',
        output='screen',
        remappings=remap,
    )

    traffic_node = Node(
        package='physicar_vision',
        executable='traffic_light_node',
        name='traffic_light_node',
        output='screen',
        remappings=remap,
    )

    return LaunchDescription([image_topic_arg, lane_node, traffic_node])
