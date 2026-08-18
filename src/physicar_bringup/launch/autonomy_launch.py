"""One-command startup for the full perception-judgment-control stack:
sensors -> physicar_driver -> physicar_nav (obstacle avoidance) ->
physicar_vision (lane following + traffic light) -> physicar_judgment
(final arbitration to /speed + /steering).

Usage:
    ros2 launch physicar_bringup autonomy_launch.py

This launches everything with the judgment node's lane/traffic gates
enabled (the safe, full-stack default) -- for a lidar-only bench test with
no camera, use physicar_nav/avoid_test_launch.py instead.

The practice chassis differs from Physicar in two ways that flip signs, so
the overrides below are not optional -- without them the car reads its own
rear as the way ahead and steers into obstacles rather than around them.
Both were confirmed by bench test on 2026-08-14; see avoid_node.py for why
the node's own defaults describe Physicar instead.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    def include(pkg, launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(pkg), 'launch', launch_file)
            )
        )

    sensors_launch = include('physicar_bringup', 'sensors_launch.py')
    vision_launch = include('physicar_vision', 'vision_launch.py')

    driver_node = Node(
        package='physicar_driver',
        executable='driver_node',
        name='physicar_driver_node',
        output='screen',
    )

    avoid_node = Node(
        package='physicar_nav',
        executable='avoid_node',
        name='obstacle_avoid_node',
        output='screen',
        parameters=[{
            # Practice lidar is mounted backwards: scan angle 0 points at the
            # rear bumper, so the front cone has to be read 180 deg around.
            'front_offset_deg': 180.0,
            # Practice servo wiring makes positive /steering turn RIGHT, the
            # opposite of Physicar, so avoidance would steer into the obstacle.
            'avoid_steer_sign': -1.0,
        }],
    )

    judgment_node = Node(
        package='physicar_judgment',
        executable='judgment_node',
        name='judgment_node',
        output='screen',
    )

    return LaunchDescription([
        sensors_launch,
        vision_launch,
        driver_node,
        avoid_node,
        judgment_node,
    ])
