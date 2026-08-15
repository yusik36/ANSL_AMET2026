"""Lidar-only bench test: lidar + driver + obstacle-avoid + judgment, with the
judgment node's lane/traffic gates bypassed (no camera in this launch) so the
car drives straight and only reacts to /scan obstacles -- equivalent to what
this launch did before physicar_judgment existed, now going through the same
arbitration node the full stack uses instead of a special-cased direct path.
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
        launch_arguments={'serial_port': '/dev/ttyUSB2'}.items(),
    )

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
    )

    judgment_node = Node(
        package='physicar_judgment',
        executable='judgment_node',
        name='judgment_node',
        output='screen',
        parameters=[{
            'require_lane_gate': False,
            'require_traffic_gate': False,
        }],
    )

    return LaunchDescription([sllidar_launch, driver_node, avoid_node, judgment_node])
