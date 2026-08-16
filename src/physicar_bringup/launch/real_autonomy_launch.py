"""Real-vehicle deployment launch: perception + judgment ONLY.

Unlike autonomy_launch.py (practice platform), this does NOT launch
sensors_launch.py or physicar_driver's driver_node -- the real Physicar
already runs its own camera/lidar/IMU drivers and its own official
physicar_driver_node as system services before myapp.sh ever starts.
Re-launching any of that here would conflict with (or be redundant to)
what's already running.

This is what myapp.sh -- the official student-code slot, deployed via the
:5000 web UI to /opt/physicar/userdata/myapp.sh -- should invoke on race day:

    source install/setup.bash
    ros2 launch physicar_bringup real_autonomy_launch.py

Real-vehicle topic differences handled here (confirmed against the official
physicar-ros source and a live `ros2 topic list`, 2026-08-16):
  - camera publishes /camera/image_raw, not /image_raw (practice's usb_cam) --
    remapped via the image_topic launch argument.
  - /scan, /imu, /speed, /steering topic names already match the practice
    platform, no remapping needed for those.

Still placeholder/unverified going into this (see README calibration
checklist): lane HSV thresholds, front_offset_deg/avoid_steer_sign (these are
practice-chassis-specific and were never re-measured on the real car).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='Real Physicar camera topic.',
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('physicar_vision'), 'launch', 'vision_launch.py')
        ),
        launch_arguments={'image_topic': LaunchConfiguration('image_topic')}.items(),
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
    )

    return LaunchDescription([
        image_topic_arg,
        vision_launch,
        avoid_node,
        judgment_node,
    ])
