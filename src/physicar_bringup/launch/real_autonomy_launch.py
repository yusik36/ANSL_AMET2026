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
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# Tunables worth sweeping between benchmark runs, exposed as launch
# arguments. The nodes read their parameters once at construction and cache
# them, so `ros2 param set` on a running node has no effect -- an override has
# to arrive at launch time. Empty means "leave the node's own default alone".
TUNABLES = {
    'obstacle_avoid_node': (
        'physicar_nav', 'avoid_node',
        ('forward_speed', 'avoid_speed', 'stop_distance', 'avoid_distance',
         'front_offset_deg', 'avoid_steer_deg', 'avoid_steer_sign'),
    ),
    'judgment_node': (
        'physicar_judgment', 'judgment_node',
        ('require_lane_gate', 'require_traffic_gate', 'lane_hold_s',
         'input_stale_s'),
    ),
}


def parse_override(text):
    """Launch arguments arrive as strings; node parameters are typed.

    Returns the value in the type the node declared, or None for "not set",
    which is how an untouched argument is dropped instead of reaching the
    node as an empty string it would reject.
    """
    if text is None:
        return None
    text = text.strip()
    if text == '':
        return None
    low = text.lower()
    if low in ('true', 'false'):
        return low == 'true'
    try:
        # Deliberately float, never int. Every numeric tunable above is
        # declared as a double, and rclpy rejects an int for a double
        # parameter -- so `forward_speed:=2` would fail while `2.0` worked,
        # which is a miserable thing to discover mid-session. Anything
        # integer-typed added to TUNABLES needs handling here.
        return float(text)
    except ValueError:
        pass
    return text


def _build(context, *_args, **_kwargs):
    nodes = []
    for node_name, (pkg, executable, names) in TUNABLES.items():
        params = {}
        for n in names:
            value = parse_override(LaunchConfiguration(n).perform(context))
            if value is not None:
                params[n] = value
        nodes.append(Node(
            package=pkg,
            executable=executable,
            name=node_name,
            output='screen',
            parameters=[params] if params else [],
        ))
    return nodes


def generate_launch_description():
    args = [DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='Real Physicar camera topic.',
    )]
    for node_name, (_pkg, _exe, names) in TUNABLES.items():
        for n in names:
            args.append(DeclareLaunchArgument(
                n, default_value='',
                description='Override %s on %s (empty = node default).'
                            % (n, node_name),
            ))

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('physicar_vision'), 'launch', 'vision_launch.py')
        ),
        launch_arguments={'image_topic': LaunchConfiguration('image_topic')}.items(),
    )

    return LaunchDescription(args + [vision_launch, OpaqueFunction(function=_build)])
