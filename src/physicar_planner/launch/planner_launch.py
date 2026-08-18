"""Corridor planner alone, for looking at what it decides.

The full race stack is physicar_bringup/real_autonomy_launch.py -- this is
for pointing the car at something and reading the numbers.

    ros2 launch physicar_planner planner_launch.py debug:=true
    ros2 launch physicar_planner planner_launch.py aggression:=2.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TUNABLES = ('aggression', 'lookahead_gain', 'lookahead_base', 'max_range',
            'max_lateral_slope', 'block_h_min', 'block_h_max', 'block_s_min',
            'speed_cap', 'debug')


def _parse(text):
    """Launch args arrive as strings; parameters are typed. Empty means
    'leave the node default alone' rather than 'set it to ""'."""
    if text is None or text.strip() == '':
        return None
    low = text.strip().lower()
    if low in ('true', 'false'):
        return low == 'true'
    try:
        return float(text)
    except ValueError:
        return text


def _build(context, *_a, **_k):
    params = {}
    for n in TUNABLES:
        v = _parse(LaunchConfiguration(n).perform(context))
        if v is not None:
            # These three are counts, not distances; rclpy is strict about it.
            if n in ('block_h_min', 'block_h_max', 'block_s_min') and \
                    isinstance(v, float):
                v = int(v)
            params[n] = v
    return [Node(package='physicar_planner', executable='planner_node',
                 name='planner_node', output='screen',
                 parameters=[params] if params else [],
                 remappings=[('image_raw',
                              LaunchConfiguration('image_topic').perform(context))])]


def generate_launch_description():
    args = [DeclareLaunchArgument('image_topic', default_value='/camera/image_raw')]
    args += [DeclareLaunchArgument(n, default_value='') for n in TUNABLES]
    return LaunchDescription(args + [OpaqueFunction(function=_build)])
