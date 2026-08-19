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

Driving is physicar_planner: it reads the drivable corridor from the camera
and treats a cone as a place where that corridor narrows, so there is no
separate avoidance layer to hand control to and take it back from. The lane
follower and avoid_node it replaced are still in the tree as a fallback, but
nothing here starts them.

Only the traffic light is still taken from physicar_vision -- it answers a
different question (may the car move at all) and gates speed alone.

Measured in the simulator on 2026-08-18 and baked into the planner defaults:
the corridor is trustworthy to 2.5 m, and road/grass/cone separate cleanly by
hue and saturation. Both need re-measuring at the real venue -- the colours
under its lighting with hsv_calibrate_node, and the camera geometry with
tools/YS_perception_probe.py.
"""
from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Tunables worth sweeping between benchmark runs, exposed as launch
# arguments. The nodes read their parameters once at construction and cache
# them, so `ros2 param set` on a running node has no effect -- an override has
# to arrive at launch time. Empty means "leave the node's own default alone".
TUNABLES = {
    'planner_node': (
        'physicar_planner', 'planner_node',
        ('aggression', 'lookahead_gain', 'lookahead_base', 'max_range',
         'max_accel', 'max_decel',
         'max_lateral_slope', 'road_h_min', 'road_h_max', 'paint_s_max', 'paint_v_min',
         'mark_h_min', 'mark_h_max', 'mark_s_min', 'max_span',
         'speed_cap', 'debug'),
    ),
    'judgment_node': (
        'physicar_judgment', 'judgment_node',
        ('require_traffic_gate', 'plan_hold_s', 'input_stale_s'),
    ),
}

# Counts, not distances -- rclpy rejects a float for an integer parameter.
INT_PARAMS = ('road_h_min', 'road_h_max', 'paint_s_max', 'paint_v_min',
         'mark_h_min', 'mark_h_max', 'mark_s_min', 'max_span', 'samples')


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
                if n in INT_PARAMS and isinstance(value, float):
                    value = int(value)
                params[n] = value
        remaps = []
        if node_name == 'planner_node':
            remaps = [('image_raw',
                       LaunchConfiguration('image_topic').perform(context))]
        nodes.append(Node(
            package=pkg,
            executable=executable,
            name=node_name,
            output='screen',
            parameters=[params] if params else [],
            remappings=remaps,
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

    traffic_light = Node(
        package='physicar_vision',
        executable='traffic_light_node',
        name='traffic_light_node',
        output='screen',
        remappings=[('image_raw', LaunchConfiguration('image_topic'))],
    )

    return LaunchDescription(args + [traffic_light, OpaqueFunction(function=_build)])
