from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/arduino')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200')

    node = Node(
        package='physicar_driver',
        executable='driver_node',
        name='physicar_driver_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
        }],
    )

    return LaunchDescription([port_arg, baud_arg, node])
