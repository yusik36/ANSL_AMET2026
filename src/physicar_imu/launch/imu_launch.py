from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/imu')
    baud_arg = DeclareLaunchArgument('baud', default_value='921600')
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='imu_link')

    node = Node(
        package='physicar_imu',
        executable='hfi_imu_node',
        name='hfi_imu_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
    )

    return LaunchDescription([port_arg, baud_arg, frame_id_arg, node])
