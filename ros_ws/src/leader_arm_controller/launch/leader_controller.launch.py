from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    baud_rate = LaunchConfiguration('baud_rate', default='1000000')
    publish_rate = LaunchConfiguration('publish_rate', default='100.0')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('publish_rate', default_value='100.0'),

        Node(
            package='leader_arm_controller',
            executable='leader_controller_node',
            name='leader_arm_controller',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate': baud_rate,
                'publish_rate': publish_rate,
            }],
            output='screen',
            emulate_tty=True,
            respawn=True,
        ),
    ])
