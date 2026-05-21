from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    leader_port = LaunchConfiguration('leader_port', default='/dev/ttyUSB0')
    follower_port = LaunchConfiguration('follower_port', default='/dev/ttyUSB1')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mode = LaunchConfiguration('mode', default='FOLLOWING')

    # URDF path
    urdf_path = os.path.join(
        get_package_share_directory('leader_follower_description'),
        'urdf',
        'leader_follower.urdf.xacro'
    )

    return LaunchDescription([
        DeclareLaunchArgument('leader_port', default_value='/dev/ttyUSB0',
                              description='Leader STM32 serial port'),
        DeclareLaunchArgument('follower_port', default_value='/dev/ttyUSB1',
                              description='Follower STM32 serial port'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time'),
        DeclareLaunchArgument('mode', default_value='FOLLOWING',
                              description='System mode: FOLLOWING, SIMULATION, INFERENCE, CALIBRATION'),

        # Leader Arm Controller
        Node(
            package='leader_arm_controller',
            executable='leader_controller_node',
            name='leader_arm_controller',
            parameters=[{
                'serial_port': leader_port,
                'baud_rate': 1000000,
                'publish_rate': 100.0,
            }],
            output='screen',
            emulate_tty=True,
            respawn=True,
        ),

        # Follower Arm Controller
        Node(
            package='follower_arm_controller',
            executable='follower_controller_node',
            name='follower_arm_controller',
            parameters=[{
                'serial_port': follower_port,
                'baud_rate': 1000000,
                'auto_follow': True,
                'joint_scale': 1.0,
                'mode': mode,
            }],
            output='screen',
            emulate_tty=True,
            respawn=True,
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_leader',
            parameters=[{
                'robot_description': open(
                    os.path.join(
                        get_package_share_directory('leader_follower_description'),
                        'urdf', 'leader_arm.urdf.xacro'
                    )
                ).read(),
                'use_sim_time': use_sim_time,
            }],
            remappings=[('joint_states', '/leader_joint_states')],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('leader_follower_description'),
                'config', 'leader_follower.rviz'
            )],
            condition=LaunchConfiguration('use_gui', default='true'),
        ),

        LogInfo(msg=['Leader-Follower system started in ', mode, ' mode']),
        LogInfo(msg=['Leader port: ', leader_port, ', Follower port: ', follower_port]),
    ])
