from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import datetime

def generate_launch_description():
    timestamp = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    rosbag_name = f'rosbags/curva_test_{timestamp}'

    mode = LaunchConfiguration('mode')
    left_thrust = LaunchConfiguration('left_thrust')
    right_thrust = LaunchConfiguration('right_thrust')
    alpha = LaunchConfiguration('alpha')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='curva'),
        DeclareLaunchArgument('left_thrust', default_value='0.1'),
        DeclareLaunchArgument('right_thrust', default_value='0.2'),
        DeclareLaunchArgument('alpha', default_value='1.0'),

        Node(
            package='my_publisher_pkg',
            executable='publisher_node',
            name='publisher_node',
            parameters=[{
                'mode': mode,
                'left_thrust': left_thrust,
                'right_thrust': right_thrust,
                'alpha': alpha,
            }],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', rosbag_name,
                '--all',
                '--storage', 'mcap',
            ],
            output='screen'
        )
    ])

