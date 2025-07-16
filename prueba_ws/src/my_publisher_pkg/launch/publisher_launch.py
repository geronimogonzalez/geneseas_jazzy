from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import datetime

def generate_launch_description():
    timestamp = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    rosbag_name = f'rosbags/derecho_test_{timestamp}'

    mode = LaunchConfiguration('mode')
    thrust = LaunchConfiguration('thrust')
    alpha = LaunchConfiguration('alpha')  # corregido

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='derecho'),
        DeclareLaunchArgument('thrust', default_value='0.0'),
        DeclareLaunchArgument('alpha', default_value='1.0'),

        Node(
            package='my_publisher_pkg',
            executable='publisher_node',
            name='publisher_node',
            parameters=[{
                'mode': mode,
                'thrust': thrust,
                'alpha': alpha  # agregado
            }],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', rosbag_name,
                '--all',
                '--storage', 'mcap'
            ],
            output='screen'
        )
    ])

