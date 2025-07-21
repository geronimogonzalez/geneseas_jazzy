from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import datetime

def generate_launch_description():
    timestamp = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    rosbag_name = f'rosbags/prbs_test_{timestamp}'

    amplitudes = LaunchConfiguration('amplitudes')
    switch_interval = LaunchConfiguration('switch_interval')

    return LaunchDescription([
        DeclareLaunchArgument(
            'amplitudes',
            default_value='[0.4,0.5,0.6]',
            description='Vector de amplitudes para la senal PRBS'
        ),
        DeclareLaunchArgument(
            'switch_interval',
            default_value='4.0',
            description='Intervalo de cambio de senal en segundos'
        ),

        Node(
            package='my_publisher_pkg',
            executable='prbs_publisher',
            name='prbs_node',
            parameters=[{
                'amplitudes': amplitudes,
                'switch_interval': switch_interval
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
