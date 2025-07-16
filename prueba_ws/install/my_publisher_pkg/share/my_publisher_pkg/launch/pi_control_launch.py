from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import datetime

def generate_launch_description():
    # Generar timestamp para el nombre de la carpeta
    timestamp = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    rosbag_name = f'rosbags/PI_test_{timestamp}'

    return LaunchDescription([
        DeclareLaunchArgument('kp', default_value='60.0'),
        DeclareLaunchArgument('ki', default_value='30.0'),
        DeclareLaunchArgument('kd', default_value='0.0'),
        DeclareLaunchArgument('right_thrust', default_value='0.50'),
        DeclareLaunchArgument('output_limits', default_value='[-1.0, 1.0]'),
        DeclareLaunchArgument('sample_time', default_value='0.1'),
        DeclareLaunchArgument('realimentacion', default_value='1.0'),
        DeclareLaunchArgument('offset_motor', default_value='0.50'),

        Node(
            package='my_publisher_pkg',
            executable='pi_controller',
            name='pi_controller',
            parameters=[{
                'kp': LaunchConfiguration('kp'),
                'ki': LaunchConfiguration('ki'),
                'kd': LaunchConfiguration('kd'),
                'right_thrust': LaunchConfiguration('right_thrust'),
                'output_limits': LaunchConfiguration('output_limits'),
                'sample_time': LaunchConfiguration('sample_time'),
                'realimentacion': LaunchConfiguration('realimentacion'),
                'offset_motor': LaunchConfiguration('offset_motor')
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
