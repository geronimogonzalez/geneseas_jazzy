from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import datetime

def generate_launch_description():
    timestamp = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    rosbag_name = f'rosbags/square_wave_test_{timestamp}'

    return LaunchDescription([
        # Declaración de argumentos
        DeclareLaunchArgument('valor_alto', default_value='80.0'),
        DeclareLaunchArgument('valor_bajo', default_value='-20.0'),
        DeclareLaunchArgument('periodo', default_value='10.0'),
        DeclareLaunchArgument('alpha', default_value='1.0'),

        # Nodo publicador
        Node(
            package='my_publisher_pkg',
            executable='square_wave_publisher',
            name='square_wave_node',
            parameters=[{
                'valor_alto': LaunchConfiguration('valor_alto'),
                'valor_bajo': LaunchConfiguration('valor_bajo'),
                'periodo': LaunchConfiguration('periodo'),
                'alpha': LaunchConfiguration('alpha')
            }],
            output='screen'
        ),

        # Grabación de rosbag
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

