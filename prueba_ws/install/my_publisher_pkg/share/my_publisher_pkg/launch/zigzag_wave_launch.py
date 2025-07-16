from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import datetime

def generate_launch_description():
    # Generar timestamp para el nombre de la carpeta
    timestamp = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    rosbag_name = f'rosbags/zigzag_test_{timestamp}'

    # Declarar configuraciones de lanzamiento
    valor_alto = LaunchConfiguration('valor_alto')
    valor_bajo = LaunchConfiguration('valor_bajo')
    periodo = LaunchConfiguration('periodo')
    alpha = LaunchConfiguration('alpha')

    return LaunchDescription([
        # Argumentos que se pueden pasar por línea de comandos
        DeclareLaunchArgument('valor_alto', default_value='0.80'),
        DeclareLaunchArgument('valor_bajo', default_value='-0.20'),
        DeclareLaunchArgument('periodo', default_value='10.0'),
        DeclareLaunchArgument('alpha', default_value='1.0'),

        Node(
            package='my_publisher_pkg',
            executable='zigzag_wave_publisher',
            name='zigzag_wave_node',
            parameters=[{
                'valor_alto': valor_alto,
                'valor_bajo': valor_bajo,
                'periodo': periodo,
                'alpha': alpha
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
