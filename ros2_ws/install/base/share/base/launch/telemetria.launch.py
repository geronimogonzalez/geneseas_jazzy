from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno055',
            namespace='bno055',
            executable='bno055',
            name='imu'
        ),
        Node(
            package='nmea_navsat_driver',
            namespace='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps'
        ),
        Node(
            package='serial_robot',
            namespace='serial_robot',
            executable='serial_communication',
            name='arduino_mega'
        )
    ])