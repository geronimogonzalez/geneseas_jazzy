from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_pub',
            arguments=['0.8', '0', '0', '0', '0', '0', 'base_footprint', 'bno055'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'gps'],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=['/home/robot/dev_ws/src/my_robot_localization/config/ekf.yaml'],
            output='screen',
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=['/home/robot/dev_ws/src/my_robot_localization/config/navsat_transform.yaml'],
            remappings=[
                ('/gps/fix', '/nmea_navsat_driver/fix'),
                ('/imu', '/bno055/bno055/imu'),
                ('/odometry/gps', '/odometry/gps')
            ],
            output='screen',
        ),
    ])

