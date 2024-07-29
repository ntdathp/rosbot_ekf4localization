from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            parameters=[
                {'use_mag': False}
            ],
            remappings=[
                ('/imu/data_raw', '/camera/camera/imu')
            ]
        ),
    ])
