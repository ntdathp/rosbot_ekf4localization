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
        Node(
            package='imu_filter_package',
            executable='imu',
            name='imu',
            output='screen',
            parameters=[{'new_frame_id': 'base_imu'}] 
        ),
        Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_imu'],
           output='screen'
       )
    ])
