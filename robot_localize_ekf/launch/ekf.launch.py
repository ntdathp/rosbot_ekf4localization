import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='robot_localize_ekf').find('robot_localize_ekf')
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[robot_localization_file_path, {'use_sim_time': False}],
        ),
    ])