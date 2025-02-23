import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('selfhelp'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='selfhelp',
            executable='selfhelp_node',
            name='selfhelp',
            parameters=[config_path],
            output='screen'
        )
    ])