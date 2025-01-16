import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory for cropbox_filter
    config_directory = get_package_share_directory('cropbox_filter')
    config_file = os.path.join(config_directory, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='cropbox_filter',
            executable='cropbox_filter_node',
            name='cropbox_filter_node',
            output='screen',
            parameters=[config_file]
        )
    ])

