"""Safety node launch."""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    safety_dir = get_package_share_directory('robot_safety')
    params_file = os.path.join(safety_dir, 'config', 'safety_params.yaml')

    return LaunchDescription([
        Node(
            package='robot_safety',
            executable='safety_node',
            name='safety_node',
            parameters=[params_file],
            output='screen',
        ),
    ])
