"""Robot Brain launch — Vision + LLM task planner."""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    brain_dir = get_package_share_directory('robot_brain')
    bringup_dir = get_package_share_directory('robot_bringup')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')

    # Launch arguments
    launch_zed_arg = DeclareLaunchArgument(
        'launch_zed', default_value='false',
        description='Whether to launch ZED camera driver')

    camera_model_arg = DeclareLaunchArgument(
        'camera_model', default_value='zedxm',
        description='ZED camera model')

    launch_zed = LaunchConfiguration('launch_zed')
    camera_model = LaunchConfiguration('camera_model')

    # ZED camera (optional)
    zed_override_path = os.path.join(
        bringup_dir, 'config', 'zed_override.yaml')

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': camera_model,
            'ros_params_override_path': zed_override_path,
        }.items(),
        condition=IfCondition(launch_zed),
    )

    # Brain node (delay 3s if ZED is launching)
    brain_params = os.path.join(brain_dir, 'config', 'brain_params.yaml')

    brain_node = Node(
        package='robot_brain',
        executable='brain_node',
        name='brain_node',
        parameters=[brain_params],
        output='screen',
    )

    # Delay brain if ZED is being launched (give camera time to start)
    brain_delayed = TimerAction(
        period=3.0,
        actions=[brain_node],
    )

    return LaunchDescription([
        launch_zed_arg,
        camera_model_arg,
        zed_launch,
        brain_delayed,
    ])
