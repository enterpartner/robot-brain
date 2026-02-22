"""Demo launch: Robot model + Joint GUI + ZED camera + Brain + RViz."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    bringup_dir = get_package_share_directory('robot_bringup')
    brain_dir = get_package_share_directory('robot_brain')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')

    urdf_file = os.path.join(bringup_dir, 'urdf', 'robot.urdf')
    rviz_config = os.path.join(bringup_dir, 'config', 'demo.rviz')
    zed_override = os.path.join(bringup_dir, 'config', 'zed_override.yaml')
    brain_params = os.path.join(brain_dir, 'config', 'brain_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    launch_zed_arg = DeclareLaunchArgument(
        'launch_zed', default_value='true',
        description='Launch ZED camera')
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true',
        description='Launch RViz2')
    launch_brain_arg = DeclareLaunchArgument(
        'launch_brain', default_value='true',
        description='Launch Brain node')

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Motion Executor (replaces joint_state_publisher_gui)
    # Publishes /joint_states and accepts commands from Brain
    motion_executor = Node(
        package='robot_brain',
        executable='motion_executor',
        output='screen',
    )

    # ZED Camera (delayed)
    zed_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
                ),
                launch_arguments={
                    'camera_model': 'zedxm',
                    'ros_params_override_path': zed_override,
                    'publish_tf': 'false',
                    'publish_urdf': 'true',
                }.items(),
            ),
        ],
    )

    # RViz2 (delayed - wait for ZED)
    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
            ),
        ],
    )

    # Brain Node (delayed - wait for ZED)
    brain_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='robot_brain',
                executable='brain_node',
                parameters=[brain_params],
                output='screen',
            ),
        ],
    )

    # rqt_image_view — RGB (delayed)
    rqt_rgb = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                arguments=['/zed/zed_node/rgb/color/rect/image'],
                output='screen',
            ),
        ],
    )

    # rqt_image_view — Depth (delayed)
    rqt_depth = TimerAction(
        period=17.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                arguments=['/zed/zed_node/depth/depth_registered'],
                output='screen',
            ),
        ],
    )

    # rqt_graph — Node topology (delayed)
    rqt_graph = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='rqt_graph',
                executable='rqt_graph',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        launch_zed_arg,
        launch_rviz_arg,
        launch_brain_arg,
        robot_state_pub,
        motion_executor,
        zed_launch,
        rviz_node,
        brain_node,
        rqt_rgb,
        rqt_depth,
        rqt_graph,
    ])
