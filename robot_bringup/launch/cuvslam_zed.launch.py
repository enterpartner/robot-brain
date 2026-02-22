"""cuVSLAM launch ZED X Mini-vel - robot fejben."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_dir = get_package_share_directory('robot_bringup')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')

    # Launch arguments
    camera_model_arg = DeclareLaunchArgument(
        'camera_model', default_value='zedxm',
        description='ZED camera model (zedxm for ZED X Mini)')

    camera_model = LaunchConfiguration('camera_model')

    # === ZED X Mini driver ===
    zed_override_path = os.path.join(
        bringup_dir, 'config', 'zed_override.yaml')

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': camera_model,
            'camera_name': 'zed',
            'publish_urdf': 'true',
            'publish_tf': 'false',
            'ros_params_override_path': zed_override_path,
        }.items()
    )

    # === cuVSLAM ===
    cuvslam_node = TimerAction(period=5.0, actions=[
        Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='visual_slam',
            remappings=[
                ('visual_slam/image_0', '/zed/zed_node/left/gray/rect/image'),
                ('visual_slam/camera_info_0', '/zed/zed_node/left/gray/rect/camera_info'),
                ('visual_slam/image_1', '/zed/zed_node/right/gray/rect/image'),
                ('visual_slam/camera_info_1', '/zed/zed_node/right/gray/rect/camera_info'),
                ('visual_slam/imu', '/zed/zed_node/imu/data'),
            ],
            parameters=[{
                'base_frame': 'base_link',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'input_base_frame': 'zed_camera_link',
                'input_left_camera_frame': 'zed_left_camera_frame_optical',
                'input_right_camera_frame': 'zed_right_camera_frame_optical',
                'input_imu_frame': 'zed_camera_center',

                'publish_odom_to_base_tf': True,
                'publish_map_to_odom_tf': True,
                'invert_odom_to_base_tf': False,

                'enable_imu_fusion': True,
                'gyro_noise_density': 0.000244,
                'gyro_random_walk': 0.000019393,
                'accel_noise_density': 0.001862,
                'accel_random_walk': 0.003,

                'enable_slam_visualization': False,
                'enable_landmarks_view': False,
                'enable_observations_view': False,
                'num_cameras': 2,
                'min_num_images': 2,

                'enable_localization_n_mapping': True,
            }],
            output='screen'
        ),
    ])

    return LaunchDescription([
        camera_model_arg,
        zed_launch,
        cuvslam_node,
    ])
