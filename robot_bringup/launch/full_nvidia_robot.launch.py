"""Teljes robot bringup - NVIDIA Isaac stack + ZED X Mini."""
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

    bringup_dir = get_package_share_directory('robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')

    # === Launch Arguments ===
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Launch Nav2 navigation stack')
    use_cuvslam_arg = DeclareLaunchArgument(
        'use_cuvslam', default_value='true',
        description='Launch cuVSLAM visual SLAM')
    use_nvblox_arg = DeclareLaunchArgument(
        'use_nvblox', default_value='true',
        description='Launch nvblox 3D reconstruction')
    camera_model_arg = DeclareLaunchArgument(
        'camera_model', default_value='zedxm',
        description='ZED camera model')

    use_nav2 = LaunchConfiguration('use_nav2')
    use_cuvslam = LaunchConfiguration('use_cuvslam')
    use_nvblox = LaunchConfiguration('use_nvblox')
    camera_model = LaunchConfiguration('camera_model')

    # ============================================================
    # 1. ROBOT LEIRAS (URDF -> TF fa)
    # ============================================================
    urdf_file = os.path.join(bringup_dir, 'urdf', 'robot.urdf')
    robot_description_content = ''
    if os.path.exists(urdf_file):
        with open(urdf_file, 'r') as f:
            robot_description_content = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
        }],
        condition=IfCondition(str(bool(robot_description_content))),
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # ============================================================
    # 2. ZED X MINI (kamera driver)
    # ============================================================
    # A ZED wrapper v5.1 launch argumentumai korlatozottak.
    # Config parametreket (depth_mode, resolution, stb.) a
    # ros_params_override_path YAML fajlon keresztul allitjuk.
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

    # ============================================================
    # 3. cuVSLAM (GPU-gyorsitott SLAM + odometria)
    # ============================================================
    # ZED wrapper v5.1 topic nevek:
    #   Left gray:  /zed/zed_node/left_gray/image_rect_gray
    #   Right gray: /zed/zed_node/right_gray/image_rect_gray
    #   IMU:        /zed/zed_node/imu/data
    cuvslam_node = TimerAction(period=5.0, actions=[
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            condition=IfCondition(use_cuvslam),
            remappings=[
                ('visual_slam/image_0', '/zed/zed_node/left/gray/rect/image'),
                ('visual_slam/camera_info_0', '/zed/zed_node/left/gray/rect/camera_info'),
                ('visual_slam/image_1', '/zed/zed_node/right/gray/rect/image'),
                ('visual_slam/camera_info_1', '/zed/zed_node/right/gray/rect/camera_info'),
                ('visual_slam/imu', '/zed/zed_node/imu/data'),
            ],
            parameters=[{
                # Frame-ek
                'base_frame': 'base_link',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'input_base_frame': 'zed_camera_link',
                'input_left_camera_frame': 'zed_left_camera_frame_optical',
                'input_right_camera_frame': 'zed_right_camera_frame_optical',
                'input_imu_frame': 'zed_camera_center',

                # TF publikalas
                'publish_odom_to_base_tf': True,
                'publish_map_to_odom_tf': True,
                'invert_odom_to_base_tf': False,

                # IMU fusion (ZED X Mini szenzorok)
                'enable_imu_fusion': True,
                'gyro_noise_density': 0.000244,
                'gyro_random_walk': 0.000019393,
                'accel_noise_density': 0.001862,
                'accel_random_walk': 0.003,

                # Teljesitmeny
                'enable_slam_visualization': False,
                'enable_landmarks_view': False,
                'enable_observations_view': False,
                'num_cameras': 2,
                'min_num_images': 2,

                # Mapping
                'enable_localization_n_mapping': True,
            }],
            output='screen'
        ),
    ])

    # ============================================================
    # 4. NVBLOX (3D rekonstrukcio -> costmap)
    # ============================================================
    # ZED wrapper v5.1 topic nevek:
    #   Depth:  /zed/zed_node/depth/depth_registered
    #   Color:  /zed/zed_node/rgb/color/rect/image
    nvblox_node = TimerAction(period=8.0, actions=[
        Node(
            package='nvblox_ros',
            executable='nvblox_node',
            name='nvblox_node',
            condition=IfCondition(use_nvblox),
            remappings=[
                ('depth/image', '/zed/zed_node/depth/depth_registered'),
                ('depth/camera_info', '/zed/zed_node/depth/depth_registered/camera_info'),
                ('color/image', '/zed/zed_node/rgb/color/rect/image'),
                ('color/camera_info', '/zed/zed_node/rgb/color/rect/image/camera_info'),
                ('pose', '/visual_slam/tracking/odometry'),
            ],
            parameters=[
                os.path.join(bringup_dir, 'config', 'nvblox_params.yaml')
            ],
        ),
    ])

    # ============================================================
    # 5. NAV2 (navigacio - nvblox costmap-pel)
    # ============================================================
    nav2_launch = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(
                    bringup_dir, 'config', 'nav2_params.yaml'),
            }.items(),
            condition=IfCondition(use_nav2),
        ),
    ])

    # ============================================================
    # 6. SAFETY NODE (e-stop, velocity limits, human proximity)
    # ============================================================
    safety_dir = get_package_share_directory('robot_safety')
    safety_node = Node(
        package='robot_safety',
        executable='safety_node',
        name='safety_node',
        parameters=[
            os.path.join(safety_dir, 'config', 'safety_params.yaml')
        ],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        use_nav2_arg,
        use_cuvslam_arg,
        use_nvblox_arg,
        camera_model_arg,
        # Nodes (indulasi sorrend: safety -> zed -> cuvslam -> nvblox -> nav2)
        robot_state_pub,
        joint_state_pub,
        safety_node,
        zed_launch,
        cuvslam_node,
        nvblox_node,
        nav2_launch,
    ])
