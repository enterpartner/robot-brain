"""Manipulacio: RT-DETR + FoundationPose + cuMotion + MoveIt2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.5',
        description='RT-DETR detection confidence threshold')

    confidence = LaunchConfiguration('confidence_threshold')

    # ============================================================
    # RT-DETR objektum detektalas
    # ============================================================
    rtdetr_node = Node(
        package='isaac_ros_rtdetr',
        executable='rtdetr_node',
        name='rtdetr',
        remappings=[
            ('image', '/zed/zed_node/rgb/image_rect_color'),
            ('camera_info', '/zed/zed_node/rgb/camera_info'),
        ],
        parameters=[{
            'confidence_threshold': confidence,
        }],
    )

    # ============================================================
    # FoundationPose 6D pozbecslesi
    # ============================================================
    foundationpose_node = TimerAction(period=2.0, actions=[
        Node(
            package='isaac_ros_foundationpose',
            executable='foundationpose_node',
            name='foundationpose',
            remappings=[
                ('color/image_raw', '/zed/zed_node/rgb/image_rect_color'),
                ('depth/image_raw', '/zed/zed_node/depth/depth_registered'),
                ('color/camera_info', '/zed/zed_node/rgb/camera_info'),
                ('detections', '/rtdetr/detections'),
            ],
            parameters=[{
                'optical_frame_name': 'zed_left_camera_optical_frame',
                # Targy mesh/texture - MODOSITSD!
                # 'mesh_file_path': '/path/to/object.obj',
                # 'texture_path': '/path/to/object_texture.png',
            }],
        ),
    ])

    # ============================================================
    # cuMotion robot szegmentalas (robot test kiszurese depth-bol)
    # ============================================================
    # Uncomment ha kell:
    # cumotion_seg_node = Node(
    #     package='isaac_ros_cumotion',
    #     executable='cumotion_robot_segmentation_node',
    #     name='cumotion_segmentation',
    #     remappings=[
    #         ('depth_image', '/zed/zed_node/depth/depth_registered'),
    #         ('depth_camera_info', '/zed/zed_node/depth/camera_info'),
    #     ],
    #     parameters=[{
    #         'joint_states_topic': '/joint_states',
    #     }],
    # )

    # ============================================================
    # MoveIt2 + cuMotion
    # ============================================================
    # Uncomment es modositsd a robot_moveit_config csomag alapjan:
    # moveit_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         'robot_moveit_config/launch/move_group.launch.py'),
    # )

    return LaunchDescription([
        confidence_arg,
        rtdetr_node,
        foundationpose_node,
    ])
