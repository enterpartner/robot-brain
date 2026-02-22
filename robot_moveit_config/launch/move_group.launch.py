"""MoveIt2 move_group launch for dual-arm robot."""
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load a yaml file from a package share directory."""
    full_path = os.path.join(
        get_package_share_directory(package_name), file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config_dir = get_package_share_directory('robot_moveit_config')
    bringup_dir = get_package_share_directory('robot_bringup')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2 with MoveIt plugin')

    use_rviz = LaunchConfiguration('use_rviz')

    # Load URDF
    urdf_file = os.path.join(bringup_dir, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Load SRDF
    srdf_file = os.path.join(moveit_config_dir, 'config', 'robot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # Load configs
    kinematics_yaml = load_yaml('robot_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('robot_moveit_config', 'config/joint_limits.yaml')
    planning_pipeline_yaml = load_yaml('robot_moveit_config', 'config/planning_pipeline.yaml')
    ompl_planning_yaml = load_yaml('robot_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml('robot_moveit_config', 'config/moveit_controllers.yaml')

    # MoveIt2 parameters
    moveit_config = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
        'planning_pipelines': planning_pipeline_yaml['planning_pipelines'],
        'ompl': ompl_planning_yaml,
    }
    moveit_config.update(moveit_controllers_yaml)

    # move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config,
            {'use_sim_time': False},
            {'publish_robot_description_semantic': True},
        ],
    )

    # robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ros2_control controller_manager
    ros2_controllers_yaml = os.path.join(
        moveit_config_dir, 'config', 'ros2_controllers.yaml')
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            ros2_controllers_yaml,
        ],
        output='screen',
    )

    # Spawn controllers
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    spawn_left_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
    )
    spawn_right_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager'],
    )
    spawn_left_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_gripper_controller', '--controller-manager', '/controller_manager'],
    )
    spawn_right_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_gripper_controller', '--controller-manager', '/controller_manager'],
    )
    spawn_head = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_controller', '--controller-manager', '/controller_manager'],
    )

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(moveit_config_dir, 'config', 'moveit.rviz')],
        parameters=[moveit_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_rviz_arg,
        rsp_node,
        controller_manager_node,
        spawn_jsb,
        spawn_left_arm,
        spawn_right_arm,
        spawn_left_gripper,
        spawn_right_gripper,
        spawn_head,
        move_group_node,
        rviz_node,
    ])
