
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder import get_path

def generate_launch_description():
    # Configure robot_description
    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="uni_pal_description")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    rviz_config_file = get_path('uni_pal_description', 'rviz/run_move_group.rviz')

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,           
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description]
    )
    
    robot_driver_node = Node(
        package='ur_robot_driver',
        executable='ur_ros2_control_node',
        output='screen',
    )

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
            robot_driver_node,
        ]
    )