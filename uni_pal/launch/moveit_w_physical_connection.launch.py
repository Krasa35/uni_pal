
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_param_builder import get_path
    
def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("controller_spawner_timeout",default_value="10"),
        DeclareLaunchArgument("tf_prefix",default_value="")
    ]
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="uni_pal_description")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .trajectory_execution()
        .to_moveit_configs()
    )
    moveit_config.trajectory_execution["moveit_simple_controller_manager"]["scaled_joint_trajectory_controller"]["default"] = False
    moveit_config.trajectory_execution["moveit_simple_controller_manager"]["joint_trajectory_controller"]["default"] = True
    rviz_config_file = get_path('uni_pal_description', 'rviz/run_move_group.rviz')
    initial_joint_controllers = get_path("ur_robot_driver", "config/ur_controllers.yaml")
    update_rate_config_file = get_path("ur_robot_driver", "config/ur10_update_rate.yaml")

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

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

    # Spawn controllers
    def controller_spawner(name, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags,
        )

    controller_spawner_names = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "joint_trajectory_controller"
    ]
    controller_spawner_inactive_names = ["forward_position_controller"]

    controller_spawners = [controller_spawner(name) for name in controller_spawner_names] + [
        controller_spawner(name, active=False) for name in controller_spawner_inactive_names
    ]

    nodes_to_start = [
        ur_control_node,
        robot_state_publisher_node,
        rviz_node,
        run_move_group_node
    ] + controller_spawners

    return LaunchDescription(declared_arguments + nodes_to_start)      