

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="uni_pal_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for         multi-robot setup. If changed, also joint names in the controllers' configuration         have to be updated.",
        )
    )
    
    description_package = LaunchConfiguration("description_package")
    ur_package = LaunchConfiguration("ur_package")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    tf_prefix = LaunchConfiguration("tf_prefix")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_package), "config", "ur10", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(ur_package), "config", "ur10", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_package), "config", "ur10", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_package), "config", "ur10", "visual_parameters.yaml"]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("uni_pal_description"), "urdf", "robot.urdf.xacro"]),
            " ",
            "no_robot:=false",
            " ",
            "simulation_robot:=true",
            " ",
            "simulation_scene:=true",
            " ",
            "robot_ip:=",
            "192.168.56.101",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "name:=",
            "ur10",
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )
    initial_joint_controllers = PathJoinSubstitution( #ur_robot_driver - ur_controllers.yaml
        [FindPackageShare("ur_robot_driver"), "config", "ur_controllers.yaml"]
    )
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ur_robot_driver"),
            "config",
            "ur10_update_rate.yaml",
        ]
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
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
    ]
    controller_spawner_inactive_names = ["forward_position_controller"]

    controller_spawners = [controller_spawner(name) for name in controller_spawner_names] + [
        controller_spawner(name, active=False) for name in controller_spawner_inactive_names
    ]
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        ur_control_node,
        robot_state_publisher_node,
        rviz_node,
    ] + controller_spawners

    return LaunchDescription(declared_arguments + nodes_to_start)
    