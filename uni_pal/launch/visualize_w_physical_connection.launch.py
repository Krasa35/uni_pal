
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from moveit_configs_utils.substitutions import Xacro
from launch_param_builder import get_path

def generate_launch_description():
    declared_arguments=[DeclareLaunchArgument("tf_prefix", default_value="")]

    urdf_file = get_path("uni_pal_description", "urdf/robot.urdf.xacro")
    rviz_config_file = get_path("uni_pal_description", "rviz/view_robot.rviz")
    initial_joint_controllers = get_path("ur_robot_driver", "config/ur_controllers.yaml")
    update_rate_config_file = get_path("ur_robot_driver", "config/ur10_update_rate.yaml")

    robot_description = {"robot_description": ParameterValue(
        Xacro(urdf_file), value_type=str
        )}
    
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

    joint_state_broadcaster_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "10",
            ]
    )
    # "joint_state_broadcaster",
    # "io_and_status_controller",
    # "speed_scaling_state_broadcaster",
    # "force_torque_sensor_broadcaster",
    # "forward_position_controller"]

    nodes_to_start = [
        ur_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
    