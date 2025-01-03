
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.substitutions import Xacro
from launch_param_builder import get_path

def generate_launch_description():
    
    urdf_file = get_path("uni_pal_description", "urdf/robot.urdf.xacro")
    rviz_config_file = get_path("uni_pal_description", "rviz/view_robot.rviz")

    robot_description = {"robot_description": ParameterValue(
        Xacro(urdf_file), value_type=str
        )}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
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

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)
    