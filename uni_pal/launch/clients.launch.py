from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.substitutions import Xacro
from launch_param_builder import get_path

def generate_launch_description():
    
    read_json = Node(
        package="uni_pal",
        executable="jsonread.py",
    )    
    robot_republisher = Node(
        package="uni_pal",
        executable="robot_republisher",
    )
    scene_client = Node(
        package="uni_pal",
        executable="scene_client",
    )
    robot_client = Node(
        package="uni_pal",
        executable="robot_client",
    )

    nodes_to_start = [
        read_json,
        robot_republisher,
        scene_client,
        robot_client,
    ]

    return LaunchDescription(nodes_to_start)
    