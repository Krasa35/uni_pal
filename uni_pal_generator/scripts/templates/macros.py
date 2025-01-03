box_template_ = """<box size="{{ size }}"/>"""
#mesh_template_ = """<mesh filename="package://uni_pal_description/meshes/{{ mesh }}" scale="{{ scale }}"/>"""
mesh_template_ = """<mesh filename="file://$(find uni_pal_description)/meshes/{{ mesh }}" scale="{{ scale }}"/>"""
origin_template_ = """<origin xyz="{{ origin_xyz }}" rpy="{{ origin_rpy }}"/>"""
xacro_include_template = """\t<xacro:include filename="{{ file_path }}"/> """
urdf_template = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="{{ robot_name }}">
  <xacro:arg name="name" default="{{ robot_name }}"/>      
  <link name="world" />

</robot>
"""

element_template = """<?xml version="1.0"?>
<robot name="{{ name }}">
  <link name="{{ name }}">
      <visual>
        {{ origin }}
        <geometry>
        {{ geometry }}
        </geometry>
        <material name="Gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
      </visual>
      <collision>
        {{ origin }}
        <geometry>
        {{ geometry }}
        </geometry>
      </collision>
  </link>
  <joint name="{{ parent }} - {{ name }}" type="fixed">
    <origin xyz="{{ position }}" rpy="{{ orientation }}" />
    <parent link="{{ parent }}" />
    <child link="{{ name }}" />
  </joint>
</robot>"""

techman_robots = {
    'urdf_macro_include': """$(find tm_description)/xacro/macro.{{ model }}-nominal.urdf.xacro""",
    'urdf_macro': """
  <xacro:arg name="ns" default="" />
  <xacro:arg name="prefix" default="" />
  <xacro:arg name="color" default="none" />
  <xacro:arg name="trans_hw_iface" default="hardware_interface/PositionJointInterface" />

  <xacro:include filename="$(find tm_description)/xacro/macro.gazebo.xacro" />
  <xacro:include filename="$(find tm_description)/xacro/macro.transmission.xacro" />
  <xacro:include filename="$(find tm_description)/xacro/macro.materials.xacro" />
  <xacro:include filename="$(find tm_description)/xacro/macro.{{ model }}-nominal.urdf.xacro" />

  <!--  -->
  <xacro:tmr_gazebo ns="$(arg ns)" prefix="$(arg prefix)" />
  <xacro:tmr_transmission prefix="$(arg prefix)" hw_iface="$(arg trans_hw_iface)" />
  <xacro:tmr_materials/>


  <!-- Arm -->
  <xacro:property name="color" value="$(arg color)"/>
  <xacro:if value="${color == 'none'}">
    <xacro:{{ model }} ns="$(arg ns)" prefix="$(arg prefix)" />
  </xacro:if>


  <!-- Arm.color.stl -->
  <xacro:unless value="${color == 'none'}">
    <xacro:{{ model }} ns="$(arg ns)" prefix="$(arg prefix)" color="${color}" format="stl" />
  </xacro:unless>

  <joint name="base - {{ parent }}" type="fixed">
   <parent link="{{ parent }}" />
   <child link="base" />
   <origin xyz="{{ origin_xyz }}" rpy="{{ origin_rpy }}" />
  </joint>

  <xacro:arg name="initial_positions_file" default="$(find uni_pal_description)/config/initial_positions.yaml" />
  <xacro:include filename="$(find uni_pal_description)/config/tm12.ros2_control.xacro" />
  <xacro:tm12_ros2_control name="FakeSystem" initial_positions_file="$(arg initial_positions_file)"/>

""",
    'links': {
        'base': 'link_0',
        'base_2': 'link_0',
        'link_1': 'link_1',
        'link_2': 'link_2',
        'link_3': 'link_3',
        'link_4': 'link_4',
        'link_5': 'link_5',
        'link_6': 'link_6',
    },
    'joints': {
        'joint_1': 'joint_1',
        'joint_2': 'joint_2',
        'joint_3': 'joint_3',
        'joint_4': 'joint_4',
        'joint_5': 'joint_5',
        'joint_6': 'joint_6',
    }
}

universal_robots = {
    'urdf_macro_include': """$(find ur_description)/urdf/ur_macro.xacro""",
    'urdf_macro': """
  <!-- import main macro -->
  <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
  <xacro:arg name="ur_type" default="{{ model }}"/>
  <!-- parameters -->
  <xacro:arg name="tf_prefix" default="" />
  <xacro:arg name="joint_limit_params" default="$(find ur_description)/config/$(arg ur_type)/joint_limits.yaml"/>
  <xacro:arg name="kinematics_params" default="$(find ur_description)/config/$(arg ur_type)/default_kinematics.yaml"/>
  <xacro:arg name="physical_params" default="$(find ur_description)/config/$(arg ur_type)/physical_parameters.yaml"/>
  <xacro:arg name="visual_params" default="$(find ur_description)/config/$(arg ur_type)/visual_parameters.yaml"/>
  <xacro:arg name="transmission_hw_interface" default=""/>
  <xacro:arg name="safety_limits" default="false"/>
  <xacro:arg name="safety_pos_margin" default="0.15"/>
  <xacro:arg name="safety_k_position" default="20"/>
  <!-- ros2_control related parameters -->
  <xacro:arg name="headless_mode" default="false" />
  <xacro:arg name="robot_ip" default="{{ robot_ip }}" />
  <xacro:arg name="script_filename" default="$(find ur_client_library)/resources/external_control.urscript"/>
  <xacro:arg name="output_recipe_filename" default="$(find ur_robot_driver)/resources/rtde_output_recipe.txt"/>
  <xacro:arg name="input_recipe_filename" default="$(find ur_robot_driver)/resources/rtde_input_recipe.txt"/>
  <xacro:arg name="reverse_ip" default="0.0.0.0"/>
  <xacro:arg name="script_command_port" default="50004"/>
  <xacro:arg name="reverse_port" default="50001"/>
  <xacro:arg name="script_sender_port" default="50002"/>
  <xacro:arg name="trajectory_port" default="50003"/>
  <!--   tool communication related parameters-->
  <xacro:arg name="use_tool_communication" default="false" />
  <xacro:arg name="tool_voltage" default="0" />
  <xacro:arg name="tool_parity" default="0" />
  <xacro:arg name="tool_baud_rate" default="115200" />
  <xacro:arg name="tool_stop_bits" default="1" />
  <xacro:arg name="tool_rx_idle_chars" default="1.5" />
  <xacro:arg name="tool_tx_idle_chars" default="3.5" />
  <xacro:arg name="tool_device_name" default="/tmp/ttyUR" />
  <xacro:arg name="tool_tcp_port" default="54321" />
  <!-- Simulation parameters -->
  <xacro:arg name="use_fake_hardware" default="false" />
  <xacro:arg name="fake_sensor_commands" default="false" />
  <xacro:arg name="sim_gazebo" default="false" />
  <xacro:arg name="sim_ignition" default="false" />
  <xacro:arg name="simulation_controllers" default="" />
  <!-- initial position for simulations (Fake Hardware, Gazebo, Ignition) -->
  <xacro:arg name="initial_positions_file" default="$(find ur_description)/config/initial_positions.yaml"/>
  <!-- convert to property to use substitution in function -->
  <xacro:property name="initial_positions_file" default="$(arg initial_positions_file)"/>
  <xacro:property name="is_sim_gazebo" value="$(arg sim_gazebo)"/>
  <xacro:property name="is_sim_ignition" value="$(arg sim_ignition)"/>
  <!-- arm -->
  <xacro:ur_robot
    name="$(arg name)"
    tf_prefix="$(arg tf_prefix)"
    parent="{{ parent }}"
    joint_limits_parameters_file="$(arg joint_limit_params)"
    kinematics_parameters_file="$(arg kinematics_params)"
    physical_parameters_file="$(arg physical_params)"
    visual_parameters_file="$(arg visual_params)"
    transmission_hw_interface="$(arg transmission_hw_interface)"
    safety_limits="$(arg safety_limits)"
    safety_pos_margin="$(arg safety_pos_margin)"
    safety_k_position="$(arg safety_k_position)"
    use_fake_hardware="$(arg use_fake_hardware)"
    fake_sensor_commands="$(arg fake_sensor_commands)"
    sim_gazebo="$(arg sim_gazebo)"
    sim_ignition="$(arg sim_ignition)"
    headless_mode="$(arg headless_mode)"
    initial_positions="${xacro.load_yaml(initial_positions_file)}"
    use_tool_communication="$(arg use_tool_communication)"
    tool_voltage="$(arg tool_voltage)"
    tool_parity="$(arg tool_parity)"
    tool_baud_rate="$(arg tool_baud_rate)"
    tool_stop_bits="$(arg tool_stop_bits)"
    tool_rx_idle_chars="$(arg tool_rx_idle_chars)"
    tool_tx_idle_chars="$(arg tool_tx_idle_chars)"
    tool_device_name="$(arg tool_device_name)"
    tool_tcp_port="$(arg tool_tcp_port)"
    robot_ip="$(arg robot_ip)"
    script_filename="$(arg script_filename)"
    output_recipe_filename="$(arg output_recipe_filename)"
    input_recipe_filename="$(arg input_recipe_filename)"
    reverse_ip="$(arg reverse_ip)"
    script_command_port="$(arg script_command_port)"
    reverse_port="$(arg reverse_port)"
    script_sender_port="$(arg script_sender_port)"
    trajectory_port="$(arg trajectory_port)"
    >
    <origin xyz="{{ origin_xyz }}" rpy="{{ origin_rpy }}" />          <!-- position robot in the world -->
  </xacro:ur_robot>

""",
    'links': {
        'base': 'base_link_inertia',
        'base_2': 'base_link',
        'link_1': 'shoulder_link',
        'link_2': 'upper_arm_link',
        'link_3': 'forearm_link',
        'link_4': 'wrist_1_link',
        'link_5': 'wrist_2_link',
        'link_6': 'wrist_3_link',
    },
    'joints': {
        'joint_1': 'shoulder_pan_joint',
        'joint_2': 'shoulder_lift_joint',
        'joint_3': 'elbow_joint',
        'joint_4': 'wrist_1_joint',
        'joint_5': 'wrist_2_joint',
        'joint_6': 'wrist_3_joint',
    },
    'to_ompl': """
planning_plugin: ompl_interface/OMPLPlanner
request_adapters: default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints
start_state_max_bounds_error: 0.1
""",
    'to_controllers': """
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01
  trajectory_duration_monitoring: true

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager: 
"""
}

launch_templates = {
    'visualize_w_joint_gui': """
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
    """,
    'visualize_w_physical_connection': """
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
    """,
    "moveit_simulation": """
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

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
        ]
    )
""",
    "moveit_w_physical_connection": """
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
"""
}

srdf_template = """<?xml version="1.0" encoding="UTF-8"?>
<robot name="{{ robot_model }}">
    <group name="{{ robot_model }}_manipulator">
        <chain base_link="base" tip_link="flange"/>
    </group>
{{ group_states }}
{{ disabled_collisions }}
</robot>
"""

