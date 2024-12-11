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
"""
}

universal_robots = {
    'urdf_macro_include': """$(find ur_description)/urdf/ur_macro.xacro""",
    'urdf_macro': """
  <!-- import main macro -->
  <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
  <xacro:arg name="ur_type" default="ur5x"/>
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
  <xacro:arg name="robot_ip" default="0.0.0.0" />
  <xacro:arg name="script_filename" default=""/>
  <xacro:arg name="output_recipe_filename" default=""/>
  <xacro:arg name="input_recipe_filename" default=""/>
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

"""
}

