import os
from jinja2 import Template
from templates import robot_macros

# element_template = """<?xml version="1.0"?>
# <robot name="{{ name }}">
#   <link name="world" />
#   <link name="{{ name }}">
#       <visual>
#         <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
#         <geometry>
#         <mesh filename="{{ mesh }}"/>
#         </geometry>
#           <material name="Gray">
#           <color rgba="0.5 0.5 0.5 1.0"/>
#           </material>
#       </visual>
#       <collision>
#         <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
#         <geometry>
#         <mesh filename="{{ mesh }}"/>
#         </geometry>
#       </collision>
#   </link>
#   <joint name="{{ parent }} - {{ name }}" type="fixed">
#     <origin xyz="{{ position }}" rpy="{{ orientation }}" />
#     <parent link="{{ parent }}" />
#     <child link="{{ name }}" />
#   </joint>
# </robot>"""

element_template = """<?xml version="1.0"?>
<robot name="{{ name }}">
  <link name="{{ name }}">
      <visual>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
        <box size="{{ size }}"/>
        </geometry>
          <material name="Gray">
          <color rgba="0.5 0.5 0.5 1.0"/>
          </material>
      </visual>
      <collision>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
        <box size="{{ size }}"/>
        </geometry>
      </collision>
  </link>
  <joint name="{{ parent }} - {{ name }}" type="fixed">
    <origin xyz="{{ position }}" rpy="{{ orientation }}" />
    <parent link="{{ parent }}" />
    <child link="{{ name }}" />
  </joint>
</robot>"""

xacro_include_template = """\t<xacro:include filename="{{ file_path }}"/> """

urdf_begin_template = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="{{ robot_name }}">
   <xacro:arg name="name" default="{{ robot_name }}"/>      
   <link name="world" />
</robot>
"""

def insert_content(file_path, content):
  with open(file_path, 'r') as urdf_file:
      lines=urdf_file.readlines()
  lines.insert(-1, content)
  lines.insert(-1, "\n")
  with open(file_path, 'w') as urdf_file:
      urdf_file.writelines(lines)

def get_robot_specific(robot_type):
    return robot_macros.universal_robots if robot_type == 'universal_robots' else robot_macros.techman_robots if robot_type == 'techman_robots' else None

def start_urdf(file_path, name):
  template = Template(urdf_begin_template)
  urdf_content = template.render(
      robot_name=name
  )
  with open(file_path, 'w') as urdf_file:
      urdf_file.write(urdf_content)

def append_element(file_path, element_path):
  template = Template(xacro_include_template)
  append_content = template.render(
      file_path=element_path
  )
  insert_content(file_path, append_content)

def generate_before_robot_scene_elements(urdf_path, scene, output_dir):
    template = Template(element_template)
    for name, properties in scene.items():
        element_content = template.render(
            name=name,
            position=properties['position'],
            orientation=properties['orientation'],
            # mesh=properties['mesh'],
            size=properties['size'],
            parent=properties['parent']
        )
        element_file_path = os.path.join(output_dir, f"{name}.urdf")
        element_find_path = os.path.join("$(find uni_pal_description)", "urdf", f"{name}.urdf")
        with open(element_file_path, 'w') as element_file:
            element_file.write(element_content)
        append_element(urdf_path, element_find_path)
        print(f"Element file generated at {element_file_path}")

def append_robot(urdf_path, robot_type):
   insert_content(urdf_path, get_robot_specific(robot_type)['urdf_macro'])

