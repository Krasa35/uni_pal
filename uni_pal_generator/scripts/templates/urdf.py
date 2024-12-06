import os
import shutil
from jinja2 import Template
from templates import robot_macros
import glob

box_template_ = """<box size="{{ size }}"/>"""
mesh_template_ = """<mesh filename="file://$(find uni_pal_description)/meshes/{{ mesh }}" scale="{{ scale }}"/>"""
origin_template_ = """<origin xyz="{{ origin_xyz }}" rpy="{{ origin_rpy }}"/>"""

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

xacro_include_template = """\t<xacro:include filename="{{ file_path }}"/> """

urdf_template = """<?xml version="1.0"?>
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
  template = Template(urdf_template)
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

def generate_before_robot_scene_elements(urdf_path, scene, description_dir):
    template = Template(element_template)
    for name, properties in scene.items():
        if "mesh" in properties:
           mesh_path = os.path.join(description_dir, 'meshes', os.path.basename(properties["mesh"]))
           shutil.copy(properties["mesh"], mesh_path)
           geometry_template = Template(mesh_template_)
           geometry_ = geometry_template.render(
                mesh=os.path.basename(properties["mesh"]),
                scale=properties["size"] if "size" in properties else "1.0 1.0 1.0"
           )
        elif "mesh" not in properties and "size" in properties:
           geometry_template = Template(box_template_)
           geometry_ = geometry_template.render(
              size=properties["size"]
           )
        origin_template = Template(origin_template_)
        origin_xyz = properties["origin_xyz"] if "origin_xyz" in properties else "0.0 0.0 0.0"
        origin_rpy = properties["origin_rpy"] if "origin_rpy" in properties else "0.0 0.0 0.0"
        origin_ = origin_template.render(
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy
        )
        element_content = template.render(
            name=name,
            position=properties['position'],
            orientation=properties['orientation'],
            geometry=geometry_,
            origin=origin_,
            parent=properties['parent']
        )
        element_file_path = os.path.join(description_dir, 'urdf', f"{name}.urdf")
        element_find_path = os.path.join("$(find uni_pal_description)", "urdf", f"{name}.urdf")
        with open(element_file_path, 'w') as element_file:
            element_file.write(element_content)
        append_element(urdf_path, element_find_path)
        print(f"Element file generated at {element_file_path}")

def append_robot(urdf_path, robot):
   robot_template = Template(get_robot_specific(robot["type"])['urdf_macro'])
   origin_xyz_ = robot["origin_xyz"] if "origin_xyz" in robot else "0.0 0.0 0.0"
   origin_rpy_ = robot["origin_rpy"] if "origin_rpy" in robot else "0.0 0.0 0.0"

   robot_ = robot_template.render(
      parent=robot["parent"],
      origin_xyz=origin_xyz_,
      origin_rpy=origin_rpy_
   )
   insert_content(urdf_path, robot_)

def delete_old_files(description_dir, ignore_list_file):
    with open(ignore_list_file, 'r') as file:
        ignore_list = [line.strip() for line in file.readlines()]
    
    files = glob.glob(os.path.join(description_dir, '*'))
    for f in files:
        if os.path.basename(f) not in ignore_list and f != ignore_list_file:
            os.remove(f)            