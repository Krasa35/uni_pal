import os
import subprocess
import shutil
from jinja2 import Template
from templates import macros
from uni_pal_pylib.utils import insert_content, indent_content
import glob

def start_launch(file_path, launch_type, robot_model_, robot_specific_, robot_ip_="xxx.xxx.xxx.xxx"):
  robot_driver_package_ = robot_specific_['driver_package']
  robot_driver_executable_ = robot_specific_['driver_executable']
  template = Template(macros.launch_templates[launch_type])
  launch_content = template.render(
      robot_model=robot_model_,
      robot_ip=robot_ip_,
      robot_driver_package=robot_driver_package_,
      robot_driver_executable=robot_driver_executable_,
  )
  with open(file_path, 'w') as launch_file:
      launch_file.write(launch_content)
      print(f"Launch file generated at {file_path}")
  
def replace_group_name_in_file(file_path, arm_group_name_previous, replacement):
    with open(file_path, 'r') as file:
        content = file.read()

    content = content.replace(arm_group_name_previous, replacement)

    with open(file_path, 'w') as file:
        file.write(content)

def copy_config(target_dir, robot_type, robot_model, arm_group_name):
    config_dir = os.path.join(target_dir, 'config')
    config_package = 'ur_description' if robot_type == "universal_robots" else robot_model + '_moveit_config' if robot_type == "techman_robots" else "NOT SUPPORTED"
    try:
        source_path = subprocess.check_output(['ros2', 'pkg', 'prefix', config_package, "--share"]).decode().strip()
        config_source_path = os.path.join(source_path, 'config', robot_model) if robot_type == "universal_robots" else os.path.join(source_path, 'config')
        shutil.copytree(config_source_path, config_dir, dirs_exist_ok=True)
        if robot_type == "universal_robots":
          source_path2 = subprocess.check_output(['ros2', 'pkg', 'prefix', 'ur_moveit_config', "--share"]).decode().strip()
          shutil.copytree(os.path.join(source_path2, 'config'), config_dir, dirs_exist_ok=True)
          shutil.copyfile(os.path.join(source_path2, 'srdf', 'ur.srdf.xacro'), os.path.join(config_dir, robot_model + '.srdf.xacro'))
          os.rename(os.path.join(config_dir, 'controllers.yaml'), os.path.join(config_dir, 'moveit_controllers.yaml'))
          print(f"Copied files from:\n\t{os.path.join(source_path2, 'config')}\nto:\n\t{config_dir}")
          insert_content(os.path.join(config_dir, 'ompl_planning.yaml'), macros.universal_robots['to_ompl'],0)
          indent_content(os.path.join(config_dir, 'moveit_controllers.yaml'))
          insert_content(os.path.join(config_dir, 'moveit_controllers.yaml'), macros.universal_robots['to_controllers'],0)
          replace_group_name_in_file(os.path.join(config_dir, 'ompl_planning.yaml'), 'ur_manipulator', arm_group_name)
          replace_group_name_in_file(os.path.join(config_dir, 'kinematics.yaml'), 'ur_manipulator', arm_group_name)
          replace_group_name_in_file(os.path.join(config_dir, 'ur_servo.yaml'), 'ur_manipulator', arm_group_name)
        print(f"Copied files from:\n\t{config_source_path}\nto:\n\t{config_dir}")
    except subprocess.CalledProcessError as e:
        print(f"Error finding {config_package} package: {e}")
        return

    # Construct the model directory path
    model_dir_for_UR = os.path.join(source_path, robot_model) if robot_type == "universal_robot" else source_path