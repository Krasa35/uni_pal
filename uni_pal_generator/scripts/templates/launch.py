import os
import subprocess
import shutil
from jinja2 import Template
from templates import macros
import glob

def start_launch(file_path, launch_type, robot_model_, robot_ip_="xxx.xxx.xxx.xxx"):
  template = Template(macros.launch_templates[launch_type])
  srdf_subfix_ = 'srdf.xacro' if robot_model_[:2] == 'ur' else 'srdf'
  launch_content = template.render(
      robot_model=robot_model_,
      robot_ip=robot_ip_,
      srdf_subfix=srdf_subfix_
  )
  with open(file_path, 'w') as launch_file:
      launch_file.write(launch_content)
      print(f"Launch file generated at {file_path}")
  
def copy_config(target_dir, robot_type, robot_model):
    config_dir = os.path.join(target_dir, 'config')
    config_package = 'ur_description' if robot_type == "universal_robots" else robot_model + '_moveit_config' if robot_type == "techman_robots" else "NOT SUPPORTED"
    try:
        if robot_type == "universal_robots":
          source_path2 = subprocess.check_output(['ros2', 'pkg', 'prefix', 'ur_moveit_config', "--share"]).decode().strip()
          shutil.copytree(os.path.join(source_path2, 'config'), config_dir, dirs_exist_ok=True)
          shutil.copyfile(os.path.join(source_path2, 'srdf', 'ur.srdf.xacro'), os.path.join(config_dir, robot_model + '.srdf.xacro'))
          print(f"Copied files from:\n\t{os.path.join(source_path2, 'config')}\nto:\n\t{config_dir}")
        source_path = subprocess.check_output(['ros2', 'pkg', 'prefix', config_package, "--share"]).decode().strip()
        config_source_path = os.path.join(source_path, 'config', robot_model) if robot_type == "universal_robots" else os.path.join(source_path, 'config')
        shutil.copytree(config_source_path, config_dir, dirs_exist_ok=True)
        print(f"Copied files from:\n\t{config_source_path}\nto:\n\t{config_dir}")
    except subprocess.CalledProcessError as e:
        print(f"Error finding {config_package} package: {e}")
        return

    # Construct the model directory path
    model_dir_for_UR = os.path.join(source_path, robot_model) if robot_type == "universal_robot" else source_path