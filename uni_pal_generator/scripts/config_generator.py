import yaml
import os, sys
from jinja2 import Template
from templates import urdf, launch

uni_pal_description_dir = "/home/ws/src/uni_pal_description"
urdf_dir = os.path.join(uni_pal_description_dir, 'urdf')
mesh_dir = os.path.join(uni_pal_description_dir, 'meshes')
config_dir = os.path.join(uni_pal_description_dir, 'config')
if len(sys.argv) > 1:
    if sys.argv[1] == "clear":
        urdf.delete_old_files(urdf_dir, os.path.join(urdf_dir, '.ignore'))
        urdf.delete_old_files(mesh_dir, os.path.join(mesh_dir, '.ignore'))
        urdf.delete_old_files(config_dir, os.path.join(config_dir, '.ignore'))
        sys.exit()

config_file_path = '/home/ws/config.yaml'

with open(config_file_path, 'r') as file:
    config = yaml.safe_load(file)

system = config['system']
robot = config['robot']
robot['specific'] = urdf.get_robot_specific(robot['type'])
scene = config['scene']
accessories = config['accessories']

######################################################

# URDF generation
output_file_path = os.path.join(urdf_dir, 'robot.urdf.xacro')

urdf.delete_old_files(urdf_dir, os.path.join(urdf_dir, '.ignore'))
urdf.delete_old_files(mesh_dir, os.path.join(mesh_dir, '.ignore'))
urdf.start_urdf(output_file_path, robot['model'])
urdf.generate_elements(output_file_path, scene, robot['specific'], uni_pal_description_dir)
urdf.append_robot(output_file_path, robot)
urdf.generate_elements(output_file_path, accessories, robot['specific'], uni_pal_description_dir)

# launch file generation
uni_pal_dir = "/home/ws/src/uni_pal"
launch_file_path = os.path.join(uni_pal_dir, 'launch', system['launch_type'] + '.launch.py')

urdf.delete_old_files(config_dir, os.path.join(config_dir, '.ignore'))
launch.start_launch(launch_file_path, system['launch_type'], robot['model'], robot['ip'])
launch.copy_config(uni_pal_description_dir, robot['type'], robot['model']) if system['launch_type'] == 'test_demo' else None

print(f"URDF file generated at {output_file_path}")