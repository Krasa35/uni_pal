import yaml
import os
from jinja2 import Template
from templates import urdf

config_file_path = '/home/ws/config.yaml'

with open(config_file_path, 'r') as file:
    config = yaml.safe_load(file)

system = config['system']
robot = config['robot']
scene = config['scene']
accessories = config['accessories']

######################################################

uni_pal_description_dir = "/home/ws/src/uni_pal_description"
urdf_dir = os.path.join(uni_pal_description_dir, 'urdf')
mesh_dir = os.path.join(uni_pal_description_dir, 'meshes')
output_file_path = os.path.join(urdf_dir, 'robot.urdf.xacro')

# URDF generation
urdf.delete_old_files(urdf_dir, os.path.join(urdf_dir, '.ignore'))
urdf.delete_old_files(mesh_dir, os.path.join(mesh_dir, '.ignore'))
urdf.start_urdf(output_file_path, robot['model'])
urdf.generate_elements(output_file_path, scene, uni_pal_description_dir)
urdf.append_robot(output_file_path, robot)
urdf.generate_elements(output_file_path, accessories, uni_pal_description_dir)

print(f"URDF file generated at {output_file_path}")