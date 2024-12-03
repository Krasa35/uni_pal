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

######################################################

uni_pal_description_dir = "/home/ws/src/uni_pal_description"
output_file_path = os.path.join(uni_pal_description_dir, 'urdf', 'robot.urdf.xacro')

urdf.start_urdf(output_file_path, "ur10")
urdf.generate_before_robot_scene_elements(output_file_path, scene, uni_pal_description_dir)
urdf.append_robot(output_file_path, robot['type'])

print(f"URDF file generated at {output_file_path}")