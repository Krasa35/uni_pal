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

output_dir = "/home/ws/src/uni_pal_description/urdf"
output_file_path = os.path.join(output_dir, 'robot.urdf.xacro')

urdf.start_urdf(output_file_path, "ur10")
urdf.generate_before_robot_scene_elements(output_file_path, scene, output_dir)
urdf.append_robot(output_file_path, robot['type'])

print(f"URDF file generated at {output_file_path}")