import yaml
import os, sys
from templates import urdf, launch, srdf, predefined

config_file_path = '/home/ws/config.yaml'

with open(config_file_path, 'r') as file:
    config = yaml.safe_load(file)

system = config['system']
robot = config['robot']
robot['specific'] = urdf.get_robot_specific(robot['type'])
scene = config['scene']
accessories = config['accessories']
srdf_config = config['srdf']
combined_elements = {}
if accessories:
    combined_elements.update(accessories)
if scene:
    combined_elements.update(scene)

uni_pal_description_dir = "/home/ws/src/uni_pal_description"
uni_pal_dir = "/home/ws/src/uni_pal"
urdf_dir = os.path.join(uni_pal_description_dir, 'urdf')
mesh_dir = os.path.join(uni_pal_description_dir, 'meshes')
config_dir = os.path.join(uni_pal_description_dir, 'config')
main_urdf_file_path = os.path.join(urdf_dir, 'robot.urdf.xacro')
launch_file_path = os.path.join(uni_pal_dir, 'launch', system['launch_type'] + '.launch.py')
srdf_file_path = os.path.join(config_dir, 'robot.srdf')

######################################################

if len(sys.argv) > 1:
    if sys.argv[1] == "clear":
        urdf.delete_old_files(urdf_dir, os.path.join(urdf_dir, '.ignore'))
        urdf.delete_old_files(mesh_dir, os.path.join(mesh_dir, '.ignore'))
        urdf.delete_old_files(config_dir, os.path.join(config_dir, '.ignore'))
        sys.exit()

######################################################

# URDF generation
urdf.delete_old_files(urdf_dir, os.path.join(urdf_dir, '.ignore'))
urdf.delete_old_files(mesh_dir, os.path.join(mesh_dir, '.ignore'))
urdf.start_urdf(main_urdf_file_path, robot['model'])
urdf.generate_elements(main_urdf_file_path, scene, robot['specific'], uni_pal_description_dir)
urdf.append_robot(main_urdf_file_path, robot)
urdf.generate_elements(main_urdf_file_path, accessories, robot['specific'], uni_pal_description_dir)
print(f"URDF file generated at {main_urdf_file_path}")

# launch file generation
urdf.delete_old_files(config_dir, os.path.join(config_dir, '.ignore'))
launch.start_launch(launch_file_path, system['launch_type'], robot['model'], robot['ip'])
launch.copy_config(uni_pal_description_dir, robot['type'], robot['model'])

# SRDF generation
srdf.start_srdf(srdf_file_path, robot['model'], robot['specific'], srdf_config, combined_elements)

# Copy Robot Config file and create predefined values file
uni_pal_pylib_dir = "/home/ws/src/uni_pal_pylib/utils/uni_pal_pylib"
predefined.copy_json_file(os.path.join(config_dir, 'robot_config.json'), system['json_path'])
predefined.start_predefined(os.path.join(uni_pal_pylib_dir, 'predefined.py'), robot['data'], robot['type'])