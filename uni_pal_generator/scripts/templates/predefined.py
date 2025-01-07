import shutil
from jinja2 import Template
from templates import macros


def copy_json_file(target_path, source_path):
    shutil.copyfile(source_path, target_path)
    print(f"Copied file from {source_path} to {target_path}")

def start_predefined(file_path, data, robot_type):
    robot = macros.universal_robots if robot_type == 'universal_robots' else macros.techman_robots if robot_type == 'techman_robots' else None
    template = Template(macros.predefined_template)
    predefined_content = template.render(
        max_reach=data['max_reach'],
        gripper_width=data['gripper_width'],
        gripper_length=data['gripper_length'],
        gripper_height=data['gripper_height'],
        gripper_offset=data['gripper_offset'],
        frames=data['frames'],
        eefs=data['eefs'],
        joint_names=list(robot['joints'].values()),
        link_names=list(robot['links'].values()),
        flange_name=data['flange_name'],
        arm_group_name=data['arm_group_name'],
        planning_groups=data['planning_groups'],
    )
    with open(file_path, 'w') as predefined_file:
        predefined_file.write(predefined_content)
        print(f"Predefined file generated at {file_path}")

