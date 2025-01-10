from jinja2 import Template
from templates import macros

def start_joint_limits(file_path, robot_specific, joint_limits_config):
    joint_limit = ""
    for joint, joint_ in robot_specific['joints'].items():
        joint_limit += f'  {joint_}:\n'
        
        if joint in joint_limits_config and 'max_velocity' in joint_limits_config[joint] and 'no_velocity_limit' == joint_limits_config[joint]['max_velocity']:
            joint_limit += "    has_velocity_limits: false\n"
        elif joint in joint_limits_config and 'max_velocity' in joint_limits_config[joint]:
            joint_limit += "    has_velocity_limits: true\n"
            joint_limit += f"    max_velocity: {joint_limits_config[joint]['max_velocity']}\n"
        else:
            joint_limit += "    has_velocity_limits: true\n"
            joint_limit += "    max_velocity: 1.0\n"
        
        if joint in joint_limits_config and 'max_acceleration' in joint_limits_config[joint] and 'no_acceleration_limit' == joint_limits_config[joint]['max_acceleration']:
            joint_limit += "    has_acceleration_limits: false\n"
        elif joint in joint_limits_config and 'max_acceleration' in joint_limits_config[joint]:
            joint_limit += "    has_acceleration_limits: true\n"
            joint_limit += f"    max_acceleration: {joint_limits_config[joint]['max_acceleration']}\n"
        else:
            joint_limit += "    has_acceleration_limits: true\n"
            joint_limit += "    max_acceleration: 5.0\n"
        
    template = Template(macros.joint_limits_template)
    joint_limits_content = template.render(
        joint_limits=joint_limit,
    )

    with open(file_path, 'w') as joint_limits_file:
        joint_limits_file.write(joint_limits_content)
        print(f"joint_limits file generated at {file_path}")