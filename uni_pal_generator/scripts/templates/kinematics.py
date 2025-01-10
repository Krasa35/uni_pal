from jinja2 import Template
from templates import macros

def start_kinematics(file_path, arm_group_name_):
    template = Template(macros.kinematics_template)
    kinematics_content = template.render(
        arm_group_name=arm_group_name_,
    )

    with open(file_path, 'w') as kinematics_file:
        kinematics_file.write(kinematics_content)
        print(f"kinematics file generated at {file_path}")