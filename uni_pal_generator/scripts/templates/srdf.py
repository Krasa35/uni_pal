from jinja2 import Template
from templates import macros

def start_srdf(file_path, robot_model_, robot_specific, srdf_config, accessories, arm_group_name_):
    accessories.update(robot_specific['links'])
    group_states_ = ""
    for state_name, joints in srdf_config['group_states']['manipulator'].items():
        group_states_ += f'    <group_state name="{state_name}" group="{arm_group_name_}">\n'
        for joint, value in zip(robot_specific['joints'].values(), joints):
            group_states_ += f'        <joint name="{joint}" value="{value}"/>\n'
        group_states_ += '    </group_state>\n'

    disabled_collisions_ = ""
    for link1, link2_list in srdf_config['disabled_collisions'].items():
        if link1 not in accessories:
            print(f"{link1} is not present in URDF configuration!!!")
            continue
        link1 = robot_specific['links'][link1] if link1 in robot_specific['links'] else link1 
        for link2 in link2_list:
            if link2 not in accessories:
                print(f"{link2} is not present in URDF configuration!!!")
                continue
            link2 = robot_specific['links'][link2] if link2 in robot_specific['links'] else link2
            disabled_collisions_ += f'    <disable_collisions link1="{link1}" link2="{link2}" reason="Adjacent"/>\n'

    template = Template(macros.srdf_template)
    srdf_content = template.render(
        arm_group_name=arm_group_name_,
        robot_model=robot_model_,
        group_states=group_states_,
        disabled_collisions=disabled_collisions_,
    )

    with open(file_path, 'w') as srdf_file:
        srdf_file.write(srdf_content)
        print(f"SRDF file generated at {file_path}")