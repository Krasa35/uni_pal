from dataclasses import dataclass
from typing import List

@dataclass
class data:
    max_reach: int
    gripper_width: int
    gripper_length: int
    gripper_height: int
    gripper_offset: int
    frames: List[str]
    eefs: List[str]
    joint_names: List[str]
    link_names: List[str]
    flange_name: str
    arm_group_name: str
    planning_groups: List[str]

Data = data(
    max_reach=1.3,
    gripper_width=0.3,
    gripper_length=0.5,
    gripper_height=0.1,
    gripper_offset=0.05,
    frames=['world', 'base', 'conveyor'],
    eefs=['flange', 'tool0', 'gripper'],
    joint_names={'joint_1': 'shoulder_pan_joint', 'joint_2': 'shoulder_lift_joint', 'joint_3': 'elbow_joint', 'joint_4': 'wrist_1_joint', 'joint_5': 'wrist_2_joint', 'joint_6': 'wrist_3_joint'},
    link_names={'base': 'base_link_inertia', 'base_2': 'base_link', 'link_1': 'shoulder_link', 'link_2': 'upper_arm_link', 'link_3': 'forearm_link', 'link_4': 'wrist_1_link', 'link_5': 'wrist_2_link', 'link_6': 'wrist_3_link'},
    flange_name="flange",
    arm_group_name="manipulator",
    planning_groups=['manipulator']
)