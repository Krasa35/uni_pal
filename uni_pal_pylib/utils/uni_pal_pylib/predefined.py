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
    frames=['world', 'base', 'base_link'],
    eefs=['flange', 'tool0', 'gripper'],
    joint_names=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
    link_names=['base_link', 'base_link_inertia', 'base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0'],
    flange_name="flange",
    arm_group_name="ur_manipulator",
    planning_groups=['manipulator']
)