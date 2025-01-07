#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from uni_pal_msgs.srv import GetPalParams, GetPlacePos, GetConfigParams
from uni_pal_pylib.utils import create_pose_stamped
from uni_pal_pylib.predefined import Data
from dataclasses import dataclass, field
from typing import List

@dataclass
class BoxPosition:
    id: int
    x: int
    y: int
    rotation: int

@dataclass
class LayerPattern:
    name: str
    boxPositions: List[BoxPosition]
    boxPositionsAlt: List[BoxPosition]
    boxGroups: List[List[int]]
    boxGroupsAlt: List[List[int]]
    gripperSections: List[List[int]]
    gripperSectionsAlt: List[List[int]]
    manualOrder: bool
    manualOrderAlt: bool

@dataclass
class layers:
    number: int
    layerPattern: str

@dataclass
class Pallet:
    palletType: str
    length: int
    width: int
    overhangL: int
    overhangT: int
    overhangR: int
    overhangB: int
    layerPatterns: List[LayerPattern]
    layers: List[layers]
    interlayer: List[bool]
    height: int

@dataclass
class Box:
    length: int
    width: int
    height: int
    weight: int
    lengthMargin: int
    widthMargin: int
    ignoreBoxLimitations: bool

@dataclass
class ProgramSettings:
    robotType: str
    approachAngle: int
    conveyorConfiguration: str
    boxOrientationOnConveyor: str
    labelPosition: str
    additionalPedestalHeight: int

@dataclass
class JsonData:
    guid: str
    name: str
    created: str
    updated: str
    layers_per_pallet: int
    boxes_per_layer: int
    box: Box
    pallet: Pallet
    programsettings: ProgramSettings

class ServiceServer(Node):

    def __init__(self):
        super().__init__('read_json_node')
        package_path = get_package_share_directory('uni_pal_description')
        file_path = os.path.join(package_path, 'config/robot_config.json')
        self.declare_parameter('config_path', file_path)
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        file_path = config_path  # Replace with the path to your JSON file
        self.json_data: JsonData = self.read_json_file(file_path)
        self._service = self.create_service(
            GetPlacePos,
            'read_json_node/get_box_position',
            self.find_box_position)
        self.get_logger().info('/read_json_node/get_box_position service server has been started.')
        self._service = self.create_service(
            GetPalParams,
            'read_json_node/get_pallet_params',
            self.handle_palletize_parameters)
        self.get_logger().info('/read_json_node/get_pallet_params service server has been started.')
        self._service = self.create_service(
            GetConfigParams,
            'read_json_node/get_config_params',
            self.handle_config_parameters)
        self.get_logger().info('/read_json_node/get_config_params service server has been started.')

    def read_json_file(self, file_path: str) -> JsonData:
        with open(file_path, 'r') as file:
            data = json.load(file)
        
        # Parse the JSON data into the dataclass structure
        box_positions = [BoxPosition(**pos) for pos in data['pallet']['layerPatterns'][0]['boxPositions']]
        layer_patterns = [LayerPattern(**lp) for lp in data['pallet']['layerPatterns']]
        layers_ = [layers(**lay) for lay in data['pallet']['layers']]
        programSettings = [ProgramSettings(**data['programSettings'])] 
        pallet = Pallet(
            palletType=data['pallet']['palletType'],
            length=data['pallet']['length'],
            width=data['pallet']['width'],
            overhangL=data['pallet']['overhangL'],
            overhangT=data['pallet']['overhangT'],
            overhangR=data['pallet']['overhangR'],
            overhangB=data['pallet']['overhangB'],
            layerPatterns=layer_patterns,
            layers=layers_,
            interlayer=data['pallet']['interlayerSheetPositions'],
            height=data['pallet']['height']
        )
        box = Box(**data['box'])
        json_data = JsonData(
            guid=data['guid'],
            name=data['name'],
            created=data['created'],
            updated=data['updated'],
            layers_per_pallet = len(pallet.layers),
            boxes_per_layer = len(pallet.layerPatterns[0].boxPositions),
            box=box,
            pallet=pallet,
            programsettings=programSettings
        )
        return json_data

    def find_box_position(self, request: GetPlacePos.Request, response: GetPlacePos.Response):
        self.get_logger().info('Handling /read_json_node/get_box_position request...')
        # Find the layer pattern for the given layer number
        layer_pattern = None
        for layer in self.json_data.pallet.layers:
            if layer.number == request.layer_no:
                layer_pattern = layer.layerPattern
                break

        if layer_pattern is None:
            raise ValueError(f"Layer number {request.layer_no} not found")

        # Find the box position for the given box ID in the layer pattern
        for pattern in self.json_data.pallet.layerPatterns:
            if pattern.name == layer_pattern:
                if request.box_no <= len(pattern.boxPositions):
                    if request.pallet_side == "LEFT":
                        response.place_pose = create_pose_stamped(
                            xyz_=[pattern.boxPositions[request.box_no]['x'], 
                                  pattern.boxPositions[request.box_no]['y'], 
                                  0.0], 
                            rpy_=[0.0, 0.0, 0.0], 
                            seconds_=0,
                            frame_id_="base_link")
                    if  request.pallet_side == "RIGHT":
                        response.place_pose = create_pose_stamped(
                            xyz_=[pattern.boxPositionsAlt[request.box_no]['x'], 
                                  pattern.boxPositionsAlt[request.box_no]['y'], 
                                  0.0], 
                            rpy_=[0.0, 0.0, 0.0], 
                            seconds_=0,
                            frame_id_="base_link")
        return response

    def handle_palletize_parameters(self, request, response: GetPalParams.Response):
        self.get_logger().info('Handling /read_json_node/get_pallet_params request...')

        response.params.boxes_per_layer = self.json_data.boxes_per_layer
        response.params.layers_per_pallet = self.json_data.layers_per_pallet
        response.params.interlayers = self.json_data.pallet.interlayer
        response.params.box.width = float(self.json_data.box.width)
        response.params.box.length = float(self.json_data.box.length)
        response.params.box.height = float(self.json_data.box.height)
        response.params.box.weight = float(self.json_data.box.weight)
        response.params.pallet.width = float(self.json_data.pallet.width)
        response.params.pallet.length = float(self.json_data.pallet.length)
        response.params.pallet.height = float(self.json_data.pallet.height)
        response.params.pallet.overhang_l = float(self.json_data.pallet.overhangL)
        response.params.pallet.overhang_t = float(self.json_data.pallet.overhangT)
        response.params.pallet.overhang_r = float(self.json_data.pallet.overhangR)
        response.params.pallet.overhang_b = float(self.json_data.pallet.overhangB)
        response.params.programsettings.robot_type = self.json_data.programsettings[0].robotType
        response.params.programsettings.conveyor_configuration = self.json_data.programsettings[0].conveyorConfiguration
        response.params.programsettings.box_orientation = self.json_data.programsettings[0].boxOrientationOnConveyor
        response.params.programsettings.label_position = self.json_data.programsettings[0].labelPosition
        response.params.programsettings.additional_pedestal_height = float(self.json_data.programsettings[0].additionalPedestalHeight)
        response.params.programsettings.approach_angle = float(self.json_data.programsettings[0].approachAngle)

        return response

    def handle_config_parameters(self, request, response: GetConfigParams.Response):
        self.get_logger().info('Handling /read_json_node/get_config_params request...')
        response.config.max_reach = Data.max_reach
        response.config.gripper_width = Data.gripper_width
        response.config.gripper_length = Data.gripper_length
        response.config.gripper_height = Data.gripper_height
        response.config.gripper_offset = Data.gripper_offset
        response.config.frames = Data.frames
        response.config.eefs = Data.eefs
        response.config.joint_names = Data.joint_names
        response.config.link_names = Data.link_names
        response.config.flange_name = Data.flange_name
        response.config.arm_group_name = Data.arm_group_name
        response.config.planning_groups = Data.planning_groups

        return response

def main(args=None):
    rclpy.init(args=args)
    action_server = ServiceServer()
    rclpy.spin(action_server)

if __name__ == '__main__':
    main()