#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from uni_pal_msgs.srv import GetPalParams
from uni_pal_msgs.srv import GetPlacePos
from uni_pal_pylib.utils import create_pose_stamped, print_pose_stamped
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
        self.declare_parameter('config_path', '/home/ws/.github/Company_data/RobotConfig/TEST_2LAY_28102024/P_T_2410280951_1219X1016_90.json')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        # file_path = '/home/ws/.github/Company_data/RobotConfig/TEST_28102024_RIGHT/P_T_2410301058_1219X1016_48.json'  # Replace with the path to your JSON file
        # file_path = '/home/ws/.github/Company_data/RobotConfig/TEST_28102024/P_T_2410280951_1219X1016_48.json'  # Replace with the path to your JSON file
        file_path = config_path  # Replace with the path to your JSON file
        self.json_data: JsonData = self.read_json_file(file_path)
        self.get_logger().info('PickPlaceTrajectory action server has been started.')
        self._service = self.create_service(
            GetPlacePos,
            'read_json_node/get_box_position',
            self.find_box_position)
        self._service = self.create_service(
            GetPalParams,
            'read_json_node/get_pallet_params',
            self.handle_palletize_parameters)
        self.get_logger().info('PalletizeParameters service server has been started.')

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

        # Find the layer pattern for the given layer number
        layer_pattern = None
        self.get_logger().info(f'Got request for /get_box_position service: \n\tlayer_no: {request.layer_no}\n\tbox_no: {request.box_no}\n\tpallet_side: {request.pallet_side}')
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
        self.get_logger().info(f'Processed request for /get_box_position service: \nBox:{print_pose_stamped(response.place_pose)}')
        return response

    def handle_palletize_parameters(self, request, response: GetPalParams.Response):
        self.get_logger().info('Handling request...')

        response.boxes_per_layer = self.json_data.boxes_per_layer# Assuming all layers have the same number of boxes
        response.layers_per_pallet = self.json_data.layers_per_pallet
        response.interlayers = self.json_data.pallet.interlayer
        response.box_width = float(self.json_data.box.width)
        response.box_length = float(self.json_data.box.length)
        response.box_height = float(self.json_data.box.height)
        response.box_weight = float(self.json_data.box.weight)
        response.pallet_width = float(self.json_data.pallet.width)
        response.pallet_length = float(self.json_data.pallet.length)
        response.pallet_height = float(self.json_data.pallet.height)
        response.pallet_overhang_l = float(self.json_data.pallet.overhangL)
        response.pallet_overhang_t = float(self.json_data.pallet.overhangT)
        response.pallet_overhang_r = float(self.json_data.pallet.overhangR)
        response.pallet_overhang_b = float(self.json_data.pallet.overhangB)
        response.robot_type = self.json_data.programsettings[0].robotType
        response.conveyor_configuration = self.json_data.programsettings[0].conveyorConfiguration
        response.box_orientation = self.json_data.programsettings[0].boxOrientationOnConveyor
        response.label_position = self.json_data.programsettings[0].labelPosition
        response.additional_pedestal_height = float(self.json_data.programsettings[0].additionalPedestalHeight)
        response.approach_angle = float(self.json_data.programsettings[0].approachAngle)

        self.get_logger().info(f"Boxes per layer: {response.boxes_per_layer}")
        self.get_logger().info(f"Layers per pallet: {response.layers_per_pallet}")
        self.get_logger().info(f"Box width: {response.box_width}")
        self.get_logger().info(f"Box length: {response.box_length}")
        self.get_logger().info(f"Box height: {response.box_height}")
        self.get_logger().info(f"Box weight: {response.box_weight}")
        self.get_logger().info(f"Pallet width: {response.pallet_width}")
        self.get_logger().info(f"Pallet length: {response.pallet_length}")
        self.get_logger().info(f"Pallet height: {response.pallet_height}")
        self.get_logger().info(f"Pallet overhang L: {response.pallet_overhang_l}")
        self.get_logger().info(f"Pallet overhang T: {response.pallet_overhang_t}")
        self.get_logger().info(f"Pallet overhang R: {response.pallet_overhang_r}")
        self.get_logger().info(f"Pallet overhang B: {response.pallet_overhang_b}")
        self.get_logger().info(f"Robot type: {response.robot_type}")
        self.get_logger().info(f"Conveyor configuration: {response.conveyor_configuration}")
        self.get_logger().info(f"Box orientation: {response.box_orientation}")
        self.get_logger().info(f"Label position: {response.label_position}")
        self.get_logger().info(f"Additional pedestal height: {response.additional_pedestal_height}")

        return response

def main(args=None):
    rclpy.init(args=args)
    action_server = ServiceServer()
    rclpy.spin(action_server)

if __name__ == '__main__':
    main()